/*
  ESP32 four-load-cell thrust stand

  HX711/load cell 1: DT GPIO4,  SCK GPIO5
  HX711/load cell 2: DT GPIO18, SCK GPIO19
  HX711/load cell 3: DT GPIO21, SCK GPIO22
  HX711/load cell 4: DT GPIO25, SCK GPIO26
  MAUCH PL-100: current signal GPIO34, voltage signal GPIO35

  Wireless access point:
    SSID: ThrustStand
    Password: set WIFI_PASSWORD locally before flashing
    TCP: 192.168.4.1:3333

  Commands (USB serial or Wi-Fi TCP, newline terminated):
    TARE           zero all available cells with the stationary rig installed
    CALLBF1 <lbf>  calibrate cell 1 with a known force applied to cell 1
    CALLBF2 <lbf>  calibrate cell 2 with a known force applied to cell 2
    CALLBF3 <lbf>  calibrate cell 3 with a known force applied to cell 3
    CALLBF4 <lbf>  calibrate cell 4 with a known force applied to cell 4
    INVERT         reverse all calibrated force directions
    INFO           print configuration
    RESETCAL       erase all four calibration factors
    POWERINFO      print MAUCH electrical-sensor configuration
    ZEROAMPS       capture zero-current ADC offset with motor off
    ZEROVOLTS      capture zero-voltage ADC offset with battery disconnected
    CALAMPS <A>    calibrate current against a DC clamp meter
    CALVOLTS <V>   calibrate voltage against a DVM
    RESETPOWER     erase electrical calibration
*/

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <math.h>
#include <stdarg.h>

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint8_t CELL_COUNT = 4;
constexpr uint16_t TCP_PORT = 3333;
constexpr char WIFI_SSID[] = "ThrustStand";
// Public-repository placeholder. Replace this locally with a private password
// of at least eight characters before flashing either ESP32.
constexpr char WIFI_PASSWORD[] = "password";
constexpr uint32_t HX_TIMEOUT_MS = 1500;
constexpr size_t ZERO_SAMPLES = 20;
constexpr size_t CAL_SAMPLES = 20;
constexpr size_t STARTUP_DISCARD_SAMPLES = 12;
constexpr double MIN_COUNTS_PER_KG = 1000.0;
constexpr double STANDARD_GRAVITY = 9.80665;
constexpr double NEWTONS_PER_LBF = 4.4482216152605;
constexpr double EMA_ALPHA = 0.30;

// MAUCH PL-100 integration.
// Its conditioned current output is 0.0-3.3 V for 0-100 A. The voltage
// output is battery voltage divided by the board's configured divider.
constexpr bool ENABLE_MAUCH_SENSOR = true;
constexpr uint8_t MAUCH_CURRENT_PIN = 34; // ADC1; safe while Wi-Fi is active
constexpr uint8_t MAUCH_VOLTAGE_PIN = 35; // ADC1; safe while Wi-Fi is active
constexpr double MAUCH_VOLTAGE_DIVIDER = 10.0; // solder bridge IN, max 33 V
constexpr double DEFAULT_CURRENT_A_PER_COUNT = 100.0 / 4095.0;
constexpr double DEFAULT_VOLTAGE_V_PER_COUNT =
    (3.3 * MAUCH_VOLTAGE_DIVIDER) / 4095.0;
constexpr double POWER_EMA_ALPHA = 0.20;
constexpr uint32_t POWER_INTERVAL_MS = 50;      // 20 electrical samples/s
constexpr uint8_t ADC_AVERAGE_SAMPLES = 16;

struct LoadCell {
  uint8_t doutPin;
  uint8_t sckPin;
  const char *zeroKey;
  const char *scaleKey;
  int64_t zeroOffset;
  double countsPerKg;
  double filteredKgf;
};

LoadCell cells[CELL_COUNT] = {
  {4, 5, "zero1", "scale1", 0, NAN, NAN},
  {18, 19, "zero2", "scale2", 0, NAN, NAN},
  {21, 22, "zero3", "scale3", 0, NAN, NAN},
  {25, 26, "zero4", "scale4", 0, NAN, NAN}
};

Preferences prefs;
WiFiServer tcpServer(TCP_PORT);
WiFiServer webServer(80);
WiFiClient tcpClient;
WiFiClient webClient;
WiFiClient eventClient;
String serialCommand;
String wifiCommand;
String webRequest;
uint32_t webClientStartedMs = 0;
uint32_t lastWebEventMs = 0;
constexpr uint32_t WEB_EVENT_INTERVAL_MS = 250;
uint32_t lastPowerSampleMs = 0;
double filteredCurrentA = NAN;
double filteredVoltageV = NAN;
double latestFilteredPowerW = NAN;
uint32_t latestCurrentRaw = 0;
uint32_t latestVoltageRaw = 0;
int32_t latestCellRaw[CELL_COUNT] = {0, 0, 0, 0};
double latestFilteredCellLbf[CELL_COUNT] = {NAN, NAN, NAN, NAN};
uint32_t latestCellSampleMs[CELL_COUNT] = {0, 0, 0, 0};
uint32_t currentZeroRaw = 0;
uint32_t voltageZeroRaw = 0;
double currentAperCount = DEFAULT_CURRENT_A_PER_COUNT;
double voltageVperCount = DEFAULT_VOLTAGE_V_PER_COUNT;

const char WEB_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>UCI Thrust Stand</title>
  <style>
    :root{color-scheme:dark;--bg:#07111f;--panel:#111d2e;--line:#26364d;
      --text:#edf4ff;--muted:#91a4bd;--blue:#3b82f6;--green:#22c55e;
      --amber:#f59e0b;--red:#ef4444}
    *{box-sizing:border-box}
    body{margin:0;background:linear-gradient(145deg,#07111f,#0c1727 55%,#0b1421);
      color:var(--text);font-family:Inter,Segoe UI,Arial,sans-serif;min-height:100vh}
    main{width:min(1180px,94vw);margin:auto;padding:30px 0 44px}
    header{display:flex;justify-content:space-between;align-items:center;gap:20px;
      margin-bottom:24px}
    h1{font-size:clamp(24px,4vw,38px);margin:0;letter-spacing:-.03em}
    .sub{color:var(--muted);margin-top:7px}
    .status{display:flex;align-items:center;gap:8px;color:var(--muted);
      background:#0c1727;border:1px solid var(--line);padding:9px 13px;border-radius:999px}
    .dot{width:9px;height:9px;border-radius:50%;background:var(--amber);
      box-shadow:0 0 12px currentColor}.dot.online{background:var(--green)}
    .grid{display:grid;grid-template-columns:repeat(12,1fr);gap:14px}
    .card{grid-column:span 3;background:linear-gradient(180deg,#142238,#101b2c);
      border:1px solid var(--line);border-radius:17px;padding:20px;
      box-shadow:0 12px 32px #0004}
    .card.total{grid-column:span 6;border-color:#315f9d}
    .card.electrical{grid-column:span 2}
    .label{color:var(--muted);font-size:13px;text-transform:uppercase;
      letter-spacing:.11em;font-weight:700}
    .value{font-size:clamp(29px,4vw,45px);font-variant-numeric:tabular-nums;
      font-weight:750;margin:12px 0 8px;white-space:nowrap}
    .unit{font-size:.46em;color:var(--muted);margin-left:5px}
    .detail{font-size:13px;color:var(--muted);min-height:18px}
    .warning{color:var(--amber)}
    .plots{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;margin-top:14px}
    .plot-card{background:linear-gradient(180deg,#142238,#101b2c);
      border:1px solid var(--line);border-radius:17px;padding:18px;
      box-shadow:0 12px 32px #0004;min-width:0}
    .plot-card.thrust{grid-column:1/-1}
    .plot-title{display:flex;align-items:center;justify-content:space-between;
      gap:12px;margin-bottom:12px}
    .plot-title h2{font-size:15px;margin:0}.plot-title span{color:var(--muted);font-size:12px}
    canvas{display:block;width:100%;height:220px}
    .legend{display:flex;flex-wrap:wrap;gap:12px;margin-top:10px;
      color:var(--muted);font-size:12px}
    .key{display:inline-flex;align-items:center;gap:5px}
    .swatch{width:9px;height:9px;border-radius:50%;display:inline-block}
    footer{color:var(--muted);font-size:12px;margin-top:20px;text-align:center}
    @media(max-width:800px){.card{grid-column:span 6}.card.total{grid-column:span 12}
      .card.electrical{grid-column:span 4}.plots{grid-template-columns:1fr}}
    @media(max-width:520px){header{align-items:flex-start;flex-direction:column}
      .card,.card.total,.card.electrical{grid-column:span 12}}
  </style>
</head>
<body>
<main>
  <header>
    <div><h1>UCI Aero Design Thrust Stand</h1>
      <div class="sub">Four load cells + MAUCH PL-100</div></div>
    <div class="status"><span id="dot" class="dot"></span><span id="status">Connecting</span></div>
  </header>
  <section class="grid">
    <article class="card"><div class="label">Load cell 1</div>
      <div id="cell1" class="value">--</div><div id="cell1d" class="detail">Waiting for data</div></article>
    <article class="card"><div class="label">Load cell 2</div>
      <div id="cell2" class="value">--</div><div id="cell2d" class="detail">Waiting for data</div></article>
    <article class="card"><div class="label">Load cell 3</div>
      <div id="cell3" class="value">--</div><div id="cell3d" class="detail">Waiting for data</div></article>
    <article class="card"><div class="label">Load cell 4</div>
      <div id="cell4" class="value">--</div><div id="cell4d" class="detail">Waiting for data</div></article>
    <article class="card total"><div class="label">Combined thrust</div>
      <div id="total" class="value">--</div><div class="detail">Sum of fresh calibrated cells</div></article>
    <article class="card electrical"><div class="label">Voltage</div>
      <div id="voltage" class="value">--</div><div id="voltaged" class="detail">MAUCH</div></article>
    <article class="card electrical"><div class="label">Current</div>
      <div id="current" class="value">--</div><div id="currentd" class="detail">MAUCH</div></article>
    <article class="card electrical"><div class="label">Power</div>
      <div id="power" class="value">--</div><div class="detail">Voltage × current</div></article>
  </section>
  <section class="plots">
    <article class="plot-card thrust">
      <div class="plot-title"><h2>Thrust history</h2><span>Last 30 seconds</span></div>
      <canvas id="thrustChart"></canvas>
      <div class="legend">
        <span class="key"><i class="swatch" style="background:#60a5fa"></i>Cell 1</span>
        <span class="key"><i class="swatch" style="background:#fb923c"></i>Cell 2</span>
        <span class="key"><i class="swatch" style="background:#4ade80"></i>Cell 3</span>
        <span class="key"><i class="swatch" style="background:#f87171"></i>Cell 4</span>
        <span class="key"><i class="swatch" style="background:#f8fafc"></i>Total</span>
      </div>
    </article>
    <article class="plot-card">
      <div class="plot-title"><h2>Voltage</h2><span>V</span></div>
      <canvas id="voltageChart"></canvas>
    </article>
    <article class="plot-card">
      <div class="plot-title"><h2>Current</h2><span>A</span></div>
      <canvas id="currentChart"></canvas>
    </article>
    <article class="plot-card">
      <div class="plot-title"><h2>Electrical power</h2><span>W</span></div>
      <canvas id="powerChart"></canvas>
    </article>
  </section>
  <footer>Display only. Use the VS Code workflow for calibration and CSV recording.</footer>
</main>
<script>
  const byId=id=>document.getElementById(id);
  const number=(value,digits,unit)=>value===null?"--":
    Number(value).toFixed(digits)+"<span class='unit'>"+unit+"</span>";
  const history={
    cell1:[],cell2:[],cell3:[],cell4:[],total:[],
    voltage:[],current:[],power:[]
  };
  const historyLimit=120;
  const pushHistory=(name,value)=>{
    history[name].push(value===null?null:Number(value));
    if(history[name].length>historyLimit)history[name].shift();
  };
  function drawChart(canvasId,series,unit){
    const canvas=byId(canvasId);
    const width=Math.max(260,canvas.clientWidth);
    const height=220;
    const ratio=window.devicePixelRatio||1;
    canvas.width=Math.floor(width*ratio);
    canvas.height=Math.floor(height*ratio);
    const context=canvas.getContext("2d");
    context.setTransform(ratio,0,0,ratio,0,0);
    context.clearRect(0,0,width,height);
    const pad={left:48,right:12,top:12,bottom:26};
    const plotWidth=width-pad.left-pad.right;
    const plotHeight=height-pad.top-pad.bottom;
    const values=[];
    series.forEach(item=>item.values.forEach(value=>{
      if(value!==null&&Number.isFinite(value))values.push(value);
    }));
    let minimum=values.length?Math.min(...values):0;
    let maximum=values.length?Math.max(...values):1;
    if(minimum>0)minimum=0;
    if(maximum<0)maximum=0;
    if(maximum-minimum<0.001){maximum+=0.5;minimum-=0.5}
    const margin=(maximum-minimum)*0.08;
    maximum+=margin;minimum-=margin;
    context.font="11px Segoe UI,Arial";
    context.strokeStyle="#26364d";context.fillStyle="#91a4bd";context.lineWidth=1;
    for(let row=0;row<=4;row++){
      const y=pad.top+plotHeight*row/4;
      context.beginPath();context.moveTo(pad.left,y);context.lineTo(width-pad.right,y);
      context.stroke();
      const tick=maximum-(maximum-minimum)*row/4;
      context.fillText(tick.toFixed(Math.abs(tick)<10?2:1),4,y+4);
    }
    context.fillText("-30 s",pad.left,height-6);
    context.fillText("now",width-pad.right-22,height-6);
    context.fillText(unit,4,11);
    series.forEach(item=>{
      context.strokeStyle=item.color;context.lineWidth=item.width||2;
      context.beginPath();let drawing=false;
      const historyOffset=Math.max(0,historyLimit-item.values.length);
      item.values.forEach((value,index)=>{
        if(value===null||!Number.isFinite(value)){drawing=false;return}
        const x=pad.left+plotWidth*((historyOffset+index)/Math.max(1,historyLimit-1));
        const y=pad.top+plotHeight*(maximum-value)/(maximum-minimum);
        if(!drawing){context.moveTo(x,y);drawing=true}else context.lineTo(x,y);
      });
      context.stroke();
    });
  }
  function drawCharts(){
    drawChart("thrustChart",[
      {values:history.cell1,color:"#60a5fa"},
      {values:history.cell2,color:"#fb923c"},
      {values:history.cell3,color:"#4ade80"},
      {values:history.cell4,color:"#f87171"},
      {values:history.total,color:"#f8fafc",width:3}
    ],"lbf");
    drawChart("voltageChart",[{values:history.voltage,color:"#60a5fa"}],"V");
    drawChart("currentChart",[{values:history.current,color:"#fb923c"}],"A");
    drawChart("powerChart",[{values:history.power,color:"#4ade80"}],"W");
  }
  function applyData(data){
    try{
      data.cells.forEach((cell,index)=>{
        const value=byId("cell"+(index+1));
        const detail=byId("cell"+(index+1)+"d");
        if(!cell.available){
          value.innerHTML="--";detail.textContent="Not connected";detail.className="detail warning";
        }else if(!cell.calibrated){
          value.innerHTML="--";detail.textContent="Uncalibrated · raw "+cell.raw;
          detail.className="detail warning";
        }else{
          value.innerHTML=number(cell.lbf,3,"lbf");detail.textContent="Raw ADC "+cell.raw;
          detail.className="detail";
        }
      });
      byId("total").innerHTML=number(data.total_lbf,3,"lbf");
      byId("voltage").innerHTML=number(data.voltage_v,3,"V");
      byId("current").innerHTML=number(data.current_a,3,"A");
      byId("power").innerHTML=number(data.power_w,2,"W");
      const saturated=data.current_raw>=4000||data.voltage_raw>=4000;
      byId("voltaged").textContent=saturated?"ADC near saturation":"MAUCH";
      byId("currentd").textContent=saturated?"Check signal level":"MAUCH";
      byId("voltaged").className=saturated?"detail warning":"detail";
      byId("currentd").className=saturated?"detail warning":"detail";
      data.cells.forEach((cell,index)=>
        pushHistory("cell"+(index+1),cell.available&&cell.calibrated?cell.lbf:null));
      pushHistory("total",data.total_lbf);
      pushHistory("voltage",data.voltage_v);
      pushHistory("current",data.current_a);
      pushHistory("power",data.power_w);
      drawCharts();
      byId("dot").className="dot online";byId("status").textContent="Live";
    }catch(error){
      byId("dot").className="dot";byId("status").textContent="Data error";
    }
  }
  const eventStream=new EventSource("/events");
  eventStream.onopen=()=>{
    byId("dot").className="dot online";byId("status").textContent="Live";
  };
  eventStream.onmessage=event=>{
    try{applyData(JSON.parse(event.data));}
    catch(error){byId("dot").className="dot";byId("status").textContent="Data error";}
  };
  eventStream.onerror=()=>{
    byId("dot").className="dot";byId("status").textContent="Reconnecting";
  };
  window.addEventListener("resize",drawCharts);
</script>
</body>
</html>
)HTML";

IPAddress networkIp() {
  return WiFi.softAPIP();
}

void emitLine(const String &line) {
  Serial.println(line);
  if (tcpClient && tcpClient.connected()) tcpClient.println(line);
}

void emitf(const char *format, ...) {
  char buffer[320];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  emitLine(buffer);
}

bool hxReady(const LoadCell &cell, uint32_t timeoutMs) {
  const uint32_t started = millis();
  while (digitalRead(cell.doutPin) == HIGH) {
    if (millis() - started >= timeoutMs) return false;
    delay(1);
  }
  return true;
}

bool hxRead(LoadCell &cell, int32_t &value, uint32_t timeoutMs = HX_TIMEOUT_MS) {
  if (!hxReady(cell, timeoutMs)) return false;

  uint32_t data = 0;
  noInterrupts();
  for (uint8_t i = 0; i < 24; ++i) {
    digitalWrite(cell.sckPin, HIGH);
    delayMicroseconds(1);
    data = (data << 1) | (digitalRead(cell.doutPin) ? 1U : 0U);
    digitalWrite(cell.sckPin, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(cell.sckPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(cell.sckPin, LOW);
  interrupts();

  if (data & 0x800000UL) data |= 0xFF000000UL;
  value = static_cast<int32_t>(data);
  return true;
}

bool averageRaw(LoadCell &cell, size_t sampleCount, int64_t &average) {
  int64_t sum = 0;
  for (size_t i = 0; i < sampleCount; ++i) {
    int32_t raw;
    if (!hxRead(cell, raw)) return false;
    sum += raw;
  }
  average = sum / static_cast<int64_t>(sampleCount);
  return true;
}

void printInfo() {
  for (uint8_t i = 0; i < CELL_COUNT; ++i) {
    const String scale = isfinite(cells[i].countsPerKg)
        ? String(cells[i].countsPerKg, 6) : "not_calibrated";
    emitf("# INFO,cell=%u,dout=%u,sck=%u,zero=%lld,counts_per_kg=%s",
          i + 1, cells[i].doutPin, cells[i].sckPin,
          cells[i].zeroOffset, scale.c_str());
  }
  emitf("# INFO,wifi_ssid=%s,wifi_ip=%s,tcp_port=%u",
        WIFI_SSID, networkIp().toString().c_str(), TCP_PORT);
  emitf("# POWERINFO,enabled=%u,current_pin=%u,voltage_pin=%u,voltage_divider=%.6f,current_zero_raw=%lu,voltage_zero_raw=%lu,current_A_per_count=%.9f,voltage_V_per_count=%.9f",
        ENABLE_MAUCH_SENSOR ? 1 : 0, MAUCH_CURRENT_PIN, MAUCH_VOLTAGE_PIN,
        MAUCH_VOLTAGE_DIVIDER, currentZeroRaw, voltageZeroRaw,
        currentAperCount, voltageVperCount);
}

uint32_t averageAdcRaw(uint8_t pin, uint16_t samples = ADC_AVERAGE_SAMPLES) {
  uint32_t total = 0;
  for (uint16_t i = 0; i < samples; ++i) total += analogRead(pin);
  return total / samples;
}

void resetPowerFilters() {
  filteredCurrentA = NAN;
  filteredVoltageV = NAN;
}

void zeroPowerChannel(bool currentChannel) {
  if (!ENABLE_MAUCH_SENSOR) {
    emitLine("# ERROR,Electrical sensor is disabled on this ESP");
    return;
  }
  const uint8_t pin = currentChannel ? MAUCH_CURRENT_PIN : MAUCH_VOLTAGE_PIN;
  const uint32_t raw = averageAdcRaw(pin, 128);
  if (currentChannel) {
    currentZeroRaw = raw;
    prefs.putUInt("izero", currentZeroRaw);
    emitf("# STATUS,Current zero saved,current_zero_raw=%lu", currentZeroRaw);
  } else {
    voltageZeroRaw = raw;
    prefs.putUInt("vzero", voltageZeroRaw);
    emitf("# STATUS,Voltage zero saved,voltage_zero_raw=%lu", voltageZeroRaw);
  }
  resetPowerFilters();
}

void calibratePowerChannel(bool currentChannel, double referenceValue) {
  const double maximum = currentChannel ? 100.0 : 60.0;
  if (!ENABLE_MAUCH_SENSOR) {
    emitLine("# ERROR,Electrical sensor is disabled on this ESP");
    return;
  }
  if (!isfinite(referenceValue) || referenceValue <= 0.0 || referenceValue > maximum) {
    emitf("# ERROR,%s reference must be greater than 0 and at most %.1f",
          currentChannel ? "Current" : "Voltage", maximum);
    return;
  }
  const uint8_t pin = currentChannel ? MAUCH_CURRENT_PIN : MAUCH_VOLTAGE_PIN;
  const uint32_t zero = currentChannel ? currentZeroRaw : voltageZeroRaw;
  const uint32_t raw = averageAdcRaw(pin, 128);
  if (raw <= zero + 8) {
    emitf("# ERROR,%s calibration signal is too close to zero; check wiring and reference load",
          currentChannel ? "Current" : "Voltage");
    return;
  }
  const double scale = referenceValue / static_cast<double>(raw - zero);
  if (currentChannel) {
    currentAperCount = scale;
    prefs.putDouble("iscale", currentAperCount);
    emitf("# STATUS,Current calibration saved,reference_A=%.5f,raw=%lu,A_per_count=%.9f",
          referenceValue, raw, currentAperCount);
  } else {
    voltageVperCount = scale;
    prefs.putDouble("vscale", voltageVperCount);
    emitf("# STATUS,Voltage calibration saved,reference_V=%.5f,raw=%lu,V_per_count=%.9f",
          referenceValue, raw, voltageVperCount);
  }
  resetPowerFilters();
}

void sampleElectricalPower() {
  if (!ENABLE_MAUCH_SENSOR) return;
  const uint32_t now = millis();
  if (now - lastPowerSampleMs < POWER_INTERVAL_MS) return;
  lastPowerSampleMs = now;

  const uint32_t currentRaw = averageAdcRaw(MAUCH_CURRENT_PIN);
  const uint32_t voltageRaw = averageAdcRaw(MAUCH_VOLTAGE_PIN);
  const uint32_t correctedCurrentRaw = currentRaw > currentZeroRaw ? currentRaw - currentZeroRaw : 0;
  const uint32_t correctedVoltageRaw = voltageRaw > voltageZeroRaw ? voltageRaw - voltageZeroRaw : 0;
  double currentA = correctedCurrentRaw * currentAperCount;
  double voltageV = correctedVoltageRaw * voltageVperCount;
  if (currentA < 0.05) currentA = 0.0;
  if (voltageV < 0.02) voltageV = 0.0;

  if (!isfinite(filteredCurrentA)) filteredCurrentA = currentA;
  else filteredCurrentA += POWER_EMA_ALPHA * (currentA - filteredCurrentA);
  if (!isfinite(filteredVoltageV)) filteredVoltageV = voltageV;
  else filteredVoltageV += POWER_EMA_ALPHA * (voltageV - filteredVoltageV);

  const double powerW = voltageV * currentA;
  const double filteredPowerW = filteredVoltageV * filteredCurrentA;
  latestCurrentRaw = currentRaw;
  latestVoltageRaw = voltageRaw;
  latestFilteredPowerW = filteredPowerW;
  emitf("POWER,%lu,%lu,%lu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f",
        now, currentRaw, voltageRaw, currentA, voltageV, powerW,
        filteredCurrentA, filteredVoltageV, filteredPowerW);
}

void tareAll() {
  emitLine("# STATUS,Zeroing available cells: motor off, no applied thrust, keep the rig still");
  uint8_t zeroed = 0;
  for (uint8_t i = 0; i < CELL_COUNT; ++i) {
    int64_t average;
    if (!averageRaw(cells[i], ZERO_SAMPLES, average)) {
      emitf("# STATUS,HX711 cell %u unavailable; skipped during tare", i + 1);
      continue;
    }
    cells[i].zeroOffset = average;
    cells[i].filteredKgf = NAN;
    prefs.putLong64(cells[i].zeroKey, cells[i].zeroOffset);
    ++zeroed;
  }
  if (zeroed) {
    emitf("# STATUS,Tare complete,zeroed_cells=%u", zeroed);
  } else {
    emitLine("# ERROR,No HX711 cells responded during tare");
  }
}

void calibrateCell(uint8_t index, double knownLbf) {
  if (index >= CELL_COUNT) return;
  if (!isfinite(knownLbf) || knownLbf <= 0.0 || knownLbf > 44.0925) {
    emitLine("# ERROR,Calibration force must be greater than 0 and at most 44.0925 lbf per cell");
    return;
  }

  emitf("# STATUS,Calibrating cell %u with %.6f lbf; hold the load steady over that cell",
        index + 1, knownLbf);
  int64_t loadedAverage;
  if (!averageRaw(cells[index], CAL_SAMPLES, loadedAverage)) {
    emitf("# ERROR,HX711 cell %u timeout during calibration", index + 1);
    return;
  }

  const double knownKgEquivalent = knownLbf * NEWTONS_PER_LBF / STANDARD_GRAVITY;
  const double factor = static_cast<double>(loadedAverage - cells[index].zeroOffset) /
                        knownKgEquivalent;
  if (!isfinite(factor) || fabs(factor) < MIN_COUNTS_PER_KG) {
    emitLine("# ERROR,Calibration signal is too small; use a larger known force and check mounting/wiring");
    return;
  }

  cells[index].countsPerKg = factor;
  cells[index].filteredKgf = NAN;
  prefs.putDouble(cells[index].scaleKey, cells[index].countsPerKg);
  emitf("# STATUS,Cell %u calibration saved,counts_per_kg=%.6f",
        index + 1, cells[index].countsPerKg);
}

void invertDirection() {
  uint8_t inverted = 0;
  for (uint8_t i = 0; i < CELL_COUNT; ++i) {
    if (!isfinite(cells[i].countsPerKg)) continue;
    cells[i].countsPerKg = -cells[i].countsPerKg;
    cells[i].filteredKgf = NAN;
    prefs.putDouble(cells[i].scaleKey, cells[i].countsPerKg);
    ++inverted;
  }
  if (inverted) emitf("# STATUS,Direction inverted for %u calibrated cell(s)", inverted);
  else emitLine("# ERROR,Calibrate at least one available cell before changing direction");
}

void handleCommand(String command) {
  command.trim();
  if (!command.length()) return;
  String upper = command;
  upper.toUpperCase();

  if (upper == "T" || upper == "TARE") {
    tareAll();
  } else if (upper.startsWith("CALLBF1 ")) {
    calibrateCell(0, command.substring(8).toDouble());
  } else if (upper.startsWith("CALLBF2 ")) {
    calibrateCell(1, command.substring(8).toDouble());
  } else if (upper.startsWith("CALLBF3 ")) {
    calibrateCell(2, command.substring(8).toDouble());
  } else if (upper.startsWith("CALLBF4 ")) {
    calibrateCell(3, command.substring(8).toDouble());
  } else if (upper.startsWith("CALLBF ")) {
    emitLine("# ERROR,Use CALLBF1, CALLBF2, CALLBF3, or CALLBF4");
  } else if (upper == "INVERT") {
    invertDirection();
  } else if (upper == "I" || upper == "INFO") {
    printInfo();
  } else if (upper == "POWERINFO") {
    printInfo();
  } else if (upper == "ZEROAMPS") {
    zeroPowerChannel(true);
  } else if (upper == "ZEROVOLTS") {
    zeroPowerChannel(false);
  } else if (upper.startsWith("CALAMPS ")) {
    calibratePowerChannel(true, command.substring(8).toDouble());
  } else if (upper.startsWith("CALVOLTS ")) {
    calibratePowerChannel(false, command.substring(9).toDouble());
  } else if (upper == "RESETPOWER") {
    currentZeroRaw = 0;
    voltageZeroRaw = 0;
    currentAperCount = DEFAULT_CURRENT_A_PER_COUNT;
    voltageVperCount = DEFAULT_VOLTAGE_V_PER_COUNT;
    prefs.remove("izero");
    prefs.remove("vzero");
    prefs.remove("iscale");
    prefs.remove("vscale");
    resetPowerFilters();
    emitLine("# STATUS,Electrical calibration reset to defaults");
  } else if (upper == "RESETCAL") {
    for (uint8_t i = 0; i < CELL_COUNT; ++i) {
      cells[i].countsPerKg = NAN;
      cells[i].filteredKgf = NAN;
      prefs.remove(cells[i].scaleKey);
    }
    emitLine("# STATUS,All four calibration factors erased");
  } else {
    emitLine("# ERROR,Use TARE, CALLBF1/2/3/4 <lbf>, INVERT, INFO, ZEROAMPS, ZEROVOLTS, CALAMPS <A>, CALVOLTS <V>, RESETPOWER, or RESETCAL");
  }
}

void readCommands(Stream &stream, String &buffer) {
  while (stream.available()) {
    const char c = static_cast<char>(stream.read());
    if (c == '\n' || c == '\r') {
      if (buffer.length()) {
        handleCommand(buffer);
        buffer = "";
      }
    } else if (buffer.length() < 80) {
      buffer += c;
    } else {
      buffer = "";
      emitLine("# ERROR,Command too long");
    }
  }
}

void serviceWifiClient() {
  if (tcpClient && !tcpClient.connected()) {
    tcpClient.stop();
    wifiCommand = "";
  }
  WiFiClient candidate = tcpServer.available();
  if (candidate) {
    if (tcpClient && tcpClient.connected()) {
      candidate.println("# ERROR,Another logger is already connected");
      candidate.stop();
    } else {
      tcpClient = candidate;
      tcpClient.setNoDelay(true);
      wifiCommand = "";
      emitLine("# STATUS,Wireless logger connected");
      printInfo();
    }
  }
  if (tcpClient && tcpClient.connected()) readCommands(tcpClient, wifiCommand);
}

void printJsonNumber(WiFiClient &client, double value, uint8_t decimals) {
  if (isfinite(value)) client.print(value, decimals);
  else client.print("null");
}

void sendWebDashboard() {
  webClient.println("HTTP/1.1 200 OK");
  webClient.println("Content-Type: text/html; charset=utf-8");
  webClient.println("Cache-Control: no-store");
  webClient.println("Connection: close");
  webClient.println();
  webClient.print(WEB_PAGE);
}

void writeWebJson(WiFiClient &client) {
  const uint32_t now = millis();
  client.print("{\"time_ms\":");
  client.print(now);
  client.print(",\"cells\":[");
  double dashboardTotalLbf = 0.0;
  uint8_t dashboardCellCount = 0;
  for (uint8_t i = 0; i < CELL_COUNT; ++i) {
    if (i) client.print(',');
    const bool available =
        latestCellSampleMs[i] && now - latestCellSampleMs[i] <= 2000;
    if (available && isfinite(latestFilteredCellLbf[i])) {
      dashboardTotalLbf += latestFilteredCellLbf[i];
      ++dashboardCellCount;
    }
    client.print("{\"available\":");
    client.print(available ? "true" : "false");
    client.print(",\"calibrated\":");
    client.print(isfinite(cells[i].countsPerKg) ? "true" : "false");
    client.print(",\"raw\":");
    client.print(latestCellRaw[i]);
    client.print(",\"lbf\":");
    printJsonNumber(client, latestFilteredCellLbf[i], 5);
    client.print('}');
  }
  client.print("],\"total_lbf\":");
  printJsonNumber(client, dashboardCellCount ? dashboardTotalLbf : NAN, 5);
  client.print(",\"current_raw\":");
  client.print(latestCurrentRaw);
  client.print(",\"voltage_raw\":");
  client.print(latestVoltageRaw);
  client.print(",\"current_a\":");
  printJsonNumber(client, filteredCurrentA, 5);
  client.print(",\"voltage_v\":");
  printJsonNumber(client, filteredVoltageV, 5);
  client.print(",\"power_w\":");
  printJsonNumber(client, latestFilteredPowerW, 5);
  client.print('}');
}

void sendWebData() {
  webClient.println("HTTP/1.1 200 OK");
  webClient.println("Content-Type: application/json");
  webClient.println("Cache-Control: no-store");
  webClient.println("Connection: close");
  webClient.println();
  writeWebJson(webClient);
}

void startWebEvents() {
  if (eventClient) eventClient.stop();
  eventClient = webClient;
  eventClient.setNoDelay(true);
  eventClient.println("HTTP/1.1 200 OK");
  eventClient.println("Content-Type: text/event-stream");
  eventClient.println("Cache-Control: no-cache");
  eventClient.println("Connection: keep-alive");
  eventClient.println();
  eventClient.println("retry: 2000");
  eventClient.println();
  webClient = WiFiClient();
  webRequest = "";
  lastWebEventMs = 0;
}

void serviceWebEvents() {
  if (!eventClient || !eventClient.connected()) {
    if (eventClient) eventClient.stop();
    return;
  }
  const uint32_t now = millis();
  if (now - lastWebEventMs < WEB_EVENT_INTERVAL_MS) return;
  lastWebEventMs = now;
  eventClient.print("data: ");
  writeWebJson(eventClient);
  eventClient.print("\n\n");
}

void sendWebNotFound() {
  webClient.println("HTTP/1.1 404 Not Found");
  webClient.println("Content-Type: text/plain; charset=utf-8");
  webClient.println("Connection: close");
  webClient.println();
  webClient.println("Not found");
}

void serviceWebClient() {
  if (webClient && !webClient.connected()) {
    webClient.stop();
    webRequest = "";
  }
  if (!webClient) {
    WiFiClient candidate = webServer.available();
    if (candidate) {
      webClient = candidate;
      webRequest = "";
      webClientStartedMs = millis();
    }
  }
  if (!webClient) return;

  while (webClient.available()) {
    const char c = static_cast<char>(webClient.read());
    if (c == '\n') {
      if (webRequest.startsWith("GET /events ")) {
        startWebEvents();
        return;
      }
      if (webRequest.startsWith("GET /data ")) sendWebData();
      else if (webRequest.startsWith("GET / ")) sendWebDashboard();
      else sendWebNotFound();
      webClient.stop();
      webRequest = "";
      return;
    }
    if (c != '\r' && webRequest.length() < 256) webRequest += c;
  }
  if (millis() - webClientStartedMs > 1000) {
    webClient.stop();
    webRequest = "";
  }
}

void setup() {
  for (uint8_t i = 0; i < CELL_COUNT; ++i) {
    pinMode(cells[i].doutPin, INPUT_PULLUP);
    pinMode(cells[i].sckPin, OUTPUT);
    digitalWrite(cells[i].sckPin, LOW);
  }
  Serial.begin(SERIAL_BAUD);
  delay(700);

  if (ENABLE_MAUCH_SENSOR) {
    pinMode(MAUCH_CURRENT_PIN, INPUT);
    pinMode(MAUCH_VOLTAGE_PIN, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(MAUCH_CURRENT_PIN, ADC_11db);
    analogSetPinAttenuation(MAUCH_VOLTAGE_PIN, ADC_11db);
  }

  WiFi.mode(WIFI_AP);
  const bool wifiReady = WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  if (wifiReady) {
    tcpServer.begin();
    tcpServer.setNoDelay(true);
    webServer.begin();
    webServer.setNoDelay(true);
    Serial.printf("# STATUS,WiFi ready: %s:%u\n",
                  networkIp().toString().c_str(), TCP_PORT);
    Serial.printf("# STATUS,Web dashboard: http://%s/\n",
                  networkIp().toString().c_str());
  } else {
    Serial.println("# ERROR,Could not establish WiFi; USB serial still works");
  }

  prefs.begin("thrust-stand", false);
  currentZeroRaw = prefs.getUInt("izero", 0);
  voltageZeroRaw = prefs.getUInt("vzero", 0);
  currentAperCount = prefs.getDouble("iscale", DEFAULT_CURRENT_A_PER_COUNT);
  voltageVperCount = prefs.getDouble("vscale", DEFAULT_VOLTAGE_V_PER_COUNT);
  if (!isfinite(currentAperCount) || currentAperCount <= 0.0) {
    currentAperCount = DEFAULT_CURRENT_A_PER_COUNT;
  }
  if (!isfinite(voltageVperCount) || voltageVperCount <= 0.0) {
    voltageVperCount = DEFAULT_VOLTAGE_V_PER_COUNT;
  }
  // Migrate calibration made by the original single-cell firmware to cell 1.
  if (!prefs.isKey(cells[0].zeroKey) && prefs.isKey("zero")) {
    prefs.putLong64(cells[0].zeroKey, prefs.getLong64("zero", 0));
    emitLine("# STATUS,Migrated legacy zero offset to cell 1");
  }
  if (!prefs.isKey(cells[0].scaleKey) && prefs.isKey("scale")) {
    const double legacyScale = prefs.getDouble("scale", NAN);
    if (isfinite(legacyScale) && fabs(legacyScale) >= MIN_COUNTS_PER_KG) {
      prefs.putDouble(cells[0].scaleKey, legacyScale);
      emitLine("# STATUS,Migrated legacy calibration factor to cell 1");
    }
  }
  for (uint8_t i = 0; i < CELL_COUNT; ++i) {
    cells[i].zeroOffset = prefs.getLong64(cells[i].zeroKey, 0);
    cells[i].countsPerKg = prefs.getDouble(cells[i].scaleKey, NAN);
    if (isfinite(cells[i].countsPerKg) && fabs(cells[i].countsPerKg) < MIN_COUNTS_PER_KG) {
      cells[i].countsPerKg = NAN;
      prefs.remove(cells[i].scaleKey);
    }
  }

  emitLine("# THRUST_STAND_V8_FOUR_CELL_POWER_CAL");
  emitLine("# COLUMNS,time_ms,raw1,raw2,raw3,raw4,kgf1,kgf2,kgf3,kgf4,total_kgf,total_force_N,filtered_total_force_N");
  if (ENABLE_MAUCH_SENSOR) {
    emitLine("# POWER_COLUMNS,time_ms,current_adc_raw,voltage_adc_raw,current_A,voltage_V,power_W,filtered_current_A,filtered_voltage_V,filtered_power_W");
  }
  printInfo();

  for (uint8_t i = 0; i < CELL_COUNT; ++i) {
    if (!hxReady(cells[i], HX_TIMEOUT_MS)) {
      emitf("# ERROR,HX711 cell %u not ready; check its wiring and power", i + 1);
    }
  }

  for (size_t sample = 0; sample < STARTUP_DISCARD_SAMPLES; ++sample) {
    for (uint8_t i = 0; i < CELL_COUNT; ++i) {
      int32_t discarded;
      if (digitalRead(cells[i].doutPin) == LOW) hxRead(cells[i], discarded, 20);
    }
    delay(10);
  }

  bool missingZero = false;
  bool missingCalibration = false;
  for (uint8_t i = 0; i < CELL_COUNT; ++i) {
    missingZero |= cells[i].zeroOffset == 0;
    missingCalibration |= !isfinite(cells[i].countsPerKg);
  }
  if (missingZero) {
    emitLine("# STATUS,Send TARE with the complete stand unloaded");
  }
  if (missingCalibration) {
    emitLine("# STATUS,Calibrate each sensor separately with CALLBF1 through CALLBF4");
  }
}

void loop() {
  readCommands(Serial, serialCommand);
  serviceWifiClient();
  serviceWebClient();
  serviceWebEvents();
  sampleElectricalPower();

  int32_t raw[CELL_COUNT] = {0, 0, 0, 0};
  bool received[CELL_COUNT] = {false, false, false, false};
  bool anyReceived = false;
  for (uint8_t i = 0; i < CELL_COUNT; ++i) {
    if (digitalRead(cells[i].doutPin) == LOW) received[i] = hxRead(cells[i], raw[i], 20);
    anyReceived |= received[i];
  }
  if (!anyReceived) return;
  const uint32_t sampleNow = millis();

  double kgf[CELL_COUNT] = {NAN, NAN, NAN, NAN};
  double filteredKgf[CELL_COUNT] = {NAN, NAN, NAN, NAN};
  double totalKgf = 0.0;
  double filteredTotalKgf = 0.0;
  uint8_t validCells = 0;
  for (uint8_t i = 0; i < CELL_COUNT; ++i) {
    if (!received[i]) continue;
    latestCellRaw[i] = raw[i];
    latestCellSampleMs[i] = sampleNow;
    latestFilteredCellLbf[i] = NAN;
    if (!isfinite(cells[i].countsPerKg)) continue;
    kgf[i] = static_cast<double>(static_cast<int64_t>(raw[i]) - cells[i].zeroOffset) /
             cells[i].countsPerKg;
    if (!isfinite(cells[i].filteredKgf)) cells[i].filteredKgf = kgf[i];
    else cells[i].filteredKgf += EMA_ALPHA * (kgf[i] - cells[i].filteredKgf);
    filteredKgf[i] = cells[i].filteredKgf;
    latestFilteredCellLbf[i] =
        filteredKgf[i] * STANDARD_GRAVITY / NEWTONS_PER_LBF;
    totalKgf += kgf[i];
    filteredTotalKgf += filteredKgf[i];
    ++validCells;
  }

  const double reportedTotalKgf = validCells ? totalKgf : NAN;
  const double totalForceN = validCells ? totalKgf * STANDARD_GRAVITY : NAN;
  const double filteredTotalN = validCells ? filteredTotalKgf * STANDARD_GRAVITY : NAN;
  emitf("DATA,%lu,%ld,%ld,%ld,%ld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
        sampleNow, raw[0], raw[1], raw[2], raw[3],
        kgf[0], kgf[1], kgf[2], kgf[3], reportedTotalKgf,
        totalForceN, filteredTotalN);
}
