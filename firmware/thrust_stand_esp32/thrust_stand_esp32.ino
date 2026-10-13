/*
  ESP32 dual-load-cell thrust stand

  HX711/load cell 1: DT GPIO4,  SCK GPIO5
  HX711/load cell 2: DT GPIO18, SCK GPIO19

  Wireless access point:
    SSID: ThrustStand
    Password: thrust20kg
    TCP: 192.168.4.1:3333

  Set DEVICE_NUMBER to 1 before uploading ESP 1. Set it to 2 before
  uploading ESP 2. ESP 1 creates the network; ESP 2 joins it at 192.168.4.2.

  Commands (USB serial or Wi-Fi TCP, newline terminated):
    TARE           zero both cells with the complete stationary rig installed
    CALLBF1 <lbf>  calibrate cell 1 with a known force applied to cell 1
    CALLBF2 <lbf>  calibrate cell 2 with a known force applied to cell 2
    INVERT         reverse both displayed force directions
    INFO           print configuration
    RESETCAL       erase both calibration factors
*/

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <math.h>
#include <stdarg.h>

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint8_t DEVICE_NUMBER = 1; // valid values: 1 (primary) or 2 (secondary)
constexpr uint16_t TCP_PORT = 3333;
constexpr char WIFI_SSID[] = "ThrustStand";
constexpr char WIFI_PASSWORD[] = "thrust20kg";
constexpr uint32_t HX_TIMEOUT_MS = 1500;
constexpr size_t ZERO_SAMPLES = 20;
constexpr size_t CAL_SAMPLES = 20;
constexpr size_t STARTUP_DISCARD_SAMPLES = 12;
constexpr double MIN_COUNTS_PER_KG = 1000.0;
constexpr double STANDARD_GRAVITY = 9.80665;
constexpr double NEWTONS_PER_LBF = 4.4482216152605;
constexpr double EMA_ALPHA = 0.30;

struct LoadCell {
  uint8_t doutPin;
  uint8_t sckPin;
  const char *zeroKey;
  const char *scaleKey;
  int64_t zeroOffset;
  double countsPerKg;
  double filteredKgf;
};

LoadCell cells[2] = {
  {4, 5, "zero1", "scale1", 0, NAN, NAN},
  {18, 19, "zero2", "scale2", 0, NAN, NAN}
};

Preferences prefs;
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;
String serialCommand;
String wifiCommand;

IPAddress networkIp() {
  return DEVICE_NUMBER == 1 ? WiFi.softAPIP() : WiFi.localIP();
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
  const String scale1 = isfinite(cells[0].countsPerKg) ? String(cells[0].countsPerKg, 6) : "not_calibrated";
  const String scale2 = isfinite(cells[1].countsPerKg) ? String(cells[1].countsPerKg, 6) : "not_calibrated";
  emitf("# INFO,cell1_dout=%u,cell1_sck=%u,cell1_zero=%lld,cell1_counts_per_kg=%s",
        cells[0].doutPin, cells[0].sckPin, cells[0].zeroOffset, scale1.c_str());
  emitf("# INFO,cell2_dout=%u,cell2_sck=%u,cell2_zero=%lld,cell2_counts_per_kg=%s",
        cells[1].doutPin, cells[1].sckPin, cells[1].zeroOffset, scale2.c_str());
  emitf("# INFO,device=%u,wifi_ssid=%s,wifi_ip=%s,tcp_port=%u",
        DEVICE_NUMBER, WIFI_SSID, networkIp().toString().c_str(), TCP_PORT);
}

void tareAll() {
  emitLine("# STATUS,Zeroing both cells: motor off, no applied thrust, keep the rig still");
  int64_t averages[2];
  if (!averageRaw(cells[0], ZERO_SAMPLES, averages[0])) {
    emitLine("# ERROR,HX711 cell 1 timeout during tare");
    return;
  }
  if (!averageRaw(cells[1], ZERO_SAMPLES, averages[1])) {
    emitLine("# ERROR,HX711 cell 2 timeout during tare");
    return;
  }
  for (uint8_t i = 0; i < 2; ++i) {
    cells[i].zeroOffset = averages[i];
    cells[i].filteredKgf = NAN;
    prefs.putLong64(cells[i].zeroKey, cells[i].zeroOffset);
  }
  emitf("# STATUS,Tare complete,cell1_zero=%lld,cell2_zero=%lld",
        cells[0].zeroOffset, cells[1].zeroOffset);
}

void calibrateCell(uint8_t index, double knownLbf) {
  if (index > 1) return;
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
  if (!isfinite(cells[0].countsPerKg) || !isfinite(cells[1].countsPerKg)) {
    emitLine("# ERROR,Calibrate both cells before changing direction");
    return;
  }
  for (uint8_t i = 0; i < 2; ++i) {
    cells[i].countsPerKg = -cells[i].countsPerKg;
    cells[i].filteredKgf = NAN;
    prefs.putDouble(cells[i].scaleKey, cells[i].countsPerKg);
  }
  emitLine("# STATUS,Both displayed force directions inverted and saved");
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
  } else if (upper.startsWith("CALLBF ")) {
    emitLine("# ERROR,Two-cell mode requires CALLBF1 <lbf> and CALLBF2 <lbf>");
  } else if (upper == "INVERT") {
    invertDirection();
  } else if (upper == "I" || upper == "INFO") {
    printInfo();
  } else if (upper == "RESETCAL") {
    for (uint8_t i = 0; i < 2; ++i) {
      cells[i].countsPerKg = NAN;
      cells[i].filteredKgf = NAN;
      prefs.remove(cells[i].scaleKey);
    }
    emitLine("# STATUS,Both calibration factors erased");
  } else {
    emitLine("# ERROR,Use TARE, CALLBF1 <lbf>, CALLBF2 <lbf>, INVERT, INFO, or RESETCAL");
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

void setup() {
  for (uint8_t i = 0; i < 2; ++i) {
    pinMode(cells[i].doutPin, INPUT_PULLUP);
    pinMode(cells[i].sckPin, OUTPUT);
    digitalWrite(cells[i].sckPin, LOW);
  }
  Serial.begin(SERIAL_BAUD);
  delay(700);

  bool wifiReady = false;
  if (DEVICE_NUMBER == 1) {
    WiFi.mode(WIFI_AP);
    wifiReady = WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  } else {
    WiFi.mode(WIFI_STA);
    const IPAddress localIp(192, 168, 4, DEVICE_NUMBER);
    const IPAddress gateway(192, 168, 4, 1);
    const IPAddress subnet(255, 255, 255, 0);
    WiFi.config(localIp, gateway, subnet);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    const uint32_t connectStarted = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - connectStarted < 15000) delay(100);
    wifiReady = WiFi.status() == WL_CONNECTED;
  }

  if (wifiReady) {
    tcpServer.begin();
    tcpServer.setNoDelay(true);
    Serial.printf("# STATUS,ESP %u WiFi ready: %s:%u\n",
                  DEVICE_NUMBER, networkIp().toString().c_str(), TCP_PORT);
  } else {
    Serial.println("# ERROR,Could not establish WiFi; USB serial still works");
  }

  prefs.begin("thrust-stand", false);
  for (uint8_t i = 0; i < 2; ++i) {
    cells[i].zeroOffset = prefs.getLong64(cells[i].zeroKey, 0);
    cells[i].countsPerKg = prefs.getDouble(cells[i].scaleKey, NAN);
    if (isfinite(cells[i].countsPerKg) && fabs(cells[i].countsPerKg) < MIN_COUNTS_PER_KG) {
      cells[i].countsPerKg = NAN;
      prefs.remove(cells[i].scaleKey);
    }
  }

  emitf("# THRUST_STAND_V4_DUAL_WIFI,device=%u", DEVICE_NUMBER);
  emitLine("# COLUMNS,time_ms,raw1,raw2,kgf1,kgf2,total_kgf,total_force_N,filtered_total_force_N");
  printInfo();

  for (uint8_t i = 0; i < 2; ++i) {
    if (!hxReady(cells[i], HX_TIMEOUT_MS)) {
      emitf("# ERROR,HX711 cell %u not ready; check its wiring and power", i + 1);
    }
  }

  for (size_t sample = 0; sample < STARTUP_DISCARD_SAMPLES; ++sample) {
    for (uint8_t i = 0; i < 2; ++i) {
      int32_t discarded;
      if (!hxRead(cells[i], discarded)) break;
    }
  }

  if (!cells[0].zeroOffset || !cells[1].zeroOffset) {
    emitLine("# STATUS,Send TARE with the complete stand unloaded");
  }
  if (!isfinite(cells[0].countsPerKg) || !isfinite(cells[1].countsPerKg)) {
    emitLine("# STATUS,Calibrate each sensor separately with CALLBF1 and CALLBF2");
  }
}

void loop() {
  readCommands(Serial, serialCommand);
  serviceWifiClient();

  if (digitalRead(cells[0].doutPin) == LOW && digitalRead(cells[1].doutPin) == LOW) {
    int32_t raw[2];
    if (hxRead(cells[0], raw[0], 20) && hxRead(cells[1], raw[1], 20)) {
      double kgf[2] = {NAN, NAN};
      double filteredKgf[2] = {NAN, NAN};
      for (uint8_t i = 0; i < 2; ++i) {
        if (isfinite(cells[i].countsPerKg)) {
          kgf[i] = static_cast<double>(static_cast<int64_t>(raw[i]) - cells[i].zeroOffset) /
                   cells[i].countsPerKg;
          if (!isfinite(cells[i].filteredKgf)) cells[i].filteredKgf = kgf[i];
          else cells[i].filteredKgf += EMA_ALPHA * (kgf[i] - cells[i].filteredKgf);
          filteredKgf[i] = cells[i].filteredKgf;
        }
      }

      const double totalKgf = kgf[0] + kgf[1];
      const double totalForceN = totalKgf * STANDARD_GRAVITY;
      const double filteredTotalN = (filteredKgf[0] + filteredKgf[1]) * STANDARD_GRAVITY;
      emitf("DATA,%lu,%ld,%ld,%.6f,%.6f,%.6f,%.6f,%.6f",
            millis(), raw[0], raw[1], kgf[0], kgf[1], totalKgf,
            totalForceN, filteredTotalN);
    }
  }
}
