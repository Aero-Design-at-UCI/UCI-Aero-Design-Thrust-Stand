# ESP32 dual-load-cell HX711 thrust stand

This project reads two 20 kg load cells through two HX711 modules, streams both readings and their summed thrust from one ESP32 over USB or Wi-Fi, saves the data to CSV, and plots pounds-force (`lbf`) in real time. `1 lbf = 4.448221615 N`.

## Confirmed hardware parameters

- Connected ESP32 USB/UART: Silicon Labs CP210x on **COM7**
- Two HX711 modules: 24-bit ADC, channel A gain 128
- HX711 supply range: 2.6–5.5 V; this setup uses **3.3 V** for ESP32-safe logic
- HX711 output: normally 10 samples/s; 80 samples/s only if the board's RATE jumper/pin is configured for it
- Load-cell capacity: 20 kg (44.09 lbf) per cell; neither cell may exceed its individual rating
- Serial link: 115200 baud
- Wireless link: ESP32 access point `ThrustStand`, TCP port `3333`

Do not exceed the load cell's rated load. Rocket/motor thrust can produce shock and off-axis loads well above the steady reading, so use a rigid mount, a physical shield, remote operation, and an appropriate safety factor.

## Wiring

### Each load cell to its own HX711

Use the same wiring for each common WishioT four-wire cell:

| Load-cell wire | HX711 |
|---|---|
| Red | E+ |
| Black | E- |
| Green | A+ |
| White | A- |

If applied force reads negative, that is not dangerous or a calibration failure: the firmware learns the sign during calibration. Wire colors can vary, so use the markings supplied with your exact cell if they differ.

### Both HX711 modules to the ESP32

| Module | HX711 pin | ESP32 |
|---|---|---|
| HX711 1 | VCC | 3V3 |
| HX711 1 | GND | GND |
| HX711 1 | DT / DOUT | GPIO 4 |
| HX711 1 | SCK / CLK | GPIO 5 |
| HX711 2 | VCC | 3V3 |
| HX711 2 | GND | GND |
| HX711 2 | DT / DOUT | GPIO 18 |
| HX711 2 | SCK / CLK | GPIO 19 |

All grounds must be common. To use different pins, edit the two entries in the `cells` array near the top of the sketch.

## 1. Upload the ESP32 firmware

Open `firmware/thrust_stand_esp32/thrust_stand_esp32.ino` in Arduino IDE.

1. Install Espressif's **esp32** board package in Boards Manager.
2. Select your exact ESP32 board (for a common DevKit, use **ESP32 Dev Module**).
3. Leave `DEVICE_NUMBER = 1`, select the first ESP32's port, and upload it. This ESP creates the Wi-Fi network at `192.168.4.1`.
4. For an optional second ESP32, change the constant to `DEVICE_NUMBER = 2`, select that board's port, and upload again. ESP 2 joins ESP 1's network at `192.168.4.2`.

The firmware contains its own HX711 reader, so no HX711 Arduino library is required.

ESP 1 provides the shared network:

| Setting | Value |
|---|---|
| Wi-Fi name | `ThrustStand` |
| Password | `thrust20kg` |
| ESP 1 address | `192.168.4.1` |
| ESP 2 address | `192.168.4.2` |
| TCP port | `3333` |

Change `WIFI_SSID` and `WIFI_PASSWORD` near the top of the sketch if desired. The password must contain at least eight characters.

## 2. Install and run the PC logger

In PowerShell, from this project folder:

```powershell
.\run_logger.ps1
```

The first run creates a Python virtual environment and installs `pyserial` and `matplotlib`. If your ESP32 later appears on another port:

```powershell
.\run_logger.ps1 -Port COM8
```

Alternatively, install the requirements and launch the Python file directly:

```powershell
python -m pip install -r .\logger\requirements.txt
python .\logger\realtime_logger.py --port COM7
```

CSV files are written into `logs` with timestamps. Closing the graph flushes and closes the file cleanly.

### Run wirelessly

1. Power ESP 1 first, then ESP 2 if used. Each can use a USB power bank or another stable 5 V USB supply.
2. On the plotting computer, connect Wi-Fi to **ThrustStand** using password **thrust20kg**. Windows may report “No internet”; remain connected because this is a private instrument network.
3. Start the wireless logger:

```powershell
.\run_logger.ps1 -Wifi
```

Or launch it directly:

```powershell
python .\logger\realtime_logger.py --wifi
```

The logger initially connects to ESP 1. To add ESP 2, leave **Add host** set to `192.168.4.2` and click **+ Add ESP**. Use **Target ESP** to choose which board receives zero, calibration, and invert commands. Each connected ESP gets two separate load-cell plots, and a final plot shows combined thrust from every available load cell. Recording and PNG export preserve the same per-cell and combined layout.

The plotter supports one or two ESPs. Each ESP still accepts only one network logger connection, and USB serial remains available for diagnostics.

### Record and save a complete plot image

1. Click **Start recording** immediately before a test. This clears the previous image capture and begins recording every plotted sample.
2. Run the test for as long as needed. The live graph can continue scrolling normally.
3. Click **Stop & save plot**. A timestamped `thrust_plot_YYYYmmdd_HHMMSS.png` containing the entire recorded interval is saved in the `logs` folder beside the CSV—not only the last visible plot window.

## 3. Tare and calibrate

Each connected load cell must be calibrated independently before it can contribute accurate thrust data.

1. Leave the motor and all mounting hardware installed. Turn the motor off and make sure nothing is pushing or pulling on the rig.
2. Select the board in **Target ESP**, click **Zero target ESP**, and keep its complete rig still. This zeros every responding cell on that ESP and skips a disconnected cell.
3. Apply a precisely known force directly over cell 1 so the calibration load is carried by that sensor. Enter the force in pounds-force and click **Calibrate cell 1**.
4. Move the same known force directly over cell 2 and click **Calibrate cell 2**.
5. Remove the force. The total should return close to zero. If applied thrust is displayed negative, click **Invert target** once; every calibrated cell on that ESP is reversed.
6. If a second ESP is connected, select target `2` and repeat the calibration for its two sensors.

The ESP32 stores separate zero and calibration values for both sensors in nonvolatile memory. Before later tests, leave the complete rig installed and click **Zero assembled rig** after warm-up. Recalibrate a sensor when its mounting, wiring, excitation, or load cell changes.

## Measurement notes

- Each ESP has a separate plot for load cell 1 and load cell 2. The bottom plot sums every fresh, calibrated load cell across the connected ESPs. Faint lines are raw data and solid lines are filtered data.
- A missing, unpowered, or uncalibrated load cell is reported as `NaN`, bypassed automatically, and excluded from the combined total. The other load cell continues sampling normally.
- If a cell is responding but has no calibration, its individual panel switches to **ADC counts (uncalibrated)** so wiring and sensor activity remain visible. ADC counts never contribute to the pounds-force total.
- Upgrading from the original single-cell firmware automatically migrates its saved `zero` and `scale` values to load cell 1.
- The CSV retains the ESP identity, both raw ADC counts, both calibrated cell forces, and each ESP's available-cell total.
- The default 10 SPS mode is good for static measurements but may miss fast thrust peaks. If the HX711 module exposes a RATE pad/jumper, set RATE high for 80 SPS and restart. Many boards require a solder-jumper modification—verify the board markings first.
- Keep load-cell leads short, route them away from motor/igniter wiring, and use a rigid axial mount. USB power noise, cable strain, vibration, and temperature drift all affect readings.
- Run a test with a known load before any live motor test, and compare the recorded peak and steady value against that known load.
