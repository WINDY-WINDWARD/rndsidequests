# AutoCar RC Controller & Tools

A dual-axis joystick-based remote controller for NodeMCU ESP8266, featuring web portal control, gyro tilt sensing with guided arming delay, HC-SR04 ultrasonic forward collision caution (40% throttle < 60cm, full stop <= 15cm), locked 65% speed limiter, and temporary 100% boost headroom.

---

## 📁 Workspace Structure

```
rndsidequests/
├── autoCar/                    # Main RC car firmware project
│   ├── src/                    # Source files (empty placeholder)
│   ├── autoCar.ino             # Arduino firmware for NodeMCU ESP-12E
│   ├── build.ps1               # Automated build script
│   ├── flash.ps1               # Automated upload script for Windows
│   └── index.html              # Embedded web portal source
├── tools/                      # Development utilities
│   └── cp210x_driver/          # CP210x USB-to-UART bridge driver (Windows)
└── README.md                   # This file
```

---

## 🏁 AutoCar Firmware (`autoCar/`)

### Overview
A web-based RC car controller for NodeMCU ESP-12E with:
- **Dual-axis virtual joysticks** (thumb controls on mobile)
- **Front Ultrasonic Auto-Brake & Telemetry** (HC-SR04 collision avoidance, 15 cm stop / 25 cm release)
- **Gyro tilt sensing** (optional autopilot mode)
- **Boost mode** (temporary max throttle - 5s burst, 20s cooldown)
- **Tuning panel** (speed limiter, steering trim)
- **OTA firmware updates** (web upload via ElegantOTA at `/update`)

### Quick Start

#### Over-The-Air (OTA) Web Portal Update:
```powershell
# Compile the firmware .bin file
.\autoCar\build.ps1
```
1. Connect to car Wi-Fi: **SSID:** `RC_Car_AP` | **Password:** `password123`
2. Open **`http://192.168.4.1/update`** or click **OTA UPDATE** in the controller web portal.
3. Upload `autoCar\build\autoCar.ino.bin` under the **Firmware** tab and click **Update** (device auto-reboots).

#### Manual USB Upload via Automated Script:
```powershell
.\autoCar\flash.ps1
```

### Pin Configuration (NodeMCU ESP-12E)
| Function                | GPIO | NodeMCU Pin | Operating Voltage | Notes |
|-------------------------|------|-------------|-------------------|-------|
| Steering Servo          | 5    | D1          | 5V Power / 3.3V Signal | 1000 µs = Right, 1500 µs = Center, 2000 µs = Left |
| Throttle ESC            | 4    | D2          | 5V Power / 3.3V Signal | 1000 µs = Forward, 1500 µs = Neutral, 2000 µs = Reverse |
| Ultrasonic Trigger      | 14   | D5          | 3.3V Signal       | HC-SR04 Trig (direct 3.3V logic) |
| Ultrasonic Echo         | 12   | D6          | 3.3V Input        | HC-SR04 Echo via 1kΩ/2kΩ voltage divider |
| Ultrasonic Power (VCC)  | —    | VIN         | 5V Supply         | Must be powered from 5V rail |
| Common Ground           | —    | GND         | Ground            | Common ground for NodeMCU, sensor & ESC |

### HC-SR04 Wiring & Voltage Divider
The HC-SR04 operates at 5V and outputs a 5V TTL pulse on its **Echo** pin. To protect the 3.3V ESP8266 GPIO inputs, a 1kΩ / 2kΩ resistor divider is required:
```text
HC-SR04 VCC   -----> NodeMCU VIN (5V)
HC-SR04 GND   -----> NodeMCU GND (Common Ground)
HC-SR04 Trig  -----> NodeMCU D5 (GPIO 14, 3.3V direct)

HC-SR04 Echo (5V)
       │
      [1kΩ / or 10kΩ]
       ├───> NodeMCU D6 (GPIO 12, 3.3V safe: 5V * 2k / (1k + 2k) = 3.33V)
      [2kΩ / or two 10kΩ in series = 20kΩ]
       │
      GND

*(Note: Can also be wired with three identical 10kΩ resistors: 10kΩ between Echo and D6, and two 10kΩ in series between D6 and GND).*
```

---

## 🛑 Collision Avoidance & Auto-Braking Logic

- **Non-blocking State Machine:** Samples distance at 60 ms cadence with a 30 ms echo timeout; runs in `loop()` without `pulseIn()` or hardware interrupts to preserve IRAM headroom.
- **Valid Range:** 2 cm to 400 cm.
- **Stop Threshold:** `<= 15 cm` latches obstacle detection and forces forward throttle to neutral (`1500 µs`).
- **Release Threshold:** `>= 25 cm` releases obstacle latch (10 cm hysteresis prevents oscillating stops).
- **Directional Gating:** Gates **only** forward throttle (`< 1500 µs`). Reverse (`> 1500 µs`) and steering are **never** blocked, allowing immediate backing out of dead ends.
- **Fault Handling:**
  - Firmware boots with sensor marked unhealthy and forward blocked until the first valid echo is confirmed.
  - Three consecutive invalid readings or timeouts engage `SENSOR FAULT` and block forward motion.
- **Override Toggle:** The web portal features an `AUTO-BRAKE: ON/OFF` button (`AUTOBRAKE_OFF` / `AUTOBRAKE_ON`). Disabling auto-brake overrides both stop-zone obstacle blocks and sensor faults. Auto-brake returns to enabled (`ON`) upon device reboot (not persisted in flash).
- **Throttle Priority Hierarchy:**
  1. Watchdog failsafe neutral (highest priority, timeout > 500 ms)
  2. Auto-brake neutral (active when forward is blocked and forward throttle is requested)
  3. Collision caution zone 40% forward limit (`1300 µs` when < 60 cm)
  4. Boost mode temporary 100% forward headroom (`1000 µs` during 5s burst)
  5. Default locked 65% forward ceiling (`1175 µs`)
  6. Requested operator throttle (joystick / gyro)
- **Boost Safety:** Active boost cancels immediately if an obstacle is detected. Boost activation requests (`BOOST_ON`) are rejected while forward is blocked.

---

## 📡 Protocol Format

### WebSocket Payload (Port 81)

**Standard Joystick Telemetry (25 Hz):**
```text
T:xxxx,S:yyyy
```
| Field | Range     | Meaning                                        |
|-------|-----------|------------------------------------------------|
| T     | 1000–2000 | Throttle PWM µs (1000=FWD, 1500=NEUTRAL, 2000=REV) |
| S     | 1000–2000 | Steering PWM µs (1000=RIGHT, 1500=CENTER, 2000=LEFT) |

**Gyro Telemetry:**
```text
y:XY,tx:X,ty:X
```
Yaw (deg), tiltX (roll), tiltY (pitch) — used for tilt driving.

**Client Commands:**
| Command          | Effect                                       |
|------------------|----------------------------------------------|
| `BOOST_ON`       | Activate max throttle burst (rejected if blocked) |
| `BOOST_OFF`      | Deactivate boost                             |
| `AUTOBRAKE_ON`   | Enable ultrasonic auto-brake & obstacle latch |
| `AUTOBRAKE_OFF`  | Override/disable auto-brake                  |
| `GYRO_CAL`       | Recalibrate gyro level zero                  |

**Server-to-Client Telemetry (200 ms cadence & state changes):**
```text
U:<cm>,<healthy>,<forwardBlocked>,<autoBrakeEnabled>
```
| Field             | Value Range | Meaning                                    |
|-------------------|-------------|--------------------------------------------|
| `cm`              | 0–400       | Distance in cm (`0` = invalid or fault)    |
| `healthy`         | 0 or 1      | `1` = sensor healthy, `0` = sensor fault   |
| `forwardBlocked`  | 0 or 1      | `1` = forward throttle gated, `0` = clear  |
| `autoBrakeEnabled`| 0 or 1      | `1` = auto-brake active, `0` = overridden  |

---

## 🎮 Controls (Web Portal)

- **Top-Center Radar Overlay:** Displays live distance readout, status badge (`CLEAR`, `STOP ZONE`, `SENSOR FAULT`, `OVERRIDE`, `LINK LOST`), and `AUTO-BRAKE: ON/OFF` toggle button.
- **Throttle pad:** Forward (slide up) / Reverse (slide down) with neutral center.
- **Steering pad:** Left (slide left) / Right (slide right) with spring return.
- **Tuning panel:** Max speed limiter slider (20–100%) and steering trim slider (-100 to +100 µs).
- **Boost button:** 5-second burst to 1000 µs forward (with 20s cooldown); dynamically disabled and badged `BOOST (BLOCKED)` when forward path is blocked.

---

## 🛠️ Troubleshooting

| Issue                     | Solution                                                              |
|---------------------------|-----------------------------------------------------------------------|
| "SENSOR FAULT" displayed  | Check Echo voltage divider (1k/2k), Trig pin D5, and 5V supply to VCC |
| Car will not move forward | Obstacle detected (< 15 cm) or sensor fault. Click `AUTO-BRAKE: OFF` to override |
| "No port detected"        | Connect via Wi-Fi AP and use OTA update (`http://192.168.4.1/update`) |
| Reverse works, forward stops | Auto-brake engaged; clear the front area or toggle auto-brake override |

---

## 📜 License

See individual component licenses. Main firmware uses standard Arduino license (CERN/Adafruit-style).
