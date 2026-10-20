# AutoCar RC Controller & Tools

A dual-axis joystick-based remote controller for NodeMCU ESP8266, featuring web portal control, gyro tilt sensing, and boost mode.

---

## 📁 Workspace Structure

```
rndsidequests/
├── autoCar/                    # Main RC car firmware project
│   ├── src/                    # Source files (empty placeholder)
│   ├── autoCar.ino             # Arduino firmware for NodeMCU ESP-12E
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
3. Upload `autoCar\build\autoCar.ino.bin` and click **Update** (device auto-reboots).

#### Manual USB Upload via Automated Script:
```powershell
.\autoCar\flash.ps1
```

### Pin Configuration (NodeMCU ESP-12E)
| Function      | GPIO | NodeMCU Pin |
|---------------|------|-------------|
| Steering Servo | 5   | D1          |
| Throttle ESC   | 4   | D2          |

### Web Portal Controls
Connect to `http://[BOARD_IP]` for the embedded controller UI.

---

## 🔧 CP210x Driver Tools (`tools/cp210x_driver/`)

USB-to-UART bridge driver package for CP210x chips (Silicon Labs). Use to enable serial communication on devices using CP210x USB modules.

### Files
- `CP210x_Universal_Windows_Driver.zip` — Official driver installer
- `silabser.inf` / `silabser.cat` — Installation manifest & catalog
- `UpdateParameters.reg` — Registry tweak for auto-detection (optional)

---

## 📡 Protocol Format

### WebSocket Payload (port 81)
**Standard Control:**
```text
T:xxxx,S:yyyy
```
| Field | Range     | Meaning                  |
|-------|-----------|--------------------------|
| T     | 1000–2000 | Throttle PWM µs          |
| S     | 1000–2000 | Steering PWM µs          |

**Gyro Telemetry:**
```text
y:XY,tx:X,ty:X
```
Yaw (deg), tiltX, tiltY — used for autopilot mode.

**Boost Commands:**
| Command   | Effect                              |
|-----------|-------------------------------------|
| `BOOST_ON`  | Activate max throttle burst         |
| `BOOST_OFF` | Deactivate boost                    |

---

## 🎮 Controls (Web Portal)

- **Throttle pad**: Forward/Reverse with neutral center
- **Steering pad**: Left/Right with center spring return
- **Tuning panel**: Max speed limiter, steering trim
- **Boost button**: Temporary lock-on max throttle

### Gyro Mode
Send `y:XY,tx:X,ty:X` to enable tilt-controlled driving.

---

## 🛠️ Troubleshooting

| Issue                  | Solution                              |
|------------------------|---------------------------------------|
| "No port detected"     | Check device is in USB-Serial mode    |
| Upload fails           | Verify baud rate (115200)             |
| Motors twitch on boot  | Firmware sets neutral PWM upfront    |

---

## 📜 License

See individual component licenses. Main firmware uses standard Arduino license (CERN/Adafruit-style).