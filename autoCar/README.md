# AutoCar Firmware

NodeMCU ESP-12E RC Car Controller — Dual-axis joystick web portal with gyro tilt control and boost mode.

---

## Quick Start (Windows)

### Option 1: Over-The-Air (OTA) Web Portal (Recommended)
No USB cables needed once flashed once:
1. Compile the firmware binary:
   ```powershell
   .\build.ps1
   ```
   *(Builds to `build/autoCar.ino.bin`)*
2. Power on the RC car and connect to its Wi-Fi AP (`RC_Car_AP` / `password123`).
3. Open `http://192.168.4.1/update` (or click **OTA UPDATE** in the controller web portal).
4. Select **Firmware**, choose `build/autoCar.ino.bin`, and click **Update**.
5. The car automatically reboots into the new firmware.

### Option 2: USB Cable Flashing
```powershell
.\flash.ps1   # Compiles and auto-detects USB serial board
```

---

## Hardware Pinout (NodeMCU ESP-12E)

| Function      | GPIO | NodeMCU Label |
|---------------|------|---------------|
| Steering Servo | 5   | D1            |
| Throttle ESC   | 4   | D2            |

**Servo Pulse Windows:** 1000–2000 µs (neutral @ 1500 µs)

---

## Features

- **Web Portal Controls**: Dual thumb joysticks for mobile devices
- **Gyro Tilt Control**: Optional autopilot using tilt/yaw data
- **Boost Mode**: Temporary max throttle burst (5s on, 20s cooldown)
- **Tuning Panel**: Speed limiter & steering trim
- **OTA Web Updates**: In-browser firmware flashing via ElegantOTA (`/update`)

---

## Web Portal (Port 81)

Connect to `http://[BOARD_IP]` for the embedded controller UI.

| Control       | Behavior                              |
|---------------|---------------------------------------|
| Throttle pad  | Forward/Reverse with neutral center   |
| Steering pad  | Left/Right with spring return         |
| Boost button  | Lock-on max throttle (5s burst)       |
| Tuning panel  | Max speed limiter, steering trim      |

**Gyro mode**: Send `y:XY,tx:X,ty:X` for tilt-based control.

---

## Protocol Format

### Standard Control Payload
```text
T:1600,S:1350   # T = throttle µs, S = steering µs (both 1000–2000)
```

### Gyro Telemetry
```text
y:4.2,tx:-1.5,ty:3.0   # yaw, tiltX, tiltY in degrees
```

### Boost Commands
| Command      | Effect                               |
|--------------|--------------------------------------|
| `BOOST_ON`    | Activate max throttle burst          |
| `BOOST_OFF`   | Deactivate boost                     |

---

## Tuning Constants (Edit `autoCar.ino`)

| Constant                    | Default     | Purpose                           |
|-----------------------------|-------------|-----------------------------------|
| `BOOST_DURATION_MS`         | 5000       | Boost burst length                |
| `COOLDOWN_DURATION_MS`      | 20000      | Time between boosts               |
| `GYRO_SCALE`                | 0.25       | Motion sensitivity factor         |
| `GYRO_THRESHOLD_DEG`        | 3          | Deadzone angle (°)                |
| `MAX_THROTTLE_PERCENT`      | 65         | Speed limiter ceiling (%)         |

---

## Build Requirements

- Arduino CLI: https://arduino-cli.github.io/
- Platform: `esp8266:esp8266:nodemcuv2` (by esp8266 community)

---

## Serial Monitoring

```bash
# Windows PowerShell
$port = [System.IO.Ports.SerialPort]::GetPortNames() | Where-Object { $_ -ne "COM1" } | Select-Object -First 1
Start-Process powershell -ArgumentList "-NoExit", "-Command", "& '$port' $cli monitor --bailout -p"
```

**Or use `arduino-cli monitor` from terminal.**

---

## Troubleshooting

| Symptom                    | Solution                                      |
|---------------------------|-----------------------------------------------|
| "No port detected"        | Verify device is in USB-Serial mode           |
| Upload fails              | Confirm baud rate set to 115200 in config    |
| Motors twitch on startup  | Neutral PWM is pre-set before servo attach   |
| Web portal not loading    | Check IP address matches AP gateway          |

---

## Files

- `autoCar.ino` — Main firmware source (~870 lines)
- `flash.ps1` — Automated Windows upload script
- `index.html` — Embedded web portal (single HTML+embedded JS/CSS)

---

## Notes

- Firmware sets neutral PWM on startup to prevent motor lurch.
- Web controls auto-reset to neutral when released ("spring return").
- Boost mode overrides throttle limits for short bursts.
- Gyro calibration resets automatically after `GYRO_CAL` command.