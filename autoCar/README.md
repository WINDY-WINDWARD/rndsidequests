# AutoCar Firmware

NodeMCU ESP-12E RC Car Controller — Dual-axis joystick web portal with HC-SR04 ultrasonic collision avoidance, gyro tilt control, and boost mode.

---

## Quick Start (Windows)

### Option 1: Over-The-Air (OTA) Web Portal (Recommended)
No USB cables needed once flashed once:
1. Compile the firmware binary:
   ```powershell
   .\build.ps1
   ```
   *(Builds to `build\autoCar.ino.bin`)*
2. Power on the RC car and connect to its Wi-Fi AP (`RC_Car_AP` / `password123`).
3. Open `http://192.168.4.1/update` (or click **OTA UPDATE** in the controller web portal).
4. Select **Firmware**, choose `build\autoCar.ino.bin`, and click **Update**.
5. The car automatically reboots into the new firmware.

### Option 2: USB Cable Flashing
```powershell
.\flash.ps1   # Compiles and auto-detects USB serial board
```

---

## Hardware Pinout (NodeMCU ESP-12E)

| Function                | GPIO | NodeMCU Label | Operating Level | Notes |
|-------------------------|------|---------------|-----------------|-------|
| Steering Servo          | 5    | D1            | 5V VCC / 3.3V PWM | 1000 µs = Right, 1500 µs = Center, 2000 µs = Left |
| Throttle ESC            | 4    | D2            | 5V VCC / 3.3V PWM | 1000 µs = Forward, 1500 µs = Neutral, 2000 µs = Reverse |
| Ultrasonic Trigger      | 14   | D5            | 3.3V Output     | HC-SR04 Trig pin (10 µs pulse) |
| Ultrasonic Echo         | 12   | D6            | 3.3V Input      | HC-SR04 Echo pin via 1kΩ / 2kΩ divider |
| Ultrasonic VCC          | —    | VIN           | 5V Supply       | 5V power rail (required for HC-SR04) |
| Common Ground           | —    | GND           | Ground          | Shared common ground across all modules |

### HC-SR04 Echo Voltage Divider Schematic
The HC-SR04 operates at 5V and outputs 5V logic on its Echo line. To safely interface with the 3.3V NodeMCU ESP8266 GPIOs:
```text
HC-SR04 VCC   -----> NodeMCU VIN (5V rail)
HC-SR04 GND   -----> NodeMCU GND
HC-SR04 Trig  -----> NodeMCU D5 (GPIO 14)

HC-SR04 Echo (5V)
       │
      [1kΩ Resistor / or 10kΩ]
       ├───> NodeMCU D6 (GPIO 12, ~3.33V logic high)
      [2kΩ Resistor / or two 10kΩ in series = 20kΩ]
       │
      GND

*(Note: Three 10kΩ resistors can be used: 10kΩ between Echo and D6, and two 10kΩ in series between D6 and GND).*
```

---

## Features

- **Front HC-SR04 Radar & Auto-Brake**: Collision avoidance at <= 80 cm (full stop), 150 cm caution zone (forward throttle reduced to 40%), release at >= 90 cm, 60 ms sampling, and web portal override.
- **Live Ping Latency Display**: Real-time round-trip latency badge next to the online status board, checked every 5 seconds.
- **Web Portal Controls**: Dual thumb joysticks styled in dark-amber terminal aesthetic.
- **Direction & Pulse Logic**: Physical ESC/steering calibrated (1000 µs forward/right, 1500 µs neutral/center, 2000 µs reverse/left).
- **Gyro Tilt Control**: Full end-to-end steering servo travel (1000–2000 µs) calibrated to 25° tilt angle.
- **Boost Mode**: Temporarily raises forward throttle ceiling from locked 65% to 100% (5s burst, 20s cooldown; does not force automatic acceleration), automatically cancelled and rejected when in collision caution (< 150 cm) or stop zone (<= 80 cm).
- **Tuning Panel**: Speed limiter locked to 65% (user override disallowed; boost unlocks 100% temporary headroom), steering trim adjustment.
- **OTA Web Updates**: In-browser firmware flashing via ElegantOTA (`/update`) with neutral motor lock during upload.

---

## Collision Avoidance & Safety State Machine

- **Cadence & Range:** 60 ms sample cadence, 30 ms echo timeout (~515 cm), valid measurement range 2–400 cm.
- **Non-blocking Sampling:** Non-blocking state machine in `loop()` without `pulseIn()` or interrupts to maintain IRAM headroom and low-latency Wi-Fi handling.
- **Obstacle Latch:**
  - Engages when distance `<= 80 cm`, forcing forward throttle to neutral (`1500 µs`).
  - Clears when distance `>= 90 cm` (10 cm hysteresis prevents throttle chatter).
  - Collision caution zone active when distance `< 150 cm`, limiting forward throttle to 40% (`1300 µs`).
- **Selective Direction Gating:**
  - Only forward throttle (`< 1500 µs`) is gated.
  - Reverse throttle (`> 1500 µs`) and steering are **never** blocked, allowing immediate backing out of dead ends.
- **Fault Tolerance:**
  - Firmware boots with sensor marked unhealthy and forward blocked until a verified valid echo is received.
  - Three consecutive invalid readings or timeouts engage `SENSOR FAULT` and gate forward motion.
- **Web Portal Override:**
  - Clicking `AUTO-BRAKE: ON/OFF` sends `AUTOBRAKE_OFF` to bypass auto-braking and sensor faults.
  - Setting is in-memory only and resets to enabled (`ON`) upon reboot.
- **Throttle Priority Hierarchy:**
  1. Watchdog failsafe neutral (highest priority, timeout > 500 ms)
  2. Auto-brake neutral (active when forward path is blocked)
  3. Boost mode maximum forward (`1000 µs`)
  4. Requested operator throttle (joystick / gyro)

---

## Web Portal (Port 81)

Connect to `http://[BOARD_IP]` for the embedded controller UI.

| Control               | Behavior                                                     |
|-----------------------|--------------------------------------------------------------|
| Front Radar Overlay   | Live distance (cm), state badge, alert banner, auto-brake toggle |
| Throttle pad          | Forward (1000 µs) / Reverse (2000 µs) with spring return     |
| Steering pad          | Right (1000 µs) / Left (2000 µs) with spring return          |
| Boost button          | 1000 µs forward burst (disabled when path is blocked)        |
| Auto-Brake button     | Toggle collision avoidance override (ON / OFF)               |
| Tuning panel          | Max speed limiter, steering trim                             |

---

## Protocol Format

### Standard Control Payload (Client -> Server, 25 Hz)
```text
T:1350,S:1600   # T = throttle µs, S = steering µs (both 1000–2000)
```

### Gyro Telemetry (Client -> Server)
```text
y:4.2,tx:-1.5,ty:3.0   # yaw, tiltX (roll), tiltY (pitch) in degrees
```

### Client Commands
| Command          | Effect                                                     |
|------------------|------------------------------------------------------------|
| `BOOST_ON`       | Engage 5-second forward burst (rejected if path blocked)   |
| `BOOST_OFF`      | Deactivate boost                                           |
| `AUTOBRAKE_ON`   | Enable auto-brake collision avoidance                      |
| `AUTOBRAKE_OFF`  | Override/disable auto-brake                                |
| `GYRO_CAL`       | Zero current orientation angles                            |
| `PING:<ts>`      | Echo test for latency (`PONG:<ts>` response)              |

### Ultrasonic Telemetry (Server -> Client, 200 ms cadence & state changes)
```text
U:<cm>,<healthy>,<forwardBlocked>,<autoBrakeEnabled>
```
Example: `U:12,1,1,1` (12 cm distance, sensor healthy, forward blocked, auto-brake enabled).

---

## Tuning Constants (Edit `autoCar.ino`)

| Constant                       | Default | Purpose                                    |
|--------------------------------|---------|--------------------------------------------|
| `PIN_ULTRASONIC_TRIG`          | 14      | NodeMCU D5 / GPIO 14                       |
| `PIN_ULTRASONIC_ECHO`          | 12      | NodeMCU D6 / GPIO 12                       |
| `US_MEASURE_CADENCE_MS`        | 60      | Ultrasonic sampling cadence                |
| `US_ECHO_TIMEOUT_US`           | 30000   | Echo timeout in microseconds               |
| `DISTANCE_STOP_CM`             | 80      | Auto-brake trigger threshold (cm)          |
| `DISTANCE_CLEAR_CM`            | 90      | Auto-brake release threshold (cm)          |
| `DISTANCE_SLOW_CM`             | 150     | Collision caution slowdown threshold (cm)  |
| `SENSOR_FAULT_THRESHOLD`       | 3       | Consecutive invalid readings before fault  |
| `BOOST_DURATION_MS`            | 5000    | Boost burst length                         |
| `COOLDOWN_DURATION_MS`         | 20000   | Time between boosts                        |
| `GYRO_SCALE`                   | 1.00    | Motion sensitivity factor (full travel)    |
| `GYRO_MAX_TILT_DEG`            | 25.0    | Max tilt angle for full deflection (deg)   |
| `MAX_THROTTLE_PERCENT`         | 65      | Default speed limiter ceiling (%)          |

---

## Build & OTA Verification

Compile using the Arduino CLI:
```powershell
.\build.ps1
```
Expected output:
- RAM: ~40% (32,440 / 80,192 bytes)
- IRAM: 93% (61,423 / 65,536 bytes) — Preserves IRAM headroom
- Flash: ~32% (341,204 / 1,048,576 bytes) — Well within 1019 KB OTA slot
- Binary: `autoCar\build\autoCar.ino.bin`

Flash via browser:
1. Connect to Wi-Fi `RC_Car_AP`
2. Open `http://192.168.4.1/update`
3. Upload `autoCar\build\autoCar.ino.bin` under **Firmware**
