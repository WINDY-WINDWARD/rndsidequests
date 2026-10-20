# AutoCar Firmware & Web Portal — Comprehensive Fix Summary

## [P0] Critical Compilation Blockers (Resolved)
1. **Incomplete String Type:** Replaced non-standard `std::string` calls with Arduino `String`.
2. **Undeclared Functions:** Positioned `calculateGyroPWM` and `calculateGyroHeadingPWM` helper functions above `webSocketEvent`.
3. **Invalid C++ Math Identifiers:** Replaced JavaScript `Math.abs()` and `Math.round()` with `<cmath>` standard `std::fabs()` and `std::round()`.
4. **Mangled Control Flow:** Rebuilt the `WStype_TEXT` switch case block with clean braces, resolving unclosed `if (payloadStr == "BOOST_ON")` and misplaced `break` statements.
5. **Undeclared Variables:** Re-declared `parsedThrottle` and `parsedSteering` bounded properly.
6. **Stray Global Scope Code:** Removed duplicate floating snippets and misplaced return statements outside function definitions.

---

## [P0] Failsafe Priority & Safety Inversion (Resolved)
1. **Watchdog Priority over Boost:** In `loop()`, failsafe evaluation now strictly precedes Boost mode. If a client disconnects or connection drops, throttle snaps immediately to `1500µs` neutral, preventing runaway acceleration.
2. **Disconnect State Reset:** `WStype_DISCONNECTED` explicitly sets `isBoostingActive = false` and writes neutral PWM to the ESC.
3. **Watchdog Cancellation:** Watchdog timeout immediately sets `isBoostingActive = false`.

---

## [P1] Directional Inversion & Control Math (Resolved)
1. **Throttle Axis:** Fixed subtraction to addition `1500 + (throttleRaw * 500 * scale)` so forward upward thumb motion produces forward PWM (`>1500µs`).
2. **Steering Axis:** Fixed subtraction to addition `1500 + (steeringRaw * 500) + steeringTrimUs` so rightward thumb motion produces right turn PWM (`>1500µs`).

---

## [P1] Gyro Tilt Control Implementation (Resolved)
1. **Web Portal Sensor Integration:** Added `GYRO: OFF/ON` toggle button and `CALIBRATE` button with iOS permission handling (`DeviceOrientationEvent.requestPermission`).
2. **Relative Zero Calibration:** Captures current orientation on enable or calibrate click to zero out pitch/roll offsets.
3. **Touch Override:** Actively touching thumb joysticks takes immediate priority over gyro sensor streaming.
4. **Firmware Telemetry:** Firmware parses `y:%f,tx:%f,ty:%f` packets with deadband and deflection limits.

---

## [P1] Boost Mode UI Synchronization (Resolved)
1. Synchronized `autoCar/index.html` and embedded PROGMEM `INDEX_HTML`.
2. Added visual state styling: normal (`BOOST`), active burst (`BOOSTING (Xs)`), and cooldown (`COOLDOWN (Ys)`).
3. Enforces 5-second burst and 20-second cooldown in both client UI and ESP8266 firmware.

---

## [P2] Repository & Tooling Hygiene (Resolved)
1. Added `.gitignore` to prevent tracking build artifacts, binaries, and local worktrees.
2. Made `flash.ps1` dynamically locate `arduino-cli` and sketch paths.

---

## Verification Summary
- **Compilation:** Clean build with `esp8266:esp8266:nodemcuv2` (0 errors, 0 warnings). RAM: 39%, Flash: 29%.
- **Validation:** JavaScript syntax validated with Node.js parser.
