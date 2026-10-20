#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <Servo.h>

// ==========================================
// 1. HARDWARE PIN DEFINITIONS & CONSTANTS
// ==========================================
// NodeMCU ESP-12E Pin Map:
// D1 = GPIO 5 (Steering Servo)
// D2 = GPIO 4 (Brushed ESC Throttle)
const uint8_t PIN_STEERING = 5; // D1
const uint8_t PIN_THROTTLE = 4; // D2

// Standard 50Hz PWM Servo Microsecond timings
const int PULSE_MIN_US     = 1000;
const int PULSE_NEUTRAL_US = 1500;
const int PULSE_MAX_US     = 2000;

// Watchdog / Failsafe threshold (ms)
const unsigned long FAILSAFE_TIMEOUT_MS = 500;

// ==========================================
// 2. NETWORK & SERVER CONFIGURATION
// ==========================================
const char* AP_SSID = "RC_Car_AP";
const char* AP_PASS = "password123";

IPAddress local_ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

ESP8266WebServer server(80);
WebSocketsServer webSocket(81);

Servo steeringServo;
Servo escThrottle;

// State variables
volatile unsigned long lastPacketTime = 0;
volatile bool failsafeActive          = false;

// *** BOOST MODE STATE VARIABLES ***
volatile bool isBoostingActive     = false;
volatile unsigned long boostStartTime = 0; // Time when boost was activated
const unsigned long BOOST_DURATION_MS = 5000; // Boost lasts 5 seconds
const unsigned long COOLDOWN_DURATION_MS = 20000; // Must wait 20 seconds before next boost
volatile unsigned long lastBoostTime   = 0;  // Time of the last successful boost activation

// *** GYRO CONTROL CONSTANTS ***
const float GYRO_SCALE = 0.25f;               // Input scale factor (adjust for responsiveness)
const int     GYRO_THRESHOLD_DEG = 3;         // Minimum tilt before motion is applied
const int     GYRO_DEADZONE_US = 1490;        // Neutral PWM tolerance (±10us around 1500)
const char*   CALIBRATE_CMD = "GYRO_CAL";     // Command to reset zero-point heading

int currentThrottleUs                 = PULSE_NEUTRAL_US;
int currentSteeringUs                 = PULSE_NEUTRAL_US;

// ==========================================
// 3. EMBEDDED WEB PORTAL (PROGMEM)
// ==========================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no, viewport-fit=cover">
  <title>RC CAR // WEBSOCKET CONTROLLER</title>
  <style>
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      user-select: none;
      -webkit-user-select: none;
      -webkit-touch-callout: none;
    }
    body, html {
      width: 100%;
      height: 100%;
      overflow: hidden;
      background: #0a0c10;
      color: #e2e8f0;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace;
      touch-action: none;
    }
    #app {
      display: flex;
      flex-direction: column;
      width: 100vw;
      height: 100vh;
      position: relative;
    }
    /* Top Header Bar */
    header {
      height: 48px;
      background: #11141c;
      border-bottom: 1px solid #1e2533;
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 0 16px;
      z-index: 20;
    }
    .brand {
      display: flex;
      align-items: center;
      gap: 8px;
      font-size: 13px;
      font-weight: 700;
      letter-spacing: 1px;
      color: #f59e0b;
    }
    .status-group {
      display: flex;
      align-items: center;
      gap: 12px;
      font-size: 12px;
    }
    .badge {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 4px 10px;
      border-radius: 999px;
      font-weight: 600;
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 0.5px;
      background: #1e293b;
      color: #94a3b8;
    }
    .badge.connected {
      background: rgba(16, 185, 129, 0.15);
      color: #10b981;
      border: 1px solid rgba(16, 185, 129, 0.4);
    }
    .badge.disconnected {
      background: rgba(239, 68, 68, 0.15);
      color: #ef4444;
      border: 1px solid rgba(239, 68, 68, 0.4);
    }
    .dot {
      width: 7px;
      height: 7px;
      border-radius: 50%;
      background: currentColor;
    }
    .btn-toggle {
      background: #1e2533;
      border: 1px solid #334155;
      color: #cbd5e1;
      padding: 5px 12px;
      border-radius: 6px;
      font-size: 11px;
      font-weight: 600;
      cursor: pointer;
    }
    .btn-toggle:active {
      background: #334155;
    }
    /* Trim & Tuning Drawer */
    #tuning-panel {
      display: none;
      background: #141923;
      border-bottom: 1px solid #232c3d;
      padding: 10px 20px;
      display: flex;
      gap: 24px;
      justify-content: center;
      align-items: center;
      font-size: 12px;
      z-index: 15;
    }
    .tune-item {
      display: flex;
      align-items: center;
      gap: 10px;
    }
    .tune-item label {
      color: #94a3b8;
      font-weight: 600;
      min-width: 90px;
    }
    .tune-item input[type="range"] {
      width: 140px;
      accent-color: #f59e0b;
      cursor: pointer;
    }
    .tune-val {
      min-width: 44px;
      font-family: monospace;
      color: #f8fafc;
      font-weight: 700;
    }
    /* Main Control Arena */
    main {
      flex: 1;
      display: flex;
      position: relative;
      background: radial-gradient(circle at center, #131722 0%, #0a0c10 100%);
    }
    .control-zone {
      flex: 1;
      position: relative;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      padding: 16px;
    }
    .divider {
      width: 1px;
      background: linear-gradient(to bottom, transparent, #222c3e 20%, #222c3e 80%, transparent);
    }
    .zone-label {
      font-size: 11px;
      letter-spacing: 1.5px;
      color: #64748b;
      font-weight: 700;
      margin-bottom: 12px;
      text-transform: uppercase;
    }
    .readout {
      margin-top: 12px;
      font-family: monospace;
      font-size: 13px;
      font-weight: 700;
      color: #f59e0b;
      background: rgba(15, 23, 42, 0.7);
      padding: 4px 12px;
      border-radius: 4px;
      border: 1px solid #1e293b;
    }
    /* Throttle Vertical Pad */
    .vertical-track {
      width: 90px;
      height: 240px;
      background: #11151f;
      border: 2px solid #1e293b;
      border-radius: 45px;
      position: relative;
      box-shadow: inset 0 2px 8px rgba(0,0,0,0.6);
      display: flex;
      align-items: center;
      justify-content: center;
    }
    .track-center-line {
      position: absolute;
      top: 50%;
      left: 10px;
      right: 10px;
      height: 2px;
      background: #334155;
      transform: translateY(-50%);
    }
    .vertical-knob {
      width: 74px;
      height: 74px;
      border-radius: 50%;
      background: radial-gradient(circle at 35% 35%, #384252, #1a202c);
      border: 2px solid #475569;
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      box-shadow: 0 4px 12px rgba(0,0,0,0.5);
      pointer-events: none;
      transition: border-color 0.1s;
    }
    .vertical-knob::after {
      content: '';
      position: absolute;
      width: 28px;
      height: 4px;
      border-radius: 2px;
      background: #f59e0b;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
    }
    /* Steering Horizontal Pad */
    .horizontal-track {
      width: 260px;
      height: 90px;
      background: #11151f;
      border: 2px solid #1e293b;
      border-radius: 45px;
      position: relative;
      box-shadow: inset 0 2px 8px rgba(0,0,0,0.6);
      display: flex;
      align-items: center;
      justify-content: center;
    }
    .track-center-vert-line {
      position: absolute;
      left: 50%;
      top: 10px;
      bottom: 10px;
      width: 2px;
      background: #334155;
      transform: translateX(-50%);
    }
    .horizontal-knob {
      width: 74px;
      height: 74px;
      border-radius: 50%;
      background: radial-gradient(circle at 35% 35%, #384252, #1a202c);
      border: 2px solid #475569;
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      box-shadow: 0 4px 12px rgba(0,0,0,0.5);
      pointer-events: none;
      transition: border-color 0.1s;
    }
    .horizontal-knob::after {
      content: '';
      position: absolute;
      width: 4px;
      height: 28px;
      border-radius: 2px;
      background: #06b6d4;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
    }
    .active-knob {
      border-color: #f59e0b !important;
    }
    .landscape-hint {
      display: none;
      position: fixed;
      inset: 0;
      background: #090b10;
      z-index: 100;
      color: #f59e0b;
      font-size: 14px;
      font-weight: 700;
      justify-content: center;
      align-items: center;
      text-align: center;
      padding: 20px;
    }
    @media (orientation: portrait) {
      .landscape-hint {
        display: flex;
      }
    }
  </style>
</head>
<body>
  <div class="landscape-hint">
    ↺ ROTATE YOUR DEVICE TO LANDSCAPE FOR DUAL-THUMB RC CONTROLS
  </div>

  <div id="app">
    <header>
      <div class="brand">
        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <rect x="2" y="7" width="20" height="13" rx="2"></rect>
          <path d="M16 21V5a2 2 0 0 0-2-2h-4a2 2 0 0 0-2 2v16"></path>
        </svg>
        <span>RC // NODEMCU ESP-12E</span>
      </div>
      <div class="status-group">
        <div id="statusBadge" class="badge disconnected">
          <span class="dot"></span>
          <span id="statusText">CONNECTING</span>
        </div>
        <button id="tuneToggleBtn" class="btn-toggle">TUNING & TRIM</button>
        <!-- BOOST MODE OVERRIDE -->
        <button id="boostBtn" class="btn-toggle" style="background-color: #ef4444; border-color: #dc2626;">BOOST (LOCKED)</button>
      </div>
    </header>

    <div id="tuning-panel">
      <div class="tune-item">
        <label for="speedLimit">MAX THROTTLE:</label>
        <input type="range" id="speedLimit" min="20" max="100" value="65">
        <span id="speedVal" class="tune-val">65%</span>
      </div>
      <div class="tune-item">
        <label for="steerTrim">STEER TRIM:</label>
        <input type="range" id="steerTrim" min="-100" max="100" value="0">
        <span id="trimVal" class="tune-val">0µs</span>
      </div>
      <button id="resetTrimBtn" class="btn-toggle">RESET TRIM</button>
    </div>

    <main>
      <!-- LEFT THUMB: THROTTLE -->
      <section class="control-zone" id="throttleZone">
        <div class="zone-label">▲ FORWARD / ▼ REVERSE</div>
        <div class="vertical-track" id="throttleTrack">
          <div class="track-center-line"></div>
          <div class="vertical-knob" id="throttleKnob"></div>
        </div>
        <div class="readout" id="throttleReadout">1500 µs (NEUTRAL)</div>
      </section>

      <div class="divider"></div>

      <!-- RIGHT THUMB: STEERING -->
      <section class="control-zone" id="steeringZone">
        <div class="zone-label">◄ STEERING ►</div>
        <div class="horizontal-track" id="steeringTrack">
          <div class="track-center-vert-line"></div>
          <div class="horizontal-knob" id="steeringKnob"></div>
        </div>
        <div class="readout" id="steeringReadout">1500 µs (CENTER)</div>
      </section>
    </main>
  </div>

  <script>
    // State
    let ws = null;
    let throttleRaw = 0.0; // -1.0 (full reverse) to +1.0 (full forward)
    let steeringRaw = 0.0; // -1.0 (full left) to +1.0 (full right)

    let throttleTouchId = null;
    let steeringTouchId = null;

    let maxThrottlePercent = 65; // Speed ceiling for indoor safety
    let steeringTrimUs = 0;

    // Elements
    const statusBadge = document.getElementById('statusBadge');
    const statusText = document.getElementById('statusText');
    const throttleTrack = document.getElementById('throttleTrack');
    const throttleKnob = document.getElementById('throttleKnob');
    const throttleReadout = document.getElementById('throttleReadout');
    const steeringTrack = document.getElementById('steeringTrack');
    const steeringKnob = document.getElementById('steeringKnob');
    const steeringReadout = document.getElementById('steeringReadout');

    const tuneToggleBtn = document.getElementById('tuneToggleBtn');
    const tuningPanel = document.getElementById('tuning-panel');
    const speedLimitInput = document.getElementById('speedLimit');
    const speedVal = document.getElementById('speedVal');
    const steerTrimInput = document.getElementById('steerTrim');
    const trimVal = document.getElementById('trimVal');
    const resetTrimBtn = document.getElementById('resetTrimBtn');

    // Toggle Tuning
    tuneToggleBtn.addEventListener('click', () => {
      tuningPanel.style.display = (tuningPanel.style.display === 'flex') ? 'none' : 'flex';
    });

    speedLimitInput.addEventListener('input', (e) => {
      maxThrottlePercent = parseInt(e.target.value);
      speedVal.textContent = maxThrottlePercent + '%';
    });

    steerTrimInput.addEventListener('input', (e) => {
      steeringTrimUs = parseInt(e.target.value);
      trimVal.textContent = (steeringTrimUs > 0 ? '+' : '') + steeringTrimUs + 'µs';
      updateOutputs();
    });

    resetTrimBtn.addEventListener('click', () => {
      steerTrimInput.value = 0;
      steeringTrimUs = 0;
      trimVal.textContent = '0µs';
      updateOutputs();
    });

    // ==========================================
    // TOUCH / MOUSE CONTROLLER LOGIC
    // ==========================================
    const TRACK_V_TRAVEL = 160; // Max vertical travel range in px
    const TRACK_H_TRAVEL = 170; // Max horizontal travel range in px

    function updateThrottleVisual(normVal) {
      // normVal: -1.0 to 1.0 (Up is positive 1.0, Down is negative -1.0)
      const px = -(normVal * (TRACK_V_TRAVEL / 2));
      throttleKnob.style.transform = `translate(-50%, calc(-50% + ${px}px))`;
    }

    function updateSteeringVisual(normVal) {
      // normVal: -1.0 to 1.0 (Right is positive 1.0, Left is negative -1.0)
      const px = normVal * (TRACK_H_TRAVEL / 2);
      steeringKnob.style.transform = `translate(calc(-50% + ${px}px), -50%)`;
    }

    // Multi-touch tracking on Throttle Track
    throttleTrack.addEventListener('touchstart', (e) => {
      e.preventDefault();
      if (throttleTouchId === null) {
        const touch = e.changedTouches[0];
        throttleTouchId = touch.identifier;
        throttleKnob.classList.add('active-knob');
        handleThrottleMove(touch.clientY);
      }
    }, { passive: false });

    // Multi-touch tracking on Steering Track
    steeringTrack.addEventListener('touchstart', (e) => {
      e.preventDefault();
      if (steeringTouchId === null) {
        const touch = e.changedTouches[0];
        steeringTouchId = touch.identifier;
        steeringKnob.classList.add('active-knob');
        handleSteeringMove(touch.clientX);
      }
    }, { passive: false });

    window.addEventListener('touchmove', (e) => {
      for (let i = 0; i < e.changedTouches.length; i++) {
        const touch = e.changedTouches[i];
        if (touch.identifier === throttleTouchId) {
          handleThrottleMove(touch.clientY);
        } else if (touch.identifier === steeringTouchId) {
          handleSteeringMove(touch.clientX);
        }
      }
    }, { passive: false });

    function releaseTouch(id) {
      if (id === throttleTouchId) {
        throttleTouchId = null;
        throttleRaw = 0.0; // Auto-spring to neutral!
        throttleKnob.classList.remove('active-knob');
        updateThrottleVisual(0.0);
        updateOutputs();
      }
      if (id === steeringTouchId) {
        steeringTouchId = null;
        steeringRaw = 0.0; // Auto-spring to center!
        steeringKnob.classList.remove('active-knob');
        updateSteeringVisual(0.0);
        updateOutputs();
      }
    }

    window.addEventListener('touchend', (e) => {
      for (let i = 0; i < e.changedTouches.length; i++) {
        releaseTouch(e.changedTouches[i].identifier);
      }
    });

    window.addEventListener('touchcancel', (e) => {
      for (let i = 0; i < e.changedTouches.length; i++) {
        releaseTouch(e.changedTouches[i].identifier);
      }
    });

    // Touch position handlers
    function handleThrottleMove(clientY) {
      const rect = throttleTrack.getBoundingClientRect();
      const centerY = rect.top + rect.height / 2;
      const deltaY = clientY - centerY;
      const maxHalf = TRACK_V_TRAVEL / 2;
      // Upward motion (deltaY < 0) corresponds to positive forward throttle
      let norm = - (deltaY / maxHalf);
      norm = Math.max(-1.0, Math.min(1.0, norm));
      // Deadband around neutral
      if (Math.abs(norm) < 0.04) norm = 0.0;
      throttleRaw = norm;
      updateThrottleVisual(norm);
      updateOutputs();
    }

    function handleSteeringMove(clientX) {
      const rect = steeringTrack.getBoundingClientRect();
      const centerX = rect.left + rect.width / 2;
      const deltaX = clientX - centerX;
      const maxHalf = TRACK_H_TRAVEL / 2;
      // Right motion (deltaX > 0) is positive right steering
      let norm = deltaX / maxHalf;
      norm = Math.max(-1.0, Math.min(1.0, norm));
      if (Math.abs(norm) < 0.04) norm = 0.0;
      steeringRaw = norm;
      updateSteeringVisual(norm);
      updateOutputs();
    }

    // Mouse Fallbacks for PC testing
    let isMouseDownThrottle = false;
    let isMouseDownSteering = false;

    throttleTrack.addEventListener('mousedown', (e) => {
      isMouseDownThrottle = true;
      throttleKnob.classList.add('active-knob');
      handleThrottleMove(e.clientY);
    });

    steeringTrack.addEventListener('mousedown', (e) => {
      isMouseDownSteering = true;
      steeringKnob.classList.add('active-knob');
      handleSteeringMove(e.clientX);
    });

    window.addEventListener('mousemove', (e) => {
      if (isMouseDownThrottle) handleThrottleMove(e.clientY);
      if (isMouseDownSteering) handleSteeringMove(e.clientX);
    });

    window.addEventListener('mouseup', () => {
      if (isMouseDownThrottle) {
        isMouseDownThrottle = false;
        throttleRaw = 0.0;
        throttleKnob.classList.remove('active-knob');
        updateThrottleVisual(0.0);
        updateOutputs();
      }
      if (isMouseDownSteering) {
        isMouseDownSteering = false;
        steeringRaw = 0.0;
        steeringKnob.classList.remove('active-knob');
        updateSteeringVisual(0.0);
        updateOutputs();
      }
    });

    // Calculate Microseconds and update UI readout
    function calculatePwm() {
      // THROTTLE LIMIT SET TO 65% OF MAX POWER
const BOOST_LIMIT_PERCENT = 0.65;
// Throttle scaled by speed limiter: range +/- (500 * limit)
      const scale = maxThrottlePercent / 100.0;
      const tUs = Math.round(1500 - (throttleRaw * 500 * scale));
      
      // Steering with trim offset: clamped strictly between 1000 and 2000
      let sUs = Math.round(1500 - (steeringRaw * 500) + steeringTrimUs);
      sUs = Math.max(1000, Math.min(2000, sUs));

      return { tUs, sUs };
    }

    function updateOutputs() {
      const { tUs, sUs } = calculatePwm();
      let tLabel = "NEUTRAL";
      if (tUs > 1520) tLabel = "FWD";
      else if (tUs < 1480) tLabel = "REV";

      let sLabel = "CENTER";
      if (sUs > 1520) sLabel = "RIGHT";
      else if (sUs < 1480) sLabel = "LEFT";

      throttleReadout.textContent = `${tUs} µs (${tLabel})`;
      steeringReadout.textContent = `${sUs} µs (${sLabel})`;
    }

    // ==========================================
    // WEBSOCKET TELEMETRY & HEARTBEAT
    // ==========================================
    function initWebSocket() {
      const host = window.location.hostname || "192.168.4.1";
      const wsUrl = `ws://${host}:81/`;
      ws = new WebSocket(wsUrl);

      ws.onopen = () => {
        statusBadge.className = 'badge connected';
        statusText.textContent = 'ONLINE (LOW-LATENCY)';
      };

      ws.onclose = () => {
        statusBadge.className = 'badge disconnected';
        statusText.textContent = 'DISCONNECTED (FAILSAFE)';
        setTimeout(initWebSocket, 1000);
      };

      ws.onerror = () => {
        statusBadge.className = 'badge disconnected';
        statusText.textContent = 'ERROR';
        ws.close();
      };
    }

    // 25Hz transmission loop (every 40ms) provides smooth response and heartbeat
    setInterval(() => {
      if (ws && ws.readyState === WebSocket.OPEN) {
        const { tUs, sUs } = calculatePwm();
        // Send compact payload: "T:xxxx,S:xxxx"
        ws.send(`T:${tUs},S:${sUs}`);
      }
    }, 40);

    // Initial load
    updateOutputs();
    window.addEventListener('DOMContentLoaded', initWebSocket);
  </script>
</body>
</html>
)rawliteral";

// ==========================================
// 4. HTTP & WEBSOCKET EVENT HANDLERS
// ==========================================
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u disconnected -> Failsafe engaged\n", num);
      // Immediately stop motor on client disconnect
      escThrottle.writeMicroseconds(PULSE_NEUTRAL_US);
      failsafeActive = true;
      break;

    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[WS] Client #%u connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      lastPacketTime = millis();
      failsafeActive = false;
      break;
    }

    case WStype_TEXT: {
      // Check for specific commands first
      std::string payloadStr((char*)payload);

      // Raw Device Motion Data: yaw|tiltX|tiltY
      float yawDeg = 0, tiltX = 0, tiltY = 0;
      if (sscanf((char*)payload, "y:%f,tx:%f,ty:%f", &yawDeg, &tiltX, &tiltY) == 3) {
        // Apply gyro control — throttle from tiltY, steering from yaw+tiltX
        currentThrottleUs = calculateGyroPWM(tiltY);
        currentSteeringUs = calculateGyroHeadingPWM(yawDeg + tiltX);

        if (!isBoostingActive) {
          escThrottle.writeMicroseconds(currentThrottleUs);
          steeringServo.writeMicroseconds(currentSteeringUs);
        }

        lastPacketTime = millis(); // Valid heartbeat from gyro data
      }

      if (payloadStr == "BOOST_ON") {
        unsigned long now = millis();
        long timeSinceLastBoost = now - lastBoostTime;
        
        // 1. ENFORCE COOLDOWN
        if (!isBoostingActive && (timeSinceLastBoost >= COOLDOWN_DURATION_MS)) {
          isBoostingActive = true;
          boostStartTime = now;
          Serial.println("[BOOST] Boost activated! Maximum throttle engaged for 5s.");
          lastBoostTime = now; // Record success time
        } else if (isBoostingActive) {
           // Already boosting, ignore command
        } else {
          long timeLeft = COOLDOWN_DURATION_MS - timeSinceLastBoost;
          Serial.printf("[BOOST] Cooldown active. %ld ms remaining.\n", timeLeft);
        yawDeg = 0; tiltX = 0; tiltY = 0;     // Reset zero-point for next calibration cycle
        lastPacketTime = now;                // Treat any valid packet (even failed boost attempt) as a heartbeat
        }
 
      else if (payloadStr == "BOOST_OFF") {
        isBoostingActive = false;
        Serial.println("[BOOST] Boost manually deactivated.");
        lastPacketTime = millis(); // Heartbeat reset
        break;
      }

      // Standard Telemetry Packet: T:xxxx,S:yyyy
      else if (sscanf((char*)payload, "T:%d,S:%d", &parsedThrottle, &parsedSteering) == 2) {
        // Clamp bounds for hardware safety
        currentThrottleUs = constrain(parsedThrottle, PULSE_MIN_US, PULSE_MAX_US);
        currentSteeringUs = constrain(parsedSteering, PULSE_MIN_US, PULSE_MAX_US);

        if (!isBoostingActive) {
            // Apply to ESC and Servo only if not in Boost mode (Boost is handled by the loop() function)
            escThrottle.writeMicroseconds(currentThrottleUs);
            steeringServo.writeMicroseconds(currentSteeringUs);
        } 
        // If in boost, the loop() handles writing the max throttle, so we only need to update state variables here.

        // Reset watchdog timer (This is always a valid heartbeat)
        lastPacketTime = millis();


          failsafeactive = false;
          Serial.println("[FAILSAFE] Heartbeat restored. Resuming control.");
        }
      }
      break;
    }

    default:
      break;
  }

// ==========================================
// HELPER FUNCTIONS — GYRO CONTROL
    float yaw, tiltX, tiltY;
    if (sscanf((char*)payload.c_str(), "y:%f,tx:%f,ty:%f", &yaw, &tiltX, &tiltY) == 3) {
      currentThrottleUs = calculateGyroPWM(tiltY);
      currentSteeringUs = calculateGyroHeadingPWM(yaw + tiltX);

      if (!isBoostingActive) {
        escThrottle.writeMicroseconds(currentThrottleUs);
        steeringServo.writeMicroseconds(currentSteeringUs);
      }

      lastPacketTime = millis(); // Valid heartbeat from gyro data
    }
  
// ==========================================
int calculateGyroPWM(float degTilt, int deadzoneUs = GYRO_DEADZONE_US) {
  if (Math.abs(degTilt) < GYRO_THRESHOLD_DEG) return PULSE_NEUTRAL_US; // Dead zone
  float scaled = degTilt * GYRO_SCALE / GYRO_THRESHOLD_DEG;
  int pwm = Math.round(PULSE_NEUTRAL_US + (scaled * (PULSE_MAX_US - PULSE_MIN_US) / 2.0));
  return constrain(pwm, PULSE_MIN_US, PULSE_MAX_US);
}

int calculateGyroHeadingPWM(float degYaw, int deadzoneUs = GYRO_DEADZONE_US) {
  if (Math.abs(degYaw) < GYRO_THRESHOLD_DEG) return PULSE_NEUTRAL_US; // Dead zone
  float scaled = degYaw * GYRO_SCALE / GYRO_THRESHOLD_DEG;
  int pwm = Math.round(PULSE_NEUTRAL_US + (scaled * (PULSE_MAX_US - PULSE_MIN_US) / 2.0));
  return constrain(pwm, PULSE_MIN_US, PULSE_MAX_US);
}
  if (sscanf(payload, "y:%f,tx:%f,ty:%f", &yawDeg, &tiltX, &tiltY) != 3) return 0;
  currentThrottleUs = calculateGyroPWM(tiltY);
  currentSteeringUs = calculateGyroHeadingPWM(yawDeg + tiltX);
  if (!isBoostingActive) {
    escThrottle.writeMicroseconds(currentThrottleUs);
    steeringServo.writeMicroseconds(currentSteeringUs);
  }
  lastPacketTime = millis(); // Valid heartbeat
  return currentThrottleUs;
}

// ==========================================
// 5. ARDUINO SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n\n========================================");
  Serial.println("  NodeMCU ESP8266 RC CAR CONTROLLER     ");
  Serial.println("========================================");

  // CRITICAL: Pre-set neutral pulse BEFORE attaching to avoid start-up twitch/lurch
  escThrottle.writeMicroseconds(PULSE_NEUTRAL_US);
  steeringServo.writeMicroseconds(PULSE_NEUTRAL_US);

  escThrottle.attach(PIN_THROTTLE, PULSE_MIN_US, PULSE_MAX_US);
  steeringServo.attach(PIN_STEERING, PULSE_MIN_US, PULSE_MAX_US);
  Serial.printf("[PULSE] Attached ESC to D2 (GPIO %d), Steering to D1 (GPIO %d) @ 1500us\n", PIN_THROTTLE, PIN_STEERING);

  // Initialize SoftAP
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  bool apSuccess = WiFi.softAP(AP_SSID, AP_PASS);

  if (apSuccess) {
    Serial.printf("[WIFI] Access Point '%s' initialized.\n", AP_SSID);
    Serial.print("[WIFI] Static IP Address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("[WIFI] SoftAP initialization FAILED!");
  }

  // Setup Web Server
  server.on("/", HTTP_GET, handleRoot);
  server.onNotFound([]() {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
  });
  server.begin();
  Serial.println("[HTTP] Web Server started on port 80");

  // Setup WebSocket Server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("[WS] WebSocket Server started on port 81");

  lastPacketTime = millis();
  failsafeActive = false;
  Serial.println("[SYSTEM] Armed and ready for connection.");
}

void loop() {
  // Service network stacks
  webSocket.loop();
  server.handleClient();

  unsigned long now = millis();
  int targetThrottleUs = PULSE_NEUTRAL_US; // Default to neutral

  // 1. BOOST CHECK (Highest Priority)
  if (isBoostingActive && (now - boostStartTime < BOOST_DURATION_MS)) {
    targetThrottleUs = PULSE_MAX_US; // Force Max Throttle during active boost
  } 
  // 2. THROTTLE/FAILSAFE CHECK
  else if (!failsafeActive) {
    // Use the current throttled value received from web socket (and limited by JS client)
    targetThrottleUs = currentThrottleUs;
  } else {
    // Failsafe active: keep neutral
    targetThrottleUs = PULSE_NEUTRAL_US;
  }

  // Apply Throttle Command
  escThrottle.writeMicroseconds(targetThrottleUs);

  // Watchdog Failsafe Check (If no heartbeat received)
  if (!failsafeActive && (now - lastPacketTime > FAILSAFE_TIMEOUT_MS)) {
    failsafeActive = true;
    Serial.println("[FAILSAFE] WATCHDOG TRIGGERED! Throttle snapped to 1500us neutral.");
  }

  // Boost Timeout Check (Must run every loop iteration)
  if (isBoostingActive && (now - boostStartTime >= BOOST_DURATION_MS)) {
    isBoostingActive = false;
    Serial.println("[BOOST] Boost period ended. Reverting to normal throttle control.");
    // Note: The next cycle's logic will handle reverting the PWM value.
  }
}
