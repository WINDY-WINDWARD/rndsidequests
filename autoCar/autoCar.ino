#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <Servo.h>
#include <cmath>
#include <ElegantOTA.h>

// ==========================================
// 1. HARDWARE PIN DEFINITIONS & CONSTANTS
// ==========================================
// NodeMCU ESP-12E Pin Map:
// D1 = GPIO 5 (Steering Servo)
// D2 = GPIO 4 (Brushed ESC Throttle)
// D5 = GPIO 14 (HC-SR04 Ultrasonic Trigger)
// D6 = GPIO 12 (HC-SR04 Ultrasonic Echo via 1k/2k divider)
const uint8_t PIN_STEERING = 5; // D1
const uint8_t PIN_THROTTLE = 4; // D2
const uint8_t PIN_ULTRASONIC_TRIG = 14; // D5
const uint8_t PIN_ULTRASONIC_ECHO = 12; // D6
const uint8_t PIN_LED_HEARTBEAT    = LED_BUILTIN; // GPIO 2 (D4, active-low ESP-12 LED)
const uint8_t PIN_LED_BOARD        = 16;          // GPIO 16 (D0, active-low NodeMCU board LED)

// Ultrasonic HC-SR04 Sensor Constants
const unsigned long US_MEASURE_CADENCE_MS     = 60;    // 60ms measurement cadence
const unsigned long US_PULSE_MIN_VALID_US    = 116;   // 2 cm (2 * 58.2)
const unsigned long US_PULSE_MAX_VALID_US    = 23280; // 400 cm (400 * 58.2)
const unsigned long US_RISE_TIMEOUT_US       = 8000;  // 8ms timeout for echo to rise after trigger (normal ~450us)
const unsigned long US_FALL_TIMEOUT_US       = 24000; // 24ms (~412cm) maximum echo wait duration
const int DISTANCE_MIN_VALID_CM               = 2;
const int DISTANCE_MAX_VALID_CM               = 400;
const int DISTANCE_STOP_CM                    = 80;    // Complete stop threshold (<=80cm)
const int DISTANCE_CLEAR_CM                   = 90;    // Auto-brake release threshold (>=90cm)
const int DISTANCE_SLOW_CM                    = 150;   // Collision range threshold (<150cm = 40% forward)
const uint8_t SENSOR_FAULT_THRESHOLD          = 3;     // 3 consecutive missed readings for fault

// Forward Throttle Limits
const float THROTTLE_LIMIT_DEFAULT            = 0.65f; // Default 65% forward max (user cannot override)
const float THROTTLE_LIMIT_SLOW               = 0.40f; // 40% forward max when < 150cm
const float THROTTLE_LIMIT_BOOST              = 1.00f; // 100% temporary headroom during boost

// Ultrasonic Non-Blocking State Machine States
enum UltrasonicState {
  US_IDLE,
  US_WAIT_RISE,
  US_WAIT_FALL,
  US_WAIT_LINE_CLEAR
};

UltrasonicState usState = US_IDLE;
unsigned long usLastMeasureTime = 0;
unsigned long usTriggerTime = 0;
unsigned long usEchoStartTime = 0;

int currentDistanceCm = 0;
bool sensorHealthy = false;          // Starts unhealthy until first valid echo
bool autoBrakeEnabled = true;        // Defaults ON every boot, not persisted
bool obstacleLatch = false;          // Latches at <=80cm, clears at >=90cm
bool forwardBlocked = true;          // Initially true because sensorHealthy is false & autoBrakeEnabled is true
uint8_t consecutiveInvalidCount = 0;

// 5-sample rolling window tracking filter (5 samples * 60ms = 300ms window)
const uint8_t DIST_WINDOW_SIZE = 5;
int distWindow[DIST_WINDOW_SIZE] = {0, 0, 0, 0, 0};
uint8_t distWindowIdx = 0;
bool distWindowFilled = false;

int applyDistanceFilter(int rawDistCm) {
  if (!distWindowFilled) {
    for (uint8_t i = 0; i < DIST_WINDOW_SIZE; i++) {
      distWindow[i] = rawDistCm;
    }
    distWindowFilled = true;
    return rawDistCm;
  }

  distWindow[distWindowIdx] = rawDistCm;
  distWindowIdx = (distWindowIdx + 1) % DIST_WINDOW_SIZE;

  int obstacleSum = 0;
  int obstacleCount = 0;
  for (uint8_t i = 0; i < DIST_WINDOW_SIZE; i++) {
    if (distWindow[i] > 0 && distWindow[i] <= DISTANCE_MAX_VALID_CM) {
      obstacleSum += distWindow[i];
      obstacleCount++;
    }
  }

  // If majority (>= 3 out of 5) detect an obstacle, average the obstacle readings
  if (obstacleCount >= 3) {
    return (int)round((float)obstacleSum / (float)obstacleCount);
  } else {
    return 0; // Path is clear (> 400 cm)
  }
}

unsigned long lastSensorBroadcastTime = 0;
const unsigned long SENSOR_BROADCAST_INTERVAL_MS = 200;

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

// Wi-Fi Station connection event handlers
WiFiEventHandler stationConnectedHandler;
WiFiEventHandler stationDisconnectedHandler;

Servo steeringServo;
Servo escThrottle;

// State variables
volatile unsigned long lastPacketTime = 0;
volatile bool failsafeActive          = false;

// Boost mode state variables
volatile bool isBoostingActive        = false;
volatile unsigned long boostStartTime = 0;
const unsigned long BOOST_DURATION_MS = 5000;      // 5-second max burst
const unsigned long COOLDOWN_DURATION_MS = 20000;  // 20-second cooldown
volatile unsigned long lastBoostTime  = 0;

// Gyro control parameters
const float GYRO_SCALE            = 1.0f;   // Full travel scaling (1.0 = full 1000us - 2000us deflection)
const float GYRO_THRESHOLD_DEG    = 3.0f;   // Deadzone angle
const float GYRO_MAX_TILT_DEG     = 25.0f;  // Max tilt angle for full deflection

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
      gap: 8px;
    }
    .brand {
      display: flex;
      align-items: center;
      gap: 8px;
      font-size: 13px;
      font-weight: 700;
      letter-spacing: 1px;
      color: #f59e0b;
      white-space: nowrap;
    }
    .actions-group {
      display: flex;
      align-items: center;
      gap: 8px;
      flex-wrap: nowrap;
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
      white-space: nowrap;
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
    .badge.ping {
      background: rgba(6, 182, 212, 0.12);
      color: #06b6d4;
      border: 1px solid rgba(6, 182, 212, 0.35);
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
      font-size: 11px;
      min-width: 64px;
      justify-content: center;
    }
    .badge.ping.warn {
      background: rgba(245, 158, 11, 0.12);
      color: #f59e0b;
      border: 1px solid rgba(245, 158, 11, 0.35);
    }
    .badge.ping.bad {
      background: rgba(239, 68, 68, 0.12);
      color: #ef4444;
      border: 1px solid rgba(239, 68, 68, 0.35);
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
      padding: 5px 10px;
      border-radius: 6px;
      font-size: 11px;
      font-weight: 600;
      cursor: pointer;
      white-space: nowrap;
      transition: background-color 0.2s, border-color 0.2s, color 0.2s;
    }
    .btn-toggle:active {
      background: #334155;
    }
    #boostBtn {
      background: rgba(239, 68, 68, 0.2);
      border-color: #ef4444;
      color: #fca5a5;
    }
    #boostBtn.active-boost {
      background: #ef4444;
      color: #ffffff;
      animation: pulse-red 0.6s infinite alternate;
    }
    #boostBtn.cooldown {
      background: #1e2533;
      border-color: #475569;
      color: #64748b;
      cursor: not-allowed;
    }
    #gyroToggleBtn.active-gyro {
      background: rgba(6, 182, 212, 0.2);
      border-color: #06b6d4;
      color: #67e8f9;
    }
    .gyro-status-badge {
      font-size: 10px;
      font-weight: 700;
      letter-spacing: 0.5px;
      padding: 3px 8px;
      border-radius: 4px;
      background: rgba(6, 182, 212, 0.15);
      border: 1px solid #06b6d4;
      color: #67e8f9;
      display: inline-flex;
      align-items: center;
      font-variant-numeric: tabular-nums;
    }
    .gyro-status-badge.arming {
      background: rgba(245, 158, 11, 0.2);
      border-color: #f59e0b;
      color: #fbbf24;
    }
    .gyro-status-badge.warn {
      background: rgba(239, 68, 68, 0.2);
      border-color: #ef4444;
      color: #fca5a5;
    }
    @keyframes pulse-red {
      from { transform: scale(1); }
      to { transform: scale(1.05); }
    }
    /* Front Ultrasonic Radar Overlay */
    .radar-overlay {
      position: absolute;
      top: 10px;
      left: 50%;
      transform: translateX(-50%);
      background: rgba(17, 20, 28, 0.94);
      border: 1px solid #1e2533;
      border-radius: 8px;
      padding: 6px 12px;
      display: flex;
      flex-direction: column;
      gap: 5px;
      z-index: 10;
      backdrop-filter: blur(6px);
      box-shadow: 0 4px 16px rgba(0, 0, 0, 0.6);
      min-width: 200px;
      pointer-events: auto;
      touch-action: manipulation;
    }
    .radar-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
    }
    .radar-title-group {
      display: flex;
      align-items: center;
      gap: 6px;
    }
    .radar-dot {
      width: 7px;
      height: 7px;
      border-radius: 50%;
      background: #94a3b8;
    }
    .radar-title {
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 1px;
      color: #94a3b8;
      text-transform: uppercase;
    }
    .radar-badge {
      font-size: 10px;
      font-weight: 700;
      letter-spacing: 0.5px;
      padding: 2px 7px;
      border-radius: 999px;
      text-transform: uppercase;
      background: #1e293b;
      color: #94a3b8;
      border: 1px solid #334155;
    }
    .radar-badge.clear {
      background: rgba(16, 185, 129, 0.15);
      color: #10b981;
      border-color: rgba(16, 185, 129, 0.4);
    }
    .radar-badge.stop-zone {
      background: rgba(239, 68, 68, 0.2);
      color: #ef4444;
      border-color: #ef4444;
      animation: pulse-red 0.5s infinite alternate;
    }
    .radar-badge.fault {
      background: rgba(239, 68, 68, 0.15);
      color: #f87171;
      border-color: rgba(239, 68, 68, 0.4);
    }
    .radar-badge.override {
      background: rgba(245, 158, 11, 0.15);
      color: #f59e0b;
      border-color: rgba(245, 158, 11, 0.4);
    }
    .radar-badge.caution {
      background: rgba(245, 158, 11, 0.2);
      color: #f59e0b;
      border-color: #f59e0b;
    }
    .radar-content {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
    }
    .radar-readout {
      display: flex;
      align-items: baseline;
      gap: 4px;
      font-family: monospace;
    }
    .radar-value {
      font-size: 20px;
      font-weight: 800;
      color: #f59e0b;
      min-width: 38px;
      text-align: right;
    }
    .radar-unit {
      font-size: 11px;
      font-weight: 700;
      color: #64748b;
    }
    .radar-toggle {
      font-size: 10px;
      padding: 4px 8px;
    }
    .radar-toggle.active-brake {
      background: rgba(16, 185, 129, 0.15);
      color: #10b981;
      border-color: rgba(16, 185, 129, 0.4);
    }
    .radar-toggle.override-brake {
      background: rgba(245, 158, 11, 0.2);
      color: #f59e0b;
      border-color: #f59e0b;
    }
    .radar-alert {
      font-size: 10px;
      font-weight: 700;
      letter-spacing: 0.5px;
      color: #fca5a5;
      background: rgba(239, 68, 68, 0.2);
      border: 1px solid rgba(239, 68, 68, 0.4);
      border-radius: 4px;
      padding: 3px 6px;
      text-align: center;
      text-transform: uppercase;
    }
    #boostBtn.blocked {
      background: #1e2533 !important;
      border-color: #ef4444 !important;
      color: #ef4444 !important;
      cursor: not-allowed !important;
      animation: none !important;
    }
    /* Trim & Tuning Drawer */
    #tuning-panel {
      display: none;
      background: #141923;
      border-bottom: 1px solid #232c3d;
      padding: 10px 16px;
      gap: 20px;
      justify-content: center;
      align-items: center;
      font-size: 12px;
      z-index: 15;
      flex-wrap: wrap;
    }
    .tune-item {
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .tune-item label {
      color: #94a3b8;
      font-weight: 600;
    }
    .tune-item input[type="range"] {
      width: 120px;
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
      <div class="actions-group">
        <button id="gyroToggleBtn" class="btn-toggle">GYRO: OFF</button>
        <button id="gyroCalBtn" class="btn-toggle" style="display:none;">CALIBRATE</button>
        <span id="gyroStatus" class="gyro-status-badge" style="display:none;"></span>
        <button id="boostBtn" class="btn-toggle">BOOST</button>
        <button id="tuneToggleBtn" class="btn-toggle">TUNING & TRIM</button>
        <a id="otaBtn" href="/update" target="_blank" class="btn-toggle" style="text-decoration:none; display:inline-flex; align-items:center;">OTA UPDATE</a>
        <div id="statusBadge" class="badge disconnected">
          <span class="dot"></span>
          <span id="statusText">CONNECTING</span>
        </div>
        <div id="pingBadge" class="badge ping">
          <span id="pingText">PING: --</span>
        </div>
      </div>
    </header>

    <div id="tuning-panel">
      <div class="tune-item">
        <label>MAX THROTTLE:</label>
        <span class="tune-val" style="color: #06b6d4; font-weight: 700;">65% (LOCKED // BOOST UNLOCKS 100%)</span>
      </div>
      <div class="tune-item">
        <label for="steerTrim">STEER TRIM:</label>
        <input type="range" id="steerTrim" min="-100" max="100" value="0">
        <span id="trimVal" class="tune-val">0µs</span>
      </div>
      <button id="resetTrimBtn" class="btn-toggle">RESET TRIM</button>
      <a id="otaTuningBtn" href="/update" target="_blank" class="btn-toggle" style="text-decoration:none; text-align:center; display:inline-flex; align-items:center; justify-content:center; color:#f59e0b; border-color:#f59e0b;">FIRMWARE OTA UPDATE</a>
    </div>

    <main>
      <!-- TOP-CENTER OVERLAY: FRONT SENSOR & AUTO-BRAKE -->
      <div id="radarOverlay" class="radar-overlay">
        <div class="radar-header">
          <div class="radar-title-group">
            <span class="radar-dot" id="radarDot"></span>
            <span class="radar-title">FRONT RADAR</span>
          </div>
          <span id="radarBadge" class="radar-badge fault">CONNECTING</span>
        </div>
        <div class="radar-content">
          <div class="radar-readout">
            <span id="radarDistance" class="radar-value">--</span>
            <span class="radar-unit">CM</span>
          </div>
          <button id="autoBrakeBtn" class="btn-toggle radar-toggle active-brake">AUTO-BRAKE: ON</button>
        </div>
        <div id="radarAlertBanner" class="radar-alert" style="display:none;"></div>
      </div>

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
    // ==========================================
    // STATE & VARIABLES
    // ==========================================
    let ws = null;
    let throttleRaw = 0.0; // -1.0 (reverse) to +1.0 (forward)
    let steeringRaw = 0.0; // -1.0 (left) to +1.0 (right)

    let throttleTouchId = null;
    let steeringTouchId = null;

    let maxThrottlePercent = 65;
    let steeringTrimUs = 0;

    // Boost Mode State
    const BOOST_DURATION_MS = 5000;
    const COOLDOWN_DURATION_MS = 20000;
    let isBoosting = false;
    let lastBoostTime = 0;
    let boostTimer = null;

    // Ultrasonic & Auto-Brake State
    let autoBrakeEnabled = true;
    let sensorHealthy = false;
    let isForwardBlocked = false;
    let currentDistanceCm = 0;
    let receivedFirstTelemetry = false;

    // Gyro Mode State
    let gyroActive = false;
    let rawBeta = 0;   // Forward / backward tilt
    let rawGamma = 0;  // Left / right tilt
    let zeroBeta = 0;
    let zeroGamma = 0;

    // DOM Elements
    const statusBadge = document.getElementById('statusBadge');
    const statusText = document.getElementById('statusText');
    const pingBadge = document.getElementById('pingBadge');
    const pingText = document.getElementById('pingText');
    let lastPingSentTime = 0;
    let pingIntervalId = null;

    function sendPing() {
      if (ws && ws.readyState === WebSocket.OPEN) {
        lastPingSentTime = performance.now();
        ws.send(`PING:${Math.round(lastPingSentTime)}`);
      }
    }
    const throttleTrack = document.getElementById('throttleTrack');
    const throttleKnob = document.getElementById('throttleKnob');
    const throttleReadout = document.getElementById('throttleReadout');
    const steeringTrack = document.getElementById('steeringTrack');
    const steeringKnob = document.getElementById('steeringKnob');
    const steeringReadout = document.getElementById('steeringReadout');

    const tuneToggleBtn = document.getElementById('tuneToggleBtn');
    const tuningPanel = document.getElementById('tuning-panel');
    const steerTrimInput = document.getElementById('steerTrim');
    const trimVal = document.getElementById('trimVal');
    const resetTrimBtn = document.getElementById('resetTrimBtn');

    const boostBtn = document.getElementById('boostBtn');
    const gyroToggleBtn = document.getElementById('gyroToggleBtn');
    const gyroCalBtn = document.getElementById('gyroCalBtn');
    const gyroStatus = document.getElementById('gyroStatus');
    let gyroArming = false;
    let gyroArmTimer = null;
    let gyroEventCount = 0;

    const radarOverlay = document.getElementById('radarOverlay');
    const radarBadge = document.getElementById('radarBadge');
    const radarDot = document.getElementById('radarDot');
    const radarDistance = document.getElementById('radarDistance');
    const autoBrakeBtn = document.getElementById('autoBrakeBtn');
    const radarAlertBanner = document.getElementById('radarAlertBanner');

    // ==========================================
    // UI CONTROLS & TUNING
    // ==========================================
    tuneToggleBtn.addEventListener('click', () => {
      tuningPanel.style.display = (tuningPanel.style.display === 'flex') ? 'none' : 'flex';
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
    // BOOST MODE
    // ==========================================
    function updateBoostButton() {
      const now = Date.now();
      if (isForwardBlocked) {
        boostBtn.className = 'btn-toggle blocked';
        boostBtn.textContent = 'BOOST (BLOCKED)';
        boostBtn.disabled = true;
        return;
      }
      const elapsedSinceBoost = now - lastBoostTime;

      if (isBoosting) {
        const remaining = Math.max(0, Math.ceil((BOOST_DURATION_MS - (now - boostStartTime)) / 1000));
        boostBtn.textContent = `BOOSTING (${remaining}s)`;
        boostBtn.className = 'btn-toggle active-boost';
        boostBtn.disabled = true;
      } else if (elapsedSinceBoost < COOLDOWN_DURATION_MS) {
        const remaining = Math.max(1, Math.ceil((COOLDOWN_DURATION_MS - elapsedSinceBoost) / 1000));
        boostBtn.textContent = `COOLDOWN (${remaining}s)`;
        boostBtn.className = 'btn-toggle cooldown';
        boostBtn.disabled = true;
      } else {
        boostBtn.textContent = 'BOOST';
        boostBtn.className = 'btn-toggle';
        boostBtn.disabled = false;
      }
    }

    let boostStartTime = 0;
    boostBtn.addEventListener('click', () => {
      const now = Date.now();
      if (isForwardBlocked || isBoosting || (now - lastBoostTime < COOLDOWN_DURATION_MS)) return;

      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send("BOOST_ON");
        isBoosting = true;
        boostStartTime = now;
        lastBoostTime = now;
        updateBoostButton();

        if (boostTimer) clearInterval(boostTimer);
        boostTimer = setInterval(() => {
          const currentTime = Date.now();
          if (isBoosting && (currentTime - boostStartTime >= BOOST_DURATION_MS)) {
            isBoosting = false;
            if (ws && ws.readyState === WebSocket.OPEN) {
              ws.send("BOOST_OFF");
            }
          }
          updateBoostButton();
          if (!isBoosting && (currentTime - lastBoostTime >= COOLDOWN_DURATION_MS)) {
            clearInterval(boostTimer);
            boostTimer = null;
          }
        }, 500);
      }
    });

    // ==========================================
    // ULTRASONIC RADAR & AUTO-BRAKE
    // ==========================================
    autoBrakeBtn.addEventListener('click', (e) => {
      e.stopPropagation();
      e.preventDefault();
      if (ws && ws.readyState === WebSocket.OPEN) {
        if (autoBrakeEnabled) {
          ws.send("AUTOBRAKE_OFF");
        } else {
          ws.send("AUTOBRAKE_ON");
        }
      }
    });

    function handleUltrasonicTelemetry(payload) {
      // Format: U:<cm>,<healthy>,<forwardBlocked>,<autoBrakeEnabled>
      const parts = payload.substring(2).split(',');
      if (parts.length < 4) return;
      currentDistanceCm = parseInt(parts[0], 10) || 0;
      sensorHealthy = (parts[1] === '1');
      isForwardBlocked = (parts[2] === '1');
      autoBrakeEnabled = (parts[3] === '1');
      receivedFirstTelemetry = true;

      updateRadarUI();
      updateOutputs();
    }

    function updateRadarUI() {
      // 1. Distance Readout
      if (!receivedFirstTelemetry) {
        radarDistance.textContent = '--';
        radarDistance.style.color = '#94a3b8';
      } else if (sensorHealthy) {
        if (currentDistanceCm > 0 && currentDistanceCm <= 400) {
          radarDistance.textContent = currentDistanceCm;
          if (currentDistanceCm <= 80) {
            radarDistance.style.color = '#ef4444';
          } else if (currentDistanceCm <= 90) {
            radarDistance.style.color = '#f59e0b';
          } else {
            radarDistance.style.color = '#10b981';
          }
        } else {
          radarDistance.textContent = '400+';
          radarDistance.style.color = '#10b981';
        }
      } else {
        radarDistance.textContent = '--';
        radarDistance.style.color = '#94a3b8';
      }

      // 2. Auto-Brake Toggle Button
      if (autoBrakeEnabled) {
        autoBrakeBtn.textContent = 'AUTO-BRAKE: ON';
        autoBrakeBtn.className = 'btn-toggle radar-toggle active-brake';
      } else {
        autoBrakeBtn.textContent = 'AUTO-BRAKE: OFF';
        autoBrakeBtn.className = 'btn-toggle radar-toggle override-brake';
      }

      // 3. Status Badge & Alert Banner
      if (!ws || ws.readyState !== WebSocket.OPEN) {
        radarBadge.textContent = 'LINK LOST';
        radarBadge.className = 'radar-badge fault';
        radarDot.style.background = '#ef4444';
        radarAlertBanner.textContent = 'WEBSOCKET DISCONNECTED';
        radarAlertBanner.style.display = 'block';
      } else if (!receivedFirstTelemetry) {
        radarBadge.textContent = 'CONNECTING';
        radarBadge.className = 'radar-badge';
        radarDot.style.background = '#94a3b8';
        radarAlertBanner.style.display = 'none';
      } else if (!autoBrakeEnabled) {
        radarBadge.textContent = 'OVERRIDE';
        radarBadge.className = 'radar-badge override';
        radarDot.style.background = '#f59e0b';
        radarAlertBanner.textContent = 'AUTO-BRAKE OVERRIDDEN';
        radarAlertBanner.style.display = 'block';
      } else if (!sensorHealthy) {
        radarBadge.textContent = 'SENSOR FAULT';
        radarBadge.className = 'radar-badge fault';
        radarDot.style.background = '#ef4444';
        radarAlertBanner.textContent = 'SENSOR FAULT // FORWARD BLOCKED';
        radarAlertBanner.style.display = 'block';
      } else if (isForwardBlocked || (sensorHealthy && currentDistanceCm > 0 && currentDistanceCm <= 80)) {
        radarBadge.textContent = 'STOP ZONE';
        radarBadge.className = 'radar-badge stop-zone';
        radarDot.style.background = '#ef4444';
        radarAlertBanner.textContent = 'OBSTACLE DETECTED (<= 80CM) // FORWARD STOPPED';
        radarAlertBanner.style.display = 'block';
      } else if (autoBrakeEnabled && sensorHealthy && currentDistanceCm > 0 && currentDistanceCm < 150) {
        radarBadge.textContent = 'SLOW (40%)';
        radarBadge.className = 'radar-badge caution';
        radarDot.style.background = '#f59e0b';
        radarAlertBanner.textContent = 'COLLISION RANGE (< 150CM) // SPEED REDUCED TO 40%';
        radarAlertBanner.style.display = 'block';
      } else {
        radarBadge.textContent = 'CLEAR';
        radarBadge.className = 'radar-badge clear';
        radarDot.style.background = '#10b981';
        radarAlertBanner.style.display = 'none';
      }

      // 4. Boost safety synchronization: cancel boost if blocked or in collision range (< 150cm)
      if ((isForwardBlocked || (autoBrakeEnabled && sensorHealthy && currentDistanceCm > 0 && currentDistanceCm < 150)) && isBoosting) {
        isBoosting = false;
        if (boostTimer) {
          clearInterval(boostTimer);
          boostTimer = null;
        }
      }
      updateBoostButton();
    }

    // ==========================================
    // GYRO TILT SENSING WITH 3-SECOND ARMING DELAY
    // ==========================================
    async function enableGyro() {
      if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        try {
          const permission = await DeviceOrientationEvent.requestPermission();
          if (permission !== 'granted') {
            alert('Device orientation permission denied.');
            return false;
          }
        } catch (err) {
          console.error(err);
          return false;
        }
      }
      return true;
    }

    function handleOrientation(e) {
      if (e.beta !== null && e.beta !== undefined) {
        rawBeta = e.beta;
        gyroEventCount++;
      }
      if (e.gamma !== null && e.gamma !== undefined) {
        rawGamma = e.gamma;
      }
    }

    function handleMotion(e) {
      if (gyroEventCount > 10) return; // Prefer orientation if actively firing
      const acc = e.accelerationIncludingGravity;
      if (!acc || acc.x === null || acc.y === null || acc.z === null) return;
      gyroEventCount++;
      const x = acc.x;
      const y = acc.y;
      const z = acc.z;
      // Convert accelerometer gravity vector to pitch and roll
      const pitch = Math.atan2(y, Math.sqrt(x * x + z * z)) * (180.0 / Math.PI);
      const roll = Math.atan2(x, Math.sqrt(y * y + z * z)) * (180.0 / Math.PI);
      rawBeta = pitch;
      rawGamma = roll;
    }

    gyroToggleBtn.addEventListener('click', async () => {
      if (!gyroActive && !gyroArming) {
        const ok = await enableGyro();
        if (ok) {
          // Listen to all available orientation and motion streams
          window.addEventListener('deviceorientation', handleOrientation, true);
          window.addEventListener('deviceorientationabsolute', handleOrientation, true);
          window.addEventListener('devicemotion', handleMotion, true);

          gyroArming = true;
          gyroActive = false;
          gyroEventCount = 0;
          gyroToggleBtn.classList.add('active-gyro');
          gyroStatus.style.display = 'inline-flex';
          gyroStatus.className = 'gyro-status-badge arming';

          let countdown = 7;
          gyroToggleBtn.textContent = `HOLD STEADY (${countdown}s)`;
          gyroStatus.textContent = `GET READY: HOLD PHONE STEADY IN DRIVING POSITION (${countdown}s)`;

          gyroArmTimer = setInterval(() => {
            countdown--;
            if (countdown > 0) {
              gyroToggleBtn.textContent = `HOLD STEADY (${countdown}s)`;
              gyroStatus.textContent = `GET READY: HOLD PHONE STEADY IN DRIVING POSITION (${countdown}s)`;
            } else {
              clearInterval(gyroArmTimer);
              gyroArmTimer = null;
              gyroArming = false;
              gyroActive = true;
              zeroBeta = rawBeta;
              zeroGamma = rawGamma;
              gyroToggleBtn.textContent = 'GYRO: ON';
              gyroCalBtn.style.display = 'inline-block';
              gyroStatus.className = 'gyro-status-badge';
              gyroStatus.textContent = 'GYRO ARMED // READY! TILT TO DRIVE';
            }
          }, 1000);
        }
      } else {
        if (gyroArmTimer) {
          clearInterval(gyroArmTimer);
          gyroArmTimer = null;
        }
        window.removeEventListener('deviceorientation', handleOrientation, true);
        window.removeEventListener('deviceorientationabsolute', handleOrientation, true);
        window.removeEventListener('devicemotion', handleMotion, true);
        gyroArming = false;
        gyroActive = false;
        gyroEventCount = 0;
        gyroToggleBtn.textContent = 'GYRO: OFF';
        gyroToggleBtn.classList.remove('active-gyro');
        gyroCalBtn.style.display = 'none';
        gyroStatus.style.display = 'none';
        if (ws && ws.readyState === WebSocket.OPEN) {
          ws.send("GYRO_CAL");
        }
        updateThrottleVisual(0.0);
        updateSteeringVisual(0.0);
        updateOutputs();
      }
    });

    gyroCalBtn.addEventListener('click', () => {
      zeroBeta = rawBeta;
      zeroGamma = rawGamma;
      gyroCalBtn.textContent = 'ZEROED!';
      setTimeout(() => { gyroCalBtn.textContent = 'CALIBRATE'; }, 1000);
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send("GYRO_CAL");
      }
    });

    // ==========================================
    // JOYSTICK TOUCH & MOUSE CONTROLS
    // ==========================================
    const TRACK_V_TRAVEL = 160;
    const TRACK_H_TRAVEL = 170;

    function updateThrottleVisual(normVal) {
      const px = -(normVal * (TRACK_V_TRAVEL / 2));
      throttleKnob.style.transform = `translate(-50%, calc(-50% + ${px}px))`;
    }

    function updateSteeringVisual(normVal) {
      const px = normVal * (TRACK_H_TRAVEL / 2);
      steeringKnob.style.transform = `translate(calc(-50% + ${px}px), -50%)`;
    }

    throttleTrack.addEventListener('touchstart', (e) => {
      e.preventDefault();
      if (throttleTouchId === null) {
        const touch = e.changedTouches[0];
        throttleTouchId = touch.identifier;
        throttleKnob.classList.add('active-knob');
        handleThrottleMove(touch.clientY);
      }
    }, { passive: false });

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
        throttleRaw = 0.0;
        throttleKnob.classList.remove('active-knob');
        updateThrottleVisual(0.0);
        updateOutputs();
      }
      if (id === steeringTouchId) {
        steeringTouchId = null;
        steeringRaw = 0.0;
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

    function handleThrottleMove(clientY) {
      const rect = throttleTrack.getBoundingClientRect();
      const centerY = rect.top + rect.height / 2;
      const deltaY = clientY - centerY;
      let norm = -(deltaY / (TRACK_V_TRAVEL / 2));
      norm = Math.max(-1.0, Math.min(1.0, norm));
      if (Math.abs(norm) < 0.04) norm = 0.0;
      throttleRaw = norm;
      updateThrottleVisual(norm);
      updateOutputs();
    }

    function handleSteeringMove(clientX) {
      const rect = steeringTrack.getBoundingClientRect();
      const centerX = rect.left + rect.width / 2;
      const deltaX = clientX - centerX;
      let norm = deltaX / (TRACK_H_TRAVEL / 2);
      norm = Math.max(-1.0, Math.min(1.0, norm));
      if (Math.abs(norm) < 0.04) norm = 0.0;
      steeringRaw = norm;
      updateSteeringVisual(norm);
      updateOutputs();
    }

    // Mouse Fallback
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

    // ==========================================
    // PWM OUTPUT CALCULATION (CORRECTED SIGNS)
    // ==========================================
    function calculatePwm() {
      // Scale throttle: 1500 - forward offset (1000us full forward, 2000us full reverse)
      const scale = maxThrottlePercent / 100.0;
      const tUs = Math.round(1500 - (throttleRaw * 500 * scale));

      // Scale steering: 1500 - right offset (1000us full right, 2000us full left)
      let sUs = Math.round(1500 - (steeringRaw * 500) - steeringTrimUs);
      sUs = Math.max(1000, Math.min(2000, sUs));

      return { tUs, sUs };
    }

    function updateOutputs() {
      const { tUs, sUs } = calculatePwm();
      let tLabel = "NEUTRAL";
      if (tUs < 1480) {
        tLabel = isForwardBlocked ? "BLOCKED" : "FWD";
      } else if (tUs > 1520) {
        tLabel = "REV";
      }

      let sLabel = "CENTER";
      if (sUs < 1480) {
        sLabel = "RIGHT";
      } else if (sUs > 1520) {
        sLabel = "LEFT";
      }

      throttleReadout.textContent = `${tUs} µs (${tLabel})`;
      steeringReadout.textContent = `${sUs} µs (${sLabel})`;
    }

    // ==========================================
    // WEBSOCKET TELEMETRY LOOP
    // ==========================================
    function initWebSocket() {
      const host = window.location.hostname || "192.168.4.1";
      const wsUrl = `ws://${host}:81/`;
      ws = new WebSocket(wsUrl);

      ws.onopen = () => {
        statusBadge.className = 'badge connected';
        statusText.textContent = 'ONLINE (LOW-LATENCY)';
        updateRadarUI();
        sendPing();
        if (pingIntervalId) clearInterval(pingIntervalId);
        pingIntervalId = setInterval(sendPing, 5000);
      };

      ws.onclose = () => {
        statusBadge.className = 'badge disconnected';
        statusText.textContent = 'DISCONNECTED (FAILSAFE)';
        receivedFirstTelemetry = false;
        if (pingIntervalId) {
          clearInterval(pingIntervalId);
          pingIntervalId = null;
        }
        pingText.textContent = 'PING: --';
        pingBadge.className = 'badge ping';
        updateRadarUI();
        setTimeout(initWebSocket, 1000);
      };

      ws.onmessage = (event) => {
        if (typeof event.data === 'string') {
          const str = event.data.trim();
          if (str.startsWith("U:")) {
            handleUltrasonicTelemetry(str);
          } else if (str.startsWith("PONG")) {
            const now = performance.now();
            let rtt = 0;
            if (str.startsWith("PONG:")) {
              const sentTime = parseFloat(str.substring(5));
              rtt = Math.max(0, Math.round(now - sentTime));
            } else if (lastPingSentTime > 0) {
              rtt = Math.max(0, Math.round(now - lastPingSentTime));
            }
            pingText.textContent = `PING: ${rtt}ms`;
            pingBadge.className = 'badge ping' + (rtt > 150 ? ' bad' : (rtt > 60 ? ' warn' : ''));
          }
        }
      };

      ws.onerror = () => {
        statusBadge.className = 'badge disconnected';
        statusText.textContent = 'ERROR';
        if (pingIntervalId) {
          clearInterval(pingIntervalId);
          pingIntervalId = null;
        }
        pingText.textContent = 'PING: --';
        pingBadge.className = 'badge ping';
        updateRadarUI();
        ws.close();
      };
    }

    // 25Hz transmission loop (every 40ms)
    setInterval(() => {
      if (ws && ws.readyState === WebSocket.OPEN) {
        // If user is actively touching controls, touch has priority over gyro!
        const isUserTouching = (throttleTouchId !== null || steeringTouchId !== null || isMouseDownThrottle || isMouseDownSteering);

        if (gyroActive && !isUserTouching) {
          const tiltPitch = (rawBeta - zeroBeta);
          const tiltRoll = (rawGamma - zeroGamma);
          ws.send(`y:0.0,tx:${tiltRoll.toFixed(1)},ty:${tiltPitch.toFixed(1)}`);

          // Live visual feedback on screen
          if (gyroEventCount === 0) {
            gyroStatus.textContent = 'NO SENSOR: CHECK PERMISSIONS';
            gyroStatus.className = 'gyro-status-badge warn';
          } else {
            const pSign = tiltPitch > 0 ? '+' : '';
            const rSign = tiltRoll > 0 ? '+' : '';
            gyroStatus.textContent = `P:${pSign}${tiltPitch.toFixed(0)}° R:${rSign}${tiltRoll.toFixed(0)}°`;
            gyroStatus.className = 'gyro-status-badge';
          }

          // Move the on-screen knobs visually with the phone tilt
          const normThrottle = Math.max(-1.0, Math.min(1.0, tiltPitch / 25.0));
          const normSteering = Math.max(-1.0, Math.min(1.0, tiltRoll / 25.0));
          updateThrottleVisual(normThrottle);
          updateSteeringVisual(normSteering);

          let gyroForwardScale = maxThrottlePercent / 100.0;
          if (autoBrakeEnabled && sensorHealthy && currentDistanceCm > 0 && currentDistanceCm < 150) {
            gyroForwardScale = 0.40;
          } else if (isBoosting) {
            gyroForwardScale = 1.0;
          }
          const gyroReverseScale = maxThrottlePercent / 100.0;

          let tUs;
          if (normThrottle > 0) {
            if (autoBrakeEnabled && isForwardBlocked) {
              tUs = 1500;
            } else {
              tUs = Math.round(1500 - (normThrottle * 500 * gyroForwardScale));
            }
          } else if (normThrottle < 0) {
            tUs = Math.round(1500 - (normThrottle * 500 * gyroReverseScale));
          } else {
            tUs = 1500;
          }

          const sUs = Math.max(1000, Math.min(2000, Math.round(1500 - (normSteering * 500) - steeringTrimUs)));
          throttleReadout.textContent = `${tUs} µs (GYRO)`;
          steeringReadout.textContent = `${sUs} µs (GYRO)`;
        } else {
          // Manual joystick touch controls (touch has priority over gyro)
          const { tUs, sUs } = calculatePwm();
          ws.send(`T:${tUs},S:${sUs}`);
        }
      }
    }, 40);

    updateOutputs();
    if (document.readyState === 'loading') {
      window.addEventListener('DOMContentLoaded', initWebSocket);
    } else {
      initWebSocket();
    }
  </script>
</body>
</html>
)rawliteral";

// ==========================================
// 4. HELPER & CALCULATION FUNCTIONS
// ==========================================
int calculateGyroPWM(float degTilt) {
  if (std::fabs(degTilt) < GYRO_THRESHOLD_DEG) return PULSE_NEUTRAL_US;
  float clamped = std::fmax(-GYRO_MAX_TILT_DEG, std::fmin(GYRO_MAX_TILT_DEG, degTilt));
  float norm = clamped / GYRO_MAX_TILT_DEG; // -1.0 to +1.0
  int pwm = static_cast<int>(std::round(PULSE_NEUTRAL_US - (norm * 500.0f * GYRO_SCALE)));
  return constrain(pwm, PULSE_MIN_US, PULSE_MAX_US);
}

int calculateGyroHeadingPWM(float degRoll) {
  if (std::fabs(degRoll) < GYRO_THRESHOLD_DEG) return PULSE_NEUTRAL_US;
  float clamped = std::fmax(-GYRO_MAX_TILT_DEG, std::fmin(GYRO_MAX_TILT_DEG, degRoll));
  float norm = clamped / GYRO_MAX_TILT_DEG; // -1.0 to +1.0
  int pwm = static_cast<int>(std::round(PULSE_NEUTRAL_US - (norm * 500.0f * GYRO_SCALE)));
  return constrain(pwm, PULSE_MIN_US, PULSE_MAX_US);
}

int resolveThrottle(int requestedThrottleUs) {
  if (failsafeActive) {
    isBoostingActive = false;
    return PULSE_NEUTRAL_US;
  }

  // REVERSE (requested > 1500us): Always 100% responsive, never blocked by front sensor
  if (requestedThrottleUs > PULSE_NEUTRAL_US) {
    return requestedThrottleUs;
  }

  // FORWARD (requested < 1500us):
  if (requestedThrottleUs < PULSE_NEUTRAL_US) {
    if (autoBrakeEnabled && forwardBlocked) {
      // Complete stop zone (<=80cm) or sensor fault
      isBoostingActive = false;
      return PULSE_NEUTRAL_US;
    }

    // Determine current forward throttle ceiling
    float maxForwardScale = THROTTLE_LIMIT_DEFAULT; // 65% default limit

    if (autoBrakeEnabled && sensorHealthy && currentDistanceCm > 0 && currentDistanceCm < DISTANCE_SLOW_CM) {
      // Collision caution zone (< 150cm): reduce throttle limit to 40%
      maxForwardScale = THROTTLE_LIMIT_SLOW;
      if (isBoostingActive) {
        isBoostingActive = false;
        Serial.println("[BOOST] Boost cancelled: entered <150cm collision caution zone.");
      }
    } else if (isBoostingActive) {
      // Boost mode: temporarily unlocks up to 100%
      maxForwardScale = THROTTLE_LIMIT_BOOST;
    }

    // Clamp forward throttle: min allowed PWM (where 1000us is 100% forward)
    int minAllowedUs = PULSE_NEUTRAL_US - static_cast<int>(500.0f * maxForwardScale);
    return std::max(minAllowedUs, requestedThrottleUs);
  }

  return PULSE_NEUTRAL_US;
}

void broadcastSensorStatus() {
  char buf[32];
  snprintf(buf, sizeof(buf), "U:%d,%d,%d,%d",
           currentDistanceCm,
           sensorHealthy ? 1 : 0,
           forwardBlocked ? 1 : 0,
           autoBrakeEnabled ? 1 : 0);
  webSocket.broadcastTXT(buf);

  static unsigned long lastSerialLog = 0;
  static int lastLoggedDist = -1;
  static bool lastLoggedHealthy = false;
  static bool lastLoggedBlocked = false;
  unsigned long now = millis();
  if (now - lastSerialLog >= 2000 || currentDistanceCm != lastLoggedDist || sensorHealthy != lastLoggedHealthy || forwardBlocked != lastLoggedBlocked) {
    lastSerialLog = now;
    lastLoggedDist = currentDistanceCm;
    lastLoggedHealthy = sensorHealthy;
    lastLoggedBlocked = forwardBlocked;
    Serial.printf("[SENSOR] Telemetry: dist=%d cm, healthy=%d, forwardBlocked=%d, autoBrake=%d\n",
                  currentDistanceCm, sensorHealthy ? 1 : 0, forwardBlocked ? 1 : 0, autoBrakeEnabled ? 1 : 0);
  }
}

void sendSensorStatus(uint8_t clientNum) {
  char buf[32];
  snprintf(buf, sizeof(buf), "U:%d,%d,%d,%d",
           currentDistanceCm,
           sensorHealthy ? 1 : 0,
           forwardBlocked ? 1 : 0,
           autoBrakeEnabled ? 1 : 0);
  webSocket.sendTXT(clientNum, buf);
}

void updateForwardBlocked() {
  bool prevBlocked = forwardBlocked;
  if (autoBrakeEnabled) {
    if (!sensorHealthy) {
      forwardBlocked = true;
    } else {
      forwardBlocked = obstacleLatch;
    }
  } else {
    forwardBlocked = false;
  }

  // Enforce throttle safety immediately if forward became blocked or caution zone entered
  if (forwardBlocked || (autoBrakeEnabled && sensorHealthy && currentDistanceCm > 0 && currentDistanceCm < DISTANCE_SLOW_CM)) {
    if (isBoostingActive) {
      isBoostingActive = false;
      Serial.println("[BOOST] Boost cancelled: obstacle in collision range.");
    }
  }

  if (!failsafeActive) {
    escThrottle.writeMicroseconds(resolveThrottle(currentThrottleUs));
  }

  if (forwardBlocked != prevBlocked) {
    broadcastSensorStatus();
  }
}

void handleSensorSuccess(int rawDistCm) {
  sensorHealthy = true;
  consecutiveInvalidCount = 0;

  // Apply 5-sample (300ms) rolling window tracking filter
  currentDistanceCm = applyDistanceFilter(rawDistCm);

  if (currentDistanceCm > 0 && currentDistanceCm <= DISTANCE_MAX_VALID_CM) {
    if (currentDistanceCm <= DISTANCE_STOP_CM) {
      obstacleLatch = true;
    } else if (currentDistanceCm >= DISTANCE_CLEAR_CM) {
      obstacleLatch = false;
    }
  } else {
    // 0 or > 400 cm means no obstacle in range (path is clear)
    obstacleLatch = false;
  }

  updateForwardBlocked();
}

void handleSensorFault() {
  if (consecutiveInvalidCount < 255) {
    consecutiveInvalidCount++;
  }
  if (consecutiveInvalidCount >= SENSOR_FAULT_THRESHOLD) {
    sensorHealthy = false;
    currentDistanceCm = 0;
    distWindowFilled = false; // Reset rolling window on sensor fault
  }
  updateForwardBlocked();
}

void updateUltrasonic(unsigned long nowMs) {
  unsigned long nowUs = micros();

  switch (usState) {
    case US_IDLE: {
      if (nowMs - usLastMeasureTime < US_MEASURE_CADENCE_MS) {
        return;
      }

      // Check if echo line is still HIGH from a previous long timeout
      if (digitalRead(PIN_ULTRASONIC_ECHO) == HIGH) {
        usState = US_WAIT_LINE_CLEAR;
        usEchoStartTime = nowUs;
        return;
      }

      // Send 10us trigger pulse
      digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

      usTriggerTime = nowUs;
      usLastMeasureTime = nowMs;
      usState = US_WAIT_RISE;
      break;
    }

    case US_WAIT_RISE: {
      if (digitalRead(PIN_ULTRASONIC_ECHO) == HIGH) {
        usEchoStartTime = nowUs;
        usState = US_WAIT_FALL;
      } else if (nowUs - usTriggerTime > US_RISE_TIMEOUT_US) {
        // Echo never rose within 8ms -> sensor unplugged, no power, or dead
        handleSensorFault();
        usState = US_IDLE;
      }
      break;
    }

    case US_WAIT_FALL: {
      if (digitalRead(PIN_ULTRASONIC_ECHO) == LOW) {
        // 4us digital noise filter to reject 50Hz servo PWM crosstalk
        delayMicroseconds(4);
        if (digitalRead(PIN_ULTRASONIC_ECHO) == LOW) {
          unsigned long pulseDuration = nowUs - usEchoStartTime;
          if (pulseDuration >= US_PULSE_MIN_VALID_US && pulseDuration <= US_PULSE_MAX_VALID_US) {
            int distCm = (int)round((float)pulseDuration / 58.2f);
            handleSensorSuccess(distCm);
          } else {
            handleSensorSuccess(0);
          }
          usState = US_IDLE;
        }
      } else if (nowUs - usEchoStartTime > US_PULSE_MAX_VALID_US) {
        // Echo stayed HIGH past 400cm (23.28ms) -> path is CLEAR (>400cm)
        handleSensorSuccess(0);
        usState = US_WAIT_LINE_CLEAR;
      }
      break;
    }

    case US_WAIT_LINE_CLEAR: {
      if (digitalRead(PIN_ULTRASONIC_ECHO) == LOW) {
        // Hardware timeout finished and line returned to LOW
        usState = US_IDLE;
      } else if (nowUs - usEchoStartTime > 250000) {
        // Line stuck HIGH for > 250ms -> hardware fault or short circuit
        handleSensorFault();
        usState = US_IDLE;
      }
      break;
    }
  }
}

// ==========================================
// 5. OTA (OVER-THE-AIR) UPDATE CALLBACKS
// ==========================================
void onOTAStart() {
  failsafeActive = true;
  isBoostingActive = false;
  currentThrottleUs = PULSE_NEUTRAL_US;
  currentSteeringUs = PULSE_NEUTRAL_US;
  escThrottle.writeMicroseconds(PULSE_NEUTRAL_US);
  steeringServo.writeMicroseconds(PULSE_NEUTRAL_US);
  Serial.println("[OTA] Firmware update initiated. Motors and steering locked neutral.");
}

void onOTAEnd(bool success) {
  if (success) {
    Serial.println("[OTA] Firmware update succeeded! Device auto-rebooting...");
  } else {
    Serial.println("[OTA] Firmware update failed!");
  }
}

void onOTAProgress(size_t current, size_t final) {
  static unsigned long lastProgressLog = 0;
  if (millis() - lastProgressLog > 1000) {
    lastProgressLog = millis();
    float pct = (final > 0) ? ((float)current / (float)final * 100.0f) : 0.0f;
    Serial.printf("[OTA] Progress: %u / %u bytes (%.1f%%)\n", current, final, pct);
  }
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.send_P(200, "text/html", INDEX_HTML);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u disconnected -> Failsafe engaged\n", num);
      failsafeActive = true;
      isBoostingActive = false; // Immediately cancel active boost
      currentThrottleUs = PULSE_NEUTRAL_US;
      currentSteeringUs = PULSE_NEUTRAL_US;
      escThrottle.writeMicroseconds(PULSE_NEUTRAL_US);
      steeringServo.writeMicroseconds(PULSE_NEUTRAL_US);
      break;

    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[WS] Client #%u connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      lastPacketTime = millis();
      failsafeActive = false;
      isBoostingActive = false;
      sendSensorStatus(num);
      break;
    }

    case WStype_TEXT: {
      String payloadStr = String((char*)payload);
      unsigned long now = millis();

      static unsigned long lastPktLog = 0;
      if (now - lastPktLog >= 1000) {
        lastPktLog = now;
        Serial.printf("[WS IN] %s\n", (char*)payload);
      }
      float yawDeg = 0.0f, tiltX = 0.0f, tiltY = 0.0f;
      int parsedThrottle = PULSE_NEUTRAL_US;
      int parsedSteering = PULSE_NEUTRAL_US;

      // 1. Gyro packet telemetry: "y:%f,tx:%f,ty:%f"
      if (sscanf((char*)payload, "y:%f,tx:%f,ty:%f", &yawDeg, &tiltX, &tiltY) == 3) {
        currentThrottleUs = calculateGyroPWM(tiltY);
        currentSteeringUs = calculateGyroHeadingPWM(yawDeg + tiltX);

        if (!failsafeActive) {
          steeringServo.writeMicroseconds(currentSteeringUs);
          escThrottle.writeMicroseconds(resolveThrottle(currentThrottleUs));
        }
        lastPacketTime = now;
        failsafeActive = false;
      }
      // 2. Boost commands
      else if (payloadStr == "BOOST_ON") {
        if (forwardBlocked || (autoBrakeEnabled && sensorHealthy && currentDistanceCm > 0 && currentDistanceCm < DISTANCE_SLOW_CM)) {
          Serial.println("[BOOST] Boost request rejected: obstacle in collision range (< 150cm).");
        } else {
          long timeSinceLastBoost = now - lastBoostTime;
          if (!isBoostingActive && (lastBoostTime == 0 || timeSinceLastBoost >= (long)COOLDOWN_DURATION_MS)) {
            isBoostingActive = true;
            boostStartTime = now;
            lastBoostTime = now;
            Serial.println("[BOOST] Boost headroom unlocked to 100% temporarily (5s).");
          } else {
            Serial.println("[BOOST] Boost request ignored (active or cooling down).");
          }
        }
        lastPacketTime = now;
        failsafeActive = false;
      }
      else if (payloadStr == "AUTOBRAKE_ON") {
        autoBrakeEnabled = true;
        updateForwardBlocked();
        broadcastSensorStatus();
        Serial.println("[AUTOBRAKE] Enabled by client");
        lastPacketTime = now;
        failsafeActive = false;
      }
      else if (payloadStr == "AUTOBRAKE_OFF") {
        autoBrakeEnabled = false;
        updateForwardBlocked();
        broadcastSensorStatus();
        Serial.println("[AUTOBRAKE] Disabled (Override) by client");
        lastPacketTime = now;
        failsafeActive = false;
      }
      else if (payloadStr == "BOOST_OFF") {
        isBoostingActive = false;
        Serial.println("[BOOST] Boost deactivated.");
        lastPacketTime = now;
      }
      // 3. Ping command: echoes back "PONG:<ts>" for live latency measurement
      else if (strncmp((char*)payload, "PING", 4) == 0) {
        char pongBuf[32];
        if (strncmp((char*)payload, "PING:", 5) == 0) {
          snprintf(pongBuf, sizeof(pongBuf), "PONG:%s", (char*)payload + 5);
        } else {
          snprintf(pongBuf, sizeof(pongBuf), "PONG");
        }
        webSocket.sendTXT(num, pongBuf);
      }
      // 4. Joystick packet telemetry: "T:%d,S:%d"
      else if (sscanf((char*)payload, "T:%d,S:%d", &parsedThrottle, &parsedSteering) == 2) {
        currentThrottleUs = constrain(parsedThrottle, PULSE_MIN_US, PULSE_MAX_US);
        currentSteeringUs = constrain(parsedSteering, PULSE_MIN_US, PULSE_MAX_US);

        if (!failsafeActive) {
          steeringServo.writeMicroseconds(currentSteeringUs);
          escThrottle.writeMicroseconds(resolveThrottle(currentThrottleUs));
        }
        lastPacketTime = now;
        failsafeActive = false;
      }
      break;
    }

    default:
      break;
  }
}

// ==========================================
// 6. SETUP & MAIN LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n\n========================================");
  Serial.println("  NodeMCU ESP8266 RC CAR CONTROLLER     ");
  Serial.println("========================================");

  // Arming safeguard: pre-set 1500us neutral pulse BEFORE attach
  escThrottle.writeMicroseconds(PULSE_NEUTRAL_US);
  steeringServo.writeMicroseconds(PULSE_NEUTRAL_US);

  escThrottle.attach(PIN_THROTTLE, PULSE_MIN_US, PULSE_MAX_US);
  steeringServo.attach(PIN_STEERING, PULSE_MIN_US, PULSE_MAX_US);
  Serial.printf("[PULSE] Attached ESC to D2 (GPIO %d), Steering to D1 (GPIO %d) @ 1500us\n", PIN_THROTTLE, PIN_STEERING);

  // Initialize Ultrasonic Pins
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  Serial.printf("[ULTRASONIC] HC-SR04 configured: Trig D5 (GPIO %d), Echo D6 (GPIO %d)\n", PIN_ULTRASONIC_TRIG, PIN_ULTRASONIC_ECHO);

  // Initialize Onboard Heartbeat LEDs (active LOW on NodeMCU)
  pinMode(PIN_LED_HEARTBEAT, OUTPUT);
  digitalWrite(PIN_LED_HEARTBEAT, HIGH);
  pinMode(PIN_LED_BOARD, OUTPUT);
  digitalWrite(PIN_LED_BOARD, HIGH);
  Serial.printf("[LED] Heartbeat LEDs initialized on GPIO %d and GPIO %d\n", PIN_LED_HEARTBEAT, PIN_LED_BOARD);

  // Ensure clean, high-power SoftAP configuration with no modem sleep
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setOutputPower(20.5f);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  bool apSuccess = WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4, 100);

  // Register station connect/disconnect callbacks
  stationConnectedHandler = WiFi.onSoftAPModeStationConnected([](const WiFiEventSoftAPModeStationConnected& evt) {
    Serial.printf("[WIFI] Station connected: MAC %02X:%02X:%02X:%02X:%02X:%02X (Total stations: %d)\n",
                  evt.mac[0], evt.mac[1], evt.mac[2], evt.mac[3], evt.mac[4], evt.mac[5],
                  WiFi.softAPgetStationNum());
  });
  stationDisconnectedHandler = WiFi.onSoftAPModeStationDisconnected([](const WiFiEventSoftAPModeStationDisconnected& evt) {
    Serial.printf("[WIFI] Station disconnected: MAC %02X:%02X:%02X:%02X:%02X:%02X (Total stations: %d)\n",
                  evt.mac[0], evt.mac[1], evt.mac[2], evt.mac[3], evt.mac[4], evt.mac[5],
                  WiFi.softAPgetStationNum());
  });

  if (apSuccess) {
    Serial.printf("[WIFI] Access Point '%s' broadcast active on Channel %d (unhidden).\n", AP_SSID, WiFi.channel());
    Serial.printf("[WIFI] SoftAP MAC Address: %s\n", WiFi.softAPmacAddress().c_str());
    Serial.print("[WIFI] Static IP Address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("[WIFI] SoftAP initialization FAILED!");
  }

  // Setup ElegantOTA
  ElegantOTA.begin(&server);
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onEnd(onOTAEnd);
  ElegantOTA.onProgress(onOTAProgress);

  // Setup Web Server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/update/", HTTP_GET, []() {
    server.sendHeader("Location", "/update");
    server.send(302, "text/plain", "");
  });
  server.onNotFound([]() {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
  });
  server.begin();
  Serial.println("[HTTP] Web Server started on port 80");
  Serial.println("[OTA] ElegantOTA initialized at /update");

  // Setup WebSocket Server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("[WS] WebSocket Server started on port 81");

  lastPacketTime = millis();
  failsafeActive = false;
  isBoostingActive = false;
  Serial.println("[SYSTEM] Armed and ready for connection.");
}

void loop() {
  webSocket.loop();
  server.handleClient();
  ElegantOTA.loop();

  unsigned long now = millis();

  // Periodic Onboard Heartbeat LED: pulse once every 15 seconds (200ms flash)
  const unsigned long HEARTBEAT_PERIOD_MS = 15000;
  const unsigned long HEARTBEAT_PULSE_MS  = 200;
  static unsigned long lastHeartbeatCycle = 0;
  if (now - lastHeartbeatCycle >= HEARTBEAT_PERIOD_MS) {
    lastHeartbeatCycle = now;
  }
  bool isLedOn = (now - lastHeartbeatCycle < HEARTBEAT_PULSE_MS);
  digitalWrite(PIN_LED_HEARTBEAT, isLedOn ? LOW : HIGH); // Active LOW on NodeMCU
  digitalWrite(PIN_LED_BOARD, isLedOn ? LOW : HIGH);     // Active LOW on NodeMCU

  // Run non-blocking ultrasonic state machine
  updateUltrasonic(now);

  // Periodic sensor telemetry broadcast
  if (now - lastSensorBroadcastTime >= SENSOR_BROADCAST_INTERVAL_MS) {
    lastSensorBroadcastTime = now;
    broadcastSensorStatus();
  }

  // 1. Watchdog Failsafe Check (HIGHEST PRIORITY)
  if (!failsafeActive && (now - lastPacketTime > FAILSAFE_TIMEOUT_MS)) {
    failsafeActive = true;
    isBoostingActive = false; // Always cancel boost immediately!
    currentThrottleUs = PULSE_NEUTRAL_US;
    currentSteeringUs = PULSE_NEUTRAL_US;
    escThrottle.writeMicroseconds(PULSE_NEUTRAL_US);
    steeringServo.writeMicroseconds(PULSE_NEUTRAL_US);
    Serial.println("[FAILSAFE] WATCHDOG TRIGGERED! Throttle snapped to 1500us neutral.");
  }

  // 2. Boost Timeout Check
  if (isBoostingActive && (now - boostStartTime >= BOOST_DURATION_MS)) {
    isBoostingActive = false;
    Serial.println("[BOOST] Boost duration elapsed. Throttle headroom reverting to 65%.");
  }

  // 3. Throttle Resolution (enforced continuously)
  int targetThrottleUs = resolveThrottle(currentThrottleUs);
  static int lastWrittenThrottle = -1;
  if (targetThrottleUs != lastWrittenThrottle) {
    lastWrittenThrottle = targetThrottleUs;
    escThrottle.writeMicroseconds(targetThrottleUs);
  }
}
