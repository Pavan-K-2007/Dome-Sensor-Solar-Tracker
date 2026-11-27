/*
  =============================================================================
  DOME SENSOR DUAL-AXIS STACKED SERVO TRACKER — ARDUINO UNO R4 WIFI
  =============================================================================
  Board:   Arduino UNO R4 WiFi
  Driver:  PCA9685 16-Channel PWM Servo Driver (I2C address 0x40)
  Servos:  Channel 0 = Bottom Base Servo (Azimuth / Left-Right, 0..180 deg)
           Channel 1 = Upper Stacked Servo (Elevation / Tilt,   0..180 deg)
  Input:   5-sensor (or 9-sensor) CSV stream from ESP32:  S1,S2,S3,S4,S5\n
           - Wired UART:  ESP32 TX2 (GPIO 17) -> UNO R4 Pin 0 (RX) + Common GND
                          (Note: UNO R4 RX is 5V CMOS; use a 3.3V->5V level
                           shifter if direct 3.3V UART drops packets, OR set
                           USE_WIFI_UDP 1 below for a wireless link!)
           - USB Serial:  Also accepts S1..S5 lines from PC for bench testing
           - Manual Jog:  Pins 2, 3, 4, 5 to GND override auto-tracking anytime
  =============================================================================
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Set to 1 to receive ESP32 sensor packets wirelessly over UNO R4 WiFi SoftAP
// (0 = Wired UART on Pin 0 RX / USB Serial; 1 = Wi-Fi Access Point + UDP 4210)
#define USE_WIFI_UDP 0

// 0 = Stationary Central Dome (3D Least-Squares -> maps Az/El to Base & Tilt)
// 1 = Dome Mounted ON the Moving Upper Servo (Closed-loop nulling toward S1 Top)
#define TRACKING_MODE 0

#if USE_WIFI_UDP
#include <WiFiS3.h>
#include <WiFiUdp.h>
const char* AP_SSID = "DomeTracker_R4";
const char* AP_PASS = "solar1234";
const unsigned int UDP_PORT = 4210;
WiFiUDP udp;
#endif

Adafruit_PWMServoDriver pwm(0x40);

// -------------------- Manual jumper-wire inputs --------------------
const int BASE_LEFT  = 2;
const int BASE_RIGHT = 3;
const int TILT_UP    = 4;
const int TILT_DOWN  = 5;

// -------------------- PCA9685 servo channels -----------------------
const int BASE_SERVO = 0;  // Bottom servo: left/right (Azimuth)
const int TILT_SERVO = 1;  // Upper servo:  up/down    (Elevation, 90 = flat up)

// Servo pulse range in microseconds (calibrate for your servos)
const int SERVO_MIN_US = 600;
const int SERVO_MAX_US = 2400;

// Safe movement range (tighten TILT to e.g. 20..160 if bracket hits base horn)
const int BASE_MIN_ANGLE = 0;
const int BASE_MAX_ANGLE = 180;
const int TILT_MIN_ANGLE = 0;
const int TILT_MAX_ANGLE = 180;

// Flip to true if either servo rotates opposite to your physical mounting
const bool INVERT_BASE = false;
const bool INVERT_TILT = false;

// Current & target servo angles (90/90 = centered & flat facing Zenith)
int baseAngle = 90;
int tiltAngle = 90;
int targetBaseAngle = 90;
int targetTiltAngle = 90;

// Movement smoothing & anti-hunting settings
const int STEP_SIZE = 2;                        // Degrees per step
const int ANGLE_DEADBAND = 3;                   // Hold servo if error < 3 deg
const unsigned long MOVE_INTERVAL_MS = 40;      // 40 ms = smooth 50 deg/s slew
const unsigned long MANUAL_HOLD_MS   = 2500;    // Resume auto 2.5s after manual jog
const unsigned long SENSOR_TIMEOUT_MS = 3000;   // Hold if ESP32 stream stops

unsigned long lastMoveTime = 0;
unsigned long lastManualTime = 0;
unsigned long lastSensorTime = 0;

// -------------------- Dome Sensor Geometry & Calibration -----------
const int MAX_SENSORS = 9;
const float ADC_FULL_SCALE = 4095.0f;           // ESP32 12-bit ADC counts
const float MIN_LIGHT_THR  = 0.03f;             // Below 3% = Dark (hold position)
const float CLOUD_SPREAD_THR = 0.20f;           // Ring spread < 0.20 = Cloudy (stow 90)
const float EMA_ALPHA      = 0.25f;             // Low-pass smoothing factor

// [Azimuth deg, Elevation deg] — first 5 match S1..S5 on your breadboard;
// S6..S9 support the 9-sensor 3D-printed dome automatically if 9 values arrive.
const float SENSOR_AZ_EL[MAX_SENSORS][2] = {
  {  0.0f, 90.0f}, // S1 Top (Zenith)
  {  0.0f, 55.0f}, // S2 North
  { 90.0f, 55.0f}, // S3 East
  {180.0f, 55.0f}, // S4 South
  {270.0f, 55.0f}, // S5 West
  { 45.0f, 30.0f}, // S6 NE (optional 9-sensor dome)
  {135.0f, 30.0f}, // S7 SE
  {225.0f, 30.0f}, // S8 SW
  {315.0f, 30.0f}  // S9 NW
};

float sensorNormals[MAX_SENSORS][3];
float sensorGain[MAX_SENSORS]   = {1, 1, 1, 1, 1, 1, 1, 1, 1};
float sensorOffset[MAX_SENSORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float smVal[MAX_SENSORS]        = {0};
int activeSensorCount = 5;
bool hasSensorLock = false;

String usbBuf = "";
String uartBuf = "";

// -------------------- PCA9685 Helpers ------------------------------
int pulseToTicks(int pulseUs) {
  return (int)((pulseUs * 4096.0f) / 20000.0f);
}

void moveServo(int channel, int angle) {
  angle = constrain(angle, 0, 180);
  int hwAngle = angle;
  if (channel == BASE_SERVO && INVERT_BASE) hwAngle = 180 - angle;
  if (channel == TILT_SERVO && INVERT_TILT) hwAngle = 180 - angle;

  int pulseUs = map(hwAngle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  pwm.setPWM(channel, 0, pulseToTicks(pulseUs));
}

// -------------------- 3x3 Least-Squares Sun Vector Solver ----------
float det3(const float m[3][3]) {
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
       - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
       + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

bool solveSunVector(int count, float outVec[3]) {
  float mx = 0.0f;
  for (int i = 0; i < count; i++) {
    if (smVal[i] > mx) mx = smVal[i];
  }
  if (mx < MIN_LIGHT_THR) return false;

  float thr = max(MIN_LIGHT_THR, 0.08f * mx);
  int actIdx[MAX_SENSORS];
  int actN = 0;
  for (int i = 0; i < count; i++) {
    if (smVal[i] > thr) actIdx[actN++] = i;
  }

  bool solved = false;
  float v[3] = {0.0f, 0.0f, 0.0f};

  if (actN >= 3) {
    float A[3][3] = {{0}};
    float b[3]    = {0};
    for (int k = 0; k < actN; k++) {
      int i = actIdx[k];
      for (int r = 0; r < 3; r++) {
        b[r] += sensorNormals[i][r] * smVal[i];
        for (int c = 0; c < 3; c++) {
          A[r][c] += sensorNormals[i][r] * sensorNormals[i][c];
        }
      }
    }
    float d = det3(A);
    if (fabs(d) > 1e-3f) {
      for (int col = 0; col < 3; col++) {
        float M[3][3];
        for (int r = 0; r < 3; r++) {
          for (int c = 0; c < 3; c++) {
            M[r][c] = (c == col) ? b[r] : A[r][c];
          }
        }
        v[col] = det3(M) / d;
      }
      solved = true;
    }
  }

  if (!solved) {
    // Fallback weighted vector sum when fewer than 3 sensors are lit
    for (int i = 0; i < count; i++) {
      v[0] += smVal[i] * sensorNormals[i][0];
      v[1] += smVal[i] * sensorNormals[i][1];
      v[2] += smVal[i] * sensorNormals[i][2];
    }
  }

  float mag = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (mag < 1e-4f) return false;

  outVec[0] = v[0] / mag; // +X = East
  outVec[1] = v[1] / mag; // +Y = Up (Zenith)
  outVec[2] = v[2] / mag; // +Z = North
  return true;
}

// -------------------- Map Sensor Readings -> Stacked Servo Targets --
void updateTargetsFromSensors() {
  float peak = 0.0f;
  for (int i = 0; i < activeSensorCount; i++) {
    if (smVal[i] > peak) peak = smVal[i];
  }

  // 1. Darkness check: hold current angles to save power
  if (peak < MIN_LIGHT_THR) {
    targetBaseAngle = baseAngle;
    targetTiltAngle = tiltAngle;
    return;
  }

  // 2. Diffuse / Cloudy sky check on perimeter ring S2..S5 (indices 1..4)
  float rMax = 0.0f, rMin = 1.0f;
  for (int i = 1; i <= 4 && i < activeSensorCount; i++) {
    if (smVal[i] > rMax) rMax = smVal[i];
    if (smVal[i] < rMin) rMin = smVal[i];
  }
  if (rMax > MIN_LIGHT_THR && ((rMax - rMin) / rMax) < CLOUD_SPREAD_THR) {
    // Cloudy sky -> stow panel flat horizontal (90 deg) and hold base
    targetTiltAngle = constrain(90, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
    targetBaseAngle = baseAngle;
    return;
  }

#if TRACKING_MODE == 0
  // ---- MODE 0: Stationary Central Dome -> Absolute (Base, Tilt) Angles ----
  // Physical alignment:
  //   Base = 0 deg   -> East  (Az 90)
  //   Base = 90 deg  -> South (Az 180)
  //   Base = 180 deg -> West  (Az 270)
  //   Tilt = 90 deg  -> Flat horizontal (Zenith)
  //   Tilt < 90 deg  -> Tilts toward South/East/West front hemisphere
  //   Tilt > 90 deg  -> Tilts over-the-top toward North rear hemisphere
  float sunVec[3];
  if (!solveSunVector(activeSensorCount, sunVec)) return;

  float azDeg = fmodf(atan2f(sunVec[0], sunVec[2]) * (180.0f / PI) + 360.0f, 360.0f);
  float elDeg = asinf(constrain(sunVec[1], -1.0f, 1.0f)) * (180.0f / PI);
  elDeg = constrain(elDeg, 0.0f, 90.0f);
  float zenithDist = 90.0f - elDeg; // 0 deg when overhead, 90 deg at horizon

  int newBase = baseAngle;
  int newTilt = 90;

  if (elDeg > 83.0f) {
    // Sun nearly straight overhead -> level tilt to 90, prevent base gimbal spin
    newTilt = 90;
    newBase = baseAngle;
  } else if (azDeg >= 90.0f && azDeg <= 270.0f) {
    // Daytime sun arc (East 90 -> South 180 -> West 270): smooth 0..180 sweep
    newBase = (int)roundf(azDeg - 90.0f);
    newTilt = (int)roundf(90.0f - zenithDist);
  } else {
    // Northern half of sky (270..360 or 0..90): wrap base +180 and tilt over-the-top
    float wrappedAz = fmodf(azDeg + 180.0f, 360.0f); // Maps into 90..270
    newBase = (int)roundf(wrappedAz - 90.0f);
    newTilt = (int)roundf(90.0f + zenithDist);
  }

  targetBaseAngle = constrain(newBase, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
  targetTiltAngle = constrain(newTilt, TILT_MIN_ANGLE, TILT_MAX_ANGLE);

#else
  // ---- MODE 1: Dome Mounted ON Upper Servo -> Differential Visual Nulling --
  float n = smVal[1], e = smVal[2], s = smVal[3], w = smVal[4];
  float ewErr = (e + w > MIN_LIGHT_THR) ? (e - w) / (e + w) : 0.0f;
  float nsErr = (n + s > MIN_LIGHT_THR) ? (n - s) / (n + s) : 0.0f;

  if (fabs(ewErr) > 0.06f) {
    targetBaseAngle = constrain(baseAngle + (ewErr > 0 ? -STEP_SIZE : STEP_SIZE),
                                BASE_MIN_ANGLE, BASE_MAX_ANGLE);
  }
  if (fabs(nsErr) > 0.06f) {
    targetTiltAngle = constrain(tiltAngle + (nsErr > 0 ? STEP_SIZE : -STEP_SIZE),
                                TILT_MIN_ANGLE, TILT_MAX_ANGLE);
  }
#endif
}

// -------------------- Parse Incoming ESP32 CSV Line ----------------
void parseSensorLine(String line, const char* srcLabel) {
  line.trim();
  if (line.length() == 0 || line[0] == '#') return;

  float vals[MAX_SENSORS];
  int count = 0;
  int idx = 0;
  int len = line.length();

  while (idx < len && count < MAX_SENSORS) {
    while (idx < len && (line[idx] == ',' || line[idx] == ' ' || line[idx] == '\t' || line[idx] == '[' || line[idx] == ']')) {
      idx++;
    }
    if (idx >= len) break;
    int start = idx;
    while (idx < len && line[idx] != ',' && line[idx] != ' ' && line[idx] != '\t' && line[idx] != ']') {
      idx++;
    }
    String token = line.substring(start, idx);
    char* endPtr = nullptr;
    float v = strtof(token.c_str(), &endPtr);
    if (endPtr != token.c_str()) {
      vals[count++] = v;
    } else {
      return; // Ignore non-numeric status messages
    }
  }

  if (count < 5) return;
  activeSensorCount = (count >= 9) ? 9 : 5;

  for (int i = 0; i < activeSensorCount; i++) {
    float span = max(1.0f, ADC_FULL_SCALE - sensorOffset[i]);
    float norm = constrain(sensorGain[i] * (vals[i] - sensorOffset[i]) / span, 0.0f, 1.0f);
    if (!hasSensorLock) {
      smVal[i] = norm;
    } else {
      smVal[i] += EMA_ALPHA * (norm - smVal[i]);
    }
  }

  hasSensorLock = true;
  lastSensorTime = millis();
  updateTargetsFromSensors();

  // Echo telemetry to USB Serial so PC Serial Monitor / Dashboard can observe
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 250) {
    lastPrint = millis();
    Serial.print("[");
    Serial.print(srcLabel);
    Serial.print("] Base: ");
    Serial.print(baseAngle);
    Serial.print(" -> ");
    Serial.print(targetBaseAngle);
    Serial.print(" deg | Tilt: ");
    Serial.print(tiltAngle);
    Serial.print(" -> ");
    Serial.print(targetTiltAngle);
    Serial.println(" deg");
  }
}

void pollStream(Stream& port, String& buffer, const char* label) {
  while (port.available() > 0) {
    char c = (char)port.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        parseSensorLine(buffer, label);
        buffer = "";
      }
    } else if (buffer.length() < 160) {
      buffer += c;
    } else {
      buffer = "";
    }
  }
}

// -------------------- Arduino Setup & Main Loop --------------------
void setup() {
  Serial.begin(115200);   // USB-C to PC
  Serial1.begin(115200);  // UNO R4 Hardware UART: Pin 0 (RX) <- ESP32 TX

  // Precompute 3D unit normals [x=East, y=Up, z=North] for all 9 sensor slots
  for (int i = 0; i < MAX_SENSORS; i++) {
    float az = SENSOR_AZ_EL[i][0] * (PI / 180.0f);
    float el = SENSOR_AZ_EL[i][1] * (PI / 180.0f);
    sensorNormals[i][0] = cosf(el) * sinf(az);
    sensorNormals[i][1] = sinf(el);
    sensorNormals[i][2] = cosf(el) * cosf(az);
  }

  // Manual jumper-wire override inputs (idle HIGH; active when tied to GND)
  pinMode(BASE_LEFT, INPUT_PULLUP);
  pinMode(BASE_RIGHT, INPUT_PULLUP);
  pinMode(TILT_UP, INPUT_PULLUP);
  pinMode(TILT_DOWN, INPUT_PULLUP);

  Wire.begin();

  if (!pwm.begin()) {
    Serial.println("PCA9685 not detected at 0x40. Check SDA, SCL and V+ power.");
    while (true) {
      delay(100);
    }
  }

  pwm.setPWMFreq(50);
  delay(500);

  // Start both stacked servos at center (90 deg pan, 90 deg flat zenith)
  moveServo(BASE_SERVO, baseAngle);
  moveServo(TILT_SERVO, tiltAngle);

#if USE_WIFI_UDP
  Serial.print("Starting UNO R4 WiFi Access Point: ");
  Serial.println(AP_SSID);
  if (WiFi.beginAP(AP_SSID, AP_PASS) == WL_AP_LISTENING) {
    udp.begin(UDP_PORT);
    Serial.print("UDP listening on ");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(UDP_PORT);
  } else {
    Serial.println("WiFi AP failed to start; continuing on Serial1 UART.");
  }
#endif

  Serial.println("UNO R4 WiFi Dual-Servo Dome Tracker ready (Auto + Manual Override).");
}

void loop() {
  // 1. Read incoming sensor packets from ESP32 (Pin 0 RX), USB Serial, or Wi-Fi UDP
  pollStream(Serial1, uartBuf, "ESP32-UART");
  pollStream(Serial,  usbBuf,  "USB-TEST");

#if USE_WIFI_UDP
  int pktSize = udp.parsePacket();
  if (pktSize > 0) {
    char pkt[160];
    int n = udp.read(pkt, sizeof(pkt) - 1);
    if (n > 0) {
      pkt[n] = '\0';
      parseSensorLine(String(pkt), "ESP32-UDP");
    }
  }
#endif

  // 2. Check manual jumper-wire inputs (active LOW)
  bool leftPressed  = (digitalRead(BASE_LEFT)  == LOW);
  bool rightPressed = (digitalRead(BASE_RIGHT) == LOW);
  bool upPressed    = (digitalRead(TILT_UP)    == LOW);
  bool downPressed  = (digitalRead(TILT_DOWN)  == LOW);
  bool manualActive = leftPressed || rightPressed || upPressed || downPressed;

  if (manualActive) {
    lastManualTime = millis();
  }

  // 3. Step servos smoothly every MOVE_INTERVAL_MS (40 ms)
  if (millis() - lastMoveTime >= MOVE_INTERVAL_MS) {
    lastMoveTime = millis();

    if (manualActive) {
      // ---- Manual Jumper Override (exact behavior from your base sketch) ----
      if (leftPressed && !rightPressed) {
        baseAngle = constrain(baseAngle - STEP_SIZE, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
        moveServo(BASE_SERVO, baseAngle);
      } else if (rightPressed && !leftPressed) {
        baseAngle = constrain(baseAngle + STEP_SIZE, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
        moveServo(BASE_SERVO, baseAngle);
      }

      if (upPressed && !downPressed) {
        tiltAngle = constrain(tiltAngle + STEP_SIZE, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
        moveServo(TILT_SERVO, tiltAngle);
      } else if (downPressed && !upPressed) {
        tiltAngle = constrain(tiltAngle - STEP_SIZE, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
        moveServo(TILT_SERVO, tiltAngle);
      }

      // Sync auto targets so releasing jumper doesn't snap back abruptly
      targetBaseAngle = baseAngle;
      targetTiltAngle = tiltAngle;
    }
    else if (hasSensorLock &&
             (millis() - lastManualTime > MANUAL_HOLD_MS) &&
             (millis() - lastSensorTime < SENSOR_TIMEOUT_MS)) {
      // ---- Automatic Dome-Sensor Tracking with Anti-Hunting Deadband ----
      int baseDiff = targetBaseAngle - baseAngle;
      if (abs(baseDiff) >= ANGLE_DEADBAND) {
        int step = min(STEP_SIZE, abs(baseDiff));
        baseAngle = constrain(baseAngle + (baseDiff > 0 ? step : -step),
                              BASE_MIN_ANGLE, BASE_MAX_ANGLE);
        moveServo(BASE_SERVO, baseAngle);
      }

      int tiltDiff = targetTiltAngle - tiltAngle;
      if (abs(tiltDiff) >= ANGLE_DEADBAND) {
        int step = min(STEP_SIZE, abs(tiltDiff));
        tiltAngle = constrain(tiltAngle + (tiltDiff > 0 ? step : -step),
                              TILT_MIN_ANGLE, TILT_MAX_ANGLE);
        moveServo(TILT_SERVO, tiltAngle);
      }
    }
  }
}
