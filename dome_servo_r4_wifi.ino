/*
  =============================================================================
  DOME SENSOR + WEB MANUAL DUAL-SERVO CONTROLLER — ARDUINO UNO R4 WIFI
  =============================================================================
  - Hosts Wi-Fi Hotspot "DomeTracker_R4" (password: "solar1234", IP: 192.168.4.1)
  - Receives from ESP32 over UDP (port 4210), USB Serial, OR Direct Wi-Fi HTTP:
      * Sensor stream:  S1,S2,S3,S4,S5   -> Automatic 3D Solar Tracking
      * Web Manual cmd: CMD:MANUAL,b,t   -> Moves Base to b (0..180), Tilt to t (0..180)
      * Web Auto cmd:   CMD:AUTO         -> Resumes automatic Dome-Sensor tracking
      * Direct HTTP:    http://192.168.4.1/cmd?m=manual&b=90&t=90 (if Wi-Fi Direct checked)
  - Physical jumper wires on Pins 2, 3, 4, 5 still override anytime!
  =============================================================================
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>

const char* AP_SSID = "DomeTracker_R4";
const char* AP_PASS = "solar1234";
const unsigned int UDP_PORT = 4210;

// 0 = Stationary Central Dome (3D Least-Squares -> Base & Tilt)
// 1 = Dome Mounted ON the Moving Upper Servo (Differential nulling)
#define TRACKING_MODE 0

WiFiUDP udp;
WiFiServer httpServer(80);
Adafruit_PWMServoDriver pwm(0x40);

// Manual jumper-wire inputs
const int BASE_LEFT  = 2;
const int BASE_RIGHT = 3;
const int TILT_UP    = 4;
const int TILT_DOWN  = 5;

// PCA9685 servo channels
const int BASE_SERVO = 0;  // Bottom servo: left/right (Azimuth)
const int TILT_SERVO = 1;  // Upper servo:  up/down    (Elevation, 90 = flat up)

// Servo pulse range in microseconds
const int SERVO_MIN_US = 600;
const int SERVO_MAX_US = 2400;

// Safe movement range
const int BASE_MIN_ANGLE = 0;
const int BASE_MAX_ANGLE = 180;
const int TILT_MIN_ANGLE = 0;
const int TILT_MAX_ANGLE = 180;

const bool INVERT_BASE = false;
const bool INVERT_TILT = false;

// Servo state
int baseAngle = 90;
int tiltAngle = 90;
int targetBaseAngle = 90;
int targetTiltAngle = 90;

// Website Manual Mode state
bool webManualMode = false;
int webTargetBase  = 90;
int webTargetTilt  = 90;

// Movement settings
const int STEP_SIZE     = 2;                    // Auto & jumper step size
const int WEB_STEP_SIZE = 4;                    // Snappier step for website sliders
const int ANGLE_DEADBAND = 3;
const unsigned long MOVE_INTERVAL_MS  = 35;
const unsigned long MANUAL_HOLD_MS    = 2500;
const unsigned long SENSOR_TIMEOUT_MS = 3000;

unsigned long lastMoveTime   = 0;
unsigned long lastManualTime = 0;
unsigned long lastSensorTime = 0;

// Dome Sensor Geometry & Calibration
const int MAX_SENSORS = 9;
const float ADC_FULL_SCALE   = 4095.0f;
const float MIN_LIGHT_THR    = 0.03f;
const float CLOUD_SPREAD_THR = 0.20f;
const float EMA_ALPHA        = 0.25f;

const float SENSOR_AZ_EL[MAX_SENSORS][2] = {
  {  0.0f, 90.0f}, // S1 Top
  {  0.0f, 55.0f}, // S2 North
  { 90.0f, 55.0f}, // S3 East
  {180.0f, 55.0f}, // S4 South
  {270.0f, 55.0f}, // S5 West
  { 45.0f, 30.0f}, // S6 NE
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

float det3(const float m[3][3]) {
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
       - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
       + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

bool solveSunVector(int count, float outVec[3]) {
  float mx = 0.0f;
  for (int i = 0; i < count; i++) if (smVal[i] > mx) mx = smVal[i];
  if (mx < MIN_LIGHT_THR) return false;

  float thr = max(MIN_LIGHT_THR, 0.08f * mx);
  int actIdx[MAX_SENSORS], actN = 0;
  for (int i = 0; i < count; i++) if (smVal[i] > thr) actIdx[actN++] = i;

  bool solved = false;
  float v[3] = {0.0f, 0.0f, 0.0f};

  if (actN >= 3) {
    float A[3][3] = {{0}}, b[3] = {0};
    for (int k = 0; k < actN; k++) {
      int i = actIdx[k];
      for (int r = 0; r < 3; r++) {
        b[r] += sensorNormals[i][r] * smVal[i];
        for (int c = 0; c < 3; c++) A[r][c] += sensorNormals[i][r] * sensorNormals[i][c];
      }
    }
    float d = det3(A);
    if (fabs(d) > 1e-3f) {
      for (int col = 0; col < 3; col++) {
        float M[3][3];
        for (int r = 0; r < 3; r++)
          for (int c = 0; c < 3; c++) M[r][c] = (c == col) ? b[r] : A[r][c];
        v[col] = det3(M) / d;
      }
      solved = true;
    }
  }
  if (!solved) {
    for (int i = 0; i < count; i++) {
      v[0] += smVal[i] * sensorNormals[i][0];
      v[1] += smVal[i] * sensorNormals[i][1];
      v[2] += smVal[i] * sensorNormals[i][2];
    }
  }

  float mag = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (mag < 1e-4f) return false;
  outVec[0] = v[0] / mag;
  outVec[1] = v[1] / mag;
  outVec[2] = v[2] / mag;
  return true;
}

void updateTargetsFromSensors() {
  float peak = 0.0f;
  for (int i = 0; i < activeSensorCount; i++) if (smVal[i] > peak) peak = smVal[i];

  if (peak < MIN_LIGHT_THR) {
    targetBaseAngle = baseAngle;
    targetTiltAngle = tiltAngle;
    return;
  }

  float rMax = 0.0f, rMin = 1.0f;
  for (int i = 1; i <= 4 && i < activeSensorCount; i++) {
    if (smVal[i] > rMax) rMax = smVal[i];
    if (smVal[i] < rMin) rMin = smVal[i];
  }
  if (rMax > MIN_LIGHT_THR && ((rMax - rMin) / rMax) < CLOUD_SPREAD_THR) {
    targetTiltAngle = constrain(90, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
    targetBaseAngle = baseAngle;
    return;
  }

#if TRACKING_MODE == 0
  float sunVec[3];
  if (!solveSunVector(activeSensorCount, sunVec)) return;

  float azDeg = fmodf(atan2f(sunVec[0], sunVec[2]) * (180.0f / PI) + 360.0f, 360.0f);
  float elDeg = constrain(asinf(constrain(sunVec[1], -1.0f, 1.0f)) * (180.0f / PI), 0.0f, 90.0f);
  float zenithDist = 90.0f - elDeg;

  int newBase = baseAngle, newTilt = 90;
  if (elDeg > 83.0f) {
    newTilt = 90;
  } else if (azDeg >= 90.0f && azDeg <= 270.0f) {
    newBase = (int)roundf(azDeg - 90.0f);
    newTilt = (int)roundf(90.0f - zenithDist);
  } else {
    float wrappedAz = fmodf(azDeg + 180.0f, 360.0f);
    newBase = (int)roundf(wrappedAz - 90.0f);
    newTilt = (int)roundf(90.0f + zenithDist);
  }

  targetBaseAngle = constrain(newBase, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
  targetTiltAngle = constrain(newTilt, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
#else
  float n = smVal[1], e = smVal[2], s = smVal[3], w = smVal[4];
  float ewErr = (e + w > MIN_LIGHT_THR) ? (e - w) / (e + w) : 0.0f;
  float nsErr = (n + s > MIN_LIGHT_THR) ? (n - s) / (n + s) : 0.0f;
  if (fabs(ewErr) > 0.06f)
    targetBaseAngle = constrain(baseAngle + (ewErr > 0 ? -STEP_SIZE : STEP_SIZE), BASE_MIN_ANGLE, BASE_MAX_ANGLE);
  if (fabs(nsErr) > 0.06f)
    targetTiltAngle = constrain(tiltAngle + (nsErr > 0 ? STEP_SIZE : -STEP_SIZE), TILT_MIN_ANGLE, TILT_MAX_ANGLE);
#endif
}

// Handle BOTH Website Commands (CMD:...) AND Sensor Packets (S1,S2,S3,S4,S5)
void handleIncomingPacket(String line) {
  line.trim();
  if (line.length() == 0 || line[0] == '#') return;

  // 1. Check for Website Control Commands
  if (line.startsWith("CMD:AUTO")) {
    webManualMode = false;
    Serial.println("[WEB] Mode -> AUTO (Dome Sensor Tracking)");
    return;
  }
  if (line.startsWith("CMD:MANUAL,")) {
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      int b = line.substring(c1 + 1, c2).toInt();
      int t = line.substring(c2 + 1).toInt();
      webManualMode = true;
      webTargetBase = constrain(b, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
      webTargetTilt = constrain(t, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
      Serial.print("[WEB] Manual -> Base: ");
      Serial.print(webTargetBase);
      Serial.print(" deg | Tilt: ");
      Serial.print(webTargetTilt);
      Serial.println(" deg");
    }
    return;
  }

  // 2. Otherwise parse 5-channel (or 9-channel) Photodiode Readings
  float vals[MAX_SENSORS];
  int count = 0, idx = 0, len = line.length();
  while (idx < len && count < MAX_SENSORS) {
    while (idx < len && (line[idx] == ',' || line[idx] == ' ' || line[idx] == '\t' || line[idx] == '[' || line[idx] == ']')) idx++;
    if (idx >= len) break;
    int start = idx;
    while (idx < len && line[idx] != ',' && line[idx] != ' ' && line[idx] != '\t' && line[idx] != ']') idx++;
    String token = line.substring(start, idx);
    char* endPtr = nullptr;
    float v = strtof(token.c_str(), &endPtr);
    if (endPtr != token.c_str()) vals[count++] = v;
    else return;
  }
  if (count < 5) return;
  activeSensorCount = (count >= 9) ? 9 : 5;

  for (int i = 0; i < activeSensorCount; i++) {
    float span = max(1.0f, ADC_FULL_SCALE - sensorOffset[i]);
    float norm = constrain(sensorGain[i] * (vals[i] - sensorOffset[i]) / span, 0.0f, 1.0f);
    smVal[i] = hasSensorLock ? (smVal[i] + EMA_ALPHA * (norm - smVal[i])) : norm;
  }
  hasSensorLock = true;
  lastSensorTime = millis();
  if (!webManualMode) {
    updateTargetsFromSensors();
  }
}

// Optional Direct Wi-Fi HTTP handler (when laptop/phone joins DomeTracker_R4 Wi-Fi)
void pollHttpDirect() {
  WiFiClient client = httpServer.available();
  if (!client) return;

  unsigned long t0 = millis();
  String req = "";
  while (client.connected() && millis() - t0 < 20) {
    if (client.available()) {
      char c = (char)client.read();
      if (c == '\n') break;
      if (req.length() < 120) req += c;
    }
  }

  if (req.indexOf("/cmd?m=auto") >= 0) {
    webManualMode = false;
  } else if (req.indexOf("/cmd?m=manual") >= 0) {
    int bi = req.indexOf("b=");
    int ti = req.indexOf("t=");
    if (bi > 0 && ti > 0) {
      webManualMode = true;
      webTargetBase = constrain(req.substring(bi + 2).toInt(), BASE_MIN_ANGLE, BASE_MAX_ANGLE);
      webTargetTilt = constrain(req.substring(ti + 2).toInt(), TILT_MIN_ANGLE, TILT_MAX_ANGLE);
    }
  }

  client.print("HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: 2\r\nConnection: close\r\n\r\nOK");
  client.stop();
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < MAX_SENSORS; i++) {
    float az = SENSOR_AZ_EL[i][0] * (PI / 180.0f);
    float el = SENSOR_AZ_EL[i][1] * (PI / 180.0f);
    sensorNormals[i][0] = cosf(el) * sinf(az);
    sensorNormals[i][1] = sinf(el);
    sensorNormals[i][2] = cosf(el) * cosf(az);
  }

  pinMode(BASE_LEFT, INPUT_PULLUP);
  pinMode(BASE_RIGHT, INPUT_PULLUP);
  pinMode(TILT_UP, INPUT_PULLUP);
  pinMode(TILT_DOWN, INPUT_PULLUP);

  Wire.begin();
  if (!pwm.begin()) {
    Serial.println("PCA9685 not detected. Check SDA, SCL and power.");
    while (true) { delay(100); }
  }
  pwm.setPWMFreq(50);
  delay(500);

  moveServo(BASE_SERVO, baseAngle);
  moveServo(TILT_SERVO, tiltAngle);

  WiFi.config(IPAddress(192, 168, 4, 1));
  if (WiFi.beginAP(AP_SSID, AP_PASS) == WL_AP_LISTENING) {
    udp.begin(UDP_PORT);
    httpServer.begin();
    Serial.print("Wi-Fi AP Started! SSID: ");
    Serial.print(AP_SSID);
    Serial.print(" | IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi AP failed to start!");
  }
}

void loop() {
  // 1. Receive UDP packets from ESP32 (both sensor readings & forwarded Web commands!)
  int pktSize = udp.parsePacket();
  if (pktSize > 0) {
    char pkt[160];
    int n = udp.read(pkt, sizeof(pkt) - 1);
    if (n > 0) {
      pkt[n] = '\0';
      handleIncomingPacket(String(pkt));
    }
  }

  // 2. Also accept commands over UNO R4's own USB Serial or Direct Wi-Fi HTTP
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbBuf.length() > 0) { handleIncomingPacket(usbBuf); usbBuf = ""; }
    } else if (usbBuf.length() < 120) usbBuf += c;
    else usbBuf = "";
  }
  pollHttpDirect();

  // 3. Read physical jumper-wire overrides (Pins 2, 3, 4, 5)
  bool leftPressed  = (digitalRead(BASE_LEFT)  == LOW);
  bool rightPressed = (digitalRead(BASE_RIGHT) == LOW);
  bool upPressed    = (digitalRead(TILT_UP)    == LOW);
  bool downPressed  = (digitalRead(TILT_DOWN)  == LOW);
  bool manualJumper = leftPressed || rightPressed || upPressed || downPressed;
  if (manualJumper) lastManualTime = millis();

  // 4. Smoothly step servos every 35 ms
  if (millis() - lastMoveTime >= MOVE_INTERVAL_MS) {
    lastMoveTime = millis();

    if (manualJumper) {
      // Priority 1: Physical Jumper Wires
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
      targetBaseAngle = webTargetBase = baseAngle;
      targetTiltAngle = webTargetTilt = tiltAngle;
    }
    else if (webManualMode) {
      // Priority 2: Website Manual Sliders / D-Pad
      int bDiff = webTargetBase - baseAngle;
      if (bDiff != 0) {
        int step = min(WEB_STEP_SIZE, abs(bDiff));
        baseAngle = constrain(baseAngle + (bDiff > 0 ? step : -step), BASE_MIN_ANGLE, BASE_MAX_ANGLE);
        moveServo(BASE_SERVO, baseAngle);
      }
      int tDiff = webTargetTilt - tiltAngle;
      if (tDiff != 0) {
        int step = min(WEB_STEP_SIZE, abs(tDiff));
        tiltAngle = constrain(tiltAngle + (tDiff > 0 ? step : -step), TILT_MIN_ANGLE, TILT_MAX_ANGLE);
        moveServo(TILT_SERVO, tiltAngle);
      }
      targetBaseAngle = baseAngle;
      targetTiltAngle = tiltAngle;
    }
    else if (hasSensorLock &&
             (millis() - lastManualTime > MANUAL_HOLD_MS) &&
             (millis() - lastSensorTime < SENSOR_TIMEOUT_MS)) {
      // Priority 3: Automatic Dome-Sensor Solar Tracking
      int baseDiff = targetBaseAngle - baseAngle;
      if (abs(baseDiff) >= ANGLE_DEADBAND) {
        int step = min(STEP_SIZE, abs(baseDiff));
        baseAngle = constrain(baseAngle + (baseDiff > 0 ? step : -step), BASE_MIN_ANGLE, BASE_MAX_ANGLE);
        moveServo(BASE_SERVO, baseAngle);
      }
      int tiltDiff = targetTiltAngle - tiltAngle;
      if (abs(tiltDiff) >= ANGLE_DEADBAND) {
        int step = min(STEP_SIZE, abs(tiltDiff));
        tiltAngle = constrain(tiltAngle + (tiltDiff > 0 ? step : -step), TILT_MIN_ANGLE, TILT_MAX_ANGLE);
        moveServo(TILT_SERVO, tiltAngle);
      }
    }
  }
}
