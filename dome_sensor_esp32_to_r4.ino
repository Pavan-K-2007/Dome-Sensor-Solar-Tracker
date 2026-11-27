/*
  =============================================================================
  ESP32 DOME SENSOR <-> WEBSITE <-> ARDUINO UNO R4 WIFI (BIDIRECTIONAL BRIDGE)
  =============================================================================
  1. Reads 5 photodiodes on ESP32 ADC1 pins (36, 39, 34, 35, 32) -> sends
     S1,S2,S3,S4,S5\n to both USB Serial (Website) and Wi-Fi UDP (UNO R4 WiFi).
  2. Reads manual servo commands (CMD:MANUAL,base,tilt or CMD:AUTO) sent from
     the Website over USB Serial and immediately forwards them over Wi-Fi UDP
     to the Arduino UNO R4 WiFi!
  =============================================================================
*/

#include <WiFi.h>
#include <WiFiUdp.h>

const char* AP_SSID = "DomeTracker_R4";
const char* AP_PASS = "solar1234";
const uint16_t R4_UDP_PORT = 4210;

WiFiUDP udp;

const int N = 5;
const int PINS[N] = {36, 39, 34, 35, 32}; // S1 Top, S2 N, S3 E, S4 S, S5 W
const int OVERSAMPLE = 16;
const unsigned long PERIOD_MS = 50;       // 20 Hz sensor stream
unsigned long lastWifiRetry = 0;

String webCmdBuf = "";

void connectToArduinoWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // Keep UDP latency low for manual servo jogging
  WiFi.setAutoReconnect(true);
  WiFi.begin(AP_SSID, AP_PASS);
}

void sendUdpToR4(const char* payload) {
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress r4Ip = WiFi.gatewayIP();
    udp.beginPacket(r4Ip, R4_UDP_PORT);
    udp.print(payload);
    udp.print('\n');
    udp.endPacket();
  }
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  for (int i = 0; i < N; i++) {
    pinMode(PINS[i], INPUT);
    analogSetPinAttenuation(PINS[i], ADC_11db);
  }

  connectToArduinoWiFi();
  udp.begin(4211);

  Serial.println("# ESP32 Dome Sensor + Web Servo Bridge ready");
}

void loop() {
  // 1. Forward any Website Servo Commands (CMD:MANUAL,... / CMD:AUTO) to R4 immediately!
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      webCmdBuf.trim();
      if (webCmdBuf.startsWith("CMD:")) {
        sendUdpToR4(webCmdBuf.c_str());
      }
      webCmdBuf = "";
    } else if (webCmdBuf.length() < 96) {
      webCmdBuf += c;
    } else {
      webCmdBuf = "";
    }
  }

  // 2. Auto-reconnect Wi-Fi if UNO R4 WiFi booted later or reset
  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiRetry >= 4000) {
    lastWifiRetry = millis();
    WiFi.disconnect();
    WiFi.begin(AP_SSID, AP_PASS);
  }

  // 3. Sample & transmit 5 photodiode channels at 20 Hz
  static unsigned long lastSend = 0;
  if (millis() - lastSend < PERIOD_MS) return;
  lastSend = millis();

  uint32_t v[N];
  for (int i = 0; i < N; i++) {
    uint32_t sum = 0;
    for (int k = 0; k < OVERSAMPLE; k++) {
      sum += analogRead(PINS[i]);
    }
    v[i] = sum / OVERSAMPLE;
  }

  char line[64];
  snprintf(line, sizeof(line), "%lu,%lu,%lu,%lu,%lu",
           (unsigned long)v[0], (unsigned long)v[1], (unsigned long)v[2],
           (unsigned long)v[3], (unsigned long)v[4]);

  Serial.println(line);
  sendUdpToR4(line);
}
