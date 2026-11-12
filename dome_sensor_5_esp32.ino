/*
  Dome sensor - 5 photodiode amplifier outputs -> ESP32 -> Dome_Sensor_5.html

  Sends one line over USB serial, 20 times a second:   S1,S2,S3,S4,S5
  Example:  512,300,88,1200,4095

  Wiring (classic ESP32 DevKit / ESP32-WROOM). Each pin reads one MCP6004 output:
    S1 Top   -> GPIO 36 (VP)
    S2 North -> GPIO 39 (VN)
    S3 East  -> GPIO 34
    S4 South -> GPIO 35
    S5 West  -> GPIO 32
  All five are ADC1 pins (they keep working if you add Wi-Fi later).
  On an ESP32-S3 / C3 board the pin numbers differ - change PINS below.
  Op-amps must be powered from the ESP32 3V3 pin so outputs never exceed 3.3 V.
*/

#define SEND_MILLIVOLTS 0   // 0 = raw ADC counts 0..4095 (page default)
                            // 1 = calibrated millivolts; then open the page with ?adc=3100

const int N = 5;
const int PINS[N] = {36, 39, 34, 35, 32};   // S1..S5 - same order as the page
const int OVERSAMPLE = 16;                  // average 16 reads per sensor to cut noise
const unsigned long PERIOD_MS = 50;         // 50 ms = 20 lines per second

void setup() {
  Serial.begin(115200);                     // must match ?baud= in the page (default 115200)
  analogReadResolution(12);                 // counts 0..4095
  for (int i = 0; i < N; i++) {
    pinMode(PINS[i], INPUT);
    analogSetPinAttenuation(PINS[i], ADC_11db);   // input range about 0..3.1 V
  }
  delay(500);
  Serial.println("# dome sensor ready: 5 values per line (S1..S5)");  // the page ignores this line
}

void loop() {
  static unsigned long lastSend = 0;
  if (millis() - lastSend < PERIOD_MS) return;
  lastSend = millis();

  for (int i = 0; i < N; i++) {
    uint32_t sum = 0;
    for (int k = 0; k < OVERSAMPLE; k++) {
#if SEND_MILLIVOLTS
      sum += analogReadMilliVolts(PINS[i]);
#else
      sum += analogRead(PINS[i]);
#endif
    }
    Serial.print(sum / OVERSAMPLE);
    if (i < N - 1) Serial.print(',');
  }
  Serial.println();
}
