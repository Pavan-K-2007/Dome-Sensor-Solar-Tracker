# Light-Adaptive Solar Panel System: Dome Sensor Solar Tracker
> **RV College of Engineering®** | Experiential Learning (EL) Phase-II  
> **Theme:** IoT and Embedded Systems  
> **Track:** Solar Tracking & Renewable Energy Automation

---

## 📌 Project Overview

Conventional solar tracking systems often suffer from significant limitations:
- **Fixed solar panels** lose substantial solar energy because they remain static while the sun moves across the sky.
- **Astronomical/Clock-based trackers** follow theoretical solar coordinates regardless of local cloud cover, often pointing towards dark clouds when diffuse reflection is brighter elsewhere.
- **Individual LDR trackers** require individual photoresistors on every panel, increasing wiring complexity, bill of materials, and failure rates (LDRs also exhibit heavy thermal drift).

This project presents a **central 5-photodiode Dome Light Sensor** that determines the real-time direction of peak illumination to intelligently steer a solar panel array with minimal motor hunting.

---

## 🏗 System Architecture

```
[ 5 BPW34 Photodiodes ] 
  (Hemispherical Dome: Top, N, E, S, W)
             │ (Photocurrent, µA)
             ▼
[ 2× MCP6004 Quad Op-Amps ] 
  (Transimpedance Amplifier: 4.7 kΩ ∥ 100 pF)
             │ (Voltage 0 – 3.3V)
             ▼
[ ESP32 DevKit V1 (ADC1) ] 
  (16× Oversampling @ 20 Hz)
             │ (USB Serial / WebSocket: 115200 baud)
             ▼
[ Interactive 3D Web Dashboard ] 
  (Least-Squares Direction Vector + East-West Error + Tilt Preview)
```

---

## ⚡ Circuit & Hardware Specifications

### 1. Sensing Element
- **Photodiodes:** 5× **BPW34** Silicon PIN Photodiodes with high sensitivity and linear response.
- **Arrangement:** Hemispherical arrangement:
  - `S1 Top`: Azimuth $0^\circ$, Elevation $90^\circ$ (Zenith)
  - `S2 North`: Azimuth $0^\circ$, Elevation $55^\circ$
  - `S3 East`: Azimuth $90^\circ$, Elevation $55^\circ$
  - `S4 South`: Azimuth $180^\circ$, Elevation $55^\circ$
  - `S5 West`: Azimuth $270^\circ$, Elevation $55^\circ$

### 2. Transimpedance Amplifier Stage
- **Op-Amps:** 2× **MCP6004** Rail-to-Rail I/O quad op-amps powered from the ESP32 3.3V rail.
- **Gain:** Transimpedance feedback resistance $R_f = 4.7\text{ k}\Omega$ with $C_f = 100\text{ pF}$ filter capacitor.
- **Gain Multiplication:** Yields $\sim 47\times$ higher voltage output compared to passive $100\ \Omega$ shunt measurement ($40\text{ mV}$ in ambient room light, $70\text{ mV}$ with torch test).
- **Decoupling:** $100\text{ nF}$ decoupling capacitor placed across Pins 4 ($V_{DD}$) and 11 ($V_{SS}$) of each chip.

### 3. ESP32 Pin Allocation (ADC1)
ADC1 pins are used so analog conversions remain active even with Wi-Fi / Bluetooth enabled:
| Sensor Channel | Dome Orientation | Op-Amp Pin | ESP32 GPIO |
|---|---|---|---|
| **Sensor 1** | Top (Zenith) | U1 Pin 1 | **GPIO 36 (VP)** |
| **Sensor 2** | North | U1 Pin 7 | **GPIO 39 (VN)** |
| **Sensor 3** | East | U1 Pin 8 | **GPIO 34** |
| **Sensor 4** | South | U1 Pin 14 | **GPIO 35** |
| **Sensor 5** | West | U2 Pin 1 | **GPIO 32** |

---

## 🧮 Mathematical Modeling & Tracking Logic

### 1. 3D Light Vector Estimation (Least Squares)
Under a diffuser, photodiode response follows Lambert's cosine law:
$$R_i \approx \mathbf{S}_i \cdot \mathbf{L}$$
where $\mathbf{S}_i$ is the known unit normal of sensor $i$ and $\mathbf{L} = [L_x, L_y, L_z]^T$ is the unknown light direction vector.
To prevent top-zenith bias (which occurs with standard weighted averages), the dashboard formulates a least-squares normal equation over active sensors ($k \ge 3$):
$$\mathbf{A} = \sum_{i \in \text{active}} \mathbf{S}_i \mathbf{S}_i^T, \quad \mathbf{b} = \sum_{i \in \text{active}} R_i \mathbf{S}_i$$
$$\mathbf{L} = \mathbf{A}^{-1} \mathbf{b}$$
Azimuth ($\theta$) and Elevation ($\phi$) are derived from the normalized vector $\mathbf{\hat{L}}$:
$$\theta = \left(\text{atan2}(\hat{L}_x, \hat{L}_z) \times \frac{180}{\pi} + 360\right) \bmod 360^\circ$$
$$\phi = \arcsin(\hat{L}_y) \times \frac{180}{\pi}$$

### 2. East-West Tracking Error & Deadband
Single-axis tracking computes normalized differential illumination:
$$\text{Error}_{EW} = \frac{E - W}{E + W}$$
- **Threshold / Deadband:** If $|\text{Error}_{EW}| < 0.05$, the system status is set to `HOLD` to eliminate continuous mechanical oscillation (hunting).
- **Direction:** If $\text{Error}_{EW} > +0.05$, tilt toward East; if $\text{Error}_{EW} < -0.05$, tilt toward West.

### 3. Diffuse Sky (Cloudy) Detection
The 4 perimeter sensors ($S_2, S_3, S_4, S_5$) are evaluated for uniformity:
$$\text{Spread} = \frac{\max(S_\text{ring}) - \min(S_\text{ring})}{\max(S_\text{ring})}$$
If $\text{Spread} < 0.20$, light is diffuse (cloudy). The tracker stows the solar panel **flat** to maximize isotropic diffuse radiation capture and prevent parasitic motor consumption.

---

## 💻 Dashboard Features (`index.html`)

- **3D Celestial Dome Canvas:** Real-time projection of celestial hemisphere with azimuth indicators ($0^\circ, 90^\circ, 180^\circ, 270^\circ$).
- **Web Serial API Integration:** Directly streams live sensor readings from the ESP32 over USB serial in Chromium browsers (Chrome/Edge) without needing local intermediary servers.
- **WebSocket Fallback:** Network telemetry support over Wi-Fi (`ws://<ESP32_IP>:81`).
- **Live HUD Metrics:**
  - Sun Azimuth & Elevation angles
  - Normalized East-West Error
  - Sky Condition Classifier: `Dark` | `Direct light` | `Diffuse (cloudy)`
  - Decision Preview: `toward East` | `toward West` | `near flat (cloudy)` | `hold (inside deadband)`
  - Individual sensor bar indicators and raw ADC counts.
- **Calibration Utilities:** Single-click Dark Offset Zeroing (`Zero (dark)`) with persistent browser `localStorage`.
- **Demo Mode:** Built-in simulation of sun orbit for UI testing without physical hardware connected.

---

## 🚀 How to Run

1. **Flash ESP32 Firmware:**
   - Open `dome_sensor_5_esp32.ino` in Arduino IDE or PlatformIO.
   - Select `ESP32 Dev Module`, compile, and flash to your board at 115200 baud.
2. **Launch Dashboard:**
   - Open `index.html` in Google Chrome or Microsoft Edge.
   - Click **USB Serial** and select the ESP32 COM port.
   - (Optional) Click **Demo** to preview dynamic orbital tracking without hardware.
