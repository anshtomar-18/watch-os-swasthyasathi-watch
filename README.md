# SwasthyaSathi Watch OS ⌚

**SwasthyaSathi Watch OS (Complete BLE Edition)** is an ESP32-S3 based smartwatch firmware engineered for health monitoring, environmental awareness, and personal safety telemetry.

---

## 🌟 Key Features

- **Health Telemetry**:
  - Pulse & Heart Rate calculation via MAX30102.
  - Blood Oxygen (SpO2) estimation.
  - Finger presence detection.
- **Environmental Tracking**:
  - Ambient Temperature and Humidity readings using DHT22.
- **Location & Safety**:
  - Real-time GPS coordinate tracking (NEO GPS module).
  - **Emergency SOS trigger**: Holding the touch sensor for 3 seconds broadcasts an immediate SOS telemetry packet.
- **Multi-Screen OLED UI**:
  - 128x64 SH1106 OLED display.
  - Navigation cycle via short tap: **Home** ➔ **Health** ➔ **Environment** ➔ **Location**.
- **Connectivity**:
  - **Bluetooth Low Energy (BLE)**: Broadcasts live health, environment, and SOS packets to companion apps.
  - **WiFi & NTP**: Automatic time synchronization with NTP servers.

---

## 🛠️ Hardware & Pin Configuration

| Component | Interface / Pins | ESP32-S3 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **SH1106 OLED** | I2C (Wire) | `SDA: 4`, `SCL: 5` | 128x64 Monochrome Display |
| **MAX30102** | I2C (Wire1) | `SDA: 8`, `SCL: 9` | Heart Rate & Pulse Oximetry |
| **DHT22** | Digital 1-Wire | `Pin 1` | Temperature & Humidity Sensor |
| **NEO GPS** | Hardware Serial | `TX: 17`, `RX: 18` | NMEA GPS coordinates |
| **Touch Sensor** | Digital / Capacitive | `Pin 45` | Short Tap (Screen) / Hold 3s (SOS) |
| **Battery Monitor**| Analog ADC | `Pin 12` | Battery voltage sensing |

---

## 🕹️ Controls & Navigation

- **Short Tap (< 1 sec)**: Cycle to the next screen (`Home` ➔ `Health` ➔ `Environment` ➔ `Location`).
- **Hold (3 seconds)**: Trigger emergency SOS alert over BLE with coordinates.

---

## 🚀 Getting Started

### Prerequisites
- [VS Code](https://code.visualstudio.com/) with the [PlatformIO IDE Extension](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)
- ESP32-S3 DevKitC-1 development board

### Build and Upload
1. Clone the repository:
   ```bash
   git clone https://github.com/anshtomar-18/watch-os-swasthyasathi-watch.git
   cd watch-os-swasthyasathi-watch
   ```
2. Open the project folder in VS Code with PlatformIO.
3. Build the project:
   ```bash
   pio run
   ```
4. Upload firmware to ESP32-S3:
   ```bash
   pio run --target upload
   ```
5. Open Serial Monitor:
   ```bash
   pio device monitor
   ```

---

## 📦 Dependencies
Defined in `platformio.ini`:
- `adafruit/Adafruit GFX Library @ ^1.11.9`
- `adafruit/Adafruit SH110X @ ^2.1.10`
- `sparkfun/SparkFun MAX3010x Pulse and Proximity Sensor Library @ ^1.1.2`
- `adafruit/DHT sensor library @ ^1.4.6`
- `adafruit/Adafruit Unified Sensor @ ^1.1.14`

---

## ⚠️ Disclaimer
*The MAX30102 and firmware calculations are designed for prototype reference and educational purposes. This is not a certified medical device and should not be used for medical diagnostics.*
