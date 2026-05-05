# Autonomous Pilot Hypoxia & Incapacitation Monitoring System

A real-time, deterministic avionic safety system designed to detect pilot hypoxia and physical incapacitation using physiological and environmental cross-validation. 

Built on an **ESP32** microcontroller, this project processes multi-sensor data (SpO2, barometric pressure, and head tilt) using a single-core, non-blocking superloop architecture to ensure maximum stability and prevent I2C bus collisions.

## 🚀 Engineering Highlights & Architecture

Instead of simply wiring modules together, this project addresses real-world aerospace and embedded system challenges:

*   **Register-Level Hardware Control (MPU6050):** Standard bloated libraries were discarded. The MPU6050 is configured via direct I2C register manipulation (`0x19`, `0x1A`, `0x38`).
*   **Hardware Filtering (DLPF):** Aircraft engine vibrations are mitigated by activating the MPU6050's internal 44Hz Digital Low-Pass Filter and downscaling the sample rate to 50Hz, ensuring only actual pilot head movements are processed.
*   **Interrupt-Driven Polling:** MPU6050 uses a hardware interrupt (`FALLING` edge) to signal data readiness. This prevents polling overhead and CPU waste.
*   **DSP for SpO2 (MAX30102):** Raw DC light division causes 100% false-positive rates due to static tissue interference. This system uses a SparkFun-derived DSP algorithm to buffer 100 samples, separate the AC pulse wave from DC noise, and calculate medical-grade SpO2.
*   **Software Fault Tolerance:** Includes a zero-division protection protocol during pressure derivative calculations to prevent processor exceptions when loop times fall below 1 millisecond.
*   **Moving Average Filter:** BMP280 raw pressure data is smoothed using a 10-sample moving average to prevent false alarms caused by cabin airflow or turbulence.

## ⚙️ Hardware Components
*   **MCU:** ESP32 (Operating on a single-core to ensure thread-safe I2C communication).
*   **MAX30102:** Pulse Oximetry and Heart Rate Sensor (I2C).
*   **MPU6050:** 6-Axis Accelerometer and Gyroscope (I2C).
*   **BMP280:** Barometric Pressure Sensor (I2C).
*   **Indicator:** Standard LED for system state feedback.

## 🧠 Decision Tree & Cross-Validation Logic

The system does not rely on a single point of failure. It cross-validates physiological data with environmental factors to determine the true state of the pilot.

1.  **Sensor Fault Warning:** Verifies tissue contact (`IR > 30000`). If the sensor falls off, the system reports a hardware fault instead of a false emergency.
2.  **Stage 1 (Preventive Warning):** Triggered when cabin pressure drops OR blood oxygen drops, but the pilot's head is upright (pilot is conscious but in danger).
3.  **Stage 2 (Critical Emergency):** Triggered ONLY when physical incapacitation (critical head tilt) is accompanied by physiological distress (SpO2 drop or pressure drop). Indicates severe hypoxia and alerts the cabin crew.

## 💻 Installation & Setup

1. Clone this repository:
   ```bash
   git clone [https://github.com/yourusername/Pilot-Hypoxia-Monitor.git](https://github.com/yourusername/Pilot-Hypoxia-Monitor.git)
Open the project in Arduino IDE.

Ensure the following libraries are installed via the Library Manager:

Wire.h (Built-in)

Adafruit BMP280 Library

SparkFun MAX3010x Pulse and Proximity Sensor Library

Compile and upload to your ESP32 board.

Open the Serial Monitor at 115200 baud rate to observe real-time cross-validation metrics.

📂 I2C Wiring
SDA: GPIO 21

SCL: GPIO 22

MPU6050 INT: GPIO 18

LED Output: GPIO 2

📝 License
This project is open-source and available under the MIT License.