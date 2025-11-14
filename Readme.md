# Hand-e-Kraft Tremor Suppression System

This repository contains the complete solution for a Parkinson's Disease tremor suppression system, comprising both the ESP32 firmware (C++/Arduino) and a Flutter mobile application for Bluetooth control and telemetry. The project is divided into two main components:

- **Flutter App:** Enables user interaction and control of the glove via Bluetooth.
- **ESP32 Firmware:** Implements tremor detection and servo control for the mechanical glove.

---

## Repository Structure

```
.
├── final_code.ino
└── hand_e_kraft_appworking/
    └── ... (Flutter app source files)
```
- `final_code.ino`: The main Arduino/C++ firmware for the ESP32, responsible for real-time tremor detection, Bluetooth communication, and servo actuation.
- `hand_e_kraft_appworking/`: A folder containing the full source code of the Flutter mobile application which pairs with the ESP32 device via Bluetooth.

---

## System Overview

### 1. ESP32-Based Tremor Suppression Glove (`final_code.ino`)

#### Features
- **Tremor Detection**: Uses an MPU6050 IMU to detect tremors via FFT analysis of hand movement.
- **Servo Control**: Drives up to 5 servos to suppress tremor via mechanical actuation.
- **Bluetooth Classic (Serial)**: Communicates with the Flutter app for mode selection, status telemetry, and manual servo control.
- **Dual Modes**:
  - **AUTO**: Automatic tremor suppression based on detected motion.
  - **MANUAL**: Direct user control from the app.

#### Code Explanation

##### Core Libraries Used
- `Adafruit_MPU6050` & `Adafruit_Sensor`: For IMU sensor interfacing.
- `ESP32Servo`: To control servo motors.
- `BluetoothSerial`: Enables classic Bluetooth communication.
- `ArduinoFFT`: Enables FFT-based frequency analysis for tremor detection.

##### Key Functions

- **FFT Tremor Analysis**: Samples gyroscope data and computes the dominant tremor frequency and power. Tremor is detected if the frequency is within a pathological band (3-10Hz) and the power exceeds a calibrated threshold.
- **Servo Control**: Servos move to "pinch" positions when tremor is detected, and relax when steady. In manual mode, the user can set positions or oscillate the grip.
- **Bluetooth Command Handling**: The ESP32 listens for commands such as `GRIP`, `RELEASE`, `AUTO_ON`, `AUTO_OFF`, `STATUS`, and direct servo positioning (e.g., `SERVO 1 90`). Telemetry (tremor data, mode, status) is sent back to the app every 250ms.
- **Calibration**: On startup, the system calibrates the sensor's power threshold to distinguish real tremors from noise.
- **Safety & Smooth Movement**: All servo angles are constrained and moved smoothly to avoid mechanical stress.

##### Main Loop Logic

1. Listen for Bluetooth commands.
2. (If manual grip) Oscillate the servos.
3. In AUTO mode, run tremor analysis:
   - If tremor detected, actuate servos for suppression.
   - Else, relax servos or maintain for hysteresis time after tremor.
4. In MANUAL mode, smoothly move servos to user-set positions.
5. Send frequent status telemetry over Bluetooth.

---

### 2. Flutter App (`hand_e_kraft_appworking/`)

#### Features

- **Bluetooth Classic Pairing**: Connects to the ESP32 device (`HandEKraft`) using Bluetooth.
- **User Interface**: Allows toggling between AUTO and MANUAL modes, sending grip/release commands, adjusting individual servos, and viewing real-time telemetry.
- **Status Display**: Shows the current tremor metrics, glove mode, and servo status based on ESP32 telemetry data.

> **Note:** The app folder (`hand_e_kraft_appworking/`) contains all Flutter/Dart files needed for building and running the mobile application. Refer to its own README or documentation for instructions on app setup.

---

## Quick Start

1. **Build and Flash ESP32 Firmware**
   - Open `final_code.ino` in Arduino IDE or PlatformIO.
   - Ensure required libraries are installed.
   - Flash to your ESP32 device.
2. **Install and Run Flutter App**
   - Open `hand_e_kraft_appworking/` in your preferred Flutter IDE.
   - Run `flutter pub get`.
   - Run the app on your mobile device.

3. **Connect Devices**
   - Power on the ESP32 glove. It will show up as `HandEKraft` over Bluetooth.
   - Pair with the glove from the Flutter app.
   - Use the app to switch modes, send grip/release commands, or adjust servos.

---

## Technical Highlights

- **Tremor Analysis:** Uses FFT to detect tremor frequency and amplitude from IMU data—critical for distinguishing pathological tremors from voluntary movements.
- **Bluetooth Telemetry:** Real-time, robust communication between glove and app for both commands and status feedback.
- **Safe and Smooth Actuation:** Designed with mechanical constraints and smooth transitions to avoid damage and ensure user comfort.

---

## File Reference

### `final_code.ino`
- Defines hardware pin mappings, servo configuration, and sensor processing logic.
- Contains all the main logic for tremor suppression, Bluetooth comms, mode handling, and calibration.

### `hand_e_kraft_appworking/`
- Contains all Dart/Flutter code for cross-platform mobile app control.
- Bluetooth serial communication, UI logic, status/telemetry display.

---

## Contributions & Support

PRs, issues, and suggestions are welcome! See individual folders for more details or contact repository owner.

---

## License

This project is released under the MIT License.

---

```
