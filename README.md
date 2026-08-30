# Wireless Gesture-Controlled Robotic Arm

An end-to-end wearable telemetry and multi-axis robotic actuation system powered by dual ESP32 microcontrollers over the ESP-NOW wireless protocol.

This repository contains the complete embedded C++ firmware, hardware pinouts, signal conditioning algorithms, and kinematic mapping logic for a 5-DoF robotic arm driven by a custom sensor glove.

---

## System Overview

```
 [ DIY Graphite Flex Sensor ] ---> [ ADC1 (GPIO 34) ]
                                            |
 [ MPU6050 6-DoF IMU ] ---------> [ I2C (14/13) ]  ---> [ ESP32 Transmitter ]
                                                                   |
                                                         ( ESP-NOW Protocol )
                                                         ( ~25 Hz Telemetry )
                                                                   |
                                                                   v
 [ 5x MG996R/SG90 Servos ] <--- [ PCA9685 PWM ] <--- [ I2C (21/22) ] <--- [ ESP32 Receiver ]
```

The system captures hand orientation and finger flexion on a wearable glove, packages kinematic parameters into a lightweight 12-byte telemetry struct, and transmits it wirelessly to a base station microcontroller. The receiver decodes the payload, executes joint-limit validation, and drives multi-axis servos via a dedicated 12-bit PWM controller.

---

## Key Features

* **Custom Graphite-Foil Flex Sensor:** Replaces commercial resistive flex sensors with a custom graphite-trace variable resistor, conditioned via a 30-sample moving average and a non-blocking 40 ms latching debounce filter.
* **Ultra-Low-Latency ESP-NOW Transport:** Peer-to-peer MAC-layer wireless communication eliminating Wi-Fi router association overhead.
* **Dedicated 12-Bit PWM Offloading:** Uses an I2C PCA9685 PWM driver to deliver jitter-free 50 Hz pulse widths across all servos without taxing microcontroller hardware timers.
* **Safety & Range-Constrained Kinematics:** Piecewise angle mapping functions enforce mechanical soft limits across all joints to prevent servo stall and binding.
* **Hybrid Structural Fabrication:** Combines laser-cut plywood linkages for planar stiffness with 3D-printed brackets for multi-axis joint mounts.

---

## Hardware Architecture

### Bill of Materials (BOM)

| Component | Function / Subsystem | Interface |
| :--- | :--- | :--- |
| **2x ESP32 DevKit V1** | Transmitter & Receiver Microcontrollers | Wi-Fi (ESP-NOW) |
| **PCA9685 (16-Channel)** | 12-Bit PWM Servo Driver Board | I2C (Address `0x40`) |
| **MPU6050** | 6-DoF Accelerometer & Gyroscope | I2C |
| **Custom Flex Sensor** | Graphite trace + Aluminum foil grip sensor | Analog ADC1 |
| **Micro Servos (SG90 / MG996R)** | Multi-axis arm actuation & gripper claw | PWM (50 Hz) |
| **5V External Power Supply** | Dedicated servo power rail | Direct DC |

---

### Hardware Pinout

#### Glove Transmitter (ESP32)
* `GPIO 14` $\rightarrow$ MPU6050 `SDA`
* `GPIO 13` $\rightarrow$ MPU6050 `SCL`
* `GPIO 34` $\rightarrow$ Flex Sensor Voltage Divider Output (ADC1)
* `3V3` & `GND` $\rightarrow$ Sensor VCC & Common Ground

#### Arm Base Receiver (ESP32)
* `GPIO 21` $\rightarrow$ PCA9685 `SDA`
* `GPIO 22` $\rightarrow$ PCA9685 `SCL`
* `PCA9685 Port 0` $\rightarrow$ Base Rotation Servo (Mapped to Roll)
* `PCA9685 Port 1` $\rightarrow$ Lower Arm Joint (Fixed Support Hold)
* `PCA9685 Port 2` $\rightarrow$ Upper Arm Joint (Mapped to Pitch)
* `PCA9685 Port 3` $\rightarrow$ Wrist Orientation Servo (Fixed Level Hold)
* `PCA9685 Port 4` $\rightarrow$ End-Effector Claw (Mapped to Flex Sensor)

---

## Repository Structure

```
├── firmware/
│   ├── Transmitter_Glove/
│   │   └── Transmitter_Glove.ino   # IMU reading, ADC debounce, ESP-NOW transmit
│   └── Receiver_Arm/
│       └── Receiver_Arm.ino        # ESP-NOW callback, joint limits, PCA9685 control
├── hardware/
│   ├── cad/                        # 3D print STLs & laser-cut DXF chassis files
│   └── schematics/                 # Wiring diagrams and pinout documentation
└── README.md
```

---

## Firmware Highlights

### 1. 40 ms Grip Debounce & Oversampling
To prevent false triggering from electrical contact noise on the DIY graphite resistor, the ADC reading is smoothed across 30 consecutive samples and latched through a time-windowed state machine:

```cpp
// 30-sample moving average filter
long sum = 0;
for (int i = 0; i < ADC_OVERSAMPLES; i++) {
  sum += analogRead(FLEX_PIN);
  delayMicroseconds(200);
}
int currentADC = sum / ADC_OVERSAMPLES;

// State-latch with 40ms debounce window
if ((millis() - gripStateChangeTime) >= GRIP_DEBOUNCE_MS) {
  currentGripAngle = pendingGripAngle;
}
```

### 2. Piecewise Kinematic Angle Mapping
Neutral hand orientations ($0^\circ$ physical tilt) map to a mid-point integer payload ($90$). The receiver translates tilts symmetrically around calibrated rest positions:

```cpp
if (incomingData.roll >= 90) {
  baseAngle = map(incomingData.roll, 90, 180, DEFAULT_BASE, BASE_MAX);
} else {
  baseAngle = map(incomingData.roll, 90, 0, DEFAULT_BASE, BASE_MIN);
}
baseAngle = constrain(baseAngle, BASE_MIN, BASE_MAX);
