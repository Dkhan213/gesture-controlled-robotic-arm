/**
 * @file Transmitter_Glove.ino
 * @brief ESP32 Wearable Telemetry Glove for Gesture-Controlled Robotic Arm
 * @details Reads 6-DoF inertial telemetry via MPU6050 (I2C) and finger flexion via
 *          a custom graphite-foil variable resistor. Formats kinematics into a 
 *          compact payload and transmits in real-time over ESP-NOW.
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- Pin & Hardware Definitions ---
#define I2C_SDA 14
#define I2C_SCL 13
#define FLEX_PIN 34 // ADC1 channel (GPIO 34)

// --- End-Effector Grip Calibration ---
#define GRIP_OPEN_ANGLE    180 // Servo degrees for fully opened claw
#define GRIP_CLOSED_ANGLE  125 // Servo degrees for fully closed claw
#define FLEX_THRESHOLD       0 // ADC threshold dividing open and closed states
#define ADC_OVERSAMPLES     30 // Number of reads for moving-average noise filter
#define GRIP_DEBOUNCE_MS    40 // Sustained transition time window (25 Hz debounce)

// --- Target Receiver MAC Address ---
// NOTE: Replace with the MAC address of your receiver ESP32 board
uint8_t receiverAddress[] = {0x20, 0x50, 0x0D, 0xE4, 0xC3, 0xA0};

// --- Telemetry Data Structure ---
typedef struct struct_message {
  int pitch;
  int roll;
  int grip;
} struct_message;

struct_message outgoingData;
esp_now_peer_info_t peerInfo;
Adafruit_MPU6050 mpu;

// --- Grip Debounce State Tracking ---
int currentGripAngle = GRIP_OPEN_ANGLE;
int pendingGripAngle = GRIP_OPEN_ANGLE;
unsigned long gripStateChangeTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(FLEX_PIN, INPUT);

  // Configure ESP32 12-bit ADC (0 - 4095 range, 0 - 3.3V attenuation)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  WiFi.mode(WIFI_STA);
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize MPU6050 IMU
  if (!mpu.begin()) {
    Serial.println("Error: MPU6050 not detected. Verify SDA/SCL wiring.");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize ESP-NOW Protocol
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error: ESP-NOW initialization failed");
    return;
  }

  // Register Peer (Receiver)
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error: Failed to register ESP-NOW peer");
    return;
  }

  Serial.println("\n=============================================");
  Serial.println("   GLOVE TRANSMITTER INITIALIZED & ACTIVE    ");
  Serial.println("=============================================\n");
}

void loop() {
  // --- 1. KINEMATIC INERTIAL TRACKING (MPU6050) ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate raw physical tilt angles (-90° to +90°)
  float rawPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float rawRoll  = atan2(a.acceleration.y, -a.acceleration.z) * 180.0 / PI;

  // Map ±90° physical tilt into standard 0–180 payload (Neutral hand position = 90)
  outgoingData.pitch = constrain((int)rawPitch + 90, 0, 180);
  outgoingData.roll  = constrain((int)rawRoll  + 90, 0, 180);

  // --- 2. FLEX SENSOR SIGNAL FILTERING & DEBOUNCE ---
  long sum = 0;
  for (int i = 0; i < ADC_OVERSAMPLES; i++) {
    sum += analogRead(FLEX_PIN);
    delayMicroseconds(200);
  }
  int currentADC = sum / ADC_OVERSAMPLES;

  // Classify target state based on ADC threshold
  int targetAngle = (currentADC <= FLEX_THRESHOLD) ? GRIP_CLOSED_ANGLE : GRIP_OPEN_ANGLE;

  // Reset timer on state transition
  if (targetAngle != pendingGripAngle) {
    pendingGripAngle = targetAngle;
    gripStateChangeTime = millis();
  }

  // Latch angle change after reading remains stable across debounce window
  if ((millis() - gripStateChangeTime) >= GRIP_DEBOUNCE_MS) {
    currentGripAngle = pendingGripAngle;
  }

  outgoingData.grip = currentGripAngle;

  // --- 3. WIRELESS PAYLOAD DISPATCH ---
  esp_now_send(receiverAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));

  // Debug Output
  Serial.printf("Pitch: %3d° | Roll: %3d° | ADC: %4d | Grip: %3d° (%s)\n",
                outgoingData.pitch,
                outgoingData.roll,
                currentADC,
                currentGripAngle,
                (currentGripAngle == GRIP_OPEN_ANGLE) ? "OPEN" : "CLOSED");

  delay(40); // 25 Hz update loop
}
