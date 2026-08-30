/**
 * @file Receiver_Arm.ino
 * @brief ESP32 Receiver & Servo Controller for Gesture-Controlled Robotic Arm
 * @details Receives kinematics (pitch/roll) and grip states via ESP-NOW, mapping
 *          telemetry to PCA9685 PWM servo channels with software range limits.
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// --- I2C & PCA9685 Configuration ---
#define I2C_SDA 21
#define I2C_SCL 22
#define PCA9685_ADDR 0x40
#define PWM_FREQ 50 // 50 Hz standard servo frequency

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA9685_ADDR);

// --- Hardware Port Assignments ---
#define CH_BASE         0  // Base rotation (mapped from roll)
#define CH_BOTTOM_ELBOW 1  // Lower link (fixed hold)
#define CH_TOP_ELBOW    2  // Upper link (mapped from pitch)
#define CH_ROTATION     3  // Wrist roll (fixed hold)
#define CH_GRIP         4  // End-effector claw (binary/proportional grip)

// --- Calibrated Resting Angles (Degrees) ---
#define DEFAULT_BASE    60
#define DEFAULT_BOTTOM  130
#define DEFAULT_TOP     80
#define DEFAULT_ROLL    125
#define DEFAULT_GRIP    180 // Parked / Open

// --- Physical Joint Limit Constraints (Degrees) ---
#define BASE_MIN  0
#define BASE_MAX  150
#define TOP_MIN   20
#define TOP_MAX   180
#define GRIP_MIN  125 // Fully Closed
#define GRIP_MAX  180 // Fully Open

// --- Servo Pulse Width Calibration (at 50 Hz / 4096 ticks) ---
#define SERVOMIN  80  // ~400 µs pulse (0°)
#define SERVOMAX  530 // ~2600 µs pulse (180°)

// --- Global State & Data Structures ---
typedef struct struct_message {
  int pitch;
  int roll;
  int grip;
} struct_message;

struct_message incomingData;
volatile bool isTrackingActive = false;

// --- Helper Functions ---
int angleToPulse(int angle) {
  return map(constrain(angle, 0, 180), 0, 180, SERVOMIN, SERVOMAX);
}

void parkArmAtDefaults() {
  pca.setPWM(CH_BASE,         0, angleToPulse(DEFAULT_BASE));
  pca.setPWM(CH_BOTTOM_ELBOW, 0, angleToPulse(DEFAULT_BOTTOM));
  pca.setPWM(CH_TOP_ELBOW,    0, angleToPulse(DEFAULT_TOP));
  pca.setPWM(CH_ROTATION,     0, angleToPulse(DEFAULT_ROLL));
  pca.setPWM(CH_GRIP,         0, angleToPulse(DEFAULT_GRIP));
}

// --- ESP-NOW Packet Receive Callback ---
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
  if (len != sizeof(struct_message)) return;
  memcpy(&incomingData, data, sizeof(incomingData));

  if (!isTrackingActive) return;

  // 1. Base Rotation (Piecewise mapping around neutral 90° center)
  int baseAngle;
  if (incomingData.roll >= 90) {
    baseAngle = map(incomingData.roll, 90, 180, DEFAULT_BASE, BASE_MAX);
  } else {
    baseAngle = map(incomingData.roll, 90, 0, DEFAULT_BASE, BASE_MIN);
  }
  baseAngle = constrain(baseAngle, BASE_MIN, BASE_MAX);

  // 2. Top Elbow Pitch (Inverted direction mapping around neutral 90° center)
  int topAngle;
  if (incomingData.pitch >= 90) {
    topAngle = map(incomingData.pitch, 90, 180, DEFAULT_TOP, TOP_MIN);
  } else {
    topAngle = map(incomingData.pitch, 90, 0, DEFAULT_TOP, TOP_MAX);
  }
  topAngle = constrain(topAngle, TOP_MIN, TOP_MAX);

  // 3. End-Effector Grip
  int gripAngle = constrain(incomingData.grip, GRIP_MIN, GRIP_MAX);

  // Update active servos
  pca.setPWM(CH_BASE,      0, angleToPulse(baseAngle));
  pca.setPWM(CH_TOP_ELBOW, 0, angleToPulse(topAngle));
  pca.setPWM(CH_GRIP,      0, angleToPulse(gripAngle));

  // Maintain fixed joint positions
  pca.setPWM(CH_BOTTOM_ELBOW, 0, angleToPulse(DEFAULT_BOTTOM));
  pca.setPWM(CH_ROTATION,     0, angleToPulse(DEFAULT_ROLL));

  Serial.printf("Base: %3d° | Top: %3d° | Grip: %3d° (%s)\n",
                baseAngle, topAngle, gripAngle,
                (gripAngle == GRIP_MAX) ? "OPEN" : "CLOSED");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  Wire.begin(I2C_SDA, I2C_SCL);
  pca.begin();
  pca.setPWMFreq(PWM_FREQ);

  parkArmAtDefaults();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error: ESP-NOW initialization failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("\n=============================================");
  Serial.println("   ROBOTIC ARM RECEIVER INITIALIZED          ");
  Serial.println("   Commands: 'A' to Activate | 'S' to Park   ");
  Serial.println("=============================================\n");
}

void loop() {
  // Non-blocking serial command interface
  while (Serial.available() > 0) {
    char cmd = (char)Serial.read();

    if (cmd == 'A' || cmd == 'a') {
      isTrackingActive = true;
      Serial.println("\n>>> [ARM ACTIVATED] Tracking live kinematics <<<\n");
    } 
    else if (cmd == 'S' || cmd == 's') {
      isTrackingActive = false;
      parkArmAtDefaults();
      Serial.println("\n>>> [ARM PARKED] Servos returned to resting defaults <<<\n");
    }
  }
}
