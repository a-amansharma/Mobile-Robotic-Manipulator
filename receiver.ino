/*
  MOBILE ROBOTIC MANIPULATOR — RECEIVER ESP32

  Fixed wiring:
  Left IBT-2  RPWM GPIO25, LPWM GPIO26
  Right IBT-2 RPWM GPIO27, LPWM GPIO14
  PCA9685 SDA GPIO21, SCL GPIO22
  MPU6050 SDA GPIO18, SCL GPIO19
  Gripper GPIO32, Wrist Roll GPIO33
  Rear HC-SR04 TRIG GPIO4, ECHO GPIO5
  Buzzer GPIO13, Headlight GPIO23
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>
#include <math.h>

constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint8_t NUMBER_OF_POTS = 6;
constexpr uint16_t PACKET_MAGIC = 0x4D52;
constexpr uint8_t PACKET_VERSION = 1;

constexpr uint8_t LEFT_RPWM_PIN = 25;
constexpr uint8_t LEFT_LPWM_PIN = 26;
constexpr uint8_t RIGHT_RPWM_PIN = 27;
constexpr uint8_t RIGHT_LPWM_PIN = 14;

// Fixed LEDC channels reserved only for the four motor PWM pins.
// GPIO32/33 servos use separate timers, preventing PWM interference.
constexpr uint8_t LEFT_RPWM_CHANNEL = 8;
constexpr uint8_t LEFT_LPWM_CHANNEL = 9;
constexpr uint8_t RIGHT_RPWM_CHANNEL = 10;
constexpr uint8_t RIGHT_LPWM_CHANNEL = 11;
constexpr uint32_t MOTOR_PWM_FREQUENCY = 10000;
constexpr uint8_t MOTOR_PWM_RESOLUTION = 8;

constexpr uint8_t PCA_SDA_PIN = 21;
constexpr uint8_t PCA_SCL_PIN = 22;
constexpr uint8_t MPU_SDA_PIN = 18;
constexpr uint8_t MPU_SCL_PIN = 19;
constexpr uint8_t MPU6050_ADDRESS = 0x68;

constexpr uint8_t GRIPPER_SERVO_PIN = 32;
constexpr uint8_t WRIST_ROLL_SERVO_PIN = 33;
constexpr uint8_t WRIST_PITCH_CHANNEL = 4;
constexpr uint8_t ELBOW_CHANNEL = 6;
constexpr uint8_t SHOULDER_CHANNEL = 8;
constexpr uint8_t BASE_CHANNEL = 10;

constexpr uint8_t ULTRASONIC_TRIG_PIN = 4;
constexpr uint8_t ULTRASONIC_ECHO_PIN = 5;
constexpr uint8_t BUZZER_PIN = 13;
constexpr uint8_t HEADLIGHT_PIN = 23;

constexpr float REAR_STOP_DISTANCE_CM = 30.0f;
constexpr float REAR_DISTANCE_MINIMUM_CM = 2.0f;
constexpr uint8_t OBSTACLE_CONFIRMATION_COUNT = 1;

constexpr int JOYSTICK_CENTER = 2048;
constexpr int JOYSTICK_DEADZONE = 300;

constexpr bool INVERT_LEFT_MOTOR = false;
constexpr bool INVERT_RIGHT_MOTOR = false;
constexpr bool INVERT_JOYSTICK_X = false;
constexpr bool INVERT_JOYSTICK_Y = false;

constexpr float STEERING_GAIN_WHILE_DRIVING = 2.00f;
constexpr int MOTOR_MINIMUM_PWM = 60;

// Rear-obstacle warning: three short beeps every second, but only while
// the operator is commanding backward movement and the rear is blocked.
constexpr uint32_t OBSTACLE_BEEP_PERIOD_MS = 180;
constexpr uint32_t OBSTACLE_BEEP_ON_TIME_MS = 100;

// MPU6050 tilt warning: 5 beeps per second while absolute roll or pitch
// exceeds 15 degrees. Yaw/flat-ground turning is intentionally ignored.
constexpr float MPU_TILT_WARNING_ANGLE_DEG = 15.0f;
constexpr uint32_t MPU_TILT_BEEP_PERIOD_MS = 200;
constexpr uint32_t MPU_TILT_BEEP_ON_TIME_MS = 100;

// Disconnect indication: two short, clear buzzer-only beeps.
// Total pattern is comfortably below half a second.
constexpr uint32_t DISCONNECT_BEEP_ON_TIME_MS = 120;
constexpr uint32_t DISCONNECT_BEEP_GAP_MS = 90;
constexpr uint32_t DISCONNECT_SIGNAL_DURATION_MS =
    DISCONNECT_BEEP_ON_TIME_MS * 2 + DISCONNECT_BEEP_GAP_MS;

// Both controls must be activated within this window to enter combined blink.
constexpr uint32_t DUAL_BUTTON_WINDOW_MS = 200;
constexpr uint32_t DUAL_BUTTON_BLINK_PERIOD_MS = 160;
constexpr uint32_t DUAL_BUTTON_BLINK_ON_TIME_MS = 80;

constexpr uint16_t PCA_SERVO_MIN = 110;
constexpr uint16_t PCA_SERVO_MAX = 510;
constexpr uint32_t CONTROL_TIMEOUT_MS = 500;
constexpr uint32_t MOTOR_START_DELAY_MS = 500;
constexpr uint32_t MOTOR_RAMP_INTERVAL_MS = 10;
constexpr int MOTOR_RAMP_STEP = 6;
constexpr uint32_t SERIAL_INTERVAL_MS = 250;
constexpr uint32_t MPU_UPDATE_INTERVAL_MS = 20;
constexpr uint32_t SERVO_UPDATE_INTERVAL_MS = 10;
constexpr uint32_t ULTRASONIC_INTERVAL_MS = 40;

struct __attribute__((packed)) ControlPacket {
  uint16_t magic;
  uint8_t version;
  uint32_t packetNumber;
  uint16_t joystickX;
  uint16_t joystickY;
  uint16_t armPot[NUMBER_OF_POTS];
  uint8_t armDataValid;
  uint8_t lightState;
  uint8_t buzzerPressed;
};

ControlPacket pendingPacket = {};
ControlPacket activePacket = {};
portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool newPacketAvailable = false;
volatile uint32_t lastPacketReceivedTime = 0;

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
TwoWire mpuWire = TwoWire(1);
Servo gripperServo;
Servo wristRollServo;

enum ServoIndex {
  GRIPPER = 0,
  WRIST_ROLL = 1,
  WRIST_PITCH = 2,
  ELBOW = 3,
  SHOULDER = 4,
  BASE_ROTATION = 5
};

float currentServoAngle[NUMBER_OF_POTS] = {0, 0, 0, 0, 0, 0};
float targetServoAngle[NUMBER_OF_POTS] = {0, 0, 0, 0, 0, 0};

bool reversePot[NUMBER_OF_POTS] = {
  false, false, false, false, false, false
};

// Slightly expands slave-arm travel to compensate for the observed
// increasing angle shortage compared with the master arm.
// Order: gripper, wrist roll, wrist pitch, elbow, shoulder, base.
constexpr float SERVO_ANGLE_GAIN[NUMBER_OF_POTS] = {
  1.08f, 1.08f, 1.08f, 1.08f, 1.12f, 1.08f
};

float rearDistanceCm = 999.0f;
bool rearObstacleDetected = false;
uint8_t obstacleDetectionCounter = 0;

uint32_t lastUltrasonicTime = 0;
uint32_t lastServoUpdateTime = 0;
uint32_t lastMotorRampTime = 0;
uint32_t communicationStartedTime = 0;
uint32_t lastSerialTime = 0;
uint32_t lastMpuUpdateTime = 0;

float mpuRoll = 0.0f;
float mpuPitch = 0.0f;
float mpuGyroX = 0.0f;
float mpuGyroY = 0.0f;
float mpuGyroZ = 0.0f;
bool mpuAvailable = false;
bool excessiveTiltDetected = false;
float mpuTiltAngle = 0.0f;
uint32_t lastMpuReconnectAttemptTime = 0;

int requestedForwardCommand = 0;
int requestedTurnCommand = 0;
int targetLeftSpeed = 0;
int targetRightSpeed = 0;
int currentLeftSpeed = 0;
int currentRightSpeed = 0;

bool communicationActive = false;
bool previousCommunicationState = false;
bool servosArmed = false;
bool reverseBlocked = false;

bool previousLightState = false;
bool previousBuzzerPressed = false;
bool dualButtonBlinkActive = false;
uint32_t lastLightButtonEventTime = 0;
uint32_t lastBuzzerPressTime = 0;
uint32_t connectionSignalUntil = 0;
uint32_t disconnectSignalStart = 0;

void storeReceivedPacket(const uint8_t *incomingData, int length) {
  if (length != sizeof(ControlPacket)) return;

  ControlPacket temporaryPacket;
  memcpy(&temporaryPacket, incomingData, sizeof(temporaryPacket));

  if (temporaryPacket.magic != PACKET_MAGIC ||
      temporaryPacket.version != PACKET_VERSION) return;

  portENTER_CRITICAL(&packetMux);
  pendingPacket = temporaryPacket;
  lastPacketReceivedTime = millis();
  newPacketAvailable = true;
  portEXIT_CRITICAL(&packetMux);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(const esp_now_recv_info_t *info,
                    const uint8_t *incomingData,
                    int length) {
  storeReceivedPacket(incomingData, length);
}
#else
void onDataReceived(const uint8_t *macAddress,
                    const uint8_t *incomingData,
                    int length) {
  storeReceivedPacket(incomingData, length);
}
#endif

bool attachMotorPwm(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttachChannel(pin, MOTOR_PWM_FREQUENCY,
                           MOTOR_PWM_RESOLUTION, channel);
#else
  ledcSetup(channel, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(pin, channel);
  return true;
#endif
}

void writeMotorPwm(uint8_t pin, uint8_t channel, uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

void setupMotorPwm() {
  bool leftR = attachMotorPwm(LEFT_RPWM_PIN, LEFT_RPWM_CHANNEL);
  bool leftL = attachMotorPwm(LEFT_LPWM_PIN, LEFT_LPWM_CHANNEL);
  bool rightR = attachMotorPwm(RIGHT_RPWM_PIN, RIGHT_RPWM_CHANNEL);
  bool rightL = attachMotorPwm(RIGHT_LPWM_PIN, RIGHT_LPWM_CHANNEL);

  if (!leftR || !leftL || !rightR || !rightL) {
    Serial.printf("PWM attach status L-R:%d L-L:%d R-R:%d R-L:%d\n",
                  leftR, leftL, rightR, rightL);
  }
}

void writeSingleMotor(uint8_t rpwmPin, uint8_t rpwmChannel,
                      uint8_t lpwmPin, uint8_t lpwmChannel,
                      int signedSpeed, bool invertDirection) {
  signedSpeed = constrain(signedSpeed, -255, 255);
  if (invertDirection) signedSpeed = -signedSpeed;

  if (signedSpeed > 0) {
    writeMotorPwm(lpwmPin, lpwmChannel, 0);
    writeMotorPwm(rpwmPin, rpwmChannel, static_cast<uint8_t>(signedSpeed));
  } else if (signedSpeed < 0) {
    writeMotorPwm(rpwmPin, rpwmChannel, 0);
    writeMotorPwm(lpwmPin, lpwmChannel, static_cast<uint8_t>(-signedSpeed));
  } else {
    writeMotorPwm(rpwmPin, rpwmChannel, 0);
    writeMotorPwm(lpwmPin, lpwmChannel, 0);
  }
}

void applyMotorOutputs() {
  writeSingleMotor(LEFT_RPWM_PIN, LEFT_RPWM_CHANNEL,
                   LEFT_LPWM_PIN, LEFT_LPWM_CHANNEL,
                   currentLeftSpeed, INVERT_LEFT_MOTOR);

  writeSingleMotor(RIGHT_RPWM_PIN, RIGHT_RPWM_CHANNEL,
                   RIGHT_LPWM_PIN, RIGHT_LPWM_CHANNEL,
                   currentRightSpeed, INVERT_RIGHT_MOTOR);
}

void stopMotorsImmediately() {
  targetLeftSpeed = 0;
  targetRightSpeed = 0;
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  applyMotorOutputs();
}

int addMinimumMotorPwm(int speed) {
  if (speed == 0) return 0;

  int magnitude = abs(speed);
  magnitude = map(magnitude, 1, 255, MOTOR_MINIMUM_PWM, 255);
  return speed > 0 ? magnitude : -magnitude;
}

int normalizeJoystick(uint16_t rawValue, bool invertAxis) {
  int value = static_cast<int>(rawValue) - JOYSTICK_CENTER;
  if (abs(value) <= JOYSTICK_DEADZONE) return 0;

  int normalizedValue;
  if (value > 0) {
    normalizedValue = map(value, JOYSTICK_DEADZONE,
                          4095 - JOYSTICK_CENTER, 0, 255);
  } else {
    normalizedValue = map(value, -JOYSTICK_DEADZONE,
                          -JOYSTICK_CENTER, 0, -255);
  }

  normalizedValue = constrain(normalizedValue, -255, 255);
  return invertAxis ? -normalizedValue : normalizedValue;
}

void calculateMotorTargets() {
  if (millis() - communicationStartedTime < MOTOR_START_DELAY_MS) {
    requestedForwardCommand = 0;
    requestedTurnCommand = 0;
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    reverseBlocked = false;
    return;
  }

  requestedTurnCommand = normalizeJoystick(activePacket.joystickX,
                                             INVERT_JOYSTICK_X);
  requestedForwardCommand = normalizeJoystick(activePacket.joystickY,
                                                INVERT_JOYSTICK_Y);

  // On this joystick, positive Y is the backward direction.
  // Therefore the rear sensor blocks only backward movement.
  // Forward and normal left/right steering remain available.
  reverseBlocked = rearObstacleDetected && requestedForwardCommand > 0;
  if (reverseBlocked) {
    stopMotorsImmediately();
    return;
  }

  // Keep pure left/right controls unchanged. While the rover is also moving
  // forward or backward, make small sideways joystick movement more effective.
  // At a small turn input, the inside wheel slows strongly (approximately half
  // speed), while full corners still stop the inside wheel without reversing it.
  int effectiveTurnCommand = requestedTurnCommand;
  if (requestedForwardCommand != 0) {
    effectiveTurnCommand = static_cast<int>(roundf(
        effectiveTurnCommand * STEERING_GAIN_WHILE_DRIVING));

    // Do not let partial steering reverse the inside wheel.
    effectiveTurnCommand = constrain(effectiveTurnCommand,
                                     -abs(requestedForwardCommand),
                                      abs(requestedForwardCommand));
  }

  // Swap only the two backward/lower corners.
  if (requestedForwardCommand > 0) {
    effectiveTurnCommand = -effectiveTurnCommand;
  }

  int left = requestedForwardCommand + effectiveTurnCommand;
  int right = requestedForwardCommand - effectiveTurnCommand;
  int highestMagnitude = max(abs(left), abs(right));

  if (highestMagnitude > 255) {
    left = static_cast<long>(left) * 255L / highestMagnitude;
    right = static_cast<long>(right) * 255L / highestMagnitude;
  }

  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  targetLeftSpeed = addMinimumMotorPwm(left);
  targetRightSpeed = addMinimumMotorPwm(right);
}

int rampValue(int currentValue, int targetValue, int maximumStep) {
  if (currentValue < targetValue) return min(currentValue + maximumStep, targetValue);
  if (currentValue > targetValue) return max(currentValue - maximumStep, targetValue);
  return currentValue;
}

void updateMotorRamp() {
  if (millis() - lastMotorRampTime < MOTOR_RAMP_INTERVAL_MS) return;
  lastMotorRampTime = millis();

  currentLeftSpeed = rampValue(currentLeftSpeed, targetLeftSpeed, MOTOR_RAMP_STEP);
  currentRightSpeed = rampValue(currentRightSpeed, targetRightSpeed, MOTOR_RAMP_STEP);
  applyMotorOutputs();
}

float measureRearDistance() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long echoDuration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 25000UL);
  if (echoDuration == 0) return 999.0f;
  return echoDuration * 0.0343f / 2.0f;
}

void updateUltrasonic() {
  if (millis() - lastUltrasonicTime < ULTRASONIC_INTERVAL_MS) return;
  lastUltrasonicTime = millis();

  rearDistanceCm = measureRearDistance();
  bool validObstacle = rearDistanceCm >= REAR_DISTANCE_MINIMUM_CM &&
                       rearDistanceCm <= REAR_STOP_DISTANCE_CM;

  if (validObstacle) {
    if (obstacleDetectionCounter < OBSTACLE_CONFIRMATION_COUNT) {
      obstacleDetectionCounter++;
    }
  } else {
    obstacleDetectionCounter = 0;
  }

  rearObstacleDetected =
      obstacleDetectionCounter >= OBSTACLE_CONFIRMATION_COUNT;
}

uint16_t angleToPcaPulse(float angle) {
  angle = constrain(angle, 0.0f, 180.0f);
  return static_cast<uint16_t>(PCA_SERVO_MIN +
         (angle / 180.0f) * (PCA_SERVO_MAX - PCA_SERVO_MIN));
}

void writeServoAngle(uint8_t servoIndex, float angle) {
  angle = constrain(angle, 0.0f, 180.0f);

  switch (servoIndex) {
    case GRIPPER:
      if (servosArmed) gripperServo.write(static_cast<int>(roundf(angle)));
      break;

    case WRIST_ROLL:
      if (servosArmed) wristRollServo.write(static_cast<int>(roundf(angle)));
      break;

    case WRIST_PITCH:
      pca9685.setPWM(WRIST_PITCH_CHANNEL, 0, angleToPcaPulse(angle));
      break;

    case ELBOW:
      pca9685.setPWM(ELBOW_CHANNEL, 0, angleToPcaPulse(angle));
      break;

    case SHOULDER:
      pca9685.setPWM(SHOULDER_CHANNEL, 0, angleToPcaPulse(angle));
      break;

    case BASE_ROTATION:
      pca9685.setPWM(BASE_CHANNEL, 0, angleToPcaPulse(angle));
      break;
  }
}

float potToServoAngle(uint8_t servoIndex, uint16_t rawPotValue) {
  int potValue = constrain(static_cast<int>(rawPotValue), 0, 4095);
  if (reversePot[servoIndex]) potValue = 4095 - potValue;

  float masterAngle = static_cast<float>(potValue) * 180.0f / 4095.0f;
  float correctedAngle = masterAngle * SERVO_ANGLE_GAIN[servoIndex];
  return constrain(correctedAngle, 0.0f, 180.0f);
}

void armServosAtMasterPosition() {
  // First put PCA9685 servos at the exact received master-arm positions.
  pca9685.setPWM(WRIST_PITCH_CHANNEL, 0, angleToPcaPulse(currentServoAngle[WRIST_PITCH]));
  pca9685.setPWM(ELBOW_CHANNEL, 0, angleToPcaPulse(currentServoAngle[ELBOW]));
  pca9685.setPWM(SHOULDER_CHANNEL, 0, angleToPcaPulse(currentServoAngle[SHOULDER]));
  pca9685.setPWM(BASE_CHANNEL, 0, angleToPcaPulse(currentServoAngle[BASE_ROTATION]));

  // Store the required position before attaching GPIO32/33.
  gripperServo.setPeriodHertz(50);
  wristRollServo.setPeriodHertz(50);
  gripperServo.write(static_cast<int>(roundf(currentServoAngle[GRIPPER])));
  wristRollServo.write(static_cast<int>(roundf(currentServoAngle[WRIST_ROLL])));

  gripperServo.attach(GRIPPER_SERVO_PIN, 500, 2500);
  wristRollServo.attach(WRIST_ROLL_SERVO_PIN, 500, 2500);

  servosArmed = true;
  gripperServo.write(static_cast<int>(roundf(currentServoAngle[GRIPPER])));
  wristRollServo.write(static_cast<int>(roundf(currentServoAngle[WRIST_ROLL])));
}

void updateServoTargets() {
  if (!activePacket.armDataValid) return;

  for (uint8_t i = 0; i < NUMBER_OF_POTS; i++) {
    targetServoAngle[i] = potToServoAngle(i, activePacket.armPot[i]);
  }

  if (!servosArmed) {
    // Start every joint directly at the first valid received master position.
    // No joint is forced through an intermediate startup angle.
    for (uint8_t i = 0; i < NUMBER_OF_POTS; i++) {
      currentServoAngle[i] = targetServoAngle[i];
    }

    armServosAtMasterPosition();
  }
}

void smoothlyUpdateServos() {
  if (!servosArmed ||
      millis() - lastServoUpdateTime < SERVO_UPDATE_INTERVAL_MS) return;

  lastServoUpdateTime = millis();

  for (uint8_t i = 0; i < NUMBER_OF_POTS; i++) {
    float maximumStep;

    if (i == WRIST_ROLL) {
      maximumStep = 10.0f;
    } else if (i == GRIPPER) {
      maximumStep = 6.0f;
    } else {
      maximumStep = 4.0f;
    }

    float difference = targetServoAngle[i] - currentServoAngle[i];

    if (fabsf(difference) <= maximumStep) {
      currentServoAngle[i] = targetServoAngle[i];
    } else {
      currentServoAngle[i] += difference > 0.0f ? maximumStep : -maximumStep;
    }

    writeServoAngle(i, currentServoAngle[i]);
  }
}

bool writeMpuRegister(uint8_t reg, uint8_t value) {
  mpuWire.beginTransmission(MPU6050_ADDRESS);
  mpuWire.write(reg);
  mpuWire.write(value);
  return mpuWire.endTransmission() == 0;
}

void setupMpu6050() {
  mpuWire.begin(MPU_SDA_PIN, MPU_SCL_PIN, 400000);
  delay(20);

  mpuAvailable = writeMpuRegister(0x6B, 0x00);
  if (mpuAvailable) {
    writeMpuRegister(0x1A, 0x03);
    writeMpuRegister(0x1B, 0x00);
    writeMpuRegister(0x1C, 0x00);
  }
}

void updateMpu6050() {
  uint32_t now = millis();

  // MPU6050 works independently of ESP-NOW. If it was not found or briefly
  // disconnects, retry initialization once every second.
  if (!mpuAvailable) {
    if (now - lastMpuReconnectAttemptTime >= 1000) {
      lastMpuReconnectAttemptTime = now;
      setupMpu6050();
    }
    return;
  }

  if (now - lastMpuUpdateTime < MPU_UPDATE_INTERVAL_MS) return;
  lastMpuUpdateTime = now;

  mpuWire.beginTransmission(MPU6050_ADDRESS);
  mpuWire.write(0x3B);
  if (mpuWire.endTransmission(false) != 0) {
    mpuAvailable = false;
    return;
  }

  if (mpuWire.requestFrom(static_cast<int>(MPU6050_ADDRESS), 14, true) != 14) {
    return;
  }

  int16_t axRaw = (mpuWire.read() << 8) | mpuWire.read();
  int16_t ayRaw = (mpuWire.read() << 8) | mpuWire.read();
  int16_t azRaw = (mpuWire.read() << 8) | mpuWire.read();
  mpuWire.read();
  mpuWire.read();
  int16_t gxRaw = (mpuWire.read() << 8) | mpuWire.read();
  int16_t gyRaw = (mpuWire.read() << 8) | mpuWire.read();
  int16_t gzRaw = (mpuWire.read() << 8) | mpuWire.read();

  float ax = axRaw / 16384.0f;
  float ay = ayRaw / 16384.0f;
  float az = azRaw / 16384.0f;

  mpuRoll = atan2f(ay, az) * 180.0f / PI;
  mpuPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
  mpuGyroX = gxRaw / 131.0f;
  mpuGyroY = gyRaw / 131.0f;
  mpuGyroZ = gzRaw / 131.0f;

  // Convert the accelerometer readings into physical roll and pitch angles.
  // Tilt angle is the larger absolute value of roll or pitch. Yaw is ignored.
  mpuTiltAngle = max(fabsf(mpuRoll), fabsf(mpuPitch));
  excessiveTiltDetected = mpuTiltAngle > MPU_TILT_WARNING_ANGLE_DEG;
}

void updateCombinedButtonMode() {
  if (!communicationActive) {
    previousLightState = false;
    previousBuzzerPressed = false;
    dualButtonBlinkActive = false;
    lastLightButtonEventTime = 0;
    lastBuzzerPressTime = 0;
    return;
  }

  bool lightState = activePacket.lightState != 0;
  bool buzzerPressed = activePacket.buzzerPressed != 0;
  uint32_t now = millis();

  // lightState is a toggle value, so any change represents a light-button press.
  if (lightState != previousLightState) {
    lastLightButtonEventTime = now;
  }

  // buzzerPressed is momentary, so use only its rising edge.
  if (buzzerPressed && !previousBuzzerPressed) {
    lastBuzzerPressTime = now;
  }

  bool eventsAreClose =
      lastLightButtonEventTime != 0 &&
      lastBuzzerPressTime != 0 &&
      abs(static_cast<int32_t>(lastLightButtonEventTime -
                               lastBuzzerPressTime)) <=
          static_cast<int32_t>(DUAL_BUTTON_WINDOW_MS);

  // Blink only while the buzzer button is still being held after the two
  // button events occurred within 0.5 second. Releasing it returns to normal.
  dualButtonBlinkActive = eventsAreClose && buzzerPressed;

  previousLightState = lightState;
  previousBuzzerPressed = buzzerPressed;
}

void updateBuzzerAndHeadlight() {
  uint32_t now=millis();
  if (now < connectionSignalUntil) {
    digitalWrite(BUZZER_PIN,HIGH);
    digitalWrite(HEADLIGHT_PIN,HIGH);
    return;
  }
  // The MPU6050 warning is completely independent of ESP-NOW and operates
  // immediately after receiver power-up. It also remains active if the
  // transmitter is disconnected.
  if (excessiveTiltDetected) {
    uint32_t phase = now % MPU_TILT_BEEP_PERIOD_MS;
    digitalWrite(BUZZER_PIN,
                 phase < MPU_TILT_BEEP_ON_TIME_MS ? HIGH : LOW);
    digitalWrite(HEADLIGHT_PIN,
                 communicationActive && activePacket.lightState ? HIGH : LOW);
    return;
  }

  if (!communicationActive) {
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(HEADLIGHT_PIN, LOW);
    return;
  }

  if (dualButtonBlinkActive) {
    uint32_t phase = millis() % DUAL_BUTTON_BLINK_PERIOD_MS;
    bool blinkOn = phase < DUAL_BUTTON_BLINK_ON_TIME_MS;
    digitalWrite(BUZZER_PIN, blinkOn ? HIGH : LOW);
    digitalWrite(HEADLIGHT_PIN, blinkOn ? HIGH : LOW);
    return;
  }

  if (reverseBlocked) {
    // Rear warning is active only while reverse is actually blocked.
    uint32_t phase = now % OBSTACLE_BEEP_PERIOD_MS;
    digitalWrite(BUZZER_PIN,
                 phase < OBSTACLE_BEEP_ON_TIME_MS ? HIGH : LOW);
  } else {
    digitalWrite(BUZZER_PIN,
                 activePacket.buzzerPressed ? HIGH : LOW);
  }

  bool storedLight = activePacket.lightState;
  digitalWrite(HEADLIGHT_PIN, storedLight ? HIGH : LOW);
}

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed.");
    while (true) {
      stopMotorsImmediately();
      digitalWrite(BUZZER_PIN, LOW);
      delay(1000);
    }
  }

  esp_now_register_recv_cb(onDataReceived);
}

void setupServoHardwareSafely() {
  // Keep direct-servo pins quiet until the first valid master-arm packet arrives.
  pinMode(GRIPPER_SERVO_PIN, OUTPUT);
  pinMode(WRIST_ROLL_SERVO_PIN, OUTPUT);
  digitalWrite(GRIPPER_SERVO_PIN, LOW);
  digitalWrite(WRIST_ROLL_SERVO_PIN, LOW);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  Wire.begin(PCA_SDA_PIN, PCA_SCL_PIN);
  pca9685.begin();
  pca9685.setOscillatorFrequency(27000000);
  pca9685.setPWMFreq(50);

  // Disable only the four used PCA channels until valid master positions arrive.
  pca9685.setPWM(WRIST_PITCH_CHANNEL, 0, 4096);
  pca9685.setPWM(ELBOW_CHANNEL, 0, 4096);
  pca9685.setPWM(SHOULDER_CHANNEL, 0, 4096);
  pca9685.setPWM(BASE_CHANNEL, 0, 4096);
}

void printDiagnostics() {
  if (millis() - lastSerialTime < SERIAL_INTERVAL_MS) return;
  lastSerialTime = millis();

  Serial.printf(
      "JOY X:%u Y:%u | MOTOR L:%d R:%d | REAR:%.1fcm OBS:%d BLOCK:%d | ",
      activePacket.joystickX,
      activePacket.joystickY,
      currentLeftSpeed,
      currentRightSpeed,
      rearDistanceCm,
      rearObstacleDetected,
      reverseBlocked);

  if (mpuAvailable) {
    Serial.printf("MPU Roll:%.1f Pitch:%.1f TiltAngle:%.1f Alarm:%d | Servos:%s\n",
                  mpuRoll, mpuPitch, mpuTiltAngle, excessiveTiltDetected,
                  servosArmed ? "ARMED" : "WAITING");
  } else {
    Serial.printf("MPU:NOT FOUND | Servos:%s\n",
                  servosArmed ? "ARMED" : "WAITING");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(HEADLIGHT_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(HEADLIGHT_PIN, LOW);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  setupMotorPwm();
  stopMotorsImmediately();
  setupServoHardwareSafely();
  setupMpu6050();
  setupESPNow();

  Serial.println();
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Receiver ready. MPU6050 tilt warning is active immediately from power-up. Rear warning works only while reversing.");
}

void loop() {
  // Read the MPU6050 continuously from power-up, regardless of ESP-NOW state.
  updateMpu6050();

  uint32_t receivedTime;
  portENTER_CRITICAL(&packetMux);
  receivedTime = lastPacketReceivedTime;
  portEXIT_CRITICAL(&packetMux);

  uint32_t packetAge =
      receivedTime == 0 ? UINT32_MAX : millis() - receivedTime;
  communicationActive =
      receivedTime != 0 && packetAge <= CONTROL_TIMEOUT_MS;

  if (communicationActive && !previousCommunicationState) {
    communicationStartedTime = millis();
    connectionSignalUntil = millis() + 500;
  }
  if (!communicationActive && previousCommunicationState) {
    disconnectSignalStart = millis();
  }
  previousCommunicationState = communicationActive;

  bool copiedNewPacket = false;
  if (communicationActive) {
    portENTER_CRITICAL(&packetMux);
    if (newPacketAvailable) {
      activePacket = pendingPacket;
      newPacketAvailable = false;
      copiedNewPacket = true;
    }
    portEXIT_CRITICAL(&packetMux);
  }

  if (!communicationActive) {
    // Keep the complete rover inactive until valid ESP-NOW packets arrive.
    requestedForwardCommand = 0;
    requestedTurnCommand = 0;
    reverseBlocked = false;
    rearObstacleDetected = false;
    obstacleDetectionCounter = 0;
    rearDistanceCm = 999.0f;
    stopMotorsImmediately();
    digitalWrite(HEADLIGHT_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  } else {
    // Ultrasonic and commanded outputs become active after connection.
    // MPU6050 is already updating independently above.
    updateUltrasonic();

    if (copiedNewPacket) {
      updateServoTargets();
    }

    calculateMotorTargets();
    updateMotorRamp();
  }

  updateCombinedButtonMode();
  smoothlyUpdateServos();
  updateBuzzerAndHeadlight();
  printDiagnostics();
}