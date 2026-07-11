#include <Wire.h>
#include <math.h>

#define MPU_ADDRESS 0x68

#define PWR_MGMT_1   0x6B
#define CONFIG_REG   0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define DATA_START   0x3B
#define WHO_AM_I     0x75

float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;

float roll = 0;
float pitch = 0;
float yaw = 0;

unsigned long previousMicros;
unsigned long previousPrint = 0;

void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte readRegister(byte reg) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDRESS, 1, true);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0xFF;
}

bool readSensor(
  int16_t &ax,
  int16_t &ay,
  int16_t &az,
  int16_t &temperature,
  int16_t &gx,
  int16_t &gy,
  int16_t &gz
) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(DATA_START);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(MPU_ADDRESS, 14, true);

  if (Wire.available() != 14) {
    return false;
  }

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  temperature = (Wire.read() << 8) | Wire.read();

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  return true;
}

void calibrateGyroscope() {
  const int samples = 1000;

  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;

  Serial.println("CALIBRATING");
  Serial.println("Keep the vehicle completely still.");

  delay(1500);

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    int16_t temperature;
    int16_t gx, gy, gz;

    if (readSensor(ax, ay, az, temperature, gx, gy, gz)) {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
    }

    delay(2);
  }

  gyroOffsetX = sumX / (float)samples;
  gyroOffsetY = sumY / (float)samples;
  gyroOffsetZ = sumZ / (float)samples;

  Serial.println("READY");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(100000);

  byte deviceID = readRegister(WHO_AM_I);

  Serial.print("WHO_AM_I: 0x");
  Serial.println(deviceID, HEX);

  if (deviceID != 0x70 && deviceID != 0x68) {
    Serial.println("Unsupported sensor identification.");
    while (true) {
      delay(100);
    }
  }

  // Wake sensor and use internal clock
  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);

  // Low-pass filter
  writeRegister(CONFIG_REG, 0x03);

  // Gyroscope range: ±250 degrees/second
  writeRegister(GYRO_CONFIG, 0x00);

  // Accelerometer range: ±2g
  writeRegister(ACCEL_CONFIG, 0x00);

  delay(100);

  calibrateGyroscope();

  int16_t axRaw, ayRaw, azRaw;
  int16_t temperatureRaw;
  int16_t gxRaw, gyRaw, gzRaw;

  if (readSensor(
    axRaw, ayRaw, azRaw,
    temperatureRaw,
    gxRaw, gyRaw, gzRaw
  )) {
    float ax = axRaw / 16384.0;
    float ay = ayRaw / 16384.0;
    float az = azRaw / 16384.0;

    roll = atan2(ay, az) * 180.0 / PI;

    pitch = atan2(
      -ax,
      sqrt((ay * ay) + (az * az))
    ) * 180.0 / PI;
  }

  yaw = 0;
  previousMicros = micros();
}

void loop() {
  int16_t axRaw, ayRaw, azRaw;
  int16_t temperatureRaw;
  int16_t gxRaw, gyRaw, gzRaw;

  if (!readSensor(
    axRaw, ayRaw, azRaw,
    temperatureRaw,
    gxRaw, gyRaw, gzRaw
  )) {
    Serial.println("READ_ERROR");
    delay(50);
    return;
  }

  unsigned long currentMicros = micros();

  float deltaTime =
    (currentMicros - previousMicros) / 1000000.0;

  previousMicros = currentMicros;

  if (deltaTime <= 0 || deltaTime > 0.1) {
    deltaTime = 0.01;
  }

  // ±2g accelerometer scaling
  float ax = axRaw / 16384.0;
  float ay = ayRaw / 16384.0;
  float az = azRaw / 16384.0;

  // ±250°/s gyroscope scaling
  float gyroX =
    (gxRaw - gyroOffsetX) / 131.0;

  float gyroY =
    (gyRaw - gyroOffsetY) / 131.0;

  float gyroZ =
    (gzRaw - gyroOffsetZ) / 131.0;

  float accelerometerRoll =
    atan2(ay, az) * 180.0 / PI;

  float accelerometerPitch =
    atan2(
      -ax,
      sqrt((ay * ay) + (az * az))
    ) * 180.0 / PI;

  // Complementary filter
  roll =
    0.98 * (roll + gyroX * deltaTime) +
    0.02 * accelerometerRoll;

  pitch =
    0.98 * (pitch + gyroY * deltaTime) +
    0.02 * accelerometerPitch;

  yaw += gyroZ * deltaTime;

  if (yaw > 180) {
    yaw -= 360;
  }

  if (yaw < -180) {
    yaw += 360;
  }

  // Send data 50 times per second
  if (millis() - previousPrint >= 20) {
    previousPrint = millis();

    Serial.print(roll, 2);
    Serial.print(",");
    Serial.print(pitch, 2);
    Serial.print(",");
    Serial.println(yaw, 2);
  }
}

