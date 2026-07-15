/*
  ESP32 DEVKIT V1 30-PIN — TRANSMITTER
  MOBILE ROBOTIC MANIPULATOR

  Joystick:
  VRX → GPIO34
  VRY → GPIO35
  VCC → 3.3V
  GND → GND

  Motor speed potentiometer:
  Middle pin     → GPIO32
  One outer pin  → 3.3V
  Other outer pin → GND

  Light button:
  GPIO25 → Button → GND

  Buzzer button:
  GPIO33 → Button → GND

  STM32 UART:
  STM32 PA9 TX → ESP32 GPIO16 RX2
  STM32 GND    → ESP32 GND

  Onboard blue LED:
  GPIO2
  OFF when receiver is disconnected
  ON continuously when ESP-NOW packets are delivered

  IMPORTANT:
  Keep the joystick centred and untouched during transmitter startup.

  MOTOR SPEED CONTROL:
  Potentiometer minimum → Rover stops
  Potentiometer middle  → Approximately half speed
  Potentiometer maximum → Full speed

  The speed changes live while the rover is moving.

  The ESP-NOW packet structure remains unchanged,
  so the receiver code does not require modification.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

constexpr uint8_t ESPNOW_CHANNEL = 1;

constexpr uint8_t JOYSTICK_X_PIN = 34;
constexpr uint8_t JOYSTICK_Y_PIN = 35;

// Motor speed potentiometer uses ADC1
constexpr uint8_t SPEED_POT_PIN = 32;

// Light button moved from GPIO32 to GPIO25
constexpr uint8_t LIGHT_BUTTON_PIN = 25;

constexpr uint8_t BUZZER_BUTTON_PIN = 33;

constexpr uint8_t STM32_RX_PIN = 16;
constexpr uint8_t STM32_TX_PIN = 17;

constexpr uint8_t BLUE_LED_PIN = 2;

constexpr uint8_t NUMBER_OF_POTS = 6;

constexpr uint16_t PACKET_MAGIC = 0x4D52;
constexpr uint8_t PACKET_VERSION = 1;

constexpr uint32_t SEND_INTERVAL_MS = 20;
constexpr uint32_t PRINT_INTERVAL_MS = 250;

constexpr uint32_t CONNECTION_TIMEOUT_MS = 1000;
constexpr uint32_t STM32_DATA_TIMEOUT_MS = 500;

constexpr uint32_t BUTTON_DEBOUNCE_MS = 35;

constexpr uint8_t JOYSTICK_FILTER_SAMPLES = 8;
constexpr uint8_t SPEED_POT_FILTER_SAMPLES = 8;

constexpr uint16_t CALIBRATION_SAMPLES = 150;
constexpr uint16_t CALIBRATION_DELAY_MS = 8;

// ADC noise near the minimum position is treated as zero.
constexpr uint16_t SPEED_POT_ZERO_THRESHOLD = 40;

// Receiver ESP32 MAC address
uint8_t receiverMac[] = {
  0x30, 0x76, 0xF5, 0xE4, 0xFD, 0xD4
};

struct __attribute__((packed)) ArmFrame {
  uint8_t header1;
  uint8_t header2;
  uint16_t pot[NUMBER_OF_POTS];
  uint8_t checksum;
};

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

ControlPacket outgoingPacket = {};

uint16_t latestArmPot[NUMBER_OF_POTS] = {
  2048, 2048, 2048, 2048, 2048, 2048
};

uint16_t joystickCenterX = 2048;
uint16_t joystickCenterY = 2048;

uint16_t currentSpeedPotValue = 0;

uint32_t packetCounter = 0;
uint32_t lastSendTime = 0;
uint32_t lastPrintTime = 0;
uint32_t lastValidArmFrameTime = 0;

bool lightState = false;

bool previousLightReading = HIGH;
bool stableLightReading = HIGH;

uint32_t lastLightDebounceTime = 0;

uint8_t armReceiveBuffer[sizeof(ArmFrame)];
size_t armReceiveIndex = 0;

volatile bool receiverConnected = false;
volatile uint32_t lastSuccessfulSendTime = 0;

uint8_t calculateArmChecksum(const ArmFrame &frame) {
  const uint8_t *data =
    reinterpret_cast<const uint8_t *>(&frame);

  uint8_t checksum = 0;

  for (size_t i = 0; i < sizeof(ArmFrame) - 1; i++) {
    checksum ^= data[i];
  }

  return checksum;
}

uint16_t readAverageAnalog(
  uint8_t pin,
  uint8_t numberOfSamples
) {
  uint32_t total = 0;

  for (uint8_t i = 0; i < numberOfSamples; i++) {
    total += analogRead(pin);
  }

  return static_cast<uint16_t>(
    total / numberOfSamples
  );
}

uint16_t readAverageJoystick(uint8_t pin) {
  return readAverageAnalog(
    pin,
    JOYSTICK_FILTER_SAMPLES
  );
}

uint16_t readSpeedPotentiometer() {
  uint16_t rawSpeed = readAverageAnalog(
    SPEED_POT_PIN,
    SPEED_POT_FILTER_SAMPLES
  );

  rawSpeed = constrain(rawSpeed, 0, 4095);

  if (rawSpeed <= SPEED_POT_ZERO_THRESHOLD) {
    return 0;
  }

  return static_cast<uint16_t>(
    map(
      rawSpeed,
      SPEED_POT_ZERO_THRESHOLD,
      4095,
      0,
      4095
    )
  );
}

/*
  Converts the actual joystick centre reading to 2048.
*/
uint16_t recenterJoystick(
  uint16_t rawValue,
  uint16_t measuredCenter
) {
  rawValue = constrain(rawValue, 0, 4095);
  measuredCenter = constrain(measuredCenter, 200, 3895);

  long correctedValue;

  if (rawValue >= measuredCenter) {
    correctedValue = map(
      rawValue,
      measuredCenter,
      4095,
      2048,
      4095
    );
  } else {
    correctedValue = map(
      rawValue,
      0,
      measuredCenter,
      0,
      2048
    );
  }

  return static_cast<uint16_t>(
    constrain(correctedValue, 0L, 4095L)
  );
}

/*
  Scales joystick movement around the centre value of 2048.

  Speed potentiometer at zero:
  Every joystick position becomes 2048, so the rover stops.

  Speed potentiometer at half:
  Joystick movement is reduced to approximately half.

  Speed potentiometer at maximum:
  The complete joystick movement is transmitted.
*/
uint16_t applySpeedLimitToJoystick(
  uint16_t joystickValue,
  uint16_t speedValue
) {
  joystickValue = constrain(joystickValue, 0, 4095);
  speedValue = constrain(speedValue, 0, 4095);

  if (speedValue == 0) {
    return 2048;
  }

  int32_t joystickDifference =
    static_cast<int32_t>(joystickValue) - 2048;

  int32_t scaledDifference =
    joystickDifference *
    static_cast<int32_t>(speedValue) /
    4095;

  int32_t scaledJoystickValue =
    2048 + scaledDifference;

  return static_cast<uint16_t>(
    constrain(
      scaledJoystickValue,
      static_cast<int32_t>(0),
      static_cast<int32_t>(4095)
    )
  );
}

void calibrateJoystick() {
  digitalWrite(BLUE_LED_PIN, LOW);

  Serial.println();
  Serial.println("Joystick calibration started.");
  Serial.println("Keep the joystick centred and untouched.");

  uint32_t totalX = 0;
  uint32_t totalY = 0;

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
    totalX += analogRead(JOYSTICK_X_PIN);
    totalY += analogRead(JOYSTICK_Y_PIN);

    delay(CALIBRATION_DELAY_MS);
  }

  joystickCenterX =
    static_cast<uint16_t>(
      totalX / CALIBRATION_SAMPLES
    );

  joystickCenterY =
    static_cast<uint16_t>(
      totalY / CALIBRATION_SAMPLES
    );

  joystickCenterX =
    constrain(joystickCenterX, 200, 3895);

  joystickCenterY =
    constrain(joystickCenterY, 200, 3895);

  Serial.print("Calibrated X centre: ");
  Serial.println(joystickCenterX);

  Serial.print("Calibrated Y centre: ");
  Serial.println(joystickCenterY);

  Serial.println("Joystick calibration completed.");
  Serial.println();
}

void processSTM32Byte(uint8_t receivedByte) {
  if (armReceiveIndex == 0) {
    if (receivedByte == 0xAA) {
      armReceiveBuffer[armReceiveIndex++] =
        receivedByte;
    }

    return;
  }

  if (armReceiveIndex == 1) {
    if (receivedByte == 0x55) {
      armReceiveBuffer[armReceiveIndex++] =
        receivedByte;
    } else {
      armReceiveIndex = 0;

      if (receivedByte == 0xAA) {
        armReceiveBuffer[armReceiveIndex++] =
          receivedByte;
      }
    }

    return;
  }

  armReceiveBuffer[armReceiveIndex++] =
    receivedByte;

  if (armReceiveIndex >= sizeof(ArmFrame)) {
    ArmFrame receivedFrame;

    memcpy(
      &receivedFrame,
      armReceiveBuffer,
      sizeof(receivedFrame)
    );

    armReceiveIndex = 0;

    if (
      receivedFrame.header1 == 0xAA &&
      receivedFrame.header2 == 0x55 &&
      receivedFrame.checksum ==
        calculateArmChecksum(receivedFrame)
    ) {
      for (uint8_t i = 0; i < NUMBER_OF_POTS; i++) {
        latestArmPot[i] = constrain(
          receivedFrame.pot[i],
          0,
          4095
        );
      }

      lastValidArmFrameTime = millis();
    }
  }
}

void readSTM32Data() {
  while (Serial2.available() > 0) {
    processSTM32Byte(
      static_cast<uint8_t>(Serial2.read())
    );
  }
}

void updateLightButton() {
  bool currentReading =
    digitalRead(LIGHT_BUTTON_PIN);

  if (currentReading != previousLightReading) {
    lastLightDebounceTime = millis();
  }

  if (
    millis() - lastLightDebounceTime >=
    BUTTON_DEBOUNCE_MS
  ) {
    if (currentReading != stableLightReading) {
      stableLightReading = currentReading;

      if (stableLightReading == LOW) {
        lightState = !lightState;
      }
    }
  }

  previousLightReading = currentReading;
}

void updateConnectionLed() {
  bool connected = receiverConnected;
  uint32_t lastSuccess = lastSuccessfulSendTime;

  if (
    connected &&
    millis() - lastSuccess >
      CONNECTION_TIMEOUT_MS
  ) {
    receiverConnected = false;
    connected = false;
  }

  digitalWrite(
    BLUE_LED_PIN,
    connected ? HIGH : LOW
  );
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3

void onDataSent(
  const wifi_tx_info_t *txInfo,
  esp_now_send_status_t status
) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastSuccessfulSendTime = millis();
    receiverConnected = true;
  }
}

#else

void onDataSent(
  const uint8_t *macAddress,
  esp_now_send_status_t status
) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastSuccessfulSendTime = millis();
    receiverConnected = true;
  }
}

#endif

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_channel(
    ESPNOW_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  if (esp_now_init() != ESP_OK) {
    Serial.println(
      "ESP-NOW initialization failed."
    );

    while (true) {
      digitalWrite(BLUE_LED_PIN, LOW);
      delay(1000);
    }
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    receiverMac,
    6
  );

  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(
      "Failed to add receiver peer."
    );

    while (true) {
      digitalWrite(BLUE_LED_PIN, LOW);
      delay(1000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LIGHT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_BUTTON_PIN, INPUT_PULLUP);

  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
  pinMode(SPEED_POT_PIN, INPUT);

  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(BLUE_LED_PIN, LOW);

  analogReadResolution(12);

  analogSetPinAttenuation(
    JOYSTICK_X_PIN,
    ADC_11db
  );

  analogSetPinAttenuation(
    JOYSTICK_Y_PIN,
    ADC_11db
  );

  analogSetPinAttenuation(
    SPEED_POT_PIN,
    ADC_11db
  );

  Serial2.begin(
    115200,
    SERIAL_8N1,
    STM32_RX_PIN,
    STM32_TX_PIN
  );

  calibrateJoystick();
  setupESPNow();

  outgoingPacket.magic = PACKET_MAGIC;
  outgoingPacket.version = PACKET_VERSION;

  Serial.print("Transmitter MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.println("ESP-NOW transmitter ready.");

  Serial.println(
    "Speed potentiometer connected to GPIO32."
  );

  Serial.println(
    "Light button connected to GPIO25."
  );
}

void loop() {
  readSTM32Data();
  updateLightButton();
  updateConnectionLed();

  if (
    millis() - lastSendTime >=
    SEND_INTERVAL_MS
  ) {
    lastSendTime = millis();

    uint16_t rawJoystickX =
      readAverageJoystick(JOYSTICK_X_PIN);

    uint16_t rawJoystickY =
      readAverageJoystick(JOYSTICK_Y_PIN);

    currentSpeedPotValue =
      readSpeedPotentiometer();

    uint16_t centredJoystickX =
      recenterJoystick(
        rawJoystickX,
        joystickCenterX
      );

    uint16_t centredJoystickY =
      recenterJoystick(
        rawJoystickY,
        joystickCenterY
      );

    outgoingPacket.magic = PACKET_MAGIC;
    outgoingPacket.version = PACKET_VERSION;

    outgoingPacket.packetNumber =
      ++packetCounter;

    /*
      Apply the speed potentiometer before sending
      the joystick values to the receiver.
    */
    outgoingPacket.joystickX =
      applySpeedLimitToJoystick(
        centredJoystickX,
        currentSpeedPotValue
      );

    outgoingPacket.joystickY =
      applySpeedLimitToJoystick(
        centredJoystickY,
        currentSpeedPotValue
      );

    for (uint8_t i = 0; i < NUMBER_OF_POTS; i++) {
      outgoingPacket.armPot[i] =
        latestArmPot[i];
    }

    outgoingPacket.armDataValid =
      lastValidArmFrameTime != 0 &&
      millis() - lastValidArmFrameTime <=
        STM32_DATA_TIMEOUT_MS
        ? 1
        : 0;

    outgoingPacket.lightState =
      lightState ? 1 : 0;

    outgoingPacket.buzzerPressed =
      digitalRead(BUZZER_BUTTON_PIN) == LOW
        ? 1
        : 0;

    esp_err_t sendResult = esp_now_send(
      receiverMac,
      reinterpret_cast<const uint8_t *>(
        &outgoingPacket
      ),
      sizeof(outgoingPacket)
    );

    if (
      millis() - lastPrintTime >=
      PRINT_INTERVAL_MS
    ) {
      lastPrintTime = millis();

      uint8_t speedPercentage =
        static_cast<uint8_t>(
          map(
            currentSpeedPotValue,
            0,
            4095,
            0,
            100
          )
        );

      Serial.print("X:");
      Serial.print(outgoingPacket.joystickX);

      Serial.print(" Y:");
      Serial.print(outgoingPacket.joystickY);

      Serial.print(" Speed:");
      Serial.print(speedPercentage);
      Serial.print("%");

      Serial.print(" SpeedRaw:");
      Serial.print(currentSpeedPotValue);

      Serial.print(" Pots:");

      for (uint8_t i = 0; i < NUMBER_OF_POTS; i++) {
        Serial.print(outgoingPacket.armPot[i]);

        if (i < NUMBER_OF_POTS - 1) {
          Serial.print(",");
        }
      }

      Serial.print(" STM:");
      Serial.print(
        outgoingPacket.armDataValid
          ? "OK"
          : "LOST"
      );

      Serial.print(" Light:");
      Serial.print(outgoingPacket.lightState);

      Serial.print(" Buzzer:");
      Serial.print(
        outgoingPacket.buzzerPressed
      );

      Serial.print(" Link:");
      Serial.print(
        receiverConnected
          ? "CONNECTED"
          : "DISCONNECTED"
      );

      Serial.print(" Send:");
      Serial.println(
        sendResult == ESP_OK
          ? "QUEUED"
          : "ERROR"
      );
    }
  }
}