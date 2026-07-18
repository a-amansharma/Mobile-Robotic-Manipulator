/*
  STM32F103C8T6 BLUE PILL
  Six potentiometer reader for Mobile Robotic Manipulator

  Potentiometers:
  Gripper     → PA0
  Wrist Roll  → PA1
  Wrist Pitch → PA2
  Elbow       → PA3
  Shoulder    → PA4
  Base        → PA5

  UART:
  PA9 / USART1 TX → ESP32 GPIO16 / RX2
*/

#include <Arduino.h>

constexpr uint8_t NUMBER_OF_POTS = 6;

const uint32_t potPins[NUMBER_OF_POTS] = {
  PA0,
  PA1,
  PA2,
  PA3,
  PA4,
  PA5
};

struct __attribute__((packed)) ArmFrame {
  uint8_t header1;
  uint8_t header2;
  uint16_t pot[NUMBER_OF_POTS];
  uint8_t checksum;
};

uint16_t filteredPot[NUMBER_OF_POTS] = {
  2048, 2048, 2048, 2048, 2048, 2048
};

uint8_t calculateChecksum(const ArmFrame &frame) {
  const uint8_t *data = reinterpret_cast<const uint8_t *>(&frame);

  uint8_t checksum = 0;

  for (size_t i = 0; i < sizeof(ArmFrame) - 1; i++) {
    checksum ^= data[i];
  }

  return checksum;
}

void setup() {
  analogReadResolution(12);

  for (uint8_t i = 0; i < NUMBER_OF_POTS; i++) {
    pinMode(potPins[i], INPUT);
    filteredPot[i] = analogRead(potPins[i]);
  }

  // Green onboard LED (PC13)
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);   // OFF (LED is active LOW)

  // Serial1 uses PA9 TX and PA10 RX
  Serial1.begin(115200);
}

void loop() {

  // --------- Green LED Blink (300 ms) ---------
  static unsigned long previousMillis = 0;
  static bool ledState = false;

  if (millis() - previousMillis >= 300) {
    previousMillis = millis();
    ledState = !ledState;

    // Active LOW LED
    digitalWrite(PC13, ledState ? LOW : HIGH);
  }
  // --------------------------------------------

  ArmFrame frame;

  frame.header1 = 0xAA;
  frame.header2 = 0x55;

  for (uint8_t i = 0; i < NUMBER_OF_POTS; i++) {
    uint16_t rawValue = analogRead(potPins[i]);

    filteredPot[i] =
      ((uint32_t)filteredPot[i] * 3UL + rawValue) / 4UL;

    frame.pot[i] = filteredPot[i];
  }

  frame.checksum = calculateChecksum(frame);

  Serial1.write(
    reinterpret_cast<const uint8_t *>(&frame),
    sizeof(frame)
  );

  delay(20);
}