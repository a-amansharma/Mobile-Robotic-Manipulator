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

  OLED SSD1306 (safe 3.3V wiring):
  VCC → 3.3V
  GND → GND
  SDA → GPIO21
  SCL → GPIO22
  Display is rotated 180 degrees in software.

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
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

// 0.96-inch SSD1306 OLED (I2C)
// These pins are not used by the existing transmitter wiring.
constexpr uint8_t OLED_SDA_PIN = 21;
constexpr uint8_t OLED_SCL_PIN = 22;
constexpr uint8_t OLED_I2C_ADDRESS = 0x3C;
constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;
constexpr int8_t OLED_RESET_PIN = -1;
constexpr uint32_t DISPLAY_UPDATE_INTERVAL_MS = 80;
constexpr uint16_t DISPLAY_JOYSTICK_DEADZONE = 180;

// Most joystick modules give a lower Y value when pushed forward.
// Change this to false only if the OLED arrow moves backward.
constexpr bool JOYSTICK_FORWARD_IS_LOW = true;

constexpr uint8_t NUMBER_OF_POTS = 6;

constexpr uint16_t PACKET_MAGIC = 0x4D52;
constexpr uint8_t PACKET_VERSION = 1;

constexpr uint32_t SEND_INTERVAL_MS = 20;
constexpr uint32_t PRINT_INTERVAL_MS = 250;

constexpr uint32_t CONNECTION_TIMEOUT_MS = 1000;
constexpr uint32_t STM32_DATA_TIMEOUT_MS = 500;

constexpr uint32_t BUTTON_DEBOUNCE_MS = 35;

constexpr uint8_t JOYSTICK_FILTER_SAMPLES = 8;
constexpr uint8_t SPEED_POT_FILTER_SAMPLES = 16;
constexpr uint8_t DISPLAY_PERCENT_HYSTERESIS = 2;

constexpr uint16_t CALIBRATION_SAMPLES = 150;
constexpr uint16_t CALIBRATION_DELAY_MS = 8;

// ADC noise near the minimum position is treated as zero.
constexpr uint16_t SPEED_POT_ZERO_THRESHOLD = 40;

Adafruit_SSD1306 display(
  OLED_WIDTH,
  OLED_HEIGHT,
  &Wire,
  OLED_RESET_PIN
);

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
uint16_t filteredSpeedPotValue = 0;
bool speedPotFilterInitialized = false;

uint32_t packetCounter = 0;
uint32_t lastSendTime = 0;
uint32_t lastPrintTime = 0;
uint32_t lastDisplayUpdateTime = 0;
uint32_t lastValidArmFrameTime = 0;

bool lightState = false;

bool previousLightReading = HIGH;
bool stableLightReading = HIGH;

uint32_t lastLightDebounceTime = 0;

uint8_t armReceiveBuffer[sizeof(ArmFrame)];
size_t armReceiveIndex = 0;

volatile bool receiverConnected = false;
volatile uint32_t lastSuccessfulSendTime = 0;

bool displayAvailable = false;
uint16_t displayJoystickX = 2048;
uint16_t displayJoystickY = 2048;
uint8_t stableLimitPercentage = 0;
uint8_t stableDrivePercentage = 0;


void drawLightIcon(int16_t x, int16_t y, bool active) {
  if (active) {
    display.fillCircle(x, y, 4, SSD1306_WHITE);
  } else {
    display.drawCircle(x, y, 4, SSD1306_WHITE);
  }

  display.drawLine(x - 2, y + 5, x + 2, y + 5, SSD1306_WHITE);
  display.drawLine(x - 2, y + 7, x + 2, y + 7, SSD1306_WHITE);

  if (active) {
    display.drawLine(x, y - 7, x, y - 5, SSD1306_WHITE);
    display.drawLine(x - 7, y, x - 5, y, SSD1306_WHITE);
    display.drawLine(x + 5, y, x + 7, y, SSD1306_WHITE);
    display.drawLine(x - 5, y - 5, x - 4, y - 4, SSD1306_WHITE);
    display.drawLine(x + 4, y - 4, x + 5, y - 5, SSD1306_WHITE);
  }
}

void drawSpeakerIcon(int16_t x, int16_t y, bool active) {
  // Standard speaker icon:
  // rectangular rear body + a four-corner speaker cone.
  // The lower diagonal starts exactly at the body's lower-right corner.
  constexpr int16_t bodyWidth = 4;
  constexpr int16_t bodyTop = -3;
  constexpr int16_t bodyBottom = 3;
  constexpr int16_t coneRight = 10;
  constexpr int16_t coneTop = -7;
  constexpr int16_t coneBottom = 7;

  if (active) {
    display.fillRect(
      x,
      y + bodyTop,
      bodyWidth,
      bodyBottom - bodyTop + 1,
      SSD1306_WHITE
    );

    // Fill the complete four-corner cone using two triangles.
    // This keeps the lower diagonal at the proper lower-right body corner.
    display.fillTriangle(
      x + bodyWidth - 1, y + bodyTop,
      x + coneRight, y + coneTop,
      x + coneRight, y + coneBottom,
      SSD1306_WHITE
    );

    display.fillTriangle(
      x + bodyWidth - 1, y + bodyTop,
      x + coneRight, y + coneBottom,
      x + bodyWidth - 1, y + bodyBottom,
      SSD1306_WHITE
    );

    // Three sound waves appear only while the button is pressed.
    display.drawLine(x + 13, y - 2, x + 13, y + 2, SSD1306_WHITE);
    display.drawLine(x + 16, y - 4, x + 16, y + 4, SSD1306_WHITE);
    display.drawLine(x + 19, y - 6, x + 19, y + 6, SSD1306_WHITE);
  } else {
    display.drawRect(
      x,
      y + bodyTop,
      bodyWidth,
      bodyBottom - bodyTop + 1,
      SSD1306_WHITE
    );

    display.drawLine(
      x + bodyWidth - 1, y + bodyTop,
      x + coneRight, y + coneTop,
      SSD1306_WHITE
    );
    display.drawLine(
      x + coneRight, y + coneTop,
      x + coneRight, y + coneBottom,
      SSD1306_WHITE
    );
    display.drawLine(
      x + coneRight, y + coneBottom,
      x + bodyWidth - 1, y + bodyBottom,
      SSD1306_WHITE
    );
  }
}

// Very small 3x5 uppercase font used only for the signature.
uint16_t tinyGlyph(char character) {
  switch (character) {
    case 'A': return 0b010101111101101;
    case 'B': return 0b110101110101110;
    case 'C': return 0b011100100100011;
    case 'D': return 0b110101101101110;
    case 'E': return 0b111100110100111;
    case 'G': return 0b011100101101011;
    case 'H': return 0b101101111101101;
    case 'I': return 0b111010010010111;
    case 'J': return 0b001001001101010;
    case 'M': return 0b101111111101101;
    case 'N': return 0b101111111111101;
    case 'O': return 0b010101101101010;
    case 'P': return 0b110101110100100;
    case 'R': return 0b110101110101101;
    case 'S': return 0b011100010001110;
    case 'T': return 0b111010010010010;
    case 'Y': return 0b101101010010010;
    case '&': return 0b010101010101011;
    case ' ': return 0;
    default:  return 0;
  }
}

void drawTinyText(int16_t x, int16_t y, const char *text) {
  while (*text != '\0') {
    uint16_t glyph = tinyGlyph(*text++);

    for (uint8_t row = 0; row < 5; row++) {
      for (uint8_t column = 0; column < 3; column++) {
        uint8_t bitIndex = 14 - (row * 3 + column);

        if (glyph & (1U << bitIndex)) {
          display.drawPixel(x + column, y + row, SSD1306_WHITE);
        }
      }
    }

    x += 4;
  }
}

void drawTinyTextCondensed(int16_t x, int16_t y, const char *text) {
  while (*text != '\0') {
    uint16_t glyph = tinyGlyph(*text++);

    for (uint8_t row = 0; row < 5; row++) {
      for (uint8_t column = 0; column < 3; column++) {
        uint8_t bitIndex = 14 - (row * 3 + column);

        if (glyph & (1U << bitIndex)) {
          display.drawPixel(x + column, y + row, SSD1306_WHITE);
        }
      }
    }

    // No blank column: intentionally condensed to fit the lower-right corner.
    x += 3;
  }
}

void drawDirectionArrow(uint16_t joystickX, uint16_t joystickY) {
  // Reverse only the OLED X direction so left appears left and right appears right.
  // This does not change the rover control packet.
  int32_t xDifference = 2048 - static_cast<int32_t>(joystickX);
  int32_t yDifference = JOYSTICK_FORWARD_IS_LOW
    ? 2048 - static_cast<int32_t>(joystickY)
    : static_cast<int32_t>(joystickY) - 2048;

  if (abs(xDifference) < DISPLAY_JOYSTICK_DEADZONE) {
    xDifference = 0;
  }

  if (abs(yDifference) < DISPLAY_JOYSTICK_DEADZONE) {
    yDifference = 0;
  }

  // When stopped, the arrow remains centred and points forward.
  if (xDifference == 0 && yDifference == 0) {
    yDifference = 2048;
  }

  float magnitude = sqrtf(
    static_cast<float>(xDifference * xDifference) +
    static_cast<float>(yDifference * yDifference)
  );

  if (magnitude < 1.0f) {
    magnitude = 1.0f;
  }

  constexpr int16_t centreX = 109;
  constexpr int16_t centreY = 34;
  constexpr float arrowLength = 12.0f;
  constexpr float headLength = 5.0f;
  constexpr float headWidth = 3.0f;

  float unitX = static_cast<float>(xDifference) / magnitude;
  float unitY = -static_cast<float>(yDifference) / magnitude;

  int16_t tipX = centreX + static_cast<int16_t>(unitX * arrowLength);
  int16_t tipY = centreY + static_cast<int16_t>(unitY * arrowLength);

  float perpendicularX = -unitY;
  float perpendicularY = unitX;

  int16_t headBaseX = tipX - static_cast<int16_t>(unitX * headLength);
  int16_t headBaseY = tipY - static_cast<int16_t>(unitY * headLength);

  int16_t leftX = headBaseX + static_cast<int16_t>(perpendicularX * headWidth);
  int16_t leftY = headBaseY + static_cast<int16_t>(perpendicularY * headWidth);
  int16_t rightX = headBaseX - static_cast<int16_t>(perpendicularX * headWidth);
  int16_t rightY = headBaseY - static_cast<int16_t>(perpendicularY * headWidth);

  // Only the arrow is shown; the surrounding circle was intentionally removed.
  display.fillCircle(centreX, centreY, 2, SSD1306_WHITE);
  display.drawLine(centreX, centreY, tipX, tipY, SSD1306_WHITE);
  display.fillTriangle(tipX, tipY, leftX, leftY, rightX, rightY, SSD1306_WHITE);
}

uint8_t applyPercentageHysteresis(
  uint8_t previousValue,
  uint8_t newValue
) {
  newValue = constrain(newValue, 0, 100);

  if (newValue == 0 || newValue == 100) {
    return newValue;
  }

  if (
    abs(static_cast<int16_t>(newValue) -
        static_cast<int16_t>(previousValue)) >=
    DISPLAY_PERCENT_HYSTERESIS
  ) {
    return newValue;
  }

  return previousValue;
}

uint8_t calculateDrivePercentage(
  uint16_t joystickX,
  uint16_t joystickY
) {
  int32_t xAmount = abs(
    static_cast<int32_t>(joystickX) - 2048
  );

  int32_t yAmount = abs(
    static_cast<int32_t>(joystickY) - 2048
  );

  int32_t strongestCommand = max(xAmount, yAmount);

  if (strongestCommand < DISPLAY_JOYSTICK_DEADZONE) {
    return 0;
  }

  return static_cast<uint8_t>(
    constrain(
      map(strongestCommand, 0, 2047, 0, 100),
      0L,
      100L
    )
  );
}

void drawHorizontalLever(
  int16_t x,
  int16_t y,
  int16_t width,
  uint8_t percentage
) {
  display.drawRoundRect(x, y, width, 7, 2, SSD1306_WHITE);

  int16_t fillWidth = map(
    percentage,
    0,
    100,
    0,
    width - 4
  );

  if (fillWidth > 0) {
    display.fillRoundRect(
      x + 2,
      y + 2,
      fillWidth,
      3,
      1,
      SSD1306_WHITE
    );
  }
}

void updateOLED() {
  if (!displayAvailable) {
    return;
  }

  if (millis() - lastDisplayUpdateTime < DISPLAY_UPDATE_INTERVAL_MS) {
    return;
  }

  lastDisplayUpdateTime = millis();

  uint8_t newLimitPercentage = static_cast<uint8_t>(
    map(currentSpeedPotValue, 0, 4095, 0, 100)
  );

  uint8_t newDrivePercentage = calculateDrivePercentage(
    displayJoystickX,
    displayJoystickY
  );

  stableLimitPercentage = applyPercentageHysteresis(
    stableLimitPercentage,
    newLimitPercentage
  );

  stableDrivePercentage = applyPercentageHysteresis(
    stableDrivePercentage,
    newDrivePercentage
  );

  bool connected = receiverConnected;
  bool speakerActive = digitalRead(BUZZER_BUTTON_PIN) == LOW;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // One-line status bar: circle/dot + ESP-NOW + bold cross/tick.
  if (connected) {
    display.fillCircle(3, 5, 3, SSD1306_WHITE);
  } else {
    display.drawCircle(3, 5, 3, SSD1306_WHITE);
  }

  display.setTextSize(1);
  display.setCursor(9, 1);
  display.print(F("ESP-NOW"));

  if (connected) {
    // Bold tick with a small gap after ESP-NOW.
    display.drawLine(54, 5, 57, 8, SSD1306_WHITE);
    display.drawLine(57, 8, 63, 1, SSD1306_WHITE);
    display.drawLine(54, 4, 57, 7, SSD1306_WHITE);
    display.drawLine(57, 7, 63, 0, SSD1306_WHITE);
  } else {
    // Bold cross with a small gap after ESP-NOW.
    display.drawLine(55, 1, 62, 8, SSD1306_WHITE);
    display.drawLine(62, 1, 55, 8, SSD1306_WHITE);
    display.drawLine(56, 1, 63, 8, SSD1306_WHITE);
    display.drawLine(63, 1, 56, 8, SSD1306_WHITE);
  }

  // Both icons are shifted to the far-right side.
  drawLightIcon(92, 6, lightState);
  drawSpeakerIcon(105, 6, speakerActive);

  display.drawLine(0, 14, 127, 14, SSD1306_WHITE);

  // SPEED label: clean, plain text with no heavy bold overdraw.
  // It is positioned slightly lower for a visually larger, cleaner appearance.
  display.setTextSize(1);
  display.setCursor(1, 19);
  display.print(F("SPEED"));

  display.setTextSize(2);
  display.setCursor(39, 16);
  if (stableLimitPercentage < 10) {
    display.print('0');
  }
  display.print(stableLimitPercentage);
  display.print('%');

  // SPEED and DRIVE levers use exactly the same dimensions.
  drawHorizontalLever(1, 31, 91, stableLimitPercentage);

  // DRIVE uses the same clean, plain font and alignment as SPEED.
  display.setTextSize(1);
  display.setCursor(1, 43);
  display.print(F("DRIVE"));

  display.setTextSize(2);
  display.setCursor(39, 40);
  if (stableDrivePercentage < 10) {
    display.print('0');
  }
  display.print(stableDrivePercentage);
  display.print('%');

  drawHorizontalLever(1, 55, 91, stableDrivePercentage);

  // Direction arrow remains unchanged.
  drawDirectionArrow(displayJoystickX, displayJoystickY);

  // Compact designer mark at the extreme lower-right corner.
  // It uses the same text size as the ESP-NOW status text.
  display.setTextSize(1);
  display.setCursor(116, 56);
  display.print(F("AS"));

  display.display();
}

void drawCenteredBoldText(const char *text, int16_t y) {
  int16_t x1, y1;
  uint16_t width, height;

  display.setTextSize(1);
  display.getTextBounds(text, 0, y, &x1, &y1, &width, &height);

  int16_t x = (OLED_WIDTH - static_cast<int16_t>(width)) / 2;

  // Draw twice with a one-pixel offset for a bold appearance.
  display.setCursor(x, y);
  display.print(text);
  display.setCursor(x + 1, y);
  display.print(text);
}


void drawCenteredPlainText(const char *text, int16_t y) {
  int16_t x1, y1;
  uint16_t width, height;

  display.setTextSize(1);
  display.getTextBounds(text, 0, y, &x1, &y1, &width, &height);

  int16_t x = (OLED_WIDTH - static_cast<int16_t>(width)) / 2;

  display.setCursor(x, y);
  display.print(text);
}

void showStartupScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Main project title: large-looking, centered and bold.
  drawCenteredBoldText("MOBILE ROBOTIC", 5);
  drawCenteredBoldText("MANIPULATOR", 18);

  display.drawLine(14, 31, 113, 31, SSD1306_WHITE);

  // Startup credit: simple, plain, non-bold text.
  drawCenteredPlainText("PROJECT AND DESIGN", 43);
  drawCenteredPlainText("BY AMAN SHARMA", 54);

  display.display();

  // Show this splash screen at every power-up for exactly two seconds.
  delay(2000);
}

void setupOLED() {
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(400000);

  displayAvailable = display.begin(
    SSD1306_SWITCHCAPVCC,
    OLED_I2C_ADDRESS
  );

  if (!displayAvailable) {
    Serial.println(F("SSD1306 OLED initialization failed."));
    return;
  }

  // The OLED is physically mounted upside down with its pins at the bottom.
  display.setRotation(2);
  showStartupScreen();
}

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
    rawSpeed = 0;
  } else {
    rawSpeed = static_cast<uint16_t>(
      map(
        rawSpeed,
        SPEED_POT_ZERO_THRESHOLD,
        4095,
        0,
        4095
      )
    );
  }

  // Low-pass filtering prevents ADC noise from making 22/23% flicker.
  // The filter remains responsive enough for normal knob movement.
  if (!speedPotFilterInitialized) {
    filteredSpeedPotValue = rawSpeed;
    speedPotFilterInitialized = true;
  } else {
    filteredSpeedPotValue = static_cast<uint16_t>(
      (
        static_cast<uint32_t>(filteredSpeedPotValue) * 3UL +
        static_cast<uint32_t>(rawSpeed)
      ) / 4UL
    );
  }

  if (filteredSpeedPotValue < 20) {
    filteredSpeedPotValue = 0;
  }

  if (filteredSpeedPotValue > 4075) {
    filteredSpeedPotValue = 4095;
  }

  return filteredSpeedPotValue;
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

  setupOLED();
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
  updateOLED();

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

    // OLED direction and DRIVE lever follow the actual speed-limited command.
    displayJoystickX = outgoingPacket.joystickX;
    displayJoystickY = outgoingPacket.joystickY;

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