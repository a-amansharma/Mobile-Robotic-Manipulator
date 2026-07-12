#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

const char* WIFI_NAME = "Mobile-Robotic-Manipulator";
const char* WIFI_PASSWORD = "87654321";

WebServer server(80);

const uint8_t PCA_SDA_PIN = 21;
const uint8_t PCA_SCL_PIN = 22;

const uint8_t MPU_SDA_PIN = 18;
const uint8_t MPU_SCL_PIN = 19;

TwoWire MPU_WIRE = TwoWire(1);

const uint8_t LEFT_RPWM_PIN = 25;
const uint8_t LEFT_LPWM_PIN = 26;

const uint8_t RIGHT_RPWM_PIN = 27;
const uint8_t RIGHT_LPWM_PIN = 14;

const bool REVERSE_LEFT_SIDE = false;
const bool REVERSE_RIGHT_SIDE = false;

const uint32_t MOTOR_PWM_FREQUENCY = 5000;
const uint8_t MOTOR_PWM_RESOLUTION = 8;
const uint8_t LEFT_RPWM_CHANNEL = 0;
const uint8_t LEFT_LPWM_CHANNEL = 1;
const uint8_t RIGHT_RPWM_CHANNEL = 2;
const uint8_t RIGHT_LPWM_CHANNEL = 3;

const int MOTOR_SPEED_LIMIT = 220;
const int JOYSTICK_DEADZONE = 8;

const unsigned long MOTOR_COMMAND_TIMEOUT_MS = 650;

unsigned long lastMotorCommandTime = 0;

const uint8_t BUZZER_PIN = 13;
const uint8_t HEADLIGHT_PIN = 23;

const bool BUZZER_ACTIVE_HIGH = true;
const bool HEADLIGHT_ACTIVE_HIGH = true;

bool buzzerOn = false;
bool headlightOn = false;

const unsigned long BUZZER_TIMEOUT_MS = 700;
unsigned long lastBuzzerRefreshTime = 0;

const uint8_t SERVO_A_PIN = 32;
const uint8_t SERVO_B_PIN = 33;

const uint32_t SERVO_PWM_FREQUENCY = 50;
const uint8_t SERVO_PWM_RESOLUTION = 16;
const uint8_t SERVO_A_CHANNEL = 4;
const uint8_t SERVO_B_CHANNEL = 5;

const uint16_t DIRECT_SERVO_MIN_US = 500;
const uint16_t DIRECT_SERVO_MAX_US = 2500;

Adafruit_PWMServoDriver pca9685 =
  Adafruit_PWMServoDriver(0x40, Wire);

const uint8_t SERVO_C_CHANNEL = 4;
const uint8_t SERVO_D_CHANNEL = 6;
const uint8_t SERVO_E_CHANNEL = 8;
const uint8_t SERVO_F_CHANNEL = 10;

const uint16_t PCA_SERVO_MIN = 102;
const uint16_t PCA_SERVO_MAX = 512;

Adafruit_MPU6050 mpu6050;

bool mpuDetected = false;

bool tiltProtectionEnabled = true;

bool dangerousTilt = false;

float rollAngle = 0.0f;
float pitchAngle = 0.0f;

float rollOffset = 0.0f;
float pitchOffset = 0.0f;

float filteredRoll = 0.0f;
float filteredPitch = 0.0f;

const float TILT_STOP_ANGLE = 45.0f;
const float TILT_RESET_ANGLE = 40.0f;

const unsigned long MPU_INTERVAL_MS = 20;
unsigned long lastMPUReadTime = 0;

const unsigned long STATUS_INTERVAL_MS = 150;

int armAngleA = 0;
int armAngleB = 0;
int armAngleC = 8;
int armAngleD = 8;
int armAngleE = 67;
int armAngleF = 105;

const char WEBPAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">

<head>

<meta charset="UTF-8">

<meta
  name="viewport"
  content="width=device-width,
           initial-scale=1,
           maximum-scale=1,
           user-scalable=no,
           viewport-fit=cover"
>

<title>Mobile Robotic Manipulator</title>

<style>

:root {
  --bg-top: #16243a;
  --bg-bottom: #05080e;
  --panel-top: #263750;
  --panel-bottom: #101827;
  --inner-top: #1c2a40;
  --inner-bottom: #0c121d;
  --text: #f4f8ff;
  --muted: #9baac0;
  --blue: #3cbcff;
  --blue-dark: #0879bb;
  --green: #45e59b;
  --red: #ff4e5c;
  --amber: #ffc24a;
  --track: #35445a;
}

* {
  box-sizing: border-box;
  user-select: none;
  -webkit-user-select: none;
  -webkit-tap-highlight-color: transparent;
}

html,
body {
  width: 100%;
  height: 100%;
  margin: 0;
  overflow: hidden;
  overscroll-behavior: none;
  background:
    radial-gradient(circle at 50% -20%, #29466f 0%, transparent 48%),
    linear-gradient(180deg, var(--bg-top), var(--bg-bottom));
  color: var(--text);
  font-family: Montserrat, Arial, Helvetica, sans-serif;
}

body {
  padding:
    env(safe-area-inset-top)
    env(safe-area-inset-right)
    env(safe-area-inset-bottom)
    env(safe-area-inset-left);
}

.app {
  width: 100%;
  max-width: 760px;
  height: 100dvh;
  margin: auto;
  padding: 7px;
  display: grid;
  grid-template-rows: 44% 56%;
  gap: 7px;
}

.panel {
  position: relative;
  min-height: 0;
  overflow: hidden;
  border-radius: 22px;
  border: 1px solid rgba(255, 255, 255, 0.11);
  background:
    linear-gradient(145deg, var(--panel-top), var(--panel-bottom));
  box-shadow:
    0 18px 35px rgba(0, 0, 0, 0.48),
    inset 0 1px 1px rgba(255, 255, 255, 0.14),
    inset 0 -1px 0 rgba(0, 0, 0, 0.75);
}

.panel::before {
  content: "";
  position: absolute;
  inset: 1px 12% auto;
  height: 1px;
  background:
    linear-gradient(
      90deg,
      transparent,
      rgba(255, 255, 255, 0.35),
      transparent
    );
}

.panel-title {
  height: 35px;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  text-transform: uppercase;
  letter-spacing: 1.2px;
  font-size: 12px;
  font-weight: 800;
}

.status-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background: var(--green);
  box-shadow:
    0 0 5px var(--green),
    0 0 12px var(--green);
}

.drive-layout {
  height: calc(100% - 35px);
  display: grid;
  grid-template-columns: minmax(0, 1fr) 115px;
  gap: 8px;
  padding: 3px 12px 11px;
}

.joystick-container {
  min-height: 0;
  display: flex;
  align-items: center;
  justify-content: center;
}

.joystick {
  position: relative;
  width: min(52vw, 225px);
  height: min(52vw, 225px);
  max-height: 100%;
  aspect-ratio: 1;
  border-radius: 50%;
  touch-action: none;

  background:
    radial-gradient(
      circle at 36% 28%,
      #4c6281 0%,
      #273a56 25%,
      #111b2a 67%,
      #070b12 100%
    );

  border: 2px solid rgba(255, 255, 255, 0.13);

  box-shadow:
    inset 8px 8px 18px rgba(255, 255, 255, 0.05),
    inset -15px -18px 26px rgba(0, 0, 0, 0.76),
    0 13px 21px rgba(0, 0, 0, 0.47),
    0 0 0 5px rgba(0, 0, 0, 0.13);
}

.joystick::before,
.joystick::after {
  content: "";
  position: absolute;
  pointer-events: none;
  background: rgba(255, 255, 255, 0.08);
}

.joystick::before {
  width: 1px;
  height: 82%;
  top: 9%;
  left: 50%;
}

.joystick::after {
  width: 82%;
  height: 1px;
  top: 50%;
  left: 9%;
}

.stick {
  position: absolute;
  width: 36%;
  height: 36%;
  left: 32%;
  top: 32%;
  border-radius: 50%;
  pointer-events: none;

  background:
    radial-gradient(
      circle at 30% 22%,
      #d4f4ff 0%,
      #7fd9ff 12%,
      #31b9ff 38%,
      #0879bc 72%,
      #034b77 100%
    );

  border: 2px solid rgba(255, 255, 255, 0.52);

  box-shadow:
    inset 6px 7px 9px rgba(255, 255, 255, 0.23),
    inset -8px -10px 12px rgba(0, 0, 0, 0.35),
    0 10px 17px rgba(0, 0, 0, 0.52),
    0 0 20px rgba(60, 188, 255, 0.42);

  transform: translate(0, 0);
}

.drive-side {
  display: flex;
  flex-direction: column;
  justify-content: center;
  gap: 7px;
}

.mini-card {
  padding: 7px 5px;
  border-radius: 13px;
  text-align: center;
  background:
    linear-gradient(145deg, #1c2a40, #0b111c);
  border: 1px solid rgba(255, 255, 255, 0.07);
  box-shadow:
    inset 0 1px rgba(255, 255, 255, 0.07),
    0 5px 10px rgba(0, 0, 0, 0.28);
}

.mini-label {
  color: var(--muted);
  font-size: 7px;
  font-weight: 700;
  letter-spacing: 0.8px;
  text-transform: uppercase;
}

.mini-value {
  margin-top: 2px;
  font-size: 15px;
  font-weight: 900;
}

.tilt-card {
  transition: 0.2s ease;
}

.tilt-card.danger {
  background:
    linear-gradient(145deg, #9f2731, #3a0b10);
  border-color: rgba(255, 90, 100, 0.85);
  box-shadow:
    0 0 17px rgba(255, 78, 92, 0.42),
    inset 0 1px rgba(255, 255, 255, 0.14);
}

.control-row {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 6px;
}

.glossy-button {
  position: relative;
  min-height: 38px;
  padding: 7px 4px;
  border: 1px solid rgba(255, 255, 255, 0.13);
  border-radius: 12px;
  color: white;
  font-size: 9px;
  font-weight: 900;
  letter-spacing: 0.3px;
  background:
    linear-gradient(145deg, #31435d, #111a29);
  box-shadow:
    inset 0 1px rgba(255, 255, 255, 0.15),
    inset 0 -2px rgba(0, 0, 0, 0.45),
    0 6px 10px rgba(0, 0, 0, 0.34);
}

.glossy-button::before {
  content: "";
  position: absolute;
  left: 10%;
  right: 10%;
  top: 3px;
  height: 34%;
  border-radius: 8px;
  background:
    linear-gradient(
      180deg,
      rgba(255, 255, 255, 0.16),
      transparent
    );
  pointer-events: none;
}

.glossy-button:active,
.glossy-button.pressed {
  transform: translateY(2px);
  box-shadow:
    inset 0 3px 6px rgba(0, 0, 0, 0.45);
}

.glossy-button.active {
  background:
    linear-gradient(145deg, #28c9ff, #05649f);
  box-shadow:
    inset 0 1px rgba(255, 255, 255, 0.35),
    0 0 13px rgba(60, 188, 255, 0.45);
}

.glossy-button.warning-active {
  background:
    linear-gradient(145deg, #ffca4f, #a76500);
  color: #171000;
}

.stop-button {
  background:
    linear-gradient(145deg, #ff5b68, #8d1019);
  min-height: 36px;
}

.arm-grid {
  height: calc(100% - 35px);
  display: grid;
  grid-template-columns: 1fr;
  grid-template-rows: repeat(6, minmax(0, 1fr));
  gap: 4px;
  padding: 2px 11px 10px;
}

.servo-control {
  min-height: 0;
  display: grid;
  grid-template-columns: 58px 1fr 44px;
  align-items: center;
  gap: 6px;
  padding: 3px 8px;
  border-radius: 14px;

  background:
    linear-gradient(145deg, var(--inner-top), var(--inner-bottom));

  border: 1px solid rgba(255, 255, 255, 0.07);

  box-shadow:
    inset 0 1px rgba(255, 255, 255, 0.07),
    inset 0 -1px rgba(0, 0, 0, 0.65),
    0 5px 9px rgba(0, 0, 0, 0.2);
}

.servo-name {
  font-size: 13px;
  font-weight: 900;
}

.servo-name small {
  display: block;
  margin-top: 1px;
  color: var(--muted);
  font-size: 7px;
  font-weight: 600;
  white-space: nowrap;
}

.servo-angle {
  text-align: right;
  font-size: 12px;
  font-weight: 900;
}

input[type="range"] {
  width: 100%;
  height: 34px;
  margin: 0;
  appearance: none;
  -webkit-appearance: none;
  background: transparent;
  touch-action: none;
}

input[type="range"]::-webkit-slider-runnable-track {
  height: 8px;
  border-radius: 10px;

  background:
    linear-gradient(
      90deg,
      var(--blue) var(--fill, 0%),
      var(--track) var(--fill, 0%)
    );

  box-shadow:
    inset 0 2px 4px rgba(0, 0, 0, 0.58),
    0 1px rgba(255, 255, 255, 0.08);
}

input[type="range"].reverse-slider {
  direction: rtl;
}

input[type="range"]::-webkit-slider-thumb {
  width: 24px;
  height: 24px;
  margin-top: -8px;
  border-radius: 50%;
  appearance: none;
  -webkit-appearance: none;

  background:
    radial-gradient(
      circle at 34% 25%,
      white,
      #d9e8f5 34%,
      #8fa8bd 72%,
      #596e81
    );

  border: 2px solid var(--blue);

  box-shadow:
    inset 2px 2px 3px rgba(255, 255, 255, 0.8),
    inset -3px -3px 4px rgba(0, 0, 0, 0.22),
    0 5px 8px rgba(0, 0, 0, 0.52),
    0 0 8px rgba(60, 188, 255, 0.38);
}

@media (orientation: landscape) {
  .app {
    max-width: none;
    grid-template-columns: 44% 56%;
    grid-template-rows: 100%;
  }

  .joystick {
    width: min(29vw, 255px);
    height: min(29vw, 255px);
  }
}

</style>

</head>

<body>

<main class="app">

  <section class="panel">

    <div class="panel-title">
      <span class="status-dot"></span>
      UGV Control
    </div>

    <div class="drive-layout">

      <div class="joystick-container">

        <div id="joystick" class="joystick">
          <div id="stick" class="stick"></div>
        </div>

      </div>

      <div class="drive-side">

        <div class="control-row">

          <div class="mini-card">
            <div class="mini-label">Steer</div>
            <div id="xDisplay" class="mini-value">0</div>
          </div>

          <div class="mini-card">
            <div class="mini-label">Drive</div>
            <div id="yDisplay" class="mini-value">0</div>
          </div>

        </div>

        <div id="tiltCard" class="mini-card tilt-card">

          <div class="mini-label">Roll / Pitch</div>

          <div class="mini-value">
            <span id="rollDisplay">0.0</span>°
            /
            <span id="pitchDisplay">0.0</span>°
          </div>

          <div
            id="tiltStatus"
            class="mini-label"
            style="margin-top:3px"
          >
            SAFE
          </div>

        </div>

        <div class="control-row">

          <button
            id="gyroButton"
            class="glossy-button active"
            onclick="toggleGyro()"
          >
            TILT ON
          </button>

          <button
            id="lightButton"
            class="glossy-button"
            onclick="toggleLight()"
          >
            LIGHT OFF
          </button>

        </div>

        <div class="control-row">

          <button
            id="buzzerButton"
            class="glossy-button warning-active"
          >
            HOLD HORN
          </button>

          <button
            class="glossy-button stop-button"
            onclick="emergencyStop()"
          >
            STOP
          </button>

        </div>

      </div>

    </div>

  </section>

  <section class="panel">

    <div class="panel-title">
      Six-Axis Robotic Arm
    </div>

    <div class="arm-grid">

      <div class="servo-control">
        <div class="servo-name">
          A
          <small>Gripper</small>
        </div>

        <input
          type="range"
          min="0"
          max="180"
          value="0"
          data-servo="A"
        >

        <div id="valueA" class="servo-angle">0°</div>
      </div>

      <div class="servo-control">
        <div class="servo-name">
          B
          <small>Roll</small>
        </div>

        <input
          type="range"
          min="0"
          max="180"
          value="0"
          data-servo="B"
        >

        <div id="valueB" class="servo-angle">0°</div>
      </div>

      <div class="servo-control">
        <div class="servo-name">
          C
          <small>Pitch</small>
        </div>

        <input
          type="range"
          min="0"
          max="180"
          value="8"
          data-servo="C"
        >

        <div id="valueC" class="servo-angle">8°</div>
      </div>

      <div class="servo-control">
        <div class="servo-name">
          D
          <small>Elbow</small>
        </div>

        <input
          type="range"
          min="0"
          max="180"
          value="8"
          data-servo="D"
        >

        <div id="valueD" class="servo-angle">8°</div>
      </div>

      <div class="servo-control">
        <div class="servo-name">
          E
          <small>Shoulder</small>
        </div>

        <input
          type="range"
          min="0"
          max="180"
          value="67"
          data-servo="E"
        >

        <div id="valueE" class="servo-angle">67°</div>
      </div>

      <div class="servo-control">
        <div class="servo-name">
          F
          <small>Base</small>
        </div>

        <input
          type="range"
          min="0"
          max="180"
          value="105"
          data-servo="F"
          class="reverse-slider"
        >

        <div id="valueF" class="servo-angle">105°</div>
      </div>

    </div>

  </section>

</main>

<script>

const joystick = document.getElementById("joystick");
const stick = document.getElementById("stick");

const xDisplay = document.getElementById("xDisplay");
const yDisplay = document.getElementById("yDisplay");

let joystickActive = false;
let activePointerID = null;

let joystickX = 0;
let joystickY = 0;

let lastDriveSend = 0;
const driveSendInterval = 45;

function sendDrive(x, y, force = false) {

  const now = Date.now();

  if (
    !force &&
    now - lastDriveSend < driveSendInterval
  ) {
    return;
  }

  lastDriveSend = now;

  fetch(
    `/drive?x=${x}&y=${y}`,
    {
      cache: "no-store"
    }
  ).catch(() => {});
}

function moveJoystick(clientX, clientY) {

  const rect = joystick.getBoundingClientRect();

  const centreX = rect.left + rect.width / 2;
  const centreY = rect.top + rect.height / 2;

  const maximumRadius = rect.width * 0.32;

  let dx = clientX - centreX;
  let dy = clientY - centreY;

  const distance = Math.sqrt(dx * dx + dy * dy);

  if (distance > maximumRadius) {
    dx = dx * maximumRadius / distance;
    dy = dy * maximumRadius / distance;
  }

  stick.style.transform =
    `translate(${dx}px, ${dy}px)`;

  joystickX = Math.round(dx / maximumRadius * 100);
  joystickY = Math.round(-dy / maximumRadius * 100);

  if (Math.abs(joystickX) < 5) joystickX = 0;
  if (Math.abs(joystickY) < 5) joystickY = 0;

  xDisplay.textContent = joystickX;
  yDisplay.textContent = joystickY;

  sendDrive(joystickX, joystickY);
}

function resetJoystick() {

  joystickActive = false;
  activePointerID = null;

  joystickX = 0;
  joystickY = 0;

  stick.style.transform = "translate(0px, 0px)";

  xDisplay.textContent = "0";
  yDisplay.textContent = "0";

  sendDrive(0, 0, true);
}

joystick.addEventListener("pointerdown", event => {

  event.preventDefault();

  joystickActive = true;
  activePointerID = event.pointerId;

  joystick.setPointerCapture(event.pointerId);

  moveJoystick(event.clientX, event.clientY);
});

joystick.addEventListener("pointermove", event => {

  if (
    !joystickActive ||
    event.pointerId !== activePointerID
  ) {
    return;
  }

  event.preventDefault();

  moveJoystick(event.clientX, event.clientY);
});

joystick.addEventListener("pointerup", event => {

  if (event.pointerId === activePointerID) {
    resetJoystick();
  }
});

joystick.addEventListener(
  "pointercancel",
  resetJoystick
);

joystick.addEventListener(
  "lostpointercapture",
  resetJoystick
);

setInterval(() => {

  if (joystickActive) {
    sendDrive(joystickX, joystickY, true);
  }

}, 180);

function emergencyStop() {

  resetJoystick();

  fetch(
    "/stop",
    {
      cache: "no-store"
    }
  ).catch(() => {});
}

let gyroEnabled = true;

function toggleGyro() {

  gyroEnabled = !gyroEnabled;

  fetch(
    `/tilt-enable?state=${gyroEnabled ? 1 : 0}`,
    {
      cache: "no-store"
    }
  ).catch(() => {});

  updateGyroButton();
}

function updateGyroButton() {

  const button =
    document.getElementById("gyroButton");

  button.textContent =
    gyroEnabled
      ? "TILT ON"
      : "TILT OFF";

  button.classList.toggle(
    "active",
    gyroEnabled
  );
}

let headlightEnabled = false;

function toggleLight() {

  headlightEnabled = !headlightEnabled;

  fetch(
    `/light?state=${headlightEnabled ? 1 : 0}`,
    {
      cache: "no-store"
    }
  ).catch(() => {});

  updateLightButton();
}

function updateLightButton() {

  const button =
    document.getElementById("lightButton");

  button.textContent =
    headlightEnabled
      ? "LIGHT ON"
      : "LIGHT OFF";

  button.classList.toggle(
    "active",
    headlightEnabled
  );
}

const buzzerButton =
  document.getElementById("buzzerButton");

let buzzerHeld = false;
let buzzerRefreshTimer = null;

function buzzerStart(event) {

  if (event) {
    event.preventDefault();
  }

  if (buzzerHeld) {
    return;
  }

  buzzerHeld = true;

  buzzerButton.classList.add("pressed");

  fetch(
    "/buzzer?state=1",
    {
      cache: "no-store"
    }
  ).catch(() => {});

  buzzerRefreshTimer =
    setInterval(() => {

      fetch(
        "/buzzer?state=1",
        {
          cache: "no-store"
        }
      ).catch(() => {});

    }, 300);
}

function buzzerStop(event) {

  if (event) {
    event.preventDefault();
  }

  buzzerHeld = false;

  buzzerButton.classList.remove("pressed");

  clearInterval(buzzerRefreshTimer);
  buzzerRefreshTimer = null;

  fetch(
    "/buzzer?state=0",
    {
      cache: "no-store"
    }
  ).catch(() => {});
}

buzzerButton.addEventListener(
  "pointerdown",
  buzzerStart
);

buzzerButton.addEventListener(
  "pointerup",
  buzzerStop
);

buzzerButton.addEventListener(
  "pointercancel",
  buzzerStop
);

buzzerButton.addEventListener(
  "pointerleave",
  event => {

    if (buzzerHeld) {
      buzzerStop(event);
    }
  }
);

const servoTimers = {};

document
  .querySelectorAll('input[type="range"]')
  .forEach(slider => {

    updateSliderFill(slider);

    slider.addEventListener("input", () => {

      const servo = slider.dataset.servo;
      const angle = Number(slider.value);

      document.getElementById(
        `value${servo}`
      ).textContent = `${angle}°`;

      updateSliderFill(slider);

      clearTimeout(servoTimers[servo]);

      servoTimers[servo] =
        setTimeout(() => {

          sendServo(servo, angle);

        }, 25);
    });

    slider.addEventListener("change", () => {

      sendServo(
        slider.dataset.servo,
        Number(slider.value)
      );
    });
  });

function sendServo(servo, angle) {

  fetch(
    `/servo?id=${servo}&angle=${angle}`,
    {
      cache: "no-store"
    }
  ).catch(() => {});
}

function updateSliderFill(slider) {

  const min = Number(slider.min);
  const max = Number(slider.max);
  const value = Number(slider.value);

  const percentage =
    (value - min) / (max - min) * 100;

  slider.style.setProperty(
    "--fill",
    `${percentage}%`
  );
}

function requestStatus() {

  fetch(
    "/status",
    {
      cache: "no-store"
    }
  )
  .then(response => response.json())
  .then(data => {

    document.getElementById(
      "rollDisplay"
    ).textContent = Number(data.roll).toFixed(1);

    document.getElementById(
      "pitchDisplay"
    ).textContent = Number(data.pitch).toFixed(1);

    gyroEnabled = Boolean(data.tiltEnabled);
    headlightEnabled = Boolean(data.light);

    updateGyroButton();
    updateLightButton();

    const tiltCard =
      document.getElementById("tiltCard");

    const tiltStatus =
      document.getElementById("tiltStatus");

    tiltCard.classList.toggle(
      "danger",
      Boolean(data.danger)
    );

    if (!data.mpu) {
      tiltStatus.textContent = "MPU NOT FOUND";
    }
    else if (!gyroEnabled) {
      tiltStatus.textContent = "PROTECTION OFF";
    }
    else if (data.danger) {
      tiltStatus.textContent = "MOTORS LOCKED";
    }
    else {
      tiltStatus.textContent = "SAFE";
    }
  })
  .catch(() => {});
}

setInterval(requestStatus, 150);
requestStatus();

document.addEventListener(
  "visibilitychange",
  () => {

    if (document.hidden) {
      emergencyStop();
      buzzerStop();
    }
  }
);

window.addEventListener(
  "pagehide",
  () => {

    navigator.sendBeacon("/stop");
    navigator.sendBeacon("/buzzer?state=0");
  }
);

</script>

</body>
</html>
)rawliteral";

void writeLogicOutput(
  uint8_t pin,
  bool state,
  bool activeHigh
) {
  digitalWrite(
    pin,
    state == activeHigh ? HIGH : LOW
  );
}

void setBuzzer(bool state) {
  buzzerOn = state;

  writeLogicOutput(
    BUZZER_PIN,
    buzzerOn,
    BUZZER_ACTIVE_HIGH
  );

  if (state) {
    lastBuzzerRefreshTime = millis();
  }
}

void setHeadlight(bool state) {
  headlightOn = state;

  writeLogicOutput(
    HEADLIGHT_PIN,
    headlightOn,
    HEADLIGHT_ACTIVE_HIGH
  );
}

void writeMotorSide(
  int command,
  uint8_t rpwmChannel,
  uint8_t lpwmChannel,
  bool reverseDirection
) {
  command = constrain(command, -MOTOR_SPEED_LIMIT, MOTOR_SPEED_LIMIT);

  if (reverseDirection) {
    command = -command;
  }

  if (command > 0) {
    ledcWriteChannel(rpwmChannel, command);
    ledcWriteChannel(lpwmChannel, 0);
  }
  else if (command < 0) {
    ledcWriteChannel(rpwmChannel, 0);
    ledcWriteChannel(lpwmChannel, -command);
  }
  else {
    ledcWriteChannel(rpwmChannel, 0);
    ledcWriteChannel(lpwmChannel, 0);
  }
}

void driveMotors(
  int leftCommand,
  int rightCommand
) {
  if (
    tiltProtectionEnabled &&
    dangerousTilt
  ) {
    leftCommand = 0;
    rightCommand = 0;
  }

  writeMotorSide(
    leftCommand,
    LEFT_RPWM_CHANNEL,
    LEFT_LPWM_CHANNEL,
    REVERSE_LEFT_SIDE
  );

  writeMotorSide(
    rightCommand,
    RIGHT_RPWM_CHANNEL,
    RIGHT_LPWM_CHANNEL,
    REVERSE_RIGHT_SIDE
  );
}

void stopMotors() {
  writeMotorSide(
    0,
    LEFT_RPWM_CHANNEL,
    LEFT_LPWM_CHANNEL,
    REVERSE_LEFT_SIDE
  );

  writeMotorSide(
    0,
    RIGHT_RPWM_CHANNEL,
    RIGHT_LPWM_CHANNEL,
    REVERSE_RIGHT_SIDE
  );
}

int applyJoystickDeadZone(int value) {
  if (abs(value) <= JOYSTICK_DEADZONE) {
    return 0;
  }

  return value;
}

int percentageToPWM(int percentage) {
  return map(
    constrain(abs(percentage), 0, 100),
    0,
    100,
    0,
    MOTOR_SPEED_LIMIT
  );
}

void writeDirectServo(
  uint8_t servoChannel,
  int angle
) {
  angle = constrain(angle, 0, 180);

  uint32_t pulseWidthUs = map(
    angle,
    0,
    180,
    DIRECT_SERVO_MIN_US,
    DIRECT_SERVO_MAX_US
  );

  uint32_t duty =
    ((uint64_t)pulseWidthUs * 65535ULL) /
    20000ULL;

  ledcWriteChannel(servoChannel, duty);
}

void writePCA9685Servo(
  uint8_t channel,
  int angle
) {
  angle = constrain(angle, 0, 180);

  uint16_t pulse = map(
    angle,
    0,
    180,
    PCA_SERVO_MIN,
    PCA_SERVO_MAX
  );

  pca9685.setPWM(
    channel,
    0,
    pulse
  );
}

void setArmServo(
  char servoID,
  int angle
) {
  angle = constrain(angle, 0, 180);

  switch (servoID) {

    case 'A':
      armAngleA = angle;
      writeDirectServo(SERVO_A_CHANNEL, angle);
      break;

    case 'B':
      armAngleB = angle;
      writeDirectServo(SERVO_B_CHANNEL, angle);
      break;

    case 'C':
      armAngleC = angle;
      writePCA9685Servo(
        SERVO_C_CHANNEL,
        angle
      );
      break;

    case 'D':
      armAngleD = angle;
      writePCA9685Servo(
        SERVO_D_CHANNEL,
        angle
      );
      break;

    case 'E':
      armAngleE = angle;
      writePCA9685Servo(
        SERVO_E_CHANNEL,
        angle
      );
      break;

    case 'F':
      armAngleF = angle;
      writePCA9685Servo(
        SERVO_F_CHANNEL,
        angle
      );
      break;
  }
}

void calculateRawTilt(
  float ax,
  float ay,
  float az,
  float& rawRoll,
  float& rawPitch
) {
  rawRoll =
    atan2f(
      ay,
      sqrtf(ax * ax + az * az)
    ) * 180.0f / PI;

  rawPitch =
    atan2f(
      -ax,
      sqrtf(ay * ay + az * az)
    ) * 180.0f / PI;
}

void calibrateMPU() {
  if (!mpuDetected) {
    return;
  }

  Serial.println(
    "Keep the UGV flat and still."
  );

  Serial.println(
    "Calibrating MPU6050..."
  );

  const int samples = 120;

  float rollTotal = 0.0f;
  float pitchTotal = 0.0f;

  for (int i = 0; i < samples; i++) {

    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu6050.getEvent(
      &acceleration,
      &gyro,
      &temperature
    );

    float rawRoll;
    float rawPitch;

    calculateRawTilt(
      acceleration.acceleration.x,
      acceleration.acceleration.y,
      acceleration.acceleration.z,
      rawRoll,
      rawPitch
    );

    rollTotal += rawRoll;
    pitchTotal += rawPitch;

    delay(10);
  }

  rollOffset = rollTotal / samples;
  pitchOffset = pitchTotal / samples;

  filteredRoll = 0.0f;
  filteredPitch = 0.0f;

  Serial.print("Roll offset: ");
  Serial.println(rollOffset, 2);

  Serial.print("Pitch offset: ");
  Serial.println(pitchOffset, 2);

  Serial.println(
    "MPU6050 calibration complete."
  );
}

void updateMPU6050() {
  if (!mpuDetected) {
    return;
  }

  unsigned long now = millis();

  if (
    now - lastMPUReadTime <
    MPU_INTERVAL_MS
  ) {
    return;
  }

  lastMPUReadTime = now;

  sensors_event_t acceleration;
  sensors_event_t gyro;
  sensors_event_t temperature;

  mpu6050.getEvent(
    &acceleration,
    &gyro,
    &temperature
  );

  float rawRoll;
  float rawPitch;

  calculateRawTilt(
    acceleration.acceleration.x,
    acceleration.acceleration.y,
    acceleration.acceleration.z,
    rawRoll,
    rawPitch
  );

  rawRoll -= rollOffset;
  rawPitch -= pitchOffset;

  const float filterAlpha = 0.22f;

  filteredRoll =
    filteredRoll +
    filterAlpha * (rawRoll - filteredRoll);

  filteredPitch =
    filteredPitch +
    filterAlpha * (rawPitch - filteredPitch);

  rollAngle = filteredRoll;
  pitchAngle = filteredPitch;

  if (!tiltProtectionEnabled) {
    dangerousTilt = false;
    return;
  }

  bool thresholdExceeded =
    abs(rollAngle) >= TILT_STOP_ANGLE ||
    abs(pitchAngle) >= TILT_STOP_ANGLE;

  bool safelyRecovered =
    abs(rollAngle) <= TILT_RESET_ANGLE &&
    abs(pitchAngle) <= TILT_RESET_ANGLE;

  if (thresholdExceeded) {

    if (!dangerousTilt) {
      Serial.println(
        "DANGEROUS TILT: MOTORS STOPPED"
      );
    }

    dangerousTilt = true;
    stopMotors();
  }
  else if (
    dangerousTilt &&
    safelyRecovered
  ) {
    dangerousTilt = false;

    Serial.println(
      "Tilt returned to safe range."
    );
  }
}

void handleMainPage() {
  server.send_P(
    200,
    "text/html",
    WEBPAGE
  );
}

void handleDrive() {
  if (
    !server.hasArg("x") ||
    !server.hasArg("y")
  ) {
    stopMotors();

    server.send(
      400,
      "text/plain",
      "Missing joystick values"
    );

    return;
  }

  if (
    tiltProtectionEnabled &&
    dangerousTilt
  ) {
    stopMotors();

    server.send(
      423,
      "text/plain",
      "Drive locked by tilt protection"
    );

    return;
  }

  int steering = constrain(
    server.arg("x").toInt(),
    -100,
    100
  );

  int throttle = constrain(
    server.arg("y").toInt(),
    -100,
    100
  );

  steering =
    applyJoystickDeadZone(steering);

  throttle =
    applyJoystickDeadZone(throttle);

  int leftPercentage =
    throttle + steering;

  int rightPercentage =
    throttle - steering;

  int largestMagnitude = max(
    abs(leftPercentage),
    abs(rightPercentage)
  );

  if (largestMagnitude > 100) {

    leftPercentage =
      leftPercentage * 100 /
      largestMagnitude;

    rightPercentage =
      rightPercentage * 100 /
      largestMagnitude;
  }

  int leftPWM = 0;
  int rightPWM = 0;

  if (leftPercentage != 0) {
    leftPWM =
      percentageToPWM(leftPercentage);

    if (leftPercentage < 0) {
      leftPWM = -leftPWM;
    }
  }

  if (rightPercentage != 0) {
    rightPWM =
      percentageToPWM(rightPercentage);

    if (rightPercentage < 0) {
      rightPWM = -rightPWM;
    }
  }

  driveMotors(
    leftPWM,
    rightPWM
  );

  lastMotorCommandTime = millis();

  server.send(
    200,
    "text/plain",
    "OK"
  );
}

void handleServo() {
  if (
    !server.hasArg("id") ||
    !server.hasArg("angle")
  ) {
    server.send(
      400,
      "text/plain",
      "Missing servo data"
    );

    return;
  }

  String idArgument = server.arg("id");

  if (idArgument.length() != 1) {
    server.send(
      400,
      "text/plain",
      "Invalid servo ID"
    );

    return;
  }

  char servoID =
    toupper(idArgument.charAt(0));

  if (
    servoID < 'A' ||
    servoID > 'F'
  ) {
    server.send(
      400,
      "text/plain",
      "Servo must be A-F"
    );

    return;
  }

  int angle = constrain(
    server.arg("angle").toInt(),
    0,
    180
  );

  setArmServo(
    servoID,
    angle
  );

  server.send(
    200,
    "text/plain",
    "OK"
  );
}

void handleTiltEnable() {
  if (!server.hasArg("state")) {
    server.send(
      400,
      "text/plain",
      "Missing state"
    );

    return;
  }

  tiltProtectionEnabled =
    server.arg("state").toInt() == 1;

  if (!tiltProtectionEnabled) {
    dangerousTilt = false;
  }
  else {
    bool currentlyUnsafe =
      abs(rollAngle) >= TILT_STOP_ANGLE ||
      abs(pitchAngle) >= TILT_STOP_ANGLE;

    if (currentlyUnsafe) {
      dangerousTilt = true;
      stopMotors();
    }
  }

  server.send(
    200,
    "text/plain",
    tiltProtectionEnabled
      ? "TILT PROTECTION ON"
      : "TILT PROTECTION OFF"
  );
}

void handleLight() {
  if (!server.hasArg("state")) {
    server.send(
      400,
      "text/plain",
      "Missing state"
    );

    return;
  }

  setHeadlight(
    server.arg("state").toInt() == 1
  );

  server.send(
    200,
    "text/plain",
    headlightOn
      ? "LIGHT ON"
      : "LIGHT OFF"
  );
}

void handleBuzzer() {
  if (!server.hasArg("state")) {
    setBuzzer(false);

    server.send(
      400,
      "text/plain",
      "Missing state"
    );

    return;
  }

  bool requestedState =
    server.arg("state").toInt() == 1;

  setBuzzer(requestedState);

  server.send(
    200,
    "text/plain",
    buzzerOn
      ? "BUZZER ON"
      : "BUZZER OFF"
  );
}

void handleStop() {
  stopMotors();
  setBuzzer(false);

  server.send(
    200,
    "text/plain",
    "STOPPED"
  );
}

void handleStatus() {
  String json = "{";

  json += "\"mpu\":";
  json += mpuDetected ? "true" : "false";

  json += ",\"roll\":";
  json += String(rollAngle, 1);

  json += ",\"pitch\":";
  json += String(pitchAngle, 1);

  json += ",\"tiltEnabled\":";
  json += tiltProtectionEnabled
    ? "true"
    : "false";

  json += ",\"danger\":";
  json += dangerousTilt
    ? "true"
    : "false";

  json += ",\"light\":";
  json += headlightOn
    ? "true"
    : "false";

  json += ",\"buzzer\":";
  json += buzzerOn
    ? "true"
    : "false";

  json += ",\"A\":";
  json += armAngleA;

  json += ",\"B\":";
  json += armAngleB;

  json += ",\"C\":";
  json += armAngleC;

  json += ",\"D\":";
  json += armAngleD;

  json += ",\"E\":";
  json += armAngleE;

  json += ",\"F\":";
  json += armAngleF;

  json += "}";

  server.send(
    200,
    "application/json",
    json
  );
}

void handleNotFound() {
  server.send(
    404,
    "text/plain",
    "Page not found"
  );
}

void setup() {
  Serial.begin(115200);

  delay(300);

  Serial.println();
  Serial.println(
    "===================================="
  );
  Serial.println(
    "Mobile Robotic Manipulator"
  );
  Serial.println(
    "===================================="
  );

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(HEADLIGHT_PIN, OUTPUT);

  setBuzzer(false);
  setHeadlight(false);

  bool motor1 = ledcAttachChannel(
    LEFT_RPWM_PIN,
    MOTOR_PWM_FREQUENCY,
    MOTOR_PWM_RESOLUTION,
    LEFT_RPWM_CHANNEL
  );

  bool motor2 = ledcAttachChannel(
    LEFT_LPWM_PIN,
    MOTOR_PWM_FREQUENCY,
    MOTOR_PWM_RESOLUTION,
    LEFT_LPWM_CHANNEL
  );

  bool motor3 = ledcAttachChannel(
    RIGHT_RPWM_PIN,
    MOTOR_PWM_FREQUENCY,
    MOTOR_PWM_RESOLUTION,
    RIGHT_RPWM_CHANNEL
  );

  bool motor4 = ledcAttachChannel(
    RIGHT_LPWM_PIN,
    MOTOR_PWM_FREQUENCY,
    MOTOR_PWM_RESOLUTION,
    RIGHT_LPWM_CHANNEL
  );

  if (
    !motor1 ||
    !motor2 ||
    !motor3 ||
    !motor4
  ) {
    Serial.println(
      "Motor PWM initialization failed."
    );
  }

  stopMotors();

  bool directServoA = ledcAttachChannel(
    SERVO_A_PIN,
    SERVO_PWM_FREQUENCY,
    SERVO_PWM_RESOLUTION,
    SERVO_A_CHANNEL
  );

  bool directServoB = ledcAttachChannel(
    SERVO_B_PIN,
    SERVO_PWM_FREQUENCY,
    SERVO_PWM_RESOLUTION,
    SERVO_B_CHANNEL
  );

  if (
    !directServoA ||
    !directServoB
  ) {
    Serial.println(
      "Direct servo PWM initialization failed."
    );
  }

  Wire.begin(
    PCA_SDA_PIN,
    PCA_SCL_PIN,
    400000
  );

  pca9685.begin();
  pca9685.setOscillatorFrequency(27000000);
  pca9685.setPWMFreq(50);

  delay(10);

  MPU_WIRE.begin(
    MPU_SDA_PIN,
    MPU_SCL_PIN,
    400000
  );

  mpuDetected = mpu6050.begin(
    0x68,
    &MPU_WIRE
  );

  if (mpuDetected) {

    mpu6050.setAccelerometerRange(
      MPU6050_RANGE_8_G
    );

    mpu6050.setGyroRange(
      MPU6050_RANGE_500_DEG
    );

    mpu6050.setFilterBandwidth(
      MPU6050_BAND_21_HZ
    );

    Serial.println(
      "MPU6050 detected."
    );

    calibrateMPU();
  }
  else {
    tiltProtectionEnabled = false;

    Serial.println(
      "MPU6050 not detected."
    );

    Serial.println(
      "Tilt protection disabled."
    );
  }

  setArmServo('A', 0);
  setArmServo('B', 0);
  setArmServo('C', 8);
  setArmServo('D', 8);
  setArmServo('E', 67);
  setArmServo('F', 105);

  Serial.println(
    "Servo positions: "
    "A=0 B=0 C=8 D=8 E=67 F=105"
  );

  WiFi.mode(WIFI_AP);

  bool accessPointStarted = WiFi.softAP(
    WIFI_NAME,
    WIFI_PASSWORD
  );

  if (accessPointStarted) {
    Serial.println(
      "Wi-Fi access point started."
    );
  }
  else {
    Serial.println(
      "Wi-Fi access point failed."
    );
  }

  Serial.print("Wi-Fi: ");
  Serial.println(WIFI_NAME);

  Serial.print("Password: ");
  Serial.println(WIFI_PASSWORD);

  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());

  server.on(
    "/",
    HTTP_GET,
    handleMainPage
  );

  server.on(
    "/drive",
    HTTP_GET,
    handleDrive
  );

  server.on(
    "/servo",
    HTTP_GET,
    handleServo
  );

  server.on(
    "/tilt-enable",
    HTTP_GET,
    handleTiltEnable
  );

  server.on(
    "/light",
    HTTP_GET,
    handleLight
  );

  server.on(
    "/buzzer",
    HTTP_ANY,
    handleBuzzer
  );

  server.on(
    "/stop",
    HTTP_ANY,
    handleStop
  );

  server.on(
    "/status",
    HTTP_GET,
    handleStatus
  );

  server.onNotFound(handleNotFound);

  server.begin();

  lastMotorCommandTime = millis();

  Serial.println(
    "Web server started."
  );

  Serial.println(
    "System ready."
  );
}

void loop() {
  server.handleClient();

  updateMPU6050();

  unsigned long now = millis();

  if (
    now - lastMotorCommandTime >
    MOTOR_COMMAND_TIMEOUT_MS
  ) {
    stopMotors();
  }

  if (
    buzzerOn &&
    now - lastBuzzerRefreshTime >
    BUZZER_TIMEOUT_MS
  ) {
    setBuzzer(false);
  }
}