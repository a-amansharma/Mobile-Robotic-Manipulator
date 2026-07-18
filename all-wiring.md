# Wiring

| <h2 align="center">🟩 Transmitter Unit</h2> | <h2 align="center">🟦 Receiver Unit</h2> |
|:-------------------------------------------:|:----------------------------------------:|
| **ESP32 Controller** | **ESP32 Rover** |
| | |
| ### 🎮 Joystick | ### 🚗 IBT-2 Motor Drivers |
| • VRX → GPIO34 | • Left RPWM → GPIO25 |
| • VRY → GPIO35 | • Left LPWM → GPIO26 |
| • VCC → 5V | • Right RPWM → GPIO27 |
| • GND → GND | • Right LPWM → GPIO14 |
| | |
| ### 🎚 Speed Pot | ### 🤖 PCA9685 |
| • Signal → GPIO32 | • SDA → GPIO21 |
| • VCC → 3.3V | • SCL → GPIO22 |
| • GND → GND | • Servo Channels |
| | |
| ### 🔘 Buttons | • Gripper → GPIO32 |
| • Light → GPIO25 | • Wrist Roll → GPIO33 |
| • Buzzer → GPIO33 | • Wrist Pitch → CH4 |
| | • Elbow → CH6 |
| ### 📺 OLED | • Shoulder → CH8 |
| • SDA → GPIO21 | • Base → CH10 |
| • SCL → GPIO22 | |
| • VCC → 3.3V | ### 📡 MPU6050 |
| • GND → GND | • SDA → GPIO18 |
| | • SCL → GPIO19 |
| ### 🔄 STM32 UART | • VCC → 3.3V |
| • PA9 → GPIO16 | • GND → GND |
| | |
| ### 🦾 STM32 Master Arm | ### 📏 HC-SR04 |
| • PA0 → Gripper | • TRIG → GPIO4 |
| • PA1 → Wrist Roll | • ECHO → GPIO5 |
| • PA2 → Wrist Pitch | • Voltage Divider Required |
| • PA3 → Elbow | |
| • PA4 → Shoulder | ### 🔊 Other Components |
| • PA5 → Base | • Buzzer → GPIO13 |
| | • Headlight → GPIO23 |
| | • Onboard LED → GPIO2 |