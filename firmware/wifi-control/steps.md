# Mobile Robotic Manipulator

A wireless 6-DOF Mobile Robotic Manipulator featuring a six-wheel drive rover and a robotic arm. The robot is controlled through a custom web interface hosted by the ESP32 over Wi-Fi.

---

## Features

- 6-Wheel Drive Rover
- 6-DOF Robotic Arm
- Wi-Fi Web Controller
- Smooth Joystick Control
- Adjustable Speed
- Headlight Control
- Horn/Buzzer Control
- MPU6050 Tilt Protection
- OLED/Web Interface
- Emergency Stop

---

# Hardware

- ESP32 DevKit V1
- STM32 Blue Pill
- PCA9685 Servo Driver
- 6 DC Geared Motors
- 4 × MG996R Servos
- 2 × MG90S Servos
- 2 × BTS7960 (IBT-2)
- MPU6050
- HC-SR04 Ultrasonic Sensor
- SSD1306 OLED
- 2S Li-ion Battery

---

# Receiver Wiring

## Motor Drivers

| Function | GPIO |
|----------|------|
| Left RPWM | GPIO25 |
| Left LPWM | GPIO26 |
| Right RPWM | GPIO27 |
| Right LPWM | GPIO14 |

## Direct Servos

| Servo | GPIO |
|--------|------|
| Gripper | GPIO32 |
| Wrist Roll | GPIO33 |

## PCA9685

| Signal | GPIO |
|--------|------|
| SDA | GPIO21 |
| SCL | GPIO22 |

## MPU6050

| Signal | GPIO |
|--------|------|
| SDA | GPIO18 |
| SCL | GPIO19 |

---

# How to Use

### Step 1

Power ON the robot.

---

### Step 2

On your phone, open **Wi-Fi Settings**.

Connect to:

```
SSID : Mobile-Robotic-Manipulator
Password : 87654321
```

---

### Step 3

Open any web browser.

Go to:

```
http://192.168.4.1
```

---

### Step 4

The control panel will open automatically.

You can now control:

- Rover Movement
- Robotic Arm
- Speed
- Headlight
- Horn
- Emergency Stop
- Tilt Protection

---

# Safety

- Use a regulated power supply for all servos.
- Never power MG90S servos directly from a 2S or 3S battery.
- Connect all grounds together.
- Verify all wiring before powering the system.

---

# Project Structure

```
firmware/
    receiver/
    transmitter/

docs/
    wiring.md
    build-guide.md
    bill-of-materials.md
    power-system.md
    troubleshooting.md

assets/
    images/
    videos/
```

---

## Author

Designed and developed by **Aman Sharma**.