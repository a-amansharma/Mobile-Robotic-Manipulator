# Mobile Robotic Manipulator

A **6-DOF Mobile Robotic Manipulator** that combines a six-wheel unmanned ground vehicle (UGV) with a six-degree-of-freedom robotic arm for remote operation, object manipulation, and embedded robotics research.

This project has been **designed, engineered, programmed, assembled, tested, and documented entirely by me**. From the mechanical design and electronics integration to the embedded firmware, communication protocols, control interface, and documentation, every component has been developed through my own ideas, experimentation, and engineering approach.

> **Designed & Developed by Aman Sharma**

---

## Project Overview

The Mobile Robotic Manipulator is built to serve as a modular robotics platform capable of:

- 6-Wheel Differential Drive Rover
- 6-DOF Robotic Manipulator
- ESP32-based Wireless Communication
- Real-Time Control
- Sensor Integration
- Embedded Motion Control
- Robotics Research & Development

The project is being developed in multiple stages, with each subsystem tested individually before full system integration.

---

## Current Development

The repository currently contains:

- Wi-Fi Control Firmware
- ESP-NOW Communication Firmware
- MPU6050 Orientation Test
- Processing 3D Orientation Visualizer
- Hardware Documentation
- Wiring Documentation
- Build Guide
- Bill of Materials
- Development Roadmap
- Troubleshooting Guide

Additional features, optimizations, and documentation will continue to be added as development progresses.

---

## Hardware Used

- ESP32 Development Boards
- NodeMCU-32S ESP32
- PCA9685 Servo Driver
- MPU6050 IMU
- HC-SR04 Ultrasonic Sensor
- SSD1306 OLED Display
- IBT-2 Motor Drivers
- DC Geared Motors
- MG996R & MG90S Servo Motors
- Lithium Battery Power System

---

## Repository Structure

```text
Mobile-Robotic-Manipulator/
│
├── assets/
│   └── images/
│       ├── angle-view.webp
│       ├── both1.webp
│       ├── close-arm.webp
│       ├── grip.webp
│       ├── main.webp
│       ├── manipulator.webp
│       ├── setup.webp
│       ├── side-view.webp
│       ├── soldering.webp
│       ├── ultrasonic.webp
│       └── wires.webp
│
├── docs/
│   ├── bill-of-materials.md
│   ├── build-guide.md
│   ├── development-roadmap.md
│   ├── hardware.md
│   ├── power-system.md
│   ├── troubleshooting.md
│   └── wiring.md
│
├── firmware/
│   ├── receiver-esp-now/
│   │   └── receiver.ino
│   │
│   ├── transmitter-esp-now/
│   │   └── transmitter.ino
│   │
│   ├── wifi-control/
│   │   ├── mobile-robotic-manipulator.ino
│   │   └── steps.md
│   │
│   └── tests/
│       └── mpu-orientation-test/
│           ├── mpu-orientation-test.ino
│           ├── processing-code.pde
│           └── steps.md
│
├── README.md
└── .gitignore
```

---

## Project Status

**Status:** Active Development

This repository is continuously updated with new firmware, hardware improvements, documentation, and testing results as the project evolves.

---

## Notice

This project is **NOT an open-source project**.

The complete hardware architecture, firmware, PCB/wiring design, documentation, mechanical integration, and overall system design are original work created solely by **Aman Sharma**.

Please do not copy, redistribute, or reproduce any part of this project without prior permission.

---

## Author

**Aman Sharma**

Embedded Systems • Robotics • Mechatronics • Computer Vision

---