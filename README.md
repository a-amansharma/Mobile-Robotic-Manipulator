# Mobile Robotic Manipulator

A modular robotics project combining a six-wheel unmanned ground vehicle with a six-degree-of-freedom robotic manipulator.

The project is currently under active development. The repository presently contains an MPU-based orientation test and a live Processing 3D visualizer. Complete motion control, robotic-arm control and system integration will be added progressively.

## Current Status

The following components are currently being tested:

- NodeMCU-32S ESP32 development board
- MPU-compatible inertial measurement unit
- Real-time roll, pitch and yaw calculation
- USB serial communication
- Processing-based 3D orientation visualization
- Gyroscope calibration and stationary-noise filtering

The full Mobile Robotic Manipulator firmware is not yet complete.

## Repository Structure

```text
Mobile-Robotic-Manipulator/
├── firmware/
│   ├── tests/
│   │   └── mpu-orientation-test/
│   │       └── mpu-orientation-test.ino
│   └── main/
│       └── mobile-robotic-manipulator.ino
├── visualization/
│   └── processing/
│       └── mpu-3d-visualizer/
│           └── mpu-3d-visualizer.pde
├── docs/
│   ├── wiring.md
│   ├── hardware.md
│   └── development-roadmap.md
├── assets/
│   └── images/
├── .gitignore
└── README.md