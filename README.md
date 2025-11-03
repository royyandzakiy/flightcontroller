# STM32 Flight Controller
This repository hosts an STM32F103-based flight controller for drones, featuring MPU6050, ExpressLRS, PID control, and motor mixing. Built with PlatformIO for lightweight, efficient UAV control.

## Key Features
- MPU6050: Gyroscope & accelerometer for precise motion tracking.
- ExpressLRS: Low-latency radio communication with Crossfire (CRSF).
- PID Loop: Custom control loop for stable flight.
- Motor Mixing: Generates motor signals from PID outputs.
- Bit Banging: Faster pin control for improved performance.
- Profiling: Optimizes software metrics for efficiency.

## Development Notes
- [x] compile STM32F103 project in Arduino IDE & PlatformIO
- [x] basic functions running: MPU6050, ELRS
- [ ] pinout diagram
- [ ] PID loop
- [ ] ESC PWM / dshot control
