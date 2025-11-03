# Flight Controller

A professional-grade flight controller for drones based on STM32F103 or ESP32, featuring sensor integration, low-latency communication, and real-time control systems. Built with PlatformIO for robust and efficient UAV operations.

## Features

**Core Systems**
- **MPU6050 Integration**: Gyroscope and accelerometer for precise motion tracking
- **ExpressLRS Communication**: Low-latency CRSF protocol for reliable radio control
- **Modular Architecture**: Clean, maintainable codebase with separated concerns
- **Multi-Platform Support**: Compatible with STM32 and ESP32 architectures

**Flight Control**
- Real-time sensor data processing
- PWM signal generation for flight surfaces
- Configurable control mixing
- Timing-optimized loop execution

## Hardware Requirements

- **Main Controller**: STM32F103C8 (Blue Pill) or ESP32
- **Sensors**: MPU6050 (I²C)
- **Receiver**: ExpressLRS-compatible module
- **Power**: 3.3V regulated supply

## Pin Configuration

| Function | STM32 | ESP32 |
|----------|-------|-------|
| Status LED | PC13 | GPIO2 |
| I²C SDA | PB7 | GPIO21 |
| I²C SCL | PB6 | GPIO22 |
| ELRS RX | PB11 | Configurable |
| ELRS TX | PB10 | Configurable |

## Development Status

- [x] **Foundation**: STM32 compilation in PlatformIO
- [x] **Sensors**: MPU6050 integration and calibration
- [x] **Communication**: ExpressLRS CRSF protocol
- [ ] **Control Systems**: PID loop development
- [ ] **Motor Control**: ESC PWM/Dshot implementation

## Building

```bash
# Build for STM32
pio run -e bluepill_f103c8

# Build for ESP32
pio run -e esp32

# Upload to device
pio run -e bluepill_f103c8 -t upload
```

## Configuration

Modify `platformio.ini` for target-specific settings:
- Serial baud rates
- Pin assignments
- Timing intervals
- Communication protocols

## License

Professional use - Contact for commercial licensing.