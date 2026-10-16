# Flight Controller

A quadcopter flight controller built for learning. It's Arduino-friendly, written in plain C++11
style, and readable top to bottom. Hardware-specific code lives in one board file per MCU, with no `#ifdef`s.

`src/main.cpp` tells the story of one control loop. The details live in folders around it.
The full design and roadmap are in [plan.md](plan.md).

## Status

Builds for both targets, and the pure logic is unit-tested on the PC. **Not yet tested on real
hardware**: follow the bench checklist below before any props go on.

| Part | State |
|---|---|
| Board layer (ESP32-S3, STM32F103) | done, bench test pending |
| CRSF receiver input, link timeout, link stats | done, unit-tested |
| MPU6050 driver, gyro/accel calibration | done, bench test pending |
| Rate PID (acro), angle mode, quad-X mixer | done, unit-tested |
| Arming checks, staged failsafe, runaway detection | done, unit-tested |
| Event log, performance stats, CLI, parameters | done |
| SD-card blackbox + decode/plot tools | done, decoder verified against the encoder |
| CRSF telemetry (battery, attitude, mode) | done |
| DShot | not started (plan M9) |

## Hardware

| Part | Choice |
|---|---|
| MCU | ESP32-S3 DevKitC-1 (1 kHz loop) or STM32F103 Blue Pill, 128 KB variant (500 Hz loop) |
| IMU | MPU6050 over I2C |
| Receiver | ExpressLRS / CRSF |
| ESCs | OneShot125 |
| Blackbox | microSD module over SPI |
| Battery | resistor divider into an ADC pin (default 10k / 1k) |

### Pins

| Function | ESP32-S3 | STM32F103 |
|---|---|---|
| Motor 1-4 | GPIO 4, 5, 6, 7 | PA0, PA1, PA2, PA3 |
| I2C SDA / SCL | GPIO 8 / 9 | PB7 / PB6 |
| CRSF RX / TX | GPIO 18 / 17 | PB11 / PB10 |
| SD SCK / MISO / MOSI / CS | GPIO 12 / 13 / 11 / 10 | PA5 / PA6 / PA7 / PA4 |
| Battery ADC | GPIO 1 | PB0 |
| Status LED | GPIO 2 | PC13 |
| Debug / CLI serial | USB-UART (UART0) | PA9 / PA10 |

Motor order is Betaflight quad-X, props in: 1 rear-right, 2 front-right, 3 rear-left, 4 front-left.
Radio: AETR channel order, aux 1 (ch5) = arm, aux 2 (ch6) = mode (low acro, mid/high angle).

## Layout

```
src/main.cpp      the control loop story
src/config.h      compile-time settings
src/board/        hardware interface + one implementation per MCU
src/drivers/      IMU, receiver, battery, status LED
src/system/       blackbox, event log, perf, params, CLI, telemetry
lib/              pure logic without Arduino.h, unit-tested on the PC
test/             unit tests (pio test -e native)
tools/            blackbox decoder and plotter
```

## Building

```bash
pio run -e esp32s3
```

```bash
pio run -e bluepill
```

```bash
pio test -e native
```

```bash
pio run -e esp32s3 -t upload
```

## Using it

Open the serial monitor at 460800 baud and type `help`. The most useful commands:

- `status`: arm state, **why arming is blocked**, link quality, attitude, battery
- `perf`: time per loop stage, CPU load, overruns, SD write latency
- `dump`, `get`, `set`, `save`: parameters
- `motor 1 10 props-off`: spin one motor at 10% for 2 s (props off)

The LED shows the state: solid = armed, fast blink = failsafe, short flash = ready to arm,
N blinks = arming blocked by flag N (see `status`).

### Reading a blackbox log

```bash
python tools/bb_decode.py LOG00001.BBL
```

```bash
python tools/bb_plot.py LOG00001_loop.csv
```

## Bench checklist before the first flight

Props off for all of these.

1. `status` shows the IMU, a calibrated gyro and the RC link. Tilting the quad changes roll/pitch with the right sign (right side down = positive roll, nose up = positive pitch).
2. `perf` shows no overruns and CPU load well below 100%.
3. A scope on the motor pins shows 125 us pulses while disarmed.
4. `motor N 10 props-off` spins the motor at the expected corner for each N.
5. Armed at low throttle, tilting the quad by hand speeds up the motors on the low side.
6. Switching the transmitter off while armed: failsafe within 250 ms, motors stop 1 s later, and re-arming needs the switch cycled.
7. `debug hang` resets the board through the watchdog, and the next boot logs it.

## License

Professional use - Contact for commercial licensing.
