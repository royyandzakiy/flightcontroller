# Flight Controller Plan

A quadcopter flight controller built for **learning**: Arduino-friendly, readable top to bottom,
hardened enough to fly safely, and portable across MCUs without `#ifdef` soup.

Targets: **ESP32-S3** (`esp32-s3-devkitc-1`) and **STM32F103** (Blue Pill), both on the Arduino framework via PlatformIO.

---

## 1. Goals and non-goals

**Goals**
- Fly a quad in **Acro** (rate) and **Angle** (self-level) modes.
- `main.cpp` tells the whole story of one control loop; details live in folders.
- Same flight code on every MCU; hardware differences live in one board file per MCU.
- Safe by default: boots disarmed, explicit arming checks, staged failsafe, watchdog.
- Observable: SD-card blackbox, event log, per-stage timing and loop-health stats.
- Pure flight logic is unit-tested on the PC (`pio test -e native`).

**Non-goals (for now)**
- GPS, position hold, altitude hold, OSD, Betaflight Configurator compatibility.
- Squeezing out the last microsecond: if clear code is a bit slower, we keep it clear.
- DShot in the first version (it's planned as a later milestone, see M9).

---

## 2. Coding rules

Keep it **basic C++11**, the kind most embedded/Arduino people read comfortably.

| Rule | Why |
|---|---|
| No heap: no `new`, `malloc`, `String`, `std::vector`, `std::string`, STL containers | No fragmentation, no surprise allocation in flight |
| All buffers are fixed-size arrays, sized by `#define` or `static const` | Memory use is known at link time |
| No `constexpr`, no `std::optional`/`expected`, no heavy templates, no lambdas in flight code | Easy to follow for C/Arduino background |
| `enum class` and plain structs are fine | Basic C++11, makes states readable |
| No exceptions, no RTTI (`-fno-exceptions -fno-rtti`) | Smaller, predictable |
| No `#ifdef` for board differences | Build system picks the board file instead (see §4) |
| `float` only (no `double`), `f` suffix on literals | STM32F103 has no FPU; double is even slower |
| No `delay()` after `setup()`, no blocking reads in the loop | The loop must keep its rate |
| No `printf` with floats on hot paths; print scaled integers | Float printf is big and slow on STM32 |
| Modules that exist once = `namespace` with `init()` / `update()` | Matches current style, no object juggling |
| Things that exist several times (PID per axis, filters) = small class/struct | One instance per use, all static |
| Pure logic in `lib/` never includes `Arduino.h` | So it can be unit-tested on the PC |
| Warnings on: `-Wall -Wextra` for our code | Catch mistakes early |

**Heap guard:** `board::heapGuardArm()` at the end of `setup()`. After that any heap allocation
calls `board::fatal()`. Each board hooks it its own way: the ESP32 replaces the global
`operator new`. The STM32 core already defines `operator new`, so that board wraps `_sbrk` at link
time (`-Wl,--wrap=_sbrk`), which also catches C `malloc`. Free heap is reported by `status`.
`strtof` is avoided for the same reason (newlib allocates inside it), the CLI uses `fcParseFloat`.

**About the compiler standard:** our code is written in C++11 style, but we don't force
`-std=gnu++11`. The ESP32 Arduino core's own headers need a newer standard.

---

## 3. Hardware

| Part | Choice | Notes |
|---|---|---|
| MCU A | ESP32-S3 DevKitC-1 | FPU, plenty of RAM/flash, target loop 1 kHz |
| MCU B | STM32F103C8 Blue Pill | No FPU, 20 KB RAM, 64 KB flash, target loop 500 Hz |
| IMU | MPU6050 over I2C @ 400 kHz | Own small driver (learning value) |
| Receiver | ExpressLRS, CRSF @ 420000 baud | Own small parser |
| ESCs | OneShot125 first | DShot later |
| Blackbox | microSD module over SPI | SdFat library |
| Battery sense | Resistor divider into ADC | e.g. 10k/1k for up to 4S |
| Status | Onboard LED (+ optional buzzer) | Blink codes per state |

**Blue Pill risk:** SdFat + floats in software + everything else may not fit in 64 KB flash.
Plan: try first with `bluepill_f103c8_128k` (most C8 chips actually have 128 KB). If that's
still too tight, switch the STM32 target to a **Black Pill F411** (same Arduino core, has an FPU),
which means adding one board folder and one env and no other code changes.

### Proposed pin map (final check in M1)

| Function | ESP32-S3 | STM32F103 |
|---|---|---|
| Motor 1–4 (OneShot125) | GPIO 4, 5, 6, 7 | PA0, PA1, PA2, PA3 (TIM2 CH1–4) |
| I2C SDA / SCL | GPIO 8 / 9 | PB7 / PB6 |
| CRSF RX / TX | GPIO 18 / 17 (UART1) | PB11 / PB10 (USART3) |
| SD SPI SCK / MISO / MOSI / CS | GPIO 12 / 13 / 11 / 10 | PA5 / PA6 / PA7 / PA4 (SPI1) |
| Battery ADC | GPIO 1 | PB0 |
| Status LED | GPIO 2 (or onboard RGB on 48) | PC13 (active low) |
| Debug / CLI serial | USB | USART1 PA9 / PA10 |

---

## 4. Multi-MCU without `#ifdef`: build-selected board files

Every board implements the same `src/board/board.h`. PlatformIO compiles only the folder for
the current env:

```ini
[env]
framework = arduino
lib_deps = greiman/SdFat@^2.2.3
build_flags = -Wall -Wextra -fno-exceptions -fno-rtti

[env:esp32s3]
platform = espressif32
board = esp32-s3-devkitc-1
build_src_filter = +<*> -<board/> +<board/esp32s3/>

[env:bluepill]
platform = ststm32
board = bluepill_f103c8_128k
board_build.f_cpu = 72000000L
build_src_filter = +<*> -<board/> +<board/stm32f103/>

[env:native]
platform = native
test_framework = unity
```

`board.h` is the *only* hardware surface the rest of the code sees:

```cpp
namespace board {
    const Info& info();                          // name, loop rate, blackbox rate divider
    void     init(uint32_t debugBaud);
    uint32_t micros();
    uint32_t millis();

    void     motorsInit();                       // OneShot125 on 4 pins, stop pulse first
    void     motorsWrite(const float command[4]);// 0..1 per motor -> 125..250 us
    void     motorsStop();

    void     rcSerialBegin(uint32_t baud);
    Stream&  rcSerial();                         // CRSF UART
    Stream&  debugSerial();                      // CLI and text log
    TwoWire& imuWire();
    SPIClass& sdSpi();
    uint8_t  sdCsPin();

    float    batteryPinVolts();                  // divider undone by the battery driver

    void     ledWrite(bool on);
    void     watchdogInit(uint32_t timeoutMs);
    void     watchdogKick();
    bool     resetWasWatchdog();
    const char* resetReason();
    void     reboot();
    uint32_t freeHeap();
    void     heapGuardArm();
    uint8_t* blackboxBuffer(uint32_t& size);     // RAM budget decided per MCU
    bool     storageLoad(uint8_t* data, uint32_t size);
    bool     storageSave(const uint8_t* data, uint32_t size);
    void     fatal(const char* reason);          // motors off, print, blink SOS forever
}
```

Each `board.cpp` holds its pins as `static const` values and uses its own vendor APIs:
- ESP32-S3: `ledcAttach()` / `ledcWrite()` (arduino-esp32 v3 API), `esp_task_wdt`.
- STM32: `HardwareTimer` on TIM2 for the 4 motor channels, `IWatchdog`.

Adding a new MCU = new `src/board/<mcu>/board.cpp` + new env. Nothing else changes.

---

## 5. Folder structure

**Rule:** `src/` = code that touches hardware or the Arduino API. `lib/` = pure logic, PC-testable.

```
plan.md
platformio.ini
src/
  main.cpp                 the story: setup() + the control loop
  config.h                 compile-time constants: timeouts, channel map, IMU mounting, limits
  board/
    board.h                hardware interface (above)
    board_common.h         OneShot125 math and SOS pattern shared by the boards
    esp32s3/board.cpp      includes the operator new heap guard
    stm32f103/board.cpp    includes the _sbrk heap guard
  drivers/
    imu_mpu6050.h/.cpp     init, WHO_AM_I, DLPF, read -> deg/s and g, calibration
    rc_crsf.h/.cpp         drains UART into lib/crsf parser, link timeout, link stats
    battery.h/.cpp         ADC -> filtered volts, cell count, low/critical with hysteresis
    status_led.h/.cpp      blink patterns per state
  system/
    blackbox.h/.cpp        SD writer: ring buffer -> 512-byte block writes
    console.h/.cpp         non-blocking text output to the debug serial
    event_log.h/.cpp       event queue in the loop, formatted in a slow task
    perf.h/.cpp            per-stage timing, loop jitter, overruns, CPU load
    params.h/.cpp          tunable parameters, flash load/save with CRC
    cli.h/.cpp             serial commands (only while disarmed)
    telemetry.h/.cpp       CRSF telemetry back to the radio
lib/
  common/                  fc_math, crc, byte_io, byte_ring_buffer, loop_timer, text_format
  crsf/                    crsf_protocol, crsf_parser, crsf_telemetry
  flight/
    flight_types.h         ImuSample, RcInput, RateSetpoint, AxisTorque, MotorOutputs, Angles
    pt1_filter, pid, rate_controller
    rates                  sticks -> deg/s (max rate + expo)
    attitude               complementary filter -> roll/pitch angles
    angle_mode             angle error -> rate setpoint (outer P loop)
    mixer                  quad-X mixer table + desaturation
    runaway_detector
  modes/
    mode_manager           arming checks, flight modes, failsafe state machine
  blackbox_format/
    bb_record              record layout, encode/decode
test/
  test_common/  test_crsf/  test_flight/  test_modes/  test_blackbox/
tools/
  bb_decode.py             blackbox .BBL -> CSV
  bb_plot.py               quick plots: gyro vs setpoint, PID terms, motors, loop time
docs/                      datasheets (existing)
```

Removed from the current repo: `lib/defines.h`, `lib/drv_hrt.h`, `lib/queue.h`, `lib/time.h`,
`lib/common_rc/`, and the PX4 `lib/crsf/` (replaced by our own parser).

---

## 6. `main.cpp`: what it should read like

The main file is the explanation. It shows the order of work in one loop pass, and every line
calls into a folder. Simplified target shape:

```cpp
void setup() {
    board::init();
    logEvents::init();
    params::loadOrDefaults();

    imu::init();               // fails -> board::fatal("imu")
    rc::init();
    battery::init();
    blackbox::init();          // missing SD is a warning, not fatal
    board::motorsInit();       // outputs stop pulse from the first moment

    imu::calibrateGyro();      // quad must be still; retries until it is
    modes.init();
    board::watchdogInit(WATCHDOG_TIMEOUT_MS);
}

void loop() {
    if (!loopTimer.waitForNextTick()) return;   // fixed rate, counts overruns
    perf::loopStart();

    // 1. Inputs
    rc::update();                                // drain UART, track link age
    ImuSample imu = imu::read();                 // deg/s and g, filtered
    attitude.update(imu, dt);

    // 2. Decide what we are allowed to do
    modes.update(rc::input(), rc::linkOk(), imu::healthy(), battery::state(), now);

    // 3. Compute what we want
    RateSetpoint target = (modes.flightMode() == FlightMode::Angle)
                        ? angleMode.update(rc::input(), attitude.angles())
                        : rates::fromSticks(rc::input());
    if (modes.inFailsafe()) target = failsafeSetpoint(attitude.angles());

    Torque cmd = rateController.update(target, imu.gyro, dt, modes.isArmed());

    // 4. Act
    MotorOutputs out = mixer::mix(cmd, modes.throttle(rc::input()));
    if (!modes.isArmed()) out = MotorOutputs::stopped();
    board::motorsWrite(out.us);

    // 5. Record and housekeeping
    blackbox::logLoop(imu, target, rateController, out, modes);
    runSlowTasks();                              // 1 task per tick, round robin
    board::watchdogKick();
    perf::loopEnd();
}
```

`runSlowTasks()` spreads the slow work across ticks so no single tick gets the whole cost:

| Task | Rate | Work |
|---|---|---|
| blackbox flush | every tick if a 512 B block is ready | one SD block write |
| battery | 50 Hz | ADC read + filter |
| telemetry | 10 Hz | one CRSF telemetry frame |
| log flush | 20 Hz | print/write queued events |
| status LED | 20 Hz | pattern step |
| CLI | 20 Hz | read chars (only when disarmed) |
| perf report | 1 Hz | roll up the stats window |

---

## 7. Module design

### 7.1 IMU (`drivers/mpu6050`)
- On init: check WHO_AM_I (0x68, accept common clone values and log which one was found),
  wake the chip, gyro ±2000 °/s, accel ±8 g, DLPF ~188 Hz, sample rate 1 kHz.
- Read 14 bytes in one burst (accel + temp + gyro); check `Wire` return codes and byte count.
- I2C timeout (`Wire.setTimeout` / core equivalent). Count consecutive read errors.
  More than N in a row → `imu::healthy()` = false → mode manager disarms.
- Board orientation: a simple axis remap + sign table in `config.h` (e.g. `IMU_ROTATION_CW90`).
- Gyro calibration at boot: average ~1 s of samples. If the spread is too large (quad moved),
  restart. Arming is blocked until calibration succeeds.
- Accel calibration (level trim): CLI command `calib accel`, stored in params.
- Output in physical units (°/s, g), then a gyro PT1 low-pass (default ~90 Hz).

### 7.2 RC input (`drivers/rc_crsf` + `lib/crsf`)
- Parser is a byte-by-byte state machine: wait sync/address → length → type → payload → CRC.
  Every frame is CRC-checked and length-checked. Bad frames are counted, never used.
- Channels unpacked with explicit bit shifts (no bitfields, no compiler-layout dependence).
- Also decode `LINK_STATISTICS` (RSSI, LQ, SNR) for logs and failsafe info.
- `rc::update()` drains *all* available UART bytes every tick (non-blocking).
- `rc::linkOk()` = a valid RC frame within `RC_LINK_TIMEOUT_MS` (default 250 ms) **and** the
  receiver doesn't report LQ = 0.
- Output `RcInput`: roll/pitch/yaw in −1..+1, throttle 0..1, aux switches decoded to positions
  (low/mid/high). Values clamped, deadband applied around center.

### 7.3 Rates (`lib/flight/rates`)
- Simple and readable: `rate = maxRate * (expo * x³ + (1 − expo) * x)`.
- Params: `rate_max_roll/pitch/yaw` (°/s), `rate_expo`.

### 7.4 Attitude (`lib/flight/attitude`)
- Complementary filter: integrate gyro, correct slowly toward the accel angle.
- Accel correction is skipped when |accel| is far from 1 g (during hard maneuvers).
- Gives roll and pitch angles for Angle mode, the tilt arming check and the blackbox.
- Mahony is an optional later upgrade behind the same interface.

### 7.5 Angle mode (`lib/flight/angle_mode`)
- Stick → target angle (`angle_max`, default 45°). Error × `angle_p` → rate setpoint, then clamped.
- Yaw stays rate-controlled.

### 7.6 Rate PID (`lib/flight/pid`)
- One `Pid` instance per axis. P on error, **D on measurement** (no kick on stick moves),
  D-term PT1 filter, I with clamp (anti-windup).
- I is frozen at low throttle while on the ground, and reset when disarmed.
- Output is NaN-checked. If NaN shows up, output 0, raise a fault flag, and log it.
- Exposes P, I, D, and the sum for the blackbox.

### 7.7 Mixer (`lib/flight/mixer`)
- Quad-X table using Betaflight motor order (1 = rear right, 2 = front right, 3 = rear left,
  4 = front left; props-in by default). Kept as a table so other frames are a table change.
- Desaturation: if roll/pitch/yaw would push a motor beyond its range, shift all motors to keep
  the attitude command (control authority beats exact throttle). Then clamp.
- Output `MotorOutputs`: 0..1 per motor → converted to OneShot125 µs (125..250) at the board layer,
  with `motor_idle` (e.g. 5%) applied while armed so props keep spinning.

### 7.8 Motors (`board::motorsWrite`)
- OneShot125: pulse 125–250 µs, refreshed every loop (1 kHz / 500 Hz).
- `motorsStop()` from the very first line of `motorsInit()`, so ESCs never see garbage on boot.
- DShot is a later milestone (M9): ESP32 via RMT, STM32 via timer DMA.

---

## 8. Mode management and failsafe

All state lives in `lib/modes/mode_manager`. It's pure logic, gets time passed in, and is fully unit-tested.

### 8.1 States

Gyro calibration runs blocking in `setup()` before the loop starts, so in code the mode manager
only has DISARMED, ARMED and FAILSAFE; `NOT_CALIBRATED` stays as an arming check.

```
          ┌────────────┐  gyro cal OK   ┌──────────┐  arm checks pass +   ┌────────┐
 power ──►│ CALIBRATING├───────────────►│ DISARMED │──arm switch edge────►│ ARMED  │
          └────────────┘                └──────────┘◄──arm switch off─────└───┬────┘
                                          ▲      ▲                            │ link lost
                                          │      │ link back < stage1 end     ▼
                                          │      └───────────────────── ┌───────────┐
                                          │                             │ FAILSAFE  │
                                          │      stage 2 → motors off   │ (stage 1) │
                                          └─────────────────────────────┴───────────┘
                         any state: IMU failure or watchdog/fatal → DISARMED (motors stopped)
```

### 8.2 Arming checks (all must pass, otherwise the reason is reported)

Each failed check sets a bit in an `armBlockFlags` bitmask. It's shown by the `status` CLI command,
by LED blink count and by telemetry, and logged when the arm attempt fails:

| Flag | Condition |
|---|---|
| `NO_LINK` | `rc::linkOk()` is false |
| `THROTTLE_HIGH` | throttle > `arm_throttle_max` (e.g. 5%) |
| `ARM_SWITCH_NOT_CYCLED` | arm switch was ON at boot or after failsafe; must see OFF first |
| `NOT_CALIBRATED` | gyro calibration not done |
| `IMU_FAULT` | IMU unhealthy |
| `TILTED` | angle > `arm_max_angle` (e.g. 25°) |
| `CLI_ACTIVE` | CLI session open |
| `BATTERY_CRITICAL` | below critical voltage at rest |
| `LOOP_OVERLOAD` | recent loop overruns above threshold |
| `BOOT_GRACE` | less than 2 s since boot |

Arming happens on the **edge** of the arm switch (off → on) only. A switch already ON never arms.

### 8.3 Flight modes
- AUX1 = arm. AUX2 = mode: low = **Acro**, high = **Angle** (mid = Angle by default).
- Mode change while armed resets the PID I-terms (keeps transitions calm).
- Mode name ("ACRO", "ANGL", "!FS!") goes to telemetry and the blackbox.

### 8.4 Failsafe (staged)

| Stage | When | What |
|---|---|---|
| Guard | no RC frame for < `RC_LINK_TIMEOUT_MS` (250 ms, `config.h`) | keep last valid input; parser hiccups don't count |
| Stage 1 | until `fs_stage1_ms` (1000 ms) after the link is declared lost | force Angle mode, sticks centered (level), throttle = `fs_throttle` |
| Recovery | link back during stage 1 | return to normal (logged) |
| Stage 2 | after stage 1 | **motors off, disarm**, arm switch must be cycled before re-arming |

Defaults favor **drop** over "try to land", the safe choice for indoor and learning flights.
`fs_throttle` = 0 by default. Raise it only after hover throttle is known.

### 8.5 Other protections
- **IMU fault** → immediate disarm.
- **Battery**: low → warning (LED, telemetry, log). Critical → warning only in flight (never cut motors
  mid-air because of voltage), but blocks arming on the ground.
- **Watchdog** (~100 ms): if the loop hangs, the MCU resets. Motors then see the stop pulse from boot.
- **Runaway detection** (later, M8): armed + gyro error stays large with saturated PID for > N ms → disarm.
- **Motor test** from the CLI only while disarmed, with a required `props-off` confirmation and auto-stop after 2 s.

---

## 9. Observability

### 9.1 Event log (`system/log`)
- Small fixed queue of event structs `{ timeUs, level, code, a, b }`. Pushing one costs nothing
  in the fast loop (no formatting there).
- The slow task formats it into a fixed `char[96]` with `snprintf` and sends it to the debug serial
  **and** to the blackbox as an event record.
- Levels: `ERROR`, `WARN`, `INFO`, `DEBUG` (runtime-selectable via param).
- Events: boot, IMU found/fault, calibration done/retry, arm/disarm + reason, arm blocked + flags,
  mode change, failsafe enter/stage/exit, battery low/critical, SD error, loop overrun burst,
  param saved, watchdog reset detected at boot (via the board's reset-cause API).
- Queue full → drop oldest and count drops (the count itself is reported).

### 9.2 Blackbox (`system/blackbox` + `lib/blackbox_format`)
- **File per arm**: `LOG00042.BBL` opened on arm, closed on disarm (plus periodic sync while armed).
- **Header**: magic, format version, board name, loop rate, log rate divider, all param values, field list.
- **Loop record** (fixed-size, packed, little-endian, scaled ints, ~64 bytes):
  `time_us`, `loop_us`, `gyro_raw[3]`, `gyro_filt[3]`, `setpoint[3]`, `p[3]`, `i[3]`, `d[3]`,
  `motor[4]`, `rc[4]`, `aux` bits, `roll/pitch angle`, `battery_mV`, `mode`, `flags`.
- **Event record**: tagged record carrying the log events above.
- **Rate**: logs every Nth loop (`bb_rate_div`). Defaults: 500 Hz on ESP32, 250 Hz on Blue Pill.
- **No blocking in the loop**: records go into a RAM ring buffer (ESP32 16 KB, Blue Pill 2–4 KB).
  The slow task writes one 512-byte block per tick when one is ready. The file is **pre-allocated**
  on open (SdFat `preAllocate`), which keeps SD write latency spikes low.
- Buffer overflow → records dropped and counted (shown in perf and logged), flight unaffected.
- SD missing or failing → warning event, blackbox disabled, flying still allowed.
- **Tools**: `tools/bb_decode.py` → CSV (also works with PlotJuggler); `tools/bb_plot.py` → standard
  tuning plots (gyro vs setpoint per axis, PID terms, motors, loop time).

### 9.3 Performance measuring (`system/perf`)
- Each stage is wrapped in `perf::begin(Stage::Imu)` / `perf::end(Stage::Imu)` (array index, no strings).
- Per stage over a 1 s window: **min / avg / max µs**.
- Loop level: period jitter (actual vs target), **overrun count** (tick started late), worst-case loop time,
  **CPU load %** (busy time / period).
- Also: SD write latency max, blackbox dropped records, CRSF frames ok/bad, I2C errors,
  free heap (must stay constant), free stack where the core supports it.
- Shown by the `perf` CLI command, summarized into the blackbox at 1 Hz, and overrun bursts raise a log event.

**Timing budget (to verify in M3):**

| Stage | ESP32-S3 @ 1 kHz | Blue Pill @ 500 Hz |
|---|---|---|
| Budget per loop | 1000 µs | 2000 µs |
| IMU I2C read (14 B @ 400 kHz) | ~350 µs | ~350 µs |
| Filters + attitude + PID + mixer | < 50 µs | ~300–500 µs (soft float) |
| Blackbox record pack | < 20 µs | ~50 µs |
| Slow task slice (incl. SD block) | < 300 µs | < 800 µs |

If the Blue Pill can't hold 500 Hz with margin, drop it to 333 Hz (one `config.h` value)
or move to the Black Pill F411.

### 9.4 Parameters and CLI
- Param table: `{ name, float* value, min, max, default }` in one array. No heap, names are `const char*`.
- Stored with the Arduino `EEPROM` API (both cores provide it) with version + CRC. Bad CRC → defaults + warning.
- CLI over the debug serial, fixed `char[64]` line buffer, `strtok` / `strtof`:
  `help`, `status`, `perf`, `get <name>`, `set <name> <value>`, `save`, `defaults`, `dump`,
  `calib accel`, `motor <n> <0-100> props-off`, `bb info`, `log level <lvl>`, `reboot`.
- CLI only accepted while disarmed. An open CLI session blocks arming.

### 9.5 Telemetry to radio (`system/telemetry`)
- CRSF frames: battery (voltage, current if available), attitude, flight mode string.
- One frame per slow-task slot, written to `board::rcSerial()` (no POSIX `write()`).

---

## 10. Testing strategy

| Level | How | Covers |
|---|---|---|
| Unit (PC) | `pio test -e native` (Unity) | CRSF parser (valid, bad CRC, split frames, garbage), PID, filters, rates, mixer + desaturation, attitude, mode manager (every arming check, failsafe timeline), blackbox record encode/decode, ring buffer |
| Build | `pio run` for every env | Both MCUs always compile, no warnings in our code |
| Bench, props off | USB + battery, scope/logic analyzer | Loop timing, OneShot pulses, failsafe by switching the TX off, watchdog by forcing a hang (CLI `debug hang`) |
| Tethered / first hover | Props on, tied down or in an open field | Arming, Acro hover, blackbox review |
| Tuning | Blackbox plots | PID gains, filter cutoffs |

**Safety rules for every hardware test:** props off unless it's a planned flight test, smoke stopper
on first power-up, failsafe tested on the bench before every first flight after changes.

---

## 11. Milestones

Each milestone ends with both targets building and the unit tests passing.

**Status (2026-10-17):** M0–M8 code is written. Both targets build without warnings in our code
(Blue Pill: 75 KB flash, 13 KB static RAM), and 64 native unit tests pass. The blackbox decoder has been
checked against the C++ encoder. **No hardware testing has been done yet**: every "Done when" that needs
a scope, a board or a flight is still open. The README's bench checklist covers them in order.
M9 is not started.

### M0: Clean slate
- Remove PX4 leftovers (`defines.h`, `drv_hrt.h`, `queue.h`, `time.h`, `common_rc`, PX4 crsf).
- New folder skeleton, `platformio.ini` with `esp32s3`, `bluepill`, `native` envs and `build_src_filter`.
- Heap guard, `-Wall -Wextra`, empty `main.cpp` with the story outline as comments.
- README updated to match reality.
- **Done when:** all three envs build; `pio test -e native` runs an empty test.

### M1: Board layer
- `board.h` + both implementations: time, LED, serials, I2C, SPI, ADC, watchdog, fatal.
- Motors: OneShot125 on both MCUs, `motorsStop()` from boot.
- **Done when:** scope shows 125 µs pulses on all 4 pins on both boards; LED blinks; watchdog resets on a forced hang.

### M2: RC input
- Own CRSF parser in `lib/crsf` with unit tests, `drivers/rc_crsf`, link stats, link timeout.
- **Done when:** tests cover split/garbage/bad-CRC frames; channels print live; TX off → `linkOk()` false within 250 ms.

### M3: IMU + fixed loop + perf
- MPU6050 driver, gyro calibration, units, orientation, PT1 gyro filter.
- Loop timer, `perf` module, `perf` CLI command (minimal CLI here).
- **Done when:** loop runs at target rate with measured jitter/overruns; timing budget table filled with real numbers.

### M4: Event log + params + CLI
- Event queue, param table with EEPROM + CRC, full CLI.
- **Done when:** gains can be changed, saved, survive reboot; events show on serial.

### M5: Mode manager + safety (before any motor spins)
- `lib/modes` with full unit tests: arming checks + flags, arm edge, Acro/Angle switch, staged failsafe.
- Status LED patterns, battery monitor.
- **Done when:** every arming block reason can be triggered on the bench and is reported;
  failsafe timeline verified with the TX switched off (props off, motors observed via scope/ESC beeps).

### M6: Rate control (Acro)
- Rates, rate PID, mixer + desaturation, motor idle, I-term handling. All unit-tested.
- Motor test command.
- **Done when:** props-off bench test shows the correct motors speed up when the quad is tilted by hand;
  first tethered hover in Acro.

### M7: Blackbox
- Record format + tests, SD writer with ring buffer and pre-allocation, file per arm, header with params.
- `bb_decode.py`, `bb_plot.py`.
- **Done when:** a flight log decodes to CSV and plots gyro vs setpoint; SD latency and dropped
  records stay visible in perf.

### M8: Angle mode + hardening
- Complementary filter, angle mode, tilt arming check, accel calibration.
- Runaway detection, NaN guards verified, overrun → arming block, reset-cause logging.
- **Done when:** stable self-level hover; the failsafe stage-1 auto-level works in a real (low) flight.

### M9: Extras (pick any)
- CRSF telemetry to the radio.
- DShot (ESP32 RMT first, then STM32 DMA) behind the same `board::motorsWrite`.
- Mahony filter, gyro notch filter, buzzer/beeper, a third board (RP2040 or Black Pill F411).

---

## 12. External libraries

| Library | Use | Note |
|---|---|---|
| `greiman/SdFat` 2.x | Blackbox on SD | Static objects only, supports pre-allocation |
| Arduino core `Wire`, `SPI`, `EEPROM` | Bus + param storage | Available on both cores |
| STM32 core `HardwareTimer`, `IWatchdog` | Motors, watchdog (STM32 board file only) | Built into STM32duino |
| ESP-IDF `esp_task_wdt` | Watchdog (ESP32 board file only) | Built into arduino-esp32 |
| Unity | Unit tests | Bundled with PlatformIO |
| Python: `numpy`, `matplotlib` | Blackbox decode/plot tools | PC side only |

We write the IMU driver and CRSF parser ourselves on purpose: both are small, and they're the
best learning parts. If DShot gets hard on ESP32, the `DShotRMT` library is a fallback.

---

## 13. Default parameters (starting points, tune from blackbox)

| Param | Default | Meaning |
|---|---|---|
| `loop_hz` | 1000 / 500 | Per board (compile-time, `config.h`) |
| `gyro_lpf_hz` | 90 | Gyro PT1 cutoff |
| `dterm_lpf_hz` | 80 | D-term PT1 cutoff |
| `roll_p/i/d` | 0.0020 / 0.0030 / 0.00002 | Rate PID, normalized torque per deg/s of error |
| `pitch_p/i/d` | 0.0022 / 0.0030 / 0.000022 | |
| `yaw_p/i/d` | 0.0030 / 0.0030 / 0 | |
| `i_limit` | 0.3 | I-term clamp |
| `rate_max_rp` / `rate_max_y` | 500 / 400 °/s | Acro rates |
| `rate_expo` | 0.3 | |
| `angle_max` | 45° | Angle mode tilt limit |
| `angle_p` | 5.0 | Angle error → rate |
| `motor_idle` | 0.05 | Armed idle |
| `arm_throttle_max` | 0.05 | |
| `arm_max_angle` | 25° | |
| `RC_LINK_TIMEOUT_MS` | 250 | Compile-time (`config.h`), also the failsafe guard |
| `fs_stage1_ms` | 1000 | |
| `angle_max_rate` | 300 °/s | Limit on what angle mode may ask for |
| `runaway_dps` / `runaway_ms` | 300 / 1000 | Saturated with this much error for this long = disarm |
| `fs_throttle` | 0.0 | Raise after hover throttle is known |
| `bat_cell_low` / `bat_cell_crit` | 3.5 / 3.3 V | |
| `bb_rate_div` | 2 / 2 | Log every 2nd loop |

PID values are placeholders in a normalized unit. Expect to tune them in M6 using the blackbox.

---

## 14. Risks and how we handle them

| Risk | Mitigation |
|---|---|
| Blue Pill flash/RAM too small | 128 KB variant first; fallback Black Pill F411 (one board folder) |
| Soft-float too slow on F103 | 500 Hz target, measure in M3, lower rate if needed |
| SD write latency spikes | RAM ring buffer, pre-allocated file, block writes in slow task, drop-and-count |
| MPU6050 clones with odd WHO_AM_I | Accept known clone IDs, log the value found |
| I2C bus lockup | Timeouts, error counter → IMU fault → disarm; bus recovery attempt on the ground |
| arduino-esp32 API changes (v2 → v3) | Pin platform versions in `platformio.ini` |
| ESC startup garbage | Stop pulse from the first line of `motorsInit()` |
| Unexpected heap use from a library | Heap guard + free-heap check in `status` |
