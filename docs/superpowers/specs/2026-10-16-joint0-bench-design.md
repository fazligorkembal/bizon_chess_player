# Joint0 Bench — Electronics, Firmware and Motion Tests

**Date:** 2026-10-16
**Status:** Approved in conversation, section by section.
**Scope:** One axis (`bizon2rev1`, "joint0"; "J1" in the wiring guide) on a hand-soldered
perfboard, its bench firmware, and the host-side tool that measures it. The goal is to find
how fast joint0 can reach a commanded angle without shaking the board the pieces stand on.

Builds on `docs/superpowers/specs/2026-08-21-hardware-bringup-design.md` (open-loop contract,
endstop homing, fault chain) and `docs/hardware/f411-tmc2209-wiring.md` (pinout, driver and
endstop circuits, phase-1 bring-up). Nothing here changes the open-loop design: there is no
encoder, the angle is kept from the step count, and the optical endstop zeroes it.

## Goal

Measure, on the assembled arm, the motion settings for joint0 that give the shortest time to a
still board, and carry them into the game:

- real `max_velocity` and `max_acceleration` for `bizon2rev1` in
  `ros_env/src/bizon2_moveit_pkg/config/joint_limits.yaml`, which today holds a placeholder
  velocity of 12.56 rad/s and `has_acceleration_limits: false`;
- the TMC2209 settings that achieve them: run current, microstep resolution, StealthChop or
  SpreadCycle;
- how the MCU fills the 10 ms between the 100 Hz setpoints the game will send.

In the game MoveIt plans the motion and the JTC streams it at 100 Hz; the MCU does not choose
the profile. The bench therefore measures both the physical envelope and the cost of the 100 Hz
path, so that the numbers it produces mean the same thing when MoveIt is driving.

## Decisions

| Decision | Choice | Rationale |
|---|---|---|
| Axis | `bizon2rev1` alone, one TMC2209 | Incremental bring-up; the wiring guide's phase 1. |
| Load | The complete arm, mounted | Settings found on a bare shaft are meaningless once the arm's inertia is attached. |
| Position feedback | None; step count plus optical endstop | Given. No encoder exists on this machine. |
| Vibration sensor | The "10DOF" IMU, accelerometer only, on the wooden platform beside the chessboard | Robot base, board and IMU share one platform, so the IMU sees exactly the vibration that walks pieces. Magnetometer and barometer are unused. |
| Where the profile lives | Two stages: MCU-generated first, then replayed as 100 Hz setpoints | Separates the arm's physical limit from the artifacts of the 100 Hz path. The MCU needs a profile generator anyway, for homing and as the velocity/acceleration clamp. |
| Supply | 24 V | Given. More torque at speed than 12 V; regenerative rise during deceleration is measured, not assumed away. |
| Firmware base | STM32CubeIDE project, HAL + LL | CubeIDE 1.18.1, with its built-in CubeMX editor, is on the development machine. |
| Debug | ST-Link over SWD | Available. DFU over USB is the fallback only. |
| Instruments | Multimeter and digital logic analyzer | Available. No oscilloscope is assumed. |

## 1. Success Criterion and Mechanical Preconditions

A move is **valid** only if all of the following hold, and among valid moves the one with the
shortest time to a still board wins (`t_move + t_settle`, defined in §7):

1. **Zero lost steps.** After every trial the axis re-approaches the endstop slowly and records
   the step count at the edge. The difference from the reference edge is the number of steps
   the move lost. The tolerance is the homing repeatability measured in stage B, not zero.
2. **Peak horizontal platform acceleration below `A_max`.** `A_max` is found empirically: with
   pieces on the board, run a few deliberately aggressive moves, note the platform acceleration
   at which pieces start to shift, and take half of it.
3. **The platform settles.** After the commanded motion ends, the platform returns to its
   noise floor within the settle window.
4. **`VM` stays below 27 V.** The arm's inertia pumps the motor rail during deceleration and the
   TMC2209 is rated to about 29 V, so on this direct-drive axis the deceleration limit may be
   set by the rail rather than by torque.

**rev2 and rev3 must be locked mechanically during joint0 tests.** With the full arm attached
and those joints unpowered, link2 swings freely whenever joint0 accelerates; the IMU would then
measure the swinging link rather than joint0's profile and every result would be meaningless.
Lock them with a clamp, pin or bolt in the **fully extended** configuration, which has the
largest inertia about joint0, so that the limits found are worst-case. Repeat the final
settings once with the arm folded as a check. The Z axis needs nothing: its T8 leadscrew
self-locks (hardware design §6).

**Cable wrap.** Rotating joint0 winds the cables that run up the arm. The firmware enforces a
software angle limit around home (for example ±170°); the real value comes from the cable
length and is an open item below.

## 2. Stages

Each stage has its own acceptance tests (§6) and is finished before the next is wired.

| Stage | Hardware added | Software added | Proves |
|---|---|---|---|
| **A** | BlackPill, IMU, logic-analyzer header, MARK line | CubeMX project, USB CDC text protocol, IMU driver, host `record`/`analyze` | Sampling, timestamps, the I2C cable, platform noise and resonances |
| **B** | TMC2209, NEMA17, 24 V supply, endstop, `VM` sense | TMC UART, step generation, endstop, ADC | Wiring guide phase 1, steps 1-6 |
| **C** | — | MCU-side trapezoid and S-curve profiles, host sweep | The physical envelope of the arm |
| **D** | — | 100 Hz setpoint streaming and interpolation | The cost of the game's 100 Hz path |

Stage A is built on a 6 × 9 cm trial perfboard that carries only the logic side. Stage B
moves to a 10 × 10 cm board carrying both sides; the BlackPill and every module sit on
female headers, so they move across unchanged. The schematic and each board's placement
are therefore drawn per stage: the stage-A plan draws stage A, and stage B's spec update
extends both.

Stage B, C and D firmware is bounded here, not specified line by line. This document is
updated with their detail before each of those stages is planned.

## 3. Electronics

### 3.1 Pin allocation

Verified against ST's own pin data (`STMicroelectronics/STM32_open_pin_data`,
`mcu/STM32F411C(C-E)Ux.xml` and `mcu/IP/GPIO-STM32F411_gpio_v1_0_Modes.xml`). Every claim in the
wiring guide's allocation checked out. One change is needed.

**I2C1 on PB6/PB7 requires moving J4 STEP.** None of the pins left free by the wiring guide's
allocation carries an I2C function: I2C1's only clean pair is PB6/PB7, and PB6 is J4 STEP
(TIM4_CH1). Therefore:

- J4 STEP moves from **PB6 to PA1**, TIM5_CH2 (AF2). TIM5 is 32-bit, which suits a stepper
  time base.
- The control tick moves from **TIM5 to TIM4**. The tick needs a time base, not a pin, so TIM4's
  pins (PB6-PB9) stay free for I2C1, the servo (PB8, TIM10) and endstop 4 (PB9).

SPI was considered for the IMU and rejected: every SPI instance on this package collides with
the existing STEP, DIR, DIAG or endstop pins, and I2C at 400 kHz carries 1 kHz accelerometer
data with room to spare.

**Chip correction, 2026-11-01.** The board's chip is an STM32F401CEU6 (SWD device ID 0x433, chip
marking confirmed), not the F411 this section was checked against. The allocation was compared
again against CubeIDE 1.18.1's `STM32F401C(D-E)Ux.xml` and `GPIO-STM32F401_gpio_v1_0_Modes.xml`:
same signals, same AF numbers, no pin moves. The F401 lacks only SPI4/5, I2S and SDIO.

| Function | Pin | Mode | Stage |
|---|---|---|---|
| IMU SCL | PB6 | I2C1_SCL (AF4), open-drain | A |
| IMU SDA | PB7 | I2C1_SDA (AF4), open-drain | A |
| IMU INT (data ready) | PA2 | GPIO input, EXTI2, rising edge | A |
| MARK | PA4 | GPIO output, push-pull | A |
| Heartbeat LED | PC13 | GPIO output, active low | A |
| J1 STEP | PA8 | TIM1_CH1 (AF1) | B |
| J1 DIR | PB12 | GPIO output | B |
| `EN`, all drivers | PB5 | GPIO output, active low, 10 kΩ pull-up to 3.3 V | B |
| TMC UART | PA9 | USART1_TX (AF7), single-wire half-duplex | B |
| Endstop 1 | PB10 | GPIO input, EXTI10 | B |
| DIAG 1 | PA5 | GPIO input, polled | B |
| `VM` sense | PA3 | ADC1_IN3 | B |
| J4 STEP (future) | PA1 | TIM5_CH2 (AF2) | — |
| Spare | PA0 (user KEY on most boards), PA10 | — | — |

IMU INT uses EXTI2. The endstops use EXTI numbers 10, 3, 4 and 9, so there is no collision.
PA2, PA3 and PA4 are `TTa` (3.3 V only). That is fine here: INT comes from a 3.3 V device, the
`VM` divider output stays below 3.3 V, and MARK is an output. PB6 and PB7 are `FT`.

The wiring guide's allocation table is updated to match as part of this work.

### 3.2 Stage A circuit

- **BlackPill** on two 20-pin female headers, never soldered directly to the board. Measure the
  row spacing with calipers before soldering the headers.
- **IMU connector**, 5 pins: `VCC` (from the BlackPill 5 V pin), `GND`, `SCL`, `SDA`, `INT`.
  The board is a GY-87: `VCC_IN`, `GND`, `SCL`, `SDA` and `INTA` are used, and `FSYNC` is
  tied to the module's `GND` at the module end, because a floating FSYNC input is undefined.
  It carries its own 3.3 V regulator and pull-ups to it, so it takes 5 V on `VCC_IN` and
  presents 3.3 V logic.
- **2.2 kΩ pull-ups** on `SCL` and `SDA` to 3.3 V on the perfboard, behind a 2-pin jumper each so
  they can be removed. With 30-60 cm of cable the board's own 4.7-10 kΩ pull-ups give rise
  times outside the 400 kHz specification; the extra pull-ups in parallel bring them back.
- **IMU cable**, 30-60 cm to the platform beside the board. Twist `SCL` with `GND` and `SDA`
  with `VCC`, and route it away from the motor cable.
- **Logic-analyzer header**, 1×9 male: `CH0 STEP`, `CH1 DIR`, `CH2 ENDSTOP`, `CH3 TMC_UART`,
  `CH4 SCL`, `CH5 SDA`, `CH6 IMU_INT`, `CH7 MARK`, `GND`. Channels 0-3 are empty until stage B.
  Never connect an analyzer channel to `VM` or a motor coil.
- **Test points:** `EN`, 3.3 V, 5 V, `GND`.
- **SWD:** the ST-Link connects to the BlackPill's own SWD pins. PA13/PA14 are never allocated.

### 3.3 Stage B circuit

These circuits come from the wiring guide and are not changed:

- the TMC2209 per-driver circuit, address 0 (MS1/MS2 to GND), with `PDN_UART` through 1 kΩ to
  the PA9 node;
- the optical endstop with its 10 kΩ pull-up, 1 kΩ series resistor and 100 nF;
- the star ground.

New or changed here:

- **24 V input:** screw terminal, then a 2 A slow-blow fuse, then a **1000 µF / 50 V** bulk
  capacitor at the board entry, then the `VM` bus.
- **At the driver:** 100 µF / 50 V low-ESR plus 100 nF ceramic across its own `VM`/`GND` pads,
  as the wiring guide requires.
- **`VM` sense:** 100 kΩ / 10 kΩ divider with 100 nF to PA3. At 35 V the pin sees 3.18 V, so a
  regenerative spike well past the driver's rating is still measured rather than clipped.
  Resolution is about 9 mV per ADC count.
- **No TVS diode.** A 24 V-class TVS does not clamp reliably below the TMC2209's 29 V. The
  remedy for regenerative rise is limiting deceleration, and the `VM` sense exists to measure
  it.
- **The TMC2209 module** sits on two 8-pin female headers (StepStick footprint, 0.5" row
  spacing), so a dead driver is a swap rather than a desolder.
- **Motor:** 4-pin connector. Never plug or unplug it with `VM` live.

### 3.4 Perfboard layout

- The power zone runs along one edge: 24 V terminal, fuse, bulk capacitor, driver and motor
  connector. The logic zone (BlackPill, IMU connector, analyzer header) is on the other side.
- The star point sits next to the bulk capacitor, and the PSU ground lands there. The BlackPill
  ground reaches it by one wire.
- `VM` and its ground return run in AWG18-20 wire. Signals run in thin wire.
- Space for drivers J2-J4 is left empty beside J1 so the board grows to four axes in place.

### 3.5 Deliverables

Stored in `hardware/joint_bench/`:

1. **KiCad project:** the schematic and a project symbol library for the BlackPill and the
   headers, which are not in the stock libraries. The schematic is also exported to PDF and
   SVG, and the exports are committed.
2. **Placement drawing:** a KiCad board laid out on the 2.54 mm grid at the perfboard's real
   size and exported 1:1, so it can be printed and laid under the board. It places components
   only; the wiring is defined by the net list below.
3. **Net checklist:** every net with every pin on it and a checkbox, used while soldering and
   then for a continuity test with the multimeter. It is **generated** from the KiCad netlist by
   a script (`kicad-cli` netlist export, then a short Python converter), so it cannot drift from
   the schematic.

All three are generated by `tools/kicad_gen/generate.py` from `hardware/joint_bench/nets.yaml`
and never edited by hand. KiCad is not installed on the host: `scripts/kicad-cli` runs
`kicad-cli` from the `kicad/kicad:10.0.6-full` image, pinned by digest.

## 4. Firmware

### 4.1 Toolchain and layout

The firmware lives in `firmware/joint_bench/`. It is not a ROS package and is not part of the
colcon workspace.

- **`joint_bench.ioc`** is the STM32CubeIDE project, with Toolchain/IDE set to **STM32CubeIDE** and
  "Generate peripheral initialization as a pair of .c/.h files" enabled. It is committed along
  with the generated code, so the project builds on a machine without CubeMX. The `.ioc`
  records the CubeMX version and the `STM32Cube FW_F4` package version; those are the pins for
  ST's sources.
- **Our code** lives in `app/` and `lib/`, in its own files. Generated files are touched only
  inside `USER CODE` blocks, and only to make one-line calls into `app/`, so regenerating from
  CubeMX never overwrites our logic.
- **Language:** C++17 for `app/` and `lib/` (`-fno-exceptions -fno-rtti`, no heap after init).
  The generated code stays C.
- **Flash and debug:** ST-Link over SWD, using OpenOCD or STM32CubeProgrammer/CubeIDE. DFU is
  the fallback only.
- **Verification on this machine:** a pinned `docker/dockerfile-firmware` (Ubuntu 22.04, a fixed
  Arm GNU Toolchain release, CMake, Ninja, a pinned GoogleTest) runs the `lib/` unit tests on the
  host. Target builds, flashing and running on hardware happen in CubeIDE.

### 4.2 CubeMX configuration (stage A)

**Clock.** HSE 25 MHz crystal → PLL with M = 25, N = 336, P = 4, Q = 7. That gives SYSCLK
84 MHz, the F401's maximum, and exactly 48 MHz for USB. AHB = 84 MHz, APB1 = 42 MHz (APB1 timers
84 MHz), APB2 = 84 MHz. Flash latency 2 wait states. The crystal's value is assumed until USB
enumerates on a build clocked from HSE.

**Peripherals:**

| Peripheral | Setting |
|---|---|
| SYS | Debug: Serial Wire. Timebase: SysTick. |
| USB_OTG_FS | Device only. Middleware `USB_DEVICE`, class CDC. |
| I2C1 | PB6/PB7, Fast Mode 400 kHz, DMA on RX. |
| GPIO | PA2 EXTI rising; PA4 output, low; PC13 output, high (LED off). |
| DWT | Cycle counter enabled in code; it is not a CubeMX setting. |

**NVIC priority order, highest first:**

1. reserved for the step timers (stage B);
2. EXTI2, I2C1 event and error, and the I2C1 RX DMA stream;
3. TIM4 control tick (stage B);
4. USB OTG FS;
5. SysTick.

USB sits below sampling because a late USB interrupt costs only latency, while a late sampling
interrupt corrupts a timestamp.

### 4.3 Layers

- **`lib/`** holds pure logic with no HAL includes: the command parser, the telemetry line
  formatter, the 64-bit extension of the cycle counter, the sample queue, and later the profile
  generator and step executor. It is built and unit-tested on the host, the same pattern as
  `fen_utils.hpp`.
- **`drivers/`** wraps hardware: the IMU driver over HAL I2C + DMA, and the USB CDC transmit ring.
  Stage B adds TMC UART, the step timer, the ADC and the endstop.
- **`app/`** holds the command loop and streaming state, called from `main.c`.

### 4.4 Protocol (stage A)

The protocol is line-based ASCII over USB CDC in both directions, so `screen` or `minicom` can
drive it by hand. The framed binary protocol in the host plan's Task 1 is for the game and is
not used here.

**Commands:**

| Command | Reply |
|---|---|
| `ver` | Firmware version (`git describe` embedded at build time) |
| `i2cscan` | Responding 7-bit addresses |
| `imu info` | Detected chip, configuration, LSB-to-g scale |
| `imu cfg rate=<Hz> dlpf=<Hz> fs=<g>` | Applies the configuration and echoes what the chip accepted |
| `stream on` / `stream off` | Starts or stops telemetry |
| `mark [label]` | Toggles MARK and emits an event line |
| `stats` | Counters: samples, `imu_overrun`, `i2c_err`, `tx_drop` |

**Line types:**

- `OK ...` / `ERR ...` are command replies.
- `T,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>` is one IMU sample. Values are raw counts, and
  the scale is in `imu info` and the recording's metadata. At 1 kHz this is about 50 kB/s,
  comfortable for USB full speed.
- `E,<t_us>,<name>` is an event, such as `mark`.
- `# ...` is a log line.

`seq` increments per sample. `t_us` comes from the DWT cycle counter, extended to 64 bits in
software and converted to microseconds. Stage B timestamps the step counter and `VM` on the
same clock, which is what puts motion and vibration on one time axis.

**The firmware never blocks on USB.** Lines go into a transmit ring. If the host stops reading
and the ring fills, whole lines are dropped and counted in `tx_drop`, and the host sees the gap
in `seq`. A stalled host must not stall sampling.

### 4.5 IMU sampling

The firmware identifies the chip itself: GY-87 boards normally carry an MPU-6050, but clones
ship with other parts. It runs
`i2cscan`, then reads `WHO_AM_I`: MPU6050 returns 0x68, MPU6500 0x70, MPU9250 0x71, and an
ADXL345 at address 0x53 returns `DEVID` 0xE5. A driver is written only for the family actually
found.

**Primary path, data-ready interrupt.** The IMU samples on its own clock at 1 kHz and raises
`INT`. The EXTI2 handler timestamps the edge and starts a DMA read of the data block. The
DMA-complete callback pushes the raw sample and its timestamp into a single-producer,
single-consumer queue, and the main loop formats and transmits it. The MCU never polls the IMU
on a timer of its own, because two free-running clocks drift and the result is duplicated or
skipped samples. An `INT` that arrives while a read is still in flight is counted as
`imu_overrun`.

The board is a GY-87, which breaks out `INTA`, so the FIFO fallback once considered here for
boards without an `INT` pin is not built.

**Default configuration for the MPU family:** clock source the gyro PLL (more accurate than
the internal oscillator, which matters for test A3), 1 kHz output rate, accelerometer DLPF at
about 188 Hz, full scale ±4 g. ±4 g leaves headroom for the tap test, where a pencil strike can exceed
±2 g. The accelerometer's noise density is far above its resolution, so ±4 g costs nothing.
Gyro data is streamed because it rides in the same read, but no metric uses it.

### 4.6 Stage B and later (boundary only)

A single executor, driven by the 1 kHz TIM4 tick, carries out one instruction: "be at step
position P at the end of this tick". Steps come from hardware PWM, with the timer's period
updated every tick. A fractional step carries into the next tick, so position never drifts.

- In **MCU-profile mode** (stage C), a trapezoid or S-curve generator produces P(t).
- In **streaming mode** (stage D), P(t) comes from interpolating the host's 100 Hz setpoints,
  either held as steps or with linear velocity.

Stages C and D therefore exercise the same executor and differ only in where P(t) comes from.
That is what makes their results comparable.

## 5. Host Tool

The host tool is `tools/joint_bench/`, a Python 3.10 package run as `python -m joint_bench`. It
runs on the host directly, with no ROS, from a virtualenv built from a pinned
`requirements.txt` (pyserial, numpy, scipy, matplotlib, pyyaml).

- **`console`** is an interactive serial session that logs everything it sees.
- **`record --seconds N --label L`** writes `debug/bench/<timestamp>_<L>/raw.csv` and
  `meta.yaml`. The metadata holds the firmware version, the IMU chip and configuration, the git
  sha and the label. `debug/` is already gitignored.
- **`analyze <dir>`** reports dropped samples, the sample-interval distribution, per-axis noise
  RMS, a Welch PSD, tap detection, and the dominant frequency and damping ratio of each tap.
  It writes its plots as PNG files beside the data.
- **`sweep <sweep.yaml>`** (stage C) runs a trial list, computes the §7 metrics for each trial,
  and writes `summary.csv` and a time-versus-vibration Pareto plot.

Parsing and metric code are pure functions, tested with pytest on synthetic signals: a known
sine must yield its frequency, an injected `seq` gap must be counted, and a known
damped-oscillation decay must give back its damping ratio.

## 6. Acceptance Tests

### Stage A

| # | Test | Pass |
|---|---|---|
| A1 | `i2cscan` + `WHO_AM_I` | The chip is identified, and recorded in this document. |
| A2 | Static, platform at rest | Mean \|a\| within 1 g ± 5%. The platform's vertical axis is identified, and the other two define the horizontal plane. |
| A3 | 10 min stream | Zero `seq` gaps. Mean sample interval within 1 ms ± 2% (it follows the IMU's own oscillator). |
| A4 | Same 10 min, cable at final length | `i2c_err` = 0 in `stats`. A logic-analyzer capture of SCL/SDA decodes cleanly. |
| A5 | Noise floor, 60 s at rest | Horizontal RMS and PSD recorded. This is the baseline for `t_settle`. |
| A6 | Tap test: 5 strikes at each of 3 points (beside the board, beside the robot base, a corner) | Platform natural frequencies and damping ratios recorded. These are the frequencies the motion profile must avoid exciting. |
| A7 | `mark` ×5 with the analyzer on `MARK` and `IMU_INT` | For each mark, the interval from the MARK edge to the next INT edge on the analyzer matches the interval from the `E` line's `t_us` to the next `T` sample's `t_us` within 20 µs. This shows the firmware's timestamps sit where the hardware edges are. The five intervals are read off by hand with the analyzer's cursors. |

### Stage B

Wiring guide phase 1, steps 1-6, as written there: UART readback with `VM` off, endstop with the
motor off, low-current motion, endstop under motor current, ten homing runs from ten starting
positions, and the angle check with a pointer. In addition:

- `VM` sense readings agree with the multimeter within 2% at rest;
- the homing repeatability (max − min of the ten edge positions, in microsteps) is recorded,
  because it becomes the lost-step tolerance for stage C.

### Stage C

- **Targets:** 15°, 45°, 90° and 180°. Replace them with the angles the real board geometry
  needs once those are computed.
- **Each trial:** endstop check, move, 2 s dwell, return, endstop check.
- **Parameters:** profile type (trapezoid or S-curve), `v_max`, `a_max`, jerk or jerk time, run
  current, microstep resolution, StealthChop or SpreadCycle. The S-curve's jerk time is chosen
  relative to the platform frequencies from A6.
- **Search:** a ladder, not a grid. Start from conservative values and raise one parameter at a
  time until the first failure. A full grid is combinatorially large, and every failed trial
  on the far side of the limit is a trial that stresses the hardware.
- **Abort:** any lost steps, `VM` > 27 V, a StallGuard flag, or a driver over-temperature flag
  ends the trial and that ladder.
- **Output:** `summary.csv` and the Pareto plot.

Before stage C's sweeps count, `A_max` is established with pieces on the board, as described in
§1.

### Stage D

Replay the best stage-C profiles as 100 Hz setpoint streams with each interpolation (step hold,
linear velocity), and compare against the same profile run MCU-side. The chosen values go into
`joint_limits.yaml` in a separate commit that cites the recording directories they came from.

## 7. Metrics

- **Horizontal acceleration** `a_h(t)`: the IMU acceleration minus the static mean from a rest
  window, projected onto the platform plane found in A2, as a magnitude. Pieces slide and tip
  under horizontal acceleration, so this is the quantity that matters.
- **`noise_rms`:** RMS of `a_h` over a rest recording (A5).
- **`t_move`:** from the first commanded step to the last.
- **`t_settle`:** from the end of `t_move` until the 50 ms sliding RMS of `a_h` stays below
  3 × `noise_rms`.
- **`peak_a_h`**, **`rms_a_h`:** over `[start, end of t_settle]`.
- **`lost_steps`:** the edge position after the trial minus the reference edge, in microsteps.
- **`vm_max`:** the maximum of the `VM` sense over the trial.

**Objective:** minimise `t_move + t_settle` subject to `peak_a_h < A_max`,
`|lost_steps| ≤ homing repeatability`, and `vm_max < 27 V`.

## Open Items

These need to be read off labels or measured. None of them blocks stage A except the trial
board's hole count, which the placement drawing needs.

- The IMU chip on the GY-87. Firmware test A1 settles it.
- The 6 × 9 cm trial board's hole count, and the BlackPill's row spacing. Measure both.
- The 10 × 10 cm stage-B board, not yet bought.
- NEMA17 model, rated phase current and holding torque.
- TMC2209 module make and its sense resistor (0.11 Ω and 0.15 Ω both ship, and the `VREF`
  formula moves with it).
- Supply current rating.
- Optical endstop module type (bare photo-interrupter or a 3-wire module, and whether its
  output is push-pull or open-collector).
- Software angle limit for joint0, from the cable length.
- How the endstop flag is mounted on joint0.

## Out of Scope

- The framed binary protocol and the `serial` ros2_control branch (host plan Tasks 1-5).
- Axes other than joint0.
- Closed-loop control.
- The IMU's magnetometer and barometer. The magnetometer is useless beside stepper motors in
  any case.
- Input shaping. It becomes a candidate follow-up if A6 shows a single dominant low-frequency
  platform mode that S-curve jerk timing cannot avoid.
