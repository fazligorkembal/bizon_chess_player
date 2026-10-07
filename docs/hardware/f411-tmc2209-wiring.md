# Wiring and Pinout — BlackPill F411 + TMC2209 + Optical Endstops

**Date:** 2026-09-04
**Status:** Proposed. Nothing here has been built or measured yet.
**Scope:** The electrical side of the incremental bring-up: joint 1 alone first, then the
remaining axes onto the same allocation without rewiring.

Companion to `docs/superpowers/specs/2026-08-21-hardware-bringup-design.md` (§1 defines the
hardware/software boundary this pinout serves) and
`docs/superpowers/plans/2026-08-21-hardware-bringup.md` (Tasks 1-5 are the host-side work that
consumes it).

The board assumed is a WeAct BlackPill, STM32F411CEU6 in UFQFPN48. That package has PORTA 0-15,
PORTB 0-15, PC13/14/15 and PH0/PH1 — **no PORTC 0-12**, which is why USART6 is unusable here
(its only pins on this package are PA11/PA12, taken by USB).

Verify every alternate-function claim below against DS10314 Table 9 and your own board's
schematic before soldering. Board revisions differ in what they tie to PA0, PA9 and PB2.

## Read this before every power-on

The four ways to destroy a TMC2209, in the order people actually manage it:

1. **No bulk capacitor.** 100 µF low-ESR across `VM`/`GND` — the **motor** rail, never `VIO` —
   **on every driver**, at the driver's own `VM`/`GND` pins or socket pads, not at the PSU. One
   per driver. Adding a second driver without a second cap counts as forgetting it. Electrolytic,
   so polarity matters: **+ to `VM`, − to `GND`**. Rate it at roughly twice the supply: 50 V for a
   24 V rail, 35 V minimum for 12 V; deceleration pushes the rail above nominal and the TMC2209
   itself tops out near 29 V. This is the single most common way these modules die, and it kills
   them silently on the first fast move rather than at power-on.
2. **Plugging or unplugging a motor while `VM` is live.** Power down first. Every time.
3. **`VIO` on 5 V.** It must be the same 3.3 V rail as the MCU, or `DIAG` and `PDN_UART` push
   5 V into pins that may not tolerate it.
4. **No star ground.** PSU GND, servo BEC GND and BlackPill GND meet at one point. Motor return
   current through the signal ground is how endstops start triggering on their own.

## Reserved pins — do not allocate

| Pin | Why |
|---|---|
| PA11, PA12 | USB D-/D+. This is the host link. |
| PA13, PA14 | SWDIO/SWCLK. Keep them: bring-up without a debugger is self-harm. |
| PH0, PH1 | 25 MHz HSE crystal. |
| PB2 | BOOT1. Driving it at reset changes the boot mode. |
| PA0 | User KEY button on most BlackPill revisions. Usable, but check first. |
| PC13 | Onboard LED. Fine to use as the heartbeat/fault indicator. |

**5 V tolerance:** the ADC-capable pins (PA0-PA7, PB0, PB1) are `TTa` — **3.3 V only**. Every
other GPIO here is `FT`. All endstop inputs below sit on `FT` pins so a 5 V-powered sensor
module cannot destroy them, but powering the sensors from 3.3 V is still the better answer.

## Full allocation — four steppers, one servo gripper

One timer per axis, as §1 of the design doc requires: each axis needs its own step frequency,
so they cannot share a timer's time base.

| Function | Pin | Mode | Note |
|---|---|---|---|
| J1 `rev1` STEP | PA8 | TIM1_CH1 (AF1) | |
| J1 `rev1` DIR | PB12 | GPIO out | |
| J2 `pris1` STEP | PA15 | TIM2_CH1 (AF1) | JTDI by default; free under 2-wire SWD |
| J2 `pris1` DIR | PB13 | GPIO out | |
| J3 `rev2` STEP | PB0 | TIM3_CH3 (AF2) | |
| J3 `rev2` DIR | PB14 | GPIO out | |
| J4 `rev3` STEP | PB6 | TIM4_CH1 (AF2) | |
| J4 `rev3` DIR | PB15 | GPIO out | |
| Gripper servo | PB8 | TIM10_CH1 (AF3) | 50 Hz, 1 MHz time base |
| `EN`, all drivers | PB5 | GPIO out, active LOW | one net, see below |
| TMC UART | PA9 | USART1_TX (AF7), half-duplex single-wire | 115200 baud |
| Endstop 1 | PB10 | input, pull-up, EXTI10 | |
| Endstop 2 | PB3 | input, pull-up, EXTI3 | JTDO/SWO by default; disable SWO |
| Endstop 3 | PB4 | input, pull-up, EXTI4 | NJTRST by default |
| Endstop 4 | PB9 | input, pull-up, EXTI9 | |
| DIAG 1-4 (StallGuard) | PA5, PA6, PA7, PB1 | input, polled | 3.3 V from driver VIO |
| Heartbeat / fault LED | PC13 | GPIO out | |
| Spare timer | TIM5 | — | reserved for the 100 Hz control tick |

**EXTI numbers must be distinct.** EXTI lines are shared by *pin number* across ports, so PA10
and PB10 cannot both be edge sources. The endstops above use numbers 10, 3, 4 and 9 — no
collision. The DIAG lines are polled rather than edge-triggered precisely to avoid burning four
more EXTI numbers on a signal that is a slow fault flag, not a timing event.

**One `EN` net for all drivers.** A fault should drop every axis, and the host's latched-fault
path (Task 4 of the host plan) is all-or-nothing anyway. Pull `EN` **high** to VIO through 10 kΩ
so the drivers are disabled while the MCU is in reset or unprogrammed. Disabling the drivers is
safe for `bizon2pris1`, the only gravity-loaded axis, because its T8 leadscrew has a 2 mm lead
and self-locks (design §6). Swap it for an 8 mm lead and that stops being true: Z then falls on
every disable, and nothing in firmware prevents it.

## TMC2209, per driver

```
                 +-------------------+
   VM  12-24V ---| VM            VIO |--- 3.3 V  (from BlackPill 3V3)
   PSU GND    ---| GND           GND |--- GND
                 |                   |
   PA8  ---------| STEP          DIR |--------- PB12
                 |                   |
   PB5  ---------| EN         PDN_UART|---[1k]--- PA9   (single-wire bus)
                 |                   |
   PA5  ---------| DIAG          MS1 |--- addr bit0   (GND or VIO)
                 |               MS2 |--- addr bit1   (GND or VIO)
                 |  INDEX  unused    |
                 | 1A 1B 2A 2B       |
                 +-------------------+
                     |  |  |  |
                   NEMA17 coils A / B
```

- **100 µF low-ESR electrolytic across VM/GND, as close to the driver as the layout allows, on
  every driver.** Not optional. A missing bulk cap is the single most common way to kill a
  TMC2209.
- **Never plug or unplug a motor while VM is live.** Same outcome.
- **VIO must be 3.3 V**, the same rail as the MCU. Feeding VIO 5 V while the MCU drives 3.3 V
  logic is out of spec at the driver's input threshold and puts 5 V on DIAG and PDN_UART.
- **MS1/MS2 select the UART address, not the microstepping,** once UART control is used.
  Microsteps come from `CHOPCONF.MRES` over the bus.

  | Driver | MS1 | MS2 | Address |
  |---|---|---|---|
  | J1 | GND | GND | 0 |
  | J2 | VIO | GND | 1 |
  | J3 | GND | VIO | 2 |
  | J4 | VIO | VIO | 3 |

  Four addresses per bus is exactly four drivers, so a servo gripper means **one UART bus is
  enough** — the two-bus split flagged in design §1 only applies if the fifth axis is a stepper.
- **PDN_UART is half-duplex.** All four `PDN_UART` pins hang on one node, each through its own
  1 kΩ series resistor, and that node goes to PA9 configured as USART1 in single-wire
  half-duplex mode. PA10 stays free.
- **Current.** Set `IHOLD_IRUN` over UART; the on-board `VREF` pot still sets the ceiling on most
  modules, so trim it conservatively first. For a 0.11 Ω-sense SilentStepStick the usual
  approximation is `I_rms ≈ VREF × 1.77 / 2.5` — 1.0 V gives about 0.7 A RMS. **Check the sense
  resistor on the module you actually bought**; 0.11 Ω and 0.15 Ω variants both ship, and the
  formula moves with it. Start at roughly 60% of the motor's rated phase current.
- Reduced run current on the gripper axis is the force-limit trick in design §5. It disappears
  if the gripper becomes a hobby servo — see the servo note at the end.

## Optical endstop

A slotted photo-interrupter (H21A1, ITR9608 and similar) wired discretely:

```
   3.3 V ---[220R]---|>|--- GND            LED side (If ~ 10 mA)

   3.3 V ---[10k]---+--- collector
                    |
                    +---[1k]---> PB10      output to MCU
                    |
                  [100n]
                    |
   GND ------------ +--- emitter
```

- Flag **in** the slot: phototransistor off, output pulled **HIGH**.
  Flag **out**: transistor conducts, output **LOW**.
  Fix this polarity in firmware as a per-axis `invert` flag rather than in the wiring — every
  ready-made module inverts it differently.
- The 10 kΩ pull-up and the internal pull-up can coexist; keep the external one so the line is
  defined even with the MCU unpowered.
- The 1 kΩ series resistor and 100 nF are not decoration. Endstop wires run alongside motor
  cables, and stepper current makes exactly the kind of edge that a bare input reads as a trigger.
- **Debounce in firmware anyway:** require the level to hold for 1-5 ms. An optical sensor has no
  contact bounce, but the cable does have induced noise.
- A ready-made 3-wire module (VCC/GND/OUT) works the same way; power it from **3.3 V**, not 5 V,
  and confirm whether its output is push-pull or open-collector before trusting the level.

## Power and grounding

| Rail | Source | Feeds |
|---|---|---|
| 12-24 V | Bench PSU | TMC2209 `VM` only |
| 5-6 V, ≥ 3 A | Separate BEC/regulator | Servo gripper only |
| 3.3 V | BlackPill regulator | Driver `VIO`, endstops, logic |
| 5 V | USB | BlackPill only |

- Star-ground: PSU GND, BEC GND and BlackPill GND meet at **one** point. Motor return current
  through a shared signal ground is how endstops start triggering by themselves.
- Do **not** power the servo from the BlackPill's 5 V pin. An MG996R-class servo stalls at
  1.5-2.5 A; USB will brown out the MCU mid-move.
- Keep the USB cable and the motor cables physically apart. A USB CDC dropout during a move is
  indistinguishable from a link timeout in the host's fault chain.

## Phase 1 — joint 1 alone

Wire only: TMC2209 #1 (address 0, MS1/MS2 to GND), PA8 STEP, PB12 DIR, PB5 EN, PA9 UART, PB10
endstop, PA5 DIAG, and the motor. No other driver populated, no servo.

1. **VIO only, VM off.** Read `IOIN` (0x06) over UART and check the version field, then read back
   a register you have written. This proves the bus, the address strap and the 1 kΩ resistor
   before any motor current exists. If this fails, nothing downstream is worth trying.
2. **Endstop with the motor still off.** Poll PB10 and print it. Pass a piece of card through the
   slot and confirm the level flips, that it is stable at rest, and that it does not chatter.
3. **VM on, low current.** `IRUN` at roughly a third of target. Command 200 full steps one way,
   200 back. Confirm direction matches the DIR level and that the motor is not skipping or
   screaming. Raise current only after direction and smoothness are right.
4. **Endstop with the motor running.** Repeat step 2 while the axis moves. This is the test that
   actually matters: if the endstop only misbehaves under motor current, the fault is the ground
   or the shielding, not the sensor.
5. **Homing.** Seek toward the endstop at low speed, stop on the edge, back off ~2 mm, re-approach
   at a tenth of the speed, then zero the step counter. Do this ten times from ten different
   starting positions and record where it stops. Repeatability is what the whole open-loop design
   rests on (design §3), so measure it now rather than believing it.
6. **Angle check.** From home, command a known step count and measure the angle the joint actually
   turned. **Not with a digital angle gauge:** every revolute joint here turns about a vertical
   `0 0 1` axis, and an inclinometer measures tilt against gravity, so it reads the same value
   at every joint angle. Fix a long pointer to the link instead:
   - **Full turn**, if the endstop and cabling allow one: line the pointer up on a fixed mark and
     count the steps until it returns to the mark. That count is steps per revolution, and since
     the measurement error is spread over 2π this is the most accurate method available.
   - **Chord**, otherwise: mark the pointer tip on paper at both ends of the move, measure the
     distance `c` between the marks with calipers, and take `θ = 2·asin(c / 2r)`. At r = 300 mm,
     0.1 mm of caliper resolution is about 0.02°, finer than a 0.1° gauge.

   The error is systematic and constant — that is exactly the term the calibration in design §4
   and Task 7 of the host plan exists to absorb. Write the measured steps-per-radian down; it is
   a calibration input, not a number to derive from CAD.

Only after step 6 repeats cleanly should driver #2 go in.

**Firmware for phase 1 does not need the binary protocol.** A text REPL over USB CDC
(`m1 1000`, `home 1`, `es 1`) is faster to debug with a scope in one hand and costs nothing later
— the framed protocol in Task 1 of the host plan is a separate layer, and Task 2's pty stub
already lets the host side proceed against a fake MCU in parallel. Do not let the two block each
other.

## Going from one axis to four

Nothing changes in the protocol: the command frame already carries all five axes, so commanding
four motors is one frame per 100 Hz tick, not four messages. Nothing changes on the host either —
`arm_group_controller` already lists the four joints and `bizon_system_interface.xacro:20-23`
already declares their interfaces. What is new is per-joint `steps_per_rad`, and three real
concerns.

**Coordination is the host's job, already done.** MoveIt plans a joint trajectory; the JTC samples
it at 100 Hz and emits one setpoint per joint per cycle, all valid at the same instant. The
trajectory *is* the synchronisation.

**Every axis must finish on the same tick boundary.** The naive MCU implementation runs each axis
toward its own target at maximum velocity, so the short axis arrives early and the path between
waypoints is not straight in joint space. Set each axis's step rate to
`delta_steps / tick_period` instead, so all four land exactly at the end of the 10 ms window. The
MCU's velocity and acceleration limits are then a safety clamp, not the motion profile — MoveIt
already produced a profile that respects the limits.

**A late or lost frame must not stop dead or run away.** Continue at the last commanded velocity
for a bounded number of ticks, then decelerate to a stop and raise the fault.
`link_timeout_cycles` is that bound; `read()` returns ERROR and the fault chain in design §2 takes
over.

Load, for reference: four independent hardware timers, four ARR reloads per tick — 400 register
writes per second, nothing. Power is the part that scales: budget roughly a third to a half of
each motor's RMS phase current off the supply at 24 V, so four axes at 0.7 A RMS sit comfortably
on a 24 V / 5 A bench supply.

**The test that matters when driver #2 goes in** is not "does the new axis move". It is:

> Does axis 1 still home to the same place while axis 2 is running?

The failure you will actually hit is motor noise coupling into the endstop lines and the shared
ground. So repeat phase-1 steps 4 and 5 — endstop under motor current, then ten homing runs — with
**every** axis moving, not just the new one, each time a driver is added.

**Do not run ros2_control against a partially populated machine.** The JTC commands all four
joints regardless; the MCU will happily step a driver that has no motor, the counter advances,
and `read()` stops reporting reality — which is the one rule design §2 says everything else rests
on. Either keep an axis-present mask in firmware, or stay on the REPL until all four are wired.

The gripper is not part of this: the arm and the gripper never move simultaneously (see
`CLAUDE.md`), so the frame carries the field but the behavior tree never commands both at once.

## If the gripper becomes a servo

Consequences, none of them blocking, all of them worth deciding before the board is laid out:

- Four drivers, one UART bus, addresses 0-3. The two-bus split in design §1 goes away.
- The force limit from reduced run current goes away with it. A hobby servo stalls by buzzing and
  stripping gears. Either add current sense on the servo rail, or use a bus servo
  (Feetech STS3215, Dynamixel XL330) whose UART reports position and load.
- A plain PWM servo reports nothing, so the hand joint's `state_interfaces` must echo its command
  — the one deliberate exception to the "never echo `write()` into `read()`" rule in design §2.
  Document it where the exception lives, and stop expecting `hand_group_controller` to catch
  anything.
- The wire protocol's fifth `int32` stops being steps. Redefine it as servo pulse width in
  microseconds, with `homed_bits` bit 4 always set and `stall_bits` bit 4 always clear.
- PB8's 3.3 V logic drives most servos directly, but check yours; some need a level shifter.
