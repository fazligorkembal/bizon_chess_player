# Hardware Bring-Up Design — Single Arm vs. Human Opponent

**Date:** 2026-08-21
**Status:** Approved in conversation; sections 3-7 recorded as decisions rather than reviewed line by line.
**Supersedes nothing.** Complements `docs/superpowers/plans/2026-08-19-pre-hardware-refactor.md`, which
prepares the stack *in simulation*. This document covers the move to physical hardware.

## Goal

Run one physical Bizon arm against a human opponent, using the existing perception,
decision, and behavior-tree stack unchanged wherever possible.

## Scope Decisions

| Decision | Choice | Rationale |
|---|---|---|
| Robot count | One arm; human plays the other side | Half the hardware, one calibration, far easier debugging. The `wait` path added in Task 3 already models an opponent the robot does not control. |
| Motion stack | Keep MoveIt2 **and** ros2_control; add a third hardware branch | Chosen by the maintainer over the leaner alternatives, to keep the door open for Task 7 (planning-scene collision objects). Imposes the fault-chain requirement in §2. |
| Actuators | 4x NEMA 17 (arm) + 1x NEMA 17 (gripper), TMC2209 drivers, endstops on every axis | Given. |
| MCU | BlackPill (STM32F411CEU6), USB CDC to the host | Given. Ample: 100 MHz Cortex-M4F versus Marlin running five steppers on a 16 MHz AVR. |
| Arm geometry | Fixed. `l1 = 0.29 m`, `l2 = 0.18 m`, direct drive, no reduction, no encoder | Given. Drives the accuracy strategy in §4. |
| Gripper fingers | Redesignable | Enables the self-centering profile in §5, the single largest tolerance win available. |
| Host | Jetson | Assumption, from the pre-hardware plan's stated goal. |

## 1. Hardware / Software Boundary

The MCU is a dumb, fast, safe axis controller. **It holds no chess knowledge and no
kinematics.** Everything else stays on the host, which has no real-time requirement.

**BlackPill owns:**
- Step/dir generation for five axes, one hardware timer per axis (TIM1-TIM5 are all 4-channel).
- Velocity- and acceleration-limited interpolation toward the most recent commanded position.
- Endstop monitoring and the homing routine.
- TMC2209 UART configuration: microstepping, run/hold current, StealthChop, StallGuard threshold.
- StallGuard monitoring, raised as a fault flag.
- Limit handling: an endstop triggering outside the homing routine asserts TMC `EN`
  inactive immediately and latches the fault until the host clears it. The endstops are
  the machine's only physical stop input — there is no separate emergency-stop button.

**Host owns:** perception, Stockfish, behavior tree, MoveIt, ros2_control, calibration.

### Step-rate budget

`bizon2rev1`'s URDF velocity limit is 12.56 rad/s (2 rev/s) — a placeholder, absurdly fast for
this application. Even taken at face value: 200 steps/rev at 1/16 microstepping is 3200
steps/rev, so 6400 steps/s per axis, roughly 32 kstep/s with any modest reduction. Five axes
aggregate well under what one timer per axis handles on this part.

### Protocol

Framed binary over USB CDC at 100 Hz, matching `update_rate: 100` in
`bizon2_full_ros2_controllers.yaml`.

```
Host -> MCU : sync | seq | 5 x int32 target_steps | flags | crc16      (~32 B)
MCU  -> Host: sync | seq | 5 x int32 actual_steps | status | crc16     (~32 B)

flags  : enable, home_request, gripper_current_level
status : homed_bits | endstop_bits | stall_bits | limit_hit | fault_code
```

About 3.2 kB/s each way — negligible for USB CDC.

Positions travel as **integer steps, not radians**: no floating-point drift, and the MCU never
needs the kinematic model. The steps/unit conversion lives in the hardware interface, fed by
per-joint `hardware_parameters` in the xacro.

`bizon_system_interface.hpp` already declares exactly the two converters this needs, and neither
is defined yet:

```cpp
int radian_angle_to_step_converter(double angle_rad, int micro_steps, double step_angles_deg, double gear_ratio);
int distance_to_step_converter(double distance_m, int micro_steps, double step_angles_deg, double gear_ratio);
```

The slot is pre-shaped throughout: `// todo: add serialComm.h` in the header, `// todo: add
serialComm uart_comm;` as a member, and the `ros2_control_hardware_type` dispatch in `on_init`
already switches on a string with `rviz` and `isaac` filled in.

### TMC2209 UART addressing

MS1/MS2 give only four addresses per bus. Five drivers therefore need two UART buses. The F411
has USART1, USART2 and USART6 available, with USB CDC on a separate peripheral — no conflict,
but the bus split must be fixed before the board is laid out.

## 2. The Fault Chain

Because the design keeps MoveIt, and MoveIt knows only what `read()` reports, this chain is not
optional:

```
MCU detects stall / endstop / limit violation
  -> stops; the step counter stops advancing
  -> read() reports the real counter
  -> JTC position error exceeds its trajectory tolerance
  -> FollowJointTrajectory aborts
  -> ArmPlugin returns FAILED
  -> RecoveryNode runs release-before-retreat
```

Every link is currently missing.

**`state_interfaces` must report the MCU's step counter and must never echo the command
written to it.** This is the single most important rule of the bring-up. If `read()` echoes
`write()`, tracking is perfect by construction, JTC never aborts, RecoveryNode never runs, and
MoveIt reports success for every motion including the ones that crashed. Every safety mechanism
above this line becomes decorative. Note that `read_rviz_sensors()` does exactly this echo — it
is correct for RViz visualisation and would be catastrophic if copied into the serial branch.

`bizon2_full_ros2_controllers.yaml` currently declares no tolerances at all. Required:

```yaml
constraints:
  stopped_velocity_tolerance: 0.02
  goal_time: 0.5
  bizon2rev1:  { trajectory: 0.05,  goal: 0.01  }
  bizon2pris1: { trajectory: 0.005, goal: 0.002 }
  bizon2rev2:  { trajectory: 0.05,  goal: 0.01  }
  bizon2rev3:  { trajectory: 0.10,  goal: 0.02  }
```

These are starting values to be tightened once real tracking error is measured.

## 3. Homing and the Open-Loop Contract

There is no encoder. Two consequences, with different remedies.

**Systematic error** — `link_l1`, `link_l2`, `robot_base_offset_x`, `box_size` will not match the
built robot exactly. Large (easily 5-10 mm) but constant, and therefore calibratable.

**Random error** — microstep accuracy and load-angle variation, roughly 0.1 to 0.5 of a full
step under load.

Steppers are far more *repeatable* than they are *accurate*. That asymmetry is what makes fixed
direct-drive geometry workable: calibrate the systematic part away and only the random part
remains.

- Home all axes on `on_activate()`. MoveIt must not plan before homing completes.
- Re-home every N moves. A chess robot has minutes between moves; homing costs seconds. This is
  the only defence against accumulated skipped steps.
- Do not use StallGuard for homing. StallGuard4 works only in StealthChop and is unreliable at
  low speed. Endstops are already in the design; use them. Keep StallGuard for collision
  detection, where it is the only torque-limit substitute available.

## 4. Geometry Unification and Calibration

`bizon_chess::worldToJointAngles` already rejects unreachable targets:

```cpp
if (!(d >= -1.0 && d <= 1.0)) { return false; }
```

The running code does not use it. `make_decision_client_node.cpp` still calls its own
`calculate_joint_angles`, which feeds `sqrtf(1 - D*D)` unguarded and emits NaN joint targets for
any out-of-reach square. On hardware that is a corrupt position command to the drivers.

Deleting the duplicate and calling `bizon_chess` is therefore a fix, not a refactor. It is also
what makes calibration possible: `RobotParams` was written for this, and says so.

Calibration replaces the analytic constants with measured values: teach the four corner squares,
fit the model, derive all 64. `RobotParams` moves from compile-time defaults to ROS parameters
loaded from a per-robot calibration file.

## 5. Gripper

The URDF declares three finger joints and `hand_group` commands all three. One stepper driving
all three fingers mechanically means the URDF needs `mimic` joints and ros2_control needs a
single command interface. This is a real change, not a rename.

**Force control without a force sensor:** TMC2209 `IHOLD_IRUN` sets run current over UART. Close
the gripper at reduced current, and the motor stalls harmlessly against a rigid piece rather than
crushing it or burning the driver. This is what closes finding F1 — the stall that currently
surfaces as a `FollowJointTrajectory` failure. A stepper gripper is a *better* choice than a
hobby servo here precisely because of this knob.

**Self-centering fingers:** tapered profiles that draw a cylindrical piece base toward the
centre as they close. This widens the placement tolerance roughly threefold and re-centres the
piece on every grasp, which also breaks error accumulation. Given fixed arm geometry, this is
the largest single accuracy win available and it touches only the finger STL.

## 6. Safety

**There is no emergency-stop button on this machine.** The only physical stop inputs are the
per-axis endstops; everything else is software or the power switch. Do not write code or
documentation that assumes an independent hardware E-stop exists — nothing in the design has
ever provided one, and treating the endstops as if they were one is how the two got confused.

- **Endstops are the physical limit input.** The MCU stops the axis, asserts TMC2209 `EN`
  inactive and latches the fault; the host reads `limit_hit` in the status word and `read()`
  returns ERROR, which walks the fault chain in §2 up to `RecoveryNode`.
- **The stop path in software is the lifecycle manager.** `PAUSE` deactivates `behavior_server`,
  `IsSystemActive` fails, and the tree stops issuing arm goals (Task 6 of the pre-hardware plan,
  landed in `7e2226b` / `09e1489`). This is a stop, not an emergency stop: it takes a BT tick to
  act, it cannot cut driver current, and it does nothing if the host itself is wedged.
- **The Z axis falls when the drivers are disabled.** With all revolute axes on a vertical `0 0 1`
  axis, gravity loads only `bizon2pris1`. This applies to any driver disable — a latched fault, a
  power cut, or a deliberate shutdown. **Settled (2026-10-07): the Z axis uses a T8 leadscrew
  with a 2 mm lead, which self-locks**, so Z holds position through any driver disable. Keep it
  that way: an 8 mm lead does not self-lock, and swapping one in would need a brake or a circuit
  that keeps Z energised while the other axes drop — expensive to retrofit.
- **If an emergency stop is ever wanted**, it is added hardware, not a code change: a button
  breaking TMC2209 `EN` in copper, independent of firmware, with a status line the MCU can latch
  on. Scope it as its own task rather than assuming it into an existing one.

## 7. Playing a Human

The decision node already reports `move_type: "wait"` and the tree already skips the move subtree
on it, so an opponent the robot does not control is modelled correctly. Two additions:

- **FEN debounce.** The camera will capture the board mid-move, with a hand across it. Require N
  consecutive identical board readings before accepting a FEN. This also serves as the safety
  interlock: the arm only moves after the board has been stable, which means the hand is clear.
- **Move-history file.** `last_moves_path` is hardcoded to `/home/user/Documents/bizon_chess_player`
  and there is no reset between games — a stale file starts the next game from the wrong
  position. Make it a ROS parameter with an explicit new-game reset.

The `killking` behaviour physically removes the opponent king from the board. Against a human it
should place the piece, not sweep it. Low priority, but do not ship it as a sweep.

## Out of Scope

Second arm, arm-to-arm collision avoidance, CAN or EtherCAT (the F411 has no CAN peripheral),
micro-ROS on the MCU (workable at 128 KB RAM but buys nothing — the `SystemInterface` already
lives on the host).
