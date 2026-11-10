# J1 Perfboard — Design

Date: 2026-11-10. Status: reviewed by Codex (DÜZELTMEYLE ONAY) and approved by the user on 2026-11-10.

## Intent

Move joint 1 (`bizon2rev1`, "joint0" on the bench) off the breadboard onto a soldered board,
so the motor can be mounted on the robot and run reliably. On the breadboard tonight the
UART dropped replies after long moves, a BlackPill died while a heatsink was being fitted, and
connections had to be reseated more than once. The board carries one axis now; a larger board
for four steppers and a servo gripper follows later, with the same pin allocation, so the
firmware does not change between them.

## Goals

- One soldered 7 x 9 cm perfboard (2.54 mm pitch), in a box on the robot's base board beside
  the robot, not on or inside the robot.
- BlackPill and the TMC2209 module on female headers, so either is a swap, not a desolder.
- Power zone and logic zone kept apart, with a star ground, so motor return current does not
  run through the logic ground.
- Uses exactly the pins of the full four-axis allocation
  (`docs/hardware/f411-tmc2209-wiring.md`, "Full allocation"). Nothing on this board takes a
  pin another axis, endstop or the servo needs later.
- Connectors for motor 1, endstop 1 and the bench IMU, and the bench's VM sense input.

## Non-goals

- Drivers 2-4, endstops 2-4 and the servo. Their pins stay reserved; they go on the larger board.
- A fuse, a TVS diode, reverse-polarity protection. The bench supply's current limit covers the
  trials; protection is revisited for the larger board.
- A PCB. This is a hand-soldered perfboard.

## Anti-goals

- A layout that forces a different pin allocation later.
- Motor current or the 1000 uF charging current flowing through the BlackPill's ground pin.
- A board on which the module can be plugged in reversed without it being obvious.

## Constraints and facts this rests on

- BlackPill: STM32F401xD/E (WeAct), 53 x 21 mm, two 20-pin rows 15.24 mm (6 holes) apart. USB-C
  on one short end, SWD header on the other.
- TMC2209 module: Twotrees "TMC2209 V2.0" on Makerbase's MKS TMC2209 V2.0_001 layout, two 8-pin
  rows 12.7 mm (5 holes) apart. Sense resistors R110. On-module: 20 kohm EN pull-up to VIO
  (R2), 20 kohm PDN_UART pull-down (R1). Pins, confirmed unpowered on 2026-11-10, with the
  module copper side up, pot and three holes at the left:
  - top row, left to right: VM, GND, 2B, 2A, 1A, 1B, VIO, GND
  - bottom row, left to right: EN, MS1, MS2, (UART via R8), PDN_UART, (CLK via R5), STEP, DIR
  - three holes by the pot: VREF (the one beside EN), INDEX, DIAG
- The module is fitted copper side up, so its heatsink can go on later. The heatsink is bare
  aluminium on GND, an edge away from the pins: insulating pad, every supply off.
- Motor: 17HS4401S, about 1.5-1.7 A rated; runs at IRUN <= 16 with VREF 0.50 V for now.
- Endstop: four ready-made optical endstops (Robotistan "Kablolu ve Soketli Optik Endstop"),
  3-wire, red VCC, black GND, white signal, 1 m cable, 3-pin JST-XH socket on the sensor. Output
  type and active level not documented; handled by the pull-up and a firmware invert flag.
- Supply: Korad KA3005D, 12 V for now, current limit set per test. Board parts rated for 24 V.
- Firmware already drives: EN PB5 (initial high), STEP PA8, DIR PB12, UART PA9 half-duplex.

## Pins used on this board

All from the full allocation. "Reserved" pins are not connected on this board.

| BlackPill pin | Net | Goes to |
|---|---|---|
| A8 | J1 STEP | module STEP |
| B12 | J1 DIR | module DIR |
| B5 | EN (all drivers later) | module EN |
| A9 | TMC UART | 1 kohm, then module PDN_UART |
| B10 | Endstop 1 | endstop filter, then J_ES1 signal |
| A5 | DIAG 1 | optional: module DIAG hole (flying lead) |
| A3 | VM sense | 100k/10k divider, 100 nF |
| B6, B7, B8 | IMU INT, SDA, SCL | J_IMU |
| 3V3 | logic 3.3 V | module VIO, UART pull-up, endstop VCC, endstop pull-up |
| 5V | USB 5 V | J_IMU VCC (GY-521 has its own regulator) |
| G | logic ground | logic ground bus, one wire to the star point |

Reserved, not connected here: A15, B13 (J2); B0, B14 (J3); A1, B15 (J4); A2 (servo); B3, B4,
B9 (endstops 2-4); A6, A7, B1 (DIAG 2-4); A4 (MARK). Free after the full system: A0, A10.

## Circuit

### Power zone

- **J_PWR**, 2-way screw terminal: pin 1 VM (+), pin 2 GND.
- **C_BULK** 1000 uF / 63 V across J_PWR, legs straight into the terminal's rows. Its negative
  leg is the **star point**.
- **Module socket**, 2 x 8 female headers, 5 holes apart.
  - VM pin to J_PWR + with a short, thick wire (AWG20 or heavier).
  - **Both** module GND pins (beside VM and beside VIO) to the star point with short, thick
    wires. They are one net inside the module, so the module's GND is power ground; the logic
    bus must not touch either GND pin, or the module becomes a second path between the two
    grounds (Codex review B1).
  - **C_MOD** 100 uF / 50 V low-ESR and **C_HF** 100 nF directly across the module's VM and
    power GND socket pins.
- **J_MOT**, 4-pin male header (or JST-XH 4-pin): 1 = 2B, 2 = 2A, 3 = 1A, 4 = 1B, in the module's
  own order, so each coil pair is adjacent: pins 1-2 one coil, pins 3-4 the other. Thick wires.
- **VM sense:** VM, 100 kohm, node, 10 kohm to logic ground; 100 nF node to logic ground; node
  to A3. 1.09 V at 12 V, 2.18 V at 24 V.

### Logic zone

- **BlackPill socket**, 2 x 20 female headers, 6 holes apart, USB-C end at a board edge so the
  cable plugs in with the board in its box.
- **Logic ground bus**: BlackPill G, endstop GND, IMU GND, the VM-sense 10 kohm and its 100 nF.
  One wire from this bus to the star point; no other path between them. Not the module's GND
  pins (see the power zone).
- Module **VIO** to 3V3. Module **MS1, MS2** to logic ground (UART address 0). Module **EN** to
  B5; the module's own 20 kohm pull-up keeps it high while the MCU resets.
- **UART:** A9, 1 kohm, node at the module's PDN_UART pin; 4.7 kohm from that node to 3V3. The
  module's second UART pin (via R8) and CLK pin stay unconnected.
- **STEP** to A8, **DIR** to B12, direct.
- **J_ES1**, 3-pin male header (or JST-XH 3-pin), in the sensor's own order **S, V, G**
  (signal, VCC, GND), as printed beside the sensor's socket (read by the user on 2026-11-10), so
  the supplied cable plugs in straight. Check the cable is straight-through (sensor S to header
  S) before the first power-up. Mark the header.
  - 10 kohm from SIG to 3V3.
  - SIG to 1 kohm, node to B10.
  - 100 nF from the node to logic ground. With an open-collector output that is about 1.1 ms
    rising and 0.1 ms falling, which the firmware debounce and the homing speed must allow for.
  - A broken cable reads as the pulled-up level; this circuit does not detect it.
- **J_IMU**, 5-pin male header: 5V, GND, SCL (B8), SDA (B7), INT (B6), for the GY-521 cable.
- **DIAG 1 (optional):** a wire from the module's DIAG hole to A5. Not needed for the first
  runs.

### Power order and handling rules

- Power up USB first (BlackPill and 3.3 V), then VM. Power down VM first, measure VM at the
  module near 0 V, then USB. VM on with the BlackPill unpowered back-feeds A3 through the divider
  and the logic inputs through the module (Codex review B4).
- Before plugging or unplugging the module, the motor, or fitting the heatsink: every supply
  off, USB included, and VM measured discharged.
- Mark the module socket's VM corner and the matching corner of the module permanently. Plugged
  in 180 degrees round, the socket's VM lands on the module's DIR pin.
- The supply's current limit is a test aid, not protection: it does not stop reverse polarity,
  regenerative overvoltage or the local capacitors' short-circuit current. First acceptance at
  12 V and the current tonight's tests used. Before 24 V, measure the peak VM at the module
  during braking and disable; the TMC2209's limit is 29 V operating, 32 V absolute (datasheet
  p.75), whatever the capacitors are rated for.

### Layout (left to the builder, within these rules)

- BlackPill along one long edge, USB-C at a short edge. Module socket on the opposite half.
  J_PWR, C_BULK and J_MOT on the module's side of the board, along the far edge; J_ES1 and J_IMU
  on the BlackPill's side.
- Thick wires only in the power zone: J_PWR, C_BULK, module VM and power GND, J_MOT.
- Nothing taller than the module's top surface beside its copper side, so a heatsink fits.

## Parts

| Part | Qty |
|---|---|
| 7 x 9 cm perfboard, 2.54 mm | 1 |
| 20-pin female header (or cut strips) | 2 |
| 8-pin female header | 2 |
| 2-way screw terminal, 5.08 mm | 1 |
| 1000 uF / 63 V electrolytic (ordered Koshin PKLH-063V102MJ355) | 1 |
| 100 uF / 50 V low-ESR (ordered Koshin PKLH-050V101MG120) | 1 |
| 100 nF ceramic 50 V (ordered Kingtronics) | 3 (module, endstop filter, divider) |
| 1 kohm | 2 (UART, endstop) |
| 4.7 kohm | 1 (UART pull-up) |
| 10 kohm | 2 (endstop pull-up, divider) |
| 100 kohm | 1 (divider) |
| Male pin header: 4-pin, 3-pin, 5-pin | 1 each |
| Wire: AWG20 or heavier for power; thin for signals | |

## Bring-up on the board

1. Unpowered, nothing plugged in: continuity check VM-GND, 3V3-GND, 5V-GND, VM-3V3 (no steady
   beep), and each net in the pin table above.
2. BlackPill only, on USB: `ver`, then `tmc read 0x06` gives `no reply` (no module).
3. All power off; module in. USB, then 12 V at 0.30 A, motor and endstop unplugged: phase 1
   step 1 again (`tmc read 0x06` = 0x21000041, GCONF write counted). Check VREF is still 0.50 V.
4. Endstop plugged in, VM off: measure the sensor's current from 3.3 V and SIG at J_ES1 with the
   slot empty and blocked, to learn its output type and levels. If it does not work at 3.3 V,
   stop and redesign its input before trying 5 V. Then phase 1 step 2 (level flips with a card
   in the slot, steady at rest).
5. VM off and measured discharged, motor plugged in: phase 1 step 3 (`move 1600 400` and back).
6. On the robot: phase 1 steps 4-6 from the wiring guide.

Firmware for step 4 exists: `es` reads endstop 1 (B10, pull-up, 2 ms debounce, raw-edge count)
and `es watch on` sends every accepted change as an event line. Still to come: an invert flag
once the sensor's active level is known, and homing. Not part of this board's design.

## Left to the builder

Exact hole positions; header colours; whether to use JST-XH or plain pin headers for J_MOT and
J_ES1, as long as the pin order holds.

## Decided by the user

7 x 9 cm perfboard; board in a box beside the robot on the base board; one motor now, four
steppers and a servo later on a larger board; the four optical endstops above; the endstop is
tested on its own before the board is built.

## Claude's calls, for checking

- Connectors are 2.54 mm headers (or JST-XH) rather than screw terminals for the motor and
  endstop.
- VM sense divider and the IMU header are on this board, though not needed tomorrow.
- No fuse for now.
- EN relies on the module's 20 kohm pull-up; the 10 kohm in the wiring guide is for the
  four-driver board.
- Endstop powered from 3.3 V. If it needs 5 V, its input is redesigned rather than just moved
  to 5 V, because a 5 V push-pull output would push current into 3.3 V through the 10 kohm.
- The module's 5VOUT capacitor (C2) is 100 nF on the Makerbase schematic, where the datasheet
  recommends 2.2-4.7 uF (p.11). Left as it is: the module is used as bought. Its real value is
  unknown; it is a suspect, not a known cause, if the UART keeps dropping replies after moves.
- GY-521 header: the module's own regulator feeds its SDA/SCL pull-ups, as used in stage A at 5 V.
- PB3 and PB4 (endstops 2-3) are JTAG pins at reset and PA15 (J2 STEP) is JTDI; the firmware
  configures them explicitly and keeps SWO off. PA0 is free only if the board's KEY button does
  not load it.
