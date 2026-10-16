# Joint0 Bench, Stage A — Bench Runbook

What to do at the bench, in order, for stage A: build the trial board, bring it up, and run
acceptance tests A1-A7. Design and pass criteria:
`docs/superpowers/specs/2026-10-16-joint0-bench-design.md` §6. Board files:
`hardware/joint_bench/` (see its README).

Instruments: multimeter, logic analyzer (8 channels, PulseView/sigrok), ST-Link.

## 1. Before soldering

1. **Count the trial board's holes** along both edges. If they are not 34 × 22, correct
   `board: {cols, rows}` in `hardware/joint_bench/trial_6x9.yaml` and regenerate
   (`python3 tools/kicad_gen/generate.py`). Moving a part means editing its `at:`.
2. **Measure the BlackPill's row spacing** with calipers, from the centre of a pin in one row
   to the centre of the pin opposite. 15.24 mm is 6 holes. Anything else: set
   `blackpill_row_spacing` and regenerate.
3. **Compare the BlackPill's silkscreen with the drawing.** Hold the board USB end left, as in
   `exports/trial_6x9_top.pdf`. The top row must read `B12 B13 … 5V G 3V3` and the bottom row
   `VB C13 … 3V3 G 5V`. If yours differs, stop: `BLACKPILL_PINS` in
   `tools/kicad_gen/kicad_gen/design.py` is wrong for your board and must be fixed first.
4. **Print** `exports/trial_6x9_top.pdf` and `exports/trial_6x9_bottom_mirrored.pdf` at 100 %
   ("actual size", no fit-to-page). Lay the board on the top print: every pad must sit on a
   hole. If not, the printer scaled it.

## 2. Soldering

Order: female headers for the BlackPill first (seat the BlackPill in them while soldering so
they stay straight, then remove it), then the pin headers (J1, J2, J3, JP1, JP2), then R1
and R2. Then the wires, from the bottom (mirrored) drawing, one net at a time, ticking
`hardware/joint_bench/trial_6x9_checklist.md` as you go.

## 3. Multimeter checks, board unpowered, BlackPill out

1. **Continuity:** every line in the checklist's continuity section must beep end to end.
2. **Shorts:** nothing in the shorts section may beep. Do the power pairs first.
3. **GY-87 pull-ups:** with the GY-87 unpowered and disconnected, measure SCL to its 3.3 V
   pin. If it reads 2.7 kΩ or less, leave JP1 and JP2 open (no jumper caps); otherwise fit
   both caps. Write down the value.

## 4. First power

1. Seat the BlackPill. USB only; nothing on J1.
2. Measure J3: 3V3 to GND 3.2-3.4 V, 5V to GND 4.75-5.25 V.

## 5. Flash

1. ST-Link to the BlackPill's SWD header (3V3, SWDIO, SWCLK, GND).
2. Build on the development machine from `firmware/joint_bench` (CubeIDE, or
   `cmake --preset Debug && cmake --build build/Debug`) and flash `joint_bench.elf` with
   STM32CubeProgrammer or OpenOCD.
3. The LED blinks fast (no IMU yet). The PC sees a USB serial port (VID:PID 0483:5740).

## 6. Connect the IMU

1. Unplug USB. Wire the GY-87 cable: J1 VCC → VCC_IN, GND → GND, SCL → SCL, SDA → SDA,
   INT → INTA. On the GY-87 itself, bridge FSYNC to GND.
2. Twist SCL with GND and SDA with VCC; route the cable away from where the motor cable will
   run. Put the GY-87 on the wooden platform beside the chessboard, flat, and fix it down
   (double-sided tape or a screw): a loose sensor measures its own rattle.
3. Plug USB. The LED blinks slowly.

## 7. Software on the bench machine

    python3 -m pip install --user -r tools/requirements.txt
    cd tools/joint_bench
    python3 -m joint_bench console        # type: ver, i2cscan, imu info, stats; Ctrl-D quits

Recordings land in the repository root's `debug/bench/<timestamp>_<label>/` (from
`tools/joint_bench` that is `../../debug/bench/`). `record` analyses each recording when
it finishes; `python3 -m joint_bench analyze <dir>` re-runs that.

## 8. Logic analyzer setup

Clip J2: CH0-CH7 to analyzer channels 0-7, J2 GND to analyzer GND. Sample at 4 MHz or more
(400 kHz I2C needs at least 10 samples per bit). In PulseView add an I2C decoder with SCL on
CH4 and SDA on CH5.

## 9. Acceptance tests

Record each result in the table at the end.

**A1 — identify.** In `console`: `i2cscan`, then `imu info`.
Pass: `i2cscan` lists 68 (and 77, the GY-87's BMP180); `imu info` reports a chip. Note
`chip=` and `who=` and add them to the design doc's Open Items.

**A5 — noise floor** (run before A2, it provides the data). Nobody touches the table:

    python3 -m joint_bench record -s 60 -l rest

Pass: the summary prints a horizontal noise figure. Keep the directory name; A6 uses it.

**A2 — static.** From the `rest` summary. Pass: the line `PASS  A2 gravity within 5%` (the summary puts two spaces after PASS). Note the
vertical axis it reports.

**A3 and A4 — ten minutes, cable at final length.** With the analyzer capturing a few seconds
of CH4/CH5 at some point during the run:

    python3 -m joint_bench record -s 600 -l a3_a4

Pass: `PASS  A3 no missing samples`, `PASS  A3 interval within 2%`, `PASS  A4 no I2C errors`.
In the capture, each transaction is: START, 0xD0, 0x3B, repeated START, 0xD1, 14 data bytes
with NACK on the last, STOP, and no NACK anywhere else.

**A6 — tap test.** For each of three points (beside the board, beside the robot base, a
corner), strike the platform five times with a pencil, about two seconds apart:

    python3 -m joint_bench record -s 15 -l tap_board
    python3 -m joint_bench record -s 15 -l tap_base
    python3 -m joint_bench record -s 15 -l tap_corner
    python3 -m joint_bench analyze ../../debug/bench/<tap dir> \
        --noise-from ../../debug/bench/<rest dir>

Pass: five taps found per recording, each with a frequency and damping. Write down the
frequencies that recur: they are the platform modes stage C must not excite.

**A7 — timestamp alignment.** Analyzer on CH6 (IMU_INT) and CH7 (MARK), capture running:

    python3 -m joint_bench record -s 8 -l a7 --marks 5

For each of the five MARK edges, put cursors on it and on the next rising IMU_INT edge and
read the interval. Compare with the summary's `mark:a7-N: next sample X us later` lines.
Pass: every pair agrees within 20 µs.

## 10. Robustness checks

These pin down failures the unit tests cannot reach.

1. **No IMU at power-up.** Unplug USB, disconnect J1, plug USB: fast blink, and `imu info`
   answers `ERR imu: not found` without the firmware hanging. Reconnect J1 with power on
   (if the BlackPill resets, that is the GY-87's inrush, not a bug), then `imu info`
   succeeds and the LED slows down.
2. **Host stops reading.** In `console`, `stream on`, then suspend it with Ctrl-Z for about
   five seconds, `fg`, `stream off`, `stats`. Pass: `tx_drop` is above zero, the firmware
   did not reset (`samples` kept counting) and the console works normally.
3. **USB drops out mid-stream.** Power the BlackPill from the ST-Link's 3.3 V as well, so it
   stays up without USB. `stream on`, unplug USB, wait five seconds, plug it back, start
   `console` again. Pass: `stats` answers and `stream on` streams again. This exercises the
   branch in `usb_pump()` that drops a transfer the unplug left unfinished; without it the
   port would stay silent until reset.

## Results

| Test | Date | Result | Notes (values, directory names) |
|---|---|---|---|
| A1 | | | |
| A2 | | | |
| A3 | | | |
| A4 | | | |
| A5 | | | |
| A6 | | | |
| A7 | | | |
| R1 no IMU at power-up | | | |
| R2 host stops reading | | | |
| R3 USB drops out | | | |
