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
3. **GY-521 pull-ups:** the module has no 3.3 V pin. With it unpowered and disconnected, read
   the code on its SCL pull-up resistor (472 is 4.7 kΩ, 222 is 2.2 kΩ), or measure SCL to the
   3.3 V output of its regulator. If it is 2.7 kΩ or less, leave JP1 and JP2 open (no jumper
   caps); otherwise fit both caps. Write down the value.

## 4. First power

1. Seat the BlackPill. USB only; nothing on J1.
2. Measure J3: 3V3 to GND 3.2-3.4 V, 5V to GND 4.75-5.25 V.

## 5. Flash

1. ST-Link to the BlackPill's SWD header: SWDIO, SWCLK and GND only. Leave the ST-Link's
   3.3 V pin unconnected: the BlackPill is already powered from USB, and two 3.3 V supplies
   would fight each other.
2. Build the project in STM32CubeIDE 1.18.1 and flash it with *Run As → STM32 C/C++
   Application*.
3. The LED blinks fast (no IMU yet). The PC sees a USB serial port (VID:PID 0483:5740).

`codes_stm32.launch` runs SWD at 100 kHz. Over jumper wires, reads at 480 kHz and above sometimes
returned wrong bytes. A flash takes about 25 s at this speed.

## 6. Connect the IMU

1. Unplug USB. Wire the GY-521 cable: J1 VCC → VCC, GND → GND, SCL → SCL, SDA → SDA,
   INT → INT. Leave XDA, XCL and AD0 on the GY-521 unconnected.
2. Twist SCL with GND and SDA with VCC; route the cable away from where the motor cable will
   run. Put the GY-521 on the wooden platform beside the chessboard, flat, with its X arrow
   along the files (a to h) and its Y arrow along the ranks (1 to 8), and fix it down
   (double-sided tape or a screw): a loose sensor measures its own rattle.
3. Plug USB. The LED blinks slowly.

Without the trial board, wire the module straight to the BlackPill with short jumpers and no
extra pull-ups: INT to PB6, SDA to PB7, SCL to PB8, VCC to 5V, GND to G, all on one row.

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
Pass: `i2cscan` lists only 68 (the GY-521 has no second chip); `imu info` reports a chip.
`ERR imu: bus busy` means a line is held low: check the wiring first. SCL on the module's INT pin
does exactly this, because INT idles low. Note
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

**A8 — direction.** Three pencil strikes on the platform's edge along X, three along Y, then
three sharp twists of the platform about the vertical by hand, about two seconds apart:

    python3 -m joint_bench record -s 10 -l dir_x
    python3 -m joint_bench record -s 10 -l dir_y
    python3 -m joint_bench record -s 10 -l dir_twist

Pass: every `tap at` line says `x`, `y` and `twist` in the matching recording. If twists and
strikes do not separate, note each line's `twist` over `peak`: `TWIST_DPS_PER_G` in
`tools/joint_bench/joint_bench/analysis.py` is provisional and belongs between the two groups.

## 10. Robustness checks

These pin down failures the unit tests cannot reach.

1. **No IMU at power-up.** Unplug USB, disconnect J1, plug USB: fast blink, and `imu info`
   answers `ERR imu: not found` without the firmware hanging. Reconnect J1 with power on
   (if the BlackPill resets, that is the GY-521's inrush, not a bug), then `imu info`
   succeeds and the LED slows down.
2. **Host stops reading.** In `console`, `stream on`, then suspend it with Ctrl-Z for about
   five seconds, `fg`, `stream off`, `stats`. Pass: `tx_drop` is above zero, the firmware
   did not reset (`samples` kept counting) and the console works normally.
3. **USB drops out mid-stream.** Power is never cut: the GY-521 takes its power from USB's 5 V
   (J1 VCC is on the +5V net), so unplugging the cable would also reset the IMU, and that
   would test R1's re-probe rather than the USB code. Instead make the host drop the device
   logically, which leaves VBUS and the IMU powered. Linux only; on another bench machine
   skip R3 and note that in the results table.

   In one terminal run `console`, then `stream on`. In a second terminal:

       D=$(dirname "$(grep -l 5740 /sys/bus/usb/devices/*/idProduct | head -1)")
       echo 0 | sudo tee "$D/authorized"; sleep 5; echo 1 | sudo tee "$D/authorized"

   Expect the first terminal's console to fail when the port vanishes: its reader thread
   dies with a serial exception and the console no longer works. Quit it with Ctrl-C (or
   Ctrl-D; a traceback on exit is fine), then start `console` again. Pass: `stats` answers,
   `tx_drop` has grown, and `stream on` streams again, with no `imu info` needed because the
   IMU never lost power. This exercises the branch in `usb_pump()` that drops a transfer
   left unfinished when the device stops being configured; without it the port would stay
   silent until reset.

## 11. Live view in RViz

    scripts/rosbuild --packages-select bizon_imu
    scripts/imu_view

This starts the serial stream, `imu_filter_madgwick` and RViz inside the container. Keep the
module still for the first 2 s while the gyro bias is measured.

`docker stop -s INT bizon_imu_view` ends it cleanly in under a second. A plain `docker stop` sends
SIGTERM, which ends the nodes before they can turn the stream off: the firmware then keeps
streaming into a closed port until the next reader stops it. The node holds the serial port
exclusively, so stop the view before `joint_bench record`. If the board resets or USB drops, the
node restarts by itself and measures the bias again.

Yaw has no absolute reference without a magnetometer, so it drifts. At rest it moved 0.05 degrees
in two minutes in one run. Two ten-minute recordings integrated with a 2 s bias, as the node
measures it, gave 0.18 and 0.31 degrees a minute, so expect anything up to about that.

## Results

| Test | Date | Result | Notes (values, directory names) |
|---|---|---|---|
| A1 | 2026-11-03 | PASS | `i2cscan` lists 68; `chip=MPU6050 who=0x68` |
| A2 | 2026-11-03 | PASS, provisional | 1.0207 g on z, on a breadboard with short jumpers; repeat on the platform. `20261103_210909_rest` |
| A3 | 2026-11-03 | PASS, provisional | 600358 samples in 600 s, 0 missing, interval 999.4 us mean, 0.5 std, 998..1001. Short jumpers on a breadboard, not the final cable; no analyzer capture. `20261103_215835_a3_a4` |
| A4 | 2026-11-03 | PASS, provisional | Same recording: `i2c_err=0 imu_overrun=0 queue_drop=0 tx_drop=0`. The analyzer check of each transaction is not done |
| A5 | 2026-11-03 | PASS, provisional | 3.80 mg rms horizontal, same recording and setup; repeat on the platform |
| A6 | | | |
| A7 | | | |
| A8 | | | |
| R1 no IMU at power-up | 2026-11-03 | Partly | Not run with the module unplugged, which needs hands. On PB7/PB8: BUSY forced over SWD and cleared by `i2cscan`; 20 of 20 MCU resets mid-stream came back with the IMU answering |
| R2 host stops reading | 2026-11-03 | PASS | Scripted, 5 s without reading: `tx_drop` 0 to 4704, `samples` 1001 to 7089, console fine |
| R3 USB drops out | 2026-11-03 | PASS | 6 of 6, `tx_drop` grew each time and the stream resumed with no `imu info`. One earlier `can't set config #1, error -110`, right after a failed SWD test, cleared by a reset and did not recur |
