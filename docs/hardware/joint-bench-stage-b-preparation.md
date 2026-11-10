# Joint0 bench — Stage B software preparation

Date: 2026-11-08. Status: protocol logic only; motor hardware is not commissioned.

Stage A's remaining acceptance tests in `joint-bench-stage-a-runbook.md` still apply before
stage B hardware bring-up. The user has one NEMA17, one TMC2209 module and a DC supply;
the supply's actual voltage/current range, motor phase-current rating and exact driver-module
variant are not yet recorded. Capacitors have been ordered: exact part numbers, links and
the deliberate 1000 µF / 63 V substitution are in `hardware/joint_bench/README.md`.

## First software increment

`codes_stm32/lib/include/jb/tmc_uart.hpp` and `lib/src/tmc_uart.cpp` provide allocation-free,
HAL-independent TMC2209 UART packet construction and reply validation. Protocol reference:
[TMC2209 datasheet, revision 1.09, sections 4.1–4.2](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf).

- `make_read` emits four bytes; `make_write` emits eight, including the CRC and big-endian data.
- Node addresses must be 0–3, and register addresses 0–127. Invalid requests leave the output
  unchanged, rather than silently masking an invalid address.
- `decode_reply` consumes exactly one eight-byte response, checks its header, master address
  0xFF, expected register and CRC, and updates the register value only on success.
- The caller must assemble the response and handle echo, deadlines and stale bytes. A valid
  response does not carry a slave address or transaction ID, so this layer cannot identify
  a delayed reply from a previous read of the same register.

GoogleTest cases live in `lib/tests/test_tmc_uart.cpp`. The existing host CMake build discovers
the new sources/tests automatically; CubeIDE includes `lib/src` as a source folder. Run
`scripts/fwbuild --test` in the documented firmware Docker environment.

Validation on 2026-11-08: all nine UART GoogleTests passed with native GCC and the locally
available GoogleTest headers/libraries. The production module also compiled for Cortex-M4
with CubeIDE's Arm GNU 13.3 toolchain, C++17, exceptions/RTTI disabled and warnings treated
as errors. Docker was unavailable in this session; the pinned container suite and the full
STM32 target build were not run.

## Next hardware-facing increment

Before implementing it, extend the stage-B design with the actual module/supply details,
then plan the UART transport, pin initialization and acceptance checks. The intended first
hardware check is an IOIN read with motor outputs disabled, followed by an IFCNT-verified
register write. It is not a movement command. Only after that come current configuration,
bounded STEP generation, the endstop and homing tests.

The protocol logic added here is not connected to the command loop or any GPIO/UART peripheral.
No firmware was flashed and no motor was powered for this increment.

## Phase 1 step 1 result — 2026-11-10

Passed on the breadboard. Module: "TMC2209 V2.0" sold by Twotrees, built on Makerbase's
MKS TMC2209 V2.0_001 layout (schematic and pinout in `debug/datasheets/mks_tmc2209_v2.0/`,
gitignored), sense resistors R110. Pins were confirmed unpowered against that layout: GND by
continuity to the back copper, VIO-EN and PDN_UART-GND by their 20 kohm resistors, VM through C5.

Setup: VIO 3.3 V from the BlackPill, VM 12 V at a 0.30 A limit (the supply read 0.003 A), EN tied
to 3.3 V, MS1/MS2 to GND, PDN_UART through 1 kohm to PA9 with 4.7 kohm to 3.3 V, motor unplugged.

| Check | Result |
|---|---|
| `tmc read 0x06` with VM off | `no reply`, as expected |
| IOIN | `0x21000041`: VERSION 0x21, ENN 1, MS1 = MS2 = 0, PDN_UART high |
| GCONF at power-up | `0x00000101`: I_scale_analog, multistep_filt; internal_Rsense 0 |
| `tmc write 0x00 0x00000141` (pdn_disable) | IFCNT 0 -> 1, read back `0x00000141` |
| `tmc write 0x10 ...` | refused by the bench policy |
| 500 IOIN reads in a row | 500 correct, 1.85 ms per command round trip over USB |

The first reads with VM on got no reply until the user reseated a connection; which one is not
recorded. Codex decoded the registers against datasheet Rev. 1.09 and agreed the step passes.

Flashing, as observed in this session: with USB in and three ST-Link wires (no 3.3 V), every SWD
connect failed with `Unable to get core ID` (113 attempts over four minutes). With USB out and the
ST-Link's 3.3 V wire powering the BlackPill and the wired TMC module, the BlackPill's LED glowed
dimly and the ST-Link then failed with `DEV_USB_COMM_ERR` until it was replugged; the voltage was
not measured. With USB in and all four ST-Link wires, SWD connected. Hot-plug mode then failed with
`failed to erase memory`, and the default (halting) connect mode flashed and verified.
