# Joint0 bench hardware

Everything here except `nets.yaml`, the `trial_*.yaml` placement files and this README is
generated. Edit the YAML, then regenerate:

    python3 tools/kicad_gen/generate.py

That writes the schematic, runs KiCad's ERC, checks the exported netlist against
`nets.yaml`, and rebuilds each placement board with its drawings and soldering checklist.
KiCad does not need to be installed: `scripts/kicad-cli` runs `kicad-cli` from the
`kicad/kicad:10.0.6-full` Docker image, pinned by digest. Opening the files in a local
KiCad 8 or later works too, for viewing.

| File | What it is |
|---|---|
| `nets.yaml` | Source of truth: parts and nets. |
| `trial_6x9.yaml` | Where each part sits on the 6 x 9 cm trial board, in holes. |
| `joint_bench.kicad_sch` | Schematic. Every pin ends in a net label; read it by label. |
| `exports/joint_bench.pdf` | The schematic, for printing. |
| `trial_6x9.kicad_pcb` | Placement board. Not for fabrication. |
| `exports/trial_6x9_top.pdf` | Component side, 1:1. Print at 100 % ("actual size"). |
| `exports/trial_6x9_bottom_mirrored.pdf` | Solder side, mirrored, 1:1: wire from this one. |
| `trial_6x9_checklist.md` | Every wire to solder, then continuity and short checks. |

Design: `docs/superpowers/specs/2026-10-16-joint0-bench-design.md`. Bench procedure:
`docs/hardware/joint-bench-stage-a-runbook.md`.

Stage-B software preparation and remaining hardware-facing work:
`docs/hardware/joint-bench-stage-b-preparation.md`.

## Capacitors ordered — 2026-11-08

The user confirmed ordering the following exact products from Özdisan. This records the
order, not receipt or inspection of the parts; purchased quantities were not confirmed.
All three products are through-hole, not SMD.

| Use | Ordered part | Supplier link |
|---|---|---|
| Across each TMC2209's VM/GND, close to the driver | Koshin `PKLH-050V101MG120-T/A5.0`, 100 µF / 50 V, low impedance, 105 °C, 10 × 12 mm body, 5 mm lead spacing | [Özdisan](https://www.ozdisan.com/p/aluminyum-kapasitorler-180/koshin-pklh-050v101mg120-t-a5-0-355181) |
| Motor power input bulk capacitor | Koshin `PKLH-063V102MJ355`, 1000 µF / **63 V**, low impedance, 105 °C, 16 × 35.5 mm body, 7.5 mm lead spacing | [Özdisan](https://www.ozdisan.com/p/aluminyum-kapasitorler-180/koshin-pklh-063v102mj355-345727) |
| Ceramic bypass across the driver's VM/GND, parallel to the 100 µF capacitor | Kingtronics `MKT0805B104K500PF3R`, 100 nF / 50 V, X7R, ±10%, 5.08 mm lead spacing | [Özdisan](https://www.ozdisan.com/p/multilayer-kapasitorler-182/kingtronics-mkt0805b104k500pf3r-383365) |

The **1000 µF / 63 V** part deliberately replaces the **1000 µF / 50 V** originally specified
for the planned 12–24 V motor supply. Capacitance is unchanged; 63 V is the capacitor's
voltage rating, not the supply setting or a voltage clamp. The TMC2209's supply limits remain
unchanged, and this substitution requires no firmware change. Allow for the ordered part's
larger body and 7.5 mm lead spacing when laying out stage B. Both electrolytics are polarized:
connect positive to VM and negative to GND.
