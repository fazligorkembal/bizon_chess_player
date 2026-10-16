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
