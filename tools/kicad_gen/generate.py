#!/usr/bin/env python3
"""Regenerate everything in hardware/joint_bench/ from nets.yaml and the placement files.

    python3 tools/kicad_gen/generate.py

Writes the schematic, runs KiCad's ERC on it, exports its netlist and checks that against
nets.yaml, plots it, then builds each placement board with its drawings and soldering
checklist. An ERC error or a netlist difference stops the run with a non-zero exit before
anything is plotted, so no drawing or checklist ever comes from a schematic that
disagrees with nets.yaml. kicad-cli comes from $KICAD_CLI, defaulting to the pinned
Docker wrapper in scripts/.
"""

import glob
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from kicad_gen import board, checklist, design, netlist, schematic  # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
HW = os.path.join(REPO, "hardware", "joint_bench")
KICAD_CLI = os.environ.get("KICAD_CLI", os.path.join(REPO, "scripts", "kicad-cli"))
NOTES = [
    "GY-87 cable, 30-60 cm: J1 VCC to VCC_IN, GND to GND, SCL to SCL, SDA to SDA, INT to INTA.",
    "Tie the GY-87's FSYNC pin to its GND at the module end. Twist SCL with GND, SDA with VCC.",
    "J2: logic analyzer CH0-CH7 then GND. Never clip an analyzer channel to VM or a motor coil.",
]


def kicad(*args):
    result = subprocess.run([KICAD_CLI, *args], cwd=HW, capture_output=True, text=True)
    if result.returncode != 0:
        sys.stderr.write(result.stdout + result.stderr)
        raise SystemExit(f"kicad-cli {args[0]} {args[1]} failed")
    return result


def main():
    d = design.load(os.path.join(HW, "nets.yaml"))
    schematic.write(d, HW, notes=NOTES)
    sch = f"{schematic.LIB}.kicad_sch"
    os.makedirs(os.path.join(HW, "exports"), exist_ok=True)
    with tempfile.TemporaryDirectory(dir=HW) as tmp:
        report = os.path.join(tmp, "erc.rpt")
        erc = subprocess.run([KICAD_CLI, "sch", "erc", "--severity-error",
                              "--exit-code-violations", "-o", report, sch],
                             cwd=HW, capture_output=True, text=True)
        if erc.returncode != 0:
            if os.path.exists(report):
                sys.stderr.write(open(report, encoding="utf-8").read())
            else:
                sys.stderr.write(erc.stderr)
            raise SystemExit("ERC found errors")
        net_path = os.path.join(tmp, "joint_bench.net")
        kicad("sch", "export", "netlist", "-o", net_path, sch)
        exported = netlist.parse(open(net_path, encoding="utf-8").read())
    problems = netlist.compare(d, exported)
    if problems:
        raise SystemExit("schematic and nets.yaml disagree:\n  " + "\n  ".join(problems))
    kicad("sch", "export", "pdf", "-o", "exports/joint_bench.pdf", sch)
    kicad("sch", "export", "svg", "-o", "exports", sch)
    for path in sorted(glob.glob(os.path.join(HW, "trial_*.yaml"))):
        name = os.path.splitext(os.path.basename(path))[0]
        placement = board.load_placement(path)
        bad = board.validate(d, placement)
        if bad:
            raise SystemExit(f"{name}: placement problems:\n  " + "\n  ".join(bad))
        pcb = f"{name}.kicad_pcb"
        board.write(d, placement, os.path.join(HW, pcb), f"{d.title} - {name}")
        top = "F.Cu,F.SilkS,F.Fab,Dwgs.User,Edge.Cuts"
        bottom = "B.Cu,B.SilkS,B.Fab,Dwgs.User,Edge.Cuts"
        kicad("pcb", "export", "pdf", "--mode-single", "-l", top,
              "-o", f"exports/{name}_top.pdf", pcb)
        kicad("pcb", "export", "pdf", "--mode-single", "--mirror", "-l", bottom,
              "-o", f"exports/{name}_bottom_mirrored.pdf", pcb)
        svg = ["pcb", "export", "svg", "--mode-single", "--page-size-mode", "2",
               "--exclude-drawing-sheet"]
        kicad(*svg, "-l", top, "-o", f"exports/{name}_top.svg", pcb)
        kicad(*svg, "--mirror", "-l", bottom, "-o", f"exports/{name}_bottom_mirrored.svg", pcb)
        title = f"{name.replace('_', ' ').capitalize()} board"
        with open(os.path.join(HW, f"{name}_checklist.md"), "w", encoding="utf-8") as fh:
            fh.write(checklist.render(d, placement, exported, title))
    print(f"OK: {len(d.parts)} parts, {len(d.nets)} nets, ERC clean, netlist matches nets.yaml")


if __name__ == "__main__":
    main()
