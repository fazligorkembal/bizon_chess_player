# Joint0 Bench, Stage A — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and bring up the stage-A bench (BlackPill F401 and a GY-521 IMU on a 6 × 9 cm trial perfboard), with generated board drawings to solder from, firmware that streams timestamped 1 kHz IMU data over USB, and a host tool that records and analyses it against acceptance tests A1-A7.

**Architecture:** Three independent parts meet at two contracts. The board is described once in `hardware/joint_bench/nets.yaml`; `tools/kicad_gen` generates the KiCad schematic, a hole-grid placement board and a soldering checklist from it, and verifies the schematic's exported netlist against it. The firmware is a CubeMX project whose generated C calls four functions in `app/app.h`; all logic that can run on a host lives in `lib/` and is unit-tested with GoogleTest, while `drivers/` wraps HAL. The host tool `tools/joint_bench` speaks the firmware's line protocol (spec §4.4), records to `debug/bench/`, and analyses recordings with pure functions tested on synthetic signals.

**Tech Stack:** Python 3.10+ (numpy, scipy, matplotlib, PyYAML, pyserial, pytest); KiCad 10.0.6 `kicad-cli` in Docker; STM32CubeIDE 1.18.1 with its built-in CubeMX editor, STM32 HAL + USB Device CDC; C++17 on `arm-none-eabi-gcc` 10.3; GoogleTest 1.11; Docker.

**Spec:** `docs/superpowers/specs/2026-10-16-joint0-bench-design.md`. Read it before any task.

## Correction 2026-11-01: the chip is an STM32F401CEU6, built in CubeIDE

An SWD read on 2026-11-01 returned device ID 0x433, and the chip's marking reads STM32F401CEU6,
not the F411 this plan was written for. These points override any older text below; the code
blocks of finished tasks are left as the record of what was committed.

- **Clock:** 84 MHz, the F401's maximum. HSE 25 MHz, PLL M = 25, N = 336, P = 4, Q = 7, 2 flash
  wait states. The 25 MHz crystal is confirmed: on 2026-11-01 USB enumerated on a build clocked from
  HSE, with RCC reading back M = 25, N = 336, P = 4, Q = 7 and the PLL as system clock.
- **Memory:** 96 KB RAM instead of 128 KB; 512 KB flash, unchanged.
- **Pins:** unchanged. Every alternate function in the allocation exists on the F401 with the same
  AF number, compared against CubeIDE's `STM32F401C(D-E)Ux.xml` and
  `GPIO-STM32F401_gpio_v1_0_Modes.xml`. The F401 lacks only SPI4/5, I2S and SDIO, none of them used.
- **Toolchain:** the user creates, builds and flashes the project in STM32CubeIDE 1.18.1, which still
  has the built-in `.ioc` editor (2.x dropped it). There is no generated CMake project, so
  `scripts/fwbuild` remains only for `--test`, the host tests of `lib/`.
- **Project location:** the CubeIDE project is `codes_stm32/` at the repository root, and `lib/`
  moved into it.

## Global Constraints

- Scope is stage A only: BlackPill, GY-521 IMU, logic-analyzer header, MARK line. No TMC2209, motor, 24 V, endstop or `VM` sense circuitry is built, drawn or coded here.
- Pins (spec §3.1): IMU SCL **PB6**, SDA **PB7** (I2C1, AF4, Fast Mode 400 kHz); IMU INT **PA2** (EXTI2, rising edge); MARK **PA4** (output, starts low); LED **PC13** (active low). PA13/PA14 are never allocated.
- Clock (spec §4.2): HSE 25 MHz; PLL M = 25, N = 336, P = 4, Q = 7; SYSCLK 84 MHz; USB 48 MHz; AHB 84 MHz; APB1 42 MHz; APB2 84 MHz; flash latency 2 wait states.
- NVIC order, highest first: step timers (reserved, stage B); EXTI2, I2C1 event/error and I2C1 RX DMA; TIM4 tick (stage B); USB OTG FS; SysTick.
- Firmware language: C++17 for `app/`, `drivers/`, `lib/`, with `-fno-exceptions -fno-rtti`, no heap after init. Generated CubeMX code stays C and is edited only inside `USER CODE` blocks.
- `lib/` never includes HAL or CMSIS headers.
- Protocol (spec §4.4), exact line formats: `T,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>`; `E,<t_us>,<name>`; `OK ...` / `ERR ...`; `# ...`. Lines end in `\n`.
- IMU default (spec §4.5): 1 kHz, accelerometer DLPF nearest 188 Hz (184 Hz on an MPU-6050), ±4 g, gyro PLL as clock source, data-ready interrupt as the sample clock.
- The firmware never blocks on USB; a full transmit ring drops whole lines and counts them.
- Every data-ready edge consumes one `seq`, whether or not its sample survives.
- Host tools: Python ≥ 3.10, dependencies from `tools/requirements.txt` only, no ROS. Recordings go to `debug/bench/<YYYYmmdd_HHMMSS>_<label>/`.
- KiCad files in `hardware/joint_bench/` other than `nets.yaml`, `trial_*.yaml` and `README.md` are generated and never edited by hand. `kicad-cli` runs only through `scripts/kicad-cli` (image `kicad/kicad@sha256:e36e29b5f4a638a1a7238eba3d2fcbc8e061ffed1e99a8f39cb722706c94a58e`).
- Target builds and flashing run in STM32CubeIDE 1.18.1. Host-side firmware tests run only through `scripts/fwbuild --test` (image built from `docker/dockerfile-firmware`).
- Python code passes `flake8 --max-line-length 99`, the repository's limit.
- Do not touch `ros_env/`.
- Comments explain *why*, as in the rest of the repository. Conventional commits with a scope, ending with the `Co-Authored-By` line from the session's attribution reminder.
- A step marked **[HUMAN]** needs the user at the bench or at CubeMX. Stop at it, say exactly what is needed, and wait.

## Review Focus

The five failure modes most likely to bite a person using this, that the spec implies but no ordinary unit test reaches. Each has a test in the task named.

1. **USB not enumerated, or unplugged mid-stream while the board stays powered.** Expected: no hard fault, and after replug the console answers and streams again. `CDC_Transmit_FS` dereferences null class data before enumeration, and an unplugged transfer never completes. Owner: Task 11 (bench step R3; the guard is in `usb_pump()`).
2. **The GY-521 not connected at power-up, or connected later.** Expected: the firmware keeps running, the LED blinks fast, `imu info` probes again and succeeds once the cable is in. Owner: Task 11 (bench step R1).
3. **The host stops reading mid-stream** (console suspended, recorder killed). Expected: sampling continues, losses show as `tx_drop` and `seq` gaps, the stream resumes; a killed recording leaves no `meta.yaml` and the analyser refuses it. Owners: Task 4 (`LineRing` drop test), Task 8 (recorder stall test), Task 9 (`load` refuses an interrupted recording), Task 11 (bench step R2).
4. **Input that would corrupt the CSV stream**: a mark label containing a comma or space, or an overlong command line. Expected: rejected with an `ERR` line, never echoed into a `T` or `E` line, and never run as a truncated or split command. Owner: Task 6 (parser and assembler tests).
5. **The real BlackPill does not match the drawing** (silkscreen order or row spacing). Expected: caught before soldering. Owners: Task 3 (row-spacing test on `pad_holes`, and the **[HUMAN]** silkscreen and caliper check).

---

## File Structure

```
.gitignore                                   MOD  Python and KiCad scratch files
docker/dockerfile-firmware                   NEW  pinned ARM GCC + host GCC + GoogleTest
scripts/kicad-cli                            NEW  kicad-cli from the pinned KiCad image
scripts/fwbuild                              NEW  firmware build / lib tests in Docker

tools/requirements.txt                       NEW  host-tool dependencies, both tools
tools/kicad_gen/
  generate.py                                NEW  entry point: YAML -> KiCad -> checks -> exports
  kicad_gen/__init__.py                      NEW
  kicad_gen/sexpr.py                         NEW  S-expression writer/reader
  kicad_gen/design.py                        NEW  nets.yaml loader + validation; BlackPill pinout
  kicad_gen/netlist.py                       NEW  kicad-cli netlist parser + comparison
  kicad_gen/schematic.py                     NEW  .kicad_sch / .kicad_sym / project writer
  kicad_gen/board.py                         NEW  placement board on the hole grid
  kicad_gen/checklist.py                     NEW  soldering / continuity / shorts checklist
  tests/conftest.py, tests/test_*.py         NEW

hardware/joint_bench/
  nets.yaml                                  NEW  source of truth (hand-written)
  trial_6x9.yaml                             NEW  placement on the trial board (hand-written)
  README.md                                  NEW
  joint_bench.kicad_sch/.kicad_sym/.kicad_pro, sym-lib-table      GENERATED
  trial_6x9.kicad_pcb, trial_6x9_checklist.md, exports/*.pdf|svg  GENERATED

codes_stm32/
  lib/CMakeLists.txt                         NEW  host build of lib/ + GoogleTest
  lib/include/jb/*.hpp, lib/src/*.cpp        NEW  pure logic
  lib/tests/test_*.cpp                       NEW
  codes_stm32.ioc + CubeIDE output           NEW  [HUMAN] generated in Task 10
  .project, .cproject                        NEW  (CubeIDE-generated) C++ and our sources
  app/app.h, app/app.cpp                     NEW  the seam CubeMX code calls
  drivers/clock.*, drivers/usb_port.*, drivers/imu.*   NEW  HAL wrappers

tools/joint_bench/
  joint_bench/__init__.py, protocol.py, recorder.py, analysis.py, report.py, __main__.py   NEW
  tests/fake_port.py, tests/test_*.py        NEW

docs/hardware/f411-tmc2209-wiring.md         MOD  pin allocation: IMU, MARK, J4 STEP, tick timer
docs/hardware/joint-bench-stage-a-runbook.md NEW  bench procedure, tests A1-A7, R1-R3
docs/superpowers/specs/2026-10-16-joint0-bench-design.md  MOD  GY-521, trial board, A7
```

---

### Task 1: KiCad generator core — S-expressions, the design model, netlist comparison

**Files:**
- Create: `tools/requirements.txt`, `tools/kicad_gen/kicad_gen/__init__.py`, `tools/kicad_gen/kicad_gen/sexpr.py`, `tools/kicad_gen/kicad_gen/design.py`, `tools/kicad_gen/kicad_gen/netlist.py`
- Test: `tools/kicad_gen/tests/conftest.py`, `tools/kicad_gen/tests/test_sexpr.py`, `tools/kicad_gen/tests/test_design.py`, `tools/kicad_gen/tests/test_netlist.py`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `kicad_gen.sexpr`: `class Sym(str)` (bare atom); `dumps(node) -> str`; `loads(text) -> list` (all atoms come back as `str`).
  - `kicad_gen.design`: `BLACKPILL_PINS: list[str]` (40 names, index i = pin i+1); `@dataclass Part(ref, kind, value, pins)` with `numbers_for(name_or_number) -> list[str]`; `@dataclass Design(title, parts: dict[str, Part], nets: dict[str, list[str]])` with nodes as `"REF.NUMBER"` and `net_of(ref, number) -> str | None`; `class DesignError(ValueError)`; `load(path) -> Design`; `from_dict(data) -> Design`. Kinds: `"blackpill"`, `"header"`, `"resistor"` (resistor pins are named `"~"`).
  - `kicad_gen.netlist`: `parse(text) -> dict[str, dict[str, str]]` (net → {"REF.NUMBER": pinfunction}); `compare(design, exported) -> list[str]` (empty means equal).

- [ ] **Step 1: Ignore Python and KiCad scratch files**

Append to `.gitignore`:

```gitignore
__pycache__/
.pytest_cache/
# KiCad writes these beside any file it opens; none of them is source.
*.kicad_prl
*-backups/
fp-info-cache
*.lck
_autosave-*
```

- [ ] **Step 2: Write the dependency list**

`tools/requirements.txt`:

```text
# Host tools for the joint0 bench: tools/kicad_gen and tools/joint_bench.
# Install with: python3 -m pip install --user -r tools/requirements.txt
# Upper bounds stop a new major release from changing behaviour under us; the lower
# bounds are the oldest versions the tests have been run against.
numpy>=1.26,<3
scipy>=1.11,<2
matplotlib>=3.8,<4
PyYAML>=6,<7
pyserial==3.5
pytest>=8,<10
```

Run: `python3 -m pip install --user -r tools/requirements.txt`
Expected: completes; `python3 -c "import yaml, serial, scipy"` prints nothing.

- [ ] **Step 3: Write the failing tests**

`tools/kicad_gen/tests/conftest.py`:

```python
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
```

`tools/kicad_gen/tests/test_sexpr.py`:

```python
from kicad_gen.sexpr import Sym, dumps, loads


def test_round_trips_atoms_strings_and_numbers():
    node = [Sym("pin"), Sym("passive"), [Sym("at"), -12.7, 0.0, 180], [Sym("name"), 'a "q" b']]
    assert loads(dumps(node)) == ["pin", "passive", ["at", "-12.7", "0", "180"],
                                  ["name", 'a "q" b']]


def test_numbers_have_no_trailing_zeros_or_negative_zero():
    assert dumps([Sym("xy"), 1.27, -0.0, 2.0]) == "(xy 1.27 0 2)"
```

`tools/kicad_gen/tests/test_design.py`:

```python
import pytest

from kicad_gen.design import DesignError, from_dict


def _d(nets, parts=None):
    parts = parts or {
        "U1": {"kind": "blackpill"},
        "J1": {"kind": "header", "pins": ["A", "B"]},
        "R1": {"kind": "resistor"},
    }
    return from_dict({"title": "t", "parts": parts, "nets": nets})


def test_a_module_pin_name_covers_every_pin_with_that_name():
    d = _d({"GND": ["U1.GND", "J1.A"]})
    assert d.nets["GND"] == ["J1.1", "U1.19", "U1.39"]


def test_pins_can_be_named_by_number():
    d = _d({"N": ["R1.1", "J1.2"]})
    assert d.nets["N"] == ["J1.2", "R1.1"]
    assert d.net_of("R1", "1") == "N"
    assert d.net_of("R1", "2") is None


@pytest.mark.parametrize("nets, message", [
    ({"N": ["U9.PA0", "J1.A"]}, "unknown part"),
    ({"N": ["U1.PZ9", "J1.A"]}, "has no pin"),
    ({"N": ["J1.A"]}, "fewer than two pins"),
    ({"N": ["U1.PA0", "J1.A"], "M": ["U1.PA0", "J1.B"]}, "is on both"),
])
def test_rejects_designs_that_cannot_be_drawn(nets, message):
    with pytest.raises(DesignError, match=message):
        _d(nets)


def test_rejects_unknown_kinds_and_bad_headers():
    with pytest.raises(DesignError, match="unknown kind"):
        _d({}, parts={"X1": {"kind": "relay"}})
    with pytest.raises(DesignError, match="unique"):
        _d({}, parts={"J1": {"kind": "header", "pins": ["A", "A"]}})
```

`tools/kicad_gen/tests/test_netlist.py`:

```python
from kicad_gen.design import from_dict
from kicad_gen.netlist import compare, parse

NETLIST = """(export (version "E")
  (nets
    (net (code "1") (name "/SCL")
      (node (ref "U1") (pin "14") (pinfunction "PB6") (pintype "passive"))
      (node (ref "J1") (pin "1") (pinfunction "A") (pintype "passive")))
    (net (code "2") (name "unconnected-(U1-PA0-Pad26)")
      (node (ref "U1") (pin "26") (pinfunction "PA0") (pintype "passive")))))
"""


def test_parse_strips_the_sheet_prefix_and_drops_unconnected_pins():
    assert parse(NETLIST) == {"SCL": {"U1.14": "PB6", "J1.1": "A"}}


PARTS = {"U1": {"kind": "blackpill"}, "J1": {"kind": "header", "pins": ["A", "B"]}}


def test_compare_reports_every_kind_of_difference():
    d = from_dict({"parts": PARTS,
                   "nets": {"SCL": ["U1.PB6", "J1.B"], "SDA": ["U1.PB7", "J1.A"]}})
    problems = compare(d, parse(NETLIST))
    assert "net SDA is in nets.yaml but not in the schematic" in problems
    assert "J1.2 should be on SCL but is not" in problems
    assert "J1.1 is on SCL but should not be" in problems


def test_compare_is_empty_when_they_agree():
    d = from_dict({"parts": PARTS,
                   "nets": {"SCL": ["U1.PB6", "J1.A"]}})
    assert compare(d, parse(NETLIST)) == []
```

- [ ] **Step 4: Run them and verify they fail**

Run: `cd tools/kicad_gen && python3 -m pytest -q tests`
Expected: FAIL, `ModuleNotFoundError: No module named 'kicad_gen'`.

- [ ] **Step 5: Write the implementation**

`tools/kicad_gen/kicad_gen/__init__.py`:

```python
"""Generate the joint0 bench's KiCad schematic, placement boards and soldering checklists
from hardware/joint_bench/nets.yaml. Entry point: tools/kicad_gen/generate.py."""
```

`tools/kicad_gen/kicad_gen/sexpr.py`:

```python
"""Minimal S-expression writer and reader for KiCad files.

KiCad's formats are S-expressions, but no parser for them ships with Python and the
pcbnew SWIG bindings are deprecated, so the generator writes the text itself. Lists are
nodes, `Sym` is a bare atom (`yes`, `passive`), `str` is always quoted, and numbers are
written with at most four decimals, which is finer than KiCad's 1 nm resolution needs
for millimetre coordinates on a 0.05 mm grid.
"""


class Sym(str):
    """A bare atom, written without quotes."""


def _atom(value):
    if isinstance(value, Sym):
        return str(value)
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        text = f"{value:.4f}".rstrip("0").rstrip(".")
        return "0" if text in ("-0", "") else text
    if isinstance(value, str):
        escaped = value.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
        return f'"{escaped}"'
    raise TypeError(f"cannot serialise {value!r}")


def dumps(node, indent=0):
    """Serialise a nested list. Short leaf lists stay on one line, so diffs stay readable."""
    if not isinstance(node, list):
        return _atom(node)
    pad = "  " * indent
    if all(not isinstance(child, list) for child in node):
        return pad + "(" + " ".join(_atom(child) for child in node) + ")"
    head = []
    rest = []
    for child in node:
        if not rest and not isinstance(child, list):
            head.append(_atom(child))
        else:
            rest.append(child)
    lines = [pad + "(" + " ".join(head)]
    for child in rest:
        if isinstance(child, list):
            lines.append(dumps(child, indent + 1))
        else:
            lines.append("  " * (indent + 1) + _atom(child))
    lines[-1] += ")"
    return "\n".join(lines)


def loads(text):
    """Parse one S-expression into nested lists of str. Quoted and bare atoms both become str."""
    tokens = _tokenize(text)
    pos = 0

    def parse():
        nonlocal pos
        token = tokens[pos]
        pos += 1
        if token == "(":
            items = []
            while tokens[pos] != ")":
                items.append(parse())
            pos += 1
            return items
        if token == ")":
            raise ValueError("unexpected ')'")
        return token

    result = parse()
    return result


def _tokenize(text):
    tokens = []
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        if c in " \t\r\n":
            i += 1
        elif c in "()":
            tokens.append(c)
            i += 1
        elif c == '"':
            i += 1
            out = []
            while text[i] != '"':
                if text[i] == "\\":
                    i += 1
                    out.append({"n": "\n"}.get(text[i], text[i]))
                else:
                    out.append(text[i])
                i += 1
            i += 1
            tokens.append("".join(out))
        else:
            j = i
            while j < n and text[j] not in ' \t\r\n()"':
                j += 1
            tokens.append(text[i:j])
            i = j
    return tokens
```

`tools/kicad_gen/kicad_gen/design.py`:

```python
"""Load and validate nets.yaml, the single source of truth for the bench board.

The schematic, the placement board and the soldering checklist are all generated from the
`Design` this module returns, so a mistake caught here cannot reach the copper.
"""

from dataclasses import dataclass, field

import yaml

# WeAct BlackPill F411CEU6, silkscreen order. Header A runs from the USB end to the SWD
# end on one long edge, header B along the other, in the same direction. Pins 1-20 are
# header A, 21-40 header B. Verify against your own board's silkscreen before soldering.
BLACKPILL_PINS = [
    "PB12", "PB13", "PB14", "PB15", "PA8", "PA9", "PA10", "PA11", "PA12", "PA15",
    "PB3", "PB4", "PB5", "PB6", "PB7", "PB8", "PB9", "5V", "GND", "3V3",
    "VB", "PC13", "PC14", "PC15", "NRST", "PA0", "PA1", "PA2", "PA3", "PA4",
    "PA5", "PA6", "PA7", "PB0", "PB1", "PB2", "PB10", "3V3", "GND", "5V",
]

KINDS = ("blackpill", "header", "resistor")


@dataclass
class Part:
    ref: str
    kind: str
    value: str
    pins: list  # pin names, index i is pin number i + 1

    def numbers_for(self, name_or_number):
        """Pin numbers a reference names. A module name such as GND may cover several pins."""
        hits = [str(i + 1) for i, name in enumerate(self.pins) if name == name_or_number]
        if hits:
            return hits
        if name_or_number.isdigit() and 1 <= int(name_or_number) <= len(self.pins):
            return [name_or_number]
        return []


@dataclass
class Design:
    title: str
    parts: dict  # ref -> Part
    nets: dict = field(default_factory=dict)  # net name -> sorted list of "REF.NUMBER"

    def net_of(self, ref, number):
        for net, nodes in self.nets.items():
            if f"{ref}.{number}" in nodes:
                return net
        return None


class DesignError(ValueError):
    pass


def _make_part(ref, spec):
    kind = spec.get("kind")
    if kind not in KINDS:
        raise DesignError(f"{ref}: unknown kind {kind!r}, expected one of {KINDS}")
    value = str(spec.get("value", ""))
    if kind == "blackpill":
        pins = list(BLACKPILL_PINS)
    elif kind == "resistor":
        pins = ["~", "~"]
    else:
        pins = [str(p) for p in spec.get("pins", [])]
        if not pins:
            raise DesignError(f"{ref}: a header needs a non-empty pins list")
        if len(set(pins)) != len(pins):
            raise DesignError(f"{ref}: header pin names must be unique")
    return Part(ref=ref, kind=kind, value=value, pins=pins)


def load(path):
    with open(path, encoding="utf-8") as fh:
        return from_dict(yaml.safe_load(fh))


def from_dict(data):
    parts = {ref: _make_part(ref, spec) for ref, spec in data.get("parts", {}).items()}
    design = Design(title=str(data.get("title", "")), parts=parts)
    owner = {}
    for net, refs in data.get("nets", {}).items():
        nodes = []
        for text in refs:
            ref, _, pin = str(text).partition(".")
            if ref not in parts:
                raise DesignError(f"net {net}: unknown part {ref!r} in {text!r}")
            numbers = parts[ref].numbers_for(pin)
            if not numbers:
                raise DesignError(f"net {net}: {ref} has no pin {pin!r}")
            for number in numbers:
                node = f"{ref}.{number}"
                if node in owner:
                    raise DesignError(f"{node} is on both {owner[node]} and {net}")
                owner[node] = net
                nodes.append(node)
        if len(nodes) < 2:
            raise DesignError(f"net {net} has fewer than two pins")
        design.nets[net] = sorted(nodes, key=_node_key)
    return design


def _node_key(node):
    ref, _, number = node.partition(".")
    return (ref, int(number))
```

`tools/kicad_gen/kicad_gen/netlist.py`:

```python
"""Read a netlist exported by `kicad-cli sch export netlist` and compare it with the design.

This is the check that the drawn schematic says what nets.yaml says. KiCad prefixes
sheet-local net names with the sheet path ("/SCL" on the root sheet) and names every
unconnected pin's net "unconnected-(...)"; both are normalised away here.
"""

from .sexpr import loads


def _children(node, tag):
    return [c for c in node if isinstance(c, list) and c and c[0] == tag]


def _value(node, tag):
    hits = _children(node, tag)
    return hits[0][1] if hits else None


def parse(text):
    """Return {net name: {"REF.NUMBER": pin function}} for every connected net."""
    root = loads(text)
    nets = {}
    for nets_node in _children(root, "nets"):
        for net in _children(nets_node, "net"):
            name = _value(net, "name")
            if name.startswith("unconnected-"):
                continue
            name = name.lstrip("/")
            nodes = {}
            for node in _children(net, "node"):
                key = f"{_value(node, 'ref')}.{_value(node, 'pin')}"
                nodes[key] = _value(node, "pinfunction") or ""
            nets[name] = nodes
    return nets


def compare(design, exported):
    """List every difference between the design's nets and the exported ones. Empty means equal."""
    problems = []
    want = {net: set(nodes) for net, nodes in design.nets.items()}
    got = {net: set(nodes) for net, nodes in exported.items()}
    for net in sorted(want.keys() - got.keys()):
        problems.append(f"net {net} is in nets.yaml but not in the schematic")
    for net in sorted(got.keys() - want.keys()):
        problems.append(f"net {net} is in the schematic but not in nets.yaml")
    for net in sorted(want.keys() & got.keys()):
        for node in sorted(want[net] - got[net]):
            problems.append(f"{node} should be on {net} but is not")
        for node in sorted(got[net] - want[net]):
            problems.append(f"{node} is on {net} but should not be")
    return problems
```

- [ ] **Step 6: Run the tests and verify they pass**

Run: `cd tools/kicad_gen && python3 -m pytest -q tests && python3 -m flake8 --max-line-length 99 .`
Expected: `12 passed`; flake8 prints nothing. (If flake8 is missing: `python3 -m pip install --user flake8`.)

- [ ] **Step 7: Commit**

```bash
git add .gitignore tools/requirements.txt tools/kicad_gen
git commit -m "feat(hardware): add the nets.yaml model and netlist check for the bench board"
```

---

### Task 2: The schematic — writer, nets.yaml, kicad-cli wrapper, doc updates

**Files:**
- Create: `tools/kicad_gen/kicad_gen/schematic.py`, `tools/kicad_gen/generate.py`, `scripts/kicad-cli`, `hardware/joint_bench/nets.yaml`
- Test: `tools/kicad_gen/tests/test_schematic.py`
- Generated (commit them): `hardware/joint_bench/joint_bench.kicad_sch`, `joint_bench.kicad_sym`, `joint_bench.kicad_pro`, `sym-lib-table`, `exports/joint_bench.pdf`, `exports/joint_bench.svg`
- Modify: `docs/hardware/f411-tmc2209-wiring.md`, `docs/superpowers/specs/2026-10-16-joint0-bench-design.md`

**Interfaces:**
- Consumes: `design.load`, `Design.net_of`, `netlist.parse`, `netlist.compare`, `sexpr.Sym`, `sexpr.dumps` (Task 1).
- Produces:
  - `kicad_gen.schematic`: `LIB = "joint_bench"`; `GRID = 1.27`; `uid(*key) -> str` (deterministic UUID); `symbol_for(part) -> SymbolDef`; `build(design, notes=()) -> list`; `library(design) -> list`; `write(design, outdir, notes=())`.
  - `scripts/kicad-cli <args>`: runs `kicad-cli` in the pinned image; must be called from inside the repository.
  - `tools/kicad_gen/generate.py`: exits non-zero on an ERC error or a netlist difference, prints `OK: <n> parts, <m> nets, ERC clean, netlist matches nets.yaml` on success.

- [ ] **Step 1: Write the failing test**

`tools/kicad_gen/tests/test_schematic.py`:

```python
from kicad_gen.design import from_dict
from kicad_gen.schematic import GRID, build, uid

D = from_dict({"title": "t",
               "parts": {"U1": {"kind": "blackpill"}, "J1": {"kind": "header", "pins": ["A", "B"]},
                         "R1": {"kind": "resistor"}},
               "nets": {"SCL": ["U1.PB6", "J1.A"], "PU": ["J1.B", "R1.1"]}})


def _nodes(tree, tag):
    found = []

    def walk(node):
        if isinstance(node, list):
            if node and node[0] == tag:
                found.append(node)
            for child in node:
                walk(child)

    walk(tree)
    return found


def _at(node):
    return next(c for c in node if isinstance(c, list) and c[0] == "at")


def test_every_connected_pin_gets_a_label_and_every_other_pin_a_no_connect():
    tree = build(D)
    labels = sorted(n[1] for n in _nodes(tree, "label"))
    assert labels == ["PU", "PU", "SCL", "SCL"]
    # 40 BlackPill pins + 2 header + 2 resistor, minus the 4 that are connected
    assert len(_nodes(tree, "no_connect")) == 40 + 2 + 2 - 4


def test_every_connection_point_sits_on_the_50_mil_grid():
    tree = build(D)
    for tag in ("label", "no_connect"):
        for node in _nodes(tree, tag):
            x, y = _at(node)[1], _at(node)[2]
            assert abs(x / GRID - round(x / GRID)) < 1e-6, (tag, x)
            assert abs(y / GRID - round(y / GRID)) < 1e-6, (tag, y)


def test_uuids_are_stable_across_runs():
    assert uid("sym", "U1") == uid("sym", "U1")
    assert uid("sym", "U1") != uid("sym", "U2")
```

- [ ] **Step 2: Run it and verify it fails**

Run: `cd tools/kicad_gen && python3 -m pytest -q tests/test_schematic.py`
Expected: FAIL, `ModuleNotFoundError: No module named 'kicad_gen.schematic'`.

- [ ] **Step 3: Write the schematic writer**

`tools/kicad_gen/kicad_gen/schematic.py`:

```python
"""Write the KiCad schematic, its symbol library and project file from a Design.

The schematic is label-based: every pin gets a short wire stub ending in a net label, and
unused pins get a no-connect flag. That is the honest drawing for a board that is mostly
modules on headers, and it makes the generated netlist equal to nets.yaml by construction.
Coordinates sit on KiCad's 1.27 mm (50 mil) grid, or connections silently fail to join.
"""

import json
import uuid

from .sexpr import Sym, dumps

LIB = "joint_bench"
FORMAT_VERSION = 20231120  # KiCad 8 format; KiCad 9 and 10 read it unchanged
GRID = 1.27
PITCH = 2.54
STUB = 5.08
_NS = uuid.UUID("6f1d2a8e-3c47-4b0e-9a51-0c7f2b9d4e10")


def uid(*key):
    """Deterministic UUID, so regenerating an unchanged design produces an unchanged file."""
    return str(uuid.uuid5(_NS, "/".join(str(k) for k in key)))


def _font(size=1.27, hide=False, justify=None):
    effects = [Sym("effects"), [Sym("font"), [Sym("size"), size, size]]]
    if justify:
        effects.append([Sym("justify")] + [Sym(j) for j in justify])
    if hide:
        effects.append([Sym("hide"), Sym("yes")])
    return effects


def _prop(name, value, x, y, hide=False):
    return [Sym("property"), name, value, [Sym("at"), x, y, 0], _font(hide=hide)]


def _pin(number, name, x, y, angle, length=PITCH):
    return [
        Sym("pin"), Sym("passive"), Sym("line"),
        [Sym("at"), x, y, angle], [Sym("length"), length],
        [Sym("name"), name, _font()],
        [Sym("number"), number, _font()],
    ]


def _rect(x1, y1, x2, y2):
    return [
        Sym("rectangle"), [Sym("start"), x1, y1], [Sym("end"), x2, y2],
        [Sym("stroke"), [Sym("width"), 0.254], [Sym("type"), Sym("default")]],
        [Sym("fill"), [Sym("type"), Sym("background")]],
    ]


class SymbolDef:
    """A library symbol plus where each pin's connection point sits relative to its origin.

    `pins` maps pin number to (x, y, side). Library y points up; `side` is the direction a
    stub leaves the pin: "left" or "right".
    """

    def __init__(self, name, body, pins, ref_prefix, height):
        self.name = name
        self.body = body
        self.pins = pins
        self.ref_prefix = ref_prefix
        self.height = height

    def lib_node(self, pin_names, full_name):
        pins = []
        for number, (x, y, side) in self.pins.items():
            angle = 0 if side == "left" else 180
            pins.append(_pin(number, pin_names[int(number) - 1], x, y, angle))
        top = self.height / 2 + GRID
        return [
            Sym("symbol"), full_name,
            [Sym("exclude_from_sim"), Sym("no")], [Sym("in_bom"), Sym("yes")],
            [Sym("on_board"), Sym("yes")],
            _prop("Reference", self.ref_prefix, 0, top + GRID),
            _prop("Value", self.name, 0, -top - GRID),
            _prop("Footprint", "", 0, 0, hide=True),
            _prop("Datasheet", "", 0, 0, hide=True),
            _prop("Description", "", 0, 0, hide=True),
            [Sym("symbol"), f"{self.name}_0_1", self.body],
            [Sym("symbol"), f"{self.name}_1_1"] + pins,
        ]


def symbol_for(part):
    """Build the library symbol for one part. Headers get one symbol per pin count."""
    if part.kind == "blackpill":
        pins = {}
        for k in range(20):
            y = 24.13 - PITCH * k
            pins[str(k + 1)] = (-12.7, y, "left")
            pins[str(k + 21)] = (12.7, y, "right")
        return SymbolDef("BlackPill", _rect(-10.16, 26.67, 10.16, -26.67), pins, "U", 53.34)
    if part.kind == "resistor":
        pins = {"1": (-3.81, 0.0, "left"), "2": (3.81, 0.0, "right")}
        return SymbolDef("R", _rect(-1.27, 1.016, 1.27, -1.016), pins, "R", 2.54)
    n = len(part.pins)
    top = GRID * (n - 1)
    pins = {str(i + 1): (-5.08, top - PITCH * i, "left") for i in range(n)}
    body = _rect(-2.54, top + GRID, 17.78, top - PITCH * (n - 1) - GRID)
    return SymbolDef(f"Conn_{part.ref}", body, pins, "J", PITCH * n)


# Where each part's symbol origin goes on the sheet. The BlackPill sits on the left, every
# other part stacks down a column to its right. Values are multiples of 1.27 mm.
_U_ORIGIN = (63.5, 88.9)
_COLUMN_X = 177.8
_COLUMN_TOP = 33.02


def layout(design):
    """Return {ref: (x, y)} sheet positions for the symbol origins."""
    positions = {}
    y = _COLUMN_TOP
    for ref, part in design.parts.items():
        if part.kind == "blackpill":
            positions[ref] = _U_ORIGIN
            continue
        sym = symbol_for(part)
        y_origin = y + sym.height / 2
        y_origin = round(y_origin / GRID) * GRID
        positions[ref] = (_COLUMN_X, y_origin)
        y = y_origin + sym.height / 2 + 3 * PITCH
    return positions


def build(design, notes=()):
    root = uid("root")
    positions = layout(design)
    lib_symbols = {}
    items = []
    for ref, part in design.parts.items():
        sym = symbol_for(part)
        full = f"{LIB}:{sym.name}"
        lib_symbols[full] = sym.lib_node(part.pins, full)
        x0, y0 = positions[ref]
        node = [
            Sym("symbol"), [Sym("lib_id"), full], [Sym("at"), x0, y0, 0], [Sym("unit"), 1],
            [Sym("exclude_from_sim"), Sym("no")], [Sym("in_bom"), Sym("yes")],
            [Sym("on_board"), Sym("yes")], [Sym("dnp"), Sym("no")],
            [Sym("uuid"), uid("sym", ref)],
            _prop("Reference", ref, x0, y0 - sym.height / 2 - 2 * GRID),
            _prop("Value", part.value, x0, y0 + sym.height / 2 + 2 * GRID),
            _prop("Footprint", "", x0, y0, hide=True),
            _prop("Datasheet", "", x0, y0, hide=True),
            _prop("Description", "", x0, y0, hide=True),
        ]
        for number in sym.pins:
            node.append([Sym("pin"), number, [Sym("uuid"), uid("pin", ref, number)]])
        node.append([
            Sym("instances"),
            [Sym("project"), LIB,
             [Sym("path"), f"/{root}", [Sym("reference"), ref], [Sym("unit"), 1]]],
        ])
        items.append(node)
        for number, (px, py, side) in sym.pins.items():
            ax, ay = x0 + px, y0 - py  # library y is up, sheet y is down
            net = design.net_of(ref, number)
            if net is None:
                items.append([Sym("no_connect"), [Sym("at"), ax, ay],
                              [Sym("uuid"), uid("nc", ref, number)]])
                continue
            ex = ax - STUB if side == "left" else ax + STUB
            items.append([
                Sym("wire"), [Sym("pts"), [Sym("xy"), ax, ay], [Sym("xy"), ex, ay]],
                [Sym("stroke"), [Sym("width"), 0], [Sym("type"), Sym("default")]],
                [Sym("uuid"), uid("wire", ref, number)],
            ])
            if side == "left":
                angle, justify = 180, ["right", "bottom"]
            else:
                angle, justify = 0, ["left", "bottom"]
            items.append([
                Sym("label"), net, [Sym("at"), ex, ay, angle],
                [Sym("fields_autoplaced"), Sym("yes")], _font(justify=justify),
                [Sym("uuid"), uid("label", ref, number)],
            ])
    for i, text in enumerate(notes):
        items.append([
            Sym("text"), text, [Sym("exclude_from_sim"), Sym("no")],
            [Sym("at"), 20.32, 180.34 + 6 * GRID * i, 0], _font(justify=["left", "bottom"]),
            [Sym("uuid"), uid("note", i)],
        ])
    return [
        Sym("kicad_sch"), [Sym("version"), FORMAT_VERSION], [Sym("generator"), "kicad_gen"],
        [Sym("generator_version"), "1.0"], [Sym("uuid"), root], [Sym("paper"), "A4"],
        [Sym("title_block"), [Sym("title"), design.title], [Sym("rev"), "A"]],
        [Sym("lib_symbols")] + list(lib_symbols.values()),
    ] + items + [[Sym("sheet_instances"), [Sym("path"), "/", [Sym("page"), "1"]]]]


def library(design):
    """The project symbol library, so KiCad's GUI finds every lib_id the schematic uses."""
    symbols = {}
    for part in design.parts.values():
        sym = symbol_for(part)
        symbols[sym.name] = sym.lib_node(part.pins, sym.name)
    return [Sym("kicad_symbol_lib"), [Sym("version"), FORMAT_VERSION],
            [Sym("generator"), "kicad_gen"]] + list(symbols.values())


def write(design, outdir, notes=()):
    """Write joint_bench.kicad_sch, .kicad_sym, .kicad_pro and sym-lib-table into outdir."""
    with open(f"{outdir}/{LIB}.kicad_sch", "w", encoding="utf-8") as fh:
        fh.write(dumps(build(design, notes)) + "\n")
    with open(f"{outdir}/{LIB}.kicad_sym", "w", encoding="utf-8") as fh:
        fh.write(dumps(library(design)) + "\n")
    with open(f"{outdir}/sym-lib-table", "w", encoding="utf-8") as fh:
        fh.write(dumps([Sym("sym_lib_table"), [Sym("version"), 7],
                        [Sym("lib"), [Sym("name"), LIB], [Sym("type"), "KiCad"],
                         [Sym("uri"), f"${{KIPRJMOD}}/{LIB}.kicad_sym"],
                         [Sym("options"), ""], [Sym("descr"), ""]]]) + "\n")
    with open(f"{outdir}/{LIB}.kicad_pro", "w", encoding="utf-8") as fh:
        json.dump({"meta": {"filename": f"{LIB}.kicad_pro", "version": 1}}, fh, indent=2)
        fh.write("\n")
```

- [ ] **Step 4: Run the tests and verify they pass**

Run: `cd tools/kicad_gen && python3 -m pytest -q tests`
Expected: `15 passed`.

- [ ] **Step 5: Write the kicad-cli wrapper**

`scripts/kicad-cli`:

```bash
#!/usr/bin/env bash
# kicad-cli from the pinned KiCad image, so ERC, netlists and plots do not depend on which
# KiCad (if any) is installed on the host. The repository is mounted at its own path, so
# absolute paths and the current directory mean the same thing inside and out.
#
# Usage: scripts/kicad-cli <kicad-cli arguments...>
set -euo pipefail

SELF="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REPO="${BIZON_REPO:-$(dirname "$SELF")}"
# kicad/kicad:10.0.6-full, pinned by digest: a moving tag can change ERC rules or the
# plot output between two runs of the same design.
IMAGE="kicad/kicad@sha256:e36e29b5f4a638a1a7238eba3d2fcbc8e061ffed1e99a8f39cb722706c94a58e"

case "$PWD/" in
  "$REPO"/*) ;;
  *) echo "kicad-cli: run from inside $REPO" >&2; exit 2 ;;
esac

exec docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$REPO:$REPO" -w "$PWD" "$IMAGE" kicad-cli "$@"
```

Run: `chmod +x scripts/kicad-cli && scripts/kicad-cli version`
Expected: the first run pulls the image (about 1.4 GB), then prints `10.0.6`.

- [ ] **Step 6: Write nets.yaml**

`hardware/joint_bench/nets.yaml`:

```yaml
# Joint0 bench, stage A: BlackPill, IMU connector, logic-analyzer header.
#
# This file is the source of truth for the board. The schematic, the placement board and
# the soldering checklist are generated from it by tools/kicad_gen; never edit those by
# hand. Pin references are REF.NAME (a module pin name such as GND covers every pin with
# that name) or REF.NUMBER. Design: docs/superpowers/specs/2026-10-16-joint0-bench-design.md
title: Joint0 bench - stage A

parts:
  U1: {kind: blackpill, value: WeAct BlackPill F411CEU6}
  # The GY-521 sits on the platform beside the chessboard, 30-60 cm away; this is the
  # perfboard end of its cable. The module breaks out no FSYNC pin.
  J1: {kind: header, value: IMU GY-521, pins: [VCC, GND, SCL, SDA, INT]}
  J2:
    kind: header
    value: Logic analyzer
    pins: [CH0_STEP, CH1_DIR, CH2_ENDSTOP, CH3_TMC_UART, CH4_SCL, CH5_SDA, CH6_IMU_INT, CH7_MARK, GND]
  J3: {kind: header, value: Test points, pins: [3V3, 5V, GND]}
  # 2.2k pull-ups in parallel with the GY-521's own. Open the jumpers when the module's
  # pull-ups alone already measure 2.7k or less, or the bus sinks more than 3 mA.
  JP1: {kind: header, value: SCL pull-up, pins: [A, B]}
  JP2: {kind: header, value: SDA pull-up, pins: [A, B]}
  R1: {kind: resistor, value: 2.2k}
  R2: {kind: resistor, value: 2.2k}

nets:
  +5V: [U1.5V, J1.VCC, J3.5V]
  GND: [U1.GND, J1.GND, J2.GND, J3.GND]
  +3V3: [U1.3V3, J3.3V3, JP1.A, JP2.A]
  SCL: [U1.PB6, J1.SCL, J2.CH4_SCL, R1.2]
  SDA: [U1.PB7, J1.SDA, J2.CH5_SDA, R2.2]
  SCL_PU: [JP1.B, R1.1]
  SDA_PU: [JP2.B, R2.1]
  IMU_INT: [U1.PA2, J1.INT, J2.CH6_IMU_INT]
  MARK: [U1.PA4, J2.CH7_MARK]
  # Stage-B signals, brought to the analyzer header now so the pins can be probed from
  # the first board onward.
  STEP1: [U1.PA8, J2.CH0_STEP]
  DIR1: [U1.PB12, J2.CH1_DIR]
  ENDSTOP1: [U1.PB10, J2.CH2_ENDSTOP]
  TMC_UART: [U1.PA9, J2.CH3_TMC_UART]
```

- [ ] **Step 7: Write the generator entry point (schematic only for now)**

`tools/kicad_gen/generate.py`:

```python
#!/usr/bin/env python3
"""Regenerate the schematic in hardware/joint_bench/ from nets.yaml.

    python3 tools/kicad_gen/generate.py

Writes the schematic, runs KiCad's ERC on it, exports its netlist and checks that against
nets.yaml, then plots it. An ERC error or a netlist difference stops the run with a
non-zero exit before anything is plotted, so no drawing ever comes from a schematic that
disagrees with nets.yaml. kicad-cli comes from $KICAD_CLI, defaulting to the pinned
Docker wrapper in scripts/.
"""

import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from kicad_gen import design, netlist, schematic  # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
HW = os.path.join(REPO, "hardware", "joint_bench")
KICAD_CLI = os.environ.get("KICAD_CLI", os.path.join(REPO, "scripts", "kicad-cli"))
NOTES = [
    "GY-521 cable, 30-60 cm: J1 VCC to VCC, GND to GND, SCL to SCL, SDA to SDA, INT to INT.",
    "Leave the GY-521's XDA, XCL and AD0 unconnected. Twist SCL with GND, SDA with VCC.",
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
    print(f"OK: {len(d.parts)} parts, {len(d.nets)} nets, ERC clean, netlist matches nets.yaml")


if __name__ == "__main__":
    main()
```

- [ ] **Step 8: Generate and verify**

Run: `python3 tools/kicad_gen/generate.py`
Expected: `OK: 8 parts, 13 nets, ERC clean, netlist matches nets.yaml`, and `hardware/joint_bench/exports/joint_bench.pdf` exists.

Run it a second time after `git add hardware/joint_bench`: `git status --short hardware/joint_bench`
must list only files under `exports/`. UUIDs are derived from names, so an unchanged design
regenerates byte-identical KiCad files; the PDF and SVG plots embed a timestamp and always
differ. Commit the exports only together with a real design change.

- [ ] **Step 9: Update the wiring guide's pin allocation**

In `docs/hardware/f411-tmc2209-wiring.md`, replace:

```
Verify every alternate-function claim below against DS10314 Table 9 and your own board's
schematic before soldering. Board revisions differ in what they tie to PA0, PA9 and PB2.
```

with:

```
The alternate functions below were checked against ST's open pin data
(`STM32_open_pin_data`, F411 UFQFPN48) on 2026-10-16. Still check your own board's
schematic before soldering: board revisions differ in what they tie to PA0, PA9 and PB2.
```

Replace the heading `## Full allocation — four steppers, one servo gripper` with `## Full allocation — four steppers, one servo gripper, the bench IMU`.

Replace the row

```
| J4 `rev3` STEP | PB6 | TIM4_CH1 (AF2) | |
```

with:

```
| J4 `rev3` STEP | PA1 | TIM5_CH2 (AF2) | moved off PB6 so I2C1 can have PB6/PB7 |
```

Replace the row `| Spare timer | TIM5 | — | reserved for the 100 Hz control tick |` with:

```
| Control tick | TIM4 | — | 1 kHz executor tick; a time base, needs no pin |
```

After the row `| Heartbeat / fault LED | PC13 | GPIO out | |` insert:

```
| IMU SCL | PB6 | I2C1_SCL (AF4), open-drain | GY-521 on the platform, 400 kHz |
| IMU SDA | PB7 | I2C1_SDA (AF4), open-drain | |
| IMU INT | PA2 | input, EXTI2, rising edge | data ready; the sample clock |
| `VM` sense | PA3 | ADC1_IN3 | 100k/10k divider, stage B |
| MARK | PA4 | GPIO out | logic-analyzer timing reference |
```

Replace

```
and PB10 cannot both be edge sources. The endstops above use numbers 10, 3, 4 and 9 — no
collision.
```

with:

```
and PB10 cannot both be edge sources. The endstops above use numbers 10, 3, 4 and 9, and
the IMU's data-ready line uses 2 — no collision.
```

- [ ] **Step 10: Record what is now known in the design doc**

In `docs/superpowers/specs/2026-10-16-joint0-bench-design.md`:

Replace:

```
The perfboard layout is drawn for the stage-B board from the start. Stage A solders only the
logic side; stage B adds the power side without removing or rerouting anything.
```

with:

```
Stage A is built on a 6 × 9 cm trial perfboard that carries only the logic side. Stage B
moves to a 10 × 10 cm board carrying both sides; the BlackPill and every module sit on
female headers, so they move across unchanged. The schematic and each board's placement
are therefore drawn per stage: the stage-A plan draws stage A, and stage B's spec update
extends both.
```

Replace:

```
- **IMU connector**, 5 pins: `VCC` (from the BlackPill 5 V pin), `GND`, `SCL`, `SDA`, `INT`.
  All three candidate boards (GY-87, GY-91, GY-80) carry their own 3.3 V regulator and pull-ups
  to it, so they take 5 V on `VCC` and present 3.3 V logic.
```

with:

```
- **IMU connector**, 5 pins: `VCC` (from the BlackPill 5 V pin), `GND`, `SCL`, `SDA`, `INT`.
  The board is a GY-521: `VCC`, `GND`, `SCL`, `SDA` and `INT` are used, and `XDA`, `XCL` and
  `AD0` stay unconnected (the firmware tries 0x68, then 0x69). It breaks out no `FSYNC` pin,
  and the firmware leaves `EXT_SYNC_SET` at 0, so FSYNC is ignored. It carries its own 3.3 V
  regulator and pull-ups to it, so it takes 5 V on `VCC` and presents 3.3 V logic.
```

Replace:

```
1. **KiCad project:** the schematic, plus a project symbol and footprint library for the
   BlackPill and StepStick modules, which are not in the stock libraries. The schematic is also
   exported to PDF and SVG, and the exports are committed.
```

with:

```
1. **KiCad project:** the schematic and a project symbol library for the BlackPill and the
   headers, which are not in the stock libraries. The schematic is also exported to PDF and
   SVG, and the exports are committed.
```

Replace:

```
KiCad must be installed on this machine for ERC and the exports (`kicad-cli`). The KiCad version
used is recorded in the project's README.
```

with:

```
All three are generated by `tools/kicad_gen/generate.py` from `hardware/joint_bench/nets.yaml`
and never edited by hand. KiCad is not installed on the host: `scripts/kicad-cli` runs
`kicad-cli` from the `kicad/kicad:10.0.6-full` image, pinned by digest.
```

Replace:

```
The firmware identifies the chip itself, since the board model is not yet known. It runs
```

with:

```
The firmware identifies the chip itself: GY-521 boards normally carry an MPU-6050, but clones
ship with other parts. It runs
```

Replace:

```
**Fallback, when the board breaks out no `INT` pin** (the GY-91 does not): run the IMU's FIFO
at 1 kHz and drain it in bursts every 10 ms. Each sample's timestamp is reconstructed from its
index and the burst's read time.
```

with:

```
The board is a GY-521, which breaks out `INT`, so the FIFO fallback once considered here for
boards without an `INT` pin is not built.
```

Replace:

```
These need to be read off labels or measured. None of them blocks stage A except the
perfboard size, which the placement drawing needs.

- IMU board model. Firmware test A1 settles it.
- Perfboard dimensions.
```

with:

```
These need to be read off labels or measured. None of them blocks stage A except the trial
board's hole count, which the placement drawing needs.

- The IMU chip on the GY-521. Firmware test A1 settles it.
- The 6 × 9 cm trial board's hole count, and the BlackPill's row spacing. Measure both.
- The 10 × 10 cm stage-B board, not yet bought.
```

- [ ] **Step 11: Commit**

```bash
git add tools/kicad_gen scripts/kicad-cli hardware/joint_bench docs/hardware/f411-tmc2209-wiring.md docs/superpowers/specs/2026-10-16-joint0-bench-design.md
git commit -m "feat(hardware): generate the stage-A bench schematic from nets.yaml"
```

---

### Task 3: The placement board and the soldering checklist

**Files:**
- Create: `tools/kicad_gen/kicad_gen/board.py`, `tools/kicad_gen/kicad_gen/checklist.py`, `hardware/joint_bench/trial_6x9.yaml`, `hardware/joint_bench/README.md`
- Modify: `tools/kicad_gen/generate.py` (replace whole file)
- Test: `tools/kicad_gen/tests/test_board.py`, `tools/kicad_gen/tests/test_checklist.py`
- Generated (commit them): `hardware/joint_bench/trial_6x9.kicad_pcb`, `trial_6x9_checklist.md`, `exports/trial_6x9_top.pdf`, `exports/trial_6x9_top.svg`, `exports/trial_6x9_bottom_mirrored.pdf`, `exports/trial_6x9_bottom_mirrored.svg`

**Interfaces:**
- Consumes: `Design`, `Part` (Task 1); `schematic.uid` (Task 2); `netlist.parse` output (Task 1).
- Produces:
  - `kicad_gen.board`: `@dataclass Placement(cols, rows, pitch, margin, row_spacing, parts)` where `parts[ref] = {"at": (col, row), "dir": "right"|"down", "span": int}`; `load_placement(path) -> Placement`; `pad_holes(part, spec, placement) -> dict[str, tuple[int, int]]`; `validate(design, placement) -> list[str]`; `build(design, placement, title) -> list`; `write(design, placement, path, title)`.
  - `kicad_gen.checklist`: `render(design, placement, exported, board_name) -> str`; `shorts_to_check(on_board, holes) -> list[tuple[str, str]]`.

- [ ] **Step 1: Write the failing tests**

`tools/kicad_gen/tests/test_board.py`:

```python
from kicad_gen.board import Placement, pad_holes, validate
from kicad_gen.design import from_dict

D = from_dict({"parts": {"U1": {"kind": "blackpill"},
                         "J1": {"kind": "header", "pins": ["A", "B", "C"]},
                         "R1": {"kind": "resistor"}},
               "nets": {"N": ["J1.A", "R1.1"]}})


def _p(**parts):
    return Placement(cols=30, rows=20, pitch=2.54, margin=2.5, row_spacing=6, parts=parts)


def test_blackpill_headers_run_side_by_side_row_spacing_apart():
    holes = pad_holes(D.parts["U1"], {"at": (2, 5), "dir": "right", "span": 4}, _p())
    assert holes["1"] == (2, 5) and holes["20"] == (21, 5)
    assert holes["21"] == (2, 11) and holes["40"] == (21, 11)


def test_headers_and_resistors_follow_their_direction():
    assert pad_holes(D.parts["J1"], {"at": (3, 3), "dir": "down", "span": 4}, _p())["3"] == (3, 5)
    assert pad_holes(D.parts["R1"], {"at": (3, 3), "dir": "right", "span": 4}, _p())["2"] == (7, 3)


def test_validate_catches_parts_off_the_board_and_shared_holes():
    problems = validate(D, _p(J1={"at": (28, 0), "dir": "right", "span": 4},
                              R1={"at": (28, 0), "dir": "down", "span": 4}))
    assert any("off the board" in p for p in problems)
    assert any("share hole (28, 0)" in p for p in problems)


def test_validate_catches_a_placed_part_missing_from_the_design():
    problems = validate(D, _p(X9={"at": (0, 0), "dir": "right", "span": 4}))
    assert problems == ["X9 is placed but not in nets.yaml"]


def test_build_gives_every_pad_its_net():
    from kicad_gen.board import build
    tree = build(D, _p(J1={"at": (2, 2), "dir": "right", "span": 4},
                       R1={"at": (2, 6), "dir": "right", "span": 4}), "t")
    pads = {}
    for fp in (n for n in tree if isinstance(n, list) and n[0] == "footprint"):
        ref = next(c[2] for c in fp if isinstance(c, list) and c[:2] == ["property", "Reference"])
        for pad in (c for c in fp if isinstance(c, list) and c[0] == "pad"):
            net = next((c[2] for c in pad if isinstance(c, list) and c[0] == "net"), None)
            pads[f"{ref}.{pad[1]}"] = net
    assert pads == {"J1.1": "N", "J1.2": None, "J1.3": None, "R1.1": "N", "R1.2": None}
```

`tools/kicad_gen/tests/test_checklist.py`:

```python
from kicad_gen.checklist import shorts_to_check


def test_power_pairs_come_first_then_only_nets_on_neighbouring_holes():
    on_board = {"GND": ["J1.1", "U1.19"], "+3V3": ["J1.2", "U1.20"], "SCL": ["J1.3", "U1.14"],
                "SDA": ["J2.1", "U1.15"]}
    holes = {"J1.1": (0, 0), "J1.2": (1, 0), "J1.3": (2, 0), "U1.19": (10, 10), "U1.20": (11, 10),
             "U1.14": (5, 5), "U1.15": (6, 5), "J2.1": (20, 20)}
    assert shorts_to_check(on_board, holes) == [("GND", "+3V3"), ("+3V3", "SCL"), ("SCL", "SDA")]
```

- [ ] **Step 2: Run them and verify they fail**

Run: `cd tools/kicad_gen && python3 -m pytest -q tests/test_board.py tests/test_checklist.py`
Expected: FAIL, `ModuleNotFoundError: No module named 'kicad_gen.board'`.

- [ ] **Step 3: Write the placement board**

`tools/kicad_gen/kicad_gen/board.py`:

```python
"""Write a placement board: the perfboard drawn hole for hole, with every part where it goes.

This is not a PCB to fabricate. It exists to be printed 1:1 and laid under the perfboard,
and its mirrored bottom view is what you wire from, since the wires go on the solder side.
Footprints are generated here rather than taken from KiCad's libraries so the drawing does
not depend on which library version is installed, and pads sit exactly on the hole grid.
"""

from dataclasses import dataclass

import yaml

from .schematic import uid
from .sexpr import Sym, dumps

FORMAT_VERSION = 20240108  # KiCad 8 format; KiCad 9 and 10 read it unchanged
ORIGIN = (20.0, 20.0)  # sheet position of the board's top-left corner, in mm
PAD = 1.8
DRILL = 1.0
_DIRS = {"right": (1, 0), "down": (0, 1)}


@dataclass
class Placement:
    cols: int
    rows: int
    pitch: float
    margin: float
    row_spacing: int  # holes between the BlackPill's two headers
    parts: dict  # ref -> {"at": (col, row), "dir": str, "span": int}


def load_placement(path):
    with open(path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    board = data["board"]
    parts = {}
    for ref, spec in data["parts"].items():
        col, row = spec["at"]
        parts[ref] = {"at": (int(col), int(row)), "dir": spec.get("dir", "right"),
                      "span": int(spec.get("span", 4))}
    return Placement(cols=int(board["cols"]), rows=int(board["rows"]),
                     pitch=float(board.get("pitch", 2.54)), margin=float(board.get("margin", 2.5)),
                     row_spacing=int(data.get("blackpill_row_spacing", 6)), parts=parts)


def pad_holes(part, spec, placement):
    """Return {pin number: (col, row)} for every pad of a placed part."""
    col, row = spec["at"]
    if part.kind == "blackpill":
        holes = {}
        for k in range(20):
            holes[str(k + 1)] = (col + k, row)
            holes[str(k + 21)] = (col + k, row + placement.row_spacing)
        return holes
    dx, dy = _DIRS[spec["dir"]]
    if part.kind == "resistor":
        span = spec["span"]
        return {"1": (col, row), "2": (col + dx * span, row + dy * span)}
    return {str(i + 1): (col + dx * i, row + dy * i) for i in range(len(part.pins))}


def validate(design, placement):
    """Every part on the board must sit inside it and no two pads may share a hole."""
    problems = []
    taken = {}
    for ref, spec in placement.parts.items():
        if ref not in design.parts:
            problems.append(f"{ref} is placed but not in nets.yaml")
            continue
        for number, (col, row) in pad_holes(design.parts[ref], spec, placement).items():
            if not (0 <= col < placement.cols and 0 <= row < placement.rows):
                problems.append(f"{ref}.{number} at ({col}, {row}) is off the board")
            if (col, row) in taken:
                other = taken[(col, row)]
                problems.append(f"{ref}.{number} and {other} share hole ({col}, {row})")
            taken[(col, row)] = f"{ref}.{number}"
    return problems


def _xy(placement, col, row):
    x0, y0 = ORIGIN
    return (x0 + placement.margin + col * placement.pitch,
            y0 + placement.margin + row * placement.pitch)


def _stroke(width=0.12):
    return [Sym("stroke"), [Sym("width"), width], [Sym("type"), Sym("solid")]]


def _text(kind, text, x, y, layer, key, size=0.8, angle=0, mirror=False):
    effects = [Sym("effects"), [Sym("font"), [Sym("size"), size, size], [Sym("thickness"), 0.12]]]
    if mirror:
        effects.append([Sym("justify"), Sym("mirror")])
    node = [Sym(kind)]
    if kind == "fp_text":
        node.append(Sym("user"))
    node += [text, [Sym("at"), x, y, angle], [Sym("layer"), layer], [Sym("uuid"), uid(*key)],
             effects]
    return node


def _footprint(ref, part, spec, placement, design):
    holes = pad_holes(part, spec, placement)
    anchor = _xy(placement, *spec["at"])
    rel = {n: (_xy(placement, c, r)[0] - anchor[0], _xy(placement, c, r)[1] - anchor[1])
           for n, (c, r) in holes.items()}
    xs = [p[0] for p in rel.values()]
    ys = [p[1] for p in rel.values()]
    half = placement.pitch / 2
    if part.kind == "blackpill":
        # The module overhangs its pin field: 53.0 x 20.9 mm against 48.26 x 15.24 mm.
        x1, x2 = min(xs) - 2.37, max(xs) + 2.37
        y1, y2 = min(ys) - 2.83, max(ys) + 2.83
    else:
        x1, x2 = min(xs) - half, max(xs) + half
        y1, y2 = min(ys) - half, max(ys) + half
    node = [
        Sym("footprint"), f"joint_bench:{part.kind}_{ref}", [Sym("layer"), "F.Cu"],
        [Sym("uuid"), uid("fp", ref)], [Sym("at"), anchor[0], anchor[1]],
        [Sym("property"), "Reference", ref, [Sym("at"), x1, y1 - 1.0, 0],
         [Sym("layer"), "F.SilkS"],
         [Sym("uuid"), uid("fpref", ref)],
         [Sym("effects"), [Sym("font"), [Sym("size"), 1.0, 1.0], [Sym("thickness"), 0.15]],
          [Sym("justify"), Sym("left")]]],
        [Sym("property"), "Value", part.value, [Sym("at"), x1, y2 + 1.0, 0],
         [Sym("layer"), "F.Fab"],
         [Sym("uuid"), uid("fpval", ref)],
         [Sym("effects"), [Sym("font"), [Sym("size"), 0.8, 0.8], [Sym("thickness"), 0.12]],
          [Sym("justify"), Sym("left")]]],
    ]
    for layer in ("F.SilkS", "B.SilkS"):
        node.append([Sym("fp_rect"), [Sym("start"), x1, y1], [Sym("end"), x2, y2], _stroke(),
                     [Sym("fill"), Sym("none")], [Sym("layer"), layer],
                     [Sym("uuid"), uid("rect", ref, layer)]])
    if part.kind == "blackpill":
        mid = (y1 + y2) / 2
        node.append(_text("fp_text", "USB", x1 + 1.2, mid, "F.SilkS", ("usb", ref), 1.0, 90))
        node.append(_text("fp_text", "SWD", x2 - 1.2, mid, "F.SilkS", ("swd", ref), 1.0, 90))
    for number, (px, py) in rel.items():
        name = part.pins[int(number) - 1]
        label = number if name == "~" else name
        # Names sit beside their pad, outward from the part, so neighbours never overlap.
        if part.kind == "blackpill":
            ty = py - 2.6 if int(number) <= 20 else py + 2.6
            tx, angle = px, 90
        elif spec["dir"] == "down":
            tx, ty, angle = px + 2.2, py, 0
        else:
            tx, ty, angle = px, py - 2.4, 90
        node.append(_text("fp_text", label, tx, ty, "F.Fab", ("ft", ref, number), 0.7, angle))
        node.append(_text("fp_text", label, tx, ty, "B.Fab", ("bt", ref, number), 0.7, angle,
                          True))
        net = design.net_of(ref, number)
        shape = "rect" if number == "1" else "circle"
        pad = [Sym("pad"), number, Sym("thru_hole"), Sym(shape), [Sym("at"), px, py],
               [Sym("size"), PAD, PAD], [Sym("drill"), DRILL], [Sym("layers"), "*.Cu", "*.Mask"]]
        if net:
            pad.append([Sym("net"), _net_codes(design)[net], net])
        pad.append([Sym("uuid"), uid("pad", ref, number)])
        node.append(pad)
    return node


def _net_codes(design):
    return {net: i + 1 for i, net in enumerate(sorted(design.nets))}


def build(design, placement, title):
    x0, y0 = ORIGIN
    width = 2 * placement.margin + (placement.cols - 1) * placement.pitch
    height = 2 * placement.margin + (placement.rows - 1) * placement.pitch
    layers = [
        [0, "F.Cu", Sym("signal")], [31, "B.Cu", Sym("signal")],
        [36, "B.SilkS", Sym("user"), "B.Silkscreen"], [37, "F.SilkS", Sym("user"), "F.Silkscreen"],
        [38, "B.Mask", Sym("user")], [39, "F.Mask", Sym("user")],
        [40, "Dwgs.User", Sym("user"), "User.Drawings"], [44, "Edge.Cuts", Sym("user")],
        [48, "B.Fab", Sym("user")], [49, "F.Fab", Sym("user")],
    ]
    nodes = [
        Sym("kicad_pcb"), [Sym("version"), FORMAT_VERSION], [Sym("generator"), "kicad_gen"],
        [Sym("generator_version"), "1.0"],
        [Sym("general"), [Sym("thickness"), 1.6], [Sym("legacy_teardrops"), Sym("no")]],
        [Sym("paper"), "A4"], [Sym("title_block"), [Sym("title"), title]],
        [Sym("layers")] + layers,
        [Sym("setup"), [Sym("pad_to_mask_clearance"), 0]],
        [Sym("net"), 0, ""],
    ]
    for net, code in _net_codes(design).items():
        nodes.append([Sym("net"), code, net])
    for ref, spec in placement.parts.items():
        nodes.append(_footprint(ref, design.parts[ref], spec, placement, design))
    nodes.append([Sym("gr_rect"), [Sym("start"), x0, y0], [Sym("end"), x0 + width, y0 + height],
                  [Sym("stroke"), [Sym("width"), 0.15], [Sym("type"), Sym("default")]],
                  [Sym("fill"), Sym("none")], [Sym("layer"), "Edge.Cuts"],
                  [Sym("uuid"), uid("edge")]])
    for col in range(placement.cols):
        for row in range(placement.rows):
            cx, cy = _xy(placement, col, row)
            nodes.append([Sym("gr_circle"), [Sym("center"), cx, cy], [Sym("end"), cx + 0.5, cy],
                          [Sym("stroke"), [Sym("width"), 0.1], [Sym("type"), Sym("solid")]],
                          [Sym("fill"), Sym("none")], [Sym("layer"), "Dwgs.User"],
                          [Sym("uuid"), uid("hole", col, row)]])
    # Every fifth column and row is numbered outside the edge, on both fab layers, so a
    # hole can be found by counting from the nearest number on either side of the board.
    for col in range(0, placement.cols, 5):
        cx, _ = _xy(placement, col, 0)
        for layer, mirror in (("F.Fab", False), ("B.Fab", True)):
            nodes.append(_text("gr_text", str(col), cx, y0 - 1.5, layer, ("colnum", col, layer),
                               1.0, 0, mirror))
    for row in range(0, placement.rows, 5):
        _, cy = _xy(placement, 0, row)
        for layer, mirror in (("F.Fab", False), ("B.Fab", True)):
            nodes.append(_text("gr_text", str(row), x0 - 2.0, cy, layer, ("rownum", row, layer),
                               1.0, 0, mirror))
    return nodes


def write(design, placement, path, title):
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(dumps(build(design, placement, title)) + "\n")
```

- [ ] **Step 4: Write the checklist**

`tools/kicad_gen/kicad_gen/checklist.py`:

```python
"""Render the soldering and continuity checklist for one placed board.

The nets come from the netlist KiCad exported from the schematic, not from nets.yaml, so
the list describes what was drawn. Only parts placed on this board appear: a net whose
other end is on a part this board does not carry is listed with the pins it does have.
"""

from .board import pad_holes

POWER = ("GND", "+3V3", "+5V")


def render(design, placement, exported, board_name):
    lines = [
        f"# {board_name} — soldering checklist",
        "",
        "Generated by `python -m kicad_gen` from `hardware/joint_bench/nets.yaml` through the",
        "schematic's netlist. Do not edit; regenerate. Holes are (col, row) from the top-left",
        "hole seen from the component side; the mirrored bottom drawing shows the same numbers.",
        "",
        "## Placement",
        "",
        "| Ref | Part | Pin 1 hole | Runs |",
        "|---|---|---|---|",
    ]
    holes = {}
    for ref, spec in placement.parts.items():
        part = design.parts[ref]
        for number, hole in pad_holes(part, spec, placement).items():
            holes[f"{ref}.{number}"] = hole
        runs = "two rows, see drawing" if part.kind == "blackpill" else spec["dir"]
        lines.append(f"| {ref} | {part.value} | {spec['at'][0]}, {spec['at'][1]} | {runs} |")
    lines += ["", "## Wiring, one net at a time", ""]
    on_board = {}
    for net in sorted(exported):
        nodes = [n for n in sorted(exported[net], key=_key) if n in holes]
        if len(nodes) >= 2:
            on_board[net] = nodes
    for net, nodes in on_board.items():
        lines.append(f"### {net}")
        lines.append("")
        for node in nodes:
            ref, _, number = node.partition(".")
            name = design.parts[ref].pins[int(number) - 1]
            label = f"pin {number}" if name == "~" else f"pin {number} ({name})"
            col, row = holes[node]
            lines.append(f"- [ ] {ref} {label} — hole {col}, {row}")
        lines.append("")
    lines += [
        "## Continuity, board unpowered, multimeter on beep",
        "",
        "Each line must beep end to end.",
        "",
    ]
    for net, nodes in on_board.items():
        lines.append(f"- [ ] {net}: " + " — ".join(nodes))
    lines += [
        "",
        "## Shorts, board unpowered",
        "",
        "None of these may beep. The power pairs come first: a short between the rails",
        "destroys the BlackPill's regulator the moment USB is plugged in. The rest are nets",
        "with pads in neighbouring holes, which is where solder bridges actually form.",
        "",
    ]
    for pair in shorts_to_check(on_board, holes):
        lines.append(f"- [ ] {pair[0]} to {pair[1]}")
    return "\n".join(lines) + "\n"


def shorts_to_check(on_board, holes):
    """Power-rail pairs, then every pair of nets that own pads in adjacent holes."""
    power = [p for p in POWER if p in on_board]
    pairs = [(a, b) for i, a in enumerate(power) for b in power[i + 1:]]
    net_at = {holes[node]: net for net, nodes in on_board.items() for node in nodes}
    adjacent = set()
    for (col, row), net in net_at.items():
        for neighbour in ((col + 1, row), (col, row + 1)):
            other = net_at.get(neighbour)
            if other and other != net:
                adjacent.add(tuple(sorted((net, other))))
    seen = {frozenset(p) for p in pairs}
    pairs += [p for p in sorted(adjacent) if frozenset(p) not in seen]
    return pairs


def _key(node):
    ref, _, number = node.partition(".")
    return (ref, int(number))
```

- [ ] **Step 5: Run the tests and verify they pass**

Run: `cd tools/kicad_gen && python3 -m pytest -q tests && python3 -m flake8 --max-line-length 99 .`
Expected: `21 passed`; flake8 prints nothing.

- [ ] **Step 6: Write the trial board's placement and the README**

`hardware/joint_bench/trial_6x9.yaml`:

```yaml
# Placement on the 6 x 9 cm trial perfboard (stage A only). Holes are (col, row) counted
# from 0 at the top-left hole, seen from the component side, long edge horizontal.
# Count your board's holes and correct cols/rows before printing the drawing.
board: {cols: 34, rows: 22, pitch: 2.54, margin: 2.5}

# Holes between the BlackPill's two pin rows. 6 is 15.24 mm (0.6"); measure yours.
blackpill_row_spacing: 6

parts:
  U1: {at: [2, 5]}                 # pin 1 = PB12, at the USB end
  J1: {at: [31, 3], dir: down}     # IMU cable leaves the right edge
  JP1: {at: [30, 12], dir: right}
  R1: {at: [24, 12], dir: right, span: 4}
  JP2: {at: [30, 15], dir: right}
  R2: {at: [24, 15], dir: right, span: 4}
  J2: {at: [3, 19], dir: right}    # logic analyzer, CH0 at the left
  J3: {at: [17, 19], dir: right}   # 3V3, 5V, GND test points
```

`hardware/joint_bench/README.md`:

```markdown
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
```

- [ ] **Step 7: Replace the generator with the full version**

`tools/kicad_gen/generate.py`:

```python
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
    "GY-521 cable, 30-60 cm: J1 VCC to VCC, GND to GND, SCL to SCL, SDA to SDA, INT to INT.",
    "Leave the GY-521's XDA, XCL and AD0 unconnected. Twist SCL with GND, SDA with VCC.",
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
```

- [ ] **Step 8: Generate and inspect**

Run: `python3 tools/kicad_gen/generate.py`
Expected: `OK: 8 parts, 13 nets, ERC clean, netlist matches nets.yaml`; `hardware/joint_bench/trial_6x9_checklist.md` exists and lists 13 nets under "Wiring".

Render the drawings and look at them: `pdftoppm -png -r 150 hardware/joint_bench/exports/trial_6x9_top.pdf /tmp/top` and the same for `_bottom_mirrored`. Check: every pad sits on a grid circle; U1's two rows are six holes apart; pin names on the bottom view read normally (they are mirrored on the layer so the mirrored plot un-mirrors them); no two parts overlap.

- [ ] **Step 9: [HUMAN] Check the drawing against the real parts**

Ask the user to do steps 1-3 of `docs/hardware/joint-bench-stage-a-runbook.md` §1 (written in Task 12; until then, use this list):

1. Count the trial board's holes. If not 34 × 22, correct `board:` in `trial_6x9.yaml`.
2. Measure the BlackPill's row spacing with calipers. If not 15.24 mm, set `blackpill_row_spacing`.
3. Hold the BlackPill USB end left and compare its silkscreen with `exports/trial_6x9_top.pdf`. Top row `B12 B13 … 5V G 3V3`, bottom row `VB C13 … 3V3 G 5V`. If it differs, fix `BLACKPILL_PINS` in `design.py` before anything is soldered.

If anything changed, regenerate (`python3 tools/kicad_gen/generate.py`) and re-inspect as in Step 8.

- [ ] **Step 10: Commit**

```bash
git add tools/kicad_gen hardware/joint_bench
git commit -m "feat(hardware): draw the 6x9 trial board and its soldering checklist"
```

---
### Task 4: Firmware toolchain, the host test build, and the three queues

**Files:**
- Create: `docker/dockerfile-firmware`, `scripts/fwbuild`, `codes_stm32/lib/CMakeLists.txt`, `codes_stm32/lib/include/jb/cycle_clock.hpp`, `codes_stm32/lib/include/jb/spsc_queue.hpp`, `codes_stm32/lib/include/jb/line_ring.hpp`
- Test: `codes_stm32/lib/tests/test_cycle_clock.cpp`, `codes_stm32/lib/tests/test_spsc_queue.cpp`, `codes_stm32/lib/tests/test_line_ring.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `scripts/fwbuild` (target build, needs Task 10), `scripts/fwbuild --test` (host tests of `lib/`), `scripts/fwbuild --log`. Success prints `TESTS PASSED in <s>s — 100% tests passed, 0 tests failed out of <n>`.
  - `jb::CycleClock(uint32_t hz)`: `uint64_t extend(uint32_t raw)`, `uint64_t to_us(uint64_t cycles) const`. Not reentrant.
  - `jb::SpscQueue<T, N>` (N a power of two): `bool push(const T&)` (producer), `bool pop(T*)` (consumer), `uint32_t size() const`.
  - `jb::LineRing<N>` (N a power of two): `bool push(const char*, uint32_t)` (whole line or nothing), `uint32_t peek(const uint8_t**) const`, `void consume(uint32_t)`, `uint32_t size() const`, `uint32_t dropped() const`.

- [ ] **Step 1: Write the toolchain image**

`docker/dockerfile-firmware`:

```dockerfile
# Toolchain for codes_stm32. It compile-checks the STM32 target and runs the
# host-side unit tests of lib/; flashing and running on hardware happen on the bench
# machine. Base image and packages are pinned so a rebuild months from now produces the
# same compiler: an unpinned toolchain is how a working build breaks without a commit.
FROM ubuntu@sha256:5ec03bb3441e8b0bf3b4f9cd4629a1ae763010dc3035bb8da3ae6cf026486401

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        gcc-arm-none-eabi=15:10.3-2021.07-4 \
        libnewlib-arm-none-eabi=3.3.0-1.3 \
        libstdc++-arm-none-eabi-newlib=15:10.3-2021.07-4+17 \
        binutils-arm-none-eabi=2.38-3ubuntu1+15build1 \
        cmake=3.22.1-1ubuntu1.22.04.2 \
        ninja-build=1.10.1-1 \
        g++=4:11.2.0-1ubuntu1 \
        make=4.3-4.1build1 \
        libgtest-dev=1.11.0-3 \
    && rm -rf /var/lib/apt/lists/*
```

The versions are what Ubuntu 22.04 ships, read with `apt-cache policy` inside the pinned base image; `apt-get install` fails loudly if any of them stops being available, which is the point.

- [ ] **Step 2: Write the build wrapper**

`scripts/fwbuild`:

```bash
#!/usr/bin/env bash
# Build the joint0 bench firmware, or run its host-side unit tests, inside the pinned
# firmware image (docker/dockerfile-firmware), and print only what is worth reading: one
# line on success, the decisive errors on failure. The full log is always kept.
#
# Usage:
#   fwbuild            compile the STM32 target (needs the CubeMX project generated)
#   fwbuild --test     build and run the lib/ unit tests with the host compiler
#   fwbuild --log      print the path of the last full log
set -uo pipefail

SELF="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REPO="${BIZON_REPO:-$(dirname "$SELF")}"
FW="$REPO/codes_stm32"
LOGDIR="${BIZON_LOGDIR:-$HOME/.cache/bizon-build}"
LAST="$LOGDIR/fw-last.log"
DOCKERFILE="$REPO/docker/dockerfile-firmware"
# The tag carries the Dockerfile's hash, so editing the pins rebuilds the image instead of
# silently reusing the old one.
IMAGE="bizon-firmware:$(sha256sum "$DOCKERFILE" | cut -c1-12)"

mkdir -p "$LOGDIR"

case "${1:-}" in
  --log) echo "$LAST"; exit 0 ;;
  --test) MODE=test ;;
  "") MODE=build ;;
  *) echo "usage: fwbuild [--test|--log]" >&2; exit 2 ;;
esac

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "building $IMAGE ..."
  docker build -q -t "$IMAGE" -f "$DOCKERFILE" "$REPO/docker" >/dev/null || exit 1
fi

if [ "$MODE" = test ]; then
  INNER="cmake -S lib -B /tmp/host -G Ninja && cmake --build /tmp/host && ctest --test-dir /tmp/host --output-on-failure"
else
  if [ ! -f "$FW/CMakePresets.json" ]; then
    echo "no CubeMX project in $FW yet; generate it from codes_stm32.ioc first" >&2
    exit 1
  fi
  # A throwaway build directory, so a build here never fights with the bench machine's
  # IDE over build/ in the shared checkout.
  INNER="cmake --preset Debug -B /tmp/target && cmake --build /tmp/target"
fi

START=$(date +%s)
docker run --rm -u "$(id -u):$(id -g)" -v "$REPO:$REPO" -w "$FW" "$IMAGE" bash -c "$INNER" >"$LAST" 2>&1
RC=$?
ELAPSED=$(( $(date +%s) - START ))

if [ $RC -eq 0 ]; then
  if [ "$MODE" = test ]; then
    echo "TESTS PASSED in ${ELAPSED}s — $(grep -E 'tests passed' "$LAST" | tail -1)"
  else
    echo "BUILD OK in ${ELAPSED}s — $(grep -cE 'warning:' "$LAST") warnings"
  fi
  echo "full log: $LAST"
  exit 0
fi

echo "=== ${MODE^^} FAILED (exit $RC) after ${ELAPSED}s"
grep -nE "error:|CMake Error|FAILED|undefined reference|Failure|Expected|Which is" "$LAST" | head -25
echo "--- tail:"
tail -n 15 "$LAST"
echo "full log: $LAST"
exit $RC
```

Run: `chmod +x scripts/fwbuild`

- [ ] **Step 3: Write the host test build**

`codes_stm32/lib/CMakeLists.txt`:

```cmake
# Host build of lib/: the firmware's pure logic, compiled with the host compiler and tested
# with GoogleTest. The target build compiles the same sources from the CubeMX project's
# top-level CMakeLists.txt; this file is never used for the target.
#
# Sources and tests are globbed so that adding a module is adding its two files, with no
# edit here. CONFIGURE_DEPENDS makes a new file trigger a re-glob on the next build.
cmake_minimum_required(VERSION 3.22)
project(joint_bench_lib CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

file(GLOB JB_SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB JB_TESTS CONFIGURE_DEPENDS tests/*.cpp)

enable_testing()
find_package(GTest REQUIRED)
add_executable(jb_tests ${JB_SOURCES} ${JB_TESTS})
target_include_directories(jb_tests PRIVATE include)
target_link_libraries(jb_tests PRIVATE GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(jb_tests)
```

- [ ] **Step 4: Write the failing tests**

`codes_stm32/lib/tests/test_cycle_clock.cpp`:

```cpp
#include <gtest/gtest.h>

#include "jb/cycle_clock.hpp"

TEST(CycleClock, PassesThroughBeforeTheFirstWrap)
{
  jb::CycleClock clock(96000000);
  EXPECT_EQ(clock.extend(1000), 1000u);
  EXPECT_EQ(clock.extend(5000), 5000u);
}

TEST(CycleClock, CarriesIntoTheHighWordWhenTheCounterWraps)
{
  jb::CycleClock clock(96000000);
  clock.extend(0xFFFFFF00u);
  EXPECT_EQ(clock.extend(0x10u), 0x100000010ull);
  clock.extend(0xFFFFFFF0u);
  EXPECT_EQ(clock.extend(0x0u), 0x200000000ull);
}

TEST(CycleClock, StaysMonotonicAcrossManyWraps)
{
  jb::CycleClock clock(96000000);
  uint64_t last = 0;
  uint32_t raw = 0;
  for (int i = 0; i < 10000; ++i) {
    raw += 1500000000u;  // under one wrap per call, as the contract requires
    const uint64_t now = clock.extend(raw);
    EXPECT_GT(now, last);
    last = now;
  }
}

TEST(CycleClock, ConvertsCyclesToMicrosecondsAt96MHz)
{
  jb::CycleClock clock(96000000);
  EXPECT_EQ(clock.to_us(96), 1u);
  EXPECT_EQ(clock.to_us(95), 0u);
  EXPECT_EQ(clock.to_us(96000000ull * 3600), 3600000000ull);
}
```

`codes_stm32/lib/tests/test_spsc_queue.cpp`:

```cpp
#include <gtest/gtest.h>

#include "jb/spsc_queue.hpp"

TEST(SpscQueue, PopsInPushOrder)
{
  jb::SpscQueue<int, 4> q;
  EXPECT_TRUE(q.push(1));
  EXPECT_TRUE(q.push(2));
  int v = 0;
  EXPECT_TRUE(q.pop(&v));
  EXPECT_EQ(v, 1);
  EXPECT_TRUE(q.pop(&v));
  EXPECT_EQ(v, 2);
  EXPECT_FALSE(q.pop(&v));
}

TEST(SpscQueue, RefusesAPushWhenFullAndKeepsTheContents)
{
  jb::SpscQueue<int, 4> q;
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(q.push(i));
  }
  EXPECT_FALSE(q.push(99));
  EXPECT_EQ(q.size(), 4u);
  int v = -1;
  EXPECT_TRUE(q.pop(&v));
  EXPECT_EQ(v, 0);
}

TEST(SpscQueue, SurvivesIndexWrapAround)
{
  jb::SpscQueue<uint32_t, 8> q;
  for (uint32_t i = 0; i < 100000; ++i) {
    ASSERT_TRUE(q.push(i));
    uint32_t v = 0;
    ASSERT_TRUE(q.pop(&v));
    ASSERT_EQ(v, i);
  }
  EXPECT_EQ(q.size(), 0u);
}
```

`codes_stm32/lib/tests/test_line_ring.cpp`:

```cpp
#include <gtest/gtest.h>

#include <string>

#include "jb/line_ring.hpp"

namespace
{
template<uint32_t N>
std::string drain(jb::LineRing<N> & ring)
{
  std::string out;
  const uint8_t * p = nullptr;
  while (uint32_t n = ring.peek(&p)) {
    out.append(reinterpret_cast<const char *>(p), n);
    ring.consume(n);
  }
  return out;
}
}  // namespace

TEST(LineRing, DeliversPushedLinesInOrder)
{
  jb::LineRing<64> ring;
  EXPECT_TRUE(ring.push("a\n", 2));
  EXPECT_TRUE(ring.push("bc\n", 3));
  EXPECT_EQ(drain(ring), "a\nbc\n");
}

TEST(LineRing, DropsALineThatDoesNotFitWholeAndCountsIt)
{
  jb::LineRing<8> ring;
  EXPECT_TRUE(ring.push("12345\n", 6));
  EXPECT_FALSE(ring.push("abc\n", 4));  // 2 bytes free, 4 needed: nothing is written
  EXPECT_EQ(ring.dropped(), 1u);
  EXPECT_EQ(drain(ring), "12345\n");
}

TEST(LineRing, SplitsPeekAtTheBufferEndButKeepsTheBytes)
{
  jb::LineRing<8> ring;
  ring.push("123456", 6);
  const uint8_t * p = nullptr;
  ring.consume(ring.peek(&p));
  EXPECT_TRUE(ring.push("abcd", 4));  // wraps: 2 bytes at the end, 2 at the start
  EXPECT_EQ(ring.peek(&p), 2u);
  EXPECT_EQ(drain(ring), "abcd");
}

TEST(LineRing, PeekOnEmptyReturnsZero)
{
  jb::LineRing<8> ring;
  const uint8_t * p = nullptr;
  EXPECT_EQ(ring.peek(&p), 0u);
}
```

- [ ] **Step 5: Run them and verify they fail**

Run: `scripts/fwbuild --test`
Expected: the first run builds the image (a few minutes), then `=== TEST FAILED` with `fatal error: jb/cycle_clock.hpp: No such file or directory`.

- [ ] **Step 6: Write the three headers**

`codes_stm32/lib/include/jb/cycle_clock.hpp`:

```cpp
#pragma once

#include <cstdint>

namespace jb
{

// Extends the 32-bit DWT cycle counter to 64 bits and converts cycles to microseconds.
// At 84 MHz the raw counter wraps every 51.1 s, far shorter than a recording, so every
// timestamp goes through here. The extension is correct only if extend() sees the counter
// at least once per wrap; the main loop calls it every iteration, which guarantees that.
//
// Not reentrant. The firmware calls it with interrupts masked, because the IMU interrupt
// and the main loop both take timestamps and an interleaved update would lose a wrap.
class CycleClock
{
public:
  // hz must be a whole number of MHz (84 MHz here); to_us() divides by hz / 1e6.
  explicit constexpr CycleClock(uint32_t hz)
  : cycles_per_us_(hz / 1000000u) {}

  uint64_t extend(uint32_t raw)
  {
    if (raw < last_) {
      high_ += 0x100000000ull;
    }
    last_ = raw;
    return high_ + raw;
  }

  uint64_t to_us(uint64_t cycles) const {return cycles / cycles_per_us_;}

private:
  uint32_t cycles_per_us_;
  uint32_t last_ = 0;
  uint64_t high_ = 0;
};

}  // namespace jb
```

`codes_stm32/lib/include/jb/spsc_queue.hpp`:

```cpp
#pragma once

#include <atomic>
#include <cstdint>

namespace jb
{

// Lock-free queue for exactly one producer and one consumer. The IMU's DMA-complete
// interrupt pushes samples and the main loop pops them, so the interrupt never waits on
// USB and the main loop never runs with interrupts masked. N must be a power of two so
// the free-running indices stay correct across their own wrap.
template<typename T, uint32_t N>
class SpscQueue
{
  static_assert(N != 0 && (N & (N - 1)) == 0, "N must be a power of two");

public:
  // Producer side. Returns false, and stores nothing, when the queue is full.
  bool push(const T & value)
  {
    const uint32_t head = head_.load(std::memory_order_relaxed);
    const uint32_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail == N) {
      return false;
    }
    buf_[head & (N - 1)] = value;
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  // Consumer side. Returns false when the queue is empty.
  bool pop(T * out)
  {
    const uint32_t tail = tail_.load(std::memory_order_relaxed);
    const uint32_t head = head_.load(std::memory_order_acquire);
    if (head == tail) {
      return false;
    }
    *out = buf_[tail & (N - 1)];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  uint32_t size() const
  {
    return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
  }

private:
  T buf_[N] {};
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> tail_{0};
};

}  // namespace jb
```

`codes_stm32/lib/include/jb/line_ring.hpp`:

```cpp
#pragma once

#include <cstdint>

namespace jb
{

// Byte ring holding whole text lines on their way to USB. A line that does not fit is
// dropped whole and counted: the host then sees a gap in `seq`, which it can count,
// instead of half a line spliced onto the next, which it would misparse. Used from the
// main loop only, so it needs no atomics. N must be a power of two.
template<uint32_t N>
class LineRing
{
  static_assert(N != 0 && (N & (N - 1)) == 0, "N must be a power of two");

public:
  bool push(const char * text, uint32_t len)
  {
    if (len > N - size()) {
      ++dropped_;
      return false;
    }
    for (uint32_t i = 0; i < len; ++i) {
      buf_[(head_ + i) & (N - 1)] = static_cast<uint8_t>(text[i]);
    }
    head_ += len;
    return true;
  }

  // The longest readable run that is contiguous in memory, so USB can send it in place.
  // The bytes stay valid until consume() is called, which must not happen before the
  // transfer that reads them has completed.
  uint32_t peek(const uint8_t ** data) const
  {
    const uint32_t used = size();
    if (used == 0) {
      return 0;
    }
    const uint32_t start = tail_ & (N - 1);
    const uint32_t run = (N - start < used) ? N - start : used;
    *data = &buf_[start];
    return run;
  }

  void consume(uint32_t len) {tail_ += len;}
  uint32_t size() const {return head_ - tail_;}
  uint32_t dropped() const {return dropped_;}

private:
  uint8_t buf_[N] {};
  uint32_t head_ = 0;
  uint32_t tail_ = 0;
  uint32_t dropped_ = 0;
};

}  // namespace jb
```

- [ ] **Step 7: Run the tests and verify they pass**

Run: `scripts/fwbuild --test`
Expected: `TESTS PASSED ... out of 11`.

- [ ] **Step 8: Commit**

```bash
git add docker/dockerfile-firmware scripts/fwbuild codes_stm32/lib
git commit -m "build(firmware): add the pinned firmware toolchain and the lib/ host test build"
```

---

### Task 5: Text formatting and telemetry lines

**Files:**
- Create: `codes_stm32/lib/include/jb/text_writer.hpp`, `codes_stm32/lib/src/text_writer.cpp`, `codes_stm32/lib/include/jb/telemetry.hpp`, `codes_stm32/lib/src/telemetry.cpp`
- Test: `codes_stm32/lib/tests/test_telemetry.cpp`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces:
  - `jb::TextWriter(char* buf, uint32_t cap)`: `put(char)`, `str(std::string_view)`, `u64(uint64_t)`, `i32(int32_t)`, `hex8(uint8_t)` (two lower-case digits), `bool ok() const`, `uint32_t size() const`.
  - `struct jb::ImuSample { uint32_t seq; uint64_t t_us; int16_t accel[3]; int16_t gyro[3]; }`.
  - `constexpr uint32_t jb::kMaxLine = 96`.
  - `uint32_t jb::format_sample(char*, uint32_t cap, const ImuSample&)`, `uint32_t jb::format_event(char*, uint32_t cap, uint64_t t_us, std::string_view name)`; both return 0 when the line does not fit.

- [ ] **Step 1: Write the failing test**

`codes_stm32/lib/tests/test_telemetry.cpp`:

```cpp
#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "jb/telemetry.hpp"
#include "jb/text_writer.hpp"

TEST(TextWriter, FormatsExtremeIntegers)
{
  char buf[64];
  jb::TextWriter w(buf, sizeof(buf));
  w.u64(0);
  w.put(' ');
  w.u64(UINT64_MAX);
  w.put(' ');
  w.i32(INT32_MIN);
  w.put(' ');
  w.hex8(0x0a);
  ASSERT_TRUE(w.ok());
  EXPECT_EQ(std::string(buf, w.size()), "0 18446744073709551615 -2147483648 0a");
}

TEST(TextWriter, NeverWritesPastCapacityAndReportsIt)
{
  char buf[8] = {'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x'};
  jb::TextWriter w(buf, 4);
  w.str("123456");
  EXPECT_FALSE(w.ok());
  EXPECT_EQ(w.size(), 4u);
  EXPECT_EQ(buf[4], 'x');
}

TEST(Telemetry, FormatsASampleLine)
{
  jb::ImuSample s{42, 123456789012ull, {-1, 2, -16384}, {0, 32767, -32768}};
  char buf[jb::kMaxLine];
  const uint32_t n = jb::format_sample(buf, sizeof(buf), s);
  EXPECT_EQ(std::string(buf, n), "T,42,123456789012,-1,2,-16384,0,32767,-32768\n");
}

TEST(Telemetry, TheLongestSampleFitsInMaxLine)
{
  jb::ImuSample s{UINT32_MAX, UINT64_MAX, {-32768, -32768, -32768}, {-32768, -32768, -32768}};
  char buf[jb::kMaxLine];
  EXPECT_EQ(jb::format_sample(buf, sizeof(buf), s), 76u);
}

TEST(Telemetry, ReturnsZeroRatherThanATruncatedLine)
{
  jb::ImuSample s{1, 2, {3, 4, 5}, {6, 7, 8}};
  char buf[10];
  EXPECT_EQ(jb::format_sample(buf, sizeof(buf), s), 0u);
}

TEST(Telemetry, FormatsAnEventLine)
{
  char buf[jb::kMaxLine];
  const uint32_t n = jb::format_event(buf, sizeof(buf), 7, "mark:tap-1");
  EXPECT_EQ(std::string(buf, n), "E,7,mark:tap-1\n");
}
```

- [ ] **Step 2: Run it and verify it fails**

Run: `scripts/fwbuild --test`
Expected: `=== TEST FAILED` with `fatal error: jb/telemetry.hpp: No such file or directory`.

- [ ] **Step 3: Write the implementation**

`codes_stm32/lib/include/jb/text_writer.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <string_view>

namespace jb
{

// Appends text and integers to a fixed buffer. newlib-nano's printf, which the firmware
// links, has no %llu, and the timestamps are 64-bit, so numbers are formatted by hand.
// Writing never passes `cap`; once anything fails to fit, ok() stays false and the caller
// discards the line instead of sending a truncated one.
class TextWriter
{
public:
  TextWriter(char * buf, uint32_t cap)
  : buf_(buf), cap_(cap) {}

  void put(char c);
  void str(std::string_view text);
  void u64(uint64_t value);
  void i32(int32_t value);
  void hex8(uint8_t value);  // always two digits, lower case, no prefix

  bool ok() const {return ok_;}
  uint32_t size() const {return len_;}

private:
  char * buf_;
  uint32_t cap_;
  uint32_t len_ = 0;
  bool ok_ = true;
};

}  // namespace jb
```

`codes_stm32/lib/src/text_writer.cpp`:

```cpp
#include "jb/text_writer.hpp"

namespace jb
{

void TextWriter::put(char c)
{
  if (!ok_ || len_ == cap_) {
    ok_ = false;
    return;
  }
  buf_[len_++] = c;
}

void TextWriter::str(std::string_view text)
{
  for (char c : text) {
    put(c);
  }
}

void TextWriter::u64(uint64_t value)
{
  char digits[20];
  int n = 0;
  do {
    digits[n++] = static_cast<char>('0' + value % 10);
    value /= 10;
  } while (value != 0);
  while (n > 0) {
    put(digits[--n]);
  }
}

void TextWriter::i32(int32_t value)
{
  // Negate in 64 bits: -INT32_MIN does not fit in an int32_t.
  int64_t wide = value;
  if (wide < 0) {
    put('-');
    wide = -wide;
  }
  u64(static_cast<uint64_t>(wide));
}

void TextWriter::hex8(uint8_t value)
{
  static constexpr char kHex[] = "0123456789abcdef";
  put(kHex[value >> 4]);
  put(kHex[value & 0x0f]);
}

}  // namespace jb
```

`codes_stm32/lib/include/jb/telemetry.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <string_view>

namespace jb
{

// One IMU reading as it leaves the interrupt: raw counts, no scaling. The scale travels
// once, in the `imu info` reply, rather than in every line.
struct ImuSample
{
  uint32_t seq;
  uint64_t t_us;
  int16_t accel[3];
  int16_t gyro[3];
};

// Longest possible line is 76 bytes ("T," + 10 + "," + 20 + 6 x 7 + "\n"); 96 leaves room.
constexpr uint32_t kMaxLine = 96;

// "T,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n". Returns the length, or 0 if it does
// not fit in cap.
uint32_t format_sample(char * buf, uint32_t cap, const ImuSample & s);

// "E,<t_us>,<name>\n". Returns the length, or 0 if it does not fit in cap.
uint32_t format_event(char * buf, uint32_t cap, uint64_t t_us, std::string_view name);

}  // namespace jb
```

`codes_stm32/lib/src/telemetry.cpp`:

```cpp
#include "jb/telemetry.hpp"

#include "jb/text_writer.hpp"

namespace jb
{

uint32_t format_sample(char * buf, uint32_t cap, const ImuSample & s)
{
  TextWriter w(buf, cap);
  w.str("T,");
  w.u64(s.seq);
  w.put(',');
  w.u64(s.t_us);
  for (int16_t v : s.accel) {
    w.put(',');
    w.i32(v);
  }
  for (int16_t v : s.gyro) {
    w.put(',');
    w.i32(v);
  }
  w.put('\n');
  return w.ok() ? w.size() : 0;
}

uint32_t format_event(char * buf, uint32_t cap, uint64_t t_us, std::string_view name)
{
  TextWriter w(buf, cap);
  w.str("E,");
  w.u64(t_us);
  w.put(',');
  w.str(name);
  w.put('\n');
  return w.ok() ? w.size() : 0;
}

}  // namespace jb
```

- [ ] **Step 4: Run the tests and verify they pass**

Run: `scripts/fwbuild --test`
Expected: `TESTS PASSED ... out of 17`.

- [ ] **Step 5: Commit**

```bash
git add codes_stm32/lib
git commit -m "feat(firmware): format telemetry lines without printf"
```

---

### Task 6: Command input — line assembly and the parser

**Files:**
- Create: `codes_stm32/lib/include/jb/line_assembler.hpp`, `codes_stm32/lib/src/line_assembler.cpp`, `codes_stm32/lib/include/jb/command.hpp`, `codes_stm32/lib/src/command.cpp`
- Test: `codes_stm32/lib/tests/test_line_assembler.cpp`, `codes_stm32/lib/tests/test_command.cpp`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces:
  - `jb::LineAssembler`: `static constexpr uint32_t kMax = 64`; `enum class Result { None, Line, Overflow }`; `Result feed(char)`; `std::string_view line() const` (valid until the next `feed`).
  - `enum class jb::CmdKind { Invalid, Help, Ver, I2cScan, ImuInfo, ImuCfg, StreamOn, StreamOff, Mark, Stats }`.
  - `struct jb::ImuCfgArgs { uint16_t rate_hz = 0; uint16_t dlpf_hz = 0; uint8_t fs_g = 0; }` (0 = keep current).
  - `struct jb::Command { CmdKind kind; ImuCfgArgs cfg; char label[24]; const char* error; }`.
  - `jb::Command jb::parse_command(std::string_view line)`.

- [ ] **Step 1: Write the failing tests**

`codes_stm32/lib/tests/test_line_assembler.cpp`:

```cpp
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "jb/line_assembler.hpp"

namespace
{
// Feeds text and returns what came out: lines as text, overflows as "<overflow>".
std::vector<std::string> feed(jb::LineAssembler & la, const std::string & text)
{
  std::vector<std::string> out;
  for (char c : text) {
    switch (la.feed(c)) {
      case jb::LineAssembler::Result::Line: out.emplace_back(la.line()); break;
      case jb::LineAssembler::Result::Overflow: out.emplace_back("<overflow>"); break;
      case jb::LineAssembler::Result::None: break;
    }
  }
  return out;
}
}  // namespace

TEST(LineAssembler, AcceptsLfCrAndCrlfAndSkipsEmptyLines)
{
  jb::LineAssembler la;
  EXPECT_EQ(feed(la, "ver\nstats\r\n\r\ni2cscan\r"),
    (std::vector<std::string>{"ver", "stats", "i2cscan"}));
}

TEST(LineAssembler, DiscardsAnOverlongLineWholeAndRecovers)
{
  jb::LineAssembler la;
  const std::string longline(jb::LineAssembler::kMax + 10, 'a');
  EXPECT_EQ(feed(la, longline + "\nver\n"), (std::vector<std::string>{"<overflow>", "ver"}));
}

TEST(LineAssembler, AcceptsALineOfExactlyMaxLength)
{
  jb::LineAssembler la;
  const std::string line(jb::LineAssembler::kMax, 'b');
  EXPECT_EQ(feed(la, line + "\n"), (std::vector<std::string>{line}));
}
```

`codes_stm32/lib/tests/test_command.cpp`:

```cpp
#include <gtest/gtest.h>

#include <string>

#include "jb/command.hpp"

using jb::CmdKind;
using jb::parse_command;

TEST(Command, ParsesTheBareCommands)
{
  EXPECT_EQ(parse_command("help").kind, CmdKind::Help);
  EXPECT_EQ(parse_command("ver").kind, CmdKind::Ver);
  EXPECT_EQ(parse_command("i2cscan").kind, CmdKind::I2cScan);
  EXPECT_EQ(parse_command("stats").kind, CmdKind::Stats);
  EXPECT_EQ(parse_command("imu info").kind, CmdKind::ImuInfo);
  EXPECT_EQ(parse_command("stream on").kind, CmdKind::StreamOn);
  EXPECT_EQ(parse_command("stream off").kind, CmdKind::StreamOff);
}

TEST(Command, ToleratesExtraSpaces)
{
  EXPECT_EQ(parse_command("  stream   on  ").kind, CmdKind::StreamOn);
}

TEST(Command, RejectsUnknownWordsAndStrayArguments)
{
  EXPECT_EQ(parse_command("reboot").kind, CmdKind::Invalid);
  EXPECT_STREQ(parse_command("reboot").error, "unknown command");
  EXPECT_EQ(parse_command("ver now").kind, CmdKind::Invalid);
  EXPECT_EQ(parse_command("stream").kind, CmdKind::Invalid);
  EXPECT_EQ(parse_command("imu").kind, CmdKind::Invalid);
  EXPECT_EQ(parse_command("").kind, CmdKind::Invalid);
  EXPECT_EQ(parse_command("a b c d e f g").kind, CmdKind::Invalid);
}

TEST(Command, ParsesImuCfgKeysAndLeavesTheRestAtZero)
{
  const jb::Command c = parse_command("imu cfg rate=500 fs=8");
  ASSERT_EQ(c.kind, CmdKind::ImuCfg);
  EXPECT_EQ(c.cfg.rate_hz, 500);
  EXPECT_EQ(c.cfg.dlpf_hz, 0);
  EXPECT_EQ(c.cfg.fs_g, 8);
}

TEST(Command, RejectsBadImuCfgValues)
{
  EXPECT_STREQ(parse_command("imu cfg").error, "cfg: nothing to set");
  EXPECT_STREQ(parse_command("imu cfg rate").error, "cfg: expected key=value");
  EXPECT_STREQ(parse_command("imu cfg rate=").error, "cfg: bad value for rate");
  EXPECT_STREQ(parse_command("imu cfg rate=-5").error, "cfg: bad value for rate");
  EXPECT_STREQ(parse_command("imu cfg rate=0").error, "cfg: bad value for rate");
  EXPECT_STREQ(parse_command("imu cfg rate=99999").error, "cfg: bad value for rate");
  EXPECT_STREQ(parse_command("imu cfg fs=256").error, "cfg: bad value for fs");
  EXPECT_STREQ(parse_command("imu cfg speed=1").error, "cfg: unknown key");
}

TEST(Command, ParsesMarkWithAndWithoutALabel)
{
  EXPECT_EQ(parse_command("mark").kind, CmdKind::Mark);
  EXPECT_STREQ(parse_command("mark").label, "");
  const jb::Command c = parse_command("mark tap_board-1");
  ASSERT_EQ(c.kind, CmdKind::Mark);
  EXPECT_STREQ(c.label, "tap_board-1");
}

TEST(Command, RejectsALabelThatWouldBreakTheCsvLine)
{
  EXPECT_STREQ(parse_command("mark a,b").error, "mark: label must be [A-Za-z0-9_-]");
  EXPECT_STREQ(parse_command("mark a b").error, "mark: one label at most");
  EXPECT_STREQ(parse_command(("mark " + std::string(24, 'x')).c_str()).error,
    "mark: label too long");
  EXPECT_EQ(parse_command(("mark " + std::string(23, 'x')).c_str()).kind, CmdKind::Mark);
}
```

- [ ] **Step 2: Run them and verify they fail**

Run: `scripts/fwbuild --test`
Expected: `=== TEST FAILED` with `fatal error: jb/line_assembler.hpp: No such file or directory`.

- [ ] **Step 3: Write the implementation**

`codes_stm32/lib/include/jb/line_assembler.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <string_view>

namespace jb
{

// Collects received bytes into command lines. Either CR or LF ends a line, and empty lines
// are skipped, so CR, LF and CRLF terminals all work. A line longer than the buffer is
// discarded whole and reported once at its terminator: executing its first 64 bytes, or
// its tail as a second command, would run something the user never typed.
class LineAssembler
{
public:
  static constexpr uint32_t kMax = 64;

  enum class Result { None, Line, Overflow };

  // On Line, line() holds the text without its terminator until the next call to feed().
  Result feed(char c);
  std::string_view line() const {return std::string_view(buf_, line_len_);}

private:
  char buf_[kMax] {};
  uint32_t len_ = 0;
  uint32_t line_len_ = 0;
  bool overflow_ = false;
};

}  // namespace jb
```

`codes_stm32/lib/src/line_assembler.cpp`:

```cpp
#include "jb/line_assembler.hpp"

namespace jb
{

LineAssembler::Result LineAssembler::feed(char c)
{
  if (c == '\r' || c == '\n') {
    if (overflow_) {
      overflow_ = false;
      len_ = 0;
      return Result::Overflow;
    }
    if (len_ == 0) {
      return Result::None;
    }
    line_len_ = len_;
    len_ = 0;
    return Result::Line;
  }
  if (overflow_) {
    return Result::None;
  }
  if (len_ == kMax) {
    overflow_ = true;
    return Result::None;
  }
  buf_[len_++] = c;
  return Result::None;
}

}  // namespace jb
```

`codes_stm32/lib/include/jb/command.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <string_view>

namespace jb
{

enum class CmdKind
{
  Invalid,
  Help,
  Ver,
  I2cScan,
  ImuInfo,
  ImuCfg,
  StreamOn,
  StreamOff,
  Mark,
  Stats,
};

// Fields left at 0 keep their current value. Whether a value is one the chip supports is
// the IMU driver's call, not the parser's: the parser only knows it is a number.
struct ImuCfgArgs
{
  uint16_t rate_hz = 0;
  uint16_t dlpf_hz = 0;
  uint8_t fs_g = 0;
};

struct Command
{
  CmdKind kind = CmdKind::Invalid;
  ImuCfgArgs cfg;
  char label[24] {};  // mark only; NUL-terminated, empty when no label was given
  const char * error = nullptr;  // set when kind is Invalid
};

// Parses one line from the console. Words are separated by one or more spaces.
//
//   help | ver | i2cscan | stats
//   imu info
//   imu cfg [rate=<Hz>] [dlpf=<Hz>] [fs=<g>]   at least one key
//   stream on | stream off
//   mark [label]                               label: 1-23 chars of [A-Za-z0-9_-]
//
// The label is restricted because it is written into a comma-separated event line; a
// comma in it would shift every field the host parses after it.
Command parse_command(std::string_view line);

}  // namespace jb
```

`codes_stm32/lib/src/command.cpp`:

```cpp
#include "jb/command.hpp"

#include <cstddef>

namespace jb
{
namespace
{

constexpr int kMaxWords = 6;

// string_view::substr and ::copy throw std::out_of_range on a bad index; even with
// -fno-exceptions that drags libstdc++'s throw helpers into the image. Slicing by pointer
// cannot fail, so this file never calls them.
std::string_view slice(std::string_view s, size_t pos, size_t len)
{
  return std::string_view(s.data() + pos, len);
}

int split(std::string_view line, std::string_view * words)
{
  int n = 0;
  size_t i = 0;
  while (i < line.size()) {
    while (i < line.size() && line[i] == ' ') {
      ++i;
    }
    if (i == line.size()) {
      break;
    }
    size_t j = i;
    while (j < line.size() && line[j] != ' ') {
      ++j;
    }
    if (n == kMaxWords) {
      return kMaxWords + 1;
    }
    words[n++] = slice(line, i, j - i);
    i = j;
  }
  return n;
}

// Decimal digits only, 1..max. Anything else, including a sign or an overflow, is false.
bool parse_uint(std::string_view text, uint32_t max, uint32_t * out)
{
  if (text.empty() || text.size() > 5) {
    return false;
  }
  uint32_t value = 0;
  for (char c : text) {
    if (c < '0' || c > '9') {
      return false;
    }
    value = value * 10 + static_cast<uint32_t>(c - '0');
  }
  if (value == 0 || value > max) {
    return false;
  }
  *out = value;
  return true;
}

bool label_char(char c)
{
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
         c == '_' || c == '-';
}

Command invalid(const char * error)
{
  Command cmd;
  cmd.error = error;
  return cmd;
}

Command kind_only(CmdKind kind)
{
  Command cmd;
  cmd.kind = kind;
  return cmd;
}

Command parse_cfg(const std::string_view * words, int n)
{
  if (n == 2) {
    return invalid("cfg: nothing to set");
  }
  Command cmd;
  cmd.kind = CmdKind::ImuCfg;
  for (int i = 2; i < n; ++i) {
    const size_t eq = words[i].find('=');
    if (eq == std::string_view::npos) {
      return invalid("cfg: expected key=value");
    }
    const std::string_view key = slice(words[i], 0, eq);
    const std::string_view val = slice(words[i], eq + 1, words[i].size() - eq - 1);
    uint32_t v = 0;
    if (key == "rate") {
      if (!parse_uint(val, 65535, &v)) {return invalid("cfg: bad value for rate");}
      cmd.cfg.rate_hz = static_cast<uint16_t>(v);
    } else if (key == "dlpf") {
      if (!parse_uint(val, 65535, &v)) {return invalid("cfg: bad value for dlpf");}
      cmd.cfg.dlpf_hz = static_cast<uint16_t>(v);
    } else if (key == "fs") {
      if (!parse_uint(val, 255, &v)) {return invalid("cfg: bad value for fs");}
      cmd.cfg.fs_g = static_cast<uint8_t>(v);
    } else {
      return invalid("cfg: unknown key");
    }
  }
  return cmd;
}

Command parse_mark(const std::string_view * words, int n)
{
  Command cmd;
  cmd.kind = CmdKind::Mark;
  if (n == 1) {
    return cmd;
  }
  if (n > 2) {
    return invalid("mark: one label at most");
  }
  const std::string_view label = words[1];
  if (label.size() >= sizeof(cmd.label)) {
    return invalid("mark: label too long");
  }
  for (char c : label) {
    if (!label_char(c)) {
      return invalid("mark: label must be [A-Za-z0-9_-]");
    }
  }
  for (size_t i = 0; i < label.size(); ++i) {
    cmd.label[i] = label[i];
  }
  cmd.label[label.size()] = '\0';
  return cmd;
}

}  // namespace

Command parse_command(std::string_view line)
{
  std::string_view words[kMaxWords];
  const int n = split(line, words);
  if (n == 0) {
    return invalid("empty line");
  }
  if (n > kMaxWords) {
    return invalid("too many words");
  }
  const std::string_view head = words[0];
  if (head == "mark") {
    return parse_mark(words, n);
  }
  if (head == "imu") {
    if (n >= 2 && words[1] == "cfg") {
      return parse_cfg(words, n);
    }
    if (n == 2 && words[1] == "info") {
      return kind_only(CmdKind::ImuInfo);
    }
    return invalid("imu: expected info or cfg");
  }
  if (head == "stream") {
    if (n == 2 && words[1] == "on") {
      return kind_only(CmdKind::StreamOn);
    }
    if (n == 2 && words[1] == "off") {
      return kind_only(CmdKind::StreamOff);
    }
    return invalid("stream: expected on or off");
  }
  CmdKind kind = CmdKind::Invalid;
  if (head == "help") {
    kind = CmdKind::Help;
  } else if (head == "ver") {
    kind = CmdKind::Ver;
  } else if (head == "i2cscan") {
    kind = CmdKind::I2cScan;
  } else if (head == "stats") {
    kind = CmdKind::Stats;
  } else {
    return invalid("unknown command");
  }
  if (n != 1) {
    return invalid("this command takes no arguments");
  }
  return kind_only(kind);
}

}  // namespace jb
```

- [ ] **Step 4: Run the tests and verify they pass**

Run: `scripts/fwbuild --test`
Expected: `TESTS PASSED ... out of 27`.

- [ ] **Step 5: Commit**

```bash
git add codes_stm32/lib
git commit -m "feat(firmware): parse console commands and reject input that would corrupt the stream"
```

---

### Task 7: MPU-6050 / MPU-6500 register logic

**Files:**
- Create: `codes_stm32/lib/include/jb/mpu.hpp`, `codes_stm32/lib/src/mpu.cpp`
- Test: `codes_stm32/lib/tests/test_mpu.cpp`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces (namespace `jb::mpu`):
  - `kAddrLow = 0x68`, `kAddrHigh = 0x69`, `kResetValue = 0x80`, `kBlockLen = 14`; `namespace reg { kSmplrtDiv, kConfig, kGyroConfig, kAccelConfig, kAccelConfig2, kIntPinCfg, kIntEnable, kAccelXoutH, kPwrMgmt1, kWhoAmI }`.
  - `enum class Chip : uint8_t { Unknown, Mpu6050, Mpu6500 }`; `Chip identify(uint8_t who_am_i)`; `const char* chip_name(Chip)`.
  - `struct Config { uint16_t rate_hz; uint16_t dlpf_hz; uint8_t fs_g; }`; `constexpr Config kDefault{1000, 188, 4}`.
  - `struct RegWrite { uint8_t reg, value; }`; `struct Plan { RegWrite writes[8]; uint8_t count; Config accepted; uint16_t lsb_per_g; }`.
  - `const char* plan_config(Chip, const Config&, Plan*)` (nullptr on success).
  - `struct Raw { int16_t accel[3]; int16_t temp; int16_t gyro[3]; }`; `Raw parse_block(const uint8_t*)`.

- [ ] **Step 1: Write the failing test**

`codes_stm32/lib/tests/test_mpu.cpp`:

```cpp
#include <gtest/gtest.h>

#include <map>

#include "jb/mpu.hpp"

using jb::mpu::Chip;

namespace
{
std::map<uint8_t, uint8_t> writes(const jb::mpu::Plan & p)
{
  std::map<uint8_t, uint8_t> m;
  for (uint8_t i = 0; i < p.count; ++i) {
    m[p.writes[i].reg] = p.writes[i].value;
  }
  return m;
}
}  // namespace

TEST(Mpu, IdentifiesTheSupportedChipsAndNothingElse)
{
  EXPECT_EQ(jb::mpu::identify(0x68), Chip::Mpu6050);
  EXPECT_EQ(jb::mpu::identify(0x70), Chip::Mpu6500);
  EXPECT_EQ(jb::mpu::identify(0x71), Chip::Mpu6500);
  EXPECT_EQ(jb::mpu::identify(0x98), Chip::Unknown);
  EXPECT_EQ(jb::mpu::identify(0x00), Chip::Unknown);
}

TEST(Mpu, PlansTheDefaultConfigurationForAnMpu6050)
{
  jb::mpu::Plan p{};
  ASSERT_EQ(jb::mpu::plan_config(Chip::Mpu6050, jb::mpu::kDefault, &p), nullptr);
  const auto w = writes(p);
  EXPECT_EQ(w.at(jb::mpu::reg::kPwrMgmt1), 0x01);   // gyro PLL clock
  EXPECT_EQ(w.at(jb::mpu::reg::kSmplrtDiv), 0);     // 1 kHz
  EXPECT_EQ(w.at(jb::mpu::reg::kConfig), 1);        // 184 Hz, nearest to 188
  EXPECT_EQ(w.at(jb::mpu::reg::kAccelConfig), 0x08);  // +-4 g
  EXPECT_EQ(w.at(jb::mpu::reg::kIntPinCfg), 0x10);
  EXPECT_EQ(w.at(jb::mpu::reg::kIntEnable), 0x01);
  EXPECT_EQ(w.count(jb::mpu::reg::kAccelConfig2), 0u);  // not an MPU-6050 register
  EXPECT_EQ(p.accepted.rate_hz, 1000);
  EXPECT_EQ(p.accepted.dlpf_hz, 184);
  EXPECT_EQ(p.lsb_per_g, 8192);
}

TEST(Mpu, WritesTheSeparateAccelFilterOnAnMpu6500)
{
  jb::mpu::Plan p{};
  ASSERT_EQ(jb::mpu::plan_config(Chip::Mpu6500, {500, 100, 2}, &p), nullptr);
  const auto w = writes(p);
  EXPECT_EQ(w.at(jb::mpu::reg::kSmplrtDiv), 1);     // 500 Hz
  EXPECT_EQ(w.at(jb::mpu::reg::kAccelConfig2), 2);  // 99 Hz
  EXPECT_EQ(p.accepted.dlpf_hz, 99);
  EXPECT_EQ(p.lsb_per_g, 16384);
}

TEST(Mpu, RejectsWhatTheChipCannotDo)
{
  jb::mpu::Plan p{};
  EXPECT_STREQ(jb::mpu::plan_config(Chip::Mpu6050, {300, 188, 4}, &p), "rate must divide 1000 Hz");
  EXPECT_STREQ(jb::mpu::plan_config(Chip::Mpu6050, {2000, 188, 4}, &p), "rate must divide 1000 Hz");
  EXPECT_STREQ(jb::mpu::plan_config(Chip::Mpu6050, {2, 188, 4}, &p), "rate must divide 1000 Hz");
  EXPECT_STREQ(jb::mpu::plan_config(Chip::Mpu6050, {1000, 188, 3}, &p), "fs must be 2, 4, 8 or 16");
  EXPECT_STREQ(jb::mpu::plan_config(Chip::Unknown, jb::mpu::kDefault, &p), "unknown chip");
}

TEST(Mpu, DecodesTheBigEndianBlock)
{
  const uint8_t block[14] = {0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x12, 0x34,
    0x7F, 0xFF, 0x00, 0x00, 0xC0, 0x00};
  const jb::mpu::Raw r = jb::mpu::parse_block(block);
  EXPECT_EQ(r.accel[0], 1);
  EXPECT_EQ(r.accel[1], -1);
  EXPECT_EQ(r.accel[2], -32768);
  EXPECT_EQ(r.temp, 0x1234);
  EXPECT_EQ(r.gyro[0], 32767);
  EXPECT_EQ(r.gyro[1], 0);
  EXPECT_EQ(r.gyro[2], -16384);
}
```

- [ ] **Step 2: Run it and verify it fails**

Run: `scripts/fwbuild --test`
Expected: `=== TEST FAILED` with `fatal error: jb/mpu.hpp: No such file or directory`.

- [ ] **Step 3: Write the implementation**

`codes_stm32/lib/include/jb/mpu.hpp`:

```cpp
#pragma once

#include <cstdint>

// Register-level logic for the InvenSense MPU-6050 (GY-521) and the MPU-6500 family
// (MPU-6500, MPU-9250), kept free of I/O so it is testable on the host. The driver in
// drivers/ performs the writes this module plans.
namespace jb::mpu
{

constexpr uint8_t kAddrLow = 0x68;   // AD0 low, the GY-521 default
constexpr uint8_t kAddrHigh = 0x69;  // AD0 high

namespace reg
{
constexpr uint8_t kSmplrtDiv = 0x19;
constexpr uint8_t kConfig = 0x1A;
constexpr uint8_t kGyroConfig = 0x1B;
constexpr uint8_t kAccelConfig = 0x1C;
constexpr uint8_t kAccelConfig2 = 0x1D;  // MPU-6500 family only
constexpr uint8_t kIntPinCfg = 0x37;
constexpr uint8_t kIntEnable = 0x38;
constexpr uint8_t kAccelXoutH = 0x3B;  // start of the 14-byte data block
constexpr uint8_t kPwrMgmt1 = 0x6B;
constexpr uint8_t kWhoAmI = 0x75;
}  // namespace reg

constexpr uint8_t kResetValue = 0x80;  // PWR_MGMT_1.DEVICE_RESET; wait 100 ms after it
constexpr uint8_t kBlockLen = 14;      // accel xyz, temperature, gyro xyz, big-endian

enum class Chip : uint8_t { Unknown, Mpu6050, Mpu6500 };

// WHO_AM_I: 0x68 MPU-6050, 0x70 MPU-6500, 0x71 MPU-9250 (an MPU-6500 core plus a
// magnetometer). Clone parts answer other values; those are reported, not guessed at.
Chip identify(uint8_t who_am_i);
const char * chip_name(Chip chip);

struct Config
{
  uint16_t rate_hz;  // output data rate
  uint16_t dlpf_hz;  // accelerometer low-pass bandwidth; the nearest supported is used
  uint8_t fs_g;      // accelerometer full scale: 2, 4, 8 or 16
};

constexpr Config kDefault{1000, 188, 4};

struct RegWrite
{
  uint8_t reg;
  uint8_t value;
};

struct Plan
{
  RegWrite writes[8];
  uint8_t count;
  Config accepted;      // what the chip will actually do, echoed back to the user
  uint16_t lsb_per_g;   // raw accelerometer counts per g at the accepted full scale
};

// Plans the register writes that put the chip in `want`, to be applied after a reset.
// Returns nullptr on success, or a message naming the value that cannot be met.
const char * plan_config(Chip chip, const Config & want, Plan * out);

struct Raw
{
  int16_t accel[3];
  int16_t temp;
  int16_t gyro[3];
};

// Decodes the 14-byte block read from ACCEL_XOUT_H.
Raw parse_block(const uint8_t * block);

}  // namespace jb::mpu
```

`codes_stm32/lib/src/mpu.cpp`:

```cpp
#include "jb/mpu.hpp"

#include <cstddef>

namespace jb::mpu
{
namespace
{

struct Option
{
  uint8_t code;
  uint16_t hz;
};

// DLPF_CFG 0 and 7 are excluded on purpose: on both families they switch the gyro to an
// 8 kHz internal rate, which changes what SMPLRT_DIV divides and breaks the rate maths.
constexpr Option kMpu6050Dlpf[] = {{1, 184}, {2, 94}, {3, 44}, {4, 21}, {5, 10}, {6, 5}};
constexpr Option kMpu6500Gyro[] = {{1, 184}, {2, 92}, {3, 41}, {4, 20}, {5, 10}, {6, 5}};
constexpr Option kMpu6500Accel[] = {{1, 218}, {2, 99}, {3, 45}, {4, 21}, {5, 10}, {6, 5}};

template<std::size_t N>
const Option & nearest(const Option (&options)[N], uint16_t hz)
{
  const Option * best = &options[0];
  for (const Option & o : options) {
    const int d = o.hz > hz ? o.hz - hz : hz - o.hz;
    const int bd = best->hz > hz ? best->hz - hz : hz - best->hz;
    if (d < bd) {
      best = &o;
    }
  }
  return *best;
}

}  // namespace

Chip identify(uint8_t who_am_i)
{
  switch (who_am_i) {
    case 0x68: return Chip::Mpu6050;
    case 0x70:
    case 0x71: return Chip::Mpu6500;
    default: return Chip::Unknown;
  }
}

const char * chip_name(Chip chip)
{
  switch (chip) {
    case Chip::Mpu6050: return "MPU6050";
    case Chip::Mpu6500: return "MPU6500";
    default: return "unknown";
  }
}

const char * plan_config(Chip chip, const Config & want, Plan * out)
{
  if (chip == Chip::Unknown) {
    return "unknown chip";
  }
  // With the DLPF on, the output rate is 1 kHz / (1 + SMPLRT_DIV), divider 0..255.
  if (want.rate_hz == 0 || want.rate_hz > 1000 || 1000 % want.rate_hz != 0 ||
    1000 / want.rate_hz > 256)
  {
    return "rate must divide 1000 Hz";
  }
  uint8_t fs_bits = 0;
  uint16_t lsb = 0;
  switch (want.fs_g) {
    case 2: fs_bits = 0u << 3; lsb = 16384; break;
    case 4: fs_bits = 1u << 3; lsb = 8192; break;
    case 8: fs_bits = 2u << 3; lsb = 4096; break;
    case 16: fs_bits = 3u << 3; lsb = 2048; break;
    default: return "fs must be 2, 4, 8 or 16";
  }
  if (want.dlpf_hz == 0) {
    return "dlpf must be positive";
  }
  Plan p{};
  auto add = [&p](uint8_t r, uint8_t v) {p.writes[p.count++] = RegWrite{r, v};};
  // CLKSEL = 1: the X-gyro PLL, far more accurate than the internal 8 MHz oscillator,
  // and the sample interval follows it.
  add(reg::kPwrMgmt1, 0x01);
  add(reg::kSmplrtDiv, static_cast<uint8_t>(1000 / want.rate_hz - 1));
  uint16_t accel_hz = 0;
  if (chip == Chip::Mpu6050) {
    // One DLPF setting serves both sensors. EXT_SYNC_SET stays 0, so FSYNC is ignored.
    const Option & o = nearest(kMpu6050Dlpf, want.dlpf_hz);
    add(reg::kConfig, o.code);
    accel_hz = o.hz;
  } else {
    const Option & g = nearest(kMpu6500Gyro, want.dlpf_hz);
    const Option & a = nearest(kMpu6500Accel, want.dlpf_hz);
    add(reg::kConfig, g.code);
    accel_hz = a.hz;
  }
  add(reg::kGyroConfig, 1u << 3);  // +-500 dps; streamed, but no metric uses it
  add(reg::kAccelConfig, fs_bits);
  if (chip == Chip::Mpu6500) {
    add(reg::kAccelConfig2, nearest(kMpu6500Accel, want.dlpf_hz).code);
  }
  // Active-high push-pull 50 us pulse per sample (LATCH_INT_EN = 0), status cleared by
  // any read (INT_RD_CLEAR = 1). A pulse cannot get stuck high if a read is ever missed.
  add(reg::kIntPinCfg, 0x10);
  add(reg::kIntEnable, 0x01);  // DATA_RDY_EN
  p.accepted = Config{want.rate_hz, accel_hz, want.fs_g};
  p.lsb_per_g = lsb;
  *out = p;
  return nullptr;
}

Raw parse_block(const uint8_t * b)
{
  auto be16 = [b](int i) {
      return static_cast<int16_t>(static_cast<uint16_t>(b[i] << 8 | b[i + 1]));
    };
  return Raw{{be16(0), be16(2), be16(4)}, be16(6), {be16(8), be16(10), be16(12)}};
}

}  // namespace jb::mpu
```

- [ ] **Step 4: Run the tests and verify they pass**

Run: `scripts/fwbuild --test`
Expected: `TESTS PASSED ... out of 32`.

- [ ] **Step 5: Check that lib/ builds for the target and pulls in nothing heavy**

`lib/` must compile for the Cortex-M4 with the firmware's flags and must not reference libstdc++'s exception helpers (`std::string_view::substr` would). Run:

```bash
docker run --rm -u "$(id -u):$(id -g)" -v "$PWD:$PWD" -w "$PWD/codes_stm32/lib" \
  "bizon-firmware:$(sha256sum docker/dockerfile-firmware | cut -c1-12)" bash -c '
  F="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -std=c++17 -fno-exceptions
     -fno-rtti -fno-threadsafe-statics -Os -Wall -Wextra -Werror -Iinclude"
  for f in src/*.cpp; do arm-none-eabi-g++ $F -c $f -o /tmp/$(basename $f).o || exit 1; done
  arm-none-eabi-nm -u /tmp/*.o | grep -iE "throw|__atomic" || echo CLEAN'
```

Expected: `CLEAN`.

- [ ] **Step 6: Commit**

```bash
git add codes_stm32/lib
git commit -m "feat(firmware): plan MPU-6050 and MPU-6500 configuration as register writes"
```

---
### Task 8: Host tool — protocol parsing and the recorder

**Files:**
- Create: `tools/joint_bench/joint_bench/__init__.py`, `tools/joint_bench/joint_bench/protocol.py`, `tools/joint_bench/joint_bench/recorder.py`
- Test: `tools/joint_bench/tests/fake_port.py`, `tools/joint_bench/tests/test_protocol.py`, `tools/joint_bench/tests/test_recorder.py`

**Interfaces:**
- Consumes: the line protocol (Global Constraints); `tools/requirements.txt` (Task 1).
- Produces:
  - `joint_bench.protocol`: frozen dataclasses `Sample(seq, t_us, accel, gyro)`, `Event(t_us, name)`, `Reply(ok, text)` with `fields() -> dict[str, str]`, `Log(text)`, `Unknown(text)`; `parse_line(str) -> Sample | Event | Reply | Log | Unknown`.
  - `joint_bench.recorder`: `STALL_S = 2.0`; `class RecordError(RuntimeError)`; `git_sha(repo) -> str`; `Link(port, clock=time.monotonic)` with `send(cmd)`, `lines()` (yields `str`, or `None` on a read timeout), `command(cmd, timeout_s=2.0, on_other=None) -> Reply`; `record(link, outdir, seconds, label, repo, marks=()) -> dict` writing `raw.csv`, `events.csv`, `log.txt`, then `meta.yaml` last.
  - The firmware replies the recorder depends on, which Task 11 must produce exactly: `OK ver joint_bench <version>`; `OK imu chip=<name> who=0x<hh> addr=0x<hh> rate=<Hz> dlpf=<Hz> fs=<g> lsb_per_g=<n>`; `OK stream on`; `OK stream off`; `OK stats samples=<n> imu_overrun=<n> i2c_err=<n> queue_drop=<n> tx_drop=<n> rx_drop=<n>`; for `mark [label]` an `E,<t_us>,mark` or `E,<t_us>,mark:<label>` line, then `OK mark`.

- [ ] **Step 1: Write the fake firmware and the failing tests**

`tools/joint_bench/tests/fake_port.py`:

```python
"""A scripted stand-in for the firmware's serial port."""

from collections import deque


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class FakeFirmware:
    """Answers commands like the firmware does and streams samples at 1 kHz while on.

    Each readline() advances the fake clock by one millisecond. `drop` lists seq numbers
    the stream skips, to imitate lost samples. `stall_after` stops the stream after that
    many samples, to imitate a dead link.
    """

    def __init__(self, clock, drop=(), stall_after=None):
        self.clock = clock
        self.out = deque()
        self.streaming = False
        self.seq = 0
        self.drop = set(drop)
        self.stall_after = stall_after
        self.sent = []

    def write(self, data):
        cmd = data.decode().strip()
        self.sent.append(cmd)
        if cmd == "stream on":
            self.streaming = True
            self.out.append("OK stream on\n")
        elif cmd == "stream off":
            self.streaming = False
            self.out.append("OK stream off\n")
        elif cmd == "ver":
            self.out.append("OK ver joint_bench test\n")
        elif cmd == "imu info":
            self.out.append("OK imu chip=MPU6050 who=0x68 addr=0x68 rate=1000 dlpf=184 fs=4"
                            " lsb_per_g=8192\n")
        elif cmd == "stats":
            self.out.append(f"OK stats samples={self.seq} imu_overrun=0 i2c_err=0 tx_drop=0"
                            " queue_drop=0\n")
        elif cmd.startswith("mark"):
            label = cmd.partition(" ")[2]
            name = f"mark:{label}" if label else "mark"
            self.out.append(f"E,{self.seq * 1000 + 300},{name}\n")
            self.out.append("OK mark\n")
        else:
            self.out.append("ERR unknown command\n")

    def readline(self):
        self.clock.t += 0.001
        if self.out:
            return self.out.popleft().encode()
        if self.streaming and (self.stall_after is None or self.seq < self.stall_after):
            seq = self.seq
            self.seq += 1
            if seq in self.drop:
                return self.readline()
            return f"T,{seq},{seq * 1000},0,0,8192,0,0,0\n".encode()
        return b""
```

`tools/joint_bench/tests/test_protocol.py`:

```python
from joint_bench.protocol import Event, Log, Reply, Sample, Unknown, parse_line


def test_parses_a_sample_line():
    assert parse_line("T,42,123456789012,-1,2,-16384,0,32767,-32768\r\n") == Sample(
        42, 123456789012, (-1, 2, -16384), (0, 32767, -32768))


def test_a_truncated_or_garbled_sample_is_unknown_not_an_exception():
    assert isinstance(parse_line("T,42,1234,-1,2"), Unknown)
    assert isinstance(parse_line("T,42,12x4,-1,2,3,4,5,6"), Unknown)


def test_parses_events_replies_and_logs():
    assert parse_line("E,7,mark:tap-1\n") == Event(7, "mark:tap-1")
    assert parse_line("OK stream on") == Reply(True, "stream on")
    assert parse_line("ERR unknown command") == Reply(False, "unknown command")
    assert parse_line("# imu: reset") == Log("imu: reset")
    assert isinstance(parse_line("\x00\x7fgarbage"), Unknown)


def test_reply_fields_are_key_value_pairs():
    reply = parse_line("OK imu chip=MPU6050 who=0x68 rate=1000 lsb_per_g=8192")
    assert reply.fields() == {"chip": "MPU6050", "who": "0x68", "rate": "1000",
                              "lsb_per_g": "8192"}
```

`tools/joint_bench/tests/test_recorder.py`:

```python
import os

import pytest
import yaml

from fake_port import FakeClock, FakeFirmware
from joint_bench.recorder import Link, RecordError, record


def _record(tmp_path, fw, clock, seconds=1, marks=()):
    out = tmp_path / "rec"
    meta = record(Link(fw, clock=clock), str(out), seconds, "test", str(tmp_path), marks=marks)
    return out, meta


def test_records_the_requested_duration_of_firmware_time(tmp_path):
    clock = FakeClock()
    fw = FakeFirmware(clock)
    out, meta = _record(tmp_path, fw, clock, seconds=1)
    rows = (out / "raw.csv").read_text().splitlines()
    assert rows[0] == "seq,t_us,ax,ay,az,gx,gy,gz"
    assert len(rows) - 1 == 1001  # t = 0 .. 1 000 000 us inclusive
    assert meta["imu"]["lsb_per_g"] == "8192"
    assert meta["stats"]["i2c_err"] == "0"
    assert fw.sent[-2:] == ["stream off", "stats"]
    assert yaml.safe_load((out / "meta.yaml").read_text())["label"] == "test"


def test_sends_scheduled_marks_and_records_their_events(tmp_path):
    clock = FakeClock()
    fw = FakeFirmware(clock)
    out, meta = _record(tmp_path, fw, clock, seconds=1, marks=[(0.2, "a7-1"), (0.5, "a7-2")])
    assert "mark a7-1" in fw.sent and "mark a7-2" in fw.sent
    events = (out / "events.csv").read_text().splitlines()
    assert events[0] == "t_us,name"
    assert [e.split(",")[1] for e in events[1:]] == ["mark:a7-1", "mark:a7-2"]
    assert meta["events"] == 2


def test_a_dead_link_raises_and_leaves_no_meta(tmp_path):
    clock = FakeClock()
    fw = FakeFirmware(clock, stall_after=100)
    with pytest.raises(RecordError, match="stalled"):
        _record(tmp_path, fw, clock, seconds=1)
    assert not os.path.exists(tmp_path / "rec" / "meta.yaml")


def test_refuses_to_overwrite_an_existing_recording(tmp_path):
    (tmp_path / "rec").mkdir()
    clock = FakeClock()
    with pytest.raises(FileExistsError):
        _record(tmp_path, FakeFirmware(clock), clock)
```

- [ ] **Step 2: Run them and verify they fail**

Run: `cd tools/joint_bench && python3 -m pytest -q tests`
Expected: FAIL, `ModuleNotFoundError: No module named 'joint_bench'`.

- [ ] **Step 3: Write the implementation**

`tools/joint_bench/joint_bench/__init__.py`:

```python
"""Host side of the joint0 bench: talk to the firmware, record, analyse.

Design: docs/superpowers/specs/2026-10-16-joint0-bench-design.md
"""
```

`tools/joint_bench/joint_bench/protocol.py`:

```python
"""Parse the firmware's line protocol.

Every line the firmware sends is one of: a sample (`T,...`), an event (`E,...`), a command
reply (`OK ...` / `ERR ...`) or a log line (`# ...`). Anything else is line noise and is
returned as `Unknown` so the caller can count it rather than crash on it.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Sample:
    seq: int
    t_us: int
    accel: tuple  # raw counts, x y z
    gyro: tuple   # raw counts, x y z


@dataclass(frozen=True)
class Event:
    t_us: int
    name: str


@dataclass(frozen=True)
class Reply:
    ok: bool
    text: str  # everything after "OK " / "ERR "

    def fields(self):
        """key=value pairs in the reply, e.g. {'chip': 'MPU6050', 'rate': '1000'}."""
        out = {}
        for word in self.text.split():
            key, sep, value = word.partition("=")
            if sep:
                out[key] = value
        return out


@dataclass(frozen=True)
class Log:
    text: str


@dataclass(frozen=True)
class Unknown:
    text: str


def parse_line(line):
    line = line.rstrip("\r\n")
    if line.startswith("T,"):
        parts = line.split(",")
        if len(parts) == 9:
            try:
                nums = [int(p) for p in parts[1:]]
            except ValueError:
                return Unknown(line)
            return Sample(nums[0], nums[1], tuple(nums[2:5]), tuple(nums[5:8]))
        return Unknown(line)
    if line.startswith("E,"):
        parts = line.split(",", 2)
        if len(parts) == 3 and parts[1].isdigit():
            return Event(int(parts[1]), parts[2])
        return Unknown(line)
    if line == "OK" or line.startswith("OK "):
        return Reply(True, line[3:])
    if line == "ERR" or line.startswith("ERR "):
        return Reply(False, line[4:])
    if line.startswith("#"):
        return Log(line[1:].strip())
    return Unknown(line)
```

`tools/joint_bench/joint_bench/recorder.py`:

```python
"""Record one IMU stream to disk.

The recording directory holds raw.csv (samples), events.csv (marks), log.txt (everything
else the firmware said) and meta.yaml (what produced the data). meta.yaml is written last,
so a directory without one is an interrupted recording and the analyser refuses it.
"""

import datetime
import os
import subprocess
import time

import yaml

from .protocol import Event, Reply, Sample, Unknown, parse_line

STALL_S = 2.0  # no line for this long while streaming means the link is gone


class RecordError(RuntimeError):
    pass


def git_sha(repo):
    try:
        out = subprocess.run(["git", "-C", repo, "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, check=True)
        return out.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


class Link:
    """Line-level wrapper around a pyserial-like port: write(bytes), readline() -> bytes."""

    def __init__(self, port, clock=time.monotonic):
        self.port = port
        self.clock = clock

    def send(self, command):
        self.port.write((command + "\n").encode("ascii"))

    def lines(self):
        while True:
            raw = self.port.readline()
            if not raw:
                yield None  # read timeout
                continue
            yield raw.decode("ascii", errors="replace")

    def command(self, command, timeout_s=2.0, on_other=None):
        """Send a command and return its Reply. Lines that arrive first go to on_other."""
        self.send(command)
        deadline = self.clock() + timeout_s
        for line in self.lines():
            if line is None:
                if self.clock() > deadline:
                    raise RecordError(f"no reply to {command!r}")
                continue
            msg = parse_line(line)
            if isinstance(msg, Reply):
                return msg
            if on_other:
                on_other(msg)


def record(link, outdir, seconds, label, repo, marks=()):
    """Stream for `seconds` of firmware time into outdir. Returns the meta dict.

    `marks` is a list of (seconds after the stream starts, label): the recorder sends
    `mark <label>` at each, since it holds the port and nothing else can.
    """
    os.makedirs(outdir, exist_ok=False)
    others = []
    link.command("stream off", on_other=others.append)
    ver = link.command("ver")
    info = link.command("imu info")
    if not info.ok:
        raise RecordError(f"imu info failed: {info.text}")
    fields = info.fields()
    samples = events = unknown = 0
    first_t = last_t = None
    with open(os.path.join(outdir, "raw.csv"), "w", encoding="ascii") as raw, \
            open(os.path.join(outdir, "events.csv"), "w", encoding="ascii") as ev, \
            open(os.path.join(outdir, "log.txt"), "w", encoding="ascii") as log:
        raw.write("seq,t_us,ax,ay,az,gx,gy,gz\n")
        ev.write("t_us,name\n")
        reply = link.command("stream on")
        if not reply.ok:
            raise RecordError(f"stream on failed: {reply.text}")
        start = last_line = link.clock()
        pending = sorted(marks)
        for line in link.lines():
            if pending and link.clock() - start >= pending[0][0]:
                link.send(f"mark {pending.pop(0)[1]}")
            now = link.clock()
            if line is None:
                if now - last_line > STALL_S:
                    raise RecordError(f"stream stalled after {samples} samples")
                continue
            last_line = now
            msg = parse_line(line)
            if isinstance(msg, Sample):
                raw.write(f"{msg.seq},{msg.t_us},{','.join(map(str, msg.accel))},"
                          f"{','.join(map(str, msg.gyro))}\n")
                samples += 1
                first_t = msg.t_us if first_t is None else first_t
                last_t = msg.t_us
                if last_t - first_t >= seconds * 1_000_000:
                    break
            elif isinstance(msg, Event):
                ev.write(f"{msg.t_us},{msg.name}\n")
                events += 1
            elif isinstance(msg, Unknown):
                unknown += 1
                log.write(f"? {msg.text}\n")
            else:
                log.write(f"{msg}\n")
        link.command("stream off", on_other=lambda m: None)
        stats = link.command("stats")
    meta = {
        "label": label,
        "started": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "seconds": seconds,
        "samples": samples,
        "events": events,
        "unknown_lines": unknown,
        "firmware": ver.text,
        "imu": fields,
        "stats": stats.fields(),
        "git": git_sha(repo),
    }
    with open(os.path.join(outdir, "meta.yaml"), "w", encoding="utf-8") as fh:
        yaml.safe_dump(meta, fh, sort_keys=False)
    return meta
```

- [ ] **Step 4: Run the tests and verify they pass**

Run: `cd tools/joint_bench && python3 -m pytest -q tests && python3 -m flake8 --max-line-length 99 .`
Expected: `8 passed`; flake8 prints nothing.

- [ ] **Step 5: Commit**

```bash
git add tools/joint_bench
git commit -m "feat(tools): record the bench firmware's IMU stream to debug/bench"
```

---

### Task 9: Host tool — analysis, report and command line

**Files:**
- Create: `tools/joint_bench/joint_bench/analysis.py`, `tools/joint_bench/joint_bench/report.py`, `tools/joint_bench/joint_bench/__main__.py`
- Test: `tools/joint_bench/tests/test_analysis.py`, `tools/joint_bench/tests/test_report.py`

**Interfaces:**
- Consumes: the recording layout and `meta.yaml` keys `imu.rate`, `imu.lsb_per_g`, `stats.i2c_err`, `label` (Task 8); `recorder.Link`, `recorder.record` (Task 8).
- Produces:
  - `joint_bench.analysis`: `Recording`; `load(path)` (raises `FileNotFoundError` mentioning "interrupted" without `meta.yaml`); `integrity(seq, t_us) -> dict`; `gravity(accel_g) -> (g, magnitude, vertical_axis)`; `horizontal_basis(g)`; `horizontal(accel_g, g) -> (n, 2)`; `noise_rms(h)`; `robust_noise(h)`; `psd(h, fs) -> (f, p)`; `detect_taps(h, fs, noise, k=10.0, quiet_s=0.1) -> list[int]`; `TapResponse(index, peak_g, freq_hz, damping)`; `tap_response(h, fs, index, noise, window_s=0.5)`; `mark_offsets_us(events, t_us) -> list[(name, us)]`.
  - `joint_bench.report`: `analyze(path, noise_from=None) -> dict` (also writes `analysis.json`, `horizontal.png`, `intervals.png`, `psd.png`); `summary(result) -> str` with one `PASS`/`FAIL` line per check.
  - CLI: `python3 -m joint_bench console [--port P]`, `record -s SECONDS -l LABEL [--port P] [--out DIR] [--marks N --mark-start S --mark-every S]`, `analyze DIR [--noise-from DIR]`. The port defaults to the single USB device with VID:PID 0483:5740.

- [ ] **Step 1: Write the failing tests**

`tools/joint_bench/tests/test_analysis.py`:

```python
import numpy as np
import pytest

from joint_bench import analysis

FS = 1000.0


def _rest(n=20000, sigma=0.002, seed=1):
    rng = np.random.default_rng(seed)
    a = rng.normal(0.0, sigma, size=(n, 3))
    a[:, 2] += 1.0  # gravity on z
    return a


def test_integrity_counts_missing_and_out_of_order_samples():
    seq = np.array([0, 1, 2, 5, 6, 6, 7])
    t_us = np.array([0, 1000, 2000, 5000, 6000, 6000, 7000])
    r = analysis.integrity(seq, t_us)
    assert r["missing"] == 2
    assert r["out_of_order"] == 1
    assert r["interval_max_us"] == 3000


def test_gravity_finds_one_g_on_the_vertical_axis_despite_taps():
    a = _rest()
    a[5000:5050, 0] += 0.8  # a tap must not drag the static estimate
    g, magnitude, axis = analysis.gravity(a)
    assert magnitude == pytest.approx(1.0, abs=0.005)
    assert axis == 2


def test_horizontal_removes_gravity_and_keeps_platform_motion():
    a = _rest(sigma=0.0)
    a[:, 0] += 0.1 * np.sin(np.arange(len(a)) / FS * 2 * np.pi * 10)
    g, _, _ = analysis.gravity(a)
    h = analysis.horizontal(a, np.array([0.0, 0.0, 1.0]))
    assert np.max(np.abs(np.hypot(h[:, 0], h[:, 1]))) == pytest.approx(0.1, rel=0.01)


def test_tilted_mounting_still_yields_horizontal_motion_only():
    tilt = np.deg2rad(10)
    g = np.array([np.sin(tilt), 0.0, np.cos(tilt)])
    a = np.tile(g, (1000, 1))
    assert np.max(np.abs(analysis.horizontal(a, g))) < 1e-12


def test_robust_noise_matches_the_true_noise_and_ignores_taps():
    sigma = 0.002
    a = _rest(sigma=sigma)
    g, _, _ = analysis.gravity(a)
    h = analysis.horizontal(a, g)
    clean = analysis.noise_rms(h)
    assert clean == pytest.approx(sigma * np.sqrt(2), rel=0.05)
    h[1000:1100] += 0.5
    assert analysis.robust_noise(h) == pytest.approx(clean, rel=0.05)


def test_psd_peaks_at_the_injected_frequency():
    t = np.arange(20000) / FS
    h = np.column_stack([0.01 * np.sin(2 * np.pi * 63 * t), np.zeros_like(t)])
    f, p = analysis.psd(h, FS)
    assert f[np.argmax(p)] == pytest.approx(63, abs=0.5)


def _tapped(f0, zeta, taps_at, noise=0.0005, n=10000, seed=2):
    rng = np.random.default_rng(seed)
    h = rng.normal(0.0, noise / np.sqrt(2), size=(n, 2))
    t = np.arange(n) / FS
    direction = np.array([0.6, 0.8])
    for t0 in taps_at:
        tt = t - t0
        decay = np.exp(-zeta * 2 * np.pi * f0 * tt)
        ring = np.where(tt >= 0, 0.3 * decay * np.sin(2 * np.pi * f0 * tt), 0)
        h += np.outer(ring, direction)
    return h


def test_detects_each_tap_once():
    h = _tapped(37.0, 0.03, taps_at=[1.0, 3.0, 5.5])
    taps = analysis.detect_taps(h, FS, noise=0.0005)
    assert [round(i / FS, 1) for i in taps] == [1.0, 3.0, 5.5]


def test_recovers_frequency_and_damping_of_the_ringing():
    h = _tapped(37.0, 0.03, taps_at=[1.0])
    tap = analysis.detect_taps(h, FS, noise=0.0005)[0]
    r = analysis.tap_response(h, FS, tap, noise=0.0005)
    assert r.freq_hz == pytest.approx(37.0, abs=0.5)
    assert r.damping == pytest.approx(0.03, rel=0.15)


def test_mark_offsets_measure_time_to_the_next_sample():
    t_us = np.arange(0, 10000, 1000)
    events = [(2300, "mark:a"), (5000, "mark"), (7000, "other")]
    assert analysis.mark_offsets_us(events, t_us) == [("mark:a", 700), ("mark", 0)]


def test_load_refuses_an_interrupted_recording(tmp_path):
    (tmp_path / "raw.csv").write_text("seq,t_us,ax,ay,az,gx,gy,gz\n0,0,0,0,8192,0,0,0\n")
    with pytest.raises(FileNotFoundError, match="interrupted"):
        analysis.load(str(tmp_path))
```

`tools/joint_bench/tests/test_report.py`:

```python
import os

import numpy as np
import yaml

from joint_bench import report


def _write_recording(path, a_g, lsb=8192, rate=1000, drop=()):
    os.makedirs(path)
    n = len(a_g)
    seq = np.array([i for i in range(n + len(drop)) if i not in set(drop)])[:n]
    counts = np.round(a_g * lsb).astype(int)
    with open(os.path.join(path, "raw.csv"), "w") as fh:
        fh.write("seq,t_us,ax,ay,az,gx,gy,gz\n")
        for s, c in zip(seq, counts):
            fh.write(f"{s},{s * 1000},{c[0]},{c[1]},{c[2]},0,0,0\n")
    with open(os.path.join(path, "events.csv"), "w") as fh:
        fh.write("t_us,name\n2300,mark:a7-1\n")
    meta = {"label": "synthetic", "imu": {"rate": str(rate), "lsb_per_g": str(lsb)},
            "stats": {"i2c_err": "0"}}
    with open(os.path.join(path, "meta.yaml"), "w") as fh:
        yaml.safe_dump(meta, fh)


def test_a_clean_tapped_recording_passes_and_reports_the_tap(tmp_path):
    rng = np.random.default_rng(3)
    n = 6000
    t = np.arange(n) / 1000
    a = rng.normal(0, 0.001, size=(n, 3))
    a[:, 2] += 1.0
    tt = t - 2.0
    ring = 0.2 * np.exp(-0.04 * 2 * np.pi * 45 * tt) * np.sin(2 * np.pi * 45 * tt)
    a[:, 0] += np.where(tt >= 0, ring, 0)
    path = str(tmp_path / "rec")
    _write_recording(path, a)
    result = report.analyze(path)
    assert all(result["checks"].values()), result["checks"]
    assert len(result["taps"]) == 1
    assert abs(result["taps"][0]["freq_hz"] - 45) < 1
    assert result["marks"] == [{"name": "mark:a7-1", "to_next_sample_us": 700}]
    for name in ("analysis.json", "horizontal.png", "intervals.png", "psd.png"):
        assert os.path.exists(os.path.join(path, name))
    assert "PASS  A3 no missing samples" in report.summary(result)


def test_missing_samples_fail_a3(tmp_path):
    a = np.tile([0.0, 0.0, 1.0], (3000, 1))
    path = str(tmp_path / "rec")
    _write_recording(path, a, drop=(100, 101))
    result = report.analyze(path)
    assert result["checks"]["A3 no missing samples"] is False
```

- [ ] **Step 2: Run them and verify they fail**

Run: `cd tools/joint_bench && python3 -m pytest -q tests`
Expected: FAIL, `ImportError: cannot import name 'analysis' from 'joint_bench'`.

- [ ] **Step 3: Write the analysis**

`tools/joint_bench/joint_bench/analysis.py`:

```python
"""Analysis of one recording: sample integrity, gravity, noise, spectrum, taps, marks.

Everything here is a pure function of arrays, so it is tested on synthetic signals with
known answers. Accelerations are in g; times in seconds unless a name says otherwise.
"""

import csv
import os
from dataclasses import dataclass

import numpy as np
import yaml
from scipy import signal


@dataclass
class Recording:
    meta: dict
    seq: np.ndarray
    t_s: np.ndarray        # firmware time, seconds from the first sample
    t_us: np.ndarray       # firmware time, raw microseconds
    accel_g: np.ndarray    # shape (n, 3)
    events: list           # [(t_us, name)]


def load(path):
    meta_path = os.path.join(path, "meta.yaml")
    if not os.path.exists(meta_path):
        raise FileNotFoundError(f"{path} has no meta.yaml; the recording was interrupted")
    with open(meta_path, encoding="utf-8") as fh:
        meta = yaml.safe_load(fh)
    data = np.loadtxt(os.path.join(path, "raw.csv"), delimiter=",", skiprows=1,
                      dtype=np.int64, ndmin=2)
    lsb = float(meta["imu"]["lsb_per_g"])
    events = []
    with open(os.path.join(path, "events.csv"), encoding="ascii") as fh:
        for row in csv.DictReader(fh):
            events.append((int(row["t_us"]), row["name"]))
    t_us = data[:, 1]
    return Recording(meta=meta, seq=data[:, 0], t_us=t_us, t_s=(t_us - t_us[0]) / 1e6,
                     accel_g=data[:, 2:5] / lsb, events=events)


def integrity(seq, t_us):
    """Missing samples from seq gaps, and the sample-interval distribution in microseconds."""
    dseq = np.diff(seq)
    dt = np.diff(t_us).astype(float)
    return {
        "samples": int(len(seq)),
        "missing": int(np.sum(dseq[dseq > 1] - 1)),
        "out_of_order": int(np.sum(dseq < 1)),
        "interval_mean_us": float(dt.mean()) if len(dt) else float("nan"),
        "interval_std_us": float(dt.std()) if len(dt) else float("nan"),
        "interval_min_us": float(dt.min()) if len(dt) else float("nan"),
        "interval_max_us": float(dt.max()) if len(dt) else float("nan"),
    }


def gravity(accel_g):
    """Static acceleration from the per-axis median, so sparse taps do not bias it."""
    g = np.median(accel_g, axis=0)
    magnitude = float(np.linalg.norm(g))
    vertical_axis = int(np.argmax(np.abs(g)))
    return g, magnitude, vertical_axis


def horizontal_basis(g):
    """Two orthonormal vectors spanning the platform plane, the plane normal to gravity."""
    u = g / np.linalg.norm(g)
    helper = np.eye(3)[int(np.argmin(np.abs(u)))]
    e1 = np.cross(u, helper)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(u, e1)
    return e1, e2


def horizontal(accel_g, g):
    """(n, 2) horizontal components of the dynamic acceleration. Pieces slide and tip under
    horizontal acceleration, so this, not the raw vector, is what every metric uses."""
    e1, e2 = horizontal_basis(g)
    dyn = accel_g - g
    return np.column_stack([dyn @ e1, dyn @ e2])


def noise_rms(h):
    """RMS of the horizontal magnitude: the floor every later threshold is a multiple of."""
    return float(np.sqrt(np.mean(np.sum(h ** 2, axis=1))))


def robust_noise(h):
    """Noise estimate that sparse taps do not inflate: 1.4826 x MAD of each component,
    combined as an RMS magnitude. Equals noise_rms for pure Gaussian noise."""
    sigma = [1.4826 * np.median(np.abs(c - np.median(c))) for c in h.T]
    return float(np.sqrt(sum(s ** 2 for s in sigma)))


def psd(h, fs):
    """Welch PSD of each horizontal component, summed: g^2/Hz against Hz."""
    nperseg = int(min(len(h), 4 * fs))
    f, p1 = signal.welch(h[:, 0], fs=fs, nperseg=nperseg)
    _, p2 = signal.welch(h[:, 1], fs=fs, nperseg=nperseg)
    return f, p1 + p2


def detect_taps(h, fs, noise, k=10.0, quiet_s=0.1):
    """Indices where |h| crosses k x noise. After a tap the detector re-arms only once |h|
    has stayed below the threshold for quiet_s: a lightly damped platform rings above it
    for most of a second, and without the hysteresis one strike would count as several."""
    mag = np.hypot(h[:, 0], h[:, 1])
    threshold = k * noise
    quiet_needed = int(quiet_s * fs)
    taps = []
    armed = True
    quiet = 0
    for i, m in enumerate(mag):
        if m > threshold:
            quiet = 0
            if armed:
                taps.append(i)
                armed = False
        else:
            quiet += 1
            if quiet >= quiet_needed:
                armed = True
    return taps


@dataclass
class TapResponse:
    index: int
    peak_g: float
    freq_hz: float
    damping: float  # damping ratio zeta; nan when the decay could not be fitted


def tap_response(h, fs, index, noise, window_s=0.5):
    """Dominant frequency and damping ratio of the ringing after one tap.

    The ringing is projected onto its own principal horizontal direction first: the
    magnitude |h| is rectified and would show every frequency doubled.
    """
    seg = h[index:index + int(window_s * fs)]
    seg = seg - seg.mean(axis=0)
    _, _, vt = np.linalg.svd(seg, full_matrices=False)
    s = seg @ vt[0]
    peak = int(np.argmax(np.abs(s)))
    s = s[peak:]
    n = len(s)
    spec = np.abs(np.fft.rfft(s * np.hanning(n), n=8 * n))
    freqs = np.fft.rfftfreq(8 * n, 1 / fs)
    band = (freqs >= 2.0) & (freqs <= 0.45 * fs)
    f0 = float(freqs[band][np.argmax(spec[band])])
    damping = _damping(s, fs, f0, noise)
    return TapResponse(index=index, peak_g=float(np.max(np.abs(seg))), freq_hz=f0, damping=damping)


def _damping(s, fs, f0, noise):
    # Envelope of the band around f0, fitted as exp(-zeta * 2 pi f0 * t) while it is still
    # well above the noise. Below 3 x noise the fit would be measuring the noise floor.
    lo, hi = 0.7 * f0, min(1.3 * f0, 0.45 * fs)
    sos = signal.butter(2, [lo, hi], btype="band", fs=fs, output="sos")
    env = np.abs(signal.hilbert(signal.sosfiltfilt(sos, s)))
    usable = np.flatnonzero(env < 3 * noise)
    end = int(usable[0]) if len(usable) else len(env)
    start = int(round(fs / f0))  # skip the first cycle, where the filter is settling
    if end - start < int(2 * fs / f0):
        return float("nan")
    t = np.arange(start, end) / fs
    slope = np.polyfit(t, np.log(env[start:end]), 1)[0]
    return float(-slope / (2 * np.pi * f0))


def mark_offsets_us(events, t_us):
    """For each mark event, microseconds until the next sample. Compared in test A7 with
    the MARK-edge-to-INT-edge interval read off the logic analyzer."""
    out = []
    for t, name in events:
        if not name.startswith("mark"):
            continue
        after = t_us[t_us >= t]
        if len(after):
            out.append((name, int(after[0] - t)))
    return out
```

- [ ] **Step 4: Write the report**

`tools/joint_bench/joint_bench/report.py`:

```python
"""Turn one recording into analysis.json, plots and a short verdict against tests A2-A3.

The thresholds are the stage-A acceptance criteria in the design doc, section 6; they are
written out here once so the printed PASS/FAIL means exactly what the doc means.
"""

import json
import os

import matplotlib

matplotlib.use("Agg")  # no display on a bench machine reached over SSH, and none needed
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from . import analysis  # noqa: E402

GRAVITY_TOLERANCE = 0.05    # A2: |a| within 1 g +- 5 %
INTERVAL_TOLERANCE = 0.02   # A3: mean interval within 1/rate +- 2 %


def analyze(path, noise_from=None):
    rec = analysis.load(path)
    fs = float(rec.meta["imu"]["rate"])
    result = {"label": rec.meta.get("label"), "rate_hz": fs, "stats": rec.meta.get("stats", {})}
    result["integrity"] = analysis.integrity(rec.seq, rec.t_us)
    g, magnitude, axis = analysis.gravity(rec.accel_g)
    result["gravity"] = {"vector_g": g.tolist(), "magnitude_g": magnitude,
                         "vertical_axis": "xyz"[axis]}
    h = analysis.horizontal(rec.accel_g, g)
    if noise_from:
        ref = analysis.load(noise_from)
        gr, _, _ = analysis.gravity(ref.accel_g)
        noise = analysis.noise_rms(analysis.horizontal(ref.accel_g, gr))
        result["noise_source"] = noise_from
    else:
        noise = analysis.robust_noise(h)
        result["noise_source"] = "robust estimate from this recording"
    result["noise_rms_g"] = noise
    result["horizontal_rms_g"] = analysis.noise_rms(h)
    taps = analysis.detect_taps(h, fs, noise)
    responses = [analysis.tap_response(h, fs, i, noise) for i in taps]
    result["taps"] = [{"t_s": float(rec.t_s[r.index]), "peak_g": r.peak_g, "freq_hz": r.freq_hz,
                       "damping": r.damping} for r in responses]
    result["marks"] = [{"name": n, "to_next_sample_us": d}
                       for n, d in analysis.mark_offsets_us(rec.events, rec.t_us)]
    result["checks"] = _checks(result, fs)
    with open(os.path.join(path, "analysis.json"), "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=2)
    _plots(path, rec, h, fs, noise, responses)
    return result


def _checks(result, fs):
    integ = result["integrity"]
    expected_us = 1e6 / fs
    return {
        "A2 gravity within 5%": abs(result["gravity"]["magnitude_g"] - 1.0) <= GRAVITY_TOLERANCE,
        "A3 no missing samples": integ["missing"] == 0 and integ["out_of_order"] == 0,
        "A3 interval within 2%":
            abs(integ["interval_mean_us"] - expected_us) <= INTERVAL_TOLERANCE * expected_us,
        "A4 no I2C errors": str(result["stats"].get("i2c_err", "0")) == "0",
    }


def _plots(path, rec, h, fs, noise, responses):
    mag = np.hypot(h[:, 0], h[:, 1])
    fig, ax = plt.subplots(figsize=(10, 3.5))
    ax.plot(rec.t_s, mag * 1000, lw=0.5)
    ax.axhline(10 * noise * 1000, color="tab:red", lw=0.8, label="tap threshold (10 x noise)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("horizontal |a| [mg]")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(path, "horizontal.png"), dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 3.5))
    ax.hist(np.diff(rec.t_us), bins=100)
    ax.set_xlabel("sample interval [us]")
    ax.set_ylabel("count")
    ax.set_yscale("log")
    fig.tight_layout()
    fig.savefig(os.path.join(path, "intervals.png"), dpi=120)
    plt.close(fig)

    f, p = analysis.psd(h, fs)
    fig, ax = plt.subplots(figsize=(8, 3.5))
    ax.semilogy(f, np.maximum(p, 1e-20))  # a perfectly still axis has zero power
    for r in responses:
        ax.axvline(r.freq_hz, color="tab:orange", lw=0.6)
    ax.set_xlabel("frequency [Hz]")
    ax.set_ylabel("PSD [g^2/Hz]")
    fig.tight_layout()
    fig.savefig(os.path.join(path, "psd.png"), dpi=120)
    plt.close(fig)


def summary(result):
    lines = [f"{result['label']}: {result['integrity']['samples']} samples"
             f" at {result['rate_hz']:.0f} Hz"]
    i = result["integrity"]
    lines.append(f"  missing {i['missing']}, out of order {i['out_of_order']}, interval "
                 f"{i['interval_mean_us']:.1f} us mean, {i['interval_std_us']:.1f} std, "
                 f"{i['interval_min_us']:.0f}..{i['interval_max_us']:.0f}")
    g = result["gravity"]
    lines.append(f"  gravity {g['magnitude_g']:.4f} g, vertical axis {g['vertical_axis']}")
    lines.append(f"  horizontal noise {result['noise_rms_g'] * 1000:.3f} mg rms"
                 f" ({result['noise_source']})")
    for t in result["taps"]:
        lines.append(f"  tap at {t['t_s']:.2f} s: peak {t['peak_g'] * 1000:.1f} mg, "
                     f"{t['freq_hz']:.1f} Hz, damping {t['damping']:.3f}")
    for m in result["marks"]:
        lines.append(f"  {m['name']}: next sample {m['to_next_sample_us']} us later")
    for name, ok in result["checks"].items():
        lines.append(f"  {'PASS' if ok else 'FAIL'}  {name}")
    return "\n".join(lines)
```

- [ ] **Step 5: Write the command line**

`tools/joint_bench/joint_bench/__main__.py`:

```python
"""Command line for the joint0 bench.

    python -m joint_bench console                 talk to the firmware by hand
    python -m joint_bench record -s 60 -l rest    record 60 s into debug/bench/
    python -m joint_bench analyze <dir>           analyse a recording

Run from tools/joint_bench, or with it on PYTHONPATH.
"""

import argparse
import datetime
import os
import sys
import threading

from . import recorder, report
from .protocol import Sample, parse_line

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_OUT = os.path.join(REPO, "debug", "bench")
ST_VCP = (0x0483, 0x5740)  # STMicroelectronics Virtual COM Port, CubeMX's CDC default


def find_port():
    from serial.tools import list_ports
    hits = [p.device for p in list_ports.comports() if (p.vid, p.pid) == ST_VCP]
    if len(hits) != 1:
        raise SystemExit(f"expected one ST virtual COM port, found {hits or 'none'}; pass --port")
    return hits[0]


def open_port(name):
    import serial
    return serial.Serial(name or find_port(), baudrate=115200, timeout=0.1)


def cmd_console(args):
    port = open_port(args.port)
    link = recorder.Link(port)
    stop = threading.Event()

    def reader():
        samples = 0
        for line in link.lines():
            if stop.is_set():
                return
            if line is None:
                continue
            if isinstance(parse_line(line), Sample):
                samples += 1
                if samples % 1000 == 0:
                    print(f"  ... {samples} samples", flush=True)
                continue
            print(line.rstrip(), flush=True)

    threading.Thread(target=reader, daemon=True).start()
    print("type commands, 'help' lists them, Ctrl-D quits")
    try:
        for line in sys.stdin:
            link.send(line.strip())
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        link.send("stream off")


def cmd_record(args):
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = os.path.join(args.out, f"{stamp}_{args.label}")
    marks = [(args.mark_start + i * args.mark_every, f"{args.label}-{i + 1}")
             for i in range(args.marks)]
    port = open_port(args.port)
    try:
        meta = recorder.record(recorder.Link(port), outdir, args.seconds, args.label, REPO,
                               marks=marks)
    finally:
        port.close()
    print(f"{meta['samples']} samples, {meta['events']} events -> {outdir}")
    print(report.summary(report.analyze(outdir)))


def cmd_analyze(args):
    print(report.summary(report.analyze(args.dir, noise_from=args.noise_from)))


def main(argv=None):
    parser = argparse.ArgumentParser(prog="joint_bench")
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("console")
    p.add_argument("--port")
    p.set_defaults(func=cmd_console)
    p = sub.add_parser("record")
    p.add_argument("-s", "--seconds", type=float, required=True)
    p.add_argument("-l", "--label", required=True)
    p.add_argument("--port")
    p.add_argument("--out", default=DEFAULT_OUT)
    p.add_argument("--marks", type=int, default=0,
                   help="send this many marks during the recording")
    p.add_argument("--mark-start", type=float, default=1.0)
    p.add_argument("--mark-every", type=float, default=1.0)
    p.set_defaults(func=cmd_record)
    p = sub.add_parser("analyze")
    p.add_argument("dir")
    p.add_argument("--noise-from", help="a rest recording to take the noise floor from")
    p.set_defaults(func=cmd_analyze)
    args = parser.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: Run the tests and verify they pass**

Run: `cd tools/joint_bench && python3 -m pytest -q tests && python3 -m flake8 --max-line-length 99 . && python3 -m joint_bench --help`
Expected: `20 passed`; flake8 prints nothing; the help lists `console`, `record`, `analyze`.

- [ ] **Step 7: Commit**

```bash
git add tools/joint_bench
git commit -m "feat(tools): analyse bench recordings for sample integrity, noise and platform modes"
```

---

### Task 10: The CubeMX project and the seam

**Files:**
- Create **[HUMAN]**: `codes_stm32/codes_stm32.ioc` and everything CubeMX generates beside it.
- Create: `codes_stm32/app/app.h`, `codes_stm32/app/app.cpp` (placeholder)
- Modify (generated files, inside `USER CODE` blocks only): `codes_stm32/Core/Src/main.c`, `codes_stm32/USB_DEVICE/App/usbd_cdc_if.c`
- Modify: `codes_stm32/.cproject` (through CubeIDE's project properties dialogs)

**Interfaces:**
- Consumes: `lib/` (Tasks 4-7), compiled into the target from here on.
- Produces:
  - CubeMX pin labels: `IMU_INT_Pin`/`IMU_INT_GPIO_Port` (PA2), `MARK_Pin`/`MARK_GPIO_Port` (PA4), `LED_Pin`/`LED_GPIO_Port` (PC13), in `main.h`.
  - `extern I2C_HandleTypeDef hi2c1;` in `i2c.h`; `extern USBD_HandleTypeDef hUsbDeviceFS;` from `usb_device.c`; `uint8_t CDC_Transmit_FS(uint8_t*, uint16_t)` in `usbd_cdc_if.h`.
  - `app/app.h`: `void app_init(void); void app_loop(void); void app_usb_rx(const uint8_t*, uint32_t); void app_usb_tx_done(void);` with C linkage.
  - The target build: every `.cpp` in `app/`, `drivers/` and `lib/src/` is compiled in, by glob, so Task 11 adds files without touching CMake.

- [ ] **Step 1: [HUMAN] Generate the project in STM32CubeIDE**

Stop and ask the user to do this in STM32CubeIDE 1.18.1, then commit the result. Every setting comes from the design doc §4.2.

1. **File → New → STM32 Project** → MCU selector → `STM32F401CEUx`. Project name `codes_stm32`; untick *Use default location* and set the location to the repository root, so the `.ioc` lands at `codes_stm32/codes_stm32.ioc`; Targeted Language *C++*; Binary *Executable*; Project Type *STM32Cube*.
2. **System Core → RCC:** High Speed Clock (HSE) = *Crystal/Ceramic Resonator*. Low Speed Clock (LSE) = *Disable*.
3. **System Core → SYS:** Debug = *Serial Wire*. Timebase Source = *SysTick*.
4. **Connectivity → I2C1:** mode *I2C*. Make sure the pins are **PB6 = I2C1_SCL, PB7 = I2C1_SDA** (click the pin in the chip view to reassign if CubeMX picked PB8/PB9). Parameter Settings: I2C Speed Mode = *Fast Mode*, Clock Speed = *400000*. DMA Settings: *Add* `I2C1_RX`, priority *High*, mode *Normal*. NVIC Settings: tick *I2C1 event interrupt* and *I2C1 error interrupt*.
5. **Connectivity → USB_OTG_FS:** Mode = *Device_Only*. Leave *Activate_VBUS* disabled: it would take PA9, the stage-B TMC UART pin.
6. **Middleware → USB_DEVICE:** Class for FS IP = *Communication Device Class (Virtual Port Com)*. Leave VID 1155 (0x0483) and PID 22336 (0x5740): the host tool finds the board by that pair.
7. **GPIO, in the chip view:**
   - PA2 → *GPIO_EXTI2*; in System Core → GPIO: mode *External Interrupt Mode with Rising edge trigger detection*, no pull, User Label `IMU_INT`.
   - PA4 → *GPIO_Output*; output level *Low*, User Label `MARK`.
   - PC13 → *GPIO_Output*; output level *High* (LED off), User Label `LED`.
8. **System Core → NVIC:** enable *EXTI line2 interrupt*. Priority group *4 bits*. Preemption priorities: EXTI line2 = 2, I2C1 event = 2, I2C1 error = 2, DMA1 stream (I2C1_RX) = 2, USB On The Go FS global = 4, System tick timer = 15. Priorities 0-1 stay free for stage B's step timers.
9. **Clock Configuration:** input frequency 25 MHz; PLL Source Mux = *HSE*; /M = 25, ×N = 336, /P = 4, /Q = 7; System Clock Mux = *PLLCLK*; AHB /1, APB1 /2, APB2 /1. Check that HCLK reads 84 MHz and the 48 MHz clock reads 48 MHz, with no red fields, and that RCC's flash latency reads 2 WS.
10. **Project Manager → Project:** Toolchain/IDE stays *STM32CubeIDE*. Use the installed STM32Cube FW_F4 package; the `.ioc` records its version.
11. **Project Manager → Code Generator:** *Copy only the necessary library files*; tick *Generate peripheral initialization as a pair of '.c/.h' files per peripheral*; tick *Keep User Code when re-generating*.
12. **Generate Code.** Commit:

```bash
git add codes_stm32
git commit -m "build(firmware): generate the codes_stm32 STM32CubeIDE project"
```

Before continuing, confirm these files exist in `codes_stm32/`: `.project`, `.cproject`, `codes_stm32.ioc`, `Core/Src/main.c`, `Core/Inc/main.h`, `Core/Inc/i2c.h`, `USB_DEVICE/App/usbd_cdc_if.c`. If CubeIDE laid them out differently, adjust the paths in the steps below to match; the edits themselves do not change.

- [ ] **Step 2: Write the seam header and the placeholder application**

`codes_stm32/app/app.h`:

```c
/*
 * The seam between CubeMX-generated C and the bench firmware. Generated files call only
 * these four functions, each from inside a USER CODE block, so regenerating the project
 * from codes_stm32.ioc never touches the bench logic.
 */
#ifndef JOINT_BENCH_APP_H_
#define JOINT_BENCH_APP_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* main.c, USER CODE BEGIN 2: once, after every MX_*_Init() call. */
void app_init(void);

/* main.c, USER CODE BEGIN 3: every iteration of the main while(1). */
void app_loop(void);

/* usbd_cdc_if.c, CDC_Receive_FS, USER CODE BEGIN 6. USB interrupt context. */
void app_usb_rx(const uint8_t * buf, uint32_t len);

/* usbd_cdc_if.c, CDC_TransmitCplt_FS, USER CODE BEGIN 13. USB interrupt context. */
void app_usb_tx_done(void);

#ifdef __cplusplus
}
#endif

#endif  /* JOINT_BENCH_APP_H_ */
```

`codes_stm32/app/app.cpp`:

```cpp
// Placeholder application: proves the CubeMX seam builds, flashes and runs before any bench
// logic exists. The heartbeat at 2 Hz is the whole of its behaviour. Task 11 replaces it.
#include "app.h"

#include "main.h"

extern "C" void app_init(void) {}

extern "C" void app_loop(void)
{
  static uint32_t last = 0;
  if (HAL_GetTick() - last >= 250) {
    last = HAL_GetTick();
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

extern "C" void app_usb_rx(const uint8_t * buf, uint32_t len)
{
  (void)buf;
  (void)len;
}

extern "C" void app_usb_tx_done(void) {}
```

- [ ] **Step 3: Call the seam from the generated code**

In `Core/Src/main.c`:

```c
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */
```

```c
  /* USER CODE BEGIN 2 */
  app_init();
  /* USER CODE END 2 */
```

```c
    /* USER CODE BEGIN 3 */
    app_loop();
  }
  /* USER CODE END 3 */
```

In `USB_DEVICE/App/usbd_cdc_if.c`:

```c
/* USER CODE BEGIN INCLUDE */
#include "app.h"
/* USER CODE END INCLUDE */
```

In `CDC_Receive_FS`, first line of the block:

```c
  /* USER CODE BEGIN 6 */
  app_usb_rx(Buf, *Len);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
```

In `CDC_TransmitCplt_FS`:

```c
  /* USER CODE BEGIN 13 */
  app_usb_tx_done();
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
```

Keep whatever else CubeMX put in each block; only the `app_*` lines and the include are new.

- [ ] **Step 4: [HUMAN] Add the C++ sources to the CubeIDE project**

In *Project → Properties*, with configuration *[All configurations]* selected:

1. **C/C++ General → Paths and Symbols → Source Location:** add the folders `app` and `lib/src`. Add `drivers` in Task 11, once it exists. Do not add `lib` itself: `lib/tests` needs GoogleTest and must stay out of the target.
2. **C/C++ General → Paths and Symbols → Includes,** for both *GNU C* and *GNU C++*: add `app` and `lib/include`, with *Is a workspace path* ticked; `drivers` follows in Task 11.
3. **C/C++ Build → Settings → MCU G++ Compiler → General:** Language standard *ISO C++17*.
4. **C/C++ Build → Settings → MCU G++ Compiler → Optimization:** tick the options for `-fno-exceptions`, `-fno-rtti` and `-fno-threadsafe-statics`. Any that has no tick box goes under *Miscellaneous*.

`JB_VERSION` is not defined by this build, so `ver` reports `unknown` through the fallback in `app.cpp`. A stale or missing string says "older than you think", never something false.

- [ ] **Step 5: [HUMAN] Build in CubeIDE**

Build with *Project → Build Project*.
Expected: `Build Finished. 0 errors`. Warnings from CubeMX's generated code or HAL are not ours to fix, but none may come from `app/`, `drivers/` or `lib/`.

- [ ] **Step 6: [HUMAN] Flash and check the seam**

Ask the user to build on the bench machine, flash with the ST-Link, and report: the LED blinks at 2 Hz, and the PC sees a USB serial device with VID:PID 0483:5740 (`lsusb` on Linux, Device Manager on Windows). No serial traffic is expected yet.

- [ ] **Step 7: Commit**

```bash
git add codes_stm32
git commit -m "feat(firmware): wire the CubeMX project to the bench application seam"
```

---

### Task 11: Drivers and the bench application

**Files:**
- Create: `codes_stm32/drivers/clock.hpp`, `drivers/clock.cpp`, `drivers/usb_port.hpp`, `drivers/usb_port.cpp`, `drivers/imu.hpp`, `drivers/imu.cpp`
- Modify: `codes_stm32/app/app.cpp` (replace whole file)

**Interfaces:**
- Consumes: `app.h`, CubeMX pin labels and handles (Task 10); `CycleClock`, `SpscQueue`, `LineRing` (Task 4); `TextWriter`, `ImuSample`, `format_sample`, `format_event`, `kMaxLine` (Task 5); `LineAssembler`, `parse_command`, `Command` (Task 6); `jb::mpu::*` (Task 7). The exact reply strings in Task 8's Interfaces.
- Produces: the stage-A firmware. `drivers/clock`: `jb::clock_init()`, `uint64_t jb::now_us()`. `drivers/usb_port`: `usb_send`, `usb_pump`, `usb_rx_pop`, `usb_tx_dropped`, `usb_rx_dropped`, `usb_rx_isr`, `usb_tx_done_isr`. `drivers/imu`: `ImuCounters`, `imu_probe`, `imu_configure`, `imu_present`, `imu_chip`, `imu_who_am_i`, `imu_address`, `imu_plan`, `imu_stream`, `imu_streaming`, `imu_pop`, `imu_counters`, `imu_scan`, and the three HAL callbacks `HAL_GPIO_EXTI_Callback`, `HAL_I2C_MemRxCpltCallback`, `HAL_I2C_ErrorCallback`.

This task's logic is the HAL-facing glue; everything that could be tested on the host already was, in Tasks 4-7. Its tests are the compile, and the bench steps at the end, which are the only place USB and I2C behaviour can be observed.

- [ ] **Step 1: Write the clock**

`codes_stm32/drivers/clock.hpp`:

```cpp
#pragma once

#include <cstdint>

namespace jb
{

// Starts the DWT cycle counter. Call once, before anything takes a timestamp.
void clock_init();

// Microseconds since clock_init(), 64-bit. Safe from any context: it masks interrupts for
// the few cycles it takes, because the IMU interrupt and the main loop both call it.
uint64_t now_us();

}  // namespace jb
```

`codes_stm32/drivers/clock.cpp`:

```cpp
#include "clock.hpp"

#include "jb/cycle_clock.hpp"
#include "main.h"

namespace jb
{
namespace
{
CycleClock g_clock(84000000);  // SYSCLK; see the clock tree in the design doc, section 4.2
}

void clock_init()
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint64_t now_us()
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint64_t cycles = g_clock.extend(DWT->CYCCNT);
  __set_PRIMASK(primask);
  return g_clock.to_us(cycles);
}

}  // namespace jb
```

- [ ] **Step 2: Write the USB port**

`codes_stm32/drivers/usb_port.hpp`:

```cpp
#pragma once

#include <cstdint>

namespace jb
{

// Queues one whole line for the host. Returns false, and counts the drop, when the
// transmit ring is full; it never waits, because a stalled host must not stall sampling.
bool usb_send(const char * line, uint32_t len);

// Starts the next USB transfer when the previous one has finished. Main loop only.
void usb_pump();

// Pops one received byte. Main loop only.
bool usb_rx_pop(char * c);

uint32_t usb_tx_dropped();
uint32_t usb_rx_dropped();

// Interrupt-side hooks, called through app.h from usbd_cdc_if.c.
void usb_rx_isr(const uint8_t * buf, uint32_t len);
void usb_tx_done_isr();

}  // namespace jb
```

`codes_stm32/drivers/usb_port.cpp`:

```cpp
#include "usb_port.hpp"

#include "jb/line_ring.hpp"
#include "jb/spsc_queue.hpp"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

namespace jb
{
namespace
{
// 8 KB holds about 160 ms of telemetry at 1 kHz: enough to ride out a host that is busy
// for a moment, small enough that a host that has stopped reading is noticed quickly.
LineRing<8192> g_tx;
SpscQueue<char, 256> g_rx;
uint32_t g_inflight = 0;      // bytes handed to the USB stack and not yet confirmed
volatile bool g_tx_done = false;
volatile uint32_t g_rx_dropped = 0;

bool configured()
{
  // CDC_Transmit_FS dereferences the class data unchecked, and that pointer is null until
  // the host has enumerated the device. Sending before then is a hard fault, so check.
  return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceFS.pClassData != nullptr;
}
}  // namespace

bool usb_send(const char * line, uint32_t len)
{
  return g_tx.push(line, len);
}

void usb_pump()
{
  if (!configured()) {
    // Unplugged or not yet enumerated. A transfer that was in flight will never complete,
    // and waiting for it would silence the port for good after a replug. Drop it.
    if (g_inflight != 0) {
      g_tx.consume(g_inflight);
      g_inflight = 0;
    }
    return;
  }
  if (g_inflight != 0) {
    if (!g_tx_done) {
      return;
    }
    // Only now may the ring reuse those bytes: the stack read them in place.
    g_tx.consume(g_inflight);
    g_inflight = 0;
  }
  const uint8_t * data = nullptr;
  uint32_t len = g_tx.peek(&data);
  if (len == 0) {
    return;
  }
  if (len > 4096) {
    len = 4096;  // CDC_Transmit_FS takes a uint16_t length
  }
  g_tx_done = false;
  if (CDC_Transmit_FS(const_cast<uint8_t *>(data), static_cast<uint16_t>(len)) == USBD_OK) {
    g_inflight = len;
  }
}

bool usb_rx_pop(char * c)
{
  return g_rx.pop(c);
}

uint32_t usb_tx_dropped() {return g_tx.dropped();}
uint32_t usb_rx_dropped() {return g_rx_dropped;}

void usb_rx_isr(const uint8_t * buf, uint32_t len)
{
  for (uint32_t i = 0; i < len; ++i) {
    if (!g_rx.push(static_cast<char>(buf[i]))) {
      g_rx_dropped = g_rx_dropped + 1;
    }
  }
}

void usb_tx_done_isr()
{
  g_tx_done = true;
}

}  // namespace jb
```

- [ ] **Step 3: Write the IMU driver**

`codes_stm32/drivers/imu.hpp`:

```cpp
#pragma once

#include <cstdint>

#include "jb/mpu.hpp"
#include "jb/telemetry.hpp"

namespace jb
{

struct ImuCounters
{
  uint32_t samples;    // data-ready edges seen while streaming
  uint32_t overrun;    // an edge arrived while the previous read was still in flight
  uint32_t i2c_err;    // a read could not start or ended in a bus error
  uint32_t queue_drop; // the main loop fell behind and the sample queue was full
};

// Probes 0x68 then 0x69, identifies the chip and applies `cfg` after a reset. Blocking;
// never call it while streaming. Returns nullptr on success or an error message.
const char * imu_probe(const mpu::Config & cfg);

// Reconfigures a chip already found. Blocking; never call it while streaming.
const char * imu_configure(const mpu::Config & cfg);

bool imu_present();
mpu::Chip imu_chip();
uint8_t imu_who_am_i();
uint8_t imu_address();
const mpu::Plan & imu_plan();

void imu_stream(bool on);
bool imu_streaming();

// Main loop: the next sample the interrupt path produced, if any.
bool imu_pop(ImuSample * out);

ImuCounters imu_counters();

// Blocking scan of 0x08..0x77. Writes up to `max` addresses, returns how many were found.
int imu_scan(uint8_t * found, int max);

// Interrupt-side hooks, called from the HAL callbacks in imu.cpp.
void imu_on_data_ready();
void imu_on_read_done();
void imu_on_read_error();

}  // namespace jb
```

`codes_stm32/drivers/imu.cpp`:

```cpp
#include "imu.hpp"

#include "clock.hpp"
#include "i2c.h"
#include "jb/spsc_queue.hpp"
#include "main.h"

namespace jb
{
namespace
{
constexpr uint32_t kTimeoutMs = 10;

mpu::Chip g_chip = mpu::Chip::Unknown;
uint8_t g_who = 0;
uint8_t g_addr = 0;
mpu::Plan g_plan{};

volatile bool g_streaming = false;
volatile bool g_busy = false;
uint32_t g_seq = 0;
uint64_t g_t_us = 0;           // timestamp of the edge whose read is in flight
uint32_t g_read_seq = 0;
uint8_t g_block[mpu::kBlockLen];
SpscQueue<ImuSample, 64> g_queue;
volatile ImuCounters g_count{};

// Blocking bus use must wait out a DMA read that was already in flight when streaming
// stopped; starting a polled transfer underneath it corrupts both.
bool wait_idle()
{
  const uint32_t start = HAL_GetTick();
  while (g_busy) {
    if (HAL_GetTick() - start > kTimeoutMs) {
      return false;
    }
  }
  return true;
}

HAL_StatusTypeDef write_reg(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, static_cast<uint16_t>(g_addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                           &value, 1, kTimeoutMs);
}

const char * apply(const mpu::Config & cfg)
{
  mpu::Plan plan{};
  if (const char * err = mpu::plan_config(g_chip, cfg, &plan)) {
    return err;
  }
  if (write_reg(mpu::reg::kPwrMgmt1, mpu::kResetValue) != HAL_OK) {
    return "imu: reset write failed";
  }
  HAL_Delay(100);
  for (uint8_t i = 0; i < plan.count; ++i) {
    if (write_reg(plan.writes[i].reg, plan.writes[i].value) != HAL_OK) {
      return "imu: config write failed";
    }
  }
  g_plan = plan;
  return nullptr;
}
}  // namespace

const char * imu_probe(const mpu::Config & cfg)
{
  if (!wait_idle()) {
    return "imu: bus busy";
  }
  g_chip = mpu::Chip::Unknown;
  for (uint8_t addr : {mpu::kAddrLow, mpu::kAddrHigh}) {
    uint8_t who = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, static_cast<uint16_t>(addr << 1), mpu::reg::kWhoAmI,
      I2C_MEMADD_SIZE_8BIT, &who, 1, kTimeoutMs) != HAL_OK)
    {
      continue;
    }
    g_addr = addr;
    g_who = who;
    g_chip = mpu::identify(who);
    if (g_chip == mpu::Chip::Unknown) {
      return "imu: unsupported who_am_i";
    }
    return apply(cfg);
  }
  return "imu: not found";
}

const char * imu_configure(const mpu::Config & cfg)
{
  if (g_chip == mpu::Chip::Unknown) {
    return "imu: not found";
  }
  if (!wait_idle()) {
    return "imu: bus busy";
  }
  return apply(cfg);
}

bool imu_present() {return g_chip != mpu::Chip::Unknown;}
mpu::Chip imu_chip() {return g_chip;}
uint8_t imu_who_am_i() {return g_who;}
uint8_t imu_address() {return g_addr;}
const mpu::Plan & imu_plan() {return g_plan;}

void imu_stream(bool on)
{
  g_streaming = on;
}

bool imu_streaming() {return g_streaming;}

bool imu_pop(ImuSample * out)
{
  return g_queue.pop(out);
}

ImuCounters imu_counters()
{
  __disable_irq();
  const ImuCounters c{g_count.samples, g_count.overrun, g_count.i2c_err, g_count.queue_drop};
  __enable_irq();
  return c;
}

int imu_scan(uint8_t * found, int max)
{
  if (!wait_idle()) {
    return 0;
  }
  int n = 0;
  for (uint8_t addr = 0x08; addr <= 0x77 && n < max; ++addr) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, static_cast<uint16_t>(addr << 1), 2, 2) == HAL_OK) {
      found[n++] = addr;
    }
  }
  return n;
}

// Every data-ready edge consumes a sequence number, whether or not its sample survives.
// That is what turns every loss downstream (overrun, bus error, full queue, full USB
// ring) into a gap the host can count, instead of a silent shortening of the recording.
void imu_on_data_ready()
{
  if (!g_streaming) {
    return;
  }
  const uint32_t seq = g_seq++;
  g_count.samples = g_count.samples + 1;
  if (g_busy) {
    g_count.overrun = g_count.overrun + 1;
    return;
  }
  g_t_us = now_us();
  g_read_seq = seq;
  g_busy = true;
  if (HAL_I2C_Mem_Read_DMA(&hi2c1, static_cast<uint16_t>(g_addr << 1), mpu::reg::kAccelXoutH,
    I2C_MEMADD_SIZE_8BIT, g_block, mpu::kBlockLen) != HAL_OK)
  {
    g_busy = false;
    g_count.i2c_err = g_count.i2c_err + 1;
  }
}

void imu_on_read_done()
{
  const mpu::Raw raw = mpu::parse_block(g_block);
  const ImuSample s{g_read_seq, g_t_us, {raw.accel[0], raw.accel[1], raw.accel[2]},
    {raw.gyro[0], raw.gyro[1], raw.gyro[2]}};
  if (!g_queue.push(s)) {
    g_count.queue_drop = g_count.queue_drop + 1;
  }
  g_busy = false;
}

void imu_on_read_error()
{
  g_count.i2c_err = g_count.i2c_err + 1;
  g_busy = false;
}

}  // namespace jb

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  if (pin == IMU_INT_Pin) {
    jb::imu_on_data_ready();
  }
}

extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c)
{
  if (hi2c == &hi2c1) {
    jb::imu_on_read_done();
  }
}

extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef * hi2c)
{
  if (hi2c == &hi2c1) {
    jb::imu_on_read_error();
  }
}
```

- [ ] **Step 4: Replace the placeholder application**

`codes_stm32/app/app.cpp`:

```cpp
#include "app.h"

#include <string_view>

#include "clock.hpp"
#include "imu.hpp"
#include "jb/command.hpp"
#include "jb/line_assembler.hpp"
#include "jb/telemetry.hpp"
#include "jb/text_writer.hpp"
#include "main.h"
#include "usb_port.hpp"

#ifndef JB_VERSION
#define JB_VERSION "unknown"
#endif

namespace
{

jb::LineAssembler g_rx;
jb::mpu::Config g_cfg = jb::mpu::kDefault;
uint32_t g_last_blink = 0;

void send(const jb::TextWriter & w, const char * buf)
{
  if (w.ok()) {
    jb::usb_send(buf, w.size());
  }
}

void reply_text(bool ok, std::string_view text)
{
  char buf[128];
  jb::TextWriter w(buf, sizeof(buf));
  w.str(ok ? "OK " : "ERR ");
  w.str(text);
  w.put('\n');
  send(w, buf);
}

// "OK imu chip=MPU6050 who=0x68 addr=0x68 rate=1000 dlpf=184 fs=4 lsb_per_g=8192"
void reply_imu_info()
{
  const jb::mpu::Plan & p = jb::imu_plan();
  char buf[jb::kMaxLine];
  jb::TextWriter w(buf, sizeof(buf));
  w.str("OK imu chip=");
  w.str(jb::mpu::chip_name(jb::imu_chip()));
  w.str(" who=0x");
  w.hex8(jb::imu_who_am_i());
  w.str(" addr=0x");
  w.hex8(jb::imu_address());
  w.str(" rate=");
  w.u64(p.accepted.rate_hz);
  w.str(" dlpf=");
  w.u64(p.accepted.dlpf_hz);
  w.str(" fs=");
  w.u64(p.accepted.fs_g);
  w.str(" lsb_per_g=");
  w.u64(p.lsb_per_g);
  w.put('\n');
  send(w, buf);
}

// A missing IMU at boot is not final: the cable may simply not have been plugged in yet.
// Every command that needs the chip probes again first.
bool ensure_imu()
{
  if (jb::imu_present()) {
    return true;
  }
  if (const char * err = jb::imu_probe(g_cfg)) {
    reply_text(false, err);
    return false;
  }
  return true;
}

void cmd_i2cscan()
{
  uint8_t found[16];
  const int n = jb::imu_scan(found, 16);
  char buf[jb::kMaxLine];
  jb::TextWriter w(buf, sizeof(buf));
  w.str("OK i2cscan");
  if (n == 0) {
    w.str(" none");
  }
  for (int i = 0; i < n; ++i) {
    w.put(' ');
    w.hex8(found[i]);
  }
  w.put('\n');
  send(w, buf);
}

void cmd_imu_cfg(const jb::ImuCfgArgs & args)
{
  if (!ensure_imu()) {
    return;
  }
  jb::mpu::Config want = g_cfg;
  if (args.rate_hz) {want.rate_hz = args.rate_hz;}
  if (args.dlpf_hz) {want.dlpf_hz = args.dlpf_hz;}
  if (args.fs_g) {want.fs_g = args.fs_g;}
  if (const char * err = jb::imu_configure(want)) {
    reply_text(false, err);
    return;
  }
  g_cfg = want;
  reply_imu_info();
}

void cmd_mark(const char * label)
{
  // The timestamp and the edge are taken together with interrupts masked, so the E line
  // says exactly when MARK moved. Test A7 compares this against the logic analyzer.
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint64_t t = jb::now_us();
  HAL_GPIO_TogglePin(MARK_GPIO_Port, MARK_Pin);
  __set_PRIMASK(primask);
  char name[32];
  jb::TextWriter n(name, sizeof(name));
  n.str("mark");
  if (label[0] != '\0') {
    n.put(':');
    n.str(label);
  }
  char buf[jb::kMaxLine];
  const uint32_t len = jb::format_event(buf, sizeof(buf), t, std::string_view(name, n.size()));
  if (len) {
    jb::usb_send(buf, len);
  }
  reply_text(true, "mark");
}

void cmd_stats()
{
  const jb::ImuCounters c = jb::imu_counters();
  char buf[jb::kMaxLine * 2];
  jb::TextWriter w(buf, sizeof(buf));
  w.str("OK stats samples=");
  w.u64(c.samples);
  w.str(" imu_overrun=");
  w.u64(c.overrun);
  w.str(" i2c_err=");
  w.u64(c.i2c_err);
  w.str(" queue_drop=");
  w.u64(c.queue_drop);
  w.str(" tx_drop=");
  w.u64(jb::usb_tx_dropped());
  w.str(" rx_drop=");
  w.u64(jb::usb_rx_dropped());
  w.put('\n');
  send(w, buf);
}

void handle(const jb::Command & cmd)
{
  const bool streaming = jb::imu_streaming();
  switch (cmd.kind) {
    case jb::CmdKind::Help:
      reply_text(true, "help ver i2cscan stats | imu info | imu cfg rate= dlpf= fs= | "
        "stream on|off | mark [label]");
      break;
    case jb::CmdKind::Ver:
      reply_text(true, "ver joint_bench " JB_VERSION);
      break;
    case jb::CmdKind::I2cScan:
      // Scanning and reconfiguring use the bus in blocking mode, which would collide with
      // the DMA reads that streaming starts from the interrupt.
      if (streaming) {reply_text(false, "stop the stream first"); break;}
      cmd_i2cscan();
      break;
    case jb::CmdKind::ImuInfo:
      if (ensure_imu()) {reply_imu_info();}
      break;
    case jb::CmdKind::ImuCfg:
      if (streaming) {reply_text(false, "stop the stream first"); break;}
      cmd_imu_cfg(cmd.cfg);
      break;
    case jb::CmdKind::StreamOn:
      if (ensure_imu()) {
        jb::imu_stream(true);
        reply_text(true, "stream on");
      }
      break;
    case jb::CmdKind::StreamOff:
      jb::imu_stream(false);
      reply_text(true, "stream off");
      break;
    case jb::CmdKind::Mark:
      cmd_mark(cmd.label);
      break;
    case jb::CmdKind::Stats:
      cmd_stats();
      break;
    case jb::CmdKind::Invalid:
      reply_text(false, cmd.error);
      break;
  }
}

void blink()
{
  // Slow blink: running with an IMU. Fast blink: no IMU found. Steady: the loop is stuck.
  const uint32_t period = jb::imu_present() ? 500 : 100;
  if (HAL_GetTick() - g_last_blink >= period) {
    g_last_blink = HAL_GetTick();
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

}  // namespace

extern "C" void app_init(void)
{
  jb::clock_init();
  jb::imu_probe(g_cfg);  // failure is fine here; see ensure_imu()
}

extern "C" void app_loop(void)
{
  jb::now_us();  // keeps the 64-bit clock extension fed even when nothing else is running
  char c;
  while (jb::usb_rx_pop(&c)) {
    switch (g_rx.feed(c)) {
      case jb::LineAssembler::Result::Line: handle(jb::parse_command(g_rx.line())); break;
      case jb::LineAssembler::Result::Overflow: reply_text(false, "line too long"); break;
      case jb::LineAssembler::Result::None: break;
    }
  }
  jb::ImuSample s;
  char buf[jb::kMaxLine];
  // Bounded, so a burst of samples cannot starve command handling or the USB pump.
  for (int i = 0; i < 16 && jb::imu_pop(&s); ++i) {
    const uint32_t len = jb::format_sample(buf, sizeof(buf), s);
    if (len) {
      jb::usb_send(buf, len);
    }
  }
  jb::usb_pump();
  blink();
}

extern "C" void app_usb_rx(const uint8_t * buf, uint32_t len)
{
  jb::usb_rx_isr(buf, len);
}

extern "C" void app_usb_tx_done(void)
{
  jb::usb_tx_done_isr();
}
```

- [ ] **Step 5: [HUMAN] Build**

Build in CubeIDE, with `drivers` added to the source and include paths as Task 10 Step 4 says, then run `scripts/fwbuild --test`.
Expected: `Build Finished. 0 errors`, no warnings from `app/`, `drivers/` or `lib/`, then `TESTS PASSED ... out of 32`.

- [ ] **Step 6: Commit**

```bash
git add codes_stm32
git commit -m "feat(firmware): stream timestamped IMU samples over USB CDC"
```

- [ ] **Step 7: [HUMAN] Smoke test on the bench**

Ask the user to flash the new build, connect the GY-521 as in runbook §6 (written in Task 12; the wiring is J1 VCC → VCC, GND → GND, SCL → SCL, SDA → SDA, INT → INT), and run from `tools/joint_bench`:

1. `python3 -m joint_bench console`, then `ver`. Expected: `OK ver joint_bench <sha>`.
2. `i2cscan`. Expected: `OK i2cscan 68`.
3. `imu info`. Expected: `OK imu chip=MPU6050 who=0x68 addr=0x68 rate=1000 dlpf=184 fs=4 lsb_per_g=8192` (another `chip`/`who` is a clone part; note it).
4. `stream on`, watch the `... N samples` counter climb by about 1000 per second, `stream off`, `stats`. Expected: `imu_overrun=0 i2c_err=0 queue_drop=0`.
5. Ctrl-D, then `python3 -m joint_bench record -s 10 -l smoke`. Expected: every check line `PASS`.
6. The robustness checks R1-R3 from runbook §10 (Review Focus 1-3): IMU absent at power-up then connected; console suspended with Ctrl-Z for five seconds while streaming; USB unplugged mid-stream with the board kept powered from the ST-Link's 3.3 V.

Any failure: collect the console output and `stats`, and fix before Task 12.

---

### Task 12: The stage-A bench runbook

**Files:**
- Create: `docs/hardware/joint-bench-stage-a-runbook.md`
- Modify: `docs/superpowers/specs/2026-10-16-joint0-bench-design.md`

**Interfaces:**
- Consumes: every command, file and check from Tasks 1-11.
- Produces: the procedure the user follows for A1-A7 and R1-R3, and the results table they fill in.

- [ ] **Step 1: Write the runbook**

`docs/hardware/joint-bench-stage-a-runbook.md`:

```markdown
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

1. ST-Link to the BlackPill's SWD header (3V3, SWDIO, SWCLK, GND).
2. Build on the development machine from `codes_stm32` (CubeIDE, or
   `cmake --preset Debug && cmake --build build/Debug`) and flash `joint_bench.elf` with
   STM32CubeProgrammer or OpenOCD.
3. The LED blinks fast (no IMU yet). The PC sees a USB serial port (VID:PID 0483:5740).

## 6. Connect the IMU

1. Unplug USB. Wire the GY-521 cable: J1 VCC → VCC, GND → GND, SCL → SCL, SDA → SDA,
   INT → INT. Leave XDA, XCL and AD0 on the GY-521 unconnected.
2. Twist SCL with GND and SDA with VCC; route the cable away from where the motor cable will
   run. Put the GY-521 on the wooden platform beside the chessboard, flat, and fix it down
   (double-sided tape or a screw): a loose sensor measures its own rattle.
3. Plug USB. The LED blinks slowly.

## 7. Software on the bench machine

    python3 -m pip install --user -r tools/requirements.txt
    cd tools/joint_bench
    python3 -m joint_bench console        # type: ver, i2cscan, imu info, stats; Ctrl-D quits

Recordings land in `debug/bench/<timestamp>_<label>/`. `record` analyses each recording when
it finishes; `python3 -m joint_bench analyze <dir>` re-runs that.

## 8. Logic analyzer setup

Clip J2: CH0-CH7 to analyzer channels 0-7, J2 GND to analyzer GND. Sample at 4 MHz or more
(400 kHz I2C needs at least 10 samples per bit). In PulseView add an I2C decoder with SCL on
CH4 and SDA on CH5.

## 9. Acceptance tests

Record each result in the table at the end.

**A1 — identify.** In `console`: `i2cscan`, then `imu info`.
Pass: `i2cscan` lists only 68 (the GY-521 has no second chip); `imu info` reports a chip. Note
`chip=` and `who=` and add them to the design doc's Open Items.

**A5 — noise floor** (run before A2, it provides the data). Nobody touches the table:

    python3 -m joint_bench record -s 60 -l rest

Pass: the summary prints a horizontal noise figure. Keep the directory name; A6 uses it.

**A2 — static.** From the `rest` summary. Pass: `PASS A2 gravity within 5%`. Note the
vertical axis it reports.

**A3 and A4 — ten minutes, cable at final length.** With the analyzer capturing a few seconds
of CH4/CH5 at some point during the run:

    python3 -m joint_bench record -s 600 -l a3_a4

Pass: `PASS A3 no missing samples`, `PASS A3 interval within 2%`, `PASS A4 no I2C errors`.
In the capture, each transaction is: START, 0xD0, 0x3B, repeated START, 0xD1, 14 data bytes
with NACK on the last, STOP, and no NACK anywhere else.

**A6 — tap test.** For each of three points (beside the board, beside the robot base, a
corner), strike the platform five times with a pencil, about two seconds apart:

    python3 -m joint_bench record -s 15 -l tap_board
    python3 -m joint_bench record -s 15 -l tap_base
    python3 -m joint_bench record -s 15 -l tap_corner
    python3 -m joint_bench analyze debug/bench/<tap dir> --noise-from debug/bench/<rest dir>

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
   (if the BlackPill resets, that is the GY-521's inrush, not a bug), then `imu info`
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
```

- [ ] **Step 2: Align test A7 in the design doc with the runbook**

In `docs/superpowers/specs/2026-10-16-joint0-bench-design.md`, replace the A7 row

```
| A7 | `mark` ×20 with the analyzer on `MARK` and `IMU_INT` | For each mark, the interval from the MARK edge to the next INT edge on the analyzer matches the interval from the `E` line's `t_us` to the next `T` sample's `t_us` within 20 µs. This shows the firmware's timestamps sit where the hardware edges are. |
```

with:

```
| A7 | `mark` ×5 with the analyzer on `MARK` and `IMU_INT` | For each mark, the interval from the MARK edge to the next INT edge on the analyzer matches the interval from the `E` line's `t_us` to the next `T` sample's `t_us` within 20 µs. This shows the firmware's timestamps sit where the hardware edges are. The five intervals are read off by hand with the analyzer's cursors. |
```

- [ ] **Step 3: Check every command in the runbook against the code**

Run each of these and compare with the runbook text: `python3 -m joint_bench --help`, `python3 -m joint_bench record --help`, `python3 -m joint_bench analyze --help` (from `tools/joint_bench`); `grep -n '"OK \|reply_text' codes_stm32/app/app.cpp` for the reply strings the runbook quotes. Fix the runbook wherever they differ.

- [ ] **Step 4: Commit**

```bash
git add docs/hardware/joint-bench-stage-a-runbook.md docs/superpowers/specs/2026-10-16-joint0-bench-design.md
git commit -m "docs(hardware): add the stage-A bench runbook"
```
