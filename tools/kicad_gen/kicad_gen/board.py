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
