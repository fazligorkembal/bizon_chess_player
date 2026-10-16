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
