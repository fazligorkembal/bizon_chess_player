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
