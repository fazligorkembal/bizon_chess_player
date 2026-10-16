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
