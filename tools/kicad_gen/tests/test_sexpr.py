from kicad_gen.sexpr import Sym, dumps, loads


def test_round_trips_atoms_strings_and_numbers():
    node = [Sym("pin"), Sym("passive"), [Sym("at"), -12.7, 0.0, 180], [Sym("name"), 'a "q" b']]
    assert loads(dumps(node)) == ["pin", "passive", ["at", "-12.7", "0", "180"],
                                  ["name", 'a "q" b']]


def test_numbers_have_no_trailing_zeros_or_negative_zero():
    assert dumps([Sym("xy"), 1.27, -0.0, 2.0]) == "(xy 1.27 0 2)"
