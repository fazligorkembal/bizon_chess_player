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
