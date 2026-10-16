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
