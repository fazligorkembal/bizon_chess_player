from kicad_gen.checklist import shorts_to_check


def test_power_pairs_come_first_then_only_nets_on_neighbouring_holes():
    on_board = {"GND": ["J1.1", "U1.19"], "+3V3": ["J1.2", "U1.20"], "SCL": ["J1.3", "U1.14"],
                "SDA": ["J2.1", "U1.15"]}
    holes = {"J1.1": (0, 0), "J1.2": (1, 0), "J1.3": (2, 0), "U1.19": (10, 10), "U1.20": (11, 10),
             "U1.14": (5, 5), "U1.15": (6, 5), "J2.1": (20, 20)}
    assert shorts_to_check(on_board, holes) == [("GND", "+3V3"), ("+3V3", "SCL"), ("SCL", "SDA")]
