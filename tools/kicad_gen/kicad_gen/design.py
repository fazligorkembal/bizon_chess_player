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
