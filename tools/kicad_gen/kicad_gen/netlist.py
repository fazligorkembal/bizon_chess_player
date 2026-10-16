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
