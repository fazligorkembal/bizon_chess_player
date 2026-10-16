"""Minimal S-expression writer and reader for KiCad files.

KiCad's formats are S-expressions, but no parser for them ships with Python and the
pcbnew SWIG bindings are deprecated, so the generator writes the text itself. Lists are
nodes, `Sym` is a bare atom (`yes`, `passive`), `str` is always quoted, and numbers are
written with at most four decimals, which is finer than KiCad's 1 nm resolution needs
for millimetre coordinates on a 0.05 mm grid.
"""


class Sym(str):
    """A bare atom, written without quotes."""


def _atom(value):
    if isinstance(value, Sym):
        return str(value)
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        text = f"{value:.4f}".rstrip("0").rstrip(".")
        return "0" if text in ("-0", "") else text
    if isinstance(value, str):
        escaped = value.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
        return f'"{escaped}"'
    raise TypeError(f"cannot serialise {value!r}")


def dumps(node, indent=0):
    """Serialise a nested list. Short leaf lists stay on one line, so diffs stay readable."""
    if not isinstance(node, list):
        return _atom(node)
    pad = "  " * indent
    if all(not isinstance(child, list) for child in node):
        return pad + "(" + " ".join(_atom(child) for child in node) + ")"
    head = []
    rest = []
    for child in node:
        if not rest and not isinstance(child, list):
            head.append(_atom(child))
        else:
            rest.append(child)
    lines = [pad + "(" + " ".join(head)]
    for child in rest:
        if isinstance(child, list):
            lines.append(dumps(child, indent + 1))
        else:
            lines.append("  " * (indent + 1) + _atom(child))
    lines[-1] += ")"
    return "\n".join(lines)


def loads(text):
    """Parse one S-expression into nested lists of str. Quoted and bare atoms both become str."""
    tokens = _tokenize(text)
    pos = 0

    def parse():
        nonlocal pos
        token = tokens[pos]
        pos += 1
        if token == "(":
            items = []
            while tokens[pos] != ")":
                items.append(parse())
            pos += 1
            return items
        if token == ")":
            raise ValueError("unexpected ')'")
        return token

    result = parse()
    return result


def _tokenize(text):
    tokens = []
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        if c in " \t\r\n":
            i += 1
        elif c in "()":
            tokens.append(c)
            i += 1
        elif c == '"':
            i += 1
            out = []
            while text[i] != '"':
                if text[i] == "\\":
                    i += 1
                    out.append({"n": "\n"}.get(text[i], text[i]))
                else:
                    out.append(text[i])
                i += 1
            i += 1
            tokens.append("".join(out))
        else:
            j = i
            while j < n and text[j] not in ' \t\r\n()"':
                j += 1
            tokens.append(text[i:j])
            i = j
    return tokens
