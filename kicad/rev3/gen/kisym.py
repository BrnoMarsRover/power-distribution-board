"""Minimal KiCad S-expression reader for symbol libraries (used by build.py)."""
import re
from pathlib import Path

TOKEN = re.compile(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()]+')


def parse(text):
    stack, cur = [], []
    for m in TOKEN.finditer(text):
        t = m.group(0)
        if t == '(':
            stack.append(cur)
            cur = []
        elif t == ')':
            done = cur
            cur = stack.pop()
            cur.append(done)
        else:
            cur.append(t)
    return cur[0] if len(cur) == 1 else cur


def unq(s):
    return s[1:-1].replace('\\"', '"') if isinstance(s, str) and s.startswith('"') else s


def dump(node, indent=0):
    """Serialize a parsed node back to KiCad text."""
    if not isinstance(node, list):
        return node
    if all(not isinstance(c, list) for c in node):
        return '(' + ' '.join(node) + ')'
    pad = '\t' * (indent + 1)
    parts = []
    head = []
    for c in node:
        if isinstance(c, list):
            break
        head.append(c)
    rest = node[len(head):]
    s = '(' + ' '.join(head)
    for c in rest:
        s += '\n' + pad + dump(c, indent + 1) if isinstance(c, list) else ' ' + c
    return s + '\n' + '\t' * indent + ')'


def find(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]


class Library:
    _cache = {}

    def __init__(self, path):
        self.path = Path(path)
        tree = parse(self.path.read_text())
        self.symbols = {unq(s[1]): s for s in find(tree, 'symbol')}

    @classmethod
    def load(cls, path):
        if path not in cls._cache:
            cls._cache[path] = cls(path)
        return cls._cache[path]

    def flat(self, name):
        """Return the symbol with any 'extends' resolved, as a fresh parsed tree."""
        import copy
        sym = copy.deepcopy(self.symbols[name])
        ext = find(sym, 'extends')
        if not ext:
            return sym
        parent = self.flat(unq(ext[0][1]))
        pname = unq(parent[1])
        out = [sym[0], sym[1]]
        child_props = {unq(p[1]): p for p in find(sym, 'property')}
        for c in parent[2:]:
            if isinstance(c, list) and c[0] == 'property' and unq(c[1]) in child_props:
                out.append(child_props.pop(unq(c[1])))
            elif isinstance(c, list) and c[0] == 'symbol':
                c = copy.deepcopy(c)
                c[1] = '"' + unq(c[1]).replace(pname, name, 1) + '"'
                out.append(c)
            else:
                out.append(c)
        out.extend(child_props.values())
        return out


def pins(sym, unit=1):
    """Pins of a (flattened) symbol: list of dicts with number, name, x, y, angle, type."""
    res = []
    for sub in find(sym, 'symbol'):
        m = re.match(r'.*_(\d+)_(\d+)$', unq(sub[1]))
        u, style = int(m.group(1)), int(m.group(2))
        if u not in (0, unit) or style not in (0, 1):
            continue
        for p in find(sub, 'pin'):
            at = find(p, 'at')[0]
            res.append(dict(type=p[1], number=unq(find(p, 'number')[0][1]),
                            name=unq(find(p, 'name')[0][1]),
                            x=float(at[1]), y=float(at[2]), angle=float(at[3]) if len(at) > 3 else 0.0,
                            length=float(find(p, 'length')[0][1]),
                            hidden=any(c == 'hide' for c in p) or bool(find(p, 'hide'))))
    return res
