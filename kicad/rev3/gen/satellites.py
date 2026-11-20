"""Template placement of a controller IC's small parts ("satellites") around it, by net.

Used by pcb_place.py after the power blocks are placed and before the generic packer.

For one IC the caller gives its position and orientation, the area its parts may use, and the order of the IC's
pins by how much their parts matter (gate drive and bootstrap first, pull-downs last). Each small part on the IC's
sheet is then anchored by its nets:
  - to a fixed point given for that net (a FET's gate escape via, an LED via, a spot under a FET), if any;
  - else to the IC pin on that net;
  - else (second hop, e.g. a dV/dt cap behind its resistor) to the pad of the part it hangs on, placed right after it;
  - else to a default point (dividers whose other end is an ADC elsewhere).
Greedy first: parts in that priority order (decoupling caps before the rest at the same pin), each at the nearest
legal spot: its anchored pad as close to the anchor as the courtyards allow (rings of 0.25 mm, four rotations),
inside the area, clear of everything already there. Then simulated annealing shortens the connections (moves,
jumps next to a terminal, swaps of same-size parts), always keeping the courtyards legal.
The test pads go first, in one column with their names to the right, so the cluster grows around them.
"""
import math
import os
import random
import time

import pcbnew

import power42
import power_rails
from power42 import MM, TOMM, X0, Y0, _at

SUPPLY = {"GND", "+3V3", "+5V_LOGIC", "+5V3_AUX", "BUS42", "VBAT", "OUT42D", "OUT42C", "OUT24", "OUT12", "OUT5",
          "VIN24", "VIN12", "VIN5", "OUT54"}
GAP = 0.55                             # between courtyards (rule 0.5)
GATE_PINS = 4                          # the first pins of a pin order drive gates: their nets weigh more
ITERATIONS = int(os.environ.get("SAT_ITER", 150000))   # annealing moves per run (SAT_ITER: quick debug runs)
RUNS = 3                               # annealing runs per cluster (different seeds), the best is kept
COMPACT = 0.15                         # cost per mm from a part to its IC: keeps the cluster round
CAP_PULL = 2.0                         # cost per mm from a capacitor pad to the IC pin on its net (GND aside)


def _box(fp):
    fp.BuildCourtyardCaches()
    bb = fp.GetCourtyard(pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd).BBox()
    return TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0, TOMM(bb.GetBottom()) - Y0


def _pos(fp):
    return TOMM(fp.GetPosition().x) - X0, TOMM(fp.GetPosition().y) - Y0


def _pad_xy(p):
    return TOMM(p.GetPosition().x) - X0, TOMM(p.GetPosition().y) - Y0


def _overlap(a, b, gap):
    return not (a[2] + gap <= b[0] or b[2] + gap <= a[0] or a[3] + gap <= b[1] or b[3] + gap <= a[1])


def _small(fp):
    r = fp.GetReference()
    lib = fp.GetFPIDAsString().split(":")[-1]
    if r.startswith(("R", "C", "TH")):
        return not any(k in lib for k in ("CP_", "2512", "Kelvin"))
    return r.startswith(("D", "Q")) and lib.startswith(("SOT", "SOD", "D_SOD", "D_SMA"))


def place_ic(fp, x, y, test):
    """Bottom side, centre at (x, y), first quarter turn for which test(pad_xy_by_number) holds."""
    for deg in (0, 90, 180, 270):
        _at(fp, x, y, deg, bottom=True)
        if test({p.GetNumber(): _pad_xy(p) for p in fp.Pads() if p.GetNumber()}):
            return
    raise RuntimeError(f"{fp.GetReference()}: no orientation")


class Part:
    """A satellite on the bottom: pads and courtyard per rotation (relative to the origin), and its state."""

    def __init__(self, fp):
        self.fp, self.ref = fp, fp.GetReference()
        self.shapes = {}
        for deg in (0, 90, 180, 270):
            _at(fp, 0.0, 0.0, deg, bottom=True)
            ox, oy = _pos(fp)
            pads = [(p.GetNumber(), p.GetNetname(), _pad_xy(p)[0] - ox, _pad_xy(p)[1] - oy) for p in fp.Pads()]
            l, t, r, b = _box(fp)
            self.shapes[deg] = (pads, (l - ox, t - oy, r - ox, b - oy))
        self.decoupling = self.ref.startswith("C") and any(n == "GND" for _, n, _, _ in self.shapes[0][0])
        l, t, r, b = self.shapes[0][1]
        self.area = (r - l) * (b - t)
        self.state = None                                 # (x, y, deg)

    def box(self, state=None):
        x, y, deg = state or self.state
        l, t, r, b = self.shapes[deg][1]
        return (x + l, y + t, x + r, y + b)

    def pads(self, state=None):
        x, y, deg = state or self.state
        return [(num, n, x + dx, y + dy) for num, n, dx, dy in self.shapes[deg][0]]

    def pad_offset(self, num, deg):
        return next((dx, dy) for k, n, dx, dy in self.shapes[deg][0] if k == num)

    def at_box(self, left, top, deg):
        """State with this rotation whose courtyard's top-left corner is (left, top)."""
        l, t, _, _ = self.shapes[deg][1]
        return (left - l, top - t, deg)

    def apply(self):
        _at(self.fp, *self.state, bottom=True)


def _rects(area):
    """An area is one rectangle (l, t, r, b) or a list of them."""
    return [tuple(area)] if isinstance(area[0], (int, float)) else [tuple(a) for a in area]


def _inside(box, area):
    return any(box[0] >= a[0] and box[1] >= a[1] and box[2] <= a[2] and box[3] <= a[3] for a in _rects(area))


def _bbox(area):
    rs = _rects(area)
    return min(a[0] for a in rs), min(a[1] for a in rs), max(a[2] for a in rs), max(a[3] for a in rs)


def _legal(box, area, obstacles):
    if not _inside(box, area):
        return False
    return not any(_overlap(box, o, GAP) for o in obstacles)


def put_ic(fp, x, y, test, area, obstacles, rmax=12.0):
    """Bottom side: the first quarter turn passing test(pad_xy_by_number), at the nearest legal spot to (x, y).
    Locks it and adds its courtyard to obstacles."""
    degs = []
    for deg in (0, 90, 180, 270):
        _at(fp, x, y, deg, bottom=True)
        if test({p.GetNumber(): _pad_xy(p) for p in fp.Pads() if p.GetNumber()}):
            degs.append(deg)
    if not degs:
        raise RuntimeError(f"{fp.GetReference()}: no orientation")
    deg = degs[0]
    _at(fp, 0.0, 0.0, deg, bottom=True)
    ox, oy = _pos(fp)
    l, t, r, b = _box(fp)
    d = 0.0
    while d <= rmax:
        n = max(1, int(2 * math.pi * d / 0.35))
        for k in range(n):
            a = 2 * math.pi * k / n
            cx, cy = x + d * math.cos(a), y + d * math.sin(a)
            box = (cx + l - ox, cy + t - oy, cx + r - ox, cy + b - oy)
            if _legal(box, area, obstacles):
                _at(fp, cx, cy, deg, bottom=True)
                fp.SetLocked(True)
                obstacles.append(_box(fp))
                return True
        d += 0.25
    raise RuntimeError(f"{fp.GetReference()}: no room near ({x}, {y})")


def _nearest_spot(part, num, ax, ay, area, obstacles, rmax=16.0):
    """State putting part's pad `num` as close to (ax, ay) as the courtyards allow, or None."""
    near = [o for o in obstacles if o[0] - rmax - 8 < ax < o[2] + rmax + 8 and o[1] - rmax - 8 < ay < o[3] + rmax + 8]
    d = 0.0
    while d <= rmax:
        n = max(1, int(2 * math.pi * d / 0.35))
        for k in range(n):
            a = 2 * math.pi * k / n
            tx, ty = ax + d * math.cos(a), ay + d * math.sin(a)
            for deg in (0, 90, 180, 270):
                dx, dy = part.pad_offset(num, deg)
                st = (tx - dx, ty - dy, deg)
                if _legal(part.box(st), area, near):
                    return st
        d += 0.25
    return None


def place_cluster(*args, gap=None, **kw):
    """_place_cluster with another courtyard gap (the logic groups use 0.5 mm, the rule; the eFuses keep GAP)."""
    global GAP
    saved = GAP
    GAP = gap or GAP
    try:
        return _place_cluster(*args, **kw)
    finally:
        GAP = saved


def _place_cluster(FP, ic_ref, area, obstacles, pin_order, fixed_anchors=None, default_anchor=None, tp_column=None,
                   supply_pull=None, members=None, extra_ics=(), locked_terminals=False, tp_area=None, decaps=False):
    """Place the small parts of ic_ref's sheet around it. obstacles: list of boxes (bottom side), extended in place.
    fixed_anchors: {net: (priority, (x, y))}; supply_pull: {supply net: f(x, y) -> cost of a pad there}.
    area: a rectangle or a list of them. Options for the logic groups (all off for the eFuses):
      members: the parts to place (default: the IC sheet's small parts and test pads);
      extra_ics: more ICs of the group (their pins anchor after pin_order's, in pin-number order);
      locked_terminals: pads of parts already placed (connectors, the ESP32, FET gates) count as terminals;
      tp_area: where the test-pad columns go (default: area), a new column started when one is full;
      decaps: capacitors on supplies only go one per IC supply pin, 100 nF first.
    Returns (placed refs, refs that found no spot)."""
    ic = FP[ic_ref]
    sheet = ic.GetSheetname()
    pins = {p.GetNumber(): p for p in ic.Pads() if p.GetNumber()}
    anchors = {}                                          # net -> (priority, (x, y))
    for i, num in enumerate(pin_order):
        net = pins[num].GetNetname()
        if net not in SUPPLY and net not in anchors:
            anchors[net] = (i, _pad_xy(pins[num]))
    all_pins = list(pins.values())
    k = len(pin_order)
    for ref in extra_ics:
        for p in sorted((p for p in FP[ref].Pads() if p.GetNumber()), key=lambda p: (len(p.GetNumber()), p.GetNumber())):
            all_pins.append(p)
            if p.GetNetname() not in SUPPLY and p.GetNetname() not in anchors:
                anchors[p.GetNetname()] = (k, _pad_xy(p))
                k += 1
    terminals = {}                                        # net -> [(x, y)]: IC pins and fixed points
    for p in all_pins:
        terminals.setdefault(p.GetNetname(), []).append(_pad_xy(p))
    for net, (pr, pt) in (fixed_anchors or {}).items():    # a fixed point is the net's only target
        anchors[net] = (pr, pt)
        terminals[net] = [pt]
    if members is None:
        members = [r for r, f in FP.items() if f.GetSheetname() == sheet and r != ic_ref and not f.IsLocked()
                   and (_small(f) or r.startswith("TP"))]
    else:
        members = [r for r in members if not FP[r].IsLocked()]
    if locked_terminals:                                  # pads already placed on the members' nets
        mine = {p.GetNetname() for r in members for p in FP[r].Pads()} - SUPPLY
        own = set(members) | {ic_ref, *extra_ics}
        bx0, by0, bx1, by1 = _bbox(area)
        mid = ((bx0 + bx1) / 2, (by0 + by1) / 2)
        for ref in sorted(FP):
            f = FP[ref]
            if ref in own or not f.IsLocked():
                continue
            for p in f.Pads():
                n = p.GetNetname()
                if n in mine and n not in (fixed_anchors or {}):
                    terminals.setdefault(n, []).append(_pad_xy(p))
        for n in sorted(mine):
            if n not in anchors and terminals.get(n):
                anchors[n] = (k + 10, min(terminals[n], key=lambda q: math.dist(q, mid)))
    cap_pin = {}                                          # decoupling cap -> (its supply pad, the IC pin)
    if decaps:
        supply_pins = [p for p in all_pins if p.GetNetname() in SUPPLY and p.GetNetname() != "GND"]
        caps = sorted((r for r in members if r.startswith("C")
                       and all(p.GetNetname() in SUPPLY for p in FP[r].Pads())),
                      key=lambda r: (not FP[r].GetValue().startswith("100n"), int(r[1:])))
        used = {}
        for r in caps:
            net = next(p.GetNetname() for p in FP[r].Pads() if p.GetNetname() != "GND")
            cands = [p for p in supply_pins if p.GetNetname() == net]
            if cands:
                pin = min(cands, key=lambda p: used.get(id(p), 0))
                used[id(pin)] = used.get(id(pin), 0) + 1
                num = next(p.GetNumber() for p in FP[r].Pads() if p.GetNetname() == net)
                cap_pin[r] = (num, _pad_xy(pin))
    done, missed = [], []

    # test pads first: one column, names to the right (pcb_silk.py writes the net name where the value sits)
    tps = sorted((r for r in members if r.startswith("TP") and "Keystone" not in FP[r].GetFPIDAsString()),
                 key=lambda r: FP[r].GetValue())
    t_area = tp_area or area
    bottom_y = _bbox(t_area)[3]
    if tp_column and tps:
        x, y = tp_column
        col_w = max(0.75 * len(FP[r].GetValue()) + 0.8 for r in tps) + 2.5 + GAP
        for r in tps:
            f = FP[r]
            label = 0.75 * len(f.GetValue()) + 0.8
            x_was, y_was = x, y
            while True:
                _at(f, x, y, 0, bottom=True)
                l, t, rr, b = _box(f)
                box = (l, t, rr + label, b)
                if box[3] > bottom_y and tp_area and x + col_w < _bbox(t_area)[2]:
                    x, y = x + col_w, tp_column[1]            # next column
                    continue
                if box[3] > bottom_y or _legal(box, t_area, obstacles):
                    break
                y += 0.05 if tp_area else 0.25
            if box[3] > bottom_y:
                if os.environ.get("SAT_DEBUG"):
                    _at(f, x, tp_column[1], 0, bottom=True)
                    l, t, rr, b = _box(f)
                    bx = (l, t, rr + label, b)
                    print("   TP blocked", r, [tuple(round(v, 2) for v in o) for o in obstacles if _overlap(bx, o, GAP)], t_area)
                missed.append(r)
                if tp_area:                                   # the next pad starts where this one did
                    x, y = x_was, y_was
                continue
            obstacles.append(box)
            f.SetLocked(True)
            done.append(r)
            v = f.GetField(pcbnew.FIELD_T_VALUE)
            v.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_CENTER)
            py = _pad_xy(next(iter(f.Pads())))[1]
            v.SetPosition(pcbnew.VECTOR2I(MM(X0 + rr + 0.3 + (label - 0.8) / 2), MM(Y0 + py)))
            y = box[3] + GAP + (b - t) / 2 + (1e-6 if tp_area else 0.0)    # (slack: exact fits)

    # greedy: pin-anchored parts by pin priority, each followed by what hangs on it
    parts = {r: Part(FP[r]) for r in members if not r.startswith("TP")}
    rest = list(parts)
    prio = {}
    unfit = []                                            # no legal spot in the greedy pass
    hang = {}                                             # net -> (priority, (x, y)) of a placed satellite's pad
    while rest:
        best = None
        for r in rest:
            pt = parts[r]
            if r in cap_pin:                              # decoupling cap: at its IC supply pin, early
                cand = (-0.5, False, pt.area, r, cap_pin[r][0], cap_pin[r][1])
                if best is None or cand < best:
                    best = cand
                continue
            for num, n, _, _ in pt.shapes[0][0]:
                if n in anchors:
                    pr, xy = anchors[n]
                elif n in hang:
                    pr, xy = hang[n]
                else:
                    continue
                cand = (pr, not pt.decoupling, pt.area, r, num, xy)
                if best is None or cand < best:
                    best = cand
        if best is None:                                  # nothing anchors the rest: default point, resistors first
            r = min(rest, key=lambda q: (not q.startswith("R"), int(q.lstrip("ABCDEFGHIJKLMNOPQRSTUVWXYZ") or 0)))
            num = next((k for k, n, _, _ in parts[r].shapes[0][0] if n not in SUPPLY), parts[r].shapes[0][0][0][0])
            cx0, cy0, cx1, cy1 = _bbox(area)
            best = (len(pin_order) + 50, False, 0, r, num, default_anchor or ((cx0 + cx1) / 2, (cy0 + cy1) / 2))
        pr, _, _, r, num, (ax, ay) = best
        rest.remove(r)
        pt = parts[r]
        st = _nearest_spot(pt, num, ax, ay, area, obstacles, rmax=30.0)
        if st is None and not _inside((ax, ay, ax, ay), area):   # anchor outside the area: from its nearest point
            ax, ay = min(((min(max(ax, a[0]), a[2]), min(max(ay, a[1]), a[3])) for a in _rects(area)),
                         key=lambda q: math.dist(q, (ax, ay)))
            st = _nearest_spot(pt, num, ax, ay, area, obstacles, rmax=30.0)
        if st is None:                                    # no room yet: on its anchor, the annealing makes room
            dx, dy = pt.pad_offset(num, 0)
            pt.state = (ax - dx, ay - dy, 0)
            unfit.append(r)
        else:
            pt.state = st
            obstacles.append(pt.box())
        prio[r] = pr
        for _, n, x, y in pt.pads():
            if n not in SUPPLY and n not in anchors and n not in hang:
                hang[n] = (pr + 0.5, (x, y))

    # improvement by simulated annealing. Cost per net: the spanning tree over its terminals (IC pins, fixed points,
    # test pads, the satellites' pads), weighted: gate-drive nets (fixed points and the first GATE_PINS pins) x3;
    # plus, for capacitors, the distance from each pad to the IC pin on its net (bootstrap cap: BST and SRC; VS,
    # timer and IMON caps), GND aside; plus supply_pull for supply pads that must reach a pour (VIN pads towards
    # the VIN area); plus a little compactness.
    weight = {n: 3.0 if pr < GATE_PINS else 1.0 for n, (pr, _) in anchors.items()}
    tp_pts = {}                                           # placed test pads: a light pull on their net's parts
    for r in done:
        for p in FP[r].Pads():
            tp_pts.setdefault(p.GetNetname(), []).append(_pad_xy(p))
    net_parts = {}                                        # net -> satellites on it (sorted: runs must repeat)
    for r in sorted(parts):
        for _, n, _, _ in parts[r].shapes[0][0]:
            if n not in SUPPLY and r not in net_parts.setdefault(n, []):
                net_parts[n].append(r)
    pin_pts = {}
    for p in all_pins:
        pin_pts.setdefault(p.GetNetname(), []).append(_pad_xy(p))
    pull = supply_pull or {}

    def mst(pts):
        if len(pts) < 2:
            return 0.0
        dist = [math.hypot(pts[0][0] - x, pts[0][1] - y) for x, y in pts]
        used = [False] * len(pts)
        used[0] = True
        total = 0.0
        for _ in range(len(pts) - 1):
            j = min((k for k in range(len(pts)) if not used[k]), key=lambda k: dist[k])
            total += dist[j]
            used[j] = True
            xj, yj = pts[j]
            for k in range(len(pts)):
                if not used[k]:
                    dist[k] = min(dist[k], math.hypot(xj - pts[k][0], yj - pts[k][1]))
        return total

    def net_cost(n, st):
        pts = list(terminals.get(n, [])) + [(x, y) for r in net_parts[n] if r in parts
                                            for _, nn, x, y in parts[r].pads(st[r]) if nn == n]
        return weight.get(n, 1.0) * mst(pts)

    cx, cy = _pos(ic)

    def part_cost(r, s):
        """Terms that depend on this part alone: decoupling distance, test-pad and supply pulls, compactness."""
        pt, c = parts[r], COMPACT * math.hypot(s[0] - cx, s[1] - cy)
        for num, n, x, y in pt.pads(s):
            if r in cap_pin:
                if num == cap_pin[r][0]:
                    c += 2 * CAP_PULL * math.dist((x, y), cap_pin[r][1])
            elif pt.ref.startswith("C") and n in pin_pts and n != "GND":
                c += CAP_PULL * min(math.hypot(x - px, y - py) for px, py in pin_pts[n])
            if n in tp_pts:
                c += 0.1 * min(math.hypot(x - px, y - py) for px, py in tp_pts[n])
            if n in pull:
                c += pull[n](x, y)
        return c

    greedy = {r: pt.state for r, pt in parts.items()}
    for r in parts:
        if r not in unfit:
            obstacles.remove(parts[r].box())
    wx0, wy0, wx1, wy1 = _bbox(area)
    window = (wx0 - 1, wy0 - 1, wx1 + 1, wy1 + 1)
    static = sorted(tuple(o) for o in obstacles if _overlap(o, window, 0.0))    # sorted: runs must repeat
    refs = sorted(parts)

    def ov(a, b):
        """Overlap area of two courtyards grown by half the gap each."""
        w = min(a[2], b[2]) - max(a[0], b[0]) + GAP
        h = min(a[3], b[3]) - max(a[1], b[1]) + GAP
        return w * h if w > 0 and h > 0 else 0.0

    def inside(bx):
        return _inside(bx, area)

    def total_cost(st):
        return sum(net_cost(n, st) for n in net_parts) + sum(part_cost(r, st[r]) for r in refs)

    same = [(a, b) for i, a in enumerate(refs) for b in refs[i + 1:] if abs(parts[a].area - parts[b].area) < 0.05]

    def anneal(seed):
        """One annealing run from the greedy placement, then legalised. Returns (cost, states, refs left out)."""
        rng = random.Random(seed)
        st = dict(greedy)
        ncost = {n: net_cost(n, st) for n in net_parts}
        pcost = {r: part_cost(r, st[r]) for r in refs}
        boxes = {r: parts[r].box(st[r]) for r in refs}

        def overlap_of(r, bx):
            return sum(ov(bx, o) for o in static) * 4.0 + sum(ov(bx, boxes[q]) for q in refs if q != r)

        over = {r: overlap_of(r, boxes[r]) for r in refs}
        T0, T1 = 3.0, 0.02
        for it in range(ITERATIONS):
            f = it / ITERATIONS
            T = T0 * (T1 / T0) ** f
            lam = 0.3 * (200.0 ** f)                      # overlap price per mm2, 0.3 -> 60
            if same and rng.random() < 0.2:               # swap two same-size parts, each maybe half a turn
                a, b = rng.choice(same)
                ba, bb = boxes[a], boxes[b]
                moved = {a: parts[a].at_box(bb[0], bb[1], rng.choice((st[a][2], (st[a][2] + 180) % 360))),
                         b: parts[b].at_box(ba[0], ba[1], rng.choice((st[b][2], (st[b][2] + 180) % 360)))}
            else:                                         # move one part: nudge, or jump next to a terminal
                r = rng.choice(refs)
                pt = parts[r]
                deg = rng.choice((0, 90, 180, 270)) if rng.random() < 0.25 else st[r][2]
                if rng.random() < 0.7:
                    sg = 0.15 + 2.0 * T / T0
                    s = (st[r][0] + rng.gauss(0, sg), st[r][1] + rng.gauss(0, sg), deg)
                else:
                    nets = [n for _, n, _, _ in pt.shapes[0][0] if n not in SUPPLY]
                    if not nets:                          # supply-only (a decoupling cap): next to its pin
                        if r not in cap_pin:
                            continue
                        num, (tx, ty) = cap_pin[r]
                    else:
                        n = rng.choice(nets)
                        pts = list(terminals.get(n, [])) + [(x, y) for q in net_parts[n] if q != r
                                                             for _, nn, x, y in parts[q].pads(st[q]) if nn == n]
                        if not pts:
                            continue
                        tx, ty = rng.choice(pts)
                        num = next(k for k, nn, _, _ in pt.shapes[deg][0] if nn == n)
                    dx, dy = pt.pad_offset(num, deg)
                    s = (tx - dx + rng.gauss(0, 1.0), ty - dy + rng.gauss(0, 1.0), deg)
                moved = {r: s}
            nb = {q: parts[q].box(s) for q, s in moved.items()}
            if not all(inside(bx) for bx in nb.values()):
                continue
            trial = dict(st)
            trial.update(moved)
            nets = sorted({n for q in moved for _, n, _, _ in parts[q].shapes[0][0] if n in net_parts})
            dn = {n: net_cost(n, trial) for n in nets}
            dp = {q: part_cost(q, trial[q]) for q in moved}
            old_o = sum(over[q] for q in moved)           # overlap change, each pair counted once
            saved = {q: boxes[q] for q in moved}
            boxes.update(nb)
            new_o = {q: overlap_of(q, nb[q]) for q in moved}
            new_sum = sum(new_o.values())
            if len(moved) == 2:
                a, b = list(moved)
                old_o -= ov(saved[a], saved[b])
                new_sum -= ov(nb[a], nb[b])
            delta = (sum(dn[n] - ncost[n] for n in nets) + sum(dp[q] - pcost[q] for q in moved)
                     + lam * (new_sum - old_o))
            if delta < 0 or rng.random() < math.exp(-delta / T):
                st = trial
                ncost.update(dn)
                pcost.update(dp)
                for q in moved:                           # the others' overlap totals change with them
                    for o in refs:
                        if o not in moved:
                            over[o] += ov(boxes[o], nb[q]) - ov(boxes[o], saved[q])
                over.update(new_o)
            else:
                boxes.update(saved)
        # legalise: by priority, each part that still overlaps goes to the nearest legal spot of its anchored pad
        placed_boxes, left = list(static), []
        for r in sorted(refs, key=lambda r: prio[r]):
            pt = parts[r]
            bx = pt.box(st[r])
            if inside(bx) and not any(_overlap(bx, o, GAP) for o in placed_boxes):
                placed_boxes.append(bx)
                continue
            num, _, px, py = next(((k, n, x, y) for k, n, x, y in pt.pads(st[r]) if n not in SUPPLY),
                                  pt.pads(st[r])[0])
            s = _nearest_spot(pt, num, px, py, area, placed_boxes, rmax=30.0 if locked_terminals else 16.0)
            if s is None:
                left.append(r)
                st[r] = greedy[r]
                continue
            st[r] = s
            placed_boxes.append(pt.box(s))
        return total_cost(st) + 1000.0 * len(left), st, left

    t_start = time.time()
    start = total_cost(greedy)
    runs = [anneal(seed) for seed in range(1, RUNS + 1)]
    c0, st, left = min(runs, key=lambda t: t[0])
    if not left and not unfit and c0 > start:            # annealing made it worse: keep the greedy result
        c0, st = start, dict(greedy)
    if left:                                              # no legal spot after annealing: keep the greedy result,
        c0, st, left = start, dict(greedy), []            # without the parts it could not fit (the packer takes them)
        for r in unfit:
            missed.append(r)
            del parts[r]
    for r, pt in parts.items():
        pt.state = st[r]
        obstacles.append(pt.box())
    print(f"  {ic_ref}: cost {start:.0f} -> {c0:.0f}, runs {' '.join(f'{c:.0f}' for c, _, _ in runs)} "
          f"({time.time() - t_start:.0f} s)")
    for r, pt in parts.items():
        pt.apply()
        pt.fp.SetLocked(True)
        done.append(r)
    return done, missed


# ---------------------------------------------------------------- the rail eFuses (TPS48111)
# Pins by how much their parts matter: bootstrap, gate pull-down / pull-up, the pre-charge gate, current sense,
# VS supply; then the small parts that belong right at their pin (enable / UVLO, the INP pull-downs, current
# warning, IMON) before the bulky timer caps; fault last. Pin 10 (the remote temperature transistor) is anchored to
# the main FET instead.
TPS_PINS = ["12", "14", "15", "11", "18", "17", "20", "1", "3", "2", "8", "7", "9", "4", "5", "10", "19"]
# pin 19 last: CS- on the rails (already anchored by pin 17), ISCP (short-circuit threshold) on the 42 V blocks
EFUSE_TEST = lambda p: (abs(p["11"][1] - p["20"][1]) < 0.05 and p["11"][0] < p["20"][0]   # gate-drive row on top,
                        and p["11"][1] < p["1"][1])                                         # pin 11 at its left


def _sheet_parts(FP, sheet):
    """A sheet's parts in creation order: by prefix, then number (each rail sheet is built by the same code)."""
    key = lambda r: (r.rstrip("0123456789"), int(r[len(r.rstrip("0123456789")):] or 0))
    return sorted((r for r, f in FP.items() if f.GetSheetname() == sheet), key=key)


def _signature(fp, ic):
    """What a part connects to, independent of the rail: per pad the IC pins on its net, or the supply kind."""
    pins = {}
    for p in ic.Pads():
        pins.setdefault(p.GetNetname(), []).append(p.GetNumber())
    out = []
    for p in sorted(fp.Pads(), key=lambda p: p.GetNumber()):
        n = p.GetNetname()
        kind = "GND" if n == "GND" else ("VIN" if n.startswith("VIN") else ("OUT" if n.startswith("OUT") else ""))
        out.append((p.GetNumber(), tuple(sorted(pins.get(n, []))), kind))
    return tuple(out)


def replicate(FP, src_ic, dst_ic, dx, dy, area, obstacles):
    """Copy the placement of src_ic's cluster onto dst_ic's (same sheet structure), shifted by (dx, dy), if every
    part matches by role and lands legally. Returns the refs placed, or None (nothing changed)."""
    src, dst = FP[src_ic], FP[dst_ic]
    a, b = _sheet_parts(FP, src.GetSheetname()), _sheet_parts(FP, dst.GetSheetname())
    if len(a) != len(b):
        return None
    pairs = [(s, d) for s, d in zip(a, b) if s != src_ic and FP[s].IsLocked() and not FP[d].IsLocked()
             and (_small(FP[s]) or s.startswith("TP"))]
    if any(s.rstrip("0123456789") != d.rstrip("0123456789") or _signature(FP[s], src) != _signature(FP[d], dst)
           for s, d in pairs):
        return None
    sx, sy = _pos(src)
    _at(dst, sx + dx, sy + dy, src.GetOrientationDegrees(), bottom=True)
    boxes = [_box(dst)]
    for s, d in pairs:
        x, y = _pos(FP[s])
        _at(FP[d], x + dx, y + dy, FP[s].GetOrientationDegrees(), bottom=True)
        bx = _box(FP[d])
        if d.startswith("TP"):                            # label room, and the label itself
            bx = (bx[0], bx[1], bx[2] + 0.75 * len(FP[d].GetValue()) + 0.8, bx[3])
            vs, vd = FP[s].GetField(pcbnew.FIELD_T_VALUE), FP[d].GetField(pcbnew.FIELD_T_VALUE)
            vd.SetHorizJustify(vs.GetHorizJustify())
            vd.SetPosition(vs.GetPosition() + pcbnew.VECTOR2I(MM(dx), MM(dy)))
        boxes.append(bx)
    if not all(_legal(bx, area, obstacles) for bx in boxes):
        return None
    obstacles += boxes
    for _, d in pairs:
        FP[d].SetLocked(True)
    dst.SetLocked(True)
    return [dst_ic] + [d for _, d in pairs]


def efuse(FP, key, obstacles, like=None):
    """TPS48111 of one rail block (power_rails.RAILS[key]) and its satellites, on the bottom under the switch.
    The IC sits right of the FET source-via field with its gate-drive pins (G, BST, SRC, PD, PU) in the top row on
    the left, towards the gate escape vias, and the sense pins (CS-, CS+, VS) on the right, towards the shunt's
    Kelvin vias. like: a rail already placed; its cluster is copied when it fits, so like blocks look alike.
    Returns (refs placed and locked, refs that found no spot)."""
    r = power_rails.RAILS[key]
    g = power_rails._geo(r)
    tps = lambda k: next(f for f in FP.values() if f.GetSheetname() == FP[power_rails.RAILS[k]["fets"][1]].GetSheetname()
                         and "TPS48111" in f.GetValue())
    u = tps(key)
    area = (r["xl"] + 0.2, 12.0, r["xr"] - 0.2, 71.5)
    notch = (g["col_r"] - 0.2, g["band_bot"], r["xr"] + 1.0, 72.0) if g["col_r"] < r["xr"] else None
    if notch:                                     # L-shaped block (5 V): full width only above the band
        obstacles.append(notch)
    if like:
        dx = g["fet_cx"] - power_rails._geo(power_rails.RAILS[like])["fet_cx"]
        done = replicate(FP, tps(like).GetReference(), u.GetReference(), dx, 0.0, area, obstacles)
        if done:
            print(f"  {u.GetReference()}: copied from the {like} V eFuse")
            return done, []
    pre, main = power_rails.FET_ROWS
    place_ic(u, g["fet_cx"] + 3.4, power_rails.LED_Y + 1.0,
             lambda p: abs(p["11"][1] - p["20"][1]) < 0.05 and p["11"][0] < p["20"][0] and p["11"][1] < p["1"][1])
    u.SetLocked(True)
    obstacles.append(_box(u))
    pnet = lambda fp, num: next(p.GetNetname() for p in fp.Pads() if p.GetNumber() == num)
    led_a = next(p.GetNetname() for p in FP[r["led"]].Pads() if p.GetNetname() != "GND")
    led_y = [y for x, y in power_rails._escapes(r)[4:]]
    fixed = {                                     # net: (priority, point); priorities count in TPS_PINS
        pnet(FP[r["fets"][1]], "1"): (-1, (g["fet_cx"] - 10.85, main - 2.54)),  # main gate resistor at its via
        led_a: (-1, (g["side_x"] + 1.7, max(led_y))),                           # LED resistor at the anode via
        pnet(u, "10"): (-1, (g["fet_cx"] + 3.4, main + 3.0)),  # temperature transistor under the tab's corner
    }
    done, missed = place_cluster(FP, u.GetReference(), area, obstacles, TPS_PINS, fixed_anchors=fixed,
                                 default_anchor=(g["fet_cx"], 60.0), tp_column=(r["xl"] + 5.6, 49.0),
                                 supply_pull={r["vin"]: lambda x, y: 0.5 * max(0.0, y - power_rails.VIN_BOT)})
    return [u.GetReference()] + done, missed


AREA42_TOP = {"42D": 20.0, "42C": 29.0}


def efuse42(FP, key, obstacles, like=None):
    """TPS48111 of one 42 V block (power42.BLOCKS[key]) and its satellites, on the bottom right of the OUT via
    field. Same orientation as the rails: gate-drive row on top (towards the pre-charge gate via, the main gate
    resistors at their own vias further down), sense pins on the right towards the Kelvin vias under the bar.
    VS, ISCP and EN feed from BUS42: those resistors are pulled towards the bar. like: a block already placed,
    copied when its parts match. Returns (refs placed and locked, refs that found no spot)."""
    blk = power42.BLOCKS[key]
    dx = blk["dx"]
    fet_cx = power42.FET_CX + dx
    pre, main1, main2 = power42.FET_ROWS
    tps = lambda k: next(f for f in FP.values() if f.GetSheetname() == FP[power42.BLOCKS[k]["fets"][1]].GetSheetname()
                         and "TPS48111" in f.GetValue())
    u = tps(key)
    # bottom of the block, under and above the BUS42 bar too; 42C stops below the input shunt's Kelvin vias,
    # where the battery monitor (INA228) belongs
    area = (blk["drn_l"], AREA42_TOP[key], blk["drn_r"] - 0.2, 74.0)
    if like:
        done = replicate(FP, tps(like).GetReference(), u.GetReference(), dx - power42.BLOCKS[like]["dx"], 0.0,
                         area, obstacles)
        if done:
            print(f"  {u.GetReference()}: copied from the {like} eFuse")
            return done, []
    place_ic(u, fet_cx + 3.4, pre + 2.9, EFUSE_TEST)
    u.SetLocked(True)
    obstacles.append(_box(u))
    pnet = lambda fp, num: next(p.GetNetname() for p in fp.Pads() if p.GetNumber() == num)
    anode = next(p for p in FP[blk["led"]].Pads() if p.GetNetname() != "GND")
    ax, ay = _pad_xy(anode)
    fixed = {                                     # net: (priority, point)
        pnet(FP[blk["fets"][1]], "1"): (-1, (fet_cx - 10.85, main1 - 2.54)),   # main gate resistors at their vias
        pnet(FP[blk["fets"][2]], "1"): (-1, (fet_cx - 10.85, main2 - 2.54)),
        anode.GetNetname(): (-1, (ax + 1.7, ay)),                              # LED resistor at the anode via
        pnet(u, "10"): (-1, (fet_cx + 3.4, main1 + 3.0)),  # temperature transistor under the first main FET's tab
    }
    bar = power42.BAR[3]
    done, missed = place_cluster(FP, u.GetReference(), area, obstacles, TPS_PINS, fixed_anchors=fixed,
                                 default_anchor=(fet_cx + 4.0, 68.0), tp_column=(blk["drn_l"] + 1.8, 61.5),
                                 supply_pull={"BUS42": lambda x, y: 0.5 * max(0.0, y - bar)})
    return [u.GetReference()] + done, missed
