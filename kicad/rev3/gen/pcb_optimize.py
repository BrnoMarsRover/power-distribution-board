#!/usr/bin/env python3
"""Placement optimizer for the small parts (run with KiCad's Python, after pcb_place.py, before pcb_silk.py):

    flatpak run --command=python3 org.kicad.KiCad kicad/rev3/gen/pcb_optimize.py [--iterations N]

Simulated annealing over positions and 90-degree rotations of the movable parts: passives, small ICs and
transistors, SMD diodes, LEDs and signal test pads. Connectors, FETs, inductors, PTCs, electrolytics, bypass
bridges, the ESP32 and the mounting holes stay where pcb_place.py put them, and no part changes side.

Cost (mm, lower is better):
- half-perimeter wire length of every net, weighted: x4 for switching / gate / Kelvin-sense nets, x1 for
  other sheet-local nets, x0.5 for named global signals; GND and nets with more than 12 pads are left out
- decoupling: every capacitor with one pad on GND pulls its other pad toward the nearest IC pin on the same
  rail (x3)
Every accepted state is legal: courtyards keep 0.6 mm apart (through-hole parts block both sides), parts
stay on the board, off the mounting holes and off the silkscreen spots pcb_place.py reserved. Signal test
pads keep their label room to the right.
"""
import math
import random
import sys
import time

from pathlib import Path

import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
import power42  # noqa: E402
import power_rails  # noqa: E402
import power54  # noqa: E402
import power_aux  # noqa: E402
import efuse_hand
import far42  # noqa: E402
import flt_bus  # noqa: E402

BOARD = "kicad/rev3/PowerBoard_rev3.kicad_pcb"
X0, Y0, W, H = 50.0, 50.0, 150.0, 150.0
GAP = 0.6
HOLES = [(3.5, 3.5), (146.5, 3.5), (3.5, 146.5), (146.5, 146.5), (75.0, 75.0)]   # as pcb_place.py
KEEP = [(8.0, 12.4, 31.0, 16.3), (51.0, 9.6, 123.0, 12.0), (78.0, 140.0, 122.0, 146.0),   # label spots
        (49.0, 49.0, 92.0, 69.5),                                                           # logo + title
        (1.5, 71.8, 122.5, 74.8)]                                                           # column headers
# moves: --iterations N, else 50 000 per movable part (at most 20 million); the templates place nearly everything,
# so the optimizer is left with a couple of dozen parts, which settle long before 20 million moves
ITER = int(sys.argv[sys.argv.index("--iterations") + 1]) if "--iterations" in sys.argv else None
SEED = 7
HEAVY = ("SW", "HO", "LO", "BST", "CSP", "CSN", "SNS", "GATE", "L:G", "RON", "FB", "RIP", "VCC", "COMP",
         "SLOPE", "PU", "TMR", "IWRN", "ISCP", "LIN")
MM, TOMM = pcbnew.FromMM, pcbnew.ToMM

random.seed(SEED)
b = pcbnew.LoadBoard(BOARD)
FPS = sorted(b.GetFootprints(), key=lambda f: f.GetReference())   # by reference: the import order varies


def mm(v):
    return (TOMM(v.x) - X0, TOMM(v.y) - Y0)


def lib(f):
    return f.GetFPID().GetLibItemName().wx_str()


def through(f):
    pads = list(f.Pads())
    return bool(pads) and sum(p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH for p in pads) * 2 > len(pads)


def movable(f):
    r, name = f.GetReference(), lib(f)
    if f.IsLocked():
        return False
    if r == "U13" or r.startswith(("J", "H", "F", "L", "JP", "SW", "LOGO")) or through(f):
        return False
    if any(k in name for k in ("TO-263", "TO-252", "Kelvin", "2512", "CP_", "Bridge", "Keystone", "D_SMC",
                               "SRR", "MSS", "HTSSOP", "1210")):
        return False
    return r.startswith(("R", "C", "D", "Q", "U", "TP", "TH"))


def courtyard(f):
    f.BuildCourtyardCaches()
    poly = f.GetCourtyard(pcbnew.B_CrtYd if f.IsFlipped() else pcbnew.F_CrtYd)
    bb = poly.BBox() if poly.OutlineCount() else f.GetBoundingBox(False)
    l, t = mm(pcbnew.VECTOR2I(bb.GetLeft(), bb.GetTop()))
    r, bt = mm(pcbnew.VECTOR2I(bb.GetRight(), bb.GetBottom()))
    return l, t, r, bt


# ---------------------------------------------------------------- parts: position, box and pads relative to it
class Part:
    pass


parts, fixed_boxes = [], {False: [], True: []}
for f in FPS:
    pos = mm(f.GetPosition())
    l, t, r, bt = courtyard(f)
    if not movable(f):
        if f.GetReference().startswith("TP") and "Keystone" not in lib(f):     # placed pads keep their label room
            r += 0.75 * len(f.GetValue()) + 0.8
        both = through(f) and f.GetReference().startswith("J")   # connectors: whole outline (see pcb_place.py)
        for side in ((False, True) if both else (f.IsFlipped(),)):
            fixed_boxes[side].append((l, t, r, bt))
        if not both:                        # holes (ESP32 solder hole, USB-C pegs, THT legs) block the other side
            m = 1.0 if through(f) else 0.0
            for pp in f.Pads():
                if pp.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):   # plated or not
                    bb = pp.GetBoundingBox()
                    a, c = mm(pcbnew.VECTOR2I(bb.GetLeft(), bb.GetTop())), mm(pcbnew.VECTOR2I(bb.GetRight(), bb.GetBottom()))
                    fixed_boxes[not f.IsFlipped()].append((a[0] - m, a[1] - m, c[0] + m, c[1] + m))
        continue
    p = Part()
    p.fp, p.ref, p.side = f, f.GetReference(), f.IsFlipped()
    p.x, p.y, p.rot = pos[0], pos[1], 0                      # rot: quarter turns added by the optimizer
    p.box = (l - pos[0], t - pos[1], r - pos[0], bt - pos[1])
    p.pads = [(pp.GetNetname(), mm(pp.GetPosition())[0] - pos[0], mm(pp.GetPosition())[1] - pos[1])
              for pp in f.Pads()]
    if p.ref.startswith("TP"):
        p.box = (p.box[0], p.box[1], p.box[2] + 0.75 * len(f.GetValue()) + 0.8, p.box[3])   # label room
        p.can_rotate = False
    else:
        p.can_rotate = True
    parts.append(p)
for hx, hy in HOLES:
    k = 3.5 if (hx, hy) == (75.0, 75.0) else 4.0     # as pcb_place.py's HOLE_KEEP
    for side in (False, True):
        fixed_boxes[side].append((hx - k, hy - k, hx + k, hy + k))
for k in KEEP + power42.KEEP_TOP + power_rails.KEEP_TOP + power54.KEEP_TOP + power_aux.KEEP_TOP:
    fixed_boxes[False].append(k)
for k in (power42.KEEP_BOTTOM + power_rails.KEEP_BOTTOM + power54.KEEP_BOTTOM + power_aux.KEEP_BOTTOM
          + efuse_hand.keep_bottom() + efuse_hand.keep_bottom42() + far42.KEEP + flt_bus.KEEP):
    fixed_boxes[True].append(k)


def rot_xy(dx, dy, q):
    """Rotate an offset by q quarter turns as seen on the board (y down): (dx, dy) -> (dy, -dx)."""
    for _ in range(q % 4):
        dx, dy = dy, -dx
    return dx, dy


def box_at(p, x, y, q):
    l, t, r, bt = p.box
    pts = [rot_xy(l, t, q), rot_xy(r, bt, q)]
    xs, ys = [a for a, _ in pts], [c for _, c in pts]
    return (x + min(xs), y + min(ys), x + max(xs), y + max(ys))


def pad_at(p, i, x, y, q):
    _, dx, dy = p.pads[i]
    dx, dy = rot_xy(dx, dy, q)
    return x + dx, y + dy


# ---------------------------------------------------------------- nets and weights
fixed_pads = {}
for f in FPS:
    if not movable(f):
        for pp in f.Pads():
            fixed_pads.setdefault(pp.GetNetname(), []).append(mm(pp.GetPosition()))
pins_on = {}
for p in parts:
    for i, (n, _, _) in enumerate(p.pads):
        pins_on.setdefault(n, []).append((p, i))
nets = {}
for n in sorted(set(pins_on) | set(fixed_pads)):           # sorted: set order changes from run to run
    count = len(pins_on.get(n, [])) + len(fixed_pads.get(n, []))
    if not n or n == "GND" or n.startswith("unconnected") or count < 2 or count > 12 or n not in pins_on:
        continue
    local = n.rsplit("/", 1)[-1]
    w = 4.0 if any(local.startswith(k) or local.endswith(k) for k in HEAVY) else (1.0 if n.startswith("/") else 0.5)
    nets[n] = w

# decoupling: caps with one GND pad pull their other pad to the nearest IC pin on that rail
ic_pins = {}
for f in FPS:
    if f.GetReference().startswith("U"):
        for pp in f.Pads():
            if pp.GetNetname() not in ("GND", ""):
                ic_pins.setdefault(pp.GetNetname(), []).append((f.GetReference(), pp))
decaps = []
for p in parts:
    if p.ref.startswith("C") and len(p.pads) == 2 and "GND" in (p.pads[0][0], p.pads[1][0]):
        i = 0 if p.pads[1][0] == "GND" else 1
        if p.pads[i][0] in ic_pins:
            decaps.append((p, i))
moving_ic = {p.ref: p for p in parts if p.ref.startswith("U")}


# precompute pad indices of moving ICs for decoupling lookups
ic_index = {}
for ref, q in moving_ic.items():
    for pp in q.fp.Pads():
        n = pp.GetNetname()
        x, y = mm(pp.GetPosition())
        for k, (pn, dx, dy) in enumerate(q.pads):
            if pn == n and abs(q.x + dx - x) < 1e-3 and abs(q.y + dy - y) < 1e-3:
                ic_index[(ref, pp.GetNumber(), x, y)] = k
dec_targets = []
for p, i in decaps:
    tg = []
    for ref, pp in ic_pins[p.pads[i][0]]:
        x, y = mm(pp.GetPosition())
        tg.append((moving_ic.get(ref), ic_index.get((ref, pp.GetNumber(), x, y)), (x, y)))
    dec_targets.append((p, i, tg))


def net_cost(n):
    xs, ys = [], []
    for p, i in pins_on[n]:
        x, y = pad_at(p, i, p.x, p.y, p.rot)
        xs.append(x)
        ys.append(y)
    for x, y in fixed_pads.get(n, []):
        xs.append(x)
        ys.append(y)
    return nets[n] * ((max(xs) - min(xs)) + (max(ys) - min(ys)))


def dec_cost(entry):
    p, i, tg = entry
    x, y = pad_at(p, i, p.x, p.y, p.rot)
    best = 1e9
    for q, k, (fx, fy) in tg:
        tx, ty = pad_at(q, k, q.x, q.y, q.rot) if q is not None and k is not None else (fx, fy)
        best = min(best, math.hypot(x - tx, y - ty))
    return 3.0 * best


nets_of = {id(p): [n for n in sorted({pn for pn, _, _ in p.pads}) if n in nets] for p in parts}
decs_of = {}
for e in dec_targets:                       # each decoupling entry once per part that moves it
    for q in [e[0]] + [q for q, _, _ in e[2] if q is not None]:
        lst = decs_of.setdefault(id(q), [])
        if not any(x is e for x in lst):
            lst.append(e)

# ---------------------------------------------------------------- legality: spatial hash of boxes per side
CELL = 6.0
grid = {False: {}, True: {}}


def cells(bx):
    for cx in range(int(bx[0] // CELL), int(bx[2] // CELL) + 1):
        for cy in range(int(bx[1] // CELL), int(bx[3] // CELL) + 1):
            yield cx, cy


def add(side, key, bx):
    for c in cells(bx):
        grid[side].setdefault(c, {})[key] = bx


def remove(side, key, bx):
    for c in cells(bx):
        grid[side].get(c, {}).pop(key, None)


def legal(side, key, bx):
    if bx[0] < 0.5 or bx[1] < 0.5 or bx[2] > W - 0.5 or bx[3] > H - 0.5:
        return False
    seen = set()
    for c in cells((bx[0] - GAP, bx[1] - GAP, bx[2] + GAP, bx[3] + GAP)):
        for k, o in grid[side].get(c, {}).items():
            if k == key or k in seen:
                continue
            seen.add(k)
            if not (bx[2] + GAP <= o[0] or o[2] + GAP <= bx[0] or bx[3] + GAP <= o[1] or o[3] + GAP <= bx[1]):
                return False
    return True


for side in (False, True):
    for k, bx in enumerate(fixed_boxes[side]):
        add(side, ("fixed", k), bx)
for p in parts:
    p.cur = box_at(p, p.x, p.y, p.rot)
    add(p.side, p.ref, p.cur)

# ---------------------------------------------------------------- anneal
cache_net = {n: net_cost(n) for n in nets}
cache_dec = {id(e): dec_cost(e) for e in dec_targets}
total = sum(cache_net.values()) + sum(cache_dec.values())
start = total
T0, T1 = 3.0, 0.02
ITER = ITER or min(20000000, 50000 * len(parts))
t_start = time.time()
accepted = 0
for it in range(ITER):
    frac = it / ITER
    T = T0 * (T1 / T0) ** frac
    reach = max(0.5, 12.0 * (1 - frac))
    p = random.choice(parts)
    ox, oy, orot = p.x, p.y, p.rot
    if p.can_rotate and random.random() < 0.2:
        nx, ny, nrot = ox, oy, orot + random.choice((1, 3))
    else:
        nx, ny, nrot = ox + random.uniform(-reach, reach), oy + random.uniform(-reach, reach), orot
    nb = box_at(p, nx, ny, nrot)
    remove(p.side, p.ref, p.cur)
    if not legal(p.side, p.ref, nb):
        add(p.side, p.ref, p.cur)
        continue
    affected_n = nets_of[id(p)]
    affected_d = decs_of.get(id(p), [])
    old = sum(cache_net[n] for n in affected_n) + sum(cache_dec[id(e)] for e in affected_d)
    p.x, p.y, p.rot = nx, ny, nrot
    new_n = {n: net_cost(n) for n in affected_n}
    new_d = {id(e): dec_cost(e) for e in affected_d}
    delta = sum(new_n.values()) + sum(new_d.values()) - old
    if delta <= 0 or random.random() < math.exp(-delta / T):
        cache_net.update(new_n)
        cache_dec.update(new_d)
        total += delta
        p.cur = nb
        add(p.side, p.ref, nb)
        accepted += 1
    else:
        p.x, p.y, p.rot = ox, oy, orot
        add(p.side, p.ref, p.cur)

# ---------------------------------------------------------------- write back (rotate, then move), verify pads
for p in parts:
    f = p.fp
    if p.rot % 4:
        sign = 1
        base = f.GetOrientationDegrees()
        f.SetOrientationDegrees((base + 90 * sign * (p.rot % 4)) % 360)
        # KiCad's angle sense on a flipped footprint is mirrored: check one pad and correct if needed
        want = pad_at(p, 0, p.x, p.y, p.rot)
        f.SetPosition(pcbnew.VECTOR2I(MM(X0 + p.x), MM(Y0 + p.y)))
        got = mm(next(iter(f.Pads())).GetPosition())
        if abs(got[0] - want[0]) > 0.05 or abs(got[1] - want[1]) > 0.05:
            f.SetOrientationDegrees((base - 90 * (p.rot % 4)) % 360)
    f.SetPosition(pcbnew.VECTOR2I(MM(X0 + p.x), MM(Y0 + p.y)))
final = sum(net_cost(n) for n in nets) + sum(dec_cost(e) for e in dec_targets)
assert abs(final - total) < 1e-3 * max(1.0, final), (final, total)     # the running total must match a recount
b.Save(BOARD)
print(f"optimized {len(parts)} parts, {len(nets)} nets, {len(dec_targets)} decoupling caps: "
      f"cost {start:.0f} -> {total:.0f} ({100 * (1 - total / start):.0f} % lower), "
      f"{accepted} of {ITER} moves accepted, {time.time() - t_start:.0f} s")
