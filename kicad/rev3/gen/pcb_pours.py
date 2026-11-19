#!/usr/bin/env python3
"""Power pours (run with KiCad's Python, after pcb_optimize.py; part of rebuild_layout.sh):

    flatpak run --command=python3 org.kicad.KiCad kicad/rev3/gen/pcb_pours.py

- L2 (In1.Cu) is a solid GND plane; L1 and L4 get a low-priority GND fill around everything else.
- Every power net gets pours shaped to its own power pads: connector pins, shunts, FET tabs and leads,
  TVS diodes, bulk capacitors, PTCs and bypass bridges. Pads more than CLUSTER mm from the rest of
  their net form a separate pour; the links between those pours are left for routed tracks.
- 42 V paths are poured on both outer layers (L1 + L4) and stitched with via arrays; the 24 / 12 / 5 V
  rails on L1 backed by L3 (In2.Cu). Overlaps are resolved by zone priority, and the fill keeps 0.5 mm
  from other nets (the 42 / 54 V clearance of the board rules).
"""
import fnmatch
import math
import pcbnew

BOARD = "kicad/rev3/PowerBoard_rev3.kicad_pcb"
X0, Y0, W, H = 50.0, 50.0, 150.0, 150.0
MM, TOMM = pcbnew.FromMM, pcbnew.ToMM
MARGIN, CLUSTER, CLEAR, CLEAR_INNER = 2.0, 18.0, 0.5, 0.3   # IPC-2221B internal, 31-100 V: 0.1 mm
VIA_D, VIA_DRILL, VIA_PITCH = 0.9, 0.45, 2.2
F, B, IN1, IN2 = pcbnew.F_Cu, pcbnew.B_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu
POWER_FP = ("XT90", "XT60", "XT30", "DEANS", "Kelvin", "2512", "TO-263", "TO-252", "D_SMB", "D_SMC",
            "CP_", "1210", "Fuse", "Bridge", "MSS", "SRR")

# net (or net suffix) -> (layers, priority, stitch)
OUTER = (F, B)
RAIL = (F, IN2)
POURS = [
    ("VBAT", OUTER, 35, True), ("BUS42", OUTER, 35, True), ("/CHG_N", OUTER, 45, True),
    ("/42 V drive group*/DRN", OUTER, 40, True), ("OUT42D", OUTER, 30, True),
    ("/42 V converter-feed*/DRN", OUTER, 40, True), ("OUT42C", OUTER, 30, True),
    ("/BRK_SW", OUTER, 45, True),
    ("VIN24", RAIL, 30, True), ("/24 V rail*/DRN", RAIL, 40, True), ("OUT24", RAIL, 30, True),
    ("VIN12", RAIL, 30, True), ("/12 V rail*/DRN", RAIL, 40, True), ("OUT12", RAIL, 30, True),
    ("VIN5", RAIL, 30, True), ("/5 V rail*/DRN", RAIL, 40, True), ("OUT5", RAIL, 30, True),
    ("/54 V supply*/DRN", OUTER, 40, True), ("SW54_OUT", OUTER, 30, True),
    ("OUT54", OUTER, 30, True),
]

b = pcbnew.LoadBoard(BOARD)
NETS = {n: b.FindNet(n) for n in [str(k) for k in b.GetNetsByName().keys()]}


def mm(v):
    return TOMM(v.x) - X0, TOMM(v.y) - Y0


def find_net(key):
    hits = [n for n in NETS if n == key or (fnmatch.fnmatch(n, key) if "*" in key else n.endswith(key))]
    assert len(hits) == 1, (key, hits)
    return hits[0]


def power_pads(netname):
    """Pad boxes plus the copper layers each pad is on (through-hole pads: all of them)."""
    out = []
    for f in b.GetFootprints():
        name = f.GetFPID().GetLibItemName().wx_str()
        if not any(k in name for k in POWER_FP):
            continue
        for p in f.Pads():
            if p.GetNetname() == netname:
                bb = p.GetBoundingBox()
                (l, t), (r, bt) = mm(pcbnew.VECTOR2I(bb.GetLeft(), bb.GetTop())), mm(pcbnew.VECTOR2I(bb.GetRight(), bb.GetBottom()))
                on = {F, B, IN1, IN2} if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH else \
                     {L for L in (F, B) if p.IsOnLayer(L)}
                out.append((l, t, r, bt, frozenset(on)))
    return out


LINK = {"OUT24": 50.0, "OUT12": 50.0, "OUT5": 50.0}   # rails: the FETs and their PTC column are one pour


def clusters(boxes, link=CLUSTER):
    """Single-linkage groups of pad boxes closer than link mm (box to box)."""
    gap = lambda a, c: math.hypot(max(0, max(a[0], c[0]) - min(a[2], c[2])), max(0, max(a[1], c[1]) - min(a[3], c[3])))
    groups = []
    for bx in boxes:
        joined = [g for g in groups if any(gap(bx, o) < link for o in g)]
        merged = [bx] + [o for g in joined for o in g]
        groups = [g for g in groups if not any(g is j for j in joined)] + [merged]
    return groups


def hull(points):
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts
    cross = lambda o, a, c: (a[0] - o[0]) * (c[1] - o[1]) - (a[1] - o[1]) * (c[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def zone(netname, layer, poly, priority, name, clearance=CLEAR, keep_islands=False):
    z = pcbnew.ZONE(b)
    z.SetLayer(layer)
    z.SetNet(NETS[netname])
    z.SetAssignedPriority(priority)
    z.SetLocalClearance(MM(clearance))
    z.SetMinThickness(MM(0.5))           # no slivers: thinner GND necks between pours carry nothing useful
    # inner rail backbones are joined to the top fragments by vias placed after the first fill
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_NEVER if keep_islands else pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetZoneName(name)
    o = z.Outline()
    o.NewOutline()
    for x, y in poly:
        o.Append(MM(X0 + min(max(x, 0.4), W - 0.4)), MM(Y0 + min(max(y, 0.4), H - 0.4)))
    b.Add(z)
    return z


# ---------------------------------------------------------------- GND: plane on L2, fill on L1 / L4
board_poly = [(0.4, 0.4), (W - 0.4, 0.4), (W - 0.4, H - 0.4), (0.4, H - 0.4)]
zone("GND", IN1, board_poly, 0, "GND plane", clearance=CLEAR_INNER)
zone("GND", F, board_poly, 0, "GND fill top")
zone("GND", B, board_poly, 0, "GND fill bottom")

# ---------------------------------------------------------------- power pours
stitch = []                       # (netname, polygon, layers)
pad_vias = []                     # (netname, pads) whose power pads get vias down to the L3 backbone
count = 0
specs = [(find_net(k), layers, prio, st) for k, layers, prio, st in POURS]
# PTC -> connector links (about 8 mm, 3 A) are routed as tracks: a pour there would swallow the PTC's
# input pad and cut it off the rail
for netname, layers, prio, st in specs:
    boxes = power_pads(netname)
    for k, g in enumerate(clusters(boxes, LINK.get(netname, CLUSTER))):
        pts = [(x, y) for l, t, r, bt, _ in g for x in (l - MARGIN, r + MARGIN) for y in (t - MARGIN, bt + MARGIN)]
        poly = hull(pts)
        if len(poly) < 3:
            continue
        has = set().union(*(on for *_, on in g))
        # L3 backbone wherever the net has an L3 pour planned and at least two power pads to join
        use = [L for L in layers if (L in has or L == IN2) and (L != IN2 or len(g) >= 2)]
        for layer in use:
            zone(netname, layer, poly, prio, f"{netname.rsplit('/', 1)[-1]} {k + 1}",
                 clearance=CLEAR_INNER if layer in (IN1, IN2) else CLEAR, keep_islands=(layer == IN2))
            count += 1
        if IN2 in use:
            pad_vias.append((netname, g))
        if st and len(g) >= 2 and len(use) >= 2:
            stitch.append((netname, poly, use))


def fill():
    filler = pcbnew.ZONE_FILLER(b)
    filler.Fill(b.Zones())


fill()

# ---------------------------------------------------------------- via stitching between the layers of a pour
def inside(poly, x, y):
    c = False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[i - 1]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


zones_of = {}
for z in b.Zones():
    zones_of.setdefault((z.GetNetname(), z.GetLayer()), []).append(z)


def filled(netname, layer, x, y):
    return any(z.HitTestFilledArea(layer, pcbnew.VECTOR2I(MM(X0 + x), MM(Y0 + y))) for z in zones_of.get((netname, layer), []))


pad_boxes = []                    # no via in or near any pad (no via-in-pad: solder would wick away)
for f in b.GetFootprints():
    for p in f.Pads():
        bb = p.GetBoundingBox()
        (l, t), (rr, bt) = mm(pcbnew.VECTOR2I(bb.GetLeft(), bb.GetTop())), mm(pcbnew.VECTOR2I(bb.GetRight(), bb.GetBottom()))
        pad_boxes.append((l - 1.0, t - 1.0, rr + 1.0, bt + 1.0))      # clear of the thermal-relief spokes
near_pad = lambda x, y: any(l - VIA_D / 2 < x < rr + VIA_D / 2 and t - VIA_D / 2 < y < bt + VIA_D / 2
                            for l, t, rr, bt in pad_boxes)
vias = 0
taken = []
r = VIA_D / 2 + 0.35
for netname, poly, layers in stitch:
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    y = min(ys) + VIA_PITCH / 2
    while y < max(ys):
        x = min(xs) + VIA_PITCH / 2
        while x < max(xs):
            ring = [(x + r * math.cos(a), y + r * math.sin(a)) for a in [k * math.pi / 4 for k in range(8)]] + [(x, y)]
            if inside(poly, x, y) and not near_pad(x, y) and all(filled(netname, L, px, py) for L in layers for px, py in ring) \
                    and all(math.hypot(x - tx, y - ty) > VIA_PITCH * 0.9 for tx, ty in taken):
                v = pcbnew.PCB_VIA(b)
                v.SetPosition(pcbnew.VECTOR2I(MM(X0 + x), MM(Y0 + y)))
                v.SetWidth(MM(VIA_D))
                v.SetDrill(MM(VIA_DRILL))
                v.SetNet(NETS[netname])
                b.Add(v)
                taken.append((x, y))
                vias += 1
            x += VIA_PITCH
        y += VIA_PITCH
# ---------------------------------------------------------------- vias beside each power pad, down to L3
def add_via(netname, x, y):
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(pcbnew.VECTOR2I(MM(X0 + x), MM(Y0 + y)))
    v.SetWidth(MM(VIA_D))
    v.SetDrill(MM(VIA_DRILL))
    v.SetNet(NETS[netname])
    b.Add(v)
    taken.append((x, y))


raw_pads = []                     # (box, netname) without margins
for f in b.GetFootprints():
    for p in f.Pads():
        bb = p.GetBoundingBox()
        (l, t), (rr, bt) = mm(pcbnew.VECTOR2I(bb.GetLeft(), bb.GetTop())), mm(pcbnew.VECTOR2I(bb.GetRight(), bb.GetBottom()))
        raw_pads.append(((l, t, rr, bt), p.GetNetname()))

pvias = 0
for netname, g in pad_vias:
    for l, t, rr, bt, on in g:
        if IN2 in on:
            continue                                  # through-hole pads already reach L3
        top = F if F in on else B
        cx, cy, hx, hy = (l + rr) / 2, (t + bt) / 2, (rr - l) / 2, (bt - t) / 2
        placed_here = 0
        for ang, extra in [(ang, e) for e in (0.85, 1.4, 2.0, 2.8) for ang in range(0, 360, 15)]:
            if placed_here >= 2:
                break
            a = math.radians(ang)
            d = max(abs(math.cos(a)) * hx, abs(math.sin(a)) * hy) + extra     # power pads connect solid
            x, y = cx + d * math.cos(a), cy + d * math.sin(a)
            ring = [(x + r * math.cos(q), y + r * math.sin(q)) for q in [k * math.pi / 4 for k in range(8)]] + [(x, y)]
            if all(filled(netname, L, px, py) for L in (top, IN2) for px, py in ring) \
                    and not any(lp - m < x < rp + m and tp - m < y < bp + m
                                for (lp, tp, rp, bp), pn in raw_pads for m in [0.8 if pn == netname else 1.4]) \
                    and all(math.hypot(x - tx, y - ty) > 1.4 for tx, ty in taken):
                add_via(netname, x, y)
                placed_here += 1
                pvias += 1

for z in b.Zones():                # vias are in: now drop every fragment nothing connects to
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
fill()
b.Save(BOARD)
print(f"pours: {count} power zones + 3 GND, {vias} stitching vias, {pvias} pad vias to the L3 rail backbones")
