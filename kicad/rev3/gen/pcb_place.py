#!/usr/bin/env python3
"""First-pass floorplan for the rev3 board (run with KiCad's Python, before any hand placement):

    flatpak run --command=python3 org.kicad.KiCad kicad/rev3/gen/pcb_place.py --overwrite-placement

150 x 150 mm (rev2 size and its five M3 holes). Power parts on top, control and logic on the bottom
(decided 2026-11-19). Places the connector banks on the edges with pin 1 (+) pointing into the board,
every output PTC beside its connector, and packs the rest of each sheet into its zone, the bottom-side
parts under their own power stage. It is a starting point: fine placement is done by hand in KiCad.
Parts that do not fit their zone are parked right of the board and listed.
"""
import sys
import pcbnew

BOARD = "kicad/rev3/PowerBoard_rev3.kicad_pcb"
X0, Y0, W, H = 50.0, 50.0, 150.0, 150.0
HOLES = [(3.5, 3.5), (146.5, 3.5), (3.5, 146.5), (146.5, 146.5), (75.0, 75.0)]     # rev2 pattern
MM, TOMM = pcbnew.FromMM, pcbnew.ToMM
GAP = 0.6                                    # between packed parts (courtyard rule is 0.5)

if "--overwrite-placement" not in sys.argv:
    raise SystemExit("pcb_place.py moves every footprint. Run it with --overwrite-placement, and only "
                     "before hand placement starts (or on a copy).")
BOARD = next((a for a in sys.argv[1:] if not a.startswith("--")), BOARD)
b = pcbnew.LoadBoard(BOARD)
FP = {f.GetReference(): f for f in b.GetFootprints()}


def pad(fp, num):
    """First pad with this number (FindPadByNumber returns an untyped SWIG object in KiCad 10)."""
    return next((p for p in fp.Pads() if p.GetNumber() == num), None)


def net(fp, num):
    p = pad(fp, num)
    return p.GetNetname() if p else ""


def box(fp):
    """Courtyard box (fallback: body) in board mm, relative to the board corner."""
    fp.BuildCourtyardCaches()
    layer = pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd
    poly = fp.GetCourtyard(layer)
    bb = poly.BBox() if poly.OutlineCount() else fp.GetBoundingBox(False)
    return (TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0, TOMM(bb.GetBottom()) - Y0)


def move_box_to(fp, cx, cy):
    l, t, r, bt = box(fp)
    d = pcbnew.VECTOR2I(MM(cx - (l + r) / 2), MM(cy - (t + bt) / 2))
    fp.SetPosition(fp.GetPosition() + d)


def set_side(fp, bottom):
    if fp.IsFlipped() != bottom:
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)


def orient_inward(fp, nx, ny):
    """Rotate so pad 1 sits further inward (direction nx, ny) than pad 2."""
    if pad(fp, "1") is None or pad(fp, "2") is None:
        return                                   # USB-C: no pin 1; orient by hand (opening to the edge)
    best = None
    for deg in (0, 90, 180, 270):
        fp.SetOrientationDegrees(deg)
        p1, p2 = pad(fp, "1").GetPosition(), pad(fp, "2").GetPosition()
        score = (p1.x - p2.x) * nx + (p1.y - p2.y) * ny
        if best is None or score > best[0]:
            best = (score, deg)
    fp.SetOrientationDegrees(best[1])


placed = {False: [], True: []}
fixed = set()                              # parts placed on the edges             # side -> boxes (top False, bottom True)


def through(fp):
    """Mostly through-hole pads: it occupies both sides (the ESP32's single solder hole does not count)."""
    pads = list(fp.Pads())
    return sum(p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH for p in pads) * 2 > len(pads)


def claim(fp, both=False):
    bx = box(fp)
    if both or through(fp):
        placed[False].append(bx)
        placed[True].append(bx)
    else:
        placed[fp.IsFlipped()].append(bx)


def edge(refs, side, start, pitch, margin=0.6):
    """Connector bank along an edge; side in top/bottom/left/right; start/pitch along the edge (mm)."""
    n = {"top": (0, 1), "bottom": (0, -1), "left": (1, 0), "right": (-1, 0)}[side]
    for k, ref in enumerate(refs):
        fp = FP[ref]
        set_side(fp, False)
        orient_inward(fp, *n)
        l, t, r, bt = box(fp)
        w, h = r - l, bt - t
        s = start + k * pitch
        cx, cy = {"top": (s, margin + h / 2), "bottom": (s, H - margin - h / 2),
                  "left": (margin + w / 2, s), "right": (W - margin - w / 2, s)}[side]
        move_box_to(fp, cx, cy)
        claim(fp)
        fixed.add(ref)


def beside(conn, ref, side, gap=1.5):
    """Put ref (a PTC) on the inner side of a connector, centred on it."""
    fp, c = FP[ref], box(FP[conn])
    set_side(fp, False)
    fp.SetOrientationDegrees(90 if side in ("top", "bottom") else 0)
    l, t, r, bt = box(fp)
    w, h = r - l, bt - t
    cx, cy = (c[0] + c[2]) / 2, (c[1] + c[3]) / 2
    pos = {"top": (cx, c[3] + gap + h / 2), "bottom": (cx, c[1] - gap - h / 2),
           "left": (c[2] + gap + w / 2, cy), "right": (c[0] - gap - w / 2, cy)}[side]
    move_box_to(fp, *pos)
    claim(fp)
    fixed.add(ref)


def overlaps(a, bx):
    return not (a[2] + GAP <= bx[0] or bx[2] + GAP <= a[0] or a[3] + GAP <= bx[1] or bx[3] + GAP <= a[1])


overflow = []
park = [W + 15.0, 0.0, 0.0]                 # x, y, row height for parts that do not fit


def fit(fp, zone, bottom):
    """Bottom-left fill: first free spot in zone (x0, y0, x1, y1), 0.5 mm grid, 0 or 90 degrees."""
    zx0, zy0, zx1, zy1 = zone
    sides = (False, True) if through(fp) else (bottom,)                 # THT pins go through
    for deg in (0, 90):
        fp.SetOrientationDegrees(deg if not bottom else (180 - deg) % 360)
        l, t, r, bt = box(fp)
        w, h = r - l, bt - t
        y = zy0
        while y + h <= zy1:
            x = zx0
            while x + w <= zx1:
                cand = (x, y, x + w, y + h)
                if not any(overlaps(cand, o) for side in sides for o in placed[side]) and \
                        not any(overlaps(cand, o) for o in hole_boxes):
                    return cand
                x += 0.5
            y += 0.5
    return None


SPILL = {False: (1, 1, 149, 149), True: (1, 1, 149, 149)}        # fallback: anywhere still free
spilled = []


def area(ref):
    bx = box(FP[ref])
    return (bx[2] - bx[0]) * (bx[3] - bx[1])


def pack(refs, zone, bottom, spill=False):
    """Returns the refs that did not fit (when spill is False)."""
    left = []
    for ref in sorted(refs, key=lambda r: -area(r)):
        fp = FP[ref]
        set_side(fp, bottom)
        ok = fit(fp, zone, bottom)
        if ok is None and not spill:
            left.append(ref)
            continue
        if ok is None:
            overflow.append(ref)
            l, t, r, bt = box(fp)
            if park[1] + (bt - t) > H:
                park[0] += 30
                park[1] = 0
            move_box_to(fp, park[0] + (r - l) / 2, park[1] + (bt - t) / 2)
            park[1] += (bt - t) + 1
            continue
        if spill:
            spilled.append(ref)
        move_box_to(fp, (ok[0] + ok[2]) / 2, (ok[1] + ok[3]) / 2)
        claim(fp)
    return left


# ---------------------------------------------------------------- holes and outline
holes = [f for r, f in FP.items() if r.startswith("H") and f.GetValue() == "M3"]
while len(holes) < len(HOLES):
    h = holes[0].Duplicate(False).Cast()
    h.SetReference(f"H{len(holes) + 1}")
    h.SetValue("M3")
    h.SetBoardOnly(True)
    h.SetExcludedFromBOM(True)
    h.GetField(pcbnew.FIELD_T_REFERENCE).SetVisible(False)
    for hp in h.Pads():
        hp.SetNet(b.FindNet("GND"))
    b.Add(h)
    holes.append(h)
    FP[h.GetReference()] = h
hole_boxes = []
for h, (hx, hy) in zip(holes, HOLES):
    h.SetPosition(pcbnew.VECTOR2I(MM(X0 + hx), MM(Y0 + hy)))
    hole_boxes.append((hx - 4, hy - 4, hx + 4, hy + 4))
# resize the import's outline rectangle (BOARD.Remove() breaks SWIG typing for the rest of the run)
rect = next(d for d in b.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts)
rect.SetStart(pcbnew.VECTOR2I(MM(X0), MM(Y0)))
rect.SetEnd(pcbnew.VECTOR2I(MM(X0 + W), MM(Y0 + H)))


# ---------------------------------------------------------------- connectors by role (from their nets)
conns = {r: f for r, f in FP.items() if r.startswith("J") and r[1:].isdigit()}
ptc_of = {}
for r, f in FP.items():
    if r.startswith("F"):
        a, c = net(f, "1"), net(f, "2")
        for jr, jf in conns.items():
            if net(jf, "1") in (a, c):
                ptc_of[jr] = (r, c if net(jf, "1") == a else a)       # (PTC, rail it hangs off)
rail_out = lambda rail: sorted((j for j, (p, rl) in ptc_of.items() if rl == rail), key=lambda s: int(s[1:]))
xt90 = lambda n: sorted((j for j, f in conns.items() if f.GetValue().startswith("XT90PB-F") and net(f, "1") == n),
                        key=lambda s: int(s[1:]))

# top edge: drive XT90 x4, brake XT60, converter-feed XT90 x4
edge(xt90("OUT42D"), "top", 33.0, 12.5)
edge(["J4"], "top", 82.0, 0)
edge(xt90("OUT42C"), "top", 94.0, 12.5)
# left edge: battery, charger, pack charge port, 12 V in, 12 V outputs with PTCs
edge(["J1"], "left", 15.0, 0)
edge(["J2", "J3"], "left", 29.0, 11.0)
edge(["J6"], "left", 53.0, 0)
o12 = rail_out("OUT12")
edge(o12, "left", 64.0, 10.5)
for j in o12:
    beside(j, ptc_of[j][0], "left")
# right edge: 5 V in and 5 V outputs with PTCs
o5 = rail_out("OUT5")
edge(["J7"], "right", 28.0, 0)
edge(o5, "right", 35.5, 7.5)
for j in o5:
    beside(j, ptc_of[j][0], "right")
# bottom edge: 24 V in and 24 V outputs with PTCs
o24 = rail_out("OUT24")
edge(["J5"], "bottom", 28.0, 0)
edge(o24, "bottom", 38.5, 10.5)
for j in o24:
    beside(j, ptc_of[j][0], "bottom")
# small connectors: Zyxel 54 V top-right corner, USB-C / UART on the right edge, CAN on the bottom edge
edge(["J16"], "right", 17.0, 0)
edge(["J43", "J44"], "right", 122.0, 12.0)
edge(["J45", "J46"], "bottom", 124.0, 11.0)
done = fixed | {h.GetReference() for h in holes}

# ---------------------------------------------------------------- zones per sheet: (top zone, bottom zone)
SHEET_ZONE = {
    # sheet prefix: (top zone, bottom zone), board mm; bands: top y<24, left x<24..28, right x>124, bottom y>118
    "42 V drive group": ((44, 24, 84, 54), (44, 24, 84, 54)),
    "42 V converter-feed": ((96, 24, 124, 60), (96, 24, 124, 60)),
    "Brake chopper": ((84, 24, 96, 54), (84, 24, 96, 54)),
    "Battery input": ((24, 24, 44, 60), (24, 24, 44, 60)),
    "54 V supply eFuse": ((96, 60, 124, 76), (96, 60, 124, 76)),
    "54 V boost": ((60, 60, 96, 96), (60, 60, 96, 96)),
    "12 V rail eFuse": ((28, 60, 44, 118), (28, 60, 44, 118)),
    "24 V rail eFuse": ((44, 96, 96, 118), (44, 96, 96, 118)),
    "5 V rail eFuse": ((96, 76, 124, 118), (96, 76, 124, 118)),
    "Always-on aux": ((44, 60, 60, 96), (44, 60, 60, 96)),
    "Rail inputs": ((24, 60, 44, 118), (24, 60, 44, 118)),
    "ESP32-S3": ((96, 100, 124, 118), (56, 50, 94, 80)),
    "Switch control": ((44, 54, 96, 62), (56, 80, 94, 100)),
    "CAN bus": ((96, 112, 124, 118), (94, 100, 124, 118)),
}
TOP_ONLY_SHEETS = ("54 V boost", "Always-on aux")      # switching layouts stay on one side


def is_bottom(fp):
    """Control and logic parts go to the bottom; power parts, indicators and probe points stay on top."""
    if fp.GetSheetname().startswith(TOP_ONLY_SHEETS):
        return False
    ref, lib = fp.GetReference(), fp.GetFPID().GetLibItemName().wx_str()
    if ref.startswith("TP") and "Loop" not in lib:
        return True                              # signal pads sit with their (bottom-side) circuitry
    if ref.startswith(("TP", "J", "SW", "JP", "H", "F")):
        return False
    if any(k in lib for k in ("LED", "2512", "Kelvin", "CP_", "1210", "TO-263", "TO-252",
                              "Inductor", "L_", "Bypass", "D_SMB", "D_SMC", "Fuse")):
        return False
    return ref.startswith(("U", "R", "C", "Q", "D", "TH"))


groups = {}
for r, f in FP.items():
    if r in done:
        continue
    key = next((k for k in SHEET_ZONE if f.GetSheetname().startswith(k)), None)
    if key is None:
        overflow.append(r)
        continue
    groups.setdefault((key, is_bottom(f)), []).append(r)
left = {False: [], True: []}
for (key, bottom), refs in sorted(groups.items(), key=lambda kv: -sum(area(r) for r in kv[1])):
    left[bottom] += pack(refs, SHEET_ZONE[key][1 if bottom else 0], bottom)
for bottom in (False, True):
    pack(left[bottom], SPILL[bottom], bottom, spill=True)

b.Save(BOARD)
top = sum(1 for f in b.GetFootprints() if not f.IsFlipped())
print(f"placed: {top} top, {len(FP) - top} bottom; outside their zone ({len(spilled)}): {' '.join(sorted(spilled))}")
print(f"no room on the board ({len(overflow)}): {' '.join(sorted(overflow))}")
