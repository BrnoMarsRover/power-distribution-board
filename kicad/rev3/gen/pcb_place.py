#!/usr/bin/env python3
"""Rev3 floorplan in rev2's style (run with KiCad's Python, before any hand placement):

    flatpak run --command=python3 org.kicad.KiCad kicad/rev3/gen/pcb_place.py --overwrite-placement

150 x 150 mm, rev2's five M3 holes. Reading the board from the top down:
  inputs row      -> each input above the switch it feeds
  switch row      -> each rail's eFuse (FETs, shunt, bypass bridge) above its outputs
  output columns  -> one straight column per rail, every connector the same way round: + on the left
Columns, left to right: 42 V DRIVE | 42 V CONVERTERS | 24 V | 12 V | 5 V | 54 V (switch, boost, Zyxel out).
Under the short XT90 columns: brake-resistor output, aux supply and the control connectors.
Power parts on top; control and logic on the bottom, under their own power stage. All connectors on top.
"""
import sys
import pcbnew

BOARD = "kicad/rev3/PowerBoard_rev3.kicad_pcb"
if "--overwrite-placement" not in sys.argv:
    raise SystemExit("pcb_place.py moves every footprint. Run it with --overwrite-placement, and only "
                     "before hand placement starts (or on a copy).")
X0, Y0, W, H = 50.0, 50.0, 150.0, 150.0
HOLES = [(3.5, 3.5), (146.5, 3.5), (3.5, 146.5), (146.5, 146.5)]   # rev2 corners; its centre hole would sit in a column
MM, TOMM = pcbnew.FromMM, pcbnew.ToMM
GAP = 0.6                                    # between packed parts (courtyard rule is 0.5)
GAP_BOTTOM = 1.0                             # the bottom has room: a little air for labels and rework

# column x ranges (board mm) and the rows
COL = {"42D": (2.0, 24.0), "42C": (27.0, 49.0), "24": (52.0, 81.0), "12": (84.0, 108.0),
       "5": (111.0, 129.0), "54": (131.0, 149.0)}
Y_IN, Y_OUT = 1.0, 75.0                      # tops of the inputs row and of the output columns
ROW = 10.2                                   # output pitch for 24 / 12 / 5 V (upright 2920 PTCs need 10.1)
Y_CTRL = 137.4                               # control-connector strip along the bottom edge
ROW90 = 12.5                                 # output pitch for the XT90 columns

b = pcbnew.LoadBoard(BOARD)
FP = {f.GetReference(): f for f in b.GetFootprints()}


def pad(fp, num):
    """First pad with this number (FindPadByNumber returns an untyped SWIG object in KiCad 10)."""
    return next((p for p in fp.Pads() if p.GetNumber() == num), None)


def net(fp, num):
    p = pad(fp, num)
    return p.GetNetname() if p else ""


def box(fp):
    """Courtyard box (fallback: body) in board mm."""
    fp.BuildCourtyardCaches()
    poly = fp.GetCourtyard(pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd)
    bb = poly.BBox() if poly.OutlineCount() else fp.GetBoundingBox(False)
    return (TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0, TOMM(bb.GetBottom()) - Y0)


def move_box(fp, left, top):
    l, t, _, _ = box(fp)
    fp.SetPosition(fp.GetPosition() + pcbnew.VECTOR2I(MM(left - l), MM(top - t)))


def set_side(fp, bottom):
    if fp.IsFlipped() != bottom:
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)


def through(fp):
    pads = list(fp.Pads())
    return sum(p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH for p in pads) * 2 > len(pads)


placed = {False: [], True: []}
fixed = set()


def claim(fp):
    bx = box(fp)
    for side in ((False, True) if through(fp) else (fp.IsFlipped(),)):
        placed[side].append(bx)


def plus_left(fp):
    """Turn a 2-pin connector so pin 1 (+) is on the left and the pins sit side by side."""
    if pad(fp, "1") is None or pad(fp, "2") is None:
        return
    for deg in (0, 90, 180, 270):
        fp.SetOrientationDegrees(deg)
        p1, p2 = pad(fp, "1").GetPosition(), pad(fp, "2").GetPosition()
        if p1.x < p2.x and abs(p1.y - p2.y) < MM(0.5):
            return


def upright(fp, tall):
    """Rotate a 2-pad part so its long side is vertical (tall) or horizontal."""
    for deg in (0, 90):
        fp.SetOrientationDegrees(deg)
        l, t, r, bt = box(fp)
        if (bt - t > r - l) == tall:
            return


def put(ref, left, top, orient=plus_left):
    fp = FP[ref]
    set_side(fp, False)
    orient(fp)
    move_box(fp, left, top)
    claim(fp)
    fixed.add(ref)
    return box(fp)


def overlaps(a, bx, gap=GAP):
    return not (a[2] + gap <= bx[0] or bx[2] + gap <= a[0] or a[3] + gap <= bx[1] or bx[3] + gap <= a[1])


def label_room(fp):
    """Signal test pads reserve room to their right for their net name (1 mm text)."""
    lib = fp.GetFPID().GetLibItemName().wx_str()
    if fp.GetReference().startswith("TP") and "Loop" not in lib:
        return 0.75 * len(fp.GetValue()) + 0.8
    return 0.0


def fit(fp, zone, bottom):
    zx0, zy0, zx1, zy1 = zone
    sides = (False, True) if through(fp) else (bottom,)
    gap = GAP_BOTTOM if bottom else GAP
    for deg in (0, 90):
        fp.SetOrientationDegrees(deg if not bottom else (180 - deg) % 360)
        l, t, r, bt = box(fp)
        w, h = r - l + label_room(fp), bt - t
        y = zy0
        while y + h <= zy1:
            x = zx0
            while x + w <= zx1:
                cand = (x, y, x + w, y + h)
                if not any(overlaps(cand, o, gap) for s in sides for o in placed[s]) and \
                        not any(overlaps(cand, o, gap) for o in hole_boxes):
                    return cand
                x += 0.5
            y += 0.5
    return None


def area(ref):
    bx = box(FP[ref])
    return (bx[2] - bx[0]) * (bx[3] - bx[1])


overflow, spilled = [], []
SPILL = (1, 1, 149, 149)


def pack(refs, zone, bottom, spill=False):
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
            move_box(fp, W + 15 + 30 * (len(overflow) // 20), 7 * (len(overflow) % 20))
            continue
        if spill:
            spilled.append(ref)
        move_box(fp, ok[0], ok[1])
        claim(fp)
        if label_room(fp):
            placed[bottom].append((ok[2] - label_room(fp), ok[1], ok[2], ok[3]))   # keep the label spot free
    return left


# ---------------------------------------------------------------- holes and outline
holes = sorted((f for r, f in FP.items() if r.startswith("H") and f.GetValue() == "M3"), key=lambda f: f.GetReference())
while len(holes) < len(HOLES):
    h = holes[0].Duplicate(False).Cast()
    h.SetReference(f"H{len(holes) + 1}")
    h.SetBoardOnly(True)
    h.SetExcludedFromBOM(True)
    h.GetField(pcbnew.FIELD_T_REFERENCE).SetVisible(False)
    b.Add(h)
    holes.append(h)
    FP[h.GetReference()] = h
hole_boxes = []
for h, (hx, hy) in zip(holes, HOLES):
    h.SetPosition(pcbnew.VECTOR2I(MM(X0 + hx), MM(Y0 + hy)))
    hole_boxes.append((hx - 4, hy - 4, hx + 4, hy + 4))
    fixed.add(h.GetReference())
# resize the import's outline rectangle (BOARD.Remove() breaks SWIG typing for the rest of the run)
rect = next(d for d in b.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts)
rect.SetStart(pcbnew.VECTOR2I(MM(X0), MM(Y0)))
rect.SetEnd(pcbnew.VECTOR2I(MM(X0 + W), MM(Y0 + H)))

# ---------------------------------------------------------------- find connectors and PTCs by their nets
conns = {r: f for r, f in FP.items() if r.startswith("J") and r[1:].isdigit()}
by_net = lambda n: sorted((j for j, f in conns.items() if net(f, "1") == n and f.GetValue().startswith("XT90PB-F")),
                          key=lambda s: int(s[1:]))
ptc_of = {}
for r, f in FP.items():
    if r.startswith("F"):
        a, c = net(f, "1"), net(f, "2")
        for j, jf in conns.items():
            if net(jf, "1") in (a, c):
                ptc_of[j] = (r, c if net(jf, "1") == a else a)
rail_out = lambda rail: sorted((j for j, (p, rl) in ptc_of.items() if rl == rail), key=lambda s: int(s[1:]))

# ---------------------------------------------------------------- inputs row
put("J1", 8.0, Y_IN)                                                # battery XT90, clear of the corner hole
put("J2", 31.5, Y_IN)                                               # charger XT60
put("J3", 31.5, Y_IN + 10.5)                                        # pack charge port XT60
for rail, ref in (("24", "J5"), ("12", "J6"), ("5", "J7")):         # converter returns, above their switch
    FP[ref].SetOrientationDegrees(0)
    plus_left(FP[ref])
    l, _, r, _ = box(FP[ref])
    put(ref, COL[rail][1] - (r - l), Y_IN)

# ---------------------------------------------------------------- output columns
for k, j in enumerate(by_net("OUT42D")):
    put(j, COL["42D"][0], Y_OUT + ROW90 * k)
put("J4", COL["42D"][0], Y_OUT + ROW90 * len(by_net("OUT42D")) + 2.0)    # brake resistor, under the drives
for k, j in enumerate(by_net("OUT42C")):
    put(j, COL["42C"][0], Y_OUT + ROW90 * k)
for rail, net_ in (("24", "OUT24"), ("12", "OUT12"), ("5", "OUT5")):
    x0, x1 = COL[rail]
    for k, j in enumerate(rail_out(net_)):
        y = Y_OUT + ROW * k
        plus_left(FP[j])
        l, t, r, bt = box(FP[j])
        cbox = put(j, x1 - (r - l), y)                               # connector flush right in its column
        f = FP[ptc_of[j][0]]
        upright(f, tall=f.GetValue() != "MF-R300")                   # 2920 stands, the radial MF-R300 lies flat
        fl, ft, fr, fb = box(f)
        put(ptc_of[j][0], cbox[0] - 1.2 - (fr - fl), (cbox[1] + cbox[3]) / 2 - (fb - ft) / 2, orient=lambda fp: None)
put("J13", COL["54"][1] - 8.4, 133.5)                                # Zyxel 54 V out, bottom of the 54 V column


def opening(fp, d=(0, 1)):
    """Side-entry connector: turn so the mating face (away from the signal pins) points along d."""
    sig = [p for p in fp.Pads() if p.GetNumber()[:1].isdigit() or p.GetNumber()[:1] in "AB"]
    best = None
    for deg in (0, 90, 180, 270):
        fp.SetOrientationDegrees(deg)
        cx = sum(p.GetPosition().x for p in sig) / len(sig)
        cy = sum(p.GetPosition().y for p in sig) / len(sig)
        bb = fp.GetBoundingBox(False)
        score = (bb.GetCenter().x - cx) * d[0] + (bb.GetCenter().y - cy) * d[1]
        if best is None or score > best[0]:
            best = (score, deg)
    fp.SetOrientationDegrees(best[1])


# control strip along the bottom edge, left to right; CAN and USB plug in from the edge
x = 50.5
for ref, orient in (("J34", opening), ("J35", opening), ("J32", opening), ("J33", plus_left),
                    ("J37", plus_left), ("J38", plus_left), ("J36", plus_left)):
    fp = FP[ref]
    orient(fp)
    l, t, r, bt = box(fp)
    put(ref, x, 149.4 - (bt - t), orient=lambda f: None)
    x += (r - l) + 0.7

# ---------------------------------------------------------------- zones: (top, bottom) per sheet prefix
ZONES = {
    "Battery input": ((1.5, 21, 50.5, 35.5), (1.5, 21, 50.5, 35.5)),
    "42 V drive group": ((1.5, 35.5, 25.5, 71.5), (1.5, 35.5, 25.5, 71.5)),
    "42 V converter-feed": ((25.5, 35.5, 50.5, 71.5), (25.5, 35.5, 50.5, 71.5)),
    "24 V rail eFuse": ((51, 12, 82.5, 71.5), (51, 12, 82.5, 71.5)),
    "12 V rail eFuse": ((82.5, 12, 110, 71.5), (82.5, 12, 110, 71.5)),
    "5 V rail eFuse": ((110, 10, 130.5, 71.5), (110, 10, 130.5, 71.5)),
    "54 V supply eFuse": ((130.5, 9, 149, 50), (130.5, 9, 149, 50)),
    "54 V boost": ((130.5, 50, 149, 133), (130.5, 50, 149, 133)),
    "Brake chopper": ((1.5, 121.5, 26, 149), (1.5, 121.5, 26, 149)),
    "Always-on aux": ((26.5, 98, 50.5, 149), (26.5, 98, 50.5, 149)),
    "Rail inputs": ((51, 131, 130.5, 137), (51, 72, 130.5, 137)),
    "ESP32-S3": ((26.5, 98, 50.5, 149), (51, 72, 130.5, 137)),
    "Switch control": ((51, 131, 130.5, 137), (51, 72, 130.5, 137)),
    "CAN bus": ((51, 131, 130.5, 137), (51, 72, 130.5, 137)),
}
TOP_ONLY = ("54 V boost", "Always-on aux")                 # switching loops stay on one side
POWER_1210 = ("Battery input", "54 V boost", "Always-on aux")


def is_bottom(fp):
    sheet, ref, lib = fp.GetSheetname(), fp.GetReference(), fp.GetFPID().GetLibItemName().wx_str()
    if sheet.startswith(TOP_ONLY) or ref.startswith(("J", "SW", "JP", "H", "F")):
        return False
    if ref.startswith("TP"):
        return "Loop" not in lib                 # signal pads sit with their (bottom-side) circuitry
    if "1210" in lib:
        return not sheet.startswith(POWER_1210)  # bootstrap caps go under their controller; bus ceramics stay up
    if any(k in lib for k in ("LED", "2512", "Kelvin", "CP_", "TO-263", "TO-252", "Inductor", "L_",
                              "Bridge", "D_SMB", "D_SMC", "Fuse")):
        return False
    return ref.startswith(("U", "R", "C", "Q", "D", "TH"))


groups = {}
for r, f in FP.items():
    if r in fixed:
        continue
    key = next((k for k in ZONES if f.GetSheetname().startswith(k)), None)
    if key is None:
        overflow.append(r)
        continue
    groups.setdefault((key, is_bottom(f)), []).append(r)
left = {False: [], True: []}
for (key, bottom), refs in sorted(groups.items(), key=lambda kv: -sum(area(r) for r in kv[1])):
    left[bottom] += pack(refs, ZONES[key][1 if bottom else 0], bottom)
for bottom in (False, True):
    pack(left[bottom], SPILL, bottom, spill=True)

b.Save(BOARD)
top = sum(1 for f in b.GetFootprints() if not f.IsFlipped())
print(f"placed: {top} top, {len(FP) - top} bottom; outside their zone ({len(spilled)}): {' '.join(sorted(spilled))}")
print(f"no room on the board ({len(overflow)}): {' '.join(sorted(overflow))}")
