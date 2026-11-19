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
COL = {"42D": (2.0, 24.0), "42C": (27.0, 49.0), "24": (52.0, 74.0), "12": (77.0, 101.0),
       "5": (104.0, 122.0), "54": (124.0, 149.0)}
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
    if not through(fp):                     # a lone through-hole pad (ESP32 solder hole) blocks the other side
        for p in fp.Pads():
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                bb = p.GetBoundingBox()
                placed[not fp.IsFlipped()].append((TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0,
                                                   TOMM(bb.GetRight()) - X0, TOMM(bb.GetBottom()) - Y0))


MINUS_PIN1 = ("XT30", "XT60", "DEANS")       # pad 1 is the - side on these footprints (see build.py CONN2)


def plus_minus(fp):
    """(+ pad number, - pad number) of a 2-pin power connector."""
    name = fp.GetFPID().GetLibItemName().wx_str().upper()
    return ("2", "1") if any(k in name for k in MINUS_PIN1) else ("1", "2")


def plus_left(fp):
    """Turn a connector so its + pad is on the left and the pins sit side by side."""
    if pad(fp, "1") is None or pad(fp, "2") is None:
        return
    pp, pm = plus_minus(fp)
    for deg in (0, 90, 180, 270):
        fp.SetOrientationDegrees(deg)
        p1, p2 = pad(fp, pp).GetPosition(), pad(fp, pm).GetPosition()
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
    if fp.GetReference().startswith("TP") and "Keystone" not in lib:
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
SPILL = (1, 13, 149, 149)                    # never into the inputs row


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
by_net = lambda n: sorted((j for j, f in conns.items() if net(f, plus_minus(f)[0]) == n and f.GetValue().startswith("XT90PB-F")),
                          key=lambda s: int(s[1:]))
ptc_of = {}
for r, f in FP.items():
    if r.startswith("F"):
        a, c = net(f, "1"), net(f, "2")
        for j, jf in conns.items():
            jp = net(jf, plus_minus(jf)[0])
            if jp in (a, c):
                ptc_of[j] = (r, c if jp == a else a)
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
        upright(f, tall=True)                                        # every PTC is a 2920 standing upright
        fl, ft, fr, fb = box(f)
        put(ptc_of[j][0], cbox[0] - 1.2 - (fr - fl), (cbox[1] + cbox[3]) / 2 - (fb - ft) / 2, orient=lambda fp: None)
# Zyxel 54 V out: bottom-left, on the board edge under the converter column
plus_left(FP["J13"])
l, t, r, bt = box(FP["J13"])
put("J13", 28.0, 149.4 - (bt - t))


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


# CAN on the right edge under the ESP32 (plug in from the edge)
for ref, y in (("J34", 94.0), ("J35", 107.3)):
    fp = FP[ref]
    opening(fp, (1, 0))
    l, t, r, bt = box(fp)
    put(ref, 149.4 - (r - l), y, orient=lambda f: None)


# JST headers in a block at the top right, a label row above each
JST_ROWS = (((131.0, 9.4), ("J37",)), ((131.0, 18.6), ("J38", "J36")), ((131.0, 27.8), ("J33",)))
for (x0, y0), refs in JST_ROWS:
    widths = []
    for ref in refs:
        plus_left(FP[ref])
        l, t, r, bt = box(FP[ref])
        widths.append(r - l)
    x = 149.4 - sum(widths) - 0.8 * (len(refs) - 1)
    for ref, w in zip(refs, widths):
        put(ref, x, y0)
        x += w + 0.8

# ---------------------------------------------------------------- zones: (top, bottom) per sheet prefix
ZONES = {
    "Battery input": ((1.5, 21, 50.5, 35.5), (1.5, 21, 50.5, 35.5)),
    "USB": ((123.5, 82, 149, 94), (123.5, 78, 149, 100)),
    "42 V drive group": ((1.5, 35.5, 25.5, 71.5), (1.5, 35.5, 25.5, 71.5)),
    "42 V converter-feed": ((25.5, 35.5, 50.5, 71.5), (25.5, 35.5, 50.5, 71.5)),
    "24 V rail eFuse": ((51, 12, 75.5, 71.5), (51, 12, 75.5, 71.5)),
    "12 V rail eFuse": ((75.5, 12, 102.5, 71.5), (75.5, 12, 102.5, 71.5)),
    "5 V rail eFuse": ((102.5, 10, 123.5, 71.5), (102.5, 10, 123.5, 71.5)),
    "54 V supply eFuse": ((123.5, 36, 149, 61.5), (123.5, 36, 149, 61.5)),
    "54 V boost": ((123.5, 120.6, 149, 149), (123.5, 36, 149, 149)),
    "Brake chopper": ((1.5, 121.5, 26, 149), (1.5, 121.5, 26, 149)),
    "Always-on aux": ((26.5, 98, 50.5, 149), (26.5, 98, 50.5, 149)),
    "Rail inputs": ((51, 131, 123.5, 149), (51, 72, 123.5, 149)),
    "ESP32-S3": ((123.5, 62, 149, 97), (123.5, 58, 149, 100)),
    "Switch control": ((78, 131, 123.5, 149), (51, 72, 123.5, 149)),
    "CAN bus": ((123.5, 93.5, 139, 120.5), (123.5, 93.5, 149, 121)),
}
SUPPLIES = ("54 V boost", "Always-on aux")   # SMD parts on the bottom, the whole switching loop on that side
POWER_1210 = ("Battery input",)


def is_bottom(fp):
    sheet, ref, lib = fp.GetSheetname(), fp.GetReference(), fp.GetFPID().GetLibItemName().wx_str()
    if ref.startswith(("J", "SW", "JP", "H", "F")):
        return False
    if sheet.startswith(SUPPLIES):                # 5.3 V aux and 54 V boost: SMD down, THT and LEDs up
        if ref.startswith("TP"):
            return "Keystone" not in lib
        return not (through(fp) or "LED" in lib)
    if ref.startswith("TP"):
        return "Keystone" not in lib                 # signal pads sit with their (bottom-side) circuitry
    if "1210" in lib:
        return not sheet.startswith(POWER_1210)  # bootstrap caps go under their controller; bus ceramics stay up
    if any(k in lib for k in ("LED", "2512", "Kelvin", "CP_", "TO-263", "TO-252", "Inductor", "L_",
                              "Bridge", "D_SMB", "D_SMC", "Fuse")):
        return False
    return ref.startswith(("U", "R", "C", "Q", "D", "TH"))


# ESP32 on top at the right edge, its u.FL antenna connector pointing out of the board
esp = FP["U13"]
set_side(esp, False)
esp.SetOrientationDegrees(270)
l, t, r, bt = box(esp)
put("U13", 149.4 - (r - l), 61.6, orient=lambda f: None)

# under the ESP32: USB-C on the edge (opening outwards), RESET and BOOT beside it
opening(FP["J32"], (1, 0))
l, t, r, bt = box(FP["J32"])
put("J32", 149.4 - (r - l), 82.4, orient=lambda f: None)
x = 149.4 - (r - l) - 0.7
for ref in ("SW1", "SW2"):                   # stood upright so both fit beside the USB-C
    fp = FP[ref]
    set_side(fp, False)
    fp.SetOrientationDegrees(90)
    l, t, r, bt = box(fp)
    put(ref, x - (r - l), 82.4, orient=lambda f: None)
    x -= (r - l) + 0.7

# keep the label spots under the inputs free (battery label, polarity rule, + / - marks)
for keep in ((8.0, 12.4, 31.0, 18.0), (51.0, 9.6, 123.0, 12.0), (78.0, 140.0, 122.0, 146.0),
             (1.5, 71.8, 122.5, 74.8)):
    placed[False].append(keep)

# the USB connector's helpers (ESD array, CC resistors) sit by the connector, not by the ESP32
usb_nets = {p.GetNetname() for p in FP["J32"].Pads()} - {"GND", ""}
usb_parts = {r for r, f in FP.items() if r not in fixed and not r.startswith(("TP", "U13"))
             and any(p.GetNetname() in usb_nets for p in f.Pads())}

groups = {}
for r, f in FP.items():
    if r in fixed:
        continue
    key = "USB" if r in usb_parts else next((k for k in ZONES if f.GetSheetname().startswith(k)), None)
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
