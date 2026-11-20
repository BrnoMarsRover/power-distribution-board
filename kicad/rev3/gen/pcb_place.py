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
from pathlib import Path

import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
import power42  # noqa: E402  hand-designed battery input and 42 V blocks
import power_rails  # noqa: E402  hand-designed 24 / 12 / 5 V blocks (the rails it lists)
import power54  # noqa: E402  hand-designed 54 V boost
import power_aux  # noqa: E402  hand-designed always-on aux supply
import satellites  # noqa: E402  template placement of the controllers' small parts
import logic_place  # noqa: E402  the same for the logic and the supplies' control networks

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
COL = {"42D": (2.0, 24.0), "42C": (27.0, 49.0), "24": (52.0, 76.0), "12": (77.0, 103.3),
       "5": (104.5, 122.5), "54": (124.0, 149.0)}         # 24 / 12 V: laid out by power_rails.py
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


def blocks_both(fp):
    """Through-hole connectors block their whole outline on the other side too: plugging flexes the board,
    which cracks ceramic parts under them. Other through-hole parts (electrolytics, bridges) only need their
    joints kept clear, so on the other side they block their holes plus room for the iron."""
    return through(fp) and fp.GetReference().startswith("J")


HOLE_MARGIN = 1.0                            # around a through-hole joint on the far side: room for the iron


def claim(fp):
    bx = box(fp)
    for side in ((False, True) if blocks_both(fp) else (fp.IsFlipped(),)):
        placed[side].append(bx)
    if not blocks_both(fp):                 # holes (ESP32 solder hole, USB-C pegs, THT legs) block the other side
        m = HOLE_MARGIN if through(fp) else 0.0
        for p in fp.Pads():
            if p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):   # plated or not
                bb = p.GetBoundingBox()
                placed[not fp.IsFlipped()].append((TOMM(bb.GetLeft()) - X0 - m, TOMM(bb.GetTop()) - Y0 - m,
                                                   TOMM(bb.GetRight()) - X0 + m, TOMM(bb.GetBottom()) - Y0 + m))


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


def fit(fp, zone, bottom, gap=None):
    zx0, zy0, zx1, zy1 = zone
    sides = (False, True) if blocks_both(fp) else (bottom,)
    gap = gap or (GAP_BOTTOM if bottom else GAP)
    for deg in (0, 90):
        fp.SetOrientationDegrees(deg if not bottom else (180 - deg) % 360)
        l, t, r, bt = box(fp)
        w, h = r - l + label_room(fp), bt - t
        legs = []                            # other through-hole parts: their holes on the far side, from (l, t)
        if through(fp) and not blocks_both(fp):
            for p in fp.Pads():
                if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                    bb = p.GetBoundingBox()
                    legs.append((TOMM(bb.GetLeft()) - X0 - l - HOLE_MARGIN, TOMM(bb.GetTop()) - Y0 - t - HOLE_MARGIN,
                                 TOMM(bb.GetRight()) - X0 - l + HOLE_MARGIN, TOMM(bb.GetBottom()) - Y0 - t + HOLE_MARGIN))
        y = zy0
        while y + h <= zy1:
            x = zx0
            while x + w <= zx1:
                cand = (x, y, x + w, y + h)
                if not any(overlaps(cand, o, gap) for s in sides for o in placed[s]) and \
                        not any(overlaps(cand, o, gap) for o in hole_boxes) and \
                        not any(overlaps((x + a, y + b_, x + c, y + d), o, 0.0)
                                for a, b_, c, d in legs for o in placed[not bottom]):
                    return cand
                x += 0.5
            y += 0.5
    return None


def area(ref):
    bx = box(FP[ref])
    return (bx[2] - bx[0]) * (bx[3] - bx[1])


overflow, spilled = [], []
SPILL = (1, 13, 149, 149)                    # never into the inputs row


def pack(refs, zone, bottom, spill=False, gap=None):
    left = []
    for ref in sorted(refs, key=lambda r: -area(r)):
        fp = FP[ref]
        set_side(fp, bottom)
        ok = fit(fp, zone, bottom, gap)
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
    if rail in power_rails.RAILS:
        continue
    FP[ref].SetOrientationDegrees(0)
    plus_left(FP[ref])
    l, _, r, _ = box(FP[ref])
    put(ref, COL[rail][1] - (r - l), Y_IN)

# ---------------------------------------------------------------- output columns
for k, j in enumerate(by_net("OUT42D")):
    put(j, COL["42D"][0], Y_OUT + ROW90 * k)
for k, j in enumerate(by_net("OUT42C")):
    put(j, COL["42C"][0], Y_OUT + ROW90 * k)
for rail, net_ in (("24", "OUT24"), ("12", "OUT12"), ("5", "OUT5")):
    if rail in power_rails.RAILS:
        continue
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
# Zyxel 54 V out: on the bottom edge at the right of the boost area, its fuse (clip holder) lying left of it,
# pad 2 next to the + pin; the boost (power54.py) sits above them and left, under the converter XT90s
plus_left(FP["J13"])
l, t, r, bt = box(FP["J13"])
jbox = put("J13", 50.0 - (r - l), 149.4 - (bt - t))
fz = FP["F19"]
set_side(fz, False)
power42._flat(fz, 0.0, 0.0, "1", "2")                         # pad 1 (OUT54) left, pad 2 (to J13 +) right
fl, ft, fr, fb = box(fz)
jy = TOMM(pad(FP["J13"], plus_minus(FP["J13"])[0]).GetPosition().y) - Y0
fy = TOMM(pad(fz, "2").GetPosition().y) - Y0
put("F19", jbox[0] - 0.8 - (fr - fl), jy - (fy - ft), orient=lambda f: None)
# the boost's status LED at the top-left of its area, locked so it stays where it can be seen
led54 = next(r for r, f in FP.items() if f.GetSheetname().startswith("54 V boost") and "LED" in f.GetFPIDAsString())
set_side(FP[led54], False)
upright(FP[led54], tall=True)
put(led54, 26.8, 114.0, orient=lambda f: None)       # below the input via field
FP[led54].SetLocked(True)


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
    "USB": ((126.1, 82, 149, 94), (123.5, 78, 149, 100)),
    "42 V drive group": ((1.5, 35.5, 25.5, 71.5), (1.5, 35.5, 25.5, 71.5)),
    "42 V converter-feed": ((25.5, 35.5, 50.5, 71.5), (25.5, 35.5, 50.5, 71.5)),
    "24 V rail eFuse": ((51, 12, 76.0, 71.5), (51, 12, 76.0, 71.5)),
    "12 V rail eFuse": ((76.5, 12, 103.3, 71.5), (76.5, 12, 103.3, 71.5)),
    "5 V rail eFuse": ((104.1, 10, 125.6, 71.5), (104.1, 10, 125.6, 71.5)),
    # 54 V boost (fed from OUT42C since 2026-11-20): under the converter XT90s, next to the Zyxel output J13
    "54 V boost": ((26.0, 100.5, 50.3, 141.0), (9.5, 100.5, 50.3, 149.4)),     # bottom: also under the brake corner
    "Brake chopper": ((1.5, 121.5, 26, 149), (1.5, 121.5, 26, 149)),
    "Always-on aux": ((126.1, 35.1, 149.4, 59.4), (125.6, 35.1, 149.4, 122.0)),   # beside the ESP32 it powers
    "Rail inputs": ((51, 131, 123.5, 149), (51, 72, 123.5, 149)),
    "ESP32-S3": ((126.1, 62, 149, 97), (123.5, 58, 149, 100)),
    "Switch control": ((78, 131, 123.5, 149), (51, 72, 123.5, 149)),
    "CAN bus": ((126.1, 93.5, 139, 120.5), (123.5, 93.5, 149, 121)),
}
SUPPLIES = ("54 V boost", "Always-on aux")   # SMD parts on the bottom, the whole switching loop on that side
TIGHT = {"54 V boost": 0.6}                  # bottom gap override: the boost's hot loop packed at the courtyard rule
POWER_1210 = ("Battery input",)


def is_bottom(fp):
    sheet, ref, lib = fp.GetSheetname(), fp.GetReference(), fp.GetFPID().GetLibItemName().wx_str()
    if ref.startswith(("J", "SW", "JP", "H", "F")):
        return False
    if sheet.startswith(SUPPLIES):                # 5.3 V aux and 54 V boost: SMD down, THT, LEDs and test pads up
        if ref.startswith("TP"):
            return False                          # probed from the top with the board mounted
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
put("U13", 149.4 - (r - l), 60.0, orient=lambda f: None)
# its status LEDs in a column beside it, red / yellow / green from the top, locked so they stay readable
y = box(esp)[1] + 0.4
for ref in ("D20", "D19", "D18"):
    fp = FP[ref]
    set_side(fp, False)
    upright(fp, tall=True)
    fl, ft, fr, fb = box(fp)
    put(ref, box(esp)[0] - 0.6 - (fr - fl), y, orient=lambda f: None)
    fp.SetLocked(True)
    y += fb - ft + 0.6

# under the ESP32: USB-C on the edge (opening outwards), RESET and BOOT beside it
# The receptacle's front face (the footprint's "PCB Edge" line) sits on the board edge, so the plug's
# overmold seats against the receptacle instead of the board; the courtyard pokes 0.5 mm past the edge.
opening(FP["J32"], (1, 0))
l, t, r, bt = box(FP["J32"])
face = max(TOMM(g.GetStart().x) - X0 for g in FP["J32"].GraphicalItems()
           if g.GetLayer() == pcbnew.Dwgs_User and isinstance(g, pcbnew.PCB_SHAPE) and g.GetShape() == pcbnew.SHAPE_T_SEGMENT)
put("J32", W - (face - l), 81.4, orient=lambda f: None)
x = W - (face - l) - 0.7
for ref in ("SW1", "SW2"):                   # stood upright so both fit beside the USB-C
    fp = FP[ref]
    set_side(fp, False)
    fp.SetOrientationDegrees(90)
    l, t, r, bt = box(fp)
    put(ref, x - (r - l), 81.4, orient=lambda f: None)
    x -= (r - l) + 0.7

# battery input, both 42 V blocks and the brake: placed by hand (power42.py), locked, their copper kept clear
for ref in power42.place(FP):
    claim(FP[ref])
    fixed.add(ref)
placed[False] += power42.KEEP_TOP
placed[True] += power42.KEEP_BOTTOM
for ref in power_rails.place(FP):            # 24 / 12 / 5 V blocks with their inputs and output columns
    claim(FP[ref])
    fixed.add(ref)
placed[False] += power_rails.KEEP_TOP
placed[True] += power_rails.KEEP_BOTTOM
for ref in power54.place(FP):                # the 54 V boost's power stage, its electrolytics and controller
    claim(FP[ref])
    fixed.add(ref)
placed[False] += power54.KEEP_TOP
placed[True] += power54.KEEP_BOTTOM
for ref in power_aux.place(FP):              # the always-on aux supply beside the ESP32
    claim(FP[ref])
    fixed.add(ref)
placed[False] += power_aux.KEEP_TOP
placed[True] += power_aux.KEEP_BOTTOM
# controllers and their small parts by template (satellites.py), before the packer; what finds no spot is packed
for key in ("24", "12", "5"):                # 12 and 5 V copy the 24 V cluster where it fits
    refs, missed = satellites.efuse(FP, key, placed[True], like=None if key == "24" else "24")
    fixed.update(refs)
    print(f"{key} V eFuse template: {len(refs)} placed" + (f", no spot: {' '.join(missed)}" if missed else ""))
for key in ("42D", "42C"):                   # 42C copies 42D where the parts match
    refs, missed = satellites.efuse42(FP, key, placed[True], like=None if key == "42D" else "42D")
    fixed.update(refs)
    print(f"{key} eFuse template: {len(refs)} placed" + (f", no spot: {' '.join(missed)}" if missed else ""))
for ref in fixed:                           # placed parts are terminals for the logic groups: lock them all
    FP[ref].SetLocked(True)
refs, _ = logic_place.place(FP, placed[True])
fixed.update(refs)
if "--dump-free" in sys.argv:                # bottom obstacles after the templates, for floorplanning
    import json
    json.dump({"bottom": placed[True], "top": placed[False]}, open("kicad/rev3/gen/out/obstacles.json", "w"))


# keep the label spots under the inputs free (battery label, polarity rule, + / - marks)
for keep in ((8.0, 12.4, 31.0, 16.3), (51.0, 9.6, 123.0, 12.0), (78.0, 140.0, 122.0, 146.0),
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
FIRST = ("Brake chopper",)                   # packed before the larger groups that share their corner
for (key, bottom), refs in sorted(groups.items(), key=lambda kv: (kv[0][0] not in FIRST, -sum(area(r) for r in kv[1]))):
    left[bottom] += pack(refs, ZONES[key][1 if bottom else 0], bottom, gap=TIGHT.get(key) if bottom else None)
for bottom in (False, True):
    pack(left[bottom], SPILL, bottom, spill=True)

b.Save(BOARD)
top = sum(1 for f in b.GetFootprints() if not f.IsFlipped())
print(f"placed: {top} top, {len(FP) - top} bottom; outside their zone ({len(spilled)}): {' '.join(sorted(spilled))}")
print(f"no room on the board ({len(overflow)}): {' '.join(sorted(overflow))}")
