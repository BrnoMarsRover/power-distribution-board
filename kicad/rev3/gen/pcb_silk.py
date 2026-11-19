#!/usr/bin/env python3
"""Silkscreen for people who have never seen the board (run with KiCad's Python, right after pcb_place.py):

    flatpak run --command=python3 org.kicad.KiCad kicad/rev3/gen/pcb_silk.py

- Column headers carry the polarity: "+" at the left end, "-" at the right, matching every connector below.
- Every input is named with its voltage; + / - sit under its pins.
- Control-strip connectors are named; LEDs say what they show; bypass bridges carry a warning.
- Reference designators move to the fab (assembly) layers, so the silkscreen only says what a user needs;
  signal test pads show their net name on the bottom silkscreen, where they are.
Text is at least 1.0 mm high and 0.15 mm thick (JLC minimum).
"""
import pcbnew

BOARD = "kicad/rev3/PowerBoard_rev3.kicad_pcb"
X0, Y0 = 50.0, 50.0
MM = pcbnew.FromMM
b = pcbnew.LoadBoard(BOARD)
FP = {f.GetReference(): f for f in b.GetFootprints()}
L, C, R = pcbnew.GR_TEXT_H_ALIGN_LEFT, pcbnew.GR_TEXT_H_ALIGN_CENTER, pcbnew.GR_TEXT_H_ALIGN_RIGHT


def to_mm(bb):
    return (pcbnew.ToMM(bb.GetLeft()) - X0, pcbnew.ToMM(bb.GetTop()) - Y0,
            pcbnew.ToMM(bb.GetRight()) - X0, pcbnew.ToMM(bb.GetBottom()) - Y0)


def obstacles(bottom):
    """Pads, part bodies (fab outline, else courtyard) and footprint silkscreen on this side;
    through-hole parts block both sides."""
    out = []
    fab, silk = (pcbnew.B_Fab, pcbnew.B_SilkS) if bottom else (pcbnew.F_Fab, pcbnew.F_SilkS)
    for f in b.GetFootprints():
        pads = list(f.Pads())
        tht = sum(p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH for p in pads) * 2 > len(pads)
        if f.IsFlipped() == bottom or tht:
            body = [g for g in f.GraphicalItems() if g.GetLayer() == (pcbnew.B_Fab if f.IsFlipped() else pcbnew.F_Fab)
                    and g.GetClass() == "PCB_SHAPE"]
            if body:
                bx = [to_mm(g.GetBoundingBox()) for g in body]
                out.append((min(v[0] for v in bx), min(v[1] for v in bx), max(v[2] for v in bx), max(v[3] for v in bx)))
            else:
                f.BuildCourtyardCaches()
                poly = f.GetCourtyard(pcbnew.B_CrtYd if f.IsFlipped() else pcbnew.F_CrtYd)
                if poly.OutlineCount():
                    out.append(to_mm(poly.BBox()))
            out += [to_mm(g.GetBoundingBox()) for g in f.GraphicalItems() if g.GetLayer() == silk]
        out += [to_mm(p.GetBoundingBox()) for p in pads if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH
                or p.IsOnLayer(pcbnew.B_Cu if bottom else pcbnew.F_Cu)]
    return out


OBST = {False: obstacles(False), True: obstacles(True)}
TEXTS = {False: [], True: []}
missed = []


def clear(bx, bottom, margin=0.25, near=None):
    if bx[0] < 0.6 or bx[1] < 0.6 or bx[2] > 149.4 or bx[3] > 149.4:
        return False
    hit = lambda o: not (bx[2] + margin <= o[0] or o[2] + margin <= bx[0] or bx[3] + margin <= o[1] or o[3] + margin <= bx[1])
    return not any(hit(o) for o in (near if near is not None else OBST[bottom] + TEXTS[bottom]))


def settle(item, bottom, reach=8.0):
    """Nudge a text item to the nearest free spot (0.25 mm steps, nearest first) and reserve it."""
    base = item.GetPosition()
    b0 = to_mm(item.GetBoundingBox())
    win = (b0[0] - reach - 1, b0[1] - reach - 1, b0[2] + reach + 1, b0[3] + reach + 1)
    near = [o for o in OBST[bottom] + TEXTS[bottom]
            if not (o[2] < win[0] or o[0] > win[2] or o[3] < win[1] or o[1] > win[3])]
    steps = sorted(((dx * 0.25, dy * 0.25) for dx in range(-int(reach * 4), int(reach * 4) + 1)
                    for dy in range(-int(reach * 4), int(reach * 4) + 1)), key=lambda d: d[0] ** 2 + d[1] ** 2)
    for dx, dy in steps:
        item.SetPosition(base + pcbnew.VECTOR2I(MM(dx), MM(dy)))
        bx = to_mm(item.GetBoundingBox())
        if clear(bx, bottom, near=near):
            TEXTS[bottom].append(bx)
            return True
    item.SetPosition(base)
    TEXTS[bottom].append(to_mm(item.GetBoundingBox()))
    missed.append(item.GetText() if hasattr(item, "GetText") else "?")
    return False


def anywhere(item, bottom, prefer_y):
    """Free spot for a long text anywhere on the board, nearest to prefer_y (1 mm grid)."""
    bx0 = to_mm(item.GetBoundingBox())
    w, h = bx0[2] - bx0[0], bx0[3] - bx0[1]
    base = item.GetPosition()
    cands = sorted(((x, y) for x in range(1, int(149 - w)) for y in range(1, int(149 - h))),
                   key=lambda c: abs(c[1] - prefer_y) + 0.01 * abs(c[0] + w / 2 - 75))
    allobs = OBST[bottom] + TEXTS[bottom]
    for x, y in cands:
        if clear((x, y, x + w, y + h), bottom, near=allobs):
            item.SetPosition(base + pcbnew.VECTOR2I(MM(x - bx0[0]), MM(y - bx0[1])))
            TEXTS[bottom].append(to_mm(item.GetBoundingBox()))
            return True
    missed.append(item.GetText())
    return False


def text(s, x, y, h=1.2, just=C, bottom=False, angle=0, bold=False, reach=8.0, free=False):
    t = pcbnew.PCB_TEXT(b)
    t.SetText(s)
    t.SetLayer(pcbnew.B_SilkS if bottom else pcbnew.F_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(MM(h), MM(h)))
    t.SetTextThickness(MM(max(0.15, h * (0.2 if bold else 0.13))))
    t.SetHorizJustify(just)
    t.SetVertJustify(pcbnew.GR_TEXT_V_ALIGN_CENTER)
    t.SetTextAngleDegrees(angle)
    if bottom:
        t.SetMirrored(True)
    t.SetPosition(pcbnew.VECTOR2I(MM(X0 + x), MM(Y0 + y)))
    b.Add(t)
    anywhere(t, bottom, y) if free else settle(t, bottom, reach)


def line(x1, y1, x2, y2, w=0.2, bottom=False):
    """Vertical or horizontal silk line, broken wherever it would cross a part."""
    n = int(max(abs(x2 - x1), abs(y2 - y1)) / 0.25)
    pts = [(x1 + (x2 - x1) * k / n, y1 + (y2 - y1) * k / n) for k in range(n + 1)]
    ok = [clear((x - w, y - w, x + w, y + w), bottom, 0.1) for x, y in pts]
    k = 0
    while k < len(pts):
        if not ok[k]:
            k += 1
            continue
        j = k
        while j + 1 < len(pts) and ok[j + 1]:
            j += 1
        if j > k:
            s = pcbnew.PCB_SHAPE(b, pcbnew.SHAPE_T_SEGMENT)
            s.SetStart(pcbnew.VECTOR2I(MM(X0 + pts[k][0]), MM(Y0 + pts[k][1])))
            s.SetEnd(pcbnew.VECTOR2I(MM(X0 + pts[j][0]), MM(Y0 + pts[j][1])))
            s.SetLayer(pcbnew.B_SilkS if bottom else pcbnew.F_SilkS)
            s.SetWidth(MM(w))
            b.Add(s)
        k = j + 1


def bbox(ref):
    bb = FP[ref].GetBoundingBox(False)
    return [pcbnew.ToMM(v) - o for v, o in ((bb.GetLeft(), X0), (bb.GetTop(), Y0), (bb.GetRight(), X0), (bb.GetBottom(), Y0))]


def pins(ref):
    """x of the + pad and the - pad (board mm). XT30 / XT60 / Deans T have + on pad 2."""
    p = {q.GetNumber(): q.GetPosition() for q in FP[ref].Pads()}
    name = FP[ref].GetFPID().GetLibItemName().wx_str().upper()
    plus, minus = ("2", "1") if any(k in name for k in ("XT30", "XT60", "DEANS")) else ("1", "2")
    return pcbnew.ToMM(p[plus].x) - X0, pcbnew.ToMM(p[minus].x) - X0


# ---------------------------------------------------------------- references to the fab layers
tp_labels = []
for f in b.GetFootprints():
    ref = f.GetField(pcbnew.FIELD_T_REFERENCE)
    ref.SetLayer(pcbnew.B_Fab if f.IsFlipped() else pcbnew.F_Fab)
    if f.GetReference().startswith("TP") and "Loop" not in f.GetFPID().GetLibItemName().wx_str():
        v = f.GetField(pcbnew.FIELD_T_VALUE)            # the net name, beside the pad, on its own side
        v.SetLayer(pcbnew.B_SilkS if f.IsFlipped() else pcbnew.F_SilkS)
        v.SetVisible(True)
        v.SetTextSize(pcbnew.VECTOR2I(MM(1.0), MM(1.0)))
        v.SetTextThickness(MM(0.15))
        tp_labels.append((v, f.IsFlipped()))

# ---------------------------------------------------------------- output columns: header with polarity
OBST = {False: obstacles(False), True: obstacles(True)}
COLS = [("42V DRIVE  40A", 2.0, 24.0), ("42V CONVERTERS", 27.0, 49.0), ("24V OUT 3A EACH", 52.0, 74.0),
        ("12V OUT 3A EACH", 77.0, 101.0), ("5V OUT 3A EACH", 104.0, 122.0)]
for name, x0, x1 in COLS:
    text("+", x0 + 0.8, 73.3, h=1.6, bold=True)
    text("-", x1 - 0.8, 73.3, h=1.6, bold=True)
    text(name, (x0 + x1) / 2, 73.3, h=1.3, bold=True)
for x in (25.5, 50.5, 75.5, 102.5, 123.0):                  # dividers between the columns
    line(x, 72.0, x, 134.0)
bx = bbox("J4")
text("BRAKE RESISTOR", (bx[0] + bx[2]) / 2, bx[1] - 1.4, h=1.0, bold=True)

# ---------------------------------------------------------------- inputs
for ref, label in (("J1", "BATTERY IN 30-42V"), ("J5", "24V IN"), ("J6", "12V IN"), ("J7", "5V IN")):
    p1, p2 = pins(ref)
    x0, y0, x1, y1 = bbox(ref)
    text("+", p1, y1 + 1.0, h=1.4, bold=True)
    text("-", p2, y1 + 1.0, h=1.4, bold=True)
    if ref == "J1":
        text(label, max((x0 + x1) / 2, 21.5), y1 + 3.0, h=1.2, bold=True)
    else:
        text(label, x0 - 1.0, (y0 + y1) / 2, h=1.3, just=R, bold=True)
for ref, label in (("J2", "CHARGER"), ("J3", "PACK CHG")):
    x0, y0, x1, y1 = bbox(ref)
    text(label, x1 + 1.0, (y0 + y1) / 2, h=1.0, just=L)
text("ALL POWER CONNECTORS:  + LEFT   - RIGHT", 100.0, 143.0, h=1.4, bold=True, free=True)

# ---------------------------------------------------------------- 54 V output, control strip
x0, y0, x1, y1 = bbox("J13")
text("54V ZYXEL", x0 - 0.8, (y0 + y1) / 2 - 0.8, h=1.0, just=R, bold=True)
p1, p2 = pins("J13")
text("+", p1, y0 - 1.0, h=1.2, bold=True)
text("-", p2, y0 - 1.0, h=1.2, bold=True)
for ref, label in (("J34", "CAN 1"), ("J35", "CAN 2"), ("J32", "USB"), ("J33", "ORIN UART"),
                   ("J37", "POWER BTN"), ("J38", "BMS SS"), ("J36", "MOTOR SW")):
    x0, y0, x1, y1 = bbox(ref)
    text(label, (x0 + x1) / 2, y0 - 1.0, h=1.0, bold=True)

# ---------------------------------------------------------------- LEDs, bypass bridges
for f in b.GetFootprints():
    lib = f.GetFPID().GetLibItemName().wx_str()
    x0, y0, x1, y1 = bbox(f.GetReference())
    if "LED" in lib and not f.IsFlipped():
        text("ON" if f.GetSheetname()[:2] in ("42", "24", "12", "5 ", "54") else f.GetValue().upper(),
             x1 + 0.6, (y0 + y1) / 2, h=1.0, just=L)

# ---------------------------------------------------------------- title, order number (bottom side)
text("BRNO MARS ROVER  POWER BOARD  ${REV}", 75.0, 4.0, h=2.0, bottom=True, bold=True, free=True)
text("BYPASS = solder a wire bridge only in an emergency: it removes all protection of that rail",
     75.0, 8.0, h=1.0, bottom=True, free=True)
text("JLCJLCJLCJLC", 75.0, 12.0, h=1.0, bottom=True, free=True)

for v, bottom in tp_labels:                                  # test-pad names last, around the rest
    settle(v, bottom, reach=4.0)

b.Save(BOARD)
print(f"silkscreen added; {len(missed)} items found no free spot: {missed[:20]}")
