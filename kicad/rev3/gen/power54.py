"""Hand-designed layout of the 54 V boost (LM5122) under the converter XT90s, next to the Zyxel output.

Used by pcb_place.py (placement, before the packer) and pcb_power42.py (copper, after the optimizer).
Board mm from the top-left corner, like power42.py. All power parts are on the bottom; the two 100 uF output
electrolytics, the fuse and the Zyxel header are on top.

Current path (bottom, top-view coordinates):
  OUT42C via row under the converter XT90s -> input ceramics, R121 (upright, right above L1's input pad)
  -> LIN -> L1 (switch-node pad on the left) -> SW: Q23's tab (low side) and Q24's source lead, both under L1
  -> Q24 (high side) tab = OUT54. Q23's source and Q24's tab both face right, where the output ceramics stand
  in two rows across GND | OUT54 | GND stripes: the hot loop (Q24 -> ceramics -> GND -> Q23 -> SW) is about
  12 x 14 mm.
The electrolytics and the fuse join OUT54 through an L3 pour, so their legs and vias stay out of the hot loop.
The controller U10 sits left of the FET stack (under the brake corner): HO, SW and BST are a few mm long, LO
reaches Q23's gate through a via. GND: the L2 plane, stitched under every GND pour.
"""
import math

import pcbnew

from power42 import MM, TOMM, X0, Y0, _at, _flat, _pad, _pxy, _upright

# ---------------------------------------------------------------- geometry
XL, XR = 24.6, 50.0                   # power-stage columns (the 42D XT90 and the brake parts end at 24.6)
R121_X, R121_Y = 38.35, 104.6          # sense resistor, upright: OUT42C pad up, LIN pad down onto L1's input pad
CIN_X, CIN_Y = (26.5, 30.4, 34.3, 42.58), 103.2      # input ceramics, OUT42C pad up
L1_X, L1_Y = 32.6, 117.0               # inductor, switch-node pad left, LIN pad right
GAP = 0.6
ROW2_Y = 143.6                         # second ceramic row: OUT54 pad up, GND pad down
COL_X = (38.25, 42.15, 46.05)          # first-row ceramics right of the FETs
ROW2_LEFT_X = (26.5, 30.4, 34.3)       # second-row ceramics under Q24
D14_X = 38.89                          # output TVS in the second row
TP_GND_XY = (32.5, 126.5)             # the GND test loop, beside Q23's tab
BULK = ((43.46, 111.6, True), (43.46, 126.3, False))   # 100 uF caps on top: body centre, legs vertical?
                                                       # (the flat one 0.8 mm south of first drawn, 2026-11-24:
                                                       #  its leg's pad touched L1's bottom-side outline)
U10_XY = (16.9, 142.85)                # controller left of Q24's leads, its pad vias clear of the brake FET's tab;
                                       # 3.2 mm west of the first spot (2026-11-22): its gate-drive side needs a
                                       # corridor for the analog escapes, LO, VCC, SW and HO up to Q24's leads
# gate-drive helpers (boost_plan.py, 2026-11-22), from U10's right pad edge E and its pins:
#   C78 (BST cap) upright right of pins 18-20, centred on HO's pin so HO leaves between its pads;
#   C79 (VCC cap) flat north of U10, VCC pad right (VCC comes up the corridor), GND pad left;
#   D13 (bootstrap diode) on the top side under pin 20, cathode right (BST via under C78), anode left (VCC via)
CB_DX = 3.02                           # C78's centre, right of E (its courtyard clear of C87)
CV_XY = (1.3, -4.27)                   # C79's centre: dx from E, dy from pin 11 (SW, 0.5 mm, passes right of it)
DB_DXY = (0.45, 2.425)                 # D13's centre on TOP: dx from E, dy from pin 20 (no room on the bottom
                                       # side under U10 with the 0.5 mm courtyard gap); BST and VCC reach it by vias
VIA_D, VIA_DRILL = 1.0, 0.45
ESC_D, ESC_DRILL = 0.9, 0.4
LEG_KEEP = 1.0                         # around a through-hole leg on the bottom: room for the iron

KEEP_BOTTOM = [(XL, 92.0, XR, 104.0),                 # OUT42C feed and input ceramics
               (XL, 104.0, 44.9, 109.2),               # input GND strips, R121, top of LIN
               (XL, 109.2, 41.2, 149.4),               # L1, LIN, SW, FET stack, left ceramic row
               (41.2, 120.5, XR, 149.4)]               # GND / OUT54 stripes, ceramics, J13's legs
KEEP_BOTTOM += [(U10_XY[0] + 3.45, 133.3, XL, 149.4)]   # U10's gate-drive corridor up to Q24's leads (boost_plan)
KEEP_TOP = [(34.0, 99.0, 36.9, 121.9), (25.2, 119.5, 36.9, 121.9),   # the controller's sense pair and feed on top
            (25.2, 119.5, 27.6, 147.0), (14.5, 144.6, 27.6, 147.0),   # (boost_plan.KEEP_TOP)
            (28.3, 122.5, 31.6, 124.1),                                 # the SW test pad and its via
            (XL, 97.8, 45.0, 113.2),                   # OUT42C via row, input GND vias: no top parts on them
            (U10_XY[0] - 1.6, U10_XY[1] - 3.0, U10_XY[0] + 1.6, U10_XY[1] + 3.0)]   # U10's pad vias


def _find(FP, sheet, lib, net):
    return sorted((r for r, f in FP.items() if f.GetSheetname().startswith(sheet) and lib in f.GetFPIDAsString()
                   and any(p.GetNetname() == net for p in f.Pads())), key=lambda r: int(r[1:]))


def caps(FP, net):
    """1210 ceramics of the boost on a net (OUT42C: input, OUT54: output)."""
    return _find(FP, "54 V boost", "1210", net)


def bulk(FP):
    return sorted((r for r, f in FP.items() if "CP_Radial" in f.GetFPIDAsString()
                   and any(p.GetNetname() == "OUT54" for p in f.Pads())), key=lambda r: int(r[1:]))


def _box(fp):
    fp.BuildCourtyardCaches()
    poly = fp.GetCourtyard(pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd)
    bb = poly.BBox()
    return TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0, TOMM(bb.GetBottom()) - Y0


def _pbox(fp, num):
    bb = _pad(fp, num).GetBoundingBox()
    return TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0, TOMM(bb.GetBottom()) - Y0


def _shift(fp, dx, dy):
    fp.SetPosition(fp.GetPosition() + pcbnew.VECTOR2I(MM(dx), MM(dy)))


def _dpak(fp, leads_right, source_low, left, top):
    """DPAK on the bottom, axis horizontal: leads right or left of the tab, source lead below or above the gate;
    courtyard top-left at (left, top)."""
    for deg in (0, 90, 180, 270):
        _at(fp, 0.0, 0.0, deg, bottom=True)
        tx, ty = _pxy(fp, "2")
        (gx, gy), (sx, sy) = _pxy(fp, "1"), _pxy(fp, "3")
        if abs(gy - sy) > 1.0 and (gx > tx) == leads_right and (sy > gy) == source_low:
            l, t, _, _ = _box(fp)
            _shift(fp, left - l, top - t)
            return
    raise RuntimeError(f"{fp.GetReference()}: no orientation")


def _centre_pads(fp, x, y):
    pts = [_pxy(fp, p.GetNumber()) for p in fp.Pads() if p.GetNumber()]
    cx, cy = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
    fp.SetPosition(fp.GetPosition() + pcbnew.VECTOR2I(MM(x - cx), MM(y - cy)))


def place(FP):
    """Position and lock the boost's power parts. Returns the refs placed."""
    done = []
    _upright(FP["R121"], R121_X, R121_Y, "1", "2", bottom=True)
    for ref, x in zip(caps(FP, "OUT42C"), CIN_X):
        _upright(FP[ref], x, CIN_Y, "1", "2", bottom=True)
        done.append(ref)
    _flat_bottom(FP["L1"], L1_X, L1_Y, "2", "1")
    top = _box(FP["L1"])[3] + GAP
    _dpak(FP["Q23"], leads_right=True, source_low=True, left=XL, top=top)      # low side: tab = SW, under L1
    top = _box(FP["Q23"])[3] + GAP
    _dpak(FP["Q24"], leads_right=False, source_low=False, left=XL, top=top)    # high side: source lead = SW
    row1 = _pxy(FP["Q23"], "3")[1]                                              # GND | OUT54 boundary
    out = caps(FP, "OUT54")
    for ref, x in zip(out[:3], COL_X):                                          # row 1: GND pad up
        _upright(FP[ref], x, row1, "2", "1", bottom=True)
    for ref, x in zip(out[3:], ROW2_LEFT_X):                                    # row 2: OUT54 pad up
        _upright(FP[ref], x, ROW2_Y, "1", "2", bottom=True)
    _upright(FP["D14"], D14_X, ROW2_Y, "1", "2", bottom=True)                  # OUT54 (cathode) up
    for ref, (bx, by, vertical) in zip(bulk(FP), BULK):                         # on top; legs reach OUT54 on L3
        if vertical:
            _upright(FP[ref], bx, by - 2.5, "1", "2")
        else:
            _flat(FP[ref], bx - 2.5, by, "1", "2")
    u = FP["U10"]
    for deg in (0, 90, 180, 270):                                               # pins 11-20 (gate drive) right
        _at(u, *U10_XY, deg, bottom=True)
        if _pxy(u, "18")[0] > _pxy(u, "3")[0] + 3:
            break
    # gate-drive helpers next to pins 16-20 (boost_plan.py): found by net, so renumbering cannot swap them
    sw, bst, vcc = (_pad(u, n).GetNetname() for n in ("18", "20", "17"))
    nets = lambda r: {p.GetNetname() for p in FP[r].Pads()}
    pnum = lambda r, n: next(p.GetNumber() for p in FP[r].Pads() if p.GetNetname() == n)
    cb = next(r for r in FP if r.startswith("C") and nets(r) == {sw, bst})
    cv = next(r for r in FP if r.startswith("C") and nets(r) == {vcc, "GND"})
    db = next(r for r in FP if r.startswith("D") and {vcc, bst} <= nets(r))
    e = _pbox(u, "19")[2]                                                       # right pad edge
    _upright(FP[cb], e + CB_DX, _pxy(u, "19")[1], pnum(cb, sw), pnum(cb, bst), bottom=True)
    _flat_bottom(FP[cv], e + CV_XY[0], _pxy(u, "11")[1] + CV_XY[1], pnum(cv, "GND"), pnum(cv, vcc))
    for deg in (0, 90, 180, 270):                                               # top: cathode right, anode upper left
        _at(FP[db], e + DB_DXY[0], _pxy(u, "20")[1] + DB_DXY[1], deg, bottom=False)
        (kx, ky), (ax, ay) = _pxy(FP[db], pnum(db, bst)), _pxy(FP[db], pnum(db, vcc))
        if kx > ax + 1.0 and ay < ky - 0.5:
            break
    done += ["R121", "L1", "Q23", "Q24", *out, "D14", *bulk(FP), "U10", cb, cv, db]
    for ref in done:
        FP[ref].SetLocked(True)
    # the boost's GND test loop (top side): beside Q23's tab, over the bottom GND (2026-11-24; the packer had put it on
    # the switch node, where no GND via fits)
    sheet = FP["U10"].GetSheetname()
    tp = next(r for r, f in FP.items() if r.startswith("TP") and f.GetSheetname() == sheet
              and {p.GetNetname() for p in f.Pads()} == {"GND"})
    _at(FP[tp], *TP_GND_XY, 0)
    _centre_pads(FP[tp], *TP_GND_XY)
    FP[tp].SetLocked(True)
    done.append(tp)
    return done


def _flat_bottom(fp, x, y, left_pad, right_pad):
    for deg in (0, 180, 90, 270):
        _at(fp, x, y, deg, bottom=True)
        (ax, ay), (bx, by) = _pxy(fp, left_pad), _pxy(fp, right_pad)
        if ax < bx and abs(ay - by) < 0.05:
            return
    raise RuntimeError(f"{fp.GetReference()}: no flat orientation")


# ---------------------------------------------------------------- copper
F, B, IN2 = pcbnew.F_Cu, pcbnew.B_Cu, pcbnew.In2_Cu


def copper(b):
    FP = {f.GetReference(): f for f in b.GetFootprints()}
    net = lambda name: b.FindNet(name)
    zones, vias, tracks = [], [], []

    def zone(netname, layer, pts, name, clear=0.4, radius=0.8, prio=40):
        z = pcbnew.ZONE(b)
        z.SetLayer(layer)
        z.SetNet(net(netname))
        z.SetAssignedPriority(prio)
        z.SetLocalClearance(MM(clear))
        z.SetMinThickness(MM(0.4))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        z.SetZoneName(name)
        if radius:
            z.SetCornerSmoothingType(pcbnew.ZONE_SETTINGS.CORNER_SMOOTHING_FILLET)
            z.SetCornerRadius(MM(radius))
        o = z.Outline()
        o.NewOutline()
        for x, y in pts:
            o.Append(MM(X0 + x), MM(Y0 + y))
        b.Add(z)
        zones.append(z)

    def rect(x0, y0, x1, y1):
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

    pads = []
    for f in b.GetFootprints():
        for p in f.Pads():
            bb = p.GetBoundingBox()
            pads.append((TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0,
                         TOMM(bb.GetBottom()) - Y0, p.GetNetname(), p.IsOnLayer(B), p.IsOnLayer(F)))
    for v in b.GetTracks():                                  # vias placed by power42 / power_rails
        if v.GetClass() == "PCB_VIA":
            pads.append((TOMM(v.GetX()) - X0 - 0.5, TOMM(v.GetY()) - Y0 - 0.5, TOMM(v.GetX()) - X0 + 0.5,
                         TOMM(v.GetY()) - Y0 + 0.5, v.GetNetname(), True, True))

    def free(x, y, netname, d=VIA_D, gap=0.4):
        rr = d / 2 + gap
        for l, t, r_, bt, n, on_b, on_f in pads:
            if n != netname and l - rr < x < r_ + rr and t - rr < y < bt + rr:
                return False
        return all((x - vx) ** 2 + (y - vy) ** 2 > (d + 0.5) ** 2 for vx, vy, _ in vias)

    def via(netname, x, y, d=VIA_D, drill=VIA_DRILL, force=False):
        if not force and not free(x, y, netname, d):
            return False
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(pcbnew.VECTOR2I(MM(X0 + x), MM(Y0 + y)))
        v.SetWidth(MM(d))
        v.SetDrill(MM(drill))
        v.SetNet(net(netname))
        b.Add(v)
        vias.append((x, y, netname))
        return True

    def track(netname, pts, w, layer=F):
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            t = pcbnew.PCB_TRACK(b)
            t.SetStart(pcbnew.VECTOR2I(MM(X0 + x1), MM(Y0 + y1)))
            t.SetEnd(pcbnew.VECTOR2I(MM(X0 + x2), MM(Y0 + y2)))
            t.SetWidth(MM(w))
            t.SetLayer(layer)
            t.SetNet(net(netname))
            b.Add(t)
            tracks.append(t)

    def pbox(ref, num):
        bb = _pad(FP[ref], num).GetBoundingBox()
        return TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0, TOMM(bb.GetBottom()) - Y0

    lin = _pad(FP["L1"], "1").GetNetname()
    sw = _pad(FP["L1"], "2").GetNetname()
    cin, cout = caps(FP, "OUT42C"), caps(FP, "OUT54")
    l1_sw, l1_lin = pbox("L1", "2"), pbox("L1", "1")
    r_lin = pbox("R121", "2")
    q23_tab, q23_s, q23_g = pbox("Q23", "2"), pbox("Q23", "3"), pbox("Q23", "1")
    q24_tab, q24_s, q24_g = pbox("Q24", "2"), pbox("Q24", "3"), pbox("Q24", "1")
    cin_gnd_top = min(pbox(r, "2")[1] for r in cin)
    row1 = (q23_s[1] + q23_s[3]) / 2
    row1_gnd_bot = max(pbox(r, "2")[3] for r in cout[:3])
    row1_out_top = min(pbox(r, "1")[1] for r in cout[:3])
    row2_out_bot = max(pbox(r, "1")[3] for r in cout[3:])
    row2_gnd_top = min(pbox(r, "2")[1] for r in cout[3:])

    # input: OUT42C from the converter block (via row under its XT90s) to the input ceramics and R121
    oin_bot = cin_gnd_top - 0.5
    zone("OUT42C", B, rect(25.8, 92.0, 44.6, oin_bot), "54 boost OUT42C")
    x = 27.0
    while x < 44.6:
        via("OUT42C", x, 99.2)
        x += 2.5
    # LIN: R121's lower pad straight down onto L1's input pad
    lin_box = (min(r_lin[0], l1_lin[0]) - 0.45, r_lin[1] - 0.3, max(r_lin[2], l1_lin[2]) + 0.45, l1_lin[3] + 0.5)
    zone(lin, B, rect(*lin_box), "54 boost LIN")
    # input ceramics' GND: a strip left of LIN (over L1's body, short of its pads) and one under the fourth cap
    gnd_in = (XL, oin_bot + 0.5, lin_box[0] - 0.5, l1_sw[1] - 1.0)
    zone("GND", B, rect(*gnd_in), "54 boost GND in")
    c4 = pbox(cin[-1], "2")
    zone("GND", B, rect(lin_box[2] + 0.5, oin_bot + 0.5, 44.6, c4[3] + 2.8), "54 boost GND in 2")
    for gx in (26.0, 28.6, 31.2, 33.8):
        for gy in (gnd_in[1] + 2.4, gnd_in[1] + 5.2, gnd_in[1] + 8.0):
            if gy < l1_sw[1] - 1.2:
                via("GND", gx, gy)
    via("GND", (c4[0] + c4[2]) / 2, c4[3] + 1.6)
    # SW: L1's switch pad, Q23's tab and Q24's source lead, nothing else
    swr = q23_tab[2] + 0.3
    sw_pts = [(XL, l1_sw[1] - 0.4), (max(l1_sw[2], swr), l1_sw[1] - 0.4), (max(l1_sw[2], swr), q23_tab[3] + 0.3),
              (q24_tab[0] - 1.0, q23_tab[3] + 0.3), (q24_tab[0] - 1.0, q24_s[3] + 0.4), (XL, q24_s[3] + 0.4)]
    zone(sw, B, sw_pts, "54 boost SW", radius=0.5)
    # GND A: right of Q23's tab down to the first ceramic row (their GND pads, Q23's source lead)
    ga = (swr + 0.5, lin_box[3] + 0.5, XR, (row1_gnd_bot + row1_out_top) / 2 - 0.25)
    zone("GND", B, rect(*ga), "54 boost GND A")
    # OUT54: Q24's tab and the ceramics' OUT54 pads, between the rows; under Q24 it reaches left to XL
    ob = (row2_out_bot + row2_gnd_top) / 2 - 0.25
    out_pts = [(q24_tab[0] - 0.5, ga[3] + 0.5), (XR, ga[3] + 0.5), (XR, ob), (XL, ob), (XL, q24_g[3] + 0.5),
               (q24_tab[0] - 0.5, q24_g[3] + 0.5)]
    zone("OUT54", B, out_pts, "54 boost OUT54", radius=0.5)
    # GND B: below the second row, to the board edge
    gb = (XL, ob + 0.5, XR, 149.6)
    zone("GND", B, rect(*gb), "54 boost GND B")
    # L3: OUT54 for the electrolytics and the fuse, kept out from under the switch node
    zone("OUT54", IN2, [(lin_box[2] + 0.5, oin_bot + 2.5), (XR, oin_bot + 2.5), (XR, 149.5), (XL, 149.5),
                        (XL, q24_g[3] + 0.5), (swr + 0.5, q24_g[3] + 0.5), (swr + 0.5, ga[1]),
                        (lin_box[2] + 0.5, ga[1])], "54 boost OUT54 (L3)", clear=0.3)

    # Q23's gate escapes to L3 for the LO run to U10 (left of the stack); HO, SW and BST are routed on the bottom
    gx = q23_g[2] + 0.75
    via(_pad(FP["Q23"], "1").GetNetname(), gx, (q23_g[1] + q23_g[3]) / 2, ESC_D, ESC_DRILL, force=True)
    track(_pad(FP["Q23"], "1").GetNetname(), [((q23_g[0] + q23_g[2]) / 2, (q23_g[1] + q23_g[3]) / 2),
                                               (gx, (q23_g[1] + q23_g[3]) / 2)], 0.6, B)
    # U10's exposed pad: GND vias, the middle one 1.2 mm so the pad can be soldered from the top
    ex = pbox("U10", "21")
    cx, cy = (ex[0] + ex[2]) / 2, (ex[1] + ex[3]) / 2
    long_y = (ex[3] - ex[1]) > (ex[2] - ex[0])
    for k in (-1.5, 1.5):                       # (+-1.5: the top one keeps 0.5 mm from the brake FET's tab, top side)
        via("GND", cx if long_y else cx + k, cy + k if long_y else cy, ESC_D, ESC_DRILL, force=True)
    via("GND", cx, cy, 1.6, 1.0, force=True)
    # the fuse's OUT54 pad (top) reaches L3 through two vias just past it
    fz = pbox("F19", "1")
    for vx in (fz[0] + 1.0, fz[2] - 1.0):
        via("OUT54", vx, fz[3] + 1.0, force=True)
        track("OUT54", [(vx, (fz[1] + fz[3]) / 2), (vx, fz[3] + 1.0)], 1.2, F)
    # L3 under the switch node: a GND shield (the +3V3 fill would otherwise sit 0.2 mm under 54 V edges);
    # it reaches up over the input GND strip's last via row, which ties it to GND
    zone("GND", IN2, rect(XL - 0.5, gnd_in[3] - 1.6, swr + 0.5, q24_s[3] + 1.0), "54 boost SW shield (L3)",
         clear=0.3, prio=41)
    # stitching: GND A / GND B to the L2 plane, OUT54 bottom to L3
    holes = [(TOMM(q.GetPosition().x) - X0, TOMM(q.GetPosition().y) - Y0, TOMM(q.GetDrillSize().x) / 2)
             for f in FP.values() for q in f.Pads() if q.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]
    for gx in (36.8, 39.4, 46.6, 49.2):
        for gy in (ga[1] + 1.2, ga[1] + 3.6, ga[1] + 6.0):
            if all(math.hypot(gx - hx, gy - hy) > hr + 0.2 + 0.25 + 0.45 for hx, hy, hr in holes):   # (hole to hole)
                via("GND", gx, gy)
    via("GND", q23_s[2] + 0.9, (q23_s[1] + q23_s[3]) / 2)
    for gx in range(26, 50, 3):
        via("GND", gx + 0.5, 148.3)
    for ox in (36.9, 39.3, 46.7, 49.1):
        for oy in ((ga[3] + ob) / 2 - 1.2, (ga[3] + ob) / 2 + 1.2):
            via("OUT54", ox, oy)
    via("OUT54", q24_tab[2] + 0.9, (q24_tab[1] + q24_tab[3]) / 2)
    via("OUT54", q24_tab[2] + 0.9, q24_tab[3] - 0.8)

    return zones, vias, tracks
