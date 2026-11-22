"""Hand-designed layout of the battery input and the two 42 V switch blocks (drive, converter feed).

Used by pcb_place.py (placement, before the packer) and pcb_power42.py (copper, after the optimizer).
All coordinates are board mm from the top-left corner (the outline starts at 50, 50 on the page).

Current path, top to bottom, the same in both blocks:
  battery XT90 + -> VBAT area (bulk caps, TVS) -> R1 shunt (upright) -> BUS42 bar (L1 + L3, across both blocks)
  -> block shunt (upright, straddling the bar) -> DRN area (right: FET tabs, bypass bridge)
  -> D2PAK source leads (left) -> OUT strip -> ~15 vias per FET -> L3 OUT plane
  -> OUT column (full width) -> XT90 + pins, which sit straight below the FET leads.
GND returns on the L2 plane and the L4 fill underneath. Each gate escapes through a via at its lead tip and each
shunt's Kelvin pair through its own vias, so the sense and gate signals are routed on the bottom.
"""
import pcbnew

X0, Y0 = 50.0, 50.0
MM, TOMM = pcbnew.FromMM, pcbnew.ToMM

# ---------------------------------------------------------------- geometry
VBAT_Y1 = 28.7                       # VBAT area bottom
BAR = (0.5, 29.7, 50.3, 36.2)        # BUS42 bar
DRN_TOP, DRN_BOT = 36.8, 71.6
OUT_TOP, COL_TOP = 37.0, 72.3        # OUT strip top, start of the full-width OUT column
FET_ROWS = (42.3, 54.2, 66.1)        # pre-charge on top, the two main FETs nearest the outputs
BLOCKS = {                           # dx, FETs (pre, main, main), shunt, bridge, LED, OUT strip right, DRN left, column bottom
    "42D": dict(dx=0.0, fets=("Q4", "Q2", "Q3"), shunt="R5", bridge="JP1", led="D2", out_r=8.2, drn_l=9.0,
                drn_r=25.0, out_l=0.5, col_bot=113.2),
    "42C": dict(dx=25.0, fets=("Q8", "Q6", "Q7"), shunt="R23", bridge="JP2", led="D3", out_r=33.2, drn_l=34.0,
                drn_r=50.3, out_l=25.8, col_bot=100.0,
                kelvin_dx=-0.35),    # Kelvin vias 0.35 mm left, as far as the shunt's pads allow: the INA228's link
}                                    # passes right of them (ina_link)


def kelvin_x(blk):
    """x of a block shunt's Kelvin vias."""
    return SHUNT_X + blk["dx"] + 2.7 + blk.get("kelvin_dx", 0.0)
FET_CX = 12.8                         # FET origin x in the 42D block (leads at -7.65, tab at +1.5)
SHUNT_X, SHUNT_Y = 21.9, 36.2         # block shunt centre (42D); pad 1 (BUS42) up in the bar, pad 4 (DRN) below
BRIDGE_X, BRIDGE_Y = 22.6, 43.2       # bypass bridge pad 1 (DRN); pad 2 (OUT) 7.62 mm below, reaches L3
LED_X, LED_Y = 22.6, 68.0
BRAKE = dict(J4=(14.7, 119.0), D12=(10.2, 126.5), Q22=(15.0, 135.0))
BRK_ZONE = (11.0, 113.8, 25.0, 141.0)
BRK_GND = (0.5, 129.5, 10.4, 141.5)  # brake FET source lead -> GND (top patch + vias)

# regions other parts must stay out of (top, bottom); pcb_place.py and pcb_optimize.py honour them
KEEP_TOP = [(0.5, 12.4, 50.5, 100.0), (0.5, 100.0, 26.0, 141.5)]
KELVIN_P, KELVIN_M = (34.0, 27.75), (35.3, 27.75)   # the input shunt's Kelvin vias: VBAT side, BUS42 side
KEEP_BOTTOM = [(0.5, 36.8, 8.9, 74.6), (25.3, 44.0, 33.9, 74.6),          # OUT via fields under the strips (42C:
                                                                            # its pre-charge RC at the field's top)
               (28.0, 19.5, 44.5, 27.0),                                    # input ceramics C3..C6
               (0.5, 135.0, 4.4, 141.5)]                                    # brake FET source vias


def _escapes():
    """Fixed positions of the escape vias (gates, Kelvin pairs, LEDs, brake gate), for reserving the bottom."""
    pts = [KELVIN_P, KELVIN_M]                                               # input shunt Kelvin pair
    pts += [(45.5, 24.3), (45.5, 25.7)]                                      # TVS ground vias
    for blk in BLOCKS.values():
        dx = blk["dx"]
        pts += [(FET_CX + dx - 10.85, cy - 2.54) for cy in FET_ROWS]         # gates
        pts += [(kelvin_x(blk), 34.0), (kelvin_x(blk), 38.9)]                # Kelvin pair
        pts += [(LED_X + dx + 1.7, LED_Y - 1.02), (LED_X + dx + 1.7, LED_Y + 1.02)]
    qx, qy = BRAKE["Q22"]
    pts.append((qx - 7.65 - 3.2, qy - 2.54))                                 # brake FET gate
    return pts


KEEP_BOTTOM += [(x - 1.0, y - 1.0, x + 1.0, y + 1.0) for x, y in _escapes()]


def _pad(fp, num):
    return next(p for p in fp.Pads() if p.GetNumber() == num)


def _pxy(fp, num):
    v = _pad(fp, num).GetPosition()
    return TOMM(v.x) - X0, TOMM(v.y) - Y0


def _at(fp, x, y, deg, bottom=False):
    if fp.IsFlipped() != bottom:
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    fp.SetOrientationDegrees(deg)
    fp.SetPosition(pcbnew.VECTOR2I(MM(X0 + x), MM(Y0 + y)))


def _upright(fp, x, y, top_pad, bottom_pad, bottom=False):
    """Place so top_pad sits straight above bottom_pad, origin at (x, y)."""
    for deg in (90, 270, 0, 180):
        _at(fp, x, y, deg, bottom)
        (ax, ay), (bx, by) = _pxy(fp, top_pad), _pxy(fp, bottom_pad)
        if ay < by and abs(ax - bx) < 0.05:
            return
    raise RuntimeError(f"{fp.GetReference()}: no upright orientation")


def _flat(fp, x, y, left_pad, right_pad):
    """Place so left_pad sits straight left of right_pad, origin at (x, y)."""
    for deg in (0, 180, 90, 270):
        _at(fp, x, y, deg)
        (ax, ay), (bx, by) = _pxy(fp, left_pad), _pxy(fp, right_pad)
        if ax < bx and abs(ay - by) < 0.05:
            return
    raise RuntimeError(f"{fp.GetReference()}: no flat orientation")


def label_under(bridge):
    """An upright bypass bridge's BYPASS text: level, centred under pad 2, clear of its outline and the FET tab."""
    x = _pxy(bridge, "2")[0]
    outline = max(TOMM(g.GetBoundingBox().GetBottom()) - Y0 for g in bridge.GraphicalItems()
                  if g.GetLayer() == pcbnew.F_SilkS and not isinstance(g, pcbnew.PCB_TEXT))
    for t in bridge.GraphicalItems():
        if isinstance(t, pcbnew.PCB_TEXT) and t.GetText() == "BYPASS":
            t.SetTextAngleDegrees(0)
            t.SetPosition(pcbnew.VECTOR2I(MM(X0 + x), MM(Y0 + outline + 0.3 + TOMM(t.GetTextHeight()) / 2)))


def place(FP):
    """Position and lock every power part of the section. Returns the refs placed."""
    done = []
    # input: bulk caps (+ left), input shunt upright, TVS, input ceramics on the bottom under the TVS
    for ref, x in (("C1", 4.6), ("C2", 18.3)):
        _flat(FP[ref], x, 24.5, "1", "2")       # low enough to leave room for the battery label
        done.append(ref)
    _upright(FP["R1"], 29.9, 29.9, "1", "4")
    _flat(FP["D1"], 39.5, 25.0, "1", "2")
    for k, ref in enumerate(("C3", "C4", "C5", "C6")):
        _upright(FP[ref], 30.0 + 4.0 * k, 24.3, "1", "2", bottom=True)
    done += ["R1", "D1", "C3", "C4", "C5", "C6"]
    # the two switch blocks
    for blk in BLOCKS.values():
        dx = blk["dx"]
        for ref, cy in zip(blk["fets"], FET_ROWS):
            _at(FP[ref], FET_CX + dx, cy, 0)       # leads left (gate above source), tab right
            done.append(ref)
        _upright(FP[blk["shunt"]], SHUNT_X + dx, SHUNT_Y, "1", "4")
        _upright(FP[blk["bridge"]], BRIDGE_X + dx, BRIDGE_Y, "1", "2")
        label_under(FP[blk["bridge"]])
        _upright(FP[blk["led"]], LED_X + dx, LED_Y, "1", "2")
        done += [blk["shunt"], blk["bridge"], blk["led"]]
    # brake: resistor XT60 with + under the drive XT90 + pins, freewheel diode straddling, brake FET below
    _flat(FP["J4"], *BRAKE["J4"], "2", "1")          # pad 2 = OUT42D (+) on the left
    _flat(FP["D12"], *BRAKE["D12"], "1", "2")        # cathode (OUT42D) left, anode (BRK_SW) right
    _at(FP["Q22"], *BRAKE["Q22"], 0)
    done += ["J4", "D12", "Q22"]
    for ref in done:
        FP[ref].SetLocked(True)
    return done


# ---------------------------------------------------------------- copper
F, B, IN1, IN2 = pcbnew.F_Cu, pcbnew.B_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu
OUT_RADIUS = 2.5                      # output strips turn into their connector columns: a wider inside fillet
VIA_D, VIA_DRILL = 1.0, 0.45            # 2 oz outer copper: annular ring >= 0.254 mm (JLC)
ESC_D, ESC_DRILL = 0.9, 0.4             # escape vias for gates, Kelvin pairs and LEDs


def copper(b):
    FP = {f.GetReference(): f for f in b.GetFootprints()}
    net = lambda name: b.FindNet(name)
    pnet = lambda ref, num: _pad(FP[ref], num).GetNetname()
    zones, vias, tracks = [], [], []

    def zone(netname, layer, pts, prio, name, clear=0.5, radius=1.0):
        z = pcbnew.ZONE(b)
        z.SetLayer(layer)
        z.SetNet(net(netname))
        z.SetAssignedPriority(prio)
        z.SetLocalClearance(MM(clear))
        z.SetMinThickness(MM(0.5))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        z.SetZoneName(name)
        if radius:                    # rounded corners: spreads the current at inside turns, finished look outside
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

    def via(netname, x, y, d=VIA_D, drill=VIA_DRILL):
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(pcbnew.VECTOR2I(MM(X0 + x), MM(Y0 + y)))
        v.SetWidth(MM(d))
        v.SetDrill(MM(drill))
        v.SetNet(net(netname))
        b.Add(v)
        vias.append((x, y, netname))

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

    pads = []                                     # every pad box, for keeping vias clear
    for f in b.GetFootprints():
        for p in f.Pads():
            bb = p.GetBoundingBox()
            pads.append((TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0,
                         TOMM(bb.GetBottom()) - Y0, p.GetNetname(), p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH,
                         p.IsOnLayer(B)))

    def clear_of_pads(x, y, netname, gap_other=0.55, gap_same=0.35):
        r = VIA_D / 2
        for l, t, rr, bt, n, pth, on_b in pads:
            g = r + (gap_same if n == netname else gap_other)
            if l - g < x < rr + g and t - g < y < bt + g:
                return False
        return all((x - vx) ** 2 + (y - vy) ** 2 > 1.5 ** 2 for vx, vy, _ in vias)

    # -------------------------------------------- input: VBAT area, BUS42 bar, charger links on L3
    zone("VBAT", F, [(0.5, 0.5), (30.9, 0.5), (30.9, 11.0), (50.3, 11.0), (50.3, VBAT_Y1), (0.5, VBAT_Y1)], 40, "VBAT")
    zone("BUS42", F, rect(*BAR), 40, "BUS42 bar")
    zone("BUS42", IN2, [(0.5, BAR[1]), (31.0, BAR[1]), (31.0, 0.5), (40.5, 0.5), (40.5, BAR[1]), (BAR[2], BAR[1]),
                        (BAR[2], BAR[3]), (0.5, BAR[3])], 40, "BUS42 bar + charger (L3)", clear=0.3)
    chg_n = pnet("J2", "1")
    zone(chg_n, IN2, rect(41.5, 0.5, 49.5, 21.0), 40, "CHG_N (L3)", clear=0.3)
    # TVS ground pad: two vias beside it, joined by a short wide track
    gx, gy = _pxy(FP["D1"], "2")
    for vy in (gy - 0.7, gy + 0.7):
        via("GND", gx + 2.6, vy)
    track("GND", [(gx, gy), (gx + 2.6, gy)], 1.6)
    # input ceramics on the bottom: each + pad gets two vias up to VBAT
    for ref in ("C3", "C4", "C5", "C6"):
        px, py = _pxy(FP[ref], "1")
        for vx in (px - 0.7, px + 0.7):
            via("VBAT", vx, py - 2.2)
        track("VBAT", [(px, py), (px, py - 2.2)], 1.4, B)
    # input shunt Kelvin pair -> vias for the INA228 on the bottom
    k2, k3 = _pxy(FP["R1"], "2"), _pxy(FP["R1"], "3")
    via(pnet("R1", "2"), KELVIN_P[0], KELVIN_P[1], ESC_D, ESC_DRILL)
    track(pnet("R1", "2"), [k2, (KELVIN_P[0], k2[1]), KELVIN_P], 0.25)
    # both sense vias go up, side by side, not one up and one down: the Kelvin lines then leave above the 42C
    # block (y < 29), whose IC's left side holds its bootstrap caps and gate drive
    via(pnet("R1", "3"), KELVIN_M[0], KELVIN_M[1], ESC_D, ESC_DRILL)
    track(pnet("R1", "3"), [k3, (KELVIN_M[0], k3[1]), KELVIN_M], 0.25)

    # -------------------------------------------- the two switch blocks
    for key, blk in BLOCKS.items():
        dx = blk["dx"]
        out = pnet(blk["fets"][1], "3")
        drn = pnet(blk["fets"][1], "2")
        ol, orr, dl, dr = blk["out_l"], blk["out_r"], blk["drn_l"], blk["drn_r"]
        col_r = dr
        if key == "42D":
            out_poly = [(ol, OUT_TOP), (orr, OUT_TOP), (orr, COL_TOP), (col_r, COL_TOP), (col_r, blk["col_bot"]),
                        (9.3, blk["col_bot"]), (9.3, 129.0), (ol, 129.0)]
        else:
            out_poly = [(ol, OUT_TOP), (orr, OUT_TOP), (orr, COL_TOP), (col_r, COL_TOP), (col_r, blk["col_bot"]),
                        (ol, blk["col_bot"])]
        zone(out, F, out_poly, 40, f"{key} OUT", radius=OUT_RADIUS)
        l3 = [(ol, OUT_TOP), (col_r, OUT_TOP)] + out_poly[4:]
        zone(out, IN2, l3, 40, f"{key} OUT (L3)", clear=0.3, radius=OUT_RADIUS)
        zone(drn, F, rect(dl, DRN_TOP, dr, DRN_BOT), 40, f"{key} DRN")
        # FETs: gate escape via at the lead tip
        for ref, cy in zip(blk["fets"], FET_ROWS):
            cx = FET_CX + dx
            gate = pnet(ref, "1")
            gx, gy = _pxy(FP[ref], "1")
            via(gate, cx - 10.85, gy, ESC_D, ESC_DRILL)
            track(gate, [(cx - 9.95 + 0.3, gy), (cx - 10.85, gy)], 0.6)
        # block shunt Kelvin pair -> vias; on the bottom they run to the TPS48111 CS+ / CS-
        sh = FP[blk["shunt"]]
        (k2x, k2y), (k3x, k3y) = _pxy(sh, "2"), _pxy(sh, "3")
        vx = kelvin_x(blk)
        via(pnet(blk["shunt"], "2"), vx, 34.0, ESC_D, ESC_DRILL)
        track(pnet(blk["shunt"], "2"), [(k2x, k2y), (vx, k2y), (vx, 34.0)], 0.25)
        via(pnet(blk["shunt"], "3"), vx, 38.9, ESC_D, ESC_DRILL)
        track(pnet(blk["shunt"], "3"), [(k3x, k3y), (vx, k3y), (vx, 38.9)], 0.25)
        # status LED inside the DRN area: both pads escape to the bottom
        led = FP[blk["led"]]
        for num in ("1", "2"):
            lx, ly = _pxy(led, num)
            via(pnet(blk["led"], num), lx + 1.7, ly, ESC_D, ESC_DRILL)
            track(pnet(blk["led"], num), [(lx, ly), (lx + 1.7, ly)], 0.4)

    # -------------------------------------------- brake: switch node patch, FET source to GND
    brk = pnet("Q22", "2")
    zone(brk, F, rect(*BRK_ZONE), 40, "BRK_SW")
    zone("GND", F, rect(*BRK_GND), 40, "brake GND")
    sx, sy = _pxy(FP["Q22"], "3")
    for vx in (1.4, 2.9):
        for vy in (sy - 0.8, sy + 0.8, sy + 2.4):
            if clear_of_pads(vx, vy, "GND"):
                via("GND", vx, vy)
    gate = pnet("Q22", "1")
    gx, gy = _pxy(FP["Q22"], "1")
    via(gate, gx - 3.2, gy, ESC_D, ESC_DRILL)
    track(gate, [(gx - 2.0, gy), (gx - 3.2, gy)], 0.6)

    # FET source vias down to the L3 OUT plane: a row between the gate and source leads, three rows below
    for key, blk in BLOCKS.items():
        dx, ol = blk["dx"], blk["out_l"]
        out = pnet(blk["fets"][1], "3")
        for ref, cy in zip(blk["fets"], FET_ROWS):
            cx = FET_CX + dx
            for vx in (cx - 9.2, cx - 7.65, cx - 6.1):
                if clear_of_pads(vx, cy + 0.1, out, gap_same=0.3):
                    via(out, vx, cy + 0.1)
            for vy in (cy + 4.2, cy + 5.9, cy + 7.6):
                if key == "42C" and vy > 73.0:        # 42D's bus legs pass below 42C's field (route_plan.LEGS42)
                    continue
                for vx in (ol + 1.1, ol + 2.9, ol + 4.7, ol + 6.5):
                    if clear_of_pads(vx, vy, out):
                        via(out, vx, vy)
    # bar stitching last, so it steers around every escape via placed above
    # one row at 3.2 mm pitch: the clearance holes must leave solid copper webs in the L2 / L4 GND under the bar,
    # because the whole GND return from the outputs back to the battery crosses this line
    x = 2.0
    while x < 49.6:
        if clear_of_pads(x, 30.9, "BUS42"):
            via("BUS42", x, 30.9)
        x += 3.2

    # -------------------------------------------- planes: GND on L2, GND fill on L4 (whole board)
    whole = rect(0.5, 0.5, 149.5, 149.5)
    zone("GND", IN1, whole, 0, "GND plane", clear=0.3, radius=0)
    zone("GND", B, whole, 0, "GND fill bottom", radius=0)
    return zones, vias, tracks
