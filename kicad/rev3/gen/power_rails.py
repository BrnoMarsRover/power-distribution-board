"""Hand-designed layout of the 24 / 12 / 5 V rail blocks: input, eFuse switch and output column.

Used by pcb_place.py (placement, before the packer) and pcb_power42.py (copper, after the optimizer).
Board mm from the top-left corner, like power42.py. Only the rails listed in RAILS are laid out here; the
others still go through pcb_place.py's generic column code.

Current path, top to bottom (the 42 V blocks' pattern, one block per column):
  input connector + -> VIN area (bulk caps, TVS; the TVS ground pad shares a patch with the connector's - pin)
  -> shunt (upright, straddling VIN / DRN) -> DRN area (FET tabs, LED, bypass bridge pad 1)
  -> D2PAK source leads (left) -> OUT strip -> full-width OUT area (bridge pad 2 lands here, on L1)
  -> OUT strip down the left of the output column, running under the PTCs: each PTC lies flat with its rail
     pad inside the strip and its other pad opening straight into the connector's + pin on the same row.
L3 carries a copy of the OUT area under the switch, stitched to L1. GND returns on the L2 plane and the L4
fill, through the connectors' plated - pins. Gates, Kelvin pairs and the LED escape to the bottom through vias.

5 V is L-shaped: its XT30 column is narrower than the switch needs, so the switch is full width only beside the
JST block (y < band_bot) and narrows to the column (col_r) below, where the 54 V eFuse sits. Its TVS stands
beside the input connector, and its bypass bridge stands under the main FET with pad 1 in a short DRN tongue.
"""
import pcbnew

from power42 import MM, TOMM, X0, Y0, _at, _flat, _pad, _pxy, _upright, label_under

# ---------------------------------------------------------------- geometry shared by every rail block
Y_IN = 1.0                          # input connector courtyard top
Y_SHUNT = 22.0                      # shunt centre: VIN above, DRN below
VIN_BOT, DRN_TOP, OUT_TOP = Y_SHUNT - 0.3, Y_SHUNT + 0.3, Y_SHUNT + 0.5
FET_ROWS = (Y_SHUNT + 6.1, Y_SHUNT + 18.0)        # pre-charge, main (main nearest the outputs)
DRN_BOT, WIDE_TOP, WIDE_BOT = 45.5, 46.3, 73.8    # DRN area bottom; full-width OUT area under the switch
Y_OUT, ROW = 75.0, 10.2             # output connector courtyard tops (same rows as pcb_place.py)
LED_Y = 30.0
TVS_Y, CAP_Y = 14.5, 16.0           # low enough to leave the input's + / - marks their spot
BRIDGE_Y = 42.6                     # bridge pad 1 (DRN); pad 2 sits 5.08 mm lower, in the OUT area
TVS_BESIDE_Y = 6.5                  # tvs_beside: upright TVS right of the input, ground pad up in the patch
CONN_GAP = 0.8                      # PTC courtyard to connector courtyard (a rail may set its own conn_gap)

# per rail: column copper edges (xl, xr) and the parts; fets = (pre-charge, main). The bulk caps are found by net.
RAILS = {
    "24": dict(xl=50.8, xr=76.0, vin="VIN24", out="OUT24", j_in="J5", tvs="D5",
               shunt="R41", fets=("Q11", "Q10"), bridge="JP3", led="D4"),
    "12": dict(xl=76.5, xr=103.2, vin="VIN12", out="OUT12", j_in="J6", tvs="D7",
               shunt="R59", fets=("Q14", "Q13"), bridge="JP4", led="D6"),
    # L-shaped: full switch width (to xr, J38 starts at 131.6) only above band_bot; DRN, OUT and outputs end at col_r
    "5": dict(xl=104.1, xr=130.9, col_r=125.6, band_bot=34.6, vin="VIN5", out="OUT5", j_in="J7", tvs="D9",
              shunt="R77", fets=("Q17", "Q16"), bridge="JP5", led="D8",
              tvs_beside=True, bridge_below=True),
}


def _geo(r):
    """Derived x positions of one block."""
    xl, xr = r["xl"], r["xr"]
    col_r = r.get("col_r", xr)
    bx = col_r - 2.2                                # bridge_below: upright under the main FET, right edge
    by = FET_ROWS[1] + 5.7 + 0.5 + 2.14             # its courtyard clears the FET's
    return dict(
        col_r=col_r, band_bot=r.get("band_bot", WIDE_BOT),
        bridge=(bx, by) if r.get("bridge_below") else (xr - 3.1, BRIDGE_Y),
        tongue=(bx - 2.2, by + 2.1) if r.get("bridge_below") else None,   # DRN tongue: left edge, bottom
        out_r=xl + 7.7, drn_l=xl + 8.5,            # OUT strip / DRN area split at the FET body
        fet_cx=xl + 12.3,                          # D2PAK origin: leads at -7.65, tab centre at +1.5
        shunt_x=xr - 3.8, side_x=xr - 3.1,         # shunt, LED and bridge on the right of the DRN area
        ptc_x=xl + 4.625,                          # flat 2920: rail pad left edge 0.1 mm inside xl
        strip_r=xl + 6.5,                          # OUT strip edge in the output column (under the PTC bodies)
    )


def _box(fp):
    fp.BuildCourtyardCaches()
    poly = fp.GetCourtyard(pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd)
    bb = poly.BBox() if poly.OutlineCount() else fp.GetBoundingBox(False)
    return TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0, TOMM(bb.GetBottom()) - Y0


def _shift(fp, dx, dy):
    fp.SetPosition(fp.GetPosition() + pcbnew.VECTOR2I(MM(dx), MM(dy)))


def _pad_on(fp, netname):
    return next(p for p in fp.Pads() if p.GetNetname() == netname)


def outputs(FP, r):
    """[(connector, PTC, PTC rail pad, PTC output pad, connector + pad, connector - pad)], ordered by reference."""
    rows = []
    for f in FP.values():
        if not f.GetReference().startswith("F") or not any(p.GetNetname() == r["out"] for p in f.Pads()):
            continue
        rail = _pad_on(f, r["out"]).GetNumber()
        other = next(p for p in f.Pads() if p.GetNumber() != rail)
        j = next(c for c in FP.values() if c.GetReference().startswith("J")
                 and any(p.GetNetname() == other.GetNetname() for p in c.Pads()))
        plus = _pad_on(j, other.GetNetname()).GetNumber()
        minus = next(p.GetNumber() for p in j.Pads() if p.GetNumber() not in (plus, ""))
        rows.append((j.GetReference(), f.GetReference(), rail, other.GetNumber(), plus, minus))
    return sorted(rows, key=lambda t: int(t[0][1:]))


def _conn_left(FP, r):
    """Courtyard left of the output connectors: PTC courtyard right + gap."""
    j, f, rail, other, plus, minus = outputs(FP, r)[0]
    ptc = FP[f]
    _flat(ptc, _geo(r)["ptc_x"], 100.0, rail, other)
    return _box(ptc)[2] + r.get("conn_gap", CONN_GAP)


def _hide_marks(fp):
    """XT30 footprints print their own + / -: on these columns they land on the PTC pads and the neighbours.
    The column header and the input's marks carry the polarity, as on the Deans and XT60 columns."""
    for t in fp.GraphicalItems():                  # KiCad cannot hide footprint user text: move it to the fab layer
        if isinstance(t, pcbnew.PCB_TEXT) and t.GetText() in ("+", "-"):
            t.SetLayer(pcbnew.B_Fab if fp.IsFlipped() else pcbnew.F_Fab)


def bulk_caps(FP, r):
    """The rail's input electrolytics, found by footprint and net (capacitor numbers move when a sheet earlier in
    the schematic gains or loses a capacitor)."""
    return sorted((f.GetReference() for f in FP.values() if "CP_Radial" in f.GetFPIDAsString()
                   and any(p.GetNetname() == r["vin"] for p in f.Pads())), key=lambda ref: int(ref[1:]))


def place(FP):
    """Position and lock the power parts of every rail in RAILS. Returns the refs placed."""
    done = []
    for r in RAILS.values():
        r["caps"] = bulk_caps(FP, r)
        g = _geo(r)
        cl = _conn_left(FP, r)
        # output column: connector + pad and PTC on one row, rail pad on the left
        for k, (j, f, rail, other, plus, minus) in enumerate(outputs(FP, r)):
            jf = FP[j]
            _flat(jf, 100.0, 100.0, plus, minus)
            l, t, _, _ = _box(jf)
            _shift(jf, cl - l, Y_OUT + ROW * k - t)
            yc = _pxy(jf, plus)[1]
            _flat(FP[f], g["ptc_x"], yc, rail, other)
            _hide_marks(jf)
            done += [j, f]
        # input connector, aligned with the outputs
        jin = FP[r["j_in"]]
        plus = _pad_on(jin, r["vin"]).GetNumber()
        minus = next(p.GetNumber() for p in jin.Pads() if p.GetNumber() not in (plus, ""))
        _flat(jin, 100.0, 100.0, plus, minus)
        l, t, _, _ = _box(jin)
        _shift(jin, cl - l, Y_IN - t)
        _hide_marks(jin)
        mx, my = _pxy(jin, minus)
        # TVS under the - pin or beside the connector (its ground pad in the patch), bulk caps on the left
        if r.get("tvs_beside"):
            _upright(FP[r["tvs"]], r["xr"] - 2.4, TVS_BESIDE_Y, "2", "1")
        else:
            _flat(FP[r["tvs"]], mx - 1.1, TVS_Y, "1", "2")
        for ref, x in zip(r["caps"], (r["xl"] + 2.35, r["xl"] + 10.15)):
            _flat(FP[ref], x, CAP_Y, "1", "2")
        # switch
        _upright(FP[r["shunt"]], g["shunt_x"], Y_SHUNT, "1", "4")
        for ref, cy in zip(r["fets"], FET_ROWS):
            _at(FP[ref], g["fet_cx"], cy, 0)
        _upright(FP[r["led"]], g["side_x"], LED_Y, "1", "2")
        br = FP[r["bridge"]]
        _upright(br, *g["bridge"], "1", "2")
        label_under(br)
        done += [r["j_in"], r["tvs"], *r["caps"], r["shunt"], *r["fets"], r["led"], r["bridge"]]
    for ref in done:
        FP[ref].SetLocked(True)
    return done


def _escapes(r):
    g = _geo(r)
    pts = [(g["fet_cx"] - 10.85, cy - 2.54) for cy in FET_ROWS]                       # gates
    pts += [(g["shunt_x"] + 2.7, Y_SHUNT - 2.2), (g["shunt_x"] + 2.7, Y_SHUNT + 2.7)]  # Kelvin pair
    pts += [(g["side_x"] + 1.7, LED_Y - 1.02), (g["side_x"] + 1.7, LED_Y + 1.02)]     # LED
    return pts


# regions other parts must stay out of (top, bottom); pcb_place.py and pcb_optimize.py honour them
KEEP_TOP, KEEP_BOTTOM = [], []
for _r in RAILS.values():
    _g = _geo(_r)
    KEEP_TOP += [(_r["xl"], 0.5, _r["xr"], _g["band_bot"]), (_r["xl"], _g["band_bot"], _g["col_r"], WIDE_BOT),
                 (_r["xl"], WIDE_BOT, _g["strip_r"], 136.0)]
    KEEP_BOTTOM += [(_r["xl"], OUT_TOP, _g["out_r"], WIDE_TOP),           # FET source via fields
                    (_r["xl"], WIDE_TOP, _r["xl"] + 3.7, WIDE_BOT)]       # stitching column
    KEEP_BOTTOM += [(x - 1.0, y - 1.0, x + 1.0, y + 1.0) for x, y in _escapes(_r)]


# ---------------------------------------------------------------- copper
F, IN2 = pcbnew.F_Cu, pcbnew.In2_Cu
VIA_D, VIA_DRILL = 1.0, 0.45
ESC_D, ESC_DRILL = 0.9, 0.4
OUT_RADIUS = 2.5


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

    pads = []
    for f in b.GetFootprints():
        for p in f.Pads():
            bb = p.GetBoundingBox()
            pads.append((TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0,
                         TOMM(bb.GetBottom()) - Y0, p.GetNetname()))

    def clear_of_pads(x, y, netname, gap_other=0.55, gap_same=0.35):
        rr = VIA_D / 2
        for l, t, r_, bt, n in pads:
            gp = rr + (gap_same if n == netname else gap_other)
            if l - gp < x < r_ + gp and t - gp < y < bt + gp:
                return False
        return all((x - vx) ** 2 + (y - vy) ** 2 > 1.5 ** 2 for vx, vy, _ in vias)

    for key, r in RAILS.items():
        g = _geo(r)
        xl, xr, vin, out = r["xl"], r["xr"], r["vin"], r["out"]
        drn = pnet(r["fets"][1], "2")
        rows = outputs(FP, r)
        mx = _pxy(FP[r["j_in"]], _pad_on(FP[r["j_in"]], "GND").GetNumber())[0]
        gnd_l = mx - 1.5                                      # ground patch: input - pin and the TVS ground pad
        gnd_bot = TOMM(_pad(FP[r["tvs"]], "2").GetBoundingBox().GetBottom()) - Y0 + 0.55
        col_r, band = g["col_r"], g["band_bot"]
        # input: VIN area with a notch for the ground patch
        zone(vin, F, [(xl, 0.5), (gnd_l - 0.6, 0.5), (gnd_l - 0.6, gnd_bot + 0.5), (xr, gnd_bot + 0.5),
                      (xr, VIN_BOT), (xl, VIN_BOT)], 40, f"{key} VIN")
        zone("GND", F, rect(gnd_l, 0.5, xr, gnd_bot), 40, f"{key} input GND")
        # switch: DRN area (full width above the band, column width below, tongue for a bridge under the FET),
        # OUT strip + full-width OUT area + output strip (L1), OUT copy under the switch (L3)
        drn_pts = [(g["drn_l"], DRN_TOP), (xr, DRN_TOP)]
        drn_pts += [(xr, band), (col_r, band)] if col_r < xr else []
        if g["tongue"]:
            tl, tb = g["tongue"]
            drn_pts += [(col_r, tb), (tl, tb), (tl, DRN_BOT)]
        else:
            drn_pts += [(col_r, DRN_BOT)]
        zone(drn, F, drn_pts + [(g["drn_l"], DRN_BOT)], 40, f"{key} DRN")
        y_end = _pxy(FP[rows[-1][1]], rows[-1][2])[1] + 3.2
        out_pts = [(xl, OUT_TOP), (g["out_r"], OUT_TOP), (g["out_r"], WIDE_TOP)]
        if g["tongue"]:                                   # around the tongue, 0.5 mm off
            out_pts += [(tl - 0.5, WIDE_TOP), (tl - 0.5, tb + 0.5), (col_r, tb + 0.5)]
        else:
            out_pts += [(col_r, WIDE_TOP)]
        zone(out, F, out_pts + [(col_r, WIDE_BOT), (g["strip_r"], WIDE_BOT), (g["strip_r"], y_end), (xl, y_end)],
             40, f"{key} OUT", radius=OUT_RADIUS)
        zone(out, IN2, rect(xl, OUT_TOP, col_r, WIDE_BOT), 40, f"{key} OUT (L3)", clear=0.3, radius=OUT_RADIUS)
        # each output: PTC output pad -> connector + pin, one patch per row
        for j, f, rail, other, plus, minus in rows:
            p = _pad(FP[f], other).GetBoundingBox()
            px, py = _pxy(FP[j], plus)
            zone(_pad(FP[j], plus).GetNetname(), F,
                 rect(TOMM(p.GetLeft()) - X0, py - 2.8, px + 1.5, py + 2.8), 40, f"{key} {j}")
        # FET gates: escape via at the lead tip
        for ref in r["fets"]:
            gx, gy = _pxy(FP[ref], "1")
            vx = g["fet_cx"] - 10.85
            via(pnet(ref, "1"), vx, gy, ESC_D, ESC_DRILL)
            track(pnet(ref, "1"), [(gx - 2.0, gy), (vx, gy)], 0.6)
        # shunt Kelvin pair -> vias; the bottom runs them to the TPS48111 CS+ / CS-
        sh = r["shunt"]
        (k2x, k2y), (k3x, k3y) = _pxy(FP[sh], "2"), _pxy(FP[sh], "3")
        vx = g["shunt_x"] + 2.7
        via(pnet(sh, "2"), vx, Y_SHUNT - 2.2, ESC_D, ESC_DRILL)
        track(pnet(sh, "2"), [(k2x, k2y), (vx, k2y), (vx, Y_SHUNT - 2.2)], 0.25)
        via(pnet(sh, "3"), vx, Y_SHUNT + 2.7, ESC_D, ESC_DRILL)
        track(pnet(sh, "3"), [(k3x, k3y), (vx, k3y), (vx, Y_SHUNT + 2.7)], 0.25)
        # status LED in the DRN area: both pads escape to the bottom
        for num in ("1", "2"):
            lx, ly = _pxy(FP[r["led"]], num)
            via(pnet(r["led"], num), lx + 1.7, ly, ESC_D, ESC_DRILL)
            track(pnet(r["led"], num), [(lx, ly), (lx + 1.7, ly)], 0.4)
        # FET source vias to the L3 copy: a row between the leads, two rows below each source lead
        for cy in FET_ROWS:
            cx = g["fet_cx"]
            for vx in (cx - 9.2, cx - 7.65, cx - 6.1):
                if clear_of_pads(vx, cy + 0.1, out, gap_same=0.3):
                    via(out, vx, cy + 0.1)
            for vy in (cy + 4.2, cy + 5.9):
                for vx in (xl + 1.1, xl + 2.9, xl + 4.7, xl + 6.5):
                    if clear_of_pads(vx, vy, out):
                        via(out, vx, vy)
        # stitching down the left of the full-width OUT area
        y = WIDE_TOP + 1.7
        while y < WIDE_BOT - 1.0:
            for vx in (xl + 0.9, xl + 2.7):
                if clear_of_pads(vx, y, out):
                    via(out, vx, y)
            y += 4.0
    return zones, vias, tracks
