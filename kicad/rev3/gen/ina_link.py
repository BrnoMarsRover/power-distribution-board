"""The INA228's link to the logic (decided 2026-11-21: four lines through the seam between the 42C and 24 V blocks).

The battery monitor sits in the battery-input corner, walled in by the input connectors' through-hole pads and the
42 V blocks (42 V copper on L1 and L3). Its SCL, SDA, ALERT and +3V3 leave on the bottom layer, no vias in the
42 V area:

  at the INA228   SCL, SDA and ALERT west out of its left pins and round its top; +3V3 up from its decoupling cap
                  (the supply pin is tied to it). ALERT's pull-up moved to the ESP32 end, where +3V3 is everywhere.
  the corner      east along y 11.0 ... 12.5 between the battery XT90's pads and the charger XT60's.
  the column      down x 49.45 ... 50.95, right of the charger XT60, left of the 24 V switch's gate track.
  the seam        between 42C's Kelvin corner and the 24 V switch: the rails have no source vias there any more
                  (power_rails, 2026-11-25), and 42C's Kelvin vias sit 0.35 mm further left (power42, as far
                  as the shunt's pads allow), so four lines pass. They step around the two FET gate vias at the
                  lead tips (x 52.25, y 25.56 / 37.46): 50.6 ... 52.1, then 50.03 ... 51.47 past the lower one
                  (0.48 mm pitch there: the gate via and 42C's CS- via leave 1.45 mm), then 50.8 ... 52.3 beside
                  42C's BUS42 feed down to y 44.9, where the I2C plan turns them east to their vias. +3V3 runs on
                  to y 53, past the BUS42 feed's end, and east under the 24 V output strip to its via in the L3
                  fill east of it (out of the strip's current, 2026-11-25). 42C's CS- comes in from the west at
                  y 27.2, just below the westmost lane's step, and down between the CS+ filter (moved 0.35 mm left,
                  efuse_hand) and the link.

Laid first of all the routes (pcb_route.py ina), as planned polylines checked exactly, so the blocks' own plans
route around it. The bus plan takes SCL / SDA / ALERT on from the ends (ENDS); +3V3 drops into the L3 fill there.
KEEP reserves the way from parts.
"""
STAGGER = 0.3
# lanes, west to east in the seam (north to south in the corner): +3V3, SCL, SDA, ALERT
NETS = ("+3V3", "I2C_SCL", "I2C_SDA", "INA_ALERT")
CORNER_Y = (12.5, 12.0, 11.5, 11.0)                 # the corner lanes
COLUMN_X = (49.45, 49.95, 50.45, 50.95)             # down beside the charger XT60
SEAM_X1 = (50.6, 51.1, 51.6, 52.1)                  # from y 27.55 (after the upper gate via)
SEAM_X2 = (50.03, 50.51, 50.99, 51.47)              # past the lower gate via (y 37.46), 0.48 mm pitch
SEAM_X3 = (50.8, 51.3, 51.8, 52.3)                  # beside 42C's BUS42 feed, down to the end
JOG1_Y, JOG2_Y, JOG3_Y, END_Y, V3_Y = 25.5, 35.5, 38.3, 44.9, 52.95
ENDS = {n: (x, V3_Y if n == "+3V3" else END_Y) for n, x in zip(NETS, SEAM_X3)}
# +3V3's via: east of the 24 V output strip (out of its current, 2026-11-25), inside the L3 +3V3 pour, between the
# eFuse's C35 and R51 on the bottom; the lane turns east to it under the strip
V3_JOG = (51.3, 53.45)
V3_VIA = (57.75, 53.45)

KEEP = [(17.6, 10.55, 52.0, 12.95), (49.0, 10.55, 51.75, 25.9), (49.6, 25.4, 52.6, 49.0), (50.3, 49.0, 51.3, 54.8)]


def _jog(lanes_from, lanes_to, y0, right):
    """One 45-degree jog of all four lanes, staggered (the leading line first): per lane (start y, end y)."""
    order = sorted(range(4), key=lambda i: -lanes_from[i] if right else lanes_from[i])
    out = {}
    for k, i in enumerate(order):
        d = abs(lanes_to[i] - lanes_from[i])
        out[i] = (y0 + STAGGER * k, y0 + STAGGER * k + d)
    return out


def paths(c8_plus, pins):
    """The four polylines. c8_plus: the decap's +3V3 pad centre; pins: {net: INA228 pin centre}."""
    j1 = _jog(COLUMN_X, SEAM_X1, JOG1_Y, True)
    j2 = _jog(SEAM_X1, SEAM_X2, JOG2_Y, False)
    j3 = _jog(SEAM_X2, SEAM_X3, JOG3_Y, True)
    out = {}
    for i, net in enumerate(NETS):
        cy, cx = CORNER_Y[i], COLUMN_X[i]
        if net == "+3V3":                      # up from the decap's +3V3 pad
            px, py = c8_plus
            pts = [(px, py), (px, cy + 0.5), (px + 0.5, cy)]
        else:                                  # west out of the left pin row, round the top of the INA228
            px, py = pins[net]
            k = {"I2C_SCL": 0, "I2C_SDA": 1, "INA_ALERT": 2}[net]
            vx = px - 0.95 - 0.5 * k           # the line turning north last runs furthest out
            pts = [(px, py), (vx, py), (vx, cy + 0.5), (vx + 0.5, cy)]
        pts += [(cx - 0.5, cy), (cx, cy + 0.5)]                       # the corner: east, then down
        a, b = j1[i]
        pts += [(cx, a), (SEAM_X1[i], b)]
        a, b = j2[i]
        pts += [(SEAM_X1[i], a), (SEAM_X2[i], b)]
        a, b = j3[i]
        pts += [(SEAM_X2[i], a), (SEAM_X3[i], b), ENDS[net]]
        if net == "+3V3":
            pts += [V3_JOG, V3_VIA]
        out[net] = pts
    return out


def route(R, FP, log=print):
    """Lay the link and its +3V3 via into the L3 fill. Returns the new segments and vias."""
    import far42
    import route_plan
    from route import B
    c8 = next(p for p in R.geo.pads if p["ref"] == "C8" and p["net"] == "+3V3")["c"]
    u1 = {p["net"]: p["c"] for p in R.geo.pads if p["ref"] == "U1" and p["net"] in NETS}
    segs, vias = [], []
    for net, pts in paths(c8, u1).items():
        segs += R.track(net, B, pts, route_plan.SIG_W)
    segs += R.track("+3V3", B, [u1["+3V3"], c8], route_plan.SIG_W)  # the supply pin to its decap
    vias.append(R.via("+3V3", *V3_VIA))
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:20]:
        log(f"   INA228 link clearance: {msg}")
    if bad:
        R.failed.append("INA228 link")
    log(f"   INA228 link: {len(segs)} segments, {len(vias)} vias, {len(bad)} clearance problems")
    return segs, vias


# ---- the shunt's sense pair (decided 2026-11-22, by hand). IN- / IN+ leave pins 9 / 10 east, in line: C7 across
# them right beside the pins, R3 / R2 in series, then the Kelvin pair east and down between the VBAT vias and between
# C3 / C4 (0.55 mm pitch, 0.25 mm from each) to the shunt's Kelvin vias (power42, y 27.75). R2 keeps 0.5 mm from the
# wide VBAT track's end (HV rule). C8 (pin 6's decap) moves up beside pin 6, out of the pins' way (the +3V3 lane
# starts at it); its GND pad ties to pin 7 and a via. VBUS (pin 8) runs east between C8 and C7, past R3, to a via in
# the L3 BUS42 strip (x 31 ... 40.5, down from the charger XT60), at signal width inside a "Sense" rule area.
FILTER = {"C8": ((27.0, 14.3), ("+3V3", "GND"), True),             # upright: first net up / left
          "C7": ((27.0, 18.3), ("Net-(U1-IN-)", "Net-(U1-IN+)"), True),
          "R3": ((30.0, 16.84), ("Net-(U1-IN-)", "Net-(R1-Pad3)"), False),
          "R2": ((30.0, 18.9), ("Net-(U1-IN+)", "Net-(R1-Pad2)"), False)}
KELVIN_X = {"Net-(R1-Pad2)": 31.725, "Net-(R1-Pad3)": 32.275}       # down between the VBAT vias and C3 / C4
KELVIN_Y = {"Net-(R1-Pad2)": 27.55, "Net-(R1-Pad3)": 26.95}         # east to the vias (Pad3 above Pad2's via)
C8_GND_VIA = (28.6, 14.0)
U1_GND_VIA = (19.0, 18.75)                                           # pins 1 / 2
VBUS_PATH = [(25.9, 17.0), (26.6, 16.3), (27.95, 16.3), (29.4, 14.85), (31.0, 14.85)]
VBUS_VIA = (31.9, 13.95)
VBUS_AREA = (24.3, 13.3, 32.5, 17.3)                                 # "Sense INA228 VBUS"


def place_filter(FP, obstacles):
    """The INA228's input filter and decap by hand (locked). Returns the references placed."""
    import power54
    from power42 import _upright
    from satellites import _box
    short = lambda p: p.GetNetname().split("/")[-1]
    done = []
    for ref, ((x, y), (a, b), upright) in FILTER.items():
        assert {short(p) for p in FP[ref].Pads()} == {a, b}, f"INA228 filter: {ref} changed"
        pa = next(p.GetNumber() for p in FP[ref].Pads() if short(p) == a)
        pb = next(p.GetNumber() for p in FP[ref].Pads() if short(p) == b)
        if upright:
            _upright(FP[ref], x, y, pa, pb, bottom=True)
        else:
            power54._flat_bottom(FP[ref], x, y, pa, pb)
        FP[ref].SetLocked(True)
        obstacles.append(_box(FP[ref]))
        done.append(ref)
    return done


def sense(R, log=print):
    """The input filter's copper, the Kelvin pair to the shunt, VBUS and the IC's GND, planned (checked exactly)."""
    import far42
    import route_plan
    from route import B
    W = route_plan.SIG_W
    short = lambda q: q["net"].split("/")[-1]
    P = lambda ref, n: next(q for q in R.geo.pads if q["ref"] == ref and short(q) == n)
    u = {q["num"]: q for q in R.geo.pads if q["ref"] == "U1"}
    inn, inp, k2, k3 = "Net-(U1-IN-)", "Net-(U1-IN+)", "Net-(R1-Pad2)", "Net-(R1-Pad3)"
    segs, vias = [], []

    def lay(net, pts, w=W):
        segs.extend(R.track(net, B, pts, w))

    # IN-: pin 9 east into C7's top pad, on to R3; IN+: pin 10 east, down into C7's bottom pad, on to R2
    p9, p10 = u["9"]["c"], u["10"]["c"]
    ct, cb, r3, r2 = P("C7", inn)["c"], P("C7", inp)["c"], P("R3", inn)["c"], P("R2", inp)["c"]
    lay(u["9"]["net"], [p9, (ct[0], p9[1]), (r3[0] - abs(p9[1] - r3[1]), p9[1]), r3])
    xd = cb[0] - 0.1 - (cb[1] - p10[1])
    lay(u["10"]["net"], [p10, (xd, p10[1]), (cb[0] - 0.1, cb[1]), cb, (r2[0] - abs(r2[1] - cb[1]), cb[1]), r2])
    # the Kelvin pair: east out of R3 / R2, down, east into the shunt's vias
    for ref, n in (("R3", k3), ("R2", k2)):
        q = P(ref, n)
        (x0, y0), x, y = q["c"], KELVIN_X[n], KELVIN_Y[n]
        v = next(v for v in R.geo.vias if v[0].split("/")[-1] == n)
        lay(q["net"], [(x0, y0), (x - 0.4, y0), (x, y0 + 0.4), (x, y - 0.2), (x + 0.2, y),
                       (v[1] - abs(v[2] - y), y), (v[1], v[2])])
    # GND: pin 7 into C8's GND pad, on to a via; pins 1 / 2 to their own via
    g7, gc = u["7"]["c"], P("C8", "GND")["c"]
    xg = gc[0] - 0.06 - (g7[1] - gc[1])                 # its diagonal between pin 6's corner and VBUS's
    lay("GND", [g7, (xg, g7[1]), (gc[0] - 0.06, gc[1]), gc, (gc[0] + 0.44, gc[1]), C8_GND_VIA], W)
    vias.append(R.via("GND", *C8_GND_VIA))
    g1, g2 = u["1"]["c"], u["2"]["c"]
    lay("GND", [g2, (g2[0] - 0.45, g2[1]), (g1[0] - 0.45, g1[1])], W)
    lay("GND", [g1, (g1[0] - 0.6, g1[1]), U1_GND_VIA], W)
    vias.append(R.via("GND", *U1_GND_VIA))
    # VBUS into the L3 BUS42 strip
    lay(u["8"]["net"], [u["8"]["c"]] + VBUS_PATH + [VBUS_VIA])
    vias.append(R.via(u["8"]["net"], *VBUS_VIA))
    _vbus_area(R.b)
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:20]:
        log(f"   INA228 sense clearance: {msg}")
    if bad:
        R.failed.append("INA228 sense")
    log(f"   INA228 sense pair, filter, VBUS and GND: {len(segs)} segments, {len(vias)} vias, "
        f"{len(bad)} clearance problems")


def _vbus_area(b):
    """A named rule area (no restrictions) over VBUS's narrow line: the "Power: sense taps" rule lets it be signal
    width there (board_rules.py)."""
    import pcbnew
    from power42 import X0, Y0
    for z in list(b.Zones()):
        if z.GetIsRuleArea() and z.GetZoneName() == "Sense INA228 VBUS":
            b.Remove(z)
    z = pcbnew.ZONE(b)
    z.SetIsRuleArea(True)
    z.SetDoNotAllowTracks(False)
    z.SetDoNotAllowVias(False)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowFootprints(False)
    z.SetDoNotAllowZoneFills(False)
    z.SetLayerSet(pcbnew.LSET.AllCuMask())
    z.SetZoneName("Sense INA228 VBUS")
    l, t, r, bt = VBUS_AREA
    o = z.Outline()
    o.NewOutline()
    for x, y in ((l, t), (r, t), (r, bt), (l, bt)):
        o.Append(pcbnew.VECTOR2I(pcbnew.FromMM(X0 + x), pcbnew.FromMM(Y0 + y)))
    b.Add(z)
