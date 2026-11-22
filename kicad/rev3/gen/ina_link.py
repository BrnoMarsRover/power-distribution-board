"""The INA228's link to the logic (decided 2026-11-21: four lines through the seam between the 42C and 24 V blocks).

The battery monitor sits in the battery-input corner, walled in by the input connectors' through-hole pads and the
42 V blocks (42 V copper on L1 and L3). Its SCL, SDA, ALERT and +3V3 leave on the bottom layer, no vias in the
42 V area:

  at the INA228   SCL, SDA and ALERT west out of its left pins and round its top; +3V3 up from its decoupling cap
                  (the supply pin is tied to it). ALERT's pull-up moved to the ESP32 end, where +3V3 is everywhere.
  the corner      east along y 11.0 ... 12.5 between the battery XT90's pads and the charger XT60's.
  the column      down x 49.45 ... 50.95, right of the charger XT60, left of the 24 V switch's gate track.
  the seam        between 42C's Kelvin corner and the 24 V switch: its leftmost column of L3 stitching vias is
                  gone (power_rails, 24 V "seam") and 42C's Kelvin vias sit 0.35 mm further left (power42, as far
                  as the shunt's pads allow), so four lines pass. They step around the two FET gate vias at the
                  lead tips (x 52.25, y 25.56 / 37.46): 50.6 ... 52.1, then 50.03 ... 51.47 past the lower one
                  (0.48 mm pitch there: the gate via and 42C's CS- via leave 1.45 mm), then 50.8 ... 52.3 beside
                  42C's BUS42 feed down to y 48.6, where the logic area's L3 opens. +3V3 runs on to y 54.3, past
                  the BUS42 feed's end, and drops into the L3 fill there. 42C's CS- comes in from the west at
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
JOG1_Y, JOG2_Y, JOG3_Y, END_Y, V3_Y = 25.5, 35.5, 38.3, 48.6, 54.3
ENDS = {n: (x, V3_Y if n == "+3V3" else END_Y) for n, x in zip(NETS, SEAM_X3)}

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
    x, y = ENDS["+3V3"]
    vias.append(R.via("+3V3", x, y))
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:20]:
        log(f"   INA228 link clearance: {msg}")
    if bad:
        R.failed.append("INA228 link")
    log(f"   INA228 link: {len(segs)} segments, {len(vias)} vias, {len(bad)} clearance problems")
    return segs, vias
