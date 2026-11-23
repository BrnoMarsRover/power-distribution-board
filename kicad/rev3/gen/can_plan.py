"""The CAN group, by hand (decided 2026-11-23): the bus side, the split termination, and U16's logic side.

U16 (TCAN1044, bottom side) has CANL (pin 6) above CANH (pin 7) on its east column. The two connectors J34 / J35
(top side, SMD, in parallel) have the same order: CANL (pin 2) above CANH (pin 1), GND (pins 3, 4) above both.
Joining J34 to J35 on one layer west of the connectors crosses the two lines (L H L H down the pad column), so
one line changes layer once.

  pair        U16's pins 6 / 7 east on the bottom side, up two vias in front of J34: CANL's (VL) east of BRAKE_ACT's
              L3 end, CANH's (VH) further west. Top side: each via straight east into its J34 pin. CANH runs from
              VH down the top side to J35's pin 1; CANL runs from VL down L3 to a via (VL2) at J35's pin 2, under
              CANH's stub to J34 (the one crossing).
  termination the split termination on the top side, a "C" west of the vias, symmetric about the pair: CANL's
              jumper JP8 and R164 in a row above, CANH's jumper JP7 and R163 in a row below, their mid node a
              short vertical with C111 (to GND, one via) west of it. Jumpers on top so they can be closed on the rover.
  GND         each connector's GND pins 3 / 4 straight west onto their own vias (to the L2 plane): J34's east of
              BRAKE_ACT, J35's between CANH's top-side line and CANL's L3 line.
  logic       STB, TXD and RXD from the ESP32's left-column escapes (module pins 19, 20, 21, top to bottom; assigned
              in that order so that nothing crosses, design.md 6.6) down a bottom-side corridor under the module's
              left edge, side by side, west to east RXD / TXD / STB, with a short step west round two pull-ups;
              past the INA228 ALERT's vias they step west into U16: RXD into pin 4 (north-west), TXD and STB
              under the body into pins 1 and 8.
  supplies    C109 (+5V_LOGIC) upright west of pin 3, its GND to pin 2 and to C111's GND via; C110 (+3V3) above pin 5,
              pin 5's +3V3 via east of it. The CAN sheet's GND test loop above C110, one GND via shared with C110.

Laid in its own routing stage (pcb_route can), after the ESP32 group's planned copper (its escapes, BRAKE_ACT,
esp_plan), before the router's stages. Polylines checked exactly.
"""
W, W_GND, W_SUP = 0.25, 0.4, 0.3
JP_X, R_X = 134.6, 130.4                 # the jumpers' and the resistors' centres (top side, flat)
Y_L, Y_H = 100.25, 103.35                # the CANL row (JP8, R164) and the CANH row (JP7, R163)
C111_XY = (127.0, 102.6625)              # upright, its mid-node pad level with the pair's centre line (y 101.8)
VL_X, VH_X = 138.8, 137.0                # the pair's vias (at J34's pin rows)
J35_GND_X = 137.9                        # J35's GND vias, between CANH (top, x 137.0) and CANL (L3, x 138.8)
C111_GND_VIA = (127.0, 105.05)           # also C109's (and so U16's) GND
# the logic lines: corridor columns (bottom side), the westward step under ALERT's vias (start y per line, so the
# step keeps the 0.7 mm pitch), and their columns at U16
CORRIDOR_X = {"RXD": 132.6, "TXD": 133.3, "STB": 134.0}
STEP_Y = {"RXD": 92.6, "TXD": 92.89, "STB": 93.18}
STEP = 1.75
# between the LED escapes and R143 the three step 0.85 mm west and back, round the packer's +3V3 pull-ups R159 /
# R142 (y 78 / 82.5, east) and past the IO0 / MOTOR_OK vias (west); each step staggered to keep the pitch
DODGE = 0.85
DODGE_Y = {"RXD": (76.0, 84.6), "TXD": (76.29, 84.31), "STB": (76.58, 84.02)}
C109_XY = (126.95, 101.2225)             # upright west of U16's pin 3, its +5V_LOGIC pad level with the pin
C110_XY = (135.23, 96.975)               # flat above pin 5, +3V3 pad west
V3_VIA = (136.4, 99.095)                 # pin 5's +3V3 via (to the L3 fill), east of it
TP_XY = (136.995, 93.2)                  # the CAN sheet's GND loop, upright, above C110
TP_GND_VIA = (136.6, 95.7)               # its via, shared with C110's GND pad
# kept free of other parts. Top (pcb_place, pcb_optimize): the termination, its GND via, the vias and lines down to
# J35. Bottom (added once U16 stands, logic_place): the pair's steps and vias east of U16, the GND vias, the logic
# lines' step into U16
KEEP_TOP = [(126.2, 97.2, 139.0, 105.7), (136.3, 105.7, 139.0, 116.5), (135.5, 90.9, 138.5, 96.2),
            (131.0, 89.1, 132.5, 92.45)]        # (the last: the INA228 ALERT's vias, i2c_plan, under no top pad)
KEEP_BOTTOM = [(136.0, 98.4, 139.4, 103.1), (137.6, 97.4, 139.4, 98.4), (137.3, 110.8, 139.4, 114.9),
               (126.4, 103.0, 127.6, 105.6), (130.4, 92.6, 134.45, 98.3)]
# (the corridor itself is not kept: a keep there re-packs the ESP32 support group, whose packer then loses the board
# NTC; the group gets its hand placement in the ESP32 leftovers step)


def _nets(fp):
    return {p.GetNumber(): p.GetNetname() for p in fp.Pads() if p.GetNumber()}


def logic_nets(R):
    """U16's TXD, RXD and STB nets (planned here, so the ESP32 group's router leaves them)."""
    return [q["net"] for q in R.geo.pads if q["ref"] == "U16" and q["num"] in ("1", "4", "8")]


def parts(FP):
    """The termination's references by net, from U16's bus pins: (JP_L, R_L, C, R_H, JP_H) and the nets."""
    u = _nets(FP["U16"])
    canh, canl = u["7"], u["6"]
    with_net = lambda prefix, net, skip=(): next(r for r, f in FP.items() if r.startswith(prefix) and r not in skip
                                                  and net in _nets(f).values())
    other = lambda ref, net: next(n for n in _nets(FP[ref]).values() if n != net)
    jh = with_net("JP", canh)
    th = other(jh, canh)
    rh = with_net("R", th)
    tmid = other(rh, th)
    c = with_net("C", tmid)
    rl = with_net("R", tmid, skip=(rh,))
    tl = other(rl, tmid)
    jl = with_net("JP", tl)
    return dict(jl=jl, rl=rl, c=c, rh=rh, jh=jh), dict(canh=canh, canl=canl, th=th, tmid=tmid, tl=tl)


def supplies(FP):
    """U16's decaps (C109 on +5V_LOGIC, C110 on +3V3) and the CAN sheet's GND test loop."""
    u = _nets(FP["U16"])
    sheet = FP["U16"].GetSheetname()
    cap = lambda n: next(r for r, f in FP.items() if r.startswith("C") and f.GetSheetname() == sheet
                         and set(_nets(f).values()) == {n, "GND"})
    tp = next(r for r, f in FP.items() if r.startswith("TP") and f.GetSheetname() == sheet)
    return cap(u["3"]), cap(u["5"]), tp


def place(FP, obstacles):
    """The termination (top side), U16's decaps (bottom) and the GND loop (top) by hand, locked. Returns the
    references placed."""
    import power42
    import power54
    from power_aux import _centre
    ref, net = parts(FP)
    pad = lambda r, n: next(k for k, v in _nets(FP[r]).items() if v == n)
    rows = ((ref["jl"], JP_X, Y_L, net["tl"], net["canl"]), (ref["rl"], R_X, Y_L, net["tmid"], net["tl"]),
            (ref["jh"], JP_X, Y_H, net["th"], net["canh"]), (ref["rh"], R_X, Y_H, net["tmid"], net["th"]))
    for r, x, y, west, east in rows:
        power42._flat(FP[r], x, y, pad(r, west), pad(r, east))
        _centre(FP[r], x, y)
    c = ref["c"]
    power42._upright(FP[c], *C111_XY, pad(c, net["tmid"]), pad(c, "GND"))
    _centre(FP[c], *C111_XY)
    c109, c110, tp = supplies(FP)
    u = _nets(FP["U16"])
    power42._upright(FP[c109], *C109_XY, pad(c109, u["3"]), pad(c109, "GND"), bottom=True)
    _centre(FP[c109], *C109_XY)
    power54._flat_bottom(FP[c110], *C110_XY, pad(c110, u["5"]), pad(c110, "GND"))
    _centre(FP[c110], *C110_XY)
    for deg in (0, 90, 180, 270):                  # the loop's pad standing (taller than wide)
        power42._at(FP[tp], *TP_XY, deg)
        bb = FP[tp].Pads()[0].GetBoundingBox()
        if bb.GetHeight() > bb.GetWidth():
            break
    _centre(FP[tp], *TP_XY)
    done = list(ref.values()) + [c109, c110, tp]
    for r in done:
        FP[r].SetLocked(True)
    obstacles += KEEP_BOTTOM
    return done


def route(R, FP, log=print):
    """The pair, the termination, the connectors' GND vias, U16's logic lines and supplies, planned (checked
    exactly)."""
    import esp_plan
    import far42
    import route_plan
    from route import B, F, IN2
    ref, net = parts(FP)
    c = lambda r, n: next(q["c"] for q in R.geo.pads if q["ref"] == r and q["net"] == n)
    pin = lambda r, num: next(q["c"] for q in R.geo.pads if q["ref"] == r and q["num"] == num)
    segs, vias = [], []

    def lay(n, layer, pts, w=W):
        segs.extend(R.track(n, layer, pts, w))

    def via(n, xy):
        vias.append(R.via(n, *xy))

    canh, canl = net["canh"], net["canl"]
    vl, vh = (VL_X, pin("J34", "2")[1]), (VH_X, pin("J34", "1")[1])
    vl2 = (VL_X, pin("J35", "2")[1])
    u6, u7 = pin("U16", "6"), pin("U16", "7")
    # bottom side: U16 east, a 45-degree step down into each via
    lay(canl, B, [u6, (vl[0] - (vl[1] - u6[1]), u6[1]), vl])
    lay(canh, B, [u7, (vh[0] - (vh[1] - u7[1]), u7[1]), vh])
    via(canl, vl)
    via(canh, vh)
    # top side: into J34; CANH on down to J35's pin 1; CANL down L3 to its via at J35's pin 2
    lay(canl, F, [vl, pin("J34", "2")])
    lay(canh, F, [vh, pin("J34", "1")])
    j35h = pin("J35", "1")
    lay(canh, F, [vh, (vh[0], j35h[1] - 1.0), (vh[0] + 1.0, j35h[1]), j35h])
    lay(canl, IN2, [vl, vl2])
    via(canl, vl2)
    lay(canl, F, [vl2, pin("J35", "2")])
    # the termination: each via west into its jumper, jumper to resistor, the mid node, C111 to GND
    jl, jh = c(ref["jl"], canl), c(ref["jh"], canh)
    lay(canl, F, [vl, (jl[0] + abs(vl[1] - jl[1]), vl[1]), jl])
    lay(canh, F, [vh, (vh[0] - abs(jh[1] - vh[1]), jh[1]), jh])
    lay(net["tl"], F, [c(ref["jl"], net["tl"]), c(ref["rl"], net["tl"])])
    lay(net["th"], F, [c(ref["jh"], net["th"]), c(ref["rh"], net["th"])])
    ml, mh, mc = c(ref["rl"], net["tmid"]), c(ref["rh"], net["tmid"]), c(ref["c"], net["tmid"])
    lay(net["tmid"], F, [ml, mh])
    lay(net["tmid"], F, [mc, (ml[0], mc[1])])
    lay("GND", F, [c(ref["c"], "GND"), C111_GND_VIA], W_GND)
    via("GND", C111_GND_VIA)
    # the connectors' GND pins: straight west onto a via each
    for j, x in (("J34", VL_X), ("J35", J35_GND_X)):
        for num in ("3", "4"):
            p = pin(j, num)
            lay("GND", F, [p, (x, p[1])], W_GND)
            via("GND", (x, p[1]))
    # the logic lines: escape via -> 45 degrees into the corridor -> south -> step west -> into U16
    u = {q["num"]: q for q in R.geo.pads if q["ref"] == "U16"}
    esc = esp_plan.escape_points(R)
    ends = {"RXD": "4", "TXD": "1", "STB": "8"}
    for key, num in ends.items():
        n, (px, py) = u[num]["net"], u[num]["c"]
        (ex, ey), = esc[n]
        x0, y0 = CORRIDOR_X[key], STEP_Y[key]
        x1 = x0 - STEP
        d0, d1 = DODGE_Y[key]
        pts = [(ex, ey), (x0, ey + (x0 - ex)), (x0, d0), (x0 - DODGE, d0 + DODGE), (x0 - DODGE, d1), (x0, d1 + DODGE),
               (x0, y0), (x1, y0 + STEP)]
        if key == "RXD":                            # north-west pin: down beside the column, in from its corner
            pts += [(x1, py - (x1 - 130.16)), (130.16, py), (px, py)]
        elif key == "TXD":                          # under the body, west into pin 1
            pts += [(x1, py - 0.7), (x1 - 0.7, py), (px, py)]
        else:                                       # under the body, east into pin 8
            pts += [(x1, py - 0.7), (x1 + 0.7, py), (px, py)]
        lay(n, B, pts)
    # supplies: pin 3 west to C109, C109's GND to pin 2 and down to C111's GND via; C110 to pin 5, pin 5's via;
    # the GND loop and C110's GND on one via
    c109, c110, tp = supplies(FP)
    v5, v3 = u["3"]["net"], u["5"]["net"]
    p3, p2, p5 = u["3"]["c"], u["2"]["c"], u["5"]["c"]
    a, g = c(c109, v5), c(c109, "GND")
    lay(v5, B, [p3, a], W_SUP)
    lay("GND", B, [g, (g[0] + (g[1] - p2[1]), p2[1]), p2], W_SUP)
    lay("GND", B, [g, C111_GND_VIA], W_GND)
    lay(v3, B, [c(c110, v3), p5], W_SUP)
    lay(v3, B, [p5, V3_VIA], W_SUP)
    via(v3, V3_VIA)
    t = c(tp, "GND")
    lay("GND", F, [t, (t[0], TP_GND_VIA[1] - (t[0] - TP_GND_VIA[0])), TP_GND_VIA], W_GND)
    via("GND", TP_GND_VIA)
    lay("GND", B, [TP_GND_VIA, c(c110, "GND")], W_GND)
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:25]:
        log(f"   CAN clearance: {msg}")
    if bad:
        R.failed.append("CAN")
    log(f"   CAN group: {len(segs)} segments, {len(vias)} vias, {len(bad)} clearance problems")
