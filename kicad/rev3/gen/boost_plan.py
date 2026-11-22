"""The 54 V boost's controller side (decided 2026-11-22: hand-planned, laid out for the converter's function first).

The power stage is power54's (Q23 low side with its tab on the switch node, Q24 high side with its leads facing
left, the output ceramics in two rows). U10 (LM5122, bottom side) sits left of Q24's leads, its gate-drive pins
11 ... 20 facing them. Down that column: COMP, SLOPE, GND, RES, PGND, LO, VCC, SW, HO, BST. Every line from those
pins leaves east, so the order is fixed: a line turning north passes outside every pin below it. The plan:

  analog    COMP, SLOPE and RES turn north at once and west over the IC's top (a band kept free of parts) to the
            loop network north-west of U10.
  LO        a via right beside its pin; L3 north, then east between Q24's source and gate leads and under its
            OUT54 tab (quiet), north to Q23's gate via (power54's escape). It never runs under the switch node.
  VCC       north up the corridor to its cap C79 (flat, north of U10); a via at the foot of the run carries it
            on L3 down to the bootstrap diode D13's anode (D13 sits under pin 20, by the BST cap).
  SW        into the BST cap C78's SW pad (C78 stands right of pins 18 ... 20), from its top north up the corridor
            and into Q24's source lead: the high-side driver's return is Q24's source.
  HO        leaves between C78's pads (0.2 mm neck), north outside SW and into Q24's gate lead from below.
  BST       into C78's BST pad, on to D13's cathode.
  GND       pins 13 / 15 (and 2 / 9) tie into the exposed pad; C79's GND pad has its own via.
  FB feed   R130 (the divider's top, at the FB pin) gets OUT54 from a via in the slot between Q24's gate lead and
            its tab, on L3 west, north of LO's run; the router finishes it at R130.

Laid after the power copper (key "boost"), as planned polylines checked exactly; the router then takes the loop
network, the sense filter and the brake comparator.
"""
import math

W, DRV_W, NECK_W = 0.25, 0.3, 0.2
# corridor columns, right of U10's pad edge E (U10's right pads end at E): analog lines, then VCC
COL_DX = {"COMP": 0.4, "SLOPE": 0.9, "RES": 1.4, "VCC": 1.9}
TURN_DY = {"COMP": -0.675, "SLOPE": -1.175, "RES": -1.675}   # their westward turns, above pin 11's centre
LO_VIA_DX, LO_VIA_DY = 0.75, -0.225                          # LO's via: right of E, from pin 16's centre
VCC_VIA_DY = -0.975                                          # VCC's via on its column, from pin 17's centre
SW_X_DX = 3.4                                                # SW up the corridor: right of E (C78's SW pad)
HO_X_DX = 4.12                                               # HO up the corridor, outside C78
LO_L3_Y = 137.6                                              # LO on L3: east between Q24's leads
LO_L3_X = 35.2                                               # ... north, east of Q23's GND lead
FB_VIA = (28.09, 136.6)                                      # OUT54 in the slot between Q24's gate lead and tab
FB_L3_X = 20.0                                               # the FB feed's planned part ends here (L3)
VCC_L3_Y = 147.9                                             # VCC on L3 down the corridor, then west
D13_VIA_DX = 1.26                                            # VCC's via left of D13's anode (D13 on top)
BST_VIA = (23.57, 147.25)                                    # BST down from C78's BST pad to D13's cathode (top)
C79_GND_VIA = (21.9, 134.3)                                  # C79's GND via: right of the brake FET's tab (top side)
U10_AT = (16.9, 142.85)
SW_W, FEED_W = 0.5, 0.5                                      # power-class nets: 0.5 mm outside the pin rows
# the band over U10's top where the analog lines turn west, and the corridor: no parts (logic_place)
KEEP = [(12.9, 137.9, 22.3, 139.95)]
# hand-placed loop network (bottom side), in the order the analog lines peel off the band, and the sense filter:
# (net pair (pad up / left, pad down / right), centre, upright?)
ROW_Y = 136.0                                   # the upright row's centre: signal pad down, over the band
# courtyards: 0603 upright 1.55 x 3.38, flat 3.39 x 1.56; 0.5 mm between them; C79 (VCC cap) ends the row east
NORTH = [(("GND", "RES"), (18.745, ROW_Y), True),           # C81
         (("GND", "SLOPE"), (16.685, ROW_Y), True),         # R128
         (("FB", "COMP"), (13.705, 137.0), False),          # C82, flat
         (("CC", "COMP"), (13.705, 134.92), False),         # R129, flat
         (("CC", "FB"), (10.715, ROW_Y), True),             # C83
         (("GND", "FB"), (8.645, ROW_Y), True),             # R131
         (("OUT54", "FB"), (6.575, ROW_Y), True)]           # R130
# the 42 V side of the supply and sense filters on the TOP side, south-west of U10: the Kelvin pair and the OUT42C
# feed arrive there on the top side (south of U10's pad vias, the only way through). East column, flat, input pads
# east: R125 (UVLO top), R122 (VIN filter), R123 / R124 (sense filter); west of it the VIN cap C76 (1206, upright,
# below the brake FET's courtyard) and the sense cap C77 (flat). Each IC-side node drops to the bottom side through one
# via (TOP_VIAS); C76's GND pad has its own.
COL_X = 13.19
TOP = [(("OUT42C", "UVLO"), (COL_X, 142.13), False, "R"),   # R125 ("UVLO" is Net-(Q25-D))
       (("OUT42C", "VIN"), (COL_X, 144.19), False, "R"),     # R122
       (("OUT42C", "CSP"), (COL_X, 146.25), False, "R"),      # R123: Kelvin pad east
       (("LIN", "CSN"), (COL_X, 148.31), False, "R"),        # R124: Kelvin pad east
       (("GND", "VIN"), (9.2, 143.8), True, "C"),            # C76: VIN pad down, by R122's (0.5 mm from Q22)
       (("CSN", "CSP"), (9.25, 148.31), False, "C")]         # C77: CSN pad east, by R124's
TOP_VIAS = {"UVLO": (11.3, 142.75), "VIN": (11.05, 143.85), "CSP": (7.9, 147.25), "CSN": (11.2, 147.45)}
C76_GND_VIA = (7.6, 142.00)                     # west of C76's GND pad (top side), clear of the bottom parts
# bottom side, west of U10: RT's resistor right above pin 8's way north; the UVLO pull-down FET in the strip north-west
# of the network (BOOST_DIS reaches it from the north); the packer puts SS's cap and UVLO's bottom resistor near
WEST = [(("GND", "RT"), (10.7, 139.1), False)]             # R127, flat, RT pad east
Q25_AT = (3.4, 135.5, 90)                       # drain up (UVLO), gate / source down
BOOST_AREA = [(1.9, 138.2, 12.6, 142.25)]      # the packer's area: SS's cap, UVLO's bottom resistor
KEEP_VIAS = [(7.4, 142.45, 13.0, 149.4),         # bottom side: the vias and the sense pair's way to pins 4 / 3
             (6.9, 141.3, 8.3, 142.7),           # C76's GND via
             (5.9, 133.2, 7.3, 134.6),           # R130's OUT54 via (the FB feed's L3 end)
             (18.1, 146.55, 19.5, 147.95),       # VCC's via up to D13's anode
             (22.87, 146.55, 24.27, 147.95)]     # BST's via up to D13's cathode
TP27_XY = (16.6, 148.7)                         # the VCC test pad (top), beside VCC's via
OUT54_FB_VIA_DY = -1.235                         # R130's OUT54 via, above its pad (L3 east along FB_L3_Y)
FB_L3_Y = 133.9


# ---- the long lines on the top side: the sense pair (Kelvin, from the shunt R121's pads) and the OUT42C feed for the
# VIN and UVLO resistors (from the input via row). Down the top side east of the input GND via field, west above the
# GND vias at y 122, down the free strip east of the brake pour, west under U10's south edge (its pad vias leave only
# y > 145.1 there) into the filter column. West to east in the strips: feed, OUT42C (Kelvin), LIN (Kelvin).
KELVIN_VIA = {"OUT42C": (36.2, 103.0), "LIN": (36.2, 106.4)}       # beside R121's pads (bottom side links; the
                                                                   # OUT42C one clear of BRAKE_ACT's L3 at y 102)
FEED_VIA = (34.5, 99.2)                                            # the input row's OUT42C via the feed starts at
STRIP1_X = {"feed": 34.7, "OUT42C": 35.3, "LIN": 36.2}
RUN1_Y = {"feed": 120.1, "OUT42C": 120.7, "LIN": 121.3}
STRIP2_X = {"feed": 25.8, "OUT42C": 26.4, "LIN": 27.0}
RUN2_Y = {"feed": 145.2, "OUT42C": 145.8, "LIN": 146.4}
COL_IN_X = {"feed": 15.0, "OUT42C": 14.9, "LIN": 15.5}             # where each turns into the filter column
TP28_XY = (33.4, 118.5)                                            # OUT42C test pad, on the feed
TP26_XY, TP26_VIA = (30.5, 123.3), (29.1, 123.3)                   # SW test pad, its via into the SW pour
# the "Sense boost" rule areas over the sense pair and feed: power-net tracks may be signal width inside them
SENSE_AREAS = [(34.0, 98.6, 37.4, 108.0), (34.0, 98.6, 37.0, 122.0), (25.3, 119.3, 37.0, 122.0),
               (25.3, 119.3, 27.7, 147.2), (13.3, 141.4, 27.7, 147.2), (13.3, 141.4, 16.0, 149.0),
               (32.4, 117.4, 35.2, 119.6)]
# top side kept from the packer: the strips and runs
KEEP_TOP = [(34.0, 99.0, 36.9, 121.9), (25.2, 119.5, 36.9, 121.9), (25.2, 119.5, 27.6, 147.0),
            (14.5, 144.6, 27.6, 147.0), (28.3, 122.5, 31.6, 124.1)]


def place_keep(obstacles):
    obstacles += KEEP


def place(FP, obstacles):
    """Hand-place the loop network and the sense filter around U10 (after power54 placed U10), locked. Returns the
    references placed; their courtyards join the obstacles."""
    import power54
    from power42 import _upright, _flat
    from satellites import _box
    u = FP["U10"]
    def short(n):                               # "Net-(U10-FB)" -> "FB"; "CC" the compensation node, "UVLO"
        n = n.split("/")[-1]
        if "-Pad" in n:
            return "CC"
        if n == "Net-(Q25-D)":
            return "UVLO"
        if n.endswith("RT)"):                   # Net-(U10-SYNCIN{slash}RT)
            return "RT"
        return n.replace("Net-(U10-", "").rstrip(")")
    sheet = u.GetSheetname()
    done = []
    for (a, b), (x, y), upright, *top in NORTH + WEST + TOP:
        ref = next(r for r, f in FP.items() if f.GetSheetname() == sheet and r[0] in "RC" and not f.IsLocked()
                   and {short(p.GetNetname()) for p in f.Pads()} == {a, b} and "1210" not in f.GetFPIDAsString()
                   and (not top or r[0] == top[0]))
        pa = next(p.GetNumber() for p in FP[ref].Pads() if short(p.GetNetname()) == a)
        pb = next(p.GetNumber() for p in FP[ref].Pads() if short(p.GetNetname()) == b)
        if upright:
            _upright(FP[ref], x, y, pa, pb, bottom=not top)
        elif top:                                   # flat on top: a on the right (east), b on the left
            _flat(FP[ref], x, y, pb, pa)
        else:
            power54._flat_bottom(FP[ref], x, y, pa, pb)
        FP[ref].SetLocked(True)
        if not top:
            obstacles.append(_box(FP[ref]))
        done.append(ref)
    from power42 import _at
    q = next(r for r, f in FP.items() if r.startswith("Q") and f.GetSheetname() == sheet
             and "BOOST_DIS" in {p.GetNetname() for p in f.Pads()})
    _at(FP[q], Q25_AT[0], Q25_AT[1], Q25_AT[2], bottom=True)
    FP[q].SetLocked(True)
    obstacles.append(_box(FP[q]))
    done.append(q)
    tp = next(r for r, f in FP.items() if r.startswith("TP") and f.GetSheetname() == sheet
              and {p.GetNetname() for p in f.Pads()} == {_pnet(u, "17")})
    _at(FP[tp], *TP27_XY, 0)
    FP[tp].SetLocked(True)
    done.append(tp)
    for n, xy in (("OUT42C", TP28_XY), ("SW", TP26_XY)):                   # the input / switch-node test pads
        t = next(r for r, f in FP.items() if r.startswith("TP") and f.GetSheetname() == sheet
                 and {p.GetNetname().split("/")[-1] for p in f.Pads()} == {n})
        _at(FP[t], *xy, 0)
        FP[t].SetLocked(True)
        done.append(t)
    obstacles += KEEP + KEEP_VIAS
    return done


def _pnet(fp, num):
    return next(p.GetNetname() for p in fp.Pads() if p.GetNumber() == num)


def _pad(R, ref, num):
    return next(q for q in R.geo.pads if q["ref"] == ref and q["num"] == num)


def route(R, FP, log=print):
    """Lay the controller side as planned polylines (checked exactly), then the router for the UVLO pull-downs."""
    import far42
    import route_plan
    from route import B, F, IN2
    u = {q["num"]: q for q in R.geo.pads if q["ref"] == "U10"}
    ux = (u["21"]["box"][0] + u["21"]["box"][2]) / 2
    uy = (u["21"]["box"][1] + u["21"]["box"][3]) / 2
    assert math.hypot(ux - U10_AT[0], uy - U10_AT[1]) < 0.05, "boost plan: U10 moved"
    E = u["19"]["box"][2]                                   # right pad edge
    WE = u["10"]["box"][0]                                  # left pad edge
    c = {n: u[n]["c"] for n in u}
    net = {n: u[n]["net"] for n in u}

    def part(ref):
        return {q["net"].split("/")[-1]: q for q in R.geo.pads if q["ref"] == ref}

    def pad_of(n, refs):
        return next(q for q in R.geo.pads if q["net"] == n and q["ref"] in refs)

    c78 = {q["net"]: q for q in R.geo.pads if q["ref"] == "C78"}
    c79 = {q["net"]: q for q in R.geo.pads if q["ref"] == "C79"}
    d13 = {q["net"]: q for q in R.geo.pads if q["ref"] == "D13"}
    q24g = next(q for q in R.geo.pads if q["ref"] == "Q24" and q["num"] == "1")
    q24s = next(q for q in R.geo.pads if q["ref"] == "Q24" and q["num"] == "3")
    q23v = min((v for v in R.geo.vias if v[0] == net["16"]), key=lambda v: v[1])
    vcc, sw, bst, ho, lo = net["17"], net["18"], net["20"], net["19"], net["16"]
    comp, slope, res, fb = net["11"], net["12"], net["14"], net["10"]
    rt, ss, uvlo, vin, csp, csn = net["8"], net["7"], net["6"], net["5"], net["4"], net["3"]
    north = ("C81", "R128", "C82", "R129", "C83", "R131", "R130", "R127", "C80")
    top = ("R125", "R122", "R123", "R124", "C76", "C77", "TP27")
    P = lambda n, refs=north + top: pad_of(n, refs)["c"]
    segs, vias = [], []
    # ---- the analog lines: east, north, west over the top into their parts' pads (north row)
    for k, num, n in (("COMP", "11", comp), ("SLOPE", "12", slope), ("RES", "14", res)):
        x, ty = E + COL_DX[k], c["11"][1] + TURN_DY[k]
        tx, tyy = P(n, ("C82", "R128", "C81"))
        segs += R.track(n, B, [c[num], (x, c[num][1]), (x, ty), (tx, ty), (tx, tyy)], W)
    segs += R.track(comp, B, [P(comp, ("C82",)), P(comp, ("R129",))], W)            # COMP to R129
    cc = pad_of(next(q["net"] for q in R.geo.pads if q["ref"] == "C83" and "-Pad" in q["net"]), ("R129",))["net"]
    segs += R.track(cc, B, [P(cc, ("R129",)), P(cc, ("C83",))], W)                    # the compensation node
    fx = c["10"][0] - 0.44                                                            # FB: north off pin 10's pad
    c82fb = P(fb, ("C82",))
    segs += R.track(fb, B, [c["10"], (fx, c["10"][1]), (fx, c82fb[1] + 0.6), c82fb], W)
    segs += R.track(fb, B, [c82fb, P(fb, ("C83",)), P(fb, ("R131",)), P(fb, ("R130",))], W)
    # ---- GND pins into the exposed pad
    for num in ("13", "15"):
        segs += R.track("GND", B, [c[num], (u["21"]["box"][2] - 0.2, c[num][1])], DRV_W)
    for num in ("2", "9"):
        segs += R.track("GND", B, [c[num], (u["21"]["box"][0] + 0.2, c[num][1])], DRV_W)
    # ---- LO: a via beside its pin; L3 north, east between Q24's leads, north to Q23's gate via
    lv = (E + LO_VIA_DX, c["16"][1] + LO_VIA_DY)
    segs += R.track(lo, B, [c["16"], (E + 0.2, c["16"][1]), lv], DRV_W)
    vias.append(R.via(lo, *lv))
    segs += R.track(lo, IN2, [lv, (lv[0], LO_L3_Y), (LO_L3_X, LO_L3_Y), (LO_L3_X, q23v[2] + 1.0),
                              (q23v[1], q23v[2])], DRV_W)
    # ---- VCC: up the corridor to C79; its via at the foot, L3 down to D13's anode (and the VCC test pad)
    vx = E + COL_DX["VCC"]
    vv = (vx, c["17"][1] + VCC_VIA_DY)
    segs += R.track(vcc, B, [c["17"], (vx, c["17"][1]), (vx, c79[vcc]["c"][1])], DRV_W)
    vias.append(R.via(vcc, *vv))
    da = d13[vcc]["c"]                                                                # D13 is on the top side
    dv = (da[0] - D13_VIA_DX, da[1])
    segs += R.track(vcc, IN2, [vv, (vx, VCC_L3_Y), (dv[0] + 0.65, VCC_L3_Y), dv], DRV_W)
    vias.append(R.via(vcc, *dv))
    segs += R.track(vcc, F, [dv, da], DRV_W)
    segs += R.track(vcc, F, [P(vcc, ("TP27",)), dv], DRV_W)
    gp = c79["GND"]["c"]                                                              # C79's GND via
    segs += R.track("GND", B, [gp, (gp[0], c79["GND"]["box"][1] + 0.1), C79_GND_VIA], DRV_W)
    vias.append(R.via("GND", *C79_GND_VIA))
    # ---- SW: into C78's SW pad, from its top up the corridor into Q24's source lead
    sx = E + SW_X_DX
    sp = c78[sw]
    segs += R.track(sw, B, [c["18"], (sp["box"][0] + 0.25, c["18"][1])], SW_W)
    segs += R.track(sw, B, [(sx, sp["box"][3] - 0.3), (sx, q24s["c"][1]), (q24s["box"][0] + 0.4, q24s["c"][1])],
                    SW_W)
    # ---- HO: between C78's pads, up outside SW, into Q24's gate lead from below
    hx = E + HO_X_DX
    segs += R.track(ho, B, [c["19"], (hx, c["19"][1])], W)
    gl = q24g["box"]
    segs += R.track(ho, B, [(hx, c["19"][1]), (hx, gl[3] + 0.5), (gl[0] + 0.45, gl[3] - 0.15)], DRV_W)
    # ---- BST: into C78's BST pad, on to D13's cathode
    bp = c78[bst]
    segs += R.track(bst, B, [c["20"], (bp["box"][0] + 0.25, c["20"][1])], DRV_W)
    segs += R.track(bst, B, [bp["c"], BST_VIA], DRV_W)
    vias.append(R.via(bst, *BST_VIA))
    segs += R.track(bst, F, [BST_VIA, d13[bst]["c"]], DRV_W)
    # ---- the left column: RT north into R127, SS into C80, UVLO / VIN / CSP / CSN to their vias
    rx = WE - 0.35
    segs += R.track(rt, B, [c["8"], (rx, c["8"][1]), (rx, P(rt)[1]), P(rt)], W)
    sp_ = P(ss)
    segs += R.track(ss, B, [c["7"], (WE - 0.85, c["7"][1]), (sp_[0] + 0.2, sp_[1] + 0.3)], W)
    tv = TOP_VIAS
    segs += R.track(uvlo, B, [c["6"], tv["UVLO"]], W)
    segs += R.track(vin, B, [c["5"], (WE - 0.65, c["5"][1]), tv["VIN"]], W)
    segs += R.track(csp, B, [c["4"], (WE - 0.95, c["4"][1]), (WE - 1.35, c["4"][1] + 0.775), tv["CSP"]], W)
    segs += R.track(csn, B, [c["3"], (WE - 0.5, c["3"][1]), tv["CSN"]], W)
    for k, n in (("UVLO", uvlo), ("VIN", vin), ("CSP", csp), ("CSN", csn)):
        vias.append(R.via(n, *tv[k]))
    # ---- top side: the filters' IC-side pads to their vias, C76 / C77 across them, C76's GND via
    segs += R.track(uvlo, F, [P(uvlo, top), tv["UVLO"]], W)
    vf = P(vin, ("R122",))
    segs += R.track(vin, F, [vf, tv["VIN"], P(vin, ("C76",))], DRV_W)
    pf = P(csp, ("R123",))
    c77p = P(csp, ("C77",))
    cy = (pad_of(vin, ("C76",))["box"][3] + 0.325 + tv["CSN"][1] - 0.775) / 2   # between C76's VIN pad and CSN's via
    segs += R.track(csp, F, [pf, (pf[0] - 0.5, cy), (c77p[0], cy), c77p], W)
    segs += R.track(csp, F, [(c77p[0], tv["CSP"][1]), tv["CSP"]], W)
    nf = P(csn, ("R124",))
    segs += R.track(csn, F, [nf, P(csn, ("C77",))], W)
    segs += R.track(csn, F, [(tv["CSN"][0], nf[1]), tv["CSN"]], W)
    c76g = pad_of("GND", ("C76",))["c"]
    segs += R.track("GND", F, [c76g, C76_GND_VIA], DRV_W)
    vias.append(R.via("GND", *C76_GND_VIA))
    # ---- FB feed: OUT54 via in Q24's lead / tab slot, L3 west north of LO's run, up and west to R130's via
    r130 = pad_of("OUT54", ("R130",))["c"]
    ov = (r130[0], r130[1] + OUT54_FB_VIA_DY)
    vias.append(R.via("OUT54", *FB_VIA))
    vias.append(R.via("OUT54", *ov))
    segs += R.track("OUT54", IN2, [FB_VIA, (FB_L3_X, FB_VIA[1]), (FB_L3_X, FB_L3_Y), (ov[0], FB_L3_Y), ov], FEED_W)
    segs += R.track("OUT54", B, [ov, r130], FEED_W)
    q24t = next(q for q in R.geo.pads if q["ref"] == "Q24" and q["num"] == "2")       # the slot via into the tab
    segs += R.track("OUT54", B, [FB_VIA, (q24t["box"][0] + 0.5, FB_VIA[1])], FEED_W)
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:30]:
        log(f"   boost controller clearance: {msg}")
    if bad:
        R.failed.append("boost controller")
    log(f"   boost controller: {len(segs)} planned segments, {len(vias)} vias, {len(bad)} clearance problems")
    # ---- the long lines on top: Kelvin taps at R121 (bottom), the feed from the input via row, TP28 / TP26
    r121 = {q["net"].split("/")[-1]: q for q in R.geo.pads if q["ref"] == "R121"}
    col = {"feed": (pad_of("OUT42C", ("R125",))["c"], pad_of("OUT42C", ("R122",))["c"]),
           "OUT42C": pad_of("OUT42C", ("R123",))["c"], "LIN": pad_of(next(q["net"] for q in R.geo.pads
                                                                          if q["ref"] == "R124" and q["net"].endswith("LIN")), ("R124",))["c"]}
    lin = next(q["net"] for q in R.geo.pads if q["ref"] == "R121" and q["net"].endswith("LIN"))
    fsegs, fvias = [], []
    for k, n in (("OUT42C", "OUT42C"), ("LIN", lin)):
        p = r121[k]
        inner = (p["box"][0] + 0.25, p["box"][3] - 0.2) if k == "OUT42C" else (p["box"][0] + 0.25, p["box"][1] + 0.2)
        fsegs += R.track(n, B, [inner, KELVIN_VIA[k]], W)
        fvias.append(R.via(n, *KELVIN_VIA[k]))
    starts = {"feed": [FEED_VIA, (STRIP1_X["feed"], FEED_VIA[1] + 0.5)],
              "OUT42C": [KELVIN_VIA["OUT42C"], (STRIP1_X["OUT42C"], KELVIN_VIA["OUT42C"][1])],
              "LIN": [KELVIN_VIA["LIN"]]}
    for k, n in (("feed", "OUT42C"), ("OUT42C", "OUT42C"), ("LIN", lin)):
        pts = starts[k] + [(STRIP1_X[k], RUN1_Y[k]), (STRIP2_X[k], RUN1_Y[k]), (STRIP2_X[k], RUN2_Y[k]),
                           (COL_IN_X[k], RUN2_Y[k])]
        if k == "feed":
            pts += [(COL_IN_X[k], col[k][0][1]), col[k][0]]
            fsegs += R.track(n, F, [(COL_IN_X[k], col[k][1][1]), col[k][1]], W)
        elif k == "OUT42C":
            pts += [(col[k][0] + 0.3, col[k][1])]
        else:
            pts += [(COL_IN_X[k], col[k][1]), col[k]]
        fsegs += R.track(n, F, pts, W)
    tp28 = next(q["c"] for q in R.geo.pads if q["ref"].startswith("TP") and q["net"] == "OUT42C"
                and abs(q["c"][0] - TP28_XY[0]) < 0.1)
    fsegs += R.track("OUT42C", F, [tp28, (STRIP1_X["feed"], tp28[1])], W)
    tp26 = next(q for q in R.geo.pads if q["ref"].startswith("TP") and q["net"] == sw)
    fsegs += R.track(sw, F, [tp26["c"], TP26_VIA], 0.5)
    fvias.append(R.via(sw, *TP26_VIA))
    _sense_areas(R.b)
    bad = route_plan._check_exact(R, fsegs) + far42._check_vias(R, fvias)
    for msg in bad[:20]:
        log(f"   boost sense / feed clearance: {msg}")
    if bad:
        R.failed.append("boost sense / feed")
    log(f"   boost sense pair and feed: {len(fsegs)} segments, {len(fvias)} vias, {len(bad)} clearance problems")
    # ---- the router: UVLO's pull-downs (R126, Q25) to the UVLO node
    r126 = pad_of(uvlo, ("R126",))
    q25 = next(q for q in R.geo.pads if q["net"] == uvlo and q["ref"].startswith("Q"))
    if not R.route_net(uvlo, [R.t_pad(r126["ref"], r126["num"]), R.t_pad(q25["ref"], q25["num"]),
                              R.t_via(*tv["UVLO"])], W, (1.0, 130.0, 14.0, 145.0), label="UVLO (boost)"):
        log("   UVLO: no route")


def _sense_areas(b):
    """Named rule areas (no restrictions) over the sense pair and the feed: the design rules let power-net tracks be
    signal width inside them only ("Power: sense taps", board_rules.py)."""
    import pcbnew
    from power42 import X0, Y0
    for z in list(b.Zones()):
        if z.GetIsRuleArea() and z.GetZoneName().startswith("Sense boost"):
            b.Remove(z)
    for k, (l, t, r, bt) in enumerate(SENSE_AREAS):
        z = pcbnew.ZONE(b)
        z.SetIsRuleArea(True)
        z.SetDoNotAllowTracks(False)
        z.SetDoNotAllowVias(False)
        z.SetDoNotAllowPads(False)
        z.SetDoNotAllowFootprints(False)
        z.SetDoNotAllowZoneFills(False)
        z.SetLayerSet(pcbnew.LSET.AllCuMask())
        z.SetZoneName(f"Sense boost {k + 1}")
        o = z.Outline()
        o.NewOutline()
        for x, y in ((l, t), (r, t), (r, bt), (l, bt)):
            o.Append(pcbnew.VECTOR2I(pcbnew.FromMM(X0 + x), pcbnew.FromMM(Y0 + y)))
        b.Add(z)
