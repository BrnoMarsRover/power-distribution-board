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
WEST = [(("GND", "RT"), (10.7, 139.1), False),             # R127, flat, RT pad east
        (("GND", "UVLO"), (6.79, 138.99), False),          # R126 (UVLO's bottom), flat, east of BOOST_DIS's via
        (("GND", "SS"), (10.7, 141.17), False)]            # C80 (soft start), flat, under R127
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

# ---- the corner's last parts and nets (decided 2026-11-22)
# BOOST_DIS: the expander's pin 18 (U17, under the 12 V outputs) to the UVLO pull-down Q25 in the corner, ~130 mm, a
# DC line. U17's pocket is closed on L3 (the I2C trunk, MOTOR_ARM's hop), so: a via beside pin 18 (in route_plan's
# U17_ESC_KEEP), L3 south-west beside I2C_SCL's diagonal to a via above the 12 V XT60 J20, the bottom side down
# between J20 / J21's left pins and I2C_SDA's hop vias, L3 again west along y 102.0 beside BRAKE_ACT (y 101.4,
# esp_plan) and down the corner with it, east of it: down the 42D plane's east edge, down beside the switch-node
# shield, west under the comparator's south edge (BRAKE_ACT turns west above the comparator's IC), down between the
# brake FET's gate via and R130's OUT54 via to a via under Q25. The pull-up R135 sits on the bottom-side column below
# J21 (the corner under Q25 is full: the UVLO and SS parts), its +3V3 from the L3 fill north of the lanes.
DIS_V1 = (94.7, 72.5)                  # beside U17's pin 18
DIS_V2 = (90.5, 81.5)                  # above J20, below the latch's MOTOR_KILL test pad
DIS_V3 = (87.95, 102.6)                # below J21, south of BRAKE_ACT's lane
DIS_COL_B = 87.95                      # bottom side: between J20 / J21 (x 88.44) and SDA's hop vias (x 87.0)
DIS_L3 = [(87.35, 102.0), (25.6, 102.0), (25.0, 102.6), (25.0, 109.6), (23.6, 111.0), (23.6, 132.75), (23.2, 133.15),
          (7.5, 133.15), (6.9, 132.55), (5.9, 132.55), (5.3, 133.15), (5.3, 137.5)]
DIS_V4 = (4.35, 138.45)                # under Q25's gate pad
R135_AT = (85.6, 100.6)                # pull-up, flat on the bottom: +3V3 west, BOOST_DIS east to the column
R135_V3V3 = (83.3, 100.6)              # its +3V3 via (L3 fill, 0.225 mm north of BRAKE_ACT's lane)
R112_VIA_DY = 0.95                     # the comparator's OUT42D feed: a via into the 42D planes above R112's pad
# the boost's status LED D15 (top, pcb_place) and its resistor R132 beside it on the anode's line; R132's OUT54 comes
# up from the output plane on L3 (the plane is south of y 121 there, the sense strips close the top side east)
R132_X = 31.8                          # flat on top: D15-A pad west, OUT54 pad east
LED_OUT54_VIA, LED_OUT54_L3_Y = (33.35, 116.6), 122.0
LED_GND_X = 26.0                       # D15's cathode west and up to the input field's GND via (26.0, 112.1)
# the output side: TP30 (OUT54) on top above the fuse's input pad; the ADC divider (R133 OUT54 / VSNS, R134 and C92
# VSNS / GND) in a bottom column under the 24 V PTCs, OUT54 west into the output pour's via, VSNS east for U21's leg
TP30_XY = (31.9, 140.8)
DIV_X, DIV_Y = 52.2, (139.2, 141.26, 143.32)
DIV_GND_VIA = (51.375, 144.75)
O54_W = 1.5                            # the fuse to the Zyxel connector (1.85 A at most)
KEEP_CORNER = [(82.75, 100.05, 83.85, 101.15)]   # R135's +3V3 via (packer)


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
    return done + place_outer(FP, obstacles)


def place_outer(FP, obstacles):
    """The corner's last parts by hand (locked): BOOST_DIS's pull-up under Q25, the LED's resistor beside the LED, the
    OUT54 test pad above the fuse, the ADC divider under the 24 V PTCs."""
    import power54
    from power42 import _upright, _flat, _at, _pxy
    from satellites import _box
    sheet = FP["U10"].GetSheetname()
    nets = lambda r: {p.GetNetname().split("/")[-1] for p in FP[r].Pads()}
    num = lambda r, n: next(p.GetNumber() for p in FP[r].Pads() if p.GetNetname().split("/")[-1] == n)
    mine = [r for r, f in FP.items() if f.GetSheetname() == sheet]
    done = []
    r135 = next(r for r in mine if r[0] == "R" and "BOOST_DIS" in nets(r))
    power54._flat_bottom(FP[r135], *R135_AT, num(r135, "+3V3"), num(r135, "BOOST_DIS"))
    led = next(r for r in mine if "LED" in FP[r].GetFPIDAsString())
    anode = next(n for n in nets(led) if n != "GND")
    r132 = next(r for r in mine if r[0] == "R" and anode in nets(r))
    _flat(FP[r132], R132_X, _pxy(FP[led], num(led, anode))[1], num(r132, anode), num(r132, "OUT54"))
    tp30 = next(r for r in mine if r.startswith("TP") and nets(r) == {"OUT54"})
    _at(FP[tp30], *TP30_XY, 0)
    div = []
    for y, kind, want in zip(DIV_Y, "RRC", (("OUT54", "VSNS_OUT_54"), ("GND", "VSNS_OUT_54"), ("GND", "VSNS_OUT_54"))):
        r = next(r for r in mine if r[0] == kind and nets(r) == set(want))
        power54._flat_bottom(FP[r], DIV_X, y, num(r, want[0]), num(r, want[1]))
        div.append(r)
    for r in (r135, r132, tp30, *div):
        FP[r].SetLocked(True)
        if FP[r].IsFlipped():
            obstacles.append(_box(FP[r]))
        done.append(r)
    obstacles += KEEP_CORNER
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
    outer(R, log)
    # ---- the router: UVLO's pull-downs (R126, Q25) to the UVLO node
    r126 = pad_of(uvlo, ("R126",))
    q25 = next(q for q in R.geo.pads if q["net"] == uvlo and q["ref"].startswith("Q"))
    if not R.route_net(uvlo, [R.t_pad(r126["ref"], r126["num"]), R.t_pad(q25["ref"], q25["num"]),
                              R.t_via(*tv["UVLO"])], W, (1.0, 130.0, 14.0, 145.0), label="UVLO (boost)"):
        log("   UVLO: no route")


def outer(R, log=print):
    """The corner's last planned copper: BOOST_DIS from the expander, the LED and its resistor, TP30 and the fuse link
    to the Zyxel connector, the ADC divider's ties, the comparator's OUT42D feed (checked exactly)."""
    import far42
    import route_plan
    from route import B, F, IN2
    pad = lambda ref, n: next(q for q in R.geo.pads if q["ref"] == ref and q["net"].split("/")[-1] == n)
    by_net = lambda n, pre: next(q for q in R.geo.pads if q["net"].split("/")[-1] == n and q["ref"].startswith(pre))
    segs, vias = [], []

    def lay(net, layer, pts, w=W):
        segs.extend(R.track(net, layer, pts, w))

    def via(net, xy):
        vias.append(R.via(net, *xy))

    # BOOST_DIS
    n = "BOOST_DIS"
    p18 = pad("U17", n)["c"]
    x1, y1 = DIS_V1
    lay(n, B, [p18, (p18[0], p18[1] + 1.04), (x1, p18[1] + 1.04 + (p18[0] - x1)), DIS_V1])
    via(n, DIS_V1)
    x2, y2 = DIS_V2
    lay(n, IN2, [DIS_V1, (x1, y1 + 0.4), (x2, y1 + 0.4 + (x1 - x2)), DIS_V2])
    via(n, DIS_V2)
    xb = DIS_COL_B
    lay(n, B, [DIS_V2, (xb + 1.05, y2), (xb, y2 + 1.05), DIS_V3])
    via(n, DIS_V3)
    q = pad("Q25", n)["c"]
    lay(n, IN2, [DIS_V3] + DIS_L3 + [DIS_V4])
    via(n, DIS_V4)
    r135 = next(q for q in R.geo.pads if q["net"] == n and q["ref"].startswith("R"))
    lay(n, B, [q, DIS_V4])
    lay(n, B, [r135["c"], (DIS_COL_B, r135["c"][1])])
    v33 = next(q for q in R.geo.pads if q["ref"] == r135["ref"] and q["net"] == "+3V3")
    lay("+3V3", B, [v33["c"], R135_V3V3])
    via("+3V3", R135_V3V3)
    # the LED: anode to R132 on its line, R132's OUT54 down to the output plane on L3, the cathode to the GND field
    led = next(q for q in R.geo.pads if q["ref"].startswith("D") and q["net"].startswith("Net-(D15"))
    an = led["net"]
    ka = next(q for q in R.geo.pads if q["ref"] == led["ref"] and q["net"] == "GND")
    ra = next(q for q in R.geo.pads if q["ref"].startswith("R") and q["net"] == an)
    ro = next(q for q in R.geo.pads if q["ref"] == ra["ref"] and q["net"] != an)
    lay(an, F, [led["c"], ra["c"]])
    lay(ro["net"], F, [ro["c"], LED_OUT54_VIA], 0.5)
    via(ro["net"], LED_OUT54_VIA)
    lay(ro["net"], IN2, [LED_OUT54_VIA, (LED_OUT54_VIA[0], LED_OUT54_L3_Y)], 0.5)
    gv = (LED_GND_X, 112.1)
    lay("GND", F, [ka["c"], (LED_GND_X, ka["c"][1]), gv], 0.4)
    # the output: TP30 into the fuse's input pad, the fuse's output pad to J13's +
    tp = by_net("OUT54", "TP")
    f1 = next(q for q in R.geo.pads if q["ref"].startswith("F") and q["net"].endswith("OUT54") and q["ref"] != tp["ref"])
    lay(f1["net"], F, [tp["c"], (tp["c"][0], f1["box"][1] + 0.45)], 0.5)
    f2 = next(q for q in R.geo.pads if q["ref"] == f1["ref"] and q["num"] != f1["num"])
    j13 = next(q for q in R.geo.pads if q["net"] == f2["net"] and q["ref"].startswith("J"))
    lay(f2["net"], F, [f2["c"], (j13["c"][0], f2["c"][1])], O54_W)
    # the ADC divider: OUT54 west to the output pour's via, VSNS down its east side, GND down its west side to a via
    col = sorted((q for q in R.geo.pads if q["ref"][0] in "RC" and abs(q["c"][0] - DIV_X) < 1.2
                  and DIV_Y[0] - 0.5 < q["c"][1] < DIV_Y[-1] + 0.5), key=lambda q: (q["c"][1], q["c"][0]))
    top = [q for q in col if abs(q["c"][1] - DIV_Y[0]) < 0.1]
    o54 = next(q for q in top if q["net"].endswith("OUT54"))
    ov = min((v for v in R.geo.vias if v[0] == o54["net"] and v[1] < DIV_X), key=lambda v: math.dist(v[1:3], o54["c"]))
    lay(o54["net"], B, [o54["c"], (ov[1] + abs(o54["c"][1] - ov[2]), o54["c"][1]), ov[1:3]], 0.5)
    vs = [q for q in col if q["net"].endswith("VSNS_OUT_54")]
    lay(vs[0]["net"], B, [vs[0]["c"], vs[-1]["c"]])
    gs = [q for q in col if q["net"] == "GND"]
    lay("GND", B, [gs[0]["c"], gs[-1]["c"], DIV_GND_VIA], 0.4)
    via("GND", DIV_GND_VIA)
    # the comparator's OUT42D feed: R112 (to the zener) from the 42D planes right above it
    r112 = min((q for q in R.geo.pads if q["net"].endswith("OUT42D") and q["ref"].startswith("R")
                and 19.9 < q["c"][0] < 24.0 and 111.8 < q["c"][1] < 123.4), key=lambda q: q["c"][1])
    rv = (r112["c"][0], r112["box"][1] - R112_VIA_DY)
    assert rv[1] < 113.2 - 0.45, "the zener feed's resistor: its OUT42D pad must face the 42D planes (north)"
    lay(r112["net"], B, [r112["c"], rv], 0.5)
    via(r112["net"], rv)
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:20]:
        log(f"   boost outer clearance: {msg}")
    if bad:
        R.failed.append("boost outer")
    log(f"   boost outer (BOOST_DIS, LED, output, divider): {len(segs)} segments, {len(vias)} vias, "
        f"{len(bad)} clearance problems")


# ---- the brake comparator (decided 2026-11-22, by hand; the packer had IN+ / IN- parts on the wrong sides of U8).
# U8 (TLV1805, bottom) keeps its pins: inputs on top (IN- left, IN+ right), OUT bottom-left, V+ bottom-right. The IN+
# network (R115 from OUT42D, which taps the brake XT60's OUT42D pin right above it, R116 to GND, R117 from OUT) stands
# north of U8, OUT goes west to the gate resistor R118 and the gate via, V+ east to its cap C71, the zener D11 and the
# feed R112 / R113. OUT-to-IN+ and V+-to-IN- cross; IN- (the LM4040's low-impedance node) takes the gap between U8's
# pin columns, south to U9 and R114. BRAKE_ACT's status divider (R119 from the gate, R120 to GND) at the west edge.
# BRAKE_ACT (L3, esp_plan) crosses west above U9 at y 129.9, BOOST_DIS (L3) under the south row at y 133.15: no vias
# in those bands. Spec: (nets: top / left pad first), centre, upright?
CMP_IC = {"U8": (9.0, 127.45, 0), "U9": (8.745, 131.5, 180)}
CMP = [(("OUT42D", "Net-(U8-IN+)"), (6.6, 124.0), False),          # R115 (1206)
       (("Net-(U8-IN+)", "GND"), (11.6, 124.0), False),            # R116
       (("Net-(U8-IN+)", "Net-(U8-OUT)"), (5.6, 127.6), True),     # R117
       (("GND", "BRAKE_ACT"), (1.6, 127.25), True),                # R120
       (("BRAKE_ACT", "Net-(Q22-G)"), (1.6, 131.15), True),        # R119
       (("Net-(Q22-G)", "Net-(U8-OUT)"), (4.57, 130.6), False),    # R118
       (("Net-(U8-IN-)", "Net-(D11-K)"), (12.92, 131.65), False),  # R114
       (("GND", "Net-(D11-K)"), (12.8, 127.85), True),             # C71 (1206)
       (("Net-(D11-K)", "GND"), (17.0, 128.0), False),             # D11 (SOD-123)
       (("OUT42D", "Net-(R112-Pad2)"), (21.11, 115.16), True),     # R112 (1206): the zener feed
       (("Net-(R112-Pad2)", "Net-(D11-K)"), (21.09, 120.66), True)]   # R113
CMP_GND_VIAS = {"U8.2": (3.6, 127.6), "U8.5": (14.4, 126.29), "R116": (13.9, 124.0), "R120": (1.6, 125.2),
                "U9": (10.75, 132.3), "D11": (20.0, 128.0)}
CMP_IN_X = 9.0                         # IN- down between U8's pin columns


def place_comparator(FP, obstacles):
    """The brake comparator's parts by hand (locked). Returns the references placed."""
    import power54
    from power42 import _upright, _at
    from satellites import _box
    sheet = FP["U8"].GetSheetname()
    short = lambda p: p.GetNetname().split("/")[-1]
    done = []
    for ref, (x, y, deg) in CMP_IC.items():
        _at(FP[ref], x, y, deg, bottom=True)
        done.append(ref)
    for (a, b), (x, y), upright in CMP:
        ref = next(r for r, f in FP.items() if f.GetSheetname() == sheet and r[0] in "RCD" and r not in done
                   and {short(p) for p in f.Pads()} == {a, b})
        pa = next(p.GetNumber() for p in FP[ref].Pads() if short(p) == a)
        pb = next(p.GetNumber() for p in FP[ref].Pads() if short(p) == b)
        if upright:
            _upright(FP[ref], x, y, pa, pb, bottom=True)
        else:
            power54._flat_bottom(FP[ref], x, y, pa, pb)
        done.append(ref)
    for ref in done:
        FP[ref].SetLocked(True)
        obstacles.append(_box(FP[ref]))
    return done


def comparator(R, log=print):
    """The brake comparator's copper, planned (checked exactly)."""
    import far42
    import route_plan
    from route import B
    short = lambda q: q["net"].split("/")[-1]
    P = lambda ref, n: next(q for q in R.geo.pads if q["ref"] == ref and short(q) == n)
    by = lambda n, pre: next(q for q in R.geo.pads if short(q) == n and q["ref"].startswith(pre))
    ref_of = lambda a, b: next(q["ref"] for q in R.geo.pads if short(q) == a and q["ref"][0] in "RCD"
                               and any(short(o) == b for o in R.geo.pads if o["ref"] == q["ref"]))
    segs, vias = [], []

    def lay(net, pts, w=W):
        segs.extend(R.track(net, B, pts, w))

    def via(net, xy):
        vias.append(R.via(net, *xy))

    inp, inn, out, vp, gate = "Net-(U8-IN+)", "Net-(U8-IN-)", "Net-(U8-OUT)", "Net-(D11-K)", "Net-(Q22-G)"
    u8 = {q["num"]: q for q in R.geo.pads if q["ref"] == "U8"}
    r115, r116, r117 = ref_of("OUT42D", inp), ref_of(inp, "GND"), ref_of(inp, out)
    r118, r119, r120 = ref_of(gate, out), ref_of("BRAKE_ACT", gate), ref_of("BRAKE_ACT", "GND")
    r114 = ref_of(inn, vp)
    c71 = next(q["ref"] for q in R.geo.pads if q["ref"].startswith("C") and short(q) == vp)
    d11 = next(q["ref"] for q in R.geo.pads if q["ref"].startswith("D") and short(q) == vp)
    r112, r113 = ref_of("OUT42D", "Net-(R112-Pad2)"), ref_of("Net-(R112-Pad2)", vp)
    o = P(r115, "OUT42D")["c"]
    j4 = min((q for q in R.geo.pads if short(q) == "OUT42D" and q["ref"].startswith("J")
              and q["box"][0] < o[0] < q["box"][2] and q["box"][3] < o[1]), key=lambda q: o[1] - q["box"][3])
    c = lambda ref, n: P(ref, n)["c"]
    # OUT42D: R115 straight up into the brake XT60's OUT42D pin
    jc = j4["c"]                                       # (the pin's pad is round: end at its centre)
    yb = jc[1] + (jc[0] - o[0])
    lay(P(r115, "OUT42D")["net"], [o, (o[0], yb), jc], 0.5)
    # IN+: R117's top pad round R115's east pad, R115 to R116, pin 4 up into that line
    t, e, w16, p4 = c(r117, inp), c(r115, inp), c(r116, inp), u8["4"]["c"]
    yl = P(r115, inp)["box"][3] + 0.725
    n = P(r117, inp)["net"]
    lay(n, [t, (t[0], yl + 0.35), (t[0] + 0.35, yl), (e[0] - 0.35, yl), (e[0], yl - 0.35), e])
    lay(n, [e, w16])
    lay(n, [p4, (p4[0], e[1])])
    # OUT: pin 1 west into R117's bottom pad, on down into R118's east pad
    o1, b17, e18 = u8["1"]["c"], c(r117, out), c(r118, out)
    lay(u8["1"]["net"], [o1, (b17[0], o1[1])])
    lay(u8["1"]["net"], [(b17[0], o1[1]), (b17[0], e18[1])])
    # the gate: R118's west pad and R119's bottom pad to the brake FET's gate via
    gv = next(v for v in R.geo.vias if v[0].split("/")[-1] == gate)[1:3]
    lay(P(r118, gate)["net"], [c(r118, gate), gv])
    lay(P(r118, gate)["net"], [gv, c(r119, gate)])
    # BRAKE_ACT between the divider's resistors (its line comes in on L3, esp_plan.brake)
    lay(P(r119, "BRAKE_ACT")["net"], [c(r120, "BRAKE_ACT"), c(r119, "BRAKE_ACT")])
    # IN-: pin 3 east into the gap between the pin columns, down and out to U9's cathode, on to R114
    i3, k9, w14 = u8["3"]["c"], c("U9", inn), c(r114, inn)
    yo = k9[1] - (k9[0] - CMP_IN_X)
    lay(u8["3"]["net"], [i3, (CMP_IN_X, i3[1]), (CMP_IN_X, yo), k9])
    xk = w14[0] - (w14[1] - k9[1])
    lay(u8["3"]["net"], [k9, (xk, k9[1]), w14])
    # V+: pin 6 to C71's bottom pad, on to R114, to D11's cathode, D11 up and east to the feed R113
    v6, vc, e14, k11, f13 = u8["6"]["c"], c(c71, vp), c(r114, vp), c(d11, vp), c(r113, vp)
    n = u8["6"]["net"]
    lay(n, [v6, (vc[0] - (vc[1] - v6[1]), v6[1]), vc])
    lay(n, [vc, (e14[0], vc[1] + (e14[0] - vc[0])), e14])
    lay(n, [vc, (k11[0] - (vc[1] - k11[1]), vc[1]), k11])
    yv = k11[1] - 2.0
    lay(n, [k11, (k11[0], yv), (f13[0] - (yv - f13[1] - 2.2), yv), (f13[0], f13[1] + 2.2), f13])
    # the feed: R112 to R113
    lay(P(r112, "Net-(R112-Pad2)")["net"], [c(r112, "Net-(R112-Pad2)"), c(r113, "Net-(R112-Pad2)")])
    # GND: pin 5 and C71's top pad, pin 2 west between R117's pads, U9's anode, D11's anode, R116, R120: vias
    g5, gc = u8["5"]["c"], c(c71, "GND")
    lay("GND", [g5, (gc[0] - (g5[1] - gc[1]) - 0.34, g5[1]), (gc[0] - 0.34, gc[1]), gc, CMP_GND_VIAS["U8.5"]], 0.4)
    g2 = u8["2"]["c"]
    gy = CMP_GND_VIAS["U8.2"][1]
    lay("GND", [(g2[0], gy), CMP_GND_VIAS["U8.2"]], 0.25)
    lay("GND", [c("U9", "GND"), CMP_GND_VIAS["U9"]], 0.4)
    lay("GND", [c(d11, "GND"), CMP_GND_VIAS["D11"]], 0.4)
    lay("GND", [c(r116, "GND"), CMP_GND_VIAS["R116"]], 0.4)
    lay("GND", [c(r120, "GND"), CMP_GND_VIAS["R120"]], 0.4)
    for k, xy in CMP_GND_VIAS.items():
        via("GND", xy)
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:20]:
        log(f"   brake comparator clearance: {msg}")
    if bad:
        R.failed.append("brake comparator")
    log(f"   brake comparator: {len(segs)} segments, {len(vias)} vias, {len(bad)} clearance problems")


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
