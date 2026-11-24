"""The ESP32 group's routing plan (in progress, 2026-11-22).

The host UART (decided 2026-11-22): the ESD array U15 sits at J33 (bottom side, below its pins 1 / 2), the 1 k
series resistors R151-R153 at the module, on the top side right north of their pins (the top side north of the
module is empty; the bottom there is the aux supply's). The module's pins: 12 TX, 10 RX, 8 SP (TX west of RX, as at
U15: no crossing anywhere). Each line:

  pin -> top track straight north -> its resistor (upright, MCU pad south)
  -> top track north and east (nested: TX outermost) -> a via in the corridor right of the aux inductor L2
  -> L3 north (under the aux network's parts, clear of the buck's switch node)
  -> TX / RX: a via south of U15 and a bottom stub into its pin 3 / 1;  SP: straight into J33's pin 4 (through-hole)

U15's connector-side pins 4 / 6 go straight up to J33's pins 1 / 2 (0.4 mm, the discharge's path), its supply pin
to a +3V3 via between them, its GND pin to a via below it.

The north bundle (decided 2026-11-22: MOTOR_OK moved to pin 35): PWR_HOLD, MOTOR_OK and the power button's sense line
leave the module's bottom row together (pins 33, 35; PWR_BTN's pin 34 goes through its RC filter R175 / C120, placed
right there, so the bundle's third line is BTN_SNS) and run on L3 as one bundle to the top-right corner:

  vias inside the module's bottom edge -> nested runs east under the module -> north up the board's right side
  (right of the I2C path, under U12 and the aux diodes) -> a jog right past the aux supply's GND via -> north right
  of J33's, J36's pins -> west between J36 and J37 into the corner, each line peeling off in order:
  BTN_SNS north into J37's pin 3 (through-hole), MOTOR_OK to a via at R167, PWR_HOLD to vias at R172 and R173.

KEEP_TOP: the band above the module's top pin row (its pins' escapes: the FLT bus's pin vias and these lines) and
the resistors with their tracks; KEEP_BOTTOM: the vias south of U15, in the aux network's pocket.
"""
W, ESD_W = 0.25, 0.4
KEEP_TOP = [(129.2, 55.5, 149.4, 60.3),          # above the module's top pin row (pads from y 60.3)
            (133.0, 49.3, 142.4, 55.5),          # the UART resistors and their tracks to the corridor vias
            (139.8, 37.2, 141.1, 38.6), (140.85, 39.95, 142.15, 41.25), (143.15, 39.3, 144.45, 40.6),  # U15's vias
            (129.45, 79.75, 147.5, 81.05)]       # the module's bottom-row escape vias, just outside it
KEEP_MODULE = [(138.4, 72.4, 146.2, 78.3)]       # the bundle's vias, PWR_BTN's RC and stubs, under the module
KEEP_BOTTOM = [(139.8, 37.2, 141.1, 38.6),       # U15's vias: TX beside pin 3 (in the power stage's kept area),
               (140.85, 39.95, 142.15, 41.25),   # GND down-left of pin 2,
               (143.15, 39.3, 144.45, 40.6)]     # RX under pin 1; the pocket right of U15 stays whole

NETS = ("UART_TX", "UART_RX", "UART_SP")         # west to east
RES_Y = 53.0                                     # the resistors' centres (upright)
LEVEL_Y = {"UART_TX": 49.9, "UART_RX": 50.9, "UART_SP": 51.9}    # top tracks east, nested; the corridor vias' y
VIA_X = {"UART_TX": 140.1, "UART_RX": 140.9, "UART_SP": 141.7}   # the corridor (right of L2's outline, 139.29)
# U15's vias, none right of it (the aux network's resistors need that pocket whole): TX left of pin 3 (inside the
# power stage's kept area), RX straight under pin 1, GND down-left of pin 2
U15_VIA = {"UART_TX": (140.45, 37.9), "UART_RX": (143.8, 39.95), "GND": (141.5, 40.6)}
U15_V3 = 34.3                                    # U15's +3V3 via, between its connector lines
SP_COL = (141.7, 43.3, 145.0)                    # SP: north from its via to y 43.3, north-east to x 145, north

# the north bundle, west to east heading north
BUNDLE = ("PWR_HOLD", "MOTOR_OK", "BTN")         # (BTN: the power button's sense line, after R175)
MOD_VIA_Y = 76.9                                 # the bundle's vias inside the module's bottom edge
RUN_Y = {"PWR_HOLD": 74.5, "MOTOR_OK": 75.3, "BTN": 76.1}         # nested runs east under the module
COL_X = {"PWR_HOLD": 146.3, "MOTOR_OK": 146.8, "BTN": 147.3}      # north under the module's right side
JOG_Y, JOG_DX = {"PWR_HOLD": 45.3, "MOTOR_OK": 45.6, "BTN": 45.9}, 1.3   # right, past the aux GND via (148.2, 46.23)
WEST_Y = {"PWR_HOLD": 18.4, "MOTOR_OK": 17.5, "BTN": 16.6}        # west between J36 and J37 (0.9 pitch: vias)
R175_C, C120_C = (143.6, 75.5), (143.6, 73.4)    # PWR_BTN's RC at its pin (bottom side), the PWR_BTN pads left
BTN_DETOUR_Y = 77.75                             # PWR_BTN: under the pin row, past MOTOR_OK's via, up to the RC
C120_GND_VIA = (145.4, 72.9)
import corner_plan                                # the bundle's corner ports: corner_plan lays the rest (by hand)
PORT_X = corner_plan.PORT_X
CORNER_BOX = (125.5, 0.6, 139.2, 20.0)           # (the corner group packs itself: its pads move)
J33_TURN = {"4": 34.0, "6": 33.3}                # U15's connector lines: straight up to here, then north-west


def _pad(FP, ref, net):
    return next(p for p in FP[ref].Pads() if p.GetNetname() == net)


def _res(FP, net):
    """The series resistor on a UART net (its other net is the U15 / J33 side)."""
    out = [r for r, f in FP.items() if r.startswith("R") and net in {p.GetNetname() for p in f.Pads()}]
    assert len(out) == 1, f"ESP plan: resistor on {net}: {out}"
    return out[0]


def place(FP, obstacles):
    """The UART's series resistors on the top side, upright in columns at their module pins, locked (KEEP_BOTTOM:
    logic_place, right after U15, before the aux network packs; KEEP_TOP: pcb_place, pcb_optimize). Returns the
    references placed."""
    import power42
    done = []
    for net in NETS:
        ref = _res(FP, net)
        mcu = _pad(FP, ref, net).GetNumber()
        other = next(p.GetNumber() for p in FP[ref].Pads() if p.GetNumber() != mcu)
        px, _ = power42._pxy(FP["U13"], _pad(FP, "U13", net).GetNumber())
        power42._upright(FP[ref], px, RES_Y, other, mcu, bottom=False)
        FP[ref].SetLocked(True)
        done.append(ref)
    # PWR_BTN's RC filter at its pin (bottom side, under the module): R175 (to the sense line) and C120
    from i2c_plan import _flat_b
    from satellites import _box
    r175 = next(r for r, f in FP.items() if r.startswith("R") and "PWR_BTN" in {p.GetNetname() for p in f.Pads()})
    c120 = next(r for r, f in FP.items() if r.startswith("C") and {p.GetNetname() for p in f.Pads()} == {"PWR_BTN", "GND"})
    for ref, (x, y) in ((r175, R175_C), (c120, C120_C)):
        a = _pad(FP, ref, "PWR_BTN").GetNumber()
        b = next(p.GetNumber() for p in FP[ref].Pads() if p.GetNumber() != a)
        _flat_b(FP[ref], x, y, a, b)
        FP[ref].SetLocked(True)
        obstacles.append(_box(FP[ref]))
        done.append(ref)
    obstacles += KEEP_MODULE + KEEP_ESC
    return done


def route(R, FP, log=print):
    """Lay the host UART. Returns the new segments and vias."""
    import far42
    import route_plan
    from route import B, F, IN2
    segs, vias = [], []

    def lay(net, layer, pts, w=W):
        segs.extend(R.track(net, layer, pts, w))

    def via(net, x, y):
        vias.append(R.via(net, x, y))

    c = lambda ref, num: R.pad(ref, num)["c"]
    pin = lambda ref, net: next(q for q in R.geo.pads if q["ref"] == ref and q["net"] == net)
    u15 = {q["num"]: q for q in R.geo.pads if q["ref"] == "U15"}
    j33 = {q["num"]: q for q in R.geo.pads if q["ref"] == "J33"}

    for net in NETS:
        ref = _res(FP, net)
        mcu = pin(ref, net)
        far = next(q for q in R.geo.pads if q["ref"] == ref and q is not mcu)
        (px, py), (mx, my), (fx, fy) = pin("U13", net)["c"], mcu["c"], far["c"]
        assert abs(px - mx) < 1e-3 and abs(mx - fx) < 1e-3 and fy < my, f"ESP plan: {ref} is off its column"
        lay(net, F, [(px, py), (mx, my)])                         # pin to resistor, straight north
        y, vx = LEVEL_Y[net], VIA_X[net]
        side = far["net"]                                         # the U15 / J33 side
        turn = [(fx, y + 0.5), (fx + 0.5, y)] if fy - y > 0.6 else [(fx + (fy - y), y)]
        lay(side, F, [(fx, fy)] + turn + [(vx, y)])
        via(side, vx, y)
        if net == "UART_SP":                                      # L3 to J33's pin 4
            jx, jy = j33["4"]["c"]
            x0, y1, x2 = SP_COL                                   # north, north-east past U15, north to the pin
            lay(side, IN2, [(vx, y), (x0, y1), (x2, y1 - (x2 - x0)), (x2, jy + 2.0),
                            (jx, jy + 2.0 - (jx - x2)), (jx, jy)])
            continue
        ux, uy = U15_VIA[net]
        num = next(n for n, q in u15.items() if q["net"] == side)
        lay(side, IN2, [(vx, y), (vx, uy + (ux - vx)), (ux, uy)])     # north, then north-east into the via
        via(side, ux, uy)
        (qx, qy) = u15[num]["c"]
        if abs(qx - ux) < 1e-6:                                   # straight under its pin
            lay(side, B, [(qx, qy), (ux, uy)])
        else:                                                     # 45 degrees off the pin, then level
            lay(side, B, [(qx, qy), (qx - abs(qy - uy), uy), (ux, uy)])
    # U15: GND below, +3V3 between its connector lines, the connector lines up to J33
    gx, gy = U15_VIA["GND"]
    (qx, qy) = u15["2"]["c"]
    lay("GND", B, [(qx, qy), (qx, gy - (qx - gx)), (gx, gy)], ESD_W)       # down, then 45 degrees left
    via("GND", gx, gy)
    vx, vy = u15["5"]["c"]
    lay("+3V3", B, [(vx, vy), (vx, U15_V3)], ESD_W)
    via("+3V3", vx, U15_V3)
    for num, jn in (("4", "1"), ("6", "2")):
        (ax, ay), (jx, jy) = u15[num]["c"], j33[jn]["c"]
        ty = J33_TURN[num]
        net = u15[num]["net"]
        assert net == j33[jn]["net"], f"ESP plan: U15.{num} / J33.{jn} on different nets"
        lay(net, B, [(ax, ay), (ax, ty), (jx, ty - (ax - jx)), (jx, jy)], ESD_W)

    s2, v2 = north(R, FP, lay, via)
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:30]:
        log(f"   host UART / north bundle clearance: {msg}")
    if bad:
        R.failed.append("host UART / north bundle")
    log(f"   host UART and north bundle: {len(segs)} segments, {len(vias)} vias, {len(bad)} clearance problems")
    return segs, vias


def north(R, FP, lay, via):
    """The north bundle (lay / via: route()'s, so its copper is checked with the UART's)."""
    from route import B, F, IN2
    c = lambda ref, num: R.pad(ref, num)["c"]
    at = lambda ref, net: next(q for q in R.geo.pads if q["ref"] == ref and q["net"] == net)
    r175 = next(q["ref"] for q in R.geo.pads if q["net"] == "PWR_BTN" and q["ref"].startswith("R"))
    btn = next(q["net"] for q in R.geo.pads if q["ref"] == r175 and q["net"] != "PWR_BTN")
    nets = {"PWR_HOLD": "PWR_HOLD", "MOTOR_OK": "MOTOR_OK", "BTN": btn}
    c120 = next(q["ref"] for q in R.geo.pads if q["net"] == "PWR_BTN" and q["ref"].startswith("C"))
    # the module end: PWR_HOLD / MOTOR_OK pins to vias inside the edge; PWR_BTN's pin to its via, under the pin row
    # to its RC; the sense line from R175 to its via
    start = {}
    for key in ("PWR_HOLD", "MOTOR_OK"):
        px, py = at("U13", nets[key])["c"]
        lay(nets[key], F, [(px, py), (px, MOD_VIA_Y)])
        via(nets[key], px, MOD_VIA_Y)
        start[key] = (px, MOD_VIA_Y)
    px, py = at("U13", "PWR_BTN")["c"]
    lay("PWR_BTN", F, [(px, py), (px, MOD_VIA_Y)])
    via("PWR_BTN", px, MOD_VIA_Y)
    (rx, ry), (cx, cy) = at(r175, "PWR_BTN")["c"], at(c120, "PWR_BTN")["c"]
    d = BTN_DETOUR_Y
    lay("PWR_BTN", B, [(px, MOD_VIA_Y), (px, d), (rx - (d - ry - 1.46), d), (rx, ry + 1.46), (rx, ry), (cx, cy)])
    bx, by = at(r175, btn)["c"]
    lay(btn, B, [(bx, by), (bx, MOD_VIA_Y)])
    via(btn, bx, MOD_VIA_Y)
    start["BTN"] = (bx, MOD_VIA_Y)
    gx, gy = next(q["c"] for q in R.geo.pads if q["ref"] == c120 and q["net"] == "GND")
    lay("GND", B, [(gx, gy), C120_GND_VIA], ESD_W)
    via("GND", *C120_GND_VIA)
    # the bundle on L3
    j37 = {q["num"]: q for q in R.geo.pads if q["ref"].startswith("J") and q["net"] == btn}
    (tx, ty), = [q["c"] for q in j37.values()]
    for key in BUNDLE:
        vx, vy = start[key]
        ry_, cx_, jy, wy = RUN_Y[key], COL_X[key], JOG_Y[key], WEST_Y[key]
        ex = cx_ + JOG_DX
        pts = [(vx, vy), (vx, ry_ + 0.5), (vx + 0.5, ry_), (cx_ - 0.5, ry_), (cx_, ry_ - 0.5), (cx_, jy),
               (ex, jy - JOG_DX), (ex, wy + 0.5), (ex - 0.5, wy)]
        if key == "BTN":                                          # north into J37's pin 3
            pts += [(tx + 0.5, wy), (tx, wy - 0.5), (tx, ty)]
        else:                                                     # to its port at the corner's edge
            pts += [(PORT_X[key], wy)]
        lay(nets[key], IN2, pts)
    return None, None


def corner(R, FP, log=print):
    """MOTOR_OK and PWR_HOLD from the bundle's ports on, with the rest of the power-button corner (corner_plan)."""
    corner_plan.route(R, FP, log=log)


# ---- the group's own nets (pcb_route.py espnet, after the switch-control group): planned escapes, then the router
ESC_LEFT_X, ESC_BOTTOM_Y = 131.6, 80.4           # the left column's vias just inside the edge, the bottom row's outside
LEFT_PINS = ("19", "20", "21", "23", "24", "25")  # CAN x3, LED x3
BOTTOM_PINS = ("31", "36", "37", "38", "39")      # spare, RXD0, TXD0, AUX5 and NTC sense (32 BRAKE_ACT: its own plan)
IO0_VIA = (130.1, 80.4)                          # IO0: beside pin 27, a top-side stub on to the BOOT switch
EN_VIA, EN_SW_VIA = (145.38, 62.55), (136.9, 84.5)   # EN: under the module edge; at the RESET switch (under its body)
V3_VIA = (148.08, 63.05)                          # +3V3: pin 2 east under the module edge, right of the north
GND_VIAS = {"1": (148.08, 61.9), "40": (147.92, 80.4)}   # bundle's L3 columns, then a bottom stub to U12's tab
KEEP_ESC = [(130.95, 67.2, 132.25, 76.2), (129.45, 79.75, 147.5, 81.05), (136.25, 83.85, 137.55, 85.15)]
# BRAKE_ACT (pin 32) to the brake comparator in the bottom-left corner, about 170 mm on L3, all planned: south beside
# the ESP32's group (east of the I2C trunk's end), west in the one long free L3 lane, between output rows 2 and 3
# (y 101.4, BOOST_DIS beside it at y 102.0, boost_plan), down the 42D plane's east edge and beside the switch-node
# shield (BOOST_DIS east of it), west above the comparator's south row to a via between its status divider's
# resistors (boost_plan.CMP: R120 above, R119 below), a bottom-side stub into the link between them
BRAKE_Y = 101.4                                  # (the run starts at pin 32's x)
BRAKE_PATH = [(24.8, 101.4), (24.4, 101.8), (24.4, 109.35), (23.0, 110.75), (23.0, 129.5), (22.6, 129.9), (3.7, 129.9)]
BRAKE_VIA = (3.0, 129.2)
LOCAL_BOX = (120.0, 58.0, 149.4, 126.0)


def escapes(R, FP, log=print):
    """Each of the group's module pins a planned escape (laid with the north bundle, before the rails' legs and the
    switch-control group, which route around them). Returns {net: [escape via points]} (also escape_points())."""
    import math
    import route_plan
    from route import B, F, IN2
    segs, vias = [], []

    def lay(net, layer, pts, w=W):
        segs.extend(R.track(net, layer, pts, w))

    def via(net, x, y):
        vias.append(R.via(net, x, y))

    pin = {q["num"]: q for q in R.geo.pads if q["ref"] == "U13"}
    esc = {}                                                      # net -> escape via
    for num in LEFT_PINS:
        q = pin[num]
        (px, py), net = q["c"], q["net"]
        lay(net, F, [(px, py), (ESC_LEFT_X, py)])
        via(net, ESC_LEFT_X, py)
        esc[net] = [(ESC_LEFT_X, py)]
    for num in BOTTOM_PINS:
        q = pin[num]
        (px, py), net = q["c"], q["net"]
        lay(net, F, [(px, py), (px, ESC_BOTTOM_Y)])
        via(net, px, ESC_BOTTOM_Y)
        esc[net] = [(px, ESC_BOTTOM_Y)]
    # IO0: pin -> via -> the BOOT switch's pad (top side); the router brings its pull-up to the via
    q = pin["27"]
    (px, py), net = q["c"], q["net"]
    vx, vy = IO0_VIA
    lay(net, F, [(px, py), (px, vy - (px - vx)), (vx, vy)])
    via(net, vx, vy)
    sw = min((s for s in R.geo.pads if s["net"] == net and s["ref"].startswith("SW")), key=lambda s: math.dist(s["c"], (vx, vy)))
    sx, sy = sw["c"]
    lay(net, F, [(vx, vy), (sx, vy + (vx - sx)), (sx, sy)])
    esc[net] = [(vx, vy)]
    # EN: pin 3 -> a via under the module edge (on to R142 / C107 / the RESET switch: esp_rest, 2026-11-23)
    q = pin["3"]
    (px, py), net = q["c"], q["net"]
    lay(net, F, [(px, py), EN_VIA])
    via(net, *EN_VIA)
    esc[net] = [EN_VIA]
    # +3V3: pin 2 straight down to U12's tab (0.5 mm); GND pins 1 / 40: a via each
    q = pin["2"]
    (px, py) = q["c"]
    tab = max((s for s in R.geo.pads if s["ref"] == "U12" and s["net"] == "+3V3"), key=lambda s: s["box"][1])  # (the tab)
    vx, vy = V3_VIA
    lay("+3V3", F, [(px, py), (px, vy - 0.9), (px + 0.9, vy), (vx, vy)], 0.5)
    via("+3V3", vx, vy)
    tx = min(vx, tab["box"][2] - 0.4)
    lay("+3V3", B, [(vx, vy), (vx, tab["box"][1] + 0.4), (tx, tab["box"][1] + 0.4)], 0.5)
    for num, (gx, gy) in GND_VIAS.items():
        (qx, qy) = pin[num]["c"]
        lay("GND", F, [(qx, qy), (qx, gy - abs(gx - qx)), (gx, gy)] if abs(gx - qx) > 1e-6 else [(qx, qy), (gx, gy)], 0.4)
        via("GND", gx, gy)
    import far42
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:20]:
        log(f"   ESP32 escapes clearance: {msg}")
    if bad:
        R.failed.append("ESP32 escapes")
    log(f"   ESP32 escapes: {len(segs)} segments, {len(vias)} vias, {len(bad)} clearance problems")
    return esc


def escape_points(R):
    """{net: [escape via points]}, as escapes() laid them."""
    pin = {q["num"]: q for q in R.geo.pads if q["ref"] == "U13"}
    esc = {pin[n]["net"]: [(ESC_LEFT_X, pin[n]["c"][1])] for n in LEFT_PINS}
    esc.update({pin[n]["net"]: [(pin[n]["c"][0], ESC_BOTTOM_Y)] for n in BOTTOM_PINS})
    esc[pin["27"]["net"]] = [IO0_VIA]
    esc[pin["3"]["net"]] = [EN_VIA]
    return esc


def brake(R, FP, log=print):
    """BRAKE_ACT: pin 32's escape and the planned L3 run to the comparator's status divider."""
    import route_plan
    import far42
    from route import B, F, IN2
    q = next(q for q in R.geo.pads if q["ref"] == "U13" and q["net"] == "BRAKE_ACT")
    (px, py) = q["c"]
    segs = R.track("BRAKE_ACT", F, [(px, py), (px, ESC_BOTTOM_Y)], W)
    vias = [R.via("BRAKE_ACT", px, ESC_BOTTOM_Y)]
    segs += R.track("BRAKE_ACT", IN2, [(px, ESC_BOTTOM_Y), (px, BRAKE_Y - 0.5), (px - 0.5, BRAKE_Y)] + BRAKE_PATH
                    + [BRAKE_VIA], W)
    vias.append(R.via("BRAKE_ACT", *BRAKE_VIA))
    div = [p for p in R.geo.pads if p["net"] == "BRAKE_ACT" and p["ref"] != "U13"]
    xd = div[0]["c"][0]
    assert all(abs(p["c"][0] - xd) < 0.05 for p in div) and min(p["c"][1] for p in div) < BRAKE_VIA[1] \
        < max(p["c"][1] for p in div), "BRAKE_ACT: the status divider's resistors moved"
    segs += R.track("BRAKE_ACT", B, [BRAKE_VIA, (xd, BRAKE_VIA[1])], W)
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:20]:
        log(f"   BRAKE_ACT clearance: {msg}")
    if bad:
        R.failed.append("BRAKE_ACT")
    log(f"   BRAKE_ACT: {len(segs)} planned segments, {len(bad)} clearance problems")


def local(R, FP, log=print):
    """The ESP32 group's own nets, by the router from their escapes to the group's parts (bottom-side pads; the
    switches' pads are reached by the escapes' top-side stubs)."""
    import route
    from route import B
    esc = escape_points(R)
    import can_plan
    import esp_rest
    for net in can_plan.logic_nets(R) + esp_rest.logic_nets(R):   # (planned: can_plan, esp_rest)
        esc.pop(net, None)
    R.geo.route_keep, R.geo.via_keep, R.geo.hv_ok = [], [], {}
    N = route.Negotiator(R, LOCAL_BOX, origin=LOCAL_BOX[:2])
    for net, pts in esc.items():
        terms = [R.t_via(x, y) for x, y in pts]
        terms += [R.t_pad(q["ref"], q["num"]) for q in R.geo.pads
                  if q["net"] == net and q["ref"] != "U13" and not q["ref"].startswith("SW") and B in q["layers"]]
        N.add(net, terms, W, net)
    failed = N.run(log=log)
    N.polish(log=log)
    failed = sorted(set(failed) | set(N._exact(log)))
    if failed:
        R.failed += failed
        log(f"   unrouted: {', '.join(failed)}")
    n_v = {net: sum(1 for v in R.geo.vias if v[0] == net) for net in esc}
    log("   ESP32 nets, vias each: " + ", ".join(f"{k.split('/')[-1]} {v}" for k, v in sorted(n_v.items())))


# ---- USB-C (J32) and its ESD array U14 (decided 2026-11-22). J32's pads stand in one 0.5 mm column (top side, x
# 142.07-143.22), the receptacle's body behind them (east); no 0.9 mm via fits between the pads, and none goes under
# the body. North to south: GND, VBUS, CC2, SBU1, D+ B6, D- A7, D+ A6, D- B7, CC1, SBU2, VBUS, GND. Plan:
#  - D+ joined behind the pads (in the strip before the body), D- in front; so D+ leaves from B6 and D- from the
#    middle of its join, and every line leaves west on the top side to its own via, in the 1.3 mm band between the
#    pads and RESET's (SW1's) pads: VBUS north, CC2, D+, D-, CC1, VBUS south;
#  - bottom side: CC2 / CC1 east to their 5.1 k resistors under the pads, D+ through U14's channel (between its pin
#    rows) into pin 3, D- round it into pin 1; U14's GND pin to a via south of it, its VBUS pin to a via north of it
#    (between the DP / DN exits) and L3 to the north VBUS via;
#  - both VBUS pads joined on L3, under the pad column;
#  - GND pads into the shield tabs (top side; the tabs are plated through to the GND plane);
#  - USB_DP / USB_DN: the module's pins 14 / 13 to vias under its corner, the router from there to U14's pins 4 / 6.
# VBUS_USB on to the OR diode D17: since 2026-11-23 D17 sits just south of the receptacle (logic5v).
USB_J32 = {"A9": (142.645, 84.365), "B5": (142.645, 85.015), "B6": (142.645, 86.015), "A7": (142.645, 86.515),
           "A6": (142.645, 87.015), "B7": (142.645, 87.515), "A5": (142.645, 88.015), "A4": (142.645, 89.165),
           "A12": (142.62, 83.565), "A1": (142.62, 89.965)}
USB_U14 = {"1": (139.696, 89.033), "3": (137.796, 89.033), "4": (137.796, 86.758), "6": (139.696, 86.758)}
USB_JOIN_DP_X = 143.55                            # D+ behind the pads (0.33 past their far ends)
USB_JOIN_DN_X = 141.64                            # D- in front (0.43 before their near ends)
USB_VIA = {"VBUS_N": (141.4, 83.85), "CC2": (141.35, 85.015), "D+": (140.75, 86.015), "D-": (141.25, 87.2),
           "CC1": (141.4, 88.55), "VBUS_S": (141.3, 90.35), "U14_VBUS": (138.75, 85.2), "U14_GND": (138.75, 90.6),
           "DP": (131.75, 64.15), "DN": (132.9, 64.6)}
USB_DP_B = [(140.33, 86.435), (140.33, 87.6), (140.03, 87.9), (138.1, 87.9), (137.8, 88.2), (137.8, 88.6)]
USB_DN_B = [(140.82, 87.63), (140.82, 88.0), (140.32, 88.5), (139.7, 88.5)]
USB_VBUS_W = 0.3                                  # the VBUS stubs out of the pad column (0.4 on L3)
USB_VBUS_L3 = [(142.55, 83.85), (142.55, 89.4), (141.6, 90.35)]
# USB_DP / USB_DN as a pair on the bottom side, 0.5 mm apart (DN on the east side): from the module's vias south-east
# between the left-column escapes and R156, south west of the module's decap C105 (crossing the I2C rows, never
# along an L3 line), south-east under the module's bottom row, west of the PWR_HOLD via, through the escape row east
# of BRAKE_ACT's via, then DP west and DN east round U14's VBUS via into pins 4 / 6
USB_DP_PATH = [(131.75, 64.15), (131.75, 64.81), (134.9, 67.96), (134.9, 73.3), (137.4, 75.8), (137.4, 77.5),
               (138.6, 78.7), (138.6, 84.0), (137.8, 84.8), (137.8, 86.5)]
USB_DN_PATH = [(132.9, 64.6), (132.9, 65.253), (135.4, 67.753), (135.4, 73.093), (137.9, 75.593), (137.9, 77.293),
               (139.1, 78.493), (139.1, 84.0), (139.7, 84.6), (139.7, 86.5)]
# MOTOR_OK's latch side (C114 / D21 / R169, west of the module) to pin 35: on the top side, from pin 35's pad south and
# west along the module's south edge (between the bottom row's escape vias and the BOOT / RESET pads) to a via by
# BOOT; the router (switch control) takes it on from there. So it crosses neither the USB pair nor the I2C lines
MOK_LANE_Y = 81.3
MOK_VIA = (130.9, 81.4)
# bottom side kept from the packer (after U14 is placed): the via band, U14's way north, its GND via
USB_KEEP = [(140.2, 82.9, 141.95, 91.0), (137.2, 81.1, 140.2, 86.0), (138.2, 90.0, 139.3, 91.2)]


# the ESP32 support group's small parts, frozen where the packer had put them on 2026-11-22 (reviewed and routed
# that way: USB, LEDs, EN / IO0, NTC, AUX5, CC). Its packer re-shuffles them whenever anything near the module
# changes (2026-11-23: the CAN pin swap put R162 across the CAN lines and C105 on USB_DN). ref: (x, y, deg, nets)
SUPPORT = {"C104": (143.35, 68.289, 0.0, "+3V3 GND"),    # (the aux sheet's spare +3V3 cap, packed with these)
           "C105": (136.657, 70.027, -90.0, "+3V3 GND"), "C106": (147.254, 68.277, 0.0, "+3V3 GND"),
           "C107": (135.62, 87.463, -90.0, "ESP_EN GND"), "C108": (147.901, 73.65, -90.0, "NTC_SNS GND"),
           "R142": (134.949, 82.503, 180.0, "+3V3 ESP_EN"), "R143": (130.188, 86.722, 180.0, "+3V3 ESP_IO0"),
           "R146": (146.621, 70.641, 180.0, "NTC_SNS GND"), "R147": (147.556, 87.318, 90.0, "+5V_LOGIC AUX5_SNS"),
           "R148": (146.27, 84.321, 180.0, "AUX5_SNS GND"), "R149": (145.404, 88.184, -90.0, "CC1 GND"),
           "R150": (143.234, 86.398, -90.0, "CC2 GND"), "R159": (135.553, 78.457, 0.0, "+3V3 SPARE_IO38"),
           "R160": (128.669, 73.278, 180.0, "LED_1 Net-(D18-A)"), "R161": (128.74, 75.333, 180.0, "LED_2 Net-(D19-A)"),
           "R162": (129.417, 77.485, 180.0, "LED_3 Net-(D20-A)"), "TH1": (147.545, 77.542, -90.0, "+3V3 NTC_SNS")}
TP_LOOP = ("TP41", (128.795, 94.895, 0.0))       # the group's GND test loop (top side), clear of ALERT's vias
KEEP_TOP_LOOP = [(126.6, 93.5, 131.0, 96.3)]      # (pcb_place / pcb_optimize: its courtyard, for the top packer)


def place_support(FP, obstacles):
    """The support group's parts at their frozen spots (bottom side) and its GND loop (top), locked. Returns the
    references placed."""
    import power42
    from satellites import _box
    done = []
    for ref, (x, y, deg, nets) in SUPPORT.items():
        have = sorted(p.GetNetname().split("/")[-1] for p in FP[ref].Pads() if p.GetNumber())
        if have != sorted(nets.split()):
            raise SystemExit(f"esp_plan.SUPPORT: {ref} is on {have}, not {nets} (references renumbered?)")
        power42._at(FP[ref], x, y, deg, bottom=True)
        obstacles.append(_box(FP[ref]))
        done.append(ref)
    ref, (x, y, deg) = TP_LOOP
    power42._at(FP[ref], x, y, deg)
    done.append(ref)
    import esp_rest
    obstacles += esp_rest.LED_KEEP_BOTTOM            # the status LEDs' fan (the switch-control packer comes later)
    for r in done:
        FP[r].SetLocked(True)
    return done


def usb_keep(obstacles):
    """After U14 is placed (logic_place): the bottom-side spots of the USB plan, kept from the packer."""
    obstacles += USB_KEEP


def usb(R, FP, log=print):
    """J32's fan-out, U14's links and the USB pair to the module (planned), USB_DP / USB_DN by the router."""
    import math
    import far42
    import route_plan
    from route import B, F, IN2
    pad = {q["num"]: q for q in R.geo.pads if q["ref"] == "J32" and q["num"]}
    for num, (x, y) in USB_J32.items():
        assert math.hypot(pad[num]["c"][0] - x, pad[num]["c"][1] - y) < 0.01, f"ESP plan: J32 pad {num} moved"
    u14 = {q["num"]: q for q in R.geo.pads if q["ref"] == "U14"}
    for num, (x, y) in USB_U14.items():
        assert math.hypot(u14[num]["c"][0] - x, u14[num]["c"][1] - y) < 0.01, f"ESP plan: U14 pin {num} moved"
    net = {k: pad[n]["net"] for k, n in (("D+", "B6"), ("D-", "A7"), ("CC1", "A5"), ("CC2", "B5"))}
    vbus, gnd = pad["A9"]["net"], "GND"
    c = {n: pad[n]["c"] for n in USB_J32}
    v = USB_VIA
    segs, vias = [], []
    # top side: the joins, the stubs to the band's vias, the GND pads into the shield tabs
    segs += R.track(net["D+"], F, [c["B6"], (USB_JOIN_DP_X, c["B6"][1]), (USB_JOIN_DP_X, c["A6"][1]), c["A6"]], W)
    segs += R.track(net["D-"], F, [c["A7"], (USB_JOIN_DN_X, c["A7"][1]), (USB_JOIN_DN_X, v["D-"][1]),
                                   (USB_JOIN_DN_X, c["B7"][1]), c["B7"]], W)
    segs += R.track(net["D-"], F, [(USB_JOIN_DN_X, v["D-"][1]), v["D-"]], W)
    segs += R.track(net["D+"], F, [c["B6"], v["D+"]], W)
    segs += R.track(net["CC2"], F, [c["B5"], v["CC2"]], W)
    segs += R.track(net["CC1"], F, [c["A5"], (141.75, c["A5"][1]), (v["CC1"][0], 88.365), v["CC1"]], W)
    segs += R.track(vbus, F, [c["A9"], (141.85, c["A9"][1]), v["VBUS_N"]], USB_VBUS_W)
    segs += R.track(vbus, F, [c["A4"], (142.05, c["A4"][1]), (v["VBUS_S"][0], 89.915), v["VBUS_S"]], USB_VBUS_W)
    for num, ty in (("A12", 82.45), ("A1", 91.1)):
        segs += R.track(gnd, F, [c[num], (c[num][0], ty)], 0.5)
    for k, n in (("VBUS_N", vbus), ("CC2", net["CC2"]), ("D+", net["D+"]), ("D-", net["D-"]), ("CC1", net["CC1"]),
                 ("VBUS_S", vbus), ("U14_VBUS", vbus), ("U14_GND", gnd)):
        vias.append(R.via(n, *v[k]))
    # bottom side: D+ through U14's channel into pin 3, D- round it into pin 1, U14's VBUS and GND pins to their vias
    segs += R.track(net["D+"], B, [v["D+"]] + USB_DP_B, W)
    segs += R.track(net["D-"], B, [v["D-"]] + USB_DN_B, W)
    segs += R.track(vbus, B, [u14["5"]["c"], v["U14_VBUS"]], W)
    segs += R.track(gnd, B, [u14["2"]["c"], v["U14_GND"]], 0.4)
    # L3: U14's VBUS via to the north VBUS via; the two VBUS vias joined under the pad column
    segs += R.track(vbus, IN2, [v["U14_VBUS"], (140.1, v["VBUS_N"][1]), v["VBUS_N"]], W)
    segs += R.track(vbus, IN2, [v["VBUS_N"]] + USB_VBUS_L3 + [v["VBUS_S"]], 0.4)
    # the module's USB pins: top-side stubs south under its corner to their vias; the pair on the bottom side to U14
    for key, net_, path in (("DP", "USB_DP", USB_DP_PATH), ("DN", "USB_DN", USB_DN_PATH)):
        q = next(q for q in R.geo.pads if q["ref"] == "U13" and q["net"] == net_)
        (px, py), (vx, vy) = q["c"], v[key]
        assert path[0] == (vx, vy), "ESP plan: the USB pair starts at the module's vias"
        segs += R.track(net_, F, [(px, py), (px, vy - (vx - px)), (vx, vy)], W)
        vias.append(R.via(net_, vx, vy))
        segs += R.track(net_, B, path, W)
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:20]:
        log(f"   USB-C clearance: {msg}")
    if bad:
        R.failed.append("USB-C fan-out")
    # the router: CC1 / CC2 from their vias to the resistors
    for k in ("CC2", "CC1"):
        r = next(q for q in R.geo.pads if q["net"] == net[k] and q["ref"] != "J32")
        if not R.route_net(net[k], [R.t_pad(r["ref"], r["num"]), R.t_via(*v[k])], W, (139.5, 83.0, 146.0, 91.0),
                           label=f"{k} (USB-C)"):
            log(f"   {k}: no route")
    log(f"   USB-C: {len(segs)} planned segments, {len(vias)} vias, {len(bad)} clearance problems")



def motor_ok_link(R, log=print):
    """MOTOR_OK's top-side lane from pin 35 to its via by BOOT (the switch-control group routes the rest)."""
    import far42
    import route_plan
    from route import F
    q = next(q for q in R.geo.pads if q["ref"] == "U13" and q["net"] == "MOTOR_OK")
    (px, py) = q["c"]
    segs = R.track("MOTOR_OK", F, [(px, py), (px, MOK_LANE_Y), (MOK_VIA[0] + 0.1, MOK_LANE_Y), MOK_VIA], W)
    vias = [R.via("MOTOR_OK", *MOK_VIA)]
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:10]:
        log(f"   MOTOR_OK lane clearance: {msg}")
    if bad:
        R.failed.append("MOTOR_OK lane")
    log(f"   MOTOR_OK lane: {len(segs)} segments, {len(bad)} clearance problems")
