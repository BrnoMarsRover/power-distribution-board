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
PORT_X = {"MOTOR_OK": 137.6, "PWR_HOLD": 136.6}  # the bundle's corner ports: the router takes them on to their pads
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
    """MOTOR_OK and PWR_HOLD from the bundle's ports to their pads in the corner, by the router (after the bundle is
    laid and checked)."""
    from route import IN2
    for key in ("MOTOR_OK", "PWR_HOLD"):
        net = key
        port = dict(points=[(IN2, PORT_X[key], WEST_Y[key])], pads=set(), segs=[], vias=[])
        pads = [R.t_pad(q["ref"], q["num"]) for q in R.geo.pads
                if q["net"] == net and q["ref"].startswith("R") and q["c"][1] < CORNER_BOX[3]]
        if not R.route_net(net, pads + [port], W, CORNER_BOX, label=f"{net} in the corner"):   # (pads seed the
            log(f"   {net}: no way from the bundle's port to its pads")                           # tree, the port joins)


# ---- the group's own nets (pcb_route.py espnet, after the switch-control group): planned escapes, then the router
ESC_LEFT_X, ESC_BOTTOM_Y = 131.6, 80.4           # the left column's vias just inside the edge, the bottom row's outside
LEFT_PINS = ("19", "20", "21", "23", "24", "25")  # CAN x3, LED x3
BOTTOM_PINS = ("31", "36", "37", "38", "39")      # spare, RXD0, TXD0, AUX5 and NTC sense (32 BRAKE_ACT: its own plan)
IO0_VIA = (130.1, 80.4)                          # IO0: beside pin 27, a top-side stub on to the BOOT switch
EN_VIA, EN_SW_VIA = (145.38, 62.55), (136.9, 84.5)   # EN: under the module edge; at the RESET switch (under its body)
V3_VIA = (148.4, 63.05)                          # +3V3: pin 2 east under the module edge, right of the north
GND_VIAS = {"1": (148.4, 61.9), "40": (147.92, 80.4)}   # bundle's L3 columns, then a bottom stub to U12's tab
KEEP_ESC = [(130.95, 67.2, 132.25, 76.2), (129.45, 79.75, 147.5, 81.05), (136.25, 83.85, 137.55, 85.15)]
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
    # EN: pin 3 -> a via under the module edge; a via at the RESET switch with a top-side stub to its pad
    q = pin["3"]
    (px, py), net = q["c"], q["net"]
    lay(net, F, [(px, py), EN_VIA])
    via(net, *EN_VIA)
    sw = min((s for s in R.geo.pads if s["net"] == net and s["ref"].startswith("SW")), key=lambda s: math.dist(s["c"], EN_SW_VIA))
    (sx, sy), (vx, vy) = sw["c"], EN_SW_VIA
    via(net, vx, vy)
    lay(net, F, [(vx, vy), (vx, sy + (vx - sx)), (sx, sy)])
    esc[net] = [EN_VIA, EN_SW_VIA]
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
    esc[pin["3"]["net"]] = [EN_VIA, EN_SW_VIA]
    return esc


def local(R, FP, log=print):
    """The ESP32 group's own nets, by the router from their escapes to the group's parts (bottom-side pads; the
    switches' pads are reached by the escapes' top-side stubs)."""
    import route
    from route import B
    esc = escape_points(R)
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
