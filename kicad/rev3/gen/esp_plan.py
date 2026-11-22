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

KEEP_TOP: the band above the module's top pin row (its pins' escapes: the FLT bus's pin vias and these lines) and
the resistors with their tracks; KEEP_BOTTOM: the vias south of U15, in the aux network's pocket.
"""
W, ESD_W = 0.25, 0.4
KEEP_TOP = [(129.2, 55.5, 149.4, 60.3),          # above the module's top pin row (pads from y 60.3)
            (133.0, 49.3, 142.4, 55.5),          # the UART resistors and their tracks to the corridor vias
            (139.8, 37.2, 141.1, 38.6), (140.85, 39.95, 142.15, 41.25), (143.15, 39.3, 144.45, 40.6)]  # U15's vias
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
SP_X = 146.2                                     # SP: north-east to x 146.2, north to J33's pin 4
U15_V3 = 34.3                                    # U15's +3V3 via, between its connector lines
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
            lay(side, IN2, [(vx, y), (SP_X, y - (SP_X - vx)), (SP_X, jy + 2.0), (jx, jy + 2.0 - (jx - SP_X)),
                            (jx, jy)])
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

    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:30]:
        log(f"   host UART clearance: {msg}")
    if bad:
        R.failed.append("host UART")
    log(f"   host UART: {len(segs)} segments, {len(vias)} vias, {len(bad)} clearance problems")
    return segs, vias
