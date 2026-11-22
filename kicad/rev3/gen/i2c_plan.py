"""The I2C bus and the INA228's ALERT (decided 2026-11-22: hand-planned, laid out for the bus's function first).

Seven devices hang on SCL / SDA: the INA228 (its link ends at the seam, ina_link), the ADCs U20 / U21, the I/O
expander U17, the ESP32 (the master), the pull-ups and two test pads. ALERT runs from the INA228 to the ESP32's
pin 18 with its pull-up. The plan:

  the trunk     on L3 between output rows 1 and 2 (y 90.85 ... 92.55), from the column at the board's left to the
                ESP32: L2's GND plane above, the bottom GND fill below, nothing analog beside it. North to south
                SCL, ALERT, SDA: the quiet interrupt line between clock and data, 0.85 mm pitch. Under the ICs
                (y 71 ... 84) run the 42 V far end's IMON / VSNS lines and the rails' legs; the bus crosses them,
                it does not run with them.
  the column    the link's three ends (bottom layer, x 51.3 ... 52.3) onto L3 at the seam, one via each: SDA and
                ALERT straight down, SCL under them on the bottom layer to the column's east side (the trunk's
                order needs SCL there). Down x 51.62 ... 52.6, west of the 42 V far end's L3 hop, to the trunk.
  the branches  U20 and U17: a via under each I2C pin, L3 diagonally down across the 42 V bundle (it is on the
                bottom layer there), straight down to the trunk; the line arriving at the near trunk line tees
                into it, the other one hops over on the bottom layer (two vias). U21: its pins face the FLT lines'
                L3 diagonals, so both lines stay on the bottom layer, down through the gap between the output
                connectors' pads to a via on each trunk line.
  the ESP32     SCL / SDA turn north under the module between its bottom-row pins 28 ... 30 (unconnected: no
                escapes there), east under it (below its GND pad's through-hole) and up into vias under the module
                edge at pins 5 / 4 (the LDO U12 fills the bottom side above them). ALERT leaves the trunk before
                the turn: a via, a bottom-layer hop under SCL with its pull-up R4 on it, L3 north beside the module
                and a via under its edge at pin 18 (north of it run the FLT lines on L3).
  the test pads SCL / SDA from vias on the trunk down the bottom layer to the test-pad field (TP40 / TP39), the
                pull-ups R145 / R144 on them right below the trunk.
  the ICs       their GND pins between the bus pins tie under the IC body to a GND pin on the other row (U20 4 to
                11, U17 21 to 3, U21 4 to 9), so the bus pins' vias have room. U21: its address strap (A0, pin 7,
                between two GND pins) goes under the body to the west end, where both strap resistors now sit;
                its 100 nF lies under pins 5 ... 8, supply pin above one pad and GND pin 8 above the other.

Laid after the FLT bus, before the rails' legs (they are routed around it), as planned polylines checked exactly.
"""
import math

W, GND_W = 0.25, 0.3
TRUNK_Y = {"I2C_SCL": 90.85, "INA_ALERT": 91.7, "I2C_SDA": 92.55}
NETS = ("I2C_SCL", "I2C_SDA", "INA_ALERT")

# the column: from the link's ends (bottom layer) onto L3
COL_B = {"I2C_SDA": [(51.8, 48.6), (51.8, 49.45), (52.15, 49.8)],
         "INA_ALERT": [(52.3, 48.6), (53.3, 49.6)],
         "I2C_SCL": [(51.3, 48.6), (51.3, 50.9), (51.95, 51.55), (54.4, 51.55)]}
COL_L3 = {"I2C_SDA": [(52.15, 49.8), (51.62, 50.33), (51.62, 92.07), (52.1, 92.55)],
          "INA_ALERT": [(53.3, 49.6), (53.3, 50.4), (52.1, 51.6), (52.1, 91.2), (52.6, 91.7)],
          "I2C_SCL": [(54.4, 51.55), (52.6, 53.35), (52.6, 90.35), (53.1, 90.85)]}

# U20 / U17: the west bus pin's via straight below it, the east one's 0.28 mm east and 0.75 mm lower (the vias'
# 1.1 mm); L3 diagonally down to a column (x), straight down to the trunk. The line on the far trunk line (SDA)
# hops over SCL on the bottom layer (vias at HOP_Y and on the SDA line)
VIA_Y, VIA_Y2, JOG = 71.65, 72.4, 0.28
BRANCH = {"U20": {"I2C_SCL": 60.0, "I2C_SDA": 61.0, "gnd": ("4", "11")},
          "U17": {"I2C_SDA": 87.0, "I2C_SCL": 88.0, "gnd": ("21", "3")}}   # (left of the 12 V XT60's pad)
HOP_Y = 89.85                                    # the hops' north via: 1 mm north of SCL
# U21: bottom layer down through the output connectors' pad gap (x) to a via on each trunk line
U21_GAP = {"I2C_SCL": (71.2, 119.45), "I2C_SDA": (70.9, 120.6)}     # (where the jog starts, the gap's x)
U21_A0_Y, U21_GND_Y = 68.0, 65.6                 # A0 and GND 4 -> 9 under the body
STRAP_DX = 1.85                                  # the address strap's column: left of pin 8
STRAP_Y = {"+3V3": 65.9, "GND": 70.1}            # the +3V3 half above, the GND half below, A0 between them
STRAP_V3_Y = 63.65                               # the +3V3 half's via, up-left of its pad (laid before the legs)
CAP_DX, CAP_DY = 1.2, 2.355                      # U21's 100 nF (horizontal) from pin 8: pads under pins 8 and 5
# the ESP32: the turn north, east under the module (SCL y 72, SDA y 73), up the pins' x to the vias under its edge
ESP_TURN = {"I2C_SCL": (133.3, 72.0), "I2C_SDA": (134.3, 73.0)}
ESP_VIA_Y = 62.55
ALERT_X, ALERT_PIN_VIA = 131.75, (131.75, 67.45)
# the test pads' lines and the pull-ups (bottom layer)
TP_X = {"I2C_SCL": 125.7, "I2C_SDA": 124.75}
PU_Y = 94.0
PU_SPOTS = {"I2C_SCL": (127.6125, PU_Y), "I2C_SDA": (122.8375, PU_Y)}   # centres: the bus pad 0.9125 mm in
R4_SPOT = (ALERT_X - 0.9125, (HOP_Y + TRUNK_Y["INA_ALERT"]) / 2)       # its ALERT pad on the hop

# kept from the packers: early (the ESP32 / CAN areas) and after the switch-control ICs (near their pins)
KEEP_EARLY = [(131.1, 66.8, 132.4, 68.1),                    # ALERT's via at pin 18
              (131.1, 89.2, 132.4, 92.35),                   # its hop
              (124.3, 90.2, 126.15, 111.5)]                  # the test pads' lines
KEEP_SW = [(69.9, 70.9, 72.0, 73.0),                         # U20's pin vias
           (91.5, 70.9, 93.6, 73.0),                         # U17's
           (117.2, 70.9, 121.0, 74.6),                       # U21's lines
           (112.85, 62.7, 114.05, 63.95),                    # U21's GND via
           (111.3, 63.0, 112.6, 64.3)]                       # the strap's +3V3 via
KEEP = KEEP_EARLY + KEEP_SW


def _flat_b(fp, x, y, left_pad, right_pad):
    """Bottom side, left_pad straight left of right_pad, origin at (x, y)."""
    import power42
    for deg in (0, 180, 90, 270):
        power42._at(fp, x, y, deg, bottom=True)
        (ax, ay), (bx, by) = power42._pxy(fp, left_pad), power42._pxy(fp, right_pad)
        if ax < bx and abs(ay - by) < 0.05:
            return
    raise RuntimeError(f"{fp.GetReference()}: no flat orientation")


def _by_nets(FP, prefix, nets):
    out = [r for r, f in FP.items() if r.startswith(prefix) and {p.GetNetname() for p in f.Pads()} == nets]
    assert len(out) == 1, f"I2C plan: {prefix} on {nets}: {out}"
    return out[0]


def _pad_of(FP, ref, net):
    return next(p.GetNumber() for p in FP[ref].Pads() if p.GetNetname() == net)


def place(FP, obstacles):
    """The pull-ups (SCL's and SDA's under the trunk, ALERT's R4 on its hop) on their spots, locked; their boxes
    and KEEP_EARLY become obstacles. Run before the logic groups. Returns the references placed."""
    from satellites import _box
    done = []
    for net, (x, y) in list(PU_SPOTS.items()) + [("INA_ALERT", R4_SPOT)]:
        ref = _by_nets(FP, "R", {net, "+3V3"})
        bus, sup = _pad_of(FP, ref, net), _pad_of(FP, ref, "+3V3")
        # SCL's bus pad on the left (its line is left of it); SDA's and ALERT's on the right
        _flat_b(FP[ref], x, y, *((bus, sup) if net == "I2C_SCL" else (sup, bus)))
        FP[ref].SetLocked(True)
        obstacles.append(_box(FP[ref]))
        done.append(ref)
    obstacles += KEEP_EARLY
    return done


def place_sw(FP, obstacles):
    """After the switch-control ICs (U21's spot is the packer's, so this goes by its pins): its address strap
    west of it, upright (the +3V3 half above, the GND half below, A0 between them), and its 100 nF under pins
    5 ... 8 (the supply pad under pin 5, the GND pad under pin 8), locked; KEEP_SW becomes obstacles. Returns the
    references placed."""
    import power42
    from satellites import _box
    pins = {p.GetNumber(): (p.GetNetname(), power42._pxy(FP["U21"], p.GetNumber())) for p in FP["U21"].Pads()}
    a0 = pins["7"][0]
    x8, y8 = pins["8"][1]
    done = []
    for sup in ("+3V3", "GND"):
        ref = _by_nets(FP, "R", {a0, sup})
        top, bottom = (_pad_of(FP, ref, sup), _pad_of(FP, ref, a0)) if sup == "+3V3" else \
                      (_pad_of(FP, ref, a0), _pad_of(FP, ref, sup))
        power42._upright(FP[ref], x8 - STRAP_DX, STRAP_Y[sup], top, bottom, bottom=True)
        done.append(ref)
    cap, = CAP_U21
    assert {p.GetNetname() for p in FP[cap].Pads()} == {"+3V3", "GND"}, f"I2C plan: {cap} is not a decap"
    _flat_b(FP[cap], x8 + CAP_DX, y8 + CAP_DY, _pad_of(FP, cap, "GND"), _pad_of(FP, cap, "+3V3"))
    done.append(cap)
    for ref in done:
        FP[ref].SetLocked(True)
        obstacles.append(_box(FP[ref]))
    obstacles += KEEP_SW
    return done


CAP_U21 = ("C118",)                              # U21's 100 nF (its 1 uF, C119: logic_place.SW_SPOTS, on the plane)


def route(R, FP, log=print):
    """Lay the bus. Returns the new segments and vias."""
    import far42
    import ina_link
    import route_plan
    from route import B, F, IN2
    segs, vias = [], []

    def lay(net, layer, pts, w=W):
        segs.extend(R.track(net, layer, pts, w))

    def via(net, x, y):
        vias.append(R.via(net, x, y))

    c = lambda ref, num: R.pad(ref, num)["c"]
    pin = lambda ref, net: next(q["num"] for q in R.geo.pads if q["ref"] == ref and q["net"] == net)
    at = lambda ref, net: c(ref, pin(ref, net))

    # the column: the link's ends onto L3, down to the trunk
    for net in NETS:
        assert math.dist(COL_B[net][0], ina_link.ENDS[net]) < 1e-6, f"I2C plan: {net} off the link's end"
        lay(net, B, COL_B[net])
        via(net, *COL_B[net][-1])
        lay(net, IN2, COL_L3[net])

    # the ESP32 end of SCL / SDA: the turn north, east under the module, up to the vias under its edge
    esp = {}
    for net in ("I2C_SCL", "I2C_SDA"):
        tx, ty = ESP_TURN[net]
        px, _ = at("U13", net)
        y0 = TRUNK_Y[net]
        esp[net] = [(tx - 0.5, y0), (tx, y0 - 0.5), (tx, ty + 0.5), (tx + 0.5, ty), (px - 0.5, ty), (px, ty - 0.5),
                    (px, ESP_VIA_Y)]

    # the trunk, with a vertex at every tee and via on it
    stops = {"I2C_SCL": [BRANCH["U20"]["I2C_SCL"], BRANCH["U17"]["I2C_SCL"], U21_GAP["I2C_SCL"][1],
                         TP_X["I2C_SCL"]],
             "I2C_SDA": [BRANCH["U20"]["I2C_SDA"], BRANCH["U17"]["I2C_SDA"], U21_GAP["I2C_SDA"][1],
                         TP_X["I2C_SDA"]],
             "INA_ALERT": []}
    for net in NETS:
        y = TRUNK_Y[net]
        x0 = COL_L3[net][-1][0]
        x1 = esp[net][0][0] if net in esp else ALERT_X
        lay(net, IN2, [(x0, y)] + [(x, y) for x in sorted(stops[net])] + [(x1, y)] + esp.get(net, [])[1:])

    # U20 and U17: pin stubs and vias, the L3 lines down to the trunk, SDA's hop, the GND pin under the body
    for ic, spec in BRANCH.items():
        (wx, py), (ex, _) = sorted((at(ic, n) for n in ("I2C_SCL", "I2C_SDA")), key=lambda p: p[0])
        for net in ("I2C_SCL", "I2C_SDA"):
            px, _ = at(ic, net)
            if px == wx:                       # the west pin: straight down to its via
                pts, (vx, vy) = [(px, py), (px, VIA_Y)], (px, VIA_Y)
            else:                              # the east pin: a short jog east, down to its via
                vx, vy = px + JOG, VIA_Y2
                pts = [(px, py), (px, VIA_Y - 0.65), (vx, VIA_Y - 0.65 + JOG), (vx, vy)]
            lay(net, B, pts)
            via(net, vx, vy)
            x = spec[net]
            top = HOP_Y if net == "I2C_SDA" else TRUNK_Y[net]
            lay(net, IN2, [(x, top), (x, vy + (vx - x)), (vx, vy)])
        x = spec["I2C_SDA"]
        via("I2C_SDA", x, HOP_Y)
        via("I2C_SDA", x, TRUNK_Y["I2C_SDA"])
        lay("I2C_SDA", B, [(x, HOP_Y), (x, TRUNK_Y["I2C_SDA"])])
        g, g2 = spec["gnd"]
        (gx, gy), (hx, hy) = c(ic, g), c(ic, g2)
        lay("GND", B, [(gx, gy), (gx, 68.5), (hx, 68.5 - abs(gx - hx)), (hx, hy)], GND_W)

    # U21: both lines on the bottom layer to the trunk; GND 4 to 9 under the body and 9's via; GND 8 into the
    # 100 nF's GND pad; A0 under the body to the strap
    for net in ("I2C_SCL", "I2C_SDA"):
        px, py = at("U21", net)
        y0, x = U21_GAP[net]
        lay(net, B, [(px, py), (px, y0), (x, y0 + (x - px)), (x, TRUNK_Y[net])])
        via(net, x, TRUNK_Y[net])
    (x4, y4), (x9, y9), (x8, y8), (x7, y7) = (c("U21", n) for n in ("4", "9", "8", "7"))
    lay("GND", B, [(x4, y4), (x4, U21_GND_Y + 0.5), (x4 - 0.5, U21_GND_Y), (x9 + 0.5, U21_GND_Y),
                   (x9, U21_GND_Y - 0.5), (x9, y9)], GND_W)
    lay("GND", B, [(x9, y9), (x9 - 0.85, y9 - 0.85)], GND_W)
    via("GND", x9 - 0.85, y9 - 0.85)
    cap = next(q for q in R.geo.pads if q["ref"] in CAP_U21 and q["net"] == "GND")
    assert cap["box"][0] < x8 < cap["box"][2], "I2C plan: U21's 100 nF is off pin 8"
    lay("GND", B, [(x8, y8), (x8, cap["c"][1])], GND_W)
    a0 = R.pad("U21", "7")["net"]
    strap = sorted((q for q in R.geo.pads if q["net"] == a0 and q["ref"] != "U21"), key=lambda q: q["c"][1])
    sx = x8 - STRAP_DX
    assert all(abs(q["c"][0] - sx) < 1e-3 for q in strap), "I2C plan: the strap is off its column"
    lay(a0, B, [(x7, y7), (x7, U21_A0_Y), (sx, U21_A0_Y)])
    lay(a0, B, [strap[0]["c"], strap[-1]["c"]])
    top = next(q for q in R.geo.pads if q["ref"] == strap[0]["ref"] and q["net"] == "+3V3")
    lay("+3V3", B, [top["c"], (sx, STRAP_V3_Y + 0.5), (sx - 0.5, STRAP_V3_Y)])
    via("+3V3", sx - 0.5, STRAP_V3_Y)

    # the ESP32: SCL / SDA into the pins from the vias under the module edge (top side)
    for net in ("I2C_SCL", "I2C_SDA"):
        px, py = at("U13", net)
        via(net, px, ESP_VIA_Y)
        lay(net, F, [(px, ESP_VIA_Y), (px, py)])
    # ALERT: the trunk's end, the hop with R4 on it, L3 north, the via at pin 18, a top-side stub into the pin
    via("INA_ALERT", ALERT_X, TRUNK_Y["INA_ALERT"])
    via("INA_ALERT", ALERT_X, HOP_Y)
    lay("INA_ALERT", B, [(ALERT_X, TRUNK_Y["INA_ALERT"]), (ALERT_X, HOP_Y)])
    lay("INA_ALERT", IN2, [(ALERT_X, HOP_Y), ALERT_PIN_VIA])
    via("INA_ALERT", *ALERT_PIN_VIA)
    ax, ay = at("U13", "INA_ALERT")
    vx, vy = ALERT_PIN_VIA
    lay("INA_ALERT", F, [(vx, vy), (vx - (vy - ay), ay), (ax, ay)])

    # the test pads and the pull-ups (bottom layer, from vias on the trunk)
    tps = {net: next(q["c"] for q in R.geo.pads if q["ref"].startswith("TP") and q["net"] == net)
           for net in ("I2C_SCL", "I2C_SDA")}
    assert tps["I2C_SCL"][1] < tps["I2C_SDA"][1], "I2C plan: the SDA test pad must be below SCL's"
    for net in ("I2C_SCL", "I2C_SDA"):
        x, y = TP_X[net], TRUNK_Y[net]
        via(net, x, y)
        tx, ty = tps[net]
        lay(net, B, [(x, y), (x, PU_Y), (x, ty), (tx, ty)])
        pu = next(q for q in R.geo.pads if q["ref"].startswith("R") and q["net"] == net and abs(q["c"][1] - PU_Y) < 1e-3)
        lay(net, B, [(x, PU_Y), pu["c"]])

    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:30]:
        log(f"   I2C bus clearance: {msg}")
    if bad:
        R.failed.append("I2C bus")
    log(f"   I2C bus: {len(segs)} segments, {len(vias)} vias, {len(bad)} clearance problems")
    return segs, vias
