"""The FLT bus: the five eFuses' fault lines to the ESP32 (decided 2026-11-21: hand-planned, the hybrid plan).

Each FLT line already runs from its eFuse to a test pad (the block plans; the 42 V ones by far42). From there to
the ESP32 the router weaved through the rails' bus legs with up to five vias a line; this plan takes two:

  the rails' 24 / 12 V  their lines run straight down their block lanes on the bottom layer, across the blocks'
                        bottom edge. There a via each takes them onto L3, east under the blocks' edge in the two
                        L3 lanes the edge leaves free (before the rails' legs take theirs): FLT_24 at y 49.7, just
                        under the bypass bridges' pads (JP3 / JP4), jogging between JP5's two pads; FLT_12 at
                        y 54.2, round JP5's lower pad. At the ESP32 both turn down: FLT_24 east over the top pin
                        row to its pin via, FLT_12 down beside the module to a via between the status LEDs D20 /
                        D19 and a top-side track into pin 17 (no via fits beside that pin).
  5 V                   from its test pad (TP18, right beside the module) a via down, L3 south-east, east under
                        the module and up into its pin via (it may not cross FLT_12's line: round below its via).
  42C / 42D             42C from its dive right of U17 (far42), 42D from a via west of its test pad, L3 east and north-east
                        to join FLT_5's run as a three-line bundle under the module, each peeling up into its
                        pin via in turn (FLT_5, 42C, 42D: no crossings).

The pin vias sit above the module's top pin row (outside its body), a short top-side stub into each pin. The
pull-ups hang on them on the bottom layer: FLT_24's and FLT_5's below their vias, FLT_42C's between the 5 V and
42C vias, their +3V3 ends on one 0.5 mm spine into the LDO's output cap (C103); FLT_12's below its via beside the
module; FLT_42D's below its test pad (no room at the module: U12 and C103 fill the bottom there). The pull-ups are
placed here (before the logic groups) and their room kept from the groups and the packer.
Everything is laid as planned polylines and then checked exactly, new copper against new copper too.
"""
import math

BUS_Y = {"FLT_24": 49.7, "FLT_12": 54.2}        # the rails' lanes on L3 (and the vias onto them)

# pull-ups: net -> centre (x, y), upright, the FLT pad (2) on top
PULLUPS = {"FLT_24": (135.3, 64.9), "FLT_5": (137.76, 64.9), "FLT_42C": (139.1, 59.25),
           "FLT_12": (126.25, 66.6), "FLT_42D": (113.0, 79.5)}
SPINE_X = 139.1                                 # the pull-ups' +3V3 spine (bottom layer)
PU_V33_DX = 1.39                                # FLT_42D's pull-up: its +3V3 via this far right of its pad

# pin vias (above the top row; FLT_12's beside the module, between the LEDs D20 and D19)
PIN_VIAS = {"FLT_24": (135.3, 59.2), "FLT_5": (137.76, 59.75), "FLT_42C": (140.3, 59.75),
            "FLT_42D": (141.57, 59.75), "FLT_12": (126.25, 64.5)}

# L3 lines (from their first via on)
L3 = {
    "FLT_24": [(118.0, 49.7), (119.18, 50.88), (126.25, 50.88), (126.75, 51.38), (126.75, 56.5), (127.25, 57.0),
               (134.8, 57.0), (135.3, 57.5), (135.3, 59.2)],
    "FLT_12": [(116.0, 54.2), (117.4, 55.6), (125.75, 55.6), (126.25, 56.1), (126.25, 64.5)],
    "FLT_5": [(123.5, 63.5), (125.6, 65.6), (136.26, 65.6), (137.76, 64.1), (137.76, 59.75)],
    # 42C dives where far42's bundle opens right of U17 and stays on L3 under the risers, out of the 5 V output's
    # current (2026-11-24); its test pad hangs on a via at TP_VIA. 42D's via sits just east of that current.
    "FLT_42C": [(100.6, 73.6), (101.6, 74.6), (116.0, 74.6), (124.5, 66.1), (138.8, 66.1), (140.3, 64.6),
                (140.3, 59.75)],
    "FLT_42D": [(110.4, 76.0), (115.31, 76.0), (124.71, 66.6), (140.07, 66.6), (141.57, 65.1), (141.57, 59.75)],
}
TP_VIA = {"FLT_42C": (111.1, 74.6)}             # a test pad's via onto its L3 line (on the line)

# kept for the plan's bottom-side copper (the pull-ups block their own courtyards)
KEEP = [(134.5, 57.1, 139.4, 66.7),     # the rails' pin vias, their pull-ups and the +3V3 spine
        (139.4, 57.1, 142.1, 62.7),     # the 42 V pin vias and FLT_42C's pull-up
        (122.6, 62.6, 124.4, 64.4),     # FLT_5's via beside its test pad
        (125.6, 63.9, 127.0, 65.2),     # FLT_12's via beside the module
        (109.9, 73.9, 111.7, 76.8),     # the 42 V lines' vias (42C's dive sits in far42's keep)
        (112.6, 76.6, 114.0, 78.2)]     # FLT_42D's pull-up stub


def roles(FP):
    """net -> (test pad, pull-up, ESP32 pin) references / pin number, found by what they connect to."""
    out = {}
    for net in PULLUPS:
        tp = [r for r, f in FP.items() if r.startswith("TP") and {p.GetNetname() for p in f.Pads()} == {net}]
        pu = [r for r, f in FP.items() if r.startswith("R") and {p.GetNetname() for p in f.Pads()} == {net, "+3V3"}]
        pin = [p.GetNumber() for p in FP["U13"].Pads() if p.GetNetname() == net]
        assert len(tp) == 1 and len(pu) == 1 and len(pin) == 1, f"FLT bus: {net}: {tp} {pu} {pin}"
        out[net] = (tp[0], pu[0], pin[0])
    return out


def place(FP, obstacles):
    """Put the pull-ups on their spots and lock them; their boxes and KEEP become obstacles. Returns the
    references placed."""
    import power42
    from satellites import _box
    done = []
    for net, (_, ref, _) in sorted(roles(FP).items()):
        x, y = PULLUPS[net]
        f = FP[ref]
        power42._upright(f, x, y, "2", "1", bottom=True)
        f.SetLocked(True)
        obstacles.append(_box(f))
        done.append(ref)
    obstacles += KEEP
    return done


def _lane_x(R, net, y):
    """x of the net's vertical bottom-layer track that crosses y (its block lane)."""
    from route import B
    xs = [s[2] for s in R.geo.segs if s[0] == net and s[1] == B and abs(s[2] - s[4]) < 1e-6
          and min(s[3], s[5]) < y < max(s[3], s[5])]
    assert len(xs) == 1, f"FLT bus: {net} has {len(xs)} lanes across y {y}"
    return xs[0]


def route(R, FP, log=print):
    """Lay the FLT bus. Returns the new segments and vias."""
    import route_plan
    from route import B, F, IN2
    W, SW = route_plan.SIG_W, route_plan.SUP_W
    rl = roles(FP)
    segs, vias = [], []

    def lay(net, layer, pts, w=W):
        segs.extend(R.track(net, layer, pts, w))

    def via(net, x, y):
        vias.append(R.via(net, x, y))

    c = lambda ref, num: R.pad(ref, num)["c"]
    for net, pts in L3.items():
        tp, pu, pin = rl[net]
        if net in BUS_Y:                       # onto the bus from the block lane
            x0, y0 = _lane_x(R, net, BUS_Y[net]), BUS_Y[net]
            via(net, x0, y0)
            pts = [(x0, y0)] + pts
        else:
            via(net, *pts[0])
        if net == "FLT_5":                     # from the test pad down to the via
            lay(net, B, [c(tp, "1"), pts[0]])
        lay(net, IN2, pts)
        if net in TP_VIA:                      # the test pad on its own via off the L3 line
            via(net, *TP_VIA[net])
            lay(net, B, [TP_VIA[net], c(tp, "1")])
        vx, vy = PIN_VIAS[net]                 # the pin via and its top-side stub
        assert math.dist(pts[-1], (vx, vy)) < 1e-6, f"FLT bus: {net} ends off its pin via"
        via(net, vx, vy)
        px, py = c("U13", pin)
        if net == "FLT_12":                    # between D20 and D19, along the module's pin column
            lay(net, F, [(vx, vy), (128.7, vy), (128.7, py), (px, py)])
        elif abs(px - vx) > 1e-6:              # a 45-degree jog to the pin, then straight in
            lay(net, F, [(vx, vy), (px, vy + abs(vx - px)), (px, py)])
        else:
            lay(net, F, [(vx, vy), (px, py)])
        # the pull-up: its FLT pad on the via above it (FLT_42C's from beside; FLT_42D's from its test pad)
        fx, fy = c(pu, "2")
        if net == "FLT_42C":
            lay(net, B, [(fx, fy), (vx - 0.7, fy), (vx, fy + 0.7), (vx, vy)])
        elif net == "FLT_42D":
            tx, ty = c(tp, "1")
            lay(net, B, [(tx, ty), (fx, ty + abs(tx - fx)), (fx, fy)])
            # its +3V3 pad: a via into the L3 fill beside it, placed here so the later routes keep off the spot
            # (2026-11-24: MOTOR_OK, off its old L3 drop through the 5 V strip, took it)
            ux, uy = c(pu, "1")
            vias.append(R.via("+3V3", ux + PU_V33_DX, uy))
            lay("+3V3", B, [(ux, uy), (ux + PU_V33_DX, uy)], SW)
        else:
            lay(net, B, [(vx, vy), (fx, fy)])
    # the +3V3 spine: FLT_42C's pull-up down to the rails' pull-ups, with a branch into C103's +3V3 pad
    (ax, ay), (bx, by), (cx, cy) = (c(rl[n][1], "1") for n in ("FLT_42C", "FLT_5", "FLT_24"))
    lay("+3V3", B, [(ax, ay), (ax, by - 0.5), (ax - 0.5, by), (cx, by)], SW)
    c103 = next(p for p in R.geo.pads if p["ref"] == "C103" and p["net"] == "+3V3")["c"]
    lay("+3V3", B, [(ax, c103[1]), c103], SW)
    assert abs(by - cy) < 1e-6 and SPINE_X == ax
    bad = route_plan._check_exact(R, segs) + _check_vias(R, vias)
    for msg in bad[:20]:
        log(f"   FLT bus clearance: {msg}")
    if bad:
        R.failed.append("FLT bus")
    log(f"   FLT bus: {len(segs)} segments, {len(vias)} vias, {len(bad)} clearance problems")
    return segs, vias


def _check_vias(R, vias):
    """Exact clearance of new vias against the other nets' tracks, vias (new ones included) and pads."""
    import route
    g, out = R.geo, []
    for v in vias:
        net, x, y, r = v
        for o in g.segs:
            if o[0] != net and route._pt_seg(x, y, o[2:6]) < r + o[6] + route.CLEAR - 1e-4:
                out.append(f"{net} via ({x:.2f}, {y:.2f}) / {o[0]} track")
        for o in g.vias:
            if o is not v and o[0] != net and math.hypot(o[1] - x, o[2] - y) < r + o[3] + route.CLEAR - 1e-4:
                out.append(f"{net} via ({x:.2f}, {y:.2f}) / {o[0]} via")
        for p in g.pads:
            if p["net"] == net:
                continue
            l, t, rr, b = p["box"]
            if p.get("round"):
                d = math.hypot(x - p["c"][0], y - p["c"][1]) - (rr - l) / 2
            else:
                d = math.hypot(max(l - x, 0, x - rr), max(t - y, 0, y - b))
            if d < r + route.CLEAR - 1e-4:
                out.append(f"{net} via ({x:.2f}, {y:.2f}) / {p['ref']}.{p['num']} pad")
    return out
