"""The 42 V bus legs' far end (decided 2026-11-21: hand-planned, like the legs themselves).

The legs (route_plan.legs42) arrive at x = 50.6 mm on the bottom layer, 42C's five above 42D's five, each block
in the order VSNS_OUT, INPG, INP, FLT, IMON. Their far end wants them pairwise: VSNS at the ADC's (U20) left end,
IMON at its right end, INP / INPG at the I/O expander (U17), FLT on east to its test pads (the ESP32 group takes
FLT on from there, with the rails' FLT lines). So the lines cross about 20 times: they do it in six hops, each
where a line leaves the bundle, under the lines above it on L3 (a via down, a short L3 run, a via up):

  left of U20    VSNS_C rises from the bundle's top; VSNS_D hops. Both enter U20 under its body from the left,
                 between its pin rows (the input row faces up, the rails come in from above), their R / C pairs
                 hanging on the lines.
  under the hole the other eight drop below rev2's centre hole (75, 75), between it and the 24 V output connector
                 (the 24 / 12 V outputs moved down 6.5 mm for it), from right after the VSNS lines leave.
  right of it    IMON_C and IMON_D hop (east of the hole) and enter U20 under its body from the right; R / C pairs
                 and test pads on their lines. The other six climb back to the U17 levels.
  under U17      six lines, between the expander's pins and the 12 V output connector.
  right of U17   INP_D, INP_C and INPG_D hop, INPG_C rises; all four enter U17 under its body from the right,
                 a pull-down between each riser and the next.
  east           FLT_C and FLT_D on to their test pads.

Wherever a line hops, the bundle opens to 0.8 mm around it (a via needs 0.775 mm to the next line); elsewhere it
runs at 0.5 mm. Everything is laid as planned polylines and then checked exactly (route_plan._check_exact).
"""
import math

ROLES = {   # role: (net prefix, part kind, value test)
    "vsns_r": ("VSNS_OUT_42", "R"), "vsns_c": ("VSNS_OUT_42", "C"),
    "imon_r": ("IMON_42", "R"), "imon_c": ("IMON_42", "C"), "imon_tp": ("IMON_42", "TP"),
    "inp_r": ("INP_42", "R"), "inpg_r": ("INPG_42", "R"), "flt_tp": ("FLT_42", "TP"),
}

# parts: role + block -> (x, y, rot) on the bottom (rot: 90 = pad 1 below, -90 = pad 1 above; test pads: the label
# side, 0 = right, 90 = above). Pad 1 is the signal side of every R / C here.
SPOTS = {
    ("vsns_c", "C"): (65.0, 64.125, 90), ("vsns_r", "C"): (62.9, 64.125, 90),      # above VSNS_C's line
    ("vsns_c", "D"): (62.9, 67.975, -90), ("vsns_r", "D"): (60.8, 67.975, -90),    # below VSNS_D's line
    ("imon_c", "D"): (73.75, 64.125, 90), ("imon_r", "D"): (75.85, 64.125, 90),    # above IMON_D's line
    ("imon_tp", "D"): (78.6, 64.3, 0),
    ("imon_c", "C"): (73.75, 67.975, -90), ("imon_r", "C"): (75.85, 67.975, -90),  # below IMON_C's line
    ("imon_tp", "C"): (79.1, 69.0, 0),                                             # on IMON_C's riser (x 79.3)
    ("inp_r", "D"): (102.9, 68.6, -90), ("inp_r", "C"): (105.1, 68.6, -90),         # beside their risers
    ("inpg_r", "D"): (107.3, 68.6, -90), ("inpg_r", "C"): (109.5, 68.6, -90),
    ("flt_tp", "C"): (110.5, 73.6, 90), ("flt_tp", "D"): (113.7, 76.0, 0),          # on their lines
}

# U20's decoupling cap, beside its left end (its supply pin faces the bundle, so not below it); the group's
# packer would find no room for it near U20 any more
DECAPS = {"C113": (65.4, 69.0, -90)}

# kept for the lines (the parts block their own courtyards; the switch-control group packs around both)
KEEP = [(50.8, 65.3, 54.2, 76.6),          # the risers and VSNS_D's hop, right after the ports
        (54.2, 65.3, 66.8, 66.8),          # VSNS's lines into U20
        (50.8, 71.0, 64.0, 83.0),          # the bundle dropping below the hole
        (64.0, 78.0, 76.8, 82.6),          # below the hole (and above the 24 V output connector)
        (72.4, 65.3, 82.0, 71.2),          # IMON's lines, risers, R / C pairs and test pads right of U20
        (76.8, 71.2, 82.0, 83.2),          # IMON's hops
        (82.0, 72.8, 90.0, 82.4),          # the bundle climbing back
        (85.5, 73.1, 99.6, 76.4),          # under U17
        (99.2, 65.3, 110.8, 76.4),         # right of U17: INP / INPG's lines, risers and hops
        (110.8, 73.1, 114.0, 76.4)]        # FLT on to its test pads

U20_AT, U17_AT = (69.5, 67.0), (95.0, 67.0)  # logic_place puts them there (input rows up)

# the lines: each leg's band y (legs42 order) and its route east. Stations: (x, new y) = a 45-degree jog that
# starts at x. Bundle levels: left of U20 the hop opens the bundle at 51.6; at 54.0 it drops below the centre
# hole (78.65 ... 82.15: the top line 3.65 mm from the hole's centre, its pad is 3.2 in radius; the bottom one
# 0.66 mm clear of the 24 V connector's pads); east of the connector it opens around IMON's hop vias (76.5,
# 77.0) and climbs back at 82.2 to the U17 levels (73.5 ... 76.0); right of U17 it opens to 0.8 mm at 99.6.
L_OPEN, L_CLOSE, R_OPEN, R_CLOSE, S_OPEN = 51.6, 54.0, 76.5, 82.2, 99.6
EVENTS = [   # (x, {net key: new y}): where the bundle opens or closes. Lines that move the same way together
             # are staggered (the leading one first, each next STAGGER later), so their parallel 45-degree jogs
             # keep 0.57 mm apart (not 0.5 / sqrt 2)
    (L_OPEN, {"G_C": 71.7, "P_C": 72.2, "F_C": 72.7, "I_C": 73.2, "G_D": 74.8, "P_D": 75.3, "F_D": 75.8, "I_D": 76.3}),
    (L_CLOSE, {"G_C": 78.65, "P_C": 79.15, "F_C": 79.65, "I_C": 80.15, "G_D": 80.65, "P_D": 81.15, "F_D": 81.65,
               "I_D": 82.15}),
    (R_OPEN, {"G_D": 80.95, "P_D": 81.45, "F_D": 81.95, "I_D": 82.75}),
    (R_OPEN + 0.5, {"G_C": 78.35, "P_C": 78.85, "F_C": 79.35}),
    (R_CLOSE, {"G_C": 73.5, "P_C": 74.0, "F_C": 74.5, "G_D": 75.0, "P_D": 75.5, "F_D": 76.0}),
    (S_OPEN, {"G_C": 72.0, "P_C": 72.8, "F_C": 73.6, "G_D": 74.4, "P_D": 75.2}),
]
STAGGER = 0.3
# hops: net key -> (x of the via in the bundle, y of the upper via). Risers: net -> (x, entry index, side)
HOPS = {"V_D": (53.4, 70.9), "I_C": (79.3, 71.4), "I_D": (81.5, 71.4),
        "P_D": (101.8, 71.2), "P_C": (104.0, 71.2), "G_D": (106.2, 71.2)}
RISERS = {"V_C": (51.2, 0, "L"), "V_D": (53.4, 1, "L"), "I_D": (81.5, 0, "R"), "I_C": (79.3, 1, "R"),
          "G_C": (108.4, 0, "R"), "G_D": (106.2, 1, "R"), "P_C": (104.0, 2, "R"), "P_D": (101.8, 3, "R")}
PINS = {"V_C": ("U20", "9"), "V_D": ("U20", "10"), "I_C": ("U20", "15"), "I_D": ("U20", "16"),
        "P_D": ("U17", "4"), "P_C": ("U17", "5"), "G_D": ("U17", "10"), "G_C": ("U17", "11")}
FLT_END = {"F_C": 110.5, "F_D": 113.7}
NAMES = {"V": "VSNS_OUT", "G": "INPG", "P": "INP", "F": "FLT", "I": "IMON"}


def net_of(key):
    kind, blk = key.split("_")
    return f"{NAMES[kind]}_42{blk}"


def roles(FP):
    """(role, block) -> reference of the far-end parts, found by what they connect to."""
    out = {}
    for ref, f in sorted(FP.items()):
        nets = {p.GetNetname() for p in f.Pads()}
        for role, (prefix, kind) in ROLES.items():
            if not (ref.startswith(kind) and not ref.startswith("TH")):
                continue
            hit = [n for n in nets if n.startswith(prefix) and n[len(prefix):] in ("C", "D")]
            if not hit or (kind != "TP" and nets - set(hit) != {"GND"}):
                continue
            if prefix == "INP_42" and any(n.startswith("INPG") for n in nets):
                continue
            out[(role, hit[0][-1])] = ref
    return out


def place(FP, obstacles):
    """Put the far-end parts on their spots and lock them; their boxes and the lines' keep-outs become
    obstacles. Returns the references placed."""
    import pcbnew
    import power42
    from satellites import _box
    at = lambda f: (round(pcbnew.ToMM(f.GetPosition().x) - power42.X0, 2),
                    round(pcbnew.ToMM(f.GetPosition().y) - power42.Y0, 2))
    got = [at(FP["U20"]), at(FP["U17"])]
    assert got == [U20_AT, U17_AT], f"far42 expects U20 / U17 at {U20_AT} / {U17_AT}, not {got}"
    done = []
    for key, ref in sorted(roles(FP).items()):
        x, y, deg = SPOTS[key]
        f = FP[ref]
        power42._at(f, x, y, 0 if ref.startswith("TP") else deg, bottom=True)
        f.SetLocked(True)
        obstacles.append(_box(f))
        done.append(ref)
    for ref, (x, y, deg) in DECAPS.items():
        f = FP[ref]
        power42._at(f, x, y, deg, bottom=True)
        f.SetLocked(True)
        obstacles.append(_box(f))
        done.append(ref)
    obstacles += KEEP
    return done


def _stations(slots):
    """Each bundle line's jogs: (start x, new y), staggered within each event as EVENTS says."""
    ys = dict(slots)
    out = {k: [] for k in slots}
    for x, moves in EVENTS:
        up = sorted((k for k, y in moves.items() if y < ys[k] - 1e-6), key=lambda k: ys[k])        # top first
        down = sorted((k for k, y in moves.items() if y > ys[k] + 1e-6), key=lambda k: -ys[k])     # bottom first
        for group in (up, down):
            for i, k in enumerate(group):
                out[k].append((x + STAGGER * i, moves[k]))
                ys[k] = moves[k]
    return out


def _path(port_y, stations, x_end):
    """A bundle line from its port east, jogging 45 degrees at each station, to x_end. Returns its points and
    its last y."""
    import route_plan
    pts = [(route_plan.PORT_X, port_y)]
    y = port_y
    for x, ny in stations:
        if x >= x_end:
            break
        pts += [(x, y), (x + abs(ny - y), ny)]
        y = ny
    pts.append((x_end, y))
    return pts, y


def route(R, FP, log=print):
    """Lay the far end of the 42 V bus. Returns the new segments and vias."""
    import route
    import route_plan
    from route import B, IN2
    W = route_plan.LEG_W
    slots = {}
    for blk, key in (("C", "42C"), ("D", "42D")):
        for (name, _), y in zip(route_plan.LEGS42, route_plan.corridor42(key)["slots"]):
            kind = {"VSNS_OUT": "V", "INPG": "G", "INP": "P", "FLT": "F", "IMON": "I"}[name]
            slots[f"{kind}_{blk}"] = y
    pad = lambda ref, num: R.pad(ref, num)
    entry0 = {ic: pad(ic, PINS[k][1])["box"][3] + 0.92 for k, (ic, _) in PINS.items() for ic in [ic]}
    segs, vias = [], []

    def lay(net, pts, layer=B):
        segs.extend(R.track(net, layer, pts, W))

    def hop(net, x, y_low, y_up):
        vias.append(R.via(net, x, y_low))
        lay(net, [(x, y_low), (x, y_up)], IN2)
        vias.append(R.via(net, x, y_up))

    def rise(key, x, y_from):
        """From (x, y_from) up to the entry line, then along it into the IC under its body, up into the pin."""
        ic, num = PINS[key]
        p = pad(ic, num)
        _, ent, side = RISERS[key]
        ye = entry0[ic] + 0.5 * ent
        d = -0.5 if side == "R" else 0.5
        return [(x, y_from), (x, ye + 0.5), (x + d, ye), (p["c"][0], ye), p["c"]]

    BUNDLE = _stations({k: y for k, y in slots.items() if k not in ("V_C", "V_D")})
    for key in ("V_C", "G_C", "P_C", "F_C", "I_C", "V_D", "G_D", "P_D", "F_D", "I_D"):
        net = net_of(key)
        y0 = slots[key]
        if key == "V_C":                         # the top line rises at once
            x = RISERS[key][0]
            lay(net, [(route_plan.PORT_X, y0)] + rise(key, x, y0))
            continue
        if key == "V_D":                         # hops at once, through the opened bundle
            x, yu = HOPS[key]
            lay(net, [(route_plan.PORT_X, y0), (x, y0)])
            hop(net, x, y0, yu)
            lay(net, rise(key, x, yu))
            continue
        if key in HOPS:
            x, yu = HOPS[key]
            pts, y = _path(y0, BUNDLE[key], x if key != "I_D" else 81.5)
            lay(net, pts)
            hop(net, x, y, yu)
            lay(net, rise(key, x, yu))
            continue
        if key == "G_C":                         # the top line right of U17 rises
            x = RISERS[key][0]
            pts, y = _path(y0, BUNDLE[key], x)
            lay(net, pts[:-1] + rise(key, x, y))
            continue
        pts, y = _path(y0, BUNDLE[key], FLT_END[key])   # FLT: east to its test pad
        lay(net, pts)
    # the parts on the lines: a straight stub from the line to each signal pad
    stubs = {("vsns_c", "C"): "V_C", ("vsns_r", "C"): "V_C", ("vsns_c", "D"): "V_D", ("vsns_r", "D"): "V_D",
             ("imon_c", "D"): "I_D", ("imon_r", "D"): "I_D", ("imon_tp", "D"): "I_D",
             ("imon_c", "C"): "I_C", ("imon_r", "C"): "I_C",
             ("inp_r", "D"): "P_D", ("inp_r", "C"): "P_C", ("inpg_r", "D"): "G_D", ("inpg_r", "C"): "G_C"}
    rl = roles(FP)
    for role, key in stubs.items():
        ref = rl.get(role)
        if not ref:
            continue
        net = net_of(key)
        p = next(q for q in R.geo.pads if q["ref"] == ref and q["net"] == net)
        px, py = p["c"]
        if role[0] in ("inp_r", "inpg_r"):     # beside its riser: across to it
            lay(net, [(RISERS[key][0], py), (px, py)])
        elif role[0] == "imon_tp" or py < entry0["U20"]:   # above its line: down to it
            ye = entry0["U20"] + 0.5 * RISERS[key][1]
            lay(net, [(px, py), (px, ye)])
        else:                                  # below its line: up to it
            ye = entry0["U20"] + 0.5 * RISERS[key][1]
            lay(net, [(px, py), (px, ye)])
    tp = rl.get(("imon_tp", "C"))              # IMON_C's test pad: across to its riser
    if tp:
        p = next(q for q in R.geo.pads if q["ref"] == tp)
        lay(net_of("I_C"), [(RISERS["I_C"][0], p["c"][1]), p["c"]])
    bad = route_plan._check_exact(R, segs) + _check_vias(R, vias)
    for (role, blk), ref in sorted(rl.items()):        # then a GND via beside each part's GND pad
        for q in [q for q in R.geo.pads if q["ref"] == ref and q["net"] == "GND"]:
            R.fanout(ref, q["num"], route_plan.SUP_W, zone_layers=(route.IN1,), optional=True)
    for msg in bad[:20]:
        log(f"   far-end clearance: {msg}")
    if bad:
        R.failed.append("42 V bus far end")
    log(f"   far end: {len(segs)} segments, {len(vias)} vias, {len(bad)} clearance problems")
    return segs, vias


def _check_vias(R, vias):
    """Exact clearance of new vias against the other nets' tracks, vias and pads (all layers)."""
    import route
    g, out = R.geo, []
    ids = {id(v) for v in vias}
    for v in vias:
        net, x, y, r = v
        for o in g.segs:
            if o[0] != net and route._pt_seg(x, y, o[2:6]) < r + o[6] + route.CLEAR - 1e-4:
                out.append(f"{net} via ({x:.2f}, {y:.2f}) / {o[0]} track")
        for o in g.vias:
            if id(o) not in ids and o[0] != net and math.hypot(o[1] - x, o[2] - y) < r + o[3] + route.CLEAR - 1e-4:
                out.append(f"{net} via ({x:.2f}, {y:.2f}) / {o[0]} via")
        for p in g.pads:
            if p["net"] == net:
                continue
            l, t, rr, b = p["box"]
            d = math.hypot(max(l - x, 0, x - rr), max(t - y, 0, y - b))
            if d < r + route.CLEAR - 1e-4:
                out.append(f"{net} via ({x:.2f}, {y:.2f}) / {p['ref']}.{p['num']} pad")
    return out
