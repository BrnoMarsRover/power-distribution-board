"""Routing plans per block, by intent (see route.py for the router). Board mm from the top-left corner.

Rail eFuse (TPS48111 on the bottom under its switch), in this order:
  1. every IC pin gets its escape stub first, so later routes leave room for the pins still to come; CS- pin 19
     escapes inward and joins pin 17 under the body (CS+ sits between them);
  2. gate drive, 0.4 mm (0.25 mm at the IC): BST to the bootstrap cap; SRC from the pin through the cap's SRC pad
     to the main FET's source vias (a source reference of its own, not through a plane); PD / PU through the
     10 ohm, PD on through the 2.2 ohm to the main gate's escape via and to the GATE test pad; the pre-charge gate
     G through its RC's resistor to its escape via;
  3. the shunt's Kelvin pair, 0.25 mm: CS- from both pins to its via, then CS+ through the 100 ohm filter to its
     via, pulled along CS- and kept off the gate drive;
  4. VS and its filter, then the small-signal pins (timer, current warning, IMON, enable, INP / INPG, fault,
     the temperature transistor), the LED resistor to the LED's anode via, the two output-sense dividers;
  5. supply pads: a via into the GND plane next to every GND pad; VIN24 pads to the VIN pour (top); OUT24 pads
     to the output copper (the L3 strip under the source vias, or the L1 output area below the switch).
Nets that leave the block (INP, INPG, IMON, FLT, VSNS_IN, VSNS_OUT to the ADCs / expander / ESP32) are routed
only between their local parts here; the bus legs come with the bus routing.
"""
import math

import numpy as np

import pcbnew

import power42
import power_rails
import route
from power42 import X0, Y0
from route import B, F, IN1, IN2

GATE_W, NECK_W, SIG_W, SUP_W = 0.4, 0.25, 0.25, 0.5    # supply: the power rule's 0.5 mm


def _tps(FP, sheet):
    return next(f for f in FP.values() if f.GetSheetname() == sheet and "TPS48111" in f.GetValue())


def efuse_rail(R, FP, key, log=print):
    """A rail block's eFuse (power_rails.RAILS[key])."""
    r = power_rails.RAILS[key]
    g = power_rails._geo(r)
    pre, main = power_rails.FET_ROWS
    gate = lambda ref: next(p["net"] for p in R.geo.pads if p["ref"] == ref and p["num"] == "1")
    _efuse(R, FP, dict(
        fets=r["fets"], shunt=r["shunt"], led=r["led"], vin=r["vin"], vin_layers=(F,), out=r["out"],
        box=(r["xl"] - 0.5, 10.0, r["xr"] + 0.5, 73.0),
        gate_vias={gate(r["fets"][0]): (g["fet_cx"] - 10.85, pre - 2.54),
                   gate(r["fets"][1]): (g["fet_cx"] - 10.85, main - 2.54)},
        kelvin=((g["shunt_x"] + 2.7, power_rails.Y_SHUNT - 2.2), (g["shunt_x"] + 2.7, power_rails.Y_SHUNT + 2.7)),
        led_via=(g["side_x"] + 1.7, power_rails.LED_Y + 1.02), gnd_ok=[]), log)


def efuse42(R, FP, key, log=print):
    """A 42 V block's eFuse (power42.BLOCKS[key]): the IC sits on the bottom under the BUS42 bar. The block is 42 V
    copper on L1 throughout, so the only vias are into its own bar and output copper, the escapes, and GND vias in
    the drain copper beside the FET tabs (agreed 2026-11-21: the 42 V pour clears 0.5 mm around them)."""
    blk = power42.BLOCKS[key]
    dx = blk["dx"]
    cx = power42.FET_CX + dx
    gate = lambda ref: next(p["net"] for p in R.geo.pads if p["ref"] == ref and p["num"] == "1")
    _efuse(R, FP, dict(
        fets=blk["fets"], shunt=blk["shunt"], led=blk["led"], vin="BUS42", vin_layers=(F, IN2),
        out={"42D": "OUT42D", "42C": "OUT42C"}[key],
        box=(blk["out_l"] - 0.4, 12.0, BLOCK42_R[key], 76.0),
        gate_vias={gate(ref): (cx - 10.85, cy - 2.54) for ref, cy in zip(blk["fets"], power42.FET_ROWS)},
        kelvin=((power42.SHUNT_X + dx + 2.7, 34.0), (power42.SHUNT_X + dx + 2.7, 38.9)),
        led_via=(power42.LED_X + dx + 1.7, power42.LED_Y + 1.02),
        gnd_ok=[(cx + 6.2 + 0.6, power42.DRN_TOP, blk["drn_r"] - 0.2, power42.DRN_BOT)],
        bar=True, keep_cs_out=(cx + 5.5, 28.6, cx + 10.4, 40.6),
        # the Kelvin corner: no supply via between the Kelvin + via and the channel right of the shunt, where CS-
        # comes down to its own via (0.25 mm track, 0.2 mm each side of it)
        via_keep=[(power42.SHUNT_X + dx + 1.2, 31.0, power42.SHUNT_X + dx + 4.25, 36.3)],
        # the parts below the bar (the EN divider's top) take BUS42 from the lowest BUS42 pad in the bar (the VS
        # resistor's), up the channel right of the drain copper and left under the CS- via: no via of their own
        vin_join=True,
        # 42C: its bootstrap caps sit left of the IC (efuse_hand.SLOTS42_KEY); SRC reaches them under the body, and
        # the pre-charge gate goes down between them, not through the corner where SRC, PD and PU leave
        src_in=(key == "42C"),
        g_avoid=(cx + 2.2 - 6.4, 32.45, cx + 2.2 - 2.1, 37.5) if key == "42C" else None,
        # the bus legs' corridor (both blocks'): nothing else in it, and no via near it
        route_keep=[b for k in BLOCKS42 for b in corridor42(k)["keep"]],
        via_keep_more=[(l - 0.5, t - 0.5, r + 0.5, b_ + 0.5) for k in BLOCKS42 for l, t, r, b_ in corridor42(k)["keep"]],
        # the output divider and test pad above 42C's band reach the output copper through the drain copper, as
        # GND does (agreed 2026-11-21: same column, same 0.5 mm clearance)
        out_ok=[(cx + 6.2 + 0.6, power42.DRN_TOP, blk["drn_r"] - 0.2, power42.DRN_BOT)]), log)


BLOCK42_R = {"42D": 25.8, "42C": 50.8}          # each block's right edge for its routes: 42C's left, the 24 V block
BLOCKS42 = ("42D", "42C")

# The 42 V bus legs (agreed 2026-11-21, option A). IMON, FLT, INP and INPG leave each TPS48111's bottom row and run
# straight down a lane under their pins, between the temperature / TMR / IWRN parts on the left and the EN, VS and
# LED parts on the right; VSNS_OUT joins from the output divider. At the block's foot they turn east along a band
# under both blocks, above the XT90 pads: 42C's legs in its upper half (from 42C's lane on), 42D's in the lower
# half all the way. They end at ports at x = 50.6, in band order, where the bus routing takes them on.
LEG_W, LEG_PITCH, PORT_X = 0.25, 0.5, 50.6         # 0.5: the router's 0.2 mm + slack, on its grid
BAND_TOP = {"42C": 71.5, "42D": 74.0}         # each block's top band slot; the next ones LEG_PITCH lower
LEGS42 = (("VSNS_OUT", None), ("INPG", "2"), ("INP", "3"), ("FLT", "4"), ("IMON", "7"))   # band order, top first


def corridor42(key):
    """A 42 V block's legs: lane x per IC pin (FLT, INP and INPG straight under their pins, IMON one pitch right of
    its own, under the FLT pin it shares), band y per leg (LEGS42 order), the keep boxes (lane, band, and the jog
    under the pins) where nothing else may go."""
    import efuse_hand
    ix, _ = efuse_hand.ic42(key)
    lane = {"7": ix + 0.25, "4": ix + 0.75, "3": ix + 1.25, "2": ix + 1.75}
    slots = [BAND_TOP[key] + i * LEG_PITCH for i in range(len(LEGS42))]
    top, bot = slots[0] - 0.3, slots[-1] + 0.3
    return dict(lane=lane, slots=slots,
                keep=[(ix, 39.4, ix + 2.0, bot), (ix, top, PORT_X + 0.4, bot), (ix - 0.95, 36.5, ix + 2.0, 39.4)])


def legs42(R, FP, log=print):
    """Lay the 42 V bus legs (both blocks) as planned, on the bottom layer (no vias in the 42 V blocks): from each
    pin's escape stub (IMON with a 45-degree jog to its lane) straight down the lane, a 45-degree turn into the
    band, east to the port; VSNS_OUT from the output divider's pad down into the band's top slot. Then every new
    segment is checked exactly against the other nets' copper."""
    TURN = 1.0
    new = []
    for key in BLOCKS42:
        blk, c = power42.BLOCKS[key], corridor42(key)
        U = _tps(FP, FP[blk["fets"][1]].GetSheetname()).GetReference()
        for (name, num), y in zip(LEGS42, c["slots"]):
            if num:
                net = R.pad(U, num)["net"]
                (_, sx, sy), = R.t_pad(U, num, escape=True, w=NECK_W, length=0.45)["points"]
                x = c["lane"][num]
                pts = [(sx, sy)] + ([(x, sy + abs(x - sx))] if abs(x - sx) > 1e-6 else [])
                pts += [(x, y - TURN), (x + TURN, y), (PORT_X, y)]
            else:                                  # the output divider's VSNS pad in the block
                p = next(q for q in R.geo.pads if q["net"].startswith(name) and blk["out_l"] < q["c"][0] < BLOCK42_R[key]
                         and 36.0 < q["c"][1] < 76.0)
                net, (px, py) = p["net"], p["c"]
                pts = [(px, py), (px, y - 0.5), (px + 0.5, y), (PORT_X, y)]
            new += R.track(net, B, pts, LEG_W)
    bad = _check_exact(R, new)
    for msg in bad:
        log(f"   leg clearance: {msg}")
    if bad:
        R.failed.append("42 V bus legs")
    log(f"   {len(new)} leg segments laid, {len(bad)} clearance problems")


def _check_exact(R, segs):
    """Exact clearance of new bottom-layer segments against the other nets' tracks, vias and pads."""
    g, out = R.geo, []
    ids = {id(s) for s in segs}
    for s in segs:
        net, l_, x1, y1, x2, y2, hw = s
        for o in g.segs:
            if id(o) in ids or o[0] == net or o[1] != l_:
                continue
            d = route._seg_seg((x1, y1, x2, y2), o[2:6])
            if d < hw + o[6] + route.CLEAR - 1e-4:
                out.append(f"{net} / {o[0]} track {d:.3f} mm at ({x1:.2f}, {y1:.2f})")
        for n, vx, vy, vr in g.vias:
            if n != net and route._pt_seg(vx, vy, (x1, y1, x2, y2)) < hw + vr + route.CLEAR - 1e-4:
                out.append(f"{net} / {n} via at ({vx:.2f}, {vy:.2f})")
        for p in g.pads:
            if p["net"] == net or l_ not in p["layers"]:
                continue
            l, t, r, b = p["box"]
            inside = lambda px, py: l <= px <= r and t <= py <= b
            edges = ((l, t, r, t), (r, t, r, b), (l, b, r, b), (l, t, l, b))
            d = 0.0 if inside(x1, y1) or inside(x2, y2) else min(route._seg_seg((x1, y1, x2, y2), e) for e in edges)
            if d < hw + route.CLEAR - 1e-4:
                out.append(f"{net} / {p['ref']}.{p['num']} {p['net']} pad {d:.3f} mm")
    return out


def _efuse(R, FP, spec, log=print):
    """Route one TPS48111 cluster. spec: fets (pre, main[, main 2]), shunt, led, vin / vin_layers, out, box,
    gate_vias {gate net: point}, kelvin (Kelvin + point, CS- point), led_via, gnd_ok (areas where GND vias may
    pierce 42 V copper)."""
    sheet = FP[spec["fets"][1]].GetSheetname()
    U = _tps(FP, sheet).GetReference()
    box = spec["box"]
    R.geo.hv_gnd_ok = list(spec["gnd_ok"])
    R.geo.via_keep = list(spec.get("via_keep", [])) + list(spec.get("via_keep_more", []))
    R.geo.route_keep = list(spec.get("route_keep", []))
    R.geo.hv_ok = {spec["out"]: list(spec["out_ok"])} if spec.get("out_ok") else {}
    pins = {p["num"]: p for p in R.geo.pads if p["ref"] == U}
    net = lambda num: pins[num]["net"]
    parts = {}                                     # net -> [(ref, num)] of the sheet's other parts
    for p in R.geo.pads:
        if p["ref"] != U and FP[p["ref"]].GetSheetname() == sheet:
            parts.setdefault(p["net"], []).append((p["ref"], p["num"]))
    inside = lambda ref: box[0] <= R.pad(*parts_any[ref])["c"][0] <= box[2] and \
        box[1] <= R.pad(*parts_any[ref])["c"][1] <= box[3]
    parts_any = {}
    for n_, lst in parts.items():
        for ref, num in lst:
            parts_any.setdefault(ref, (ref, num))
    loc = lambda n: [R.t_pad(ref, num) for ref, num in parts.get(n, [])             # bottom parts in the block
                     if FP[ref].IsFlipped() and inside(ref)]
    l, t, rr, b = (min(p["box"][0] for p in pins.values()), min(p["box"][1] for p in pins.values()),
                   max(p["box"][2] for p in pins.values()), max(p["box"][3] for p in pins.values()))
    neck = ((l - 1.2, t - 1.2, rr + 1.2, b + 1.2), NECK_W)
    escape_area(R.b, U, neck[0])                   # where tracks leave the pins at pin width (board_rules.py)
    via_near = lambda net_, x, y: min(((vx, vy) for n, vx, vy, vr in R.geo.vias if n == net_),
                                      key=lambda q: math.hypot(q[0] - x, q[1] - y))
    bst, src = net("12"), net("13")
    caps = _parts_on(parts, bst, src)              # the bootstrap cap(s)

    # 1. IC pins are plain pad terminals: a route leaves through either end of the pad (outward, or inward
    # under the body between the rows), whichever the negotiation settles on. Except the bottom-row pins that
    # leave downwards (temperature diode, TMR, IWRN, IMON, INP, INPG): they first run straight out past their
    # neighbours' clearance, so below the pad row the fan can stagger its turns freely (parallel 45-degree jogs at
    # 0.5 mm pitch need each turn lower than its right-hand neighbour's; at the pad row there is no room for that)
    down = {"10", "9", "8", "7", "3", "2"}
    esc = {num: R.t_pad(U, num, escape=True, w=NECK_W, length=0.45) if num in down else R.t_pad(U, num)
           for num in pins if num and net(num)}

    # 2. supply connections first, as fixed copper (short, and they matter more than any signal's convenience):
    # the IC's GND pin to the nearest legal GND via
    R.to_pour("GND", R.t_pad(U, "6", escape="in", w=NECK_W, length=0.45) if spec.get("bar") else esc["6"],
              SUP_W, box, (IN1,), label="U GND", neck=neck)

    # 3. the signal nets, negotiated together
    gp = net("11")
    gate_vias = dict(spec["gate_vias"])
    gate_vias[gp] = gate_vias.get(gp)
    mains = [n for n in spec["gate_vias"] if n != gp]
    kp = next(n for n in parts if n.startswith(f"Net-({spec['shunt']}-Pad2"))
    csm, csp, vs = net("17"), net("18"), net("20")
    iscp = net("19") if net("19") != csm else None   # an ISCP of its own (42 V), else pin 19 is CS- too
    kv_p = via_near(kp, *spec["kelvin"][0])
    kv_m = via_near(csm, *spec["kelvin"][1])
    led_a = next(p["net"] for p in R.geo.pads if p["ref"] == spec["led"] and p["net"] != "GND")
    lv = via_near(led_a, *spec["led_via"])
    gate_nets = {bst, src, net("14"), net("15"), gp, *mains}
    rc = [n for n in parts if n.startswith("Net-(C") and any(ref.startswith("R") for ref, _ in parts[n])]
    gate_nets |= set(rc)

    def gate_segs():
        return [(x1, y1, x2, y2) for n, _, x1, y1, x2, y2, _ in R.geo.segs if n in gate_nets]

    def away_from_gate(win):                       # sense lines stay off the gate drive
        m = win.near_mask(gate_segs(), 0.0, 1.0)
        for l_ in win.layers:
            win.add_cost(l_, m, 2.5)

    def along_cs(win):                             # the Kelvin pair runs together
        away_from_gate(win)
        cs = [(x1, y1, x2, y2) for n, _, x1, y1, x2, y2, _ in R.geo.segs if n == csm]
        m = win.near_mask(cs, 0.0, 0.9)
        for l_ in win.layers:
            win.add_cost(l_, m, 0.55)

    def prefer_l3(win):                            # supply hops: down to L3 at the first via spot
        for l_ in win.layers:
            if l_ == B:
                win.cost[l_] = win.cost[l_] * 1.8

    def jobs(N):
        def add(n, nums, extra, w, label, **kw):
            N.add(n, [esc[k] for k in nums] + extra, w, label, **kw)
        add(bst, ["12"], loc(bst), GATE_W, "BST", neck=neck)
        # SRC is on the output net: 0.5 mm like all power copper, at pin width only in the pin escape area
        add(src, ["13"], [R.t_pad(cap, num) for cap in caps for ref, num in parts[src] if ref == cap], SUP_W,
            "SRC", neck=neck)
        add(net("14"), ["14"], loc(net("14")), GATE_W, "PD", neck=neck)
        add(net("15"), ["15"], loc(net("15")), GATE_W, "PU", neck=neck)
        for i, gm in enumerate(mains):
            add(gm, [], loc(gm) + [R.t_via(*gate_vias[gm])], GATE_W, "main gate" + (f" {i + 1}" if len(mains) > 1 else ""))
        add(gp, ["11"], loc(gp) + [R.t_via(*gate_vias[gp])], GATE_W, "pre-charge gate", neck=neck)
        for n in rc:
            add(n, [], loc(n), GATE_W, "pre-charge RC")
        add(csm, ["17"] if iscp else ["17", "19"], loc(csm) + [R.t_via(*kv_m)], SIG_W, "CS-", cost_fn=away_from_gate)
        if iscp:
            add(iscp, ["19"], loc(iscp), SIG_W, "ISCP", cost_fn=away_from_gate)
        add(csp, ["18"], loc(csp), SIG_W, "CS+", cost_fn=along_cs)
        add(kp, [], loc(kp) + [R.t_via(*kv_p)], SIG_W, "Kelvin +", cost_fn=along_cs)
        add(vs, ["20"], loc(vs), 0.3, "VS")
        for num, label in (("9", "TMR"), ("8", "IWRN"), ("7", "IMON"), ("1", "EN"), ("3", "INP"), ("2", "INPG"),
                           ("10", "temp sense")):
            add(net(num), [num], loc(net(num)), SIG_W, label, cost_fn=away_from_gate)
        add(net("4"), ["4", "5"], loc(net("4")), SIG_W, "FLT")
        # supply pads to their pours (input and output copper): they negotiate like the rest; a rail's run on L3
        # after a short stub (L3 under its cluster is only +3V3 fill)
        for n, zl in ((spec["vin"], spec["vin_layers"]), (spec["out"], (F, IN2))):
            for ref, num in parts.get(n, []):
                if FP[ref].IsFlipped():             # the bootstrap caps' SRC pads too: SRC's way to the source
                    N.add(n, [R.t_pad(ref, num)], SUP_W, f"{ref} {n}", zone_layers=zl, cost_fn=prefer_l3)
        add(led_a, [], loc(led_a) + [R.t_via(*lv)], SIG_W, "LED")
        for n in sorted(parts):
            if n.startswith("VSNS_"):
                add(n, [], loc(n), SIG_W, n)

    out_ = lambda num: R.t_pad(U, num, escape=True, w=NECK_W, length=0.45)
    in_ = lambda num: R.t_pad(U, num, escape="in", w=NECK_W, length=0.45)

    def around(win):                               # the CS pair goes over the top, not through the ISCP / VS region
        l_, t_, r_, b_ = spec["keep_cs_out"]
        m = (win.X >= l_) & (win.X <= r_) & (win.Y >= t_) & (win.Y <= b_)
        for ly in win.layers:
            win.add_cost(ly, m, 6.0)

    def cs_cost(fn):
        def f(win):
            fn(win)
            around(win)
        return f

    def jobs42(N):
        """42 V: every pin's side is set (the block is too tight to leave it to the negotiation): gate drive
        inward to the left, BST and SRC also up to the bootstrap caps; the CS pair up and over the top to the
        Kelvin vias, CS- and ISCP inward as well to the ISCP cap at the IC's right end; VS up and down its right
        side; EN down the channel beside the bypass bridge; the bottom row down."""
        pad_of = lambda ref, n: next(num for r_, num in parts[n] if r_ == ref)
        N.add(bst, [out_("12")] + [R.t_pad(c, pad_of(c, bst)) for c in caps], GATE_W, "BST", neck=neck)
        if spec.get("src_in"):                     # caps left of the IC: SRC under the body to the nearer one's
            near = min(caps, key=lambda c: math.dist(R.pad(c, pad_of(c, src))["c"], pins["13"]["c"]))
            N.add(src, [in_("13"), R.t_pad(near, pad_of(near, src))], SUP_W, "SRC", neck=neck)
            for c in caps:                         # output pad; each cap's output pad its own via to the output
                N.add(src, [R.t_pad(c, pad_of(c, src))], SUP_W, f"{c} to the output", zone_layers=(F, IN2))
        else:                                      # caps above: SRC up to them, and inward to the output
            N.add(src, [out_("13")] + [R.t_pad(c, pad_of(c, src)) for c in caps], SUP_W, "SRC", neck=neck)
            N.add(src, [in_("13")], SUP_W, "SRC to the output", zone_layers=(F, IN2), neck=neck)
        N.add(net("14"), [in_("14")] + loc(net("14")), GATE_W, "PD", neck=neck)
        N.add(net("15"), [in_("15")] + loc(net("15")), GATE_W, "PU", neck=neck)
        for i, gm in enumerate(mains):
            N.add(gm, loc(gm) + [R.t_via(*gate_vias[gm])], GATE_W, f"main gate {i + 1}")
        def g_cost(win):                           # the pre-charge gate keeps out of a corner other lines need
            l_, t_, r_, b_ = spec["g_avoid"]
            m = (win.X >= l_) & (win.X <= r_) & (win.Y >= t_) & (win.Y <= b_)
            for ly in win.layers:
                win.add_cost(ly, m, 6.0)
        N.add(gp, [in_("11")] + loc(gp) + [R.t_via(*gate_vias[gp])], GATE_W, "pre-charge gate", neck=neck,
              cost_fn=g_cost if spec.get("g_avoid") else None)
        for n in rc:
            N.add(n, loc(n), GATE_W, "pre-charge RC")
        N.add(csm, [out_("17"), R.t_via(*kv_m)], SIG_W, "CS- to its via", cost_fn=cs_cost(away_from_gate))
        N.add(csm, [in_("17")] + loc(csm), SIG_W, "CS- to the ISCP cap", cost_fn=away_from_gate)
        N.add(iscp, [out_("19")] + [t for t in loc(iscp) if not any(c.startswith("C") for c, _ in t["pads"])],
              SIG_W, "ISCP to its resistor", cost_fn=away_from_gate)
        N.add(iscp, [in_("19")] + [t for t in loc(iscp) if any(c.startswith("C") for c, _ in t["pads"])],
              SIG_W, "ISCP to its cap", cost_fn=away_from_gate)
        N.add(csp, [out_("18")] + loc(csp), SIG_W, "CS+", cost_fn=cs_cost(along_cs))
        N.add(kp, loc(kp) + [R.t_via(*kv_p)], SIG_W, "Kelvin +", cost_fn=cs_cost(along_cs))
        N.add(vs, [out_("20")] + loc(vs), 0.3, "VS")
        N.add(net("1"), [out_("1")] + loc(net("1")), SIG_W, "EN", cost_fn=away_from_gate)
        for num, label in (("9", "TMR"), ("8", "IWRN"), ("10", "temp sense")):
            N.add(net(num), [esc[num]] + loc(net(num)), SIG_W, label, cost_fn=away_from_gate)
        N.add(net("4"), [R.t_pad(U, "4"), R.t_pad(U, "5")], SIG_W, "FLT pins")   # its leg to the far end: later
        bar_bot = 36.2                             # the BUS42 bar's lower edge (power42): no input copper below
        low, top = [], []                          # input pads below the bar join the lowest one in it
        for n, zl in ((spec["vin"], spec["vin_layers"]), (spec["out"], (F, IN2))):
            for ref, num in parts.get(n, []):
                if FP[ref].IsFlipped() and inside(ref) and (ref not in caps or spec.get("src_in")):
                    y = R.pad(ref, num)["c"][1]
                    if n == spec["vin"] and spec.get("vin_join") and y > bar_bot:
                        low.append(R.t_pad(ref, num))
                        continue
                    if n == spec["vin"]:
                        top.append((y, ref, num))
                    if ref not in caps:            # (the caps' output pads: above, with SRC)
                        N.add(n, [R.t_pad(ref, num)], SUP_W, f"{ref} {n}", zone_layers=zl)
        if low and top:
            _, ref, num = max(top)
            N.add(spec["vin"], low + [R.t_pad(ref, num)], SUP_W, f"{spec['vin']} feed below the bar")
        N.add(led_a, loc(led_a) + [R.t_via(*lv)], SIG_W, "LED")

    N = route.Negotiator(R, box, origin=pins["12"]["c"])
    (jobs42 if spec.get("bar") else jobs)(N)
    bad = N.run(log=log)
    if bad:
        R.failed += bad
        log(f"   unrouted: {', '.join(bad)}")
        N.report(log=log)
    N.polish(log=log)

    # 4. GND: a via into the plane next to every GND pad where one fits (else the bottom GND fill takes it)
    for ref, num in parts.get("GND", []):
        if FP[ref].IsFlipped():
            R.fanout(ref, num, SUP_W, zone_layers=(IN1,), optional=True)
    R.prune_stubs(log=log)


def escape_area(b, ref, box):
    """A named rule area (no copper, no keepout) over an IC's pin rows and the band around them where tracks
    leave the pins at pin width. The design rules let power-net tracks be that narrow inside it only, so the
    TPS48111's SRC pin (on the output net) can leave its 0.5 mm pitch row before widening to 0.5 mm."""
    name = f"Pin escape {ref}"
    for z in list(b.Zones()):
        if z.GetIsRuleArea() and z.GetZoneName() == name:
            b.Remove(z)
    z = pcbnew.ZONE(b)
    z.SetIsRuleArea(True)
    z.SetDoNotAllowTracks(False)
    z.SetDoNotAllowVias(False)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowFootprints(False)
    z.SetDoNotAllowZoneFills(False)
    z.SetLayerSet(pcbnew.LSET.AllCuMask())
    z.SetZoneName(name)
    o = z.Outline()
    o.NewOutline()
    l, t, r, bt = box
    for x, y in ((l, t), (r, t), (r, bt), (l, bt)):
        o.Append(pcbnew.VECTOR2I(pcbnew.FromMM(X0 + x), pcbnew.FromMM(Y0 + y)))
    b.Add(z)


def _parts_on(parts, n1, n2):
    """The parts with pads on both nets (the bootstrap cap or caps on BST and SRC), sorted."""
    a = {ref for ref, _ in parts.get(n1, [])}
    b = {ref for ref, _ in parts.get(n2, [])}
    return sorted(a & b)
