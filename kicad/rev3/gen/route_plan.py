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

import pcbnew

import power_rails
import route
from power42 import X0, Y0
from route import B, F, IN1, IN2

GATE_W, NECK_W, SIG_W, SUP_W = 0.4, 0.25, 0.25, 0.5    # supply: the power rule's 0.5 mm


def _tps(FP, sheet):
    return next(f for f in FP.values() if f.GetSheetname() == sheet and "TPS48111" in f.GetValue())


def efuse_rail(R, FP, key, log=print):
    r = power_rails.RAILS[key]
    g = power_rails._geo(r)
    sheet = FP[r["fets"][1]].GetSheetname()
    U = _tps(FP, sheet).GetReference()
    pre, main = power_rails.FET_ROWS
    box = (r["xl"] - 0.5, 10.0, r["xr"] + 0.5, 73.0)
    pins = {p["num"]: p for p in R.geo.pads if p["ref"] == U}
    net = lambda num: pins[num]["net"]
    parts = {}                                     # net -> [(ref, num)] of the sheet's other parts
    for p in R.geo.pads:
        if p["ref"] != U and FP[p["ref"]].GetSheetname() == sheet:
            parts.setdefault(p["net"], []).append((p["ref"], p["num"]))
    loc = lambda n: [R.t_pad(ref, num) for ref, num in parts.get(n, []) if FP[ref].IsFlipped()]   # bottom parts
    l, t, rr, b = (min(p["box"][0] for p in pins.values()), min(p["box"][1] for p in pins.values()),
                   max(p["box"][2] for p in pins.values()), max(p["box"][3] for p in pins.values()))
    neck = ((l - 1.2, t - 1.2, rr + 1.2, b + 1.2), NECK_W)
    escape_area(R.b, U, neck[0])                   # where tracks leave the pins at pin width (board_rules.py)
    via_near = lambda net_, x, y: min(((vx, vy) for n, vx, vy, vr in R.geo.vias if n == net_),
                                      key=lambda q: math.hypot(q[0] - x, q[1] - y))
    bst, src = net("12"), net("13")
    cap = _part_on(parts, bst, src)

    # 1. IC pins are plain pad terminals: a route leaves through either end of the pad (outward, or inward
    # under the body between the rows), whichever the negotiation settles on. Except the bottom-row pins that
    # leave downwards (temperature diode, TMR, IWRN, IMON, INP, INPG): they first run straight out past their
    # neighbours' clearance, so below the pad row the fan can stagger its turns freely (parallel 45-degree jogs at
    # 0.5 mm pitch need each turn lower than its right-hand neighbour's; at the pad row there is no room for that)
    down = {"10", "9", "8", "7", "3", "2"}
    esc = {num: R.t_pad(U, num, escape=True, w=NECK_W, length=0.45) if num in down else R.t_pad(U, num)
           for num in pins if num and net(num)}

    # 2. supply connections first, as fixed copper (short, and they matter more than any signal's convenience):
    # the IC's GND pin under the body to the nearest legal GND via, VIN / OUT pads straight to their pours
    R.to_pour("GND", esc["6"], SUP_W, box, (IN1,), label="U GND", neck=neck)

    # 3. the signal nets, negotiated together
    gate_via_main = (g["fet_cx"] - 10.85, main - 2.54)
    gate_via_pre = (g["fet_cx"] - 10.85, pre - 2.54)
    gm = next(p["net"] for p in R.geo.pads if p["ref"] == r["fets"][1] and p["num"] == "1")
    gp = net("11")
    kp = next(n for n in parts if n.startswith(f"Net-({r['shunt']}-Pad2"))
    csm, csp, vs = net("17"), net("18"), net("20")
    sv = via_near(src, g["fet_cx"] - 6.1, main + 0.1)
    kv_m = via_near(csm, g["shunt_x"] + 2.7, power_rails.Y_SHUNT + 2.7)
    kv_p = via_near(kp, g["shunt_x"] + 2.7, power_rails.Y_SHUNT - 2.2)
    led_a = next(p["net"] for p in R.geo.pads if p["ref"] == r["led"] and p["net"] != "GND")
    lv = via_near(led_a, g["side_x"] + 1.7, power_rails.LED_Y + 1.02)
    gate_nets = {bst, src, net("14"), net("15"), gm, gp}
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

    pin_jobs = {}                                  # label -> pins it starts from (for the inward retry)

    def jobs(N):
        def add(n, nums, extra, w, label, **kw):
            N.add(n, [esc[k] for k in nums] + extra, w, label, **kw)
            pin_jobs[label] = nums
        add(bst, ["12"], loc(bst), GATE_W, "BST", neck=neck)
        # SRC is on the output net: 0.5 mm like all power copper, at pin width only in the pin escape area
        add(src, ["13"], [R.t_pad(cap, num) for ref, num in parts[src] if ref == cap], SUP_W, "SRC", neck=neck)
        add(net("14"), ["14"], loc(net("14")), GATE_W, "PD", neck=neck)
        add(net("15"), ["15"], loc(net("15")), GATE_W, "PU", neck=neck)
        add(gm, [], loc(gm) + [R.t_via(*gate_via_main)], GATE_W, "main gate")
        add(gp, ["11"], loc(gp) + [R.t_via(*gate_via_pre)], GATE_W, "pre-charge gate", neck=neck)
        for n in rc:
            add(n, [], loc(n), GATE_W, "pre-charge RC")
        add(csm, ["17", "19"], [R.t_via(*kv_m)], SIG_W, "CS-", cost_fn=away_from_gate)
        add(csp, ["18"], loc(csp), SIG_W, "CS+", cost_fn=along_cs)
        add(kp, [], loc(kp) + [R.t_via(*kv_p)], SIG_W, "Kelvin +", cost_fn=along_cs)
        add(vs, ["20"], loc(vs), 0.3, "VS")
        for num, label in (("9", "TMR"), ("8", "IWRN"), ("7", "IMON"), ("1", "EN"), ("3", "INP"), ("2", "INPG"),
                           ("10", "temp sense")):
            add(net(num), [num], loc(net(num)), SIG_W, label, cost_fn=away_from_gate)
        add(net("4"), ["4", "5"], loc(net("4")), SIG_W, "FLT")
        # supply pads to their pours (VIN up in the input area, OUT the strip / the area below the switch): they
        # negotiate like the rest, and run on L3 after a short stub (L3 under the cluster is only +3V3 fill)
        for n, zl in ((r["vin"], (F,)), (r["out"], (F, IN2))):
            for ref, num in parts.get(n, []):
                if FP[ref].IsFlipped():             # the bootstrap cap's SRC pad too: SRC's way to the source
                    N.add(n, [R.t_pad(ref, num)], SUP_W, f"{ref} {n}", zone_layers=zl, cost_fn=prefer_l3)
        add(led_a, [], loc(led_a) + [R.t_via(*lv)], SIG_W, "LED")
        for n in sorted(parts):
            if n.startswith("VSNS_"):
                add(n, [], loc(n), SIG_W, n)

    N = route.Negotiator(R, box, origin=pins["12"]["c"])
    jobs(N)
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


def _part_on(parts, n1, n2):
    """The part with pads on both nets (e.g. the bootstrap cap on BST and SRC)."""
    a = {ref for ref, _ in parts.get(n1, [])}
    b = {ref for ref, _ in parts.get(n2, [])}
    both = a & b
    return next(iter(both)) if both else None
