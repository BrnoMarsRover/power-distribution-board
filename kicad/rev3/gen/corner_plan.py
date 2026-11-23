"""The power-button corner (top right), by hand (decided 2026-11-23).

The parts: the PhotoMOS U22 (the ESP32's hold on the BMS soft-start pair, parallel to the button's pole 1), its LED
resistor R172 and PWR_HOLD's pull-down R173; the button sense's pull-up R174; the motor switch loop's input
resistors (R166 from +3V3 into J36.1, R168 from J36.2 to GND, R167 from J36.2 to MOTOR_OK). The connectors: J37 (the
button: SS_A, SS_B, BTN_SNS, GND), J38 (the BMS soft-start pair, through the E-STOP) and J36 (the motor switch loop),
all through-hole. The north bundle (esp_plan) brings PWR_HOLD, MOTOR_OK and BTN_SNS west on L3 between J37 and J36.

  soft-start  J38 to J37 on the top side (through-hole to through-hole, clear of everything on the bottom); U22's
              output pins (3 SS_A above 4 SS_B, facing east) to J37: SS_A along the bottom side, SS_B through a via
              and along the top side (U22's pin order would make the two cross on one layer)
  PWR_HOLD    the bundle's port moves west, above U22: a via there, the bottom side north to R172 (on to U22's LED
              anode) with R173 to GND on the way
  MOTOR_OK    the port moves east, beside J36: R167 (MOTOR_OK west, the loop node east) right above J36's pin 2,
              R168 above it into J37's GND pin, R166 west of J36 (its +3V3 via below it, out of the bundle's band)
  BTN_SNS     R174 right above J37's pin 3, its +3V3 via beside it (BTN_SNS reaches pin 3 on L3, esp_plan)

No via goes in the bundle's L3 band (y 15.9 ... 19.1) east of the ports. Laid in the ESP32 group's stage, after the
bundle (pcb_route esp, esp_plan.corner), as planned polylines checked exactly.
"""
W, W_SS = 0.25, 0.4
PORT_X = {"MOTOR_OK": 143.9, "PWR_HOLD": 131.0}          # (esp_plan.PORT_X: the bundle's L3 lines end here)
# (nets: first one up / west), centre, upright?
PARTS = [(("+3V3", "Net-(J36-Pin_1)"), (140.6, 22.6), False),       # R166
         (("MOTOR_OK", "Net-(J36-Pin_2)"), (145.0, 19.0), False),   # R167
         (("GND", "Net-(J36-Pin_2)"), (146.41, 15.6), True),        # R168
         (("GND", "PWR_HOLD"), (128.8, 12.1), False),               # R173
         (("+3V3", "BTN_SNS"), (143.91, 9.3), True),                # R174
         (("Net-(R172-Pad2)", "PWR_HOLD"), (128.77, 9.74), False)]  # R172
SS_B_VIA = (135.2, 7.3)
V3_VIAS = {"R166": (139.775, 23.95), "R174": (142.6, 8.475)}
R173_GND_VIA = (126.75, 12.1)
SS_F = {"SS_A": [(134.62, 21.5), (134.62, 16.0), (138.32, 12.3), (138.91, 12.3)],
        "SS_B": [(137.12, 21.5), (138.9, 19.72), (138.9, 15.0), (141.41, 12.49), (141.41, 12.3)]}


def place(FP, obstacles):
    """The corner's resistors by hand (locked; U22 is placed by logic_place). Returns the references placed."""
    import power54
    from power42 import _upright
    from satellites import _box
    sheet = FP["U22"].GetSheetname()
    short = lambda p: p.GetNetname().split("/")[-1]
    done = []
    for (a, b), (x, y), upright in PARTS:
        ref = next(r for r, f in FP.items() if f.GetSheetname() == sheet and r[0] == "R" and r not in done
                   and {short(p) for p in f.Pads()} == {a, b})
        pa = next(p.GetNumber() for p in FP[ref].Pads() if short(p) == a)
        pb = next(p.GetNumber() for p in FP[ref].Pads() if short(p) == b)
        if upright:
            _upright(FP[ref], x, y, pa, pb, bottom=True)
        else:
            power54._flat_bottom(FP[ref], x, y, pa, pb)
        FP[ref].SetLocked(True)
        obstacles.append(_box(FP[ref]))
        done.append(ref)
    return done


def route(R, FP, log=print):
    """The corner's copper, planned (checked exactly)."""
    import far42
    import route_plan
    from route import B, F
    short = lambda q: q["net"].split("/")[-1]
    P = lambda ref, n: next(q for q in R.geo.pads if q["ref"] == ref and short(q) == n)
    refs = lambda a, b: next(q["ref"] for q in R.geo.pads if short(q) == a and q["ref"].startswith("R")
                             and any(short(o) == b for o in R.geo.pads if o["ref"] == q["ref"]))
    c = lambda ref, n: P(ref, n)["c"]
    segs, vias = [], []

    def lay(net, layer, pts, w=W):
        segs.extend(R.track(net, layer, pts, w))

    def via(net, xy):
        vias.append(R.via(net, *xy))

    full = lambda n: next(q["net"] for q in R.geo.pads if short(q) == n)
    ssa, ssb = full("SS_A"), full("SS_B")
    u = {q["num"]: q for q in R.geo.pads if q["ref"] == "U22"}
    j37 = {q["num"]: q["c"] for q in R.geo.pads if q["ref"] == "J37"}
    # soft-start: J38 to J37 on top; U22's SS_A along the bottom, SS_B up through a via, along the top
    lay(ssa, F, SS_F["SS_A"], W_SS)
    lay(ssb, F, SS_F["SS_B"], W_SS)
    p3, p4 = u["3"]["c"], u["4"]["c"]
    lay(ssa, B, [p3, (j37["1"][0], p3[1]), j37["1"]], W_SS)
    lay(ssb, B, [p4, SS_B_VIA], W_SS)
    via(ssb, SS_B_VIA)
    lay(ssb, F, [SS_B_VIA, (j37["2"][0], SS_B_VIA[1]), j37["2"]], W_SS)
    # PWR_HOLD: the port's via, north to R172 with R173 on the way; R172 on to U22's LED anode
    r172, r173 = refs("PWR_HOLD", "Net-(R172-Pad2)"), refs("PWR_HOLD", "GND")
    pv = (PORT_X["PWR_HOLD"], 18.4)
    via("PWR_HOLD", pv)
    h2, h3 = c(r172, "PWR_HOLD"), c(r173, "PWR_HOLD")
    lay("PWR_HOLD", B, [pv, (pv[0], h2[1] + (pv[0] - h2[0]) - 0.87), (h2[0] + 0.87, h2[1]), h2])
    lay("PWR_HOLD", B, [(pv[0], h3[1]), h3])
    g3 = c(r173, "GND")
    lay("GND", B, [g3, R173_GND_VIA], 0.4)
    via("GND", R173_GND_VIA)
    a2, a1 = c(r172, "Net-(R172-Pad2)"), u["1"]["c"]
    lay(u["1"]["net"], B, [a2, (a1[0], a2[1] - (a2[0] - a1[0])), a1])
    # BTN_SNS: R174 into J37's pin 3, its +3V3 via
    r174 = refs("BTN_SNS", "+3V3")
    lay(full("BTN_SNS"), B, [c(r174, "BTN_SNS"), j37["3"]])
    lay("+3V3", B, [c(r174, "+3V3"), V3_VIAS["R174"]], 0.4)
    via("+3V3", V3_VIAS["R174"])
    # the motor switch loop: R166 into J36.1, +3V3 via below it; the node J36.2 - R167 - R168; R168 into J37's GND
    r166, r167, r168 = refs("Net-(J36-Pin_1)", "+3V3"), refs("Net-(J36-Pin_2)", "MOTOR_OK"), \
        refs("Net-(J36-Pin_2)", "GND")
    j36 = {q["num"]: q for q in R.geo.pads if q["ref"] == "J36"}
    e166 = c(r166, "Net-(J36-Pin_1)")
    yj = j36["1"]["box"][3] - 0.5
    lay(j36["1"]["net"], B, [e166, (e166[0] + (e166[1] - yj), yj), (j36["1"]["c"][0], yj)])
    lay("+3V3", B, [c(r166, "+3V3"), V3_VIAS["R166"]], 0.4)
    via("+3V3", V3_VIAS["R166"])
    n2, k167, k168 = j36["2"]["net"], c(r167, "Net-(J36-Pin_2)"), c(r168, "Net-(J36-Pin_2)")
    lay(n2, B, [k168, j36["2"]["c"]])
    lay(n2, B, [k167, (k168[0], k167[1])])
    lay("GND", B, [c(r168, "GND"), j37["4"]], 0.4)
    mv = (PORT_X["MOTOR_OK"], 17.5)
    via("MOTOR_OK", mv)
    m7 = c(r167, "MOTOR_OK")
    lay("MOTOR_OK", B, [m7, (mv[0], m7[1] - (m7[0] - mv[0])), mv])
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:25]:
        log(f"   power-button corner clearance: {msg}")
    if bad:
        R.failed.append("power-button corner")
    log(f"   power-button corner: {len(segs)} segments, {len(vias)} vias, {len(bad)} clearance problems")
