"""The ESP32 group's leftovers, by hand (decided 2026-11-23): the status LEDs, the switches' second pads, two +3V3
pull-ups and the debug UART0's test-pad lines. Laid in their own stage (pcb_route esprest), after +5V_LOGIC, before
the router's stages; the router (esp_plan.local) keeps EN, IO0, the spare input and the AUX5 / NTC sense lines.

  status LEDs  D20 / D19 / D18 (red / yellow / green, top side, a column against the module's left pins) take
               their anodes from the resistors R162 / R161 / R160 (bottom side, by the LED escapes). No via fits
               between the LEDs' pads or between them and the module's pins, and L3 west of them is the FLT
               bus's wall (y 65.6 ... 66.6). So each anode gets a top-side line west to its own via, clear of
               that wall: D18 straight west, D19 west and south, D20 west and south furthest out. From the vias
               the bottom side fans out to the resistors in order (D20 outermost, round the others). The
               switch-control packer keeps out of the fan (C114 moves), and U21's 1 uF C119 stands 0.4 mm
               further west (logic_place.SW_SPOTS).
  LED GND      D19's and D18's cathodes tied on the top side east of the column, D18's on west to a GND via (D20's
               has its own, from the power copper).
  switches     SW1 / SW2 have two pad-1s each (the button's two contacts); the escapes reach one, a top-side line
               joins the other.
  +3V3         R159 (the spare input's pull-up) down beside the CAN lines into R142's +3V3 pad, their via in the
               bottom escape row's gap east of SDA's L3 column; TH1 (the NTC's top) between R146's pads up into
               C106's +3V3 pad (no room for a via of its own); C119 (moved) a via of its own south of it.
  UART0        RXD0 / TXD0 (module pins 36 / 37, the debug console's test pads TP38 / TP37 below the CAN block):
               both on L3 from their escape vias, side by side east of J32's pegs, then one bottom-side diagonal
               each to its pad. The router had zigzagged RXD0 under J32 between the AUX5 / CC resistors.
"""
W, W_SUP, W_GND = 0.25, 0.4, 0.3
LED_VIAS = {"D18": (126.15, 70.05), "D19": (125.25, 68.4), "D20": (124.4, 69.6)}
LED_KEEP_BOTTOM = [(124.0, 67.6, 128.8, 78.4)]           # the fan (logic_place: before the switch-control packer)
LED_KEEP_TOP = [(122.9, 60.8, 126.6, 73.2)]              # the anodes' top-side lines and vias (pcb_place / optimize)
D18_GND_VIA = (123.4, 72.6)
CATHODE_X = 128.84                                       # the cathodes' tie, between the LEDs and the module's pins
UART_L3_X = {"RXD0": 144.7, "TXD0": 145.45}
UART_VIAS = {"RXD0": (144.2, 103.4), "TXD0": (145.45, 110.6)}
UART_RXD0_Y = 81.36                                      # between TXD0's escape via and J32's west shield pad
TH1_X = 146.625                                          # between R146's pads
V3_R142_VIA = (136.9, 84.6)                              # R142 / R159's +3V3 via, in the main fill south of the
                                                         # escape row, between EN and BRAKE_ACT (R142 turned: its
                                                         # +3V3 pad east, its EN pad west)
R159_X = 135.0                                           # R159's +3V3 down to R142's
V3_R157_VIA = (126.25, 68.9)                             # FLT_12's pull-up (the +3V3 stage lost its spot to the LEDs)
# EN (module pin 3) from its escape via to R142 / C107 / the RESET switch: the bottom side west under the module past
# U12's tab, between the EPAD / C103 and C104, then a via, L3 through the bottom escape row's gap, a via by R142
EN_B = [(144.5, 63.43), (144.1, 63.43), (141.6, 65.93), (141.6, 69.5), (137.3, 73.8)]
EN_L3_X = 135.59                                         # the gap's east side: +3V3 fill passes west of it
EN_VIA2 = (135.59, 84.3)
V3_C119_VIA = (121.7, 72.6)


def logic_nets(R):
    """The nets planned here that the ESP32 group's router (esp_plan.local) would otherwise take."""
    pin = {q["num"]: q["net"] for q in R.geo.pads if q["ref"] == "U13"}
    return [pin[n] for n in ("3", "23", "24", "25", "36", "37")]


def route(R, FP, log=print):
    import esp_plan
    import far42
    import route_plan
    from route import B, F, IN2
    pads = R.geo.pads
    pin = lambda ref, num: [q for q in pads if q["ref"] == ref and q["num"] == num]
    c_net = lambda ref, net: next(q["c"] for q in pads if q["ref"] == ref and q["net"] == net)
    segs, vias = [], []

    def lay(net, layer, pts, w=W):
        segs.extend(R.track(net, layer, pts, w))

    def via(net, xy):
        vias.append(R.via(net, *xy))

    u13 = {q["num"]: q for q in pads if q["ref"] == "U13"}
    esc = esp_plan.escape_points(R)
    # ---- status LEDs: escape -> resistor (bottom); resistor -> fan -> via -> top-side line -> anode
    for num in ("23", "24", "25"):
        net = u13[num]["net"]
        (ex, ey), = esc[net]
        r = next(q for q in pads if q["net"] == net and q["ref"].startswith("R"))
        rx, ry = r["c"]
        if ry - ey < 1.0:                                  # (LED_1: the resistor a little lower, a short step)
            lay(net, B, [(ex, ey), (ex - (ry - ey), ry), (rx, ry)])
        else:                                              # (LED_2 / LED_3: 45 degrees down to the pad's row)
            lay(net, B, [(ex, ey), (ex - (ry - ey) if ry - ey < ex - rx else rx, ey + min(ry - ey, ex - rx)),
                         (rx, ry)])
    led = {}
    for d in ("D18", "D19", "D20"):
        an = next(q for q in pads if q["ref"] == d and q["num"] == "2")
        r = next(q for q in pads if q["net"] == an["net"] and q["ref"].startswith("R"))
        led[d] = (an["net"], an["c"], r["c"])
    v18, v19, v20 = LED_VIAS["D18"], LED_VIAS["D19"], LED_VIAS["D20"]
    n18, a18, r18 = led["D18"]
    n19, a19, r19 = led["D19"]
    n20, a20, r20 = led["D20"]
    # top side: anode west to its via
    lay(n18, F, [a18, v18])
    lay(n19, F, [a19, (v19[0] + 0.5, a19[1]), (v19[0], a19[1] + 0.5), v19])
    lay(n20, F, [a20, (v20[0], a20[1]), v20])
    for n, v in ((n18, v18), (n19, v19), (n20, v20)):
        via(n, v)
    # bottom side: the fan, D18 innermost (into its resistor from above), D20 outermost
    lay(n18, B, [v18, (r18[0], v18[1] + (r18[0] - v18[0])), r18])
    x19 = 126.25
    lay(n19, B, [r19, (x19 + 0.83, r19[1]), (x19, r19[1] - 0.83), (x19, v19[1] + 2.6 + (x19 - v19[0])),
                 (v19[0], v19[1] + 2.6), v19])
    x20 = 125.4
    lay(n20, B, [r20, (x20 + 1.11, r20[1]), (x20, r20[1] - 1.11), (x20, v20[1] + 1.8 + (x20 - v20[0])),
                 (v20[0], v20[1] + 1.8), v20])
    # cathodes: D19's and D18's tied east of the column, D18's west to a GND via
    k19 = next(q["c"] for q in pads if q["ref"] == "D19" and q["num"] == "1")
    k18 = next(q["c"] for q in pads if q["ref"] == "D18" and q["num"] == "1")
    lay("GND", F, [k19, (CATHODE_X, k19[1]), (CATHODE_X, k18[1]), k18], W_GND)
    lay("GND", F, [k18, (D18_GND_VIA[0] + 0.5, k18[1]), D18_GND_VIA], W_GND)
    via("GND", D18_GND_VIA)
    # ---- the switches' second pad-1s (top side)
    for sw in ("SW1", "SW2"):
        ones = sorted((q for q in pads if q["ref"] == sw and q["num"] == "1"), key=lambda q: q["c"][1])
        lay(ones[0]["net"], F, [ones[0]["c"], ones[1]["c"]])
    # ---- +3V3: R159 into R142's pad; TH1 between R146's pads into C106's
    r159 = c_net("R159", "+3V3")
    r142 = c_net("R142", "+3V3")
    xr = R159_X
    lay("+3V3", B, [r159, (xr, r159[1] + (xr - r159[0])), (xr, r142[1] - (r142[0] - xr)), r142], W_SUP)
    vx, vy = V3_R142_VIA
    lay("+3V3", B, [r142, (vx, r142[1] + (vx - r142[0])), (vx, vy)], W_SUP)
    via("+3V3", V3_R142_VIA)
    r157 = c_net("R157", "+3V3")
    lay("+3V3", B, [r157, V3_R157_VIA], W_SUP)
    via("+3V3", V3_R157_VIA)
    # ---- EN
    en = u13["3"]["net"]
    ex, ey = esp_plan.EN_VIA
    lay(en, B, [(ex, ey)] + EN_B)
    via(en, EN_B[-1])
    x1, y1 = EN_B[-1]
    lay(en, IN2, [(x1, y1), (EN_L3_X, y1 + (x1 - EN_L3_X)), (EN_L3_X, EN_VIA2[1])])
    via(en, EN_VIA2)
    r142e = c_net("R142", en)
    c107 = c_net("C107", en)
    ye = r142e[1] + 1.0                                               # (under R142, into its EN pad from below)
    lay(en, B, [EN_VIA2, (EN_VIA2[0] - (EN_VIA2[1] - ye), ye), (r142e[0], ye), r142e])
    lay(en, B, [EN_VIA2, (c107[0], EN_VIA2[1] + abs(c107[0] - EN_VIA2[0])), c107])
    sw1 = min((q["c"] for q in pads if q["ref"] == "SW1" and q["num"] == "1"), key=lambda c: c[1])
    lay(en, F, [EN_VIA2, (sw1[0], EN_VIA2[1])])                     # into the switch's pad link
    c119 = c_net("C119", "+3V3")                                     # (C119 moved 0.4 mm west: its own via, clear
    cv = V3_C119_VIA                                                 #  of the FLT bus's L3 diagonals)
    lay("+3V3", B, [c119, (c119[0], cv[1] - (cv[0] - c119[0])), cv], W_SUP)
    via("+3V3", cv)
    c106, c104 = c_net("C106", "+3V3"), c_net("C104", "+3V3")      # (C106's L3 pocket is closed: its pad to C104's)
    lay("+3V3", B, [c106, (c106[0] - 1.02, c106[1] + 1.02), (c104[0] + 0.51, c106[1] + 1.02), (c104[0], c104[1] + 0.5),
                    c104], W_SUP)
    th1 = c_net("TH1", "+3V3")
    c106 = c_net("C106", "+3V3")
    lay("+3V3", B, [th1, (TH1_X, th1[1]), (TH1_X, c106[1] + 0.3)], W_SUP)
    # ---- UART0: L3 from the escape vias, then one bottom-side diagonal each into its test pad
    for num, key in (("36", "RXD0"), ("37", "TXD0")):
        net = u13[num]["net"]
        (ex, ey), = esc[net]
        x3 = UART_L3_X[key]
        vx, vy = UART_VIAS[key]
        if key == "RXD0":                                  # under TXD0's via, over J32's west shield pad
            y0 = UART_RXD0_Y
            pts = [(ex, ey), (ex + (y0 - ey), y0), (x3 - 0.3, y0), (x3, y0 + 0.3)]
        else:
            pts = [(ex, ey), (x3, ey + (x3 - ex))]
        pts += [(x3, vy - (x3 - vx)), (vx, vy)] if vx < x3 else [(x3, vy)]
        lay(net, IN2, pts)
        via(net, (vx, vy))
        tp = next(q["c"] for q in pads if q["net"] == net and q["ref"].startswith("TP"))
        bend = (vx - ((tp[1] - vy) - 0.0), tp[1]) if (vx - tp[0]) > (tp[1] - vy) else None
        if bend and abs(bend[0] - tp[0]) > 1e-3:
            lay(net, B, [(vx, vy), (vx - (tp[1] - vy), tp[1]), tp])
        else:
            lay(net, B, [(vx, vy), tp])
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:25]:
        log(f"   ESP32 leftovers clearance: {msg}")
    if bad:
        R.failed.append("ESP32 leftovers")
    log(f"   ESP32 leftovers: {len(segs)} segments, {len(vias)} vias, {len(bad)} clearance problems")
