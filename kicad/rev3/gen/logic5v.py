"""+5V_LOGIC and USB VBUS, by hand (decided 2026-11-23).

The logic's 5 V node is a diode-OR of the aux buck's 5.3 V (D16, in the aux block) and USB VBUS (D17). It feeds the
LDO U12 (aux block, under the ESP32's top edge), the CAN transceiver (U16, south-west of the module) and the AUX5
sense divider's top (R147, by the USB receptacle). Between the USB receptacle (south of the module) and the aux block
(north of it) every way under the module is taken: the module's bottom-row escape vias and the north bundle's L3
crossbars, the EPAD and U12 on the bottom side, the top-row escape vias on L3, the bundle's L3 columns.

So D17 moved from the aux block to the USB end, and one line crosses: +5V_LOGIC up the board's east edge on the
bottom side, from D17's cathode to U12's input pin (where the aux block's +5V_LOGIC pour starts), outside the
module's parts and vias. On the way it takes R147. The ESP32's +3V3 / GND pin-1 vias stand 0.3 mm further west for it
(esp_plan). The CAN transceiver's branch leaves D17's cathode west on the bottom side below D17 (clear of the CAN
group's vias), drops to L3 east of the CAN logic lines' step and runs under them to a via above C109 (can_plan).

  VBUS        J32's south VBUS via (esp_plan.USB_VIA) down the bottom side into D17's anode, about 5 mm
  run         D17's cathode, x 149.1 (0.7 mm from the edge), 0.4 mm (the Supply class), into U12's pin 3
  CAN branch  D17's cathode south, west along y 96.4, north-west to a via at (135.3, 93.4); L3 west and south-west to
              a via above C109; C109 (can_plan) ties to U16's pin 3
  TP35        the +5V_LOGIC test pad (top side, aux block): a via beside it, the bottom side into C102's pad
  U12         its pin 2 tied to its tab (the same pin inside the package; KiCad asks for the link)

Laid in its own stage (pcb_route v5), after the CAN group, before the router's stages. Checked exactly.
"""
W = 0.4                                   # the Supply class minimum (D17 carries the logic on USB alone, ~0.35 A)
D17_XY = (145.85, 94.0)                   # flat on the bottom side under J32's south end, anode (VBUS) west
RUN_X = 149.1                             # +5V_LOGIC up the east edge (its clearance meets the edge band: no fill sliver)
BRANCH_Y = 96.4                           # the CAN branch: west below D17
BRANCH_VIAS = ((135.3, 93.4), (126.95, 98.9))   # down to L3 east of the CAN lines' step; up above C109
BRANCH_L3_X = 131.05                      # the L3 run turns south-west here
TP35_VIA = (145.45, 47.95)


def place_d17(fp):
    """D17 at the USB end (power_aux.place calls it instead of stacking it in the aux block)."""
    import power54
    from power_aux import _centre
    num = {p.GetNetname(): p.GetNumber() for p in fp.Pads() if p.GetNumber()}
    power54._flat_bottom(fp, *D17_XY, num["VBUS_USB"], num["+5V_LOGIC"])
    _centre(fp, *D17_XY)


def route(R, FP, log=print):
    import esp_plan
    import far42
    import route_plan
    from route import B, F, IN2
    pad = lambda ref, num: next(q for q in R.geo.pads if q["ref"] == ref and q["num"] == num)
    by_net = lambda ref, net: next(q for q in R.geo.pads if q["ref"] == ref and q["net"] == net)
    segs, vias = [], []

    def lay(net, layer, pts, w=W):
        segs.extend(R.track(net, layer, pts, w))

    def via(net, xy):
        vias.append(R.via(net, *xy))

    d17 = next(q["ref"] for q in R.geo.pads if q["net"] == "VBUS_USB" and q["ref"].startswith("D"))
    an, ca = by_net(d17, "VBUS_USB")["c"], by_net(d17, "+5V_LOGIC")["c"]
    # VBUS: from J32's south VBUS via straight down, then 45 degrees into the anode
    vx, vy = esp_plan.USB_VIA["VBUS_S"]
    y1 = an[1] - (an[0] - 0.95 - vx)
    lay("VBUS_USB", B, [(vx, vy), (vx, y1), (an[0] - 0.95, an[1]), an])
    # the run: cathode -> 45 degrees to the edge column -> north -> into the LDO's input pin
    u12 = pad("U12", "3")["c"]
    lay("+5V_LOGIC", B, [ca, (RUN_X, ca[1] - (RUN_X - ca[0])), (RUN_X, u12[1] + (RUN_X - u12[0])), u12])
    r147 = next(q for q in R.geo.pads if q["net"] == "+5V_LOGIC" and q["ref"].startswith("R"))["c"]
    lay("+5V_LOGIC", B, [r147, (RUN_X, r147[1])])
    # the CAN branch
    (b1x, b1y), (b2x, b2y) = BRANCH_VIAS
    turn = 140.3
    lay("+5V_LOGIC", B, [ca, (ca[0], BRANCH_Y), (turn, BRANCH_Y), (turn - (BRANCH_Y - b1y), b1y), (b1x, b1y)])
    via("+5V_LOGIC", (b1x, b1y))
    lay("+5V_LOGIC", IN2, [(b1x, b1y), (BRANCH_L3_X, b1y), (b2x, b1y + (BRANCH_L3_X - b2x)), (b2x, b2y)])
    via("+5V_LOGIC", (b2x, b2y))
    c109 = next(q for q in R.geo.pads if q["net"] == "+5V_LOGIC" and q["ref"].startswith("C")
                and abs(q["c"][0] - b2x) < 1.0)["c"]
    lay("+5V_LOGIC", B, [(b2x, b2y), c109])
    # TP35: its own via, then the bottom side into C102's +5V_LOGIC pad
    tp = by_net("TP35", "+5V_LOGIC")["c"] if any(q["ref"] == "TP35" for q in R.geo.pads) else \
        next(q for q in R.geo.pads if q["ref"].startswith("TP") and q["net"] == "+5V_LOGIC")["c"]
    tx, ty = TP35_VIA
    lay("+5V_LOGIC", F, [tp, (tx, tp[1] + (tx - tp[0])), (tx, ty)])
    via("+5V_LOGIC", (tx, ty))
    c102 = next(q for q in R.geo.pads if q["net"] == "+5V_LOGIC" and q["ref"].startswith("C") and q["c"][1] < 60)["c"]
    lay("+5V_LOGIC", B, [(tx, ty), (c102[0], ty)])
    # U12: pin 2 to its tab, round the left of pin 3
    p2 = min((q for q in R.geo.pads if q["ref"] == "U12" and q["num"] == "2"), key=lambda q: q["box"][1])
    tab = max((q for q in R.geo.pads if q["ref"] == "U12" and q["num"] == "2"), key=lambda q: q["box"][1])
    (px, py), x2 = p2["c"], p2["box"][2] + 0.1
    lay("+3V3", B, [(px, py), (px, p2["box"][3]), (x2, p2["box"][3] + (x2 - px)), (x2, tab["box"][1] + 0.3)])
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:25]:
        log(f"   +5V_LOGIC clearance: {msg}")
    if bad:
        R.failed.append("+5V_LOGIC / VBUS")
    log(f"   +5V_LOGIC and VBUS: {len(segs)} segments, {len(vias)} vias, {len(bad)} clearance problems")
