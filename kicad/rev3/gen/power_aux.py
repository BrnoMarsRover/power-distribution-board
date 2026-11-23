"""Hand-designed layout of the always-on aux supply (LM5017 buck 42 V -> 5.3 V, OR diodes, AP7361 -> 3.3 V).

Used by pcb_place.py (placement, before the packer) and pcb_power42.py (copper, after the optimizer).
Board mm from the top-left corner. Everything is on the bottom, beside the ESP32 it powers, under the space the
54 V eFuse left (x 126-149.4, y 31-67).

Stack, top to bottom:
  input ceramics (one straight above U11's VIN / RTN pins: the buck's hot loop is that cap and the IC)
  -> U11 (LM5017: VIN / RTN pins on top, SW pin under them) -> L2 standing, its SW pad right under U11's SW pin
  -> output ceramics in a row under L2's 5.3 V pad
  right column: the OR diode D16 (aux 5.3 V) lying flat level with L2's 5.3 V pad, cathode on the right, joined on
  +5V_LOGIC with the 5 V cap above it; the AP7361 under it, input pin up, tab down, its 3.3 V cap beside the tab.
  (The USB OR diode D17 stood above D16 until 2026-11-23; it now sits at the USB receptacle, logic5v.)
BUS42 arrives on an L3 track (about 0.1 A) from the 42 V bar, along y 21.7 above the rail blocks' L3
copies; the logic rails leave by routing.
"""
import pcbnew

from power42 import MM, TOMM, X0, Y0, _at, _pad, _pxy

U11_XY = (132.65, 37.6)              # 2 mm higher than first drawn: the band under its pin row takes the BST cap
L2_TOP = 43.94                       # L2's courtyard top (unchanged; it was U11's bottom + GAP)
# the controller's own parts (decided 2026-11-22, by hand; the first layout left BST, VCC and FB no way out between
# the pin row and the switch-node pour): centres
C_IN1_XY = (127.65, 35.16)           # VIN cap, upright west of pins 1 / 2, BUS42 pad up (VIN comes over pin 1)
C_IN2_XY = (134.6, 25.2)             # second input cap, flat at the BUS42 feed, BUS42 pad west
C_BST_XY = (131.38, 42.62)           # flat under pins 8 / 7, SW pad west
C_VCC_XY = (137.6, 42.62)            # flat past the bottom-right corner, VCC pad west
BUS42_Y = 21.7                       # L3 feed from the 42 V bar runs along here (rail L3 copies start at 22.5)
GAP = 0.6
C5_BOTTOM = 48.75                    # the 5 V cap's courtyard bottom (where D17 above D16 had put it)
# the power stage's copper; the pocket right of U11 (x 135.3-149.4, y 38.2-44.8, and above it right of the input
# caps) is left for U11's resistors (placed by logic_place.py)
# and the GND-fill patch between L2 and the diodes (x 139.9-145.8, y 44.8-49.4)
KEEP_BOTTOM = [(126.1, 31.9, 140.5, 38.2), (126.1, 38.2, 135.3, 67.0), (135.3, 49.4, 149.4, 67.0),
               (135.3, 44.8, 139.9, 49.4), (145.8, 44.8, 149.4, 49.4)]     # (the network's parts are locked: no box)
KEEP_TOP = [(126.6, 31.6, 135.4, 43.4)]           # vias of U11's pad, the input caps' GND, the BUS42 feed: no top parts
# the network by hand (decided 2026-11-22): RON / UVLO north of U11 (pins 3 / 4 go straight up), hanging from a
# BUS42 bus at y 27.6 fed by its own L3 via; the FB divider east of pin 5 (FB straight into it); the ripple
# injection (Rr from SW, Cr to VOUT, Cac to FB) in the pocket beside L2. (nets: first one up / west), centre, upright
NET_RON, NET_UVLO, NET_FB, NET_A = "Net-(U11-RON)", "Net-(U11-UVLO)", "Net-(U11-FB)", "Net-(C100-Pad2)"
NETWORK = [(("BUS42", NET_RON), (136.0, 30.2), True),             # R136
           (("BUS42", NET_UVLO), (133.28, 30.2), True),           # R137
           (("GND", NET_UVLO), (131.0, 30.6), True),              # R138
           ((NET_FB, "+5V3_AUX"), (137.6, 40.08), False),         # R140, level with pin 5
           ((NET_FB, "GND"), (137.6, 37.98), False),              # R141, its GND via above it
           ((NET_FB, NET_A), (141.5, 42.3), False),               # C101 (Cac)
           (("SW", NET_A), (141.7, 45.0), False),                 # R139 (Rr), SW straight in under L2's body
           (("+5V3_AUX", NET_A), (141.5, 47.7), False)]           # C100 (Cr), VOUT from the output pour
VOUT_X = 139.3                       # VOUT: up the bottom side from the output pour to Cr, an L3 hop to the divider
# the aux test pads (top side) by hand, over their nets: SW beside a via into the small switch-node pour (not into
# L2's pad), VCC on a via under its cap's pad, VOUT on the L3 hop's via; +5V_LOGIC and +3V3 in the free top area
# east (their links come with the +5V_LOGIC / +3V3 step)
TP_XY = {"SW": (131.9, 45.6), "Net-(U11-VCC)": (136.74, 43.6), "+5V3_AUX": (VOUT_X, 47.7),
         "+5V_LOGIC": (144.6, 46.6), "+3V3": (144.6, 53.0)}
SW_TP_VIA = (130.5, 43.65)
FEED_VIAS = {"cap": (128.2, 32.3), "bus": (132.0, 27.6)}         # the BUS42 feed's two vias (L3); the cap's keeps
                                                                 # 0.25 mm from the 5 V LED's anode pad (top side)
BUS_Y = 27.6
C_IN1_GND_VIAS = ((126.5, 38.35), (128.5, 38.35))   # (VSNS_IN_5 runs down L3 between them, x 127.4)


def _box(fp):
    fp.BuildCourtyardCaches()
    bb = fp.GetCourtyard(pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd).BBox()
    return TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0, TOMM(bb.GetBottom()) - Y0


def _shift(fp, dx, dy):
    fp.SetPosition(fp.GetPosition() + pcbnew.VECTOR2I(MM(dx), MM(dy)))


def _orient(fp, test):
    """Bottom side, first quarter turn for which test(fp) holds."""
    for deg in (0, 90, 180, 270):
        _at(fp, 0.0, 0.0, deg, bottom=True)
        if test(fp):
            return
    raise RuntimeError(f"{fp.GetReference()}: no orientation")


def _put(fp, left=None, top=None, right=None, bottom=None):
    """Move by its courtyard to the given edges."""
    l, t, r, b = _box(fp)
    dx = (left - l) if left is not None else (right - r) if right is not None else 0.0
    dy = (top - t) if top is not None else (bottom - b) if bottom is not None else 0.0
    _shift(fp, dx, dy)


def _nets(fp):
    return {p.GetNetname() for p in fp.Pads()}


def _centre(fp, x, y):
    """Move so the pads' midpoint sits at (x, y)."""
    pts = [_pxy(fp, p.GetNumber()) for p in fp.Pads() if p.GetNumber()]
    _shift(fp, x - sum(p[0] for p in pts) / len(pts), y - sum(p[1] for p in pts) / len(pts))


def parts(FP):
    """The aux supply's power parts, found by net (reference numbers move when sheets change)."""
    u = FP["U11"] if "U11" in FP else next(f for f in FP.values() if "LM5017" in f.GetValue())
    sw, vcc, bst = (_pad(u, n).GetNetname() for n in ("8", "6", "7"))
    ref = lambda test: sorted((r for r, f in FP.items() if test(r, f)), key=lambda r: int(r[1:]))
    cin = ref(lambda r, f: r.startswith("C") and _nets(f) == {"BUS42", "GND"} and f.GetSheetname().startswith("Always"))
    cout = ref(lambda r, f: r.startswith("C") and _nets(f) == {"+5V3_AUX", "GND"})
    ind = ref(lambda r, f: r.startswith("L") and sw in _nets(f))[0]
    cbst = ref(lambda r, f: r.startswith("C") and _nets(f) == {sw, bst})[0]
    cvcc = ref(lambda r, f: r.startswith("C") and _nets(f) == {vcc, "GND"})[0]
    d_aux = ref(lambda r, f: r.startswith("D") and _nets(f) == {"+5V3_AUX", "+5V_LOGIC"})[0]
    d_usb = ref(lambda r, f: r.startswith("D") and _nets(f) == {"VBUS_USB", "+5V_LOGIC"})[0]
    ldo = next(r for r, f in FP.items() if "AP7361" in f.GetValue())
    c5 = ref(lambda r, f: r.startswith("C") and _nets(f) == {"+5V_LOGIC", "GND"} and f.GetSheetname().startswith("Always"))
    c33 = ref(lambda r, f: r.startswith("C") and _nets(f) == {"+3V3", "GND"} and f.GetSheetname().startswith("Always")
              and "0805" in f.GetFPIDAsString())
    return dict(u=u.GetReference(), cin=cin, cout=cout, ind=ind, cbst=cbst, cvcc=cvcc, d_aux=d_aux, d_usb=d_usb,
                ldo=ldo, c5=c5, c33=c33)


def place(FP):
    """Position and lock the aux supply's power parts. Returns the refs placed."""
    P = parts(FP)
    u = FP[P["u"]]
    # U11: VIN / RTN (pins 1, 2) on the top row, pin 1 left; SW (pin 8) straight under pin 1
    _orient(u, lambda f: _pxy(f, "1")[1] < _pxy(f, "8")[1] - 3 and _pxy(f, "1")[0] < _pxy(f, "4")[0])
    _shift(u, U11_XY[0] - TOMM(u.GetPosition().x) + X0, U11_XY[1] - TOMM(u.GetPosition().y) + Y0)
    ub = _box(u)
    p1, p2 = _pxy(u, "1"), _pxy(u, "2")
    # the input caps: 1 upright west of pins 1 / 2 (BUS42 pad up), 2 flat at the BUS42 feed (BUS42 pad west)
    c1 = FP[P["cin"][0]]
    _orient(c1, lambda f: _pxy(f, "1")[1] < _pxy(f, "2")[1] - 1)
    _centre(c1, *C_IN1_XY)
    c2 = FP[P["cin"][1]]
    _orient(c2, lambda f: _pxy(f, "1")[0] < _pxy(f, "2")[0] - 1)
    _centre(c2, *C_IN2_XY)
    # bootstrap cap flat under pins 8 / 7 (SW pad west), VCC cap flat past the corner (VCC pad west)
    sw_n = _pad(u, "8").GetNetname()
    for ref, xy in ((P["cbst"], C_BST_XY), (P["cvcc"], C_VCC_XY)):
        f = FP[ref]
        west = next(p.GetNumber() for p in f.Pads() if p.GetNetname() in (sw_n, _pad(u, "6").GetNetname()))
        east = next(p.GetNumber() for p in f.Pads() if p.GetNumber() != west)
        _orient(f, lambda f_: _pxy(f_, west)[0] < _pxy(f_, east)[0] - 1)
        _centre(f, *xy)
    # L2 standing, SW pad (1) up, right under U11's SW pin
    ind = FP[P["ind"]]
    _orient(ind, lambda f: _pxy(f, "1")[1] < _pxy(f, "2")[1] - 1)
    sx = _pxy(u, "8")[0]
    _shift(ind, sx + 2.0 - _pxy(ind, "1")[0], 0.0)
    _put(ind, top=L2_TOP)
    lb = _box(ind)
    if lb[0] < 126.1:
        _put(ind, left=126.1)
        lb = _box(ind)
    # output caps standing in a row under L2, 5.3 V pad up
    x = 126.1
    for ref in P["cout"]:
        f = FP[ref]
        _orient(f, lambda f_: _pxy(f_, "1")[1] < _pxy(f_, "2")[1] - 1)
        _put(f, left=x, top=lb[3] + GAP)
        x = _box(f)[2] + GAP
    # the aux OR diode lying flat at the right edge, anode left, level with L2's 5.3 V pad; the USB one at the USB
    # receptacle (logic5v, 2026-11-23)
    lout_y = _pxy(ind, "2")[1]
    da = FP[P["d_aux"]]
    _orient(da, lambda f: _pxy(f, "2")[0] < _pxy(f, "1")[0] - 1)
    _put(da, right=149.4)
    _shift(da, 0.0, lout_y - _pxy(da, "2")[1])
    import logic5v
    logic5v.place_d17(FP[P["d_usb"]])
    # 5 V cap standing at the cathode end, where it stood above D17 (its +5V_LOGIC pad on the column's pour)
    c5 = FP[P["c5"][0]]
    _orient(c5, lambda f: abs(_pxy(f, "1")[0] - _pxy(f, "2")[0]) < 0.05 and _pxy(f, "1")[1] > _pxy(f, "2")[1])
    _put(c5, right=149.4, bottom=C5_BOTTOM)
    # LDO under D16: pins up (input pin toward the cathodes), tab down; its 3.3 V cap left of the tab
    ldo = FP[P["ldo"]]
    _orient(ldo, lambda f: max(TOMM(p.GetPosition().y) for p in f.Pads() if p.GetNumber() == "2"
                               and p.GetSize().x * p.GetSize().y > MM(1) * MM(5)) - Y0 > _pxy(f, "3")[1] + 2
            and _pxy(f, "3")[0] > _pxy(f, "1")[0])
    _put(ldo, right=149.4, top=_box(da)[3] + GAP)
    c33 = FP[P["c33"][0]]
    _orient(c33, lambda f: abs(_pxy(f, "1")[0] - _pxy(f, "2")[0]) < 0.05 and _pxy(f, "1")[1] < _pxy(f, "2")[1])
    lb2 = _box(ldo)
    _put(c33, right=lb2[0] - GAP, bottom=lb2[3])
    done = [P["u"], *P["cin"], P["cbst"], P["cvcc"], P["ind"], *P["cout"], P["d_usb"], P["d_aux"], P["ldo"],
            P["c5"][0], P["c33"][0]]
    for ref in done:
        FP[ref].SetLocked(True)
    return done


def place_network(FP, obstacles):
    """The LM5017's RON / UVLO / FB / ripple parts by hand (locked). Returns the references placed."""
    from power42 import _upright, _flat
    import power54
    from satellites import _box
    u = FP["U11"] if "U11" in FP else next(f for f in FP.values() if "LM5017" in f.GetValue())
    sw = _pad(u, "8").GetNetname()
    sheet = u.GetSheetname()
    short = lambda p: p.GetNetname() if p.GetNetname() != sw else "SW"
    done = []
    for (a, b), (x, y), upright in NETWORK:
        ref = next(r for r, f in FP.items() if f.GetSheetname() == sheet and r[0] in "RC" and r not in done
                   and not f.IsLocked() and {short(p) for p in f.Pads()} == {a, b})
        pa = next(p.GetNumber() for p in FP[ref].Pads() if short(p) == a)
        pb = next(p.GetNumber() for p in FP[ref].Pads() if short(p) == b)
        if upright:
            _upright(FP[ref], x, y, pa, pb, bottom=True)
        else:
            power54._flat_bottom(FP[ref], x, y, pa, pb)
        FP[ref].SetLocked(True)
        obstacles.append(_box(FP[ref]))
        done.append(ref)
    from power42 import _at
    for n, xy in TP_XY.items():
        tp = next(r for r, f in FP.items() if r.startswith("TP") and f.GetSheetname() == sheet
                  and {short(p) for p in f.Pads()} == {n})
        _at(FP[tp], *xy, 0)
        FP[tp].SetLocked(True)
        done.append(tp)
    return done


# ---------------------------------------------------------------- copper
B = pcbnew.B_Cu


def copper(b):
    FP = {f.GetReference(): f for f in b.GetFootprints()}
    P = parts(FP)
    net = lambda name: b.FindNet(name)
    zones, vias, tracks = [], [], []

    def zone(netname, pts, name, clear=0.3, radius=0.5):
        z = pcbnew.ZONE(b)
        z.SetLayer(B)
        z.SetNet(net(netname))
        z.SetAssignedPriority(40)
        z.SetLocalClearance(MM(clear))
        z.SetMinThickness(MM(0.3))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        z.SetZoneName(name)
        z.SetCornerSmoothingType(pcbnew.ZONE_SETTINGS.CORNER_SMOOTHING_FILLET)
        z.SetCornerRadius(MM(radius))
        o = z.Outline()
        o.NewOutline()
        for x, y in pts:
            o.Append(MM(X0 + x), MM(Y0 + y))
        b.Add(z)
        zones.append(z)

    def rect(x0, y0, x1, y1):
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

    pads = []
    for f in b.GetFootprints():
        for p in f.Pads():
            bb = p.GetBoundingBox()
            pads.append((TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0,
                         TOMM(bb.GetBottom()) - Y0, p.GetNetname()))
    for v in b.GetTracks():
        if v.GetClass() == "PCB_VIA":
            pads.append((TOMM(v.GetX()) - X0 - 0.5, TOMM(v.GetY()) - Y0 - 0.5, TOMM(v.GetX()) - X0 + 0.5,
                         TOMM(v.GetY()) - Y0 + 0.5, v.GetNetname()))

    def via(netname, x, y, d=0.9, drill=0.4, force=False):
        rr = d / 2 + 0.35
        if not force and (any(n != netname and l - rr < x < r_ + rr and t - rr < y < bt + rr
                              for l, t, r_, bt, n in pads)
                          or any((x - vx) ** 2 + (y - vy) ** 2 < (d + 0.5) ** 2 for vx, vy, _ in vias)):
            return False
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(pcbnew.VECTOR2I(MM(X0 + x), MM(Y0 + y)))
        v.SetWidth(MM(d))
        v.SetDrill(MM(drill))
        v.SetNet(net(netname))
        b.Add(v)
        vias.append((x, y, netname))
        return True

    def pb(ref, num):
        bb = _pad(FP[ref], num).GetBoundingBox()
        return TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0, TOMM(bb.GetBottom()) - Y0

    u = P["u"]
    sw = _pad(FP[u], "8").GetNetname()
    c1v, c1g = pb(P["cin"][0], "1"), pb(P["cin"][0], "2")
    c2v = pb(P["cin"][1], "1")
    pin2, pin8 = pb(u, "2"), pb(u, "8")
    l_sw, l_out = pb(P["ind"], "1"), pb(P["ind"], "2")
    # BUS42 and the controller's own lines are planned tracks (route(), pcb_route "aux"); the input cap's GND pad: two
    # vias to the L2 plane right below it
    for x, y in C_IN1_GND_VIAS:
        via("GND", x, y, force=True)
    # U11's exposed pad: GND vias (the pad also takes RTN, pin 1)
    ex = pb(u, "9")
    cx, cy = (ex[0] + ex[2]) / 2, (ex[1] + ex[3]) / 2
    for dx in (-0.9, 0.9):
        via("GND", cx + dx, cy, force=True)
    # SW: pin 8 straight down through the bootstrap cap's SW pad onto L2's SW pad, nothing else
    cb = pb(P["cbst"], next(p.GetNumber() for p in FP[P["cbst"]].Pads() if p.GetNetname() == sw))
    x0, x1 = min(pin8[0], cb[0]) - 0.1, max(pin8[2], cb[2]) + 0.1
    zone(sw, rect(x0, pin8[1] + 0.9, x1, l_sw[1] + 0.4), "aux SW")
    # L3 under the switch node: a GND shield over U11's pad vias (they tie it to GND), so the +3V3 fill does not
    # sit 0.2 mm under the switch node
    ex9 = pb(u, "9")
    z = pcbnew.ZONE(b)
    z.SetLayer(pcbnew.In2_Cu)
    z.SetNet(net("GND"))
    z.SetAssignedPriority(41)
    z.SetLocalClearance(MM(0.3))
    z.SetMinThickness(MM(0.3))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetZoneName("aux SW shield (L3)")
    o = z.Outline()
    o.NewOutline()
    for x, y in rect(min(pin8[0], l_sw[0]) - 1.0, ex9[1] - 0.3, l_sw[2] + 1.0, l_sw[3] + 1.0):
        o.Append(MM(X0 + x), MM(Y0 + y))
    b.Add(z)
    zones.append(z)
    # 5.3 V: L2's output pad down onto the row of output caps and straight right to D16's anode
    co = [pb(r, "1") for r in P["cout"]]
    da = pb(P["d_aux"], "2")
    zone("+5V3_AUX", [(min(l_out[0], co[0][0]) - 0.3, l_out[1] - 0.3), (da[2] + 0.3, l_out[1] - 0.3),
                      (da[2] + 0.3, l_out[3] + 0.3), (max(c[2] for c in co) + 0.3, l_out[3] + 0.3),
                      (max(c[2] for c in co) + 0.3, max(c[3] for c in co) + 0.3),
                      (min(l_out[0], co[0][0]) - 0.3, max(c[3] for c in co) + 0.3)], "aux 5V3")
    # output caps' GND pads: a strip under them, stitched to the L2 plane
    cg = [pb(r, "2") for r in P["cout"]]
    g0 = (126.1, min(c[1] for c in cg) - 0.3, max(c[2] for c in cg) + 0.3, max(c[3] for c in cg) + 1.6)
    zone("GND", rect(*g0), "aux GND out")
    x = g0[0] + 1.0
    while x < g0[2] - 0.5:
        via("GND", x, g0[3] - 0.65)
        x += 2.2
    # +5V_LOGIC: a column at the right edge joining both cathodes, the 5 V cap and the LDO's input pin
    k1 = pb(P["d_aux"], "1")
    lin = pb(P["ldo"], "3")
    c5 = pb(P["c5"][0], "1")
    xl5 = min(k1[0], lin[0]) - 0.3
    zone("+5V_LOGIC", rect(xl5, c5[1] - 0.3, 149.4, lin[3] + 0.3), "aux 5V logic")
    # +3V3: the LDO's tab and output pin, the 3.3 V cap (routing takes it to the ESP32 and the logic)
    tab = max((p for p in FP[P["ldo"]].Pads() if p.GetNumber() == "2"), key=lambda p: p.GetSize().x * p.GetSize().y)
    tb = tab.GetBoundingBox()
    tb = (TOMM(tb.GetLeft()) - X0, TOMM(tb.GetTop()) - Y0, TOMM(tb.GetRight()) - X0, TOMM(tb.GetBottom()) - Y0)
    c33 = pb(P["c33"][0], "1")
    zone("+3V3", rect(min(tb[0], c33[0]) - 0.3, min(tb[1], c33[1]) - 0.3, tb[2] + 0.3, tb[3] + 0.3), "aux 3V3")
    # LDO GND pin and the two small caps' GND pads: a via each
    for ref, num in ((P["ldo"], "1"), (P["c5"][0], "2"), (P["c33"][0], "2"), (P["cvcc"], "2")):
        g = pb(ref, num)
        via("GND", (g[0] + g[2]) / 2, g[3] + 0.9) or via("GND", (g[0] + g[2]) / 2, g[1] - 0.9) or \
            via("GND", g[0] - 0.9, (g[1] + g[3]) / 2)
    # BUS42 feed (about 0.1 A): from the 42 V bar's L3 copy up the free strip right of CHG_N, along y 21.7
    # above the rail blocks' L3 copies (they start at 22.5), down past the 5 V block and through a via into
    # the input pour. 0.6 mm: the HV classes ask 0.5 mm minimum.
    # ... then west past the 5 V LED's vias to a via above the VIN cap, with a branch to the RON / UVLO bus's via
    fc, fb = FEED_VIAS["cap"], FEED_VIAS["bus"]
    feed = [(49.2, 30.6), (49.2, BUS42_Y), (131.0, BUS42_Y), (131.0, 25.5), (fc[0], 25.5 + 131.0 - fc[0]), fc]
    branch = [(131.0, 25.5), (fb[0], 25.5 + fb[0] - 131.0), fb]
    for path in (feed, branch):
        for (x1, y1), (x2, y2) in zip(path, path[1:]):
            t = pcbnew.PCB_TRACK(b)
            t.SetStart(pcbnew.VECTOR2I(MM(X0 + x1), MM(Y0 + y1)))
            t.SetEnd(pcbnew.VECTOR2I(MM(X0 + x2), MM(Y0 + y2)))
            t.SetWidth(MM(0.6))
            t.SetLayer(pcbnew.In2_Cu)
            t.SetNet(net("BUS42"))
            b.Add(t)
            tracks.append(t)
    for xy in (fc, fb):
        via("BUS42", *xy, force=True)
    return zones, vias, tracks


# ---------------------------------------------------------------- the controller's lines (planned, pcb_route "aux")
W_SIG, W_DRV, W_PWR = 0.25, 0.3, 0.5


def route(R, FP, log=print):
    """VIN / RTN, BST, VCC, FB, RON, UVLO, the BUS42 bus, the ripple network and its VOUT / SW feeds, the network's GND
    vias: all planned and checked exactly."""
    import far42
    import route_plan
    from route import B, F, IN2
    P = parts(FP)
    u = {q["num"]: q for q in R.geo.pads if q["ref"] == P["u"]}
    sw = u["8"]["net"]
    pad = lambda ref, net: next(q for q in R.geo.pads if q["ref"] == ref and q["net"] == net)
    by_nets = lambda a, b: next(q["ref"] for q in R.geo.pads if q["net"] == a and q["ref"][0] in "RC"
                                and any(o["net"] == b for o in R.geo.pads if o["ref"] == q["ref"]))
    c = lambda q: q["c"]
    segs, vias = [], []

    def lay(net, pts, w=W_SIG):
        segs.extend(R.track(net, B, pts, w))

    def via(net, xy):
        vias.append(R.via(net, *xy))

    cin1, cin2, cb, cv = P["cin"][0], P["cin"][1], P["cbst"], P["cvcc"]
    # VIN: pin 2 up, west over pin 1 into the VIN cap's BUS42 pad; the feed via down into it
    p2, ci = c(u["2"]), c(pad(cin1, "BUS42"))
    lay("BUS42", [p2, (p2[0], ci[1]), ci], W_PWR)
    fc = FEED_VIAS["cap"]
    lay("BUS42", [fc, (fc[0], pad(cin1, "BUS42")["box"][1] + 0.3)], W_PWR)
    # RTN: pin 1 down-west into the cap's GND pad (the exposed pad's vias take it to the plane too)
    p1, cg = c(u["1"]), c(pad(cin1, "GND"))
    lay("GND", [p1, (p1[0], cg[1] - 0.8), (p1[0] - 0.8, cg[1]), cg], W_PWR)
    # BST: pin 7 down into the bootstrap cap's BST pad; VCC: pin 6 down, east under pin 5 into the VCC cap
    p7, cbp = c(u["7"]), c(pad(cb, u["7"]["net"]))
    lay(u["7"]["net"], [p7, (p7[0], cbp[1] - 0.5 - abs(cbp[0] - p7[0])), (cbp[0], cbp[1] - 0.5), cbp], W_DRV)
    p6, cvp = c(u["6"]), c(pad(cv, u["6"]["net"]))
    yv = cvp[1] - 1.02
    lay(u["6"]["net"], [p6, (p6[0], yv), (cvp[0] - 0.5, yv), (cvp[0], yv + 0.5), cvp], W_DRV)
    # FB: pin 5 east into the divider's VOUT resistor, up into its GND resistor, down and east to the ripple's Cac
    fb = u["5"]["net"]
    r141, r140, c101 = by_nets(fb, "GND"), by_nets(fb, "+5V3_AUX"), by_nets(fb, NET_A)
    p5, f0, f1, fa = c(u["5"]), c(pad(r140, fb)), c(pad(r141, fb)), c(pad(c101, fb))
    lay(fb, [p5, (f0[0], p5[1])])
    lay(fb, [f0, f1])
    yf = cvp[1] - 1.47
    lay(fb, [f0, (f0[0], yf), (fa[0] - (fa[1] - 0.51 - yf), yf), (fa[0], fa[1] - 0.51), fa])
    g141 = c(pad(r141, "GND"))
    gv = (g141[0], g141[1] - 1.48)
    lay("GND", [g141, gv], 0.4)
    via("GND", gv)
    # the ripple node: Cac, Rr and Cr's east pads joined down their east side
    r139, c100 = by_nets(NET_A, sw), by_nets(NET_A, "+5V3_AUX")
    a1, a2, a3 = c(pad(c101, NET_A)), c(pad(r139, NET_A)), c(pad(c100, NET_A))
    xa = a2[0] - 0.1
    lay(NET_A, [a1, (xa, a1[1] + xa - a1[0]), (xa, a3[1] - xa + a3[0]), a3])
    # Rr's SW end: straight in from L2's SW pad, under L2's body
    l_sw = next(q for q in R.geo.pads if q["ref"] == P["ind"] and q["net"] == sw)
    s9 = c(pad(r139, sw))
    lay(sw, [(l_sw["box"][2] - 0.3, s9[1]), s9], W_PWR)
    # VOUT: from the output pour up the bottom side into Cr, an L3 hop from there to the divider's top resistor
    v0, v1 = c(pad(c100, "+5V3_AUX")), c(pad(r140, "+5V3_AUX"))
    top = next(q for q in R.geo.pads if q["ref"] == P["ind"] and q["net"] == "+5V3_AUX")["box"][1]
    lay("+5V3_AUX", [(VOUT_X, top + 0.6), (VOUT_X, v0[1]), v0])
    via("+5V3_AUX", (VOUT_X, v0[1]))
    segs.extend(R.track("+5V3_AUX", IN2, [(VOUT_X, v0[1]), (VOUT_X, v1[1])], W_SIG))
    via("+5V3_AUX", (VOUT_X, v1[1]))
    lay("+5V3_AUX", [(VOUT_X, v1[1]), v1])
    # RON / UVLO: the BUS42 bus with its feed via, R136 / R137 hanging from it, their lines straight down to pins 4 / 3
    r136, r137, r138 = by_nets("BUS42", NET_RON), by_nets("BUS42", NET_UVLO), by_nets("GND", NET_UVLO)
    fbv = FEED_VIAS["bus"]
    t136, t137, t2 = c(pad(r136, "BUS42")), c(pad(r137, "BUS42")), c(pad(cin2, "BUS42"))
    lay("BUS42", [fbv, (t136[0], BUS_Y)], W_PWR)
    for t in (t136, t137, t2):
        lay("BUS42", [t, (t[0], BUS_Y)], W_PWR)
    p4, ron = c(u["4"]), c(pad(r136, NET_RON))
    lay(NET_RON, [ron, (ron[0], ron[1] + 0.8), (p4[0], ron[1] + 0.8 + ron[0] - p4[0]), p4])
    p3, uv, ub = c(u["3"]), c(pad(r137, NET_UVLO)), c(pad(r138, NET_UVLO))
    lay(NET_UVLO, [uv, p3])
    yu = pad(r137, NET_UVLO)["box"][3] + 0.4
    lay(NET_UVLO, [ub, (ub[0], yu), (uv[0], yu)])
    g138 = c(pad(r138, "GND"))
    lay("GND", [g138, (g138[0], g138[1] - 1.475)], 0.4)
    via("GND", (g138[0], g138[1] - 1.475))
    g2 = c(pad(cin2, "GND"))
    lay("GND", [g2, (g2[0] + 1.44, g2[1])], W_PWR)
    via("GND", (g2[0] + 1.44, g2[1]))
    # test pads: SW from a via in the switch-node pour, VCC on a via under its cap, VOUT on the hop via (laid above)
    tp = lambda n: next(q for q in R.geo.pads if q["ref"].startswith("TP") and q["net"] == n)
    t32 = c(tp(sw))
    via(sw, SW_TP_VIA)
    segs.extend(R.track(sw, F, [SW_TP_VIA, (t32[0], SW_TP_VIA[1] + t32[0] - SW_TP_VIA[0]), t32], W_SIG))
    t31 = c(tp(u["6"]["net"]))
    lay(u["6"]["net"], [cvp, t31], W_DRV)
    via(u["6"]["net"], t31)
    bad = route_plan._check_exact(R, segs) + far42._check_vias(R, vias)
    for msg in bad[:25]:
        log(f"   aux controller clearance: {msg}")
    if bad:
        R.failed.append("aux controller")
    log(f"   aux controller: {len(segs)} segments, {len(vias)} vias, {len(bad)} clearance problems")
