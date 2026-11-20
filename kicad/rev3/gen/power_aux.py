"""Hand-designed layout of the always-on aux supply (LM5017 buck 42 V -> 5.3 V, OR diodes, AP7361 -> 3.3 V).

Used by pcb_place.py (placement, before the packer) and pcb_power42.py (copper, after the optimizer).
Board mm from the top-left corner. Everything is on the bottom, beside the ESP32 it powers, under the space the
54 V eFuse left (x 126-149.4, y 31-67).

Stack, top to bottom:
  input ceramics (one straight above U11's VIN / RTN pins: the buck's hot loop is that cap and the IC)
  -> U11 (LM5017: VIN / RTN pins on top, SW pin under them) -> L2 standing, its SW pad right under U11's SW pin
  -> output ceramics in a row under L2's 5.3 V pad
  right column: OR diodes lying flat, D16 (aux 5.3 V) level with L2's 5.3 V pad and D17 (USB VBUS) above it,
  cathodes on the right joined on +5V_LOGIC with the 5 V cap; the AP7361 under them, input pin up, tab down,
  its 3.3 V cap beside the tab.
BUS42 arrives as a thin track (about 0.1 A) from the 42 V bar; the logic rails leave by routing.
"""
import pcbnew

from power42 import MM, TOMM, X0, Y0, _at, _pad, _pxy

U11_XY = (132.65, 39.6)
GAP = 0.6
KEEP_BOTTOM = [(126.1, 31.9, 139.9, 67.0), (139.9, 44.8, 149.4, 67.0)]   # pocket right of U11 for its resistors
KEEP_TOP = [(126.6, 31.6, 135.4, 43.4)]           # vias of U11's pad and the input cap's GND: no top parts on them


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
    # input cap 1 straight above pins 1-2: GND pad over RTN, BUS42 pad over VIN
    c1 = FP[P["cin"][0]]
    _orient(c1, lambda f: _pxy(f, "2")[0] < _pxy(f, "1")[0] and abs(_pxy(f, "1")[1] - _pxy(f, "2")[1]) < 0.05)
    _shift(c1, (p1[0] + p2[0]) / 2 - (_pxy(c1, "1")[0] + _pxy(c1, "2")[0]) / 2, 0.0)
    _put(c1, bottom=ub[1] - GAP)
    # input cap 2 standing right of U11, BUS42 pad up
    c2 = FP[P["cin"][1]]
    _orient(c2, lambda f: _pxy(f, "1")[1] < _pxy(f, "2")[1] - 1)
    _put(c2, left=ub[2] + GAP, top=ub[1] - 0.1)
    # bootstrap and VCC caps standing left of U11, near pins 7 / 6
    for ref, top in ((P["cbst"], ub[1]), (P["cvcc"], None)):
        f = FP[ref]
        _orient(f, lambda f_: abs(_pxy(f_, "1")[0] - _pxy(f_, "2")[0]) < 0.05)
        if top is not None:
            _put(f, right=ub[0] - GAP, top=top)
            prev = _box(f)
        else:
            _put(f, right=ub[0] - GAP, top=prev[3] + GAP)
    # L2 standing, SW pad (1) up, right under U11's SW pin
    ind = FP[P["ind"]]
    _orient(ind, lambda f: _pxy(f, "1")[1] < _pxy(f, "2")[1] - 1)
    sx = _pxy(u, "8")[0]
    _shift(ind, sx + 2.0 - _pxy(ind, "1")[0], 0.0)
    _put(ind, top=ub[3] + GAP)
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
    # OR diodes lying flat at the right edge, anode left: D16 level with L2's 5.3 V pad, D17 above it
    lout_y = _pxy(ind, "2")[1]
    da = FP[P["d_aux"]]
    _orient(da, lambda f: _pxy(f, "2")[0] < _pxy(f, "1")[0] - 1)
    _put(da, right=149.4)
    _shift(da, 0.0, lout_y - _pxy(da, "2")[1])
    du = FP[P["d_usb"]]
    _orient(du, lambda f: _pxy(f, "2")[0] < _pxy(f, "1")[0] - 1)
    _put(du, right=149.4, bottom=_box(da)[1] - GAP)
    # 5 V cap standing above D17, at the cathode end
    c5 = FP[P["c5"][0]]
    _orient(c5, lambda f: abs(_pxy(f, "1")[0] - _pxy(f, "2")[0]) < 0.05 and _pxy(f, "1")[1] > _pxy(f, "2")[1])
    _put(c5, right=149.4, bottom=_box(du)[1] - GAP)
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
    # BUS42: over cap 1's input pad and across to cap 2's, kept above U11's pin row except a tab onto VIN,
    # so UVLO and RON stay free to escape
    zone("BUS42", [(c1v[0] - 0.3, c1v[1] - 0.3), (c2v[2] + 0.3, c1v[1] - 0.3), (c2v[2] + 0.3, c2v[3] + 0.3),
                   (c2v[0] - 0.3, c2v[3] + 0.3), (c2v[0] - 0.3, pin2[1] - 0.4), (pin2[2] + 0.25, pin2[1] - 0.4),
                   (pin2[2] + 0.25, pin2[1] + 0.6), (pin2[0] - 0.25, pin2[1] + 0.6), (pin2[0] - 0.25, pin2[1] - 0.4),
                   (c1v[0] - 0.3, pin2[1] - 0.4)], "aux BUS42")
    # cap 1's GND pad: two vias to the L2 plane right beside it, and RTN below joins through the pad vias
    for dy in (-0.7, 0.7):
        via("GND", c1g[0] - 0.9, (c1g[1] + c1g[3]) / 2 + dy, force=True)
    # U11's exposed pad: GND vias (the pad also takes RTN, pin 1)
    ex = pb(u, "9")
    cx, cy = (ex[0] + ex[2]) / 2, (ex[1] + ex[3]) / 2
    for dx in (-0.9, 0.9):
        via("GND", cx + dx, cy, force=True)
    # SW: pin 8 straight down onto L2's SW pad, nothing else
    zone(sw, [(pin8[0] - 0.3, pin8[1] + 0.4), (pin8[2] + 0.25, pin8[1] + 0.4), (pin8[2] + 0.25, l_sw[1] - 0.6),
              (l_sw[2] + 0.3, l_sw[1] - 0.6), (l_sw[2] + 0.3, l_sw[3] + 0.3), (l_sw[0] - 0.3, l_sw[3] + 0.3),
              (l_sw[0] - 0.3, l_sw[1] - 0.6), (pin8[0] - 0.3, l_sw[1] - 0.6)], "aux SW")
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
    k1, k2 = pb(P["d_aux"], "1"), pb(P["d_usb"], "1")
    lin = pb(P["ldo"], "3")
    c5 = pb(P["c5"][0], "1")
    xl5 = min(k1[0], k2[0], lin[0]) - 0.3
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
    return zones, vias, tracks
