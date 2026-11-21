"""Logic supply distribution: a +3V3 fill on L3 and a via fan-out from every +3V3 pin to it.

Run by pcb_power42.py after the power blocks (their copper, vias and tracks must exist first).
The fill covers the whole of L3 at low priority, so it takes only what the power blocks' L3 copies leave free
(under the output columns, the right-hand column, the corners); signal tracks routed on L3 later cut through it.
Every +3V3 pin gets one via to it with a short track (up to about 6 mm), placed at the nearest spot that clears
the other nets' pads, tracks and vias on every layer and lands where L3 is free for the fill. Pins over a power
block's L3 copy with no fill within reach are left to routing (pcb_power42.py lists them). The AP7361's 3.3 V tab, the
source, gets a few vias of its own. Fill areas that no via reaches are removed as islands.
"""
import math

import pcbnew

from power42 import MM, TOMM, X0, Y0

NET = "+3V3"
VIA_D, VIA_DRILL = 0.9, 0.4
TRACK_W = 0.3
GAP_PAD, GAP_TRACK, GAP_VIA = 0.25, 0.25, 0.35
EDGE = 1.2                                   # via centre to board edge
HOLES = [(3.5, 3.5), (146.5, 3.5), (3.5, 146.5), (146.5, 146.5)]


def copper(b):
    net = b.FindNet(NET)
    zones, vias, tracks = [], [], []

    # every pad and track on the board, by layer, for clearance checks (mm, board-relative)
    pads = []
    for f in sorted(b.GetFootprints(), key=lambda f: f.GetReference()):   # not import order
        for p in f.Pads():
            bb = p.GetBoundingBox()
            pads.append((TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0,
                         TOMM(bb.GetBottom()) - Y0, p.GetNetname(), p.IsOnLayer(pcbnew.F_Cu), p.IsOnLayer(pcbnew.B_Cu),
                         p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH)))
    segs, vpos = [], []
    for t in b.GetTracks():
        if t.GetClass() == "PCB_VIA":
            vpos.append((TOMM(t.GetX()) - X0, TOMM(t.GetY()) - Y0, TOMM(t.GetWidth()) / 2, t.GetNetname()))
        else:
            segs.append((TOMM(t.GetStart().x) - X0, TOMM(t.GetStart().y) - Y0, TOMM(t.GetEnd().x) - X0,
                         TOMM(t.GetEnd().y) - Y0, TOMM(t.GetWidth()) / 2, t.GetNetname(), t.GetLayer()))
    # L3 copper of other nets (the power blocks' copies): the fill cannot reach there. Power pours on any layer:
    # a 3.3 V via must not pierce them. Zones made in this run need their outline caches built before Collide.
    # The 24 / 12 / 5 V pours may take a via (a 1.5 mm hole in a 6.5 mm strip costs nothing); 42 / 54 V copper,
    # switch nodes and the rest may not.
    low = lambda n: (n in ("OUT24", "OUT12", "OUT5", "VIN24", "VIN12", "VIN5") or n.startswith("/Rail inputs")
                     or any(n.startswith(f"/{v} V rail eFuse") for v in ("24", "12", "5")))
    l3_other, pours = [], []
    for z in b.Zones():
        if z.GetNetname() in (NET, "GND") or z.GetIsRuleArea():
            continue
        o = z.Outline()
        o.BuildBBoxCaches()
        if z.IsOnLayer(pcbnew.In2_Cu):
            l3_other.append(o)
        elif not low(z.GetNetname()):
            pours.append(o)

    def seg_dist(px, py, x1, y1, x2, y2):
        dx, dy = x2 - x1, y2 - y1
        L = dx * dx + dy * dy
        u = 0.0 if L == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L))
        return math.hypot(px - (x1 + u * dx), py - (y1 + u * dy))

    def box_dist(px, py, l, t, r, bt):
        return math.hypot(max(l - px, 0, px - r), max(t - py, 0, py - bt))

    def via_ok(x, y):
        rr = VIA_D / 2
        if not (EDGE <= x <= 150 - EDGE and EDGE <= y <= 150 - EDGE):
            return False
        if any(math.hypot(x - hx, y - hy) < 4.5 for hx, hy in HOLES):
            return False
        for l, t, r, bt, n, on_f, on_b, th in pads:
            if n != NET and box_dist(x, y, l, t, r, bt) < rr + GAP_PAD:
                return False
        for x1, y1, x2, y2, hw, n, layer in segs:
            if n != NET and seg_dist(x, y, x1, y1, x2, y2) < rr + hw + GAP_TRACK:
                return False
        for vx, vy, vr, n in vpos:
            if math.hypot(x - vx, y - vy) < rr + vr + (GAP_VIA if n != NET else 0.2):
                return False
        pt = pcbnew.VECTOR2I(MM(X0 + x), MM(Y0 + y))
        return not any(o.Collide(pt, MM(rr + 0.6)) for o in pours + l3_other)

    def path_ok(x1, y1, x2, y2, layer, own):
        """Track from a pad to its via: clear of other nets' pads and tracks on that layer."""
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.2))
        for k in range(n + 1):
            x, y = x1 + (x2 - x1) * k / n, y1 + (y2 - y1) * k / n
            for l, t, r, bt, nn, on_f, on_b, th in pads:
                on = on_f if layer == pcbnew.F_Cu else on_b
                if nn != NET and on and box_dist(x, y, l, t, r, bt) < TRACK_W / 2 + GAP_PAD:
                    return False
            for sx1, sy1, sx2, sy2, hw, nn, sl in segs:
                if nn != NET and sl == layer and seg_dist(x, y, sx1, sy1, sx2, sy2) < TRACK_W / 2 + hw + GAP_TRACK:
                    return False
            for vx, vy, vr, nn in vpos:                   # other nets' vias go through every layer
                if nn != NET and math.hypot(x - vx, y - vy) < TRACK_W / 2 + vr + GAP_TRACK:
                    return False
        return True

    def add_via(x, y):
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(pcbnew.VECTOR2I(MM(X0 + x), MM(Y0 + y)))
        v.SetWidth(MM(VIA_D))
        v.SetDrill(MM(VIA_DRILL))
        v.SetNet(net)
        b.Add(v)
        vias.append((x, y))
        vpos.append((x, y, VIA_D / 2, NET))

    def add_track(x1, y1, x2, y2, layer):
        t = pcbnew.PCB_TRACK(b)
        t.SetStart(pcbnew.VECTOR2I(MM(X0 + x1), MM(Y0 + y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(X0 + x2), MM(Y0 + y2)))
        t.SetWidth(MM(TRACK_W))
        t.SetLayer(layer)
        t.SetNet(net)
        b.Add(t)
        tracks.append(t)
        segs.append((x1, y1, x2, y2, TRACK_W / 2, NET, layer))

    # the LDO's tab: a row of vias across it (it is the source)
    missed = []
    for f in sorted(b.GetFootprints(), key=lambda f: f.GetReference()):   # not import order
        if "AP7361" in f.GetValue():
            tab = max((p for p in f.Pads() if p.GetNetname() == NET), key=lambda p: p.GetSize().x * p.GetSize().y)
            bb = tab.GetBoundingBox()
            l, t, r, bt = (TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0,
                           TOMM(bb.GetBottom()) - Y0)
            for k in range(4):
                x = l + (r - l) * (k + 0.5) / 4
                if via_ok(x, (t + bt) / 2):
                    add_via(x, (t + bt) / 2)
    # every other +3V3 pin: nearest legal via spot, rings of candidates around the pad
    for f in sorted(b.GetFootprints(), key=lambda f: f.GetReference()):   # not import order
        for p in f.Pads():
            if p.GetNetname() != NET or "AP7361" in f.GetValue():
                continue
            px, py = TOMM(p.GetPosition().x) - X0, TOMM(p.GetPosition().y) - Y0
            layer = pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) and not p.IsOnLayer(pcbnew.B_Cu) else pcbnew.B_Cu
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                continue                                  # through-hole pins reach L3 by themselves
            half = max(TOMM(p.GetSize().x), TOMM(p.GetSize().y)) / 2
            done = False
            for dist in [half + 0.9 + 0.45 * i for i in range(12)]:     # up to about 6 mm out
                for k in range(24):
                    a = 2 * math.pi * k / 24
                    x, y = px + dist * math.cos(a), py + dist * math.sin(a)
                    if via_ok(x, y) and path_ok(px, py, x, y, layer, f.GetReference()):
                        add_via(x, y)
                        add_track(px, py, x, y, layer)
                        done = True
                        break
                if done:
                    break
            if not done:
                missed.append(f"{f.GetReference()}.{p.GetNumber()}")

    # the fill: all of L3, under every power copy, islands (no via) removed
    z = pcbnew.ZONE(b)
    z.SetLayer(pcbnew.In2_Cu)
    z.SetNet(net)
    z.SetAssignedPriority(10)
    z.SetLocalClearance(MM(0.3))
    z.SetMinThickness(MM(0.3))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetZoneName("+3V3 fill (L3)")
    o = z.Outline()
    o.NewOutline()
    for x, y in ((0.5, 0.5), (149.5, 0.5), (149.5, 149.5), (0.5, 149.5)):
        o.Append(MM(X0 + x), MM(Y0 + y))
    b.Add(z)
    zones.append(z)
    return zones, vias, tracks, missed
