"""GND stitching (decided 2026-11-24): every GND fill island and every GND pad gets a way to the L2 plane.

Run by pcb_gnd.py after the +3V3 stage, on the routed board. L2 is a whole GND plane, so a via ties anything to GND.
  islands  a GND fill island (any layer) with no GND via or through-hole GND pin in it gets a via inside it, at
           the first clear spot nearest its middle
  pads     a GND pad (top or bottom side) that sits in no island with a via gets its own via, with a short track
           (up to about 6 mm), at the nearest spot clear of other nets on every layer
Vias may pierce other nets' pours (they open their clearance round it, 0.5 mm for 42 / 54 V) where those are wide
enough not to be cut; never a switch node or the boost's LIN. In the 42 V blocks the eFuses' GND parts sit under
the output pours on the top side and L3, so that is their only way down.
Run twice, with a refill between, so pads in islands that the first pass tied count as tied.
"""
import math

import pcbnew

from power42 import MM, TOMM, X0, Y0

NET = "GND"
VIA_D, VIA_DRILL = 0.9, 0.4
TRACK_W = 0.4
GAP_PAD, GAP_TRACK, VIA_PITCH = 0.25, 0.25, 1.15
EDGE = 1.2
HOLES = [(3.5, 3.5), (146.5, 3.5), (3.5, 146.5), (146.5, 146.5), (75.0, 75.0)]
LAYERS = (pcbnew.F_Cu, pcbnew.In2_Cu, pcbnew.B_Cu)


def _mm(v):
    return pcbnew.VECTOR2I(MM(X0 + v[0]), MM(Y0 + v[1]))


def stitch(b):
    """One pass. Returns (vias added, tracks added, pads left untied)."""
    pads = []
    for f in sorted(b.GetFootprints(), key=lambda f: f.GetReference()):
        for p in f.Pads():
            bb = p.GetBoundingBox()
            pads.append((TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0, TOMM(bb.GetRight()) - X0,
                         TOMM(bb.GetBottom()) - Y0, p.GetNetname(), p.IsOnLayer(pcbnew.F_Cu), p.IsOnLayer(pcbnew.B_Cu)))
    segs, vpos = [], []
    for t in b.GetTracks():
        if t.GetClass() == "PCB_VIA":
            vpos.append((TOMM(t.GetX()) - X0, TOMM(t.GetY()) - Y0, t.GetNetname()))
        else:
            segs.append((TOMM(t.GetStart().x) - X0, TOMM(t.GetStart().y) - Y0, TOMM(t.GetEnd().x) - X0,
                         TOMM(t.GetEnd().y) - Y0, TOMM(t.GetWidth()) / 2, t.GetNetname(), t.GetLayer()))
    pours = []                                     # pours a GND via must never pierce: switch nodes, the boost's LIN
    other = pcbnew.SHAPE_POLY_SET()                # every other net's fill: pierced only where it is wide
    for z in b.Zones():
        if z.GetIsRuleArea() or z.GetNetname() == NET:
            continue
        n = z.GetNetname()
        if n.endswith(("/SW", "/LIN", "-SW)")) or "SW" in z.GetZoneName().split():
            o = z.Outline()
            o.BuildBBoxCaches()
            pours.append(o)
            continue
        for layer in LAYERS:
            if z.IsOnLayer(layer):
                other.Append(z.GetFilledPolysList(layer))
    other_wide = pcbnew.SHAPE_POLY_SET(other)
    other_wide.Deflate(MM(1.6), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.02))

    # (another net's pour flows round a GND via with its clearance, 0.5 mm for 42 / 54 V, but only where it is wide:
    # a via in a narrow neck would cut it)

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
        for l, t, r, bt, n, on_f, on_b in pads:
            if n != NET and box_dist(x, y, l, t, r, bt) < rr + GAP_PAD:
                return False
        for x1, y1, x2, y2, hw, n, layer in segs:
            if n != NET and seg_dist(x, y, x1, y1, x2, y2) < rr + hw + GAP_TRACK:
                return False
        for vx, vy, n in vpos:
            if math.hypot(x - vx, y - vy) < VIA_PITCH:
                return False
        pt = _mm((x, y))
        if other.Collide(pt, MM(rr + 0.55)) and not other_wide.Contains(pt):
            return False
        return not any(o.Collide(pt, MM(rr + 0.55)) for o in pours)

    def path_ok(x1, y1, x2, y2, layer):
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.2))
        for k in range(n + 1):
            x, y = x1 + (x2 - x1) * k / n, y1 + (y2 - y1) * k / n
            for l, t, r, bt, nn, on_f, on_b in pads:
                on = on_f if layer == pcbnew.F_Cu else on_b
                if nn != NET and on and box_dist(x, y, l, t, r, bt) < TRACK_W / 2 + GAP_PAD:
                    return False
            for sx1, sy1, sx2, sy2, hw, nn, sl in segs:
                if nn != NET and sl == layer and seg_dist(x, y, sx1, sy1, sx2, sy2) < TRACK_W / 2 + hw + GAP_TRACK:
                    return False
            for vx, vy, nn in vpos:
                if nn != NET and math.hypot(x - vx, y - vy) < TRACK_W / 2 + VIA_D / 2 + GAP_TRACK:
                    return False
        return True

    net = b.FindNet(NET)
    added_v, added_t = [], []

    def add_via(x, y):
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(_mm((x, y)))
        v.SetWidth(MM(VIA_D))
        v.SetDrill(MM(VIA_DRILL))
        v.SetNet(net)
        b.Add(v)
        added_v.append((x, y))
        vpos.append((x, y, NET))

    def add_track(x1, y1, x2, y2, layer):
        t = pcbnew.PCB_TRACK(b)
        t.SetStart(_mm((x1, y1)))
        t.SetEnd(_mm((x2, y2)))
        t.SetWidth(MM(TRACK_W))
        t.SetLayer(layer)
        t.SetNet(net)
        b.Add(t)
        added_t.append(t)
        segs.append((x1, y1, x2, y2, TRACK_W / 2, NET, layer))

    anchors = [(x, y) for x, y, n in vpos if n == NET]
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() == NET and p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                anchors.append((TOMM(p.GetPosition().x) - X0, TOMM(p.GetPosition().y) - Y0))

    def to_anchor(starts, layer, reach=6.0):
        """A track from one of starts to the nearest anchor it reaches cleanly; True if laid."""
        best = None
        for sx, sy in starts:
            for ax, ay in anchors:
                d = math.hypot(ax - sx, ay - sy)
                if d < 0.3 or d > reach or (best and d >= best[0]):
                    continue
                # stop the track at the anchor's edge (a via's ring), so the check does not trip on it
                ex, ey = ax - (ax - sx) * 0.45 / d, ay - (ay - sy) * 0.45 / d
                if path_ok(sx, sy, ex, ey, layer):
                    best = (d, sx, sy, ax, ay)
        if best:
            _, sx, sy, ax, ay = best
            add_track(sx, sy, ax, ay, layer)
            return True
        return False

    # the GND fill islands on each layer, and whether something ties each to the plane
    ties = [(x, y) for x, y, n in vpos if n == NET]
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() == NET and p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                ties.append((TOMM(p.GetPosition().x) - X0, TOMM(p.GetPosition().y) - Y0))
    islands = []                                   # (layer, outline, tied)
    for z in b.Zones():
        if z.GetNetname() != NET or z.GetIsRuleArea():
            continue
        for layer in LAYERS:
            if not z.IsOnLayer(layer):
                continue
            ps = z.GetFilledPolysList(layer)
            for i in range(ps.OutlineCount()):
                o = ps.Outline(i)
                tied = any(o.PointInside(_mm(t)) for t in ties)
                islands.append((layer, o, tied))
    for layer, o, tied in islands:
        if tied or abs(o.Area()) < MM(1) * MM(1):
            continue
        bb = o.BBox()
        l, t = TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0
        r, bt = TOMM(bb.GetRight()) - X0, TOMM(bb.GetBottom()) - Y0
        cx, cy = (l + r) / 2, (t + bt) / 2
        cands = [(l + i * 0.25, t + j * 0.25) for i in range(int((r - l) / 0.25) + 1)
                 for j in range(int((bt - t) / 0.25) + 1)]
        cands.sort(key=lambda c: math.hypot(c[0] - cx, c[1] - cy))
        for x, y in cands:
            # inside the island with room for the via
            if not o.PointInside(_mm((x, y))):
                continue
            if any(not o.PointInside(_mm((x + dx, y + dy))) for dx, dy in
                   ((0.6, 0), (-0.6, 0), (0, 0.6), (0, -0.6))):
                continue
            if via_ok(x, y):
                add_via(x, y)
                ties.append((x, y))
                break
        else:                                     # no room for a via (under other pours): a track to a GND via
            inside = [c for c in cands if o.PointInside(_mm(c)) and all(o.PointInside(_mm((c[0] + dx, c[1] + dy)))
                      for dx, dy in ((0.35, 0), (-0.35, 0), (0, 0.35), (0, -0.35)))][:400]
            if to_anchor(inside, layer):
                ties.append(inside[0] if inside else (cx, cy))
    # GND pads in no tied island: a via each. (A pad in a pour sits in its thermal-relief gap, so the island reaches
    # to within the gap of the pad's edge, not over its centre.)
    tied_isl = [(layer, o) for layer, o, tied in islands if any(o.PointInside(_mm(t)) for t in ties)]
    left = []
    for f in sorted(b.GetFootprints(), key=lambda f: f.GetReference()):
        for p in f.Pads():
            if p.GetNetname() != NET or p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                continue
            layer = pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
            px, py = TOMM(p.GetPosition().x) - X0, TOMM(p.GetPosition().y) - Y0
            reach = MM(max(TOMM(p.GetSize().x), TOMM(p.GetSize().y)) / 2 + 0.6)
            if any(lay == layer and (o.PointInside(_mm((px, py))) or o.Collide(_mm((px, py)), reach))
                   for lay, o in tied_isl):
                continue
            if any(math.hypot(px - x, py - y) < 0.05 for x, y in ties):
                continue
            bb = p.GetBoundingBox()                      # a GND track already leads off it (to a planned via)
            l, t = TOMM(bb.GetLeft()) - X0, TOMM(bb.GetTop()) - Y0
            r, bt = TOMM(bb.GetRight()) - X0, TOMM(bb.GetBottom()) - Y0
            if any(n == NET and sl == layer and any(l - 0.05 <= ex <= r + 0.05 and t - 0.05 <= ey <= bt + 0.05
                                                    for ex, ey in ((x1, y1), (x2, y2)))
                   for x1, y1, x2, y2, hw, n, sl in segs):
                continue
            half = max(TOMM(p.GetSize().x), TOMM(p.GetSize().y)) / 2
            done = False
            for dist in [half + 0.9 + 0.45 * i for i in range(12)]:
                for k in range(24):
                    a = 2 * math.pi * k / 24
                    x, y = px + dist * math.cos(a), py + dist * math.sin(a)
                    if via_ok(x, y) and path_ok(px, py, x, y, layer):
                        add_via(x, y)
                        add_track(px, py, x, y, layer)
                        ties.append((px, py))
                        done = True
                        break
                if done:
                    break
            if not done and to_anchor([(px, py)], layer):
                done = True
                ties.append((px, py))
            if not done:
                left.append(f"{f.GetReference()}.{p.GetNumber()}")
    return added_v, added_t, left
