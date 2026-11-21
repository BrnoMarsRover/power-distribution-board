"""Signal routing by intent: a grid router the per-block routing plans (route_plan.py) call net by net.

Board mm from the top-left corner, like the placement modules. Run inside KiCad's Python, after the power copper
(its pours are obstacles) and before the +3V3 fan-out (it routes around the signal tracks).

How a net is routed:
  - a grid of PITCH over a window around its terminals, on the routing layers (L4 bottom first, L3 for crossings;
    L1 carries the power copper and L2 is the solid GND plane, so neither takes signal tracks);
  - a cell is free when a track of this width there keeps CLEAR (+ SLACK for the grid) from every other net's
    pad, track, via and power pour on that layer. GND and +3V3 fills are not obstacles: they flow around tracks;
  - a via is legal where it keeps clear of other nets' copper on every layer and of pads, and does not pierce
    42 / 54 V copper (its own net's pours are fine; low-voltage pours of other nets clear around it);
  - A* over (layer, cell, heading): length, a price per turn (45 deg cheap, 90 deg dear, sharper never), a price
    per via, L3 dearer than L4, optional per-cell cost fields (keep sense lines off gate drive, pull a pair
    together);
  - several terminals: from the first, each next one joins the tree built so far (nearest first);
  - fine-pitch IC pins leave through a straight escape stub (pad width) along the pad's long axis, outward;
  - a net can be narrow near its IC (neck) and wider beyond;
  - supply pads (GND, an output or input rail) get a via into their own plane or pour next to the pad, or a
    track to the nearest spot where one fits.
Grid paths are cut into straight runs (0 / 45 / 90 degrees) and added as KiCad tracks and vias.
"""
import heapq
import math
import os
import time

import numpy as np
import pcbnew

from power42 import MM, TOMM, X0, Y0

F, IN1, IN2, B = pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu
COPPER = (F, IN1, IN2, B)
LAYER_NAME = {F: "F", IN1: "In1", IN2: "In2", B: "B"}
PITCH = 0.1
CLEAR = 0.2                     # class clearance (board_rules.py)
SLACK = 0.02                    # route points are exact grid nodes; this covers diagonal corner cuts
VIA_D, VIA_DRILL = 0.9, 0.4     # smallest via for 2 oz outer copper (board_rules.py)
EDGE = 0.45                     # copper to board edge (rule 0.3)
FILLS = {"GND", "+3V3"}         # pours that flow around other nets' tracks
HV = ("VBAT", "BUS42", "OUT42", "OUT54", "/54 V", "/42 V", "/Brake", "/Battery")
LAYER_COST = {B: 1.0, IN2: 1.6}
VIA_COST = 1.6                  # mm of track a via is worth
TURN = {0: 0.0, 1: 0.12, 2: 0.6}   # by heading change in 45 deg steps; 3 and 4 not allowed
DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
NEAR = 1.0                      # nets with copper this close to a conflict are re-routed with it (PathFinder)


def _xy(v):
    return TOMM(v.x) - X0, TOMM(v.y) - Y0


def _hv(net):
    return net.startswith(HV)


# ---------------------------------------------------------------- geometry snapshot
class Geometry:
    """Everything copper on the board in mm, kept up to date as routes are added."""

    def __init__(self, b):
        self.b = b
        self.pads = []          # dict per pad, footprints by reference (the import order varies between builds)
        for f in sorted(b.GetFootprints(), key=lambda f: f.GetReference()):
            fx, fy = _xy(f.GetPosition())
            for p in f.Pads():
                bb = p.GetBoundingBox()
                l, t = _xy(pcbnew.VECTOR2I(bb.GetLeft(), bb.GetTop()))
                r, bt = _xy(pcbnew.VECTOR2I(bb.GetRight(), bb.GetBottom()))
                npth = p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH
                layers = frozenset(COPPER) if (npth or p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH) else \
                    frozenset(l_ for l_ in COPPER if p.IsOnLayer(l_))
                self.pads.append(dict(net="" if npth else p.GetNetname(), layers=layers, box=(l, t, r, bt),
                                      c=_xy(p.GetPosition()), ref=f.GetReference(), num=p.GetNumber(),
                                      fc=(fx, fy), npth=npth, pad=p))
        self.segs = []          # (net, layer, x1, y1, x2, y2, half width)
        self.vias = []          # (net, x, y, radius)
        for t in b.GetTracks():
            if t.GetClass() == "PCB_VIA":
                x, y = _xy(t.GetPosition())
                self.vias.append((t.GetNetname(), x, y, TOMM(t.GetWidth(F)) / 2))
            else:
                (x1, y1), (x2, y2) = _xy(t.GetStart()), _xy(t.GetEnd())
                self.segs.append((t.GetNetname(), t.GetLayer(), x1, y1, x2, y2, TOMM(t.GetWidth()) / 2))
        self.zones = []         # (net, layers, priority, outline points, clearance)
        for z in b.Zones():
            o = z.Outline()
            if o.OutlineCount() == 0 or z.GetIsRuleArea():
                continue
            ch = o.Outline(0)
            pts = np.array([_xy(ch.CPoint(k)) for k in range(ch.PointCount())])
            layers = frozenset(l_ for l_ in COPPER if z.IsOnLayer(l_))
            self.zones.append((z.GetNetname(), layers, z.GetAssignedPriority(), pts,
                               TOMM(z.GetLocalClearance() or 0)))
        self.fills = []         # (net, layer, polygon points): the filled copper of every filled zone
        for z in b.Zones():
            if not z.IsFilled() or z.GetIsRuleArea():
                continue
            for l_ in COPPER:
                if not z.IsOnLayer(l_):
                    continue
                fp = z.GetFilledPolysList(l_)
                for i in range(fp.OutlineCount()):
                    ch = fp.Outline(i)
                    self.fills.append((z.GetNetname(), l_,
                                       np.array([_xy(ch.CPoint(k)) for k in range(ch.PointCount())])))

    def pads_of(self, net):
        return [p for p in self.pads if p["net"] == net]


# ---------------------------------------------------------------- rasterising (vectorised over a window)
def _dist_box(X, Y, box):
    l, t, r, b = box
    dx = np.maximum(np.maximum(l - X, 0), X - r)
    dy = np.maximum(np.maximum(t - Y, 0), Y - b)
    return np.hypot(dx, dy)


def _dist_seg(X, Y, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L = dx * dx + dy * dy
    u = np.zeros_like(X) if L == 0 else np.clip(((X - x1) * dx + (Y - y1) * dy) / L, 0.0, 1.0)
    return np.hypot(X - (x1 + u * dx), Y - (y1 + u * dy))


def _inside_poly(X, Y, pts):
    inside = np.zeros(X.shape, bool)
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        cond = (y1 > Y) != (y2 > Y)
        with np.errstate(divide="ignore", invalid="ignore"):
            xc = x1 + (Y - y1) * (x2 - x1) / (y2 - y1)
        inside ^= cond & (X < xc)
    return inside


def _seg_seg(a, b):
    """Distance between segments a = (x1, y1, x2, y2) and b (0 when they cross)."""
    def ps(px, py, x1, y1, x2, y2):
        dx, dy = x2 - x1, y2 - y1
        L = dx * dx + dy * dy
        u = 0.0 if L == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L))
        return math.hypot(px - (x1 + u * dx), py - (y1 + u * dy))
    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    p1, p2, p3, p4 = (a[0], a[1]), (a[2], a[3]), (b[0], b[1]), (b[2], b[3])
    d1, d2, d3, d4 = cross(p3, p4, p1), cross(p3, p4, p2), cross(p1, p2, p3), cross(p1, p2, p4)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 and d2 and d3 and d4:
        return 0.0
    return min(ps(*p1, *b), ps(*p2, *b), ps(*p3, *a), ps(*p4, *a))


def _pt_seg(px, py, s):
    x1, y1, x2, y2 = s
    dx, dy = x2 - x1, y2 - y1
    L = dx * dx + dy * dy
    u = 0.0 if L == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L))
    return math.hypot(px - (x1 + u * dx), py - (y1 + u * dy))


def own_copper(geo, win, net, zone_layers):
    """Cells of win where a via of net lands well inside its own filled copper on zone_layers (at least a via
    radius from the fill's edge, so the via connects once the zones are refilled). A zone that was never filled
    counts by its outline, eroded the same way."""
    own = np.zeros((win.ny, win.nx), bool)
    l0, t0 = win.x0, win.y0
    l1, t1 = win.x0 + (win.nx - 1) * PITCH, win.y0 + (win.ny - 1) * PITCH
    filled = {(n, l_) for n, l_, _ in geo.fills}
    polys = [pts for n, l_, pts in geo.fills if n == net and l_ in zone_layers]
    polys += [pts for n, zl, prio, pts, zc in geo.zones if n == net and zl & set(zone_layers)
              and not any((n, l_) in filled for l_ in zl & set(zone_layers))]
    for pts in polys:
        if pts[:, 0].max() < l0 or pts[:, 0].min() > l1 or pts[:, 1].max() < t0 or pts[:, 1].min() > t1:
            continue
        own |= _inside_poly(win.X, win.Y, pts)
    k = int(math.ceil(VIA_D / 2 / PITCH))
    pad = np.pad(own, k)
    out = own.copy()
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            if math.hypot(dx, dy) * PITCH <= VIA_D / 2:
                out &= pad[k + dy:k + dy + win.ny, k + dx:k + dx + win.nx]
    return out


class Window:
    """A routing grid over a board window, for one net and one track width."""

    def __init__(self, geo, net, box, w, layers=(B, IN2), origin=None, neck=None, soft=frozenset()):
        """origin: a point to put exactly on a grid node (a shared block grid aligns every window to it).
        soft: ids of copper that is not an obstacle here (routes being negotiated: they cost, they do not block)."""
        self.geo, self.net, self.w, self.layers, self.soft = geo, net, w, list(layers), soft
        x0, y0, x1, y1 = box
        if origin:                                       # put this point exactly on a grid node
            ox, oy = origin
            x0 = ox - math.ceil((ox - x0) / PITCH) * PITCH
            y0 = oy - math.ceil((oy - y0) / PITCH) * PITCH
        self.x0, self.y0 = x0, y0
        self.nx = int((x1 - x0) / PITCH) + 1
        self.ny = int((y1 - y0) / PITCH) + 1
        xs = x0 + np.arange(self.nx) * PITCH
        ys = y0 + np.arange(self.ny) * PITCH
        self.X, self.Y = np.meshgrid(xs, ys)
        self.box = (x0, y0, x0 + (self.nx - 1) * PITCH, y0 + (self.ny - 1) * PITCH)
        self.block = {l_: self._blocked(l_, w) for l_ in self.layers}
        if neck:                                         # narrower track allowed inside the neck box
            nbox, wn = neck
            innk = (self.X >= nbox[0]) & (self.X <= nbox[2]) & (self.Y >= nbox[1]) & (self.Y <= nbox[3])
            for l_ in self.layers:
                self.block[l_] &= ~(innk & ~self._blocked(l_, wn))
            self.neck = (nbox, wn)
        else:
            self.neck = None
        self.via_ok = self._via_ok()
        self.base = {l_: np.full((self.ny, self.nx), LAYER_COST.get(l_, 2.0)) for l_ in self.layers}
        self.reset_cost()

    def reset_cost(self):
        self.cost = {l_: c.copy() for l_, c in self.base.items()}
        self.via_cost = np.ones((self.ny, self.nx))

    def cell(self, x, y):
        return int(round((y - self.y0) / PITCH)), int(round((x - self.x0) / PITCH))

    def xy(self, iy, ix):
        return self.x0 + ix * PITCH, self.y0 + iy * PITCH

    def _near(self, box, r):
        """Index slice of the window cells within r of a box, or None."""
        l, t, rr, b = box
        ix0 = max(0, int((l - r - self.x0) / PITCH) - 1)
        ix1 = min(self.nx, int((rr + r - self.x0) / PITCH) + 2)
        iy0 = max(0, int((t - r - self.y0) / PITCH) - 1)
        iy1 = min(self.ny, int((b + r - self.y0) / PITCH) + 2)
        if ix0 >= ix1 or iy0 >= iy1:
            return None
        return slice(iy0, iy1), slice(ix0, ix1)

    def _blocked(self, layer, w):
        m = np.zeros((self.ny, self.nx), bool)
        g, net = self.geo, self.net
        r0 = w / 2 + CLEAR + SLACK
        for p in g.pads:
            if p["net"] == net and net:
                continue
            if layer not in p["layers"]:
                continue
            r = r0 + (0.05 if p["npth"] else 0.0)
            s = self._near(p["box"], r)
            if s:
                m[s] |= _dist_box(self.X[s], self.Y[s], p["box"]) <= r
        for sg in g.segs:
            n, l_, x1, y1, x2, y2, hw = sg
            if n == net or l_ != layer or id(sg) in self.soft:
                continue
            s = self._near((min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)), hw + r0)
            if s:
                m[s] |= _dist_seg(self.X[s], self.Y[s], x1, y1, x2, y2) <= hw + r0
        for v in g.vias:
            n, x, y, vr = v
            if n == net or id(v) in self.soft:
                continue
            s = self._near((x, y, x, y), vr + r0)
            if s:
                m[s] |= np.hypot(self.X[s] - x, self.Y[s] - y) <= vr + r0
        for n, layers, prio, pts, zc in g.zones:
            if n == net or n in FILLS or layer not in layers:
                continue
            r = w / 2 + max(CLEAR, zc) + SLACK
            box = (pts[:, 0].min(), pts[:, 1].min(), pts[:, 0].max(), pts[:, 1].max())
            s = self._near(box, r)
            if not s:
                continue
            X, Y = self.X[s], self.Y[s]
            z = _inside_poly(X, Y, pts)
            for i in range(len(pts)):
                (xa, ya), (xb, yb) = pts[i], pts[(i + 1) % len(pts)]
                z |= _dist_seg(X, Y, xa, ya, xb, yb) <= r
            m[s] |= z
        e = EDGE + w / 2
        m |= (self.X < e) | (self.X > 150 - e) | (self.Y < e) | (self.Y > 150 - e)
        return m

    def _via_ok(self):
        g, net = self.geo, self.net
        bad = np.zeros((self.ny, self.nx), bool)
        rv = VIA_D / 2 + CLEAR + SLACK
        for p in g.pads:
            same = p["net"] == net and net
            r = VIA_D / 2 + (0.1 if same else CLEAR + SLACK)       # never in a pad, own net included
            s = self._near(p["box"], r)
            if s:
                bad[s] |= _dist_box(self.X[s], self.Y[s], p["box"]) <= r
        for sg in g.segs:
            n, l_, x1, y1, x2, y2, hw = sg
            if n == net or id(sg) in self.soft:
                continue
            s = self._near((min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)), hw + rv)
            if s:
                bad[s] |= _dist_seg(self.X[s], self.Y[s], x1, y1, x2, y2) <= hw + rv
        for v in g.vias:
            n, x, y, vr = v
            if id(v) in self.soft and n != net:
                continue
            r = vr + VIA_D / 2 + (0.25 if n == net else CLEAR + SLACK)
            s = self._near((x, y, x, y), r)
            if s:
                bad[s] |= np.hypot(self.X[s] - x, self.Y[s] - y) <= r
        for n, layers, prio, pts, zc in g.zones:          # never through 42 / 54 V copper of another net
            if n == net or n in FILLS or not _hv(n):
                continue
            r = VIA_D / 2 + max(zc, 0.5) + SLACK
            box = (pts[:, 0].min(), pts[:, 1].min(), pts[:, 0].max(), pts[:, 1].max())
            s = self._near(box, r)
            if not s:
                continue
            X, Y = self.X[s], self.Y[s]
            z = _inside_poly(X, Y, pts)
            for i in range(len(pts)):
                (xa, ya), (xb, yb) = pts[i], pts[(i + 1) % len(pts)]
                z |= _dist_seg(X, Y, xa, ya, xb, yb) <= r
            bad[s] |= z
        e = EDGE + VIA_D / 2
        bad |= (self.X < e) | (self.X > 150 - e) | (self.Y < e) | (self.Y > 150 - e)
        return ~bad

    def add_cost(self, layer, mask, factor):
        self.cost[layer] = np.where(mask, self.cost[layer] * factor, self.cost[layer])

    def near_mask(self, segs, r0, r1):
        """Cells between r0 and r1 from the given segments (x1, y1, x2, y2)."""
        d = np.full((self.ny, self.nx), 1e9)
        for x1, y1, x2, y2 in segs:
            d = np.minimum(d, _dist_seg(self.X, self.Y, x1, y1, x2, y2))
        return (d >= r0) & (d <= r1)


# ---------------------------------------------------------------- A*
def astar(win, starts, targets, via_targets=frozenset(), max_expand=250000):
    """starts: [(layer, iy, ix)]; targets: set of (layer, iy, ix); via_targets: cells (iy, ix) where a via ends the
    route. Returns [(layer, iy, ix, via_here)] or None.
    A node is (layer, cell); the heading it was reached with is kept per node for the turn price (the usual
    approximation: a node keeps the heading of its best arrival)."""
    L = win.layers
    nL = len(L)
    li = {l_: k for k, l_ in enumerate(L)}
    nx, ny = win.nx, win.ny
    n = nx * ny
    blk, cst = [], []
    for l_ in L:
        blk += win.block[l_].ravel().tolist()
        cst += win.cost[l_].ravel().tolist()
    vok = win.via_ok.ravel().tolist()
    vcs = win.via_cost.ravel().tolist()
    tset = {li[l_] * n + iy * nx + ix for l_, iy, ix in targets if l_ in li}
    vset = {iy * nx + ix for iy, ix in via_targets}
    if not tset and not vset:
        return None
    cells = [k % n for k in tset] + list(vset)
    tb = (min(c % nx for c in cells), min(c // nx for c in cells), max(c % nx for c in cells),
          max(c // nx for c in cells))
    r2 = math.sqrt(2) - 1

    def h(i):
        y, x = divmod(i, nx)
        dx = tb[0] - x if x < tb[0] else (x - tb[2] if x > tb[2] else 0)
        dy = tb[1] - y if y < tb[1] else (y - tb[3] if y > tb[3] else 0)
        return PITCH * (dx + r2 * dy if dx > dy else dy + r2 * dx)

    INF = float("inf")
    g = [INF] * (nL * n)
    prev = [-1] * (nL * n)
    head = [8] * (nL * n)                                 # 8: no heading (start, or just after a via)
    pq = []
    for l_, iy, ix in starts:
        if l_ not in li or not (0 <= iy < ny and 0 <= ix < nx):
            continue
        u = li[l_] * n + iy * nx + ix
        g[u] = 0.0
        heapq.heappush(pq, (h(iy * nx + ix), 0.0, u))
    steps = [(dx, dy, dy * nx + dx, PITCH * (math.sqrt(2) if dx and dy else 1.0)) for dx, dy in DIRS]
    turn_cost = [[TURN.get(min((k - d) % 8, (d - k) % 8), None) for k in range(8)] for d in range(8)]
    turn_cost.append([0.0] * 8)
    expanded = 0
    while pq:
        f, gc, u = heapq.heappop(pq)
        if gc > g[u] + 1e-9:
            continue
        lk, i = divmod(u, n)
        if u in tset or (i in vset and vok[i]):
            path = []
            via_end = u not in tset
            while u != -1:
                path.append(u)
                u = prev[u]
            path.reverse()
            out = []
            for k, w_ in enumerate(path):
                l2, i2 = divmod(w_, n)
                y, x = divmod(i2, nx)
                out.append((L[l2], y, x, k > 0 and path[k - 1] // n != l2))
            if via_end:
                out.append((None, out[-1][1], out[-1][2], True))
            return out
        expanded += 1
        if expanded > max_expand:
            return None
        y, x = divmod(i, nx)
        base = lk * n
        tcs = turn_cost[head[u]]
        ci = cst[u]
        for k in range(8):
            tc = tcs[k]
            if tc is None:
                continue
            dx, dy, di, ln = steps[k]
            xx, yy = x + dx, y + dy
            if xx < 0 or xx >= nx or yy < 0 or yy >= ny:
                continue
            v = u + di
            if blk[v]:
                continue
            if dx and dy and (blk[u + dx] or blk[u + dy * nx]):
                continue                                  # no diagonal squeeze past a corner
            ng = gc + ln * 0.5 * (ci + cst[v]) + tc
            if ng < g[v] - 1e-9:
                g[v] = ng
                prev[v] = u
                head[v] = k
                heapq.heappush(pq, (ng + h(i + di), ng, v))
        if vok[i] and (head[u] != 8 or prev[u] == -1):
            for l2 in range(nL):
                if l2 == lk:
                    continue
                v = l2 * n + i
                if blk[v]:
                    continue
                ng = gc + VIA_COST * vcs[i]
                if ng < g[v] - 1e-9:
                    g[v] = ng
                    prev[v] = u
                    head[v] = 8
                    heapq.heappush(pq, (ng + h(i), ng, v))
    return None


def path_to_runs(win, path):
    """Grid path -> [(layer, [points])] straight runs and via points."""
    runs, vias = [], []
    cur, pts, last_dir = None, [], None
    for k, (l_, iy, ix, via) in enumerate(path):
        p = win.xy(iy, ix)
        if l_ is None:                                    # via that ends the route
            vias.append(p)
            continue
        if via and cur is not None:
            vias.append(p)
            runs.append((cur, pts))
            cur, pts, last_dir = l_, [p], None
            continue
        if cur is None:
            cur, pts = l_, [p]
            continue
        dirn = (round((p[0] - pts[-1][0]) / PITCH), round((p[1] - pts[-1][1]) / PITCH))
        if len(pts) >= 2 and dirn == last_dir:
            pts[-1] = p
        else:
            pts.append(p)
        last_dir = dirn
    if cur is not None:
        runs.append((cur, pts))
    return runs, vias


# ---------------------------------------------------------------- the router
class Router:
    """Adds tracks and vias to the board and keeps the geometry snapshot current.

    A terminal is dict(points=[(layer, x, y)], pads={(ref, num)}, segs=[...], vias=[...]): where a route may start,
    and the copper it owns already (its pad, its escape stub, an escape via). route_net() grows a tree from the
    first terminal; each next one is routed to the tree only (not to any other copper of the net)."""

    def __init__(self, b, log=print):
        self.b = b
        self.geo = Geometry(b)
        self.log = log
        self.failed = []
        self.objs = {}                                    # id(seg / via tuple) -> the board item, for undo
        self.last = ([], [])                              # copper added by the last route_net (undo)

    def remove(self, segs, vias):
        """Take routed copper off the board again (rip-up)."""
        for seg in segs:
            obj = self.objs.pop(id(seg), None)
            if obj is not None:
                self.b.Remove(obj)
            if seg in self.geo.segs:
                self.geo.segs.remove(seg)
        for v in vias:
            obj = self.objs.pop(id(v), None)
            if obj is not None:
                self.b.Remove(obj)
            if v in self.geo.vias:
                self.geo.vias.remove(v)

    # -- writing copper
    def restore(self, copper):
        """Put ripped-up copper back. Returns the new (segs, vias) records."""
        segs, vias = [], []
        for n, l_, x1, y1, x2, y2, hw in copper[0]:
            segs += self.track(n, l_, [(x1, y1), (x2, y2)], hw * 2)
        for n, x, y, vr in copper[1]:
            vias.append(self.via(n, x, y))
        return segs, vias

    def track(self, net, layer, pts, w):
        n = self.b.FindNet(net)
        out = []
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            if math.hypot(x2 - x1, y2 - y1) < 1e-4:
                continue
            t = pcbnew.PCB_TRACK(self.b)
            t.SetStart(pcbnew.VECTOR2I(MM(X0 + x1), MM(Y0 + y1)))
            t.SetEnd(pcbnew.VECTOR2I(MM(X0 + x2), MM(Y0 + y2)))
            t.SetWidth(MM(w))
            t.SetLayer(layer)
            t.SetNet(n)
            self.b.Add(t)
            seg = (net, layer, x1, y1, x2, y2, w / 2)
            self.geo.segs.append(seg)
            self.objs[id(seg)] = t
            out.append(seg)
        return out

    def via(self, net, x, y):
        v = pcbnew.PCB_VIA(self.b)
        v.SetPosition(pcbnew.VECTOR2I(MM(X0 + x), MM(Y0 + y)))
        v.SetWidth(MM(VIA_D))
        v.SetDrill(MM(VIA_DRILL))
        v.SetNet(self.b.FindNet(net))
        self.b.Add(v)
        rec = (net, x, y, VIA_D / 2)
        self.geo.vias.append(rec)
        self.objs[id(rec)] = v
        return rec

    # -- terminals
    def pad(self, ref, num):
        return next(p for p in self.geo.pads if p["ref"] == ref and p["num"] == num)

    def t_pad(self, ref, num, escape=False, w=0.25, length=0.5):
        """A pad terminal; escape: a fine-pitch pin leaves through a straight stub of pad width along its long axis,
        away from its footprint centre ("in": towards it, under the body), and the route starts at the stub's end."""
        p = self.pad(ref, num)
        layer = B if B in p["layers"] else next(iter(p["layers"]))
        cx, cy = p["c"]
        if not escape:
            return dict(points=[(layer, cx, cy)], pads={(ref, num)}, segs=[], vias=[])
        l, t, r, b = p["box"]
        fx, fy = p["fc"]
        inward = escape == "in"
        if (b - t) >= (r - l):
            sgn = (1 if cy >= fy else -1) * (-1 if inward else 1)
            ex, ey, sw = cx, (b if sgn > 0 else t) + sgn * length, min(w, r - l)
        else:
            sgn = (1 if cx >= fx else -1) * (-1 if inward else 1)
            ex, ey, sw = (r if sgn > 0 else l) + sgn * length, cy, min(w, b - t)
        segs = self.track(p["net"], layer, [(cx, cy), (ex, ey)], sw)
        return dict(points=[(layer, ex, ey)], pads={(ref, num)}, segs=segs, vias=[])

    def t_via(self, x, y):
        """An existing via (an escape via from the top) as a terminal."""
        return dict(points=[(B, x, y), (IN2, x, y)], pads=set(), segs=[], vias=[(x, y)])

    def tree_targets(self, win, tree):
        """Grid cells of the copper connected so far: its pads (inside, shrunk by half a track), tracks, vias."""
        tg = set()
        hw = win.w / 2
        for ref, num in tree["pads"]:
            p = self.pad(ref, num)
            l, t, r, b = p["box"]
            inner = (l + hw * 0.5, t + hw * 0.5, r - hw * 0.5, b - hw * 0.5)
            if inner[0] > inner[2] or inner[1] > inner[3]:
                inner = (p["c"][0], p["c"][1], p["c"][0], p["c"][1])
            s = win._near(inner, 0)
            if not s:
                continue
            m = _dist_box(win.X[s], win.Y[s], inner) <= PITCH * 0.6
            for l_ in win.layers:
                if l_ in p["layers"]:
                    ys, xs = np.nonzero(m)
                    tg |= {(l_, int(yy) + s[0].start, int(xx) + s[1].start) for yy, xx in zip(ys, xs)}
        for n, l_, x1, y1, x2, y2, shw in tree["segs"]:
            if l_ not in win.layers:
                continue
            s = win._near((min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)), PITCH)
            if not s:
                continue
            m = _dist_seg(win.X[s], win.Y[s], x1, y1, x2, y2) <= PITCH * 0.75
            ys, xs = np.nonzero(m)
            tg |= {(l_, int(yy) + s[0].start, int(xx) + s[1].start) for yy, xx in zip(ys, xs)}
        for v in tree["vias"]:
            iy, ix = win.cell(v[-3], v[-2]) if len(v) == 4 else win.cell(*v)
            if 0 <= iy < win.ny and 0 <= ix < win.nx:
                for l_ in win.layers:
                    tg.add((l_, iy, ix))
        return tg

    def _snap_end(self, tree, layer, p):
        """Join a run end that stops on a grid cell to the exact tree copper it meant to reach."""
        x, y = p
        for ref, num in tree["pads"]:
            q = self.pad(ref, num)
            if layer in q["layers"]:
                l, t, r, b = q["box"]
                if l - 1e-6 <= x <= r + 1e-6 and t - 1e-6 <= y <= b + 1e-6:
                    return None                           # inside its pad: connected
        for v in tree["vias"]:
            vx, vy = (v[1], v[2]) if len(v) == 4 else v
            if math.hypot(x - vx, y - vy) <= VIA_D / 2:
                return None
        best = None
        for n, l_, x1, y1, x2, y2, hw in tree["segs"]:
            if l_ != layer:
                continue
            dx, dy = x2 - x1, y2 - y1
            L = dx * dx + dy * dy
            u = 0 if L == 0 else max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / L))
            q = (x1 + u * dx, y1 + u * dy)
            d = math.hypot(x - q[0], y - q[1])
            if d <= hw - 1e-6:
                return None
            if best is None or d < best[0]:
                best = (d, q)
        return best[1] if best and best[0] < 0.3 else None

    def connect(self, net, term, tree, w, box, layers=(B, IN2), neck=None, cost_fn=None, label=None,
                zone_layers=None, win=None, quiet=False):
        """Route one terminal to the tree, or (zone_layers) to a via inside the net's own pour or plane on those
        layers. Returns (segs, vias) added, or None."""
        start_pts = term["points"]
        if win is None:
            win = Window(self.geo, net, box, w, layers=layers, origin=start_pts[0][1:], neck=neck)
            if cost_fn:
                cost_fn(win)
        saved = {l_: win.block[l_].copy() for l_ in win.layers}
        starts = []
        for l_, x, y in start_pts:
            iy, ix = win.cell(x, y)
            if 0 <= iy < win.ny and 0 <= ix < win.nx and l_ in win.layers:
                win.block[l_][iy, ix] = False             # the start sits in its own copper
                starts.append((l_, iy, ix))
        targets = self.tree_targets(win, tree) if tree else set()
        for l_, iy, ix in targets:                        # the tree's copper is reachable
            win.block[l_][iy, ix] = False
        vt = frozenset()
        if zone_layers:
            own = own_copper(self.geo, win, net, zone_layers)
            ys, xs = np.nonzero(own & win.via_ok)
            vt = frozenset(zip(ys.tolist(), xs.tolist()))
        path = astar(win, starts, targets, vt)
        win.block = saved
        if path is None:
            if not quiet:
                self.failed.append(label or net)
                self.log(f"   no route: {label or net}")
            return None
        runs, vias = path_to_runs(win, path)
        segs, vs = [], []
        for k, (l_, pts) in enumerate(runs):
            if k == 0:
                first = pts[0]
                src = min(start_pts, key=lambda q: math.hypot(q[1] - first[0], q[2] - first[1]))
                if math.hypot(src[1] - first[0], src[2] - first[1]) > 1e-6:
                    pts = [(src[1], src[2])] + pts
            if k == len(runs) - 1 and tree:
                end = self._snap_end(tree, l_, pts[-1])
                if end:
                    pts = pts + [end]
            if win.neck:
                nbox, wn = win.neck
                for a, c in zip(pts, pts[1:]):
                    mx, my = (a[0] + c[0]) / 2, (a[1] + c[1]) / 2
                    inn = nbox[0] <= mx <= nbox[2] and nbox[1] <= my <= nbox[3]
                    segs += self.track(net, l_, [a, c], wn if inn else w)
            else:
                segs += self.track(net, l_, pts, w)
        for x, y in vias:
            vs.append(self.via(net, x, y))
        return segs, vs

    def route_net(self, net, terms, w, box, label=None, win=None, **kw):
        """terms: terminals; the first seeds the tree, the rest join it, nearest first. True when all joined.
        self.last holds the copper added (for rip-up)."""
        self.last = ([], [])
        tree = dict(pads=set(terms[0]["pads"]), segs=list(terms[0]["segs"]), vias=list(terms[0]["vias"]))
        pts = [(x, y) for _, x, y in terms[0]["points"]]
        rest = list(terms[1:])
        ok = True
        while rest:
            near = pts + [((x1 + x2) / 2, (y1 + y2) / 2) for _, _, x1, y1, x2, y2, _ in tree["segs"]]
            rest.sort(key=lambda t: min(math.hypot(t["points"][0][1] - x, t["points"][0][2] - y) for x, y in near))
            t = rest.pop(0)
            res = self.connect(net, t, tree, w, box, label=label or net, win=win, **kw)
            tree["pads"] |= t["pads"]
            tree["segs"] += t["segs"]
            tree["vias"] += t["vias"]
            pts += [(x, y) for _, x, y in t["points"]]
            if res is None:
                ok = False
                continue
            tree["segs"] += res[0]
            tree["vias"] += res[1]
            self.last[0].extend(res[0])
            self.last[1].extend(res[1])
        return ok

    def fanout(self, ref, num, w=0.4, rmax=3.0, zone_layers=None, label=None, optional=False):
        """A via for a supply pad: the nearest legal spot (inside its own pour on zone_layers, if given), joined
        by a straight track from the pad centre. Returns True when done."""
        pad = self.pad(ref, num)
        net = pad["net"]
        cx, cy = pad["c"]
        layer = B if B in pad["layers"] else next(iter(pad["layers"]))
        box = (cx - rmax - 1.5, cy - rmax - 1.5, cx + rmax + 1.5, cy + rmax + 1.5)
        win = Window(self.geo, net, box, w, layers=(layer,), origin=(cx, cy))
        own = None
        if zone_layers:
            own = own_copper(self.geo, win, net, zone_layers)
        l, t, r, b = pad["box"]
        dist = max(r - l, b - t) / 2 + VIA_D / 2 + 0.2
        while dist <= rmax:
            for k in range(32):
                a = 2 * math.pi * k / 32
                x, y = cx + dist * math.cos(a), cy + dist * math.sin(a)
                iy, ix = win.cell(x, y)
                if not (0 <= iy < win.ny and 0 <= ix < win.nx) or not win.via_ok[iy, ix]:
                    continue
                if own is not None and not own[iy, ix]:
                    continue
                vx, vy = win.xy(iy, ix)
                n = max(2, int(math.hypot(vx - cx, vy - cy) / (PITCH / 2)))
                clear = True
                for kk in range(n + 1):
                    px, py = cx + (vx - cx) * kk / n, cy + (vy - cy) * kk / n
                    jy, jx = win.cell(px, py)
                    if not (l <= px <= r and t <= py <= b) and win.block[layer][jy, jx]:
                        clear = False
                        break
                if clear:
                    self.track(net, layer, [(cx, cy), (vx, vy)], w)
                    self.via(net, vx, vy)
                    return True
            dist += 0.1
        if not optional:
            self.failed.append(label or f"{ref}.{num} via")
            self.log(f"   no via spot: {label or f'{ref}.{num}'}")
        return False

    def to_pour(self, net, term, w, box, zone_layers, label=None, **kw):
        """Route a supply terminal to a via in its own pour / plane on zone_layers."""
        return self.connect(net, term, None, w, box, zone_layers=zone_layers, label=label, **kw) is not None


# ---------------------------------------------------------------- negotiated congestion (PathFinder)
def _mask(shape, X, Y, x0, y0, segs, vias, layers, extra=0.0):
    """The cells a net's copper claims (within half its width + half the clearance), one bool array per layer."""
    out = {l_: np.zeros(shape, bool) for l_ in layers}
    for n, l_, x1, y1, x2, y2, hw in segs:
        if l_ not in out:
            continue
        r = hw + CLEAR / 2 + extra
        ix0 = max(0, int((min(x1, x2) - r - x0) / PITCH) - 1)
        ix1 = min(shape[1], int((max(x1, x2) + r - x0) / PITCH) + 2)
        iy0 = max(0, int((min(y1, y2) - r - y0) / PITCH) - 1)
        iy1 = min(shape[0], int((max(y1, y2) + r - y0) / PITCH) + 2)
        if ix0 < ix1 and iy0 < iy1:
            sl = (slice(iy0, iy1), slice(ix0, ix1))
            out[l_][sl] |= _dist_seg(X[sl], Y[sl], x1, y1, x2, y2) <= r
    for v in vias:
        x, y = v[1], v[2]
        r = VIA_D / 2 + CLEAR / 2 + extra
        ix0 = max(0, int((x - r - x0) / PITCH) - 1)
        ix1 = min(shape[1], int((x + r - x0) / PITCH) + 2)
        iy0 = max(0, int((y - r - y0) / PITCH) - 1)
        iy1 = min(shape[0], int((y + r - y0) / PITCH) + 2)
        if ix0 < ix1 and iy0 < iy1:
            sl = (slice(iy0, iy1), slice(ix0, ix1))
            m = np.hypot(X[sl] - x, Y[sl] - y) <= r
            for l_ in layers:
                out[l_][sl] |= m
    return out


class Negotiator:
    """Route a set of nets together: each net may overlap the others' copper at a price that rises every round,
    and cells that keep being fought over get dearer (history), until no two nets claim the same cell."""

    def __init__(self, R, box, origin, layers=(B, IN2)):
        self.R, self.layers = R, list(layers)
        x0, y0, x1, y1 = box
        ox, oy = origin
        self.x0 = ox - math.ceil((ox - x0) / PITCH) * PITCH
        self.y0 = oy - math.ceil((oy - y0) / PITCH) * PITCH
        self.nx = int((x1 - self.x0) / PITCH) + 1
        self.ny = int((y1 - self.y0) / PITCH) + 1
        xs = self.x0 + np.arange(self.nx) * PITCH
        ys = self.y0 + np.arange(self.ny) * PITCH
        self.X, self.Y = np.meshgrid(xs, ys)
        self.occ = {l_: np.zeros((self.ny, self.nx)) for l_ in self.layers}
        self.hist = {l_: np.zeros((self.ny, self.nx)) for l_ in self.layers}
        self.jobs = []
        self.soft = set()                                 # ids of copper placed by the negotiation

    def add(self, net, terms, w, label, cost_fn=None, neck=None, zone_layers=None):
        self.jobs.append(dict(net=net, terms=terms, w=w, label=label, cost_fn=cost_fn, neck=neck,
                              zone_layers=zone_layers, copper=([], []), win=None, mask=None))

    def _window(self, job):
        pts = [(x, y) for t in job["terms"] for _, x, y in t["points"]]
        pts += [p["c"] for t in job["terms"] for p in (self.R.pad(r_, n_) for r_, n_ in t["pads"])]
        full = (self.x0, self.y0, self.x0 + (self.nx - 1) * PITCH, self.y0 + (self.ny - 1) * PITCH)
        if job["zone_layers"]:                             # to a pour: anywhere in the block
            box = full
        else:
            m = 3.5
            box = (max(full[0], min(x for x, _ in pts) - m), max(full[1], min(y for _, y in pts) - m),
                   min(full[2], max(x for x, _ in pts) + m), min(full[3], max(y for _, y in pts) + m))
        return Window(self.R.geo, job["net"], box, job["w"], layers=self.layers, origin=(self.x0, self.y0),
                      neck=job["neck"], soft=self.soft)

    def _sub(self, win, arr):
        ix = int(round((win.x0 - self.x0) / PITCH))
        iy = int(round((win.y0 - self.y0) / PITCH))
        return arr[iy:iy + win.ny, ix:ix + win.nx]

    def _route(self, job, pfac):
        R = self.R
        t0 = time.time()
        if job["win"] is None:
            job["win"] = self._window(job)
        win = job["win"]
        win.reset_cost()
        if job["cost_fn"]:
            job["cost_fn"](win)
        r_own = job["w"] / 2 + CLEAR / 2 + SLACK
        k = int(math.ceil(r_own / PITCH))
        offs = [(dy, dx) for dy in range(-k, k + 1) for dx in range(-k, k + 1) if math.hypot(dx, dy) * PITCH <= r_own]
        vk = int(math.ceil((VIA_D / 2 + CLEAR / 2) / PITCH))
        voffs = [(dy, dx) for dy in range(-vk, vk + 1) for dx in range(-vk, vk + 1)
                 if math.hypot(dx, dy) * PITCH <= VIA_D / 2 + CLEAR / 2]
        vmult = np.ones((win.ny, win.nx))
        same = [j["mask"] for j in self.jobs if j is not job and j["net"] == job["net"] and j["mask"] is not None]
        for l_ in win.layers:
            occ_all = self.occ[l_].copy()
            for m in same:                                 # own net's other jobs are no congestion
                occ_all -= m[l_]
            occ = self._sub(win, occ_all) > 0.5
            hist = self._sub(win, self.hist[l_])
            dil = np.zeros_like(occ)
            pad = np.pad(occ, k)
            for dy, dx in offs:
                dil |= pad[k + dy:k + dy + win.ny, k + dx:k + dx + win.nx]
            win.cost[l_] = win.cost[l_] * (1.0 + hist) * (1.0 + pfac * dil)
            vdil = np.zeros_like(occ)
            pv = np.pad(occ, vk)
            for dy, dx in voffs:
                vdil |= pv[vk + dy:vk + dy + win.ny, vk + dx:vk + dx + win.nx]
            vmult = vmult * (1.0 + hist) * (1.0 + pfac * vdil)
        win.via_cost = vmult
        if job["zone_layers"]:
            res = R.connect(job["net"], job["terms"][0], None, job["w"], None, zone_layers=job["zone_layers"],
                            win=win, quiet=True, label=job["label"])
            ok = res is not None
            copper = res if res else ([], [])
        else:
            mark = len(R.failed)
            ok = R.route_net(job["net"], job["terms"], job["w"], None, label=job["label"], win=win, quiet=True)
            del R.failed[mark:]
            copper = R.last
        job["copper"] = copper
        job["ok"] = ok
        if os.environ.get("ROUTE_DEBUG"):
            print(f"      {job['label']:<18} {'ok' if ok else 'FAIL'} {time.time() - t0:5.1f} s", flush=True)
        self.soft |= {id(c) for c in copper[0]} | {id(c) for c in copper[1]}
        job["mask"] = _mask((self.ny, self.nx), self.X, self.Y, self.x0, self.y0, copper[0], copper[1], self.layers)
        for l_ in self.layers:
            self.occ[l_] += job["mask"][l_]

    def report(self, log=print):
        """Which nets still claim the same cells, and where."""
        jobs = [j for j in self.jobs if j["mask"] is not None]
        for i, a in enumerate(jobs):
            for b in jobs[i + 1:]:
                if a["net"] == b["net"]:
                    continue
                for l_ in self.layers:
                    m = a["mask"][l_] & b["mask"][l_]
                    if m.any():
                        ys, xs = np.nonzero(m)
                        x, y = float(self.X[0, int(xs.mean())]), float(self.Y[int(ys.mean()), 0])
                        log(f"     {a['label']} / {b['label']} on {LAYER_NAME[l_]}: {int(m.sum())} cells around "
                            f"({x:.1f}, {y:.1f})")
        for j in self.jobs:
            if not j.get("ok"):
                log(f"     {j['label']}: no path at all")

    def _length(self, copper):
        segs, vias = copper
        return sum(math.hypot(x2 - x1, y2 - y1) * LAYER_COST.get(l_, 2.0) for _, l_, x1, y1, x2, y2, _ in segs) \
            + VIA_COST * len(vias)

    def _hard(self, job, keep):
        """Re-route one job with every other net's copper hard; keep(old_copper, new_copper) decides whether the
        new route stays. Returns True when it did."""
        if job["mask"] is not None:
            for l_ in self.layers:
                self.occ[l_] -= job["mask"][l_]
        old_copper = job["copper"]
        self.R.remove(*old_copper)
        self.soft -= {id(c) for c in old_copper[0]} | {id(c) for c in old_copper[1]}
        if job["win"] is None:
            job["win"] = self._window(job)
        win = job["win"]
        saved = {l_: win.block[l_].copy() for l_ in win.layers}
        saved_via = win.via_ok                         # the other nets' copper is hard now, by exact distance
        w2 = job["w"] / 2
        vr = VIA_D / 2
        vbad = np.zeros((win.ny, win.nx), bool)
        for other in self.jobs:
            if other is job or other["net"] == job["net"]:
                continue
            for sg in other["copper"][0]:
                n, l_, x1, y1, x2, y2, hw = sg
                for r, tgt in ((w2 + hw + CLEAR + SLACK, "track"), (vr + hw + CLEAR + SLACK, "via")):
                    sl = win._near((min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)), r)
                    if not sl:
                        continue
                    m = _dist_seg(win.X[sl], win.Y[sl], x1, y1, x2, y2) <= r
                    if tgt == "via":
                        vbad[sl] |= m
                    elif l_ in win.block:
                        win.block[l_][sl] |= m
            for v in other["copper"][1]:
                n, x, y, rv = v
                for r, tgt in ((w2 + rv + CLEAR + SLACK, "track"), (vr + rv + CLEAR + SLACK, "via")):
                    sl = win._near((x, y, x, y), r)
                    if not sl:
                        continue
                    m = np.hypot(win.X[sl] - x, win.Y[sl] - y) <= r
                    if tgt == "via":
                        vbad[sl] |= m
                    else:
                        for l_ in win.layers:
                            win.block[l_][sl] |= m
        win.via_ok = saved_via & ~vbad
        win.reset_cost()
        if job["cost_fn"]:
            job["cost_fn"](win)
        mark = len(self.R.failed)
        if job["zone_layers"]:
            res = self.R.connect(job["net"], job["terms"][0], None, job["w"], None,
                                 zone_layers=job["zone_layers"], win=win, quiet=True, label=job["label"])
            ok, copper = res is not None, (res if res else ([], []))
        else:
            ok = self.R.route_net(job["net"], job["terms"], job["w"], None, label=job["label"], win=win,
                                  quiet=True)
            copper = self.R.last
        del self.R.failed[mark:]
        win.block, win.via_ok = saved, saved_via
        took = bool(ok and keep(old_copper, copper))
        if took:
            job["copper"], job["ok"] = copper, True
        else:                                              # keep the route it had
            self.R.remove(*copper)
            job["copper"] = self.R.restore(old_copper)
        job["mask"] = _mask((self.ny, self.nx), self.X, self.Y, self.x0, self.y0, job["copper"][0],
                            job["copper"][1], self.layers)
        for l_ in self.layers:
            self.occ[l_] += job["mask"][l_]
        self.soft |= {id(c) for c in job["copper"][0]} | {id(c) for c in job["copper"][1]}
        return took

    def polish(self, log=print):
        """After the negotiation: re-route each net with everyone else's copper hard, keep it if shorter."""
        gained = 0
        for job in self.jobs:
            if not job.get("ok") or job["mask"] is None:
                continue
            old_len = self._length(job["copper"])
            if self._hard(job, lambda old, new: self._length(new) < old_len - 0.05):
                gained += old_len - self._length(job["copper"])
        log(f"   polish: {gained:.1f} mm shorter")

    def _conflicts(self):
        """Mark the jobs whose copper shares cells with another net's (jobs of one net may share copper).
        Returns (contested cells, labels of jobs to redo) and adds to the history cost."""
        per_net = {}
        for job in self.jobs:
            if job["mask"] is None:
                continue
            m = per_net.setdefault(job["net"], {l_: np.zeros((self.ny, self.nx), bool) for l_ in self.layers})
            for l_ in self.layers:
                m[l_] |= job["mask"][l_]
        over = {l_: sum(m[l_].astype(np.int16) for m in per_net.values()) > 1 if per_net else
                np.zeros((self.ny, self.nx), bool) for l_ in self.layers}
        for l_ in self.layers:
            self.hist[l_] += 0.4 * over[l_]
        near = {}                                          # within NEAR of a contested cell: the neighbours
        k = int(round(NEAR / PITCH))
        for l_ in self.layers:                             # square dilation, k cells each way
            rows = over[l_].copy()
            for d in range(1, k + 1):
                rows[d:] |= over[l_][:-d]
                rows[:-d] |= over[l_][d:]
            m = rows.copy()
            for d in range(1, k + 1):
                m[:, d:] |= rows[:, :-d]
                m[:, :-d] |= rows[:, d:]
            near[l_] = m
        for job in self.jobs:
            job["conflict"] = job["mask"] is not None and any(bool((job["mask"][l_] & over[l_]).any())
                                                              for l_ in self.layers)
            job["near"] = job["mask"] is not None and any(bool((job["mask"][l_] & near[l_]).any())
                                                          for l_ in self.layers)
        return sum(int(m.sum()) for m in over.values()), [j["label"] for j in self.jobs
                                                          if j.get("conflict") or not j.get("ok")]

    def run(self, rounds=None, log=print):
        """Negotiate; when the rounds stall, settle what still conflicts by re-routing it around everyone else's
        copper (hard). Returns the labels of nets that still conflict or could not be routed."""
        pfac = 0.6
        rounds = rounds or int(os.environ.get("ROUTE_ROUNDS", 60))
        last, same_for = None, 0
        for it in range(rounds):
            for job in self.jobs:                          # the conflicting nets and their neighbours
                if it > 0 and not job.get("conflict") and not job.get("near") and job.get("ok"):
                    continue
                if job["mask"] is not None:
                    for l_ in self.layers:
                        self.occ[l_] -= job["mask"][l_]
                    self.soft -= {id(c) for c in job["copper"][0]} | {id(c) for c in job["copper"][1]}
                    self.R.remove(*job["copper"])
                    job["mask"], job["copper"] = None, ([], [])
                self._route(job, pfac)
            n_over, bad = self._conflicts()
            log(f"   round {it + 1}: {n_over} contested cells, {len(bad)} nets to redo")
            if not bad:
                return self._exact(log)
            same_for = same_for + 1 if (n_over, bad) == last else 0
            last = (n_over, bad)
            if same_for >= 8:                              # stalled: the same conflict round after round
                break
            pfac *= 1.5
        if os.environ.get("ROUTE_DUMP"):                   # development: the negotiation's state, for inspection
            import pickle
            pickle.dump(dict(x0=self.x0, y0=self.y0, jobs=[dict(label=j["label"], net=j["net"], w=j["w"],
                                                               mask=j["mask"][B] if j["mask"] is not None else None,
                                                               wx0=j["win"].x0, wy0=j["win"].y0,
                                                               block=j["win"].block[B], neck=j["win"].neck,
                                                               copper=[(s_[2], s_[3], s_[4], s_[5]) for s_ in j["copper"][0]
                                                                       if s_[1] == B])
                                                          for j in self.jobs if j["win"] is not None]),
                        open(os.environ["ROUTE_DUMP"], "wb"))
        for job in self.jobs:                              # settle: each conflicting net around the others
            if job.get("conflict") or not job.get("ok"):
                took = self._hard(job, lambda old, new: True)
                n_over, bad = self._conflicts()
                log(f"   settle {job['label']}: {'re-routed around the others' if took else 'no clear path'}, "
                    f"{n_over} contested cells left")
                if not bad:
                    return self._exact(log)
        return bad

    def violations(self):
        """Exact clearance check between the negotiated copper of different nets (the grid masks can miss a
        shortfall of a few hundredths): the labels of the jobs involved, with where."""
        out = []
        jobs = [j for j in self.jobs if j["mask"] is not None]
        for i, a in enumerate(jobs):
            for b in jobs[i + 1:]:
                if a["net"] == b["net"]:
                    continue
                for sa in a["copper"][0]:
                    for sb in b["copper"][0]:
                        if sa[1] == sb[1] and _seg_seg(sa[2:6], sb[2:6]) < sa[6] + sb[6] + CLEAR - 1e-4:
                            out.append((a["label"], b["label"], sa[2], sa[3]))
                    for vb in b["copper"][1]:
                        if _pt_seg(vb[1], vb[2], sa[2:6]) < sa[6] + vb[3] + CLEAR - 1e-4:
                            out.append((a["label"], b["label"], vb[1], vb[2]))
                for va in a["copper"][1]:
                    for sb in b["copper"][0]:
                        if _pt_seg(va[1], va[2], sb[2:6]) < sb[6] + va[3] + CLEAR - 1e-4:
                            out.append((a["label"], b["label"], va[1], va[2]))
                    for vb in b["copper"][1]:
                        if math.hypot(va[1] - vb[1], va[2] - vb[2]) < va[3] + vb[3] + CLEAR - 1e-4:
                            out.append((a["label"], b["label"], va[1], va[2]))
        return out

    def _exact(self, log=print):
        """After the negotiation: any exact clearance shortfall left is settled by re-routing one of the two nets
        around the other's real copper. Returns the labels still in violation."""
        for _ in range(4):
            bad = self.violations()
            if not bad:
                return []
            for la, lb, x, y in bad:
                log(f"   exact check: {la} / {lb} too close at ({x:.2f}, {y:.2f})")
            done = set()
            for la, lb, x, y in bad:
                for lab in (lb, la):                       # the later job first: it was routed into the earlier
                    job = next(j for j in self.jobs if j["label"] == lab)
                    if lab in done or la in done and lb in done:
                        continue
                    if self._hard(job, lambda old, new: True):
                        done.add(lab)
                        break
        bad = self.violations()
        return sorted({l_ for la, lb, _, _ in bad for l_ in (la, lb)})
