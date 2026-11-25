#!/usr/bin/env python3
"""Track tidy-up after routing (2026-11-24): the geometry the router and the plans leave behind, cleaned up without
changing what connects to what. Run with KiCad's Python after pcb_gnd.py:

    python3 kicad/rev3/gen/pcb_tidy.py

  - duplicates: a segment lying exactly on another of the same net, layer and width goes; zero-length ones too
  - kinks: a piece shorter than 0.05 mm between two runs is absorbed, the runs meeting where their lines cross (or
    at the piece's middle where they are parallel)
  - collinear runs: two segments meeting end to end in a straight line at a bare joint become one
  - right angles: every 90-degree corner at a bare joint gets a 45-degree chamfer, as long as the legs allow
    (up to 0.8 mm, at most 45 % of each leg), trying smaller ones where the big one would come closer to other
    copper

A "bare" joint is one where exactly two segments of the net meet on that layer, with no pad or via there. A
chamfer cuts the inside of the corner, so it is checked against every other net's track, via and pad on its layer:
it must keep 0.5 mm from them, or as much as the corner it replaces kept if that was less. It must not enter a rule
area that forbids tracks unless the corner was already in it. The pours are refilled afterwards and follow the new
tracks themselves.
"""
import math
import os
import sys
from collections import defaultdict

import pcbnew

BOARD = sys.argv[1] if len(sys.argv) > 1 else "kicad/rev3/PowerBoard_rev3.kicad_pcb"
TINY = 0.08                      # mm: kinks shorter than this are absorbed
STEP = 0.3                       # mm: a run of pieces shorter than this between two longer runs is a staircase
CHAMFER = (0.8, 0.5, 0.3, 0.2, 0.12)   # mm along each leg, largest first
SAFE = 0.5                       # mm: always enough (the HV rule's clearance)
mm, iu = pcbnew.ToMM, pcbnew.FromMM
V = lambda x, y: pcbnew.VECTOR2I(int(round(x)), int(round(y)))

b = pcbnew.LoadBoard(BOARD)
CU = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]


DEAD = set()    # removed tracks' ids: BOARD.Remove() mid-run breaks pcbnew's later calls, so they go at the end


def uid(t):
    return t.m_Uuid.AsString()


def kill(t):
    DEAD.add(uid(t))


def tracks():
    return [t for t in b.GetTracks() if t.Type() == pcbnew.PCB_TRACE_T and uid(t) not in DEAD]


def pt(p):
    return (p.x, p.y)


# fixed connection points: pads and vias, per (net, layer)
def anchors():
    out = defaultdict(list)
    for fp in b.GetFootprints():
        for p in fp.Pads():
            for L in CU:
                if p.IsOnLayer(L):
                    out[(p.GetNetCode(), L)].append(p)
    for v in b.GetTracks():
        if v.Type() == pcbnew.PCB_VIA_T:
            for L in CU:
                if v.IsOnLayer(L):
                    out[(v.GetNetCode(), L)].append(v)
    return out


def on_anchor(anc, net, L, xy):
    q = V(*xy)
    for a in anc.get((net, L), ()):
        if a.HitTest(q, 0):
            return True
    return False


def joints():
    """(net, layer, point) -> [(track, which end)]"""
    j = defaultdict(list)
    for t in tracks():
        j[(t.GetNetCode(), t.GetLayer(), pt(t.GetStart()))].append((t, 0))
        j[(t.GetNetCode(), t.GetLayer(), pt(t.GetEnd()))].append((t, 1))
    return j


def far(t, end):
    return pt(t.GetEnd()) if end == 0 else pt(t.GetStart())


def set_end(t, end, xy):
    (t.SetStart if end == 0 else t.SetEnd)(V(*xy))


# ---- 0. weld: track ends within a few nm of a via centre or of each other (rounding) made to coincide ---------------
WELD = 5            # nm
welded = 0
vcs = defaultdict(list)
for v in b.GetTracks():
    if v.Type() == pcbnew.PCB_VIA_T:
        p = pt(v.GetPosition())
        vcs[(v.GetNetCode(), p[0] // 1000, p[1] // 1000)].append(p)
ends_seen = defaultdict(list)
for t in tracks():
    for e in (0, 1):
        p = pt(t.GetStart() if e == 0 else t.GetEnd())
        net = t.GetNetCode()
        target = None
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for q in vcs.get((net, p[0] // 1000 + dx, p[1] // 1000 + dy), ()):
                    if q != p and abs(q[0] - p[0]) <= WELD and abs(q[1] - p[1]) <= WELD:
                        target = q
        if target is None:
            k = (net, t.GetLayer(), p[0] // 1000, p[1] // 1000)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for q in ends_seen.get((net, t.GetLayer(), k[2] + dx, k[3] + dy), ()):
                        if q != p and abs(q[0] - p[0]) <= WELD and abs(q[1] - p[1]) <= WELD:
                            target = q
        if target is not None:
            set_end(t, e, target)
            welded += 1
            p = target
        ends_seen[(net, t.GetLayer(), p[0] // 1000, p[1] // 1000)].append(p)


# ---- 1. duplicates and zero-length segments ---------------------------------------------------------------------
seen, dup, zero = set(), 0, 0
for t in tracks():
    a, c = pt(t.GetStart()), pt(t.GetEnd())
    if a == c:
        kill(t); zero += 1; continue
    k = (t.GetNetCode(), t.GetLayer(), t.GetWidth(), min(a, c), max(a, c))
    if k in seen:
        kill(t); dup += 1
    else:
        seen.add(k)

anc = anchors()


def bare(key, lst):
    return len(lst) == 2 and not on_anchor(anc, key[0], key[1], key[2])


# ---- 2. kinks ---------------------------------------------------------------------------------------------------
def cross(p0, d0, p1, d1):
    den = d0[0] * d1[1] - d0[1] * d1[0]
    if abs(den) < 1e-9 * math.hypot(*d0) * math.hypot(*d1):
        return None
    s = ((p1[0] - p0[0]) * d1[1] - (p1[1] - p0[1]) * d1[0]) / den
    return (p0[0] + s * d0[0], p0[1] + s * d0[1])


kinks = 0
changed = True
while changed:
    changed = False
    J = joints()
    for t in tracks():
        a, c = pt(t.GetStart()), pt(t.GetEnd())
        if math.dist(a, c) >= iu(TINY):
            continue
        n, L = t.GetNetCode(), t.GetLayer()
        ja, jc = J[(n, L, a)], J[(n, L, c)]
        if not (bare((n, L, a), ja) and bare((n, L, c), jc)):
            continue
        (ta, ea), = [e for e in ja if uid(e[0]) != uid(t)]
        (tc, ec), = [e for e in jc if uid(e[0]) != uid(t)]
        if uid(ta) == uid(tc):
            continue
        pa, pc = far(ta, ea), far(tc, ec)
        x = cross(pa, (a[0] - pa[0], a[1] - pa[1]), pc, (c[0] - pc[0], c[1] - pc[1]))
        if x is None or math.dist(x, a) > iu(0.1) or math.dist(x, c) > iu(0.1):
            x = ((a[0] + c[0]) / 2, (a[1] + c[1]) / 2)
        set_end(ta, ea, x)
        set_end(tc, ec, x)
        kill(t)
        kinks += 1
        changed = True
        break


# ---- 3. collinear runs ------------------------------------------------------------------------------------------
def angle_at(t0, e0, t1, e1, p):
    u, v = far(t0, e0), far(t1, e1)
    du, dv = (u[0] - p[0], u[1] - p[1]), (v[0] - p[0], v[1] - p[1])
    lu, lv = math.hypot(*du), math.hypot(*dv)
    if lu == 0 or lv == 0:
        return None, du, dv, lu, lv
    cosv = max(-1.0, min(1.0, (du[0] * dv[0] + du[1] * dv[1]) / lu / lv))
    return math.degrees(math.acos(cosv)), du, dv, lu, lv


def merge_pass():
    """Two segments meeting end to end in a straight line at a bare joint: one."""
    J = joints()
    for key, lst in J.items():
        if not bare(key, lst):
            continue
        (t0, e0), (t1, e1) = lst
        if uid(t0) == uid(t1) or t0.GetWidth() != t1.GetWidth():
            continue
        ang = angle_at(t0, e0, t1, e1, key[2])[0]
        if ang is None or ang < 179.9:
            continue
        set_end(t0, e0, far(t1, e1))
        kill(t1)
        return True
    return False


merged = 0
while merge_pass():
    merged += 1


# ---- 4. right-angle chamfers ------------------------------------------------------------------------------------
def shape(t, L):
    return t.GetEffectiveShape(L)


def neighbours(net, L, box):
    """Other nets' tracks, vias and pads on layer L touching box (grown by SAFE)."""
    g = iu(SAFE + 0.5)
    bx = pcbnew.BOX2I(V(box[0] - g, box[1] - g), pcbnew.VECTOR2L(int(box[2] - box[0] + 2 * g),
                                                                    int(box[3] - box[1] + 2 * g)))
    out = []
    for t in b.GetTracks():
        if t.GetNetCode() != net and t.IsOnLayer(L) and uid(t) not in DEAD and t.GetBoundingBox().Intersects(bx):
            out.append(t)
    for fp in b.GetFootprints():
        if not fp.GetBoundingBox().Intersects(bx):
            continue
        for p in fp.Pads():
            if p.GetNetCode() != net and p.IsOnLayer(L) and p.GetBoundingBox().Intersects(bx):
                out.append(p)
    return out


def gap(shapes, other, L):
    """The distance from shapes to other's copper on L, to 0.01 mm, capped at SAFE."""
    so = other.GetEffectiveShape(L)
    if not any(s.Collide(so, iu(SAFE)) for s in shapes):
        return SAFE
    lo, hi = 0.0, SAFE
    while hi - lo > 0.01:
        m = (lo + hi) / 2
        if any(s.Collide(so, iu(m)) for s in shapes):
            hi = m
        else:
            lo = m
    return lo


KEEPOUTS = [z for z in b.Zones() if z.GetIsRuleArea() and z.GetDoNotAllowTracks()]


def in_keepout(sh, L):
    for z in KEEPOUTS:
        if z.IsOnLayer(L) and z.Outline().Collide(sh):
            return z
    return None


def need_for(g):
    """What new copper must keep from an item the old copper kept g from: SAFE if it had that; else it may come a
    little closer, to no less than 0.25 mm (0.05 above the 0.2 mm base rule) and no more than 0.3 mm required;
    under 0.25 mm, no closer than before."""
    if g >= SAFE:
        return SAFE
    if g >= 0.25:
        return max(0.25, min(0.3, g - 0.03))
    return g


def ok_against(old, new, net, L, box):
    """new copper keeps from other nets' items what old kept (or SAFE), and enters no new track keepout."""
    near = neighbours(net, L, box)
    for o in near:
        need = need_for(gap(old, o, L))
        so = o.GetEffectiveShape(L)
        if any(sh.Collide(so, iu(need - 0.005)) for sh in new):
            return False
    ko_old = {id(z) for z in KEEPOUTS if z.IsOnLayer(L) and any(z.Outline().Collide(sh) for sh in old)}
    for sh in new:
        z = in_keepout(sh, L)
        if z is not None and id(z) not in ko_old:
            return False
    return True


def seg(net, L, w, a, c):
    n = pcbnew.PCB_TRACK(b)
    n.SetStart(V(*a)); n.SetEnd(V(*c)); n.SetWidth(w); n.SetLayer(L); n.SetNetCode(net)
    return n


def unit(d):
    l = math.hypot(*d)
    return (d[0] / l, d[1] / l), l


def staircase_pass():
    """One router-grid staircase (two or more pieces under STEP between two longer runs) replaced by the runs
    extended to meet: straight on where they turn 45 degrees, with one 45-degree chamfer where they turn 90."""
    J = joints()
    for s_in in tracks():
        if s_in.GetLength() < iu(STEP):
            continue
        for e_in in (0, 1):
            net, L, w = s_in.GetNetCode(), s_in.GetLayer(), s_in.GetWidth()
            p = pt(s_in.GetStart()) if e_in == 0 else pt(s_in.GetEnd())
            chain, cur, prev = [], p, s_in
            s_out = None
            while True:
                lst = J[(net, L, cur)]
                if not bare((net, L, cur), lst):
                    break
                (nt, ne), = [x for x in lst if uid(x[0]) != uid(prev)]
                if nt.GetWidth() != w:
                    break
                if nt.GetLength() >= iu(STEP):
                    s_out, e_out = nt, ne
                    break
                chain.append(nt)
                cur, prev = far(nt, ne), nt
                if len(chain) > 12:
                    break
            if s_out is None or len(chain) < 2 or uid(s_out) == uid(s_in):
                continue
            a0 = far(s_in, e_in)                       # S_in's far end
            z1 = far(s_out, e_out)                     # S_out's far end
            zin = p
            zout = pt(s_out.GetStart()) if e_out == 0 else pt(s_out.GetEnd())
            (din, _), (dout, _) = unit((zin[0] - a0[0], zin[1] - a0[1])), unit((z1[0] - zout[0], z1[1] - zout[1]))
            x = cross(a0, din, zout, dout)
            if x is None:
                continue
            turn = math.degrees(math.acos(max(-1.0, min(1.0, din[0] * dout[0] + din[1] * dout[1]))))
            la = (x[0] - a0[0]) * din[0] + (x[1] - a0[1]) * din[1]          # along S_in to the meeting point
            lb = (z1[0] - x[0]) * dout[0] + (z1[1] - x[1]) * dout[1]        # from it along S_out
            if la <= 0 or lb <= 0 or math.dist(x, zin) > iu(1.5) or math.dist(x, zout) > iu(1.5):
                continue
            old = [t.GetEffectiveShape(L) for t in [s_in, s_out] + chain]
            pts = [a0, zin, zout, z1] + [far(t, 0) for t in chain] + [far(t, 1) for t in chain]
            box = (min(q[0] for q in pts), min(q[1] for q in pts), max(q[0] for q in pts), max(q[1] for q in pts))
            cands = []
            if abs(turn - 45) < 0.5:
                cands.append((x, x))
            elif abs(turn - 90) < 0.5:
                for c in CHAMFER:
                    c = iu(c)
                    if c <= 0.45 * la and c <= 0.45 * lb:
                        cands.append(((x[0] - din[0] * c, x[1] - din[1] * c), (x[0] + dout[0] * c, x[1] + dout[1] * c)))
            for qa, qb in cands:
                new = [seg(net, L, w, a0, qa), seg(net, L, w, qb, z1)]
                if qa != qb:
                    new.append(seg(net, L, w, qa, qb))
                if not ok_against(old, [n.GetEffectiveShape(L) for n in new], net, L, box):
                    continue
                set_end(s_in, e_in, qa)
                set_end(s_out, e_out, qb)
                for t in chain:
                    kill(t)
                if qa != qb:
                    b.Add(new[2])
                return True
    return False


stairs = 0
while stairs < 500 and staircase_pass():
    stairs += 1


def vias():
    return [v for v in b.GetTracks() if v.Type() == pcbnew.PCB_VIA_T]


def via_snap_pass():
    """A via a hair off the line its tracks run on (a stub under TINY from it to the run): moved onto the point
    where the runs' lines on its layers cross (within 0.1 mm), the stubs dropped, every run ending at it again."""
    J = joints()
    n_moved = 0
    for v in vias():
        net, c = v.GetNetCode(), pt(v.GetPosition())
        in_pads = [p for L in CU for p in anc.get((net, L), ()) if p.Type() == pcbnew.PCB_PAD_T and p.HitTest(V(*c), 0)]
        ends, stubs, lines = [], [], []                    # (track, end) moved to the new spot; stubs; run lines
        skip = False
        for L in CU:
            for t, e in J.get((net, L, c), ()):
                o = far(t, e)
                if math.dist(o, c) < iu(TINY):
                    lst = J[(net, L, o)]
                    others = [a for a in anc.get((net, L), ()) if not (a.Type() == pcbnew.PCB_VIA_T and
                                                                       uid(a) == uid(v))]
                    if len(lst) != 2 or any(a.HitTest(V(*o), 0) for a in others):
                        skip = True
                        continue
                    (tn, en), = [x for x in lst if uid(x[0]) != uid(t)]
                    stubs.append(t)
                    ends.append((tn, en))
                    lines.append((o, far(tn, en)))
                else:
                    ends.append((t, e))
                    lines.append((c, o))
        if skip or not stubs:
            continue
        x = None
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                (a0, a1), (c0, c1) = lines[i], lines[j]
                x = cross(a0, (a1[0] - a0[0], a1[1] - a0[1]), c0, (c1[0] - c0[0], c1[1] - c0[1]))
                if x:
                    break
            if x:
                break
        if x is None:                                       # all runs parallel: they must share one line
            (a0, a1) = lines[0]
            (d, _) = unit((a1[0] - a0[0], a1[1] - a0[1]))
            k = (c[0] - a0[0]) * d[0] + (c[1] - a0[1]) * d[1]
            x = (a0[0] + d[0] * k, a0[1] + d[1] * k)
        x = (round(x[0]), round(x[1]))

        def online(l):
            (a0, a1) = l
            (d, _) = unit((a1[0] - a0[0], a1[1] - a0[1]))
            return abs((x[0] - a0[0]) * d[1] - (x[1] - a0[1]) * d[0]) < iu(0.0015)
        if math.dist(x, c) > iu(0.1) or not all(online(l) for l in lines):
            continue
        if any(not p.HitTest(V(*x), 0) for p in in_pads):  # a via in a pad stays in it
            continue
        # clearance: the via and every moved run, layer by layer
        good = True
        for L in CU:
            old = [v.GetEffectiveShape(L)] + [t.GetEffectiveShape(L) for t, e in ends if t.GetLayer() == L] + \
                  [t.GetEffectiveShape(L) for t in stubs if t.GetLayer() == L]
            v.SetPosition(V(*x))
            new = [v.GetEffectiveShape(L)]
            v.SetPosition(V(*c))
            for t, e in ends:
                if t.GetLayer() == L:
                    a = far(t, e)
                    new.append(seg(net, L, t.GetWidth(), a, x).GetEffectiveShape(L))
            box = (min(c[0], x[0]), min(c[1], x[1]), max(c[0], x[0]), max(c[1], x[1]))
            if not ok_against(old, new, net, L, box):
                good = False
                break
        if not good:
            continue
        v.SetPosition(V(*x))
        for t, e in ends:
            set_end(t, e, x)
        for t in stubs:
            kill(t)
        n_moved += 1
    return n_moved


def via_onto(v, stub, o, J):
    """The via moved onto its stub's far end o; every other run at the via slid across by the same hair, where its
    far end lies inside a pad and stays inside it."""
    net, c = v.GetNetCode(), pt(v.GetPosition())
    off = (o[0] - c[0], o[1] - c[1])
    runs = []
    for L in CU:
        for t, e in J.get((net, L, c), ()):
            if uid(t) == uid(stub):
                continue
            q = far(t, e)
            q2 = (q[0] + off[0], q[1] + off[1])
            pads = [p for p in anc.get((net, L), ()) if p.Type() == pcbnew.PCB_PAD_T and p.HitTest(V(*q), 0)]
            if not pads or not all(p.HitTest(V(*q2), 0) for p in pads):    # (the pad joins what else ends there)
                return False
            runs.append((t, e, q2, L))
    for p in [p for L in CU for p in anc.get((net, L), ()) if p.Type() == pcbnew.PCB_PAD_T and p.HitTest(V(*c), 0)]:
        if not p.HitTest(V(*o), 0):
            return False
    for L in CU:
        old = [v.GetEffectiveShape(L)] + [t.GetEffectiveShape(L) for t, e, q2, l in runs if l == L]
        v.SetPosition(V(*o))
        new = [v.GetEffectiveShape(L)] + [seg(net, L, t.GetWidth(), o, q2).GetEffectiveShape(L)
                                          for t, e, q2, l in runs if l == L]
        v.SetPosition(V(*c))
        box = (min(c[0], o[0]) - iu(2), min(c[1], o[1]) - iu(2), max(c[0], o[0]) + iu(2), max(c[1], o[1]) + iu(2))
        if not ok_against(old, new, net, L, box):
            return False
    v.SetPosition(V(*o))
    for t, e, q2, L in runs:
        set_end(t, e, o)
        set_end(t, 1 - e, q2)
    kill(stub)
    return True


def run_across(v, stub, o, tn, en, J):
    """The run tn (from the stub's far end o onwards) moved sideways onto the via: it then starts at the via, and its
    far end slides along the next segment's line to meet it there (the next segment shortened or lengthened)."""
    net, c, L = v.GetNetCode(), pt(v.GetPosition()), tn.GetLayer()
    q = far(tn, en)
    lst = J[(net, L, q)]
    if not bare((net, L, q), lst):
        return False
    (nn, ne), = [x for x in lst if uid(x[0]) != uid(tn)]
    r = far(nn, ne)
    (d, _) = unit((q[0] - o[0], q[1] - o[1]))
    x = cross(c, d, r, (q[0] - r[0], q[1] - r[1]))
    if x is None or math.dist(x, q) > iu(0.15):
        return False
    x = (round(x[0]), round(x[1]))
    if (x[0] - r[0]) * (q[0] - r[0]) + (x[1] - r[1]) * (q[1] - r[1]) <= 0:
        return False
    old = [stub.GetEffectiveShape(L), tn.GetEffectiveShape(L), nn.GetEffectiveShape(L)]
    new = [seg(net, L, tn.GetWidth(), c, x).GetEffectiveShape(L), seg(net, L, nn.GetWidth(), x, r).GetEffectiveShape(L)]
    pts = [c, x, r, q, o]
    box = (min(p[0] for p in pts), min(p[1] for p in pts), max(p[0] for p in pts), max(p[1] for p in pts))
    if not ok_against(old, new, net, L, box):
        return False
    set_end(tn, en, c)
    set_end(tn, 1 - en, x)
    set_end(nn, ne, x)
    kill(stub)
    return True


def stub_shift_pass():
    """A via whose runs are parallel but a hair apart (a stub under TINY between them): the straight run on the
    stub's side is slid across onto the via, where its other end lies inside a pad and stays inside it."""
    J = joints()
    n_shift = 0
    for v in vias():
        net, c = v.GetNetCode(), pt(v.GetPosition())
        for L in CU:
            for t, e in J.get((net, L, c), ()):
                o = far(t, e)
                if not (0 < math.dist(o, c) < iu(TINY)):
                    continue
                lst = J[(net, L, o)]
                if len(lst) != 2:                           # a junction: only the via can move onto it
                    if via_onto(v, t, o, J):
                        return 1
                    continue
                (tn, en), = [x for x in lst if uid(x[0]) != uid(t)]
                q = far(tn, en)
                off = (c[0] - o[0], c[1] - o[1])
                q2 = (q[0] + off[0], q[1] + off[1])
                pads = [p for p in anc.get((net, L), ()) if p.Type() == pcbnew.PCB_PAD_T and p.HitTest(V(*q), 0)]
                if not pads or not all(p.HitTest(V(*q2), 0) for p in pads):
                    if via_onto(v, t, o, J) or run_across(v, t, o, tn, en, J):
                        return 1
                    continue
                new = seg(net, L, tn.GetWidth(), c, q2)
                box = (min(c[0], q2[0]), min(c[1], q2[1]), max(c[0], q2[0]), max(c[1], q2[1]))
                if not ok_against([tn.GetEffectiveShape(L), t.GetEffectiveShape(L)], [new.GetEffectiveShape(L)],
                                  net, L, box):
                    if via_onto(v, t, o, J):
                        return 1
                    continue
                set_end(tn, en, c)
                set_end(tn, 1 - en, q2)
                kill(t)
                n_shift += 1
                return n_shift
    return n_shift


def hairpin_pass():
    """Two runs folding back at a bare joint (under 45 degrees): replaced by one straight link between their far
    ends, if that keeps the clearances."""
    J = joints()
    for key, lst in J.items():
        if not bare(key, lst):
            continue
        (t0, e0), (t1, e1) = lst
        if uid(t0) == uid(t1) or t0.GetWidth() != t1.GetWidth():
            continue
        ang = angle_at(t0, e0, t1, e1, key[2])[0]
        if ang is None or ang >= 45:
            continue
        net, L, p = key
        a, c = far(t0, e0), far(t1, e1)
        new = seg(net, L, t0.GetWidth(), a, c)
        box = (min(a[0], c[0], p[0]), min(a[1], c[1], p[1]), max(a[0], c[0], p[0]), max(a[1], c[1], p[1]))
        if not ok_against([t0.GetEffectiveShape(L), t1.GetEffectiveShape(L)], [new.GetEffectiveShape(L)], net, L, box):
            continue
        set_end(t0, e0, c)
        kill(t1)
        return True
    return False


snapped = via_snap_pass()
shifted = 0
while shifted < 100 and stub_shift_pass():
    shifted += 1
hairpins = 0
while hairpins < 50 and hairpin_pass():
    hairpins += 1
while merge_pass():
    merged += 1


# ---- GND fan-out vias out of the power nets' current (2026-11-24) ----------------------------------------------
# A GND via on a single fan-out track from a GND pad, standing in a power net's current corridor
# (copper_sim.corridors), moves to the nearest spot round its pad outside every corridor: 0.25 mm from other nets'
# copper on every layer (its track 0.22 mm), no new 42 / 54 V pour pierced, no other pour's narrow neck cut (the
# GND stage's rule), clear of via keep-outs and holes. Done here, after the routing, because moving them during it
# reshuffled the negotiated routes.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import copper_sim  # noqa: E402

CORR = copper_sim.Corridors()
GNDC = b.FindNet("GND").GetNetCode() if b.FindNet("GND") else None
VIA_KEEPOUTS = [z for z in b.Zones() if z.GetIsRuleArea() and z.GetDoNotAllowVias()]
_fills = {}


def _fill(z, L):
    k = (uid(z), L)
    if k not in _fills:
        f = pcbnew.SHAPE_POLY_SET(z.GetFilledPolysList(L))
        w = pcbnew.SHAPE_POLY_SET(f)
        w.Deflate(iu(1.6), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, iu(0.02))
        _fills[k] = (f, w)
    return _fills[k]


def _hv(z):
    n = b.FindNet(z.GetNetname())
    return n is not None and n.GetNetClassName() in ("HV_Bus", "HV_Power")


def _gnd_spot_ok(v, q, pad, L, w, hv_old):
    x, y = mm(q[0]) - 50.0, mm(q[1]) - 50.0
    if not (1.2 <= x <= 148.8 and 1.2 <= y <= 148.8) or CORR.blocked("GND", x, y):
        return False
    P = V(*q)
    for z in VIA_KEEPOUTS:
        if z.Outline().Contains(P):
            return False
    r = mm(v.GetWidth(pcbnew.F_Cu)) / 2
    for z in b.Zones():
        if z.GetIsRuleArea() or z.GetNetCode() == GNDC:
            continue
        if _hv(z) and z.Outline().Contains(P) and uid(z) not in hv_old:
            return False
        for Lz in CU:
            if z.IsOnLayer(Lz):
                f, wide = _fill(z, Lz)
                if f.OutlineCount() and f.Collide(P, iu(r + 0.55)) and not wide.Contains(P):
                    return False
    for f in b.GetFootprints():                           # holes
        for p in f.Pads():
            if p.GetDrillSizeX() > 0 and math.dist(pt(p.GetPosition()), q) < iu(mm(p.GetDrillSizeX()) / 2 + 0.2 + 0.45):
                return False
    for o in vias():
        if uid(o) != uid(v) and math.dist(pt(o.GetPosition()), q) < iu(r + mm(o.GetWidth(pcbnew.F_Cu)) / 2 + 0.25):
            return False
    old = pt(v.GetPosition())
    v.SetPosition(P)
    try:
        box = (q[0] - iu(1.5), q[1] - iu(1.5), q[0] + iu(1.5), q[1] + iu(1.5))
        for Lv in CU:
            sh = v.GetEffectiveShape(Lv)
            if any(sh.Collide(o.GetEffectiveShape(Lv), iu(0.25)) for o in neighbours(GNDC, Lv, box)):
                return False
        c = pt(pad.GetPosition())
        tr = seg(GNDC, L, w, c, q).GetEffectiveShape(L)
        tb = (min(c[0], q[0]), min(c[1], q[1]), max(c[0], q[0]), max(c[1], q[1]))
        if any(tr.Collide(o.GetEffectiveShape(L), iu(0.22)) for o in neighbours(GNDC, L, tb)):
            return False
        if in_keepout(tr, L) is not None:
            return False
    finally:
        v.SetPosition(V(*old))
    return True


def gnd_relocate():
    if not CORR.masks or GNDC is None:
        return 0, []
    J = joints()
    moved, stuck = 0, []
    for v in vias():
        if v.GetNetCode() != GNDC:
            continue
        c = pt(v.GetPosition())
        x, y = mm(c[0]) - 50.0, mm(c[1]) - 50.0
        if not CORR.blocked("GND", x, y):
            continue
        att = [(t, e, L) for L in CU for t, e in J.get((GNDC, L, c), ())]
        if len(att) != 1:
            continue                                      # a stitching via, or more than a fan-out: leave it
        t, e, L = att[0]
        o = far(t, e)
        pad = next((p for p in anc.get((GNDC, L), ()) if p.Type() == pcbnew.PCB_PAD_T and p.HitTest(V(*o), 0)), None)
        if pad is None or len(J.get((GNDC, L, o), ())) != 1:
            continue
        P = pt(pad.GetPosition())
        hv_old = {uid(z) for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetCode() != GNDC and _hv(z)
                  and z.Outline().Contains(V(*c))}
        bb = pad.GetBoundingBox()
        r0 = max(bb.GetWidth(), bb.GetHeight()) / 2 + iu(mm(v.GetWidth(pcbnew.F_Cu)) / 2 + 0.25)
        dirs = [k * math.pi / 4 for k in range(8)] + [math.pi / 16 + k * math.pi / 8 for k in range(16)]
        found = None
        d = r0
        while d <= iu(3.5) and found is None:
            for a in dirs:
                q = (P[0] + d * math.cos(a), P[1] + d * math.sin(a))
                q = (round(q[0] / 1000) * 1000, round(q[1] / 1000) * 1000)   # 1 um grid
                if _gnd_spot_ok(v, q, pad, L, t.GetWidth(), hv_old):
                    found = q
                    break
            d += iu(0.1)
        if found is None:
            stuck.append((round(x, 2), round(y, 2)))
            continue
        v.SetPosition(V(*found))
        set_end(t, e, found)                              # (e is the track's end at the via)
        set_end(t, 1 - e, P)
        moved += 1
    return moved, stuck


gnd_moved, gnd_stuck = gnd_relocate()


def chamfer_pass():
    global chamfered
    skipped = []
    J = joints()
    done = set()
    for key, lst in list(J.items()):
        if not bare(key, lst):
            continue
        (t0, e0), (t1, e1) = lst
        if uid(t0) == uid(t1) or uid(t0) in done or uid(t1) in done:
            continue
        ang, du, dv, lu, lv = angle_at(t0, e0, t1, e1, key[2])
        if ang is None or abs(ang - 90.0) > 0.5:
            continue
        net, L, p = key
        w = max(t0.GetWidth(), t1.GetWidth())
        old = [shape(t0, L), shape(t1, L)]
        xs = [p[0], far(t0, e0)[0], far(t1, e1)[0]]
        ys = [p[1], far(t0, e0)[1], far(t1, e1)[1]]
        near = neighbours(net, L, (min(xs), min(ys), max(xs), max(ys)))
        need = {id(o): need_for(gap(old, o, L)) for o in near}
        ko_old = {id(z) for z in KEEPOUTS if z.IsOnLayer(L) and any(z.Outline().Collide(s) for s in old)}
        ok = None
        for c in CHAMFER:
            c = iu(c)
            if c > 0.45 * lu or c > 0.45 * lv:
                continue
            qa = (p[0] + du[0] / lu * c, p[1] + du[1] / lu * c)
            qb = (p[0] + dv[0] / lv * c, p[1] + dv[1] / lv * c)
            n = pcbnew.PCB_TRACK(b)
            n.SetStart(V(*qa)); n.SetEnd(V(*qb)); n.SetWidth(w); n.SetLayer(L); n.SetNetCode(net)
            sh = n.GetEffectiveShape(L)
            if any(sh.Collide(o.GetEffectiveShape(L), iu(need[id(o)] - 0.005)) for o in near):
                continue
            z = in_keepout(sh, L)
            if z is not None and id(z) not in ko_old:
                continue
            ok = (n, qa, qb)
            break
        if ok is None:
            skipped.append((b.FindNet(net).GetNetname().split("/")[-1], round(mm(p[0]) - 50, 2),
                            round(mm(p[1]) - 50, 2), pcbnew.LayerName(L)))
            continue
        n, qa, qb = ok
        set_end(t0, e0, qa)
        set_end(t1, e1, qb)
        b.Add(n)
        done.update((uid(t0), uid(t1)))
        chamfered += 1
    return skipped


chamfered = 0
for _ in range(4):                       # a leg shortened at one corner may have another corner at its far end
    before = chamfered
    skipped = chamfer_pass()
    if chamfered == before:
        break

for t in [t for t in b.GetTracks() if uid(t) in DEAD]:
    b.Remove(t)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
print(f"tidy: {welded} track ends welded, {dup} duplicate and {zero} zero-length segments removed, {kinks} kinks absorbed, {merged} collinear "
      f"joints merged, {stairs} staircases straightened, {snapped} vias put back on their runs, {shifted} runs slid onto their vias, {hairpins} hairpins straightened, {chamfered} right angles chamfered, {gnd_moved} GND fan-out vias moved out of the power current"
      + (f" ({len(gnd_stuck)} found no spot: {gnd_stuck})" if gnd_stuck else "")
      + (f"; no room for a chamfer at {len(skipped)}: {skipped}" if skipped else ""))
b.Save(BOARD)
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
