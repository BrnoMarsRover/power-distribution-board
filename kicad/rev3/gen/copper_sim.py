#!/usr/bin/env python3
"""DC current flow in the board's power copper (2026-11-24): a check, not part of the rebuild.

    python3 kicad/rev3/gen/copper_sim.py [OUT_DIR]
    INNER_UM=35 NETS=GND,OUT42D python3 kicad/rev3/gen/copper_sim.py /tmp/x     (what-if: 1 oz inner, two nets)

For each power net it rasterises that net's copper on every layer (filled pours, tracks, pads; 0.15 mm cells,
0.25 mm for GND), joins the layers through the net's vias and through-hole pins (plated barrels), feeds the
design currents in at the parts the current comes from and takes them out where it goes, and solves Kirchhoff's
equations for the copper. It reports per net:
  - the voltage drop from the source to each sink and the copper's I^2R loss;
  - the peak current per mm of copper width on each layer, away from the pads (the hot spots), and where;
  - the same with every other net's via and through-hole moved out of the net's pours ("no foreign vias"), which
    is what those holes cost.
A heat map per net and layer goes to OUT_DIR (default kicad/rev3/gen/out/copper_sim).

Copper: 70 um outer, 15.2 um inner (board_rules.STACKUP), resistivity at 20 C; via barrels 20 um plating; the big
connectors' pins short their layers. Rule of thumb for the maps: on 2 oz outer copper, 4 A per mm of width runs
about 10 C above ambient as a lone 1 mm trace (IPC-2221), less in a wide pour that spreads heat.
"""
import math
import os
import sys
from collections import defaultdict

import numpy as np
import pcbnew
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from PIL import Image, ImageDraw

BOARD = "kicad/rev3/PowerBoard_rev3.kicad_pcb"
OUT = sys.argv[1] if len(sys.argv) > 1 else "kicad/rev3/gen/out/copper_sim"
RHO = 1.72e-8                                   # ohm m, copper at 20 C
X0, Y0, SIZE = 50.0, 50.0, 150.0                # board position on the page, mm
LAYERS = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
LNAME = {pcbnew.F_Cu: "L1", pcbnew.In1_Cu: "L2", pcbnew.In2_Cu: "L3", pcbnew.B_Cu: "L4"}
INNER = float(os.environ.get("INNER_UM", "15.2")) * 1e-6      # what-if: INNER_UM=35 (1 oz) or 70 (2 oz)
THICK = {pcbnew.F_Cu: 70e-6, pcbnew.In1_Cu: INNER, pcbnew.In2_Cu: INNER, pcbnew.B_Cu: 70e-6}
ZPOS = {pcbnew.F_Cu: 0.0, pcbnew.In1_Cu: 0.2804, pcbnew.In2_Cu: 1.3606, pcbnew.B_Cu: 1.5862}   # mm
PLATE = 20e-6
mm, iu = pcbnew.ToMM, pcbnew.FromMM

# ---- scenarios: (net, cell mm, sources {pad: A}, sinks {pad: A}) -- design currents (design.md 4.x, 8.0)
I42D, I42C, I24, I12, I5 = 30.0, 22.0, 11.1, 11.1, 16.6
BOOST_IN = 2.6
SCEN = [
    ("VBAT", 0.15, {"J1.1": I42D + I42C}, {"R1.1": I42D + I42C}),
    ("BUS42", 0.15, {"R1.4": I42D + I42C}, {"R5.1": I42D, "R23.1": I42C}),
    ("DRN@42D", 0.15, {"R5.4": I42D}, {"Q2.2": I42D / 2, "Q3.2": I42D / 2}),
    ("DRN@42C", 0.15, {"R23.4": I42C}, {"Q6.2": I42C / 2, "Q7.2": I42C / 2}),
    ("OUT42D", 0.15, {"Q2.3": I42D / 2, "Q3.3": I42D / 2}, {"J8.1": I42D / 3, "J9.1": I42D / 3, "J10.1": I42D / 3}),
    ("OUT42C", 0.15, {"Q6.3": I42C / 2, "Q7.3": I42C / 2},
     {"J11.1": (I42C - BOOST_IN) / 2, "J12.1": (I42C - BOOST_IN) / 2, "R121.1": BOOST_IN}),
    ("VIN24", 0.15, {"J5.2": I24}, {"R41.1": I24}),
    ("DRN@24", 0.15, {"R41.4": I24}, {"Q10.2": I24}),
    ("OUT24", 0.15, {"Q10.3": I24}, {f"F{k}.1": I24 / 6 for k in range(1, 7)}),
    ("VIN12", 0.15, {"J6.2": I12}, {"R59.1": I12}),
    ("DRN@12", 0.15, {"R59.4": I12}, {"Q13.2": I12}),
    ("OUT12", 0.15, {"Q13.3": I12}, {f"F{k}.1": I12 / 6 for k in range(7, 13)}),
    ("VIN5", 0.15, {"J7.2": I5}, {"R77.1": I5}),
    ("DRN@5", 0.15, {"R77.4": I5}, {"Q16.2": I5}),
    ("OUT5", 0.15, {"Q16.3": I5}, {f"F{k}.1": I5 / 6 for k in range(13, 19)}),
    ("GND", float(os.environ.get("GND_H", "0.25")),
     dict({"J8.2": I42D / 3, "J9.2": I42D / 3, "J10.2": I42D / 3, "J11.2": I42C / 2, "J12.2": I42C / 2},
          **{f"J{k}.1": I24 / 6 for k in range(14, 20)}, **{f"J{k}.1": I12 / 6 for k in range(20, 26)},
          **{f"J{k}.1": I5 / 6 for k in range(26, 32)}),
     {"J1.2": I42D + I42C, "J5.1": I24, "J6.1": I12, "J7.1": I5}),
]

b = pcbnew.LoadBoard(BOARD)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
FP = {f.GetReference(): f for f in b.GetFootprints()}


def pad(name):
    ref, num = name.split(".")
    return next(p for p in FP[ref].Pads() if p.GetNumber() == num)


def net_of(scen_net, terms):
    """'DRN@42D' -> the DRN net the named pads sit on."""
    if "@" not in scen_net:
        return next(n for n in (pad(t).GetNetname() for t in terms) if n.split("/")[-1] == scen_net or n == scen_net)
    return pad(next(iter(terms))).GetNetname()


class Grid:
    def __init__(self, h):
        self.h, self.n = h, int(round(SIZE / h))

    def img(self):
        return Image.new("1", (self.n, self.n), 0)

    def pts(self, chain):
        return [((mm(chain.CPoint(i).x) - X0) / self.h, (mm(chain.CPoint(i).y) - Y0) / self.h)
                for i in range(chain.PointCount())]

    def polyset(self, ps, im=None, val=1):
        im = im or self.img()
        d = ImageDraw.Draw(im)
        for i in range(ps.OutlineCount()):
            p = self.pts(ps.Outline(i))
            if len(p) > 2:
                d.polygon(p, fill=val)
            for j in range(ps.HoleCount(i)):
                q = self.pts(ps.Hole(i, j))
                if len(q) > 2:
                    d.polygon(q, fill=0)
        return im

    def cell(self, x_iu, y_iu):
        return (int((mm(x_iu) - X0) / self.h), int((mm(y_iu) - Y0) / self.h))


def item_poly(item, L):
    ps = pcbnew.SHAPE_POLY_SET()
    item.TransformShapeToPolygon(ps, L, 0, iu(0.005), pcbnew.ERROR_INSIDE)
    return ps


def masks_for(net, g):
    """Copper of net on each layer, as boolean arrays [y, x]."""
    out = {}
    for L in LAYERS:
        m = np.zeros((g.n, g.n), bool)
        for z in b.Zones():
            if z.GetIsRuleArea() or z.GetNetname() != net or not z.IsOnLayer(L):
                continue
            m |= np.array(g.polyset(z.GetFilledPolysList(L)), bool)
        im = g.img()
        for t in b.GetTracks():
            if t.GetNetname() == net and t.IsOnLayer(L):
                g.polyset(item_poly(t, L), im)
        for f in b.GetFootprints():
            for p in f.Pads():
                if p.GetNetname() == net and p.IsOnLayer(L):
                    g.polyset(item_poly(p, L), im)
        out[L] = m | np.array(im, bool)
    return out


def foreign_fill(net, g, masks):
    """The net's copper with every other net's via / through-hole that sits inside its pours moved away: their
    clearance holes filled, where no other net's track or pad is."""
    out = {}
    for L in LAYERS:
        outl = [z for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetname() == net and z.IsOnLayer(L)]
        if not outl:
            out[L] = masks[L]
            continue
        holes = g.img()
        d = ImageDraw.Draw(holes)
        for t in b.GetTracks():
            if t.Type() == pcbnew.PCB_VIA_T and t.GetNetname() != net:
                p = t.GetPosition()
                if any(z.Outline().Contains(p) for z in outl):
                    r = (mm(t.GetWidth(L)) / 2 + 0.55) / g.h
                    cx, cy = (mm(p.x) - X0) / g.h, (mm(p.y) - Y0) / g.h
                    d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=1)
        for f in b.GetFootprints():
            for p in f.Pads():
                if p.GetNetname() != net and p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH and p.GetDrillSizeX() < iu(1.2):
                    c = p.GetPosition()
                    if any(z.Outline().Contains(c) for z in outl):
                        r = (mm(p.GetSize(L).x) / 2 + 0.55) / g.h
                        cx, cy = (mm(c.x) - X0) / g.h, (mm(c.y) - Y0) / g.h
                        d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=1)
        blocked = g.img()
        for t in b.GetTracks():
            if t.Type() == pcbnew.PCB_TRACE_T and t.GetNetname() != net and t.IsOnLayer(L):
                g.polyset(item_poly(t, L), blocked)
        for f in b.GetFootprints():
            for p in f.Pads():
                if p.GetNetname() != net and p.IsOnLayer(L) and not (p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH and
                                                                      p.GetDrillSizeX() < iu(1.2)):
                    g.polyset(item_poly(p, L), blocked)
        inside = np.zeros((g.n, g.n), bool)
        for z in outl:
            inside |= np.array(g.polyset(z.Outline()), bool)
        out[L] = masks[L] | (np.array(holes, bool) & inside & ~np.array(blocked, bool))
    return out


def solve(net, g, masks, src, snk):
    n = g.n
    idx = {L: np.full((n, n), -1, np.int64) for L in LAYERS}
    k = 0
    for L in LAYERS:
        c = masks[L].sum()
        idx[L][masks[L]] = np.arange(k, k + c)
        k += c
    rows, cols, vals = [], [], []

    def link(a, c, G):
        rows.extend([a, c]); cols.extend([c, a]); vals.extend([-G, -G])
        diag[a] += G; diag[c] += G

    nterm = len(src) + len(snk)
    diag = np.zeros(k + nterm)
    for L in LAYERS:
        G = THICK[L] / RHO
        I = idx[L]
        for a, c in ((I[:, :-1], I[:, 1:]), (I[:-1, :], I[1:, :])):
            ok = (a >= 0) & (c >= 0)
            aa, cc = a[ok], c[ok]
            rows.extend([aa, cc]); cols.extend([cc, aa]); vals.extend([np.full(aa.size, -G)] * 2)
            np.add.at(diag, aa, G); np.add.at(diag, cc, G)
    rows = [np.atleast_1d(np.asarray(r)) for r in rows]
    cols = [np.atleast_1d(np.asarray(c)) for c in cols]
    vals = [np.atleast_1d(np.asarray(v, float)) for v in vals]
    extra_r, extra_c, extra_v = [], [], []

    def link2(a, c, G):
        extra_r.extend([a, c]); extra_c.extend([c, a]); extra_v.extend([-G, -G])
        diag[a] += G; diag[c] += G

    def near_cell(L, cx, cy, r):
        I = idx[L]
        best = None
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                x, y = cx + dx, cy + dy
                if 0 <= x < n and 0 <= y < n and I[y, x] >= 0:
                    d2 = dx * dx + dy * dy
                    if best is None or d2 < best[0]:
                        best = (d2, I[y, x])
        return None if best is None else best[1]

    # barrels: the net's vias and through-hole pads
    barrels = []
    for t in b.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T and t.GetNetname() == net:
            barrels.append((t.GetPosition(), mm(t.GetDrillValue()), mm(t.GetWidth(pcbnew.F_Cu)) / 2, False))
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() == net and p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                barrels.append((p.GetPosition(), mm(p.GetDrillSizeX()), mm(p.GetSize(pcbnew.F_Cu).x) / 2,
                                f.GetReference().startswith("J")))
    for pos, drill, rad, pin in barrels:
        cx, cy = g.cell(pos.x, pos.y)
        r = max(1, int(rad / g.h))
        st = [(L, near_cell(L, cx, cy, r)) for L in LAYERS]
        st = [(L, c) for L, c in st if c is not None]
        for (La, a), (Lb, c) in zip(st, st[1:]):
            dz = (ZPOS[Lb] - ZPOS[La]) * 1e-3
            G = 1e4 if pin else math.pi * drill * 1e-3 * PLATE / (RHO * dz)
            link2(a, c, G)
    # terminals: a node joined to every cell of the pad
    terms = list(src.items()) + [(t, -i) for t, i in snk.items()]
    rhs = np.zeros(k + nterm)
    tcells = np.zeros((n, n), bool)
    for j, (t, cur) in enumerate(terms):
        p = pad(t)
        node = k + j
        rhs[node] = cur
        for L in LAYERS:
            if not p.IsOnLayer(L):
                continue
            pm = np.array(g.polyset(item_poly(p, L)), bool)
            cells = idx[L][pm & masks[L]]
            tcells |= pm
            Gc = 20 * THICK[L] / RHO
            for c in cells:
                link2(node, c, Gc)
    A = sp.coo_matrix((np.concatenate(vals + [np.asarray(extra_v, float)]),
                       (np.concatenate(rows + [np.asarray(extra_r, np.int64)]),
                        np.concatenate(cols + [np.asarray(extra_c, np.int64)]))), shape=(k + nterm, k + nterm)).tocsr()
    A = A + sp.diags(diag)
    # only the copper connected to the terminals (islands make the system singular)
    from scipy.sparse.csgraph import connected_components
    _, comp = connected_components(A, directed=False)
    tcomp = comp[k:k + nterm]
    if len(set(tcomp)) > 1:
        groups = defaultdict(list)
        for (t, cur), c in zip(terms, tcomp):
            groups[c].append(t)
        print("   ! terminals not joined by the copper:", list(groups.values()), flush=True)
    live = np.isin(comp, tcomp)
    ref = k + len(src)                     # the first sink is 0 V
    keep = live.copy()
    keep[ref] = False
    Ak = A[keep][:, keep]
    if Ak.shape[0] < 700000:
        v = spla.spsolve(Ak.tocsc(), rhs[keep])
    else:
        M = sp.diags(1.0 / Ak.diagonal())
        v, info = spla.cg(Ak, rhs[keep], M=M, tol=1e-10, maxiter=60000)
    V = np.zeros(k + nterm)
    V[keep] = v
    # results
    loss = 0.0
    jmap = {}
    for L in LAYERS:
        G = THICK[L] / RHO
        I = idx[L]
        Vg = np.where(I >= 0, V[np.maximum(I, 0)], np.nan)
        ix = np.zeros((n, n)); iy = np.zeros((n, n))
        fx = G * (Vg[:, :-1] - Vg[:, 1:]); fx = np.nan_to_num(fx)
        fy = G * (Vg[:-1, :] - Vg[1:, :]); fy = np.nan_to_num(fy)
        loss += float((fx ** 2).sum() / G + (fy ** 2).sum() / G)
        ix[:, :-1] += fx / 2; ix[:, 1:] += fx / 2
        iy[:-1, :] += fy / 2; iy[1:, :] += fy / 2
        jmap[L] = np.where(I >= 0, np.hypot(ix, iy) / g.h, np.nan)      # A per mm of width
    tv = {t: V[k + j] for j, (t, _) in enumerate(terms)}
    return dict(loss=loss, tv=tv, jmap=jmap, tcells=tcells, ncells=k)


def peaks(g, jmap, tcells, L, top=4):
    J = np.nan_to_num(jmap[L]).copy()
    grow = int(round(0.6 / g.h))
    t = tcells.copy()
    for _ in range(grow):
        t[1:, :] |= t[:-1, :]; t[:-1, :] |= t[1:, :]; t[:, 1:] |= t[:, :-1]; t[:, :-1] |= t[:, 1:]
    J[t] = 0
    out = []
    for _ in range(top):
        y, x = np.unravel_index(np.argmax(J), J.shape)
        if J[y, x] <= 0:
            break
        out.append((float(J[y, x]), x * g.h, y * g.h))
        r = int(3 / g.h)
        J[max(0, y - r):y + r, max(0, x - r):x + r] = 0
    return out


def heatmap(g, jmap, masks, name, vmax):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    used = [L for L in LAYERS if masks[L].any()]
    ys, xs = np.nonzero(np.any([masks[L] for L in used], axis=0))
    pad = int(2 / g.h)
    x0, x1 = max(0, xs.min() - pad), min(g.n, xs.max() + pad)
    y0, y1 = max(0, ys.min() - pad), min(g.n, ys.max() + pad)
    scale = float(os.environ.get("FIGSCALE", "1"))
    fig, axs = plt.subplots(1, len(used), figsize=(4.2 * scale * len(used) * max(0.6, (x1 - x0) / (y1 - y0)),
                                                   5.2 * scale), squeeze=False)
    for ax, L in zip(axs[0], used):
        J = jmap[L][y0:y1, x0:x1]
        im = ax.imshow(J, cmap="inferno", vmin=0, vmax=vmax, extent=(x0 * g.h, x1 * g.h, y1 * g.h, y0 * g.h),
                       interpolation="nearest")
        ax.set_title(f"{name} {LNAME[L]}", fontsize=9)
        ax.set_facecolor("#d8d8d8")
        ax.tick_params(labelsize=7)
    fig.colorbar(im, ax=axs[0].tolist(), shrink=0.8, label="A per mm of width")
    os.makedirs(OUT, exist_ok=True)
    fig.savefig(f"{OUT}/{name.replace('@', '_')}.png", dpi=110, bbox_inches="tight")
    plt.close(fig)


rows = []
ONLY = os.environ.get("NETS")                     # what-if: NETS=GND,OUT42D
for name, h, src, snk in SCEN:
    if ONLY and name not in ONLY.split(","):
        continue
    g = Grid(h)
    net = net_of(name, list(src) + list(snk))
    masks = masks_for(net, g)
    r = solve(net, g, masks, src, snk)
    r2 = r if os.environ.get("SKIP_ALT") else solve(net, g, foreign_fill(net, g, masks), src, snk)
    vsrc = max(r["tv"][t] for t in src)
    drops = {t: (vsrc - r["tv"][t]) * 1e3 for t in snk}
    drops2 = {t: (max(r2["tv"][s] for s in src) - r2["tv"][t]) * 1e3 for t in snk}
    pk = {LNAME[L]: peaks(g, r["jmap"], r["tcells"], L) for L in LAYERS if masks[L].any()}
    pk2 = {LNAME[L]: peaks(g, r2["jmap"], r2["tcells"], L, 1) for L in LAYERS if masks[L].any()}
    nvia_own = sum(1 for t in b.GetTracks() if t.Type() == pcbnew.PCB_VIA_T and t.GetNetname() == net)
    heatmap(g, r["jmap"], masks, name, vmax=float(os.environ.get("VMAX", "8")))
    rows.append((name, sum(src.values()), max(drops.values()), max(drops2.values()), r["loss"], r2["loss"], pk, pk2,
                 nvia_own))
    print(f"{name:9s} {sum(src.values()):5.1f} A  drop max {max(drops.values()):6.2f} mV "
          f"(no foreign vias {max(drops2.values()):6.2f})  loss {r['loss'] * 1e3:7.1f} mW "
          f"({r2['loss'] * 1e3:7.1f})  own vias {nvia_own:3d}", flush=True)
    for Ln, lst in pk.items():
        s = ", ".join(f"{j:.1f} A/mm @({x:.1f},{y:.1f})" for j, x, y in lst[:3])
        alt = pk2[Ln][0][0] if pk2[Ln] else 0
        print(f"      {Ln}: {s}   [no foreign vias: peak {alt:.1f}]", flush=True)
sys.stdout.flush()
os._exit(0)
