#!/usr/bin/env python3
"""DC current flow in the board's power copper (2026-11-24): a check, not part of the rebuild.

    python3 kicad/rev3/gen/copper_sim.py [OUT_DIR]
    INNER_UM=35 NETS=GND,OUT42D python3 kicad/rev3/gen/copper_sim.py /tmp/x     (what-if: 1 oz inner, two nets)

For each power net it rasterises that net's copper on every layer (filled pours, tracks, pads; 0.15 mm cells,
0.4 mm for GND), joins the layers through the net's vias and through-hole pins (plated barrels), feeds the
design currents in at the parts the current comes from and takes them out where it goes, and solves Kirchhoff's
equations for the copper. It reports per net:
  - the voltage drop from the source to each sink and the copper's I^2R loss;
  - the peak current per mm of copper width on each layer, away from the pads (the hot spots), and where;
  - the same with every other net's via and through-hole moved out of the net's pours ("no foreign vias"), which
    is what those holes cost.
A map per net goes to OUT_DIR (default kicad/rev3/gen/out/copper_sim): one column per layer, current per mm of
width on top, voltage (mV above the first sink) below, the terminals marked with their currents and the net's
parts labelled (only the connectors for GND).

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
OUT = "kicad/rev3/gen/out/copper_sim"
CORRIDORS = "kicad/rev3/gen/out/corridors.npz"
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
    ("GND", float(os.environ.get("GND_H", "0.4")),
     dict({"J8.2": I42D / 3, "J9.2": I42D / 3, "J10.2": I42D / 3, "J11.2": I42C / 2, "J12.2": I42C / 2},
          **{f"J{k}.1": I24 / 6 for k in range(14, 20)}, **{f"J{k}.1": I12 / 6 for k in range(20, 26)},
          **{f"J{k}.1": I5 / 6 for k in range(26, 32)}),
     {"J1.2": I42D + I42C, "J5.1": I24, "J6.1": I12, "J7.1": I5}),
]

b, FP = None, {}


def load(board):
    """Work on this board (a path or a loaded BOARD); its pours are refilled."""
    global b, FP
    b = pcbnew.LoadBoard(board) if isinstance(board, str) else board
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
    V = np.full(k + nterm, np.nan)
    V[keep] = v
    V[ref] = 0.0
    # results
    loss = 0.0
    jmap, vmap = {}, {}
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
        jmap[L] = np.where(np.isnan(Vg), np.nan, np.hypot(ix, iy) / g.h)   # A per mm of width
        vmap[L] = Vg * 1e3                                                   # mV above the first sink
    tv = {t: V[k + j] for j, (t, _) in enumerate(terms)}
    return dict(loss=loss, tv=tv, jmap=jmap, vmap=vmap, tcells=tcells, ncells=k)


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


def labels_for(net, src, snk):
    """(x, y, text, kind): the terminals with their currents, and the parts on the net (connectors only for GND)."""
    out = []
    terms = set()
    for t, cur in list(src.items()) + [(t, -c) for t, c in snk.items()]:
        c = pad(t).GetPosition()
        out.append((mm(c.x) - X0, mm(c.y) - Y0, f"{t} {cur:+.1f} A", "src" if cur > 0 else "snk"))
        terms.add(t.split(".")[0])
    for f in b.GetFootprints():
        ref = f.GetReference()
        if ref in terms or not any(p.GetNetname() == net for p in f.Pads()):
            continue
        if net.split("/")[-1] == "GND" and not ref.startswith("J"):
            continue
        c = f.GetPosition()
        out.append((mm(c.x) - X0, mm(c.y) - Y0, ref, "part"))
    return out


def heatmap(g, r, masks, name, vmax, labels, ref):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    used = [L for L in LAYERS if masks[L].any()]
    ys, xs = np.nonzero(np.any([masks[L] for L in used], axis=0))
    pad_ = int(2 / g.h)
    x0, x1 = max(0, xs.min() - pad_), min(g.n, xs.max() + pad_)
    y0, y1 = max(0, ys.min() - pad_), min(g.n, ys.max() + pad_)
    ext = (x0 * g.h, x1 * g.h, y1 * g.h, y0 * g.h)
    scale = float(os.environ.get("FIGSCALE", "1"))
    aspect = max(0.6, (x1 - x0) / (y1 - y0))
    fig, axs = plt.subplots(2, len(used), figsize=(4.4 * scale * len(used) * aspect, 10.6 * scale), squeeze=False)
    vals = np.concatenate([r["vmap"][L][y0:y1, x0:x1].ravel() for L in used])
    vals = vals[~np.isnan(vals)]
    vlo, vhi = (float(vals.min()), float(vals.max())) if vals.size else (0.0, 1.0)
    fs = 5.5 if len(labels) > 25 else 6.5
    for row, (key, cmap, lo, hi) in enumerate((("jmap", "inferno", 0, vmax), ("vmap", "viridis", vlo, vhi))):
        for ax, L in zip(axs[row], used):
            im = ax.imshow(r[key][L][y0:y1, x0:x1], cmap=cmap, vmin=lo, vmax=hi, extent=ext, interpolation="nearest")
            ax.set_title(f"{name} {LNAME[L]}: " + ("current" if row == 0 else "voltage"), fontsize=9)
            ax.set_facecolor("#d8d8d8")
            ax.tick_params(labelsize=7)
            for x, y, text, kind in labels:
                if not (ext[0] <= x <= ext[1] and ext[3] <= y <= ext[2]):
                    continue
                if kind == "part":
                    ax.text(x, y, text, fontsize=fs, ha="center", va="center", color="black",
                            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.65))
                else:
                    ax.plot(x, y, marker="^" if kind == "src" else "v", ms=7, mec="black", mew=0.6,
                            color="#00e5ff" if kind == "src" else "#7CFC00")
                    ax.text(x, y + 1.6, text, fontsize=fs, ha="center", va="top", color="black",
                            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.75))
        fig.colorbar(im, ax=axs[row].tolist(), shrink=0.85,
                     label="A per mm of width" if row == 0 else f"mV above {ref}")
    fig.suptitle(f"{name}: current in (^) and out (v) at the marked pads; grey = no copper of this net", fontsize=10)
    os.makedirs(OUT, exist_ok=True)
    fig.savefig(f"{OUT}/{name.replace('@', '_')}.png", dpi=110, bbox_inches="tight")
    plt.close(fig)



# ---- current corridors: where another net's via would stand in a power net's current (2026-11-24) ---------------
CORR_H = 0.25           # mm cells
CORR_J = 1.0            # A per mm of width: a cell carrying this much (on any layer) is in the corridor
CORR_GROW = 0.5         # mm: a via's clearance hole reaches this far past its centre's cell


def corridors(board, out=CORRIDORS, log=print):
    """Solve every power net but GND on the board as it stands (run on the bare power copper, before routing) and
    save, per net, the cells where its current runs at CORR_J or more, grown by CORR_GROW: another net's via
    centre should not go there. gen/out/corridors.npz: 'nets' (names) and 'mask<i>' ([y, x], CORR_H cells)."""
    load(board)
    g = Grid(CORR_H)
    grow = int(round(CORR_GROW / CORR_H))
    names, out_masks = [], []
    for name, h, src, snk in SCEN:
        if name == "GND":
            continue
        net = net_of(name, list(src) + list(snk))
        masks = masks_for(net, g)
        r = solve(net, g, masks, src, snk)
        m = np.zeros((g.n, g.n), bool)
        for L in LAYERS:
            m |= np.nan_to_num(r["jmap"][L]) >= CORR_J
        for _ in range(grow):
            m[1:, :] |= m[:-1, :]; m[:-1, :] |= m[1:, :]; m[:, 1:] |= m[:, :-1]; m[:, :-1] |= m[:, 1:]
        if net in names:                                  # a net in two scenarios: one mask
            out_masks[names.index(net)] |= m
        else:
            names.append(net)
            out_masks.append(m)
        log(f"   corridor {name}: {m.sum() * CORR_H ** 2:.0f} mm2")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    np.savez_compressed(out, nets=np.array(names), h=CORR_H, **{f"mask{i}": m for i, m in enumerate(out_masks)})
    return dict(zip(names, out_masks))


class Corridors:
    """The saved corridors, for the via placers: blocked(net, x, y) is True where a via of net would stand in
    another power net's current. A via within FANOUT mm of a pad of its own net (a fan-out) is never blocked."""
    FANOUT = 1.5

    def __init__(self, path=CORRIDORS):
        self.masks = {}
        if os.path.exists(path):
            z = np.load(path)
            self.h = float(z["h"])
            for i, n in enumerate(z["nets"]):
                self.masks[str(n)] = z[f"mask{i}"]
        self.any = None
        if self.masks:
            self.any = np.zeros_like(next(iter(self.masks.values())))
            for m in self.masks.values():
                self.any |= m

    def owner(self, x, y):
        """The power nets whose corridor covers (x, y) (board mm)."""
        if not self.masks:
            return []
        ix, iy = int(x / self.h), int(y / self.h)
        if not (0 <= iy < self.any.shape[0] and 0 <= ix < self.any.shape[1]) or not self.any[iy, ix]:
            return []
        return [n for n, m in self.masks.items() if m[iy, ix]]

    def blocked(self, net, x, y):
        return any(n != net for n in self.owner(x, y))

    def grid(self, net, X, Y):
        """Boolean mask over the board-mm grid X, Y: True where net's vias are blocked."""
        if not self.masks:
            return np.zeros(X.shape, bool)
        m = np.zeros_like(self.any)
        for n, mk in self.masks.items():
            if n != net:
                m |= mk
        ix = np.clip((X / self.h).astype(int), 0, m.shape[1] - 1)
        iy = np.clip((Y / self.h).astype(int), 0, m.shape[0] - 1)
        return m[iy, ix]


def main():
    global OUT
    OUT = sys.argv[1] if len(sys.argv) > 1 else OUT
    load(BOARD)
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
        heatmap(g, r, masks, name, float(os.environ.get("VMAX", "8")), labels_for(net, src, snk), next(iter(snk)))
        rows.append((name, sum(src.values()), max(drops.values()), max(drops2.values()), r["loss"], r2["loss"], pk, pk2,
                     nvia_own))
        print(f"{name:9s} {sum(src.values()):5.1f} A  drop max {max(drops.values()):6.2f} mV "
              f"(no foreign vias {max(drops2.values()):6.2f})  loss {r['loss'] * 1e3:7.1f} mW "
              f"({r2['loss'] * 1e3:7.1f})  own vias {nvia_own:3d}", flush=True)
        for Ln, lst in pk.items():
            s = ", ".join(f"{j:.1f} A/mm @({x:.1f},{y:.1f})" for j, x, y in lst[:3])
            alt = pk2[Ln][0][0] if pk2[Ln] else 0
            print(f"      {Ln}: {s}   [no foreign vias: peak {alt:.1f}]", flush=True)


if __name__ == "__main__":
    main()
    sys.stdout.flush()
    os._exit(0)
