"""From a padne solution: per load case (copper_sim.SCEN), pad voltages, net loss sum(I*V) and the worst drop."""
import sys, pickle
import numpy as np
from scipy.spatial import cKDTree
import pcbnew
sys.path.insert(0, __import__("os").path.join(__import__("os").path.dirname(__import__("os").path.abspath(__file__)), ".."))
import copper_sim as cs
sol = pickle.load(open(sys.argv[1], "rb"))
board = pcbnew.LoadBoard(sys.argv[2])
mm = pcbnew.ToMM
names = [l.name for l in sol.problem.layers]
trees = {}
for li, ls in enumerate(sol.layer_solutions):
    pts = [p.mesh.positions() for p in ls.potentials]
    vals = [p.values for p in ls.potentials]
    if pts:
        P = np.vstack([np.asarray(p) for p in pts]); V = np.concatenate([np.asarray(v) for v in vals])
        trees[names[li]] = (cKDTree(P), V)

def pad_v(name):
    ref, num = name.split(".")
    f = board.FindFootprintByReference(ref)
    p = next(q for q in f.Pads() if q.GetNumber() == num)
    x, y = mm(p.GetPosition().x), mm(p.GetPosition().y)
    lay = "B.Cu" if (p.IsOnLayer(pcbnew.B_Cu) and not p.IsOnLayer(pcbnew.F_Cu)) else "F.Cu"
    t, V = trees[lay]
    d, i = t.query([x, y])
    return V[i], d

tot = 0
print(f"{'net':9s} {'loss mW':>9s} {'drop mV':>8s}   (padne)")
for name, h, src, snk in cs.SCEN:
    vs = {t: pad_v(t) for t in list(src) + list(snk)}
    far = max(d for v, d in vs.values())
    P = sum(i * vs[t][0] for t, i in src.items()) - sum(i * vs[t][0] for t, i in snk.items())
    drop = max(max(vs[s][0] for s in src) - vs[k][0] for k in snk)
    tot += P
    print(f"{name:9s} {P*1e3:9.1f} {drop*1e3:8.2f}" + (f"   (pad point {far:.3f} mm from a mesh vertex)" if far > 0.01 else ""))
print(f"total {tot:.3f} W; solver info {sol.solver_info}")
