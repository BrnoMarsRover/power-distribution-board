"""Solve with padne's API. A polygon the mesher rejects ("Non-manifold mesh") is retried after cleaning:
snapped to a 1 um grid and re-buffered (+-0.5 um), which drops near-duplicate vertices; each fix is logged."""
import sys, pickle, pathlib, logging
import shapely
from padne import kicad, solver, mesh
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
log = logging.getLogger("run")
orig = mesh.Mesher.poly_to_mesh
fixed = []

def robust(self, poly, seed_points=()):
    try:
        return orig(self, poly, seed_points)
    except ValueError as e:
        b = poly.bounds
        for k, f in enumerate((lambda p: p.simplify(0.001).buffer(0),
                               lambda p: p.buffer(0.002, join_style=2).buffer(-0.002, join_style=2),
                               lambda p: p.simplify(0.005).buffer(0),
                               lambda p: shapely.set_precision(p, 0.001),
                               lambda p: p.simplify(0.02).buffer(0),
                               lambda p: p.buffer(-0.01, join_style=2).buffer(0.01, join_style=2),
                               lambda p: p.buffer(-0.03, join_style=2))):
            q = f(poly)
            if q.geom_type == "MultiPolygon":
                q = max(q.geoms, key=lambda g: g.area)
            try:
                m = orig(self, q, seed_points)
                fixed.append((b, poly.area, q.area, k))
                log.warning(f"mesh fix {k} for polygon at {tuple(round(v, 2) for v in b)} area {poly.area:.3f} -> {q.area:.3f}")
                return m
            except ValueError:
                continue
        log.error(f"polygon at {b} area {poly.area:.3f} could not be meshed: {e}")
        raise

mesh.Mesher.poly_to_mesh = robust

if __name__ == "__main__":
    pro, directives, out = map(pathlib.Path, sys.argv[1:4])
    prob = kicad.load_kicad_project(pro, extra_directives=kicad.load_directives_file(directives), schematic_directives=False)
    sol = solver.solve(prob)
    log.info(f"solver info: {sol.solver_info}")
    with open(out, "wb") as f:
        pickle.dump(sol, f)
    log.info(f"saved {out}; polygons fixed: {len(fixed)}")
