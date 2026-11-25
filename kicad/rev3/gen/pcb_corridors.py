#!/usr/bin/env python3
"""Current corridors (copper_sim.corridors): run after pcb_power42.py, before the routing. Solves each power net's
current in the bare power copper and saves where it runs (1 A per mm of width or more, grown by a via's clearance
hole) to gen/out/corridors.npz. The router, the GND stitching and the +3V3 fan-out keep other nets' vias out of
them, so no via stands in a power net's current; a plan's hand-placed via that does is reported by route.Router.via.

    python3 kicad/rev3/gen/pcb_corridors.py
"""
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import copper_sim  # noqa: E402

BOARD = sys.argv[1] if len(sys.argv) > 1 else "kicad/rev3/PowerBoard_rev3.kicad_pcb"
c = copper_sim.corridors(BOARD, log=lambda m: None)
print(f"corridors: {len(c)} power nets, {sum(int(m.sum()) for m in c.values()) * copper_sim.CORR_H ** 2:.0f} mm2")
sys.stdout.flush()
os._exit(0)
