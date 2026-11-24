#!/usr/bin/env python3
"""GND stitching (power_gnd.py): every GND fill island and GND pad tied to the L2 plane, two passes with a refill
between. Run with KiCad's Python after pcb_3v3.py:

    python3 kicad/rev3/gen/pcb_gnd.py
"""
import os
import sys
from pathlib import Path

import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
import power_gnd  # noqa: E402

BOARD = sys.argv[1] if len(sys.argv) > 1 else "kicad/rev3/PowerBoard_rev3.kicad_pcb"
b = pcbnew.LoadBoard(BOARD)
total_v, left = 0, []
for k in range(2):
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    v, t, left = power_gnd.stitch(b)
    total_v += len(v)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
# what is left, by the router: a path from the pad to a via into the L2 plane (GND vias may pierce 42 / 54 V copper,
# which clears 0.5 mm round them).
import route  # noqa: E402
R = route.Router(b)
R.geo.hv_gnd_ok = [(0.0, 0.0, 150.0, 150.0)]
routed, still = [], []
for item in left:
    ref, num = item.split(".")
    c = R.pad(ref, num)["c"]
    box = (c[0] - 8, c[1] - 8, c[0] + 8, c[1] + 8)
    ok = R.to_pour("GND", R.t_pad(ref, num), power_gnd.TRACK_W, box, (pcbnew.In1_Cu,), label=item)
    (routed if ok else still).append(item)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
print(f"GND: {total_v} stitching vias; routed to the plane: {', '.join(routed) or '-'}"
      + (f"; still open: {', '.join(still)}" if still else ""))
b.Save(BOARD)
# the native pcbnew (KiCad 10.0.8) can crash tearing itself down at interpreter exit, after the work is saved
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
