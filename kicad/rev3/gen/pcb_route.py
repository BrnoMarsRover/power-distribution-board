#!/usr/bin/env python3
"""Signal routing of one block by its plan (route_plan.py, router in route.py), on the board as pcb_power42.py
left it (the power copper's fills are the supply targets). Run with KiCad's Python, once per block, each in its
own process (pcbnew objects come back untyped after removals within one process):

    flatpak run --command=python3 org.kicad.KiCad kicad/rev3/gen/pcb_route.py 24 [board]

Blocks: the rail eFuses 24, 12, 5. Prints what could not be routed; exits 1 if anything failed.
"""
import sys
import time
from pathlib import Path

import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
import route  # noqa: E402
import route_plan  # noqa: E402

key = sys.argv[1]
BOARD = sys.argv[2] if len(sys.argv) > 2 else "kicad/rev3/PowerBoard_rev3.kicad_pcb"   # (another board: development)
b = pcbnew.LoadBoard(BOARD)
FP = {f.GetReference(): f for f in b.GetFootprints()}
t0 = time.time()
R = route.Router(b)
quiet = lambda msg: None if msg.lstrip().startswith("round ") else print(msg)
route_plan.efuse_rail(R, FP, key, log=quiet)
print(f"{key} V eFuse routed in {time.time() - t0:.0f} s" + (f"; unrouted: {', '.join(R.failed)}" if R.failed else ""))
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(BOARD)
sys.exit(1 if R.failed else 0)
