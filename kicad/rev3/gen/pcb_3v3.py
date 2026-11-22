#!/usr/bin/env python3
"""The +3V3 distribution (power_logic.py): an L3 fill and a via fan-out from every +3V3 pin, placed around the
power copper and the signal routes already on the board. Then every zone is refilled. Run with KiCad's Python
after pcb_route.py:

    python3 kicad/rev3/gen/pcb_3v3.py
"""
import os
import sys
from pathlib import Path

import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
import power_logic  # noqa: E402

BOARD = "kicad/rev3/PowerBoard_rev3.kicad_pcb"
b = pcbnew.LoadBoard(BOARD)
zones, vias, tracks, missed = power_logic.copper(b)
print(f"+3V3: L3 fill, {len(vias)} vias to it" + (f"; no via spot for {', '.join(missed)}" if missed else ""))
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(BOARD)

# the native pcbnew (KiCad 10.0.8) can crash tearing itself down at interpreter exit, after the work is saved,
# which would stop the rebuild: leave at once
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
