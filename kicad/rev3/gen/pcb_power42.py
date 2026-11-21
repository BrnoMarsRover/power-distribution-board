#!/usr/bin/env python3
"""Copper for the battery input and the two 42 V blocks (power42.py), the rail blocks (power_rails.py), the 54 V
boost, the aux supply, the GND plane (L2) and the GND fill on the bottom (L4). Run with KiCad's Python after
pcb_optimize.py; the signal routing (pcb_route.py) and the +3V3 distribution (pcb_3v3.py) follow:

    flatpak run --command=python3 org.kicad.KiCad kicad/rev3/gen/pcb_power42.py
"""
import sys
from pathlib import Path

import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
import power42  # noqa: E402
import power_rails  # noqa: E402
import power54  # noqa: E402
import power_aux  # noqa: E402

BOARD = "kicad/rev3/PowerBoard_rev3.kicad_pcb"
b = pcbnew.LoadBoard(BOARD)
zones, vias, tracks = power42.copper(b)
print(f"42 V copper: {len(zones)} zones, {len(vias)} vias, {len(tracks)} track segments")
zones, vias, tracks = power_rails.copper(b)
print(f"rail copper ({', '.join(power_rails.RAILS)} V): {len(zones)} zones, {len(vias)} vias, {len(tracks)} track segments")
zones, vias, tracks = power54.copper(b)
print(f"54 V boost copper: {len(zones)} zones, {len(vias)} vias, {len(tracks)} track segments")
zones, vias, tracks = power_aux.copper(b)
print(f"aux supply copper: {len(zones)} zones, {len(vias)} vias")
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(BOARD)
