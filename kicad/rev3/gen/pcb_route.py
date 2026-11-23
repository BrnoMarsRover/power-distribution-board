#!/usr/bin/env python3
"""Signal routing of one block by its plan (route_plan.py, router in route.py), on the board as pcb_power42.py
left it (the power copper's fills are the supply targets). Run with KiCad's Python, once per block, each in its
own process (pcbnew objects come back untyped after removals within one process):

    python3 kicad/rev3/gen/pcb_route.py 24 [board]

Blocks: ina (the INA228's link, first: the blocks route around it), the rail eFuses 24, 12, 5, the 42 V eFuses 42D, 42C, 42legs (the 42 V bus legs, after both), 42bus
(their far end), flt (the FLT bus to the ESP32), i2c (the I2C bus and ALERT, i2c_plan), rails (the rails' bus
legs, around it), sw (the switch-control group) and can (the CAN pair and termination, can_plan). Prints what could not be routed;
exits 1 if anything failed.
"""
import os
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
if key == "42legs":
    route_plan.legs42(R, FP, log=quiet)
    what = "42 V bus legs"
elif key == "rails":
    route_plan.rails_legs(R, FP, log=quiet)
    what = "rails' bus legs"
elif key == "42bus":
    import far42
    far42.route(R, FP, log=quiet)
    what = "42 V bus, far end"
elif key == "sw":
    route_plan.sw_control(R, FP, log=quiet)
    what = "switch control"
elif key == "i2c":
    import i2c_plan
    i2c_plan.route(R, FP, log=quiet)
    what = "I2C bus"
elif key == "esp":
    import esp_plan
    esp_plan.route(R, FP, log=quiet)
    esp_plan.corner(R, FP, log=quiet)
    esp_plan.escapes(R, FP, log=quiet)
    esp_plan.brake(R, FP, log=quiet)
    esp_plan.usb(R, FP, log=quiet)
    esp_plan.motor_ok_link(R, log=quiet)
    what = "ESP32 group (host UART, north bundle, BRAKE_ACT, USB)"
elif key == "can":
    import can_plan
    can_plan.route(R, FP, log=quiet)
    what = "CAN bus"
elif key == "v5":
    import logic5v
    logic5v.route(R, FP, log=quiet)
    what = "+5V_LOGIC and VBUS"
elif key == "espnet":
    import esp_plan
    esp_plan.local(R, FP, log=quiet)
    what = "ESP32 group's own nets"
elif key == "ina":
    import ina_link
    ina_link.route(R, FP, log=quiet)
    ina_link.sense(R, log=quiet)
    what = "INA228 link"
elif key == "boost":
    import boost_plan
    boost_plan.route(R, FP, log=quiet)
    what = "54 V boost controller"
elif key == "aux":
    import power_aux
    power_aux.route(R, FP, log=quiet)
    what = "aux supply controller"
elif key == "brake":
    import boost_plan
    boost_plan.comparator(R, log=quiet)
    what = "brake comparator"
elif key == "flt":
    import flt_bus
    flt_bus.route(R, FP, log=quiet)
    what = "FLT bus"
else:
    (route_plan.efuse42 if key.startswith("42") else route_plan.efuse_rail)(R, FP, key, log=quiet)
    what = f"{key} V eFuse"
print(f"{what} routed in {time.time() - t0:.0f} s" + (f"; unrouted: {', '.join(R.failed)}" if R.failed else ""))
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(BOARD)
# the native pcbnew (KiCad 10.0.8) can crash tearing itself down at interpreter exit, after the work is saved,
# which would stop the rebuild: leave at once
sys.stdout.flush()
sys.stderr.flush()
os._exit(1 if R.failed else 0)
