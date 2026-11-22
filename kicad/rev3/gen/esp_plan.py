"""The ESP32 group's routing plan (in progress, 2026-11-22).

Reserved so far: the band above the module's top pin row on the top side. The module's top-row pins escape
there (the FLT bus's pin vias and stubs, flt_bus; next the UART, USB and EN pins), so no top-side part may sit in
it (the aux supply's test pads had drifted into it).
"""

KEEP_TOP = [(129.2, 55.5, 149.4, 60.3)]         # above the module's top pin row (pads from y 60.3)
