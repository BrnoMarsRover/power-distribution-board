#!/usr/bin/env python3
"""Footprint fixes for layout (run with KiCad's Python; idempotent):
- XT90PB-F/M and DEANS-T (copied from rev2): add an F.CrtYd rectangle 0.5 mm around the part,
  reference text 0.15 mm thick (JLC silkscreen minimum).
- ESP32-S3-WROOM-1U_HandSolder: the stock footprint's twelve 0.2 mm thermal vias (below JLC's 0.3 mm
  standard drill) become one 1.2 mm plated hole, so the ground pad can be soldered from the back.
- USB_C_Receptacle_GCT_USB4105-GF-A_2oz: the stock USB4105 footprint with its four shell-leg pads
  widened from 1.0 to 1.15 mm (and 0.15 mm taller), so the plated slots keep a 0.275 mm annular ring,
  above JLC's 0.254 mm minimum for 2 oz outer copper. The four corner SMD pads lose 0.05 mm on the side
  facing the plastic locating pegs, which brings them from 0.194 to 0.24 mm off the NPTH (JLC 0.2 mm).
"""
from pathlib import Path
import sys
import os
import pcbnew

LIB = Path(__file__).resolve().parent.parent / "PowerBoard_rev3.pretty"
STOCK = "/usr/share/kicad/footprints/RF_Module.pretty"
MM = pcbnew.FromMM
IO = pcbnew.PCB_IO_KICAD_SEXPR()

# first load in the file: later pcbnew.FootprintLoad calls on another library come back untyped (SWIG)
fp = pcbnew.FootprintLoad(STOCK.replace("RF_Module", "Connector_USB"),
                          "USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal")
for pad in fp.Pads():
    if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
        drill = pad.GetDrillSize()
        pad.SetSize(pcbnew.VECTOR2I(drill.x + MM(0.55), drill.y + MM(0.55)))   # 0.275 mm ring all round
    elif pad.GetNumber() in ("A1", "B12", "A12", "B1"):   # corner pads: 0.194 -> 0.24 mm to the locating pegs
        pad.SetSize(pcbnew.VECTOR2I(pad.GetSizeX(), pad.GetSizeY() - MM(0.05)))
        pad.SetPosition(pad.GetPosition() - pcbnew.VECTOR2I(0, MM(0.025)))      # trimmed on the peg side
fp.SetFPID(pcbnew.LIB_ID("PowerBoard_rev3", "USB_C_Receptacle_GCT_USB4105-GF-A_2oz"))
fp.SetLibDescription("GCT USB4105-GF-A USB 2.0 Type-C receptacle; shell-leg pads widened for a 0.275 mm "
                     "annular ring (JLC 2 oz outer copper)")
IO.FootprintSave(str(LIB), fp)
print("USB_C_Receptacle_GCT_USB4105-GF-A_2oz saved, shell pads",
      sorted({tuple(round(pcbnew.ToMM(v), 2) for v in (p.GetSizeX(), p.GetSizeY()))
              for p in fp.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH}))

for name in ("XT90PB-F", "XT90PB-M", "DEANS-T"):
    fp = pcbnew.FootprintLoad(str(LIB), name)
    for item in list(fp.GraphicalItems()):
        if item.GetLayer() == pcbnew.F_CrtYd:
            fp.Remove(item)
    bb = fp.GetBoundingBox(False)          # pads, silk and fab, without texts
    r = pcbnew.PCB_SHAPE(fp, pcbnew.SHAPE_T_RECTANGLE)
    r.SetLayer(pcbnew.F_CrtYd)
    r.SetWidth(MM(0.05))
    grid = lambda v, up: MM(round((pcbnew.ToMM(v) + (0.5 if up else -0.5)) / 0.01) * 0.01)
    r.SetStart(pcbnew.VECTOR2I(grid(bb.GetLeft(), False), grid(bb.GetTop(), False)))
    r.SetEnd(pcbnew.VECTOR2I(grid(bb.GetRight(), True), grid(bb.GetBottom(), True)))
    fp.Add(r)
    for f in (fp.Reference(), fp.Value()):
        f.SetTextThickness(max(f.GetTextThickness(), MM(0.15)))
    IO.FootprintSave(str(LIB), fp)
    print(name, "courtyard", round(pcbnew.ToMM(bb.GetWidth()) + 1, 2), "x", round(pcbnew.ToMM(bb.GetHeight()) + 1, 2))

fp = pcbnew.FootprintLoad(STOCK, "ESP32-S3-WROOM-1U")
centre = None
for pad in list(fp.Pads()):
    if pad.GetNumber() == "41":
        if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
            fp.Remove(pad)
        else:
            centre = pad.GetPosition()
hole = pcbnew.PAD(fp)
hole.SetNumber("41")
hole.SetAttribute(pcbnew.PAD_ATTRIB_PTH)
hole.SetShape(pcbnew.PAD_SHAPE_CIRCLE)
hole.SetSize(pcbnew.VECTOR2I(MM(2.0), MM(2.0)))
hole.SetDrillSize(pcbnew.VECTOR2I(MM(1.2), MM(1.2)))
hole.SetLayerSet(pcbnew.PAD.PTHMask())
hole.SetPosition(centre)
fp.Add(hole)
fp.SetFPID(pcbnew.LIB_ID("PowerBoard_rev3", "ESP32-S3-WROOM-1U_HandSolder"))
fp.SetAttributes((fp.GetAttributes() & ~pcbnew.FP_SMD) | pcbnew.FP_THROUGH_HOLE)   # it now has a PTH pad
fp.SetLibDescription("ESP32-S3-WROOM-1U; ground pad with one 1.2 mm plated hole for soldering from the back (rev3 hand assembly)")
IO.FootprintSave(str(LIB), fp)
print("ESP32-S3-WROOM-1U_HandSolder saved, ground-pad hole at", [round(pcbnew.ToMM(v), 2) for v in (centre.x, centre.y)])

# the native pcbnew (KiCad 10.0.8) can crash tearing itself down at interpreter exit, after the work is saved,
# which would stop the rebuild: leave at once
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
