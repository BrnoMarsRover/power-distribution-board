#!/usr/bin/env python3
"""One-time import of the rev3 schematic into PowerBoard_rev3.kicad_pcb, with KiCad's own Python.

    python3 kicad/rev3/gen/pcb_import.py

Reads gen/out/netlist.net (kicad-cli sch export netlist), loads every footprint, links it to its
schematic symbol (path = sheet UUID / symbol UUID, so KiCad's "Update PCB from Schematic" matches
it), assigns pad nets, and drops each sheet's parts in its own cluster to the right of the outline.
Refuses to run on a board that already has footprints: after this, placement is done by hand and
changes come from the schematic with Update PCB from Schematic (F8).
"""
import os
import math
import sys
from pathlib import Path

import pcbnew

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from kisym import find, parse, unq  # noqa: E402

PROJ = HERE.parent
BOARD = PROJ / "PowerBoard_rev3.kicad_pcb"
NETLIST = HERE / "out" / "netlist.net"
STOCK_FP = Path("/usr/share/kicad/footprints")

OUTLINE = (120.0, 140.0)          # mm, target from design.md section 8; the user asked for smaller than rev2
ORIGIN = (50.0, 50.0)             # board corner on the page
HOLE_INSET = 4.0                  # M3 hole centres from each corner
MM = pcbnew.FromMM


def lib_dir(lib):
    return PROJ / "PowerBoard_rev3.pretty" if lib == "PowerBoard_rev3" else STOCK_FP / f"{lib}.pretty"


def field(node, key):
    f = find(node, key)
    return unq(f[0][1]) if f and len(f[0]) > 1 else ""


def main():
    board = pcbnew.LoadBoard(str(BOARD))
    if len(board.GetFootprints()):
        raise SystemExit("board already has footprints: use Update PCB from Schematic in KiCad instead")
    tree = parse(NETLIST.read_text())
    comps = find(find(tree, "components")[0], "comp")
    nets = find(find(tree, "nets")[0], "net")

    # nets first, so pads can point at them
    netinfo, pad_net = {}, {}
    for n in nets:
        name = field(n, "name")
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        netinfo[name] = ni
        for node in find(n, "node"):
            pad_net[(field(node, "ref"), field(node, "pin"))] = ni

    # load footprints, grouped by sheet
    by_sheet = {}
    for c in comps:
        ref, value, fpid = field(c, "ref"), field(c, "value"), field(c, "footprint")
        lib, name = fpid.split(":", 1)
        fp = pcbnew.FootprintLoad(str(lib_dir(lib)), name)
        if fp is None:
            raise SystemExit(f"{ref}: footprint {fpid} not found")
        fp.SetFPID(pcbnew.LIB_ID(lib, name))
        fp.SetReference(ref)
        fp.SetValue(value)
        sp = find(c, "sheetpath")[0]
        tstamps = field(sp, "tstamps")                       # "/sheet-uuid/"
        sym = field(c, "tstamps")
        fp.SetPath(pcbnew.KIID_PATH(f"{tstamps}{sym}"))
        props = {field(p, "name"): field(p, "value") for p in find(c, "property")}
        fp.SetSheetname(props.get("Sheetname", ""))
        fp.SetSheetfile(props.get("Sheetfile", ""))
        # match the symbol exactly, so the DRC schematic-parity check is clean
        flds = {field(f, "name"): (unq(f[2]) if len(f) > 2 else "") for f in find(find(c, "fields")[0], "field")}
        for key in ("Datasheet", "Description"):
            fp.SetField(key, flds.get(key, ""))
        fp.SetExcludedFromBOM("exclude_from_bom" in props)
        fp.SetDNP("dnp" in props)
        mpn = props.get("MPN", "")
        if mpn:
            fp.SetField("MPN", mpn)
            fp.GetField("MPN").SetVisible(False)
            fp.GetField("MPN").SetLayer(pcbnew.F_Fab)
        for pad in fp.Pads():
            ni = pad_net.get((ref, pad.GetNumber()))
            if ni is not None:
                pad.SetNet(ni)
        board.Add(fp)
        by_sheet.setdefault(props.get("Sheetname", "?"), []).append(fp)

    # outline and mounting holes
    x0, y0 = ORIGIN
    w, h = OUTLINE
    rect = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_RECTANGLE)
    rect.SetStart(pcbnew.VECTOR2I(MM(x0), MM(y0)))
    rect.SetEnd(pcbnew.VECTOR2I(MM(x0 + w), MM(y0 + h)))
    rect.SetLayer(pcbnew.Edge_Cuts)
    rect.SetWidth(MM(0.1))
    board.Add(rect)
    holes = pcbnew.FootprintLoad(str(STOCK_FP / "MountingHole.pretty"), "MountingHole_3.2mm_M3_Pad_Via")
    for k, (hx, hy) in enumerate([(HOLE_INSET, HOLE_INSET), (w - HOLE_INSET, HOLE_INSET),
                                  (HOLE_INSET, h - HOLE_INSET), (w - HOLE_INSET, h - HOLE_INSET)]):
        mh = pcbnew.FOOTPRINT(holes)
        mh.SetReference(f"H{k + 1}")
        mh.SetValue("M3")
        mh.SetBoardOnly(True)                # no schematic symbol: keeps the parity check clean
        mh.SetExcludedFromBOM(True)
        mh.Reference().SetVisible(False)
        mh.SetPosition(pcbnew.VECTOR2I(MM(x0 + hx), MM(y0 + hy)))
        for pad in mh.Pads():
            pad.SetNet(netinfo["GND"])
        board.Add(mh)

    # park each sheet's parts in its own labelled cluster to the right of the board
    cx, cy, col_w = x0 + w + 20, y0, 0.0
    for sheet, fps in by_sheet.items():
        fps.sort(key=lambda f: -f.GetBoundingBox(False).GetWidth())
        per_row = max(1, int(math.sqrt(len(fps))))
        x, y, row_h, width = cx, cy + 8, 0.0, 0.0
        for k, fp in enumerate(fps):
            bb = fp.GetBoundingBox(False)
            bw, bh = pcbnew.ToMM(bb.GetWidth()) + 2, pcbnew.ToMM(bb.GetHeight()) + 2
            if k and k % per_row == 0:
                x, y, row_h = cx, y + row_h, 0.0
            off = fp.GetPosition() - bb.GetCenter()
            fp.SetPosition(pcbnew.VECTOR2I(MM(x + bw / 2), MM(y + bh / 2)) + off)
            x += bw
            row_h, width = max(row_h, bh), max(width, x - cx)
        t = pcbnew.PCB_TEXT(board)
        t.SetText(sheet[:40])
        t.SetLayer(pcbnew.Cmts_User)
        t.SetPosition(pcbnew.VECTOR2I(MM(cx), MM(cy + 3)))
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
        board.Add(t)
        cy = y + row_h + 12
        col_w = max(col_w, width)
        if cy > y0 + 260:
            cx, cy, col_w = cx + col_w + 15, y0, 0.0

    board.Save(str(BOARD))
    print(f"imported {len(comps)} footprints, {len(nets)} nets, {len(by_sheet)} sheet clusters")


main()

# the native pcbnew (KiCad 10.0.8) can crash tearing itself down at interpreter exit, after the work is saved,
# which would stop the rebuild: leave at once
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
