#!/bin/bash
# Regenerate schematic + first-pass layout from scratch. DESTROYS any hand placement and any KiCad edits
# to the schematic: use only before hand layout starts. Run from the repository root.
#   kicad/rev3/gen/rebuild_layout.sh --yes
set -euo pipefail
# Hand layout started 2026-11-26 (vias cleaned up by hand in KiCad, in-pad vias moved): the board file is the source
# now, and a rebuild would throw that work away (the routing is not exactly reproducible either: the router works to
# time budgets). Kept for reference; it needs --discard-hand-layout as well.
[ "${1:-}" = "--yes" ] && [ "${2:-}" = "--discard-hand-layout" ] || {
    echo "the board is hand-maintained since 2026-11-26: a rebuild discards that work; pass --yes --discard-hand-layout"; exit 1; }
# KiCad 10 from the kicad-10.0-releases PPA: its pcbnew module is in the system Python. Development switches:
# HAND_DUMP (efuse_hand.py), SAT_NOCACHE, SAT_KEY_DUMP and SAT_DEBUG (satellites.py), from the environment
KPY="python3"
KCLI="kicad-cli"
R=kicad/rev3
python3 $R/gen/build.py --force | tail -3
$KPY $R/gen/fix_footprints.py 2>&1 | grep -v swig/python
git show HEAD:$R/PowerBoard_rev3.kicad_pcb > /dev/null            # sanity: inside the repo
cp $R/gen/empty_board.kicad_pcb $R/PowerBoard_rev3.kicad_pcb
python3 $R/gen/board_rules.py > /dev/null
$KCLI sch export netlist -o $R/gen/out/netlist.net $R/PowerBoard_rev3.kicad_sch > /dev/null
$KPY $R/gen/pcb_import.py 2>&1 | grep -v swig/python
echo "[$SECONDS s] placement"
$KPY $R/gen/pcb_place.py --overwrite-placement 2>&1 | grep -v swig/python
echo "[$SECONDS s] optimizer"
$KPY $R/gen/pcb_optimize.py 2>&1 | grep -v swig/python
echo "[$SECONDS s] power copper"
$KPY $R/gen/pcb_power42.py 2>&1 | grep -v -i "swig/python\|debug\|GetWidth called"
echo "[$SECONDS s] corridors"
$KPY $R/gen/pcb_corridors.py 2>&1 | grep -v -i "swig/python\|debug\|GetWidth called"
echo "[$SECONDS s] routing"
cp $R/PowerBoard_rev3.kicad_pcb $R/gen/out/stage_copper.kicad_pcb    # for development: placed + power copper
for block in ina 24 12 5 42D 42C 42legs 42bus boost aux flt i2c esp can v5 esprest brake rails sw espnet; do                        # signal routing, block by block
    $KPY $R/gen/pcb_route.py $block 2>&1 | grep -v -i "swig/python\|debug\|GetWidth called" || true
done
echo "[$SECONDS s] +3V3"
$KPY $R/gen/pcb_3v3.py 2>&1 | grep -v -i "swig/python\|debug\|GetWidth called"
echo "[$SECONDS s] GND"
$KPY $R/gen/pcb_gnd.py 2>&1 | grep -v -i "swig/python\|debug\|GetWidth called"
echo "[$SECONDS s] tidy"
$KPY $R/gen/pcb_tidy.py 2>&1 | grep -v -i "swig/python\|debug\|GetWidth called"
echo "[$SECONDS s] silkscreen"
$KPY $R/gen/pcb_silk.py 2>&1 | grep -v swig/python
$KPY $R/gen/pcb_models.py 2>&1 | grep -v swig/python | grep -i missing || true
echo "[$SECONDS s] DRC"
$KCLI pcb drc --schematic-parity --format json -o $R/gen/out/drc.json $R/PowerBoard_rev3.kicad_pcb | grep -i found
(cd docs/rev3 && python3 rails.py > rails.md)
$KCLI sch export bom --fields 'MPN,Value,Footprint,${QUANTITY},Reference' --labels 'MPN,Value,Footprint,Qty,References' \
    --group-by 'MPN,Footprint' --sort-field MPN -o $R/PowerBoard_rev3_bom.csv $R/PowerBoard_rev3.kicad_sch > /dev/null
python3 $R/gen/bom_lcsc.py
$KCLI sch export pdf -o pdf/rev3/PowerBoard_rev3_schematic.pdf $R/PowerBoard_rev3.kicad_sch > /dev/null
git checkout -- kicad/PowerBoard.kicad_pro 2>/dev/null || true
echo "[$SECONDS s] done"
