#!/bin/bash
# Regenerate schematic + first-pass layout from scratch. DESTROYS any hand placement and any KiCad edits
# to the schematic: use only before hand layout starts. Run from the repository root.
#   kicad/rev3/gen/rebuild_layout.sh --yes
set -euo pipefail
[ "${1:-}" = "--yes" ] || { echo "rebuilds schematic and board from the generators; pass --yes"; exit 1; }
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
echo "[$SECONDS s] routing"
cp $R/PowerBoard_rev3.kicad_pcb $R/gen/out/stage_copper.kicad_pcb    # for development: placed + power copper
for block in ina 24 12 5 42D 42C 42legs 42bus flt i2c esp rails sw; do                        # signal routing, block by block
    $KPY $R/gen/pcb_route.py $block 2>&1 | grep -v -i "swig/python\|debug\|GetWidth called" || true
done
echo "[$SECONDS s] +3V3"
$KPY $R/gen/pcb_3v3.py 2>&1 | grep -v -i "swig/python\|debug\|GetWidth called"
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
