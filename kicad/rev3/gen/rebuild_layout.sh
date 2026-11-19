#!/bin/bash
# Regenerate schematic + first-pass layout from scratch. DESTROYS any hand placement and any KiCad edits
# to the schematic: use only before hand layout starts. Run from the repository root.
#   kicad/rev3/gen/rebuild_layout.sh --yes
set -euo pipefail
[ "${1:-}" = "--yes" ] || { echo "rebuilds schematic and board from the generators; pass --yes"; exit 1; }
KPY="flatpak run --command=python3 org.kicad.KiCad"
KCLI="flatpak run --command=kicad-cli org.kicad.KiCad"
R=kicad/rev3
python3 $R/gen/build.py --force | tail -3
$KPY $R/gen/fix_footprints.py 2>&1 | grep -v swig/python
git show HEAD:$R/PowerBoard_rev3.kicad_pcb > /dev/null            # sanity: inside the repo
cp $R/gen/empty_board.kicad_pcb $R/PowerBoard_rev3.kicad_pcb
python3 $R/gen/board_rules.py > /dev/null
$KCLI sch export netlist -o $R/gen/out/netlist.net $R/PowerBoard_rev3.kicad_sch > /dev/null
$KPY $R/gen/pcb_import.py 2>&1 | grep -v swig/python
$KPY $R/gen/pcb_place.py --overwrite-placement 2>&1 | grep -v swig/python
$KPY $R/gen/pcb_optimize.py 2>&1 | grep -v swig/python
$KPY $R/gen/pcb_power42.py 2>&1 | grep -v -i "swig/python\|debug"
$KPY $R/gen/pcb_silk.py 2>&1 | grep -v swig/python
$KPY $R/gen/pcb_models.py 2>&1 | grep -v swig/python | grep -i missing || true
$KCLI pcb drc --schematic-parity --format json -o $R/gen/out/drc.json $R/PowerBoard_rev3.kicad_pcb | grep -i found
(cd docs/rev3 && python3 rails.py > rails.md)
$KCLI sch export bom --fields 'MPN,Value,Footprint,${QUANTITY},Reference' --labels 'MPN,Value,Footprint,Qty,References' \
    --group-by 'MPN,Footprint' --sort-field MPN -o $R/PowerBoard_rev3_bom.csv $R/PowerBoard_rev3.kicad_sch > /dev/null
python3 $R/gen/bom_lcsc.py
$KCLI sch export pdf -o pdf/rev3/PowerBoard_rev3_schematic.pdf $R/PowerBoard_rev3.kicad_sch > /dev/null
git checkout -- kicad/PowerBoard.kicad_pro 2>/dev/null || true
