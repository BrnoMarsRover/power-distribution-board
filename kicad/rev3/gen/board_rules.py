#!/usr/bin/env python3
"""Board rules for JLCPCB's standard 4-layer process (nothing that adds a surcharge).

    python3 kicad/rev3/gen/board_rules.py

Writes, and is safe to re-run:
  - PowerBoard_rev3.kicad_pro: board design rules, preset track / via sizes, netclasses
    and the patterns that put nets into them (merged; other settings are kept).
  - PowerBoard_rev3.kicad_dru: custom DRC rules (JLC limits KiCad has no field for, and
    the high-voltage clearance).
  - PowerBoard_rev3.kicad_pcb: only if it does not exist yet. An empty 4-layer board with
    JLC's default 1.6 mm stackup (JLC04161H-7628).
Then runs KiCad's DRC on the board so a typo in the rules fails here, not in the editor.

Limits are from https://jlcpcb.com/capabilities/pcb-capabilities and
https://jlcpcb.com/help/article/in-what-cases-will-there-be-charged-extra (2026-11-19).
"""

import json
import subprocess
from pathlib import Path

PROJ = Path(__file__).resolve().parent.parent
NAME = "PowerBoard_rev3"
KICAD_CLI = ["flatpak", "run", "--command=kicad-cli", "org.kicad.KiCad"]

# ---------------------------------------------------------------- JLC standard process
# What costs extra on a 4-layer order, and the margin taken here:
#   trace/space 3.0-3.5 mil (+20 %)       -> 0.25 mm tracks, 0.127 mm (5 mil) clearance
#   via hole < 0.3 mm                      -> 0.3 mm minimum drill
#   2 oz copper, non-1.6 mm thickness      -> 1 oz outer, 0.5 oz inner, 1.6 mm
#   filled / capped vias (via-in-pad)      -> vias tented only, never in pads
RULES = {
    "min_clearance": 0.127,             # JLC 0.09 at 1 oz
    "min_track_width": 0.25,            # team choice: robust for hand rework (JLC 0.09)
    "min_connection": 0.25,
    "min_via_diameter": 0.6,            # 0.3 drill + 2 x 0.15 annular
    "min_via_annular_width": 0.15,      # JLC absolute minimum for multilayer PTH
    "min_through_hole_diameter": 0.3,   # below 0.3 mm is charged extra
    "min_hole_clearance": 0.25,         # JLC via hole to track 0.2
    "min_hole_to_hole": 0.25,           # JLC via-to-via 0.2 (pads: 0.45, see .kicad_dru)
    "min_copper_edge_clearance": 0.3,   # JLC 0.2 to a routed edge
    "min_silk_clearance": 0.15,         # keep silkscreen off pads (JLC clips it anyway)
    "min_text_height": 1.0,             # JLC silk text 1.0 mm
    "min_text_thickness": 0.15,         # JLC silk line 0.15 mm
    "solder_mask_min_width": 0.1,       # JLC green mask dam 0.1 mm
    "solder_mask_clearance": 0.0,       # JLC opens the mask 1:1 with the pad
    "solder_mask_to_copper_clearance": 0.0,
    "min_microvia_diameter": 0.6,       # no microvias on this board
    "min_microvia_drill": 0.3,
    "min_groove_width": 0.0,
    "min_resolved_spokes": 2,
    "max_error": 0.005,
    "use_height_for_length_calcs": True,
}

# Checks that should stop a board from going out, not just warn.
SEVERITIES = {
    "courtyards_overlap": "error", "missing_courtyard": "error", "pth_inside_courtyard": "error",
    "npth_inside_courtyard": "error", "silk_over_copper": "error", "silk_edge_clearance": "error",
    "silk_overlap": "warning", "text_height": "error", "text_thickness": "error",
    "solder_mask_bridge": "error", "starved_thermal": "error", "footprint_type_mismatch": "error",
    "lib_footprint_mismatch": "warning", "lib_footprint_issues": "warning",
    "isolated_copper": "warning", "track_dangling": "warning", "via_dangling": "warning",
    "unconnected_items": "error", "schematic_parity": "error",
}

# Board name and revision, for silkscreen and fab-layer text (${BOARD} ${REV}).
TEXT_VARIABLES = {"BOARD": "BMR PowerBoard", "REV": "3.0", "FAB": "JLCPCB 4L 1.6mm 1oz/0.5oz HASL-LF"}

TRACK_WIDTHS = [0.0, 0.25, 0.3, 0.5, 1.0, 1.5, 2.0, 3.0]          # 0.0 = netclass width
VIAS = [(0.0, 0.0), (0.6, 0.3), (0.8, 0.4), (1.0, 0.6)]          # (diameter, drill)

# ---------------------------------------------------------------- netclasses
# Widths are routing defaults. Rails above ~5 A belong in pours on several layers (see
# docs/rev3/design.md, "Board rules"); the netclass width only covers short stubs.
# Clearance stays 0.2 mm for every class so fine-pitch pins on 42 V nets (TPS48111, INA238)
# pass DRC; the extra 0.5 mm for 42 / 54 V copper is a custom rule on pours and wide tracks.
CLASSES = [
    # name, track, clearance, via (dia, drill), colour, patterns
    ("Default", 0.25, 0.2, (0.6, 0.3), "rgba(0, 0, 0, 0.000)", []),
    ("HV_Bus", 3.0, 0.2, (1.0, 0.6), "rgb(255, 0, 0)",
     ["VBAT", "BUS42", "OUT42D", "OUT42C", "/42 V*/DRN", "/Brake*/BRK_SW"]),
    ("HV_Power", 1.5, 0.2, (1.0, 0.6), "rgb(255, 120, 0)",
     ["SW54_OUT", "OUT54", "/54 V*/DRN", "/54 V boost*/LIN", "/54 V boost*/SW"]),
    ("Power", 1.0, 0.2, (1.0, 0.6), "rgb(0, 160, 255)",
     ["VIN24", "VIN12", "VIN5", "OUT24", "OUT12", "OUT5", "/24 V*/DRN", "/12 V*/DRN",
      "/5 V*/DRN", "/Rail inputs*/O*", "*/CHG_N"]),
    ("Supply", 0.4, 0.2, (0.6, 0.3), "rgb(0, 200, 0)",
     ["+3V3", "+5V_LOGIC", "+5V3_AUX", "VBUS_USB"]),
    ("GND", 0.5, 0.2, (0.6, 0.3), "rgb(120, 120, 120)", ["GND"]),
]

DRU = """(version 1)
# JLCPCB standard 4-layer process: limits KiCad's board setup has no field for.
# Sources: jlcpcb.com/capabilities/pcb-capabilities, checked 2026-11-19.

(rule "JLC: plated pad annular ring >= 0.15 mm"
  (constraint annular_width (min 0.15mm))
  (condition "A.Type == 'Pad' && A.isPlated()"))

(rule "JLC: plated pad hole to copper >= 0.28 mm"
  (constraint hole_clearance (min 0.28mm))
  (condition "A.Type == 'Pad' && A.isPlated()"))

(rule "JLC: pad hole to pad hole >= 0.45 mm"
  (constraint hole_to_hole (min 0.45mm))
  (condition "A.Type == 'Pad' && B.Type == 'Pad'"))

(rule "JLC: NPTH >= 0.5 mm"
  (constraint hole_size (min 0.5mm))
  (condition "A.Type == 'Pad' && !A.isPlated()"))

(rule "JLC: NPTH to copper >= 0.2 mm"
  (constraint hole_clearance (min 0.2mm))
  (condition "A.Type == 'Pad' && !A.isPlated()"))

(rule "JLC: silkscreen text >= 1.0 mm, lines >= 0.15 mm"
  (layer "?.Silkscreen")
  (constraint text_height (min 1.0mm))
  (constraint text_thickness (min 0.15mm)))

# ---- fine-pitch footprints ----------------------------------------------------------
# Their own pads sit 0.15 mm apart, under the 0.2 mm class clearance but inside JLC's 0.127 mm.
(rule "Fine pitch: pad to pad inside VSSOP-8 / USB4085 at JLC minimum"
  (constraint clearance (min 0.127mm))
  (condition "A.Type == 'Pad' && B.Type == 'Pad' && (A.Library_Link == 'Package_SO:VSSOP-8_2.3x2mm_P0.5mm' || A.Library_Link == 'Connector_USB:USB_C_Receptacle_GCT_USB4085') && A.Library_Link == B.Library_Link"))

# ---- hand soldering ---------------------------------------------------------------
# Rules later in the file win, so the general pad rule comes first.
(rule "Hand soldering: 0.5 mm between courtyards (room for the iron)"
  (constraint courtyard_clearance (min 0.5mm)))

(rule "Hand soldering: pads in pours get thermal reliefs"
  (constraint zone_connection thermal_reliefs)
  (constraint thermal_relief_gap (opt 0.5mm))
  (constraint thermal_spoke_width (opt 0.5mm))
  (condition "A.Type == 'Pad'"))

(rule "Hand soldering: power pads connect solid"
  (constraint zone_connection solid)
  (condition "A.Type == 'Pad' && (A.Library_Link == 'Connector_AMASS:*' || A.Library_Link == 'PowerBoard_rev3:XT90*' || A.Library_Link == 'PowerBoard_rev3:DEANS*' || A.Library_Link == 'Package_TO_SOT_SMD:TO-263*' || A.Library_Link == 'Package_TO_SOT_SMD:TO-252*' || A.Library_Link == 'Fuse:*' || A.Library_Link == 'Resistor_SMD:R_2512*' || A.Library_Link == 'PowerBoard_rev3:R_Bourns_CSS2H*' || A.Library_Link == 'PowerBoard_rev3:WireBridge*' || A.Library_Link == 'PowerBoard_rev3:Fuse_*' || A.Library_Link == 'Inductor_SMD:*')"))

(rule "JLC: no blind, buried or micro vias"
  (constraint disallow buried_via micro_via))

# 42 V and 54 V copper: IPC-2221B asks 0.6 mm for bare conductors at 31-150 V (B2) and
# 0.13 mm under solder mask (B4). Pours and wide tracks get 0.5 mm from other nets; pins of
# fine-pitch parts keep the 0.2 mm default (they are masked and the part sets the pitch).
(rule "HV: 0.5 mm around 42 / 54 V pours"
  (layer outer)
  (constraint clearance (min 0.5mm))
  (condition "A.Type == 'Zone' && (A.hasNetclass('HV_Bus') || A.hasNetclass('HV_Power')) && A.Net != B.Net"))

(rule "HV: 0.5 mm around wide 42 / 54 V tracks"
  (layer outer)
  (constraint clearance (min 0.5mm))
  (condition "A.Type == 'Track' && A.Width >= 1mm && (A.hasNetclass('HV_Bus') || A.hasNetclass('HV_Power')) && A.Net != B.Net"))

# Current-carrying nets: no track thinner than the class default once it leaves a pad area.
(rule "Power: tracks >= 0.5 mm"
  (constraint track_width (min 0.5mm))
  (condition "A.Type == 'Track' && (A.hasNetclass('HV_Bus') || A.hasNetclass('HV_Power') || A.hasNetclass('Power'))"))
"""

# ---------------------------------------------------------------- board
STACKUP = """\t\t(stackup
\t\t\t(layer "F.SilkS" (type "Top Silk Screen") (color "White"))
\t\t\t(layer "F.Paste" (type "Top Solder Paste"))
\t\t\t(layer "F.Mask" (type "Top Solder Mask") (color "Green") (thickness 0.01))
\t\t\t(layer "F.Cu" (type "copper") (thickness 0.035))
\t\t\t(layer "dielectric 1" (type "prepreg") (thickness 0.2104) (material "7628") (epsilon_r 4.4) (loss_tangent 0.02))
\t\t\t(layer "In1.Cu" (type "copper") (thickness 0.0152))
\t\t\t(layer "dielectric 2" (type "core") (thickness 1.065) (material "FR4") (epsilon_r 4.6) (loss_tangent 0.02))
\t\t\t(layer "In2.Cu" (type "copper") (thickness 0.0152))
\t\t\t(layer "dielectric 3" (type "prepreg") (thickness 0.2104) (material "7628") (epsilon_r 4.4) (loss_tangent 0.02))
\t\t\t(layer "B.Cu" (type "copper") (thickness 0.035))
\t\t\t(layer "B.Mask" (type "Bottom Solder Mask") (color "Green") (thickness 0.01))
\t\t\t(layer "B.Paste" (type "Bottom Solder Paste"))
\t\t\t(layer "B.SilkS" (type "Bottom Silk Screen") (color "White"))
\t\t\t(copper_finish "HAL lead-free")
\t\t\t(dielectric_constraints no)
\t\t)"""

LAYERS = [(0, "F.Cu", "signal"), (4, "In1.Cu", "power", "GND"), (6, "In2.Cu", "mixed", "PWR"),
          (2, "B.Cu", "signal"),
          (9, "F.Adhes", "user", "F.Adhesive"), (11, "B.Adhes", "user", "B.Adhesive"),
          (13, "F.Paste", "user"), (15, "B.Paste", "user"),
          (5, "F.SilkS", "user", "F.Silkscreen"), (7, "B.SilkS", "user", "B.Silkscreen"),
          (1, "F.Mask", "user"), (3, "B.Mask", "user"),
          (17, "Dwgs.User", "user", "User.Drawings"), (19, "Cmts.User", "user", "User.Comments"),
          (21, "Eco1.User", "user", "User.Eco1"), (23, "Eco2.User", "user", "User.Eco2"),
          (25, "Edge.Cuts", "user"), (27, "Margin", "user"),
          (31, "F.CrtYd", "user", "F.Courtyard"), (29, "B.CrtYd", "user", "B.Courtyard"),
          (35, "F.Fab", "user"), (33, "B.Fab", "user")]


def board_text():
    layers = "\n".join("\t\t(" + " ".join([str(l[0]), f'"{l[1]}"', l[2]] +
                                         ([f'"{l[3]}"'] if len(l) > 3 else [])) + ")" for l in LAYERS)
    return f"""(kicad_pcb
\t(version 20241229)
\t(generator "pcbnew")
\t(generator_version "9.0")
\t(general
\t\t(thickness 1.6)
\t\t(legacy_teardrops no)
\t)
\t(paper "A3")
\t(title_block
\t\t(title "Power board rev3")
\t\t(rev "3.0")
\t\t(company "Brno Mars Rover")
\t\t(comment 1 "JLCPCB standard 4-layer: 1.6 mm, JLC04161H-7628, 1 oz outer / 0.5 oz inner, HASL lead-free")
\t)
\t(layers
{layers}
\t)
\t(setup
{STACKUP}
\t\t(pad_to_mask_clearance 0)
\t\t(allow_soldermask_bridges_in_footprints no)
\t\t(tenting front back)
\t)
\t(net 0 "")
)
"""


def update_project():
    pro = PROJ / f"{NAME}.kicad_pro"
    d = json.loads(pro.read_text())
    board = d.setdefault("board", {})
    ds = board.setdefault("design_settings", {})
    ds.setdefault("meta", {"version": 2})
    ds.setdefault("rules", {}).update(RULES)
    ds["track_widths"] = TRACK_WIDTHS
    ds["via_dimensions"] = [{"diameter": a, "drill": b} for a, b in VIAS]
    ds["diff_pair_dimensions"] = [{"gap": 0.0, "via_gap": 0.0, "width": 0.0}]
    defaults = ds.setdefault("defaults", {})
    defaults.update({"silk_line_width": 0.15, "silk_text_size_h": 1.0, "silk_text_size_v": 1.0,
                     "silk_text_thickness": 0.15, "copper_line_width": 0.2,
                     "copper_text_size_h": 1.5, "copper_text_size_v": 1.5, "copper_text_thickness": 0.3,
                     "board_outline_line_width": 0.1, "courtyard_line_width": 0.05})
    ds["rule_severities"] = {**ds.get("rule_severities", {}), **SEVERITIES}
    ds["teardrop_options"] = [{"td_onpthpad": True, "td_onroundshapesonly": False, "td_onsmdpad": True,
                               "td_ontrackend": False, "td_onvia": True}]
    d["text_variables"] = {**d.get("text_variables", {}), **TEXT_VARIABLES}
    classes, patterns = [], []
    for i, (name, track, clr, (vd, vdr), colour, pats) in enumerate(CLASSES):
        classes.append({"name": name, "track_width": track, "clearance": clr,
                        "via_diameter": vd, "via_drill": vdr,
                        "microvia_diameter": 0.6, "microvia_drill": 0.3,
                        "diff_pair_width": 0.2, "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25,
                        "pcb_color": colour, "schematic_color": "rgba(0, 0, 0, 0.000)",
                        "line_style": 0, "wire_width": 6, "bus_width": 12,
                        "priority": 2147483647 if name == "Default" else i})
        patterns += [{"netclass": name, "pattern": p} for p in pats]
    d["net_settings"] = {"classes": classes, "meta": {"version": 4}, "net_colors": None,
                         "netclass_assignments": None, "netclass_patterns": patterns}
    pro.write_text(json.dumps(d, indent=2) + "\n")


def main():
    update_project()
    (PROJ / f"{NAME}.kicad_dru").write_text(DRU)
    pcb = PROJ / f"{NAME}.kicad_pcb"
    if not pcb.exists():
        pcb.write_text(board_text())
        subprocess.run(KICAD_CLI + ["pcb", "upgrade", "--force", str(pcb)], capture_output=True)
    out = PROJ / "gen" / "out"
    out.mkdir(exist_ok=True)
    r = subprocess.run(KICAD_CLI + ["pcb", "drc", "--format", "json", "--severity-all",
                                    "-o", str(out / "drc.json"), str(pcb)], capture_output=True, text=True)
    print(r.stdout.strip().splitlines()[-1] if r.stdout.strip() else "", r.stderr.strip())
    rep = json.loads((out / "drc.json").read_text())
    kinds = [v["type"] for v in rep.get("violations", [])]
    print("DRC on the empty board:", kinds or "clean")


if __name__ == "__main__":
    main()
