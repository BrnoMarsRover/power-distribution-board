"""Hand-designed placement of a rail eFuse's satellites (TPS48111 on the bottom, under its switch).

The IC sits under the FET drain tabs, where no via fits, so its pins are routed on the bottom layer alone until the
tracks leave the tabs. Every satellite therefore has a set spot, chosen so each pin's route reaches its parts
without crossing another (the pin order around the IC is kept) and every GND pad has room for a via nearby:

  top row, left (pins 11-15, gate drive):
    pin 11 (pre-charge gate) runs left under the bootstrap cap to the gate's escape via; its RC sits above the
    source-via field, the resistor straight above that via and the capacitor right of it (the bottom layer under
    the capacitor stays clear for the bootstrap cap's output via: the output strip's filleted corner leaves no
    spot nearer the cap);
    pins 12 / 13 (BST / SRC) go straight up to the bootstrap cap, upright left of the pins (BST pad below);
    SRC goes on from the cap's top pad to a via into the output strip;
    pins 14 / 15 (PD / PU) leave inward, under the body, and out at its left end to the 10 ohm (PD pad on top);
    PD goes on down the left side through the GATE test pad to the 2.2 ohm, level with the main gate's via.
  top row, right (pins 17-20, sense):
    CS+ (18) straight up to its 100 ohm, under the Kelvin vias; CS- (17 + 19) joins under the body and leaves
    from pin 19 to its Kelvin via;
    VS (20) runs right below the Kelvin pair, past its 100 nF cap at the pin (GND via right of the tabs), then up
    the block's right edge to the 100 ohm feed resistor (VIN pad over the VIN pour, one via) and the 1 uF cap
    above it, in the VIN area.
  bottom row (pins 10-1), a ribbon straight down under the main FET's tab, in pin order:
    pin 10 to the temperature transistor at the tab's lower edge; TMR, IWRN to their parts below the tab, where
    GND vias fit; IMON down past its resistor, cap and test pad (left of the line) towards the ADC; GND (pin 6) to
    its via below the tab; FLT, INP, INPG down left of the bypass bridge, then spread below it with their test pad
    and pull-downs; EN (pin 1) out to the right, to its divider under the VS cap (sharing its GND via). The
    divider's VIN end comes down on L3 from the VIN area: three vias, since the 42 V feed to the aux supply crosses
    the block on L3 just below the VIN pour.
  LED resistor at the right edge, between the LED's anode via and the bridge's output pad; the input-sense divider
  in the VIN area, the output-sense divider below the timer caps; the output test pad below the bypass bridge, on
  the output copper. The lowest parts (the IMON and FLT test pads) end at y = 61.2 mm: the band below belongs to
  the switch-control logic.

Positions are relative to the IC's centre (the 24 V and 12 V blocks have the same geometry). Board mm from the
top-left corner, like the other placement modules.
"""
import json
import os

import pcbnew

from power42 import MM, X0, Y0, _at
from satellites import GAP, Part, _box, _overlap, _pad_xy, _pos

# role: (dx, dy from the IC's centre, rotation; for a test pad, the side its name goes: 0 right, 90 above,
# 180 left, 270 below). Rotations on the bottom side: 0 pad 1 left, 90 pad 1 below, 180 right, -90 above.
SLOTS = {
    # gate drive
    "bst_cap": (-4.9, -7.8, 90),            # BST pad below, SRC pad on top
    "pre_r": (-14.0, -10.8, 90),            # gate pad below, straight above the gate's via
    "pre_c": (-10.1, -11.0, 0),             # RC pad left, GND right (B clear below it for the cap's output via)
    "pu_r": (-4.2, 0.05, 90),               # PU pad below, PD on top
    "gate_r": (-5.75, 6.46, 180),           # PD pad right, main gate left (level with the gate's via)
    "gate_tp": (-6.1, 9.3, 0),
    # sense
    "cs_r": (2.16, -6.62, 180),             # CS+ pad left (above pin 18), Kelvin + right
    "vs_c1": (4.2, -2.7, -90),             # VS pad on top, right at pin 20
    "vs_c2": (6.5, -16.4, 180),             # VS pad right, above the feed resistor
    "vs_r": (5.8, -13.6, 0),                # VIN pad left, over the VIN pour
    # enable / UVLO divider under the VS cap (sharing its GND via), the LED resistor right of it
    "en_bot": (4.17, 1.6, 90),              # EN pad below, GND on top
    "en_top": (4.17, 5.6, 90),              # VIN pad below, EN on top
    "led_r": (8.27, 4.1, 90),               # output pad below, LED anode on top
    # bottom row
    "q_t": (-2.9, 13.2, 90),                # the diode pins (1, 3) up and right, GND (2) lower left
    "tmr_c1": (-7.0, 19.2, -90),            # TMR pads on top
    "tmr_c2": (-3.9, 19.2, -90),
    "tmr_r": (-9.9, 18.7, -90),
    "iwrn_r": (-0.4, 18.9, -90),
    "imon_r": (-0.4, 23.0, 180),            # IMON pad right
    "imon_c": (-0.4, 25.2, 180),
    "imon_tp": (-1.1, 28.9, 180),
    "inpg_r": (3.9, 21.2, -90),             # in line where INPG turns right
    "inp_r": (2.9, 25.2, -90),
    "flt_tp": (2.1, 28.8, 0),
    # sense dividers
    "vin_r": (1.0, -13.7, 0),               # VIN pad left
    "vin_bot": (1.0, -16.0, 180),           # VSNS pad right
    "vin_c": (1.0, -18.3, 180),
    "vout_r": (-9.5, 24.0, 0),              # output pad left
    "vout_bot": (-5.55, 24.0, 0),           # VSNS pad left
    "vout_c": (-9.5, 26.2, 180),
    "out_tp": (7.35, 21.5, 270),           # below the bypass bridge, on the output copper (the band below is the logic's)
}


# per rail: slots that differ from SLOTS (positions are relative to the IC, which every rail block puts at the same
# spot in its own geometry). The 5 V block is L-shaped: only the switch is full width (above y = 34.6 mm); below,
# the block ends at its column (col_r, 5.8 mm right of the IC), and its bypass bridge stands under the main FET.
RAIL_SLOTS = {"5": {
    # the bottom ribbon passes left of the bypass bridge (x 121.3-125.5 from y 46.2): the temperature transistor
    # moves left under the tab, IWRN's resistor sits right of the timer caps, IMON turns left below it to its
    # resistor, cap and test pad, and the output-sense divider stands in a column at the left
    "q_t": (-4.9, 13.2, 90),
    "iwrn_r": (-1.35, 18.9, -90),
    "imon_r": (-4.2, 23.1, 180),
    "imon_c": (-4.2, 25.3, 180),
    "imon_tp": (-4.2, 28.4, 270),
    "vout_r": (-9.7, 23.4, 0),
    "vout_bot": (-9.7, 25.6, 0),
    "vout_c": (-9.7, 27.8, 0),
    # INPG, INP and FLT end below the bridge, in that order from the top (their bus legs leave from there)
    "inpg_r": (2.5, 26.0, 0),
    "inp_r": (2.5, 28.2, 0),
    "flt_tp": (2.2, 31.0, 270),
    # the output test pad at the bottom left, on the wide output copper with a via of its own (above the bridge
    # its way to the output copper cut through the ribbon); the LED resistor in the full-width band right of the
    # VS cap, by the LED's vias
    "out_tp": (-9.3, 30.45, 270),
    "led_r": (6.95, -1.6, -90),
}}


def slots(key):
    """The template for one rail: SLOTS with that rail's changes."""
    return dict(SLOTS, **RAIL_SLOTS.get(key, {}))


def _net(fp, num):
    return next(p.GetNetname() for p in fp.Pads() if p.GetNumber() == num)


def roles(FP, u, r):
    """role -> ref for the satellites of the eFuse u in rail block r (power_rails.RAILS entry), found by net."""
    sheet = u.GetSheetname()
    pn = {p.GetNumber(): p.GetNetname() for p in u.Pads() if p.GetNumber()}
    main_g = _net(FP[r["fets"][1]], "1")
    led_a = next(p.GetNetname() for p in FP[r["led"]].Pads() if p.GetNetname() != "GND")
    vin, out = r["vin"], r["out"]
    num = lambda ref: int(ref.lstrip("ABCDEFGHIJKLMNOPQRSTUVWXYZ") or 0)
    found, rest = {}, []
    parts = sorted((ref for ref, f in FP.items() if f.GetSheetname() == sheet and ref != u.GetReference()
                    and not f.IsLocked()), key=lambda ref: (ref.rstrip("0123456789"), num(ref)))
    for ref in parts:
        f = FP[ref]
        nets = {p.GetNetname() for p in f.Pads()}
        kind = "TP" if ref.startswith("TP") else ref[0]
        has = lambda *ns: all(n in nets for n in ns)
        sense_in = any(n.startswith("VSNS_IN") for n in nets)
        sense_out = any(n.startswith("VSNS_OUT") for n in nets)
        role = None
        if kind == "C" and has(pn["12"]):
            role = "bst_cap"
        elif kind == "R" and has(pn["11"]):
            role = "pre_r"
        elif kind == "R" and has(pn["14"], pn["15"]):
            role = "pu_r"
        elif kind == "R" and has(pn["14"], main_g):
            role = "gate_r"
        elif kind == "TP" and has(pn["14"]):
            role = "gate_tp"
        elif kind == "R" and has(pn["18"]):
            role = "cs_r"
        elif kind == "C" and has(pn["20"]):
            role = "vs_c1" if "vs_c1" not in found else "vs_c2"
        elif kind == "R" and has(pn["20"]):
            role = "vs_r"
        elif kind == "R" and has(pn["1"], vin):
            role = "en_top"
        elif kind == "R" and has(pn["1"], "GND"):
            role = "en_bot"
        elif kind == "Q" and has(pn["10"]):
            role = "q_t"
        elif kind == "C" and has(pn["9"]):
            role = "tmr_c1" if "tmr_c1" not in found else "tmr_c2"
        elif kind == "R" and has(pn["9"]):
            role = "tmr_r"
        elif kind == "R" and has(pn["8"]):
            role = "iwrn_r"
        elif has(pn["7"]):
            role = {"R": "imon_r", "C": "imon_c", "TP": "imon_tp"}.get(kind)
        elif kind == "TP" and has(pn["4"]):
            role = "flt_tp"
        elif kind == "R" and has(pn["3"]):
            role = "inp_r"
        elif kind == "R" and has(pn["2"]):
            role = "inpg_r"
        elif kind == "R" and has(led_a):
            role = "led_r"
        elif sense_in:
            role = "vin_c" if kind == "C" else ("vin_r" if vin in nets else "vin_bot")
        elif sense_out:
            role = "vout_c" if kind == "C" else ("vout_r" if out in nets else "vout_bot")
        elif kind == "TP" and has(out):
            role = "out_tp"
        if role is None or role in found:
            rest.append(ref)
        else:
            found[role] = ref
    # the pre-charge RC's capacitor: on the node behind the gate resistor
    if "pre_r" in found:
        rc = {p.GetNetname() for p in FP[found["pre_r"]].Pads()} - {pn["11"]}
        for ref in list(rest):
            if ref.startswith("C") and rc & {p.GetNetname() for p in FP[ref].Pads()}:
                found["pre_c"] = ref
                rest.remove(ref)
    return found, rest


def label_tp(pt):
    """A placed test pad's name where its state says (right, above, left, below), as the placer does."""
    f = pt.fp
    l, t, rr, b = _box(f)
    v = f.GetField(pcbnew.FIELD_T_VALUE)
    v.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_CENTER)
    px, py = _pad_xy(next(iter(f.Pads())))
    w = (pt.label - 0.8) / 2
    at = {0: (rr + 0.3 + w, py), 180: (l - 0.3 - w, py), 90: (px, t - 0.75), 270: (px, b + 0.75)}[pt.state[2]]
    v.SetPosition(pcbnew.VECTOR2I(MM(X0 + at[0]), MM(Y0 + at[1])))


def place(FP, u, r, obstacles, slots=SLOTS):
    """Put every satellite of u at its slot. Returns (refs placed and locked, problems: parts with no role, and
    courtyard overlaps with each other or with obstacles; the parts are placed regardless)."""
    ux, uy = _pos(u)
    if os.environ.get("HAND_DUMP"):                   # development: the obstacles this template meets
        json.dump([list(o) for o in obstacles], open(os.environ["HAND_DUMP"], "w"))
    found, rest = roles(FP, u, r)
    problems = [f"{ref}: no slot" for ref in rest]
    placed = []
    for role, ref in found.items():
        if role not in slots:
            problems.append(f"{ref}: no slot for {role}")
            continue
        dx, dy, deg = slots[role]
        f = FP[ref]
        pt = Part(f, label=(0.75 * len(f.GetValue()) + 0.8) if ref.startswith("TP") else 0.0)
        pt.state = (ux + dx, uy + dy, deg % 360)
        pt.apply()
        if pt.label:
            label_tp(pt)
        f.SetLocked(True)
        placed.append((role, ref, pt.box()))
    for i, (role, ref, bx) in enumerate(placed):
        for role2, ref2, bx2 in placed[i + 1:]:
            if _overlap(bx, bx2, GAP - 1e-6):
                problems.append(f"{ref} ({role}) overlaps {ref2} ({role2})")
        for o in obstacles:
            if _overlap(bx, o, GAP - 1e-6):
                problems.append(f"{ref} ({role}) overlaps obstacle {tuple(round(v, 2) for v in o)}")
    obstacles += [bx for _, _, bx in placed]
    return [ref for _, ref, _ in placed], problems


def keep_bottom():
    """The rail blocks placed by this template, reserved on the bottom for their own parts: from above the
    input-sense divider to below the lowest test pads, across the block (L-shaped where the block is). Other sheets' parts (later templates,
    the logic groups, the packer, the optimizer) stay out, so they cannot land in the cluster's routes."""
    import power_rails
    import satellites
    out = []
    for k in sorted(satellites.HAND):
        r = power_rails.RAILS[k]
        g = power_rails._geo(r)
        if g["col_r"] < r["xr"]:                      # L-shaped block: full width only above the band
            out += [(r["xl"], 11.4, r["xr"], g["band_bot"]), (r["xl"], g["band_bot"], g["col_r"], 61.6)]
        else:
            out.append((r["xl"], 11.4, r["xr"], 61.6))
    return out
