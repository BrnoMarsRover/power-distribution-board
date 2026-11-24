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


# ---- 42 V blocks (power42.py): the IC sits on the bottom under the BUS42 bar, so the VS, ISCP and EN resistors
# take BUS42 through vias right beside them; nothing else can have a via anywhere in the block (its copper is 42 V)
IC42_DX, IC42_Y = 2.2, 33.0                  # IC centre: right of the FET centre line, in the middle of the bar
SLOTS42 = {   # relative to the IC (42D: (15.0, 33.0)); 42C is the same block shifted 25 mm
    # gate drive: G, SRC, PD and PU leave under the body to the left; BST and SRC also go up to the bootstrap caps,
    # upright side by side above the bar, left of the input electrolytics' holes
    "bst_cap": (-5.0, -13.8, 90),           # BST pad below, SRC on top
    "bst_cap2": (-9.0, -13.8, 90),
    "pre_r": (-13.0, 1.5, 90),              # gate pad below, straight above the pre-charge gate's via
    "pre_c": (-9.1, 0.4, 0),                # RC pad left, GND right
    "pu_r": (-4.4, 5.5, 90),                # PU pad below, PD on top; PD then runs down the left edge
    "gate_tp": (-3.85, 11.5, 90),
    "gate_r": (-4.6, 18.0, -90),            # PD pad on top, the first main gate below (its via level)
    "gate_r2": (-4.6, 30.0, -90),           # the second main gate
    # sense: the CS pair goes up and right above everything and down the block's right edge to the Kelvin vias
    # (CS- on the outside); ISCP and VS turn down right of the IC to their BUS42-fed resistors (vias into the bar)
    "iscp_c": (4.2, -0.6, -90),             # at the right end of the IC: ISCP on top, CS- below
    "cs_r": (8.9, -2.4, 90),                # Kelvin + below (to its via), CS+ on top
    "iscp_r": (6.4, -2.8, -90),             # ISCP on top, BUS42 below (CS+ passes over it)
    "vs_r": (6.4, 1.2, -90),                # VS on top, BUS42 below
    "vs_c1": (4.2, 3.4, -90),               # 100 nF at the pin, VS on top
    # the bus legs (IMON, FLT, INP, INPG; route_plan.corridor42) run straight down a lane under their pins to the
    # bus band at the block's foot: the temperature transistor, IWRN's resistor and the timer parts stack in a
    # column on its left (TMR passes right of the transistor), the rest stays right of it.
    # the bottom-row lines run straight down beside the bypass bridge; below it, the temperature transistor and
    # the timer / IWRN parts stack on the left, the VS bulk cap, the EN divider and the motor-switch FET, the LED
    # resistor, the output test pad and the output divider's top resistor on the right (GND vias in the drain
    # copper's right column). IMON, INP / INPG and FLT only pass through: their parts sit at the far end.
    # timer column (timer42.py, 2026-11-24): the temperature transistor 0.9 mm west, clear of TMR's run; both
    # timer caps upright side by side below the second gate resistor (TMR pads on top), the 100k flat under them
    # (TMR pad east), all west of TMR, so every GND pad reaches a via below the tab or in the strip gap
    "q_t": (-2.9, 23.6, 90),
    "tmr_c1": (-5.6, 34.75, -90),
    "tmr_c2": (-2.6, 34.75, -90),
    "tmr_r": (-3.4, 39.2, 180),
    "iwrn_r": (-0.7, 15.4, -90),            # first in the left column; temperature and TMR pass on its left
    "vs_c2": (4.8, 24.0, -90),
    "en_top": (7.4, 23.3, -90),             # BUS42 on top, EN below
    "en_bot": (8.0, 27.5, -90),             # EN on top, GND below
    "q_en": (4.6, 29.3, 90),
    "led_r": (6.0, 34.4, -90),              # output on top, LED anode below (level with its via)
    "out_tp": (6.15, 39.9, 180),             # clear of the bus band below; its label on its left, over the lane
    "vout_r": (8.8, 39.3, -90),             # output on top, VSNS below: straight down into the bus band
    # at the far end, next to the ICs their nets feed (logic_place.py: the switch-control group): the IMON
    # resistor and cap at the ADC, the INP / INPG pull-downs at the I/O expander, the output divider's lower half
    # at the ADC, the FLT and IMON test pads in the test-pad field (decided 2026-11-21: no room in the block)
    "imon_r": None, "imon_c": None, "imon_tp": None, "flt_tp": None, "inp_r": None, "inpg_r": None,
    "vout_bot": None, "vout_c": None,
}


SLOTS42_KEY = {"42C": {
    # 42C has no room above: the input ceramics and the battery shunt's Kelvin pair are there. Its bootstrap caps
    # stand upright left of the IC instead (BST on top, fed over the top; the output pads below, where SRC comes
    # in from under the body); the pre-charge gate runs left between the caps' pads, and its RC sits above the
    # output via field, next to the gate's via
    "bst_cap": (-6.75, -1.075, -90),
    "bst_cap2": (-11.05, -1.075, -90),      # 1 mm apart: the pre-charge gate goes down between them
    "pre_r": (-10.7, 6.6, -90),             # upright beside the gate's via: gate pad on top, RC below
    "pre_c": (-6.65, 7.6, -90),              # RC pad on top, GND below, clear of the field's top via row
    "pu_r": (-4.05, 5.85, 90),              # 0.35 mm right of and lower than 42D's: PD runs down between it and
                                            # the RC cap
    "gate_tp": (-3.4, 12.0, 90),            # its label's room clear of the RC cap; 0.2 mm west of and 0.5 mm
                                            # below 42D's spot, so TEMP's straight run passes on its right
    # above 42C's band, where 42D's EN switch sits (42C has none): the output test pad and the output divider,
    # whose VSNS pad goes straight down into the band, its output pad up to a via through the drain copper
    "out_tp": (4.0, 30.0, 90),
    "vout_r": (3.2, 34.4, -90),
    # the CS+ filter 0.35 mm left of 42D's: CS- comes down between it and the INA228's link (ina_link)
    "cs_r": (8.55, -2.4, 90),
}}


def slots42(key):
    """The 42 V template for one block: SLOTS42 with that block's changes."""
    return dict(SLOTS42, **SLOTS42_KEY.get(key, {}))


def keep_bottom42():
    """The 42 V blocks, reserved on the bottom like the rails (keep_bottom): from the bar band down to the end of
    the output via fields, across the block (42D with the channel on its right, where the BUS42 feed for the parts
    below the bar runs), and above the bar where 42D's bootstrap caps stand, with their way down to the IC."""
    import power42
    out = []
    for key, blk in sorted(power42.BLOCKS.items()):
        right = {"42D": 25.8, "42C": 50.8}[key]       # 42D with its channel up to 42C; 42C up to the 24 V block
        out.append((blk["out_l"], 28.5, right, 76.4))    # down to the XT90s: the bus band
        s, (ix, iy) = slots42(key), ic42(key)
        up = [s[k] for k in ("bst_cap", "bst_cap2") if s.get(k) and s[k][1] < -5]
        if up:
            out.append((ix + min(d[0] for d in up) - 2.0, iy + min(d[1] for d in up) - 2.8, ix + 1.0, 28.5))
    return out


def block42(key):
    """A 42 V block (power42.BLOCKS) described like a rail block for roles(): its input is the BUS42 bar."""
    import power42
    return dict(power42.BLOCKS[key], vin="BUS42", out={"42D": "OUT42D", "42C": "OUT42C"}[key])


def ic42(key):
    import power42
    return power42.FET_CX + power42.BLOCKS[key]["dx"] + IC42_DX, IC42_Y


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
    main_g2 = _net(FP[r["fets"][2]], "1") if len(r["fets"]) > 2 else None   # 42 V: two main FETs
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
            role = "bst_cap" if "bst_cap" not in found else "bst_cap2"
        elif pn["19"] != pn["17"] and has(pn["19"]) and kind in ("R", "C"):   # ISCP of its own (42 V)
            role = "iscp_r" if kind == "R" else "iscp_c"
        elif kind == "R" and has(pn["11"]):
            role = "pre_r"
        elif kind == "R" and has(pn["14"], pn["15"]):
            role = "pu_r"
        elif kind == "R" and has(pn["14"], main_g):
            role = "gate_r"
        elif kind == "R" and main_g2 and has(pn["14"], main_g2):
            role = "gate_r2"
        elif kind == "Q" and has(pn["1"]):                                    # EN pull-down (motor switch)
            role = "q_en"
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
        if slots[role] is None:                       # placed elsewhere on purpose (see the template)
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
