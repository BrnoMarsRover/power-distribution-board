#!/usr/bin/env python3
"""Rev3 rail switches: component values, per-output PTCs and the I/O map.

Run `python3 rails.py > rails.md` after changing anything here.

Architecture (decided 2026-11-18): one eFuse per rail (TPS48111-Q1 + external FETs +
shunt), plus one resettable PTC per 24/12/5 V output connector. The 15 V rail was dropped,
and a 54 V supply for the Zyxel XMG-105HP PoE switch was added: an eFuse on the battery bus
feeding an on-board LM5122 boost converter. The 42 V outputs are split into a drive group
and a converter-feed group, each with its own eFuse.

The board is hand-soldered, so every part is leaded: D2PAK / DPAK FETs, no QFN, DFN, SON or
TOLL packages. Every eFuse value is
computed from the TPS4811-Q1 datasheet (SLUSEE5E) equation cited next to it.

Values marked ASSUMED are guesses until the real loads are known.
"""

import math

E96 = [1.00, 1.02, 1.05, 1.07, 1.10, 1.13, 1.15, 1.18, 1.21, 1.24, 1.27, 1.30, 1.33,
       1.37, 1.40, 1.43, 1.47, 1.50, 1.54, 1.58, 1.62, 1.65, 1.69, 1.74, 1.78, 1.82,
       1.87, 1.91, 1.96, 2.00, 2.05, 2.10, 2.15, 2.21, 2.26, 2.32, 2.37, 2.43, 2.49,
       2.55, 2.61, 2.67, 2.74, 2.80, 2.87, 2.94, 3.01, 3.09, 3.16, 3.24, 3.32, 3.40,
       3.48, 3.57, 3.65, 3.74, 3.83, 3.92, 4.02, 4.12, 4.22, 4.32, 4.42, 4.53, 4.64,
       4.75, 4.87, 4.99, 5.11, 5.23, 5.36, 5.49, 5.62, 5.76, 5.90, 6.04, 6.19, 6.34,
       6.49, 6.65, 6.81, 6.98, 7.15, 7.32, 7.50, 7.68, 7.87, 8.06, 8.25, 8.45, 8.66,
       8.87, 9.09, 9.31, 9.53, 9.76]
E12 = [1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2]
E6 = [1.0, 1.5, 2.2, 3.3, 4.7, 6.8]


def nearest(value, series):
    decade = 10 ** math.floor(math.log10(value))
    return min([s * decade for s in series] + [series[0] * decade * 10],
               key=lambda c: abs(c - value))


def ceil_e(value, series):
    """Smallest series value >= value (for minimums like a bootstrap capacitor)."""
    decade = 10 ** math.floor(math.log10(value))
    for s in [x * decade for x in series] + [series[0] * decade * 10]:
        if s >= value * 0.999:
            return s


def fmt_r(ohms):
    for unit, scale in (("M", 1e6), ("k", 1e3)):
        if ohms >= scale:
            return f"{ohms / scale:.3g}{unit}"
    return f"{ohms:.3g}R"


def fmt_c(farads):
    for unit, scale in (("mF", 1e-3), ("uF", 1e-6), ("nF", 1e-9), ("pF", 1e-12)):
        if farads >= scale:
            return f"{farads / scale:.3g}{unit}"
    return f"{farads:.3g}F"


ADC_FULL_SCALE_V = 2.4   # headroom below the ADC128D818's internal 2.56 V reference
R_SET = 100.0            # TPS4811 characterisation value
I_TMR = 82e-6            # TMR pull-up current
R_TMR = 100e3            # across C_TMR -> latch-off mode (section 8.3.3.2)
I_G = 100e-6             # pre-charge gate pull-up current, typical (72-140 uA)

FET_QG = {"IPB042N10N3G": 117e-9, "FDD86102": 19e-9}   # total gate charge at 10 V

# FETs: name, R_DS(on) max at 10 V in ohms, JLC part, description. Leaded packages only.
FETS = {
    "IPB042N10N3G": (4.2e-3, "C69300", "100 V, D2PAK, Qg 117 nC"),
    "FDD86102": (24e-3, "C47711", "100 V, DPAK, Qg 19 nC"),
}

# PTCs: name, V max, I hold, I trip, R initial min, JLC part.
# (V max, I hold, I trip, R min, maker, package) from the Bourns MF-R / MF-LSMF datasheets.
# Hold current falls to about 2.5 A at 50 degC and 2.25 A at 60 degC (MF-LSMF derating table).
PTCS = {
    "MF-R300": (30, 3.0, 6.0, 0.020, "Bourns", "radial THT"),
    "MF-LSMF300/16X-2": (16, 3.0, 5.0, 0.015, "Bourns", "2920"),
    "MF-LSMF300X-2": (6, 3.0, 5.0, 0.015, "Bourns", "2920"),
}

# One entry per switched rail.
#   design: continuous current the input connector and copper are sized for
#   i_oc / t_oc: over-current trip (IWRN) and its delay; latch-off
#   i_sc: fast short-circuit trip (ISCP), or None to disable it (rails behind a
#         current-limited converter, so a hot-plug spike cannot trip the rail)
#   c_out, n_out: ASSUMED capacitance per output, and the output count
RAILS = [
    dict(name="42V-D", v=42.0, design=30.0, i_oc=60.0, t_oc=1.0, i_sc=150.0,
         r_sns=0.5e-3, fet="IPB042N10N3G", n_main=2, c_out=940e-6, n_out=4, conn="XT90 (drives)",
         i_inrush=0.4, ptc=None, src="battery bus", default_on=False,
         note="Drive group: EPOS4 50/5 (two per connector, two connectors in use, two spare; "
              "ASSUMED 470 uF per drive) or CubeMars AK10-9 V3.0 actuators. The 60 A / 1.1 s trip sits "
              "above four EPOS4s accelerating (15 A each); CubeMars actuators (31.9 A peak) need a "
              "configured current limit. The rear-panel motor switch feeds this driver's EN/UVLO "
              "through the latch, so opening it cuts motor power in hardware without touching the Orin. "
              "Defaults OFF: the drives are useless without a controller."),
    dict(name="42V-C", v=42.0, design=30.0, i_oc=30.0, t_oc=0.5, i_sc=100.0,
         r_sns=1e-3, fet="IPB042N10N3G", n_main=2, c_out=470e-6, n_out=4, conn="XT90 (converters)",
         i_inrush=0.4, ptc=None, src="battery bus", default_on=True,
         note="Converter-feed group: three feeds plus one spare. 24 V 300 W + 12 V 150 W + 5 V 150 W "
              "is about 22 A at 30 V; the 30 A design leaves room for a converter on the spare. "
              "Defaults ON, because the 12 V converter powers the Orin."),
    dict(name="24V", v=24.0, design=10.0, i_oc=11.0, t_oc=0.5, i_sc=None,
         r_sns=2e-3, fet="IPB042N10N3G", n_main=1, c_out=470e-6, n_out=8, conn="Deans T",
         i_inrush=1.0, ptc="MF-R300", src="300 W converter", default_on=False),
    dict(name="12V", v=12.0, design=10.0, i_oc=11.0, t_oc=0.5, i_sc=None,
         r_sns=2e-3, fet="IPB042N10N3G", n_main=1, c_out=470e-6, n_out=7, conn="XT60",
         i_inrush=2.0, ptc="MF-LSMF300/16X-2", src="150 W converter", default_on=True,
         note="Feeds the Jetson AGX Orin, so it defaults ON in hardware."),
    dict(name="5V", v=5.0, design=15.0, i_oc=16.5, t_oc=0.5, i_sc=None,
         r_sns=1e-3, fet="IPB042N10N3G", n_main=1, c_out=220e-6, n_out=11, conn="XT30",
         i_inrush=2.0, ptc="MF-LSMF300X-2", src="150 W converter, XT30-limited",
         default_on=False),
    dict(name="54V", v=42.0, design=4.0, i_oc=5.0, t_oc=0.5, i_sc=20.0,
         r_sns=3e-3, fet="IPB042N10N3G", n_main=1, c_out=600e-6, n_out=1, conn="Zyxel XMG-105HP supply",
         i_inrush=0.4, ptc=None, src="battery bus, then the LM5122 boost to 54 V",
         default_on=True,
         note="The eFuse sits on the boost's 42 V input. It switches the boost and is its "
              "short-circuit protection, because a boost cannot block current from input to output. "
              "Defaults ON: the Zyxel is the rover's main network switch. "
              "C_LOAD includes the boost's output side and the Zyxel's input: ASSUMED 600 uF."),
]


# Power-on: every eFuse comes up OFF in hardware (pull-downs on INP / INP_G). "default_on" rails are
# switched on by the ESP32 at boot, in order, each step confirmed on VSNS_OUT. The bootstrap cap
# sets the earliest moment a gate driver can act.
I_BST_TYP, V_BST_UVLOR_TYP = 100e-6, 7.6


def rail_values(r):
    """Numeric component values for one rail eFuse (TPS4811-Q1 equations, SLUSEE5E)."""
    v = dict(r_set=R_SET, r_sns=r["r_sns"], n=r["n_main"])
    v["v_wrn"] = r["i_oc"] * r["r_sns"]
    # Eq. 6: R_IWRN = 11.9 * R_SET / V_SNS(WRN)
    v["r_iwrn"] = nearest(11.9 * R_SET / v["v_wrn"], E96)
    v["i_oc"] = 11.9 * R_SET / v["r_iwrn"] / r["r_sns"]
    # Eq. 10 (latch-off, R_TMR across C_TMR): C_TMR = t_OC / (R_TMR * ln(1 / (1 - 1.2 / (R_TMR * I_TMR))))
    v["k_tmr"] = R_TMR * math.log(1 / (1 - 1.2 / (R_TMR * I_TMR)))
    v["c_tmr"] = nearest(r["t_oc"] / v["k_tmr"], E12)
    v["r_tmr"] = R_TMR
    # Eq. 11: R_ISCP = I_SC * R_SNS / 15.6 uA - 464; None = ISCP tied to CS- (disabled)
    if r["i_sc"]:
        v["r_iscp"] = nearest(r["i_sc"] * r["r_sns"] / 15.6e-6 - 464, E96)
        v["i_sc"] = (v["r_iscp"] + 464) * 15.6e-6 / r["r_sns"]
    else:
        v["r_iscp"] = v["i_sc"] = None
    # Eq. 12/13: V_IMON = V_SNS * 0.9 * R_IMON / R_SET, full scale at 1.25 x I_OC
    v["i_fs"] = 1.25 * v["i_oc"]
    v["r_imon"] = nearest(ADC_FULL_SCALE_V * R_SET / (0.9 * v["i_fs"] * r["r_sns"]), E96)
    v["mv_per_a"] = 0.9 * v["r_imon"] / R_SET * r["r_sns"] * 1e3
    # Eq. 5: pre-charge gate slew, C_g = C_LOAD * I_G / I_INRUSH
    v["c_load"] = r["c_out"] * r["n_out"]
    v["c_g"] = nearest(v["c_load"] * I_G / r["i_inrush"], E12)
    v["i_inrush"] = v["c_load"] * I_G / v["c_g"]
    v["t_ramp"] = v["c_load"] * r["v"] / v["i_inrush"]
    v["energy"] = 0.5 * v["c_load"] * r["v"] ** 2
    v["rds"], v["fet_lcsc"], v["fet_desc"] = FETS[r["fet"]]
    # Eq. 4/20: C_BST > Qg(total) / 1 V + 10 x C_g
    v["c_bst"] = ceil_e(FET_QG[r["fet"]] * (r["n_main"] + 1) / 1.0 + 10 * v["c_g"], E6)
    v["t_bst"] = v["c_bst"] * V_BST_UVLOR_TYP / I_BST_TYP      # typical: gate driver ready after EN
    return v


def design_rail(r):
    v = rail_values(r)
    n = v["n"]
    iscp = (f"{fmt_r(v['r_iscp'])} -> {v['i_sc']:.0f} A in 1.2 us" if v["r_iscp"]
            else "ISCP tied to CS- (disabled): converter-limited, survives hot-plug spikes")
    return [
        ("Feeds", f"{r['n_out']} x {r['conn']}, from {r['src']}"),
        ("Power-on default", "OFF in hardware; **ESP32 turns it on at boot**" if r["default_on"] else "OFF"),
        ("Design current", f"{r['design']:g} A"),
        ("Main FET", f"{n} x {r['fet']} ({v['fet_desc']}){' in parallel' if n > 1 else ''}, "
                     f"{r['design'] ** 2 * v['rds'] / n:.2f} W total at design current"),
        ("Pre-charge FET", f"{r['fet']} (same part)"),
        ("R_SNS", f"{r['r_sns'] * 1e3:g} mOhm, {r['design'] ** 2 * r['r_sns']:.2f} W at design current, "
                  f"{v['v_wrn'] * 1e3:.0f} mV at trip"),
        ("R_SET", fmt_r(R_SET)),
        ("R_IWRN", f"{fmt_r(v['r_iwrn'])} -> trips at {v['i_oc']:.1f} A"),
        ("C_TMR (R_TMR 100k, latch-off)", f"{fmt_c(v['c_tmr'])} -> {v['c_tmr'] * v['k_tmr'] * 1e3:.0f} ms"),
        ("R_ISCP", iscp),
        ("R_IMON", f"{fmt_r(v['r_imon'])} -> {v['mv_per_a']:.0f} mV/A, {ADC_FULL_SCALE_V:g} V at {v['i_fs']:.0f} A"),
        ("C_g / R_g (pre-charge slew)", f"{fmt_c(v['c_g'])} / 330R -> {v['i_inrush']:.1f} A for {v['t_ramp'] * 1e3:.0f} ms"),
        ("Pre-charge FET stress", f"{v['energy']:.2f} J, {r['v'] * v['i_inrush']:.0f} W peak (ASSUMED "
                                  f"{fmt_c(v['c_load'])} load) - check its SOA"),
        ("C_BST", f"{fmt_c(v['c_bst'])} (>= Qg(main + pre-charge) / 1 V + 10 x C_g)"),
        ("Power-on timing", power_on_text(r, v)),
    ]


def power_on_text(r, v):
    if not r["default_on"]:
        return "OFF until the PC asks"
    return (f"ON at boot (ESP32): driver ready ~{v['t_bst']:.2f} s after EN, "
            f"pre-charge ~{v['t_ramp'] * 1e3:.0f} ms, then INP once VSNS_OUT reaches the input")


BOOST = dict(vin_min=30.0, vin_max=42.0, vout=54.0, pout=100.0, eff=0.95, fsw=400e3,
             inductor=("MSS1583-223MED", 22e-6, 10.4, 0.026, "Coilcraft"),
             fet="FDD86102", qg_at_vcc=14e-9, c_cer=2.2e-6, n_cer=6, cer_derate=0.45,
             c_bulk=100e-6, n_bulk=2, esr_bulk=0.18,     # EEU-FC2A101: 0.18 ohm impedance at 100 kHz
             a_s=10, k_slope=0.85, f_cross=8e3)


def boost_values(b):
    """LM5122 loop values (SNVS954H eq. 5-10, 16-18). Current mode, Type 2 network between COMP and FB."""
    L, rs = b["inductor"][1], 0.01
    v = {}
    v["r_top"] = nearest(4.99e3 * (b["vout"] / 1.2 - 1), E96)
    v["vout"] = 1.2 * (1 + v["r_top"] / 4.99e3)
    d_lo = b["vin_min"] / v["vout"]                        # D' at minimum input
    d_hi = b["vin_max"] / v["vout"]
    # slope: R_SLOPE = L * 6e9 / ((K * VOUT - VIN_min) * RS * A_S), then K back over the input range
    v["r_slope"] = nearest(L * 6e9 / ((b["k_slope"] * v["vout"] - b["vin_min"]) * rs * b["a_s"]), E96)
    k = lambda vin: (1 + L * 6e9 / (vin * rs * b["a_s"] * v["r_slope"])) * vin / v["vout"]
    v["k_lo"], v["k_hi"] = k(b["vin_min"]), k(b["vin_max"])
    v["r_slope_min"] = 5.7e9 / b["fsw"] * (1.2 - b["vin_min"] / v["vout"])
    # power stage at full load
    r_load = v["vout"] ** 2 / b["pout"]
    c_out = b["n_cer"] * b["c_cer"] * b["cer_derate"] + b["n_bulk"] * b["c_bulk"]
    v["c_out"] = c_out
    v["f_rhp"] = r_load * d_lo ** 2 / (2 * math.pi * L)
    v["f_load"] = 2 / (2 * math.pi * r_load * c_out)
    v["f_esr"] = 1 / (2 * math.pi * (b["esr_bulk"] / b["n_bulk"]) * (b["n_bulk"] * b["c_bulk"]))
    # eq. 18: f_cross = R_COMP * D' / (pi * RS * R_FB2 * A_S * C_OUT); D' smallest at VIN_min
    v["r_comp"] = nearest(b["f_cross"] * math.pi * rs * v["r_top"] * b["a_s"] * c_out / d_lo, E96)
    v["c_comp"] = nearest(1 / (2 * math.pi * v["r_comp"] * v["f_load"]), E12)        # zero on the load pole
    v["c_hf"] = nearest(1 / (2 * math.pi * v["r_comp"] * v["f_esr"]), E12)            # pole on the ESR zero
    fc = lambda dp: v["r_comp"] * dp / (math.pi * rs * v["r_top"] * b["a_s"] * c_out)
    v["fc_lo"], v["fc_hi"] = fc(d_lo), fc(d_hi)
    return v


def design_boost(b):
    """54 V supply for the Zyxel XMG-105HP. LM5122 datasheet SNVS954H."""
    name, L, isat, dcr, code = b["inductor"]
    d_max = 1 - b["vin_min"] * b["eff"] / b["vout"]
    d_min = 1 - b["vin_max"] * b["eff"] / b["vout"]
    i_in = b["pout"] / (b["vin_min"] * b["eff"])
    ripple = b["vin_min"] * d_max / (b["fsw"] * L)
    i_pk = i_in + ripple / 2
    # Eq. 4: RT = 9e9 / fSW
    rt = nearest(9e9 / b["fsw"], E96)
    # cycle-by-cycle limit: V_CS-TH1 = 65.5 / 75 / 87.5 mV (min / typ / max)
    r_s = nearest(0.0655 / (1.4 * i_pk), E12)
    lim = [v / r_s for v in (0.0655, 0.075, 0.0875)]
    # V_OUT = 1.2 V * (1 + R_top / R_bot)
    bv = boost_values(b)
    r_bot = 4.99e3
    r_top, vout = bv["r_top"], bv["vout"]
    c_eff = b["n_cer"] * b["c_cer"] * b["cer_derate"]
    i_out = b["pout"] / b["vout"]
    v_ripple = i_out * d_max / (b["fsw"] * c_eff)
    rows = [
        ("Controller", "LM5122MHX/NOPB, synchronous boost, 65 V in (75 V abs max on VIN/CSP/CSN), 105 V switch node; "
                       "MODE grounded = diode emulation, so no reverse current into the battery"),
        ("Operating point", f"{b['vin_min']:g}-{b['vin_max']:g} V in -> {b['vout']:g} V, {b['pout']:g} W "
                            f"({i_out:.2f} A); the XMG-105HP draws 90 W at most, 70 W of it PoE"),
        ("Duty cycle", f"{d_min:.2f} - {d_max:.2f} (below 50 %, so slope compensation is not critical)"),
        ("Switching frequency", f"{b['fsw'] / 1e3:.0f} kHz, RT = {fmt_r(rt)}"),
        ("Inductor", f"{name} ({code}), {L * 1e6:g} uH, Isat {isat:g} A (above the {0.0875 / 0.01:.2f} A max "
                     f"current limit), DCR {dcr * 1e3:g} mOhm, "
                     f"{i_in ** 2 * dcr:.2f} W"),
        ("Input current", f"{i_in:.2f} A average at {b['vin_min']:g} V, ripple {ripple:.2f} A p-p, peak {i_pk:.2f} A"),
        ("Current sense R_S", f"{r_s * 1e3:g} mOhm -> cycle-by-cycle limit {lim[0]:.1f} / {lim[1]:.1f} / "
                              f"{lim[2]:.1f} A (min / typ / max), below Isat"),
        ("FETs", f"{b['fet']} ({FETS[b['fet']][2]}) for both low and high side. Low gate charge matters: "
                 f"the LM5122 drives both gates from its internal VCC regulator, fed from VIN. "
                 f"That is {2 * b['qg_at_vcc'] * b['fsw'] * 1e3:.0f} mA, or "
                 f"{2 * b['qg_at_vcc'] * b['fsw'] * b['vin_max']:.2f} W in the LM5122 at {b['vin_max']:g} V in. "
                 f"A D2PAK (117 nC) would burn about 3 W."),
        ("Feedback", f"R_top {fmt_r(r_top)} / R_bot {fmt_r(r_bot)} -> {vout:.2f} V"),
        ("Output capacitors", f"{b['n_cer']} x {fmt_c(b['c_cer'])} 100 V X7R 1210 (~{fmt_c(c_eff)} effective at "
                              f"{b['vout']:g} V) + {b['n_bulk']} x {fmt_c(b['c_bulk'])} / 100 V EEU-FC2A101; "
                              f"ripple ~{v_ripple * 1e3:.0f} mV"),
        ("Slope compensation", f"R_SLOPE {fmt_r(bv['r_slope'])} -> K = {bv['k_lo']:.2f} at {b['vin_min']:g} V, "
                               f"{bv['k_hi']:.2f} at {b['vin_max']:g} V (target >= 0.82; minimum R_SLOPE "
                               f"{fmt_r(bv['r_slope_min'])})"),
        ("Loop compensation (COMP to FB)", f"R_COMP {fmt_r(bv['r_comp'])} + C_COMP {fmt_c(bv['c_comp'])} in series, "
                                           f"C_HF {fmt_c(bv['c_hf'])} across. Crossover {bv['fc_lo'] / 1e3:.1f}-"
                                           f"{bv['fc_hi'] / 1e3:.1f} kHz; RHP zero {bv['f_rhp'] / 1e3:.0f} kHz at "
                                           f"{b['vin_min']:g} V; zero on the load pole ({bv['f_load']:.0f} Hz), "
                                           f"HF pole on the bulk-cap ESR zero ({bv['f_esr'] / 1e3:.1f} kHz). "
                                           "Confirm with a load-step test."),
        ("Output TVS", "SMBJ58A-13-F: standoff 58 V, above the 57 V PoE maximum; clamps at 93.6 V, under the "
                       "105 V switch node and the FETs' 100 V"),
        ("Bootstrap diode", "BAS21 (200 V): it blocks VOUT plus switching ringing, more than a 75-100 V 1N4148W should see"),
        ("Enable", "UVLO is held low by Q_BEN until firmware clears BOOST_DIS (expander P15), after the 54 V eFuse "
                   "is fully on. The boost never runs off its eFuse's pre-charge FET"),
        ("Over-voltage", "No OVP in the LM5122. Firmware reads VOUT 54V and opens the input eFuse "
                         "above 57 V; the TVS covers fast transients"),
        ("Output connector", "A type used for nothing else on the board, so a 54 V plug cannot reach a 5 / 12 / 24 V device"),
    ]
    return rows


FLT_GPIOS = [6, 7, 8, 9, 16, 38]
ADC_ADDRS = [0x1D, 0x1E]


def main():
    out = []
    w = out.append
    w("# Rev3 rail switches\n")
    w("Generated by `rails.py`. Do not edit by hand; change the script and re-run it.\n")
    w("One TPS48111-Q1 eFuse per rail (two for the 42 V outputs), with a resettable PTC on each "
      "24 / 12 / 5 V output. All parts are leaded for hand soldering. "
      "Load capacitances are **assumed** until the real loads are known.\n")

    w("## eFuse values per rail\n")
    designs = {r["name"]: design_rail(r) for r in RAILS}
    names = [r["name"] for r in RAILS]
    w("| | " + " | ".join(names) + " |")
    w("|---|" + "---|" * len(names))
    for i, (label, _) in enumerate(designs[names[0]]):
        w(f"| {label} | " + " | ".join(designs[n][i][1] for n in names) + " |")
    w("")
    for r in RAILS:
        if r.get("note"):
            w(f"- **{r['name']}:** {r['note']}")
    w("")

    w("## Power-on sequence (ESP32)\n")
    w("Every eFuse powers up OFF (pull-downs on INP and INP_G). The ESP32 runs from the always-on aux "
      "supply, boots in about 0.3-0.5 s and brings the boot rails up in order, moving on as soon as "
      "VSNS_OUT shows that a step has finished:\n")
    vals = {r["name"]: rail_values(r) for r in RAILS}
    w("| Step | Typical |\n|---|---|")
    w("| ESP32 boots, reads every VSNS_OUT and FLT | 0.3-0.5 s |")
    for name in ("42V-C", "12V", "54V"):
        vv = vals[name]
        w(f"| {name}: INP_G on; driver ready after ~{vv['t_bst']:.2f} s; pre-charge ~{vv['t_ramp'] * 1e3:.0f} ms; "
          f"INP on once VSNS_OUT is within 5 % of the input; INP_G off | |")
    w("| Orin powered | about 1-1.5 s after the battery connects |")
    w("")
    w("The 12 V eFuse starts only once the converter group is fully on, so its pre-charge never draws "
      "through the converter group's pre-charge FET. If a step does not finish in its window (pre-charge "
      "slower than 4x the expected time, or FLT asserts), firmware leaves that rail off and reports it.\n")

    w("## 54 V boost for the Zyxel XMG-105HP\n")
    w("| | |\n|---|---|")
    for k, v in design_boost(BOOST):
        w(f"| {k} | {v} |")
    w("")

    w("## Per-output PTCs\n")
    w("A PTC protects the cable to one device from an overload too small to trip the rail. "
      "Each output's cable must carry the PTC's trip current continuously. Fit a bigger "
      "PTC on any output whose device needs more than the hold current.\n")
    w("| Rail | Outputs | PTC (maker, package) | V max | Hold / trip | Drop at hold | Cable at least |")
    w("|---|---|---|---|---|---|---|")
    for r in RAILS:
        if not r["ptc"]:
            continue
        vmax, ihold, itrip, rmin, code, pkg = PTCS[r["ptc"]]
        cable = "0.5 mm2 / AWG 20" if itrip <= 6 else "0.75 mm2 / AWG 18"
        w(f"| {r['name']} | {r['n_out']} x {r['conn']} | {r['ptc']} ({code}, {pkg}) | {vmax} V | "
          f"{ihold:g} / {itrip:g} A | >= {ihold * rmin * 1e3:.0f} mV | {cable} |")
    w("")
    w("The 42 V outputs have no PTC. Their cables are heavy, and no SMD PTC is rated for "
      "42 V at drive currents. Use XT90-S anti-spark plugs on the 42 V devices, so "
      "hot-plugging onto the live group cannot trip its short-circuit detector.\n")

    w("## I/O map\n")
    w("One TCA9535 (0x20) holds every INP and INP_G, so the switches keep their state while the "
      "ESP32 resets. The fault lines are inputs only, so they go straight to ESP32 GPIOs with "
      "interrupts.\n")
    pins = [f"P{p // 8}{p % 8}" for p in range(16)]
    rows = [(f"INP {r['name']}", "out") for r in RAILS] + [(f"INP_G {r['name']}", "out") for r in RAILS]
    w("| TCA9535 pin | Signal | Dir |\n|---|---|---|")
    for i, (sig, d) in enumerate(rows):
        w(f"| {pins[i]} | {sig} | {d} |")
    for p in pins[len(rows):]:
        w(f"| {p} | spare | |")
    w("")
    w("| ESP32-S3 GPIO | Signal |\n|---|---|")
    for g, r in zip(FLT_GPIOS, RAILS):
        w(f"| {g} | FLT {r['name']} (FLT_I + FLT_T wire-OR) |")
    w("")
    adc = ([f"IMON {r['name']}" for r in RAILS] + [f"VOUT {r['name']}" for r in RAILS]
           + [f"VIN {r['name']}" for r in RAILS if r["src"].endswith("converter")
              or "converter," in r["src"]] + ["VIN 54V boost (the 54 V eFuse's output, VSNS_SW54)"])
    w(f"Two ADC128D818s (TSSOP-16, internal 2.56 V reference) at {', '.join(f'0x{a:02X}' for a in ADC_ADDRS)}. "
      "Each converts 12 ms per channel, so a full sweep runs at about 10 Hz. The 42 V bus voltage, "
      "the input of both 42 V groups and the 54 V boost, comes from the INA228. VOUT 54V is the "
      "boost output. Board NTC and aux 5 V go to ESP32 ADC1 pins (GPIO1, GPIO2).\n")
    w("| ADC | Input |\n|---|---|")
    for i, sname in enumerate(adc):
        w(f"| 0x{ADC_ADDRS[i // 8]:02X}.IN{i % 8} | {sname} |")
    for i in range(len(adc), 8 * len(ADC_ADDRS)):
        w(f"| 0x{ADC_ADDRS[i // 8]:02X}.IN{i % 8} | spare |")
    print("\n".join(out))


if __name__ == "__main__":
    main()
