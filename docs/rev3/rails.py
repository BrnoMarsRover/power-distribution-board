#!/usr/bin/env python3
"""Rev3 rail switches: component values, per-output PTCs and the I/O map.

Run `python3 rails.py > rails.md` after changing anything here.

Architecture (decided 2026-11-18): one eFuse per rail (TPS48111-Q1 + external FETs +
shunt), plus one resettable PTC per 24/15/12/5 V output connector. Every eFuse value is
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


def nearest(value, series):
    decade = 10 ** math.floor(math.log10(value))
    return min([s * decade for s in series] + [series[0] * decade * 10],
               key=lambda c: abs(c - value))


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


ADC_FULL_SCALE_V = 3.0   # headroom below the TLA2528's 3.3 V AVDD reference
R_SET = 100.0            # TPS4811 characterisation value
I_TMR = 82e-6            # TMR pull-up current
R_TMR = 100e3            # across C_TMR -> latch-off mode (section 8.3.3.2)
I_G = 100e-6             # pre-charge gate pull-up current, typical (72-140 uA)

# FETs: name, R_DS(on) max at 10 V in ohms, JLC part.
FETS = {
    "IPT015N10N5": (1.5e-3, "C108964", "100 V, TOLL"),
    "BSC040N10NS5": (4.0e-3, "C534334", "100 V, 5x6"),
    "BSC010N04LS": (1.0e-3, "C501508", "40 V, 5x6"),
}

# PTCs: name, V max, I hold, I trip, R initial min, JLC part.
PTCS = {
    "SMD1812P300TF": (8, 3.0, 5.0, 0.010, "C21004", "1812"),
    "SMD2920P300TF/16": (16, 3.0, 6.0, 0.012, "C167318", "2920"),
    "SMD2920P300TF/33": (33, 3.0, 6.0, 0.012, "C2943634", "2920"),
}

# One entry per switched rail.
#   design: continuous current the input connector and copper are sized for
#   i_oc / t_oc: over-current trip (IWRN) and its delay; latch-off
#   i_sc: fast short-circuit trip (ISCP), or None to disable it (rails behind a
#         current-limited converter, so a hot-plug spike cannot trip the rail)
#   c_out, n_out: ASSUMED capacitance per output, and the output count
RAILS = [
    dict(name="42V", v=42.0, design=40.0, i_oc=60.0, t_oc=0.5, i_sc=150.0,
         r_sns=0.5e-3, fet="IPT015N10N5", c_out=470e-6, n_out=8, conn="XT90",
         i_inrush=2.0, ptc=None, src="battery bus",
         note="Motor peaks: the 60 A / 0.5 s trip must sit above the drive's acceleration profile."),
    dict(name="24V", v=24.0, design=12.5, i_oc=13.75, t_oc=0.5, i_sc=None,
         r_sns=2e-3, fet="BSC040N10NS5", c_out=470e-6, n_out=8, conn="Deans T",
         i_inrush=2.0, ptc="SMD2920P300TF/33", src="300 W converter"),
    dict(name="15V", v=15.0, design=10.0, i_oc=11.0, t_oc=0.5, i_sc=None,
         r_sns=2e-3, fet="BSC010N04LS", c_out=470e-6, n_out=8, conn="XT60",
         i_inrush=2.0, ptc="SMD2920P300TF/33", src="150 W converter"),
    dict(name="12V", v=12.0, design=12.5, i_oc=13.75, t_oc=0.5, i_sc=None,
         r_sns=2e-3, fet="BSC010N04LS", c_out=470e-6, n_out=7, conn="XT60",
         i_inrush=2.0, ptc="SMD2920P300TF/16", src="150 W converter"),
    dict(name="5V", v=5.0, design=15.0, i_oc=16.5, t_oc=0.5, i_sc=None,
         r_sns=2e-3, fet="BSC010N04LS", c_out=220e-6, n_out=11, conn="XT30",
         i_inrush=2.0, ptc="SMD1812P300TF", src="150 W converter, XT30-limited"),
]


def design_rail(r):
    v_wrn = r["i_oc"] * r["r_sns"]
    # Eq. 6: R_IWRN = 11.9 * R_SET / V_SNS(WRN)
    r_iwrn = nearest(11.9 * R_SET / v_wrn, E96)
    i_oc = 11.9 * R_SET / r_iwrn / r["r_sns"]
    # Eq. 10 (latch-off, R_TMR across C_TMR): C_TMR = t_OC / (R_TMR * ln(1 / (1 - 1.2 / (R_TMR * I_TMR))))
    k = R_TMR * math.log(1 / (1 - 1.2 / (R_TMR * I_TMR)))
    c_tmr = nearest(r["t_oc"] / k, E12)
    if r["i_sc"]:
        # Eq. 11: R_ISCP = I_SC * R_SNS / 15.6 uA - 464
        r_iscp = nearest(r["i_sc"] * r["r_sns"] / 15.6e-6 - 464, E96)
        iscp = f"{fmt_r(r_iscp)} -> {(r_iscp + 464) * 15.6e-6 / r['r_sns']:.0f} A in 1.2 us"
    else:
        iscp = "ISCP tied to CS- (disabled): converter-limited, survives hot-plug spikes"
    # Eq. 12/13: V_IMON = V_SNS * 0.9 * R_IMON / R_SET, full scale at 1.25 x I_OC
    i_fs = 1.25 * i_oc
    r_imon = nearest(ADC_FULL_SCALE_V * R_SET / (0.9 * i_fs * r["r_sns"]), E96)
    mv_per_a = 0.9 * r_imon / R_SET * r["r_sns"] * 1e3
    # Eq. 5: pre-charge gate slew, C_g = C_LOAD * I_G / I_INRUSH
    c_load = r["c_out"] * r["n_out"]
    c_g = nearest(c_load * I_G / r["i_inrush"], E12)
    i_inrush = c_load * I_G / c_g
    t_ramp = c_load * r["v"] / i_inrush
    energy = 0.5 * c_load * r["v"] ** 2
    rds, _, fet_desc = FETS[r["fet"]]
    return [
        ("Feeds", f"{r['n_out']} x {r['conn']}, from {r['src']}"),
        ("Design current", f"{r['design']:g} A"),
        ("Main FET", f"{r['fet']} ({fet_desc}), {r['design'] ** 2 * rds:.2f} W at design current"),
        ("Pre-charge FET", f"{r['fet']} (same part)"),
        ("R_SNS", f"{r['r_sns'] * 1e3:g} mOhm, {r['design'] ** 2 * r['r_sns']:.2f} W at design current, "
                  f"{v_wrn * 1e3:.0f} mV at trip"),
        ("R_SET", fmt_r(R_SET)),
        ("R_IWRN", f"{fmt_r(r_iwrn)} -> trips at {i_oc:.1f} A"),
        ("C_TMR (R_TMR 100k, latch-off)", f"{fmt_c(c_tmr)} -> {c_tmr * k * 1e3:.0f} ms"),
        ("R_ISCP", iscp),
        ("R_IMON", f"{fmt_r(r_imon)} -> {mv_per_a:.0f} mV/A, 3.0 V at {i_fs:.0f} A"),
        ("C_g / R_g (pre-charge slew)", f"{fmt_c(c_g)} / 330R -> {i_inrush:.1f} A for {t_ramp * 1e3:.0f} ms"),
        ("Pre-charge FET stress", f"{energy:.2f} J, {r['v'] * i_inrush:.0f} W peak (ASSUMED "
                                  f"{fmt_c(c_load)} load) - check its SOA"),
        ("C_BST", f">= 10 x C_g = {fmt_c(10 * c_g)}, plus Qg(main + pre-charge) / 1 V"),
    ]


def main():
    out = []
    w = out.append
    w("# Rev3 rail switches\n")
    w("Generated by `rails.py`. Do not edit by hand; change the script and re-run it.\n")
    w("One TPS48111-Q1 eFuse per rail, with a resettable PTC on each 24 / 15 / 12 / 5 V output. "
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

    w("## Per-output PTCs\n")
    w("A PTC protects the cable to one device from an overload too small to trip the rail. "
      "Each output's cable must carry the PTC's trip current continuously. Fit a bigger "
      "PTC on any output whose device needs more than the hold current.\n")
    w("| Rail | Outputs | PTC (JLC #) | V max | Hold / trip | Drop at hold | Cable at least |")
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
    w("One TCA9535 (0x20) holds every control line, so outputs keep their state while the "
      "ESP32 resets. Its INT line goes to an ESP32 GPIO for fault events.\n")
    pins = [f"P{p // 8}{p % 8}" for p in range(16)]
    rows, k = [], 0
    for sig in ("INP", "INP_G", "FLT"):
        for r in RAILS:
            rows.append((pins[k], f"{sig} {r['name']}", "out" if sig != "FLT" else "in (FLT_I + FLT_T wire-OR)"))
            k += 1
    rows.append((pins[k], "spare", ""))
    w("| TCA9535 pin | Signal | Dir |\n|---|---|---|")
    for p, s, d in rows:
        w(f"| {p} | {s} | {d} |")
    w("")
    adc = ([f"IMON {r['name']}" for r in RAILS] + [f"VOUT {r['name']}" for r in RAILS]
           + [f"VIN {r['name']}" for r in RAILS if r["name"] != "42V"]
           + ["board NTC", "aux 5 V"])
    w("Two TLA2528s (0x10, 0x11). The 42 V bus input voltage comes from the INA228.\n")
    w("| ADC | Input |\n|---|---|")
    for i, s in enumerate(adc):
        w(f"| 0x{0x10 + i // 8:02X}.AIN{i % 8} | {s} |")
    print("\n".join(out))


if __name__ == "__main__":
    main()
