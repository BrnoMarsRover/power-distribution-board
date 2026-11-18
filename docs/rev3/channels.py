#!/usr/bin/env python3
"""Rev3 output channels: component values and I/O assignment.

Run `python3 channels.py > channels.md` after changing anything here. Every value is
computed from the equation in the part's datasheet (cited next to each formula), so
changing a limit is a one-line edit instead of a hand calculation.

Assumptions that are guesses rather than facts are marked ASSUMED. Replace them with
measured values once the loads are known.
"""

from dataclasses import dataclass, field
import math

# ---------------------------------------------------------------- E-series helpers

E96 = [1.00, 1.02, 1.05, 1.07, 1.10, 1.13, 1.15, 1.18, 1.21, 1.24, 1.27, 1.30, 1.33,
       1.37, 1.40, 1.43, 1.47, 1.50, 1.54, 1.58, 1.62, 1.65, 1.69, 1.74, 1.78, 1.82,
       1.87, 1.91, 1.96, 2.00, 2.05, 2.10, 2.15, 2.21, 2.26, 2.32, 2.37, 2.43, 2.49,
       2.55, 2.61, 2.67, 2.74, 2.80, 2.87, 2.94, 3.01, 3.09, 3.16, 3.24, 3.32, 3.40,
       3.48, 3.57, 3.65, 3.74, 3.83, 3.92, 4.02, 4.12, 4.22, 4.32, 4.42, 4.53, 4.64,
       4.75, 4.87, 4.99, 5.11, 5.23, 5.36, 5.49, 5.62, 5.76, 5.90, 6.04, 6.19, 6.34,
       6.49, 6.65, 6.81, 6.98, 7.15, 7.32, 7.50, 7.68, 7.87, 8.06, 8.25, 8.45, 8.66,
       8.87, 9.09, 9.31, 9.53, 9.76]
E12 = [1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2]


def nearest(value: float, series) -> float:
    decade = 10 ** math.floor(math.log10(value))
    candidates = [s * decade for s in series] + [series[0] * decade * 10]
    return min(candidates, key=lambda c: abs(c - value))


def fmt_r(ohms: float) -> str:
    if ohms >= 1e6:
        return f"{ohms / 1e6:.3g}M"
    if ohms >= 1e3:
        return f"{ohms / 1e3:.3g}k"
    return f"{ohms:.3g}R"


def fmt_c(farads: float) -> str:
    for unit, scale in (("uF", 1e-6), ("nF", 1e-9), ("pF", 1e-12)):
        if farads >= scale:
            return f"{farads / scale:.3g}{unit}"
    return f"{farads:.3g}F"


ADC_FULL_SCALE_V = 3.0  # leave headroom below the TLA2528's 3.3 V AVDD reference


# ---------------------------------------------------------------- per-part design

def design_tps48111(ch):
    """42 V output: TPS48111-Q1 + external 100 V FET + shunt. Datasheet SLUSEE5E."""
    p = ch.params
    r_set = 100.0                      # datasheet characterisation value
    # Shunt: aim for ~30 mV at the over-current threshold, where IMON is specified +-2 %.
    r_sns = min((1e-3, 1.5e-3, 2e-3, 3e-3, 4e-3, 5e-3),
                key=lambda r: abs(p["i_oc"] * r - 0.030))
    v_wrn = p["i_oc"] * r_sns
    # Eq. 6: R_IWRN = 11.9 * R_SET / V_SNS(WRN)
    r_iwrn = nearest(11.9 * r_set / v_wrn, E96)
    i_oc_actual = 11.9 * r_set / r_iwrn / r_sns
    # Eq. 11: R_ISCP = I_SC * R_SNS / 15.6 uA - 464
    r_iscp = nearest(p["i_sc"] * r_sns / 15.6e-6 - 464, E96)
    i_sc_actual = (r_iscp + 464) * 15.6e-6 / r_sns
    # Eq. 7: C_TMR = I_TMR * t_OC / 1.2 V, I_TMR = 82 uA
    c_tmr = nearest(82e-6 * p["t_oc"] / 1.2, E12)
    t_oc_actual = c_tmr * 1.2 / 82e-6
    # Eq. 12/13: V_IMON = V_SNS * 0.9 * R_IMON / R_SET  (offset ignored)
    r_imon = nearest(ADC_FULL_SCALE_V * r_set / (0.9 * p["i_fs"] * r_sns), E96)
    v_per_a = 0.9 * r_imon / r_set * r_sns
    # Eq. 5: pre-charge FET gate slew. C_g = C_LOAD * I_G / I_INRUSH, I_G = 100 uA
    c_g = nearest(p["c_load"] * 100e-6 / p["i_inrush"], E12)
    i_inrush_actual = p["c_load"] * 100e-6 / c_g
    t_ramp = p["c_load"] * ch.v_max / i_inrush_actual
    return {
        "R_SNS": f"{r_sns * 1e3:g}mR ({p['i_oc'] ** 2 * r_sns:.2f} W at I_OC)",
        "R_SET": fmt_r(r_set),
        "R_IWRN": f"{fmt_r(r_iwrn)} -> I_OC {i_oc_actual:.1f} A",
        "R_ISCP": f"{fmt_r(r_iscp)} -> I_SC {i_sc_actual:.0f} A",
        "C_TMR": f"{fmt_c(c_tmr)} -> {t_oc_actual * 1e3:.0f} ms",
        "R_IMON": f"{fmt_r(r_imon)} -> {v_per_a * 1e3:.0f} mV/A",
        "Pre-charge C_g": (f"{fmt_c(c_g)} -> {i_inrush_actual:.2f} A for {t_ramp * 1e3:.0f} ms, "
                           f"{0.5 * p['c_load'] * ch.v_max ** 2:.2f} J in the pre-charge FET "
                           f"(peak {ch.v_max * i_inrush_actual:.0f} W) - check its SOA"),
    }, i_oc_actual, v_per_a


def design_tps1663(ch):
    """24 V output: TPS1663, MODE = latch-off. Datasheet SLVSET9."""
    p = ch.params
    # Eq. 6: I_OL = 18 / R_ILIM(kOhm)
    r_ilim = nearest(18e3 / p["i_lim"], E96)
    i_lim_actual = 18e3 / r_ilim
    # Eq. 9: V_IMON = I * 27.9 uA/A * R_IMON
    r_imon = nearest(ADC_FULL_SCALE_V / (27.9e-6 * p["i_fs"]), E96)
    # Eq. 2: t_dVdT = 20.8e3 * V_IN * C_dVdT
    c_dvdt = nearest(p["t_ramp"] / (20.8e3 * ch.v_max), E12)
    t_ramp = 20.8e3 * ch.v_max * c_dvdt
    i_inrush = p["c_load"] * ch.v_max / t_ramp
    return {
        "R_ILIM": f"{fmt_r(r_ilim)} -> {i_lim_actual:.2f} A",
        "R_IMON": f"{fmt_r(r_imon)} -> {27.9e-6 * r_imon * 1e3:.0f} mV/A",
        "C_dVdT": f"{fmt_c(c_dvdt)} -> {t_ramp * 1e3:.1f} ms ramp, {i_inrush:.2f} A inrush",
    }, i_lim_actual, 27.9e-6 * r_imon


def design_tps25982(ch):
    """15 V / 12 V output: TPS25982. Datasheet SLVSEI3."""
    p = ch.params
    # Eq. 4: R_ILIM = 1460 / (I_LIM - 0.11)
    r_ilim = nearest(1460 / (p["i_lim"] - 0.11), E96)
    i_lim_actual = 1460 / r_ilim + 0.11
    # Eq. 12: V_IMON = 246 uA/A * I * R_IMON
    r_imon = nearest(ADC_FULL_SCALE_V / (246e-6 * p["i_fs"]), E96)
    # Eq. 3: C_dVdt(pF) = 4600 / SR(V/ms)
    sr = ch.v_max / (p["t_ramp"] * 1e3)
    c_dvdt = nearest(4600 / sr * 1e-12, E12)
    t_ramp = ch.v_max / (4600 / (c_dvdt * 1e12)) * 1e-3
    # Eq. 5: t_ITIMER(ms) = C(nF) * 0.98 V / 2.1 uA
    c_itimer = nearest(p["t_blank"] * 1e3 * 2.1 / 0.98 * 1e-9, E12)
    return {
        "R_ILIM": f"{fmt_r(r_ilim)} -> {i_lim_actual:.2f} A",
        "R_IMON": f"{fmt_r(r_imon)} -> {246e-6 * r_imon * 1e3:.0f} mV/A",
        "C_dVdt": (f"{fmt_c(c_dvdt)} -> {t_ramp * 1e3:.1f} ms ramp, "
                   f"{p['c_load'] * ch.v_max / t_ramp:.2f} A inrush"),
        "C_ITIMER": f"{fmt_c(c_itimer)} -> {c_itimer * 1e9 * 0.98 / 2.1:.1f} ms blanking",
    }, i_lim_actual, 246e-6 * r_imon


def design_tps259470(ch):
    """5 V output: TPS259470 (FLT pin variant). Datasheet SLVSFC9."""
    p = ch.params
    # Eq. 5: R_ILM = 3334 / I_OCP. The same pin is the current monitor (Eq. 9, 182 uA/A),
    # so its full scale is fixed at about 0.6 V regardless of the limit chosen.
    r_ilm = nearest(3334 / p["i_lim"], E96)
    i_lim_actual = 3334 / r_ilm
    # Eq. 4: C_dVdt(pF) = 2000 / SR(V/ms)
    sr = ch.v_max / (p["t_ramp"] * 1e3)
    c_dvdt = nearest(2000 / sr * 1e-12, E12)
    t_ramp = ch.v_max / (2000 / (c_dvdt * 1e12)) * 1e-3
    # Eq. 6: t_ITIMER(ms) = 1.51 V * C(nF) / 1.8 uA
    c_itimer = nearest(p["t_blank"] * 1e3 * 1.8 / 1.51 * 1e-9, E12)
    return {
        "R_ILM": (f"{fmt_r(r_ilm)} -> {i_lim_actual:.2f} A, monitor {182e-6 * r_ilm * 1e3:.0f} mV/A "
                  f"(tap through 10k, never load the pin directly)"),
        "C_dVdt": (f"{fmt_c(c_dvdt)} -> {t_ramp * 1e3:.1f} ms ramp, "
                   f"{p['c_load'] * ch.v_max / t_ramp:.2f} A inrush"),
        "C_ITIMER": f"{fmt_c(c_itimer)} -> {c_itimer * 1e9 * 1.51 / 1.8:.1f} ms blanking",
    }, i_lim_actual, 182e-6 * r_ilm


# ---------------------------------------------------------------- channel list

@dataclass
class Rail:
    name: str
    v_max: float             # highest steady-state input voltage
    count: int
    connector: str
    part: str
    design: callable
    design_current: float    # what the board's rail copper and input connector carry
    params: dict
    controls: tuple = ("EN",)
    vout_sense: bool = False


RAILS = [
    Rail("42V", 42.0, 8, "XT90", "TPS48111-Q1 + FET", design_tps48111, 40.0,
         dict(i_oc=16.0, i_sc=40.0, t_oc=20e-3, i_fs=20.0,
              c_load=1000e-6,   # ASSUMED: EPOS4 / converter input capacitance
              i_inrush=1.0),
         controls=("INP", "INP_G"), vout_sense=True),
    Rail("24V", 24.0, 8, "Deans T", "TPS1663 (HTSSOP-20)", design_tps1663, 12.5,
         dict(i_lim=5.0, i_fs=6.0, t_ramp=10e-3, c_load=470e-6)),          # c_load ASSUMED
    Rail("15V", 15.0, 8, "XT60", "TPS25982", design_tps25982, 20.0,
         dict(i_lim=8.0, i_fs=10.0, t_ramp=5e-3, t_blank=5e-3, c_load=470e-6)),
    Rail("12V", 12.0, 7, "XT60", "TPS25982", design_tps25982, 25.0,
         dict(i_lim=8.0, i_fs=10.0, t_ramp=5e-3, t_blank=5e-3, c_load=470e-6)),
    Rail("5V", 5.0, 11, "XT30", "TPS259470", design_tps259470, 15.0,   # XT30 input
         dict(i_lim=4.0, t_ramp=2e-3, t_blank=5e-3, c_load=220e-6)),
]

# Rail input voltage dividers into the ADC: (R_top, R_bottom), full scale ~3 V.
RAIL_VIN_DIVIDERS = {"24V": (100e3, 11.3e3), "15V": (100e3, 17.4e3),
                     "12V": (100e3, 23.2e3), "5V": (100e3, 100e3)}
VOUT_42V_DIVIDER = (100e3, 6.81e3)


@dataclass
class Channel:
    rail: Rail
    index: int
    params: dict = field(default_factory=dict)

    @property
    def name(self):
        return f"{self.rail.name}-{self.index}"

    @property
    def v_max(self):
        return self.rail.v_max


# ---------------------------------------------------------------- I/O allocation

class Allocator:
    def __init__(self, prefix, base_addr, width, pin_name):
        self.prefix, self.base, self.width, self.pin_name = prefix, base_addr, width, pin_name
        self.used = 0
        self.taken = 0

    def take(self):
        dev, pin = divmod(self.used, self.width)
        self.used += 1
        self.taken += 1
        return f"0x{self.base + dev:02X}.{self.pin_name(pin)}"

    def align(self):
        """Start the next rail on a fresh port, so each rail's lines stay together."""
        port = 8
        self.used = math.ceil(self.used / port) * port

    @property
    def devices(self):
        return math.ceil(self.used / self.width)


def main():
    exp = Allocator("TCA9535", 0x20, 16, lambda p: f"P{p // 8}{p % 8}")
    adc = Allocator("TLA2528", 0x10, 8, lambda p: f"AIN{p}")

    channels = []
    for rail in RAILS:
        exp.align()
        for i in range(1, rail.count + 1):
            ch = Channel(rail, i, dict(rail.params))
            ch.ctrl = [(c, exp.take()) for c in rail.controls]
            channels.append(ch)
        if rail.count <= 8:   # keep a rail's fault lines on one port when they fit
            exp.align()
        for ch in channels[-rail.count:]:
            ch.flt = exp.take()
    for rail in RAILS:
        adc.align()
        for ch in [c for c in channels if c.rail is rail]:
            ch.imon = adc.take()
        if rail.vout_sense:
            for ch in [c for c in channels if c.rail is rail]:
                ch.vout = adc.take()
    rail_vin = {name: adc.take() for name in RAIL_VIN_DIVIDERS}

    out = []
    w = out.append
    w("# Rev3 output channels\n")
    w("Generated by `channels.py`. Do not edit by hand; change the script and re-run it.\n")
    w("Values come from the datasheet equations cited in the script. Load capacitances are "
      "**assumed** until the real loads are known.\n")

    w("## Component values per rail\n")
    budgets = []
    for rail in RAILS:
        ch = next(c for c in channels if c.rail is rail)
        values, i_lim, v_per_a = rail.design(ch)
        w(f"### {rail.name}: {rail.count} x {rail.connector}, {rail.part}\n")
        w("| Part | Value |\n|---|---|")
        for k, v in values.items():
            w(f"| {k} | {v} |")
        w("")
        budgets.append((rail, i_lim))

    w("## Rail budget\n")
    w("Per-output limits protect each device and its cable. They are not meant to sum to "
      "the rail: when several outputs run near their limits together, the converter's own "
      "current limit acts fast and the firmware sheds outputs by priority on a slower, "
      "thermal time scale.\n")
    w("| Rail | Outputs | Limit each | Sum of limits | Rail design current |")
    w("|---|---|---|---|---|")
    for rail, i_lim in budgets:
        w(f"| {rail.name} | {rail.count} | {i_lim:.1f} A | {i_lim * rail.count:.0f} A | "
          f"{rail.design_current:.1f} A |")
    w("")

    w("## Channel map\n")
    w("Default state = OFF for every channel unless its EN pull resistor is fitted as a "
      "pull-up (see design.md, section 5.1).\n")
    w("| Channel | Control (TCA9535 addr.pin) | Fault in | Current (TLA2528 addr.ch) | Vout |")
    w("|---|---|---|---|---|")
    for ch in channels:
        ctrl = ", ".join(f"{n} {a}" for n, a in ch.ctrl)
        w(f"| {ch.name} | {ctrl} | {ch.flt} | {ch.imon} | {getattr(ch, 'vout', '')} |")
    w("")

    w("## Rail voltage sense\n")
    w("| Rail | ADC | Divider | Full scale |\n|---|---|---|---|")
    for name, (rt, rb) in RAIL_VIN_DIVIDERS.items():
        w(f"| {name} in | {rail_vin[name]} | {fmt_r(rt)} / {fmt_r(rb)} | "
          f"{ADC_FULL_SCALE_V * (rt + rb) / rb:.1f} V |")
    rt, rb = VOUT_42V_DIVIDER
    w(f"| 42V outputs | see map | {fmt_r(rt)} / {fmt_r(rb)} | "
      f"{ADC_FULL_SCALE_V * (rt + rb) / rb:.1f} V |")
    w("")

    w("## Totals\n")
    w(f"- Channels: {len(channels)}")
    w(f"- TCA9535: {exp.devices} (0x20-0x{0x20 + exp.devices - 1:02X}), "
      f"{exp.devices * 16 - exp.taken} pins spare")
    w(f"- TLA2528: {adc.devices} (0x10-0x{0x10 + adc.devices - 1:02X}), "
      f"{adc.devices * 8 - adc.taken} channels spare")
    print("\n".join(out))


if __name__ == "__main__":
    main()
