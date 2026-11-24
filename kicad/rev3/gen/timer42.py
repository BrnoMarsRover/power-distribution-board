"""The 42 V blocks' timer column, planned by hand (2026-11-24). Board mm from the top-left corner.

The TPS48111's bottom row leaves in pin order, west to east: temperature (10), TMR (9), IWRN (8), IMON (7, the
first bus leg). The column sits on the bottom under the FET drain tabs, where no via fits, so every GND pad needs
a way out to the band below the tabs (y > 71.5) or to the gap between the output strip and the drain copper
(x 8.2..9.0 in 42D). Routed freely, the TMR line ran down the column's west side and boxed in the timer caps' and
the IWRN resistor's GND pads. Planned instead:

  TMR: straight down from its pin, a 0.45 mm jog east between the IWRN resistor and the temperature transistor,
       then straight down the column's east side to the tab's foot. All the timer parts hang WEST of it, their
       TMR pads facing it: both caps upright side by side below the second gate resistor (TMR pads on top, joined
       by a stub), the 100k flat below them (TMR pad east).
  GND, west: the caps' GND pads (bottom) joined, down between them to the 100k's GND pad and a via just below
       the tab. The temperature transistor (moved west, clear of TMR) takes its GND down beside the gate
       resistor to a via in the strip gap, above the caps.
  GND, east: the IWRN resistor keeps its spot under the IWRN pin; its GND runs straight down the strip between
       TMR and the IMON leg to a via below the tab.
  Temperature: straight down from its pin beside TMR, a jog west onto the transistor's base pad, the collector
       tied back to it (diode-connected). Planned too since 42C's negotiation tangled it with PD round the gate
       test pad.
"""
import efuse_hand
import route_plan
from route import B

SIG_W, GND_W, TRUNK_W = 0.25, 0.4, 0.3
TMR_X = 13.7 - 15.0           # TMR's run, relative to the IC centre x (42D: 13.7)
JOG_Y = 51.0                  # TMR jogs east here: below the IWRN resistor, above the temperature transistor
TAB_FOOT = 71.6               # TMR turns west to the 100k here (the main FET's tab ends at 71.5)
WEST_VIA = (9.35 - 15.0, 72.65)      # below the tab, west of the 100k
EAST_VIA = {"42D": (14.3 - 15.0, 73.05),   # below the tab, right under the IWRN resistor
            "42C": (14.3 - 15.0, 72.55)}   # 42C: MOTOR_KILL runs at y 73.5 under its band
TEMP_VIA = (8.75 - 15.0, 64.85)      # strip gap, under the second gate resistor (0.25 mm off its pad)
TEMP_X = 11.4 - 15.0                 # the temperature transistor's GND, down beside the gate resistor
TEMP_JOG = 52.6                      # the temperature line jogs west onto the transistor's base here


def _by_net(R, U, num, kind=None):
    net = R.pad(U, num)["net"]
    return [p for p in R.geo.pads if p["net"] == net and p["ref"] != U and (kind is None or p["ref"].startswith(kind))]


def _other(R, p, net="GND"):
    return next(q for q in R.geo.pads if q["ref"] == p["ref"] and q["net"] == net)


def plan(R, U, esc, key, log=print):
    """Lay TMR and the column's GND copper for one 42 V block. Returns the planned GND pads [(ref, num)] (no
    fan-out for them) and the nets laid here (TMR, temperature: no negotiated route for them)."""
    ix, _ = efuse_hand.ic42(key)
    tmr = R.pad(U, "9")["net"]
    caps = sorted(_by_net(R, U, "9", "C"), key=lambda p: p["c"][0])           # west, east
    res = _by_net(R, U, "9", "R")[0]
    iwrn = _by_net(R, U, "8", "R")[0]
    temp = R.pad(U, "10")["net"]
    q_b, q_c = sorted(_by_net(R, U, "10", "Q"), key=lambda p: p["c"][1])  # base (upper), collector
    q_gnd = _other(R, q_b)
    segs = []
    tx = ix + TMR_X
    (_, ex, ey), = esc["9"]["points"]
    cy = caps[1]["c"][1]
    rx, ry = res["c"]
    segs += R.track(tmr, B, [(ex, ey), (ex, JOG_Y), (tx, JOG_Y + tx - ex), (tx, cy), caps[1]["c"], caps[0]["c"]],
                    SIG_W)
    segs += R.track(tmr, B, [(tx, cy), (tx, TAB_FOOT), (tx - (ry - TAB_FOOT), ry), (rx, ry)], SIG_W)
    # temperature: down beside TMR, west onto the base, the collector tied to it
    (_, tx0, ty0), = esc["10"]["points"]
    bx, by = q_b["c"]
    segs += R.track(temp, B, [(tx0, ty0), (tx0, TEMP_JOG), (bx, TEMP_JOG + tx0 - bx), (bx, by)], SIG_W)
    kx, ky = q_c["c"]
    segs += R.track(temp, B, [(kx, ky), (kx, ky - 0.5), (bx, ky - 0.5 - (kx - bx))], SIG_W)
    # west GND: caps' pads joined, down to the 100k's GND pad, on to the via below the tab
    g0, g1, gr = _other(R, caps[0])["c"], _other(R, caps[1])["c"], _other(R, res)["c"]
    wv = (ix + WEST_VIA[0], WEST_VIA[1])
    segs += R.track("GND", B, [g0, g1], GND_W)
    segs += R.track("GND", B, [(gr[0], g0[1]), gr, wv], GND_W)
    R.via("GND", *wv)
    # the temperature transistor's GND: down beside the gate resistor, west under it to the strip-gap via
    tv = (ix + TEMP_VIA[0], TEMP_VIA[1])
    qx, qy = q_gnd["c"]
    gx = ix + TEMP_X
    segs += R.track("GND", B, [(qx, qy), (qx, qy + 0.6), (gx, qy + 0.6 + gx - qx), (gx, tv[1]), tv], GND_W)
    R.via("GND", *tv)
    # east GND: the IWRN resistor straight down the strip to its via
    ig = _other(R, iwrn)["c"]
    ev = (ix + EAST_VIA[key][0], EAST_VIA[key][1])
    segs += R.track("GND", B, [ig, (ig[0], ev[1] - 0.6), ev] if abs(ig[0] - ev[0]) > 1e-6 else [ig, ev], TRUNK_W)
    R.via("GND", *ev)
    bad = route_plan._check_exact(R, segs)
    for m in bad:
        log(f"   timer column {key}: {m}")
    pads = [(p["ref"], p["num"]) for p in (_other(R, caps[0]), _other(R, caps[1]), _other(R, res),
                                           _other(R, iwrn), q_gnd)]
    return pads, {tmr, temp}
