"""Template placement of the logic and the supplies' control networks (run by pcb_place.py after the eFuses).

Each group: its ICs placed at a fixed spot (orientation by a pin test), then their small parts by satellites.py's
engine, anchored to the IC pins and to the pads of parts already placed (connectors, the ESP32, FET gates, LEDs),
decoupling caps one per IC supply pin, test pads in labelled columns. Order matters: earlier groups take their
room first. Board mm from the top-left corner, bottom side.

  aux network     U11's feedback, UVLO, RON and ripple parts in the pocket right of U11
  boost network   the LM5122's FB / COMP / SS / RT / SLOPE / RES / UVLO / sense filter, left of U10 in the
                  brake corner (its signal pins face left), overflow in the free strip under the 24 V PTCs
  brake           comparator, reference and divider by the brake FET's gate via
  battery monitor INA228 under the bar between the 42 V clusters, beside the input shunt's Kelvin vias
  CAN             transceiver and split termination beside the CAN connectors
  ESP32 support   under the module (pull-ups, EN RC, LED resistors, NTC, USB ESD; the UART's ESD at J33), its four test
                  pads (TXD0, RXD0, I2C) in a field below the CAN block
  switch control  ADCs, I/O expander and the motor-kill latch in the band under the rail eFuses, between
                  everything they read and drive; the MOTOR_KILL test pad left of the IMON ADC
  power button    PhotoMOS and button sense in the corner by the button and BMS connectors
"""
import satellites
from satellites import place_cluster, put_ic

SUP = satellites.SUPPLY
GAP = 0.5                              # between courtyards in the logic groups (the hand-soldering rule, design.md)


def _sheet(FP, prefix):
    return [r for r, f in FP.items() if f.GetSheetname().startswith(prefix)]


def _loose(FP, refs, tps=True):
    """The small parts (and signal test pads) among refs that are not placed yet."""
    out = []
    for r in refs:
        f = FP[r]
        if f.IsLocked():
            continue
        if r.startswith("TP"):
            if tps and "Keystone" not in f.GetFPIDAsString():
                out.append(r)
        elif satellites._small(f):
            out.append(r)
    return out


def _nets(FP, r):
    """Non-supply nets of a part, without the sheet path."""
    return {p.GetNetname().split("/")[-1] for p in FP[r].Pads()} - SUP - {""}


def _pin(FP, ref, num):
    return next(satellites._pad_xy(p) for p in FP[ref].Pads() if p.GetNumber() == num)


def _row(a, b, horizontal=True):
    """Pins a and b on one row (same y if horizontal)."""
    return abs(a[1] - b[1]) < 0.05 if horizontal else abs(a[0] - b[0]) < 0.05


# switch control's spots (decided 2026-11-21): the latch's network and the ICs' decoupling, each at its pins. Bottom
# side; rot 90 = pad 1 below, -90 = pad 1 above, 0 / 180 = pad 1 left / right
SW_SPOTS = {
    "C115": (105.25, 78.9, -90),           # the hold-off cap: its RC node on top, by the buffer's input
    "D21": (107.8, 78.9, 90),              # Schottky: anode (RC node) on top, MOTOR_OK below
    "R169": (110.4, 78.9, 90),             # 1 M: RC node on top, MOTOR_OK below
    "C116": (96.3, 77.1, 0),               # the latch's 100 nF, above it
    "C117": (68.3, 72.65, 180),            # U20's 1 uF, under it, its supply pad right below the supply pin (the
                                           # bus pins' vias beside it, i2c_plan; its 100 nF: far42.DECAPS)
    "C119": (122.6, 71.3, 0),              # U21's 1 uF, right of it on the planes (its 100 nF: i2c_plan, under it)
    "C112": (89.4, 69.7, 90),              # U17's 100 nF, left of its supply pin (pin 24, bottom left)
}


def place(FP, obstacles):
    """Place every group. obstacles: the bottom side's boxes, extended in place. Returns (placed, missed)."""
    done, missed = [], []

    def ic(ref, x, y, test, area):
        put_ic(FP[ref], x, y, test, area, obstacles)
        done.append(ref)

    def run(name, ic, area, members, pin_order=None, **kw):
        if pin_order is None:
            pin_order = sorted((p.GetNumber() for p in FP[ic].Pads() if p.GetNumber()), key=lambda n: (len(n), n))
        d, m = place_cluster(FP, ic, area, obstacles, pin_order, members=members, locked_terminals=True, gap=GAP, **kw)
        print(f"{name}: {len(d)} placed" + (f", no spot: {' '.join(m)}" if m else ""))
        done.extend(d)
        missed.extend(m)

    # ---- the FLT bus's pull-ups (flt_bus, decided 2026-11-21): on their spots by the ESP32's pin vias (and
    # FLT_42D's by its test pad), before any group; their room and the bus's bottom-side copper kept
    import flt_bus
    done.extend(flt_bus.place(FP, obstacles))
    import ina_link                                       # the INA228's link to the logic (decided 2026-11-21)
    obstacles += ina_link.KEEP
    import i2c_plan                                       # the I2C bus (decided 2026-11-22): its pull-ups by the
    done.extend(i2c_plan.place(FP, obstacles))            # trunk, ALERT's on its hop, the bus's way kept

    # ---- the host UART's ESD array at its connector (decided 2026-11-22: a cable discharge is shunted at the board
    # edge; the series resistors stay at the module, the textbook order): below J33's pins 1 / 2, right of the aux
    # input cap, its connector-side pins up and in J33's pin order (bottom side: pin 4 left of pin 6). Before the
    # aux network, which packs around it
    ic("U15", 142.85, 37.35, lambda p: p["4"][1] < p["1"][1] - 1.0 and p["4"][0] < p["6"][0] - 1.0,
       [(140.9, 35.0, 145.5, 40.2)])                  # (the corner J33's outline and the aux power stage leave)
    import esp_plan                                       # its vias south of it: kept from the aux network
    obstacles += esp_plan.KEEP_BOTTOM

    # ---- aux network: the pocket right of U11 (the aux test pads stay on top, with the packer)
    aux_all = _loose(FP, _sheet(FP, "Always-on aux"), tps=False)
    aux_3v3 = [r for r in aux_all if not _nets(FP, r)]        # the sheet's spare +3V3 cap: with the ESP32's parts
    aux = [r for r in aux_all if r not in aux_3v3]
    run("aux network", "U11", [(135.3, 38.4, 149.4, 44.6), (140.5, 35.2, 149.4, 38.4), (139.9, 44.6, 145.8, 49.2)], aux,
        pin_order=["5", "3", "4", "6", "8"], extra_ics=("U12",), decaps=True,
        supply_pull={"BUS42": lambda x, y: 0.5 * max(0.0, y - 37.5)})

    # ---- boost network: the control loop left of U10 in the brake corner (its signal pins face left); the parts
    # that need not be there (the 54 V sense divider for the ADC, BOOST_DIS's pull-up, the LED resistor) go to
    # the free strip under the 24 V PTCs and beside the LED
    boost = _loose(FP, _sheet(FP, "54 V boost"), tps=False)
    away_nets = {"VSNS_OUT_54", "BOOST_DIS"}
    away = [r for r in boost if (_nets(FP, r) & away_nets and not r.startswith("Q"))
            or any(n.startswith("Net-(D15") for n in _nets(FP, r))]
    # the loop network north of U10 and the sense filter at pins 3 / 4 by hand (boost_plan, 2026-11-22: the
    # analog lines peel off the band over U10's top in order, the Kelvin pair arrives side by side); the rest packs
    # west of U10, clear of the brake corner
    import boost_plan
    done.extend(boost_plan.place(FP, obstacles))
    boost = _loose(FP, _sheet(FP, "54 V boost"), tps=False)
    run("boost network", "U10", boost_plan.BOOST_AREA, [r for r in boost if r not in away],
        pin_order=["5", "6", "7", "8"])                  # VIN's 1206 cap first, by its pin
    run("boost, outer parts", "U10", [(50.5, 100.0, 61.0, 149.4)], away)

    bottom_band = [(51.0, 136.6, 123.5, 149.4)]                    # test pads along the bottom edge
    # ---- brake comparator: the rest of the corner, by the brake FET's gate via (x 4.2, y 132.5); its test pads
    # go to the bottom-edge field (no room left in the corner)
    corner = [(0.6, 124.3, 26.0, 142.4), (19.9, 111.8, 24.0, 123.4)]   # below and beside the brake XT60
    ic("U8", 9.0, 128.0, lambda p: True, corner)
    ic("U9", 13.0, 128.0, lambda p: True, corner)
    brake = _loose(FP, _sheet(FP, "Brake chopper"))
    run("brake", "U8", corner, brake, extra_ics=("U9",), decaps=True, tp_area=bottom_band, tp_column=(52.4, 137.9))

    # ---- battery monitor: INA228 with IN+ / IN- facing the input shunt's Kelvin vias (x 34)
    # (the band below the battery XT90: about 15 mm of Kelvin route from the vias at x 34)
    ina = [(13.0, 12.6, 28.0, 22.3)]
    ic("U1", 22.5, 17.0, lambda p: p["10"][0] > p["1"][0] + 1.0, ina)
    # (the INA228's ALERT pull-up: i2c_plan, by the ESP32 end of the line)
    batt = _loose(FP, _sheet(FP, "Battery input"))
    run("battery monitor", "U1", ina, batt, pin_order=["10", "9", "3", "4", "5"], decaps=True,
        tp_area=[(0.6, 12.4, 13.0, 22.6), (0.6, 26.9, 9.0, 36.6)], tp_column=(1.9, 14.04))

    # ---- CAN: transceiver with CANH / CANL facing the connectors on the right
    can_area = [(125.6, 92.5, 139.0, 112.0)]
    ic("U16", 132.0, 101.0, lambda p: p["7"][0] > p["2"][0] + 1.0, can_area)
    can = _loose(FP, _sheet(FP, "CAN bus"))
    run("CAN", "U16", can_area, can, decaps=True, tp_column=(127.6, 106.5))

    # ---- the host UART's series resistors on the top side, north of their module pins (esp_plan, 2026-11-22)
    import esp_plan
    done.extend(esp_plan.place(FP, obstacles))

    # ---- ESP32 support: under the module and around USB; USB ESD by the receptacle (the UART's ESD: at J33, above)
    esp_area = [(125.6, 67.2, 149.4, 92.5)]
    ic("U14", 138.3, 87.0, lambda p: p["1"][0] > p["4"][0] + 1.0, esp_area)
    esp_plan.usb_keep(obstacles)                      # USB-C's fan-out vias, U14's ways out: no parts there
    esp = _loose(FP, _sheet(FP, "ESP32-S3")) + aux_3v3
    tp_field = [(125.6, 112.5, 149.4, 142.0)]
    run("ESP32 support", "U14", esp_area, esp, extra_ics=("U15",), decaps=True,
        tp_area=tp_field, tp_column=(127.6, 113.6))

    # ---- switch control: ADCs, expander, latch in the band under the rail eFuses
    sw = _sheet(FP, "Switch control")
    button_nets = {"PWR_HOLD", "PWR_BTN", "BTN_SNS", "SS_A", "SS_B"}
    button = [r for r in sw if r == "U22" or (_nets(FP, r) & button_nets)
              or any(n.startswith(("Net-(R172", "Net-(J36")) for n in _nets(FP, r))]   # + the motor switch input
    band = [(55.0, 49.5, 125.3, 74.6)]
    adc_in_up = lambda p: _row(p["9"], p["16"]) and p["9"][1] < p["1"][1] - 1.0
    ic("U20", 69.5, 67.0, adc_in_up, band)            # IMON and the 42 V output sense
    ic("U17", 95.0, 67.0, lambda p: _row(p["1"], p["12"]) and p["1"][1] < p["13"][1] - 1.0, band)
    ic("U21", 118.5, 66.5, adc_in_up, band)           # the rails' input and output sense
    # the motor-kill latch (decided 2026-11-21): under U17, below the 42 V bundle and above the lowered 12 V
    # connectors (their outline blocks both sides and sets the row); MOTOR_KILL comes in from the left (far42),
    # MOTOR_ARM from the expander through one L3 hop, MOTOR_OK from the ESP32 side on the right. The Schmitt
    # buffer's output faces /CLR on the same row, the hold-off network (C115, D21 // R169) stands on its right
    latch = [(92.5, 76.0, 112.0, 82.7)]
    ic("U18", 96.3, 80.1, lambda p: p["3"][0] < p["6"][0] - 1.0 and p["4"][1] < p["1"][1], latch)
    ic("U19", 101.8, 78.9, lambda p: p["4"][0] < p["2"][0] - 1.0 and p["5"][1] < p["4"][1], latch)
    import power42
    for ref, (x, y, deg) in SW_SPOTS.items():
        f = FP[ref]
        power42._at(f, x, y, deg, bottom=True)
        f.SetLocked(True)
        obstacles.append(satellites._box(f))
        done.append(ref)
    # the 42 V bus's far end (far42, decided 2026-11-21): its parts on their spots beside U20 and U17, its lines'
    # way kept; the group packs around them
    import far42
    done.extend(far42.place(FP, obstacles))
    done.extend(i2c_plan.place_sw(FP, obstacles))         # U21's address strap and 100 nF, the bus pins' way
    hub = _loose(FP, [r for r in sw if r not in button])
    # the 42 V eFuses' monitoring parts (no room in their blocks, decided 2026-11-21): IMON resistor + cap and the
    # output divider's lower half at the ADC, INP / INPG pull-downs at the expander, the FLT / IMON test pads in
    # the test-pad field; the template (efuse_hand.SLOTS42) leaves them for this group
    far_nets = ("IMON_42", "INP_42", "INPG_42", "FLT_42", "VSNS_OUT_42")
    hub += _loose(FP, [r for r in _sheet(FP, "42 V") if r not in hub and not FP[r].IsLocked()
                       and any(p.GetNetname().startswith(far_nets) for p in FP[r].Pads())])
    run("switch control", "U17", band, hub, extra_ics=("U20", "U21", "U18", "U19"), decaps=True,
        tp_area=[(55.0, 61.0, 66.0, 74.6)] + bottom_band, tp_column=(56.4, 62.6))   # (MOTOR_KILL's is far42's)

    # ---- power button: PhotoMOS by the button and BMS connectors, with the motor switch's input resistors;
    # the test pads join the switch-control ones along the bottom edge
    corner_tr = [(125.9, 0.6, 135.6, 18.3)]
    ic("U22", 130.5, 6.0, lambda p: True, corner_tr)
    run("power button", "U22", corner_tr, _loose(FP, button), decaps=True, tp_area=bottom_band, tp_column=(52.4, 137.9))
    return done, missed
