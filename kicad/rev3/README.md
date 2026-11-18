# Power board rev3: KiCad 10 project

Open `PowerBoard_rev3.kicad_pro` in **KiCad 10**. The design spec lives in
[`docs/rev3/design.md`](../../docs/rev3/design.md), and every computed component value
comes from [`docs/rev3/rails.md`](../../docs/rev3/rails.md).

A review PDF is at [`pdf/rev3/PowerBoard_rev3_schematic.pdf`](../../pdf/rev3/PowerBoard_rev3_schematic.pdf).

## How the schematic was made

`gen/build.py` generated it from the design data, so nothing was drawn by hand from
scratch:

```
python3 kicad/rev3/gen/build.py
```

That run does six things:

1. Writes the custom symbols (`PowerBoard_rev3.kicad_sym`).
2. Writes the custom footprints (`PowerBoard_rev3.pretty`).
3. Writes the 13 sheets.
4. Upgrades every file to the KiCad 10 format with `kicad-cli` from the KiCad flatpak.
5. Runs ERC.
6. Exports KiCad's own netlist and checks it pin by pin against the netlist the script
   intended.

Result at generation time:
- 436 parts and 252 nets.
- Netlist check passes.
- ERC is clean at every severity.

Every sheet is drawn like a hand-made schematic: parts sit at chosen positions and real
wires connect them.
- Power flows left to right along the rails. Converters and drivers sit below their power
  path, with their setting parts below or beside them.
- Labels appear only where a signal leaves the sheet. The exception is the ESP32 module,
  whose GPIOs carry labels in the usual way, while the circuits around it (reset, boot,
  USB, host UART, LEDs, sense dividers) are drawn out.
- On the outputs sheet each rail is a column: a bus with one row per output (PTC,
  connector, GND).

The drawing goes through the same netlist check, so a wire landing where it shouldn't is
caught before anyone opens KiCad. One crossing is intentional: on the `can` sheet, CANH
crosses CANL without a junction on its way to the second connector.

## Board rules

`gen/board_rules.py` sets up the PCB side for JLCPCB's standard 4-layer process, with
nothing that adds a surcharge:
- **Stackup:** 1.6 mm, JLC04161H-7628, 1 oz outer / 0.5 oz inner, HASL lead-free.
- **Limits:** 0.25 mm minimum track, 0.127 mm clearance, vias of 0.3 mm drill and 0.6 mm pad, 1.0 mm
  silkscreen text.
- **Netclasses** for the 42 V, 54 V and rail nets, and a 0.5 mm clearance on the outer
  layers around 42 / 54 V pours.

It writes the rules into `PowerBoard_rev3.kicad_pro` and `PowerBoard_rev3.kicad_dru`.
It creates `PowerBoard_rev3.kicad_pcb` only if the board doesn't exist yet, so re-running
it never touches a layout. The values and the reasons for them are in
`docs/rev3/design.md`, section 8.

**Once you start editing in KiCad, these files are the source of truth.** Re-running
`build.py` overwrites them. The generator is a bootstrap, not a build step.

## Sheets

| Sheet | Contents |
|-------|----------|
| `power_in` | Battery XT90 input, SMCJ45A, bulk capacitance, 0.5 mΩ shunt with INA238, charger input XT60 and pack charge-port XT60 (charging flows through the shunt) |
| `sw_42d`, `sw_42c` | 42 V drive group and converter-feed group eFuses (TPS48111-Q1, 2 + 1 D2PAK). The drive group has the E-STOP kill FET on EN/UVLO. |
| `sw_24`, `sw_12`, `sw_5` | Rail eFuses, input TVS, bulk capacitance, input and output voltage sense |
| `sw_54` | eFuse in front of the 54 V boost |
| `outputs` | Rail input connectors, 8 × XT90 outputs, and the 24 / 12 / 5 V outputs with their PTCs. JST-XH 54 V output to the Zyxel |
| `boost54` | LM5122 boost, 30–42 V to 54 V, 100 W |
| `aux_power` | LMR38010 (42 V to 5.3 V), the diode-OR with USB VBUS, and the AP7361C 3.3 V LDO |
| `mcu` | ESP32-S3-WROOM-1U, USB-C with ESD, host UART on JST-GH, buttons, LEDs, NTC |
| `can` | TCAN1044AV, split termination on solder jumpers, two Molex CLIK-Mate 4p CAN connectors matching the EPOS4 50/5 (X11 / X12) |
| `io` | TCA9535 (switch control), 2 × ADC128D818, E-STOP latch (74LVC1G74) |

## Custom parts

- **Symbols:** TPS48111-Q1, LM5122, LMR38010, TCAN1044AV and INA238, with pinouts taken
  from the TI datasheets.
  - Their pins are arranged for the drawing. For example, the INA238 sense inputs sit on
    top, 7.62 mm apart, so the Kelvin lines drop straight from the shunt.
  - Q_NMOS_GDS covers the D2PAK and DPAK FETs: KiCad 10 dropped its GDS-numbered FET
    symbols.
- **Footprints:** TI_DGX0019A_VSSOP-19 is drawn from TI's land pattern, with pads
  lengthened for hand soldering. XT90PB-M, XT90PB-F and DEANS-T are copied from rev2's
  SamacSys library.

## Review before layout

These are the things the generator could not settle on its own:

- [ ] **LM5122 compensation.** R_SLOPE, R_COMP, C_COMP and C_HF are starting values from
  the datasheet example. Run TI's LM5122 quickstart calculator with 30–42 V in, 54 V,
  1.85 A, 400 kHz and 22 µH.
- [ ] **Pre-charge FET SOA.** Check the IPB042N10N3G against the energies in `rails.md`:
  the worst case is about 3.3 J on the drive group, assuming 470 µF per EPOS4.
- [ ] **Boost inductor footprint.** `L_12x12mm_H6mm` is a placeholder. Match it to the
  TMPA1265SP-220MN-D (13.5 × 12.6 mm).
- [ ] **Aux buck parts.** Confirm that the LMR38010SDDAR variant has the RT/SYNC pin, and
  pick the actual 33 µH / 1.5 A inductor for its footprint.
- [ ] **Connector footprints.** XT90 and Deans: check the M and F pin geometry against the
  Amass drawings. For every power connector, check that footprint pin 1 is the housing's
  + side.
- [ ] **Electrolytic sizes.** 100 µF / 100 V and 100 µF / 80 V parts may need a larger
  footprint than `CP_Elec_10x10.5`.
- [ ] **Hand-solder holes.** Add a 1–1.5 mm plated hole through the exposed pads of the
  LM5122, the LMR38010 and the ESP32 module (layout step).
- [ ] **Placement.** Put the MMBT3904 temperature sensors against their main FETs, and
  route every shunt Kelvin.
- [ ] **ESD arrays are flow-through.** On both USBLC6-2SC6, pins 1/6 and 3/4 are joined
  inside the part. The connector side and the MCU side are therefore separate nets, which
  forces each line across its pad pair in layout.
