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
- 435 parts and 249 nets.
- Netlist check passes.
- ERC is clean at every severity.

Every pin has a short stub and a net label instead of wires drawn between parts. That is
less pretty than a hand-drawn sheet, but it is correct by construction and easy to verify.

**Once you start editing in KiCad, these files are the source of truth.** Re-running
`build.py` overwrites them. The generator is a bootstrap, not a build step.

## Sheets

| Sheet | Contents |
|-------|----------|
| `power_in` | Battery XT90 input, SMCJ45A, bulk capacitance, 0.5 mΩ shunt with INA228, charger XT60 |
| `sw_42d`, `sw_42c` | 42 V drive group and converter-feed group eFuses (TPS48111-Q1, 2 + 1 D2PAK). The drive group has the E-STOP kill FET on EN/UVLO. |
| `sw_24`, `sw_12`, `sw_5` | Rail eFuses, input TVS, bulk capacitance, input and output voltage sense |
| `sw_54` | eFuse in front of the 54 V boost |
| `outputs` | Rail input connectors, 8 × XT90 outputs, and the 24 / 12 / 5 V outputs with their PTCs. JST-XH 54 V output to the Zyxel |
| `boost54` | LM5122 boost, 30–42 V to 54 V, 100 W |
| `aux_power` | LMR38010 (42 V to 5.3 V), the diode-OR with USB VBUS, and the AP7361C 3.3 V LDO |
| `mcu` | ESP32-S3-WROOM-1U, USB-C with ESD, host UART on JST-GH, buttons, LEDs, NTC |
| `can` | TCAN1044AV, split termination on solder jumpers, two CAN connectors |
| `io` | TCA9535 (switch control), 2 × ADC128D818, E-STOP latch (74LVC1G74) |

## Custom parts

- **Symbols:** TPS48111-Q1, LM5122, LMR38010 and TCAN1044AV, with pinouts taken from the
  TI datasheets. Q_NMOS_GDS covers the D2PAK and DPAK FETs: KiCad 10 dropped its
  GDS-numbered FET symbols.
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
- [ ] **CAN connector.** Match it to the EPOS4 harness. JST-GH 4p is a placeholder.
- [ ] **Electrolytic sizes.** 100 µF / 100 V and 100 µF / 80 V parts may need a larger
  footprint than `CP_Elec_10x10.5`.
- [ ] **Hand-solder holes.** Add a 1–1.5 mm plated hole through the exposed pads of the
  LM5122, the LMR38010 and the ESP32 module (layout step).
- [ ] **Placement.** Put the MMBT3904 temperature sensors against their main FETs, and
  route every shunt Kelvin.
