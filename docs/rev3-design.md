# Power board rev3: design spec

Status: **draft for review**. Branch `hardware/rev3`. Replaces board v2.4.0.

This is the plan for the next board, written before the schematic so the decisions can
be argued about while they are still cheap. Items marked **OPEN** need a team decision;
each has a recommended default (section 10).

---

## 1. What rev2 taught us

Team notes on v2.4.0:

- Branch switching is unreliable, and sometimes a whole branch refuses to switch. Most
  FETs ended up bridged so the rover would start reliably.
- Adding dV/dt inrush limiting to the LT1910 stages helped, but did not fix it.
- The 5 V aux supply is hard to get working on some boards.
- OCP is disabled in firmware, because bridged branches cannot be opened.

A review of the rev2 schematic, layout and firmware found the following. They are ordered by
how well each explains the symptoms.

1. **The aux buck is miswired.** IC7 (MAX17541G) pin 4 is VCC, its internal 5 V LDO
   output. Pin 5 is FB (0.9 V). The two are tied together in both the schematic and the PCB.
   FB therefore sits near 5 V, so the regulator believes the output is far too high and does
   not switch.
   - The Pico then runs only when USB from the PC back-feeds VSYS through its internal diode.
   - The PC is powered *through* this board, so a cold start can deadlock: the board waits
     for the PC to power it over USB, and the PC waits for the board to switch its power on.
   - Bridging the FETs breaks that loop. This is the leading explanation for both "aux
     supply finicky" and "switching refuses to work".
   - *Check on a rev2 board:* battery only, USB unplugged, measure TP3.3. If it is not
     about 5 V, this is the cause.
2. **The inrush fix moved the stress into the FETs.** Each slew network is 100 kΩ plus
   1 µF to GND, charged by the LT1910's small internal charge pump, so every output ramps up
   slowly.
   - Downstream converters start at their UVLO, partway up the ramp, and draw full power
     while the FET still has tens of volts across it.
   - That is linear-mode dissipation outside the FET's SOA.
   - Nothing trips, because the current itself is normal.
3. **The LT1910 has no source pin.** At turn-off it pulls the gate to *ground*. Downstream
   capacitors keep the output up, so V_GS ≈ −V_out: about −42 V on Q1 and −24 V on Q5,
   against a ±20 V rating. The gate oxide takes a hit at every turn-off.
4. **Branch drivers are powered from the rail they switch.** Switching onto a capacitive
   load with a 60 W converter behind it can push the converter into current limit. The rail
   then collapses and the driver resets. The LT1910 needs ≥ 8 V, so the 12 V rail has only
   4 V of margin.
5. **The firmware makes the board fail-off.**
   - All five enables go high in the same instant.
   - A Pico reset drops every output.
   - Boot waits up to 10 s for USB.
   - If no INA238 answers, nothing is ever switched on.

Design rules for rev3, one per lesson:

| #  | Rule |
|----|------|
| R1 | Logic power never depends on USB or on any switched output. Bring-up test: the board boots on battery alone. |
| R2 | Every switch has a source-referenced gate drive and a designed inrush (dV/dt or pre-charge), checked against the FET's SOA. |
| R3 | Short-circuit and over-current protection live in hardware, per output, and act in microseconds without the MCU. |
| R4 | Output states survive an MCU reset. Each output's power-on default is set in hardware. |
| R5 | Everything on the battery bus is rated ≥ 80 V (100 V preferred). |
| R6 | All MCU I/O is 3.3 V. Nothing pulls a GPIO to 5 V. |

---

## 2. Requirements

| Item        | Requirement |
|-------------|-------------|
| Battery     | 10S Li-ion, 30–42 V, 14.5 Ah |
| Rails       | 24 V, 15 V, 12 V, 5 V from external converters of about 300 W each. They can source 12.5 / 20 / 25 / 60 A respectively. |
| Outputs     | Every output switches on its own, with its own current measurement and its own current limit |
| Controller  | ESP32 soldered to the board, not a plug-in module |
| Wireless    | Bluetooth link to the Xbox controllers |
| Host link   | USB to the main PC, relaying controller input to it |
| Motor bus   | CAN on the board, able to drive the EPOS4 50/5 controllers before the PC has booted |
| Size        | Smaller than rev2 (150 × 150 mm) |
| Software    | The ROS 2 driver keeps working (protocol v2) |

---

## 3. Architecture

```
 BATTERY 10S 30-42 V  (pack fuse + BMS live in the harness)
   |
   XT90 in -> TVS -> bulk C -> 4-terminal shunt -> INA228 (V, I, Ah, Wh)
                                   |
                         42 V BUS (always live)
   +---------------+---------------+------------------+---------------+
   |               |               |                  |               |
 AUX PSU      42 V OUT x N     42 V OUT x M      charger port     E-STOP in
 80 V buck    motor drivers    converter feeds   (board side      (gates motor
 5.3 V -> 3V3 TPS48111 + FET   TPS48111 + FET     of the shunt)    outputs in hw)
   |                               |
   |                    external ~300 W converters
   |                               |  24 / 15 / 12 / 5 V come back in
   |                RAIL IN x4 -> TVS -> one eFuse per output -> output connectors
   |                (TPS1663 on 24 V; TPS25947 / TPS25982 on 15 / 12 / 5 V)
   |
 ESP32-S3-WROOM-1U --I2C--> TCA9535 xk  (EN outputs, FLT/PG inputs + INT)
   |   |   |                 ADS7138 xk  (per-output current, rail voltages)
   |   |   |                 INA228      (battery)
   |   |   +-- TWAI -- TCAN1044AV -- CAN -- EPOS4 50/5 x n
   |   +------ USB-C -- main PC  (CDC telemetry + HID gamepad [+ CAN bridge])
   +---------- BLE (u.FL external antenna) -- Xbox controller
```

---

## 4. Power path

### 4.1 Battery input

- **Connector:** XT90, male on the board, because the battery is the source.
- **Fuse:** a 58 V-rated fuse in the pack or harness, sized to the harness, not on the
  board. This frees the space rev2's row of ATO fuses took. Standard ATO blades are rated
  32 V DC and are not suitable on this bus.
- **TVS:** SMCJ45A-class (1500 W) at the connector *(candidate)*.
- **Bulk capacitance:** ≥ 200 µF, 100 V, low ESR, at the input.
- **Battery monitor:** 0.5 mΩ 4-terminal shunt (≥ 3 W) with an **INA228**. The INA228 is
  rated 85 V, has a 20-bit ADC and accumulates charge and energy on chip. This gives real
  coulomb counting instead of the OCV-curve estimate.
- **Charger port:** on the *board side* of the shunt, so charging is counted as well.
- **No master switch.** Per-output switches make it redundant, and dropping it removes the
  board's highest-current switch. The E-stop stays a mechanical device in the battery
  harness; see 5.4 for an additional hardware E-STOP input.

### 4.2 42 V outputs (motor drivers and converter feeds)

| Part | Choice |
|------|--------|
| Driver | **TPS48111-Q1**, one per output. 3.5–80 V operating, 100 V absolute max. Two-level over-current: IWRN with a circuit-breaker timer, and ISCP short-circuit cut-off in 1.2 µs. IMON current monitor ±2 % at 30 mV. FLT_I and FLT_T fault outputs. Integrated pre-charge driver. VSSOP-19 package. |
| FET | One 100 V N-FET, ≤ 3 mΩ, 5 × 6 mm package (*candidate*: Infineon BSC027N10NS5). |
| Shunt | 2 mΩ. 15 A gives about 30 mV, the point where IMON is specified at ±2 %. |
| Pre-charge | Small FET plus power resistor. It charges EPOS4 and converter input capacitors before the main FET closes. Size it with TI's FET-INRUSH-SOA-CALC. |

- **Single FET, not back-to-back.** Regenerative braking and back-driven motors must always
  have a path back to the battery through the body diode. A back-to-back pair would block
  that path whenever the output is off.
- **Starting limits per channel** (finalised during the schematic):
  - EPOS4 50/5 (5 A continuous, about 15 A peak): IWRN ≈ 16 A with a ~100 ms timer,
    ISCP ≈ 40 A.
  - 300 W converter feed (≈ 11 A at 30 V in): IWRN ≈ 14 A, ISCP ≈ 30 A.

### 4.3 Rail inputs (24 / 15 / 12 / 5 V)

- A 300 W converter can source far more than most outputs need. The board's rail copper is
  sized for a **rail design current** (OPEN D2).
- The sum of the per-output limits on a rail is kept at or below that design current. The
  board then cannot be overloaded, even though the converter could deliver more.
- **Input connector:** male on the board, because the converter is the source.
- **TVS:** standoff one step above nominal, for example SMBJ26A / 16A / 13A / 6.0A
  *(candidate)*. The clamp voltage must stay under the eFuse's absolute maximum.
- **Bulk capacitance** at each rail input.
- **No rail fuse.** Each output has its own limit, and the converter's current limit backs
  up the rail.

### 4.4 Per-output switches

| Rail | Part | Range (abs max) | Current limit | Current monitor | Package | Notes |
|------|------|-----------------|---------------|-----------------|---------|-------|
| 42 V | TPS48111-Q1 + FET + shunt | 3.5–80 V (100 V) | IWRN + ISCP, set by resistors | IMON ±2 % | VSSOP-19 | Pre-charge. Single FET for regen. |
| 24 V | TPS1663 | 4.5–60 V (67 V) | 0.6–6 A, ±7 % | IMON 27.9 µA/A, ±6 % | HTSSOP-20 or VQFN-24 | No reverse blocking. MODE pin selects latch-off or auto-retry. |
| 15 / 12 / 5 V, ≤ 5.5 A | TPS259470 | 2.7–23 V (28 V) | 0.5–6 A | ILM pin, ±6 % at ≥ 1 A | QFN 2 × 2 mm | True reverse-current blocking. Variant suffix A = auto-retry, L = latch-off. |
| 15 / 12 / 5 V, ≤ 15 A | TPS25982 | 2.7–24 V (30 V) | 2–15 A, ±8 % | IMON 246 µA/A | QFN 4 × 4 mm | For heavy loads. |

- **Reverse blocking:** on 5–15 V outputs it stops a device with its own supply from
  back-feeding the rail. On the 24 V and 42 V outputs we deliberately do not block, so
  motor regen has a return path.
- **24 V outputs above 6 A,** if any, use the 42 V channel design (TPS48110-Q1 + FET).
- **Every switch has a dV/dt capacitor or pre-charge,** so a capacitive load comes up on a
  defined ramp. With 300 W converters behind them, inrush no longer collapses the rail.

### 4.5 Protection layers

1. **Pack fuse + BMS** (harness): wiring and pack.
2. **Bus and rail TVS:** transients.
3. **Per-output short-circuit cut-off** (µs, no MCU involved): device and cable faults.
   This is what keeps a shorted device or cable from taking a 300 W rail with it.
4. **Per-output over-current with timer:** sustained overloads.
5. **Thermal shutdown** in every eFuse and driver.
6. **Converter current limit:** rail-level backstop.
7. **Firmware policy:** retry limits, events, telemetry, user commands.

---

## 5. Control and telemetry

### 5.1 Switching

- Each output's EN is driven by a **TCA9535** 16-bit I2C GPIO expander (TSSOP-24,
  1.65–5.5 V).
- The expander holds its outputs while the ESP32 resets, so an MCU crash or a reflash does
  not drop the rover (R4).
- **Power-on defaults (R4):** at power-on the TCA9535 pins are inputs. Each EN therefore
  gets a pull-resistor footprint: pull-up means default ON, pull-down means default OFF.
  For example, the main PC and the network switch default ON, so the rover boots even with
  a dead or unflashed MCU.
- **Firmware rules:**
  - On boot, read the expander state back. Never reset the outputs.
  - Before switching a pin to output, write its current level, so it does not glitch.
  - Enable outputs one by one, 50–100 ms apart, never all at once.

### 5.2 Faults

FLT and PG lines go to a second set of TCA9535 inputs. Their INT line goes to an ESP32
GPIO, so faults are pushed to the MCU rather than polled.

### 5.3 Current and voltage measurement

- **Per-output current:** each IMON or ILM output feeds an **ADS7138** (8-channel, 12-bit,
  I2C, WQFN-16, eight selectable addresses), one per eight outputs. The ADC's
  auto-sequence reads every channel at tens of Hz.
- **Rail voltages:** spare ADS7138 channels, through resistor dividers.
- **Rail current:** the sum of that rail's outputs.
- **Battery:** INA228.

I2C address map (one bus, 400 kHz; the ESP32-S3's second I2C controller stays free for an
external header):

| Device  | Addresses |
|---------|-----------|
| ADS7138 | 0x10–0x17 (resistor on ADDR) |
| TCA9535 | 0x20–0x27 (A2..A0) |
| INA228  | 0x40 |

### 5.4 E-STOP input (recommended)

A normally-closed loop input. Opening it forces the motor-output channels off in hardware,
by gating their EN, independent of the firmware. The ESP32 also reads its state.

---

## 6. Controller

### 6.1 MCU: ESP32-S3-WROOM-1U

- **Soldered SMD module:** the radio is pre-certified and needs no RF layout from us.
  A chip-down ESP32 design is not worth the RF work.
- **The -1U variant** has a u.FL connector for an external antenna. That is needed because
  the board will sit inside the rover's body.
- **Why the S3 over the original ESP32:**
  - Native USB, so one port can be a composite device (CDC + HID + CAN bridge) with no
    USB-UART chip.
  - A TWAI (CAN) controller.
  - BLE 5 and plenty of GPIO.
  - The catch: it has **no Bluetooth Classic** (see 6.2).

### 6.2 Xbox controllers

- Bluepad32 supports Xbox Wireless controllers on the ESP32-S3 **over BLE only**: models
  1708 and 1914 running controller firmware v5.x or newer.
- Older controller firmware uses Bluetooth Classic, which the S3 lacks. Update the pads
  with the Xbox Accessories app.
- If a Classic-only pad must work, use the original ESP32 with a USB-UART bridge instead,
  and give up native USB.

### 6.3 USB to the PC

- **Hardware:** USB-C (USB 2.0 full speed), USBLC6-2SC6 ESD array *(candidate)*, 5.1 kΩ CC
  pull-downs.
- **VBUS** feeds only the logic node, through a diode, for bench flashing (section 7). It
  never powers anything else.
- **The board enumerates as one composite device:**
  - CDC: the JSON line protocol v2 for the ROS driver.
  - HID gamepad: the Xbox controller re-exposed as a standard USB joystick, so ROS 2
    `joy_node` reads it with no custom parsing.
  - *Optional:* a second CDC acting as an slcan CAN bridge, which appears on the PC as
    SocketCAN `can0` (OPEN D3).

### 6.4 CAN

- **Transceiver:** TWAI → **TCAN1044AV-Q1**.
  - VIO 1.8–5.5 V, so it runs directly on 3.3 V logic.
  - ±58 V bus-fault protection, which matters when a CAN wire can touch 42 V.
  - CAN FD capable up to 8 Mbit/s; the EPOS4 runs classic CANopen at ≤ 1 Mbit/s.
- **Board hardware:** split 120 Ω termination on a solder jumper, an ESD diode, and two
  connectors for daisy-chaining.
- **Safety:**
  - The ESP32 produces a CANopen heartbeat, and each EPOS4 is configured to consume it.
  - The gamepad link is watched as well.
  - A dead MCU or a lost controller therefore stops the drives.
  - Drive authority (gamepad or PC) is explicit, with a handover and timeouts.

### 6.5 Boot and flashing

- BOOT and RESET buttons.
- Strapping pins (GPIO0, 3, 45, 46) are not used for I/O.
- Flashing and debugging go through the S3's built-in USB-Serial/JTAG.

---

## 7. Aux power (always on)

- **Buck:** **LMR38010**, 4.2–80 V in, 1 A, synchronous, HSOIC-8 PowerPAD. It is much easier
  to place and inspect than rev2's 2 × 3 mm TDFN. It is fed from the 42 V bus *before* any
  output switch and set to 5.3 V.
- **5 V logic node:** a Schottky diode-OR of the 5.3 V buck and USB VBUS. It feeds the CAN
  transceiver's VCC and a 1 A, 3.3 V LDO (SOT-223) that supplies the ESP32, expanders, ADCs
  and INA228.
- **Budget:** under 0.5 A at 5 V.
  - ESP32-S3 with BLE: roughly 100–150 mA, with peaks (more if Wi-Fi is used).
  - CAN transceiver: tens of mA when dominant.
  - Everything else: well under 50 mA.
- **Bring-up test (R1):** battery only, USB disconnected. 5 V and 3.3 V must be present and
  the ESP32 must boot.

---

## 8. Size and layout

- **Where rev2's 150 × 150 mm went:** 42 output connectors and 5 inputs, a row of 5 ATO
  fuses, D2PAK FETs, and the Pico module (51 × 21 mm).
- **What rev3 removes:** the fuse row, the D2PAKs on the rails (the eFuses integrate the
  FET), and the Pico, replaced by a module of about 18 × 20 mm.
- **Connectors now set the size floor** (OPEN D1). A rough guess, to firm up after
  placement: about 24 outputs fit in roughly 100 × 120 mm.
- **Stackup:** 4 layers, 2 oz outer copper, inner layers as solid GND and power.
- **Pads:** power-connector pads connect solid, with no thermal spokes.
- **Repeated channels:** each channel type is one hierarchical KiCad sheet instanced N
  times. A fix to one channel then fixes all of them.
- **Assembly:** the QFN parts (TPS25947 2 × 2 mm, TPS25982 4 × 4 mm, ADS7138 3 × 3 mm) need a
  stencil and reflow, or PCBA (OPEN D5).

---

## 9. Firmware and ROS impact

- **New firmware** in `firmware/esp32/`, built on ESP-IDF or Arduino-ESP32, with Bluepad32
  and a CANopen stack such as CANopenNode.
- **Protocol v2:**
  - Channels replace the five fixed branches. Each channel reports on/off, current, limit,
    fault and default state.
  - It adds battery Ah/Wh, E-STOP state and controller state.
  - Bump `jsonSchemaVersion` and `SUPPORTED_SCHEMA` together.
- **ROS driver:**
  - Per-channel arrays.
  - `joy_node` for the gamepad over HID.
  - If D3 goes the bridge route, ros2_canopen on `can0`.

---

## 10. Open decisions

| #   | Decision | Recommended default |
|-----|----------|---------------------|
| D1  | Output list: how many per rail, what each powers, max current per output | Trim to what the rover uses, plus 1–2 spares per rail |
| D2  | Rail design current with 300 W converters | 24 V 12.5 A, 15 V 20 A, 12 V 25 A, 5 V 30 A |
| D3  | Who owns the CAN bus once the PC is up | The ESP32 stays the only CANopen master; the PC sends drive commands to it over USB. Alternatives: the PC takes over through its own adapter, or through the ESP32 acting as a USB-CAN bridge. |
| D4  | How the PC talks to the EPOS4s today | (needed to settle D3) |
| D5  | Assembly | PCBA for the fine-pitch parts |
| D6  | Converters: external modules, each fed from a switched 42 V output on this board? | Yes |
| D7  | Connector family per voltage, to stop 12 V / 15 V mix-ups | One family per voltage; male inputs, female outputs |
| D8  | Controller models and firmware | 1708 / 1914, updated to firmware ≥ 5 |
| D9  | Board outline and mounting holes | (from the mechanical team) |
| D10 | Hardware E-STOP input | Yes, gating the motor outputs only |

---

## 11. Next steps

1. Settle D1–D10.
2. Do the per-channel calculations: limits, IMON scaling, dV/dt, and pre-charge versus FET
   SOA.
3. Draw the schematic in KiCad 9, with one hierarchical sheet per channel type.
4. Lay out the board, then write the bring-up plan and the ESP32 firmware.

---

## Sources

- TI TPS4811-Q1 datasheet, SLUSEE5E: https://www.ti.com/lit/ds/symlink/tps4811-q1.pdf
- TI TPS25947 datasheet, SLVSFC9: https://www.ti.com/lit/ds/symlink/tps25947.pdf
- TI TPS1663 datasheet, SLVSET9: https://www.ti.com/lit/ds/symlink/tps1663.pdf
- TI TPS25982 datasheet, SLVSEI3: https://www.ti.com/lit/ds/symlink/tps25982.pdf
- TI ADS7138 datasheet: https://www.ti.com/lit/ds/symlink/ads7138.pdf
- TI TCA9535 datasheet: https://www.ti.com/lit/ds/symlink/tca9535.pdf
- TI LMR38010 datasheet, SNVSC73: https://www.ti.com/lit/ds/symlink/lmr38010.pdf
- TI TCAN1044AV-Q1: https://www.ti.com/product/TCAN1044AV-Q1
- TI INA228: https://www.ti.com/product/INA228
- Espressif ESP32-S3-WROOM-1/1U datasheet: https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf
- Bluepad32 supported gamepads: https://bluepad32.readthedocs.io/en/latest/supported_gamepads/
- maxon EPOS4 feature chart: https://www.maxongroup.com/medias/sys_master/root/8934669025310/EPOS4-Feature-Chart-En.pdf
- Littelfuse ATO (32 V DC): https://littelfuse.com/products/fuses/automotive-aftermarket-products/blade-fuses/ato.aspx

Parts marked *(candidate)* have not yet been checked against their datasheets.
