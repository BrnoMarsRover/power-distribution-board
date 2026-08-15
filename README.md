# Power distribution board

Everything for the BrnoMarsRover power distribution board: the schematic and PCB, the
firmware that runs on it, and the ROS 2 driver that consumes it.

```
kicad/      schematic and PCB
pdf/        exported schematics
firmware/   Raspberry Pi Pico 2 sketch (Arduino, arduino-pico core)
ros2/       ROS 2 driver package (ament_python, Humble)
```

The firmware and the driver live together **because they share a versioned wire
protocol**. Every field name and status bit below is written twice - once in
`firmware/PowerBoard/Functions.cpp` and once in `ros2/powerboard/powerboard/powerboard_node.py`.
Splitting them across repositories would let the two halves be changed, reviewed and
deployed independently, and the failure mode is not a build error - it is silently
misparsed telemetry. Change the protocol in one commit, and bump `jsonSchemaVersion`
(firmware `Config.cpp`) together with `SUPPORTED_SCHEMA` (node) when you do.

## Hardware

Five INA238 current/voltage monitors on I2C, one per branch, each with a high-side
switch:

| Designator | I2C  | Shunt  | Rail             | Limit   |
|-----------|------|--------|------------------|---------|
| U2        | 0x40 | 3 mOhm | battery feed, and the master gate for the rest | 15 A |
| U3        | 0x41 | 5 mOhm | 24 V             | 2.5 A   |
| U4        | 0x42 | 5 mOhm | 15 V             | 4 A     |
| U5        | 0x43 | 5 mOhm | 12 V             | 5 A     |
| U6        | 0x44 | 5 mOhm | 5 V              | 12 A    |

Limits come from the 60 W rating of each DC-DC converter.

**Over-current protection ships disabled** (`ocpEnabled = false`). Some high-side
drivers were tripped by inrush and are bridged, so the board cannot actually switch
those branches off. Limits are still evaluated and reported, so an overload is visible
in telemetry even though nothing acts on it. `OCP ON` enables tripping if the hardware
is ever repaired.

Two things the board cannot measure, by construction:

- **Charging current.** The charger is wired on the battery side of U2's shunt, so
  charge current never crosses it. The driver infers charging from rising open-circuit
  voltage instead.
- **Currents above ~8 A on the INA CURRENT register**, which saturates there. The
  shunt-derived value is used for protection and for the `i` field because it has the
  range; `ir` is kept alongside it only for cross-checking.

## Serial protocol (schema v1)

USB CDC-ACM, one JSON object per line, telemetry at 2 Hz. Anything not starting with
`{` is human-readable diagnostics (`STATUS`, `RAW`) and is ignored by machine readers.

`"t"` gives the line type: `tel`, `evt`, `ack`, `err`, `info`.

### `tel` - periodic telemetry

```json
{"t":"tel","v":1,"seq":123,"up":61500,"ocp":0,"mst":1,"b":[ ... ]}
```

| Key   | Meaning                                        |
|-------|------------------------------------------------|
| `v`   | schema version                                  |
| `seq` | frame counter; gaps mean frames were lost       |
| `up`  | milliseconds since board boot                   |
| `ocp` | 1 if over-current protection is enabled         |
| `mst` | 1 if the U2 master branch is on                 |
| `b`   | one object per branch                           |

Each branch object:

| Key   | Unit | Meaning                                                   |
|-------|------|-----------------------------------------------------------|
| `n`   |      | designator, `U2`..`U6`                                    |
| `a`   |      | I2C address                                               |
| `s`   |      | status bits, see below                                    |
| `lim` | mA   | configured current limit                                  |
| `vb`  | V    | bus voltage                                               |
| `vs`  | mV   | shunt voltage                                             |
| `i`   | mA   | current derived from the shunt - **this is the real one** |
| `ir`  | mA   | INA CURRENT register, saturates at ~8 A, cross-check only |
| `tc`  | degC | INA die temperature                                       |
| `tr`  |      | cumulative trip count                                     |

Status bits in `s`:

| Bit  | Name        |
|------|-------------|
| 0x01 | online      |
| 0x02 | enabled     |
| 0x04 | tripped     |
| 0x08 | manual off  |
| 0x10 | powered     |
| 0x20 | over limit  |

### `evt` - pushed immediately, not on the telemetry tick

```json
{"t":"evt","up":61500,"ev":"over_limit","n":"U5","i":5200.0,"lim":5000,"ocp":0}
```

`ev` is one of `over_limit`, `over_limit_clear`, `ocp_trip`, `ocp_recover`. These are
pushed as they happen because with OCP disabled an over-limit event is the only
actionable signal the board can give.

### `ack` / `err` / `info`

```json
{"t":"ack","cmd":"JSON","msg":"json output"}
{"t":"err","cmd":"OFF U9","msg":"unknown branch"}
{"t":"info","fw":"...","v":1,"rails":5,"ocp":0}
```

### Commands (host to board, one per line)

| Command        | Effect                                          |
|----------------|-------------------------------------------------|
| `ON <U>`       | switch a branch on                              |
| `OFF <U>`      | switch a branch off                             |
| `OCP [ON|OFF]` | query or set over-current protection            |
| `RAW`          | dump every INA238 register, human readable      |
| `STATUS`       | human readable summary                          |
| `JSON`         | machine-readable output (the default)           |
| `HUMAN`        | switch to the plain-text table                  |
| `ONCE`         | emit a single telemetry frame                   |

## Firmware

Arduino sketch in `firmware/PowerBoard`, target Raspberry Pi Pico 2 (`rpipico2`),
arduino-pico core 5.5.1. Open `PowerBoard.ino` and flash it.

## ROS 2 driver

`ros2/powerboard`, ament_python, tested on Humble. Depends only on `rclpy`,
`std_msgs`, `sensor_msgs`, `diagnostic_msgs` and `python3-serial` - no custom
interfaces.

```bash
ros2 launch powerboard powerboard.launch.py
```

| Topic                | Type                       |
|----------------------|----------------------------|
| `~/battery`          | `sensor_msgs/BatteryState` |
| `~/rails/voltages`   | `std_msgs/Float32MultiArray` |
| `~/rails/currents`   | `std_msgs/Float32MultiArray` |
| `~/rails/powers`     | `std_msgs/Float32MultiArray` |
| `~/events`           | `std_msgs/String`          |
| `~/command`          | `std_msgs/String` (subscription, forwards any board command) |
| `/diagnostics`       | `diagnostic_msgs/DiagnosticArray` |

The rail arrays carry names in `layout.dim[i].label`, index-aligned with `data[i]`.

State of charge is estimated from a LiPo open-circuit-voltage curve, load-compensated
and smoothed, and published in `BatteryState.percentage`. It is a rough guide, not a
fuel gauge: between about 3.8 and 3.9 V/cell the curve is nearly flat, and below
3.7 V/cell it falls away sharply. Defaults assume Freya's **10S, 14.5 Ah** pack; see
`cell_count`, `design_capacity_ah` and `pack_resistance_ohm`.

**Only one process may hold the serial port.** Two readers steal bytes from each other,
which looks exactly like a flaky board - sequence gaps and a halved frame rate.
