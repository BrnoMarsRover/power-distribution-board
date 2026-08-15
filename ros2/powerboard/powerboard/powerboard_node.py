# ROS 2 driver for the BrnoMarsRover power distribution board.
#
# The board is a Raspberry Pi Pico 2 on USB CDC-ACM emitting one JSON object per line
# at 2 Hz (firmware: BrnoMarsRover/power-distribution-board, schema v1). This node
# parses that stream and republishes it, and forwards commands back to the board.
#
# Line types on the wire, distinguished by "t":
#   tel   periodic telemetry, one per cycle
#   evt   over_limit / over_limit_clear / ocp_trip / ocp_recover, pushed immediately
#   ack   command succeeded          err   command failed
#   info  emitted once at startup, carries the schema version
# Anything that is not JSON (the STATUS and RAW human diagnostics) is ignored by design.

import json
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String

from powerboard.serial_line_reader import SerialLineReader

# Schema this node was written against. The board reports its own in "v"; a mismatch is
# worth shouting about rather than silently misreading fields.
SUPPORTED_SCHEMA = 1

# Status bitfield in "s", as defined by the firmware.
S_ONLINE = 0x01
S_ENABLED = 0x02
S_TRIPPED = 0x04
S_MANUAL_OFF = 0x08
S_POWERED = 0x10
S_OVER_LIMIT = 0x20

# The board reports schematic designators (U2..U6), which mean nothing without the
# schematic in front of you. Map them to names a person can read. Overridable with the
# `rail_names` parameter as "U2:battery,U3:rail_24v,..." so the mapping can follow what
# each rail actually feeds without touching this file.
DEFAULT_RAIL_NAMES = {
    'U2': 'battery',      # 41 V battery feed, also the master gate for the others
    'U3': 'rail_24v',
    'U4': 'rail_15v',
    'U5': 'rail_12v',
    'U6': 'rail_5v',
}

# Resting (open-circuit) volts per cell -> state of charge, for LiPo. Freya carries a
# 10S LiPo, so a full pack is 42.0 V and 3.7 V/cell nominal is 37.0 V.
#
# Voltage is the only thing available: the board is a power monitor, not a fuel gauge,
# and coulomb counting is impossible because nothing here knows how full a pack was when
# it was plugged in, and packs get swapped.
#
# Treat the output as a rough guide, not a fuel gauge. Between roughly 3.8 and 3.9 V/cell
# the LiPo curve is almost flat, so a 0.1 V measurement error swings the answer by tens
# of percent; below ~3.7 V/cell it falls off a cliff and gets much sharper. That is the
# nature of voltage-based SoC, not a defect in this table.
LIPO_OCV_SOC = [
    (3.27, 0.00), (3.61, 0.05), (3.69, 0.10), (3.71, 0.15), (3.73, 0.20),
    (3.75, 0.25), (3.77, 0.30), (3.79, 0.35), (3.80, 0.40), (3.82, 0.45),
    (3.84, 0.50), (3.85, 0.55), (3.87, 0.60), (3.91, 0.65), (3.95, 0.70),
    (3.98, 0.75), (4.02, 0.80), (4.08, 0.85), (4.11, 0.90), (4.15, 0.95),
    (4.20, 1.00),
]


class PowerboardNode(Node):

    def __init__(self):
        super().__init__('powerboard')

        # by-id is the default because /dev/ttyACM* numbering moves when other USB
        # serial devices appear - the Xsens IMU on this NUC is exactly such a device.
        self.declare_parameter('port', '/dev/serial/by-id/usb-Raspberry_Pi_Pico_2_*-if00')
        self.declare_parameter('poll_rate_hz', 50.0)
        self.declare_parameter('stale_timeout_s', 3.0)
        self.declare_parameter('reconnect_period_s', 2.0)
        # Fraction of a branch's limit above which we report a warning. The board only
        # flags a hard over-limit; this gives earlier notice.
        self.declare_parameter('warn_fraction', 0.9)
        self.declare_parameter('frame_id', 'powerboard')
        # "U2:battery,U3:rail_24v" - anything unlisted keeps its designator.
        self.declare_parameter('rail_names', '')

        # -- battery state of charge -----------------------------------------------
        # Freya's pack is 10S LiPo. Set to 0 to publish NaN instead of an estimate.
        self.declare_parameter('cell_count', 10)
        # Whole-pack internal resistance, used to recover open-circuit voltage from the
        # loaded terminal voltage: without it the estimate dips whenever the motors pull
        # and springs back when they stop. ~6 mOhm per cell is typical for a pack this
        # size; measure it by taking (V_idle - V_loaded) / current and set it here.
        self.declare_parameter('pack_resistance_ohm', 0.06)
        # The curve is steep near empty and the load varies fast, so smooth the result.
        # This is a display quantity - lag is much cheaper than a jumpy number.
        self.declare_parameter('soc_filter_tau_s', 30.0)
        self.declare_parameter('soc_warn_percent', 25.0)
        self.declare_parameter('soc_error_percent', 15.0)
        # Nameplate capacity of the pack, in Ah. Used for BatteryState.design_capacity,
        # for restating the estimate in Ah, and for the runtime estimate. 0 disables all
        # three. BatteryState.capacity - the capacity this pack can still hold after
        # ageing - stays NaN, because nothing here measures it and copying the nameplate
        # figure into it would assert the pack is as-new.
        self.declare_parameter('design_capacity_ah', 14.5)
        # Charging is INVISIBLE to this board: the charger sits on the battery side of
        # U2's shunt, so charge current never crosses it and the shunt only ever sees
        # the rover's own draw. It has to be inferred instead, from the one signature
        # that a discharging pack cannot produce - open-circuit voltage rising while a
        # normal load current is flowing.
        #
        # Threshold sits between the two measured rates: the real charger gives about
        # +4.2 mV per cell per minute (+2.5 V/h at the pack), while a resting pack drifts
        # by well under 1 mV per cell per minute - quantisation is 0.31 mV/cell and the
        # load compensation contributes about 0.24 mV/cell at the observed current
        # ripple. 2 mV/cell is roughly twice the noise and half the signal.
        #
        # Known blind spot: near full, the charger moves to constant voltage, the current
        # tapers and the rise flattens out, so detection drops back to "not charging".
        # That is benign - by then the estimate is near 100% anyway.
        self.declare_parameter('charge_detect_window_s', 60.0)
        self.declare_parameter('charge_detect_rise_v_per_cell', 0.002)
        self.declare_parameter('charge_detect_enabled', True)

        self.port_pattern = self.get_parameter('port').value
        self.stale_timeout = float(self.get_parameter('stale_timeout_s').value)
        self.warn_fraction = float(self.get_parameter('warn_fraction').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.rail_names = self._parse_rail_names(self.get_parameter('rail_names').value)
        self.cell_count = int(self.get_parameter('cell_count').value)
        self.pack_resistance = float(self.get_parameter('pack_resistance_ohm').value)
        self.soc_tau = float(self.get_parameter('soc_filter_tau_s').value)
        self.soc_warn = float(self.get_parameter('soc_warn_percent').value)
        self.soc_error = float(self.get_parameter('soc_error_percent').value)
        self.design_capacity = float(self.get_parameter('design_capacity_ah').value)
        self.charge_window = float(self.get_parameter('charge_detect_window_s').value)
        self.charge_rise = float(self.get_parameter('charge_detect_rise_v_per_cell').value)
        self.charge_detect = bool(self.get_parameter('charge_detect_enabled').value)

        self.reader = SerialLineReader(self.port_pattern)

        # -- publishers ------------------------------------------------------------
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        # U2 is the actual battery feed, so BatteryState is a genuine fit there. The
        # derived rails are not batteries, so they go into the RailArray topics below
        # rather than being forced into a message that promises charge state they do
        # not have.
        # State of charge rides in BatteryState.percentage (0..1, per the ROS convention
        # for that field). There was briefly a separate ~/battery/percentage Float32 in
        # percent, but it carried the same number scaled by 100 - two sources of one
        # truth, free to drift apart. /diagnostics already has soc_percent for anyone who
        # wants percent rather than a fraction.
        self.battery_pub = self.create_publisher(BatteryState, '~/battery', 10)
        self.event_pub = self.create_publisher(
            String, '~/events', QoSPresetProfiles.SENSOR_DATA.value)
        # One message per quantity rather than a topic per rail: a consumer almost
        # always wants the whole set at one instant, and separate topics cannot be
        # correlated to a single sample.
        #
        # Float32MultiArray keeps this to standard messages - no custom interface for
        # consumers to build. data[i] is index aligned with layout.dim[i].label, which
        # carries the readable rail name, so the arrays stay self describing. That is a
        # mild stretch of MultiArrayLayout, whose dims are really meant to describe
        # tensor shape, but it beats publishing bare numbers whose meaning depends on
        # remembering the order. /diagnostics carries the same values fully named if
        # you want the unambiguous view.
        self.voltages_pub = self.create_publisher(Float32MultiArray, '~/rails/voltages', 10)
        self.currents_pub = self.create_publisher(Float32MultiArray, '~/rails/currents', 10)
        self.powers_pub = self.create_publisher(Float32MultiArray, '~/rails/powers', 10)

        # -- command input ---------------------------------------------------------
        # A String topic rather than services: it exposes the board's full command set
        # (ON/OFF per branch, OCP ON/OFF, RAW, HUMAN) without inventing a custom srv
        # for each. Anything not understood comes back as an "err" line.
        self.create_subscription(String, '~/command', self._on_command, 10)

        # -- state -----------------------------------------------------------------
        self._last_frame_time = None
        self._last_seq = None
        self._seq_gaps = 0
        self._board_schema = None
        self._parse_errors = 0
        self._reported_stale = False
        self._soc = None            # filtered, 0..1
        self._soc_raw = None        # this sample only, 0..1
        self._soc_time = None
        self._v_cell = None
        self._ocv_cell = None
        self._current_A = 0.0
        self._charging = False
        self._ocv_history = deque()   # (t, volts per cell), trimmed to the window
        self._ocv_rise = 0.0          # volts per cell across the window, signed

        poll_period = 1.0 / max(1.0, float(self.get_parameter('poll_rate_hz').value))
        self.create_timer(poll_period, self._poll)
        self.create_timer(float(self.get_parameter('reconnect_period_s').value),
                          self._ensure_connected)
        self.create_timer(1.0, self._check_stale)

        self.get_logger().info(f"powerboard node started, looking for {self.port_pattern}")

    @staticmethod
    def _parse_rail_names(spec):
        names = dict(DEFAULT_RAIL_NAMES)
        for pair in (spec or '').split(','):
            pair = pair.strip()
            if not pair or ':' not in pair:
                continue
            designator, human = pair.split(':', 1)
            names[designator.strip()] = human.strip()
        return names

    def _rail_name(self, designator):
        """Readable name for a rail, falling back to the designator if unmapped."""
        return self.rail_names.get(designator, designator)

    # -- connection ----------------------------------------------------------------

    def _ensure_connected(self):
        if self.reader.connected:
            return
        path = self.reader.connect()
        if path is None:
            return
        self.get_logger().info(f"connected to {path}")
        # The board may have been left in HUMAN mode by someone with a terminal open.
        # Asking for JSON makes the node self-correcting rather than silently seeing
        # nothing it can parse.
        self.reader.write_line('JSON')

    def _drop_connection(self, reason):
        if self.reader.connected:
            self.get_logger().warn(f"disconnected: {reason}")
        self.reader.close()
        self._last_seq = None

    # -- reading -------------------------------------------------------------------

    def _poll(self):
        if not self.reader.connected:
            return
        try:
            lines = self.reader.poll()
        except OSError as exc:
            self._drop_connection(str(exc))
            return
        for line in lines:
            self._handle_line(line)

    def _handle_line(self, line):
        if not line:
            return
        # STATUS and RAW output is human text on the same stream by design, so a line
        # that is not JSON is skipped rather than treated as an error.
        if not line.startswith('{'):
            return
        try:
            msg = json.loads(line)
        except ValueError:
            self._parse_errors += 1
            return

        kind = msg.get('t')
        if kind == 'tel':
            self._handle_telemetry(msg)
        elif kind == 'evt':
            self._handle_event(msg)
        elif kind in ('ack', 'err'):
            text = f"{msg.get('cmd', '?')}: {msg.get('msg', '')}"
            if kind == 'ack':
                self.get_logger().info(f"board ack {text}")
            else:
                self.get_logger().warn(f"board err {text}")
        elif kind == 'info':
            self._board_schema = msg.get('v')
            self.get_logger().info(
                f"board fw={msg.get('fw')} schema=v{msg.get('v')} "
                f"rails={msg.get('rails')} ocp={msg.get('ocp')}")
            self._check_schema()

    def _handle_event(self, msg):
        ev = msg.get('ev', '?')
        designator = msg.get('n', '?')
        name = f"{self._rail_name(designator)} ({designator})"
        text = (f"{ev} {name} i={msg.get('i')}mA lim={msg.get('lim')}mA "
                f"ocp={msg.get('ocp')}")
        # Over-limit with OCP disabled is the case that matters most on this board:
        # nothing will be switched off, so the log is the only warning anyone gets.
        if ev in ('over_limit', 'ocp_trip'):
            self.get_logger().error(f"POWERBOARD EVENT: {text}")
        else:
            self.get_logger().info(f"powerboard event: {text}")
        out = String()
        out.data = json.dumps(msg, separators=(',', ':'))
        self.event_pub.publish(out)

    def _check_schema(self):
        if self._board_schema is not None and self._board_schema != SUPPORTED_SCHEMA:
            self.get_logger().error(
                f"board reports schema v{self._board_schema} but this node understands "
                f"v{SUPPORTED_SCHEMA}; fields may be misread")

    # -- telemetry -----------------------------------------------------------------

    def _handle_telemetry(self, msg):
        now = self.get_clock().now()
        self._last_frame_time = now
        if self._reported_stale:
            self.get_logger().info("powerboard stream recovered")
            self._reported_stale = False

        if self._board_schema is None:
            self._board_schema = msg.get('v')
            self._check_schema()

        # A gap means frames were dropped somewhere. Without this, stale or thinned
        # data is indistinguishable from healthy data.
        seq = msg.get('seq')
        if seq is not None and self._last_seq is not None:
            expected = (self._last_seq + 1) & 0xFFFFFFFF
            if seq != expected:
                self._seq_gaps += 1
                self.get_logger().warn(
                    f"telemetry gap: expected seq {expected}, got {seq}")
        self._last_seq = seq

        ocp_enabled = bool(msg.get('ocp', 0))
        master_on = bool(msg.get('mst', 1))
        branches = msg.get('b', [])

        diag = DiagnosticArray()
        diag.header.stamp = now.to_msg()
        diag.header.frame_id = self.frame_id

        labels, volts, amps, watts = [], [], [], []
        for br in branches:
            diag.status.append(self._branch_status(br, ocp_enabled, master_on))
            designator = br.get('n', '?')
            voltage = float(br.get('vb', 0.0))
            current_mA = float(br.get('i', 0.0))
            # Keep the designator in the label as well: the schematic, the firmware and
            # anyone holding a multimeter all speak in U-numbers.
            labels.append(f'{self._rail_name(designator)}({designator})')
            volts.append(voltage)
            amps.append(current_mA)
            watts.append(voltage * current_mA / 1000.0)
            if br.get('a') == 0x40:          # U2, the battery feed
                self._publish_battery(br, now)

        self.voltages_pub.publish(self._rail_array(labels, volts, 'V'))
        self.currents_pub.publish(self._rail_array(labels, amps, 'mA'))
        self.powers_pub.publish(self._rail_array(labels, watts, 'W'))

        if self._soc is not None:
            diag.status.append(self._charge_status())
        diag.status.append(self._board_status(msg, ocp_enabled, master_on))
        self.diag_pub.publish(diag)

    def _charge_status(self):
        pct = self._soc * 100.0
        st = DiagnosticStatus()
        st.name = 'powerboard: battery charge'
        st.hardware_id = self.frame_id

        if self._charging:
            # Never ERROR while charging: the pack is being dealt with, and a red alarm
            # that nobody can act on is noise. It stays a warning rather than OK because
            # the number is an over-estimate - the charger holds the terminal voltage
            # above the pack's true open-circuit voltage, and by exactly how much cannot
            # be known without seeing the charge current.
            st.level = (DiagnosticStatus.OK if pct > self.soc_warn
                        else DiagnosticStatus.WARN)
            st.message = (f'charging, {pct:.0f}% (estimate reads high while on charge; '
                          f'true value settles a few minutes after unplugging)')
        elif pct <= self.soc_error:
            st.level = DiagnosticStatus.ERROR
            st.message = f'battery critically low, {pct:.0f}% - stop and swap the pack'
        elif pct <= self.soc_warn:
            st.level = DiagnosticStatus.WARN
            st.message = f'battery low, {pct:.0f}%'
        else:
            st.level = DiagnosticStatus.OK
            st.message = f'{pct:.0f}%'

        st.values = [
            KeyValue(key='soc_percent', value=f'{pct:.1f}'),
            # Unfiltered, so a sudden drop is visible before the smoothed value catches up.
            KeyValue(key='soc_percent_unfiltered', value=f'{self._soc_raw * 100.0:.1f}'),
            KeyValue(key='volts_per_cell_loaded', value=f'{self._v_cell:.3f}'),
            KeyValue(key='volts_per_cell_open_circuit', value=f'{self._ocv_cell:.3f}'),
            KeyValue(key='cell_count', value=str(self.cell_count)),
            KeyValue(key='pack_resistance_ohm', value=f'{self.pack_resistance:.3f}'),
            KeyValue(key='estimator', value='LiPo open-circuit voltage curve, load compensated'),
            KeyValue(key='charging', value=str(self._charging)),
            # Inferred, because the charger is wired on the battery side of the shunt and
            # its current never crosses it.
            KeyValue(key='charging_detected_from',
                     value=f'OCV trend {self._ocv_rise * 1000.0:+.1f} mV/cell '
                           f'over {self.charge_window:.0f} s'),
        ]

        if self.design_capacity > 0.0:
            remaining_Ah = self._soc * self.design_capacity
            st.values.append(KeyValue(key='design_capacity_Ah',
                                      value=f'{self.design_capacity:.2f}'))
            st.values.append(KeyValue(key='remaining_Ah', value=f'{remaining_Ah:.2f}'))
            # Straight-line extrapolation of the current draw. Honest for a steady load,
            # optimistic the moment the rover starts driving, and meaningless once the
            # voltage curve goes soft near empty - so it is a hint, not a countdown.
            if self._charging:
                # Time-to-full would need the charge current, which is the one thing
                # this board cannot see. Better to say so than to print a number.
                st.values.append(KeyValue(key='estimated_runtime_min',
                                          value='n/a (charging)'))
            elif self._current_A >= 0.05:
                st.values.append(KeyValue(
                    key='estimated_runtime_min',
                    value=f'{remaining_Ah / self._current_A * 60.0:.0f}'))
            else:
                st.values.append(KeyValue(key='estimated_runtime_min',
                                          value='n/a (no significant load)'))
        return st

    def _branch_status(self, br, ocp_enabled, master_on):
        designator = br.get('n', '?')
        name = self._rail_name(designator)
        s = int(br.get('s', 0))
        limit = float(br.get('lim', 0.0))
        current = float(br.get('i', 0.0))
        frac = (current / limit) if limit > 0 else 0.0

        st = DiagnosticStatus()
        st.name = f'powerboard: {name}'
        st.hardware_id = f"{self.frame_id}:0x{int(br.get('a', 0)):02X}"

        if not (s & S_ONLINE):
            st.level = DiagnosticStatus.ERROR
            st.message = 'INA238 offline'
        elif s & S_TRIPPED:
            st.level = DiagnosticStatus.ERROR
            st.message = 'tripped by over-current protection'
        elif s & S_OVER_LIMIT:
            # With OCP off the board will not act, so this has to be loud here.
            st.level = DiagnosticStatus.ERROR if not ocp_enabled else DiagnosticStatus.WARN
            st.message = (f'OVER LIMIT {current:.0f} / {limit:.0f} mA'
                          + ('  (OCP disabled - branch will NOT be switched off)'
                             if not ocp_enabled else ''))
        elif s & S_MANUAL_OFF:
            st.level = DiagnosticStatus.OK
            st.message = 'switched off by command'
        elif not (s & S_POWERED):
            st.level = DiagnosticStatus.WARN
            st.message = ('unpowered because the U2 master branch is off'
                          if not master_on else 'unpowered')
        elif frac >= self.warn_fraction:
            st.level = DiagnosticStatus.WARN
            st.message = f'{frac * 100:.0f}% of limit'
        else:
            st.level = DiagnosticStatus.OK
            st.message = 'ok'

        st.values = [
            KeyValue(key='voltage_V', value=f"{br.get('vb', 0.0):.3f}"),
            KeyValue(key='current_mA', value=f"{current:.1f}"),
            KeyValue(key='current_reg_mA', value=f"{br.get('ir', 0.0):.1f}"),
            KeyValue(key='shunt_mV', value=f"{br.get('vs', 0.0):.3f}"),
            KeyValue(key='limit_mA', value=f"{limit:.0f}"),
            KeyValue(key='percent_of_limit', value=f"{frac * 100:.1f}"),
            KeyValue(key='power_W', value=f"{br.get('vb', 0.0) * current / 1000.0:.2f}"),
            KeyValue(key='temperature_C', value=f"{br.get('tc', 0.0):.1f}"),
            KeyValue(key='trip_count', value=str(br.get('tr', 0))),
            KeyValue(key='designator', value=designator),
            KeyValue(key='status_bits', value=f'0x{s:02X}'),
            KeyValue(key='online', value=str(bool(s & S_ONLINE))),
            KeyValue(key='powered', value=str(bool(s & S_POWERED))),
            KeyValue(key='manual_off', value=str(bool(s & S_MANUAL_OFF))),
            KeyValue(key='tripped', value=str(bool(s & S_TRIPPED))),
        ]
        return st

    def _board_status(self, msg, ocp_enabled, master_on):
        st = DiagnosticStatus()
        st.name = 'powerboard: board'
        st.hardware_id = self.frame_id
        st.level = DiagnosticStatus.OK
        st.message = 'ok'
        if not master_on:
            st.level = DiagnosticStatus.WARN
            st.message = 'U2 master branch is off, all rails unpowered'
        st.values = [
            KeyValue(key='ocp_enabled', value=str(ocp_enabled)),
            KeyValue(key='master_on', value=str(master_on)),
            KeyValue(key='seq', value=str(msg.get('seq'))),
            KeyValue(key='uptime_ms', value=str(msg.get('up'))),
            KeyValue(key='schema', value=str(msg.get('v'))),
            KeyValue(key='seq_gaps', value=str(self._seq_gaps)),
            KeyValue(key='parse_errors', value=str(self._parse_errors)),
            KeyValue(key='port', value=str(self.reader.path)),
        ]
        return st

    @staticmethod
    def _rail_array(labels, values, unit):
        """Pack values into a Float32MultiArray, one labelled dimension per rail.

        The label is "<name>(<designator>)" plus the unit on the first entry, so a plot
        or a `ros2 topic echo` shows what each number is without a lookup table.
        """
        msg = Float32MultiArray()
        for i, label in enumerate(labels):
            dim = MultiArrayDimension()
            dim.label = f'{label} [{unit}]' if i == 0 else label
            dim.size = 1
            dim.stride = 1
            msg.layout.dim.append(dim)
        msg.layout.data_offset = 0
        msg.data = [float(v) for v in values]
        return msg

    @staticmethod
    def _soc_from_cell_voltage(v_cell):
        """Interpolate the LiPo curve. Clamped at both ends rather than extrapolated."""
        if v_cell <= LIPO_OCV_SOC[0][0]:
            return 0.0
        if v_cell >= LIPO_OCV_SOC[-1][0]:
            return 1.0
        for (v0, s0), (v1, s1) in zip(LIPO_OCV_SOC, LIPO_OCV_SOC[1:]):
            if v_cell <= v1:
                return s0 + (s1 - s0) * (v_cell - v0) / (v1 - v0)
        return 1.0

    def _update_charging(self, t):
        """Infer charging from open-circuit voltage rising over a window.

        A pack that is only being discharged cannot gain open-circuit voltage, so a
        sustained rise means something is pushing charge in. Load-compensated OCV is used
        rather than raw terminal voltage so that a load simply going away - motors
        stopping, say - does not look like a charger. Hysteresis on the way out keeps the
        state from flickering between samples.
        """
        if not self.charge_detect or self.charge_window <= 0.0:
            self._charging = False
            return

        self._ocv_history.append((t, self._ocv_cell))
        while len(self._ocv_history) > 1 and t - self._ocv_history[0][0] > self.charge_window:
            self._ocv_history.popleft()

        span = t - self._ocv_history[0][0]
        # Wait for most of a window before judging, or startup noise decides it.
        if span < self.charge_window * 0.8:
            return

        self._ocv_rise = self._ocv_cell - self._ocv_history[0][1]
        scaled = self.charge_rise * (span / self.charge_window)
        if self._ocv_rise > scaled:
            if not self._charging:
                self.get_logger().info(
                    f"battery appears to be charging (+{self._ocv_rise * 1000.0:.1f} mV/cell "
                    f"over {span:.0f} s); charge current is not measurable on this board, "
                    f"so the charge estimate reads high until it settles")
            self._charging = True
        elif self._ocv_rise < scaled / 3.0:
            if self._charging:
                self.get_logger().info("battery no longer appears to be charging")
            self._charging = False

    def _update_soc(self, voltage, current_A, now):
        """Estimate state of charge from terminal voltage, load-compensated and smoothed.

        Returns the filtered 0..1 fraction, or None if estimation is disabled or the
        reading is not usable.
        """
        if self.cell_count <= 0 or voltage <= 0.0:
            self._soc = self._soc_raw = self._v_cell = self._ocv_cell = None
            return None

        # Undo the sag the load causes, so the estimate reflects the pack rather than
        # what the motors are doing at this instant. Discharge only - the board cannot
        # tell us about charging, and it reports current as a magnitude.
        self._current_A = max(0.0, current_A)
        ocv = voltage + max(0.0, current_A) * self.pack_resistance
        self._v_cell = voltage / self.cell_count
        self._ocv_cell = ocv / self.cell_count
        self._soc_raw = self._soc_from_cell_voltage(self._ocv_cell)

        t = now.nanoseconds * 1e-9
        self._update_charging(t)

        if self._soc is None or self._soc_time is None or self.soc_tau <= 0.0:
            self._soc = self._soc_raw
        else:
            dt = max(0.0, t - self._soc_time)
            alpha = dt / (self.soc_tau + dt) if (self.soc_tau + dt) > 0 else 1.0
            self._soc += alpha * (self._soc_raw - self._soc)
        self._soc_time = t
        return self._soc

    def _publish_battery(self, br, now):
        voltage = float(br.get('vb', 0.0))
        current_A = float(br.get('i', 0.0)) / 1000.0
        soc = self._update_soc(voltage, current_A, now)

        bs = BatteryState()
        bs.header.stamp = now.to_msg()
        bs.header.frame_id = self.frame_id
        bs.voltage = voltage
        # ROS convention is negative for discharge; the board reports magnitude only.
        bs.current = -current_A
        bs.temperature = float(br.get('tc', 0.0))
        # charge is the SoC estimate restated in Ah, so it is exactly as good as the
        # percentage and no better - it is NOT a coulomb count, and it does not become
        # one by being expressed in amp-hours. capacity (what the pack can still hold
        # once aged) stays NaN because nothing here measures it.
        bs.charge = (float('nan') if (soc is None or self.design_capacity <= 0.0)
                     else soc * self.design_capacity)
        bs.capacity = float('nan')
        bs.design_capacity = (float('nan') if self.design_capacity <= 0.0
                              else self.design_capacity)
        bs.percentage = float('nan') if soc is None else soc
        # current stays positive-out: the shunt genuinely measures the rover's own draw
        # even while the charger is feeding the pack around it, so the sign is not a
        # contradiction - the two currents are simply on opposite sides of the shunt.
        bs.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING
                                  if self._charging
                                  else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        bs.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        bs.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        bs.present = bool(int(br.get('s', 0)) & S_ONLINE)
        self.battery_pub.publish(bs)

    # -- staleness -----------------------------------------------------------------

    def _check_stale(self):
        """Publish an explicit failure when the stream stops.

        Without this the last good DiagnosticArray would simply sit there and a dead
        board would look identical to a healthy idle one.
        """
        if self._last_frame_time is not None:
            age = (self.get_clock().now() - self._last_frame_time).nanoseconds / 1e9
            if age <= self.stale_timeout:
                return
        elif self.reader.connected:
            return          # connected but nothing received yet, give it a moment

        if not self.reader.connected or self._last_frame_time is not None:
            self._reported_stale = True
            diag = DiagnosticArray()
            diag.header.stamp = self.get_clock().now().to_msg()
            diag.header.frame_id = self.frame_id
            st = DiagnosticStatus()
            st.name = 'powerboard: board'
            st.hardware_id = self.frame_id
            st.level = DiagnosticStatus.ERROR
            st.message = ('no serial connection to the powerboard'
                          if not self.reader.connected
                          else f'no telemetry for more than {self.stale_timeout:.0f} s')
            st.values = [KeyValue(key='port_pattern', value=self.port_pattern)]
            diag.status.append(st)
            self.diag_pub.publish(diag)

    # -- commands ------------------------------------------------------------------

    def _on_command(self, msg):
        cmd = msg.data.strip()
        if not cmd:
            return
        if not self.reader.connected:
            self.get_logger().warn(f"cannot send '{cmd}': board not connected")
            return
        if self.reader.write_line(cmd):
            self.get_logger().info(f"sent '{cmd}' to the board")
        else:
            self._drop_connection('write failed')


def main(args=None):
    rclpy.init(args=args)
    node = PowerboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.reader.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
