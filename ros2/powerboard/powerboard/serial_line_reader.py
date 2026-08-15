# Line-oriented serial reader built on pyserial.
#
# The board is a Raspberry Pi Pico presenting a USB CDC-ACM port, so the baud rate is
# ignored by the hardware; it is set anyway because pyserial requires one and it keeps
# the port configuration explicit.

import glob
import os

import serial


class SerialLineReader:
    """Opens a serial port, hands back complete lines, and survives the device coming
    and going. Non-blocking: call poll() from a ROS timer."""

    # A frame is ~550 bytes. Anything far beyond that means we are reading noise, or a
    # device that never sends a newline, so the buffer is capped rather than left to
    # grow without bound.
    MAX_BUFFER = 16384

    BAUDRATE = 115200

    def __init__(self, port_pattern):
        self.port_pattern = port_pattern
        self.ser = None
        self.path = None
        self._buf = bytearray()

    # -- connection ---------------------------------------------------------------

    def resolve_port(self):
        """Expand the configured pattern to a real device path.

        A glob is the default because /dev/ttyACM* numbering shifts whenever another
        USB serial device appears, whereas the by-id symlink is stable per board.
        """
        if any(ch in self.port_pattern for ch in '*?['):
            matches = sorted(glob.glob(self.port_pattern))
            return matches[0] if matches else None
        return self.port_pattern if os.path.exists(self.port_pattern) else None

    def connect(self):
        """Try to open the port. Returns the path on success, None if unavailable."""
        path = self.resolve_port()
        if path is None:
            return None
        try:
            # timeout=0 makes read() non-blocking and return whatever is buffered.
            # write_timeout bounds a stuck write so a wedged port cannot hang the node.
            self.ser = serial.Serial(
                port=path,
                baudrate=self.BAUDRATE,
                timeout=0,
                write_timeout=1.0,
                # No flow control: the board drives DTR/RTS as a CDC-ACM device and
                # asserting them here can reset some boards on open.
                xonxoff=False, rtscts=False, dsrdtr=False,
            )
            self.ser.reset_input_buffer()
        except (serial.SerialException, OSError, ValueError):
            self.ser = None
            return None

        self.path = path
        self._buf.clear()
        return path

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.path = None
        self._buf.clear()

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    # -- io -----------------------------------------------------------------------

    def poll(self):
        """Read whatever is available and return a list of complete lines (str).

        Raises OSError if the device went away, so the caller can drop the connection
        and go back to searching for it.
        """
        if not self.connected:
            return []

        try:
            waiting = self.ser.in_waiting
            if waiting:
                chunk = self.ser.read(min(waiting, 8192))
                if chunk:
                    self._buf.extend(chunk)
        except (serial.SerialException, OSError) as exc:
            # Unplugged mid-read. Surface as OSError so the node handles it uniformly.
            raise OSError(str(exc)) from exc

        if len(self._buf) > self.MAX_BUFFER:
            # Keep the tail: a partial line at the end is the part worth salvaging.
            del self._buf[:-self.MAX_BUFFER // 2]

        lines = []
        while True:
            idx = self._buf.find(b'\n')
            if idx < 0:
                break
            raw = bytes(self._buf[:idx])
            del self._buf[:idx + 1]
            lines.append(raw.decode('utf-8', errors='replace').strip())
        return lines

    def write_line(self, text):
        """Send a command. Returns True if it went out."""
        if not self.connected:
            return False
        try:
            self.ser.write((text.rstrip('\r\n') + '\n').encode('utf-8'))
            return True
        except (serial.SerialException, serial.SerialTimeoutException, OSError):
            return False
