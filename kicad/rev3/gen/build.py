#!/usr/bin/env python3
"""Generate the rev3 power board schematic for KiCad 10.

    python3 kicad/rev3/gen/build.py           # write project, run ERC, verify netlist

This is a bootstrap. It turns the design in docs/rev3 (rails.py values, design.md
architecture) into a KiCad 10 project, so nothing is drawn by hand from scratch. Once
the team starts editing the schematic in KiCad, the .kicad_sch files are the source of
truth; re-running this script overwrites them.

Every pin gets a short wire stub and a net label, rather than drawn wires between
parts. That keeps the generated sheets correct by construction. The final step exports
KiCad's own netlist and checks it against the netlist this script intended, so a
geometry mistake shows up as an error instead of a silent miswire.
"""

import importlib.util
import json
import math
import re
import subprocess
import sys
import uuid
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent
REPO = PROJ.parent.parent
NAME = "PowerBoard_rev3"
sys.path.insert(0, str(HERE))
from kisym import Library, dump, find, parse, pins as lib_pins, unq  # noqa: E402

spec = importlib.util.spec_from_file_location("rails", REPO / "docs/rev3/rails.py")
rails = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rails)

KICAD_CLI = ["flatpak", "run", "--command=kicad-cli", "org.kicad.KiCad"]
SCH_VERSION = "20260306"   # KiCad 10
SYM_VERSION = "20251024"
NS = uuid.UUID("5b3f2a8e-6c1d-4d7a-9a4e-7f0c2b9d1e10")


def kicad_dir(ext):
    loc = subprocess.run(["flatpak", "info", "--show-location", f"org.kicad.KiCad.Library.{ext}"],
                         capture_output=True, text=True).stdout.strip()
    return Path(loc) / "files" / ext.lower()


SYM_DIR = kicad_dir("Symbols")
FP_DIR = kicad_dir("Footprints")


def uid(*parts):
    return str(uuid.uuid5(NS, "/".join(map(str, parts))))


def q(s):
    return '"' + str(s).replace('\\', '\\\\').replace('"', '\\"') + '"'


def fmt(v):
    return f"{round(v, 4):g}"


def snap(v, g=1.27):
    return round(v / g) * g


# ------------------------------------------------------------------ custom symbols

FONT = "(effects (font (size 1.27 1.27)))"


def box_symbol(name, left, right, ref="U", footprint="", datasheet="", desc="", top=(), bottom=()):
    """A rectangular symbol. left/right/top/bottom: lists of (number, name, type)."""
    longest = max([len(p[1]) for p in left + right] + [4])
    width = max(15.24, math.ceil((longest * 2 * 1.0 + 5) / 2.54) * 2.54, (max(len(top), len(bottom)) + 1) * 5.08)
    rows = max(len(left), len(right), 2)
    height = (rows + 1) * 2.54
    hw, hh = width / 2, height / 2
    hw = math.ceil(hw / 2.54) * 2.54
    hh = math.ceil(hh / 2.54) * 2.54
    out = []

    def pin(p, x, y, a):
        num, nm, typ = p
        out.append(f'(pin {typ} line (at {fmt(x)} {fmt(y)} {a}) (length 2.54) '
                   f'(name {q(nm)} {FONT}) (number {q(num)} {FONT}))')

    y0 = hh - 2.54
    for i, p in enumerate(left):
        pin(p, -hw - 2.54, y0 - i * 2.54, 0)
    for i, p in enumerate(right):
        pin(p, hw + 2.54, y0 - i * 2.54, 180)
    for i, p in enumerate(top):
        pin(p, snap(-((len(top) - 1) * 5.08) / 2 + i * 5.08, 2.54), hh + 2.54, 270)
    for i, p in enumerate(bottom):
        pin(p, snap(-((len(bottom) - 1) * 5.08) / 2 + i * 5.08, 2.54), -hh - 2.54, 90)
    props = [("Reference", ref, hw + 1.27, hh + 1.27, False), ("Value", name, hw + 1.27, -hh - 1.27, False),
             ("Footprint", footprint, 0, 0, True), ("Datasheet", datasheet, 0, 0, True),
             ("Description", desc, 0, 0, True)]
    s = f'(symbol {q(name)} (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)\n'
    for k, v, x, y, hide in props:
        s += (f'  (property {q(k)} {q(v)} (at {fmt(x)} {fmt(y)} 0) (effects (font (size 1.27 1.27))'
              f'{" (justify left)" if not hide else ""}{" (hide yes)" if hide else ""}))\n')
    s += (f'  (symbol {q(name + "_0_1")} (rectangle (start {fmt(-hw)} {fmt(hh)}) (end {fmt(hw)} {fmt(-hh)})'
          f' (stroke (width 0.254) (type default)) (fill (type background))))\n')
    s += f'  (symbol {q(name + "_1_1")}\n    ' + '\n    '.join(out) + ')\n  (embedded_fonts no))'
    return s


def pinned_symbol(name, hw, hh, pins, ref="U", footprint="", datasheet="", desc=""):
    """A box symbol with explicit pin positions: pins = (number, name, type, x, y, angle)."""
    out = [f'(pin {typ} line (at {fmt(x)} {fmt(y)} {a}) (length 2.54) (name {q(nm)} {FONT}) (number {q(num)} {FONT}))'
           for num, nm, typ, x, y, a in pins]
    props = [("Reference", ref, hw + 1.27, hh + 1.27, False), ("Value", name, hw + 1.27, -hh - 1.27, False),
             ("Footprint", footprint, 0, 0, True), ("Datasheet", datasheet, 0, 0, True),
             ("Description", desc, 0, 0, True)]
    s = f'(symbol {q(name)} (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)\n'
    for k, v, x, y, hide in props:
        s += (f'  (property {q(k)} {q(v)} (at {fmt(x)} {fmt(y)} 0) (effects (font (size 1.27 1.27))'
              f'{" (justify left)" if not hide else ""}{" (hide yes)" if hide else ""}))\n')
    s += (f'  (symbol {q(name + "_0_1")} (rectangle (start {fmt(-hw)} {fmt(hh)}) (end {fmt(hw)} {fmt(-hh)})'
          f' (stroke (width 0.254) (type default)) (fill (type background))))\n')
    s += f'  (symbol {q(name + "_1_1")}\n    ' + '\n    '.join(out) + ')\n  (embedded_fonts no))'
    return s


def mosfet_symbol():
    """KiCad's Q_NMOS drawing with pins renumbered 1 = G, 2 = D, 3 = S, to match D2PAK / DPAK pads."""
    sym = Library.load(SYM_DIR / "Device.kicad_sym").flat("Q_NMOS")
    sym[1] = q("Q_NMOS_GDS")
    for sub in find(sym, "symbol"):
        sub[1] = q(unq(sub[1]).replace("Q_NMOS", "Q_NMOS_GDS", 1))
        for pn in find(sub, "pin"):
            num = find(pn, "number")[0]
            num[1] = q({"D": "2", "G": "1", "S": "3"}[unq(num[1])])
    for prop in find(sym, "property"):
        key = unq(prop[1])
        if key in ("Value", "Description", "Footprint"):
            prop[2] = q({"Value": "Q_NMOS_GDS", "Footprint": "",
                         "Description": "N-channel MOSFET, pin 1 gate, 2 drain (tab), 3 source"}[key])
    return dump(sym, 0)


def conn_minus1_symbol():
    """Conn_01x02 drawn the same way, but the top pin is pad 2 (+) and the bottom pin pad 1 (-).
    AMASS XT30 / XT60 footprints have pad 1 on the chamfered, negative side, and rev2 wired its
    Deans T with pad 1 = GND, so these connectors take + on pad 2."""
    sym = Library.load(SYM_DIR / "Connector_Generic.kicad_sym").flat("Conn_01x02")
    sym[1] = q("Conn_01x02_MinusPin1")
    for sub in find(sym, "symbol"):
        sub[1] = q(unq(sub[1]).replace("Conn_01x02", "Conn_01x02_MinusPin1", 1))
        for pn in find(sub, "pin"):
            num, name = find(pn, "number")[0], find(pn, "name")[0]
            new = {"1": "2", "2": "1"}[unq(num[1])]
            num[1] = q(new)
            name[1] = q({"2": "+", "1": "-"}[new])
    for prop in find(sym, "property"):
        if unq(prop[1]) == "Value":
            prop[2] = q("Conn_01x02_MinusPin1")
        if unq(prop[1]) == "Description":
            prop[2] = q("2-pin power connector: pad 1 = -, pad 2 = + (XT30 / XT60 / Deans T)")
    return dump(sym, 0)


TI = "https://www.ti.com/lit/ds/symlink/"
CUSTOM = {
    # Pins placed for the drawn eFuse sheet: supply and sense on top (to the rails),
    # gate drive on the right (to the FETs), control on the left, settings on the bottom.
    "TPS48111-Q1": pinned_symbol(
        "TPS48111-Q1", 25.4, 22.86, [
            ("20", "VS", "power_in", -20.32, 25.4, 270), ("19", "ISCP", "input", -10.16, 25.4, 270),
            ("18", "CS+", "input", 0, 25.4, 270), ("17", "CS-", "input", 10.16, 25.4, 270),
            ("1", "EN/UVLO", "input", -27.94, 17.78, 0), ("3", "INP", "input", -27.94, 10.16, 0),
            ("2", "INP_G", "input", -27.94, -7.62, 0), ("5", "FLT_I", "open_collector", -27.94, -17.78, 0),
            ("4", "FLT_T", "open_collector", -27.94, -20.32, 0),
            ("15", "PU", "output", 27.94, 17.78, 180), ("14", "PD", "output", 27.94, 12.7, 180),
            ("12", "BST", "passive", 27.94, 5.08, 180), ("13", "SRC", "passive", 27.94, -2.54, 180),
            ("11", "G", "output", 27.94, -12.7, 180),
            ("6", "GND", "power_in", -17.78, -25.4, 90), ("10", "DIODE", "passive", -12.7, -25.4, 90),
            ("7", "IMON", "passive", 0, -25.4, 90), ("8", "IWRN", "passive", 12.7, -25.4, 90),
            ("9", "TMR", "passive", 22.86, -25.4, 90)],
        footprint="PowerBoard_rev3:TI_DGX0019A_VSSOP-19",
        datasheet=TI + "tps4811-q1.pdf",
        desc="100 V smart high-side driver with pre-charge, current monitor and protection. "
             "DGX-19: pin 16 is absent for creepage."),
    # Laid out for the drawn boost sheet: VIN / current sense / LO up to the power stage, the
    # high-side drive on the right, the setting pins on the left, FB and grounds below.
    "LM5122": pinned_symbol(
        "LM5122", 17.78, 20.32, [
            ("5", "VIN", "power_in", -15.24, 22.86, 270), ("4", "CSP", "input", -7.62, 22.86, 270),
            ("3", "CSN", "input", 0, 22.86, 270), ("16", "LO", "output", 10.16, 22.86, 270),
            ("6", "UVLO", "input", -20.32, 12.7, 0), ("7", "SS", "passive", -20.32, 7.62, 0),
            ("8", "SYNCIN/RT", "input", -20.32, 2.54, 0), ("12", "SLOPE", "passive", -20.32, -2.54, 0),
            ("14", "RES", "passive", -20.32, -7.62, 0), ("11", "COMP", "output", -20.32, -12.7, 0),
            ("18", "SW", "passive", 20.32, 15.24, 180), ("20", "BST", "passive", 20.32, 7.62, 180),
            ("17", "VCC", "power_out", 20.32, 0, 180), ("1", "SYNCOUT", "output", 20.32, -12.7, 180),
            ("19", "HO", "output", 20.32, -17.78, 180),
            ("10", "FB", "input", -12.7, -22.86, 90), ("9", "AGND", "power_in", -7.62, -22.86, 90),
            ("15", "PGND", "power_in", -2.54, -22.86, 90), ("21", "EP", "passive", 2.54, -22.86, 90),
            ("13", "MODE", "input", 7.62, -22.86, 90), ("2", "OPT", "input", 12.7, -22.86, 90)],
        footprint="Package_SO:HTSSOP-20-1EP_4.4x6.5mm_P0.65mm_EP3.4x6.5mm_Mask2.96x2.96mm",
        datasheet=TI + "lm5122.pdf", desc="Synchronous boost controller, 65 V in, HTSSOP-20 (MH)"),
    # Input on the left with RON second (its resistor comes from VIN, above the UVLO divider);
    # BST 7.62 above SW so the bootstrap cap stands between them.
    "LM5017": pinned_symbol(
        "LM5017", 12.7, 12.7, [
            ("2", "VIN", "power_in", -15.24, 10.16, 0), ("4", "RON", "input", -15.24, 2.54, 0),
            ("3", "UVLO", "input", -15.24, -2.54, 0), ("5", "FB", "input", -15.24, -10.16, 0),
            ("7", "BST", "passive", 15.24, 10.16, 180), ("8", "SW", "power_out", 15.24, 2.54, 180),
            ("6", "VCC", "power_out", 15.24, -10.16, 180),
            ("1", "RTN", "power_in", -2.54, -15.24, 90), ("9", "EP", "passive", 2.54, -15.24, 90)],
        footprint="Package_SO:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.3mm",
        datasheet=TI + "lm5017.pdf", desc="100 V 600 mA synchronous constant-on-time buck, SO-8 PowerPAD (DDA)"),
    # Sense inputs on top, 7.62 apart, so the Kelvin lines drop straight down from a 2-pin shunt.
    "INA228": pinned_symbol(
        "INA228", 10.16, 10.16, [
            ("10", "IN+", "input", -2.54, 12.7, 270), ("9", "IN-", "input", 2.54, 12.7, 270),
            ("8", "VBUS", "input", 7.62, 12.7, 270), ("6", "VS", "power_in", -12.7, 2.54, 0),
            ("3", "~{ALERT}", "open_collector", 12.7, 2.54, 180), ("4", "SDA", "bidirectional", 12.7, 0, 180),
            ("5", "SCL", "input", 12.7, -2.54, 180),
            ("7", "GND", "power_in", -5.08, -12.7, 90), ("2", "A0", "input", 0, -12.7, 90),
            ("1", "A1", "input", 5.08, -12.7, 90)],
        footprint="Package_SO:TSSOP-10_3x3mm_P0.5mm",
        datasheet=TI + "ina228.pdf", desc="85 V 20-bit power / energy / charge monitor, I2C, VSSOP-10 (DGS)"),
    "TCAN1044AV": pinned_symbol(
        "TCAN1044AV", 10.16, 10.16, [
            ("1", "TXD", "input", -12.7, 2.54, 0), ("4", "RXD", "output", -12.7, 0, 0),
            ("8", "STB", "input", -12.7, -2.54, 0),
            ("7", "CANH", "bidirectional", 12.7, 2.54, 180), ("6", "CANL", "bidirectional", 12.7, 0, 180),
            ("3", "VCC", "power_in", -2.54, 12.7, 270), ("5", "VIO", "power_in", 2.54, 12.7, 270),
            ("2", "GND", "power_in", 0, -12.7, 90)],
        footprint="Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
        datasheet=TI + "tcan1044a-q1.pdf", desc="CAN FD transceiver, +-58 V bus fault, VIO 1.8-5.5 V"),
    "TLV1805": pinned_symbol(
        "TLV1805", 7.62, 7.62, [
            ("3", "IN-", "input", -10.16, 2.54, 0), ("4", "IN+", "input", -10.16, -5.08, 0),
            ("1", "OUT", "output", 10.16, 0, 180), ("6", "V+", "power_in", 0, 10.16, 270),
            ("2", "V-", "power_in", 0, -10.16, 90), ("5", "SHDN", "input", 5.08, -10.16, 90)],
        footprint="Package_TO_SOT_SMD:SOT-23-6",
        datasheet="https://www.ti.com/lit/ds/symlink/tlv1805.pdf",
        desc="40 V push-pull comparator, 100 mA output (drives a MOSFET gate), SOT-23-6 (DBV)"),
    "Q_NMOS_GDS": mosfet_symbol(),
    "Conn_01x02_MinusPin1": conn_minus1_symbol(),
}


def write_custom_lib():
    body = "\n".join(CUSTOM.values())
    (PROJ / "PowerBoard_rev3.kicad_sym").write_text(
        f'(kicad_symbol_lib (version {SYM_VERSION}) (generator "build.py") (generator_version "10.0")\n'
        f'{body}\n)\n')


_custom_tree = None


def get_symbol(lib_id):
    """Flattened symbol tree, renamed 'Lib:Name' for embedding."""
    global _custom_tree
    lib, name = lib_id.split(":")
    if lib == "PowerBoard_rev3":
        if _custom_tree is None:
            _custom_tree = Library(PROJ / "PowerBoard_rev3.kicad_sym")
        sym = _custom_tree.flat(name)
    else:
        sym = Library.load(SYM_DIR / f"{lib}.kicad_sym").flat(name)
    sym = list(sym)
    sym[1] = q(lib_id)
    return sym


# ------------------------------------------------------------------ parts

PREFIX_COUNT = defaultdict(int)
INTENDED = []          # (ref, pin, net_key)
FOOTPRINTS = set()


class Part:
    def __init__(self, lib_id, prefix, value, footprint, conns, rot=0, fields=None, unit=1, at=None, mirror=False):
        self.lib_id, self.prefix, self.value, self.footprint = lib_id, prefix, value, footprint
        self.at, self.mirror = at, mirror
        self.label_at = {}             # field name -> (dx, dy): text placed there, reading left to right
        self.rot, self.unit, self.fields = rot, unit, fields or {}
        self.sym = get_symbol(lib_id)
        self.pins = lib_pins(self.sym, unit)
        self.conns = {str(k): v for k, v in conns.items()}
        if footprint:
            FOOTPRINTS.add(footprint)
        visible = {p["number"] for p in self.pins if not p["hidden"]}
        missing = visible - set(self.conns)
        extra = set(self.conns) - {p["number"] for p in self.pins}
        if missing or extra:
            raise ValueError(f"{lib_id} {value}: unconnected pins {sorted(missing)}, unknown {sorted(extra)}")
        # hidden pins stacked on a visible pin share its net
        at = {(p["x"], p["y"]): p["number"] for p in self.pins if not p["hidden"]}
        for p in self.pins:
            if p["hidden"] and p["number"] not in self.conns:
                self.conns[p["number"]] = self.conns[at[(p["x"], p["y"])]]

    def geom(self):
        """Pins in sheet coordinates relative to the symbol origin, with outward direction."""
        th = math.radians(self.rot)
        out = []
        for p in self.pins:
            x = p["x"] * math.cos(th) - p["y"] * math.sin(th)
            y = p["x"] * math.sin(th) + p["y"] * math.cos(th)
            a = (p["angle"] + self.rot) % 360
            o = (a + 180) % 360            # outward, lib (y-up) degrees
            if self.mirror in (True, "y"):   # KiCad (mirror y): x -> -x after rotation
                x, o = -x, (180 - o) % 360
            elif self.mirror == "x":         # KiCad (mirror x): y -> -y after rotation
                y, o = -y, (360 - o) % 360
            out.append(dict(p, sx=round(x, 3), sy=round(-y, 3), out=o))
        return out


# ------------------------------------------------------------------ sheets

POWER = {"GND": "power:GND", "+3V3": "power:+3V3"}
PAPERS = [("A4", 297, 210), ("A3", 420, 297), ("A2", 594, 420), ("A1", 841, 594), ("A0", 1189, 841)]


def rnd(pt):
    return (round(pt[0], 2), round(pt[1], 2))


def PIN(p, num):
    """Absolute sheet position of a placed part's pin."""
    num = getattr(p, "alias", {}).get(num, num)
    a = next(g for g in p.geom() if g["number"] == str(num))
    return rnd((p.at[0] + a["sx"], p.at[1] + a["sy"]))


def at_pin(p, num, pt):
    """Place part p so that its pin num lands on pt."""
    p.at = (0.0, 0.0)
    a = PIN(p, num)
    p.at = rnd((pt[0] - a[0], pt[1] - a[1]))
    return p


class Sheet:
    def __init__(self, name, title, notes=(), drawn=False, paper="A3"):
        self.name, self.title, self.notes = name, title, list(notes)
        self.drawn_paper = paper
        self.file = f"{name}.kicad_sch"
        self.parts, self.flags = [], []
        self.drawn, self.wires, self.marks = drawn, [], []

    def wire(self, *pts):
        """Polyline through the given points. Ends landing on other wires become junctions."""
        pts = [rnd(p) for p in pts]
        self.wires += list(zip(pts, pts[1:]))

    def mark(self, kind, pt, net=None, ang=0):
        """kind: gnd, 3v3, label (net + angle), flag (PWR_FLAG)."""
        self.marks.append((kind, rnd(pt), net, ang))

    def add(self, *args, **kw):
        p = Part(*args, **kw)
        self.parts.append(p)
        return p

    def flag(self, net):
        self.flags.append(net)

    def shift(self, dx, dy):
        """Move everything drawn so far."""
        mv = lambda pt: rnd((pt[0] + dx, pt[1] + dy))
        for p in self.parts:
            if p.at is not None:
                p.at = mv(p.at)
        self.wires = [(mv(a), mv(b)) for a, b in self.wires]
        self.marks = [(k, mv(pt), n, a) for k, pt, n, a in self.marks]


def label_width(net):
    return len(net.split(":")[-1]) * 1.0 + 4.0


def is_local(net):
    return net.startswith("L:")


def net_key(sheet, net):
    return f"{sheet.name}/{net[2:]}" if is_local(net) else net


def layout(sheet):
    if sheet.drawn:
        sheet.paper, sheet.pw, sheet.ph = next(pp for pp in PAPERS if pp[0] == sheet.drawn_paper)
        sheet.placed = [(p, p.at[0], p.at[1]) for p in sheet.parts]
        return
    cells = []
    for p in sheet.parts:
        g = p.geom()
        xs = [a["sx"] for a in g] or [0]
        ys = [a["sy"] for a in g] or [0]
        ext = {"left": 0, "right": 0, "up": 0, "down": 0}
        for a in g:
            net = p.conns[a["number"]]
            w = 2.54 + (0 if net is None else (6 if net in POWER else label_width(net)))
            d = {0: "right", 180: "left", 90: "up", 270: "down"}[int(a["out"])]
            ext[d] = max(ext[d], w)
        left = min(xs) - ext["left"] - 5.08
        right = max(xs) + ext["right"] + 5.08
        top = min(ys) - ext["up"] - 7.62
        bottom = max(ys) + ext["down"] + 5.08
        cells.append((p, left, right, top, bottom))
    nflag = len(sheet.flags)
    for paper, pw, ph in PAPERS:
        x0, y0 = 20.0, 30.0 + 6 * len(sheet.notes)
        x, y, row_h, ok = x0, y0, 0, True
        placed = []
        for p, l, r, t, b in cells:
            w, h = r - l, b - t
            if x + w > pw - 20:
                x, y, row_h = x0, y + row_h, 0
            placed.append((p, snap(x - l), snap(y - t), x, y, w))
            x += w
            row_h = max(row_h, h)
        if y + row_h + 20 * (1 + nflag // 10) <= ph - 45:
            sheet.paper, sheet.pw, sheet.ph = paper, pw, ph
            sheet.placed = placed
            sheet.flag_y = y + row_h + 10
            return
    raise RuntimeError(f"sheet {sheet.name} does not fit on A0")


class Emitter:
    def __init__(self, sheet, path):
        self.s, self.path, self.items = sheet, path, []
        self.lib_ids = set()
        self.pwr = 0

    def text(self, s):
        self.items.append(s)

    def wire(self, x1, y1, x2, y2):
        self.text(f'(wire (pts (xy {fmt(x1)} {fmt(y1)}) (xy {fmt(x2)} {fmt(y2)})) '
                  f'(stroke (width 0) (type default)) (uuid {q(uid(self.s.name, "w", x1, y1, x2, y2))}))')

    def label(self, net, x, y, ang):
        just = "left" if ang in (0, 90) else "right"
        key = uid(self.s.name, "lbl", net, x, y)
        if is_local(net):
            self.text(f'(label {q(net[2:])} (at {fmt(x)} {fmt(y)} {ang}) (effects (font (size 1.0 1.0)) '
                      f'(justify {just} bottom)) (uuid {q(key)}))')
        else:
            self.text(f'(global_label {q(net)} (shape passive) (at {fmt(x)} {fmt(y)} {ang}) (fields_autoplaced yes) '
                      f'(effects (font (size 1.0 1.0)) (justify {just})) (uuid {q(key)}) '
                      f'(property "Intersheetrefs" "${{INTERSHEET_REFS}}" (at {fmt(x)} {fmt(y)} 0) '
                      f'(effects (font (size 1.27 1.27)) (justify {just}) (hide yes))))')

    def symbol(self, lib_id, ref, value, x, y, rot, pins, footprint="", fields=None, unit=1, hide_ref=False,
               two_pin=False, vertical=False, mirror=False, label_at=None):
        self.lib_ids.add(lib_id)
        key = uid(self.s.name, "sym", ref)
        props = [("Reference", ref, hide_ref), ("Value", value, False), ("Footprint", footprint, True),
                 ("Datasheet", "", True), ("Description", "", True)]
        props += [(k, v, True) for k, v in (fields or {}).items()]
        s = (f'(symbol (lib_id {q(lib_id)}) (at {fmt(x)} {fmt(y)} {rot}){"" if not mirror else " (mirror x)" if mirror == "x" else " (mirror y)"} (unit {unit}) '
             f'(exclude_from_sim no) (in_bom {"no" if ref.startswith("#") else "yes"}) '
             f'(on_board {"no" if ref.startswith("#") else "yes"}) (dnp no) (uuid {q(key)})\n')
        sym = get_symbol(lib_id)
        th = math.radians(rot)
        for i, (k, v, hide) in enumerate(props):
            lib_at = next((find(pp, "at")[0] for pp in find(sym, "property") if unq(pp[1]) == k), None)
            just = ""
            if label_at and k in label_at:
                # explicit spot: horizontal text starting at the spot
                px, py = x + label_at[k][0], y + label_at[k][1]
                ang = 90 if rot in (90, 270) else 0
                right = rot in (90, 180)
                if mirror in (True, "y"):
                    right = not right
                just = " (justify right)" if right else " (justify left)"
            elif two_pin and rot in (90, 270) and vertical and k in ("Reference", "Value"):
                # diode / LED turned upright: text to the right, reading horizontally
                px, py, ang = x + 2.54, y + (-1.27 if k == "Reference" else 1.27), 90
                just = " (justify right)" if rot == 90 else " (justify left)"
            elif two_pin and rot in (90, 270) and k in ("Reference", "Value"):
                px, py, ang = x, y + (-2.54 if k == "Reference" else 2.54), 90
            elif lib_at is not None and k in ("Reference", "Value"):
                lx, ly = float(lib_at[1]), float(lib_at[2])
                ang = float(lib_at[3]) if len(lib_at) > 3 else 0
                rx = lx * math.cos(th) - ly * math.sin(th)
                ry = lx * math.sin(th) + ly * math.cos(th)
                if mirror in (True, "y"):
                    rx = -rx
                elif mirror == "x":
                    ry = -ry
                px, py = x + rx, y - ry
                lib_eff = next((find(pp, "effects")[0] for pp in find(sym, "property") if unq(pp[1]) == k), [])
                j = find(lib_eff, "justify")
                jw = list(j[0][1:]) if j else []
                if mirror in (True, "y"):
                    jw = [{"left": "right", "right": "left"}.get(w, w) for w in jw]
                just = f" (justify {' '.join(jw)})" if jw else ""
            else:
                px, py, ang = x, y, 0
            s += (f'  (property {q(k)} {q(v)} (at {fmt(px)} {fmt(py)} {fmt(ang)}) (effects (font (size 1.27 1.27))'
                  f'{just}{" (hide yes)" if hide else ""}))\n')
        for pn in pins:
            s += f'  (pin {q(pn)} (uuid {q(uid(key, "pin", pn))}))\n'
        s += (f'  (instances (project {q(NAME)} (path {q(self.path)} (reference {q(ref)}) (unit {unit}))))\n)')
        self.text(s)

    def power(self, net, x, y, out):
        self.pwr += 1
        ref = f"#PWR{self.s.name}{self.pwr:03d}".replace("_", "")
        o = {0: (1, 0), 180: (-1, 0), 90: (0, 1), 270: (0, -1)}[int(out)]
        if net == "GND":       # body points down in the library
            rot = {(0, -1): 0, (0, 1): 180, (1, 0): 90, (-1, 0): 270}[o]
        else:                  # +3V3: body points up
            rot = {(0, 1): 0, (0, -1): 180, (1, 0): 270, (-1, 0): 90}[o]
        self.symbol(POWER[net], ref, net, x, y, rot, ["1"], hide_ref=True)

    def pwr_flag(self, net, x, y):
        self.pwr += 1
        ref = f"#FLG{self.s.name}{self.pwr:03d}".replace("_", "")
        self.symbol("power:PWR_FLAG", ref, "PWR_FLAG", x, y, 0, ["1"], hide_ref=True)
        if net in POWER:
            self.wire(x, y, x, y + 2.54)
            self.power(net, x, y + 2.54, 270)
        else:
            self.wire(x, y, x, y + 2.54)
            self.label(net, x, y + 2.54, 270)

    def part(self, p, X, Y):
        PREFIX_COUNT[p.prefix] += 1
        ref = f"{p.prefix}{PREFIX_COUNT[p.prefix]}"
        p.ref = ref
        fields = dict(p.fields)
        if not fields.get("MPN"):
            fields["MPN"] = auto_mpn(p)
        g = p.geom()
        self.symbol(p.lib_id, ref, p.value, X, Y, p.rot, [a["number"] for a in p.pins],
                    footprint=p.footprint, fields=fields, unit=p.unit, two_pin=len(p.pins) == 2,
                    vertical=len(g) == 2 and g[0]["sx"] == g[1]["sx"], mirror=p.mirror, label_at=p.label_at)
        done = set()
        for a in p.geom():
            net = p.conns[a["number"]]
            INTENDED.append((ref, a["number"], None if net is None else net_key(self.s, net)))
            px, py = X + a["sx"], Y + a["sy"]
            if (px, py) in done:
                continue          # hidden pin stacked on a visible one
            done.add((px, py))
            if rnd((px, py)) in self.anchors:
                continue          # drawn sheet: a wire or symbol already lands on this pin
            if net is None:
                self.text(f'(no_connect (at {fmt(px)} {fmt(py)}) (uuid {q(uid(self.s.name, "nc", ref, a["number"]))}))')
                continue
            dx, dy = {0: (1, 0), 180: (-1, 0), 90: (0, -1), 270: (0, 1)}[int(a["out"])]
            ex, ey = px + 2.54 * dx, py + 2.54 * dy
            self.wire(px, py, ex, ey)
            if net in POWER:
                self.power(net, ex, ey, a["out"])
            else:
                self.label(net, ex, ey, int(a["out"]))

    def drawn_wires(self):
        """Split wires wherever another wire end, pin or mark lands on them; return segments."""
        s = self.s
        pins = set()
        for p, X, Y, *_ in s.placed:
            for a in p.geom():
                pins.add(rnd((X + a["sx"], Y + a["sy"])))
        marks = {pt for _, pt, _, _ in s.marks}
        pts = pins | marks | {e for w in s.wires for e in w}

        def inside(pt, a, b):
            if a[0] == b[0] == pt[0]:
                return min(a[1], b[1]) < pt[1] < max(a[1], b[1])
            if a[1] == b[1] == pt[1]:
                return min(a[0], b[0]) < pt[0] < max(a[0], b[0])
            return False

        segs = []
        for a, b in s.wires:
            if a[0] != b[0] and a[1] != b[1]:
                raise ValueError(f"{s.name}: wire {a}-{b} is not horizontal or vertical")
            chain = sorted([a, b] + [pt for pt in pts if inside(pt, a, b)],
                           key=lambda pt: abs(pt[0] - a[0]) + abs(pt[1] - a[1]))
            segs += [(u, w) for u, w in zip(chain, chain[1:]) if u != w]
        deg = defaultdict(int)
        for u, w in segs:
            deg[u] += 1
            deg[w] += 1
        for pt in pins | marks:
            if pt in deg:
                deg[pt] += 1
        return segs, [pt for pt, d in deg.items() if d >= 3]

    def render(self, root_uuid, sheet_uuid, page_title):
        s = self.s
        layout(s)
        self.anchors = set()
        if s.drawn:
            segs, junctions = self.drawn_wires()
            self.anchors = {e for w in segs for e in w} | {pt for _, pt, _, _ in s.marks}
            for (x1, y1), (x2, y2) in segs:
                self.wire(x1, y1, x2, y2)
            for x, y in junctions:
                self.text(f'(junction (at {fmt(x)} {fmt(y)}) (diameter 0) (color 0 0 0 0) '
                          f'(uuid {q(uid(s.name, "j", x, y))}))')
            for kind, (x, y), net, ang in s.marks:
                if kind == "gnd":
                    self.power("GND", x, y, 270)
                elif kind == "3v3":
                    self.power("+3V3", x, y, 90)
                elif kind == "label":
                    self.label(net, x, y, ang)
                elif kind == "flag":
                    self.pwr += 1
                    self.symbol("power:PWR_FLAG", f"#FLG{s.name}{self.pwr:03d}".replace("_", ""), "PWR_FLAG",
                                x, y, 0, ["1"], hide_ref=True)
        for i, n in enumerate([s.title] + s.notes):
            size = 2.5 if i == 0 else 1.6
            self.text(f'(text {q(n)} (exclude_from_sim no) (at 20 {fmt(18 + i * 6)} 0) '
                      f'(effects (font (size {size} {size}){" (bold yes)" if i == 0 else ""}) (justify left bottom)) '
                      f'(uuid {q(uid(s.name, "note", i))}))')
        for p, X, Y, *_ in s.placed:
            self.part(p, X, Y)
        for i, net in enumerate(s.flags):
            self.pwr_flag(net, snap(25 + (i % 10) * 25), snap(s.flag_y + (i // 10) * 15))
        libs = []
        for lid in sorted(self.lib_ids):
            libs.append(dump(get_symbol(lid), 1))
        head = (f'(kicad_sch (version {SCH_VERSION}) (generator "eeschema") (generator_version "10.0")\n'
                f'(uuid {q(sheet_uuid)})\n(paper {q(s.paper)})\n'
                f'(title_block (title {q(page_title)}) (rev "3.0") (company "Brno Mars Rover") '
                f'(comment 1 "Generated by kicad/rev3/gen/build.py from docs/rev3"))\n'
                f'(lib_symbols\n' + "\n".join(libs) + '\n)\n')
        return head + "\n".join(self.items) + '\n(embedded_fonts no)\n)\n'


# ------------------------------------------------------------------ part helpers

FP = dict(
    R0603="Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder",
    R0805="Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
    R1206="Resistor_SMD:R_1206_3216Metric_Pad1.30x1.75mm_HandSolder",
    R2512="Resistor_SMD:R_2512_6332Metric_Pad1.40x3.35mm_HandSolder",
    C0603="Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder",
    C0805="Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder",
    C1206="Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder",
    C1210="Capacitor_SMD:C_1210_3225Metric_Pad1.33x2.70mm_HandSolder",
    CP10="Capacitor_SMD:CP_Elec_10x10.5",
    CPR12="Capacitor_THT:CP_Radial_D12.5mm_P5.00mm",
    CPR6="Capacitor_THT:CP_Radial_D6.3mm_P2.50mm",
    SHUNT2512K="PowerBoard_rev3:R_Bourns_CSS2H-2512_Kelvin",
    CP8="Capacitor_SMD:CP_Elec_8x10.5",
    D2PAK="Package_TO_SOT_SMD:TO-263-2",
    DPAK="Package_TO_SOT_SMD:TO-252-2",
    SOT23="Package_TO_SOT_SMD:SOT-23",
    SMC="Diode_SMD:D_SMC",
    SMB="Diode_SMD:D_SMB",
    SMA="Diode_SMD:D_SMA",
    SOD123="Diode_SMD:D_SOD-123",
    LED="LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder",
    TP="TestPoint:TestPoint_Pad_D1.5mm",
    TPLOOP="TestPoint:TestPoint_Keystone_5015_Micro_Mini",     # SMD GND clip point: no leads through the board
    PTC2920="Fuse:Fuse_2920_7451Metric_Pad2.10x5.45mm_HandSolder",
    PTC1812="Fuse:Fuse_1812_4532Metric_Pad1.30x3.40mm_HandSolder",
    XT90M="PowerBoard_rev3:XT90PB-M", XT90F="PowerBoard_rev3:XT90PB-F",
    XT60M="Connector_AMASS:AMASS_XT60-M_1x02_P7.20mm_Vertical",
    XT60F="Connector_AMASS:AMASS_XT60-F_1x02_P7.20mm_Vertical",
    SMB_D="Diode_SMD:D_SMB",
    XT30M="Connector_AMASS:AMASS_XT30UPB-M_1x02_P5.0mm_Vertical",
    XT30F="Connector_AMASS:AMASS_XT30UPB-F_1x02_P5.0mm_Vertical",
    PTCR300="PowerBoard_rev3:Fuse_Bourns_MF-R300_P5.08mm",
    DEANS="PowerBoard_rev3:DEANS-T",
    XH2="Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical",
    XH4="Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical",
    GH4="Connector_JST:JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical",
    CLIK4="Connector_Molex:Molex_CLIK-Mate_502585-0470_1x04-1MP_P1.50mm_Horizontal",
    USBC="PowerBoard_rev3:USB_C_Receptacle_GCT_USB4105-GF-A_2oz",
    FUSE154="Fuse:Fuseholder_Littelfuse_Nano2_154x",
    BTN="Button_Switch_SMD:SW_SPST_TL3342",
    ESP="PowerBoard_rev3:ESP32-S3-WROOM-1U_HandSolder",
    SSOP8="Package_SO:SSOP-8_2.95x2.8mm_P0.65mm",
    VSSOP8="Package_SO:VSSOP-8_2.3x2mm_P0.5mm",
    SOT23_5="Package_TO_SOT_SMD:SOT-23-5",
    L12="Inductor_SMD:L_12x12mm_H6mm",
    L6="Inductor_SMD:L_6.3x6.3_H3",
    SOT223="Package_TO_SOT_SMD:SOT-223-3_TabPin2",
)


def fmt_r(v):
    return rails.fmt_r(v).replace("R", "")


def R(sh, value, a, b, fp="R0603", at=None, rot=90, **f):
    v = fmt_r(value) if isinstance(value, (int, float)) else value
    return sh.add("Device:R", "R", v, FP[fp], {1: a, 2: b}, rot=rot, fields=f, at=at)


def C(sh, value, a, b, fp="C0603", volt=None, at=None, rot=90, **f):
    v = rails.fmt_c(value) if isinstance(value, (int, float)) else value
    if volt:
        v += f"/{volt}V"
    return sh.add("Device:C", "C", v, FP[fp], {1: a, 2: b}, rot=rot, fields=f, at=at)


def CP(sh, value, plus, minus, fp="CP10", at=None, rot=90, **f):
    return sh.add("Device:C_Polarized", "C", value, FP[fp], {1: plus, 2: minus}, rot=rot, fields=f, at=at)


def DZ(sh, value, k, a, fp="SMB", at=None, rot=0, **f):
    return sh.add("Device:D_Zener", "D", value, FP[fp], {1: k, 2: a}, fields=f, at=at, rot=rot)


def DS(sh, value, k, a, fp="SMA", at=None, rot=0, **f):
    return sh.add("Device:D_Schottky", "D", value, FP[fp], {1: k, 2: a}, fields=f, at=at, rot=rot)


LED_MPN = {"green": "APT2012SGC", "yellow": "APT2012SYCK", "red": "APT2012SURCK"}   # Kingbright 0805


def LED(sh, color, k, a, at=None, rot=0):
    return sh.add("Device:LED", "D", color, FP["LED"], {1: k, 2: a}, at=at, rot=rot,
                  fields={"MPN": LED_MPN[color]})


def TP(sh, net, at=None, rot=0):
    return sh.add("Connector:TestPoint", "TP", net.split(":")[-1], FP["TP"], {1: net}, at=at, rot=rot)


def GTP(sh, at):
    """GND test loop for a scope or meter clip: the loop sits on a GND symbol."""
    sh.mark("gnd", at)
    return sh.add("Connector:TestPoint", "TP", "GND", FP["TPLOOP"], {1: "GND"}, at=at)


def tp_panel(sh, nets, x0, y0, cols=3, dx=45.72, dy=10.16):
    """A block of test points on global nets: label on the left, test point on the wire."""
    for i, net in enumerate(nets):
        x, y = x0 + dx * (i % cols), y0 + dy * (i // cols)
        sh.mark("label", (x, y), net, 180)
        sh.wire((x, y), (x + 7.62, y))
        TP(sh, net, at=(x + 7.62, y))


def FET(sh, part, g, d, s, at=None, rot=0, mirror=False):
    fp = FP["D2PAK"] if part == "IPB042N10N3G" else FP["DPAK"]
    mpn = {"IPB042N10N3G": "IPB042N10N3GATMA1"}.get(part, part)
    return sh.add("PowerBoard_rev3:Q_NMOS_GDS", "Q", part, fp, {1: g, 2: d, 3: s}, fields={"MPN": mpn}, at=at,
                  rot=rot, mirror=mirror)


MINUS_PIN1 = ("XT30F", "XT30M", "XT60F", "XT60M", "DEANS")     # pad 1 is the - side on these


def CONN2(sh, value, fp, p1, p2, at=None, rot=0, mirror=False):
    """2-pin connector; p1 is the + side and is drawn on top. On XT30 / XT60 / Deans T it lands on pad 2."""
    if fp in MINUS_PIN1:
        j = sh.add("PowerBoard_rev3:Conn_01x02_MinusPin1", "J", value, FP[fp], {2: p1, 1: p2}, at=at, rot=rot,
                   mirror=mirror)
        j.alias = {1: 2, 2: 1, "1": 2, "2": 1}      # callers keep saying 1 = top (+), 2 = bottom
        return j
    return sh.add("Connector_Generic:Conn_01x02", "J", value, FP[fp], {1: p1, 2: p2}, at=at, rot=rot, mirror=mirror)


# Part numbers for parts created without one. Passives come from mpn_passives.json, picked by
# scratchpad tooling from Yageo RC (1 %) and Samsung / KEMET MLCC series, each checked in stock at Mouser.
PASSIVE_MPN = json.loads((Path(__file__).parent / "mpn_passives.json").read_text())
FIXED_MPN = {"MMBT3904": "MMBT3904LT1G", "2N7002": "2N7002LT1G"}


def auto_mpn(p):
    fp = p.footprint.split(":")[-1]
    if p.lib_id.startswith(("Device:R", "Device:C")):
        key = f"{p.lib_id}|{p.value}|{fp}"
        if key not in PASSIVE_MPN:
            raise SystemExit(f"no MPN for {key}: rerun the passive picker")
        return PASSIVE_MPN[key]
    if p.lib_id == "Connector:TestPoint":
        return "5015" if "Keystone_5015" in fp else "none: PCB pad"   # Keystone 5015 SMD clip point
    if p.lib_id.startswith("Jumper:SolderJumper"):
        return "none: PCB solder jumper"
    if p.value in FIXED_MPN:
        return FIXED_MPN[p.value]
    first = p.value.split()[0]
    if first.startswith(("XT30", "XT60", "XT90")):
        return first                                                   # AMASS, from TME
    if first.startswith(("B2B-", "B4B-")):
        return first                                                   # JST, Mouser
    if p.value.startswith("Deans"):
        return "none: Deans T, bought outside TME / Mouser (sourcing exception)"
    raise SystemExit(f"no MPN rule for {p.lib_id} {p.value}")


def hang(sh, p, pt, gap=2.54, top=1, bot=2, gnd=True):
    """Hang an upright two-pin part below pt (a point on a rail): a short wire down to its
    top pin, and GND on its bottom pin."""
    at_pin(p, top, (pt[0], pt[1] + gap))
    if gap:
        sh.wire(pt, PIN(p, top))
    if gnd:
        sh.mark("gnd", PIN(p, bot))
    return p


def marks(sh):
    """Shorthands for a drawn sheet: wire, gnd, 3v3, label."""
    return (sh.wire, lambda pt: sh.mark("gnd", pt), lambda pt: sh.mark("3v3", pt),
            lambda net, pt, ang: sh.mark("label", pt, net, ang))


# ------------------------------------------------------------------ sheet content

RAILNET = {"42V-D": "42D", "42V-C": "42C", "24V": "24", "12V": "12", "5V": "5", "54V": "54"}
VIN = {"42V-D": "BUS42", "42V-C": "BUS42", "24V": "VIN24", "12V": "VIN12", "5V": "VIN5", "54V": "BUS42"}
OUT = {"42V-D": "OUT42D", "42V-C": "OUT42C", "24V": "OUT24", "12V": "OUT12", "5V": "OUT5", "54V": "SW54_OUT"}
UVLO_ON = {"42V-D": 26, "42V-C": 26, "24V": 18, "12V": 9, "5V": 3.8, "54V": 26}
TVS_IN = {"24V": "SMBJ26A", "12V": "SMBJ13A", "5V": "SMBJ6.0A"}


def divider_bottom(top, v_full, v_adc=2.4):
    return rails.floor_e(top * v_adc / (v_full - v_adc), rails.E24)     # floor: never above the ADC range


def sheet_power_in():
    sh = Sheet("power_in", "Battery input, bus protection and battery monitor", [
        "Battery 10S 30-42 V. The pack fuse (58 V rated) and BMS live in the harness, not on this board.",
        "R_SHUNT: Bourns CSS2H-2512K, Kelvin land pattern (net-tie footprint). Route SNS_P / SNS_N as a pair from the fingers.",
        "Charging: charger + enters on the bus side of the shunt and leaves through the pack's charge port,",
        "so the INA228 counts it (net of the rover's own load). Charger - is NOT board GND: CHG_N goes straight to the pack's C-.",
        "INA228 at 0x40 (A0 = A1 = GND), ADCRANGE = 0 (+-163.84 mV = +-82 A): bus V, current, power, and on-chip charge / energy."],
        drawn=True, paper="A4")
    W, gnd, v33, lab = marks(sh)
    Y = 99.06                                      # VBAT rail, then BUS42 after the shunt

    jb = CONN2(sh, "XT90PB-M (battery in)", "XT90M", "VBAT", "GND", mirror=True)
    at_pin(jb, 1, (30.48, Y))
    W(PIN(jb, 2), (35.56, PIN(jb, 2)[1]), (35.56, 116.84))
    gnd((35.56, 116.84))
    W((35.56, 111.76), (43.18, 111.76))
    sh.mark("flag", (43.18, 111.76))
    TP(sh, "VBAT", at=(45.72, Y))
    lab("VBAT", (162.56, Y), 90)
    sh.mark("flag", (53.34, Y))
    hang(sh, DZ(sh, "SMCJ45A", "VBAT", "GND", fp="SMC", rot=270, MPN="SMCJ45A"), (63.5, Y))
    for x in (76.2, 91.44):
        hang(sh, CP(sh, "100uF/100V", "VBAT", "GND", fp="CPR12", rot=0, MPN="EEU-FC2A101"), (x, Y))
    for x in (106.68, 121.92, 137.16, 152.4):
        hang(sh, C(sh, 2.2e-6, "VBAT", "GND", fp="C1210", volt=100, rot=0, MPN="GRM32ER72A225KA35L"), (x, Y))

    # 4-terminal shunt (Kelvin land pattern) with the INA228 input filter straight below its sense fingers
    rs = sh.add("Device:R_Shunt", "R", "2m 5W", FP["SHUNT2512K"],
                {1: "VBAT", 2: "L:SNS_P", 3: "L:SNS_N", 4: "BUS42"}, rot=90, mirror="x",
                fields={"MPN": "CSS2H-2512K-2L00F"})
    at_pin(rs, 1, (171.45, Y))
    rs.label_at = {"Reference": (-3.81, -5.08), "Value": (-3.81, -2.54)}
    W(PIN(jb, 1), PIN(rs, 1))
    rp = hang(sh, R(sh, 10, "L:SNS_P", "L:INA_P", rot=0), PIN(rs, 2), gap=5.08, gnd=False)
    rn = hang(sh, R(sh, 10, "L:SNS_N", "L:INA_N", rot=0), PIN(rs, 3), gap=5.08, gnd=False)
    cf = sh.add("Device:C_Small", "C", "100nF", FP["C0603"], {1: "L:INA_P", 2: "L:INA_N"}, rot=90)
    at_pin(cf, 1, (PIN(rp, 2)[0], PIN(rp, 2)[1] + 2.54))
    U = sh.add("PowerBoard_rev3:INA228", "U", "INA228AIDGSR", "Package_SO:TSSOP-10_3x3mm_P0.5mm",
               {1: "GND", 2: "GND", 3: "INA_ALERT", 4: "I2C_SDA", 5: "I2C_SCL", 6: "+3V3", 7: "GND",
                8: "BUS42", 9: "L:INA_N", 10: "L:INA_P"}, fields={"MPN": "INA228AIDGSR"})
    at_pin(U, 10, (PIN(rp, 2)[0], 127.0))
    P = lambda n: PIN(U, n)
    W(PIN(rp, 2), P(10))
    W(PIN(rn, 2), P(9))
    W((P(8)[0], Y), P(8))

    # supply, I2C, alert pull-up
    vs = P(6)
    W(vs, (160.02, vs[1]), (160.02, vs[1] - 5.08))
    v33((160.02, vs[1] - 5.08))
    hang(sh, C(sh, 100e-9, "+3V3", "GND", rot=0), (160.02, vs[1]))
    for n, net in ((3, "INA_ALERT"), (4, "I2C_SDA"), (5, "I2C_SCL")):
        W(P(n), (215.9, P(n)[1]))
        lab(net, (215.9, P(n)[1]), 0)
    TP(sh, "INA_ALERT", at=(210.82, P(3)[1]))
    ra = R(sh, 10e3, "+3V3", "INA_ALERT", rot=0)
    at_pin(ra, 2, (203.2, P(3)[1]))
    v33(PIN(ra, 1))
    g, a0, a1 = P(7), P(2), P(1)
    W(g, (g[0], g[1] + 2.54), (a1[0], a1[1] + 2.54), a1)
    W(a0, (a0[0], a0[1] + 2.54))
    W((a0[0], a0[1] + 2.54), (a0[0], a0[1] + 5.08))
    gnd((a0[0], a0[1] + 5.08))

    # bus side: test point, flag, charger input, label
    W(PIN(rs, 4), (254.0, Y))
    lab("BUS42", (254.0, Y), 0)
    TP(sh, "BUS42", at=(198.12, Y))
    sh.mark("flag", (205.74, Y))
    jc = CONN2(sh, "XT60PT-M (charger in)", "XT60M", "BUS42", "L:CHG_N")
    at_pin(jc, 1, (243.84, 114.3))
    jc.label_at = {"Reference": (2.54, 0), "Value": (2.54, 2.54)}
    W((238.76, Y), (238.76, 114.3), PIN(jc, 1))

    # pack charge port: + on the battery side of the shunt, - back to the charger only
    yc = 170.18
    jbc = CONN2(sh, "XT60PT-M (pack charge port)", "XT60M", "VBAT", "L:CHG_N", mirror=True)
    at_pin(jbc, 1, (30.48, 132.08))
    W(PIN(jbc, 1), (50.8, 132.08), (50.8, Y))
    W(PIN(jbc, 2), (45.72, PIN(jbc, 2)[1]), (45.72, yc), (241.3, yc), (241.3, PIN(jc, 2)[1]), PIN(jc, 2))
    lab("L:CHG_N", (127.0, yc), 0)
    TP(sh, "L:CHG_N", at=(88.9, yc))
    GTP(sh, (66.04, 124.46))
    sh.shift(0, -22.86)
    return sh


def sheet_switch(r):
    """One eFuse, drawn like a hand-made sheet.

    Power flows left to right along two rails: VIN -> shunt -> drain rail -> FETs -> output
    rail. The TPS48111 sits below; its supply and sense pins reach up to the rails, its
    gate drive goes right to the FETs, control comes in from the left, and the
    setting parts hang below it.
    """
    v = rails.rail_values(r)
    n = RAILNET[r["name"]]
    vin, out = VIN[r["name"]], OUT[r["name"]]
    on = r["default_on"]
    title = {"42V-D": "42 V drive group eFuse (3 x XT90, EPOS4)",
             "42V-C": "42 V converter-feed group eFuse (2 x XT90)",
             "24V": "24 V rail eFuse", "12V": "12 V rail eFuse (feeds the Orin)", "5V": "5 V rail eFuse",
             "54V": "54 V supply eFuse (input of the LM5122 boost)"}[r["name"]]
    notes = [f"TPS48111-Q1 + {v['n']} x {r['fet']} main + 1 pre-charge. Values from docs/rev3/rails.md "
             f"(trip {v['i_oc']:.1f} A after {v['c_tmr'] * v['k_tmr'] * 1e3:.0f} ms, latch-off; "
             + (f"short-circuit {v['i_sc']:.0f} A)." if v["r_iscp"] else "short-circuit trip disabled)."),
             f"Power-on: OFF in hardware{'; the ESP32 turns it on at boot (rails.md, power-on sequence)' if on else ''}. "
             "Q_T (MMBT3904) must sit against the main FET: remote temperature sense.",
             "Kelvin-route CS+ / CS- / ISCP from the R_SNS pads. Place C_ISCP right at the pins."]
    if r["name"] == "42V-D":
        notes.append("Motor switch (rear panel): MOTOR_KILL from the latch in the io sheet pulls EN/UVLO low through Q_EN; "
                     "the main and pre-charge FETs turn off in hardware.")
    sh = Sheet(f"sw_{n.lower()}", title, notes, drawn=True)
    W, M = sh.wire, sh.mark
    gnd = lambda pt: M("gnd", pt)
    v33 = lambda pt: M("3v3", pt)
    lab = lambda net, pt, ang: M("label", pt, net, ang)

    X, Y = 119.38, 172.72                  # TPS48111 centre
    yR, yO, yQ, yG = 96.52, 127.0, 111.76, 121.92   # VIN/drain rails, output rail, FETs, gate bus
    en = "L:EN"
    U = sh.add("PowerBoard_rev3:TPS48111-Q1", "U", "TPS48111-Q1", CUSTOM_FP_TPS,
               {20: "L:VS", 18: "L:CSP", 17: "L:SNSN", 19: "L:ISCP" if v["r_iscp"] else "L:SNSN", 1: en,
                3: f"INP_{n}", 2: f"INPG_{n}", 10: "L:DIODE", 6: "GND",
                12: "L:BST", 15: "L:PU", 14: "L:GATE", 13: out, 11: "L:GPRE",
                5: f"FLT_{n}", 4: f"FLT_{n}", 7: f"IMON_{n}", 8: "L:IWRN", 9: "L:TMR"},
               fields={"MPN": "TPS48111LQDGXRQ1"}, at=(X, Y))
    P = lambda num: PIN(U, num)
    vs, iscp, csp, csn = P(20), P(19), P(18), P(17)

    # --- power path: VIN rail, Kelvin shunt (sense pins below, the CS- one straight above CS-), drain rail
    shunt_mpn = {0.5e-3: "CSS2H-2512R-L500F", 1e-3: "CSS2H-2512R-1L00F", 2e-3: "CSS2H-2512K-2L00F",
                 3e-3: "CSS2H-2512K-3L00F"}[r["r_sns"]]
    rs = sh.add("Device:R_Shunt", "R", f"{r['r_sns'] * 1e3:g}m", FP["SHUNT2512K"],
                {1: vin, 2: "L:SNSP", 3: "L:SNSN", 4: "L:DRN"}, rot=90, mirror="x", fields={"MPN": shunt_mpn})
    at_pin(rs, 3, (csn[0], yR + 2.54))
    rs.label_at = {"Reference": (-3.81, -5.08), "Value": (-3.81, -2.54)}

    # --- top pins: R_SET from the + sense finger, CS- straight from the - finger, ISCP and VS from the rail
    rset = hang(sh, R(sh, v["r_set"], "L:SNSP", "L:CSP", rot=0), PIN(rs, 2), gnd=False)
    W(PIN(rset, 2), (PIN(rset, 2)[0], 111.76), (csp[0], 111.76), csp)
    W(PIN(rs, 3), csn)
    if v["r_iscp"]:
        risc = R(sh, v["r_iscp"], vin, "L:ISCP", at=(iscp[0], 111.76), rot=0)
        W((iscp[0], yR), PIN(risc, 1))
        W(PIN(risc, 2), iscp)
        cis = C(sh, 1e-9, "L:ISCP", "L:SNSN", at=(124.46, 134.62))
        W((iscp[0], 134.62), PIN(cis, 1))
        W(PIN(cis, 2), (csn[0], 134.62))
    else:
        W(iscp, (iscp[0], 134.62), (csn[0], 134.62))
    rvs = hang(sh, R(sh, 100, vin, "L:VS", rot=0), (vs[0], yR), gnd=False)     # TI: R_VS on noisy buses
    W(PIN(rvs, 2), vs)
    c1 = C(sh, 100e-9, "L:VS", "GND", volt=100, at=(91.44, 128.27), rot=0)
    c2 = C(sh, 1e-6, "L:VS", "GND", fp="C1206", volt=100, at=(76.2, 128.27), rot=0, MPN="GRM31CR72A105KA01L")
    W(PIN(c2, 1), (vs[0], 124.46))
    M("flag", (86.36, 124.46))         # VS is fed through R_VS, a passive part
    gnd(PIN(c1, 2))
    gnd(PIN(c2, 2))

    # --- EN/UVLO divider (and the motor-switch kill FET on the drive group)
    r_top = rails.nearest(10e3 * (UVLO_ON[r["name"]] / 1.18 - 1), rails.E24)
    ret = R(sh, r_top, vin, en, at=(66.04, 128.27), rot=0)
    reb = R(sh, 10e3, en, "GND", at=(66.04, 146.05), rot=0)
    W((66.04, yR), PIN(ret, 1))
    W(PIN(ret, 2), PIN(reb, 1))
    gnd(PIN(reb, 2))
    enp = P(1)
    W(enp, (73.66, enp[1]), (73.66, 139.7), (66.04, 139.7))
    # test pads per eFuse (2026-11-20): VOUT, IMON, FLT and GATE only; EN, INP, INP_G and VIN are probed on their
    # resistors
    if r["name"] == "42V-D":
        qe = sh.add("Transistor_FET:2N7002", "Q", "2N7002", FP["SOT23"], {1: "MOTOR_KILL", 2: "GND", 3: en},
                    at=(53.34, 144.78))
        W(PIN(qe, 3), (66.04, 139.7))
        gnd(PIN(qe, 2))
        W(PIN(qe, 1), (43.18, 144.78))
        lab("MOTOR_KILL", (43.18, 144.78), 180)

    # --- control inputs from the left. Every eFuse powers up OFF (pull-downs); the ESP32 sequences
    # the boot rails through the expander.
    inp, inpg, flti, fltt = P(3), P(2), P(5), P(4)
    for pin_, net in ((inp, f"INP_{n}"), (inpg, f"INPG_{n}")):
        y = pin_[1]
        W(pin_, (45.72, y))
        lab(net, (45.72, y), 180)
        rd = R(sh, 100e3, net, "GND", at=(60.96, y + 3.81), rot=0)
        gnd(PIN(rd, 2))
    # FLT_I / FLT_T are open-drain, wire-ORed; their pull-up sits on the mcu sheet by the ESP32 input
    W(flti, (73.66, flti[1]))
    lab(f"FLT_{n}", (73.66, flti[1]), 180)
    W(fltt, (86.36, fltt[1]), (86.36, flti[1]))
    TP(sh, f"FLT_{n}", at=(81.28, flti[1]))

    # --- gate drive to the right
    pu, pd, bst, src, g = P(15), P(14), P(12), P(13), P(11)
    # C_BST bank: n x 10 uF / 50 V X7R 1210, sized at half its nominal value (DC bias at 12 V)
    cbs = []
    for k in range(v["n_bst"]):
        cb = C(sh, rails.C_BST_PART[1], "L:BST", out, fp="C1210", volt=50,
               at=(152.4 + 10.16 * k, (bst[1] + src[1]) / 2), rot=0, MPN=rails.C_BST_PART[0])
        if v["n_bst"] > 1:
            cb.label_at = {"Reference": (-2.54, 6.35), "Value": (-2.54, 8.89)}
        W(PIN(cbs[-1], 1) if cbs else bst, PIN(cb, 1))
        if cbs:
            W(PIN(cbs[-1], 2), PIN(cb, 2))
        cbs.append(cb)
    xsrc = max(165.1, PIN(cbs[-1], 2)[0] + 5.08)
    W(src, (xsrc, src[1]), (xsrc, yO))
    rpu = R(sh, 10, "L:PU", "L:GATE", at=(154.94, pu[1]))
    W(pu, PIN(rpu, 1))
    W(PIN(rpu, 2), (160.02, pu[1]))
    W(pd, (160.02, pd[1]), (160.02, yG))
    TP(sh, "L:GATE", at=(162.56, yG))

    # --- main FETs between the drain rail and the output rail, gate bus below them
    xs = [182.88 + 33.02 * k for k in range(v["n"])]
    for k, xq in enumerate(xs):
        q_ = FET(sh, r["fet"], f"L:G{k + 1}", "L:DRN", out, at=(xq, yQ))
        rgk = R(sh, 2.2, "L:GATE", f"L:G{k + 1}", at=(xq - 11.43, yQ))
        W(PIN(rgk, 2), PIN(q_, 1))
        W(PIN(rgk, 1), (PIN(rgk, 1)[0], yG))
        W(PIN(q_, 2), (PIN(q_, 2)[0], yR))
        W(PIN(q_, 3), (PIN(q_, 3)[0], yO))
    W((160.02, yG), (xs[-1] - 15.24, yG))

    # --- pre-charge FET, gate slewed by R_g + C_g from the G pin
    xp = xs[-1] + 33.02
    qp = FET(sh, r["fet"], "L:GPRE", "L:DRN", out, at=(xp, yQ))
    W(PIN(qp, 2), (PIN(qp, 2)[0], yR))
    W(PIN(qp, 3), (PIN(qp, 3)[0], yO))
    W(g, (xp - 10.16, g[1]), (xp - 10.16, yQ), PIN(qp, 1))
    rgp = R(sh, 330, "L:GPRE", "L:CG", at=(185.42, g[1] + 3.81), rot=0)
    # C_g charges to VOUT + 12 V (about 54 V on the 42 V rails)
    cgp = C(sh, v["c_g"], "L:CG", "GND", fp="C1206", volt=100 if r["v"] > 30 else 50, at=(185.42, g[1] + 15.24), rot=0)
    W(PIN(rgp, 2), PIN(cgp, 1))
    gnd(PIN(cgp, 2))
    W(PIN(rs, 4), (PIN(qp, 2)[0], yR))
    lab("L:DRN", (PIN(rs, 4)[0] + 5.08, yR), 0)        # named so board netclasses can find it

    # --- emergency bypass: solder a short solid-copper wire bridge across these two holes to power
    # the output with this eFuse out of the loop. After the shunt, so IMON still reads.
    big = r["design"] > 20
    xb = PIN(qp, 2)[0] + 22.86
    jb = sh.add("Jumper:Jumper_2_Open", "JP", f"BYPASS ({'2.5' if big else '1.5'} mm2 wire bridge)",
                f"PowerBoard_rev3:{BRIDGE_BIG if big else BRIDGE_SMALL}",
                {1: "L:DRN", 2: out}, rot=270, fields={"MPN": "none: solid copper wire bridge, fitted only in an emergency"})
    at_pin(jb, 1, (xb, yR + 7.62))
    jb.label_at = {"Reference": (2.54, -1.27), "Value": (2.54, 1.27)}
    W((PIN(qp, 2)[0], yR), (xb, yR), PIN(jb, 1))
    W(PIN(jb, 2), (xb, yO))

    # --- output rail: voltage sense, indicator, test point, label
    xe = 302.26
    W((xsrc, yO), (xe, yO))
    lab(out, (xe, yO), 0)
    if True:     # on the 54 V eFuse this is its own output (the boost input); VSNS_OUT_54 is the boost output
        vsn = "VSNS_SW54" if r["name"] == "54V" else f"VSNS_OUT_{n}"
        xd = 271.78
        rvt = R(sh, 100e3, out, vsn, at=(xd, 134.62), rot=0)
        rvb = R(sh, divider_bottom(100e3, r["v"] * 1.15), vsn, "GND", at=(xd, 152.4), rot=0)
        W((xd, yO), PIN(rvt, 1))
        W(PIN(rvt, 2), PIN(rvb, 1))
        gnd(PIN(rvb, 2))
        W((xd, 143.51), (xd - 5.08, 143.51))
        lab(vsn, (xd - 5.08, 143.51), 180)
        cv = C(sh, 10e-9, vsn, "GND", at=(xd + 5.08, 149.86), rot=0)
        W((xd, 146.05), PIN(cv, 1))
        gnd(PIN(cv, 2))
    r_led = rails.nearest((r["v"] - 2.0) / 2e-3, rails.E6)
    rl = R(sh, r_led, out, "L:LED", fp="R1206" if r["v"] > 20 else "R0603", at=(289.56, 134.62), rot=0)
    led = LED(sh, "green", "GND", "L:LED", at=(289.56, 146.05), rot=90)
    W((289.56, yO), PIN(rl, 1))
    W(PIN(rl, 2), PIN(led, 2))
    gnd(PIN(led, 1))
    M("flag", (281.94, yO))
    TP(sh, out, at=(297.18, yO))

    # --- settings below the driver
    gp, dp, ip, wp, tp_ = P(6), P(10), P(7), P(8), P(9)
    W(gp, (gp[0], gp[1] + 2.54))
    gnd((gp[0], gp[1] + 2.54))
    qt = sh.add("Transistor_BJT:MMBT3904", "Q", "MMBT3904", FP["SOT23"], {1: "L:DIODE", 2: "GND", 3: "L:DIODE"},
                at=(101.6, 220.98))
    W(dp, (dp[0], 208.28), (PIN(qt, 1)[0], 208.28), PIN(qt, 1))
    W(PIN(qt, 3), (PIN(qt, 3)[0], 208.28))
    gnd(PIN(qt, 2))
    W(ip, (ip[0], 215.9), (175.26, 215.9))
    lab(f"IMON_{n}", (175.26, 215.9), 0)
    rim = R(sh, v["r_imon"], f"IMON_{n}", "GND", at=(ip[0], 219.71), rot=0)
    cim = C(sh, 10e-9, f"IMON_{n}", "GND", at=(ip[0] + 7.62, 219.71), rot=0)
    gnd(PIN(rim, 2))
    gnd(PIN(cim, 2))
    TP(sh, f"IMON_{n}", at=(162.56, 215.9))
    riw = R(sh, v["r_iwrn"], "L:IWRN", "GND", at=(wp[0], 205.74), rot=0)
    W(wp, PIN(riw, 1))
    gnd(PIN(riw, 2))
    W(tp_, (tp_[0], 200.66), (tp_[0] + 7.62, 200.66))
    rtm = R(sh, v["r_tmr"], "L:TMR", "GND", at=(tp_[0], 204.47), rot=0)
    prev = None
    for k, (cval, cmpn, cv) in enumerate(v["c_tmr_parts"]):
        ctm = C(sh, cval, "L:TMR", "GND", fp="C1206", volt=f"{cv:g}", at=(tp_[0] + 7.62 + 10.16 * k, 204.47), rot=0,
                MPN=cmpn)
        if prev:
            W(PIN(prev, 1), PIN(ctm, 1))
        gnd(PIN(ctm, 2))
        prev = ctm
    gnd(PIN(rtm, 2))

    # --- converter rails: input TVS, bulk capacitance, input voltage sense (left of everything)
    sh.shift(20.32, 0)
    lab(vin, (22.86, yR), 180)
    W((22.86, yR), PIN(rs, 1))
    if r["name"] in TVS_IN:
        hang(sh, DZ(sh, TVS_IN[r["name"]], vin, "GND", rot=270, MPN=TVS_IN[r['name']]),
             (30.48, yR), gap=3.81)
        for x in (45.72, 58.42):
            hang(sh, CP(sh, "47uF/35V", vin, "GND", fp="CPR6", rot=0, MPN="EEU-FC1V470"), (x, yR), gap=3.81)
        xd = 71.12
        rit = R(sh, 100e3, vin, f"VSNS_IN_{n}", at=(xd, 110.49), rot=0)
        rib = R(sh, divider_bottom(100e3, r["v"] * 1.25), f"VSNS_IN_{n}", "GND", at=(xd, 134.62), rot=0)
        W((xd, yR), PIN(rit, 1))
        W(PIN(rit, 2), PIN(rib, 1))
        gnd(PIN(rib, 2))
        W((xd, 124.46), (xd - 5.08, 124.46))
        lab(f"VSNS_IN_{n}", (xd - 5.08, 124.46), 180)
        cvi = C(sh, 10e-9, f"VSNS_IN_{n}", "GND", at=(xd + 5.08, 130.81), rot=0)
        W((xd, 127.0), PIN(cvi, 1))
        gnd(PIN(cvi, 2))
    return sh


CUSTOM_FP_TPS = "PowerBoard_rev3:TI_DGX0019A_VSSOP-19"


def sheet_brake():
    """Brake chopper on the drive-group output: dumps EPOS4 braking energy into an external resistor
    when the bus climbs above what a full battery can explain."""
    sh = Sheet("brake", "Brake chopper on the drive output (EPOS4 braking energy)", [
        "On at 47.5 V, off at 45.0 V: above a full pack (42 V), below the SMCJ45A (50 V min) and the EPOS4 50/5 (56 V abs. max, cutout 58 V).",
        "Hardware only: TLV1805 compares OUT42D against an LM4040 2.5 V reference and drives Q_BR directly. Runs from OUT42D through a 12 V zener,",
        "so it also works with the drive eFuse off (braking current then reaches OUT42D through the main FETs' body diodes).",
        "R_BRAKE is off-board on a heatsink (e.g. 10 ohm 100 W: 4.7 A at 47 V). BRAKE_ACT tells the ESP32 (IO39); firmware logs it and",
        "drops the drive group if the chopper stays on (battery not accepting charge for a long time)."], drawn=True)
    W, gnd, v33, lab = marks(sh)
    y0, yv = 81.28, 104.14                       # OUT42D rail, 12 V local supply line

    lab("OUT42D", (25.4, y0), 180)
    # 12 V supply for the comparator, from the drive output itself
    r1 = hang(sh, R(sh, 4.7e3, "OUT42D", "L:VBR_A", fp="R1206", rot=0), (129.54, y0), gnd=False)
    r2 = hang(sh, R(sh, 4.7e3, "L:VBR_A", "L:VBR", fp="R1206", rot=0), PIN(r1, 2), gnd=False)
    W(PIN(r2, 2), (129.54, yv), (83.82, yv))
    hang(sh, DZ(sh, "12V", "L:VBR", "GND", fp="SOD123", rot=270, MPN="MMSZ5242BT1G"), (109.22, yv))
    hang(sh, C(sh, 10e-6, "L:VBR", "GND", fp="C1206", volt=25, rot=0), (116.84, yv))
    TP(sh, "L:VBR", at=(93.98, yv))
    sh.mark("flag", (124.46, yv))

    U = sh.add("PowerBoard_rev3:TLV1805", "U", "TLV1805DBVR", "Package_TO_SOT_SMD:SOT-23-6",
               {3: "L:REF", 4: "L:SNS", 1: "L:BR_G", 6: "L:VBR", 2: "GND", 5: "GND"},
               fields={"MPN": "TLV1805DBVR"}, at=(101.6, 129.54))
    P = lambda n: PIN(U, n)
    W((P(6)[0], yv), P(6))
    gnd(P(2))
    gnd(P(5))

    # 2.5 V reference into IN-
    rr = hang(sh, R(sh, 10e3, "L:VBR", "L:REF", rot=0), (83.82, yv), gnd=False)
    yref = PIN(rr, 2)[1] + 2.54
    W(PIN(rr, 2), (83.82, yref), (88.9, yref), (88.9, P(3)[1]), P(3))
    ref = sh.add("Reference_Voltage:LM4040DBZ-2.5", "U", "LM4040 2.5V", "Package_TO_SOT_SMD:SOT-23",
                 {1: "L:REF", 2: "GND", 3: None}, rot=90, fields={"MPN": "LM4040B25IDBZR"})
    ref.label_at = {"Reference": (-12.7, -1.27), "Value": (-12.7, 1.27)}
    hang(sh, ref, (83.82, yref))

    # OUT42D divider into IN+, hysteresis from OUT
    rt = R(sh, 178e3, "OUT42D", "L:SNS", fp="R1206", rot=0)
    at_pin(rt, 2, (66.04, P(4)[1] - 2.54))
    W((66.04, y0), PIN(rt, 1))
    W(PIN(rt, 2), (66.04, P(4)[1]), P(4))
    hang(sh, R(sh, 10e3, "L:SNS", "GND", rot=0), (66.04, P(4)[1]))
    yh = 152.4
    rh = R(sh, 845e3, "L:BR_G", "L:SNS", rot=270)
    at_pin(rh, 2, (88.9, yh))
    W((71.12, P(4)[1]), (71.12, yh), PIN(rh, 2))
    W(PIN(rh, 1), (114.3, yh), (114.3, P(1)[1]))

    # output: gate resistor, brake FET, status divider (also the gate pull-down)
    rg = R(sh, 22, "L:BR_G", "L:BR_GATE")
    at_pin(rg, 1, (119.38, P(1)[1]))
    W(P(1), PIN(rg, 1))
    q = FET(sh, "IPB042N10N3G", "L:BR_GATE", "L:BRK_SW", "GND")
    at_pin(q, 1, (137.16, P(1)[1]))
    W(PIN(rg, 2), PIN(q, 1))
    gnd(PIN(q, 3))
    rs1 = hang(sh, R(sh, 100e3, "L:BR_GATE", "BRAKE_ACT", rot=0), (132.08, P(1)[1]), gnd=False)
    ys = PIN(rs1, 2)[1] + 10.16
    W(PIN(rs1, 2), (132.08, ys), (137.16, ys))
    hang(sh, R(sh, 36e3, "BRAKE_ACT", "GND", rot=0), (132.08, ys))
    lab("BRAKE_ACT", (137.16, ys), 0)
    TP(sh, "BRAKE_ACT", at=(134.62, ys))

    # resistor connector with the freewheel diode across it
    ysw = 96.52
    W(PIN(q, 2), (PIN(q, 2)[0], ysw), (157.48, ysw), (157.48, y0 + 2.54), (162.56, y0 + 2.54))
    lab("L:BRK_SW", (PIN(q, 2)[0] + 1.27, ysw), 0)    # named so the board netclass can find it
    j = CONN2(sh, "XT60PT-F (brake resistor)", "XT60F", "OUT42D", "L:BRK_SW")
    at_pin(j, 1, (162.56, y0))
    j.label_at = {"Reference": (2.54, 0), "Value": (2.54, 2.54)}
    W((25.4, y0), PIN(j, 1))
    dfw = DS(sh, "STPS3H100U", "OUT42D", "L:BRK_SW", fp="SMB_D", rot=270, MPN="STPS3H100U")
    hang(sh, dfw, (152.4, y0), gnd=False)
    W(PIN(dfw, 2), (152.4, ysw))
    return sh


def sheet_outputs():
    sh = Sheet("outputs", "Rail inputs, output connectors and per-output PTCs", [
        "Male connectors are inputs (the cable side is the source), female are outputs. Pin 1 = +.",
        "Check every connector footprint's pin 1 against the housing's + marking before ordering.",
        "PTC per 24 / 12 / 5 V output: 3 A hold. Device cables must carry the PTC trip current (AWG 20 or heavier).",
        "OUT42D = drive group (3 x XT90, two EPOS4 50/5 per XT90). OUT42C = 2 converter feeds; one is Y-split in "
        "the harness for the third converter."],
        drawn=True)
    W, gnd, v33, lab = marks(sh)
    pitch = 10.16

    def conn(value, fp, net, pin1):
        """A 2-pin connector with pin 1 at pin1 (pins facing left), GND on pin 2, text to the right."""
        j = CONN2(sh, value, fp, net, "GND")
        at_pin(j, 1, pin1)
        j.label_at = {"Reference": (2.54, 0), "Value": (2.54, 2.54)}
        W(PIN(j, 2), (pin1[0] - 2.54, pin1[1] + 2.54))
        gnd((pin1[0] - 2.54, pin1[1] + 2.54))
        return j

    def column(net, xb, y0, rows, ptc=None):
        """A rail bus running down from its label, one connector (behind a PTC if given) per row."""
        ys = [y0 + pitch * i for i in range(len(rows))]
        W((xb - 5.08, y0 - 5.08), (xb, y0 - 5.08), (xb, ys[-1]))
        lab(net, (xb - 5.08, y0 - 5.08), 180)
        for i, ((value, fp), y) in enumerate(zip(rows, ys)):
            if ptc:
                part, pfp = ptc
                f = sh.add("Device:Polyfuse", "F", part, FP[pfp], {1: net, 2: f"L:O{net[3:]}_{i + 1}"}, rot=90,
                           fields={"MPN": part})
                at_pin(f, 1, (xb + 2.54, y))
                f.label_at = {"Reference": (-3.81, -2.54), "Value": (-3.81, 2.54)}
                W((xb, y), PIN(f, 1))
                W(PIN(f, 2), (xb + 27.94, y))
                lab(f"L:O{net[3:]}_{i + 1}", (xb + 13.97, y), 0)
                conn(value, fp, f"L:O{net[3:]}_{i + 1}", (xb + 27.94, y))
            else:
                W((xb, y), (xb + 10.16, y))
                conn(value, fp, net, (xb + 10.16, y))

    # rail inputs: label on the left, flag, connector
    for i, (net, value, fp) in enumerate((("VIN24", "Deans T male (24 V in)", "DEANS"),
                                          ("VIN12", "XT60PT-M (12 V in)", "XT60M"),
                                          ("VIN5", "XT30UPB-M (5 V in)", "XT30M"))):
        y = 71.12 + 15.24 * i
        lab(net, (35.56, y), 180)
        W((35.56, y), (45.72, y))
        sh.mark("flag", (40.64, y))
        conn(value, fp, net, (45.72, y))

    n = {r["name"]: r["n_out"] for r in rails.RAILS}
    column("OUT42D", 116.84, 71.12, [("XT90PB-F", "XT90F")] * n["42V-D"])
    column("OUT42C", 177.8, 71.12, [("XT90PB-F", "XT90F")] * n["42V-C"])
    # 54 V output: fuse in an SMD clip holder, then the XH header. The header is added here and the fuse after
    # the other columns, so neither moves the references around it (J13 stays J13, the fuse becomes F19).
    xz, yz = 238.76, 71.12
    W((xz - 5.08, yz - 5.08), (xz, yz - 5.08), (xz, yz))
    lab("OUT54", (xz - 5.08, yz - 5.08), 180)
    xm = (xz + 13.97, yz)
    W(xm, (xz + 27.94, yz))
    lab("L:O54_1", xm, 0)
    conn("B2B-XH-A(LF)(SN) (54 V to Zyxel)", "XH2", "L:O54_1", (xz + 27.94, yz))
    column("OUT24", 45.72, 137.16, [("Deans T female", "DEANS")] * n["24V"], ("SMD2920B300TF/30", "PTC2920"))
    column("OUT12", 134.62, 137.16, [("XT60PT-F", "XT60F")] * n["12V"], ("MF-LSMF300/16X-2", "PTC2920"))
    column("OUT5", 223.52, 137.16, [("XT30UPB-F", "XT30F")] * n["5V"], ("MF-LSMF300X-2", "PTC2920"))
    fz = sh.add("Device:Fuse", "F", "0154003.DRT (3 A slow-blow, clip holder)", FP["FUSE154"],
                {1: "OUT54", 2: "L:O54_1"}, rot=90, fields={"MPN": "0154003.DRT"})
    at_pin(fz, 1, (xz + 2.54, yz))
    fz.label_at = {"Reference": (-3.81, -2.54), "Value": (-3.81, 2.54)}
    W((xz, yz), PIN(fz, 1))
    W(PIN(fz, 2), xm)
    return sh


def sheet_boost():
    b = rails.BOOST
    bv = rails.boost_values(b)
    sh = Sheet("boost54", "54 V boost for the Zyxel XMG-105HP (LM5122)", [
        "30-42 V in from the converter group's output (OUT42C; the boost's own input eFuse was dropped 2026-11-20) "
        "-> 54.35 V, 100 W, 400 kHz. Values from docs/rev3/rails.md.",
        "R_SLOPE and the Type 2 network between COMP and FB come from the LM5122 equations in rails.py "
        "(crossover about 8-11 kHz); confirm with a load-step test.",
        "Q_BEN holds UVLO low until firmware clears BOOST_DIS (expander P15), after the converter group's main FETs "
        "are on. Until then OUT54 sits about 0.7 V below OUT42C (inductor + high-side body diode).",
        "Keep the SW node, Q_LS, Q_HS and the output ceramics in one tight loop. EP must be soldered (hand-solder hole)."],
        drawn=True)
    W, gnd, v33, lab = marks(sh)
    yi = 86.36                                   # power path: input rail, SW node, output rail
    U = sh.add("PowerBoard_rev3:LM5122", "U", "LM5122MH",
               "Package_SO:HTSSOP-20-1EP_4.4x6.5mm_P0.65mm_EP3.4x6.5mm_Mask2.96x2.96mm",
               {5: "L:VIN_LM", 4: "L:CSP", 3: "L:CSN", 6: "L:UVLO", 10: "L:FB", 11: "L:COMP", 7: "L:SS",
                8: "L:RT", 12: "L:SLOPE", 14: "L:RES", 13: "GND", 2: "GND",
                20: "L:BST", 19: "L:HO", 18: "L:SW", 16: "L:LO", 17: "L:VCC", 1: None,
                9: "GND", 15: "GND", 21: "GND"}, fields={"MPN": "LM5122MHX/NOPB"}, at=(109.22, 139.7))
    P = lambda n: PIN(U, n)

    # input rail: label, ceramics, current shunt
    lab("OUT42C", (25.4, yi), 180)
    rs = R(sh, "10m 2512", "OUT42C", "L:LIN", fp="R2512", MPN="WSL2512R0100FEA")
    at_pin(rs, 1, (P(4)[0], yi))
    W((25.4, yi), PIN(rs, 1))
    for x in (40.64, 53.34, 66.04, 78.74):
        hang(sh, C(sh, 2.2e-6, "OUT42C", "GND", fp="C1210", volt=100, rot=0), (x, yi))

    # VIN through an RC filter; current sense through 100 ohm + 1 nF, Kelvin from the shunt pads
    rv = hang(sh, R(sh, 3, "OUT42C", "L:VIN_LM", rot=0), (P(5)[0], yi), gnd=False)
    W(PIN(rv, 2), P(5))
    W((P(5)[0], 106.68), (86.36, 106.68))
    hang(sh, C(sh, 1e-6, "L:VIN_LM", "GND", fp="C1206", volt=100, rot=0), (86.36, 106.68))
    sh.mark("flag", (90.17, 106.68))
    rcp = hang(sh, R(sh, 100, "OUT42C", "L:CSP", rot=0), PIN(rs, 1), gap=7.62, gnd=False)
    rcn = hang(sh, R(sh, 100, "L:LIN", "L:CSN", rot=0), PIN(rs, 2), gap=7.62, gnd=False)
    ccs = C(sh, 1e-9, "L:CSP", "L:CSN")
    at_pin(ccs, 1, (P(4)[0], 106.68))
    W(PIN(rcp, 2), P(4))
    W(PIN(rcn, 2), P(3))

    # power stage: inductor, SW node with the low-side FET, high-side FET to the output
    ind = sh.add("Device:L", "L", "22uH 10.4A", "Inductor_SMD:L_Coilcraft_MSS1583-XXX",
                 {1: "L:LIN", 2: "L:SW"}, rot=90, fields={"MPN": "MSS1583-223MED"})
    at_pin(ind, 1, (119.38, yi))
    W(PIN(rs, 2), PIN(ind, 1))
    ql = FET(sh, b["fet"], "L:LO", "L:SW", "GND")
    at_pin(ql, 2, (134.62, 91.44))
    qh = FET(sh, b["fet"], "L:HO", "OUT54", "L:SW", rot=90, mirror="y")
    at_pin(qh, 3, (144.78, yi))
    qh.label_at = {"Reference": (-3.81, -8.89), "Value": (-3.81, -6.35)}
    W(PIN(ind, 2), PIN(qh, 3))
    lab("L:LIN", (113.03, yi), 0)
    lab("L:SW", (128.27, yi), 0)
    W((134.62, yi), PIN(ql, 2))
    gnd(PIN(ql, 3))
    W(P(16), (P(16)[0], PIN(ql, 1)[1]), PIN(ql, 1))

    # gate drive: SW back to the node, bootstrap cap and diode, VCC, HO up to the high-side gate
    W(P(18), (139.7, P(18)[1]), (139.7, yi))
    TP(sh, "L:SW", at=(137.16, P(18)[1]))
    cb = C(sh, 100e-9, "L:SW", "L:BST", volt=25, rot=0)
    at_pin(cb, 2, (134.62, P(20)[1]))
    W(P(20), (137.16, P(20)[1]))
    db = sh.add("Diode:BAS21", "D", "BAS21", FP["SOT23"], {3: "L:BST", 1: "L:VCC", 2: None},
                rot=270, fields={"MPN": "BAS21-7-F"})
    at_pin(db, 3, (137.16, P(20)[1]))
    db.label_at = {"Reference": (2.54, -1.27), "Value": (2.54, 1.27)}
    W(P(17), (144.78, P(17)[1]))
    hang(sh, C(sh, 4.7e-6, "L:VCC", "GND", fp="C0805", volt=16, rot=0), (144.78, P(17)[1]))
    sh.add("Connector:TestPoint", "TP", "VCC", FP["TP"], {1: "L:VCC"}, rot=180, at=(132.08, P(17)[1]))
    W(P(19), (PIN(qh, 1)[0], P(19)[1]), PIN(qh, 1))

    # settings comb on the left: each pin runs left, then down to its part
    yt = P(11)[1] + 5.08
    rut = R(sh, 200e3, "OUT42C", "L:UVLO", rot=0)
    at_pin(rut, 2, (35.56, P(6)[1]))
    W((35.56, yi), PIN(rut, 1))
    W(P(6), PIN(rut, 2))
    hang(sh, R(sh, 9.1e3, "L:UVLO", "GND", rot=0), PIN(rut, 2))
    comb = [(7, 43.18, C(sh, 100e-9, "L:SS", "GND", rot=0)),
            (8, 50.8, R(sh, rails.nearest(9e9 / b["fsw"], rails.E24), "L:RT", "GND", rot=0)),
            (12, 58.42, R(sh, bv["r_slope"], "L:SLOPE", "GND", rot=0)),
            (14, 66.04, C(sh, 470e-9, "L:RES", "GND", rot=0))]
    for pin_, x, part in comb:
        W(P(pin_), (x, P(pin_)[1]))
        hang(sh, part, (x, P(pin_)[1]), gap=yt - P(pin_)[1])
    # Type 2 compensation between COMP and FB (the LM5122 error amplifier is a voltage-output op-amp)
    W(P(11), (73.66, P(11)[1]))
    chf = hang(sh, C(sh, bv["c_hf"], "L:COMP", "L:FB", rot=0), (73.66, P(11)[1]),
               gap=yt - P(11)[1], gnd=False)
    rc = hang(sh, R(sh, bv["r_comp"], "L:COMP", "L:CC", rot=0), (81.28, P(11)[1]), gap=yt - P(11)[1], gnd=False)
    ccp = hang(sh, C(sh, bv["c_comp"], "L:CC", "L:FB", rot=0), PIN(rc, 2), gnd=False)

    # grounds and feedback below the IC
    g = [P(n) for n in (9, 15, 21, 13, 2)]
    yb = g[0][1] + 2.54
    W(g[0], (g[0][0], yb), (g[-1][0], yb), g[-1])
    for pt in g[1:-1]:
        W(pt, (pt[0], yb))
    W((P(21)[0], yb), (P(21)[0], yb + 2.54))
    gnd((P(21)[0], yb + 2.54))
    yf, xo = 177.8, 160.02
    W(P(10), (P(10)[0], yf))
    W(PIN(chf, 2), (73.66, yf), (P(10)[0], yf))
    W(PIN(ccp, 2), (81.28, yf))
    rft = R(sh, 221e3, "OUT54", "L:FB", rot=270)
    at_pin(rft, 2, (P(10)[0] + 5.08, yf))
    W((P(10)[0], yf), PIN(rft, 2))
    W(PIN(rft, 1), (xo, yf), (xo, yi))
    TP(sh, "OUT42C", at=(30.48, yi))
    GTP(sh, (175.26, 124.46))
    hang(sh, R(sh, 4.99e3, "L:FB", "GND", rot=0), (P(10)[0], yf))

    # output rail: ceramics, bulk, TVS, indicator, voltage sense, test point, flag, label
    xe = 360.68
    W(PIN(qh, 2), (xe, yi))
    lab("OUT54", (xe, yi), 0)
    for k in range(6):
        hang(sh, C(sh, 2.2e-6, "OUT54", "GND", fp="C1210", volt=100, rot=0), (167.64 + 15.24 * k, yi))
    for x in (259.08, 274.32):
        hang(sh, CP(sh, "100uF/100V", "OUT54", "GND", fp="CPR12", rot=0, MPN="EEU-FC2A101"), (x, yi))
    hang(sh, DZ(sh, "SMBJ58A", "OUT54", "GND", rot=270, MPN="SMBJ58A-13-F"), (289.56, yi))
    rl = hang(sh, R(sh, 22e3, "OUT54", "L:LED", fp="R1206", rot=0), (304.8, yi), gnd=False)
    led = LED(sh, "green", "GND", "L:LED", rot=90)
    at_pin(led, 2, (304.8, PIN(rl, 2)[1] + 2.54))
    W(PIN(rl, 2), PIN(led, 2))
    gnd(PIN(led, 1))
    xv = 320.04
    rvt = hang(sh, R(sh, 100e3, "OUT54", "VSNS_OUT_54", rot=0), (xv, yi), gnd=False)
    yn = PIN(rvt, 2)[1] + 2.54
    W(PIN(rvt, 2), (xv, yn))
    hang(sh, R(sh, divider_bottom(100e3, 62), "VSNS_OUT_54", "GND", rot=0), (xv, yn))
    W((xv, yn), (xv + 15.24, yn))
    hang(sh, C(sh, 10e-9, "VSNS_OUT_54", "GND", rot=0), (xv + 7.62, yn))
    lab("VSNS_OUT_54", (xv + 15.24, yn), 0)
    TP(sh, "OUT54", at=(345.44, yi))
    sh.mark("flag", (353.06, yi))

    # enable: Q_BEN shorts UVLO until firmware clears BOOST_DIS (pulled up, so the boost is off at power-up)
    q_en = sh.add("Transistor_FET:2N7002", "Q", "2N7002", FP["SOT23"], {1: "BOOST_DIS", 2: "GND", 3: "L:UVLO"})
    at_pin(q_en, 3, (22.86, P(6)[1] + 5.08))
    W(PIN(rut, 2), (22.86, P(6)[1]), PIN(q_en, 3))
    gnd(PIN(q_en, 2))
    W(PIN(q_en, 1), (5.08, PIN(q_en, 1)[1]))
    lab("BOOST_DIS", (5.08, PIN(q_en, 1)[1]), 180)
    rpu = R(sh, 100e3, "+3V3", "BOOST_DIS", rot=0)
    at_pin(rpu, 2, (10.16, PIN(q_en, 1)[1]))
    v33(PIN(rpu, 1))
    sh.shift(17.78, 0)
    return sh


def sheet_aux():
    sh = Sheet("aux_power", "Always-on aux supply: 42 V -> 5.3 V -> 3.3 V", [
        "LM5017 runs from the battery bus before any switch, so logic power never depends on USB (rule R1).",
        "Constant on-time buck: RON from VIN sets ~400 kHz; Rr / Cr / Cac inject the ripple FB needs (TI 'Type 3', datasheet 7.3.11).",
        "Bring-up test: battery only, USB unplugged -> +5V3_AUX, +5V_LOGIC and +3V3 present, ESP32 boots."],
        drawn=True)
    W, gnd, v33, lab = marks(sh)
    U = sh.add("PowerBoard_rev3:LM5017", "U", "LM5017MR/NOPB",
               "Package_SO:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.3mm",
               {2: "BUS42", 4: "L:RON", 3: "L:UVLO", 5: "L:FB", 7: "L:BST", 8: "L:SW", 6: "L:VCC",
                1: "GND", 9: "GND"}, fields={"MPN": "LM5017MR/NOPB"}, at=(101.6, 106.68))
    U.label_at = {"Reference": (-12.7, -15.24), "Value": (-6.35, -15.24)}
    P = lambda n: PIN(U, n)
    yr = P(2)[1]

    # input: bus label, ceramics, RON from VIN, UVLO divider (on 20.1 V, off 18.1 V with the 20 uA hysteresis)
    lab("BUS42", (25.4, yr), 180)
    W((25.4, yr), P(2))
    for x in (35.56, 48.26):
        hang(sh, C(sh, 2.2e-6, "BUS42", "GND", fp="C1210", volt=100, rot=0, MPN="GRM32ER72A225KA35L"), (x, yr))
    ron = hang(sh, R(sh, 150e3, "BUS42", "L:RON", fp="R1206", rot=0), (78.74, yr), gap=0, gnd=False)
    W(P(4), PIN(ron, 2))
    rut = hang(sh, R(sh, 100e3, "BUS42", "L:UVLO", fp="R1206", rot=0), (63.5, yr), gap=P(3)[1] - yr - 7.62, gnd=False)
    W(P(3), PIN(rut, 2))
    hang(sh, R(sh, 6.8e3, "L:UVLO", "GND", rot=0), PIN(rut, 2))
    W(P(1), (P(1)[0], P(1)[1] + 2.54), (P(9)[0], P(9)[1] + 2.54), P(9))
    W((101.6, P(1)[1] + 2.54), (101.6, P(1)[1] + 5.08))
    gnd((101.6, P(1)[1] + 5.08))

    # switch node: bootstrap cap, inductor, output rail; VCC cap
    cb = C(sh, 10e-9, "L:BST", "L:SW", volt=25, rot=0)
    at_pin(cb, 1, (124.46, P(7)[1]))
    W(P(7), PIN(cb, 1))
    W(P(8), PIN(cb, 2))
    ind = sh.add("Device:L", "L", "33uH", "Inductor_SMD:L_Bourns_SRR1260", {1: "L:SW", 2: "+5V3_AUX"}, rot=90,
                 fields={"MPN": "SRR1260-330M"})
    at_pin(ind, 1, (134.62, P(8)[1]))
    W(PIN(cb, 2), PIN(ind, 1))
    yo = P(8)[1]
    W(P(6), (121.92, P(6)[1]))
    hang(sh, C(sh, 1e-6, "L:VCC", "GND", volt=16, rot=0), (121.92, P(6)[1]), gap=0)
    TP(sh, "L:VCC", at=(119.38, P(6)[1]))
    TP(sh, "L:SW", at=(127.0, P(8)[1]), rot=180)
    GTP(sh, (40.64, 127.0))
    for x in (157.48, 170.18, 182.88):
        hang(sh, C(sh, 22e-6, "+5V3_AUX", "GND", fp="C1206", volt=25, rot=0, MPN="CL31A226KAHNNNE"), (x, yo))

    # ripple injection (Type 3): Rr from SW, Cr to VOUT, Cac into FB
    yn = 124.46
    rr = hang(sh, R(sh, 100e3, "L:SW", "L:RIP", fp="R1206", rot=0), (132.08, yo), gap=7.62, gnd=False)
    W(PIN(rr, 2), (132.08, yn), (147.32, yn))
    cr = C(sh, 3.3e-9, "+5V3_AUX", "L:RIP", volt=50, rot=0)
    at_pin(cr, 2, (147.32, yn))
    W(PIN(cr, 1), (147.32, yo))
    yac = 134.62
    cac = C(sh, 100e-9, "L:RIP", "L:FB", volt=16, rot=270)       # pin 1 on the right, towards the ripple node
    at_pin(cac, 1, (121.92, yac))
    yf = 142.24
    W((132.08, yn), (132.08, yac), PIN(cac, 1))
    W(PIN(cac, 2), (83.82, yac), (83.82, yf))

    # feedback: divider below the IC, fed from the output rail (5.29 V)
    W(P(5), (81.28, P(5)[1]), (81.28, yf), (88.9, yf))
    rft = R(sh, 33.2e3, "+5V3_AUX", "L:FB", rot=270)
    at_pin(rft, 2, (88.9, yf))
    W(PIN(rft, 1), (195.58, yf), (195.58, yo))
    hang(sh, R(sh, 10e3, "L:FB", "GND", rot=0), (81.28, yf))
    TP(sh, "+5V3_AUX", at=(200.66, yo))
    lab("+5V3_AUX", (205.74, yo), 90)
    sh.mark("flag", (213.36, yo))

    # diode-OR with USB VBUS
    d1 = DS(sh, "SS34", "+5V_LOGIC", "+5V3_AUX", rot=180, MPN="SS34-E3/57T")
    at_pin(d1, 2, (223.52, yo))
    W(PIN(ind, 2), PIN(d1, 2))
    d2 = DS(sh, "SS34", "+5V_LOGIC", "VBUS_USB", rot=180, MPN="SS34-E3/57T")
    at_pin(d2, 2, (223.52, 124.46))
    lab("VBUS_USB", (210.82, 124.46), 180)
    W((210.82, 124.46), PIN(d2, 2))
    W(PIN(d2, 1), (236.22, 124.46), (236.22, yo))

    # +5V_LOGIC rail and the 3.3 V LDO
    # AP7361C-33ER: the SOT223R pinout (1 GND, 2 OUT + tab, 3 IN), the stocked variant; the AMS1117
    # symbol has exactly that pinout. The -33E (1 IN, 2 GND, 3 OUT) is out of stock.
    ldo = sh.add("Regulator_Linear:AMS1117-3.3", "U", "AP7361C-33ER", FP["SOT223"],
                 {1: "GND", 2: "+3V3", 3: "+5V_LOGIC"}, fields={"MPN": "AP7361C-33ER-13"})
    at_pin(ldo, 3, (279.4, yo))
    W(PIN(d1, 1), PIN(ldo, 3))
    gnd(PIN(ldo, 1))
    sh.mark("flag", (241.3, yo))
    hang(sh, C(sh, 22e-6, "+5V_LOGIC", "GND", fp="C1206", volt=25, rot=0, MPN="CL31A226KAHNNNE"), (248.92, yo))
    TP(sh, "+5V_LOGIC", at=(256.54, yo))
    W((264.16, yo), (264.16, 93.98), (269.24, 93.98))
    lab("+5V_LOGIC", (269.24, 93.98), 0)
    x3 = 325.12
    W(PIN(ldo, 2), (x3, yo))
    v33((x3, yo))
    hang(sh, C(sh, 10e-6, "+3V3", "GND", fp="C0805", volt=10, rot=0), (299.72, yo))
    hang(sh, C(sh, 100e-9, "+3V3", "GND", rot=0), (312.42, yo))
    TP(sh, "+3V3", at=(320.04, yo))
    return sh


ESP_PINS = {"1": "GND", "2": "+3V3", "3": "ESP_EN", "4": "I2C_SDA", "5": "I2C_SCL", "6": "FLT_42D",
            "7": "FLT_42C", "8": "UART_SP", "9": "FLT_5", "10": "UART_TX", "11": "UART_RX", "12": "FLT_24",
            "13": "USB_DN", "14": "USB_DP", "15": None, "16": None, "17": "FLT_12", "18": "INA_ALERT",
            "19": "CAN_TXD", "20": "CAN_RXD", "21": "CAN_STB", "22": "MOTOR_OK", "23": "LED_1",
            "24": "LED_2", "25": "LED_3", "26": None, "27": "ESP_IO0", "28": None, "29": None, "30": None,
            "31": "SPARE_IO38", "32": "BRAKE_ACT", "33": "PWR_HOLD", "34": "PWR_BTN", "35": None, "36": "L:RXD0",
            "37": "L:TXD0", "38": "AUX5_SNS", "39": "NTC_SNS"}


def sheet_mcu():
    sh = Sheet("mcu", "ESP32-S3-WROOM-1U, USB, host UART, LEDs", [
        "Pin map from docs/rev3/design.md section 6.6. Strapping pins IO0/3/45/46 are not used for I/O.",
        "Host UART1 (IO17 TX, IO18 RX) goes to the Jetson AGX Orin 40-pin header (pins 10 / 8, GND pin 6).",
        "USB-C is for flashing and debug only (built-in USB-Serial/JTAG). VBUS only feeds the logic OR in aux_power.",
        "N8 module variant (no octal PSRAM). u.FL antenna: route the cable out of the rover body.",
        "USBLC6 arrays are flow-through: pins 1/6 and 3/4 are joined inside the part. Route each line across its pad pair.",
        "UART series resistors are 1k: if the Orin is off, an idle-high TX can push at most ~3 mA into its header. "
        "Firmware also keeps TX quiet until it sees the Orin's TX idling high."],
        drawn=True)
    W, gnd, v33, lab = marks(sh)
    sot236 = FP["SOT23"].replace("SOT-23", "SOT-23-6")

    # --- module: supply and decoupling on top, signals out to labels on both sides
    E = sh.add("RF_Module:ESP32-S3-WROOM-1", "U", "ESP32-S3-WROOM-1U-N8", FP["ESP"], ESP_PINS,
               fields={"MPN": "ESP32-S3-WROOM-1U-N8"}, at=(165.1, 152.4))
    E.label_at = {"Reference": (2.54, 30.48), "Value": (2.54, 33.02)}
    P = lambda n: PIN(E, n)
    W(P(2), (P(2)[0], 99.06))
    v33((P(2)[0], 99.06))
    W((P(2)[0], 101.6), (142.24, 101.6))
    hang(sh, C(sh, 22e-6, "+3V3", "GND", fp="C1206", volt=10, rot=0), (154.94, 101.6))
    hang(sh, C(sh, 100e-9, "+3V3", "GND", rot=0), (142.24, 101.6))
    gnd(P(1))
    for num, net in ESP_PINS.items():
        if net in (None, "GND", "+3V3") or net.startswith("L:"):
            continue
        pt = P(num)
        dx = -5.08 if pt[0] < 165.1 else 5.08
        W(pt, (pt[0] + dx, pt[1]))
        lab(net, (pt[0] + dx, pt[1]), 180 if dx < 0 else 0)
    W(P(37), (185.42, P(37)[1]), (185.42, 121.92))
    TP(sh, "L:TXD0", at=(185.42, 121.92))
    W(P(36), (193.04, P(36)[1]), (193.04, 116.84))
    TP(sh, "L:RXD0", at=(193.04, 116.84))

    # --- reset and boot: pull-up, (cap), button to GND, test point on each node
    for net, y, cap in (("ESP_EN", 83.82, True), ("ESP_IO0", 116.84, False)):
        lab(net, (25.4, y), 180)
        W((25.4, y), (66.04, y))
        r = R(sh, 10e3, "+3V3", net, rot=0)
        at_pin(r, 2, (35.56, y))
        v33(PIN(r, 1))
        if cap:
            hang(sh, C(sh, 1e-6, net, "GND", rot=0), (45.72, y))
        sw = sh.add("Switch:SW_Push", "SW", "RESET" if cap else "BOOT", FP["BTN"], {1: net, 2: "GND"}, rot=270,
                    fields={"MPN": "TL3342F160QG"})
        hang(sh, sw, (55.88, y))
        sw.label_at = {"Reference": (2.54, -1.27), "Value": (2.54, 1.27)}
        TP(sh, net, at=(66.04, y))

    # --- I2C pull-ups
    for x, net in ((30.48, "I2C_SDA"), (63.5, "I2C_SCL")):
        r = R(sh, 4.7e3, "+3V3", net, rot=0)
        at_pin(r, 1, (x, 144.78))
        v33(PIN(r, 1))
        W(PIN(r, 2), (x, 154.94), (x + 7.62, 154.94))
        lab(net, (x + 7.62, 154.94), 0)

    # --- board temperature (NTC) and logic supply sense
    nt = sh.add("Device:Thermistor_NTC", "TH", "10k NTC", FP["R0603"], {1: "+3V3", 2: "NTC_SNS"},
                fields={"MPN": "NCP18XH103F03RB"})
    at_pin(nt, 1, (30.48, 175.26))
    v33(PIN(nt, 1))
    yn = PIN(nt, 2)[1] + 2.54
    W(PIN(nt, 2), (30.48, yn), (48.26, yn))
    hang(sh, R(sh, 10e3, "NTC_SNS", "GND", rot=0), (30.48, yn))
    hang(sh, C(sh, 100e-9, "NTC_SNS", "GND", rot=0), (40.64, yn))
    lab("NTC_SNS", (48.26, yn), 0)
    lab("+5V_LOGIC", (68.58, 172.72), 180)
    W((68.58, 172.72), (83.82, 172.72))
    ra = hang(sh, R(sh, 10e3, "+5V_LOGIC", "AUX5_SNS", rot=0), (83.82, 172.72), gnd=False)
    ya = PIN(ra, 2)[1] + 2.54
    W(PIN(ra, 2), (83.82, ya), (93.98, ya))
    hang(sh, R(sh, 10e3, "AUX5_SNS", "GND", rot=0), (83.82, ya))
    lab("AUX5_SNS", (93.98, ya), 0)

    # --- USB-C: VBUS out to the logic OR, CC pull-downs, D-/D+ through the ESD array
    J = sh.add("Connector:USB_C_Receptacle_USB2.0_16P", "J", "USB4105-GF-A (flash/debug)", FP["USBC"],
               {"A1": "GND", "A4": "VBUS_USB", "A5": "L:CC1", "B5": "L:CC2", "A6": "L:DP_C", "B6": "L:DP_C",
                "A7": "L:DN_C", "B7": "L:DN_C", "A8": None, "B8": None, "SH": "GND"}, at=(241.3, 101.6),
               fields={"MPN": "USB4105-GF-A"})
    Pj = lambda n: PIN(J, n)
    xu = Pj("A4")[0]
    W(Pj("A4"), (320.04, Pj("A4")[1]))
    lab("VBUS_USB", (320.04, Pj("A4")[1]), 0)
    sh.mark("flag", (271.78, Pj("A4")[1]))
    for n, net in (("A5", "L:CC1"), ("B5", "L:CC2")):
        W(Pj(n), (xu + 2.54, Pj(n)[1]))
        lab(net, (xu + 2.54, Pj(n)[1]), 0)
    W(Pj("A7"), (xu + 2.54, Pj("A7")[1]), (xu + 2.54, Pj("B7")[1]))
    W(Pj("A6"), (xu + 2.54, Pj("A6")[1]), (xu + 2.54, Pj("B6")[1]))
    W(Pj("B6"), (xu + 2.54, Pj("B6")[1]))
    gnd(Pj("A1"))
    gnd(Pj("SH"))
    esd = sh.add("Power_Protection:USBLC6-2SC6", "U", "USBLC6-2SC6", sot236,
                 {1: "L:DN_C", 6: "USB_DN", 3: "L:DP_C", 4: "USB_DP", 5: "VBUS_USB", 2: "GND"},
                 fields={"MPN": "USBLC6-2SC6"})
    at_pin(esd, 1, (276.86, Pj("B7")[1]))
    esd.label_at = {"Reference": (7.62, -7.62), "Value": (7.62, -5.08)}
    W(Pj("B7"), PIN(esd, 1))
    W((xu + 2.54, Pj("A6")[1]), PIN(esd, 3))
    W(PIN(esd, 5), (PIN(esd, 5)[0], Pj("A4")[1]))
    gnd(PIN(esd, 2))
    for n, net in ((6, "USB_DN"), (4, "USB_DP")):
        W(PIN(esd, n), (299.72, PIN(esd, n)[1]))
        lab(net, (299.72, PIN(esd, n)[1]), 0)
    for x, net in ((271.78, "L:CC1"), (292.1, "L:CC2")):
        lab(net, (x - 5.08, 124.46), 180)
        W((x - 5.08, 124.46), (x, 124.46))
        hang(sh, R(sh, 5.1e3, net, "GND", rot=0), (x, 124.46))

    # --- host UART to the Orin: 1k series, ESD on TX/RX, JST-XH 4p (same family as the power-button header)
    x0, yt = 220.98, 152.4
    lab("UART_TX", (x0, yt), 180)
    lab("UART_RX", (x0, yt + 2.54), 180)
    rtx = R(sh, 1e3, "UART_TX", "L:TX_R")
    at_pin(rtx, 1, (x0 + 5.08, yt))
    rrx = R(sh, 1e3, "L:RX_R", "UART_RX", rot=270)
    at_pin(rrx, 2, (x0 + 5.08, yt + 2.54))
    rtx.label_at = {"Reference": (-3.81, -5.08), "Value": (-3.81, -2.54)}
    rrx.label_at = {"Reference": (-3.81, 5.08), "Value": (-3.81, 7.62)}
    W((x0, yt), PIN(rtx, 1))
    W((x0, yt + 2.54), PIN(rrx, 2))
    e2 = sh.add("Power_Protection:USBLC6-2SC6", "U", "USBLC6-2SC6", sot236,
                {1: "L:TX_R", 6: "L:TX_C", 3: "L:RX_R", 4: "L:RX_C", 5: "+3V3", 2: "GND"},
                fields={"MPN": "USBLC6-2SC6"})
    at_pin(e2, 1, (x0 + 17.78, yt))
    e2.label_at = {"Reference": (7.62, -7.62), "Value": (7.62, -5.08)}
    W(PIN(rtx, 2), PIN(e2, 1))
    W(PIN(rrx, 1), PIN(e2, 3))
    v33(PIN(e2, 5))
    gnd(PIN(e2, 2))
    gh = sh.add("Connector_Generic:Conn_01x04", "J", "B4B-XH-A(LF)(SN) (host UART to Orin)",
                FP["XH4"], {1: "L:TX_C", 2: "L:RX_C", 3: "GND", 4: "L:SP_C"},
                fields={"MPN": "B4B-XH-A(LF)(SN)"})
    xg = x0 + 50.8
    at_pin(gh, 1, (xg, yt))
    gh.label_at = {"Reference": (3.81, -5.08), "Value": (3.81, -2.54)}
    W(PIN(e2, 6), PIN(gh, 1))
    W(PIN(e2, 4), PIN(gh, 2))
    W(PIN(gh, 3), (xg - 5.08, yt + 5.08), (xg - 5.08, yt + 15.24))
    gnd((xg - 5.08, yt + 15.24))
    rsp = R(sh, 1e3, "L:SP_C", "UART_SP", rot=0)
    at_pin(rsp, 1, (xg - 2.54, yt + 10.16))
    W(PIN(rsp, 1), (xg - 2.54, yt + 7.62), PIN(gh, 4))
    W(PIN(rsp, 2), (xg - 2.54, yt + 22.86), (xg - 12.7, yt + 22.86))
    lab("UART_SP", (xg - 12.7, yt + 22.86), 180)

    # --- pull-ups for the five eFuse fault lines (open-drain FLT_I + FLT_T on each eFuse sheet); the sixth,
    # FLT_54 until its eFuse was dropped, stays as a pulled-up spare input on GPIO38
    for i, rn in enumerate(("42D", "42C", "24", "12", "5", "54")):
        x = 142.24 + 10.16 * i
        net = "SPARE_IO38" if rn == "54" else f"FLT_{rn}"
        rfl = R(sh, 10e3, "+3V3", net, rot=0)
        at_pin(rfl, 1, (x, 205.74))
        v33(PIN(rfl, 1))
        W(PIN(rfl, 2), (x, 218.44))
        lab(net, (x, 218.44), 270)

    # --- test points for the buses and control lines
    tp_panel(sh, ["I2C_SDA", "I2C_SCL", "INA_ALERT", "UART_TX", "UART_RX", "UART_SP",
                  "CAN_TXD", "CAN_RXD", "CAN_STB"], 35.56, 215.9)
    GTP(sh, (127.0, 246.38))

    # --- status LEDs
    for i in (1, 2, 3):
        y = 193.04 + 10.16 * (i - 1)
        lab(f"LED_{i}", (x0, y), 180)
        rl = R(sh, 1e3, f"LED_{i}", f"L:LED{i}")
        at_pin(rl, 1, (x0 + 5.08, y))
        W((x0, y), PIN(rl, 1))
        led = LED(sh, ["green", "yellow", "red"][i - 1], "GND", f"L:LED{i}", rot=180)
        at_pin(led, 2, (x0 + 17.78, y))
        W(PIN(rl, 2), PIN(led, 2))
        W(PIN(led, 1), (x0 + 27.94, y))
        gnd((x0 + 27.94, y))
    return sh


def sheet_can():
    sh = Sheet("can", "CAN bus to the EPOS4 drives (TCAN1044AV)", [
        "Bus pins survive +-58 V faults internally. No external TVS: a 24 V CAN TVS would conduct if a CAN wire touched 42 V.",
        "Split termination enabled by closing both solder jumpers (only at a bus end).",
        "STB has an internal pull-up: the transceiver stays in standby (bus silent) until the ESP32 drives STB low.",
        "Connectors match the EPOS4 50/5 X11 / X12: Molex CLIK-Mate 4p (1 CAN_H, 2 CAN_L, 3 GND, 4 shield), "
        "so maxon's CAN-CAN cable 520858 plugs straight in. Board header 502585-0470 mates with plug 502578-0400.",
        "Terminate both bus ends only: here with the jumpers if the board is an end, on the last EPOS4 with "
        "DIP switch SW1 position 7 (120 ohm). Source: EPOS4 50/5 Hardware Reference 2023-07, section 3.3.11."],
        drawn=True)
    W, gnd, v33, lab = marks(sh)
    U = sh.add("PowerBoard_rev3:TCAN1044AV", "U", "TCAN1044AVDRQ1", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
               {1: "CAN_TXD", 4: "CAN_RXD", 8: "CAN_STB", 3: "+5V_LOGIC", 5: "+3V3",
                7: "CANH", 6: "CANL", 2: "GND"}, fields={"MPN": "TCAN1044AVDRQ1"}, at=(101.6, 132.08))
    P = lambda n: PIN(U, n)

    # logic side from the ESP32
    for pin_, net in ((1, "CAN_TXD"), (4, "CAN_RXD"), (8, "CAN_STB")):
        W(P(pin_), (73.66, P(pin_)[1]))
        lab(net, (73.66, P(pin_)[1]), 180)
    gnd(P(2))

    # supplies: VCC from +5V_LOGIC, VIO from +3V3, each with its decoupling cap
    vcc, vio = P(3), P(5)
    W(vcc, (vcc[0], 109.22), (73.66, 109.22))
    lab("+5V_LOGIC", (73.66, 109.22), 180)
    hang(sh, C(sh, 100e-9, "+5V_LOGIC", "GND", rot=0), (86.36, 109.22))
    W(vio, (vio[0], 101.6))
    v33((vio[0], 101.6))
    W((vio[0], 109.22), (119.38, 109.22))
    hang(sh, C(sh, 100e-9, "+3V3", "GND", rot=0), (119.38, 109.22))

    # bus: CANH runs along the top, CANL along the bottom, split termination between them
    yh, yl, xt = 99.06, 160.02, 154.94
    W(P(7), (127.0, P(7)[1]), (127.0, yh), (190.5, yh), (190.5, 149.86))
    W(P(6), (132.08, P(6)[1]), (132.08, yl), (195.58, yl), (195.58, 127.0))
    jp = "Jumper:SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm"
    jh = sh.add("Jumper:SolderJumper_2_Open", "JP", "TERM_H", jp, {1: "CANH", 2: "L:TH"}, rot=270)
    jl = sh.add("Jumper:SolderJumper_2_Open", "JP", "TERM_L", jp, {1: "CANL", 2: "L:TL"}, rot=90)
    rh = R(sh, 60.4, "L:TH", "L:TMID", fp="R0805", rot=0, MPN="ERJ-6ENF60R4V")
    rl = R(sh, 60.4, "L:TMID", "L:TL", fp="R0805", rot=0, MPN="ERJ-6ENF60R4V")
    at_pin(jh, 1, (xt, 104.14))
    at_pin(rh, 1, (xt, 116.84))
    at_pin(rl, 1, (xt, 132.08))
    at_pin(jl, 2, (xt, 144.78))
    W((xt, yh), PIN(jh, 1))
    W(PIN(jh, 2), PIN(rh, 1))
    W(PIN(rh, 2), PIN(rl, 1))
    W(PIN(rl, 2), PIN(jl, 2))
    W(PIN(jl, 1), (xt, yl))
    W((xt, 127.0), (165.1, 127.0))
    hang(sh, C(sh, 4.7e-9, "L:TMID", "GND", volt=50, rot=0), (165.1, 127.0))

    TP(sh, "CANH", at=(170.18, yh))
    TP(sh, "CANL", at=(170.18, yl))
    GTP(sh, (220.98, yh))

    # two connectors in parallel (CANH crosses CANL once, without a junction)
    for i, y1 in ((1, 124.46), (2, 149.86)):
        j = sh.add("Connector_Generic_MountingPin:Conn_01x04_MountingPin", "J",
                   f"CAN {i} (Molex CLIK-Mate 502585-0470)", FP["CLIK4"],
                   {1: "CANH", 2: "CANL", 3: "GND", 4: "GND", "MP": "GND"},
                   fields={"MPN": "502585-0470"})
        at_pin(j, 1, (210.82, y1))
        W((190.5, y1), PIN(j, 1))
        W((195.58, PIN(j, 2)[1]), PIN(j, 2))
        xg, yg = 205.74, PIN(j, "MP")[1] + 2.54
        W(PIN(j, 3), (xg, PIN(j, 3)[1]), (xg, yg + 2.54))
        W(PIN(j, 4), (xg, PIN(j, 4)[1]))
        W(PIN(j, "MP"), (PIN(j, "MP")[0], yg), (xg, yg))
        gnd((xg, yg + 2.54))
    return sh


# The 54 V eFuse's channels (IMON_54, VSNS_SW54, INP_54, INPG_54) are spare since it was dropped (2026-11-20):
# unused ADC inputs are tied to GND, the expander pins left open like the other spares.
ADC_MAP = [["IMON_42D", "IMON_42C", "IMON_24", "IMON_12", "IMON_5", "GND", "VSNS_OUT_42D", "VSNS_OUT_42C"],
           ["VSNS_OUT_24", "VSNS_OUT_12", "VSNS_OUT_5", "VSNS_OUT_54", "VSNS_IN_24", "VSNS_IN_12", "VSNS_IN_5", "GND"]]
EXP_MAP = {4: "INP_42D", 5: "INP_42C", 6: "INP_24", 7: "INP_12", 8: "INP_5", 9: None, 10: "INPG_42D",
           11: "INPG_42C", 13: "INPG_24", 14: "INPG_12", 15: "INPG_5", 16: None, 17: "MOTOR_ARM",
           18: "BOOST_DIS", 19: None, 20: None}


def sheet_io():
    sh = Sheet("io", "Switch control (TCA9535), measurement (ADC128D818 x2), motor-switch latch", [
        "TCA9535 at 0x20 holds INP / INP_G for all five switches, so they keep their state across an ESP32 reset.",
        "ADC128D818 at 0x1D (A1 = A0 = LOW) and 0x1E (A1 = LOW, A0 = MID). Internal 2.56 V reference (VREF pin open); "
        "firmware must select mode 1 (reg 0Bh) or IN7 is not converted. 12 ms per channel.",
        "The motor-switch RC (1 M x 100 nF hold-off, diode for a fast clear) goes through a Schmitt buffer into /CLR.",
        "Rear-panel motor switch (closed = motors allowed): opening it clears the latch (Q low, /Q high) -> MOTOR_KILL holds "
        "the drive group off.",
        "It stays off after the switch closes until the ESP32 clocks MOTOR_ARM, which it does only after dropping INP and "
        "re-running pre-charge. Power-on also starts latched off. The battery E-STOP is separate (BMS soft-start).",
        "Soft power: the BMS soft-start loop (through the E-STOP) is closed by button pole 1 or by the AQY282S that the ESP32 "
        "holds (PWR_HOLD, GPIO40). Pole 2 reads the button (PWR_BTN, GPIO41, low = on). Button off: the ESP32 asks the "
        "Jetson to stow and shut down, then releases PWR_HOLD. A reset or a dead ESP32 releases it too."],
        drawn=True)
    W, gnd, v33, lab = marks(sh)

    # --- I/O expander: I2C in from the left, switch controls out to the right
    exp = {1: None, 2: "GND", 3: "GND", 21: "GND", 22: "I2C_SCL", 23: "I2C_SDA", 24: "+3V3", 12: "GND"}
    exp.update(EXP_MAP)
    X = sh.add("Interface_Expansion:TCA9535PWR", "U", "TCA9535PWR", "Package_SO:TSSOP-24_4.4x7.8mm_P0.65mm",
               exp, fields={"MPN": "TCA9535PWR"}, at=(91.44, 109.22))
    P = lambda n: PIN(X, n)
    W(P(24), (P(24)[0], 68.58))
    v33((P(24)[0], 68.58))
    W((P(24)[0], 71.12), (111.76, 71.12))
    hang(sh, C(sh, 100e-9, "+3V3", "GND", rot=0), (111.76, 71.12))
    gnd(P(12))
    for n, net in ((23, "I2C_SDA"), (22, "I2C_SCL")):
        W(P(n), (50.8, P(n)[1]))
        lab(net, (50.8, P(n)[1]), 180)
    xa = P(3)[0] - 5.08
    W(P(3), (xa, P(3)[1]), (xa, P(21)[1] + 2.54))
    W(P(2), (xa, P(2)[1]))
    W(P(21), (xa, P(21)[1]))
    gnd((xa, P(21)[1] + 2.54))
    for n, net in EXP_MAP.items():
        if net and net != "MOTOR_ARM":
            W(P(n), (129.54, P(n)[1]))
            lab(net, (129.54, P(n)[1]), 0)

    # --- motor-switch latch: MOTOR_ARM clocks it, the switch loop clears it
    F = sh.add("74xGxx:74AUP1G74", "U", "SN74LVC1G74DCUT", FP["VSSOP8"],
               {1: "MOTOR_ARM", 2: "+3V3", 3: "MOTOR_KILL", 4: "GND", 5: "L:LATCH_Q", 6: "L:CLR",
                7: "+3V3", 8: "+3V3"}, fields={"MPN": "SN74LVC1G74DCUT"}, at=(152.4, 175.26))
    F.label_at = {"Reference": (5.08, 13.97), "Value": (5.08, 16.51)}
    Q = lambda n: PIN(F, n)
    arm = P(17)
    W(arm, (116.84, arm[1]), (116.84, Q(1)[1]), Q(1))
    hang(sh, R(sh, 100e3, "MOTOR_ARM", "GND", rot=0), (127.0, Q(1)[1]))
    TP(sh, "MOTOR_ARM", at=(132.08, Q(1)[1]))
    TP(sh, "MOTOR_KILL", at=(168.91, Q(3)[1]))
    GTP(sh, (40.64, 160.02))
    yb = 157.48                                    # +3V3 bar over the flip-flop
    W(Q(2), (134.62, Q(2)[1]), (134.62, yb), (Q(8)[0], yb), Q(8))
    W(Q(7), (Q(7)[0], yb))
    W((134.62, yb), (127.0, yb))
    hang(sh, C(sh, 100e-9, "+3V3", "GND", rot=0), (127.0, yb))
    W((144.78, yb), (144.78, yb - 2.54))
    v33((144.78, yb - 2.54))
    gnd(Q(4))
    W(Q(3), (172.72, Q(3)[1]))
    lab("MOTOR_KILL", (172.72, Q(3)[1]), 0)
    W(Q(5), (180.34, Q(5)[1]))
    TP(sh, "L:LATCH_Q", at=(180.34, Q(5)[1]))

    # --- motor-switch loop (JST-XH like the other harness headers; 2 oz rules out PH): +3V3 -> 1k -> switch (closed = run) -> 10k to GND; filtered into MOTOR_OK and the CLR RC
    yl = 198.12
    j = CONN2(sh, "B2B-XH-A(LF)(SN) (motor switch)", "XH2", "L:ESRC", "L:ELOOP", mirror=True)
    j.fields["MPN"] = "B2B-XH-A(LF)(SN)"
    at_pin(j, 2, (40.64, yl))
    W(PIN(j, 1), (48.26, PIN(j, 1)[1]))
    rsrc = R(sh, 1e3, "+3V3", "L:ESRC", rot=0)
    at_pin(rsrc, 2, (48.26, PIN(j, 1)[1]))
    v33(PIN(rsrc, 1))
    re1 = R(sh, 1e3, "L:ELOOP", "MOTOR_OK")
    at_pin(re1, 1, (63.5, yl))
    W(PIN(j, 2), PIN(re1, 1))
    hang(sh, R(sh, 10e3, "L:ELOOP", "GND", rot=0), (55.88, yl))
    r1m = R(sh, 1e6, "MOTOR_OK", "L:CLR_RC")
    at_pin(r1m, 1, (83.82, yl))
    W(PIN(re1, 2), PIN(r1m, 1))
    hang(sh, C(sh, 100e-9, "MOTOR_OK", "GND", rot=0), (76.2, yl))
    dc = DS(sh, "1N5819HW", "MOTOR_OK", "L:CLR_RC", fp="SOD123", MPN="1N5819HW-7-F")
    at_pin(dc, 1, (83.82, 190.5))
    W((78.74, yl), (78.74, 190.5), PIN(dc, 1))
    W(PIN(dc, 2), (96.52, 190.5), (96.52, yl))
    # the 100 ms RC edge is far slower than the flip-flop's 10 ns/V input limit, so a Schmitt buffer squares it
    sb = sh.add("74xGxx:74LVC1G17", "U", "SN74LVC1G17DBVR", FP["SOT23_5"],
                {1: None, 2: "L:CLR_RC", 3: "GND", 4: "L:CLR", 5: "+3V3"}, fields={"MPN": "SN74LVC1G17DBVR"})
    at_pin(sb, 2, (106.68, yl))
    W(PIN(r1m, 2), PIN(sb, 2))
    W(PIN(sb, 4), (Q(6)[0], yl), Q(6))
    gnd(PIN(sb, 3))
    W(PIN(sb, 5), (PIN(sb, 5)[0], PIN(sb, 5)[1] - 2.54))
    v33((PIN(sb, 5)[0], PIN(sb, 5)[1] - 2.54))
    hang(sh, C(sh, 100e-9, "L:CLR_RC", "GND", rot=0), (101.6, yl))
    W((73.66, yl), (73.66, 182.88), (66.04, 182.88))
    lab("MOTOR_OK", (66.04, 182.88), 180)
    TP(sh, "MOTOR_OK", at=(69.85, 182.88))

    # --- two ADCs: inputs from the left, I2C to the right, decoupling cap clear of the labels
    for i, (chans, yc) in enumerate(zip(ADC_MAP, (106.68, 177.8))):
        conns = {1: None, 2: "I2C_SDA", 3: "I2C_SCL", 4: "GND", 5: "+3V3", 6: None,
                 7: "GND" if i == 0 else "L:A0MID", 8: "GND"}
        for k, net in enumerate(chans):
            conns[16 - k] = net
        A = sh.add("Analog_ADC:ADC128D818", "U", "ADC128D818", "Package_SO:TSSOP-16_4.4x5mm_P0.65mm",
                   conns, fields={"MPN": "ADC128D818CIMTX/NOPB"}, at=(231.14, yc))
        Pa = lambda n: PIN(A, n)
        for k, net in enumerate(chans):
            pt = Pa(16 - k)
            if net == "GND":
                W(pt, (pt[0] - 5.08, pt[1]))
                gnd((pt[0] - 5.08, pt[1]))
            else:
                W(pt, (195.58, pt[1]))
                lab(net, (195.58, pt[1]), 180)
        for n, net in ((2, "I2C_SDA"), (3, "I2C_SCL")):
            W(Pa(n), (248.92, Pa(n)[1]))
            lab(net, (248.92, Pa(n)[1]), 0)
        vp = Pa(5)
        W(vp, (vp[0], vp[1] - 10.16))
        v33((vp[0], vp[1] - 10.16))
        W((vp[0], vp[1] - 7.62), (269.24, vp[1] - 7.62))
        hang(sh, C(sh, 100e-9, "+3V3", "GND", rot=0), (269.24, vp[1] - 7.62))
        W((269.24, vp[1] - 7.62), (276.86, vp[1] - 7.62))            # datasheet: 1 uF || 100 nF on V+
        hang(sh, C(sh, 1e-6, "+3V3", "GND", volt=16, rot=0), (276.86, vp[1] - 7.62))
        gnd(Pa(4))
        a0, a1 = Pa(7), Pa(8)
        xg = a0[0] - 5.08
        if i == 0:
            W(a0, (xg, a0[1]), (xg, a1[1] + 2.54))
            W(a1, (xg, a1[1]))
            gnd((xg, a1[1] + 2.54))
        else:
            W(a1, (xg, a1[1]), (xg, a1[1] + 2.54))
            gnd((xg, a1[1] + 2.54))
            W(a0, (a0[0] - 10.16, a0[1]))
            hang(sh, R(sh, 10e3, "L:A0MID", "GND", rot=0), (a0[0] - 10.16, a0[1]))
            rt = R(sh, 10e3, "+3V3", "L:A0MID")
            at_pin(rt, 2, (a0[0] - 15.24, a0[1]))
            W(PIN(rt, 2), (a0[0] - 10.16, a0[1]))
            v33(PIN(rt, 1))

    # --- soft power: BMS soft-start loop (through the E-STOP) closed by the 2-pole latching button
    #     (pole 1) or by the AQY282S PhotoMOS that the ESP32 holds (PWR_HOLD); pole 2 is a dry sense contact
    jb = sh.add("Connector_Generic:Conn_01x04", "J", "B4B-XH-A(LF)(SN) (power button, 2-pole)", FP["XH4"],
                {1: "L:SS_A", 2: "L:SS_B", 3: "L:BTN_SNS", 4: "GND"}, mirror=True)
    at_pin(jb, 1, (40.64, 228.6))
    for n_, net in ((1, "L:SS_A"), (2, "L:SS_B"), (3, "L:BTN_SNS")):
        W(PIN(jb, n_), (50.8, PIN(jb, n_)[1]))
        lab(net, (50.8, PIN(jb, n_)[1]), 0)
    W(PIN(jb, 4), (45.72, PIN(jb, 4)[1]), (45.72, PIN(jb, 4)[1] + 2.54))
    gnd((45.72, PIN(jb, 4)[1] + 2.54))
    js = CONN2(sh, "B2B-XH-A(LF)(SN) (BMS soft-start via E-STOP)", "XH2", "L:SS_A", "L:SS_B", mirror=True)
    at_pin(js, 1, (40.64, 248.92))
    for n_, net in ((1, "L:SS_A"), (2, "L:SS_B")):
        W(PIN(js, n_), (50.8, PIN(js, n_)[1]))
        lab(net, (50.8, PIN(js, n_)[1]), 0)

    k = sh.add("Relay_SolidState:AQY282S", "U", "AQY282S", "Package_SO:SO-4_4.4x4.3mm_P2.54mm",
               {1: "L:SSR_A", 2: "GND", 3: "L:SS_A", 4: "L:SS_B"}, fields={"MPN": "AQY282S"})
    at_pin(k, 1, (91.44, 241.3))
    yk = PIN(k, 1)[1]
    rk = R(sh, 330, "PWR_HOLD", "L:SSR_A")
    at_pin(rk, 2, (86.36, yk))
    W(PIN(rk, 2), PIN(k, 1))
    W(PIN(rk, 1), (68.58, yk))
    lab("PWR_HOLD", (68.58, yk), 180)
    hang(sh, R(sh, 100e3, "PWR_HOLD", "GND", rot=0), (76.2, yk))
    TP(sh, "PWR_HOLD", at=(71.12, yk))
    gnd(PIN(k, 2))
    for n_, net in ((3, "L:SS_A"), (4, "L:SS_B")):
        W(PIN(k, n_), (116.84, PIN(k, n_)[1]))
        lab(net, (116.84, PIN(k, n_)[1]), 0)
        TP(sh, net, at=(111.76, PIN(k, n_)[1]), rot=180 if n_ == 3 else 0)

    # button sense (pole 2): 10k pull-up, 1k / 100 nF filter; PWR_BTN low = button ON
    ys = 241.3
    lab("L:BTN_SNS", (139.7, ys), 180)
    rpu = R(sh, 10e3, "+3V3", "L:BTN_SNS", rot=0)
    at_pin(rpu, 2, (144.78, ys))
    W((139.7, ys), PIN(rpu, 2))
    v33(PIN(rpu, 1))
    rf = R(sh, 1e3, "L:BTN_SNS", "PWR_BTN")
    at_pin(rf, 1, (149.86, ys))
    W(PIN(rpu, 2), PIN(rf, 1))
    hang(sh, C(sh, 100e-9, "PWR_BTN", "GND", rot=0), (162.56, ys))
    W(PIN(rf, 2), (172.72, ys))
    TP(sh, "PWR_BTN", at=(167.64, ys))
    lab("PWR_BTN", (172.72, ys), 0)
    return sh


# ------------------------------------------------------------------ root, project, checks

def build():
    write_custom_lib()
    write_custom_footprints()
    # The dropped 54 V eFuse (rails.REMOVED_54V) is built but not emitted: counting its parts keeps every
    # later reference number where it was, so the layout scripts' references stay valid.
    reserved = sheet_switch(rails.REMOVED_54V)
    sheets = [sheet_power_in()] + [sheet_switch(r) for r in rails.RAILS] + [reserved] + \
             [sheet_brake(), sheet_outputs(), sheet_boost(), sheet_aux(), sheet_mcu(), sheet_can(), sheet_io()]
    root_uuid = uid("root")
    root_items = []
    x, y = 30.0, 60.0
    pages = 1
    for s in sheets:
        if s is reserved:
            for p in s.parts:
                PREFIX_COUNT[p.prefix] += 1
            continue
        su = uid("sheet", s.name)
        pages += 1
        page = str(pages)
        em = Emitter(s, f"/{root_uuid}/{su}")
        (PROJ / s.file).write_text(em.render(root_uuid, su, s.title))
        root_items.append(
            f'(sheet (at {fmt(x)} {fmt(y)}) (size 50.8 20.32) (exclude_from_sim no) (in_bom yes) (on_board yes) '
            f'(dnp no) (fields_autoplaced yes) (stroke (width 0.1524) (type solid)) (fill (color 0 0 0 0.0000)) '
            f'(uuid {q(su)}) '
            f'(property "Sheetname" {q(s.title[:60])} (at {fmt(x)} {fmt(y - 0.7)} 0) '
            f'(effects (font (size 1.27 1.27)) (justify left bottom))) '
            f'(property "Sheetfile" {q(s.file)} (at {fmt(x)} {fmt(y + 21)} 0) '
            f'(effects (font (size 1.27 1.27)) (justify left top) (hide yes))) '
            f'(instances (project {q(NAME)} (path {q("/" + root_uuid)} (page {q(page)})))))')
        x += 66.04
        if x > 330:
            x, y = 30.0, y + 40.64
    notes = ["Brno Mars Rover power board rev3. Design spec: docs/rev3/design.md; values: docs/rev3/rails.md.",
             "Generated by kicad/rev3/gen/build.py. Once you edit in KiCad, these files are the source of truth.",
             "Footprints still to create in PowerBoard_rev3.pretty: TI_DGX0019A_VSSOP-19, XT90PB-M, XT90PB-F, DEANS-T."]
    for i, n in enumerate(notes):
        root_items.append(f'(text {q(n)} (exclude_from_sim no) (at 30 {fmt(25 + i * 7)} 0) '
                          f'(effects (font (size 2 2)) (justify left bottom)) (uuid {q(uid("rootnote", i))}))')
    (PROJ / f"{NAME}.kicad_sch").write_text(
        f'(kicad_sch (version {SCH_VERSION}) (generator "eeschema") (generator_version "10.0")\n'
        f'(uuid {q(root_uuid)})\n(paper "A3")\n'
        f'(title_block (title "Power board rev3") (rev "3.0") (company "Brno Mars Rover"))\n'
        f'(lib_symbols)\n' + "\n".join(root_items) +
        '\n(sheet_instances (path "/" (page "1")))\n(embedded_fonts no)\n)\n')
    (PROJ / "sym-lib-table").write_text(
        '(sym_lib_table (version 7)\n  (lib (name "PowerBoard_rev3")(type "KiCad")'
        '(uri "${KIPRJMOD}/PowerBoard_rev3.kicad_sym")(options "")(descr "rev3 custom symbols"))\n)\n')
    (PROJ / "fp-lib-table").write_text(
        '(fp_lib_table (version 7)\n  (lib (name "PowerBoard_rev3")(type "KiCad")'
        '(uri "${KIPRJMOD}/PowerBoard_rev3.pretty")(options "")(descr "rev3 custom footprints"))\n)\n')
    sheets.remove(reserved)
    (PROJ / "PowerBoard_rev3.pretty").mkdir(exist_ok=True)
    pro = PROJ / f"{NAME}.kicad_pro"
    if not pro.exists():
        pro.write_text(json.dumps({"meta": {"filename": f"{NAME}.kicad_pro", "version": 3},
                                   "sheets": [[root_uuid, "Root"]] +
                                   [[uid("sheet", s.name), s.title[:60]] for s in sheets]}, indent=2))
    return sheets


def write_custom_footprints():
    """DGX-19 from TI's land pattern; XT90 / Deans T copied from rev2's SamacSys library."""
    lib = PROJ / "PowerBoard_rev3.pretty"
    lib.mkdir(exist_ok=True)
    # TI DGX0019A land pattern: 0.3 x 1.45 mm pads, 0.5 mm pitch, rows 4.4 mm apart (centres).
    # Pads lengthened outward to 1.8 mm for hand soldering. Pins 1-10 left (top down), 11-20 right
    # (bottom up); pin 16 is absent for creepage.
    pad_w, pad_l, row = 0.3, 1.8, 4.4 + 0.35
    pads = []
    for p in range(1, 21):
        if p == 16:
            continue
        x = -row / 2 if p <= 10 else row / 2
        y = -2.25 + (p - 1) * 0.5 if p <= 10 else 2.25 - (p - 11) * 0.5
        pads.append(f'  (pad "{p}" smd roundrect (at {fmt(x)} {fmt(y)}) (size {pad_l} {pad_w}) '
                    f'(layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.25) (uuid {q(uid("dgx", "pad", p))}))')
    cy, cx = 2.55 + 0.25, row / 2 + pad_l / 2 + 0.25
    u = lambda *k: f'(uuid {q(uid("dgx", *k))})'
    dgx = (
        '(footprint "TI_DGX0019A_VSSOP-19" (version 20241229) (generator "build.py") (layer "F.Cu")\n'
        '  (descr "TI DGX0019A VSSOP-19, 0.5 mm pitch, 5.1 x 3.0 mm body, pin 16 absent for creepage. '
        'Pads lengthened for hand soldering. https://www.ti.com/lit/ds/symlink/tps4811-q1.pdf")\n'
        '  (tags "VSSOP DGX TPS4811")\n'
        f'  (property "Reference" "REF**" (at 0 -3.9 0) (layer "F.SilkS") {u("ref")} (effects (font (size 1 1) (thickness 0.15))))\n'
        f'  (property "Value" "TI_DGX0019A_VSSOP-19" (at 0 3.9 0) (layer "F.Fab") {u("val")} (effects (font (size 1 1) (thickness 0.15))))\n'
        f'  (property "Datasheet" "https://www.ti.com/lit/ds/symlink/tps4811-q1.pdf" (at 0 0 0) (layer "F.Fab") (hide yes) {u("ds")} (effects (font (size 1.27 1.27))))\n'
        f'  (property "Description" "TI DGX0019A VSSOP-19" (at 0 0 0) (layer "F.Fab") (hide yes) {u("desc")} (effects (font (size 1.27 1.27))))\n'
        '  (attr smd)\n'
        f'  (fp_rect (start -1.5 -2.55) (end 1.5 2.55) (stroke (width 0.1) (type default)) (fill no) (layer "F.Fab") {u("fab")})\n'
        f'  (fp_line (start -1.6 -2.65) (end 1.6 -2.65) (stroke (width 0.12) (type default)) (layer "F.SilkS") {u("silk1")})\n'
        f'  (fp_line (start -1.6 2.65) (end 1.6 2.65) (stroke (width 0.12) (type default)) (layer "F.SilkS") {u("silk2")})\n'
        f'  (fp_circle (center -3.6 -2.25) (end -3.4 -2.25) (stroke (width 0.12) (type default)) (fill yes) (layer "F.SilkS") {u("pin1")})\n'
        f'  (fp_rect (start {fmt(-cx)} {fmt(-cy)}) (end {fmt(cx)} {fmt(cy)}) (stroke (width 0.05) (type default)) '
        f'(fill no) (layer "F.CrtYd") {u("crt")})\n' + "\n".join(pads) + '\n)\n')
    (lib / "TI_DGX0019A_VSSOP-19.kicad_mod").write_text(dgx)
    (lib / "R_Bourns_CSS2H-2512_Kelvin.kicad_mod").write_text(css2h_footprint())
    (lib / "Fuse_Bourns_MF-R300_P5.08mm.kicad_mod").write_text(mfr300_footprint())
    for name, wire, drill, pad, pitch in ((BRIDGE_BIG, "2.5", 2.3, 4.0, 7.62), (BRIDGE_SMALL, "1.5", 1.8, 3.2, 5.08)):
        (lib / f"{name}.kicad_mod").write_text(bypass_footprint(name, wire, drill, pad, pitch))
    rev2 = REPO / "kicad/ext/SamacSys_Parts.pretty"
    for src, dst in (("XT90", "XT90PB-M"), ("XT90", "XT90PB-F"), ("DEANS-T", "DEANS-T")):
        text = (rev2 / f"{src}.kicad_mod").read_text()
        text = re.sub(r'^\((footprint|module) "[^"]*"', rf'(\1 "{dst}"', text, count=1)
        text = re.sub(r'\n\s*\(model [^\n]*(?:\n\s+[^\n]*)*?\n\s*\)', '', text)   # drop rev2 3D paths
        (lib / f"{dst}.kicad_mod").write_text(text)


def css2h_footprint():
    """Bourns CSS2H-2512 on its recommended Kelvin pad layout (datasheet p.2).

    Current pads 1.80 x 3.40 mm, inner edges 3.40 mm apart. A 0.38 mm neck leaves each pad's
    inner edge at mid-height and turns into a 0.40 mm sense finger that ends 1.5 mm above the
    pad. Neck and finger are copper of the current pad, so the footprint is a net tie
    (1-2, 3-4): the sense nets stay separate in the netlist and meet the current path exactly
    where the resistive element starts.
    """
    u = lambda *k: f'(uuid {q(uid("css2h", *k))})'
    out = []
    for side, cur, sns in ((-1, "1", "2"), (1, "4", "3")):
        px = side * (1.70 + 0.90)
        out.append(f'  (pad "{cur}" smd rect (at {fmt(px)} 0) (size 1.8 3.4) (layers "F.Cu" "F.Paste" "F.Mask") {u("pad", cur)})')
        fx = side * 0.51
        out.append(f'  (pad "{sns}" smd rect (at {fmt(fx)} -2.94) (size 0.4 0.6) (layers "F.Cu") {u("pad", sns)})')
        pts = [(-1.70, -0.19), (-0.71, -0.19), (-0.71, -3.24), (-0.31, -3.24), (-0.31, 0.19), (-1.70, 0.19)]
        xy = " ".join(f"(xy {fmt(side * -x if side > 0 else x)} {fmt(y)})" for x, y in pts)
        out.append(f'  (fp_poly (pts {xy}) (stroke (width 0) (type solid)) (fill yes) (layer "F.Cu") {u("tie", cur)})')
    return (
        '(footprint "R_Bourns_CSS2H-2512_Kelvin" (version 20241229) (generator "build.py") (layer "F.Cu")\n'
        '  (descr "Bourns CSS2H-2512 current sense resistor on the recommended 4-terminal (Kelvin) pad layout. '
        'Pads 1/4 current, 2/3 sense. https://www.bourns.com/docs/product-datasheets/css2h-2512.pdf")\n'
        '  (tags "shunt kelvin 2512 CSS2H")\n'
        '  (net_tie_pad_groups "1, 2" "3, 4")\n'
        f'  (property "Reference" "REF**" (at 0 3.0 0) (layer "F.SilkS") {u("ref")} (effects (font (size 1 1) (thickness 0.15))))\n'
        f'  (property "Value" "CSS2H-2512" (at 0 4.5 0) (layer "F.Fab") {u("val")} (effects (font (size 1 1) (thickness 0.15))))\n'
        f'  (property "Datasheet" "https://www.bourns.com/docs/product-datasheets/css2h-2512.pdf" (at 0 0 0) (layer "F.Fab") (hide yes) {u("ds")} (effects (font (size 1.27 1.27))))\n'
        f'  (property "Description" "Bourns CSS2H-2512 Kelvin" (at 0 0 0) (layer "F.Fab") (hide yes) {u("desc")} (effects (font (size 1.27 1.27))))\n'
        '  (attr smd)\n'
        f'  (fp_rect (start -3.175 -1.525) (end 3.175 1.525) (stroke (width 0.1) (type default)) (fill no) (layer "F.Fab") {u("fab")})\n'
        f'  (fp_line (start -1.5 1.95) (end 1.5 1.95) (stroke (width 0.12) (type default)) (layer "F.SilkS") {u("silk")})\n'
        f'  (fp_rect (start -3.75 -3.5) (end 3.75 1.95) (stroke (width 0.05) (type default)) (fill no) (layer "F.CrtYd") {u("crt")})\n'
        + "\n".join(out) + "\n)\n")


def mfr300_footprint():
    """Bourns MF-R300 radial PPTC: 0.81 mm leads, 5.1 mm (5.08) spacing, body 12.0 x 3.0 mm max, 18.3 mm tall."""
    name = "Fuse_Bourns_MF-R300_P5.08mm"
    u = lambda *k: f'(uuid {q(uid(name, *k))})'
    return (
        f'(footprint "{name}" (version 20241229) (generator "build.py") (layer "F.Cu")\n'
        '  (descr "Bourns MF-R300 radial PTC resettable fuse, 3.0 A hold, 30 V. Lead spacing 5.1 mm, body 12.0 x 3.0 mm, '
        'height 18.3 mm. https://www.bourns.com/docs/product-datasheets/mfr.pdf")\n'
        '  (tags "PTC polyfuse radial MF-R")\n'
        f'  (property "Reference" "REF**" (at 2.54 -2.8 0) (layer "F.SilkS") {u("ref")} (effects (font (size 1 1) (thickness 0.15))))\n'
        f'  (property "Value" "MF-R300" (at 2.54 2.8 0) (layer "F.Fab") {u("val")} (effects (font (size 1 1) (thickness 0.15))))\n'
        f'  (property "Datasheet" "https://www.bourns.com/docs/product-datasheets/mfr.pdf" (at 0 0 0) (layer "F.Fab") (hide yes) {u("ds")} (effects (font (size 1.27 1.27))))\n'
        f'  (property "Description" "Bourns MF-R300 PPTC" (at 0 0 0) (layer "F.Fab") (hide yes) {u("desc")} (effects (font (size 1.27 1.27))))\n'
        '  (attr through_hole)\n'
        f'  (fp_rect (start -3.46 -1.5) (end 8.54 1.5) (stroke (width 0.1) (type default)) (fill no) (layer "F.Fab") {u("fab")})\n'
        f'  (fp_rect (start -3.58 -1.62) (end 8.66 1.62) (stroke (width 0.12) (type default)) (fill no) (layer "F.SilkS") {u("silk")})\n'
        f'  (fp_rect (start -3.86 -1.9) (end 8.94 1.9) (stroke (width 0.05) (type default)) (fill no) (layer "F.CrtYd") {u("crt")})\n'
        f'  (pad "1" thru_hole circle (at 0 0) (size 2.0 2.0) (drill 1.1) (layers "*.Cu" "*.Mask") {u("p1")})\n'
        f'  (pad "2" thru_hole circle (at 5.08 0) (size 2.0 2.0) (drill 1.1) (layers "*.Cu" "*.Mask") {u("p2")})\n'
        ')\n')


BRIDGE_BIG, BRIDGE_SMALL = "WireBridge_2.5mm2_P7.62mm", "WireBridge_1.5mm2_P5.08mm"


def bypass_footprint(name, wire, drill, pad, pitch):
    """Two plated holes for an emergency solid-copper wire bridge."""
    u = lambda *k: f'(uuid {q(uid(name, *k))})'
    h = pad / 2 + 0.5
    return (
        f'(footprint "{name}" (version 20241229) (generator "build.py") (layer "F.Cu")\n'
        f'  (descr "Emergency eFuse bypass: two plated holes, {pitch:g} mm apart, for a solid {wire} mm2 copper wire '
        f'bridge. Not fitted in normal use.")\n'
        '  (tags "bypass wire bridge")\n'
        f'  (property "Reference" "REF**" (at {fmt(pitch / 2)} {fmt(-h - 1.2)} 0) (layer "F.SilkS") {u("ref")} (effects (font (size 1 1) (thickness 0.15))))\n'
        f'  (property "Value" "{name}" (at {fmt(pitch / 2)} {fmt(h + 1.5)} 0) (layer "F.Fab") {u("val")} (effects (font (size 1 1) (thickness 0.15))))\n'
        f'  (property "Datasheet" "" (at 0 0 0) (layer "F.Fab") (hide yes) {u("ds")} (effects (font (size 1.27 1.27))))\n'
        f'  (property "Description" "eFuse bypass link" (at 0 0 0) (layer "F.Fab") (hide yes) {u("desc")} (effects (font (size 1.27 1.27))))\n'
        '  (attr through_hole)\n'
        f'  (fp_text user "BYPASS" (at {fmt(pitch / 2)} {fmt(h + 1.2)} 0) (layer "F.SilkS") {u("txt")} (effects (font (size 1 1) (thickness 0.15))))\n'
        f'  (fp_rect (start {fmt(-h + 0.2)} {fmt(-h + 0.2)}) (end {fmt(pitch + h - 0.2)} {fmt(h - 0.2)}) (stroke (width 0.15) (type default)) (fill no) (layer "F.SilkS") {u("box")})\n'
        f'  (fp_rect (start {fmt(-h)} {fmt(-h)}) (end {fmt(pitch + h)} {fmt(h)}) (stroke (width 0.05) (type default)) (fill no) (layer "F.CrtYd") {u("crt")})\n'
        f'  (pad "1" thru_hole circle (at 0 0) (size {pad} {pad}) (drill {drill}) (layers "*.Cu" "*.Mask") {u("p1")})\n'
        f'  (pad "2" thru_hole circle (at {fmt(pitch)} 0) (size {pad} {pad}) (drill {drill}) (layers "*.Cu" "*.Mask") {u("p2")})\n'
        ')\n')


def check_footprints():
    missing = []
    for fp in sorted(FOOTPRINTS):
        lib, name = fp.split(":")
        if lib == "PowerBoard_rev3":
            path = PROJ / "PowerBoard_rev3.pretty" / f"{name}.kicad_mod"
        else:
            path = FP_DIR / f"{lib}.pretty" / f"{name}.kicad_mod"
        if not path.exists():
            missing.append(fp)
    return missing


def run(cmd):
    return subprocess.run(KICAD_CLI + cmd, capture_output=True, text=True)


def verify():
    sch = str(PROJ / f"{NAME}.kicad_sch")
    out = PROJ / "gen" / "out"
    out.mkdir(exist_ok=True)
    net = out / "netlist.net"
    r = run(["sch", "export", "netlist", "--format", "kicadsexpr", "-o", str(net), sch])
    if r.returncode:
        print(r.stdout, r.stderr)
        raise SystemExit("netlist export failed")
    tree = parse(net.read_text())
    actual = {}
    for n in find(find(tree, "nets")[0], "net"):
        name = unq(find(n, "name")[0][1])
        for node in find(n, "node"):
            actual[(unq(find(node, "ref")[0][1]), unq(find(node, "pin")[0][1]))] = name
    want = defaultdict(set)
    for ref, pin, key in INTENDED:
        if key is not None:
            want[key].add((ref, pin))
    errors = []
    got = defaultdict(set)
    actual = {k: v for k, v in actual.items() if not k[0].startswith("#")}
    for (ref, pin), name in actual.items():
        got[name].add((ref, pin))
    for key, members in want.items():
        names = {actual.get(m) for m in members}
        if len(names) != 1 or None in names:
            errors.append(f"net {key}: pins land on {sorted(map(str, names))}")
            continue
        extra = got[names.pop()] - members
        if extra:
            errors.append(f"net {key}: also joined to {sorted(extra)[:6]}")
    for ref, pin, key in INTENDED:
        if key is None and (ref, pin) in actual and not actual[(ref, pin)].startswith("unconnected-"):
            errors.append(f"{ref}.{pin} should be unconnected but is on {actual[(ref, pin)]}")
    erc = out / "erc.json"
    run(["sch", "erc", "--format", "json", "--severity-all", "-o", str(erc), sch])
    report = json.loads(erc.read_text())
    counts = defaultdict(int)
    for s in report.get("sheets", []):
        for v in s.get("violations", []):
            counts[(v["severity"], v["type"])] += 1
    return errors, counts


if __name__ == "__main__":
    # FROZEN 2026-11-19 for layout: references are numbered in sheet order, so regenerating after an
    # edit renumbers parts and the PCB would silently swap them. Edit the schematic in KiCad instead.
    if "--force" not in sys.argv:
        raise SystemExit("build.py is frozen: the schematic is edited in KiCad from layout on. "
                         "Run with --force only if you accept redoing the PCB links.")
    sheets = build()
    r = run(["sch", "upgrade", "--force", str(PROJ / f"{NAME}.kicad_sch")])
    for s in sheets:
        run(["sch", "upgrade", "--force", str(PROJ / s.file)])
    run(["fp", "upgrade", "--force", str(PROJ / "PowerBoard_rev3.pretty")])
    parts = sum(len(s.parts) for s in sheets)
    print(f"wrote {len(sheets)} sheets, {parts} parts")
    missing = check_footprints()
    if missing:
        print("footprints not found:", *missing, sep="\n  ")
    errors, counts = verify()
    print("netlist check:", "OK" if not errors else f"{len(errors)} errors")
    for e in errors[:40]:
        print("  ", e)
    print("ERC:")
    for (sev, typ), c in sorted(counts.items()):
        print(f"  {sev:8} {typ:40} {c}")
