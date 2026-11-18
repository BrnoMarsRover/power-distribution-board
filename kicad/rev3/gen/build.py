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


TI = "https://www.ti.com/lit/ds/symlink/"
CUSTOM = {
    "TPS48111-Q1": box_symbol(
        "TPS48111-Q1",
        left=[("18", "CS+", "input"), ("17", "CS-", "input"),
              ("19", "ISCP", "input"), ("1", "EN/UVLO", "input"), ("3", "INP", "input"),
              ("2", "INP_G", "input"), ("10", "DIODE", "passive")],
        top=[("20", "VS", "power_in")], bottom=[("6", "GND", "power_in")],
        right=[("12", "BST", "passive"), ("15", "PU", "output"), ("14", "PD", "output"),
               ("13", "SRC", "passive"), ("11", "G", "output"), ("5", "FLT_I", "open_collector"),
               ("4", "FLT_T", "open_collector"), ("7", "IMON", "passive"), ("8", "IWRN", "passive"),
               ("9", "TMR", "passive")],
        footprint="PowerBoard_rev3:TI_DGX0019A_VSSOP-19",
        datasheet=TI + "tps4811-q1.pdf",
        desc="100 V smart high-side driver with pre-charge, current monitor and protection. "
             "DGX-19: pin 16 is absent for creepage."),
    "LM5122": box_symbol(
        "LM5122",
        left=[("5", "VIN", "power_in"), ("4", "CSP", "input"), ("3", "CSN", "input"),
              ("6", "UVLO", "input"), ("10", "FB", "input"), ("11", "COMP", "output"),
              ("7", "SS", "passive"), ("8", "SYNCIN/RT", "input"), ("12", "SLOPE", "passive"),
              ("14", "RES", "passive"), ("13", "MODE", "input"), ("2", "OPT", "input")],
        right=[("20", "BST", "passive"), ("19", "HO", "output"), ("18", "SW", "passive"),
               ("16", "LO", "output"), ("17", "VCC", "power_out"), ("1", "SYNCOUT", "output")],
        bottom=[("9", "AGND", "power_in"), ("15", "PGND", "power_in"), ("21", "EP", "passive")],
        footprint="Package_SO:HTSSOP-20-1EP_4.4x6.5mm_P0.65mm_EP3.4x6.5mm_Mask2.96x2.96mm",
        datasheet=TI + "lm5122.pdf", desc="Synchronous boost controller, 65 V in, HTSSOP-20 (MH)"),
    "LMR38010": box_symbol(
        "LMR38010",
        left=[("3", "VIN", "power_in"), ("2", "EN", "input"), ("4", "RT/SYNC", "passive"),
              ("5", "FB", "input")],
        right=[("7", "BOOT", "passive"), ("8", "SW", "power_out"), ("6", "PG", "open_collector")],
        bottom=[("1", "GND", "power_in"), ("9", "EP", "passive")],
        footprint="Package_SO:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.3mm",
        datasheet=TI + "lmr38010.pdf", desc="80 V 1 A synchronous buck, HSOIC-8 (DDA)"),
    "TCAN1044AV": box_symbol(
        "TCAN1044AV",
        left=[("1", "TXD", "input"), ("4", "RXD", "output"), ("8", "STB", "input")],
        right=[("7", "CANH", "bidirectional"), ("6", "CANL", "bidirectional")],
        top=[("3", "VCC", "power_in"), ("5", "VIO", "power_in")], bottom=[("2", "GND", "power_in")],
        footprint="Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
        datasheet=TI + "tcan1044a-q1.pdf", desc="CAN FD transceiver, +-58 V bus fault, VIO 1.8-5.5 V"),
    "Q_NMOS_GDS": box_symbol(
        "Q_NMOS_GDS", left=[("1", "G", "input")], right=[("2", "D", "passive"), ("3", "S", "passive")],
        ref="Q", desc="N-channel MOSFET, pin 1 gate, 2 drain (tab), 3 source"),
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
    def __init__(self, lib_id, prefix, value, footprint, conns, rot=0, fields=None, unit=1):
        self.lib_id, self.prefix, self.value, self.footprint = lib_id, prefix, value, footprint
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
            out.append(dict(p, sx=round(x, 3), sy=round(-y, 3), out=o))
        return out


# ------------------------------------------------------------------ sheets

POWER = {"GND": "power:GND", "+3V3": "power:+3V3"}
PAPERS = [("A4", 297, 210), ("A3", 420, 297), ("A2", 594, 420), ("A1", 841, 594), ("A0", 1189, 841)]


class Sheet:
    def __init__(self, name, title, notes=()):
        self.name, self.title, self.notes = name, title, list(notes)
        self.file = f"{name}.kicad_sch"
        self.parts, self.flags = [], []

    def add(self, *args, **kw):
        p = Part(*args, **kw)
        self.parts.append(p)
        return p

    def flag(self, net):
        self.flags.append(net)


def label_width(net):
    return len(net.split(":")[-1]) * 1.0 + 4.0


def is_local(net):
    return net.startswith("L:")


def net_key(sheet, net):
    return f"{sheet.name}/{net[2:]}" if is_local(net) else net


def layout(sheet):
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
               two_pin=False):
        self.lib_ids.add(lib_id)
        key = uid(self.s.name, "sym", ref)
        props = [("Reference", ref, hide_ref), ("Value", value, False), ("Footprint", footprint, True),
                 ("Datasheet", "", True), ("Description", "", True)]
        props += [(k, v, True) for k, v in (fields or {}).items()]
        s = (f'(symbol (lib_id {q(lib_id)}) (at {fmt(x)} {fmt(y)} {rot}) (unit {unit}) '
             f'(exclude_from_sim no) (in_bom {"no" if ref.startswith("#") else "yes"}) '
             f'(on_board {"no" if ref.startswith("#") else "yes"}) (dnp no) (uuid {q(key)})\n')
        sym = get_symbol(lib_id)
        th = math.radians(rot)
        for i, (k, v, hide) in enumerate(props):
            lib_at = next((find(pp, "at")[0] for pp in find(sym, "property") if unq(pp[1]) == k), None)
            just = ""
            if two_pin and rot == 90 and k in ("Reference", "Value"):
                px, py, ang = x, y + (-2.54 if k == "Reference" else 2.54), 90
            elif lib_at is not None and k in ("Reference", "Value"):
                lx, ly = float(lib_at[1]), float(lib_at[2])
                ang = float(lib_at[3]) if len(lib_at) > 3 else 0
                px = x + lx * math.cos(th) - ly * math.sin(th)
                py = y - (lx * math.sin(th) + ly * math.cos(th))
                lib_eff = next((find(pp, "effects")[0] for pp in find(sym, "property") if unq(pp[1]) == k), [])
                j = find(lib_eff, "justify")
                just = f" (justify {' '.join(j[0][1:])})" if j else ""
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
        self.symbol(p.lib_id, ref, p.value, X, Y, p.rot, [a["number"] for a in p.pins],
                    footprint=p.footprint, fields=fields, unit=p.unit, two_pin=len(p.pins) == 2)
        done = set()
        for a in p.geom():
            net = p.conns[a["number"]]
            INTENDED.append((ref, a["number"], None if net is None else net_key(self.s, net)))
            px, py = X + a["sx"], Y + a["sy"]
            if (px, py) in done:
                continue          # hidden pin stacked on a visible one
            done.add((px, py))
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

    def render(self, root_uuid, sheet_uuid, page_title):
        s = self.s
        layout(s)
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
    PTC2920="Fuse:Fuse_2920_7451Metric_Pad2.10x5.45mm_HandSolder",
    PTC1812="Fuse:Fuse_1812_4532Metric_Pad1.30x3.40mm_HandSolder",
    XT90M="PowerBoard_rev3:XT90PB-M", XT90F="PowerBoard_rev3:XT90PB-F",
    XT60M="Connector_AMASS:AMASS_XT60-M_1x02_P7.20mm_Vertical",
    XT60F="Connector_AMASS:AMASS_XT60-F_1x02_P7.20mm_Vertical",
    XT30M="Connector_AMASS:AMASS_XT30U-M_1x02_P5.0mm_Vertical",
    XT30F="Connector_AMASS:AMASS_XT30U-F_1x02_P5.0mm_Vertical",
    DEANS="PowerBoard_rev3:DEANS-T",
    XH2="Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical",
    PH2="Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical",
    GH4="Connector_JST:JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical",
    USBC="Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",
    BTN="Button_Switch_SMD:SW_SPST_TL3342",
    ESP="RF_Module:ESP32-S3-WROOM-1U",
    SSOP8="Package_SO:SSOP-8_2.95x2.8mm_P0.65mm",
    L12="Inductor_SMD:L_12x12mm_H6mm",
    L6="Inductor_SMD:L_6.3x6.3_H3",
    SOT223="Package_TO_SOT_SMD:SOT-223-3_TabPin2",
)


def fmt_r(v):
    return rails.fmt_r(v).replace("R", "")


def R(sh, value, a, b, fp="R0603", **f):
    v = fmt_r(value) if isinstance(value, (int, float)) else value
    return sh.add("Device:R", "R", v, FP[fp], {1: a, 2: b}, rot=90, fields=f)


def C(sh, value, a, b, fp="C0603", volt=None, **f):
    v = rails.fmt_c(value) if isinstance(value, (int, float)) else value
    if volt:
        v += f"/{volt}V"
    return sh.add("Device:C", "C", v, FP[fp], {1: a, 2: b}, rot=90, fields=f)


def CP(sh, value, plus, minus, fp="CP10", **f):
    return sh.add("Device:C_Polarized", "C", value, FP[fp], {1: plus, 2: minus}, rot=90, fields=f)


def DZ(sh, value, k, a, fp="SMB", **f):
    return sh.add("Device:D_Zener", "D", value, FP[fp], {1: k, 2: a}, fields=f)


def DS(sh, value, k, a, fp="SMA", **f):
    return sh.add("Device:D_Schottky", "D", value, FP[fp], {1: k, 2: a}, fields=f)


def LED(sh, color, k, a):
    return sh.add("Device:LED", "D", color, FP["LED"], {1: k, 2: a})


def TP(sh, net):
    return sh.add("Connector:TestPoint", "TP", net.split(":")[-1], FP["TP"], {1: net})


def FET(sh, part, g, d, s):
    fp = FP["D2PAK"] if part == "IPB042N10N3G" else FP["DPAK"]
    lcsc = rails.FETS[part][1]
    return sh.add("PowerBoard_rev3:Q_NMOS_GDS", "Q", part, fp, {1: g, 2: d, 3: s}, fields={"LCSC": lcsc})


def CONN2(sh, value, fp, p1, p2):
    return sh.add("Connector_Generic:Conn_01x02", "J", value, FP[fp], {1: p1, 2: p2})


# ------------------------------------------------------------------ sheet content

RAILNET = {"42V-D": "42D", "42V-C": "42C", "24V": "24", "12V": "12", "5V": "5", "54V": "54"}
VIN = {"42V-D": "BUS42", "42V-C": "BUS42", "24V": "VIN24", "12V": "VIN12", "5V": "VIN5", "54V": "BUS42"}
OUT = {"42V-D": "OUT42D", "42V-C": "OUT42C", "24V": "OUT24", "12V": "OUT12", "5V": "OUT5", "54V": "SW54_OUT"}
UVLO_ON = {"42V-D": 26, "42V-C": 26, "24V": 18, "12V": 9, "5V": 3.8, "54V": 26}
TVS_IN = {"24V": "SMBJ26A", "12V": "SMBJ13A", "5V": "SMBJ6.0A"}
CBULK_V = {"24V": 35, "12V": 25, "5V": 16}


def divider_bottom(top, v_full, v_adc=2.4):
    return rails.nearest(top * v_adc / (v_full - v_adc), rails.E96)


def sheet_power_in():
    sh = Sheet("power_in", "Battery input, bus protection and battery monitor", [
        "Battery 10S 30-42 V. The pack fuse (58 V rated) and BMS live in the harness, not on this board.",
        "R_SHUNT: Kelvin-route the INA228 sense lines from the shunt pads. Charger port is on the bus side of the shunt, so charging is counted.",
        "INA228 at 0x40 (A0 = A1 = GND): bus voltage, current, charge (Ah) and energy (Wh)."])
    CONN2(sh, "XT90 male (battery in)", "XT90M", "VBAT", "GND")
    DZ(sh, "SMCJ45A", "VBAT", "GND", fp="SMC")
    for _ in range(2):
        CP(sh, "100uF/100V", "VBAT", "GND")
    for _ in range(4):
        C(sh, 2.2e-6, "VBAT", "GND", fp="C1210", volt=100)
    R(sh, "0.5m 2512", "VBAT", "BUS42", fp="R2512", LCSC="C154666")
    R(sh, 10, "VBAT", "L:INA_P")
    R(sh, 10, "BUS42", "L:INA_N")
    C(sh, 100e-9, "L:INA_P", "L:INA_N")
    sh.add("Sensor_Energy:INA228", "U", "INA228", "Package_SO:TSSOP-10_3x3mm_P0.5mm",
           {1: "GND", 2: "GND", 3: "INA_ALERT", 4: "I2C_SDA", 5: "I2C_SCL", 6: "+3V3", 7: "GND",
            8: "BUS42", 9: "L:INA_N", 10: "L:INA_P"}, fields={"LCSC": "C2887910"})
    C(sh, 100e-9, "+3V3", "GND")
    R(sh, 10e3, "+3V3", "INA_ALERT")
    CONN2(sh, "XT60 male (charger)", "XT60M", "BUS42", "GND")
    TP(sh, "VBAT")
    TP(sh, "BUS42")
    for n in ("VBAT", "BUS42", "GND"):
        sh.flag(n)
    return sh


def sheet_switch(r):
    v = rails.rail_values(r)
    n = RAILNET[r["name"]]
    vin, out = VIN[r["name"]], OUT[r["name"]]
    on = r["default_on"]
    title = {"42V-D": "42 V drive group eFuse (4 x XT90, EPOS4)",
             "42V-C": "42 V converter-feed group eFuse (4 x XT90)",
             "24V": "24 V rail eFuse", "12V": "12 V rail eFuse (feeds the Orin)", "5V": "5 V rail eFuse",
             "54V": "54 V supply eFuse (input of the LM5122 boost)"}[r["name"]]
    notes = [f"TPS48111-Q1 + {v['n']} x {r['fet']} main + 1 pre-charge. Values from docs/rev3/rails.md "
             f"(trip {v['i_oc']:.1f} A after {v['c_tmr'] * v['k_tmr'] * 1e3:.0f} ms, latch-off; "
             + (f"short-circuit {v['i_sc']:.0f} A)." if v["r_iscp"] else "short-circuit trip disabled)."),
             f"Power-on default: {'ON (INP delayed by RC after INP_G)' if on else 'OFF'}. "
             "MMBT3904 must sit against the main FET (remote temperature sense).",
             "Kelvin-route CS+ / CS- / ISCP from the R_SNS pads. Place C_ISCP right at the pins."]
    if r["name"] == "42V-D":
        notes.append("E-STOP: ESTOP_KILL (from the latch in the io sheet) pulls EN/UVLO low through Q_EN.")
    sh = Sheet(f"sw_{n.lower()}", title, notes)
    en = "L:EN"
    sh.add("PowerBoard_rev3:TPS48111-Q1", "U", "TPS48111-Q1", CUSTOM_FP_TPS,
           {20: vin, 18: "L:CSP", 17: "L:DRN", 19: "L:ISCP" if v["r_iscp"] else "L:DRN", 1: en,
            3: f"INP_{n}", 2: f"INPG_{n}", 10: "L:DIODE", 6: "GND",
            12: "L:BST", 15: "L:PU", 14: "L:GATE", 13: out, 11: "L:GPRE",
            5: f"FLT_{n}", 4: f"FLT_{n}", 7: f"IMON_{n}", 8: "L:IWRN", 9: "L:TMR"},
           fields={"LCSC": "C17558994"})
    shunt_lcsc = {0.5e-3: "C154666", 1e-3: "C154668", 2e-3: "C160590", 5e-3: "C185817"}[r["r_sns"]]
    R(sh, f"{r['r_sns'] * 1e3:g}m 2512", vin, "L:DRN", fp="R2512", LCSC=shunt_lcsc)
    R(sh, v["r_set"], vin, "L:CSP")
    if v["r_iscp"]:
        R(sh, v["r_iscp"], vin, "L:ISCP")
        C(sh, 1e-9, "L:ISCP", "L:DRN")
    C(sh, 100e-9, vin, "GND", volt=100)
    c_vs = C(sh, 1e-6, vin, "GND", fp="C1206", volt=100)
    r_top = rails.nearest(10e3 * (UVLO_ON[r["name"]] / 1.18 - 1), rails.E96)
    R(sh, r_top, vin, en)
    R(sh, 10e3, en, "GND")
    if r["name"] == "42V-D":
        sh.add("Transistor_FET:2N7002", "Q", "2N7002", FP["SOT23"], {1: "ESTOP_KILL", 2: "GND", 3: en})
    if on:
        R(sh, 1e6, "+3V3", f"INP_{n}")
        C(sh, 1e-6, f"INP_{n}", "GND")
        R(sh, 100e3, "+3V3", f"INPG_{n}")
    else:
        R(sh, 100e3, f"INP_{n}", "GND")
        R(sh, 100e3, f"INPG_{n}", "GND")
    R(sh, 10e3, "+3V3", f"FLT_{n}")
    R(sh, v["r_imon"], f"IMON_{n}", "GND")
    C(sh, 10e-9, f"IMON_{n}", "GND")
    R(sh, v["r_iwrn"], "L:IWRN", "GND")
    C(sh, v["c_tmr"], "L:TMR", "GND", fp="C1206", volt=10)
    R(sh, v["r_tmr"], "L:TMR", "GND")
    sh.add("Transistor_BJT:MMBT3904", "Q", "MMBT3904", FP["SOT23"], {1: "L:DIODE", 2: "GND", 3: "L:DIODE"})
    R(sh, 10, "L:PU", "L:GATE")
    C(sh, v["c_bst"], "L:BST", out, fp="C0805", volt=25)
    for i in range(v["n"]):
        R(sh, 2.2, "L:GATE", f"L:G{i + 1}")
        FET(sh, r["fet"], f"L:G{i + 1}", "L:DRN", out)
    FET(sh, r["fet"], "L:GPRE", "L:DRN", out)
    R(sh, 330, "L:GPRE", "L:CG")
    C(sh, v["c_g"], "L:CG", "GND", volt=50)
    if r["name"] in TVS_IN:
        DZ(sh, TVS_IN[r["name"]], vin, "GND")
        for _ in range(2):
            CP(sh, f"47uF/{CBULK_V[r['name']]}V", vin, "GND", fp="CP8")
        rt = 100e3
        R(sh, rt, vin, f"VSNS_IN_{n}")
        R(sh, divider_bottom(rt, r["v"] * 1.25), f"VSNS_IN_{n}", "GND")
        C(sh, 10e-9, f"VSNS_IN_{n}", "GND")
    if r["name"] != "54V":       # the 54 V output voltage is measured after the boost
        R(sh, 100e3, out, f"VSNS_OUT_{n}")
        R(sh, divider_bottom(100e3, r["v"] * 1.15), f"VSNS_OUT_{n}", "GND")
        C(sh, 10e-9, f"VSNS_OUT_{n}", "GND")
    r_led = rails.nearest((r["v"] - 2.0) / 2e-3, rails.E96)
    R(sh, r_led, out, "L:LED", fp="R1206" if r["v"] > 20 else "R0603")
    LED(sh, "green", "GND", "L:LED")
    for net in (out, f"IMON_{n}", f"FLT_{n}", "L:GATE"):
        TP(sh, net)
    sh.flag(out)
    return sh


CUSTOM_FP_TPS = "PowerBoard_rev3:TI_DGX0019A_VSSOP-19"


def sheet_outputs():
    sh = Sheet("outputs", "Rail inputs, output connectors and per-output PTCs", [
        "Male connectors are inputs (the cable side is the source), female are outputs. Pin 1 = +.",
        "Check every connector footprint's pin 1 against the housing's + marking before ordering.",
        "PTC per 24 / 12 / 5 V output: 3 A hold. Device cables must carry the PTC trip current (AWG 20 or heavier)."])
    CONN2(sh, "Deans T male (24 V in)", "DEANS", "VIN24", "GND")
    CONN2(sh, "XT60 male (12 V in)", "XT60M", "VIN12", "GND")
    CONN2(sh, "XT30 male (5 V in)", "XT30M", "VIN5", "GND")
    for i in range(4):
        CONN2(sh, f"XT90 female (drives {i + 1})", "XT90F", "OUT42D", "GND")
    for i in range(4):
        CONN2(sh, f"XT90 female (converter feed {i + 1})", "XT90F", "OUT42C", "GND")
    ptc = {"24": ("SMD2920P300TF/33", "PTC2920", "C2943634", 8, "DEANS", "Deans T female"),
           "12": ("SMD2920P300TF/16", "PTC2920", "C167318", 7, "XT60F", "XT60 female"),
           "5": ("SMD1812P300TF", "PTC1812", "C21004", 11, "XT30F", "XT30 female")}
    for n, (part, fp, lcsc, count, cfp, cname) in ptc.items():
        for i in range(count):
            sh.add("Device:Polyfuse", "F", part, FP[fp], {1: f"OUT{n}", 2: f"L:O{n}_{i + 1}"}, rot=90,
                   fields={"LCSC": lcsc})
            CONN2(sh, f"{cname} ({n} V out {i + 1})", cfp, f"L:O{n}_{i + 1}", "GND")
    CONN2(sh, "JST-XH 2p (54 V to Zyxel XMG-105HP)", "XH2", "OUT54", "GND")
    for n in ("VIN24", "VIN12", "VIN5"):
        sh.flag(n)
    return sh


def sheet_boost():
    b = rails.BOOST
    sh = Sheet("boost54", "54 V boost for the Zyxel XMG-105HP (LM5122)", [
        "30-42 V in (from the 54 V supply eFuse) -> 54.35 V, 100 W, 400 kHz. Values from docs/rev3/rails.md.",
        "VERIFY before layout: R_SLOPE, R_COMP / C_COMP / C_HF are starting values from the LM5122 datasheet "
        "example. Run TI's LM5122 quickstart calculator with these operating points.",
        "Keep the SW node, Q_LS, Q_HS and the output ceramics in one tight loop. EP must be soldered (hand-solder hole)."])
    sh.add("PowerBoard_rev3:LM5122", "U", "LM5122MH",
           "Package_SO:HTSSOP-20-1EP_4.4x6.5mm_P0.65mm_EP3.4x6.5mm_Mask2.96x2.96mm",
           {5: "L:VIN_LM", 4: "L:CSP", 3: "L:CSN", 6: "L:UVLO", 10: "L:FB", 11: "L:COMP", 7: "L:SS",
            8: "L:RT", 12: "L:SLOPE", 14: "L:RES", 13: "GND", 2: "GND",
            20: "L:BST", 19: "L:HO", 18: "L:SW", 16: "L:LO", 17: "L:VCC", 1: None,
            9: "GND", 15: "GND", 21: "GND"}, fields={"LCSC": "C77241"})
    R(sh, 3, "SW54_OUT", "L:VIN_LM")
    C(sh, 1e-6, "L:VIN_LM", "GND", fp="C1206", volt=100)
    R(sh, "10m 2512", "SW54_OUT", "L:LIN", fp="R2512", LCSC="C163051")
    R(sh, 100, "SW54_OUT", "L:CSP")
    R(sh, 100, "L:LIN", "L:CSN")
    C(sh, 1e-9, "L:CSP", "L:CSN")
    for _ in range(4):
        C(sh, 2.2e-6, "SW54_OUT", "GND", fp="C1210", volt=100)
    sh.add("Device:L", "L", "22uH 10A (TMPA1265SP-220MN-D)", FP["L12"], {1: "L:LIN", 2: "L:SW"},
           rot=90, fields={"LCSC": "C357119"})
    FET(sh, b["fet"], "L:LO", "L:SW", "GND")
    FET(sh, b["fet"], "L:HO", "OUT54", "L:SW")
    C(sh, 100e-9, "L:BST", "L:SW", volt=25)
    DS(sh, "1N4148W (100 V)", "L:BST", "L:VCC", fp="SOD123")
    C(sh, 4.7e-6, "L:VCC", "GND", fp="C0805", volt=16)
    R(sh, 200e3, "SW54_OUT", "L:UVLO")
    R(sh, 9.31e3, "L:UVLO", "GND")
    C(sh, 100e-9, "L:SS", "GND")
    C(sh, 470e-9, "L:RES", "GND")
    R(sh, rails.nearest(9e9 / b["fsw"], rails.E96), "L:RT", "GND")
    R(sh, 100e3, "L:SLOPE", "GND")
    R(sh, 20e3, "L:COMP", "L:CC")
    C(sh, 22e-9, "L:CC", "GND")
    C(sh, 330e-12, "L:COMP", "GND")
    R(sh, 221e3, "OUT54", "L:FB")
    R(sh, 4.99e3, "L:FB", "GND")
    for _ in range(6):
        C(sh, 2.2e-6, "OUT54", "GND", fp="C1210", volt=100)
    for _ in range(2):
        CP(sh, "100uF/80V", "OUT54", "GND")
    DZ(sh, "SMBJ58A", "OUT54", "GND")
    R(sh, 100e3, "OUT54", "VSNS_OUT_54")
    R(sh, divider_bottom(100e3, 62), "VSNS_OUT_54", "GND")
    C(sh, 10e-9, "VSNS_OUT_54", "GND")
    R(sh, 26.1e3, "OUT54", "L:LED", fp="R1206")
    LED(sh, "green", "GND", "L:LED")
    TP(sh, "OUT54")
    TP(sh, "L:VCC")
    sh.flag("OUT54")
    sh.flag("L:VIN_LM")
    return sh


def sheet_aux():
    sh = Sheet("aux_power", "Always-on aux supply: 42 V -> 5.3 V -> 3.3 V", [
        "LMR38010 runs from the battery bus before any switch, so logic power never depends on USB (rule R1).",
        "Bring-up test: battery only, USB unplugged -> +5V3_AUX, +5V_LOGIC and +3V3 present, ESP32 boots.",
        "VERIFY the LMR38010 variant has the RT/SYNC pin (adjustable frequency); RT = 49.9k gives ~525 kHz."])
    sh.add("PowerBoard_rev3:LMR38010", "U", "LMR38010SDDAR",
           "Package_SO:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.3mm",
           {3: "BUS42", 2: "L:EN", 4: "L:RT", 5: "L:FB", 7: "L:BOOT", 8: "L:SW", 6: "L:PG",
            1: "GND", 9: "GND"}, fields={"LCSC": "C5219310"})
    for _ in range(2):
        C(sh, 2.2e-6, "BUS42", "GND", fp="C1210", volt=100)
    R(sh, 470e3, "BUS42", "L:EN")
    R(sh, 30.1e3, "L:EN", "GND")
    R(sh, 49.9e3, "L:RT", "GND")
    C(sh, 100e-9, "L:BOOT", "L:SW", volt=16)
    sh.add("Device:L", "L", "33uH 1.5A", FP["L6"], {1: "L:SW", 2: "+5V3_AUX"}, rot=90)
    R(sh, 100e3, "+5V3_AUX", "L:FB")
    R(sh, 23.2e3, "L:FB", "GND")
    for _ in range(2):
        C(sh, 22e-6, "+5V3_AUX", "GND", fp="C1206", volt=10)
    R(sh, 100e3, "+5V3_AUX", "L:PG")
    DS(sh, "SS34", "+5V_LOGIC", "+5V3_AUX")
    DS(sh, "SS34", "+5V_LOGIC", "VBUS_USB")
    C(sh, 22e-6, "+5V_LOGIC", "GND", fp="C1206", volt=10)
    sh.add("Regulator_Linear:AP7361C-33E", "U", "AP7361C-33E", FP["SOT223"],
           {1: "+5V_LOGIC", 2: "GND", 3: "+3V3"})
    C(sh, 10e-6, "+3V3", "GND", fp="C0805", volt=10)
    C(sh, 100e-9, "+3V3", "GND")
    for n in ("+5V3_AUX", "+5V_LOGIC", "+3V3", "L:PG"):
        TP(sh, n)
    for n in ("+5V3_AUX", "+5V_LOGIC"):
        sh.flag(n)
    return sh


ESP_PINS = {"1": "GND", "2": "+3V3", "3": "ESP_EN", "4": "I2C_SDA", "5": "I2C_SCL", "6": "FLT_42D",
            "7": "FLT_42C", "8": "UART_SP", "9": "FLT_5", "10": "UART_TX", "11": "UART_RX", "12": "FLT_24",
            "13": "USB_DN", "14": "USB_DP", "15": None, "16": None, "17": "FLT_12", "18": "INA_ALERT",
            "19": "CAN_TXD", "20": "CAN_RXD", "21": "CAN_STB", "22": "ESTOP_OK", "23": "LED_1",
            "24": "LED_2", "25": "LED_3", "26": None, "27": "ESP_IO0", "28": None, "29": None, "30": None,
            "31": "FLT_54", "32": None, "33": None, "34": None, "35": None, "36": "L:RXD0",
            "37": "L:TXD0", "38": "AUX5_SNS", "39": "NTC_SNS"}


def sheet_mcu():
    sh = Sheet("mcu", "ESP32-S3-WROOM-1U, USB, host UART, LEDs", [
        "Pin map from docs/rev3/design.md section 6.6. Strapping pins IO0/3/45/46 are not used for I/O.",
        "Host UART1 (IO17 TX, IO18 RX) goes to the Jetson AGX Orin 40-pin header (pins 10 / 8, GND pin 6).",
        "USB-C is for flashing and debug only (built-in USB-Serial/JTAG). VBUS only feeds the logic OR in aux_power.",
        "N8 module variant (no octal PSRAM). u.FL antenna: route the cable out of the rover body."])
    sh.add("RF_Module:ESP32-S3-WROOM-1", "U", "ESP32-S3-WROOM-1U-N8", FP["ESP"], ESP_PINS,
           fields={"LCSC": "C2980297"})
    C(sh, 22e-6, "+3V3", "GND", fp="C1206", volt=10)
    C(sh, 100e-9, "+3V3", "GND")
    R(sh, 10e3, "+3V3", "ESP_EN")
    C(sh, 1e-6, "ESP_EN", "GND")
    sh.add("Switch:SW_Push", "SW", "RESET", FP["BTN"], {1: "ESP_EN", 2: "GND"})
    R(sh, 10e3, "+3V3", "ESP_IO0")
    sh.add("Switch:SW_Push", "SW", "BOOT", FP["BTN"], {1: "ESP_IO0", 2: "GND"})
    R(sh, 4.7e3, "+3V3", "I2C_SDA")
    R(sh, 4.7e3, "+3V3", "I2C_SCL")
    sh.add("Connector:USB_C_Receptacle_USB2.0_16P", "J", "USB-C (flash/debug)", FP["USBC"],
           {"A1": "GND", "A4": "VBUS_USB", "A5": "L:CC1", "B5": "L:CC2", "A6": "USB_DP", "B6": "USB_DP",
            "A7": "USB_DN", "B7": "USB_DN", "A8": None, "B8": None, "SH": "GND"})
    R(sh, 5.1e3, "L:CC1", "GND")
    R(sh, 5.1e3, "L:CC2", "GND")
    sh.add("Power_Protection:USBLC6-2SC6", "U", "USBLC6-2SC6", FP["SOT23"].replace("SOT-23", "SOT-23-6"),
           {1: "USB_DP", 6: "USB_DP", 3: "USB_DN", 4: "USB_DN", 5: "VBUS_USB", 2: "GND"})
    sh.add("Connector_Generic:Conn_01x04", "J", "JST-GH 4p (host UART to Orin)", FP["GH4"],
           {1: "L:TX_C", 2: "L:RX_C", 3: "GND", 4: "L:SP_C"})
    R(sh, 100, "UART_TX", "L:TX_C")
    R(sh, 100, "L:RX_C", "UART_RX")
    R(sh, 100, "UART_SP", "L:SP_C")
    sh.add("Power_Protection:USBLC6-2SC6", "U", "USBLC6-2SC6", FP["SOT23"].replace("SOT-23", "SOT-23-6"),
           {1: "L:TX_C", 6: "L:TX_C", 3: "L:RX_C", 4: "L:RX_C", 5: "+3V3", 2: "GND"})
    for i in (1, 2, 3):
        R(sh, 1e3, f"LED_{i}", f"L:LED{i}")
        LED(sh, ["green", "yellow", "red"][i - 1], "GND", f"L:LED{i}")
    sh.add("Device:Thermistor_NTC", "TH", "10k NTC", FP["R0603"], {1: "+3V3", 2: "NTC_SNS"}, rot=90)
    R(sh, 10e3, "NTC_SNS", "GND")
    C(sh, 100e-9, "NTC_SNS", "GND")
    R(sh, 10e3, "+5V_LOGIC", "AUX5_SNS")
    R(sh, 10e3, "AUX5_SNS", "GND")
    for n in ("L:TXD0", "L:RXD0", "ESP_EN", "ESP_IO0"):
        TP(sh, n)
    sh.flag("VBUS_USB")
    return sh


def sheet_can():
    sh = Sheet("can", "CAN bus to the EPOS4 drives (TCAN1044AV)", [
        "Bus pins survive +-58 V faults internally. No external TVS: a 24 V CAN TVS would conduct if a CAN wire touched 42 V.",
        "Split termination enabled by closing both solder jumpers (only at a bus end).",
        "Connector type to match the EPOS4 CAN harness (JST-GH 4p placeholder: CANH, CANL, GND, NC)."])
    sh.add("PowerBoard_rev3:TCAN1044AV", "U", "TCAN1044AVDRQ1", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
           {1: "CAN_TXD", 4: "CAN_RXD", 8: "CAN_STB", 3: "+5V_LOGIC", 5: "+3V3",
            7: "CANH", 6: "CANL", 2: "GND"}, fields={"LCSC": "C3234993"})
    C(sh, 100e-9, "+5V_LOGIC", "GND")
    C(sh, 100e-9, "+3V3", "GND")
    sh.add("Jumper:SolderJumper_2_Open", "JP", "TERM_H", "Jumper:SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm",
           {1: "CANH", 2: "L:TH"})
    sh.add("Jumper:SolderJumper_2_Open", "JP", "TERM_L", "Jumper:SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm",
           {1: "CANL", 2: "L:TL"})
    R(sh, 60.4, "L:TH", "L:TMID", fp="R0805")
    R(sh, 60.4, "L:TMID", "L:TL", fp="R0805")
    C(sh, 4.7e-9, "L:TMID", "GND", volt=50)
    for i in (1, 2):
        sh.add("Connector_Generic:Conn_01x04", "J", f"CAN {i} (JST-GH 4p)", FP["GH4"],
               {1: "CANH", 2: "CANL", 3: "GND", 4: None})
    return sh


ADC_MAP = [["IMON_42D", "IMON_42C", "IMON_24", "IMON_12", "IMON_5", "IMON_54", "VSNS_OUT_42D", "VSNS_OUT_42C"],
           ["VSNS_OUT_24", "VSNS_OUT_12", "VSNS_OUT_5", "VSNS_OUT_54", "VSNS_IN_24", "VSNS_IN_12", "VSNS_IN_5", "GND"]]
EXP_MAP = {4: "INP_42D", 5: "INP_42C", 6: "INP_24", 7: "INP_12", 8: "INP_5", 9: "INP_54", 10: "INPG_42D",
           11: "INPG_42C", 13: "INPG_24", 14: "INPG_12", 15: "INPG_5", 16: "INPG_54", 17: "ESTOP_ARM",
           18: None, 19: None, 20: None}


def sheet_io():
    sh = Sheet("io", "Switch control (TCA9535), measurement (ADC128D818 x2), E-STOP latch", [
        "TCA9535 at 0x20 holds INP / INP_G for all six switches, so they keep their state across an ESP32 reset.",
        "ADC128D818 at 0x1D (A1 = A0 = LOW) and 0x1E (A1 = LOW, A0 = MID). Internal 2.56 V reference; 12 ms per channel.",
        "E-STOP: opening the loop clears the latch (Q low, /Q high) -> ESTOP_KILL holds the drive group off. "
        "It stays off after the loop closes until the ESP32 clocks ESTOP_ARM. Power-on also starts latched off."])
    exp = {1: None, 2: "GND", 3: "GND", 21: "GND", 22: "I2C_SCL", 23: "I2C_SDA", 24: "+3V3", 12: "GND"}
    exp.update(EXP_MAP)
    sh.add("Interface_Expansion:TCA9535PWR", "U", "TCA9535PWR", "Package_SO:TSSOP-24_4.4x7.8mm_P0.65mm",
           exp, fields={"LCSC": "C130204"})
    C(sh, 100e-9, "+3V3", "GND")
    for i, chans in enumerate(ADC_MAP):
        conns = {1: None, 2: "I2C_SDA", 3: "I2C_SCL", 4: "GND", 5: "+3V3", 6: None,
                 7: "GND" if i == 0 else "L:A0MID", 8: "GND"}
        for k, net in enumerate(chans):
            conns[16 - k] = net
        sh.add("Analog_ADC:ADC128D818", "U", "ADC128D818", "Package_SO:TSSOP-16_4.4x5mm_P0.65mm",
               conns, fields={"LCSC": "C139062"})
        C(sh, 100e-9, "+3V3", "GND")
    R(sh, 10e3, "+3V3", "L:A0MID")
    R(sh, 10e3, "L:A0MID", "GND")
    sh.add("Connector_Generic:Conn_01x02", "J", "E-STOP loop (JST-PH 2p, NC contact)", FP["PH2"],
           {1: "L:ESRC", 2: "L:ELOOP"})
    R(sh, 1e3, "+3V3", "L:ESRC")
    R(sh, 10e3, "L:ELOOP", "GND")
    R(sh, 1e3, "L:ELOOP", "ESTOP_OK")
    C(sh, 100e-9, "ESTOP_OK", "GND")
    R(sh, 1e6, "ESTOP_OK", "L:CLR")
    C(sh, 100e-9, "L:CLR", "GND")
    DS(sh, "1N5819HW", "ESTOP_OK", "L:CLR", fp="SOD123")
    R(sh, 100e3, "ESTOP_ARM", "GND")
    sh.add("74xGxx:74AUP1G74", "U", "74LVC1G74DCTR", FP["SSOP8"],
           {1: "ESTOP_ARM", 2: "+3V3", 3: "ESTOP_KILL", 4: "GND", 5: "L:LATCH_Q", 6: "L:CLR",
            7: "+3V3", 8: "+3V3"})
    C(sh, 100e-9, "+3V3", "GND")
    TP(sh, "L:LATCH_Q")
    TP(sh, "ESTOP_OK")
    return sh


# ------------------------------------------------------------------ root, project, checks

def build():
    write_custom_lib()
    write_custom_footprints()
    sheets = [sheet_power_in()] + [sheet_switch(r) for r in rails.RAILS] + \
             [sheet_outputs(), sheet_boost(), sheet_aux(), sheet_mcu(), sheet_can(), sheet_io()]
    root_uuid = uid("root")
    root_items = []
    x, y = 30.0, 60.0
    for i, s in enumerate(sheets):
        su = uid("sheet", s.name)
        page = str(i + 2)
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
    rev2 = REPO / "kicad/ext/SamacSys_Parts.pretty"
    for src, dst in (("XT90", "XT90PB-M"), ("XT90", "XT90PB-F"), ("DEANS-T", "DEANS-T")):
        text = (rev2 / f"{src}.kicad_mod").read_text()
        text = re.sub(r'^\((footprint|module) "[^"]*"', rf'(\1 "{dst}"', text, count=1)
        text = re.sub(r'\n\s*\(model [^\n]*(?:\n\s+[^\n]*)*?\n\s*\)', '', text)   # drop rev2 3D paths
        (lib / f"{dst}.kicad_mod").write_text(text)


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
