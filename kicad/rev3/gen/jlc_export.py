#!/usr/bin/env python3
"""JLCPCB order files for the assembled board (decided 2026-11-26: JLC assembles both sides, through-hole included;
2 oz outer, 0.5 oz inner copper). Writes kicad/rev3/fab/jlc/:

  gerbers.zip               Gerbers (4 copper, mask, paste, silkscreen, outline) + Excellon drill (PTH / NPTH)
  bom_jlc.csv               Comment, Designator, Footprint, LCSC Part #   (one board's quantities)
  cpl_jlc.csv               Designator, Mid X, Mid Y, Layer, Rotation      (every BOM part, both sides)
  sourcing.md               what is substituted, global-sourced, or left out

LCSC numbers: gen/jlc_parts.json (looked up on jlcpcb.com by MPN, 2026-11-26), overridden by SUBSTITUTE where the
looked-up code had too little stock (same part under another code, or an equivalent: same value, package and
rating). JLC places at least its minimum quantity of a part (often 20) whatever the board count, so a line needs
max(minimum, boards x per-board quantity) in stock.
Left out of the assembly: parts with MPN "none: ..." (PCB pads, solder jumpers, wire bridges) and the Deans T
connectors (no LCSC stock; fitted by hand). Rotations follow KiCad's footprint orientation: check them in JLC's
placement preview before paying.

    python3 kicad/rev3/gen/jlc_export.py
"""
import csv, json, re, subprocess, sys, zipfile
from pathlib import Path

REV = Path(__file__).resolve().parent.parent
BOARD, SCH = REV / "PowerBoard_rev3.kicad_pcb", REV / "PowerBoard_rev3.kicad_sch"
OUT = REV / "fab" / "jlc"
PARTS = json.loads((Path(__file__).resolve().parent / "jlc_parts.json").read_text())

# MPN -> (LCSC code, ordered part, why)
SUBSTITUTE = {
    "IPB042N10N3GATMA1": ("C69300", "IPB042N10N3G", "same part, the stocked code"),
    "XT90PB-M": ("C3040688", "XT90PB-M.G.Y", "same connector, gold-plated (Amass)"),
    "XT90PB-F": ("C19268045", "XT90PB-F.G.Y", "same connector, gold-plated (Amass)"),
    "XT30UPB-F": ("C108769", "XT30UPB-F30.G.Y", "same connector, gold-plated female (Amass)"),
    "AQY282S": ("C250827", "AQY282SX", "same part, tape and reel"),
    "EEU-FC1V470": ("C88836", "Rubycon 35YXF47MEFC6.3X11", "47 uF 35 V low-impedance, same 6.3 x 11 mm, 2.5 mm pitch"),
    "EEU-FC2A101": ("C5110675", "Rubycon 100ZLJ100M12.5X16", "100 uF 100 V low-impedance (FC class), 12.5 mm, 5 mm pitch"),
    "SMD2920B300TF/30": ("C22446884", "2920L300/30GR (LUTE)", "2920, 3 A hold / 5 A trip, 30 V, 12-48 mOhm (24 V outputs)"),
    "MF-LSMF300/16X-2": ("C719172", "Bourns MF-LSMF300/24X-2", "same Bourns family, 3 A hold, 24 V rated (12 V outputs)"),
    "MF-LSMF300X-2": ("C207090", "Littelfuse 2920L300DR", "2920, 3 A hold / 5 A trip, 6 V, 15-48 mOhm (5 V outputs)"),
    "C1206C154K5RACTU": ("C282843", "TCC1206X7R154K500DT", "150 nF 50 V X7R 1206"),
    "C1206C274K5RACTU": ("C577243", "Yageo CC1206KRX7R9BB274", "270 nF 50 V X7R 1206"),
    "C3216X7R2A684K160AA": ("C2976457", "Walsin 1206B684K101CT", "680 nF 100 V X7R 1206"),
    "GRM31CR72A105KA01L": ("C13832", "Samsung CL31B105KCHNNNE", "1 uF 100 V X7R 1206"),
    "ERJ-3RQF3R0V": ("C137724", "Yageo RC0603FR-073RL", "3.0 Ohm 1 % 0603"),
    "RC0603FR-1371K5L": ("C185312", "Yageo RC0603FR-0771K5L", "71.5 k 1 % 0603 (other reel size)"),
    "RC0603FR-1339K2L": ("C23037", "UniOhm 0603WAF3922T5E", "39.2 k 1 % 0603"),
    # stocked, but under JLC's minimum placement quantity (20 for these passives, whatever the board count)
    "C0603C820J5GACTU": ("C1683", "FH 0603CG820J500NT", "82 pF 50 V C0G 5 % 0603"),
    "RC0603FR-0762KL": ("C23221", "UniOhm 0603WAF6202T5E", "62 k 1 % 0603"),
    "ERJ-6ENF60R4V": ("C72998", "UniOhm 0805W8F604JT5E", "60.4 Ohm 1 % 0805"),
}
GLOBAL = {"MSS1583-223MED": "boost inductor L1 (Coilcraft): no stock",
          "CSS2H-2512R-L500F": "input shunt R5 (Bourns 0.5 mOhm Kelvin): 1 in stock, 2 needed",
          "TL3342F160QG": "BOOT / RESET buttons SW1, SW2 (E-Switch): 1 in stock, 4 needed"}


def refs(s):
    out = []
    for part in s.split(","):
        part = part.strip()
        m = re.fullmatch(r"([A-Z]+)(\d+)-\1?(\d+)", part)
        if m:
            out += [f"{m.group(1)}{i}" for i in range(int(m.group(2)), int(m.group(3)) + 1)]
        elif part:
            out.append(part)
    return out


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    tmp = OUT / "tmp"
    tmp.mkdir(exist_ok=True)
    run = lambda *a: subprocess.run(a, check=True, capture_output=True, text=True)
    # Gerbers + drill
    for f in tmp.glob("*"):
        f.unlink()
    run("kicad-cli", "pcb", "export", "gerbers", "--layers",
        "F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts",
        "--no-protel-ext", "-o", str(tmp) + "/", str(BOARD))
    run("kicad-cli", "pcb", "export", "drill", "--format", "excellon", "--excellon-units", "mm",
        "--excellon-separate-th", "--generate-map", "--map-format", "gerberx2", "-o", str(tmp) + "/", str(BOARD))
    with zipfile.ZipFile(OUT / "gerbers.zip", "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(tmp.glob("*")):
            z.write(f, f.name)
    # BOM
    run("kicad-cli", "sch", "export", "bom", "--fields", "MPN,Value,Footprint,${QUANTITY},Reference",
        "--labels", "MPN,Value,Footprint,Qty,References", "--group-by", "MPN,Footprint", "--sort-field", "MPN",
        "-o", str(tmp / "bom.csv"), str(SCH))
    by_mpn = {p["MPN"]: p for p in PARTS}
    bom, left, subs, glob_ = [], [], [], []
    for r in csv.DictReader((tmp / "bom.csv").open()):
        mpn = r["MPN"].strip()
        if not mpn or mpn.startswith("none"):
            left.append((r["Value"], r["References"], mpn))
            continue
        if mpn in SUBSTITUTE:
            code, part, why = SUBSTITUTE[mpn]
            subs.append((mpn, part, code, why, r["References"]))
        else:
            code = by_mpn.get(mpn, {}).get("LCSC", "")
        if mpn in GLOBAL:
            glob_.append((mpn, code, GLOBAL[mpn], r["References"]))
        if not code:
            sys.exit(f"no LCSC code for {mpn}")
        bom.append({"Comment": f'{r["Value"]} ({mpn})', "Designator": ",".join(refs(r["References"])),
                    "Footprint": r["Footprint"].split(":")[-1], "LCSC Part #": code})
    with (OUT / "bom_jlc.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["Comment", "Designator", "Footprint", "LCSC Part #"])
        w.writeheader()
        w.writerows(bom)
    # CPL: every BOM designator, from KiCad's position export (board origin as in the Gerbers)
    run("kicad-cli", "pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "both",
        "-o", str(tmp / "pos.csv"), str(BOARD))
    want = {d for b in bom for d in b["Designator"].split(",")}
    cpl, found = [], set()
    for r in csv.DictReader((tmp / "pos.csv").open()):
        if r["Ref"] in want:
            found.add(r["Ref"])
            cpl.append({"Designator": r["Ref"], "Mid X": f'{float(r["PosX"]):.4f}mm', "Mid Y": f'{float(r["PosY"]):.4f}mm',
                        "Layer": "Top" if r["Side"].lower() == "top" else "Bottom", "Rotation": f'{float(r["Rot"]):.1f}'})
    with (OUT / "cpl_jlc.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        w.writeheader()
        w.writerows(cpl)
    missing = sorted(want - found)
    # notes
    lines = ["# JLC order: sourcing notes (generated by gen/jlc_export.py)", "",
             f"{len(bom)} BOM lines, {len(want)} placed parts per board ({sum(1 for c in cpl if c['Layer'] == 'Top')} top, "
             f"{sum(1 for c in cpl if c['Layer'] == 'Bottom')} bottom).", "",
             "## Substituted (no stock under the original code)", "", "| Original MPN | Ordered | LCSC | Why | Parts |", "|---|---|---|---|---|"]
    lines += [f"| {a} | {b} | {c} | {d} | {e} |" for a, b, c, d, e in subs]
    lines += ["", "## Global sourcing at JLC (short of stock; choose \"Global Sourcing\" for these in the BOM step)", "",
              "| MPN | LCSC | Why | Parts |", "|---|---|---|---|"]
    lines += [f"| {a} | {b} | {c} | {d} |" for a, b, c, d in glob_]
    lines += ["", "## Not assembled by JLC", ""] + [f"- {v} ({rf}): {m}" for v, rf, m in left]
    if missing:
        lines += ["", "## In the BOM but not on the board (check!)", ""] + [f"- {m}" for m in missing]
    (OUT / "sourcing.md").write_text("\n".join(lines) + "\n")
    for f in tmp.glob("*"):
        f.unlink()
    tmp.rmdir()
    print(f"bom {len(bom)} lines, cpl {len(cpl)} parts, missing on board {missing}; {OUT}")


if __name__ == "__main__":
    main()
