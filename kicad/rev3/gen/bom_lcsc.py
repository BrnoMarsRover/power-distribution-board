#!/usr/bin/env python3
"""Turn PowerBoard_rev3_bom.csv (kicad-cli export, grouped by MPN) into a CSV for the LCSC BOM tool.

Run after `kicad-cli sch export bom` (see README):  python3 kicad/rev3/gen/bom_lcsc.py
Rows whose MPN starts with "none:" (PCB pads, solder jumpers, bolt-on links, Deans T) are left out.
Quantities are for one board; set the board count in LCSC's tool.
"""
import csv
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
SRC, DST = HERE / "PowerBoard_rev3_bom.csv", HERE / "PowerBoard_rev3_bom_lcsc.csv"

MFR = [  # MPN prefix -> manufacturer (longest match wins)
    ("RC0603", "YAGEO"), ("RC1206", "YAGEO"), ("ERJ-", "Panasonic"), ("EEU-", "Panasonic"), ("AQY", "Panasonic"),
    ("CL", "Samsung Electro-Mechanics"), ("C0603C", "KEMET"), ("C1206C", "KEMET"), ("C1210C", "KEMET"),
    ("C3216", "TDK"), ("GRM", "Murata"), ("NCP18", "Murata"), ("5001", "Keystone"), ("5015", "Keystone"),
    ("B2B-", "JST"), ("B4B-", "JST"), ("BM04B-", "JST"), ("502585", "Molex"), ("XT", "AMASS"),
    ("MMBT3904LT1G", "onsemi"), ("2N7002LT1G", "onsemi"), ("MMSZ", "onsemi"), ("FDD", "onsemi"),
    ("TCAN", "Texas Instruments"), ("TPS", "Texas Instruments"), ("LM", "Texas Instruments"),
    ("INA", "Texas Instruments"), ("TLV", "Texas Instruments"), ("ADC128", "Texas Instruments"),
    ("TCA", "Texas Instruments"), ("SN74", "Texas Instruments"), ("TL3342", "E-Switch"),
    ("AP7361", "Diodes Incorporated"), ("1N5819HW", "Diodes Incorporated"), ("BAS21-7", "Diodes Incorporated"),
    ("SMBJ58A-13-F", "Diodes Incorporated"), ("SMBJ", "Littelfuse"), ("SMCJ", "Littelfuse"),
    ("IPB", "Infineon"), ("STPS", "STMicroelectronics"), ("USBLC6", "STMicroelectronics"),
    ("ESP32", "Espressif"), ("MF-", "Bourns"), ("CSS2H", "Bourns"), ("SRR", "Bourns"), ("MSS", "Coilcraft"),
    ("APT2012", "Kingbright"), ("SMD2920", "YAGEO"), ("USB4105", "GCT"), ("WSL", "Vishay"), ("SS34-E3", "Vishay"),
]


def maker(mpn):
    hits = [m for p, m in sorted(MFR, key=lambda x: -len(x[0])) if mpn.startswith(p)]
    return hits[0] if hits else ""


rows, skipped = [], []
for r in csv.DictReader(SRC.open()):
    if r["MPN"].startswith("none"):
        skipped.append(f'{r["Qty"]} x {r["Value"]}')
        continue
    rows.append({"Quantity": r["Qty"], "Manufacturer Part": r["MPN"], "Manufacturer": maker(r["MPN"]),
                 "Comment": r["Value"], "Designator": r["References"], "Footprint": r["Footprint"].split(":")[-1]})
missing = [r["Manufacturer Part"] for r in rows if not r["Manufacturer"]]
assert not missing, f"add a manufacturer for {missing}"
with DST.open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)
print(f"{DST.name}: {len(rows)} lines, {sum(int(r['Quantity']) for r in rows)} parts; left out: {', '.join(skipped)}")
