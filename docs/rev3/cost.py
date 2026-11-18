#!/usr/bin/env python3
"""What does per-output switching cost? Live JLCPCB prices for each option.

    python3 cost.py [boards_in_batch] > cost.md

Queries JLCPCB's parts API for every part below, prices each one at the break for its
total quantity in the batch, and prints a markdown comparison. Parts JLC does not stock
(blade fuses and holders) and generic passives use the ESTIMATES table, so check those
by hand.

Only the parts that differ between options are counted. Connectors, MCU, CAN, aux supply
and battery monitor are the same in every option and are left out.
"""

import json
import sys
import urllib.request
from datetime import date

API = "https://jlcpcb.com/api/overseas-pcb-order/v1/shoppingCart/smtGood/selectSmtComponentList"

# JLC part numbers, chosen 2026-11-18 for stock and price.
JLC = {
    "TPS48111": "C17558994",   # TPS48111LQDGXRQ1, 42 V smart high-side driver
    "TPS48110": "C17556513",   # TPS48110AQDGXRQ1, same without pre-charge
    "TPS16630": "C1849461",    # TPS16630PWPR, 24 V eFuse, HTSSOP-20
    "TPS25982": "C2155765",    # TPS259827ONRGET, 15 A eFuse
    "TPS259470": "C3662799",   # TPS259470ARPWR, 5.5 A eFuse
    "TLA2528": "C2866175",     # 8-ch 12-bit I2C ADC
    "TCA9535": "C130204",      # 16-bit I2C expander, TSSOP-24
    "BSC040N10NS5": "C534334", # 100 V 4 mOhm FET, 5x6
    "CSD19534Q5A": "C114200",  # 100 V FET for pre-charge, 5x6
    "BSC010N04LS": "C501508",  # 40 V 1 mOhm FET for rail switches
    "IPT015N10N5": "C108964",  # 100 V 1.5 mOhm FET, TOLL, for the 42 V group
    "PTC 33 V 3 A": "C2943634",  # SMD2920P300TF/33
    "PTC 16 V 3 A": "C167318",   # SMD2920P300TF/16
    "PTC 8 V 3 A": "C21004",     # SMD1812P300TF
    "LM5122": "C77241",          # LM5122MHX/NOPB boost controller for the 54 V supply
    "inductor 22 uH 10 A": "C357119",  # TMPA1265SP-220MN-D
    "shunt 5 mOhm": "C185817",
    "shunt 10 mOhm": "C163051",
    "2.2 uF 100 V 1210": "C55151",
    "SMBJ58A": "C151910",
    "shunt 2 mOhm": "C160590",
    "shunt 0.5 mOhm": "C154666",
    "MMBT3904": "C20526",      # FET temperature sense
}
ESTIMATES = {                  # USD each; not from JLC
    "passive": 0.004,          # JLC basic resistor/capacitor
    "blade holder": 0.71,      # Keystone 3568, DigiKey at 100 pcs
    "blade fuse 32 V": 0.25,
    "blade fuse 58 V": 0.80,
    "100 uF 80 V bulk": 0.30,  # electrolytic, ESTIMATE
    "54 V connector": 0.50,    # unique type, ESTIMATE
}

# One entry per building block: (part, quantity).
BLOCKS = {
    "42V channel": [("TPS48111", 1), ("BSC040N10NS5", 1), ("CSD19534Q5A", 1),
                    ("shunt 2 mOhm", 1), ("MMBT3904", 1), ("passive", 14)],
    "24V channel": [("TPS16630", 1), ("passive", 7)],
    "heavy channel (TPS25982)": [("TPS25982", 1), ("passive", 8)],
    "light channel (TPS259470)": [("TPS259470", 1), ("passive", 7)],
    "42V group switch, 40 A": [("TPS48111", 1), ("BSC040N10NS5", 2), ("CSD19534Q5A", 1),
                               ("shunt 0.5 mOhm", 1), ("passive", 14)],
    "rail switch, 25 A": [("TPS48110", 1), ("BSC010N04LS", 1), ("shunt 0.5 mOhm", 1),
                          ("passive", 12)],
    "fused output <=24 V": [("blade holder", 1), ("blade fuse 32 V", 1)],
    "fused output 42 V": [("blade holder", 1), ("blade fuse 58 V", 1)],
    "42V group eFuse (chosen)": [("TPS48111", 1), ("IPT015N10N5", 2), ("shunt 0.5 mOhm", 1),
                                 ("passive", 16)],
    "24V rail eFuse (chosen)": [("TPS48111", 1), ("BSC040N10NS5", 2), ("shunt 2 mOhm", 1),
                                ("passive", 16)],
    "12/5V rail eFuse (chosen)": [("TPS48111", 1), ("BSC010N04LS", 2), ("shunt 2 mOhm", 1),
                                     ("passive", 16)],
    "PTC 24V output": [("PTC 33 V 3 A", 1)],
    "54V supply eFuse": [("TPS48111", 1), ("BSC040N10NS5", 2), ("shunt 5 mOhm", 1),
                         ("passive", 16)],
    "54V boost (LM5122)": [("LM5122", 1), ("BSC040N10NS5", 2), ("inductor 22 uH 10 A", 1),
                           ("shunt 10 mOhm", 1), ("2.2 uF 100 V 1210", 10), ("100 uF 80 V bulk", 1),
                           ("SMBJ58A", 1), ("54 V connector", 1), ("passive", 15)],
    "PTC 12V output": [("PTC 16 V 3 A", 1)],
    "PTC 5V output": [("PTC 8 V 3 A", 1)],
    "I/O expander": [("TCA9535", 1)],
    "ADC": [("TLA2528", 1)],
}

OPTIONS = [
    ("C", "**Chosen 2026-11-18:** an eFuse per rail (with current monitor), a PTC per 24/12/5 V output, "
          "no 15 V rail, plus the on-board 54 V boost for the Zyxel switch",
     {"42V group eFuse (chosen)": 1, "24V rail eFuse (chosen)": 1, "12/5V rail eFuse (chosen)": 2,
      "PTC 24V output": 8, "PTC 12V output": 7, "PTC 5V output": 11,
      "54V supply eFuse": 1, "54V boost (LM5122)": 1, "I/O expander": 1, "ADC": 2}),
    ("A", "Per-output everywhere, TPS25982 on all 15 XT60 outputs (as specced)",
     {"42V channel": 8, "24V channel": 8, "heavy channel (TPS25982)": 15,
      "light channel (TPS259470)": 11, "I/O expander": 6, "ADC": 7}),
    ("A2", "Per-output everywhere, TPS25982 only on 4 heavy XT60 outputs",
     {"42V channel": 8, "24V channel": 8, "heavy channel (TPS25982)": 4,
      "light channel (TPS259470)": 22, "I/O expander": 6, "ADC": 7}),
    ("H", "Per-output on 24/15/12/5 V; 42 V as one switched group with a fuse per output",
     {"42V group switch, 40 A": 1, "fused output 42 V": 8, "24V channel": 8,
      "heavy channel (TPS25982)": 4, "light channel (TPS259470)": 22,
      "I/O expander": 5, "ADC": 5}),
    ("B", "Per-rail switches, a blade fuse per output, per-rail current only",
     {"42V group switch, 40 A": 1, "rail switch, 25 A": 4, "fused output <=24 V": 34,
      "fused output 42 V": 8, "ADC": 1}),
    ("B0", "Per-rail switches only, nothing per output",
     {"42V group switch, 40 A": 1, "rail switch, 25 A": 4, "ADC": 1}),
]


def fetch_breaks(code):
    req = urllib.request.Request(
        API, data=json.dumps({"keyword": code, "currentPage": 1, "pageSize": 5}).encode(),
        headers={"Content-Type": "application/json", "User-Agent": "Mozilla/5.0"})
    data = json.load(urllib.request.urlopen(req, timeout=30))
    for item in data["data"]["componentPageInfo"]["list"] or []:
        if item["componentCode"] == code:
            return item["componentModelEn"], item["componentPrices"], item["stockCount"]
    raise LookupError(f"{code} not found at JLCPCB")


def unit_price(breaks, qty):
    price = breaks[0]["productPrice"]
    for b in breaks:
        if b["startNumber"] <= qty:
            price = b["productPrice"]
    return price


def main():
    boards = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    breaks = {name: fetch_breaks(code) for name, code in JLC.items()}

    def option_cost(blocks):
        parts = {}
        for block, n in blocks.items():
            for part, q in BLOCKS[block]:
                parts[part] = parts.get(part, 0) + q * n
        total = 0.0
        for part, q in parts.items():
            u = unit_price(breaks[part][1], q * boards) if part in breaks else ESTIMATES[part]
            total += u * q
        return total

    out = [f"# Cost of per-output switching\n",
           f"Generated by `cost.py` on {date.today()} from live JLCPCB prices, for a batch of "
           f"**{boards} boards**. USD per board, parts that differ between options only.\n",
           "| | Option | Parts per board | vs. B |", "|---|---|---|---|"]
    costs = {key: option_cost(blocks) for key, _, blocks in OPTIONS}
    for key, desc, _ in OPTIONS:
        out.append(f"| **{key}** | {desc} | ${costs[key]:.0f} | {costs[key] - costs['B']:+.0f} |")

    out += ["", "Options A to B0 are the 2026-11-18 comparison. They still assume the old rail set "
            "(15 V rail, no 54 V supply) and are kept as the record behind choosing C."]
    out += ["", "## Per building block\n", "| Block | Cost each |", "|---|---|"]
    ref = OPTIONS[0][2]
    for block, items in BLOCKS.items():
        each = 0.0
        for part, q in items:
            n = sum(qq * ref.get(b, 0) for b, its in BLOCKS.items() for p, qq in its if p == part)
            each += (unit_price(breaks[part][1], max(n, q) * boards) if part in breaks
                     else ESTIMATES[part]) * q
        out.append(f"| {block} | ${each:.2f} |")

    out += ["", "## JLC parts used\n", "| Part | JLC # | MPN | Stock |", "|---|---|---|---|"]
    for name, code in JLC.items():
        mpn, _, stock = breaks[name]
        out.append(f"| {name} | {code} | {mpn} | {stock} |")
    out += ["", "Estimated, not quoted: " + ", ".join(f"{k} ${v}" for k, v in ESTIMATES.items()),
            "", "Not included: PCBA fees. Each extra unique extended part adds $1.50-3 per order, "
            "and the per-output options add about five more unique parts than option B."]
    print("\n".join(out))


if __name__ == "__main__":
    main()
