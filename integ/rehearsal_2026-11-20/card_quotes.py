#!/usr/bin/env python3
"""Step 9 of the rehearsal: is each quoted program message of DECISION_CARD.txt in the code?

  ~/.venvs/vio/bin/python integ/rehearsal_2026-11-20/card_quotes.py [DECISION_CARD.txt]

A quote "a <x> b" matches when each literal part (a, b; placeholders <...> are wildcards) is in the text of the
program files, with f-string fields removed. MISSING lists the parts that no file holds.
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
FILES = ["apb_main.py", "hold.py", "capture.py", "calib.py", "fc_link.py", "settings.py", "tools/board_check.sh",
         "tools/use_profile.py", "tools/plot_log.py"]


def main():
    card = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PROJ, "DECISION_CARD.txt")
    src = ""
    for f in FILES:
        t = open(os.path.join(PROJ, f)).read()
        src += re.sub(r"\{[^{}]*\}", "\x00", t)       # an f-string field matches nothing literal
    src = re.sub(r'"\s*\n\s*f?"', "", src)             # Python's implicit joins of adjacent string parts
    src = re.sub(r"\s+", " ", src)
    row, bad = "", 0
    for line in open(card):
        m = re.match(r"(\d+)\s+SEE", line)
        row = m.group(1) if m else row
        for q in re.findall(r'"([^"]{6,})"', line):
            parts = [p.strip() for p in re.split(r"<[^>]*>|\.\.\.", q) if len(p.strip()) >= 4]
            miss = [p for p in parts if p not in src]
            bad += bool(miss)
            print(f"row {row:>2} {'OK     ' if not miss else 'MISSING'} \"{q}\"" + (f"   not found: {miss}" if miss else ""))
    print(f"{bad} quote(s) with a missing part")


if __name__ == "__main__":
    main()
