#!/usr/bin/env python3
"""Table of the hand-hover runs (integ/gap/hand_hover.py, results/hh/*.json). Code: before = old hold.py (MOVE_PX 1.5),
v1 = first "moved" patch (superseded: noise gave proofs), v2 = the patch now in hold.py (integ/gap/proposed.patch:
"moved" from measured points only, LIVE_MOVED_S 0.5 s). Runs from the patched copy. Per case: the gap before the switch for a hover of 5, 10 and
20 s (worst of the seeds), the share of hover frames sent (lowest), and for the freeze runs the frozen fixes sent
later than 0.5 s after the freeze started, the last one, and the largest error of a frozen fix.
Run: ~/.venvs/vio/bin/python integ/gap/hh_summary.py > integ/gap/results/hh_summary.txt
"""
import glob
import json
import os
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    print(__doc__.split("Run:")[0])
    runs = defaultdict(list)
    for p in sorted(glob.glob(os.path.join(HERE, "results", "hh", "*.json"))):
        r = json.load(open(p))
        # the file name tells the code version: no suffix = before, _fix = patch v1 (superseded), _fix2 = the patch
        # now in hold.py (integ/gap/proposed.patch). Both patched runs came from gap_copy, so "repo" cannot tell them apart.
        stem = os.path.basename(p)[:-5]
        code = "v2" if stem.endswith("_fix2") else "v1" if stem.endswith("_fix") else "before"
        key = (r["camera"], r["level"], r["height"], r["vib_deg"], bool(r["freeze"]), code)
        runs[key].append(r)
    print(f"{'camera':6s} {'level':7s} {'h m':>5s} {'vib':>4s} {'freeze':6s} {'code':>7s} {'seeds':>5s} "
          f"{'gap before switch T=5/10/20 s (worst)':>38s} {'sent % (low)':>12s}  frozen: sent >0.5 s / last s / max err m")
    for key in sorted(runs, key=lambda k: (k[0], k[4], k[1], -k[2], k[3], ("before", "v1", "v2").index(k[5]))):
        rs = runs[key]
        g = {T: max(r["gap_before_switch_s"].get(T, 0) for r in rs) for T in ("5", "10", "20")}
        sent = min(r["sent_pct_hover"] for r in rs)
        fz = ""
        if key[4]:
            fz = " / ".join(str(x) for x in (sum(r["frozen"]["sent_after_0_5s"] for r in rs),
                                             max(r["frozen"]["last_sent_after_freeze_s"] for r in rs),
                                             max(r["frozen"]["max_err_sent_m"] for r in rs)))
            fz += f"  (of {sum(r['frozen']['frames_after_0_5s'] for r in rs)} frozen frames after 0.5 s)"
        print(f"{key[0]:6s} {key[1]:7s} {key[2]:5.0f} {key[3]:4.1f} {'yes' if key[4] else 'no':6s} {key[5]:>7s} "
              f"{','.join(str(r['seed']) for r in rs):>5s} {g['5']:12.2f} {g['10']:6.2f} {g['20']:6.2f} {'':10s} {sent:8.1f}     {fz}")


if __name__ == "__main__":
    main()
