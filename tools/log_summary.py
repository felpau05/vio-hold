#!/usr/bin/env python3
"""Numbers from run logs (logs/run_*.csv of "apb_main.py run", or replays) for the judges' evidence (Stage 1
package rev 1.3: C onboard execution, D pilot handover, E reproducibility over repeated runs, the recovery bonus).

  log_summary.py logs/run_*.csv [--csv out.csv]

One row for each log, then the median and the range over the logs:
  s        seconds from the first to the last frame row
  sent/s   fixes sent each second; %sent  share of frame rows with sent = 1
  proc p50/p95/max   Hold's processing time per frame (ms);  age p50/p95  fix age when sent (ms)
  gap      longest time between two sent fixes (s)
  ok/deg/lost        frame rows in each Hold state
  switch   for each "engaged" note: sent fixes in the 2 s before and after it, and the longest gap in those 4 s
  recover  for each "recovery test: video back" note: seconds to the next sent fix
The numbers are the program's own. The positions here are its estimate, not the true position: this tool
gives no drift.
"""
import argparse
import csv
import sys

import numpy as np

AGE_PLAUSIBLE_MS = 1000           # a fix age over 1 s is a clock fault, not a slow fix (the FC refuses it anyway)


def f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def summary(path):
    rows = [r for r in csv.DictReader(open(path)) if f(r.get("t")) is not None]
    if not rows:
        return None
    t = np.array([f(r["t"]) for r in rows])
    sent = np.array([r["sent"] == "1" for r in rows])
    ts = t[sent]
    proc = np.array([f(r["proc_ms"]) for r in rows if f(r["proc_ms"]) and r["state"]])
    age = np.array([f(r["age_ms"]) for r in rows if r["sent"] == "1" and f(r["age_ms"]) is not None])
    dur = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    gaps = np.diff(ts) if len(ts) > 1 else np.array([dur])
    st = [r["state"] for r in rows]
    out = dict(log=path, s=dur, sent_per_s=len(ts) / dur if dur else 0.0, pct_sent=100.0 * sent.mean(),
               proc_p50=np.percentile(proc, 50) if len(proc) else None, proc_p95=np.percentile(proc, 95) if len(proc) else None,
               proc_max=proc.max() if len(proc) else None, age_p50=np.percentile(age, 50) if len(age) else None,
               age_p95=np.percentile(age, 95) if len(age) else None, gap=float(gaps.max()) if len(gaps) else None,
               ok=st.count("ok"), deg=st.count("degraded"), lost=st.count("lost"), switch=[], recover=[])
    for r in rows:
        note, tr = r.get("note") or "", f(r["t"])
        if note.startswith("engaged") or "; engaged" in note:
            lo, hi = max(tr - 2, t[0]), min(tr + 2, t[-1])         # the 4 s window, inside the log
            win = ts[(ts >= lo) & (ts <= hi)]
            g = np.diff(np.r_[lo, win, hi]).max()
            out["switch"].append((int(((ts >= tr - 2) & (ts < tr)).sum()), int(((ts >= tr) & (ts <= tr + 2)).sum()), float(g)))
        if "recovery test: video back" in note:
            nxt = ts[ts >= tr]
            out["recover"].append(float(nxt[0] - tr) if len(nxt) else None)
    return out


def fmt(x, nd=1):
    return "-" if x is None else f"{x:.{nd}f}"


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("logs", nargs="+")
    p.add_argument("--csv", help="also write the table to this CSV file")
    a = p.parse_args(argv)
    res = [x for x in (summary(q) for q in a.logs) if x]
    if not res:
        print("FAIL no frame rows in these logs")
        return 1
    head = ["log", "s", "sent/s", "%sent", "proc p50", "p95", "max", "age p50", "p95", "gap s", "ok", "deg", "lost",
            "switch (before, after, gap s)", "recover s"]
    lines = []
    for r in res:
        sw = " ".join(f"({b},{c},{g:.2f})" for b, c, g in r["switch"]) or "-"
        rc = " ".join(fmt(x, 2) for x in r["recover"]) or "-"
        lines.append([r["log"], fmt(r["s"]), fmt(r["sent_per_s"]), fmt(r["pct_sent"]), fmt(r["proc_p50"]), fmt(r["proc_p95"]),
                      fmt(r["proc_max"]), fmt(r["age_p50"], 0), fmt(r["age_p95"], 0), fmt(r["gap"], 2), str(r["ok"]),
                      str(r["deg"]), str(r["lost"]), sw, rc])
    print("  ".join(head))
    for ln in lines:
        print("  ".join(ln))
    for r in res:                     # a real fix is 0.1 to 0.3 s old; hours mean the frame stamps use another clock
        if r["age_p95"] is not None and not 0 <= r["age_p95"] <= AGE_PLAUSIBLE_MS:
            print(f"WARN {r['log']}: fix age p95 {r['age_p95']:.0f} ms is outside 0 to {AGE_PLAUSIBLE_MS} ms. The frame "
                  "stamps and the program clock (time.monotonic) differ. Do not use these ages as evidence.")
    if len(res) > 1:
        for key, name in (("sent_per_s", "sent/s"), ("proc_p95", "proc p95 ms"), ("age_p95", "age p95 ms"), ("gap", "gap s")):
            v = [r[key] for r in res if r[key] is not None]
            if v:
                print(f"{name}: median {np.median(v):.2f}, range {min(v):.2f} to {max(v):.2f} over {len(v)} logs")
    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(head)
            w.writerows(lines)
        print(f"saved {a.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
