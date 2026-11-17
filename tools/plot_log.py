#!/usr/bin/env python3
"""Quick look at one per-frame log of "apb_main.py run" (logs/run_*.csv) or of a replay: one PNG.

  plot_log.py logs/run_20261121_141500.csv [-o out.png]      (default: the CSV name with .png)

Panels, all against seconds from the first frame: the fix position (north, east), the state of each frame,
fixes sent per second (line: min_send_hz), the fix age and processing time (line: 150 ms, EVENT_DAY step 2),
tracked points (line: Hold's MIN_PTS), the height (line: min_height_m), and why frames were not sent.
Lines come from settings.json (or --settings). Takes a few seconds.
"""
import argparse
import csv
import os
import re
import sys
from collections import Counter

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402
import numpy as np                # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import settings as S              # noqa: E402

BLUE, ORANGE, INK, INK2, SURFACE = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#fcfcfb"
STATES = [("ok", "#0ca30c"), ("degraded", "#fab219"), ("init", "#2a78d6"), ("lost", "#d03b3b"), ("refused", "#8a8986")]
MIN_PTS = 20                      # hold.Hold.MIN_PTS; hold_cfg "min_pts" overrides it
AGE_MAX_MS = 150                  # EVENT_DAY step 2: fix age must be under 150 ms
LBL = dict(facecolor=SURFACE, edgecolor="none", pad=1.0)   # threshold labels stay readable over the data


def num(x):
    return float(x) if x not in ("", None) else np.nan


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log")
    p.add_argument("-o", "--out")
    p.add_argument("--settings", default=S.PATH)
    a = p.parse_args()
    s = S.load(a.settings)
    rows = list(csv.DictReader(open(a.log)))
    timed = [r for r in rows if r["t"]]
    if not timed:
        print(f"FAIL {a.log} has no frame rows")
        return 1
    t = np.array([float(r["t"]) for r in timed])
    t = t - t[0]
    col = lambda k: np.array([num(r[k]) for r in timed])     # noqa: E731
    sent = np.array([r["sent"] == "1" for r in timed])
    north, east, age, proc, n, h = col("north"), col("east"), col("age_ms"), col("proc_ms"), col("n"), col("height")

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.facecolor": SURFACE, "figure.facecolor": SURFACE,
                         "axes.grid": True, "grid.color": "#e4e3df", "grid.linewidth": 0.6,
                         "axes.spines.top": False, "axes.spines.right": False})
    fig = plt.figure(figsize=(11, 15))
    gs = fig.add_gridspec(7, 1, height_ratios=[1.3, 0.8, 0.9, 0.9, 0.9, 0.8, 1.3], hspace=0.5, top=0.95, bottom=0.04)
    ax = [fig.add_subplot(gs[i]) for i in range(6)]
    for x in ax[1:]:
        x.sharex(ax[0])
    dur = t[-1]
    fig.suptitle(f"{os.path.basename(a.log)}: {dur:.0f} s, {len(timed)} frames, {sent.mean():.0%} sent, "
                 f"median fix age {np.nanmedian(age[sent]) if sent.any() and np.isfinite(age[sent]).any() else float('nan'):.0f} ms, "
                 f"median processing {np.nanmedian(proc):.1f} ms", color=INK, fontsize=11, x=0.02, ha="left", y=0.995)

    a0 = ax[0]
    a0.plot(t[sent], north[sent], ".", ms=2.5, color=BLUE, label="north (sent)")
    a0.plot(t[sent], east[sent], ".", ms=2.5, color=ORANGE, label="east (sent)")
    a0.set_ylabel("fix position (m)")
    a0.legend(loc="lower left", bbox_to_anchor=(0, 1), frameon=False, ncol=2, markerscale=3)

    a1 = ax[1]
    state = [r["state"] or "refused" for r in timed]
    for i, (name, c) in enumerate(STATES):
        m = np.array([x == name for x in state])
        a1.plot(t[m], np.full(m.sum(), i), "|", ms=8, color=c)
    a1.set_yticks(range(len(STATES)), [f"{x} ({sum(1 for y in state if y == x)})" for x, _ in STATES])
    a1.set_ylim(-0.7, len(STATES) - 0.3)
    a1.set_ylabel("state")

    a2 = ax[2]
    bins = np.arange(0, np.floor(dur) + 2)
    a2.bar(bins[:-1] + 0.5, np.histogram(t[sent], bins)[0], width=0.85, color=BLUE)
    a2.axhline(s["min_send_hz"], color=INK2, ls="--", lw=1)
    a2.text(dur, s["min_send_hz"], f"min_send_hz {s['min_send_hz']}", color=INK2, va="bottom", ha="right", fontsize=8, bbox=LBL)
    a2.set_ylabel("fixes sent per s")

    a3 = ax[3]
    if np.isfinite(age).any():
        a3.plot(t, age, lw=1, color=BLUE, label="fix age")
    a3.plot(t, proc, lw=1, color=ORANGE, label="processing")
    a3.axhline(AGE_MAX_MS, color=INK2, ls="--", lw=1)
    a3.text(dur, AGE_MAX_MS, f"{AGE_MAX_MS} ms (EVENT_DAY step 2)", color=INK2, va="bottom", ha="right", fontsize=8, bbox=LBL)
    a3.set_ylabel("ms")
    a3.legend(loc="lower left", bbox_to_anchor=(0, 1), frameon=False, ncol=2)
    if not np.isfinite(age).any():
        a3.text(0.5, 0.85, "no fix age in this log (replay logs have none)", transform=a3.transAxes, ha="center", color=INK2)

    a4 = ax[4]
    a4.plot(t, n, ".", ms=2, color=BLUE)
    mp = s["hold_cfg"].get("min_pts", MIN_PTS)
    a4.axhline(mp, color=INK2, ls="--", lw=1)
    a4.text(dur, mp, f"MIN_PTS {mp}", color=INK2, va="bottom", ha="right", fontsize=8, bbox=LBL)
    a4.set_ylabel("tracked points")

    a5 = ax[5]
    a5.plot(t, h, lw=1, color=BLUE)
    a5.axhline(s["min_height_m"], color=INK2, ls="--", lw=1)
    a5.text(dur, s["min_height_m"], f"min_height_m {s['min_height_m']:g}", color=INK2, va="bottom", ha="right", fontsize=8, bbox=LBL)
    a5.set_ylabel("height (m)")
    a5.set_xlabel("seconds from the first frame")
    for x in ax[:5]:
        plt.setp(x.get_xticklabels(), visible=False)

    a6 = fig.add_subplot(gs[6])
    why = Counter(re.sub(r"-?\d+(\.\d+)?", "#", r["why"]) or "(none)" for r in rows if r["sent"] != "1")
    top = why.most_common(12)[::-1]
    if top:
        a6.barh(range(len(top)), [c for _, c in top], color=BLUE, height=0.7)
        a6.set_yticks([])
        for i, (w, c) in enumerate(top):      # count and reason after the bar: long reasons need no margin
            a6.text(c, i, f"  {c}  {w[:80]}", va="center", color=INK, fontsize=8)
        a6.set_xlim(0, 1.9 * top[-1][1])
    a6.set_xlabel(f"frames not sent, by reason ({sum(why.values())} of {len(rows)} rows; # = a number)")
    a6.grid(axis="y", visible=False)

    out = a.out or os.path.splitext(a.log)[0] + ".png"
    fig.savefig(out, dpi=90, bbox_inches="tight")
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
