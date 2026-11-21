#!/usr/bin/env python3
"""Area C proof picture: what the program on the drone did while the laptop was unplugged.

  ~/.venvs/vio/bin/python tools/rc_plot.py                 newest rc_*.csv from the board -> rc_plot_<name>.png
  ~/.venvs/vio/bin/python tools/rc_plot.py logs/rc_X.csv   a local file
  --last 120                                               only the last 120 s (default: the whole file)

The board program (apb_live.py --rc, msp_rc.py) writes /data/vio/logs/rc_<time>.csv at 10 Hz: the move since the
zero (floor track), the command we sent and what the FC reports back (MSP_RC). The picture has two parts:
  left:  the path of the drone from above (right, forward), with arrows = the roll/pitch command. Each arrow must
         point back toward the zero (the command pushes against the move).
  right: over time, the move (m) and the roll/pitch we sent and the FC used (us). Shaded: engage switch on.
The times come from the board clock, so the rows during the unplugged time prove that the program ran on the drone.
"""
import argparse
import csv
import os
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt     # noqa: E402

BOARD = "root@192.168.75.1"
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def fetch():
    name = subprocess.run(["ssh", "-o", "ConnectTimeout=5", BOARD, "ls -t /data/vio/logs/rc_*.csv | head -1"],
                          capture_output=True, text=True, timeout=20).stdout.strip()
    if not name:
        sys.exit("FAIL no rc_*.csv on the board. Start apb_live.py with --rc first.")
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    local = os.path.join(HERE, "logs", os.path.basename(name))
    subprocess.run(["scp", "-q", f"{BOARD}:{name}", local], check=True, timeout=60)
    return local


def num(v):
    return None if v in ("", "None") else float(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?")
    ap.add_argument("--last", type=float)
    ap.add_argument("--all", action="store_true", help="the whole file, not only from the last zero")
    a = ap.parse_args()
    path = a.csv or fetch()
    rows = [{k: num(v) for k, v in r.items()} for r in csv.DictReader(open(path))]
    if a.last and rows:
        rows = [r for r in rows if r["t"] >= rows[-1]["t"] - a.last]
    zeros = [i for i, r in enumerate(rows) if r.get("zero")]
    t_board_start = rows[0]["t"] if rows else 0.0
    if zeros and not a.all:
        rows = rows[zeros[-1]:]                      # the test starts at the last zero ("Zero here" or switch-on)
    if len(rows) < 2:
        sys.exit(f"FAIL {path}: fewer than 2 rows")
    dur = rows[-1]["t"] - rows[0]["t"]
    pts = [r for r in rows if r["right"] is not None and r["fwd"] is not None]
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.axhline(0, color="0.3", lw=1)
    ax.axvline(0, color="0.3", lw=1)
    ax.plot([0], [0], "ko", ms=10)
    ax.annotate("START\n(tracking starts here)", (0, 0), xytext=(10, -34), textcoords="offset points", fontsize=11)
    ax.annotate("", xy=(0, 0.3), xytext=(0, 0.02), arrowprops=dict(arrowstyle="-|>", color="k", lw=2))
    ax.annotate("nose", (0, 0.3), xytext=(6, -4), textcoords="offset points", fontsize=10)
    if pts:
        ax.plot([r["right"] for r in pts], [r["fwd"] for r in pts], "-", color="0.75", lw=1.5, label="where the drone went")
    # one label per place the drone stopped away from the start: the farthest point of each trip out
    trips, cur = [], []
    for r in pts:
        if (r["right"] ** 2 + r["fwd"] ** 2) ** 0.5 > 0.1:
            cur.append(r)
        elif cur:
            trips.append(cur)
            cur = []
    if cur:
        trips.append(cur)
    n_pass = 0
    for trip in trips:
        r = max(trip, key=lambda q: q["right"] ** 2 + q["fwd"] ** 2)
        x, y = r["right"], r["fwd"]
        side = abs(x) >= abs(y)
        move = f"{abs(x):.2f} m {'right' if x > 0 else 'left'}" if side else f"{abs(y):.2f} m {'forward' if y > 0 else 'back'}"
        if r["roll"] is None or r["pitch"] is None:
            continue
        stick, val = ("roll", r["roll"]) if side else ("pitch", r["pitch"])
        if side:
            push = "pushes left" if val < 1500 else "pushes right" if val > 1500 else "no push"
            ok = (x > 0 and val < 1500) or (x < 0 and val > 1500)
        else:
            push = "pushes back" if val < 1500 else "pushes forward" if val > 1500 else "no push"
            ok = (y > 0 and val < 1500) or (y < 0 and val > 1500)
        n_pass += ok
        col = "tab:green" if ok else "tab:red"
        dx, dy = (r["roll"] - 1500) / 250.0, (r["pitch"] - 1500) / 250.0      # 100 us -> 0.4 m arrow
        ax.plot([x], [y], "o", color=col, ms=12)
        ax.annotate("", xy=(x + dx, y + dy), xytext=(x, y), arrowprops=dict(arrowstyle="-|>", color=col, lw=3))
        ax.annotate(f"moved {move}\n{stick} {val:.0f} = {push}\n{'PASS' if ok else 'FAIL'}", (x, y),
                    xytext=(12, 12), textcoords="offset points", fontsize=11, color=col, fontweight="bold")
    lim = max([0.8] + [abs(v) for r in pts for v in (r["right"], r["fwd"])]) + 0.5
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("metres", fontsize=11)
    ax.set_ylabel("metres", fontsize=11)
    kw = dict(fontsize=16, fontweight="bold", color="0.35", ha="center", va="center")
    ax.text(0, lim * 0.9, "FORWARD (nose)", **kw)
    ax.text(0, -lim * 0.9, "BACK", **kw)
    ax.text(-lim * 0.85, -0.22, "LEFT", **kw)
    ax.text(lim * 0.85, -0.22, "RIGHT", **kw)
    ax.plot([], [], "-", color="tab:green", lw=3, label="arrow = command (must point back to START)")
    ax.legend(loc="lower right", fontsize=9)
    verdict = f"{n_pass} of {len(trips)} moves: command pushes back" if trips else "no move of more than 0.1 m"
    start = (f"tracking starts at START (zero set {rows[0]['t'] - t_board_start:.0f} s into the log)" if zeros and not a.all
             else "no zero in this log: shown from the first row")
    ax.set_title(f"Area C - commands made ON THE DRONE\n{verdict}\n"
                 f"{start}; {len(rows)} rows written by the drone over {dur:.0f} s ({os.path.basename(path)})", fontsize=11)
    fig.tight_layout()
    out = os.path.join(os.path.dirname(path), "rc_plot_" + os.path.splitext(os.path.basename(path))[0] + ".png")
    fig.savefig(out, dpi=110)
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
