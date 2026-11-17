#!/usr/bin/env python3
"""Run groups of apsim/sitl_ap.py cases, N side by side (one SITL instance each, own ports).
  ~/.venvs/vio/bin/python apsim/matrix_ap.py <group> [parallel]     groups: base, apb, seeds, att, holdout
Each case's output: apsim/logs/<label>.txt; results merge into apsim/results.json / results.txt."""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = os.path.expanduser("~/.venvs/vio/bin/python")
H, W = (50, 100, 122), ("calm", "w20g40")
GROUPS = {
    "base": [f"{v} {h} {w} 60" for v in ("truth", "gps", "none") for h in H for w in W],
    "apb": [f"apb {h} {w} 60" for h in H for w in W],
    "seeds": [f"apb 100 w20g40 60 seed={s}" for s in (2, 3)],
    "att": [f"apb 100 {w} 60 settings={{\"att_source\":\"fc\"}} label=ap_apb_100m_{w}_s1_attfc" for w in W],
    "holdout": [f"apb 100 w20g40 60 ground={g} seed={s} label=ap_apb_100m_w20g40_{g}_s{s}"
                for g, s in (("holdout_quarry", 101), ("holdout_field", 102), ("holdout_water", 103))],
}
GROUPS["rest"] = GROUPS["seeds"] + GROUPS["holdout"] + GROUPS["att"]
GROUPS["gps"] = [c for c in GROUPS["base"] if c.startswith("gps")]


def main():
    cases, n = GROUPS[sys.argv[1]], int(sys.argv[2]) if len(sys.argv) > 2 else 3
    running, k = [], 0
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    while cases or running:
        running = [p for p in running if p.poll() is None]
        if cases and len(running) < n:
            args = cases.pop(0).split(" ")
            off = 10 * (list(GROUPS).index(sys.argv[1]) + 1)     # groups may run side by side
            free = sorted(set(range(off, off + n + 1)) - {p.inst for p in running})[0]
            name = "_".join(a.replace("/", "") for a in args if "settings" not in a)[:80]
            log = open(os.path.join(HERE, "logs", f"m_{sys.argv[1]}_{name}.txt"), "w")
            p = subprocess.Popen([PY, os.path.join(HERE, "sitl_ap.py"), *args, f"instance={free}"],
                                 stdout=log, stderr=subprocess.STDOUT)
            p.inst, k = free, k + 1
            running.append(p)
            time.sleep(3)
        time.sleep(1)


if __name__ == "__main__":
    main()
