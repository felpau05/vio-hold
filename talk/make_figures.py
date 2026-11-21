#!/usr/bin/env python3
"""Make the talk figures from result files that exist in the repository. No simulator starts.

Run from the repository root:  ~/.venvs/vio/bin/python talk/make_figures.py
Out: talk/fusion.png, talk/outage_view.png, talk/rolling_shutter.png, talk/hold_vs_height.png, talk/betaflight_modes.png.
Every number is read from a result file. The sources are listed in talk/captions.txt.
"""
import json
import os
import re
import shutil
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "talk")
os.chdir(ROOT)

BLUE, ORANGE, GREY, GREEN = "#1f6fb4", "#d95f02", "#555555", "#1b7f5c"
plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.alpha": 0.3, "figure.dpi": 120})


def save(fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name))
    plt.close(fig)
    print("wrote talk/" + name)


# 1. Fusion figure: the project's own log plot of one closed-loop run (tools/plot_log.py).
def fusion():
    csv = "integ/gap/results/closed/gapfix_jdcal_apb_100m_w20g40_s11_onboard.csv"
    subprocess.run([sys.executable, "tools/plot_log.py", csv, "-o", os.path.join(OUT, "fusion.png")], check=True)
    print("wrote talk/fusion.png")


# 2. Outage figure: the existing picture of the simulated camera view during the outage.
def outage():
    shutil.copy("integ/judge_outage_21s.png", os.path.join(OUT, "outage_view.png"))
    print("wrote talk/outage_view.png")


# 3. Rolling shutter: RMS of three readout variants, per seed (integ/justice_d/summary.txt).
def rolling_shutter():
    rows = {}
    pat = re.compile(r"^JD_crop\s+100\s+(main|readout_0|readout_wrong_sign)\s+(1[123])\s+\d+\s+[\d.]+\s+([\d.]+)\s+([\d.]+)")
    for line in open("integ/justice_d/summary.txt"):
        m = pat.match(line)
        if m:
            rows[(m.group(1), int(m.group(2)))] = float(m.group(3))
    variants = [("main", "model (true readout)", BLUE), ("readout_0", "readout 0", ORANGE),
                ("readout_wrong_sign", "wrong sign", GREY)]
    seeds = [11, 12, 13]
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    w = 0.26
    for i, (key, label, col) in enumerate(variants):
        xs = [j + (i - 1) * w for j in range(3)]
        ys = [rows[(key, s)] for s in seeds]
        ax.bar(xs, ys, w, label=label, color=col)
        for x, y in zip(xs, ys):
            ax.text(x, y + 0.01, f"{y:.2f}", ha="center", fontsize=9)
    ax.set_xticks(range(3))
    ax.set_xticklabels([f"seed {s}" for s in seeds])
    ax.set_ylabel("RMS position error (m)")
    ax.set_title("Rolling-shutter model, 100 m, open loop: the seed spread is larger than the effect")
    ax.title.set_fontsize(10)
    ax.legend(frameon=False)
    save(fig, "rolling_shutter.png")


# 4. Hold error against height: PX4 and ArduCopter SITL, Justice D, wind 20 gusting 40 km/h, and a perfect position.
def table_rows(path, first_col_pat):
    out = {}
    for line in open(path):
        parts = line.split()
        if parts and re.match(first_col_pat, parts[0]):
            out[parts[0]] = parts
    return out


def hold_vs_height():
    pts = {"PX4": [], "ArduCopter": [], "perfect": []}
    # PX4: px4sim/low_outage_results.txt, columns: run var h_m wind outage rms_m ...
    for name, p in table_rows("px4sim/low_outage_results.txt", r"lowout_").items():
        if p[1] not in ("apb", "truth"):
            continue
        var, h, wind, outage, rms = p[1], float(p[2]), p[3], p[4], float(p[5])
        if wind != "w20g40" or outage != "-":
            continue
        pts["perfect" if var == "truth" else "PX4"].append((h, rms, "PX4"))
    # ArduCopter low heights: apsim/low_ap_results.txt, columns: run rms ...
    for name, p in table_rows("apsim/low_ap_results.txt", r"lowap_").items():
        if "w20g40" not in name or name.endswith(":"):
            continue
        h = float(re.search(r"_(\d+)m_", name).group(1))
        pts["perfect" if "truth" in name else "ArduCopter"].append((h, float(p[1]), "ArduCopter"))
    # ArduCopter 50 m and 100 m: integ/justice_d/summary.txt closed-loop table (Justice D, wind 20 gusting 40).
    for line in open("integ/justice_d/summary.txt"):
        m = re.match(r"^(jd_apb_(50|100)m_w20g40_s\d+)\s+([\d.]+)\s", line)
        if m:
            pts["ArduCopter"].append((float(m.group(2)), float(m.group(3)), "ArduCopter"))
    fig, ax = plt.subplots(figsize=(7.5, 4.6))
    for key, mk, col, lab in [("PX4", "o", BLUE, "PX4 SITL, our fixes"),
                              ("ArduCopter", "s", ORANGE, "ArduCopter SITL, our fixes"),
                              ("perfect", "D", GREEN, "perfect position (true position as the fix)")]:
        if key == "perfect":
            for (h, r, ap) in pts[key]:
                dx = -0.04 if ap == "PX4" else 0.04
                ax.scatter(h * (1 + dx), r, marker=mk, s=50, facecolors="none", edgecolors=col, linewidths=1.8,
                           label=lab if (h, r, ap) == pts[key][0] else None)
        else:
            ax.scatter([h for h, _, _ in pts[key]], [r for _, r, _ in pts[key]], marker=mk, s=45,
                       color=col, alpha=0.85, label=lab)
    ax.set_xscale("log")
    ax.set_xticks([5, 10, 50, 100])
    ax.set_xticklabels(["5", "10", "50", "100"])
    ax.set_xlabel("height (m)")
    ax.set_ylabel("RMS distance from the hold point, 60 s (m)")
    ax.set_title("Simulation only. Each mark is one run.", fontsize=10)
    ax.legend(frameon=False, fontsize=9)
    save(fig, "hold_vs_height.png")
    return pts


# 5. Betaflight simulator: position lock mode v1 (gain 0) against mode v2 (gain 0.2 per s), 60 s hold, seeds 1 and 2.
def betaflight():
    res = {}
    for g in ("0", "0.2"):
        for s in (1, 2):
            d = json.load(open(f"betasim/results/closed_g{g}_s{s}.json"))
            res[(g, s)] = (d["rms_m"], d["max_m"])
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    w = 0.34
    for i, (g, lab, col) in enumerate([("0", "mode v1 (no position lock)", GREY), ("0.2", "mode v2 (lock, gain 0.2 per s)", BLUE)]):
        xs = [j + (i - 0.5) * w for j in range(2)]
        ys = [res[(g, s)][0] for s in (1, 2)]
        ax.bar(xs, ys, w, color=col, label=lab)
        for x, y in zip(xs, ys):
            ax.text(x, y + 0.05, f"{y:.2f}", ha="center", fontsize=9)
    ax.set_xticks(range(2))
    ax.set_xticklabels(["seed 1", "seed 2"])
    ax.set_ylabel("RMS distance from the hold point, 60 s (m)")
    ax.set_title("Betaflight simulator, flow messages from a synthetic fix with 0.10 m/s speed error", fontsize=10)
    ax.legend(frameon=False)
    save(fig, "betaflight_modes.png")
    return res


if __name__ == "__main__":
    fusion()
    outage()
    rolling_shutter()
    p = hold_vs_height()
    for k, v in p.items():
        print(k, sorted(v))
    print(betaflight())
