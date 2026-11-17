#!/usr/bin/env python3
"""Tune timing, lens and two estimator knobs on one flight recording, without the true position.

  replay_tune.py logs/raw_20261121_141500_*.npz [--settings settings.json] [--jobs 6] [--out DIR] [--margin 0.03]

Every candidate goes through apb_main.replay (the same Pipe, Hold and gyro tilt as the flight) and is scored
from its own per-frame log:
  fix %     frames with a fix the program would send. Higher is better.
  jitter    RMS jump between fixes one frame apart, per axis, in cm. Lower is better. In a hover the true move
            between two frames is small, so this is mostly estimator error; a timing, readout or focal error
            adds height x tilt-change jumps (FINDINGS H1, G2, G6).
  sigma     median sigma per axis that the program sends (its own error estimate; information only)
  n, ms     median tracked points, mean processing time per frame on this laptop (information only)
Ranking: a candidate whose fix % is under 90 % of the stage's best fix % ranks after the others; the rest
rank by jitter.

Search, one setting at a time from the current settings (each stage runs its candidates in parallel):
  1 video_delay_s  current -40 ... +40 ms, step 10     2 readout_s  current, -current, 0 (the sign, G6, H12;
    the delay moves by half the readout change, so the mid-frame time stays: H12)
  3 focal_scale    current x 0.96 ... 1.04, step 0.02  4 hold_cfg.max_pts  default (150) or 120 (E9)
  5 hold_cfg.fb_every  default (1) or 3 (H6)
For delay and focal a parabola through (value, jitter^2) gives the lowest point, and one more replay checks
it. One candidate's jitter is noisy (Hold's keyframe choices differ from run to run); the curve is less so.
When the bowl's lowest point is not inside the grid, the stage runs again around the best value.
A new value is kept only when it beats the current one by --margin (for delay and focal: the curve's
prediction) and lies at least half a grid step away (5 ms, 1 % focal). Closer is under the grid's resolution.
Measured on a 60 s sim recording at 100 m in wind (tools/sim_rec.py, true delay 60.4 ms), from a delay
40, 20, 0, -20 and -40 ms off: 59.0, 54.2, 60.4 (kept), 40.4 (kept: the fit found 56.1 ms, but the predicted
gain was 2.6 %) and 54.2 ms. A delay that is too short raises the jitter less than one that is too long.
calib finds the delay to 1 ms. Use this tool to catch big errors, not the last 10 ms.
Writes DIR/tune.txt (the tables), DIR/tune_patch.json (the changed keys; apply it with use_profile.py merge) and
DIR/best.csv (the log of the final settings, for plot_log.py). DIR defaults to tune/ next to the recording.

The raw chunks are merged into one file first: apb_main.replay feeds the 1 s of attitude that each 10 s chunk
shares with the one before through the gyro filter twice, which bends the tilt at every chunk boundary.
Fly the recording with some motion (wind or small stick inputs). In a still hover a timing error moves nothing,
so nothing can be found (research/papers_tilt.md item 3). A dead-calm result keeps the current values.
"""
import argparse
import csv
import glob
import json
import multiprocessing
import os
import statistics
import sys
import time
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import settings as S        # noqa: E402

FIX_SHARE = 0.9             # assumed: under 90 % of the best fix % means the setting refuses good frames


def merge(files, path):
    """One recording file from raw chunks; attitude, height and position rows each kept once, in time order."""
    L = [np.load(f) for f in files]

    def once(k):
        parts = [x[k] for x in L if k in x and len(x[k])]
        if not parts:
            return np.zeros((0, 3))
        a = np.concatenate(parts)
        return a[np.unique(a[:, 0], return_index=True)[1]]
    order = np.argsort(np.concatenate([x["t_grab"] for x in L]), kind="stable")
    np.savez(path, imgs=np.concatenate([x["imgs"] for x in L])[order], t_grab=np.concatenate([x["t_grab"] for x in L])[order],
             att=once("att"), hgt=once("hgt"), lpos=once("lpos"), K=L[0]["K"], D=L[0]["D"])


def score(log):
    rows = list(csv.DictReader(open(log)))
    t = np.array([float(r["t"]) for r in rows])
    dt = float(np.median(np.diff(t))) if len(t) > 2 else 0.04
    fix = [i for i, r in enumerate(rows) if r["var"]]
    sent = [r for r in rows if r["sent"] == "1"]
    jumps = [(float(rows[i]["north"]) - float(rows[i - 1]["north"]), float(rows[i]["east"]) - float(rows[i - 1]["east"]))
             for i in fix if i > 0 and rows[i - 1]["var"] and t[i] - t[i - 1] < 1.5 * dt and not rows[i]["note"]]
    why = Counter(" ".join(r["why"].split()[:3]) for r in rows if r["sent"] != "1" and r["why"])
    return dict(fix=100.0 * len(sent) / max(len(rows), 1),
                jitter=100 * float(np.sqrt(np.mean(np.square(jumps)))) if jumps else float("inf"),
                sigma=100 * statistics.median(float(r["var"]) ** 0.5 / 2 ** 0.5 for r in sent) if sent else float("nan"),
                n=statistics.median(int(rows[i]["n"]) for i in fix) if fix else 0,
                ms=float(np.mean([float(r["proc_ms"]) for r in rows])),
                why=", ".join(f"{k} ({v})" for k, v in why.most_common(2)))


def run_one(job):
    s, rec, log = job
    os.environ["OMP_NUM_THREADS"] = "1"
    import cv2
    cv2.setNumThreads(1)                      # one core per candidate; the pool runs them side by side
    import apb_main
    apb_main.replay(s, [rec], log_path=log)
    return score(log)


def get(s, key):
    k, _, sub = key.partition(".")
    return s[k].get(sub) if sub else s[k]


def put(s, key, v):
    s = json.loads(json.dumps(s))
    k, _, sub = key.partition(".")
    if not sub:
        s[k] = v
    elif v is None:
        s[k].pop(sub, None)
    else:
        s[k][sub] = v
    return s


def apply(s, key, v):
    """put(), and for readout_s the delay moves by half the readout change: the mid-frame time
    t_grab - video_delay_s + readout_s / 2 stays the same (FINDINGS H12: the data fix that time, not the parts)."""
    out = put(s, key, v)
    if key == "readout_s":
        out["video_delay_s"] = round(s["video_delay_s"] + (v - s["readout_s"]) / 2, 4)
    return out


def stages(s):
    """(key, candidate values around the current value, half a grid step for a fitted curve or None)"""
    r = abs(s["readout_s"]) or S.DEFAULTS["readout_s"]
    return [("video_delay_s", lambda v: [round(v + d / 1000, 4) for d in range(-40, 41, 10)], 0.005),
            ("readout_s", lambda v: [v, -v, 0.0] if v else [0.0, r, -r], None),
            ("focal_scale", lambda v: [round(v * f, 4) for f in (0.96, 0.98, 1.0, 1.02, 1.04)], 0.01 * s["focal_scale"]),
            ("hold_cfg.max_pts", lambda v: [v, 120] if v != 120 else [v, None], None),
            ("hold_cfg.fb_every", lambda v: [v, 3] if v != 3 else [v, None], None)]


def rank_key(r, best_fix):
    return (r["fix"] < FIX_SHARE * best_fix, r["jitter"])


def vertex(res, best_fix):
    """Lowest point of a parabola through (value, jitter^2) of the candidates that keep their fixes, and the
    fitted jitter as a function; None when the curve is not a bowl or its lowest point is outside the grid.
    jitter^2, because the error a wrong value adds and the rest of the noise are independent: they add in squares."""
    ok = [r for r in res if r["fix"] >= FIX_SHARE * best_fix and np.isfinite(r["jitter"])]
    if len(ok) < 4:
        return None
    x, y = np.array([r["value"] for r in ok]), np.array([r["jitter"] for r in ok]) ** 2
    c = np.polyfit(x, y, 2)
    v = -c[1] / (2 * c[0]) if c[0] > 0 else None
    return None if v is None or not x.min() < v < x.max() else (v, lambda z: max(float(np.polyval(c, z)), 1e-9) ** 0.5)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("files", nargs="+", help="raw_*.npz chunks of one flight")
    p.add_argument("--settings", default=S.PATH, help="the settings the flight used")
    p.add_argument("--jobs", type=int, default=min(6, os.cpu_count() or 1))
    p.add_argument("--out", help="output folder (default: tune/ next to the recording)")
    p.add_argument("--margin", type=float, default=0.03, help="assumed: jitter gain a new value needs (0.03 = 3 %%)")
    a = p.parse_args()
    files = sorted(f for pat in a.files for f in glob.glob(pat))
    if not files:
        print("FAIL no recording files")
        return 1
    s0 = S.load(a.settings)
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(files[0])), "tune")
    os.makedirs(out, exist_ok=True)
    rec = os.path.join(out, "merged.npz")
    t_start = time.time()
    merge(files, rec)
    n_frames = len(np.load(rec)["t_grab"])
    report = open(os.path.join(out, "tune.txt"), "w")
    done = {}                                  # (key, value, settings) -> result: no candidate runs twice
    count = [0]

    def say(x=""):
        print(x, flush=True)
        report.write(x + "\n")

    def evaluate(pool, cur, key, vals):
        todo = [v for v in vals if (key, v, json.dumps(cur, sort_keys=True)) not in done]
        jobs = []
        for v in todo:
            count[0] += 1
            jobs.append((apply(cur, key, v), rec, os.path.join(out, f"cand_{count[0]:02d}.csv")))
        for v, j, r in zip(todo, jobs, pool.map(run_one, jobs)):
            done[(key, v, json.dumps(cur, sort_keys=True))] = dict(r, value=v, log=j[2])
        return [done[(key, v, json.dumps(cur, sort_keys=True))] for v in vals]

    def table(res, best_fix):
        say(f"  {'value':>9} {'fix %':>6} {'jitter cm':>9} {'sigma cm':>8} {'n':>4} {'ms':>5}  most refused")
        for r in sorted(res, key=lambda r: rank_key(r, best_fix)):
            v = "default" if r["value"] is None else r["value"]
            say(f"  {v:>9} {r['fix']:6.1f} {r['jitter']:9.2f} {r['sigma']:8.1f} {r['n']:4.0f} {r['ms']:5.1f}  {r['why']}")

    say(f"recording: {len(files)} file(s), {n_frames} frames; settings {a.settings}; {a.jobs} jobs")
    cur = s0
    with multiprocessing.Pool(a.jobs) as pool:
        for key, values, fit in stages(s0):
            v0 = get(cur, key)
            centre = v0
            for _ in range(2):                 # a second grid, around the best value, when the bowl's bottom is not inside
                vals = list(dict.fromkeys(v for v in values(centre) if not (key == "video_delay_s" and v < 0)))
                res = evaluate(pool, cur, key, list(dict.fromkeys([v0] + vals)))
                best_fix = max(r["fix"] for r in res)
                say(f"\n{key} (now {v0 if v0 is not None else 'default'}{'' if centre == v0 else f'; grid around {centre}'})")
                table(res, best_fix)
                best = min(res, key=lambda r: rank_key(r, best_fix))
                vx = vertex([r for r in res if r["value"] in vals], best_fix) if fit else None
                if vx is not None or not fit or best["value"] == centre:
                    break
                centre = best["value"]
            now = next(r for r in res if r["value"] == v0)
            if vx is not None:                 # the fitted bowl's lowest point, checked by one more replay
                v = round(vx[0], 4)
                conf = evaluate(pool, cur, key, [v])[0]
                gain = vx[1](v0) / vx[1](v) - 1 if abs(v - v0) >= fit else 0.0     # closer: under the grid's resolution
                say(f"  fitted curve: lowest at {v}, predicted gain {min(gain, 9.99):.0%}{'+' if gain > 9.99 else ''} over {v0}; that value gives "
                    f"fix {conf['fix']:.1f} %, jitter {conf['jitter']:.2f} cm"
                    + ("" if abs(v - v0) >= fit else f" (within half a grid step of {v0}: no change)"))
                best = conf if conf["fix"] >= FIX_SHARE * best_fix else best
                if best is not conf:
                    gain = now["jitter"] / best["jitter"] - 1
            else:
                gain = now["jitter"] / best["jitter"] - 1
            if best is not now and (now["fix"] < FIX_SHARE * best_fix or gain > a.margin):
                cur, final = apply(cur, key, best["value"]), best
                say(f"  -> {best['value']} (gain {min(gain, 9.99):.0%}{'+' if gain > 9.99 else ''}; fix {best['fix']:.1f} against {now['fix']:.1f} %)"
                    + (f"; video_delay_s -> {cur['video_delay_s']} (same mid-frame time)" if key == "readout_s" else ""))
            else:
                final = now
                say(f"  -> keep {v0 if v0 is not None else 'default'} (gain {gain:.1%} is under {a.margin:.0%})")
    best_log = final["log"]                    # the last stage ran with every earlier choice in place
    patch = {}
    for key in ("video_delay_s", "readout_s", "focal_scale"):
        if cur[key] != s0[key]:
            patch[key] = cur[key]
    if cur["hold_cfg"] != s0["hold_cfg"]:
        patch["hold_cfg"] = {kk: vv for kk, vv in cur["hold_cfg"].items() if s0["hold_cfg"].get(kk) != vv}
        if set(s0["hold_cfg"]) - set(cur["hold_cfg"]):
            say("NOTE a hold_cfg key went back to Hold's default; edit settings.json by hand: "
                f"remove {sorted(set(s0['hold_cfg']) - set(cur['hold_cfg']))} from hold_cfg")
    patch["_why"] = {"_note": f"replay_tune.py on {', '.join(os.path.basename(f) for f in files)}, {time.strftime('%Y-%m-%d %H:%M')}"}
    with open(os.path.join(out, "tune_patch.json"), "w") as f:
        json.dump(patch, f, indent=1)
    os.replace(best_log, os.path.join(out, "best.csv"))
    for f in glob.glob(os.path.join(out, "cand_*.csv")) + [rec]:
        os.remove(f)
    say(f"\nchanged: {json.dumps({k: v for k, v in patch.items() if k != '_why'}) if len(patch) > 1 else 'nothing'}")
    say(f"apply: python3 tools/use_profile.py merge {os.path.join(out, 'tune_patch.json')}   (then tools/deploy.sh)")
    say(f"log of the final settings: {os.path.join(out, 'best.csv')}; done in {time.time() - t_start:.0f} s")
    report.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
