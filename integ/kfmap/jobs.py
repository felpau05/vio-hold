#!/usr/bin/env python3
"""Job lists of the open-loop study (integ/kfmap) and a small runner: N workers, nice 10, each job one ol_kfmap.py call.
  ~/.venvs/vio/bin/python integ/kfmap/jobs.py list <set>          print the jobs of a set
  ~/.venvs/vio/bin/python integ/kfmap/jobs.py run <set> [workers]  run them (a job whose result exists is skipped)
Sets:
  main    the validation matrix: ottawa2022, farmland x 5, 10 m x seeds 11-13, re-anchor at the last fix (case B),
          versions off, dl (DENSE_LOCAL), both (DENSE_LOCAL + KF_MAP), kfonly (KF_MAP)
  place   another place after a forced loss at 10 s (same photo N m away, or the other photo): every acceptance is false
  bad     bad pictures 12-22 s after a forced loss at 10 s (with --probe), and the same bad pictures without a forced loss
  rep     repeated texture (tile periods 0.3, 0.6, 1.2 m), forced loss at 10 s at the last fix + 0, P/2 or P north
"""
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
PY = os.path.expanduser("~/.venvs/vio/bin/python")
H = "hold.py"
V = {"off": f'off={H}', "dl": f'dl={H}:{{"dense_local": true}}', "both": f'both={H}:{{"dense_local": true, "kf_map": true}}',
     "kfonly": f'kfonly={H}:{{"kf_map": true}}'}
SEEDS = (11, 12, 13)
BAD = ["frozen_bottom", "frozen_top", "overlay_half", "overlay_flat", "static std 5", "static std 30", "static std 60",
       "frozen picture", "frozen + fresh noise", "NO SIGNAL screen", "breakup bursts"]


def jobs(name):
    out = []
    if name == "main":
        for g in ("ottawa2022", "farmland"):
            for h in (5, 10):
                for s in SEEDS:
                    out.append((f"kf_{g}_crop_{h}m_s{s}_main_relast", ["--ground", g, "--height", str(h), "--seed", str(s),
                                "--tag", "_main"], ["off", "dl", "both", "kfonly"]))
    elif name == "place":
        for g, g2 in (("ottawa2022", "farmland"), ("farmland", "ottawa2022")):
            for h, shifts in ((5, (2, 5, 20, 200)), (10, (3, 10, 20, 200))):
                for s in SEEDS:
                    for d in shifts:
                        out.append((f"kf_{g}_crop_{h}m_s{s}_pl{d}_relast",
                                    ["--ground", g, "--height", str(h), "--seed", str(s), "--seconds", "30", "--force", "10",
                                     "--place_from", "10", "--place_shift", f"{d}:0", "--tag", f"_pl{d}"], ["both", "kfonly"]))
                    out.append((f"kf_{g}_crop_{h}m_s{s}_plg_relast",
                                ["--ground", g, "--height", str(h), "--seed", str(s), "--seconds", "30", "--force", "10",
                                 "--place_from", "10", "--place_ground", g2, "--tag", "_plg"], ["both", "kfonly"]))
    elif name == "bad":
        for h in (5, 10):
            for s in SEEDS:
                for k, kind in enumerate(BAD):
                    common = ["--ground", "ottawa2022", "--height", str(h), "--seed", str(s), "--seconds", "30", "--bad", kind,
                              "--bad_from", "12", "--bad_to", "22"]
                    out.append((f"kf_ottawa2022_crop_{h}m_s{s}_bf{k}_relast", common + ["--force", "10", "--probe", "--tag", f"_bf{k}"],
                                ["both", "kfonly"]))
                    out.append((f"kf_ottawa2022_crop_{h}m_s{s}_bn{k}_relast", common + ["--tag", f"_bn{k}"],
                                ["off", "dl", "both", "kfonly"]))
    elif name == "rep":
        for P in (0.3, 0.6, 1.2):
            for h in (5, 10):
                for s in SEEDS:
                    for j, f in enumerate((0.0, 0.5, 1.0)):
                        out.append((f"kf_rep{P:g}_crop_{h}m_s{s}_r{j}_relast",
                                    ["--ground", f"rep{P:g}", "--height", str(h), "--seed", str(s), "--seconds", "30",
                                     "--force", f"10:{f * P:g}:0", "--tag", f"_r{j}"], ["dl", "both", "kfonly"]))
    else:
        raise SystemExit(f"unknown set {name}")
    return out


def cmd(job):
    label, args, vers = job
    c = [PY, os.path.join(HERE, "ol_kfmap.py")] + args
    for v in vers:
        c += ["--hold", V[v]]
    return c


def run_one(job):
    label, _, vers = job
    if all(os.path.exists(os.path.join(HERE, "results", "ol", f"{label}_{v}.json.gz")) for v in vers):
        return f"skip {label}"
    env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")
    r = subprocess.run(["nice", "-n", "10"] + cmd(job), cwd=PROJ, capture_output=True, text=True, env=env)
    open(os.path.join(HERE, "results", "jobs.log"), "a").write(f"== {label} rc {r.returncode}\n{r.stdout}{r.stderr[-2000:]}\n")
    return f"{label} rc {r.returncode}"


if __name__ == "__main__":
    what, name = sys.argv[1], sys.argv[2]
    js = jobs(name)
    if what == "list":
        for j in js:
            print(" ".join(cmd(j)))
        print(len(js), "jobs")
    else:
        n = int(sys.argv[3]) if len(sys.argv) > 3 else 4
        with ThreadPoolExecutor(n) as ex:
            for x in ex.map(run_one, js):
                print(x, flush=True)
