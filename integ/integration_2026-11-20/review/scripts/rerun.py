"""Run timing on every case folder with the apb_main.py of a given folder. Fresh flight settings for each case.
Usage: rerun.py APB_DIR CASES OUT_TXT"""
import glob, json, os, re, shutil, subprocess, sys, tempfile
P = os.environ["PROJECT"]
PY = os.path.expanduser("~/.venvs/vio/bin/python")
apb, cases, out = (os.path.abspath(x) for x in sys.argv[1:4])
tmp = tempfile.mkdtemp()
with open(out, "w") as fo:
    for d in sorted(glob.glob(os.path.join(cases, "*"))):
        name = os.path.basename(d)
        h = re.match(r"e_log(\d+)m", name) or re.search(r"_(\d+)m$", name)
        s = os.path.join(tmp, "settings.json")
        shutil.copy(f"{P}/apsim/runs/reh_jd_wig_{h[1]}m_s31_settings.json", s)
        files = sorted(glob.glob(os.path.join(d, "run_*.csv"))) + sorted(glob.glob(os.path.join(d, "raw_*.npz")))
        p = subprocess.run(["nice", "-n", "5", PY, "apb_main.py", "timing", *files, "--settings", s], cwd=apb,
                           capture_output=True, text=True)
        a = json.load(open(s))
        lines = [x for x in (p.stdout + p.stderr).splitlines() if x.startswith(("PASS video", "FAIL", "INFO"))
                 or "Error" in x]
        res = next((x for x in lines if x.startswith(("PASS video", "FAIL")) or "Error" in x), "?")
        info = next((x for x in lines if x.startswith("INFO")), "")
        res = res.replace(d + "/", "")
        fo.write(f"{name:18s} exit {p.returncode}  written {a['video_delay_s']*1e3:3.0f} ms {a['readout_s']*1e3:+3.0f} ms | "
                 f"{res[:170]}{' | ' + info[5:90] if info else ''}\n")
        fo.flush()
print(open(out).read())
