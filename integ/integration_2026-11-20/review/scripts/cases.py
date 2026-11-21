"""Build timing cases from the rehearsal recordings (copies and links only) and run apb_main.py timing on each.
Usage: cases.py WORK OUT [name-prefix ...]"""
import csv, glob, json, os, shutil, subprocess, sys, time
import numpy as np

P = os.environ["PROJECT"]
PY = os.path.expanduser("~/.venvs/vio/bin/python")
WORK, OUT = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
ONLY = sys.argv[3:]
H = (10, 30, 60, 100)
LAB = {h: f"reh_jd_wig_{h}m_s31" for h in H}


def raws(h):
    return sorted(glob.glob(f"{P}/integ/rehearsal_2026-11-20/runs/{LAB[h]}_raw/raw_*.npz"))


def stamp_of(h):
    return os.path.basename(raws(h)[0])[4:19]


def shift_stamp(st, ds):
    t = time.mktime(time.strptime(st, "%Y%m%d_%H%M%S")) + ds
    return time.strftime("%Y%m%d_%H%M%S", time.localtime(t))


def rows_of(h):
    with open(f"{P}/apsim/runs/{LAB[h]}_onboard.csv") as f:
        return list(csv.DictReader(f))


def meta(h):
    js = json.load(open(f"{P}/apsim/runs/{LAB[h]}.json"))
    rows = rows_of(h)
    sw = next(float(r["t"]) for r in rows if r["note"].startswith("engaged"))
    return dict(wig=js["wiggle_t"], sw=sw, start=float(rows[0]["t"]))


def write_log(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def move_engaged(rows, t_new, extra=()):
    """Remove the flight's own 'engaged' note; put it on the row whose t is nearest t_new. extra: (t, note) to add."""
    rows = [dict(r) for r in rows]
    for r in rows:
        if r["note"].startswith("engaged"):
            r["note"] = ""
    ts = np.array([float(r["t"]) if r["t"] else np.nan for r in rows])
    for t, note in ([(t_new, "engaged: FC mode LOITER")] if t_new is not None else []) + list(extra):
        i = int(np.nanargmin(np.abs(ts - t)))
        rows[i]["note"] = note if not rows[i]["note"] else note + "; " + rows[i]["note"]   # Engage's note is first (_join)
    return rows


def altmode(rows, t_new):
    """The mode column as a pilot holding position in Altitude mode would leave it: ALT_HOLD before t_new."""
    rows = [dict(r) for r in rows]
    t = None
    for r in rows:
        t = float(r["t"]) if r["t"] else t
        r["mode"] = "ALT_HOLD" if t is None or t < t_new - 1e-6 else "LOITER"
    return rows


def case_dir(name):
    d = os.path.join(WORK, name)
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d)
    return d


def settings_copy(h, d):
    s = os.path.join(d, "settings.json")
    shutil.copy(f"{P}/apsim/runs/{LAB[h]}_settings.json", s)
    return s


def link_raws(src_files, d, st):
    out = []
    for f in src_files:
        k = os.path.basename(f)[-7:-4]
        dst = os.path.join(d, f"raw_{st}_{k}.npz")
        os.symlink(f, dst)
        out.append(dst)
    return out


def run(name, d, log, raw_files, s, desc):
    cmd = [PY, "apb_main.py", "timing", log, *raw_files, "--settings", s]
    t1 = time.time()
    p = subprocess.run(["nice", "-n", "5", *cmd], cwd=P, capture_output=True, text=True)
    dt = time.time() - t1
    after = json.load(open(s))
    txt = (f"== {name}: {desc}\n$ apb_main.py timing {os.path.relpath(log, WORK)} "
           f"{' '.join(os.path.relpath(f, WORK) for f in raw_files[:2])}{' ...' if len(raw_files) > 2 else ''} "
           f"({len(raw_files)} raw files)\n{p.stdout}{p.stderr[-1500:]}exit {p.returncode}, {dt:.1f} s, "
           f"settings after: video_delay_s {after['video_delay_s']*1e3:.0f} ms, readout_s {after['readout_s']*1e3:+.0f} ms\n")
    print(txt, flush=True)
    with open(os.path.join(OUT, "cases_all.txt"), "a") as f:
        f.write(txt + "\n")


def want(name):
    return not ONLY or any(name.startswith(x) for x in ONLY)


def base(h):
    name = f"base_{h}m"
    if not want(name):
        return
    d, st = case_dir(name), stamp_of(h)
    log = os.path.join(d, f"run_{st}.csv")
    shutil.copy(f"{P}/apsim/runs/{LAB[h]}_onboard.csv", log)
    run(name, d, log, link_raws(raws(h), d, st), settings_copy(h, d), "the flight as flown (true 40 ms, +14 ms)")


def moved(h, prefix, t_new, desc, extra=(), alt=False):
    name = f"{prefix}_{h}m"
    if not want(name):
        return
    d, st = case_dir(name), stamp_of(h)
    log = os.path.join(d, f"run_{st}.csv")
    rows = move_engaged(rows_of(h), t_new, extra)
    write_log(log, altmode(rows, t_new) if alt else rows)
    run(name, d, log, link_raws(raws(h), d, st), settings_copy(h, d), desc)


def late_start(h, lead):
    """The program (log and recorder) starts `lead` s before the switch: frames and rows before are removed."""
    name = f"c_start{lead}_{h}m"
    if not want(name):
        return
    m = meta(h)
    t_cut = m["sw"] - lead                     # log t (t0); raw t_grab = t0 + 0.070 in these flights
    d, st = case_dir(name), stamp_of(h)
    rows = [dict(r) for r in rows_of(h) if r["t"] == "" or float(r["t"]) >= t_cut]
    rows[0]["note"] = "FC mode ALT_HOLD" + ("; " + rows[0]["note"] if rows[0]["note"] else "")
    log = os.path.join(d, f"run_{st}.csv")
    write_log(log, rows)
    out = []
    for f in raws(h):
        x = np.load(f)
        tg = x["t_grab"]
        if tg[-1] < t_cut + 0.070:
            continue
        if tg[0] >= t_cut + 0.070:
            out += link_raws([f], d, st)
            continue
        k = tg >= t_cut + 0.070
        lo = tg[k][0] - 1.0
        dst = os.path.join(d, f"raw_{st}_{os.path.basename(f)[-7:-4]}.npz")
        cut = lambda a: a[a[:, 0] >= lo] if len(a) else a      # noqa: E731  the new run's FC series start
        np.savez(dst, imgs=x["imgs"][k], t_grab=tg[k], att=cut(x["att"]), hgt=cut(x["hgt"]), lpos=cut(x["lpos"]),
                 K=x["K"], D=x["D"])
        out.append(dst)
    run(name, d, log, out, settings_copy(h, d), f"program started {lead} s before the switch (frames and log rows "
        f"before t {t_cut:.1f} removed)")


def other_run(hx, hy, ds=1):
    name = f"e_log{hx}m_raw{hy}m"
    if not want(name):
        return
    d, st = case_dir(name), stamp_of(hx)
    log = os.path.join(d, f"run_{st}.csv")
    shutil.copy(f"{P}/apsim/runs/{LAB[hx]}_onboard.csv", log)
    run(name, d, log, link_raws(raws(hy), d, shift_stamp(st, ds)), settings_copy(hx, d),
        f"run log of the {hx} m flight; raw files of the {hy} m flight, named with a stamp {ds} s later "
        "(a second run in the same minute, frame clock from boot)")


def drop_chunk(h, k):
    name = f"f_drop{k:03d}_{h}m"
    if not want(name):
        return
    d, st = case_dir(name), stamp_of(h)
    log = os.path.join(d, f"run_{st}.csv")
    shutil.copy(f"{P}/apsim/runs/{LAB[h]}_onboard.csv", log)
    keep = [f for f in raws(h) if not f.endswith(f"_{k:03d}.npz")]
    run(name, d, log, link_raws(keep, d, st), settings_copy(h, d), f"raw chunk {k:03d} absent")


if __name__ == "__main__":
    for h in H:
        base(h)
    for h in H:
        m = meta(h)
        we = m["wig"][1] - 0.070           # wiggle end in log t
        ws = m["wig"][0] - 0.070
        for x in (8, 10, 12, 15, 20, 25, 30, 40):
            moved(h, f"a_still{x}", we + x, f"pilot holds still {x} s after the wiggle, then switches (the "
                  f"flight's own engaged note moved to log t {we + x:.1f})")
        for x in (8, 10, 12, 15, 20, 25, 30, 40):
            moved(h, f"a2_still{x}", we + x, f"as a_still{x}, and the mode column ALT_HOLD before the moved note (a "
                  "pilot who holds position in Altitude mode; the attitude is the hold's)", alt=True)
        moved(h, "b_wig3", ws + 4.0, f"only the first 3 s of the wiggle in the window: switch 1 s after 3 s of "
              f"wiggle (engaged note moved to log t {ws + 4:.1f})")
        for lead in (5, 7, 9, 11, 13):
            late_start(h, lead)
        moved(h, "d_ground", m["sw"], "first 'engaged' from a switch test 1 s after the program start (on the "
              "ground), released 2 s later; the real switch stays",
              extra=[(m["start"] + 1.0, "engaged: FC mode LOITER"), (m["start"] + 3.0, "released: FC mode ALT_HOLD")])
        moved(h, "d_test17", m["sw"], "first 'engaged' from a switch test 17 s after the program start, released "
              "2 s later; the real switch stays",
              extra=[(m["start"] + 17.0, "engaged: FC mode LOITER"), (m["start"] + 19.0, "released: FC mode ALT_HOLD")])
        moved(h, "d_test3", m["sw"], "first 'engaged' from a switch test 3 s after the program start, released "
              "2 s later; the real switch stays",
              extra=[(m["start"] + 3.0, "engaged: FC mode LOITER"), (m["start"] + 5.0, "released: FC mode ALT_HOLD")])
    for hx in H:
        for hy in H:
            if hx != hy:
                other_run(hx, hy)
    for h in H:
        m = meta(h)
        lo, hi = m["sw"] - 16 + 0.07, m["sw"] - 1 + 0.07
        for f in raws(h):
            tg = np.load(f)["t_grab"]
            if tg[-1] >= lo and tg[0] <= hi:
                drop_chunk(h, int(f[-7:-4]))
