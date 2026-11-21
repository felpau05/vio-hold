#!/usr/bin/env python3
"""Step 8 of the rehearsal: every profile combination that EVENT_DAY.txt, EIGHT_HOURS.txt and DECISION_CARD.txt
name, applied with tools/use_profile.py on a copy of settings.json. The real settings.json is only read; its md5 is
printed before and after.

  ~/.venvs/vio/bin/python integ/rehearsal_2026-11-20/profile_combos.py

Part A: each combination on a fresh copy of settings.json (as on the first apply of the day).
Part B: each DECISION_CARD apply line on a copy that is already set up for the Justice D and calibrated
        (as later in the day). It checks that the camera, the calibration and the other profiles stay.
Part C: each key that two profiles set to different values.
"""
import glob
import hashlib
import itertools
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, PROJ)
sys.path.insert(0, os.path.join(PROJ, "tools"))
import settings as S        # noqa: E402
import use_profile as U     # noqa: E402

REAL = os.path.join(PROJ, "settings.json")
TMP = tempfile.mkdtemp()


def md5(p):
    return hashlib.md5(open(p, "rb").read()).hexdigest()


def apply(names, path):
    """The operator's command, as EIGHT_HOURS hour 2 writes it (system python3 on the laptop)."""
    r = subprocess.run(["python3", "tools/use_profile.py", "apply", *names, "--settings", path], cwd=PROJ,
                       capture_output=True, text=True)
    return r.returncode, (r.stdout + r.stderr).strip()


def fresh(name):
    p = os.path.join(TMP, name + ".json")
    shutil.copyfile(REAL, p)
    return p


def main():
    print(f"settings.json md5 before: {md5(REAL)}")
    # ---------------------------------------------------------------- A
    combos = [["outdoor_day", "goal_a", "px4", "justice_d"]]                  # EVENT_DAY step 0 example
    combos += [[e, g, f, "justice_d"] for e in ("indoor", "outdoor_day", "outdoor_night")
               for g in ("goal_a", "goal_b") for f in ("px4", "ardupilot")]   # EIGHT_HOURS hour 2: 12
    extra = ["recovery_demo", "fresh_kf", "kf_bright", "gap_jump", "drag_fusion"]   # EVENT_DAY optional sections
    combos += [["outdoor_day", "goal_b", "ardupilot", "justice_d", x] for x in extra]
    combos += [["outdoor_day", "goal_b", "ardupilot", "justice_d", "kf_bright", "gap_jump", "recovery_demo"]]
    combos += [["outdoor_day", "goal_b", "ardupilot", "justice_d", "no_sd"],          # new: board with no SD card
               ["outdoor_day", "goal_b", "px4", "justice_d", "no_sd", "recovery_demo"]]
    print("\nA. combinations on a fresh copy of settings.json")
    fails = 0
    for names in combos:
        p = fresh("a")
        code, out = apply(names, p)
        s = S.load(p) if code == 0 else None
        ok = code == 0 and s["camera"] == "digital" and s["video_device"] == "/dev/video2"
        fails += not ok
        print(f"  {'OK  ' if ok else 'FAIL'} {' '.join(names)}" + ("" if ok else f"   exit {code}: {out.splitlines()[-1]}"))
    print(f"  {len(combos)} combinations, {fails} FAIL")

    # ---------------------------------------------------------------- B
    base = ["outdoor_day", "goal_b", "ardupilot", "justice_d"]
    cal = dict(calibrated=True, calib_capture="digital 1280x720@60 reduce=2 640x360", video_delay_s=0.0423,
               readout_s=0.0141, mask_file="mask.png", origin_lat_lon_alt=[45.4, -75.7, 70.0],
               lens_measured=dict(by="hand turn", image_size=[1280, 720], fx=638.1, fy=641.9, cx=639.5, cy=359.5,
                                  k1=-0.008, k2=0.0, k3=0.0, k4=0.0, note="rehearsal"))
    rows = [("row 13 (as written: add px4 to the apply line)", base[:2] + ["px4", "justice_d"]),
            ("row 14 (as written: indoor goal_b ardupilot)", ["indoor", "goal_b", "ardupilot"]),
            ("row 14 (corrected: + justice_d)", ["indoor", "goal_b", "ardupilot", "justice_d"]),
            ("row 15 (as written: apply outdoor_night)", ["outdoor_night"]),
            ("row 15 (corrected: outdoor_night goal_b ardupilot justice_d)", ["outdoor_night", "goal_b", "ardupilot", "justice_d"]),
            ("row 20 (apply <profile names> justice_d)", base),
            ("row 36 (add kf_bright)", base + ["kf_bright"]),
            ("row 37 (again without recovery_demo)", base)]
    print("\nB. DECISION_CARD apply lines on a calibrated Justice D copy (before: " + " ".join(base) + ", calibrated)")
    for label, names in rows:
        p = fresh("b")
        assert apply(base, p)[0] == 0
        s = S.load(p)
        s.update(cal)
        S.save(s, p)
        code, out = apply(names, p)
        t = S.load(p)
        lost = [k for k in cal if t[k] != cal[k]]
        problems = []
        if t["camera"] != "digital":
            problems.append(f"camera -> {t['camera']}, video_device -> {t['video_device']}")
        if lost:
            problems.append("calibration values changed: " + ", ".join(f"{k} {cal[k]} -> {t[k]}" for k in lost))
        if t["min_height_m"] > 3.0 and "indoor" not in names:
            problems.append(f"min_height_m -> {t['min_height_m']} (no outdoor profile)")
        print(f"  {'OK  ' if code == 0 and not problems else 'FAIL'} {label}: exit {code}"
              + ("" if not problems else "; " + "; ".join(problems)))

    # ---------------------------------------------------------------- C
    print("\nC. keys that two profiles set to different values")
    prof = {os.path.basename(f)[:-5]: U.leaves(U.read(f, overlay=True))
            for f in sorted(glob.glob(os.path.join(PROJ, "tools", "profiles", "*.json")))}
    n = 0
    for a, b in itertools.combinations(sorted(prof), 2):
        for k in sorted(set(prof[a]) & set(prof[b])):
            if prof[a][k] != prof[b][k]:
                n += 1
                print(f"  {k}: {a} {json.dumps(prof[a][k])}, {b} {json.dumps(prof[b][k])}")
    print(f"  {n} keys. Same key, same value (no conflict): " + ", ".join(
        f"{k} ({a}, {b})" for a, b in itertools.combinations(sorted(prof), 2)
        for k in sorted(set(prof[a]) & set(prof[b])) if prof[a][k] == prof[b][k]))
    # ---------------------------------------------------------------- D
    print("\nD. no_sd (new): the order of the names, and the disk space that the check line 'free disk' needs")
    for names in (base + ["no_sd"], base[:3] + ["no_sd", "justice_d"], base):
        p = fresh("d")
        code, _ = apply(names, p)
        s = S.load(p)
        k = s["video_reduce"]
        rate = min(s["video_fps"], s["record_hz"]) if s["record_hz"] > 0 else s["video_fps"]
        mbs = (s["video_size"][0] // k) * (s["video_size"][1] // k) * rate / 1e6
        need = s["min_free_gb"] * mbs / (360 * 288 * 25 / 1e6)          # as apb_main.mode_check
        print(f"  {' '.join(names)}: exit {code}, record_hz {s['record_hz']}, min_free_gb {s['min_free_gb']}, log_dir "
              f"{s['log_dir']}; raw recording {mbs:.1f} MB/s; check needs {need:.2f} GB free; the recorder stops under "
              f"{s['min_free_gb']} GB")
    print(f"\nsettings.json md5 after:  {md5(REAL)}")
    shutil.rmtree(TMP)


if __name__ == "__main__":
    main()
