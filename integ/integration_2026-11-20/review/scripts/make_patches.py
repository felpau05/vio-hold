import os
import sys
R = os.environ["WORK"]

def sub(src, old, new, path):
    assert src.count(old) == 1, (path, old[:60], src.count(old))
    return src.replace(old, new)

# ---- B1: choose the switch whose 15 s window is a clean wiggle window ----
p = f"{R}/mut4/p1/apb_main.py"
s = open(p).read()
s = sub(s, '''    t, sw = math.nan, []
    with open(logs[0]) as f:
        for row in csv.DictReader(f):
            t = float(row["t"]) if row["t"] else t       # a row with no frame has no t: use the last frame's
            if (row["note"] or "").startswith("engaged"):
                sw.append(t)
''', '''    t, sw, rows = math.nan, [], []
    with open(logs[0]) as f:
        for row in csv.DictReader(f):
            t = float(row["t"]) if row["t"] else t       # a row with no frame has no t: use the last frame's
            if row["t"]:
                rows.append((t, row.get("mode") or "", float(row["height"]) if row.get("height") else math.nan))
            if (row["note"] or "").startswith("engaged"):
                sw.append(t)
''', p)
s = sub(s, '''    d = [np.load(f) for f in raws]
    tg = [x["t_grab"] for x in d]
    keep = [(x >= sw[0] - 16.0) & (x <= sw[0] - 1.0) for x in tg]
    if not any(k.any() for k in keep):
        say(f"FAIL no raw frame in the 15 s before the switch at t {sw[0]:.1f} s''', '''    # The wiggle switch: the first "engaged" note whose 15 s before are in the log, in the air and in no hold mode.
    # A switch test on the ground, or the FC already in Loiter when the program starts, gives an earlier note.
    held = AP_HOLD | {"POSCTL", "AUTO_LOITER"}

    def why_not(x):
        w = [r[0] for r in rows if x - 16.0 <= r[0] <= x - 1.0]
        if not w or max(np.diff([x - 16.0] + w + [x - 1.0])) > 1.0:
            return "the run log has no frames for part of the 15 s before it"
        if any(m in held for u, m, _ in rows if x - 16.0 <= u <= x - 1.0):
            return "position hold in the 15 s before it"
        if any(h < s["min_height_m"] for u, _, h in rows if x - 16.0 <= u <= x - 1.0):   # empty height: not checked
            return f"under {s['min_height_m']:g} m (on the ground) in the 15 s before it"
        return None
    why = [why_not(x) for x in sw]
    ok = [x for x, y in zip(sw, why) if y is None]
    if not ok:
        say("FAIL no switch to position hold after 15 s in the air in Altitude mode: " + "; ".join(
            f"switch at t {x:.1f} s: {y}" for x, y in zip(sw, why)) + ". Settings not changed. Keep the delay and "
            "readout of the ground calibration.")
        return 1
    sw0 = ok[0]
    d = [np.load(f) for f in raws]
    tg = [x["t_grab"] for x in d]
    keep = [(x >= sw0 - 16.0) & (x <= sw0 - 1.0) for x in tg]
    if not any(k.any() for k in keep):
        say(f"FAIL no raw frame in the 15 s before the switch at t {sw0:.1f} s''', p)
s = sub(s, '''    say(f"INFO {len(rec['t_grab'])} frames from 16 s to 1 s before the switch at t {sw[0]:.1f} s"
        + (f" (the first of {len(sw)} switches in the log)" if len(sw) > 1 else ""))''',
    '''    say(f"INFO {len(rec['t_grab'])} frames from 16 s to 1 s before the switch at t {sw0:.1f} s"
        + ("" if len(sw) == 1 else f" (the first of {len(sw)} switches in the log)" if sw0 == sw[0] else
           f" (switch {sw.index(sw0) + 1} of {len(sw)} in the log; not used: " + "; ".join(
               f"t {x:.1f} s, {y}" for x, y in zip(sw, why) if y) + ")"))''', p)
open(p, "w").write(s)

# ---- B2 (on top of B1): the window must hold the wiggle ----
p2 = f"{R}/mut4/p2/apb_main.py"
s = sub(s, '''    r = calib.Rec(rec, load_mask(s, rec["imgs"].shape[1:3]))
    if r.rate_dps < s["calib_min_rate_dps"]''', '''    # The window must hold the wiggle (item 8b: still, 8 s of rocking, settle): its 8 s with the most turning turn
    # at least 3 times faster (RMS) than the rest of the window. Turning all the time (a pilot holding position in
    # gusts, a hold) passed calib's checks with wrong values. assumed: 3, set on the 4 rehearsal flights (no
    # hold-out data): flown wiggle windows 4.4 to 8.7, windows with no wiggle 1.0 to 1.5.
    a = rec["att"]
    tt = np.arange(max(sw0 - 16.0, a[0, 0]), min(sw0 - 1.0, a[-1, 0]), 0.1)
    if len(tt) < 100:
        say(f"FAIL the FC attitude of the raw files covers {len(tt) / 10:.0f} s of the 15 s before the switch at t "
            f"{sw0:.1f} s (need 10 s). Settings not changed. Keep the delay and readout of the ground calibration.")
        return 1
    ang = np.c_[[np.interp(tt, a[:, 0], c) for c in (a[:, 1], a[:, 2], np.unwrap(a[:, 3]))]].T
    e = np.sum((np.diff(ang, axis=0) / 0.1) ** 2, axis=1)
    c = np.convolve(e, np.ones(80), "valid")
    i = int(np.argmax(c))
    r_in, r_out = math.degrees(math.sqrt(c[i] / 80)), math.degrees(math.sqrt(np.r_[e[:i], e[i + 80:]].mean()))
    if r_in < 3.0 * r_out:
        say(f"FAIL no wiggle found in the 15 s before the switch at t {sw0:.1f} s: its 8 s with the most turning "
            f"turn {r_in:.0f} deg/s RMS, the rest {r_out:.0f} deg/s (need 3 times). Settings not changed. Keep the "
            "delay and readout of the ground calibration.")
        return 1
    r = calib.Rec(rec, load_mask(s, rec["imgs"].shape[1:3]))
    if r.rate_dps < s["calib_min_rate_dps"]''', p2)
open(p2, "w").write(s)
print("patched")
