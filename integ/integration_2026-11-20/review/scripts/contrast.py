"""For each case folder: the switch the new code uses (first 'engaged' note), then in its window [sw-16, sw-1]:
the RMS turn rate of the 8 s part with the most turning, and of the rest of the window (calib.Rec's rate: roll,
pitch, yaw on a 0.1 s grid, from the FC attitude of the given raw files)."""
import csv, glob, os, sys
import numpy as np

def rate(att, a, b):
    tt = np.arange(a, b, 0.1)
    yaw = np.unwrap(att[:, 3])
    ang = np.c_[[np.interp(tt, att[:, 0], c) for c in (att[:, 1], att[:, 2], yaw)]].T
    w = np.diff(ang, axis=0) / 0.1
    return w

for d in sorted(glob.glob(os.path.join(sys.argv[1], "*"))):
    log = glob.glob(os.path.join(d, "run_*.csv"))[0]
    t, sw = np.nan, []
    for r in csv.DictReader(open(log)):
        t = float(r["t"]) if r["t"] else t
        if r["note"].startswith("engaged"):
            sw.append(t)
    if not sw:
        print(f"{os.path.basename(d):22s} no engaged note"); continue
    att = np.concatenate([np.load(f)["att"] for f in sorted(glob.glob(os.path.join(d, "raw_*.npz")))])
    att = att[np.unique(att[:, 0], return_index=True)[1]]
    a, b = max(sw[0] - 16, att[0, 0]), min(sw[0] - 1, att[-1, 0])
    if b - a < 9:
        print(f"{os.path.basename(d):22s} attitude covers {b - a:.1f} s of the window"); continue
    w = rate(att, a, b)                      # one row each 0.1 s
    e = np.sum(w ** 2, axis=1)
    n = 80
    c = np.convolve(e, np.ones(n), "valid")
    i = int(np.argmax(c))
    rin = np.degrees(np.sqrt(c[i] / n))
    rest = np.r_[e[:i], e[i + n:]]
    rout = np.degrees(np.sqrt(rest.mean())) if len(rest) else float("nan")
    print(f"{os.path.basename(d):22s} window {a:6.1f}..{b:6.1f}  best 8 s {a + i * 0.1:6.1f}..{a + i * 0.1 + 8:6.1f}  "
          f"rate in {rin:5.1f}  rest {rout:5.1f}  ratio {rin / rout:5.1f}")
