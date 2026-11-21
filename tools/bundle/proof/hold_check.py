"""Hold.process on synthetic pictures: a 50 m hover for 4 s at 30 frames/s (sim_test.py: rolling shutter, noise).
It uses each OpenCV function of hold.py that a hover reaches. Prints the fix count and the error against the truth.
No times: an emulated time is not the board's speed. Run in a folder with hold.py and sim_test.py:
  python3 <this file>
"""
import os
import sys

sys.path.insert(0, os.getcwd())
import numpy as np      # noqa: E402
import sim_test as S    # noqa: E402

pos, att_fn, h_fn = S.hover(50.0)
T = 4.0
frames = [(t0, S.render(t0, pos, att_fn, h_fn, 8 * 50.0)) for t0 in np.arange(0, T, 1 / S.FPS)]
att, hgt = S.fc_feeds(att_fn, h_fn, T)
out = S.run(frames, att, hgt, S.READOUT, pos)
err = np.array([r[2] for r in out], float)
ok = ~np.isnan(err)
rms = float(np.sqrt(np.mean(err[ok] ** 2))) if ok.any() else float("nan")
print(f"frames {len(out)}, fixes {int(ok.sum())}, states {sorted(set(r[1] for r in out))}")
print(f"error against the truth: rms {rms:.4f} m, max {np.nanmax(err):.4f} m")
print("error of the last 5 fixes (m): " + " ".join(f"{e:.5f}" for e in err[ok][-5:]))
print("PASS" if ok.sum() >= len(out) // 2 and rms < 1.0 else "FAIL")   # limit: 1 m at 50 m (2 % of the height)
