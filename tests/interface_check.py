#!/usr/bin/env python3
"""Interface contract of hold.py version 2: Est fields, attitude with gyro rates, reanchor(pos, var)
with a trace or a 2x2 array, a negative readout, unknown settings refused, make_R_bc() == R_BC_NADIR.
Run: ~/.venvs/vio/bin/python tests/interface_check.py -> tests/interface_check.txt
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np          # noqa: E402

import harness as X         # noqa: E402
import hold as HM           # noqa: E402

out = []
assert HM.Est._fields == ("t", "pos", "var", "n", "state", "recovered", "why", "cov", "yaw", "scale")
assert np.allclose(HM.make_R_bc(), HM.R_BC_NADIR)
try:
    HM.Hold(X.S.K, X.S.D, X.S.R_BC_NADIR, 0.016, no_such_setting=1)
    raise SystemExit("unknown setting accepted")
except ValueError:
    pass
frames, att, hgt, pos = X.scene(h0=100, T=4.0, seed=1)
rates = HM.Series(1e9)            # (t, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)
a = att.array()
d = np.gradient(a[:, 1:4], a[:, 0], axis=0)
for row, w in zip(a, d):
    rates.add(row[0], *row[1:4], *w)
for name, kw, att_s, var in (("attitude with gyro rates, var = trace", {}, rates, 0.5),
                             ("2x2 reanchor variance", {}, att, np.diag([0.2, 0.3])),
                             ("negative readout runs (-16 ms; this sim reads top first, so the model is wrong on purpose; G6 accuracy: regression.txt)", dict(neg=True), att, None)):
    hold = HM.Hold(X.S.K, X.S.D, X.S.R_BC_NADIR, -0.016 if kw else 0.016)
    hold.reanchor(pos(np.array([0.0]))[0], var)
    sent, err = 0, []
    for t0, img in frames:
        e = hold.process(img, t0 + X.S.FRAME_T_ERR + (0.016 * 239 / 240 if kw else 0.0), att_s, hgt)
        if e.var is not None:
            assert abs(e.var - np.trace(e.cov)) < 1e-9 and e.cov.shape == (2, 2) and e.yaw is not None and e.scale is not None
            sent += 1
            err.append(np.linalg.norm(e.pos - pos(np.array([e.t]))[0]))
    out.append(f"{name}: fixes {sent}/{len(frames)}, RMS {np.sqrt(np.mean(np.square(err))):.2f} m, last yaw {e.yaw:+.4f} rad, "
               f"last scale {e.scale:.3f}, hold.rate {hold.rate:.2f} rad/s")
out.append("Est fields, make_R_bc() == R_BC_NADIR, unknown setting refused: OK")
print("\n".join(out))
open(os.path.join(X.HERE, "interface_check.txt"), "w").write("\n".join(out) + "\n")
