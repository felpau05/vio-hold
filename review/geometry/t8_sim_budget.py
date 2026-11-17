#!/usr/bin/env python3
"""Where the known 300 m sim error comes from: sim_test's own setup, with its injected errors removed
one at a time (net timing error FRAME_T_ERR - ATT_LAG = -1 ms, ATT_BIAS roll/pitch = 0.5/-0.3 deg with a +/-10 deg
yaw swing). Uses the project's sim_test as-is (this is not the independent test).

Run:  ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t8_sim_budget.py
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
import sim_test as S        # noqa: E402

h0, T = 300, 20.0
pos, att_fn, h_fn = S.hover(h0)
frames = [(t0, S.render(t0, pos, att_fn, h_fn, 8 * h0)) for t0 in np.arange(0, T, 1 / S.FPS)]
orig_bias, orig_terr = S.ATT_BIAS.copy(), S.FRAME_T_ERR
# Net timing error = FRAME_T_ERR - ATT_LAG (attitude arrives 3 ms late): shipped -1 ms.
for label, bias, terr in (("as shipped (net timing -1 ms, bias .5/-.3/1)", orig_bias, orig_terr),
                          ("net timing 0 (FRAME_T_ERR = ATT_LAG)", orig_bias, S.ATT_LAG),
                          ("net timing -3 ms (FRAME_T_ERR = 0)", orig_bias, 0.0),
                          ("roll/pitch bias = 0 (yaw bias kept)", np.r_[0.0, 0.0, orig_bias[2]], orig_terr),
                          ("net timing 0 and roll/pitch bias 0", np.r_[0.0, 0.0, orig_bias[2]], S.ATT_LAG)):
    S.ATT_BIAS, S.FRAME_T_ERR = bias, terr
    S.rng = np.random.default_rng(1)
    att, hgt = S.fc_feeds(att_fn, h_fn, T)
    out = np.array([r[2] for r in S.run(frames, att, hgt, S.READOUT, pos)], float)
    ok = ~np.isnan(out)
    print(f"  sim_test hover {h0} m, {label:42s} rms {math.sqrt(np.mean(out[ok] ** 2)):.2f} m")
