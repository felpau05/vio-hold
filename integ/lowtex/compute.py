#!/usr/bin/env python3
"""ms per frame on this laptop with cv2.setNumThreads(1): the whole-picture path alone (Hold._dense) and the
whole Pipe step (diag.py's step_ms_mean), on open-loop replays. Run alone (nothing else on the CPU).
Run: ~/.venvs/vio/bin/python integ/lowtex/compute.py > integ/lowtex/compute.txt"""
import json
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [PROJ, os.path.join(PROJ, "integ", "avail")]
cv2.setNumThreads(1)
import diag  # noqa: E402
import hold  # noqa: E402

ms = []
_d = hold.Hold._dense


def timed(self, *a):
    t = time.perf_counter()
    r = _d(self, *a)
    ms.append(1e3 * (time.perf_counter() - t))
    return r


hold.Hold._dense = timed
RUN = os.path.join(PROJ, "px4sim", "runs", "apb_100m_w20g40.npz")
for g, case in (("ottawa2019fall", None), ("prairie", None), ("holdout_field", None),
                ("night_lights", {"noise": 10, "exposure_s": 0.02})):
    ms.clear()
    s = diag.main(RUN, g, 56.2, 66.0, case=case)
    print(json.dumps(dict(ground=g, step_ms_mean=s["step_ms_mean"], dense_calls=len(ms),
                          dense_ms_mean=round(float(np.mean(ms)), 2) if ms else None,
                          dense_ms_p95=round(float(np.percentile(ms, 95)), 2) if ms else None)), flush=True)
