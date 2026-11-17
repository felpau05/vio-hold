#!/usr/bin/env python3
"""Writes apsim/profile_goal_a.json and apsim/profile_goal_b.json from the apsim results (apsim/results.json).

Each file is a settings overlay in the format of tools/use_profile.py (settings keys; keys that start with
"_" are notes and never written). The numbers in the notes are RMS / max drift in metres, middle and worst
of the seeds, from the runs named. Parameter names were read from the built ArduCopter 4.6 SITL firmware
(integ/control/ap_param_names.txt, from its own dataflash PARM log).
"""
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
R = json.load(open(os.path.join(PROJ, "apsim", "results.json")))


def stat(prefix):
    rows = [R[k] for k in sorted(R) if k.startswith(prefix) and isinstance(R[k].get("rms"), (int, float))]
    if not rows:
        return None
    rm, mx = np.array([r["rms"] for r in rows]), np.array([r["max"] for r in rows])
    tw = np.array([np.hypot(r["roll_rms_deg"], r["pitch_rms_deg"]) for r in rows])
    return dict(runs=len(rows), rms_middle=round(float(np.median(rm)), 2), rms_worst=round(float(rm.max()), 2),
                max_middle=round(float(np.median(mx)), 2), max_worst=round(float(mx.max()), 2),
                tilt_wobble_deg_middle=round(float(np.median(tw)), 2),
                each=[f"{k}: {R[k]['rms']:.2f} / {R[k]['max']:.2f}" for k in sorted(R)
                      if k.startswith(prefix) and isinstance(R[k].get("rms"), (int, float))])


COMMON = {
    "_firmware": "ArduCopter 4.6 (Orqa mil-mavl branch, SITL quad model). Names checked against the built firmware.",
    "_not_final": "Simulator gains for the default SITL quad, not the 2.3 kg 10-inch drone. Use as a starting point "
                  "and the direction of each effect; check on the real drone with a short hover first.",
    "_set_on_the_day": "VISO_DELAY_MS from 'apb_main.py delay <run log>' after the first hover (140 in apsim); "
                       "video_delay_s and readout_s from 'apb_main.py timing <raw chunks of the stick wiggle>'.",
    "send_speed": False,
    "cov_scale": 1.0,
}
MEASURED = {
    "VISO_DELAY_MS 100 -> 140 (measured)": {"before": stat("c_base_100m"), "after": stat("c_d140_100m"),
                                           "data": "VALIDATION seeds 11-13, ottawa2019fall, 100 m w20g40"},
    "trust: cov_scale 0.1 + VISO_POS_M_NSE 0.15 (dropped)": {"with": stat("c_trust_100m"), "without": stat("c_d140_100m"),
                                                         "data": "VALIDATION seeds 11-13, 100 m w20g40"},
    "speed from fix differences (dropped)": {"100m_v1": stat("c_spd_100m"), "100m_v2": stat("c_spd2_100m"),
                                             "40m_v2": stat("c_spd2_40m"), "without_100m": stat("c_d140_100m"),
                                             "without_40m": stat("c_d140_40m"), "data": "VALIDATION seeds 11-13"},
    "wind assist v2 (experiment, NOT in a profile; settings wind_assist)": {
        "100m": stat("c_wa2_100m"), "100m_loiter": stat("c_d140_100m"), "60m": stat("c_wa2_60m"),
        "60m_loiter": stat("c_d140_60m"), "20m": stat("c_wa2_20m"), "20m_loiter": stat("c_d140_20m"),
        "data": "VALIDATION seeds 11-13, stock gains, pilot selects GUIDED"},
    "PSC_VELXY_I 2 + PSC_POSXY_P 1.5": {"100m": stat("c_i2p15_100m"), "100m_stock": stat("c_d140_100m"),
                                        "40m": stat("c_i2p15_40m"), "40m_stock": stat("c_d140_40m"),
                                        "data": "VALIDATION seeds 11-13, 100 and 40 m w20g40"},
}

a = dict(COMMON)
a.update({
    "_goal": "A: robust at height in wind (tested at 100 m, wind 20 gusting 40 km/h).",
    "_choice": "Stock ArduCopter position-controller gains. PSC_VELXY_I 2 + PSC_POSXY_P 1.5 changed the 100 m drift "
               "by less than the spread between seeds (not a gain), so it is not in this profile.",
    "params_ardupilot": {"EK3_SRC1_VELXY": 0, "PSC_POSXY_P": 1.0, "PSC_VELXY_I": 1.0},
    "_measured": MEASURED,
    "_holdout_this_profile": {"100m_w20g40": stat("c_hoD_100m"),
                              "data": "HOLD-OUT sites holdout_lawn s104, holdout_quarry s105, holdout_water s106"},
})
b = dict(COMMON)
b.update({
    "_goal": "B: tightest hold at 5 to 40 m (tested at 40 m and, on hold-out ground, 30 m; wind 20 gusting 40 km/h).",
    "_choice": "Faster integral and position gain: PSC_VELXY_I 1 -> 2, PSC_POSXY_P 1 -> 1.5. At 40 m it cut the drift "
               "on every seed; at 100 m it did nothing measurable. Not tested below 30 m or indoors.",
    "params_ardupilot": {"EK3_SRC1_VELXY": 0, "PSC_POSXY_P": 1.5, "PSC_VELXY_I": 2.0},
    "_measured": MEASURED,
    "_holdout_this_profile": {"30m_w20g40": stat("c_hoB_30m"), "30m_w20g40_stock_gains": stat("c_hoD_30m"),
                              "data": "HOLD-OUT sites holdout_lawn s107, holdout_quarry s108, holdout_water s109"},
    "_px4_untested": "Same direction on PX4: MPC_XY_VEL_I_ACC (default 0.4) and MPC_XY_P (default 0.95) up "
                     "(research/papers_control.md S3). Not run.",
})
for name, d in (("profile_goal_a.json", a), ("profile_goal_b.json", b)):
    json.dump(d, open(os.path.join(PROJ, "apsim", name), "w"), indent=1)
    print(name, json.dumps(d.get("_holdout_this_profile"))[:400])
