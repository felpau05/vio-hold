#!/usr/bin/env python3
"""Run a list of closed-loop cases; merge them into px4sim/results.json and px4sim/results.txt.

Run:  ~/.venvs/vio/bin/python px4sim/matrix.py <group> [first] [last]   (runs cases[first:last])
      ~/.venvs/vio/bin/python px4sim/matrix.py table                    (rewrites results.txt only)
Groups are below. A run takes 1.5-2 min, so pass a slice to stay under 5 minutes per call.
Variants: truth = vision fed from the truth (best the controller can do); none = no vision (Altitude
mode, what happens without us); ours = Hold with PX4's ATTITUDE as sent; ours_gyro = Hold with the
gyro-propagated attitude (sitl.Link._gyro_att); apb = the onboard program itself (apb_main.run_live,
fc_link, capture, settings; px4sim/onboard.py) on a 10x slower CPU. Groups i1..i8 are the integrator's
matrix (apb against truth); results_sim_builder.* keep the earlier simulator table.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sitl  # noqa: E402

RESULTS, TABLE = os.path.join(HERE, "results.json"), os.path.join(HERE, "results.txt")
VIDEOS = os.path.join(os.path.dirname(HERE), "videos")
VARIANTS = {"truth": dict(variant="truth", speed=2.0), "none": dict(variant="none", speed=2.0),
            "ours": dict(variant="ours"), "ours_gyro": dict(variant="ours", att_source="gyro"),
            "apb": dict(variant="apb")}


def case(var, h, wind, video=False, **kw):
    c = dict(VARIANTS[var], height=float(h), wind=wind, label=f"{var}_{h}m_{wind}", **kw)
    if video:
        c["video_out"] = os.path.join(VIDEOS, f"judge_{c['label']}.mp4")
    if "label_extra" in kw:
        c["label"] += "_" + c.pop("label_extra")
    return c


def ic(var, h, wind, extra="", video=None, **kw):
    """Integrator case: label apb|truth_<h>m_<wind>[_extra]; medium vehicle and ottawa2019fall unless kw."""
    c = dict(VARIANTS[var], height=float(h), wind=wind, label=f"{var}_{h}m_{wind}" + (f"_{extra}" if extra else ""), **kw)
    if video:
        c["video_out"] = os.path.join(VIDEOS, f"judge_{video}.mp4")
    return c


# Smoothness settings (research/px4_vision.md section 6 names them). EKF2_EVP_NOISE is a lower bound on
# our own variance (EKF2_EV_NOISE_MD 0); we send about 0.5 m per axis at 100 m, so only 1.0 changes it.
TUNE = {
    "evp10": {"EKF2_EVP_NOISE": 1.0},
    "xyp06": {"MPC_XY_P": 0.6, "MPC_XY_VEL_P_ACC": 1.4},
    "evp10_xyp06": {"EKF2_EVP_NOISE": 1.0, "MPC_XY_P": 0.6, "MPC_XY_VEL_P_ACC": 1.4},
}

def _calibrated():
    p = os.path.join(os.path.dirname(HERE), "integ", "calib_loop_settings.json")
    return json.load(open(p)) if os.path.exists(p) else {}


GROUPS = {
    # i1: what matters most: 100 and 122 m, medium vehicle, both wind cases, ottawa2019fall
    "i1": [ic("apb", 100, "w20g40"), ic("truth", 100, "w20g40"),
           ic("apb", 122, "w20g40", video="apb_122m_w20g40"), ic("truth", 122, "w20g40"),
           ic("apb", 100, "w30g55"), ic("truth", 100, "w30g55"),
           ic("apb", 122, "w30g55"), ic("truth", 122, "w30g55")],
    "i2": [ic("apb", 100, "calm", video="apb_100m_calm"), ic("truth", 100, "calm"),
           ic("apb", 122, "calm"), ic("truth", 122, "calm")],
    "i3": [ic("apb", 30, "w20g40"), ic("truth", 30, "w20g40"), ic("apb", 60, "w20g40"), ic("truth", 60, "w20g40"),
           ic("apb", 200, "w20g40"), ic("truth", 200, "w20g40")],
    "i4": [ic("apb", 100, "w20g40", "light", vehicle="light"), ic("apb", 100, "w20g40", "heavy", vehicle="heavy"),
           ic("apb", 122, "w30g55", "light", vehicle="light"), ic("apb", 122, "w30g55", "heavy", vehicle="heavy")],
    "i5": [ic("apb", 100, "w20g40", "prairie", ground="prairie"),
           ic("apb", 100, "w20g40", "snow_tarps", ground="snow_tarps")],
    # failure cases at 100 m in wind; times are s after the hold starts
    "i6": [ic("apb", 100, "w20g40", "freeze2s", freeze=(20.0, 2.0)),
           ic("apb", 100, "w20g40", "breakup10", breakup=0.10),   # bursts from the ground up: see results
           # the pre-hold hover in SITL has no wind and is unrealistically still (sitl.py shortcut), so a
           # re-anchor after a burst waits for image motion that never comes; bursts during the hold only:
           ic("apb", 100, "w20g40", "breakup10_hold", breakup=0.10, breakup_from=0.0),
           ic("apb", 100, "w20g40", "outage3s", video="apb_100m_w20g40_outage3s", outage=(20.0, 3.0)),
           ic("apb", 100, "w20g40", "osd_mask", osd=True),
           ic("apb", 100, "w20g40", "outage3s_refind", outage=(20.0, 3.0), reanchor_on_lost=False),
           ic("apb", 100, "w20g40", "breakup10_hold_refind", breakup=0.10, breakup_from=0.0, reanchor_on_lost=False)],
    # calibration in the loop: the camera of integ/calib_loop.py, flown with the settings that
    # apb_main calib wrote (integ/calib_loop_settings.json); video delay 70 ms = exposure/2 + chain
    # second seeds (other gusts, other camera noise) of the headline cases
    "i9": [ic("apb", 122, "w20g40", "seed2", seed=2), ic("apb", 122, "w30g55", "seed2", seed=2),
           ic("apb", 100, "w20g40", "seed2", seed=2), ic("truth", 122, "w30g55", "seed2", seed=2),
           ic("apb", 122, "w20g40", "seed3", seed=3), ic("apb", 122, "w30g55", "seed3", seed=3)],
    "i10": [ic("apb", 100, "w30g55", "seed2", seed=2), ic("apb", 100, "w30g55", "seed3", seed=3),
            ic("apb", 100, "w20g40", "seed3", seed=3)],
    "i8": [ic("apb", 100, "w20g40", "calibrated", cam_top="rear", cam_mirror=True, cam_tilt=(1.0, -1.5),
              cam_twist=1.0, focal_scale=0.95, true_lens="Round 1 independent k2", readout_s=-0.018,
              video_delay_s=0.068, settings=_calibrated())],
    "i7": [ic("apb", 100, w, k, px4=v) for k, v in TUNE.items() for w in ("w20g40", "calm")],

    # deliverable: medium vehicle, Ottawa photo, 60 s, calm then 20 gusting 40, 50 and 100 m
    "first": [case(v, h, w, video=(v.startswith("ours") and h == 100))
              for h in (50, 100) for w in ("calm", "w20g40") for v in ("truth", "ours", "ours_gyro", "none")],
    # extension, 60 s each, medium vehicle unless named; 'ours_gyro' is the configuration that held
    "more": [
        case("ours_gyro", 20, "w20g40"), case("ours_gyro", 122, "w20g40"), case("ours", 122, "w20g40"),
        case("truth", 122, "w20g40"),
        case("ours_gyro", 100, "w20g40", vehicle="light", label_extra="light"),
        case("ours_gyro", 100, "w20g40", vehicle="heavy", label_extra="heavy"),
        case("truth", 100, "w30g55"), case("ours_gyro", 100, "w30g55"), case("none", 100, "w30g55"),
        case("ours_gyro", 100, "w20g40", osd=True, label_extra="osd_mask"),
        case("ours_gyro", 100, "w20g40", breakup=0.10, label_extra="breakup10"),
        case("ours_gyro", 50, "w20g40", ground="snow_tarps_pad", label_extra="snow_tarps_pad"),
        case("ours_gyro", 50, "calm", ground="snow", label_extra="snow_plain"),
        case("ours_gyro", 100, "w20g40", video="NTSC", readout_s=0.0152, label_extra="ntsc"),
        case("ours_gyro", 100, "w20g40", stamp_err_s=0.02, label_extra="stamp_err20ms"),
        case("ours_gyro", 100, "w20g40", proc_s=0.15, label_extra="proc150ms"),
        case("ours", 50, "w20g40", px4={"EKF2_EVP_NOISE": 1.0}, label_extra="evp1.0"),
    ],
    "final": [
        case("ours", 100, "w20g40", px4={"EKF2_EVP_NOISE": 1.0}, label_extra="evp1.0"),
        case("ours_gyro", 100, "w20g40", vehicle="light", engage_wait_s=60.0, label_extra="light"),
    ],
}
# NTSC readout 15.2 ms: assumed, active lines of one NTSC field (240 of 262.5 lines x 16.7 ms).


def load():
    try:
        return json.load(open(RESULTS))
    except (OSError, ValueError):
        return {}


def table(res):
    cols = [("rms", "drift rms m"), ("max", "drift max m"), ("judge_rms", "judge m"), ("judge_jitter_rms", "jitter m"),
            ("roll_rms_deg", "roll deg"), ("pitch_rms_deg", "pitch deg"), ("pct_frames_fix", "% fix"),
            ("fix_rate_hz", "fix Hz"), ("age_ms_p50", "age ms"), ("drift_m_per_min", "drift m/min")]
    for r in res.values():     # the SIM's own variants count frames per estimator call (25 fps)
        if "pct_frames_fix" not in r and isinstance(r.get("fix_rate_hz"), (int, float)):
            r["pct_frames_fix"] = min(100.0, 100.0 * r["fix_rate_hz"] / 25.0)
    lines = ["case".ljust(30) + "".join(n.rjust(12) for _, n in cols) + "  events"]
    for k in sorted(res):
        r = res[k]
        cells = "".join((f"{r[c]:12.3g}" if isinstance(r.get(c), (int, float)) else "-".rjust(12)) for c, _ in cols)
        ev = "; ".join(f"{t:g}s {w}" for t, w in r.get("events", [])[1:])[:120]
        lines.append(k.ljust(30) + cells + "  " + ev)
    lines.append("")
    lines.append("drift rms/max: true horizontal distance from the hold point over the 60 s hold. drift m/min: linear-fit")
    lines.append("speed. roll/pitch: RMS about the mean. judge: distance of the picture centre's ground point from the")
    lines.append("hold point (RMS); jitter: its RMS about its own mean (the judge-view wander without the steady wind")
    lines.append("tilt). % fix: fixes sent per rendered PAL frame; fix Hz: fixes sent per second; age: capture to send.")
    lines.append("apb = the onboard program (apb_main run) with a 10x slower CPU; truth = the true position as the fix.")
    lines.append("gps = PX4's own GNSS position hold (simulated GNSS on, no vision); none = no position source.")
    lines.append("b_imu / b_gyro = apb with att_source imu / gyro (run set B, seeds 11-13: validation set).")
    open(TABLE, "w").write("\n".join(lines) + "\n")
    return "\n".join(lines)


def main():
    if sys.argv[1] == "table":
        print(table(load()))
        return
    cases = GROUPS[sys.argv[1]]
    a = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    b = int(sys.argv[3]) if len(sys.argv) > 3 else len(cases)
    for c in cases[a:b]:
        try:
            r = sitl.run_case(c)
        except Exception as e:          # one broken run must not lose the others
            r = dict(case=c["label"], events=[(0, f"ERROR {type(e).__name__}: {e}")])
            print(r["events"], flush=True)
        res = load()
        res[c["label"]] = {k: v for k, v in r.items() if k != "cfg"}
        json.dump(res, open(RESULTS, "w"), indent=1, default=str)
    print(table(load()))


if __name__ == "__main__":
    main()
