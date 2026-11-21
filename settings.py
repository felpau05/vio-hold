"""Every value that can change on the event day, in one place.

settings.json holds the values. load() starts from DEFAULTS, applies the file and refuses an unknown
key or a value of the wrong kind, so a typo stops the program instead of being ignored.
calib.py writes its results back with save().
"""
import json
import os

import gnss_rule

HERE = os.path.dirname(os.path.abspath(__file__))
PATH = os.path.join(HERE, "settings.json")

DEFAULTS = {
    # Flight-controller link.
    # assumed: research/hardware.md says mavlink-router bridges FC UART4 to UDP 127.0.0.1:14540 on the APB.
    # Without mavlink-router use "/dev/ttymxc2" (needs pyserial).
    "fc_device": "udpin:127.0.0.1:14540",
    "fc_baud": 230400,                 # research/hardware.md: field-confirmed; 921600 lost the link
    # assumed: the APB UART to the FC (research/hardware.md). Probed for Betaflight/iNav (MSP) when
    # no MAVLink heartbeat arrives.
    "msp_device": "/dev/ttymxc2",
    "att_hz": 100,                     # research/px4_vision.md section 9: fits the 230400 link with other streams off
    "hgt_hz": 10,                      # LOCAL_POSITION_NED: baro height and the FC position for re-anchoring
    "min_att_hz": 40,                  # H14 start gate: attitude at 40 Hz or more
    "att_stale_s": 0.05,               # H14: no attitude for 50 ms -> stop sending
    "hgt_stale_s": 0.5,                # same limit as Hold's own "height late" check
    "att_wait_s": 0.1,                 # H5: wait up to 100 ms for attitude to cover the frame's last row
    "answer_timesync": True,           # H2: answer PX4 TIMESYNC so PX4 can use our capture time stamps
    # Attitude for Hold. "gyro": the FC's body rates integrated, leaning toward the FC attitude with
    # att_tau_s (apb_main.GyroAtt). "fc": the FC attitude as sent; in PX4 SITL it flew away at 50-122 m,
    # because PX4's tilt follows our fix while it fuses it (px4sim/results.txt).
    "att_source": "gyro",
    "att_tau_s": 20.0,                 # assumed: far slower than the 2-3 s period of that loop at 50-122 m (SITL)
    # "imu": as "gyro", but roll and pitch lean toward the accelerometer's gravity direction (SCALED_IMU at
    # imu_hz), never toward the FC's tilt (apb_main.ImuAtt). Link budget at 50 Hz: 36 B x 50 = 1.8 kB/s.
    "imu_hz": 50,
    # Lat (deg), lon (deg), alt (m AMSL) of the field. ArduPilot needs it (SET_GPS_GLOBAL_ORIGIN).
    # Not known yet: read it from a phone map at the field.
    "origin_lat_lon_alt": None,
    # Needed flight-controller parameters (research/px4_vision.md section 1, finding H2).
    # assumed EKF2_EV_DELAY 0: we answer TIMESYNC and stamp the capture time (px4_vision.md section 3).
    # If PX4's time sync does not converge, set it to the MEDIAN fix age ("apb_main.py delay <run log>" prints
    # it): without time sync PX4 stamps a fix on arrival and takes EKF2_EV_DELAY off (unlike ArduPilot's
    # smallest-age rule, apb_main.delay_advice).
    # EKF2_DELAY_MAX 300: the EKF's fusion horizon must be at least the oldest fix (research/papers_control.md
    # S13). assumed 300 ms: above the p95 fix age at send in apsim (190-313 ms, integ/control). If the FC
    # refuses it, set its largest allowed value.
    # EKF2_EV_NOISE_MD 0: our covariance sets the noise, EKF2_EVP_NOISE is only its floor (S3).
    # EKF2_RNG_CTRL 0 assumes no useful range finder at 20-122 m.
    "params_px4": {"EKF2_EV_CTRL": 1, "EKF2_GPS_CTRL": 0, "EKF2_HGT_REF": 0, "EKF2_BARO_CTRL": 1,
                   "EKF2_RNG_CTRL": 0, "EKF2_EV_NOISE_MD": 0, "EKF2_EV_DELAY": 0, "EKF2_DELAY_MAX": 300,
                   "COM_ARM_WO_GPS": 1},
    # ArduPilot autotest VisionPosition sets VISO_TYPE 1, EK3_SRC1_POSXY 6, VELXY 6, POSZ 6, VELZ 6, GPS1_TYPE 0.
    # VELXY 0 and VELZ 0 here: we send no velocity (the autotest's Vicon sim does). POSZ 1 (baro): our z is the
    # FC's own height. Closed loop with these values in ArduCopter 4.6 SITL: apsim/results.txt.
    # VISO_DELAY_MS 140: in apsim (80 ms board, 230400 link) ArduPilot's own log put our fixes 134-144 ms after
    # capture (VISP CTimeMS - RTimeUS) and the program advised 144; 100 -> 140 cut the 100 m windy drift from
    # 1.08 / 2.32 m (RMS middle / worst, validation seeds 11-13) to 0.77 / 0.87 m (integ/control). Still an
    # assumption for the real board: set it from "apb_main.py delay <run log>" after the first hover.
    # GPS1_TYPE 1, not the autotest's 0: the event GNSS rule (gnss_rule.py) needs GPS on for the judges' log,
    # and no EKF source may use it. AHRS_GPS_USE 0 and EK3_GSF_*_MASK 0 close the other two ways that ArduPilot
    # uses GPS (DCM fallback, GSF yaw). The apsim results above ran with GPS1_TYPE 0.
    "params_ardupilot": {"VISO_TYPE": 1, "EK3_SRC1_POSXY": 6, "EK3_SRC1_VELXY": 0, "EK3_SRC1_POSZ": 1,
                         "EK3_SRC1_VELZ": 0, "EK3_SRC1_YAW": 1, "GPS1_TYPE": 1, "AHRS_GPS_USE": 0,
                         "EK3_GSF_RUN_MASK": 0, "EK3_GSF_USE_MASK": 0, "VISO_DELAY_MS": 140},

    # Video. SPEC (Stage 1, Q3, rev 1.3): the nadir camera is the Justice D, a DIGITAL
    # MIPI CSI rolling-shutter camera, 1280 x 720 at 60 fps, 16:9, FOV 114 x 86 deg (121.5 deg diagonal). The
    # Justice A is analog and forward-facing; the solution uses the Justice D. "analog" stays for old recordings.
    "video_kind": "csi",               # "csi" (Justice D) or "analog" (grabber: PAL/NTSC, interlaced fields, no-signal)
    # assumed: research/hardware.md quotes a Prismo comment, /dev/video2 = MIPI downward on the APB. Stage 2 gives
    # the event acquisition details; check mode prints what the device really delivers.
    "video_device": "/dev/video2",
    "video_size": [1280, 720],         # csi: width, height asked from the driver (spec: 1280 x 720)
    "video_fps": 60.0,                 # csi: frame rate asked from the driver (spec: 60 fps)
    # csi: a GStreamer pipeline string ending in "appsink" for sensors that need one (libcamera, v4l2src, ISI on
    # the i.MX8M Plus). Empty: OpenCV opens video_device through V4L2.
    "video_pipeline": "",
    # Picture handed to Hold = frame x video_scale (INTER_AREA). assumed: 0.5 (640 x 360) so Hold's cost matches
    # what apb_main.py timing measured on 360 x 288 fields; raise it only if 'check' shows spare CPU.
    "video_scale": 0.5,
    "video_standard": "auto",          # analog only: "auto", "PAL" or "NTSC"; check mode compares it with the driver
    # analog: "top" or "bottom": one field per picture, width halved (H10; each field has one time stamp).
    # "frame": the whole frame (always for csi); readout_s must then span the whole sensor readout.
    "field": "frame",
    # Measured on real/ clip-1 (analog DVR): repeated frames differ by 0.01-0.24 grey, fresh frames by
    # 4.16 or more. Re-measure on the Justice D (digital, lower noise).
    "repeat_mad": 1.0,
    # Measured on real/ clip-1 (analog): ground at 27 m has a MAD of 6.5 grey or more at 1/8 size; a flat
    # "no signal" screen has about 0.
    "no_signal_mad": 2.0,

    # Camera geometry. calib.py writes these.
    "top": "front",                    # body direction of the picture's top edge: front, right, rear, left
    "mirror": False,
    "tilt_deg": [0.0, 0.0],            # mount roll, pitch against straight down
    "twist_deg": 0.0,
    "lens_file": "justice_lens_profiles.json",
    # Nominal Justice D from the spec FOV (justice_lens_profiles.json). The spec's 114 x 86 x 121.5 deg are not one
    # pinhole or equidistant lens (focal length differs by about 7 % between them), so calib.py must measure
    # focal_scale, and a real lens calibration should replace this profile.
    "lens_name": "Justice D nominal (spec FOV, uncalibrated)",
    "focal_scale": 1.0,
    "readout_s": 0.014,                # assumed: under one 60 fps frame (16.7 ms), NOT measured on the Justice D; measure it
                                       # (review/hardware_gap/bench_calib.py, tools/replay_tune.py). Negative = bottom to top (G6)
    "video_delay_s": 0.050,            # assumed (about 3 frames at 60 fps), NOT measured; calib measures it. t0 = frame stamp - video_delay_s (H1)

    # Hold.
    "min_height_m": 15.0,              # H9: no anchor and no fix below this height (example value in H9)
    "hold_cfg": {},                    # Hold tuning overrides by keyword
    "send_hz": 50,                     # cap; research/px4_vision.md section 4: PX4 docs say 30-50 Hz
    "min_send_hz": 10,                 # H6: PX4 does not start fusion with gaps over 200 ms
    "good_before_send": 10,            # H4/H14: good frames in a row before sending again
    # Multiplies the covariance we send. 1 = Hold's own. ArduPilot turns it into posErr =
    # cbrt(c0^2 + c6^2 + c11^2) (GCS_Common) and floors that at VISO_POS_M_NSE.
    "cov_scale": 1.0,
    # Speed from our own fixes (apb_main.Speed) as VISION_SPEED_ESTIMATE. Off: it is our positions again,
    # smoothed; see apsim/control results before turning it on. Needs EK3_SRC1_VELXY 6 on ArduPilot.
    "send_speed": False,
    "speed_window_s": 0.4,             # assumed: about 5 fixes at 13 Hz
    "speed_sd_mps": 0.3,               # PX4 only (ArduPilot uses VISO_VEL_M_NSE)
    # EXPERIMENT, ArduCopter only (apb_main.WindAssist): while the PILOT has selected GUIDED, move the position
    # target upwind by the wind push that the FC's accelerometer measures, faster than Loiter's integral term.
    # The program never selects GUIDED itself; any fault hands back to LOITER. Off until it is shown to win.
    "wind_assist": False,
    "assist_tau_s": 0.5,               # assumed: filter of the push (SCALED_IMU at 50 Hz is noisy)
    "assist_slow_s": 2.0,              # assumed: about the time ArduCopter's integral term needs (stock gains)
    "assist_max_m": 1.5,               # assumed: largest target shift (about 3 m/s^2 with stock gains)
    "assist_kp_kv": 2.0,               # PSC_POSXY_P x PSC_VELXY_P of the FC (stock 1.0 x 2.0)
    "mask_file": "",                   # PNG, 0 = not ground (OSD, airframe). Made by "apb_main.py mask".
    "log_dir": "logs",
    "raw_record": False,               # also save raw frames and FC data for replay
    "min_free_gb": 2.0,                # assumed: a 5 min raw recording of 360x288 fields at 25 Hz is about 0.8 GB

    # Calibration (calib.py).
    "calib_seconds": 30,
    "calib_min_ratio": 5.0,            # H3: best orientation residual under 1/5 of the next best
    "calib_min_rate_dps": 20.0,        # H1: refuse recordings under 20 deg/s RMS rotation
    "calib_max_resid_deg": 1.0,        # assumed: the sim gives 0.15 deg; a real lens and noise give more
    "calibrated": False,
    "calib_capture": "",               # capture mode the calibration was made in; run refuses another one
    "calib_info": {},
}

CHOICES = {"video_standard": ("auto", "PAL", "NTSC"), "field": ("top", "bottom", "frame"), "video_kind": ("csi", "analog"),
           "top": ("front", "right", "rear", "left"), "att_source": ("gyro", "fc", "imu")}


def _check(k, v):
    d = DEFAULTS[k]
    if k in CHOICES and v not in CHOICES[k]:
        return f"must be one of {', '.join(CHOICES[k])}"
    if d is None:
        ok = v is None or (isinstance(v, list) and len(v) == 3 and all(isinstance(x, (int, float)) for x in v))
        return None if ok else "must be null or [lat, lon, alt]"
    if isinstance(d, bool):
        return None if isinstance(v, bool) else "must be true or false"
    if isinstance(d, (int, float)):
        return None if isinstance(v, (int, float)) and not isinstance(v, bool) else "must be a number"
    if isinstance(d, list) and (not isinstance(v, list) or len(v) != len(d)):
        return f"must be a list of {len(d)} numbers"
    if not isinstance(v, type(d)):
        return f"must be a {type(d).__name__}"
    if k in ("params_ardupilot", "params_px4"):
        for name, x in v.items():
            if not isinstance(x, (int, float)) or isinstance(x, bool):
                return f"{name} must be a number"
            why = gnss_rule.value_problem(k[len("params_"):], name, x)
            if why:
                return f"breaks the event GNSS rule (gnss_rule.py): {why}"
    return None


def load(path=PATH):
    s = json.loads(json.dumps(DEFAULTS))          # deep copy
    if os.path.exists(path):
        with open(path) as f:
            user = json.load(f)
        for k, v in user.items():
            if k not in DEFAULTS:
                raise ValueError(f"{path}: unknown key '{k}'. Fix the spelling or remove it.")
            why = _check(k, v)
            if why:
                raise ValueError(f"{path}: '{k}' {why}; it is {v!r}.")
            s[k] = v
    return s


def save(s, path=PATH):
    for k, v in s.items():
        if k not in DEFAULTS:
            raise ValueError(f"unknown settings key '{k}'")
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(s, f, indent=1)
        f.write("\n")
    os.replace(tmp, path)


def here(p):
    """A settings path relative to the program folder."""
    return p if not p or os.path.isabs(p) else os.path.join(HERE, p)
