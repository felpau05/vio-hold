#!/usr/bin/env python3
"""The anchor hold on the APB (i.MX8M Plus), and its event-day tools.

  apb_main.py check [--set-params]   firmware, parameters, stream rates, video, CPU time, disk
  apb_main.py mask  [--rec FILE]     record while the drone is turned by hand; build the OSD mask
  apb_main.py calib [--rec FILE]     record a hand rotation; solve orientation, delay, readout, focal, tilt
  apb_main.py bench [--height 1.5]  ground test, props off: the hold at a fixed height, moves in body axes
  apb_main.py run   [--raw]          the hold: send the fix to the flight controller
  apb_main.py replay FILE...         a raw recording through the same pipeline (for tuning)
  apb_main.py delay LOG.csv...       VISO_DELAY_MS from the fix ages of a flight
  apb_main.py timing RUN.csv RAW.npz...  video delay and readout from the in-flight stick wiggle: the raw frames of
                                     the 15 s in the air before the pilot's switch to position hold in the run log

Every check prints one line: PASS or FAIL, and what to do. run refuses to start when a check fails.
Safety rule: send nothing when in doubt (stale inputs, bad picture, low height, too few good frames).
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import shutil
import sys
import threading
import time
import traceback

import cv2
import numpy as np

import settings as S
from calib import sweep          # noqa: F401  (sim_test.timing_test calls apb_main.sweep)
from hold import Hold, Series, build_static_mask, euler_R, load_lens, make_R_bc  # noqa: F401


# ---------------------------------------------------------------- the pipeline (run and replay)
def lens(s, shape, focal=True):
    h, w = shape
    if s.get("lens_measured"):       # calib or tools/lens_calib.py measured it; lens_name is then only calib's start
        import calib
        K, D = calib.measured_lens(s["lens_measured"], w, h)
    else:
        field = None if s["camera"] == "digital" else {"top": 0, "bottom": 1}.get(s["field"])   # a digital frame is whole
        K, D = load_lens(S.here(s["lens_file"]), s["lens_name"], w, h, field=field)
    if focal:
        K[0, 0] *= s["focal_scale"]
        K[1, 1] *= s["focal_scale"]
    return K, D


def load_mask(s, shape):
    if not s["mask_file"]:
        return None
    m = cv2.imread(S.here(s["mask_file"]), cv2.IMREAD_GRAYSCALE)
    if m is None:
        raise ValueError(f"cannot read mask {s['mask_file']}. Run 'apb_main.py mask' again.")
    if m.shape != tuple(shape):
        raise ValueError(f"mask is {m.shape[1]}x{m.shape[0]} but the picture is {shape[1]}x{shape[0]}. "
                         "Run 'apb_main.py mask' again.")
    return m


def make_hold(s, shape):
    K, D = lens(s, shape)
    R = make_R_bc(s["top"], s["mirror"], tuple(s["tilt_deg"]), s["twist_deg"])
    return Hold(K, D, R, s["readout_s"], mask=load_mask(s, shape), **s["hold_cfg"])


class GyroAtt:
    """The attitude Hold gets: the FC's ATTITUDE body rates integrated, leaning toward the FC's attitude
    with time constant tau_s. Why: while the FC fuses our fix, PX4 turns its gravity (tilt) fusion off
    (EKF2 gravity_fusion.cpp: !isHorizontalAidingActive()), so its tilt follows our fix, and our fix
    follows its tilt with gain = height. Closed loop in PX4 SITL with the raw FC attitude: fly-away past
    30 m at 50, 100 and 122 m (px4sim/results.txt). Call with each ATTITUDE row in time order."""

    def __init__(self, tau_s):
        self.tau, self.R, self.t, self.w = tau_s, None, None, None

    def target(self, row):
        """The slow reference the integrated rates lean toward: here the FC's attitude."""
        return euler_R(*row[1:4])

    def __call__(self, row):
        if len(row) < 7:                      # no rates: the FC attitude as it is
            return row
        t, w = row[0], np.asarray(row[4:7], float)
        R_fc = self.target(row)
        dt = math.inf if self.t is None else t - self.t
        if dt > 0.5:                          # start, or a gap in the attitude: take the FC attitude
            self.R = R_fc
        elif dt > 0:
            self.R = self.R @ cv2.Rodrigues((w + self.w) / 2 * dt)[0]            # body-frame turn
            e = cv2.Rodrigues(R_fc @ self.R.T)[0]                               # FC minus ours, NED
            self.R = cv2.Rodrigues(e * min(1.0, dt / self.tau))[0] @ self.R
            u, _, vt = np.linalg.svd(self.R)
            self.R = u @ vt
        self.t, self.w = t, w
        R = self.R
        return (t, math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))),
                math.atan2(R[1, 0], R[0, 0]), *row[4:])


class ImuAtt(GyroAtt):
    """As GyroAtt, but roll and pitch lean toward the gravity direction that the FC's accelerometer measures
    (SCALED_IMU), not toward the FC's tilt; yaw still leans toward the FC's yaw. The FC's tilt estimate is
    never used, so its vision fusion has no path back into our tilt. acc: Series (t, ax, ay, az) m/s^2 body
    FRD. Without an accelerometer sample in the last 0.1 s it leans toward the FC attitude, like GyroAtt.
    Known limit, why "gyro" stays the default: the accelerometer also measures the drone's own acceleration
    (a/g of tilt) and its own bias, which PX4's EKF estimates and this filter does not. PX4 SITL, validation
    seeds 11-13 (integ/avail/summary_b.txt): 100 m w20g40 1.25-2.75 m RMS against 0.58-0.74 m for GyroAtt;
    100 m calm 1.87-2.10 against 0.41-0.63; 122 m 2.55-4.42 against 0.99-1.20."""

    def __init__(self, tau_s, acc):
        super().__init__(tau_s)
        self.acc = acc

    def target(self, row):
        a = np.array([r for r in self.acc.rows[-10:] if row[0] - 0.1 <= r[0] <= row[0] + 0.02])
        if not len(a):
            return euler_R(*row[1:4])
        d = -a[:, 1:4].mean(0)                # down in body axes (an accelerometer at rest reads -g along down)
        roll, pitch = math.atan2(d[1], d[2]), math.atan2(-d[0], math.hypot(d[1], d[2]))
        return euler_R(roll, pitch, row[3])


def att_filter(s, fc=None):
    if s["att_source"] == "imu" and fc is not None:
        return ImuAtt(s["att_tau_s"], fc.imu)
    # shortcut: replay has no accelerometer log, so "imu" replays as "gyro"; record SCALED_IMU in Raw to replay it.
    return GyroAtt(s["att_tau_s"]) if s["att_source"] in ("gyro", "imu") else None


def catch_up(snap, live, f=None):
    """Copy the rows the FC reader thread added to live since the last call into snap. Hold reads only
    snap, which no other thread writes, so it never sees a Series grow in the middle of a frame.
    f: a row filter (GyroAtt) that sees each row once, in order."""
    last, k = snap.last_t(), 64
    while True:
        tail = live.rows[-k:]                 # one slice: atomic against the reader's append and trim
        if not tail or tail[0][0] <= last or k >= len(live.rows):
            break
        k *= 4
    for r in tail:
        if r[0] > last:
            snap.add(*(f(r) if f else r))
    return snap


# Re-anchor to the FC position on 'lost' (Pipe.step). False lets Hold re-find its last or home keyframe
# instead. PX4 SITL, 100 m, 20 gusting 40 km/h, 10x slower CPU (px4sim/results.txt): 3 s video outage
# 0.76 / 1.49 m (RMS / max) with re-anchor, 1.29 / 2.68 m with re-find; break-up bursts 1.40 / 2.99 m
# against 3.26 / 9.50 m. The re-find is slow on a slow CPU and leaves gaps of up to 4.9 s without a fix.
REANCHOR_ON_LOST = True


class Pipe:
    """Frame -> (Est or None, send?, reason). The same code serves run and replay.

    fc_pos(t) -> the FC's (north, east) or None. Re-anchors there at the start, after the height gate
    closed and after 'lost', so the EKF sees no jump (H14). Sends only after good_before_send good
    fixes in a row (H4).
    """

    def __init__(self, s, fc_pos):
        self.s, self.fc_pos, self.hold, self.good, self.anchored, self.note = s, fc_pos, None, 0, False, None

    def _reanchor(self, t):
        p = self.fc_pos(t)
        where = "FC position" if p is not None else "last position (no FC position)"
        if p is None:
            p = self.hold.pos.copy()
        self.hold.reanchor(p)
        self.anchored, self.good = True, 0
        self.note = f"re-anchored to {where} {p[0]:.2f} {p[1]:.2f}"

    def step(self, fr, att, hgt, gate=None):
        s, self.note = self.s, None
        why = gate or fr.why
        if why is None and att.last_t() < fr.t0 + max(s["readout_s"], 0.0) + 0.005:
            why = "attitude late"
        if why is None and hgt.last_t() < fr.t0 - s["hgt_stale_s"]:
            why = "height stale"
        if why is None:
            h = float(hgt.at(fr.t0))
            if h < s["min_height_m"]:
                why, self.anchored = f"height {h:.1f} m under {s['min_height_m']:.0f} m", False
        if why:
            self.good = 0
            return None, False, why
        if self.hold is None:
            self.hold = make_hold(s, fr.img.shape)
        if not self.anchored:
            self._reanchor(fr.t0)
        e = self.hold.process(fr.img, fr.t0, att, hgt)
        if e.state == "lost" and REANCHOR_ON_LOST:
            self._reanchor(e.t)
        if e.var is None:
            # A single failed frame ('degraded') does not restart the count: Hold gates every frame itself and
            # reports 'lost' after LOST_S of failures. On prairie at 100 m the old reset on every failed frame
            # kept 10 good fixes in a row from ever happening (integ/avail/base_prairie100w.txt).
            if e.state != "degraded":
                self.good = 0
            return e, False, e.why or e.state
        self.good += 1
        if self.good < s["good_before_send"]:
            return e, False, f"good fix {self.good} of {s['good_before_send']}"
        return e, True, None


# ---------------------------------------------------------------- pilot handover and the recovery demonstration
# ArduCopter custom_mode numbers: enum class Mode::Number in ArduCopter/mode.h (apsim/ardupilot, Orqa's branch of
# https://github.com/ArduPilot/ardupilot). LOITER and POSHOLD hold a position.
AP_MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE",
            9: "LAND", 16: "POSHOLD", 17: "BRAKE"}
AP_HOLD = {"LOITER", "POSHOLD"}
# PX4 custom_mode: the main mode in bits 16-23 and the sub mode in bits 24-31 (pymavlink 2.4.50, mavutil.py:
# PX4_CUSTOM_MAIN_MODE_*, PX4_CUSTOM_SUB_MODE_AUTO_LOITER and interpret_px4_mode). POSCTL, and AUTO with the sub
# mode LOITER, hold a position.
PX4_MAIN = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED"}
PX4_POSCTL, PX4_AUTO, PX4_AUTO_LOITER = 3, 4, 3


def fc_mode(fc):
    """(name, holds_position) of the FC flight mode. (None, False) before the first HEARTBEAT. A firmware whose
    modes this program does not know (Betaflight, iNav) gives ("mode N", False)."""
    m, fw = getattr(fc, "mode", None), getattr(fc, "firmware", None)
    if m is None:
        return None, False
    if fw == "ardupilot":
        name = AP_MODES.get(m, f"mode {m}")
        return name, name in AP_HOLD
    if fw == "px4":
        main, sub = (m >> 16) & 0xFF, (m >> 24) & 0xFF
        if main == PX4_AUTO and sub == PX4_AUTO_LOITER:
            return "AUTO_LOITER", True
        return PX4_MAIN.get(main, f"main mode {main}"), main == PX4_POSCTL
    return f"mode {m}", False


def _join(*notes):
    return "; ".join(n for n in notes if n) or None


class Engage:
    """Follows the FC flight mode for run_live (Stage 1 judging area D: engagement and release without a disruptive
    transition). step() gives the mode name for the run log and a note at each mode change: "engaged" when the
    pilot's switch selects position hold, "released" when it leaves it. At the switch it asks Hold for a fresh
    keyframe when fresh_kf_at_engage is set. t_engaged: monotonic time of the last switch, None when not holding."""

    def __init__(self, s):
        self.s, self.name, self.held, self.t_engaged = s, None, False, None

    def step(self, fc, pipe, now):
        name, held = fc_mode(fc)
        if name is None or name == self.name:
            return self.name or "", None
        if held and not self.held:
            self.t_engaged, note = now, f"engaged: FC mode {name}"
            if self.s["fresh_kf_at_engage"] and pipe.hold is not None:
                pipe.hold.request_keyframe()
                note += "; fresh keyframe requested"
        elif self.held and not held:
            self.t_engaged, note = None, f"released: FC mode {name}"
        else:
            note = f"FC mode {name}"
        self.name, self.held = name, held
        return name, note


class OutageTest:
    """The recovery demonstration (settings outage_test_s, outage_after_s; Stage 1 bonus route "demonstrated
    recovery"). step() gives (dropping, note): run_live gives Hold no frame while dropping, as in a video outage.
    A drop starts when the file outage_now appears in log_dir (step deletes it), or outage_after_s seconds after
    the switch engaged position hold, once for each engagement."""
    FILE = "outage_now"

    def __init__(self, s):
        self.len, self.after = float(s["outage_test_s"]), float(s["outage_after_s"])
        self.path = os.path.join(S.here(s["log_dir"]), self.FILE)
        self.dropping, self.until, self.done_for = False, -math.inf, None

    def _file(self):
        if not os.path.exists(self.path):
            return False
        try:
            os.remove(self.path)
        except OSError:
            pass
        return True

    def step(self, now, t_engaged):
        if self.len <= 0:
            return False, None
        start = self._file()
        if self.after > 0 and t_engaged is not None and t_engaged != self.done_for and now - t_engaged >= self.after:
            start, self.done_for = True, t_engaged
        if start and not self.dropping:
            self.dropping, self.until = True, now + self.len
            return True, f"recovery test: video dropped for {self.len:g} s"
        if self.dropping and now >= self.until:
            self.dropping = False
            return False, "recovery test: video back"
        return self.dropping, None


class Log:
    """The run log. A write error (full disk) never stops the fixes: errors counts it, the first prints one line."""
    COLS = ["t", "state", "n", "north", "east", "var", "sent", "why", "proc_ms", "age_ms", "height", "note", "mode"]

    def __init__(self, path):
        self.path, self.errors = path, 0
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            self.f = open(path, "w", newline="")
        except OSError as e:
            self._error(e)
            self.f = open(os.devnull, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(self.COLS)

    def _error(self, e):
        self.errors += 1
        if self.errors == 1:
            print(f"log write failed: {e}. The hold continues without a log. Free disk space at "
                  f"{os.path.dirname(self.path)} before the next flight.", flush=True)

    def row(self, fr, e, sent, why, proc_ms, age_ms, height, note, mode=""):
        f = lambda x: "" if x is None else f"{x:.4f}"      # noqa: E731
        try:
            self.w.writerow([f"{fr.t0:.4f}" if fr else "", e.state if e else "", e.n if e else "",
                             f(e.pos[0]) if e else "", f(e.pos[1]) if e else "", f(e.var) if e else "",
                             int(sent), why or "", f"{proc_ms:.1f}", f"{age_ms:.1f}" if age_ms else "",
                             f(height), note or "", mode or ""])
            self.f.flush()                                # a crash keeps every line written so far
        except OSError as e:
            self._error(e)

    def close(self):
        try:
            self.f.close()
        except OSError as e:
            self._error(e)


def stamp():
    return time.strftime("%Y%m%d_%H%M%S")


class RateGate:
    """keep(t) for the recorders (record_hz): True for the first frame in each 1/hz s slot of the stamps, so at most
    hz frames each second, each with its exact stamp. hz 0 keeps each frame. Slots, not a smallest gap: with stamps
    that jitter, a smallest gap of 1/30 s keeps every second or third frame of a 60 fps stream (about 23 each second)."""

    def __init__(self, hz):
        self.hz, self.slot = hz, None

    def __call__(self, t):
        if self.hz <= 0:
            return True
        k = math.floor(t * self.hz)
        keep, self.slot = k != self.slot, k
        return keep


class Raw:
    """Raw recording in 10 s chunks (frames, FC attitude, height, position), saved by a thread.
    At most record_hz frames each second (RateGate); Hold still gets each frame. It stops for the rest of the run
    under min_free_gb free on its disk, so the run log keeps space. One save thread at most: a chunk that comes
    while the last one is still being written is dropped (counted in dropped), so a slow disk cannot pile up threads."""

    def __init__(self, s, fc, shape):
        os.makedirs(S.here(s["log_dir"]), exist_ok=True)     # the run log may be elsewhere (apsim)
        self.base = os.path.join(S.here(s["log_dir"]), f"raw_{stamp()}")
        self.fc, self.K, self.D = fc, *lens(s, shape, focal=False)
        self.imgs, self.tg, self.k, self.keep = [], [], 0, RateGate(s["record_hz"])
        self.min_gb, self.off, self.saver, self.dropped = s["min_free_gb"], False, None, 0

    def add(self, fr):
        if self.off or not self.keep(fr.t_grab):
            return
        self.imgs.append(fr.img)
        self.tg.append(fr.t_grab)
        if fr.t_grab - self.tg[0] >= 10.0:
            self.flush()

    def flush(self):
        if not self.imgs:
            return
        where = os.path.dirname(self.base)
        free = shutil.disk_usage(where).free / 1e9
        if free < self.min_gb:
            self.off, self.imgs, self.tg = True, [], []
            print(f"the raw recorder stopped: the disk at {where} has {free:.1f} GB free, less than min_free_gb "
                  f"{self.min_gb:g}. The hold continues. Delete old logs before the next flight.", flush=True)
            return
        if self.saver is not None and self.saver.is_alive():
            self.imgs, self.tg, self.dropped = [], [], self.dropped + 1
            if self.dropped == 1:
                print(f"the raw recorder dropped a chunk: it still writes the last chunk to {where}. Set record_hz "
                      "lower, or set log_dir to a faster disk.", flush=True)
            return
        imgs, tg, self.imgs, self.tg = np.array(self.imgs), np.array(self.tg), [], []
        lo, hi = tg[0] - 1.0, tg[-1] + 1.0
        cut = lambda ser: np.array([r for r in list(ser.rows) if lo <= r[0] <= hi])     # noqa: E731
        path = f"{self.base}_{self.k:03d}.npz"
        self.k += 1
        data = dict(imgs=imgs, t_grab=tg, att=cut(self.fc.att), hgt=cut(self.fc.hgt), lpos=cut(self.fc.lpos),
                    K=self.K, D=self.D)
        self.saver = threading.Thread(target=np.savez, args=(path,), kwargs=data, daemon=False)
        self.saver.start()


def live_gate(s, fc):
    if not fc.clock.ready():
        return "FC clock offset not fitted yet"
    r = fc.rate("ATTITUDE", 1.0)
    if r < s["min_att_hz"]:
        return f"attitude {r:.0f} Hz, under {s['min_att_hz']} Hz"
    if fc.age("ATTITUDE") > s["att_stale_s"]:
        return "attitude stale"
    return None


def start_problems(s, fc, src, shape):
    """Reasons run must not start (H14). Empty list = go."""
    out = []
    if not s["calibrated"]:
        out.append("not calibrated. Run 'apb_main.py calib'.")
    import capture
    mode = capture.capture_mode(s, getattr(src, "info", {}), shape)
    if s["calibrated"] and s["calib_capture"] != mode:
        out.append(f"capture is '{mode}' but the calibration was made with '{s['calib_capture']}'. Run calib again.")
    if not s["mask_file"]:     # real/: without a mask 76 % of keyframe points sat on OSD text and the fix froze
        out.append("no mask. Run 'apb_main.py mask' (with no OSD in the picture it masks almost nothing).")
    try:
        make_hold(s, shape)                    # mask size, lens profile, hold_cfg names
    except (ValueError, KeyError, StopIteration) as e:
        out.append(f"cannot build the hold: {e!r}")
    if fc.firmware not in ("px4", "ardupilot"):
        out.append(f"firmware {fc.firmware} is not supported.")
    if fc.firmware == "ardupilot" and s["origin_lat_lon_alt"] is None:
        out.append("ArduPilot needs origin_lat_lon_alt in settings.json (lat, lon, alt of the field).")
    return out


def delay_advice(ages_ms, n=100):
    """VISO_DELAY_MS for these fix ages at send (ms). ArduPilot maps our time stamps to its clock with the
    smallest (arrival - stamp) of every n messages (AP_RTC JitterCorrection, n = 100; used by
    GCS_MAVLINK::correct_offboard_timestamp_usec_to_ms for VISION_POSITION_ESTIMATE and _SPEED_), so the
    EKF sees each fix at (capture time + that smallest age) - VISO_DELAY_MS. The right value is the
    smallest age of a block of n, not the median age. On a UART, ArduPilot also takes off the packet's
    transfer time at SERIALn_BAUD (receive_time_constraint_us), so the link adds almost nothing."""
    a = np.asarray(ages_ms, float)
    if len(a) < n:
        return float(a.min()) if len(a) else math.nan
    return float(np.median([a[i:i + n].min() for i in range(0, len(a) - n + 1, n)]))


class Speed:
    """Drone speed from our own fixes (VISION_SPEED_ESTIMATE): the least-squares slope of the good fixes of
    the last window_s seconds, stamped at their mean time (where a slope is the speed). It is only a
    smoothed difference of our positions: not new information, and its error follows theirs. Cleared at a
    re-anchor (a jump of the position frame). vz: the FC's own vertical speed at that time, because
    ArduPilot fuses vz too once it gets speed (AP_NavEKF3_PosVelFusion fuseData[2] with useExtNavVel). The
    first version took the slope of the FC's height: 1.6-2.6 m/s of vz error, EKF variance failsafe and a
    fly-away (run c_spd_100m_w20g40_s11)."""

    MAX_MPS = 10.0               # assumed: faster than any hold; a larger slope is a fix error, not speed

    def __init__(self, window_s):
        self.w, self.pts = window_s, []

    def add(self, e, note):
        if note and note.startswith("re-anchored"):
            self.pts = []
        if e is not None and e.var is not None:
            self.pts = [p for p in self.pts if p[0] > e.t - self.w] + [(e.t, *e.pos)]

    def velocity(self, vz):
        """(t, vn, ve, vd) or None when the window has under 4 fixes or spans under half its length.
        vz: Series of the FC's vertical speed (fc_link FC.vz)."""
        p = np.array(self.pts)
        if len(p) < 4 or p[-1, 0] - p[0, 0] < self.w / 2:
            return None
        t = p[:, 0] - p[:, 0].mean()
        v = (t @ (p[:, 1:3] - p[:, 1:3].mean(0))) / (t @ t)
        tm = float(p[:, 0].mean())
        if not vz.rows or not vz.rows[0][0] <= tm <= vz.last_t() + 0.2 or not np.all(np.isfinite(v)) \
                or np.hypot(*v) > self.MAX_MPS:
            return None
        return tm, float(v[0]), float(v[1]), float(vz.at(tm))


class WindAssist:
    """EXPERIMENT (settings wind_assist, off by default; ArduCopter only). While the FC is in GUIDED, which only
    the pilot selects, the position target is the hold point moved upwind by (d - d_slow) / (Kp Kv): d is the
    wind push in m/s^2, from the FC's accelerometer (SCALED_IMU body x and y: a multirotor's thrust is along body z, so
    body x and y feel only the air), low-passed with assist_tau_s and turned to north/east by the yaw. With the
    position controller's P path (Kp Kv) this pushes back at once, instead of waiting for Loiter's integral.
    Hold point: the FC's own position when GUIDED starts. Faults (no fix sent for 0.5 s, attitude or IMU stale,
    fix more than 3 m from the FC position) send LOITER and stay off until the pilot selects GUIDED again.
    The pilot takes over by any mode switch (ArduCopter ignores position targets outside GUIDED)."""

    GUIDED, LOITER = 4, 5
    GAP_S, IMU_STALE_S, FIX_JUMP_M = 0.5, 0.2, 3.0      # assumed fault limits

    def __init__(self, s):
        self.s, self.p0, self.d, self.t, self.latched, self.last_fix = s, None, np.zeros(2), None, False, None

    def step(self, fc, e, sent, now):
        """Call once per loop. Returns a note for the log or None."""
        if sent:
            self.last_fix = (now, e.pos.copy())
        if fc.mode != self.GUIDED:
            self.p0, self.latched = None, False
            return None
        if self.latched:
            return None
        lp = fc.local_pos()
        why = None
        if self.last_fix is None or now - self.last_fix[0] > self.GAP_S:
            why = "no fix"
        elif fc.age("ATTITUDE") > self.s["att_stale_s"] or not fc.imu.rows or now - fc.imu.last_t() > self.IMU_STALE_S:
            why = "attitude or IMU stale"
        elif lp is None or np.hypot(*(self.last_fix[1] - lp)) > self.FIX_JUMP_M:
            why = "fix far from the FC position"
        if why:
            fc.set_mode(self.LOITER)
            self.latched, self.p0 = True, None
            return f"wind assist off ({why}): LOITER"
        r = fc.imu.rows[-1]
        a = fc.att.rows[-1]
        cy, sy = math.cos(a[3]), math.sin(a[3])
        push = np.array([cy * r[1] - sy * r[2], sy * r[1] + cy * r[2]])          # body x, y -> north, east
        if self.p0 is None:
            # Both filters start at the push of now, so the target starts AT the hold point. Starting them at 0
            # gave the whole steady push for 2 s and a jump upwind (c_wa2_100m_w20g40_s11 with GUIDED at 5 s).
            self.p0, self.z0, self.d, self.ds, self.t = lp.copy(), -float(fc.hgt.rows[-1][1]), push.copy(), push.copy(), r[0]
            self.yaw0 = float(fc.att.rows[-1][3])               # hold the heading (yaw moves the fix, H8)
        dt = 0.0 if self.t is None else max(0.0, r[0] - self.t)
        self.d += min(1.0, dt / self.s["assist_tau_s"]) * (push - self.d)
        self.ds += min(1.0, dt / self.s["assist_slow_s"]) * (push - self.ds)
        self.t = r[0]
        # Only the CHANGE of the push: the steady part stays with ArduCopter's integral term. Version 1 moved
        # the target by the whole push; the integral term then drove the drone onto that target, 1-2 m upwind
        # (run c_wa_100m_w20g40_s11: 1.70 / 2.38 m against 0.77 / 1.27 m in plain Loiter).
        shift = -(self.d - self.ds) / self.s["assist_kp_kv"]
        n = np.hypot(*shift)
        if n > self.s["assist_max_m"]:
            shift *= self.s["assist_max_m"] / n
        tgt = self.p0 + shift
        fc.send_target(float(tgt[0]), float(tgt[1]), self.z0, self.yaw0)
        return None


NO_FRAME_S = 2.0     # s with no camera frame: run_live stops, so mode_run closes the camera and opens it again.
                     # assumed: a live camera never pauses this long; a restart takes about 1 s plus the FC connection


def run_live(s, src, fc, seconds=None, stop=None, log_path=None, fixed_height=None, on_fix=None):
    """The hold. Newest frame -> wait for attitude -> Hold.process -> send only when var is not None
    and every gate passes. Returns the log path. fixed_height (bench test only): use this height, not
    the FC's, and stop when the FC arms. on_fix(e, att) sees every fix."""
    fr, end = None, time.monotonic() + 10.0
    while fr is None or fr.why:
        fr = src.newest(timeout=1.0)
        if time.monotonic() > end:
            raise RuntimeError("no usable video frame in 10 s. Check the camera and video_device.")
    problems = start_problems(s, fc, src, fr.img.shape)
    if problems:
        raise RuntimeError("will not start: " + " ".join(problems))
    if s["origin_lat_lon_alt"] is not None:
        fc.send_origin(*s["origin_lat_lon_alt"])
    log = Log(log_path or os.path.join(S.here(s["log_dir"]), f"run_{stamp()}.csv"))
    raw = Raw(s, fc, fr.img.shape) if s["raw_record"] else None
    pipe = Pipe(s, lambda t: fc.local_pos())
    att, hgt, att_f = Series(30.0), Series(30.0), att_filter(s, fc)
    t_end = time.monotonic() + seconds if seconds else math.inf
    last_send, ages = -math.inf, []
    spd = Speed(s["speed_window_s"]) if s["send_speed"] else None
    wa = WindAssist(s) if s["wind_assist"] and fc.firmware == "ardupilot" else None
    eng, outage = Engage(s), OutageTest(s)
    t_frame = time.monotonic()
    try:
        while time.monotonic() < t_end and not (stop is not None and stop.is_set()):
            fr = src.newest(timeout=0.5)
            mode, mnote = eng.step(fc, pipe, time.monotonic())
            drop, onote = outage.step(time.monotonic(), eng.t_engaged)
            if fr is None or drop:
                pipe.good = 0
                note = wa.step(fc, None, False, time.monotonic()) if wa is not None else None
                log.row(fr, None, False, "no frame" if fr is None else "recovery test: video dropped", 0.0, None, None,
                        _join(mnote, onote, note), mode)
                if fr is not None:                                # the camera works: only the test drops its frames
                    t_frame = time.monotonic()
                    continue
                if time.monotonic() - t_frame > NO_FRAME_S:      # a stopped camera does not come back by itself
                    raise RuntimeError(f"no camera frame for {NO_FRAME_S:g} s: the camera stopped. The program opens "
                                       "it again. If this repeats, check the camera cable and video_device or "
                                       "video_command in settings.json.")
                continue
            t_frame = time.monotonic()
            need = fr.t0 + max(s["readout_s"], 0.0) + 0.005       # attitude must cover the last row (H5)
            end = time.monotonic() + s["att_wait_s"]
            while fc.att.last_t() < need and time.monotonic() < end:
                time.sleep(0.002)
            t1 = time.monotonic()
            catch_up(att, fc.att, att_f)
            if fixed_height is None:
                catch_up(hgt, fc.hgt)
            elif hgt.last_t() < fr.t0:
                hgt.add(fr.t0, fixed_height)
            gate = live_gate(s, fc) or ("FC armed: bench test stopped" if fixed_height is not None and fc.armed else None)
            e, ok, why = pipe.step(fr, att, hgt, gate=gate)
            if on_fix is not None and e is not None and e.var is not None:
                on_fix(e, att)
            proc_ms = (time.monotonic() - t1) * 1e3
            if spd is not None:
                spd.add(e, pipe.note)
            now = time.monotonic()
            sent, note = False, pipe.note
            if ok and fc.age("ATTITUDE") < s["att_stale_s"] and now - last_send >= 1.0 / s["send_hz"]:
                fc.send_fix(e, att, hgt)
                sent, last_send = True, now
                ages.append((now - e.t) * 1e3)
                if len(ages) % 100 == 0:
                    note = f"VISO_DELAY_MS advice {delay_advice(ages[-500:]):.0f}; fix age at send p50 {np.median(ages[-500:]):.0f} ms"
                v = spd.velocity(fc.vz) if spd is not None else None
                if v is not None:
                    fc.send_speed(*v)
            elif ok:
                why = "attitude stale" if fc.age("ATTITUDE") >= s["att_stale_s"] else "send rate limit"
            if wa is not None:
                note = wa.step(fc, e, sent, now) or note
            h = float(hgt.at(fr.t0)) if hgt.rows else None
            log.row(fr, e, sent, why, proc_ms, (now - e.t) * 1e3 if e is not None else None, h, _join(mnote, onote, note),
                    mode)
            if raw:
                raw.add(fr)
    finally:
        if raw:
            raw.flush()
        log.close()
    return log.path


def replay(s, paths, log_path=None):
    """Raw recording chunks through the same Pipe. Returns (log path, counts)."""
    import capture
    log = Log(log_path or os.path.join(S.here(s["log_dir"]), f"replay_{stamp()}.csv"))
    counts = {"frames": 0, "sent": 0}
    pipe, att_f = None, att_filter(s)
    att, hgt = Series(1e9), Series(1e9)
    for p in sorted(paths):
        d = np.load(p)
        # Raw.flush saves 1 s of rows before and after each chunk's frames, so neighbouring chunks share rows.
        # Each row goes into the filter once, in time order: a shared row fed twice bent GyroAtt's tilt at
        # every chunk start (tests_apb/test_replay_chunks.py).
        for r in d["att"]:
            if r[0] > att.last_t():
                att.add(*(att_f(tuple(r)) if att_f else r))
        for r in d["hgt"]:
            if r[0] > hgt.last_t():
                hgt.add(*r)
        lp = d["lpos"] if "lpos" in d and len(d["lpos"]) else None
        fc_pos = (lambda t, lp=lp: None if lp is None else
                  np.array([np.interp(t, lp[:, 0], lp[:, 1]), np.interp(t, lp[:, 0], lp[:, 2])]))
        if pipe is None:
            pipe = Pipe(s, fc_pos)
        pipe.fc_pos = fc_pos
        src = capture.FileSource(p, s)
        while (fr := src.newest()) is not None:
            t1 = time.monotonic()
            e, ok, why = pipe.step(fr, att, hgt)
            counts["frames"] += 1
            counts["sent"] += int(ok)
            if e is not None:
                counts[e.state] = counts.get(e.state, 0) + 1
            log.row(fr, e, ok, why, (time.monotonic() - t1) * 1e3, None, float(hgt.at(fr.t0)), pipe.note)
    log.close()
    return log.path, counts


# ---------------------------------------------------------------- event-day tools
def record(src, fc, seconds, say=print):
    """Frames as Hold gets them plus FC attitude/height, in the live clock. Bad pictures are skipped.
    At most record_hz usable frames each second (RateGate): 60 fps at 640x360 for 30 s is 415 MB, and np.array copies it."""
    imgs, tg, end, last = [], [], time.monotonic() + seconds, -1
    keep = RateGate(fc.s["record_hz"])
    while time.monotonic() < end:
        left = int(end - time.monotonic())
        if left != last and left % 5 == 0:
            say(f"  recording: {left} s left")
            last = left
        fr = src.newest(timeout=0.5)
        if fr is not None and fr.why is None and keep(fr.t_grab):
            imgs.append(fr.img)
            tg.append(fr.t_grab)
    if len(imgs) < 10:
        raise RuntimeError("fewer than 10 usable frames recorded. Check the camera picture.")
    time.sleep(0.1)
    rows = lambda ser: np.array([r for r in list(ser.rows) if tg[0] - 1 <= r[0] <= tg[-1] + 1])   # noqa: E731
    import capture
    imgs = np.array(imgs)
    K, D = lens(fc.s, imgs.shape[1:3], focal=False)
    mode = capture.capture_mode(fc.s, getattr(src, "info", {}), imgs.shape[1:3])
    return dict(imgs=imgs, t_grab=np.array(tg), att=rows(fc.att), hgt=rows(fc.hgt), lpos=rows(fc.lpos), K=K, D=D,
                mode=np.array(mode))


def open_video(s):
    import capture
    return capture.DigitalSource(s) if s["camera"] == "digital" else capture.V4L2Source(s)


def open_fc(s, say=print):
    import fc_link
    fc, why = fc_link.connect(s)
    if fc is None:
        raise RuntimeError(why)
    fc.setup_streams()
    end = time.monotonic() + 8.0
    while not (fc.clock.ready() and fc.rate("ATTITUDE", 1.0) >= s["min_att_hz"]) and time.monotonic() < end:
        time.sleep(0.1)
    return fc


def hold_timing(s, shape, n=60):
    """Hold.process time on this CPU, on a synthetic moving texture at this picture size (ms list). The frames are
    stamped at the camera's frame interval (analog: 25 Hz, as check mode assumes until measured)."""
    rng = np.random.default_rng(0)
    h, w = shape
    dt = 1.0 / (s["video_fps"] if s["camera"] == "digital" else 25.0)
    tex = cv2.GaussianBlur(rng.normal(128, 40, (h + 200, w + 200)).astype(np.float32), (0, 0), 2.0)
    tex = np.clip((tex - tex.mean()) * 4 + 128, 0, 255).astype(np.uint8)
    s2 = dict(s, mask_file="")
    hold = make_hold(s2, shape)
    att, hgt = Series(1e9), Series(1e9)
    for t in np.arange(-1, n * dt + 1, 0.01):
        att.add(t, 0.0, 0.0, 0.0)
    for t in np.arange(-1, n * dt + 1, 0.1):
        hgt.add(t, 50.0)
    ms = []
    for i in range(n):
        dx, dy = int(50 + 30 * math.sin(i / 9)), int(50 + 30 * math.cos(i / 11))
        img = tex[dy:dy + h, dx:dx + w].copy()
        t1 = time.perf_counter()
        hold.process(img, i * dt, att, hgt)
        ms.append((time.perf_counter() - t1) * 1e3)
    return ms[5:]


# Limit of the stamp line of check: the 95th percentile of |stamp interval - median interval|, ms. A stamp error dt moves
# a fix by about h * w_rms * dt (review/hardware_gap/out_t1.txt; w_rms 0.62 rad/s, the roll and pitch rate of the sim
# hover). There, at 100 m: 5 ms (0.31 m) kept the RMS error at 0.38 m (0.34 m with true stamps), 10 ms (0.62 m) doubled
# it, and a random error of up to 16.7 ms (p95 16 ms) gave 0.98 m, less than a fixed 16 ms. At 40 m:
# 0.31 m / (40 m * 0.62 rad/s) = 12 ms. One wrong stamp moves two intervals by its error, so the interval spread is
# not smaller than the stamp error; a slow change of the delay does not show in it.
STAMP_JITTER_MS = 12.0   # assumed: a hold at 40 m, and the real hover turns as fast as the sim hover (w_rms)


def stamp_jitter_ms(stamps):
    """95th percentile of |stamp interval - median interval| in ms, of (t_grab, seq) pairs in read order; None under
    10 intervals. seq also counts the buffers that newest() skipped: an interval over k buffers is compared with k
    median intervals."""
    if len(stamps) < 11:
        return None
    t, q = np.array(stamps, float).T
    dt, dq = np.diff(t), np.maximum(np.diff(q), 1)
    return float(np.percentile(np.abs(dt - dq * np.median(dt / dq)), 95) * 1e3)


def mode_check(s, args, say=print):
    fails = 0

    def line(ok, text):
        nonlocal fails
        fails += not ok
        say(f"{'PASS' if ok else 'FAIL'} {text}")

    line(s["calibrated"], "calibrated" if s["calibrated"] else "not calibrated. Run 'apb_main.py calib'.")
    fc = None
    try:
        fc = open_fc(s)
        line(True, f"firmware: {fc.firmware} (system {fc.sysid})")
    except Exception as e:
        line(False, f"flight controller: {e}")
    if fc is not None:
        import fc_link
        time.sleep(2.0)
        # ArduPilot sends LOCAL_POSITION_NED only once it has a position (from our fixes): its height stream is
        # GLOBAL_POSITION_INT (fc_link._read), so that is the one to measure on the ground.
        hkind = "GLOBAL_POSITION_INT" if fc.firmware == "ardupilot" else "LOCAL_POSITION_NED"
        r_att, r_pos = fc.rate("ATTITUDE"), fc.rate(hkind)
        line(r_att >= s["min_att_hz"], f"ATTITUDE {r_att:.0f} Hz (asked {s['att_hz']}, need {s['min_att_hz']})"
             + ("" if r_att >= s["min_att_hz"] else ". Turn off other streams (MAV_0_RATE 23040) and stop ULog streaming."))
        line(r_pos >= 0.5 * s["hgt_hz"], f"{hkind} {r_pos:.0f} Hz (asked {s['hgt_hz']})"
             + ("" if r_pos >= 0.5 * s["hgt_hz"] else ". The FC sends no height: check the baro and the EKF."))
        line(fc.clock.ready(), "FC clock offset fitted" if fc.clock.ready() else "FC clock offset not fitted: attitude too sparse.")
        if fc.firmware == "px4":
            fc.ping_timesync()
            time.sleep(0.3)
            rtt = np.array(fc.rtts) * 1e3
            ok = len(rtt) >= 10 and np.percentile(rtt, 90) < 10.0
            line(ok, f"TIMESYNC round trip p90 {np.percentile(rtt, 90):.1f} ms (PX4 needs under 10 ms)" if len(rtt) else
                 "TIMESYNC: no reply. Set EKF2_EV_DELAY to the fix age below.")
        for ok, text in fc_link.check_params(fc, s, write=args.set_params):
            line(ok, text[5:])
        if fc.firmware == "ardupilot":
            line(s["origin_lat_lon_alt"] is not None, "origin set" if s["origin_lat_lon_alt"] else
                 "origin_lat_lon_alt is empty. Put the field's lat, lon, alt in settings.json.")
        if fc.hgt.rows:
            say(f"INFO height now {fc.hgt.rows[-1][1]:.1f} m (run needs {s['min_height_m']:.0f} m)")
        fc.close()
    digital, k = s["camera"] == "digital", s["video_reduce"]
    shape = (int(s["video_size"][1]) // k, int(s["video_size"][0]) // k) if digital else (288, 360)
    fps = s["video_fps"] if digital else 25.0           # until measured
    try:
        import capture
        src = open_video(s)
        n, good, t1, stamps = 0, 0, time.monotonic(), []
        first = last = None
        while time.monotonic() - t1 < 2.0:
            fr = src.newest(timeout=0.5)
            if fr is not None:
                n, good, first, last = n + 1, good + (fr.why is None), first or fr, fr
                stamps.append((fr.t_grab, fr.seq))
        info = src.info
        cmd = getattr(getattr(src, "cap", None), "proc", None)     # video_command's child process
        code = None if cmd is None else cmd.poll()      # before close() stops it
        src.close()
        if digital:      # the buffer count (seq) also counts the buffers newest() skipped
            fps = (last.seq - first.seq) / (last.t_grab - first.t_grab) if n > 1 and last.t_grab > first.t_grab else 0.0
            line(first is not None, f"video: {n} frames in 2 s, {good} usable, driver {info.get('width')}x"
                 f"{info.get('height')}, pixel format {info.get('fourcc')}, buffer {info.get('buffer')}, stamps from {src.stamp}")
            ok = abs(fps - s["video_fps"]) <= 0.1 * s["video_fps"]
            line(ok, f"video rate {fps:.1f} fps (setting video_fps {s['video_fps']})" + ("" if ok else
                 ". Set video_fps to the camera's rate, or check "
                 f"{'the camera, video_command and pixel_format' if cmd is not None else 'the camera and video_device'}."))
            if cmd is not None and (first is None or code is not None):
                what = f"stopped with exit code {code}" if code is not None else "gave no frame in 2 s"
                line(False, f"video_command {what}. It must write raw {info.get('fourcc')} frames of {info.get('width')}x"
                     f"{info.get('height')} to its standard output. Read its error above, then check video_command and "
                     "pixel_format in settings.json.")
        else:
            fps = n / 2.0
            std = capture.standard(info, fps)
            want = s["video_standard"]
            line(first is not None, f"video: {n} frames in 2 s, {good} usable, driver {info.get('width')}x"
                 f"{info.get('height')} field order {info.get('field')}, stamps from {src.stamp}")
            line(std is not None and want in ("auto", std), f"video standard {std} (setting {want})")
        if first is not None:            # driver or read stamps: measured, not assumed (H1)
            j = stamp_jitter_ms(stamps)
            ok = j is not None and j <= STAMP_JITTER_MS
            line(ok, f"frame stamps from {src.stamp}: " + (
                f"{len(stamps) - 1} intervals in 2 s, too few to measure (need 10). Make the camera give frames, then "
                "run check again." if j is None else
                f"intervals differ from their median by {j:.1f} ms (p95, limit {STAMP_JITTER_MS:g} ms for a hold at "
                f"40 m)" + ("" if ok else ". A stamp error moves the fix. A lost frame also counts. Use driver stamps "
                                          "(video_device, no video_command), stop other programs on the board, and run "
                                          "check again.")))
            shape = first.img.shape
            say(f"INFO picture for Hold: {shape[1]}x{shape[0]} ({f'frame / {k}' if digital else s['field']})")
    except Exception as e:
        line(False, f"video: {e}")
    if digital:          # load_lens stretches a profile of another shape onto the picture, with no error
        import calib
        from hold import same_aspect
        try:
            pw, ph = calib.profile(S.here(s["lens_file"]), s["lens_name"])["image_size"]
            ok = same_aspect((pw, ph), shape[1], shape[0])
            line(ok, f"lens profile '{s['lens_name']}' is {pw}x{ph}, the picture {shape[1]}x{shape[0]}" + ("" if ok else
                 ". Their shapes differ. Set lens_name to a 16:9 profile, or apply the justice_d profile with "
                 "tools/use_profile.py."))
        except (ValueError, OSError) as e:
            line(False, f"lens file {s['lens_file']}: {e}")
    if s.get("lens_measured"):      # run refuses a measured lens of another shape (lens raises)
        import calib
        m = s["lens_measured"]
        try:
            calib.measured_lens(m, shape[1], shape[0])
            line(True, f"lens_measured ({m['by']}) is {m['image_size'][0]}x{m['image_size'][1]}, the picture "
                       f"{shape[1]}x{shape[0]}: run uses it, not the lens_name profile")
        except ValueError as e:
            line(False, str(e))
            s = dict(s, lens_measured=None)         # hold_timing below needs a lens that fits: it times the profile
    ms = hold_timing(s, shape)
    med = float(np.median(ms))
    rate = 1000.0 / med
    line(rate >= s["min_send_hz"], f"Hold.process median {med:.0f} ms, p95 {np.percentile(ms, 95):.0f} ms -> "
         f"about {min(rate, s['send_hz']):.0f} fixes/s (need {s['min_send_hz']})")
    age = s["video_delay_s"] * 1e3 + med + 5
    say(f"INFO fix age about {age:.0f} ms (video delay + process). Use it for EKF2_EV_DELAY when time sync "
        "fails. ArduPilot: after the first hover set VISO_DELAY_MS from 'apb_main.py delay <run log>'.")
    if s["mask_file"]:
        try:
            load_mask(s, shape)
            line(True, f"mask {s['mask_file']}")
        except ValueError as e:
            line(False, str(e))
    else:
        line(False, "no mask. Run 'apb_main.py mask'.")
    where = S.here(s["log_dir"])
    while not os.path.exists(where):     # run makes log_dir: measure the disk it will be on
        where = os.path.dirname(where)
    free = shutil.disk_usage(where).free / 1e9
    # Raw recording saves each picture Hold gets, at most at the camera rate and at record_hz (when set).
    # min_free_gb is for 360x288 at 25 Hz.
    rate = fps or (s["video_fps"] if digital else 25.0)
    if s["record_hz"] > 0:
        rate = min(rate, s["record_hz"])
    mbs = shape[0] * shape[1] * rate / 1e6
    # Never under min_free_gb: Raw stops recording under it, unscaled (no_sd, 10 Hz: the scaled value is 0.44 GB).
    need = max(s["min_free_gb"] * mbs / (360 * 288 * 25 / 1e6), s["min_free_gb"])
    rec_min = max(free - s["min_free_gb"], 0.0) * 1e3 / mbs / 60      # raw recording until Raw stops
    line(free >= need, f"free disk {free:.2f} GB at {where} (need {need:.2f} GB: raw recording takes {mbs:.1f} MB/s "
         f"at {shape[1]}x{shape[0]}, {rate:.0f} Hz), {rec_min:.0f} min of raw recording before the recorder stops"
         + ("" if free >= need else ". Set log_dir to a bigger disk (SD card: /sdcard/logs), or delete old logs."))
    if free >= need and rec_min < 10:
        say(f"WARN free disk: {rec_min:.0f} min of raw recording before the recorder stops at min_free_gb "
            f"{s['min_free_gb']:g} GB (under 10 min). run records from its start, also on the ground. Delete old "
            "logs, or set log_dir to an SD card.")
    say(f"{'ALL PASS' if fails == 0 else f'{fails} FAIL'}")
    return fails


def _recording(s, args, seconds, say):
    if args.rec:
        d = np.load(args.rec)
        return {k: d[k] for k in d.files}
    src = open_video(s)
    try:
        fc = open_fc(s)
    except BaseException:                # no FC: close the camera before the error goes on (as mode_run, D1)
        src.close()
        raise
    try:
        rec = record(src, fc, seconds, say)
    finally:
        src.close()
        fc.close()
    os.makedirs(S.here(s["log_dir"]), exist_ok=True)
    path = os.path.join(S.here(s["log_dir"]), f"rec_{stamp()}.npz")
    np.savez(path, **rec)
    say(f"INFO recording saved: {path}")
    return rec


def mode_mask(s, args, say=print):
    say("Turn and tilt the drone by hand while it records, so the ground moves and the OSD stays.")
    rec = _recording(s, args, args.seconds or 10, say)
    m = build_static_mask(rec["imgs"])
    frac = float(np.mean(m == 0))
    ok = frac < 0.5          # assumed: the OSD and airframe cover far less than half the picture (19 % on real/)
    path = S.here(s["mask_file"] or "mask.png")
    if ok:
        cv2.imwrite(path, m)
        s["mask_file"] = os.path.relpath(path, S.HERE) if path.startswith(S.HERE + os.sep) else path
        S.save(s, args.settings)
    say(f"{'PASS' if ok else 'FAIL'} mask covers {frac:.0%} of the picture"
        + (f"; saved {s['mask_file']}" if ok else ". The picture did not move enough: turn the drone more and run again."))
    return 0 if ok else 1


def mode_calib(s, args, say=print):
    import calib
    say("Point the camera at a scene more than 50 m away (drone on its side). Turn the drone by hand, "
        "+-15 to 20 deg about all three axes, about once every 2 s.")
    # The recording keeps the lens_name profile (calib's start), also when lens_measured is for another camera.
    rec = _recording(dict(s, lens_measured=None), args, args.seconds or s["calib_seconds"], say)
    mask = load_mask(s, rec["imgs"].shape[1:3])
    from hold import same_aspect
    lens_path = S.here(s["lens_file"])
    pw, ph = calib.profile(lens_path, s["lens_name"])["image_size"]
    h, w = rec["imgs"].shape[1:3]
    # A 16:9 profile is the digital camera: square pixels, so calib finds fx, fy and k1. No analog picture is
    # 16:9, and an analog profile keeps one focal scale.
    digital = same_aspect((16, 9), pw, ph)
    # A lens measured with a target is not fitted again: calib fits one focal scale against it, as for analog.
    target = (s.get("lens_measured") or {}).get("by") == "target"
    why = None
    if same_aspect((16, 9), w, h) != digital:
        why = (f"the picture is {w}x{h}, but lens profile '{s['lens_name']}' is {pw}x{ph}. Set lens_name "
               "to a profile with the same shape as the picture. For the digital camera, use a Justice D profile.")
    elif target:          # the recording's K and D become the target lens: focal_scale is measured against it
        try:
            K, D = lens(s, (h, w), focal=False)
            rec = dict(rec, K=K, D=D)
        except ValueError as e:
            why = str(e)
    r = dict(ok=False, why=why) if why else calib.solve(rec, s, mask, fit_lens=digital and not target)
    if not r["ok"]:
        say(f"FAIL calibration: {r['why']}")
        s["calibrated"] = False
        S.save(s, args.settings)
        return 1
    say(f"PASS picture top = {r['top']}{', mirrored' if r['mirror'] else ''} (residual {r['resid_deg']:.2f} deg; "
        f"next best {r['runner_up']['top']}{' mirrored' if r['runner_up']['mirror'] else ''} "
        f"{r['runner_up']['resid_deg']:.2f} deg, {r['ratio']:.0f}x)")
    if digital and not target:
        L, c = r["lens"], r["cam_rate_dps"]
        say(f"INFO video delay {r['video_delay_s'] * 1e3:.0f} ms, readout {r['readout_s'] * 1e3:+.0f} ms, lens at "
            f"{L['w']}x{L['h']}: fx {L['fx']:.1f} fy {L['fy']:.1f} k1 {L['k1']:+.4f}, tilt roll {r['tilt_deg'][0]:+.2f} "
            f"pitch {r['tilt_deg'][1]:+.2f} twist {r['twist_deg']:+.2f} deg")
        say(f"INFO quality: residual {r['resid_deg']:.3f} deg, tracked points reach {r['cover_deg']:.0f} deg from the "
            f"optical axis (picture corner {r['corner_deg']:.0f} deg), turn rate about picture x {c[0]:.0f}, y {c[1]:.0f}, "
            f"z {c[2]:.0f} deg/s")
        if not r["k1_fitted"]:
            say(f"FAIL k1 not found: {r['k1_why']}")
    else:
        say(f"INFO video delay {r['video_delay_s'] * 1e3:.0f} ms, readout {r['readout_s'] * 1e3:+.0f} ms, focal scale "
            f"{r['focal_scale']:.3f}{' against lens_measured (target)' if target else ''}, tilt roll {r['tilt_deg'][0]:+.2f} "
            f"pitch {r['tilt_deg'][1]:+.2f} twist {r['twist_deg']:+.2f} deg")
        if not 0.98 <= r["focal_scale"] <= 1.02:
            say("FAIL focal scale is outside 0.98-1.02: " + (
                "the lens measured with a target does not fit this recording. Measure the lens with the target again "
                "(tools/lens_calib.py --settings). Or set lens_measured to null, then run calib to fit the lens by hand turn."
                if target else "do the lens check (G2) before you fly."))
    prev = (s.get("calib_info") or {}).get("delay_s")
    calib.write(s, r, str(rec["mode"]) if "mode" in rec else "")
    S.save(s, args.settings)
    if target:
        say(f"PASS lens_measured (target) kept. focal_scale {r['focal_scale']:.3f} is measured against it.")
    elif digital:
        say(f"PASS measured lens written to lens_measured in {args.settings}. focal_scale is 1.0. lens_name "
            f"'{s['lens_name']}' stays as the start for calib. To use the profile lens again, set lens_measured to null.")
    if prev is not None:
        d = abs(r["video_delay_s"] - prev) * 1e3
        say(f"{'PASS' if d <= 3 else 'FAIL'} delay {d:.0f} ms from the previous calibration (need 3 ms or less)")
    say("PASS settings written; calibrated = true")
    return 0


# timing: the mode names (run log, mode column) that hold a position. fc_mode gives them; POS_HOLD: the Betaflight option.
HOLD_MODES = AP_HOLD | {"POSCTL", "AUTO_LOITER", "POS_HOLD"}
# timing: the 8 s of the window with the most turning must turn this many times faster (RMS) than the rest, else the
# window holds no wiggle. assumed: set on the same 4 rehearsal flights it was checked on, no hold-out data (wiggle
# windows as flown 4.4 to 8.7; windows that passed calib's checks with a wrong value 1.0 to 1.8; review/evidence/
# contrast.txt). Set high on purpose: a wrong FAIL keeps the ground calibration, a wrong PASS writes a wrong delay.
WIGGLE_RATIO = 4.0


def gap_s(t, lo, hi):
    """The longest time (s) in [lo, hi] with no value of t."""
    t = np.sort(t[(t >= lo) & (t <= hi)])
    return float(np.diff(np.r_[lo, t, hi]).max())


def wiggle_rates(att, lo, hi):
    """(RMS turn rate of the 8 s with the most turning, RMS turn rate of the rest), deg/s, of the FC attitude rows
    (t, roll, pitch, yaw rad) in [lo, hi], on a 0.1 s grid. None when the rows cover under 10 s of it."""
    tt = np.arange(max(lo, att[0, 0]), min(hi, att[-1, 0]), 0.1)
    if len(tt) < 100:
        return None
    ang = np.array([np.interp(tt, att[:, 0], c) for c in (att[:, 1], att[:, 2], np.unwrap(att[:, 3]))]).T
    e = np.sum((np.diff(ang, axis=0) / 0.1) ** 2, axis=1)
    c = np.convolve(e, np.ones(80), "valid")
    i = int(np.argmax(c))
    return math.degrees(math.sqrt(c[i] / 80)), math.degrees(math.sqrt(np.r_[e[:i], e[i + 80:]].mean()))


def mode_timing(s, args, say=print):
    """Video delay and readout from the in-flight wiggle (EVENT_DAY step 7): calib.py's own search with the
    calibrated picture orientation, in the flight's own light. apsim, 100 m, start 40 ms wrong: found 64.0 ms
    against a true 60.4 ms (integ/control/wiggle_*.txt). It uses only the raw frames from 16 s to 1 s before the
    first "engaged" note of the run log with 15 s in the air in no hold mode before it, and only when those 15 s hold
    a wiggle (item 8b): on the chunks of a hold in gusts it passed with 12 ms against a true 40 ms
    (integ/rehearsal_2026-11-20, D1; integ/integration_2026-11-20/review, B1 and B2)."""
    import calib
    keep_ground = " Settings not changed. Keep the delay and readout of the ground calibration, or fly the wiggle again."
    logs, raws = [f for f in args.files if f.endswith(".csv")], sorted(f for f in args.files if f.endswith(".npz"))
    if len(logs) != 1 or not raws:
        say("FAIL give one run log and the raw files of the same run: apb_main.py timing <log_dir>/run_<stamp>.csv "
            "<log_dir>/raw_<stamp>_*.npz")
        return 1
    # Raw and Log take their stamps (to 1 s) at the same start of run_live: the names of one run differ by 1 s at most.
    st = [re.search(r"_(\d{8}_\d{6})", os.path.basename(f)) for f in logs + raws]
    st = [time.mktime(time.strptime(m[1], "%Y%m%d_%H%M%S")) for m in st if m]
    if len(st) == 1 + len(raws) and max(abs(x - st[0]) for x in st) > 1:
        say(f"FAIL the raw files are not from the run of {logs[0]}: their names have another start time. Give "
            "raw_<stamp>_*.npz with the stamp of the run log.")
        return 1
    t, sw, rows = math.nan, [], []
    with open(logs[0]) as f:
        for row in csv.DictReader(f):
            t = float(row["t"]) if row["t"] else t       # a row with no frame has no t: use the last frame's
            if row["t"]:
                rows.append((t, row.get("mode") or "", float(row["height"]) if row.get("height") else math.nan))
            if (row["note"] or "").startswith("engaged"):
                sw.append(t)
    if not sw:
        say(f"FAIL no 'engaged' note in {logs[0]}: no switch to position hold, so the wiggle is not known."
            + keep_ground)
        return 1
    lt = np.array([r[0] for r in rows])
    held = np.array([r[1] in HOLD_MODES for r in rows], bool)
    lh = np.array([r[2] for r in rows])

    # The wiggle's switch: the first "engaged" note with 15 s in the air in no hold mode before it. A switch test on
    # the ground, or the FC already in a hold mode when the program starts, writes an earlier note.
    def why_not(x):
        w = (lt >= x - 16.0) & (lt <= x - 1.0)
        if math.isnan(x) or gap_s(lt, x - 16.0, x - 1.0) > 1.0:
            return "the run log covers only part of the 15 s before it (the program started later, or the camera stopped)"
        if held[w].any():
            return "position hold in the 15 s before it"
        if (lh[w] < s["min_height_m"]).any():          # no height (empty) is not checked
            return f"under min_height_m {s['min_height_m']:g} m in the 15 s before it (on the ground)"
        return None
    why = [why_not(x) for x in sw]
    sw0 = next((x for x, y in zip(sw, why) if y is None), None)
    skipped = "; ".join(f"switch at t {x:.1f} s: {y}" for x, y in zip(sw, why) if y)
    if sw0 is None:
        say(f"FAIL no switch to position hold after 15 s in the air in Altitude mode ({skipped})." + keep_ground)
        return 1
    d = [np.load(f) for f in raws]
    tg = [x["t_grab"] for x in d]
    # Each raw frame is in the run log at t = t_grab - video_delay_s (capture: Frame.t0). Raw files of another run
    # (the frame clock starts again at each boot) do not match. Rehearsal flights: 0.03 ms at most
    # (review/evidence/clock_check.txt).
    a, ls = np.concatenate(tg), np.sort(lt)
    b = a - s["video_delay_s"]
    j = np.clip(np.searchsorted(ls, b), 1, len(ls) - 1)
    miss = int(np.sum(np.minimum(abs(ls[j] - b), abs(ls[j - 1] - b)) > 0.001))
    if miss:
        say(f"FAIL {miss} of {len(a)} raw frames have no row in {logs[0]} at t_grab - video_delay_s "
            f"({s['video_delay_s'] * 1e3:.0f} ms): the raw files are from another run, or video_delay_s changed since "
            "this flight (timing writes it: run timing once for each flight)." + keep_ground)
        return 1
    # shortcut: the window compares t_grab with the log's t, so it is video_delay_s early (small against its 1 s
    # margin); kept so that the flown values stay the same. Use b (the log's t) for a new flight table.
    g = gap_s(a, sw0 - 16.0, sw0 - 1.0)
    if g > 1.0:
        say(f"FAIL the raw files miss {g:.1f} s of the 15 s before the switch at t {sw0:.1f} s: a raw chunk was "
            "dropped or is not given, or the recorder stopped (free disk)." + keep_ground)
        return 1
    keep = [(x >= sw0 - 16.0) & (x <= sw0 - 1.0) for x in tg]
    att = np.concatenate([x["att"] for x in d])
    rec = dict(imgs=np.concatenate([x["imgs"][k] for x, k in zip(d, keep) if k.any()]),    # only the window in memory
               t_grab=np.concatenate([x[k] for x, k in zip(tg, keep)]),
               att=att[np.unique(att[:, 0], return_index=True)[1]], K=d[0]["K"], D=d[0]["D"])
    say(f"INFO {len(rec['t_grab'])} frames from 16 s to 1 s before the switch at t {sw0:.1f} s"
        + (f" (switch {sw.index(sw0) + 1} of {len(sw)}; not used: {skipped})" if skipped else
           f" (the first of {len(sw)} switches)" if len(sw) > 1 else ""))
    w = wiggle_rates(rec["att"], sw0 - 16.0, sw0 - 1.0)
    if w is None or w[0] <= WIGGLE_RATIO * w[1]:          # <=: a window with no turning at all is no wiggle
        say(f"FAIL no wiggle in the 15 s before the switch at t {sw0:.1f} s: " + (
            "the FC attitude covers under 10 s of it." if w is None else
            f"its 8 s with the most turning turn {w[0]:.0f} deg/s RMS, the rest {w[1]:.1f} deg/s (need "
            f"{WIGGLE_RATIO:g} times). The switch came more than 8 s after the wiggle, or there was no wiggle.")
            + keep_ground)
        return 1
    r = calib.Rec(rec, load_mask(s, rec["imgs"].shape[1:3]))
    if r.rate_dps < s["calib_min_rate_dps"] or len(r.ia) < 1000:
        say(f"FAIL the recording turned {r.rate_dps:.0f} deg/s RMS with {len(r.ia)} matches; need "
            f"{s['calib_min_rate_dps']:.0f} deg/s and 1000. Rock the roll and pitch sticks harder for 8 s.")
        return 1
    R = make_R_bc(s["top"], s["mirror"], tuple(s["tilt_deg"]), s["twist_deg"])
    lat, ro, res = r.timing(R)
    best = r.coarse()[0]
    if best[0] * s["calib_min_ratio"] < res or res > s["calib_max_resid_deg"] or not 0.0 < lat < calib.LAT_MAX - 0.005:
        say(f"FAIL delay {lat * 1e3:.0f} ms, residual {res:.2f} deg (orientation {best[1]} fits {best[0]:.2f} deg). "
            "Settings not changed. Run calib on the ground.")
        return 1
    say(f"PASS video delay {lat * 1e3:.0f} ms (was {s['video_delay_s'] * 1e3:.0f}), readout {ro * 1e3:+.0f} ms "
        f"(was {s['readout_s'] * 1e3:+.0f}), residual {res:.2f} deg, {r.rate_dps:.0f} deg/s")
    s["video_delay_s"], s["readout_s"] = lat, ro
    S.save(s, args.settings)
    say("PASS settings written. Start run again, then set VISO_DELAY_MS from 'apb_main.py delay' after the hover.")
    return 0


def mode_bench(s, args, say=print, seconds=None):
    """Ground tests G1/H3/G2 with the propellers off: the hold at a fixed lens height, and every 0.5 s the
    fix's move since the first fix, in the body axes at that first fix (forward, right)."""
    s = dict(s, min_height_m=0.0, hold_cfg=dict(s["hold_cfg"], h_min=0.1))
    src = open_video(s)
    try:
        fc = open_fc(s)
    except BaseException:                # no FC: close the camera before the error goes on (as mode_run, D1)
        src.close()
        raise
    st = {"last": 0.0}

    def show(e, att):
        a = att.array()
        yaw = float(np.interp(e.t, a[:, 0], np.unwrap(a[:, 3])))
        if "p" not in st:
            st.update(p=e.pos.copy(), c=math.cos(yaw), s=math.sin(yaw))
        if time.monotonic() - st["last"] >= 0.5:
            st["last"] = time.monotonic()
            d = e.pos - st["p"]
            st["fwd"], st["right"] = d[0] * st["c"] + d[1] * st["s"], -d[0] * st["s"] + d[1] * st["c"]
            say(f"forward {st['fwd']:+.2f} m   right {st['right']:+.2f} m   ({e.state}, {e.n} points)")
    try:
        if fc.armed:
            say("FAIL the flight controller is armed. Disarm it and remove the propellers.")
            return 1
        say(f"Hold the lens {args.height:.2f} m above a textured floor. Press Ctrl+C to stop.")
        run_live(s, src, fc, seconds=seconds or args.seconds or 300, fixed_height=args.height, on_fix=show)
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
        fc.close()
    return 0 if "p" in st else 1


def mode_run(s, args, say=print):
    if not s["calibrated"]:
        say("FAIL not calibrated. Run 'apb_main.py calib'.")
        return 1
    if args.raw:
        s["raw_record"] = True
    while True:                          # a crash restarts clean and re-anchors to the FC position (H14)
        src = fc = None
        try:
            src = open_video(s)
            fc = open_fc(s)              # after src is set: when open_fc fails, finally closes the camera
            say(f"running; log in {S.here(s['log_dir'])}")
            run_live(s, src, fc)
        except KeyboardInterrupt:
            return 0
        except RuntimeError as e:
            say(f"FAIL {e}")
            if "will not start" in str(e):
                return 1
        except Exception:
            traceback.print_exc()
        finally:
            for x in (src, fc):
                if x is not None:
                    x.close()
        time.sleep(1.0)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=["check", "mask", "calib", "bench", "run", "replay", "delay", "timing"])
    p.add_argument("files", nargs="*", help="replay: raw recording .npz files; delay: run log .csv files; "
                                            "timing: one run log .csv and the raw .npz files of that run")
    p.add_argument("--settings", default=S.PATH)
    p.add_argument("--set-params", action="store_true", help="check: write wrong FC parameters")
    p.add_argument("--rec", help="mask/calib: use this recording instead of recording now")
    p.add_argument("--seconds", type=float, help="mask/calib: recording length")
    p.add_argument("--raw", action="store_true", help="run: also save raw frames and FC data")
    p.add_argument("--height", type=float, default=1.5, help="bench: lens height above the floor (m)")
    args = p.parse_args(argv)
    try:
        s = S.load(args.settings)
    except ValueError as e:
        print(f"FAIL settings: {e}")
        return 1
    if args.mode == "delay":          # VISO_DELAY_MS from a run log (the 'age_ms' of sent fixes)
        import csv as _csv
        a = [float(r["age_ms"]) for p in args.files for r in _csv.DictReader(open(p)) if r["sent"] == "1" and r["age_ms"]]
        if len(a) < 100:
            print(f"FAIL delay: {len(a)} sent fixes in the log; need 100 or more. Fly longer.")
            return 1
        print(f"PASS VISO_DELAY_MS {delay_advice(a):.0f} (from {len(a)} fixes; fix age at send p50 "
              f"{np.median(a):.0f} ms, p95 {np.percentile(a, 95):.0f} ms). PX4: EKF2_EV_DELAY = the p50 when "
              "time sync fails, and EKF2_DELAY_MAX at least the p95.")
        return 0
    if args.mode == "replay":
        files = [f for pat in args.files for f in sorted(glob.glob(pat))]
        if not files:
            print("FAIL replay: give the raw_*.npz files")
            return 1
        path, counts = replay(s, files)
        print(f"replay: {counts}; log {path}")
        return 0
    if args.mode == "run":
        return mode_run(s, args)
    tools = {"check": mode_check, "mask": mode_mask, "calib": mode_calib, "bench": mode_bench, "timing": mode_timing}
    try:
        return tools[args.mode](s, args)
    except (RuntimeError, ValueError) as e:     # no FC, no frames, "will not start", a bad mask: one FAIL line
        if isinstance(e, json.JSONDecodeError):  # after settings.json, the lens file is the only JSON these tools read
            e = f"the lens file {S.here(s['lens_file'])} is damaged: {e}"
        print(f"FAIL {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
