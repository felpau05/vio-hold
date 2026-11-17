#!/usr/bin/env python3
"""The anchor hold on the APB (i.MX8M Plus), and its event-day tools.

  apb_main.py check [--set-params]   firmware, parameters, stream rates, video, CPU time, disk
  apb_main.py mask  [--rec FILE]     record while the drone is turned by hand; build the OSD mask
  apb_main.py calib [--rec FILE]     record a hand rotation; solve orientation, delay, readout, focal, tilt
  apb_main.py bench [--height 1.5]  ground test, props off: the hold at a fixed height, moves in body axes
  apb_main.py run   [--raw]          the hold: send the fix to the flight controller
  apb_main.py replay FILE...         a raw recording through the same pipeline (for tuning)
  apb_main.py delay LOG.csv...       VISO_DELAY_MS from the fix ages of a flight
  apb_main.py timing RAW.npz...      video delay and readout from an in-flight stick wiggle (raw chunks)

Every check prints one line: PASS or FAIL, and what to do. run refuses to start when a check fails.
Safety rule: send nothing when in doubt (stale inputs, bad picture, low height, too few good frames).
"""
import argparse
import csv
import glob
import math
import os
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
    K, D = load_lens(S.here(s["lens_file"]), s["lens_name"], w, h, field={"top": 0, "bottom": 1}.get(s["field"]))
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


class Log:
    COLS = ["t", "state", "n", "north", "east", "var", "sent", "why", "proc_ms", "age_ms", "height", "note"]

    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.path, self.f = path, open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(self.COLS)

    def row(self, fr, e, sent, why, proc_ms, age_ms, height, note):
        f = lambda x: "" if x is None else f"{x:.4f}"      # noqa: E731
        self.w.writerow([f"{fr.t0:.4f}" if fr else "", e.state if e else "", e.n if e else "",
                         f(e.pos[0]) if e else "", f(e.pos[1]) if e else "", f(e.var) if e else "",
                         int(sent), why or "", f"{proc_ms:.1f}", f"{age_ms:.1f}" if age_ms else "",
                         f(height), note or ""])
        self.f.flush()                                    # a crash keeps every line written so far

    def close(self):
        self.f.close()


def stamp():
    return time.strftime("%Y%m%d_%H%M%S")


class Raw:
    """Raw recording in 10 s chunks (frames, FC attitude, height, position), saved by a thread."""

    def __init__(self, s, fc, shape):
        os.makedirs(S.here(s["log_dir"]), exist_ok=True)     # the run log may be elsewhere (apsim)
        self.base = os.path.join(S.here(s["log_dir"]), f"raw_{stamp()}")
        self.fc, self.K, self.D = fc, *lens(s, shape, focal=False)
        self.imgs, self.tg, self.k = [], [], 0

    def add(self, fr):
        self.imgs.append(fr.img)
        self.tg.append(fr.t_grab)
        if fr.t_grab - self.tg[0] >= 10.0:
            self.flush()

    def flush(self):
        if not self.imgs:
            return
        imgs, tg, self.imgs, self.tg = np.array(self.imgs), np.array(self.tg), [], []
        lo, hi = tg[0] - 1.0, tg[-1] + 1.0
        cut = lambda ser: np.array([r for r in list(ser.rows) if lo <= r[0] <= hi])     # noqa: E731
        path = f"{self.base}_{self.k:03d}.npz"
        self.k += 1
        data = dict(imgs=imgs, t_grab=tg, att=cut(self.fc.att), hgt=cut(self.fc.hgt), lpos=cut(self.fc.lpos),
                    K=self.K, D=self.D)
        threading.Thread(target=np.savez, args=(path,), kwargs=data, daemon=False).start()


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
    try:
        while time.monotonic() < t_end and not (stop is not None and stop.is_set()):
            fr = src.newest(timeout=0.5)
            if fr is None:
                pipe.good = 0
                note = wa.step(fc, None, False, time.monotonic()) if wa is not None else None
                log.row(None, None, False, "no frame", 0.0, None, None, note)
                continue
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
            log.row(fr, e, sent, why, proc_ms, (now - e.t) * 1e3 if e is not None else None, h, note)
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
    """Frames as Hold gets them plus FC attitude/height, in the live clock. Bad pictures are skipped."""
    imgs, tg, end, last = [], [], time.monotonic() + seconds, -1
    while time.monotonic() < end:
        left = int(end - time.monotonic())
        if left != last and left % 5 == 0:
            say(f"  recording: {left} s left")
            last = left
        fr = src.newest(timeout=0.5)
        if fr is not None and fr.why is None:
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
    return capture.V4L2Source(s)


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
    """Hold.process time on this CPU, on a synthetic moving texture at this picture size (ms list)."""
    rng = np.random.default_rng(0)
    h, w = shape
    tex = cv2.GaussianBlur(rng.normal(128, 40, (h + 200, w + 200)).astype(np.float32), (0, 0), 2.0)
    tex = np.clip((tex - tex.mean()) * 4 + 128, 0, 255).astype(np.uint8)
    s2 = dict(s, mask_file="")
    hold = make_hold(s2, shape)
    att, hgt = Series(1e9), Series(1e9)
    for t in np.arange(-1, n / 25 + 1, 0.01):
        att.add(t, 0.0, 0.0, 0.0)
    for t in np.arange(-1, n / 25 + 1, 0.1):
        hgt.add(t, 50.0)
    ms = []
    for i in range(n):
        dx, dy = int(50 + 30 * math.sin(i / 9)), int(50 + 30 * math.cos(i / 11))
        img = tex[dy:dy + h, dx:dx + w].copy()
        t1 = time.perf_counter()
        hold.process(img, i / 25, att, hgt)
        ms.append((time.perf_counter() - t1) * 1e3)
    return ms[5:]


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
        r_att, r_pos = fc.rate("ATTITUDE"), fc.rate("LOCAL_POSITION_NED")
        line(r_att >= s["min_att_hz"], f"ATTITUDE {r_att:.0f} Hz (asked {s['att_hz']}, need {s['min_att_hz']})"
             + ("" if r_att >= s["min_att_hz"] else ". Turn off other streams (MAV_0_RATE 23040) and stop ULog streaming."))
        line(r_pos >= 0.5 * s["hgt_hz"], f"LOCAL_POSITION_NED {r_pos:.0f} Hz (asked {s['hgt_hz']})"
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
    shape = (288, 360)
    try:
        import capture
        src = open_video(s)
        n, good, t1 = 0, 0, time.monotonic()
        first = None
        while time.monotonic() - t1 < 2.0:
            fr = src.newest(timeout=0.5)
            if fr is not None:
                n, good, first = n + 1, good + (fr.why is None), first or fr
        fps = n / 2.0
        info = src.info
        src.close()
        std = capture.standard(info, fps)
        want = s["video_standard"]
        line(first is not None, f"video: {n} frames in 2 s, {good} usable, driver {info.get('width')}x"
             f"{info.get('height')} field order {info.get('field')}, stamps from {src.stamp}")
        line(std is not None and want in ("auto", std), f"video standard {std} (setting {want})")
        if src.stamp == "read":
            line(False, "the driver gives no usable frame time; read time jitters by a frame (H1).")
        if first is not None:
            shape = first.img.shape
            say(f"INFO picture for Hold: {shape[1]}x{shape[0]} ({s['field']})")
    except Exception as e:
        line(False, f"video: {e}")
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
    free = shutil.disk_usage(S.here(".")).free / 1e9
    line(free >= s["min_free_gb"], f"free disk {free:.1f} GB (need {s['min_free_gb']})")
    say(f"{'ALL PASS' if fails == 0 else f'{fails} FAIL'}")
    return fails


def _recording(s, args, seconds, say):
    if args.rec:
        d = np.load(args.rec)
        return {k: d[k] for k in d.files}
    src, fc = open_video(s), open_fc(s)
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
    rec = _recording(s, args, args.seconds or s["calib_seconds"], say)
    mask = load_mask(s, rec["imgs"].shape[1:3])
    r = calib.solve(rec, s, mask)
    if not r["ok"]:
        say(f"FAIL calibration: {r['why']}")
        s["calibrated"] = False
        S.save(s, args.settings)
        return 1
    say(f"PASS picture top = {r['top']}{', mirrored' if r['mirror'] else ''} (residual {r['resid_deg']:.2f} deg; "
        f"next best {r['runner_up']['top']}{' mirrored' if r['runner_up']['mirror'] else ''} "
        f"{r['runner_up']['resid_deg']:.2f} deg, {r['ratio']:.0f}x)")
    say(f"INFO video delay {r['video_delay_s'] * 1e3:.0f} ms, readout {r['readout_s'] * 1e3:+.0f} ms, focal scale "
        f"{r['focal_scale']:.3f}, tilt roll {r['tilt_deg'][0]:+.2f} pitch {r['tilt_deg'][1]:+.2f} twist {r['twist_deg']:+.2f} deg")
    if not 0.98 <= r["focal_scale"] <= 1.02:
        say("FAIL focal scale is outside 0.98-1.02: do the lens check (G2) before you fly.")
    prev = (s.get("calib_info") or {}).get("delay_s")
    calib.write(s, r, str(rec["mode"]) if "mode" in rec else "")
    S.save(s, args.settings)
    if prev is not None:
        d = abs(r["video_delay_s"] - prev) * 1e3
        say(f"{'PASS' if d <= 3 else 'FAIL'} delay {d:.0f} ms from the previous calibration (need 3 ms or less)")
    say("PASS settings written; calibrated = true")
    return 0


def mode_timing(s, args, say=print):
    """Video delay and readout from the raw chunks of an in-flight wiggle (EVENT_DAY step 7): calib.py's own
    search with the calibrated picture orientation, in the flight's own light. apsim, 100 m, start 40 ms
    wrong: found 64.0 ms against a true 60.4 ms (integ/control/wiggle_*.txt)."""
    import calib
    d = [np.load(f) for f in sorted(args.files)]
    att = np.concatenate([x["att"] for x in d])
    rec = dict(imgs=np.concatenate([x["imgs"] for x in d]), t_grab=np.concatenate([x["t_grab"] for x in d]),
               att=att[np.unique(att[:, 0], return_index=True)[1]], K=d[0]["K"], D=d[0]["D"])
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
    src, fc = open_video(s), open_fc(s)
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
            src, fc = open_video(s), open_fc(s)
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
    p.add_argument("files", nargs="*", help="replay: raw recording .npz files; delay: run log .csv files")
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
    if args.mode == "timing":
        return mode_timing(s, args)
    modes = {"check": mode_check, "mask": mode_mask, "calib": mode_calib, "bench": mode_bench, "run": mode_run}
    return modes[args.mode](s, args)


if __name__ == "__main__":
    sys.exit(main())
