#!/usr/bin/env python3
"""Closed-loop test: real PX4 (SIH simulator in Docker) flies, wind pushes, and the only horizontal
position PX4 gets is the one our estimator computes from a rendered camera picture.

Run one case:  ~/.venvs/vio/bin/python px4sim/sitl.py [variant] [height] [wind] [hold_s]
               variant: ours | truth | none | apb | gps   wind: calm | w20g40 | w30g55
Many cases:    px4sim/matrix.py

Everything is paced by the simulator clock (HIL_STATE_QUATERNION / ATTITUDE stamps), never by
the wall clock. Recipe from research/sitl.md: px4io/px4-sitl image, SIH, GNSS off, baro height,
Altitude-mode climb, vision on, Position-mode hold, wind through SIH_WIND_N / SIH_WIND_E.
"""
import collections
import json
import math
import multiprocessing
import os
import struct
import subprocess
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path.insert(0, PROJ)
sys.path.insert(0, HERE)
import camera as C  # noqa: E402

IMAGE, NAME, PORT = "px4io/px4-sitl:latest", "px4_sim", 14540
GCS_PORT = 14550    # PX4 SITL's GCS link: the supervisor (sticks, modes, truth) when the onboard program has 14540
OUT = os.path.join(HERE, "out")
RUNS = os.path.join(HERE, "runs")
HOME = (45.417, -75.715)        # the ground photos' centre (ground/sites.json)
HOME_ALT = 70.0                 # assumed: Ottawa river flats are about 70 m above sea level; only the baro sees it
G = 9.81

# ------------------------------------------------------------------ vehicles
# Known: frame 478 g, 6S 7000 mAh battery 868 g, 2814 880 kV motors, 10-inch props, payload up to 3.5 kg.
T_MAX = 5.3 * G / 0.7 / 4       # assumed: 1.8 kg drone + 3.5 kg payload hovers at 70 % thrust -> 18.6 N per motor
ARM = 0.159                     # assumed: 450 mm motor-to-motor diagonal (typical 10-inch frame); arm per axis
Q_RATIO = 0.017                 # assumed: propeller torque / thrust = 0.017 m (10-inch prop, Cq/Ct ~ 0.07)
VEHICLES = {"light": 1.8, "medium": 2.3, "heavy": 3.5}   # assumed take-off weights (kg); not published


def vehicle(mass):
    """SIH model plus the matching control allocation and rate-loop scale for one take-off weight."""
    m_mot = 0.4                 # assumed: 4 motors with props, 100 g each, at the arm ends
    rest = mass - m_mot         # assumed: everything else as a 0.2 m square slab at the centre
    ixx = m_mot * ARM ** 2 + rest * 0.2 ** 2 / 12
    izz = m_mot * 2 * ARM ** 2 + rest * 2 * 0.2 ** 2 / 12
    p = dict(SIH_MASS=mass, SIH_IXX=ixx, SIH_IYY=ixx, SIH_IZZ=izz, SIH_L_ROLL=ARM, SIH_L_PITCH=ARM,
             SIH_T_MAX=T_MAX, SIH_Q_MAX=Q_RATIO * T_MAX,
             SIH_KDV=mass * G * math.tan(math.radians(20)) / 10,   # assumed: 20 deg of tilt holds 10 m/s of wind
             MPC_THR_HOVER=mass * G / (4 * T_MAX))                 # SIH thrust is linear in the motor signal
    # PX4's default rate gains fly SIH's default quad (L 0.2 m, T 5 N, I 0.025/0.03, Q 0.1 Nm). Scale the
    # gain by the ratio of angular-acceleration authority so that the rate loops keep the same bandwidth.
    p["MC_ROLLRATE_K"] = p["MC_PITCHRATE_K"] = (0.2 * 5.0 / 0.025) / (ARM * T_MAX / ixx)
    p["MC_YAWRATE_K"] = (0.1 / 0.03) / (Q_RATIO * T_MAX / izz)
    for i, (sx, sy, km) in enumerate(((1, 1, 1), (-1, -1, 1), (1, -1, -1), (-1, 1, -1))):   # SIH motor order
        p[f"CA_ROTOR{i}_PX"], p[f"CA_ROTOR{i}_PY"], p[f"CA_ROTOR{i}_KM"] = sx * ARM, sy * ARM, km * Q_RATIO
    return p


BASE = dict(SYS_HAS_GPS=0, SIM_GPS_USED=0, SENS_EN_GPSSIM=0, EKF2_GPS_CTRL=0, EKF2_HGT_REF=0, EKF2_BARO_CTRL=1,
            EKF2_EV_CTRL=1, EKF2_EV_NOISE_MD=0, EKF2_RNG_CTRL=0, SIH_DISTSNSR_EN=0,
            MPC_Z_VEL_MAX_UP=5.0)   # shortcut: the sim climbs at 5 m/s to save time; the hold does not depend on it

# ------------------------------------------------------------------ cases
WINDS = {"calm": (0.0, 0.0), "w20g40": (20.0, 40.0), "w30g55": (30.0, 55.0)}   # mean, gust peak (km/h)
# w30g55: assumed mean of 30 km/h; the scope only gives the 55 km/h gust.

DEFAULTS = dict(
    variant="ours", vehicle="medium", height=100.0, wind="calm", hold_s=60.0, seed=1,
    wind_from_deg=270.0,       # most frequent November direction at Ottawa (research/site_weather.md)
    ground="ottawa2019fall",   # or "snow", "snow_tarps", "snow_tarps_pad", "night_lights" (with exposure_s=0.02 noise=10)
    video="PAL",               # PAL 25 fps 360x288 (measured on the real video) or NTSC 30 fps 360x240
    readout_s=0.0184,          # assumed: active lines of one PAL field (288 of 312.5 lines x 20 ms); < 0 = bottom first
    exposure_s=0.004,          # assumed: daylight
    video_delay_s=0.04,        # assumed: one PAL frame in the capture chain
    stamp_err_s=0.0,           # error of the frame time stamp the estimator gets (calibration residual)
    proc_s=0.0,                # simulated board compute time per frame (sim s); 0 = as fast as this laptop
    noise=3.7,                 # measured: picture noise std on the real analog video (grey levels)
    noise_corr_px=1.5,         # assumed: correlation length along x (measured only as "correlated along x")
    vib_deg=0.2,               # assumed: camera vibration the FC never sees (as sim_ground.py)
    true_lens="Round 1 Metashape fisheye", model_lens="Round 1 independent k2",
    focal_scale=1.0, pp_shift=(0.0, 0.0),
    cam_top="front", cam_mirror=False, cam_tilt=(0.0, 0.0), cam_twist=0.0,
    est_top=None, est_mirror=None, est_tilt=None, est_twist=None,   # None = what the estimator is told = truth
    osd=False, osd_mask=True, breakup=0.0, freeze=None, repeat=0.0,
    freeze_noise=False,        # with freeze=(t, dur): the frozen picture gets fresh analog noise (not an exact repeat)
    att_hz=100.0,              # PX4 onboard-link default rate for ATTITUDE
    hgt_bias_m=0.0, hgt_noise_m=0.0,
    ev_delay_s=None,           # None: video delay + half readout + half exposure + processing budget
    speed=1.0, flyaway_m=30.0, video_out=None, label=None,
    engage_wait_s=25.0,        # give up when PX4 has not accepted Position mode this long after the feed starts
    att_source="px4",          # "px4": ATTITUDE as sent. "gyro": rates integrated, leaning on PX4's attitude with
                               # att_tau_s. "truth" (diagnostic): true roll/pitch, yaw turned into PX4's frame
    att_tau_s=20.0,            # assumed: far slower than the 2-3 s period of the PX4-tilt feedback at 50-100 m
    px4={},                    # extra PX4 parameters for this case, e.g. {"EKF2_EVP_NOISE": 0.5}
    hold_cfg={},               # Hold tuning overrides (hold.Hold(..., **cfg)), e.g. {"MOVE_PX": 0.3}
    outage=None,               # (t, dur) s after the hold starts: the video link drops; the receiver shows static
    breakup_from=-math.inf,    # break-up bursts only from this time after the hold starts (0: during the hold only)
    # variant "apb": the onboard program (apb_main.run_live + fc_link) in its own process (px4sim/onboard.py)
    settings={},               # settings.json overrides for the onboard program, e.g. {"att_source": "fc"}
    cpu_factor=10.0,           # assumed: the APB's A53 is about 10x slower than this laptop (CORE estimate)
    reanchor_on_lost=None,     # None: apb_main's default; True/False: A/B test of Pipe's re-anchor on 'lost'
    sync_wait_s=55.0,          # PX4 uses our time stamps after 500 TIMESYNC replies at 10 Hz (Timesync.hpp)
    est_delay_err_s=0.0,       # error of the video_delay_s setting (a calibration error)
    still_start=False,         # the camera shows analog static until the hover before Position mode: the hold must
                               # start from a still hover (wind starts only with the hold)
    # Indoor scenes (camera.py): ground="floor_concrete|tiles|carpet|wood|shiny" (tuning), "holdout_floor_a|b" (hold-out)
    flicker_hz=0.0, flicker_depth=0.5,   # mains lamp flicker (100 or 120 Hz); depth assumed: 50 % (cheap LED, old tubes)
    people=0,                  # people walking through the picture (count)
    reflect=False, ceiling_m=None,       # shiny floor: ceiling lamps mirrored in the floor
    props=False,               # blurred spinning propellers and arms in the picture corners
    mask_corners=0.0,          # mask for the onboard program: quarter discs of this radius (x picture height) at the
    mask_border=0,             # corners, and a border band of this many px (hold.corner_mask)
    # Sensor errors, injected in PX4's own simulated sensors, so the autopilot AND the onboard program (which reads
    # PX4's ATTITUDE and LOCAL_POSITION_NED) see them, as on the drone:
    baro_wander_m=0.0,         # baro height error: smooth random wander with this peak (m) during the hold
    baro_tau_s=20.0,           # assumed: time constant of that wander (indoor air drafts, doors, thermal drift)
    mag_yaw_deg=0.0,           # compass: a heading error that steps in at the hold start (field near steel changes)
    mag_yaw_dps=0.0,           # compass: plus a heading drift during the hold (deg/s)
)


def ev_delay(c):
    """Time from the fix's own time stamp to its arrival at PX4. We do not answer TIMESYNC, so PX4
    stamps the arrival and subtracts EKF2_EV_DELAY; the sender holds each fix until exactly then."""
    if c["ev_delay_s"] is not None:
        return c["ev_delay_s"]
    budget = max(c["proc_s"], 0.05)   # assumed: render + Hold take < 50 ms per frame on this laptop at speed 1
    return c["video_delay_s"] + abs(c["readout_s"]) / 2 + c["exposure_s"] / 2 + budget


# ------------------------------------------------------------------ container
def docker_stop():
    subprocess.run(["docker", "rm", "-f", NAME], capture_output=True)


def docker_start(params, speed):
    docker_stop()
    env = []
    for k, v in params.items():
        env += ["-e", f"PX4_PARAM_{k}={v:.6g}" if isinstance(v, float) else f"PX4_PARAM_{k}={v}"]
    cmd = ["docker", "run", "-d", "--rm", "--name", NAME, "--network", "host",
           "-e", f"PX4_SIM_SPEED_FACTOR={speed:g}", "-e", f"PX4_HOME_LAT={HOME[0]}",
           "-e", f"PX4_HOME_LON={HOME[1]}", "-e", f"PX4_HOME_ALT={HOME_ALT}", *env, IMAGE, "-d"]
    subprocess.run(cmd, check=True, capture_output=True)


# ------------------------------------------------------------------ MAVLink link
class Link:
    """MAVLink to PX4 on the onboard port. A receive thread keeps the truth, PX4's attitude and height,
    and the mode; a stick thread sends MANUAL_CONTROL without gaps (a gap makes PX4 land)."""

    def __init__(self, c, port=PORT):
        import hold
        from pymavlink import mavutil
        self.mav = mavutil.mavlink
        self.m = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}", source_system=255)
        self.arr = np.zeros(400_000)                  # arrival time (time.monotonic) of each truth sample
        self.c, self.lock, self.done, self.stop_all = c, threading.Lock(), False, False
        self.truth = np.zeros((400_000, 8))           # t, north, east, height, qw, qx, qy, qz
        self.n_truth = 0
        self.att, self.hgt = hold.Series(30.0), hold.Series(30.0)
        self.att_rows, self.ekf = [], []              # full logs (Series trims itself)
        self.mode, self.armed, self.texts = 0, False, []
        self.z0, self.alt0, self.throttle, self.yaw_off = None, None, 500, 0.0
        self.t_engage, self.ev_delay, self.sent, self.late = math.inf, ev_delay(c), 0, []
        self.rng = np.random.default_rng(c["seed"] + 101)
        self.hgt_noise, self.whys, self.q_f, self.t_f, self.w_f = 0.0, {}, None, None, None
        self.mag = None

    def start(self, timeout=90.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if self.m.wait_heartbeat(timeout=2.0):
                break
        else:
            raise RuntimeError("no heartbeat from PX4")
        self.ts = (self.m.target_system, self.m.target_component)
        threading.Thread(target=self._recv, daemon=True).start()
        threading.Thread(target=self._sticks, daemon=True).start()
        self.interval(115, 250.0)                     # HIL_STATE_QUATERNION: truth, as fast as SIH gives it
        self.interval(30, self.c["att_hz"])           # ATTITUDE (with rates): what the board reads
        self.interval(32, 30.0)                       # LOCAL_POSITION_NED: PX4's height and position
        if self.c["mag_yaw_deg"] or self.c["mag_yaw_dps"]:
            self.interval(105, 10.0)                  # HIGHRES_IMU: the body-frame magnetic field (compass error case)

    # -- receive
    def _recv(self):
        lat0, lon0 = map(math.radians, HOME)
        while not self.stop_all:
            msg = self.m.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            k = msg.get_type()
            if k == "HIL_STATE_QUATERNION":
                if self.n_truth >= len(self.truth):
                    continue
                n = (math.radians(msg.lat * 1e-7) - lat0) * 6371000.0
                e = (math.radians(msg.lon * 1e-7) - lon0) * 6371000.0 * math.cos(lat0)
                if self.alt0 is None:
                    self.alt0 = msg.alt * 1e-3
                self.truth[self.n_truth] = (msg.time_usec * 1e-6, n, e, msg.alt * 1e-3 - self.alt0,
                                            *msg.attitude_quaternion)
                self.arr[self.n_truth] = time.monotonic()
                self.n_truth += 1
            elif k == "ATTITUDE":
                row = (msg.time_boot_ms * 1e-3, msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)
                self.att_rows.append(row)
                if self.c["att_source"] == "gyro":
                    row = self._gyro_att(row)
                elif self.c["att_source"] == "truth" and self.n_truth > 2:
                    q = self.truth_at(np.array([row[0]]))[3]
                    r, p = C.q_roll_pitch(q)
                    row = (row[0], float(r[0]), float(p[0]), float(C.q_yaw(q)[0]) + self.yaw_off, *row[4:])
                self.att.add(*row)
            elif k == "LOCAL_POSITION_NED":
                t = msg.time_boot_ms * 1e-3
                self.ekf.append((t, msg.x, msg.y, msg.z))
                if self.z0 is not None:
                    if self.c["hgt_noise_m"] > 0:        # random walk with that std after 1 s (assumed shape)
                        self.hgt_noise += self.rng.normal(0, self.c["hgt_noise_m"] / math.sqrt(30))
                    self.hgt.add(t, self.z0 - msg.z + self.c["hgt_bias_m"] + self.hgt_noise)
            elif k == "HIGHRES_IMU":
                self.mag = (msg.xmag, msg.ymag, msg.zmag)          # gauss, body frame (with any injected offset)
            elif k == "HEARTBEAT" and msg.get_srcComponent() == 1:
                self.mode = (msg.custom_mode >> 16) & 0xFF
                self.armed = bool(msg.base_mode & 128)
            elif k == "STATUSTEXT":
                self.texts.append((self.now(), msg.text))

    def _gyro_att(self, row):
        """Attitude that PX4's vision fusion cannot pull quickly: integrate the ATTITUDE body rates and
        lean toward PX4's attitude with time constant att_tau_s. While PX4 fuses our fix it switches its
        gravity fusion off, so its tilt follows our fix, and our fix follows its tilt (gain = height)."""
        t, r, p, y, wx, wy, wz = row
        qe = C.qmul(C.q_axis(C.E_D, y), C.qmul(C.q_axis(C.E_E, p), C.q_axis(C.E_N, r)))   # yaw, pitch, roll
        w = np.array([wx, wy, wz])
        if self.q_f is None:
            self.q_f = qe
        else:
            dt = t - self.t_f
            wm = (w + self.w_f) / 2
            if np.linalg.norm(wm) > 0 and dt > 0:
                self.q_f = C.qmul(self.q_f, C.q_axis(wm, np.linalg.norm(wm) * dt))      # body-frame turn
            d = C.qmul(qe, self.q_f * np.array([1, -1, -1, -1.0]))                    # PX4 minus ours, NED
            phi = 2 * d[1:] * np.sign(d[0])
            if np.linalg.norm(phi) > 0 and dt > 0:
                self.q_f = C.qmul(C.q_axis(phi, np.linalg.norm(phi) * min(1.0, dt / self.c["att_tau_s"])), self.q_f)
            self.q_f = self.q_f / np.linalg.norm(self.q_f)
        self.t_f, self.w_f = t, w
        rr, pp = C.q_roll_pitch(self.q_f)
        return (t, float(rr), float(pp), float(C.q_yaw(self.q_f)), wx, wy, wz)

    def _sticks(self):
        k = 0
        while not self.stop_all:
            with self.lock:
                self.m.mav.manual_control_send(self.ts[0], 0, 0, int(self.throttle), 0, 0)
                if k % 25 == 0:
                    self.m.mav.heartbeat_send(6, 8, 0, 0, 0)        # GCS heartbeat, 1 Hz
            k += 1
            time.sleep(0.04)

    # -- time
    def now(self):
        return self.truth[self.n_truth - 1, 0] if self.n_truth else 0.0

    def mono_of(self, t_sim):
        """time.monotonic at sim time t_sim, as a capture driver stamps a buffer: the smallest arrival
        delay of the truth samples within 50 ms before t_sim (a late sample does not move it). Local,
        because the sim clock runs at 0.95-0.98 x wall time and the rate changes with the load."""
        n = self.n_truth
        a = self.truth[:n, 0]
        i = int(np.searchsorted(a, t_sim))
        j0, j1 = max(0, i - 12), min(n, i + 1)
        if j1 - j0 < 3:
            j0, j1 = max(0, n - 12), n
        return t_sim + float(np.min(self.arr[j0:j1] - a[j0:j1]))

    def wait_until(self, t, timeout=30.0):
        w = time.time() + timeout
        while self.now() < t and not self.done and time.time() < w:
            time.sleep(0.001)

    def wait_for(self, dt):
        self.wait_until(self.now() + dt)

    # -- truth
    def truth_at(self, t):
        """True (north, east, height, q) at times t: linear for position, normalised lerp for attitude
        (samples are 4-8 ms apart, so the lerp error is far below a pixel)."""
        a = self.truth[:self.n_truth]
        t = np.atleast_1d(np.asarray(t, float))
        i = np.clip(np.searchsorted(a[:, 0], t) - 1, 0, len(a) - 2)
        f = np.clip((t - a[i, 0]) / np.maximum(a[i + 1, 0] - a[i, 0], 1e-9), 0, 1)[:, None]
        v = a[i, 1:] * (1 - f) + a[i + 1, 1:] * f
        q0 = a[i, 4:]
        q1 = a[i + 1, 4:] * np.sign(np.sum(q0 * a[i + 1, 4:], axis=1))[:, None]
        q = q0 * (1 - f) + q1 * f
        return v[:, 0], v[:, 1], v[:, 2], q / np.linalg.norm(q, axis=1)[:, None]

    def h_now(self):
        return self.hgt.rows[-1][1] if self.hgt.rows else 0.0

    def ekf_xy(self):
        return np.array(self.ekf[-1][1:3]) if self.ekf else np.zeros(2)

    # -- send
    def cmd(self, command, *p):
        p = list(p) + [0] * (7 - len(p))
        with self.lock:
            self.m.mav.command_long_send(*self.ts, command, 0, *p)

    def interval(self, msg_id, hz):
        self.cmd(511, msg_id, 1e6 / hz)

    def set_mode(self, main):           # PX4 main modes: 2 Altitude, 3 Position
        self.cmd(176, 1, main)

    def arm(self):
        self.cmd(400, 1)

    def param(self, name, value):
        with self.lock:
            self.m.mav.param_set_send(*self.ts, name.encode(), float(value), self.mav.MAV_PARAM_TYPE_REAL32)

    def send_vision(self, t, pos, cov):
        """VISION_POSITION_ESTIMATE as research/px4_vision.md requires: NED x, y; a finite z (PX4 drops the
        whole position if z is NaN; EKF2_EV_CTRL=1 does not fuse it); attitude NaN (not used);
        variances in m^2. Held until t + EKF2_EV_DELAY so that PX4's arrival stamp is the right time."""
        self.wait_until(t + self.ev_delay)
        late = self.now() - (t + self.ev_delay)
        cv = [0.0] * 21
        cv[0], cv[1], cv[6], cv[11] = float(cov[0][0]), float(cov[0][1]), float(cov[1][1]), 1.0
        z = self.ekf[-1][3] if self.ekf else 0.0
        with self.lock:
            self.m.mav.vision_position_estimate_send(int(t * 1e6), float(pos[0]), float(pos[1]), float(z),
                                                     math.nan, math.nan, math.nan, cv, 0)
        self.sent += 1
        self.late.append(late)
        return late

    def close(self):
        self.done = self.stop_all = True
        time.sleep(0.3)
        self.m.close()


# ------------------------------------------------------------------ estimator loop
def estimator_loop(frames, link, hold, log):
    """Frames in, VISION_POSITION_ESTIMATE out: the piece that the onboard pipeline replaces.
    frames yields (picture, t0 = mid-exposure time of row 0 in PX4's clock); link gives PX4's attitude
    (link.att) and height (link.hgt) Series, PX4's own position (link.ekf_xy) and send_vision."""
    att, hgt = type(link.att)(30.0), type(link.hgt)(30.0)
    for img, t0 in frames:
        w = time.perf_counter()
        # Hold reads its Series several times per frame; a row added by the receive thread in between
        # gives arrays of different lengths (hold.attitude_R crashed this way). Copy new rows here.
        for src, dst in ((link.att, att), (link.hgt, hgt)):
            for r in [r for r in src.rows[-1000:] if r[0] > dst.last_t()]:
                dst.add(*r)
        e = hold.process(img, t0, att, hgt)
        proc = time.perf_counter() - w
        if e.state == "lost":
            hold.reanchor(link.ekf_xy())     # restart from PX4's own estimate, so the EKF sees no jump
        late = math.nan
        if e.var is not None:
            cov = getattr(e, "cov", None)
            cov = np.eye(2) * e.var / 2 if cov is None else np.asarray(cov)
            if link.c["proc_s"] > 0:                  # simulated board compute time
                link.wait_until(frames.t_avail + link.c["proc_s"])
            late = link.send_vision(e.t, e.pos, cov)
        log.append((e.t, *e.pos, e.var if e.var is not None else math.nan, STATES.index(e.state), e.n, proc, late))
        if e.why:
            w = e.why.split(" (")[0]
            link.whys[w] = link.whys.get(w, 0) + 1


STATES = ["init", "ok", "degraded", "lost"]


class TruthHold:
    """Stands in for Hold: the true position, turned into PX4's heading frame (PX4's yaw comes from the
    magnetometer and can differ from true north). The best fix any estimator could send."""

    def __init__(self, link, yaw_off, readout):
        self.link, self.readout = link, readout
        c, s = math.cos(yaw_off), math.sin(yaw_off)
        self.R = np.array([[c, -s], [s, c]])

    def process(self, img, t0, att, hgt):
        import hold
        t = t0 + self.readout / 2
        N, E, _, _ = self.link.truth_at(np.array([t]))
        return hold.Est(t, self.R @ np.array([N[0], E[0]]), 0.02, 0, "ok", False, None)

    def reanchor(self, pos, var=None):
        pass


def apb_results(c, link, child, stop_ev, tx, status_q, log_path, seen, t_end):
    """Stop the onboard program; fix statistics over the hold from its own log (time.monotonic)."""
    stop_ev.set()
    try:
        tx.send(None)
    except (BrokenPipeError, OSError):
        pass
    child.join(timeout=10)
    alive = child.is_alive()
    msgs = []
    while not status_q.empty():
        msgs.append(status_q.get())
    out = dict(onboard=[(round(t, 2), k, x[:300]) for t, k, x in msgs
                        if k not in ("param", "clock") or (k == "param" and not x.startswith("PASS"))][-20:])
    out["params_fail"] = [x for _, k, x in msgs if k == "param" and not x.startswith("PASS")]
    out["onboard_exit"] = "still running (hung)" if alive else child.exitcode
    logs = sorted(os.path.join(RUNS, f) for f in os.listdir(RUNS)
                  if f.startswith(os.path.basename(log_path)[:-4]) and f.endswith(".csv"))
    out["_logs"] = logs
    if link.t_engage == math.inf:
        return out
    # the onboard clock mapping (fc_link.Clock: FC boot time -> onboard clock = sim clock here)
    clk = np.array([x for _, k, x in msgs if k == "clock"], float).reshape(-1, 2)
    clk = clk[(clk[:, 0] >= link.t_engage) & (clk[:, 0] <= t_end)]
    if len(clk):
        e = 1e3 * (clk[:, 1] - clk[:, 0])
        out.update(clock_err_ms_mean=float(e.mean()), clock_err_ms_absmax=float(np.abs(e).max()))
    rows = onboard_log(logs, link.t_engage, t_end)
    T = max(t_end - link.t_engage, 1e-9)
    n_frames = sum(1 for t, _ in seen if link.t_engage <= t <= t_end)
    lat = np.array([d for t, d in seen if link.t_engage <= t <= t_end])
    if len(lat):      # simulator check: frame hand-over delay after its stamp (render + transfer)
        out.update(handover_ms_p50=1e3 * float(np.median(lat)), handover_ms_p95=1e3 * float(np.percentile(lat, 95)))
    sent = [r for r in rows if r["sent"] == "1"]
    st = [r["state"] for r in rows if r["state"]]
    whys = collections.Counter(r["why"].split(" (")[0] for r in rows if r["why"])
    age = np.array([float(r["age_ms"]) for r in sent if r["age_ms"]])
    proc = np.array([float(r["proc_ms"]) for r in rows if r["state"]])
    out.update(frames=n_frames, processed=len(st), fixes=len(sent), fix_rate_hz=len(sent) / T,
               pct_frames_fix=100.0 * len(sent) / max(n_frames, 1),
               **{f"frac_{s}": (st.count(s) / len(st) if st else 0.0) for s in STATES},
               proc_ms_mean=float(proc.mean()) if len(proc) else None,
               age_ms_p50=float(np.median(age)) if len(age) else None,
               age_ms_p95=float(np.percentile(age, 95)) if len(age) else None,
               hold_why=dict(whys.most_common(8)),
               reanchors=sum(1 for r in rows if r["note"].startswith("re-anchored")))
    return out


# ------------------------------------------------------------------ wind
def wind_series(c, T, dt=0.1):
    """Mean wind plus gusts: a filtered random process (Ornstein-Uhlenbeck) plus 1-cosine gust steps,
    scaled so that the peak equals the gust speed. The mean ramps in over the first 5 s.
    Returns t and the SIH parameters (SIH_WIND_N/E are the 'wind from' components)."""
    U, Gk = (v / 3.6 for v in WINDS[c["wind"]])
    t = np.arange(0, T + dt, dt)
    if U == 0 and Gk == 0:
        return t, np.zeros_like(t), np.zeros_like(t)
    rng = np.random.default_rng(c["seed"] + 13)
    tau, tau_d = 4.0, 6.0                        # assumed: gust and direction time constants (s)
    x, d = np.zeros_like(t), np.zeros_like(t)
    for i in range(1, len(t)):
        x[i] = x[i - 1] * (1 - dt / tau) + math.sqrt(2 * dt / tau) * rng.standard_normal()
        d[i] = d[i - 1] * (1 - dt / tau_d) + math.sqrt(2 * dt / tau_d) * rng.standard_normal()
    g = x.copy()
    for tg in np.arange(rng.uniform(8, 15), T - 5, 20.0):   # assumed: a 5 s gust every 20 s
        m = (t >= tg) & (t < tg + 5)
        g[m] += 2.0 * (1 - np.cos(2 * np.pi * (t[m] - tg) / 5)) / 2
    g = g - g[t >= 5].mean()                                 # zero-mean: the mean wind stays U
    g = g * (Gk - U) / max(g.max(), 1e-9)                    # the peak is the gust speed
    speed = np.maximum(np.clip(t / 5.0, 0, 1) * (U + g), 0)
    frm = np.radians(c["wind_from_deg"] + 15.0 * d)          # assumed: 15 deg direction wander
    return t, speed * np.cos(frm), speed * np.sin(frm)


def sensor_errors(c, T, dt=0.5):
    """Injected sensor errors at t = 0, dt, ... s after the hold start: baro height error (m; smooth random wander,
    peak baro_wander_m) and compass heading error (rad; mag_yaw_deg from the start, plus mag_yaw_dps)."""
    t = np.arange(0, T + dt, dt)
    x = np.zeros_like(t)
    if c["baro_wander_m"] > 0:
        rng = np.random.default_rng(c["seed"] + 29)
        for i in range(1, len(t)):
            x[i] = x[i - 1] * (1 - dt / c["baro_tau_s"]) + math.sqrt(2 * dt / c["baro_tau_s"]) * rng.standard_normal()
        x = x * c["baro_wander_m"] / max(float(np.abs(x).max()), 1e-9)
    return t, x, np.radians(c["mag_yaw_deg"] + c["mag_yaw_dps"] * t)


PA_PER_M = 1.2 * G    # baro: about 11.8 Pa per m near the ground (air 1.2 kg/m^3); a higher reading = lower pressure


# ------------------------------------------------------------------ score
def score(link, t0, t1, hold_pt, cam):
    a = link.truth[:link.n_truth]
    a = a[(a[:, 0] >= t0) & (a[:, 0] <= t1)]
    if len(a) < 10:
        return {}
    t, N, E, h, q = a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4:]
    d = np.hypot(N - hold_pt[0], E - hold_pt[1])
    vn, ve = np.polyfit(t, N, 1)[0], np.polyfit(t, E, 1)[0]
    roll, pitch = C.q_roll_pitch(q)
    gN, gE = cam.ground_point(cam.centre_b, N, E, h, q)
    j = np.hypot(gN - hold_pt[0], gE - hold_pt[1])
    jj = np.hypot(gN - gN.mean(), gE - gE.mean())
    r = lambda x: float(np.sqrt(np.mean(np.square(x))))
    return dict(rms=r(d), p95=float(np.percentile(d, 95)), max=float(d.max()), end=float(d[-1]),
                drift_m_per_min=60 * math.hypot(vn, ve),
                roll_rms_deg=math.degrees(r(roll - roll.mean())), pitch_rms_deg=math.degrees(r(pitch - pitch.mean())),
                tilt_mean_deg=math.degrees(math.hypot(roll.mean(), pitch.mean())),
                judge_rms=r(j), judge_p95=float(np.percentile(j, 95)), judge_max=float(j.max()),
                judge_jitter_rms=r(jj), height_mean=float(h.mean()), seconds=float(t[-1] - t[0]))


# ------------------------------------------------------------------ one flight
def _md5(path):
    import hashlib
    return hashlib.md5(open(path, "rb").read()).hexdigest()[:10]


HOLD_MD5 = _md5(os.path.join(PROJ, "hold.py"))


def sim_settings(c, label):
    """settings.json for the onboard program in this case: what a perfect event-day calibration and
    mask step would write (camera keys from the true camera unless est_* says otherwise), the PX4 link
    on SITL's onboard port, and c["settings"] on top. Returns (path, dict)."""
    import cv2
    import capture
    import settings as S
    s = S.load("/nonexistent")                                  # the defaults of settings.py
    W, H, _ = C.VIDEO[c["video"]]
    g = lambda k: c[k] if c["est_" + k[4:]] is None else c["est_" + k[4:]]   # noqa: E731
    ro = c["readout_s"]
    # frame stamp (end of the last row + chain delay) minus mid-exposure of row 0, as Frames renders it
    lag = abs(ro) + c["exposure_s"] + c["video_delay_s"]
    t0_off = c["exposure_s"] / 2 + (abs(ro) if ro < 0 else 0.0)
    mask = C.osd_mask(W, H) if (c["osd"] and c["osd_mask"]) else np.full((H, W), 255, np.uint8)
    if c["mask_corners"] or c["mask_border"]:
        import hold
        mask = np.minimum(mask, hold.corner_mask((H, W), c["mask_corners"], c["mask_border"]))
    mask_path = os.path.join(RUNS, label + "_mask.png")
    cv2.imwrite(mask_path, mask)
    s.update(fc_device=f"udpin:127.0.0.1:{PORT}", top=g("cam_top"), mirror=bool(g("cam_mirror")),
             tilt_deg=[float(x) for x in g("cam_tilt")], twist_deg=float(g("cam_twist")), lens_name=c["model_lens"],
             readout_s=ro, video_delay_s=lag - t0_off + c["est_delay_err_s"], hold_cfg=c["hold_cfg"],
             mask_file=mask_path, log_dir=RUNS, calibrated=True)
    s["calib_capture"] = capture.capture_mode(s, {}, (H, W))
    s.update(c["settings"])
    path = os.path.join(RUNS, label + "_settings.json")
    S.save(s, path)
    return path, s


def feed_frames(frames, link, tx, c, seen):
    """Simulator side of the capture: each rendered frame with its driver stamp. The onboard process
    runs on the sim clock (publish_clock), so the stamp is the sim time the frame became ready."""
    try:
        for img, _ in frames:
            if img is None:
                continue
            tx.send((img, frames.t_avail + c["stamp_err_s"]))
            seen.append((frames.t_avail, time.monotonic() - link.mono_of(frames.t_avail)))
    except (BrokenPipeError, EOFError, OSError):
        pass


def publish_clock(link, clk):
    """time.monotonic minus sim time, every 10 ms, for the onboard process's clock (onboard.py). PX4's
    lockstep clock runs at 0.95-0.98 x wall time and the rate changes from second to second; the FC and
    APB crystals agree within ppm. A wall-clock onboard process would see 10-20 ms clock-mapping errors
    that the drone does not have (measured: integ/clock_wobble.txt)."""
    while not link.stop_all:
        n = link.n_truth
        if n >= 12:
            clk.value = float(np.min(link.arr[n - 12:n] - link.truth[n - 12:n, 0]))
        time.sleep(0.01)


def onboard_log(paths, t0, t1):
    """Rows of the onboard fix log(s) between t0 and t1 (time.monotonic) as dicts."""
    import csv
    rows = []
    for p in paths:
        if os.path.exists(p):
            rows += [r for r in csv.DictReader(open(p)) if r["t"] and t0 <= float(r["t"]) <= t1]
    return rows


def run_case(case, quiet=False):
    """Start PX4, climb in Altitude mode, start the vision feed, switch to Position mode, hold, score."""
    import hold as hold_mod
    c = dict(DEFAULTS, **case)
    label = c["label"] or f"{c['variant']}_{c['height']:.0f}m_{c['wind']}_{c['vehicle']}"
    say = (lambda *a: None) if quiet else (lambda *a: print(f"[{label}]", *a, flush=True))
    os.makedirs(RUNS, exist_ok=True)
    apb = c["variant"] == "apb"
    if apb:       # the onboard program's own PX4 parameter list (settings params_px4), then case extras
        s_path, s_onb = sim_settings(c, label)
        params = {**BASE, **vehicle(VEHICLES[c["vehicle"]]), **s_onb["params_px4"], **c["px4"]}   # later wins (px4= overrides)
    elif c["variant"] == "gps":   # the GPS baseline: simulated GNSS on, no vision, a normal GPS position hold
        params = dict(BASE, **vehicle(VEHICLES[c["vehicle"]]), SYS_HAS_GPS=1, SIM_GPS_USED=10, SENS_EN_GPSSIM=1,
                      EKF2_GPS_CTRL=7, EKF2_EV_CTRL=0, **c["px4"])
    else:
        params = {**BASE, **vehicle(VEHICLES[c["vehicle"]]), "EKF2_EV_DELAY": round(1000 * ev_delay(c), 1), **c["px4"]}
        if ev_delay(c) > 0.18:
            params["EKF2_DELAY_MAX"] = round(1000 * ev_delay(c)) + 50
    res = dict(case=label, cfg={k: v for k, v in c.items() if k != "video_out"}, events=[])
    ev = lambda what: (res["events"].append((round(link.now() - min(link.t_engage, link.now()), 2), what)), say(what))
    docker_start(params, c["speed"])
    link = Link(c, GCS_PORT if apb else PORT)
    link.t_video = math.inf if c["still_start"] else -math.inf
    jv, child, tx, stop_ev, seen = None, None, None, None, []
    try:
        link.start()
        # Workaround, SITL only: PX4's param autosave re-checks the file after writing; a wind PARAM_SET in
        # between fails the check, and the retry's px4_usleep (parameters.cpp:882) waits on the lockstep
        # clock, which deadlocks the simulator. With no default file, autosave waits until disarmed.
        subprocess.run(["docker", "exec", NAME, "/opt/px4/bin/px4-param", "select"], capture_output=True, timeout=20)
        if apb:   # the onboard program starts with the FC, as on the drone; the camera runs from the ground up
            import onboard
            ctx = multiprocessing.get_context("spawn")
            rx, tx = ctx.Pipe(duplex=False)
            stop_ev, status_q = ctx.Event(), ctx.Queue()
            log_path = os.path.join(RUNS, label + "_onboard.csv")
            for old in [f for f in os.listdir(RUNS) if f.startswith(label + "_onboard")]:
                os.remove(os.path.join(RUNS, old))
            while link.n_truth < 20:
                time.sleep(0.05)
            clk = ctx.Value("d", 0.0, lock=False)
            threading.Thread(target=publish_clock, args=(link, clk), daemon=True).start()
            time.sleep(0.1)
            opts = dict(cpu_factor=c["cpu_factor"], reanchor_on_lost=c["reanchor_on_lost"])
            child = ctx.Process(target=onboard.main, args=(s_path, rx, stop_ev, log_path, opts, status_q, clk),
                                daemon=True)
            child.start()
            t_launch = time.monotonic()
            cam = C.Camera(c, C.make_ground(c["ground"]))
            frames = C.Frames(cam, link, c, render=True)
            threading.Thread(target=feed_frames, args=(frames, link, tx, c, seen), daemon=True).start()
        link.wait_until(link.now() + 8.0, timeout=60)          # EKF tilt and heading alignment on the ground
        if not link.ekf or not link.att.rows:
            raise RuntimeError("PX4 sends no ATTITUDE or LOCAL_POSITION_NED")
        link.z0 = float(np.mean([r[3] for r in link.ekf[-30:]]))
        link.set_mode(2)
        link.wait_for(0.5)
        link.arm()
        link.wait_for(1.0)
        t_arm = link.now()
        while not link.armed and link.now() - t_arm < 3.0:     # the armed flag comes with a 1 Hz heartbeat
            link.wait_for(0.1)
        if not link.armed:
            raise RuntimeError("PX4 did not arm: " + "; ".join(x[1] for x in link.texts[-5:]))
        target, t_climb = c["height"], link.now()
        while True:                                             # climb on the throttle stick, Altitude mode
            err = target - link.h_now()
            if abs(err) < min(0.5, 0.1 * target) or link.now() - t_climb > 120:
                break
            # stay outside PX4's stick deadzone (10 % = 50 units) or the climb stalls short of the target; on the
            # ground ask for more, or PX4 does not take off (a 2 m target with 620 units stayed on the ground)
            boost = 250 if link.h_now() < 1.0 else 0
            link.throttle = 500 + math.copysign(float(np.clip(abs(err) * 60 + boost, 120, 480)), err)
            time.sleep(0.01)
        link.throttle = 500
        link.wait_for(4.0)                                      # settle
        res["climb_s"] = round(link.now() - t_climb, 1)

        if not apb:
            cam = C.Camera(c, C.make_ground(c["ground"]))
        log, est = [], None
        a = np.array(link.att_rows[-200:])
        link.yaw_off = float(np.angle(np.mean(np.exp(1j * (a[:, 3] - C.q_yaw(link.truth_at(a[:, 0])[3]))))))
        res["ekf_yaw_minus_true_deg"] = round(math.degrees(link.yaw_off), 2)
        if apb:   # PX4 uses our stamps only after its TIMESYNC filter converged; on the drone the APB
            # runs from power-on, so this has happened before take-off
            while time.monotonic() - t_launch < c["sync_wait_s"] and child.is_alive():
                time.sleep(0.1)
        if c["variant"] not in ("none", "apb", "gps"):
            frames = C.Frames(cam, link, c, render=(c["variant"] == "ours" or bool(c["video_out"])))
            if c["variant"] == "ours":
                K, D = hold_mod.load_lens(C.LENS_JSON, c["model_lens"], cam.W, cam.H)
                g = lambda k: c[k] if c["est_" + k[4:]] is None else c["est_" + k[4:]]
                R_bc = hold_mod.make_R_bc(top=g("cam_top"), mirror=g("cam_mirror"), tilt_deg=g("cam_tilt"),
                                          twist_deg=g("cam_twist"))
                mask = frames.mask if (c["osd"] and c["osd_mask"]) else None
                est = hold_mod.Hold(K, D, R_bc, c["readout_s"], mask=mask, **c["hold_cfg"])
                est.reanchor(np.zeros(2))
            else:
                est = TruthHold(link, link.yaw_off, c["readout_s"])
            th = threading.Thread(target=estimator_loop, args=(frames, link, est, log), daemon=True)
            th.start()

        # supervise: Position mode as soon as PX4 accepts it, then wind, fly-away guard, end of hold
        t_start, t_try, wt, ww, ws = link.now(), -1.0, None, 0, 0
        if c["still_start"]:
            link.t_video = t_start
        last_now, last_wall = -1.0, time.time()
        while True:
            time.sleep(0.005)
            now = link.now()
            if now != last_now:
                last_now, last_wall = now, time.time()
            elif time.time() - last_wall > 10:
                ev(f"simulator clock stopped at {now:.2f} s (no truth for 10 s of wall time)")
                log_path = os.path.join(RUNS, label + "_px4log.txt")
                with open(log_path, "w") as f:
                    subprocess.run(["docker", "logs", "--tail", "80", NAME], stdout=f, stderr=subprocess.STDOUT)
                break
            if link.t_engage == math.inf:
                if c["variant"] == "none" or link.mode == 3:
                    link.t_engage = now
                    hold_pt = link.truth_at(np.array([now]))
                    hold_pt = np.array([hold_pt[0][0], hold_pt[1][0]])
                    ev(f"hold starts, mode {link.mode}, {now - t_start:.1f} s after the vision feed started")
                    wt, wn, we = wind_series(c, c["hold_s"] + 5)
                    st_t, st_baro, st_yaw = sensor_errors(c, c["hold_s"] + 5)
                    if link.mag is not None:   # the field in NED from the body field now (no offset yet), true yaw
                        yaw0 = float(C.q_yaw(link.truth_at(np.array([now]))[3])[0])
                        mag_ned = np.array([[math.cos(yaw0), -math.sin(yaw0)], [math.sin(yaw0), math.cos(yaw0)]]) @ np.array(link.mag[:2])
                    if c["video_out"] and c["variant"] != "none":
                        jv = C.JudgeVideo(c["video_out"], cam, link, hold_pt, label)
                        frames.judge = jv
                elif now - t_start > c["engage_wait_s"]:
                    ev("PX4 never accepted Position mode")
                    break
                elif now - t_try > 0.5:
                    link.set_mode(3)
                    t_try = now
                continue
            tt = now - link.t_engage
            if wt is not None and ww < len(wt) and tt >= wt[ww] and WINDS[c["wind"]][1] > 0:
                link.param("SIH_WIND_N", wn[ww])
                link.param("SIH_WIND_E", we[ww])
                ww += 1
            if ws < len(st_t) and tt >= st_t[ws] and (c["baro_wander_m"] > 0 or c["mag_yaw_deg"] or c["mag_yaw_dps"]):
                if c["baro_wander_m"] > 0:
                    link.param("SIM_BARO_OFF_P", -PA_PER_M * st_baro[ws])
                if (c["mag_yaw_deg"] or c["mag_yaw_dps"]) and link.mag is not None:
                    # body field now (true yaw), turned by -d in the body: the compass then reads the heading + d
                    yw = float(C.q_yaw(link.truth_at(np.array([now]))[3])[0])
                    b = np.array([[math.cos(yw), math.sin(yw)], [-math.sin(yw), math.cos(yw)]]) @ mag_ned
                    d = st_yaw[ws]
                    off = np.array([[math.cos(d), math.sin(d)], [-math.sin(d), math.cos(d)]]) @ b - b
                    link.param("SIM_MAG_OFFSET_X", off[0])
                    link.param("SIM_MAG_OFFSET_Y", off[1])
                ws += 1
            N, E, h, _ = link.truth_at(np.array([now]))
            dist = math.hypot(N[0] - hold_pt[0], E[0] - hold_pt[1])
            if c["variant"] != "none" and link.mode != 3 and not any("left Position" in x[1] for x in res["events"]):
                ev(f"PX4 left Position mode (mode {link.mode})")
            if dist > c["flyaway_m"] and c["variant"] != "none":
                ev(f"FLY-AWAY: {dist:.1f} m from the hold point; vision stopped, Altitude mode")
                link.done = True
                link.set_mode(2)
                break
            if h[0] < min(3.0, 0.4 * c["height"]):
                ev("height below min(3 m, 0.4 x the hold height): crashed or landed")
                break
            if tt >= c["hold_s"]:
                break
        t_end = link.now()
        link.done = True
        time.sleep(0.2)
        if jv:
            jv.close()
        if apb:
            res.update(apb_results(c, link, child, stop_ev, tx, status_q, log_path, seen, t_end))
        if link.t_engage < math.inf:
            res.update(score(link, link.t_engage, t_end, hold_pt, cam))
            # what the autopilot (and the onboard program, which reads the same messages) believed: PX4's height minus
            # the true height, and PX4's heading minus the true heading minus the offset at the hold start
            e = np.array([r for r in link.ekf if link.t_engage <= r[0] <= t_end])
            a = np.array([r for r in link.att_rows if link.t_engage <= r[0] <= t_end])
            if len(e) and len(a):
                dh = (link.z0 - e[:, 3]) - link.truth_at(e[:, 0])[2]
                dy = np.degrees(np.angle(np.exp(1j * (a[:, 3] - C.q_yaw(link.truth_at(a[:, 0])[3]) - link.yaw_off))))
                res.update(px4_hgt_err_m=[round(float(dh.min()), 2), round(float(dh.max()), 2)],
                           px4_yaw_err_deg=[round(float(dy.min()), 2), round(float(dy.max()), 2), round(float(dy[-1]), 2)])
        if apb:
            log = [(float(r["t"]), *(float(r[k]) if r[k] else math.nan for k in ("north", "east", "var")),
                    STATES.index(r["state"]) if r["state"] else -1, float(r["n"] or 0), float(r["proc_ms"]) / 1e3,
                    float(r["sent"])) for r in onboard_log(res.pop("_logs"), -math.inf, math.inf)]
        L = np.array(log, float).reshape(-1, 8)
        hold_log = L[L[:, 0] >= link.t_engage] if len(L) else L
        if len(hold_log) and not apb:
            st = hold_log[:, 4]
            res.update(frames=len(hold_log), fix_rate_hz=float(np.sum(~np.isnan(hold_log[:, 3])) / max(t_end - link.t_engage, 1e-9)),
                       **{f"frac_{s}": float(np.mean(st == i)) for i, s in enumerate(STATES)},
                       proc_ms_mean=1000 * float(np.mean(hold_log[:, 6])),
                       late_ms_p95=1000 * float(np.nanpercentile(hold_log[:, 7], 95)) if np.any(~np.isnan(hold_log[:, 7])) else None)
        res["ev_delay_ms"] = round(1000 * ev_delay(c), 1)
        if not apb:
            res["hold_why"] = dict(sorted(link.whys.items(), key=lambda x: -x[1])[:8])
        res["hold_py_md5"] = HOLD_MD5
        res["px4_text"] = [(round(t - min(link.t_engage, t), 2), x) for t, x in link.texts
                           if any(w in x.lower() for w in ("ev ", "vision", "fail", "reset", "yaw", "position", "land"))][-30:]
        json.dump({k: v for k, v in res.items() if k != "px4_text"}, open(os.path.join(RUNS, label + ".json"), "w"), default=str)
        np.savez_compressed(os.path.join(RUNS, label + ".npz"), truth=link.truth[:link.n_truth], arr=link.arr[:link.n_truth],
                            ekf=np.array(link.ekf), att=np.array(link.att_rows), est=L,
                            wind=np.c_[wt, wn, we] if wt is not None else np.zeros((0, 3)),
                            t_engage=link.t_engage, hold_pt=hold_pt if link.t_engage < math.inf else np.zeros(2))
        if not quiet:
            keys = ("rms", "p95", "max", "drift_m_per_min", "roll_rms_deg", "pitch_rms_deg", "judge_rms",
                    "judge_jitter_rms", "fix_rate_hz", "frac_ok", "frac_lost", "proc_ms_mean", "late_ms_p95")
            say("  ".join(f"{k} {res[k]:.3g}" for k in keys if isinstance(res.get(k), float)))
        return res
    finally:
        if child is not None:
            stop_ev.set()
            child.join(timeout=5)
            if child.is_alive():
                child.terminate()
        link.close()
        docker_stop()


if __name__ == "__main__":
    a = sys.argv[1:]
    kv = dict(x.split("=", 1) for x in a if "=" in x)
    a = [x for x in a if "=" not in x]
    case = dict(variant=a[0] if a else "truth", height=float(a[1]) if len(a) > 1 else 50.0,
                wind=a[2] if len(a) > 2 else "calm", hold_s=float(a[3]) if len(a) > 3 else 20.0)
    for k, v in kv.items():
        try:
            case[k] = json.loads(v)
        except ValueError:
            case[k] = v
    os.makedirs(OUT, exist_ok=True)
    r = run_case(case)
    print(json.dumps({k: v for k, v in r.items() if k != "cfg"}, indent=1, default=str))
