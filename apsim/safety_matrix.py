#!/usr/bin/env python3
"""Pilot takeover and failure cases on ArduCopter SITL, for PILOT_SAFETY.txt and integ/pilot_safety/README.txt.
  ~/.venvs/vio/bin/python apsim/safety_matrix.py run [<label> ...]   one run at a time (default: the 10 core runs)
  ~/.venvs/vio/bin/python apsim/safety_matrix.py table
  ~/.venvs/vio/bin/python apsim/safety_matrix.py case <label> ...      print the case; starts no simulator
Labels: saf_<a|b|c|d|e>_<height>m_s<seed>. Core runs: cases a-e at 10 m and 30 m, seed 1 (10 runs). Spare runs
(only if the core runs worked): saf_b_10m_s2, saf_c_10m_s2. At most 12 runs in total.
  a  normal hold. 10 s after Loiter holds, the pilot selects ALT_HOLD.
  b  the program stops 10 s into the hold: every MAVLink message from fc_link stops (fixes, HEARTBEAT, TIMESYNC),
     as when the board loses power. The pilot selects ALT_HOLD 1 s after the FC changes mode (assumed reaction
     time), or 25 s after the stop if the FC does not change mode.
  c  wrong fixes: 10 s into the hold the sent position starts to move north at 1 m/s (ramp). The pilot selects
     ALT_HOLD 3 s after the true distance from the hold point passes 3 m (at the latest 30 s after the start).
  d  a fix jump: 10 s into the hold every sent position moves 20 m east. Takeover rule as c (latest 25 s).
  f  (spare, added in phase 2) as d, but a re-anchor of the program keeps the jump: local_pos is not corrected, so
     the 20 m offset stays in every sent fix (in d the program re-anchored 0.6 s after the jump and removed it).
  e  as c, but the pilot selects STABILIZE, 10 s later ACRO, 10 s later ALT_HOLD. In STABILIZE and ACRO a pilot
     model holds the height on the throttle stick from the TRUE height (a harness shortcut: a real pilot sees
     the aircraft, not the truth). Roll, pitch and yaw sticks stay centred.
Each case is apsim/sitl_ap.run_case (variant apb, video JUSTICE_D, ground ottawa2019fall = tuning site, wind
w20g40, tuning seeds 1-9, DATA_SPLITS.md), with the event profile outdoor_day (settings min_height_m 3, hold_cfg
h_min 2, as apsim/low_ap_matrix.py), SITL instance 1 (ports 15020-15022). FAILSAFE PARAMETERS: the SITL
defaults (Tools/autotest/default_params/copter.parm + ArduCopter defaults: FS_EKF_ACTION 1 Land, FS_EKF_THRESH
0.8, FS_THR_ENABLE 1), so the runs show what the FC does with the defaults. ARMING_CHECK 0 (sitl_ap.parm_file).
Results: apsim/safety_results.json and apsim/safety_results.txt. Run files: apsim/runs/<label>.*.
Hooks (sitl_ap.py is NOT edited): sitl_ap.APLink is replaced by a subclass that runs the pilot actions, and the
onboard process is apsim/onboard_cpu.main with fc_link.FC._send / send_fix / local_pos wrapped. The supervisor
starts a failure by writing apsim/runs/<label>.inject (JSON: kind, sim time); the onboard wrapper reads it.
Limits: the onboard run log still shows sent = 1 after a case-b stop (run_live does not know); the ramp and the
jump are added to the sent fix only (local_pos is corrected back, so a re-anchor does not add them twice).
Gate: a run starts only when the 1-minute load is under 3.0, no other ArduCopter SITL or px4_sim runs, and
instance 1 is free. Each run is killed after RUN_TIMEOUT_S (300 s) and SITL instance 1 is stopped.
"""
import fcntl
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
PY = os.path.expanduser("~/.venvs/vio/bin/python")
OUT, TXT = os.path.join(HERE, "safety_results.json"), os.path.join(HERE, "safety_results.txt")
RUNS = os.path.join(HERE, "runs")
INSTANCE = 1
LOAD_MAX = 3.0            # 1-minute load before a start (the user's rule for phase 2)
WAIT_MAX_S = 1800         # give up on a run after 30 min of waiting for the gate
RUN_TIMEOUT_S = 300       # each run must stop within 5 minutes
INJECT_S = 10.0           # the failure or the takeover starts this long after Loiter holds
REACT_S = 1.0             # assumed pilot reaction after the FC changes mode (case b)
PASS_M, AFTER_PASS_S = 3.0, 3.0   # cases c, d, e: takeover AFTER_PASS_S after the true distance passes PASS_M
WINDOW_S = 10.0           # measurement window after the takeover
RAMP_MPS, JUMP_M = 1.0, 20.0
MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND",
         16: "POSHOLD", 17: "BRAKE", 21: "SMART_RTL"}
NUM = {v: k for k, v in MODES.items()}
HOLD_S = dict(a=24.0, b=50.0, c=40.0, d=45.0, e=50.0, f=45.0)   # run_case hold length: covers the latest takeover + window
CORE = [f"saf_{k}_{h}m_s1" for h in (10, 30) for k in "abcde"]
SPARE = ["saf_b_10m_s2", "saf_c_10m_s2"]


# Task 1: arming with the firmware's default arming checks. ARMING_CHECK (ArduCopter 4.6: libraries/AP_Arming/
# AP_Arming.cpp:163, default 1 = ALL; bits AP_Arming.h:25-45; check_enabled AP_Arming.cpp:284-290). ARM_FIX = every
# check bit 1-20 except bit 18 (VisualOdometry) = 1835006: all checks on except the vision check.
ARM_FIX = sum(1 << b for b in range(1, 21) if b != 18)
ARM_LABELS = ["arm_none_s1", "arm_gnss_s1", "arm_none_s2"]   # the third: spare for a harness failure


def arm_case(label):
    """arm_<none|gnss>_s<seed>: default level or gnss_setup, ARMING_CHECK 1 (firmware default), 10 m, 30 s hold."""
    m = re.match(r"arm_(none|gnss)_s(\d+)$", label)
    if not m:
        raise ValueError(f"unknown label {label}")
    st = dict(min_height_m=3.0)
    if m.group(1) == "gnss":
        sys.path.insert(0, PROJ)
        import settings as S
        prof = json.load(open(os.path.join(PROJ, "tools", "profiles", "gnss_setup.json")))
        st.update(gnss_mode="setup", params_ardupilot=dict(S.DEFAULTS["params_ardupilot"], **prof["params_ardupilot"]))
    return dict(variant="apb", height=10.0, wind="w20g40", hold_s=30.0, seed=int(m.group(2)), label=label,
                instance=INSTANCE, video="JUSTICE_D", ground="ottawa2019fall", flyaway_m=30.0,
                settings=st, hold_cfg=dict(h_min=2.0), ap=dict(ARMING_CHECK=1)), "arm"


def case(label):
    if label.startswith("arm_"):
        return arm_case(label)
    m = re.match(r"saf_([a-f])_(\d+)m_s(\d+)$", label)
    if not m:
        raise ValueError(f"unknown label {label}")
    k, h, seed = m.group(1), float(m.group(2)), int(m.group(3))
    if not 1 <= seed <= 9:
        raise ValueError("tuning seeds are 1-9 (DATA_SPLITS.md)")
    return dict(variant="apb", height=h, wind="w20g40", hold_s=HOLD_S[k], seed=seed, label=label,
                instance=INSTANCE, video="JUSTICE_D", ground="ottawa2019fall", flyaway_m=60.0,
                settings=dict(min_height_m=3.0), hold_cfg=dict(h_min=2.0)), k   # tools/profiles/outdoor_day.json


# ------------------------------------------------------------------ onboard process (spawned by sitl_ap.run_case)
def onboard_main(settings_path, conn, stop, log_path, opts, status_q, clk):
    """apsim/onboard_cpu.main with fc_link.FC wrapped: after <label>.inject appears, 'stop' drops every message,
    'ramp' and 'jump' move every sent fix. time.monotonic is the sim clock here (px4sim/onboard.main patches it)."""
    sys.path[:0] = [PROJ, os.path.join(PROJ, "px4sim"), HERE]
    import fc_link
    wrap_fc(fc_link.FC, log_path[:-len("_onboard.csv")] + ".inject")
    import onboard_cpu
    onboard_cpu.main(settings_path, conn, stop, log_path, opts, status_q, clk)


def wrap_fc(FC, inj_path):
    """Wrap FC._send, send_fix and local_pos of the class FC for the failure in inj_path (see onboard_main)."""
    st = {"next": -math.inf, "inj": None}

    def active():
        now = time.monotonic()
        if st["inj"] is None and now >= st["next"]:
            st["next"] = now + 0.05
            if os.path.exists(inj_path):
                try:
                    st["inj"] = json.load(open(inj_path))
                except (OSError, ValueError):
                    pass
        x = st["inj"]
        return (x, now) if x is not None and now >= x["t"] else (None, now)

    def offset():
        x, now = active()
        if x is None:
            return np.zeros(2)
        if x["kind"] == "ramp":
            return np.array(x["dir"], float) * x["v"] * (now - x["t"])
        if x["kind"] == "jump":
            return np.array(x["step"], float)
        return np.zeros(2)

    o_send, o_fix, o_lpos = FC._send, FC.send_fix, FC.local_pos

    def _send(self, fn):
        x, _ = active()
        if x is not None and x["kind"] == "stop":
            return None
        return o_send(self, fn)

    def send_fix(self, e, att=None, hgt=None):
        d = offset()
        if d.any():
            e = e._replace(pos=np.asarray(e.pos, float) + d)
        return o_fix(self, e, att, hgt)

    def local_pos(self, max_age=0.5):
        p = o_lpos(self, max_age)
        x, _ = active()
        if p is None or (x is not None and x.get("raw")):    # case f: a re-anchor keeps the jump
            return p
        return p - offset()

    FC._send, FC.send_fix, FC.local_pos = _send, send_fix, local_pos


# ------------------------------------------------------------------ supervisor side
def make_link(A, kind, label, marks, links):
    class Link(A.APLink):
        """sitl_ap.APLink plus the pilot of this case and a record of FC mode changes."""

        def __init__(self, c, port):
            super().__init__(c, port)
            self.modes, self.pilot_h, self.acks = [], None, []
            links.append(self)
            self.m.message_hooks.append(self._ack)
            for f in (self._modes, self._scenario, self._throttle_pilot):
                threading.Thread(target=f, daemon=True).start()

        def _ack(self, _conn, msg):
            if msg.get_type() == "COMMAND_ACK" and self.n_truth:
                self.acks.append((self.now(), int(msg.command), int(msg.result)))

        def arm(self):
            """Task 1 (kind "arm"): try to arm in ALT_HOLD, then in STABILIZE, with the default checks. If neither
            arms, set ARMING_CHECK = ARM_FIX (vision check off) and arm in ALT_HOLD. run_case then climbs and holds."""
            if kind != "arm":
                return super().arm()
            log = marks.setdefault("arm_log", [])

            def attempt(mode, what):
                self._select(mode)
                self._until(self.now() + 1.0)
                t, n = self.now(), len(self.acks)
                self.cmd(400, 1)
                self._until(self.now() + 3.0)
                log.append(dict(what=what, mode=mode, t=round(t, 2), armed=bool(self.armed),
                                acks=[(round(a - t, 2), r) for a, c_, r in self.acks[n:] if c_ == 400],
                                texts=[(round(a - t, 2), x) for a, x in self.texts if a >= t - 0.2]))
                return self.armed

            def disarm():
                self.cmd(400, 0)
                self._until(self.now() + 1.5)
            # Harness corrections (run arm_none_s1 showed both): run_case holds the throttle at PWM 500 before arming
            # (link.throttle -500), under FS_THR_VALUE 975, so the RC check reports "Radio failsafe on". A real
            # transmitter gives about PWM 1000 at low stick: use throttle 0 = PWM 1000. The INS check needs the IMUs
            # consistent for 10 s (libraries/AP_Arming/AP_Arming.cpp:419-460): wait until 35 s after the FC start.
            self.throttle = 0
            self._until(max(self.now() + 2.0, 35.0))
            marks["harness"] = "throttle PWM 1000 before arming; first arm attempt at %.1f s" % self.now()
            ok_alt = attempt("ALT_HOLD", "default checks")
            if ok_alt:
                disarm()
            ok_stab = attempt("STABILIZE", "default checks")
            if ok_stab:
                disarm()
            if not (ok_alt or ok_stab):
                self.param("ARMING_CHECK", ARM_FIX)
                marks["arm_fix"] = dict(ARMING_CHECK=ARM_FIX, t=round(self.now(), 2))
                self._until(self.now() + 2.0)
                for _ in range(4):
                    if attempt("ALT_HOLD", f"ARMING_CHECK {ARM_FIX} (vision check off)"):
                        break
                    self._until(self.now() + 2.0)
            else:
                attempt("ALT_HOLD", "default checks, again for the flight")
            if self.armed:
                self.set_mode(NUM["ALT_HOLD"])

        def _modes(self):
            last = None
            while not self.stop_all:
                m = getattr(self, "mode", None)
                if m != last and self.n_truth:
                    last = m
                    self.modes.append((self.now(), MODES.get(m, f"mode {m}")))
                time.sleep(0.01)

        def _until(self, t):
            while self.now() < t and not self.stop_all and not self.done:
                time.sleep(0.005)

        def _dist(self):
            n, e, _, _ = self.truth_at(np.array([self.now()]))
            return math.hypot(n[0] - marks["hold_pt"][0], e[0] - marks["hold_pt"][1])

        def _select(self, name):
            """The pilot moves the switch: MAV_CMD_DO_SET_MODE, repeated up to 3 times until the FC reports it."""
            t = self.now()
            marks.setdefault("pilot", []).append((t, name))
            for _ in range(3):
                self.set_mode(NUM[name])
                t_try = self.now()
                while self.now() - t_try < 0.5 and not self.stop_all:
                    if getattr(self, "mode", None) == NUM[name]:
                        return True
                    time.sleep(0.01)
            marks.setdefault("refused", []).append((t, name))
            return False

        def _inject(self, x):
            x = dict(x, t=self.now())
            p = os.path.join(RUNS, label + ".inject")
            open(p + ".tmp", "w").write(json.dumps(x))
            os.replace(p + ".tmp", p)
            marks["t_inject"], marks["inject"] = x["t"], x

        def _throttle_pilot(self):
            """STABILIZE / ACRO: throttle stick from the true height and climb rate (60 per m as run_case's climb)."""
            hist = []
            while not self.stop_all:
                if self.pilot_h is not None and self.n_truth > 10:
                    t, h = self.now(), self.h_now()
                    hist = [x for x in hist if x[0] > t - 0.3] + [(t, h)]
                    vz = (hist[-1][1] - hist[0][1]) / max(hist[-1][0] - hist[0][0], 1e-3) if len(hist) > 2 else 0.0
                    self.throttle = 500 + float(np.clip(60.0 * (self.pilot_h - h) - 150.0 * vz, -300, 300))
                time.sleep(0.02)

        def _scenario(self):
            try:
                while self.t_engage == math.inf and not self.stop_all:
                    time.sleep(0.02)
                if self.stop_all:
                    return
                te = self.t_engage
                n, e, _, _ = self.truth_at(np.array([te]))
                marks.update(t_engage=te, hold_pt=(float(n[0]), float(e[0])))
                if kind == "arm":
                    return
                self._until(te + INJECT_S)
                if self.done or self.stop_all:
                    return
                if kind == "a":
                    marks["t_inject"] = self.now()
                    marks["takeover"] = self.now()
                    self._select("ALT_HOLD")
                    return
                if kind == "b":
                    self._inject(dict(kind="stop"))
                    t0 = marks["t_inject"]
                    while not self.done and self.now() < t0 + 25.0 and getattr(self, "mode", None) == NUM["LOITER"]:
                        time.sleep(0.005)
                    if getattr(self, "mode", None) != NUM["LOITER"]:
                        marks["fc_change"] = (self.now(), MODES.get(self.mode, f"mode {self.mode}"))
                        self._until(self.now() + REACT_S)
                    marks["takeover"] = self.now()
                    self._select("ALT_HOLD")
                    return
                if kind in ("c", "e"):
                    self._inject(dict(kind="ramp", v=RAMP_MPS, dir=[1.0, 0.0]))
                    latest = 30.0
                else:   # d: the program's re-anchor (local_pos corrected) removes the jump; f: it keeps it
                    self._inject(dict(kind="jump", step=[0.0, JUMP_M], raw=kind == "f"))
                    latest = 25.0
                t0 = marks["t_inject"]
                while not self.done and self.now() < t0 + latest and self._dist() <= PASS_M:
                    time.sleep(0.005)
                if self._dist() > PASS_M:
                    marks["pass_3m"] = self.now()
                    self._until(self.now() + AFTER_PASS_S)
                if self.done:
                    return
                marks["takeover"] = self.now()
                if kind == "e":
                    self.pilot_h = self.h_now()
                    self._select("STABILIZE")
                    self._until(self.now() + 10.0)
                    self._select("ACRO")
                    self._until(self.now() + 10.0)
                    self.pilot_h = None
                    self.throttle = 500
                    self._select("ALT_HOLD")
                else:
                    self._select("ALT_HOLD")
            except Exception as ex:     # the run goes on; the error is in the result
                marks["scenario_error"] = f"{type(ex).__name__}: {ex}"
    return Link


# ------------------------------------------------------------------ metrics
def wrap(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


def metrics(label, marks, modes, texts, yaw_off_deg):
    sys.path.insert(0, os.path.join(PROJ, "px4sim"))
    import camera as C
    d = np.load(os.path.join(RUNS, label + ".npz"))
    tr, att, ekf = d["truth"], d["att"], d["ekf"]
    hp = np.array(marks["hold_pt"])
    t_end = float(tr[-1, 0])
    ti, tk = marks.get("t_inject"), marks.get("takeover")
    dist = np.hypot(tr[:, 1] - hp[0], tr[:, 2] - hp[1])
    pos = lambda x: np.array([np.interp(x, tr[:, 0], tr[:, 1]), np.interp(x, tr[:, 0], tr[:, 2])])   # noqa: E731
    at = lambda x: float(np.interp(x, tr[:, 0], dist))                                               # noqa: E731
    inw = lambda t, a, b: (t >= a) & (t <= b)                                                         # noqa: E731
    m = dict(t_end_after_inject=round(t_end - ti, 2) if ti else None)
    # attitude error: the FC's ATTITUDE (EKF) against SIM_STATE, at the ATTITUDE times
    j = np.clip(np.searchsorted(tr[:, 0], att[:, 0]), 0, len(tr) - 1)
    q = tr[j, 4:8]
    r_t, p_t = C.q_roll_pitch(q)
    y_t = C.q_yaw(q)
    er, ep = np.degrees(wrap(att[:, 1] - r_t)), np.degrees(wrap(att[:, 2] - p_t))
    dy = wrap(att[:, 3] - y_t)
    if ti is not None:
        base = inw(att[:, 0], ti - 5, ti)
        y0 = float(np.angle(np.mean(np.exp(1j * dy[base])))) if base.any() else 0.0
        ey = np.degrees(wrap(dy - y0))
        tilt = np.hypot(er, ep)

        def amax(x, a, b):
            w = inw(att[:, 0], a, b)
            return round(float(np.abs(x[w]).max()), 2) if w.any() else None
        m.update(tilt_err_deg_before=amax(tilt, ti - 5, ti), yaw_err_deg_before=amax(ey, ti - 5, ti))
        if tk is not None:
            m.update(tilt_err_deg_inject_to_takeover=amax(tilt, ti, tk), yaw_err_deg_inject_to_takeover=amax(ey, ti, tk),
                     tilt_err_deg_after=amax(tilt, tk, tk + WINDOW_S), yaw_err_deg_after=amax(ey, tk, tk + WINDOW_S),
                     roll_err_deg_after=amax(er, tk, tk + WINDOW_S), pitch_err_deg_after=amax(ep, tk, tk + WINDOW_S))
    if ti is not None:
        m.update(dist_at_inject=round(at(ti), 2), dist_max_after_inject=round(float(dist[tr[:, 0] >= ti].max()), 2))
    if tk is not None:
        g = np.arange(tk - 0.2, min(tk + WINDOW_S, t_end) + 1e-9, 0.05)
        p = pos(g)
        v = np.hypot(*(p[:, 5:] - p[:, :-5])) / 0.25 if p.shape[1] > 6 else np.zeros(1)
        w = inw(tr[:, 0], tk, tk + WINDOW_S)
        h0 = float(np.interp(tk, tr[:, 0], tr[:, 3]))
        m.update(dist_at_takeover=round(at(tk), 2),
                 dist_max_window=round(float(dist[w].max()), 2) if w.any() else None,
                 speed_max_window_mps=round(float(v.max()), 2),
                 speed_at_takeover_mps=round(float(np.hypot(*(pos(tk + 0.125) - pos(tk - 0.125))) / 0.25), 2),
                 height_at_takeover=round(h0, 2),
                 height_change_max_window=round(float(np.abs(tr[w, 3] - h0).max()), 2) if w.any() else None,
                 height_change_end_window=round(float(np.interp(min(tk + WINDOW_S, t_end), tr[:, 0], tr[:, 3]) - h0), 2),
                 window_s=round(min(tk + WINDOW_S, t_end) - tk, 2))
    if "fc_change" in marks and ti is not None:
        tc, mode = marks["fc_change"]
        m.update(fc_change_after_s=round(tc - ti, 2), fc_change_mode=mode, dist_at_fc_change=round(at(tc), 2),
                 flown_until_fc_change=round(float(np.linalg.norm(pos(tc) - pos(ti))), 2))
    if "pass_3m" in marks and ti is not None:
        m["pass_3m_after_s"] = round(marks["pass_3m"] - ti, 2)
    # how far the EKF position moved from the truth since the injection (the EKF frame turned by the yaw offset)
    if ti is not None and len(ekf) > 10:
        c_, s_ = math.cos(math.radians(yaw_off_deg)), math.sin(math.radians(yaw_off_deg))
        te_ = ekf[:, 0]
        N, E = np.interp(te_, tr[:, 0], tr[:, 1]), np.interp(te_, tr[:, 0], tr[:, 2])
        tru = np.c_[c_ * N - s_ * E, s_ * N + c_ * E]
        err = ekf[:, 1:3] - tru
        k0 = inw(te_, ti - 1, ti)
        if k0.any():
            err = err - err[k0].mean(0)
            n = np.linalg.norm(err, axis=1)
            b = min(tk, t_end) if tk is not None else t_end
            w = inw(te_, ti, b)
            m["ekf_err_change_max_to_takeover"] = round(float(n[w].max()), 2) if w.any() else None
            w2 = inw(te_, ti, t_end)
            m["ekf_err_change_max_all"] = round(float(n[w2].max()), 2) if w2.any() else None
    rel = (lambda x: round(x - ti, 2)) if ti is not None else (lambda x: round(x - marks.get("t_engage", 0), 2))
    m["fc_modes"] = [(rel(a), b) for a, b in modes if ti is None or a >= ti - 1]
    m["fc_texts"] = [(rel(a), x) for a, x in texts if ti is None or a >= ti - 1][:40]
    m["pilot"] = [(rel(a), b) for a, b in marks.get("pilot", [])]
    m["refused"] = [(rel(a), b) for a, b in marks.get("refused", [])]
    return m


# ------------------------------------------------------------------ one run (a child process of run)
def one(label):
    sys.path.insert(0, HERE)
    import sitl_ap as A
    c, kind = case(label)
    marks, links = {}, []
    A.APLink = make_link(A, kind, label, marks, links)
    sys.modules["onboard_cpu"] = types.SimpleNamespace(main=onboard_main)   # run_case: import onboard_cpu as onboard
    os.makedirs(RUNS, exist_ok=True)
    for p in (os.path.join(RUNS, label + ".inject"), os.path.join(RUNS, label + ".inject.tmp")):
        if os.path.exists(p):
            os.remove(p)
    t_wall, load0 = time.time(), os.getloadavg()
    try:
        res = A.run_case(c)
    except Exception as e:      # a broken run is logged, not lost
        res = dict(case=label, events=[(0, f"ERROR {type(e).__name__}: {e}")])
    r = {k: res.get(k) for k in ("case", "rms", "max", "events", "hold_s_flown", "climb_s", "pct_frames_fix",
                                   "fix_rate_hz", "ekf_yaw_minus_true_deg", "onboard_exit", "reanchors", "ap_params")}
    r.update(case_kind=kind, height=c["height"], seed=c["seed"], wall_s=round(time.time() - t_wall), load_start=load0,
             load_end=os.getloadavg(), marks={k: v for k, v in marks.items() if k not in ("pilot", "refused")})
    if links:
        k = links[-1]
        try:
            if kind == "arm":
                r.update(fc_texts_all=[(round(t, 2), x) for t, x in k.texts], fc_modes=[(round(t, 2), m) for t, m in k.modes],
                         gps_in_use_s=res.get("gps_in_use_s"), acks_all=[(round(t, 2), c_, x) for t, c_, x in k.acks])
            elif "hold_pt" in marks:
                r.update(metrics(label, marks, k.modes, k.texts, res.get("ekf_yaw_minus_true_deg") or 0.0))
            else:
                r["fc_texts_all"] = [(round(t, 2), x) for t, x in k.texts][-30:]
        except Exception as e:
            r["metrics_error"] = f"{type(e).__name__}: {e}"
    save(label, r)
    print(json.dumps({k: v for k, v in r.items() if k not in ("ap_params",)}, default=str)[:4000])


def save(label, r):
    with open(OUT + ".lock", "w") as lk:
        fcntl.flock(lk, fcntl.LOCK_EX)
        try:
            allr = json.load(open(OUT))
        except (OSError, ValueError):
            allr = {}
        allr[label] = r
        json.dump(allr, open(OUT, "w"), indent=1, default=str)
    table()


# ------------------------------------------------------------------ table
def table():
    try:
        allr = json.load(open(OUT))
    except (OSError, ValueError):
        return
    f = lambda r, x, n=8, p=2: (f"{r[x]:{n}.{p}f}" if isinstance(r.get(x), (int, float)) else "-".rjust(n))  # noqa: E731
    cols = [("dist_at_inject", "d inj"), ("fc_change_after_s", "FC chg s"), ("flown_until_fc_change", "flown"),
            ("pass_3m_after_s", "3m at s"), ("dist_at_takeover", "d take"), ("dist_max_after_inject", "d max"),
            ("tilt_err_deg_before", "tilt bef"), ("tilt_err_deg_after", "tilt aft"), ("yaw_err_deg_after", "yaw aft"),
            ("speed_max_window_mps", "v max"), ("height_change_max_window", "dh max"), ("window_s", "win s"),
            ("ekf_err_change_max_to_takeover", "ekf err")]
    lines = ["run".ljust(14) + "case".rjust(5) + "h m".rjust(5) + "".join(n.rjust(9) for _, n in cols) +
             "  FC mode at change; pilot; FC texts (s after the failure or takeover start)"]
    for k in sorted([k for k in allr if not k.startswith("arm_")], key=lambda k: (allr[k].get("height") or 0, k)):
        r = allr[k]
        ev = []
        if r.get("fc_change_mode"):
            ev.append(f"FC -> {r['fc_change_mode']}")
        ev += [f"{t:g}s pilot {m}" for t, m in r.get("pilot", [])]
        ev += [f"{t:g}s REFUSED {m}" for t, m in r.get("refused", [])]
        ev += [f"{t:g}s FC {m}" for t, m in r.get("fc_modes", [])]
        ev += [f"{t:g}s '{x}'" for t, x in r.get("fc_texts", []) if not x.startswith("PreArm")]
        ev += [f"{t:g}s {w}" for t, w in (r.get("events") or [])[1:]]
        if r.get("metrics_error") or (r.get("marks") or {}).get("scenario_error"):
            ev.append("ERROR " + str(r.get("metrics_error") or r["marks"]["scenario_error"]))
        lines.append(k.ljust(14) + str(r.get("case_kind", "?")).rjust(5) + f"{r.get('height', 0):5.0f}" +
                     "".join(f(r, x, 9) for x, _ in cols) + "  " + "; ".join(ev)[:500])
    arm = [k for k in sorted(allr) if k.startswith("arm_")]
    if arm:
        lines += ["", "Task 1: arming with ARMING_CHECK 1 (firmware default), the program running, aircraft on the ground",
                  "(no fix below 3 m). Times in s after each arm command (sim time). ACK result 0 = accepted, 4 = failed."]
        for k in arm:
            r = allr[k]
            lines.append(f"{k}: hold rms {r.get('rms')} max {r.get('max')} m, gps_in_use_s {r.get('gps_in_use_s')}, "
                         f"fix {(r.get('marks') or {}).get('arm_fix')}, events {[w for _, w in (r.get('events') or [])][:4]}")
            for a_ in (r.get("marks") or {}).get("arm_log", []):
                lines.append(f"  {a_['what']}, {a_['mode']} at {a_['t']} s: armed {a_['armed']}, ACK {a_['acks']}, "
                             f"texts {a_['texts']}"[:600])
            warn = [(t, x) for t, x in r.get("fc_texts_all", []) if not x.startswith(("PreArm", "EKF3 IMU", "AHRS"))]
            lines.append(f"  other FC texts (sim s): {warn}"[:900])
    lines += ["", "apsim/safety_matrix.py. ArduCopter 4.6 SITL (apsim/ardupilot), variant apb (the onboard program with",
              "the fixed 80 ms board model), video JUSTICE_D, ground ottawa2019fall (tuning site), wind w20g40, SITL",
              "instance 1, SITL default failsafe parameters (FS_EKF_ACTION 1 Land, FS_EKF_THRESH 0.8). Cases: see the",
              "file header. Times in s after the failure starts (case a: after the takeover, 10 s into the hold).",
              "d inj / d take / d max: true horizontal distance from the hold point at the failure, at the takeover, and",
              "the largest after the failure (m). FC chg s: time from the program stop to the FC mode change; flown: true",
              "distance flown in that time (m). 3m at s: when the true distance passed 3 m. tilt bef / tilt aft: largest",
              "tilt error, EKF (ATTITUDE) against the truth (SIM_STATE), in the 5 s before the failure / in the 10 s after",
              "the takeover (deg). yaw aft: largest change of the EKF yaw error in that window (deg). v max: largest true",
              "horizontal speed in that window (m/s, 0.25 s differences). dh max: largest height change in that window",
              "(m). win s: window length flown (shorter when the run ended). ekf err: largest change of the EKF position",
              "error from the failure to the takeover (m). One seed per cell unless the table shows more."]
    open(TXT, "w").write("\n".join(lines) + "\n")


# ------------------------------------------------------------------ runner
def sitl_running():
    out = []
    for p in os.listdir("/proc"):
        if p.isdigit():
            try:
                a = open(f"/proc/{p}/cmdline", "rb").read().split(b"\0")
            except OSError:
                continue
            if a and a[0].endswith(b"arducopter"):
                out.append(b" ".join(a).decode(errors="replace"))
    return out


def mine(cmd):
    return f" -I {INSTANCE} " in cmd


def docker_running(name):
    r = subprocess.run(["docker", "ps", "-q", "-f", f"name=^{name}$"], capture_output=True, text=True)
    return bool(r.stdout.strip())


def stop_sitl():
    subprocess.run(["sh", os.path.join(HERE, "run_sitl.sh"), "stop", str(INSTANCE)], capture_output=True)


def run(labels):
    labels = labels or CORE
    if len(labels) > 12:
        raise SystemExit("at most 12 runs")
    try:
        done = set(json.load(open(OUT)))
    except (OSError, ValueError):
        done = set()
    ap_sim_before = docker_running("ap_sim")
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    for label in labels:
        case(label)
        if label in done:
            print("skip (done)", label, flush=True)
            continue
        t0, go = time.time(), False
        while True:
            sitl = sitl_running()
            others, busy, one_min = [x for x in sitl if not mine(x)], docker_running("px4_sim"), os.getloadavg()[0]
            if not any(mine(x) for x in sitl) and not others and not busy and one_min < LOAD_MAX:
                go = True
                break
            if time.time() - t0 > WAIT_MAX_S:
                print(time.strftime("%H:%M:%S"), "gate closed for", WAIT_MAX_S, "s; skip", label, flush=True)
                break
            print(time.strftime("%H:%M:%S"), f"wait: load {one_min:.1f} (max {LOAD_MAX}), other SITL {len(others)}, "
                  f"instance {INSTANCE} busy {any(mine(x) for x in sitl)}, px4_sim {busy}", flush=True)
            time.sleep(20)
        if not go:
            continue
        print(time.strftime("%H:%M:%S"), "start", label, "load", os.getloadavg(), flush=True)
        log = open(os.path.join(HERE, "logs", f"saf_{label}.txt"), "w")
        p = subprocess.Popen([PY, os.path.abspath(__file__), "one", label], stdout=log, stderr=subprocess.STDOUT,
                             start_new_session=True)
        try:
            p.wait(timeout=RUN_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            print(time.strftime("%H:%M:%S"), "TIMEOUT", label, "- killing it", flush=True)
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(p.pid, sig)
                except ProcessLookupError:
                    break
                time.sleep(3)
            save(label, dict(case=label, case_kind=case(label)[1], height=case(label)[0]["height"],
                             events=[(0, f"ERROR timeout after {RUN_TIMEOUT_S} s")]))
        finally:
            stop_sitl()
            log.close()
        print(time.strftime("%H:%M:%S"), "end", label, flush=True)
    if not ap_sim_before and docker_running("ap_sim") and not sitl_running():
        subprocess.run(["docker", "rm", "-f", "ap_sim"], capture_output=True)   # the container this script started
        print("stopped the ap_sim container (it was not running before)", flush=True)


if __name__ == "__main__":
    a = sys.argv[1:]
    if not a:
        print(__doc__)
    elif a[0] == "one":
        one(a[1])
    elif a[0] == "run":
        run(a[1:])
    elif a[0] == "table":
        table()
        print(open(TXT).read() if os.path.exists(TXT) else "no results yet")
    elif a[0] == "case":
        for x in a[1:] or CORE + SPARE:
            print(x, case(x))
