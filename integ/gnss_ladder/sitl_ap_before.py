#!/usr/bin/env python3
"""Closed-loop test on ArduCopter 4.6 SITL (Orqa's fork, apsim/ardupilot), as px4sim/sitl.py does for PX4.

ArduCopter flies (its own physics, SITL quad model), wind pushes, and in variant "apb" the only horizontal
position it gets is the onboard program's (px4sim/onboard.py -> apb_main.run_live -> fc_link) from a picture
that px4sim/camera.py renders from the true pose. The onboard link goes through a relay that carries only
LINK_BPS bytes per second each way, as the 230400-baud UART does.

  ~/.venvs/vio/bin/python apsim/sitl_ap.py <variant> <height> <wind> [hold_s] [key=value ...]
      variant: apb | truth | gps | none     wind: calm | w20g40 | w30g55
  ~/.venvs/vio/bin/python apsim/sitl_ap.py table

Paced by ArduPilot's clock (ATTITUDE time_boot_ms on the supervisor link). Truth: SIM_STATE.
"""
import collections
import fcntl
import json
import math
import multiprocessing
import os
import socket
import subprocess
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")   # SIM_STATE lat_int/lon_int are MAVLink 2 extensions
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path.insert(0, PROJ)
sys.path.insert(0, os.path.join(PROJ, "px4sim"))
import camera as C  # noqa: E402
import sitl as P    # noqa: E402  px4sim/sitl.py: Link base, wind series, score, sim_settings, onboard log

RUNS = os.path.join(HERE, "runs")
P.RUNS = RUNS                                   # sim_settings / apb_results write and read here
RESULTS, TABLE = os.path.join(HERE, "results.json"), os.path.join(HERE, "results.txt")
LINK_BPS = 11520     # assumed: 230400 baud at half load (research/hardware.md: PX4 auto rate = baud/20); the
                     # raw 8N1 limit is 23040 B/s. shortcut: half the UART, upgrade if ArduPilot is shown to fill it
UART_BUF = 1024      # assumed: bytes the FC queues for the port; more is not sent (ArduPilot defers a message
                     # when txspace is short; here it is dropped, which gives the same rate)
COPTER_MODES = {"ALT_HOLD": 2, "GUIDED": 4, "LOITER": 5}
AP_DEFAULTS = dict(variant="apb", instance=0, speedup=None, pilot_speed_up=500, wind_turb=1.0, att_hz=200.0,
                   engage_wait_s=25.0, sync_wait_s=0.0, cpu_model="fixed", board_s=0.08, feed_delay_s=0.0,
                   feed_noise_m=0.0, feed_hz=25.0, ap={}, wiggle_s=0.0, wiggle_pwm=150, wiggle_hz=0.7,
                   pilot_guided=0.0)
# pilot_guided > 0: the pilot selects GUIDED this many seconds after Loiter holds (the wind-assist experiment,
# settings wind_assist). At 0 s the program had no fix yet in 2 of 3 runs (still-hover liveness check) and
# rightly handed back to LOITER at once (runs c_wa2_100m_w20g40_s12/s13, first try).
# wiggle_s > 0: before Loiter the pilot rocks the roll and pitch sticks in AltHold for wiggle_s seconds (a sine of
# wiggle_pwm, roll at wiggle_hz, pitch at 1.3 x wiggle_hz), so calib.py can find the video delay in flight
# (integ/control/wiggle_calib.py). assumed 150 PWM: about 9 deg of lean (ANGLE_MAX 30 deg = 500 PWM).
# cpu_model "fixed": apsim/onboard_cpu.py, every Hold.process takes board_s on the board. assumed board_s 0.08:
# the mean of the clean wall-model runs (64-83 ms per frame at 50-122 m, apsim/results_clean.txt); used for
# the control work so that other programs on the laptop cannot change the fix rate (load average up to 41).
# It loses the cost differences of Hold's paths (whole-picture fallback on grass). "thread": onboard_cpu.py
# with cpu_factor x thread CPU time. "wall": px4sim/onboard.py as before (cpu_factor x wall time).
# feed_*: variant "truth" as a stand-in for our fixes (parameter sweeps at speedup 3): the true position
# feed_delay_s old (stamped with its true time), white noise feed_noise_m per axis, feed_hz fixes per second.
# wind_turb: assumed 1 m/s SIM_WIND_TURB on top of the gust series (SIH had none); gust peaks can pass 40 km/h by it.
# pilot_speed_up 500 cm/s: shortcut, a fast climb saves time; the hold does not depend on it.


def ports(c):
    b = 15000 + 20 * c["instance"]     # away from PX4 SITL's 14540/14550/14580/18570
    return b, b + 1, b + 2             # supervisor (SERIAL0), FC side of the relay (SERIAL1), onboard program


def parm_file(c, s_onb, label):
    """Autopilot parameters of the case. apb/truth/none: the onboard program's params_ardupilot (GNSS off,
    external nav); gps: the SITL defaults (GPS on, no vision)."""
    p = dict(ARMING_CHECK=0, PILOT_SPEED_UP=c["pilot_speed_up"], SIM_WIND_T=1, SIM_WIND_SPD=0, SIM_WIND_TURB=0,
             SIM_WIND_DIR=c["wind_from_deg"],   # SIM_WIND_T 1: same wind at every height (default: weaker below 60 m)
             SERIAL1_BAUD=115)   # the relay's 11520 B/s; ArduPilot takes this baud's transfer time off each
                                 # message's arrival time (UARTDriver::receive_time_constraint_us). Default 57
                                 # took 22 ms off a fix that the relay carried in 11 ms.
    if c["variant"] != "gps":
        p.update(s_onb["params_ardupilot"])
    if c["variant"] == "truth":
        p["VISO_DELAY_MS"] = round(c["feed_delay_s"] * 1000)   # the truth fix is sent this old
    p.update(c.get("ap", {}))
    path = os.path.join(RUNS, label + ".parm")
    open(path, "w").write("".join(f"{k} {v}\n" for k, v in p.items()))
    return os.path.relpath(path, HERE), p


class Relay:
    """UDP relay with the onboard UART's limits: LINK_BPS bytes/s each way, a UART_BUF queue, whole
    datagrams dropped when it is full. Counts MAVLink messages sent and dropped by id."""

    def __init__(self, port_fc, port_onb):
        self.a = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.a.bind(("127.0.0.1", port_fc))
        self.b = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.b.bind(("127.0.0.1", 0))
        self.fc_addr, self.onb_addr, self.stop = None, ("127.0.0.1", port_onb), False
        self.sent = {d: collections.Counter() for d in ("down", "up")}
        self.dropped = {d: collections.Counter() for d in ("down", "up")}
        self.bytes = {"down": 0, "up": 0}
        for sock, d in ((self.a, "down"), (self.b, "up")):
            q = collections.deque()
            threading.Thread(target=self._rx, args=(sock, d, q), daemon=True).start()
            threading.Thread(target=self._tx, args=(d, q), daemon=True).start()

    @staticmethod
    def ids(data):
        out, i = [], 0
        while i + 8 <= len(data):
            if data[i] == 0xFD:
                n = data[i + 1]
                out.append(data[i + 7] | data[i + 8] << 8 | data[i + 9] << 16 if i + 10 <= len(data) else -1)
                i += 12 + n + (13 if data[i + 2] & 1 else 0)
            elif data[i] == 0xFE:
                out.append(data[i + 5])
                i += 8 + data[i + 1]
            else:
                i += 1
        return out

    def _rx(self, sock, d, q):
        sock.settimeout(0.2)
        while not self.stop:
            try:
                data, addr = sock.recvfrom(65536)
            except (socket.timeout, OSError):
                continue
            if d == "down":
                self.fc_addr = addr
            if sum(len(x) for x in list(q)) + len(data) > UART_BUF:
                self.dropped[d].update(self.ids(data))
            else:
                q.append(data)

    def _tx(self, d, q):
        t_free = time.monotonic()
        while not self.stop:
            if not q:
                time.sleep(0.0005)
                continue
            data = q[0]
            t_free = max(t_free, time.monotonic()) + len(data) / LINK_BPS     # serial time of these bytes
            while time.monotonic() < t_free:
                time.sleep(0.0005)
            q.popleft()
            try:
                if d == "down":
                    self.b.sendto(data, self.onb_addr)
                elif self.fc_addr:
                    self.a.sendto(data, self.fc_addr)
            except OSError:
                pass
            self.sent[d].update(self.ids(data))
            self.bytes[d] += len(data)

    def close(self):
        self.stop = True
        time.sleep(0.3)
        self.a.close()
        self.b.close()


class APLink(P.Link):
    """Supervisor link on SERIAL0: truth from SIM_STATE, ArduPilot's attitude and position, RC sticks."""

    def __init__(self, c, port):
        super().__init__(c, port)
        self.t_att = self.a_att = None
        self.q_check = []                      # SIM_STATE roll/pitch minus the roll/pitch of its quaternion
        self.lat0 = self.lon0 = None
        self.rp = (0, 0)                       # roll, pitch stick offsets (PWM) for the wiggle

    def start(self, timeout=60.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            m = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if m is not None and m.get_srcComponent() == 1 and m.autopilot != self.mav.MAV_AUTOPILOT_INVALID:
                break
        else:
            raise RuntimeError("no heartbeat from ArduCopter SITL")
        self.ts = (m.get_srcSystem(), 1)
        threading.Thread(target=self._recv, daemon=True).start()
        threading.Thread(target=self._sticks, daemon=True).start()
        self.interval(108, self.c["att_hz"])   # SIM_STATE: truth, as fast as the main loop sends it
        self.interval(30, self.c["att_hz"])    # ATTITUDE: the clock of the truth samples
        self.interval(32, 30.0)                # LOCAL_POSITION_NED

    def _recv(self):
        lat0, lon0 = map(math.radians, P.HOME)
        while not self.stop_all:
            msg = self.m.recv_match(blocking=True, timeout=0.2)
            if msg is None or msg.get_srcComponent() not in (1, 0) and msg.get_type() != "SIM_STATE":
                continue
            k = msg.get_type()
            now = time.monotonic()
            if k == "ATTITUDE":
                self.t_att, self.a_att = msg.time_boot_ms * 1e-3, now
                row = (self.t_att, msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)
                self.att_rows.append(row)
                self.att.add(*row)
            elif k == "SIM_STATE" and self.t_att is not None and self.n_truth < len(self.truth):
                # shortcut: stamped with the last ATTITUDE time (same main-loop pass, <= 2.5 ms); upgrade if a
                # time-stamped truth message is added to SITL
                t = self.t_att + (now - self.a_att)
                if self.n_truth and t <= self.truth[self.n_truth - 1, 0]:
                    continue
                n = (math.radians(msg.lat_int * 1e-7) - lat0) * 6371000.0
                e = (math.radians(msg.lon_int * 1e-7) - lon0) * 6371000.0 * math.cos(lat0)
                if self.alt0 is None:
                    self.alt0 = msg.alt
                q = np.array([msg.q1, msg.q2, msg.q3, msg.q4])
                if len(self.q_check) < 2000:
                    r, p = C.q_roll_pitch(q[None])
                    self.q_check.append(max(abs(float(r[0]) - msg.roll), abs(float(p[0]) - msg.pitch)))
                self.truth[self.n_truth] = (t, n, e, msg.alt - self.alt0, *q)
                self.arr[self.n_truth] = now
                self.n_truth += 1
            elif k == "LOCAL_POSITION_NED":
                self.ekf.append((msg.time_boot_ms * 1e-3, msg.x, msg.y, msg.z))
            elif k == "HEARTBEAT" and msg.get_srcComponent() == 1:
                self.mode, self.armed = msg.custom_mode, bool(msg.base_mode & 128)
            elif k == "STATUSTEXT":
                self.texts.append((self.now(), msg.text))

    def _sticks(self):                          # RC override (PWM); self.throttle 0..1000 as in the PX4 Link
        k = 0
        while not self.stop_all:
            with self.lock:
                self.m.mav.rc_channels_override_send(self.ts[0], 1, 1500 + self.rp[0], 1500 + self.rp[1],
                                                     int(1000 + self.throttle), 1500,
                                                     0, 0, 0, 0)
                if k % 25 == 0:
                    self.m.mav.heartbeat_send(6, 8, 0, 0, 0)
            k += 1
            time.sleep(0.04)

    def h_now(self):                            # the pilot climbs on the true height (no origin yet, no EKF height)
        return float(self.truth[self.n_truth - 1, 3]) if self.n_truth else 0.0


def yaw_offset(link):
    a = np.array(link.att_rows[-200:])
    return float(np.angle(np.mean(np.exp(1j * (a[:, 3] - C.q_yaw(link.truth_at(a[:, 0])[3]))))))


def truth_feed(link, cov=0.01):
    """Variant truth: the true position, turned into ArduPilot's heading frame (link.yaw_off), as
    VISION_POSITION_ESTIMATE at 25 Hz, from the ground up (as the onboard program's fixes)."""
    cv = [0.0] * 21
    cv[0] = cv[6] = cv[11] = max(cov, link.c["feed_noise_m"] ** 2)
    rng = np.random.default_rng(link.c["seed"] + 7)
    while not link.done:
        t = link.now() - link.c["feed_delay_s"]
        N, E, h, _ = link.truth_at(np.array([t]))
        N, E = N + link.c["feed_noise_m"] * rng.standard_normal(), E + link.c["feed_noise_m"] * rng.standard_normal()
        cs, sn = math.cos(link.yaw_off), math.sin(link.yaw_off)
        p = np.array([[cs, -sn], [sn, cs]]) @ np.array([N[0], E[0]])
        a = link.att_rows[-1]
        with link.lock:
            link.m.mav.vision_position_estimate_send(int(t * 1e6), float(p[0]), float(p[1]), -float(h[0]),
                                                     a[1], a[2], a[3], cv, 0)
        link.sent += 1
        link.wait_for(1.0 / link.c["feed_hz"])


def ground(c):
    g = c["ground"]
    return C.photo_ground(g) if not g.startswith("snow") else C.snow_ground(tarps="tarps" in g, helipad="pad" in g)


def run_case(case, quiet=False):
    c = dict(P.DEFAULTS, **AP_DEFAULTS)
    c.update(P.video_defaults(case))     # video="JUSTICE_D": the digital camera's defaults (px4sim/sitl.py)
    c.update(case)
    v = c["variant"]
    label = c["label"] or f"ap_{v}_{c['height']:.0f}m_{c['wind']}_s{c['seed']}"
    say = (lambda *a: None) if quiet else (lambda *a: print(f"[{label}]", *a, flush=True))
    os.makedirs(RUNS, exist_ok=True)
    g_port, fc_port, onb_port = ports(c)
    apb = v == "apb"
    c["settings"] = dict(dict(fc_device=f"udpin:127.0.0.1:{onb_port}",
                              origin_lat_lon_alt=[P.HOME[0], P.HOME[1], P.HOME_ALT]), **c["settings"])
    s_path, s_onb = P.sim_settings(c, label)
    parm, params = parm_file(c, s_onb, label)
    speed = c["speedup"] or (1 if apb else 3)   # apb: wall-clock rate, so the onboard CPU model holds
    res = dict(case=label, cfg={k: v for k, v in c.items() if k != "video_out"}, events=[], ap_params=params,
               link_bps=LINK_BPS)
    ev = lambda what: (res["events"].append((round(link.now() - min(link.t_engage, link.now()), 2), what)), say(what))
    sh = os.path.join(HERE, "run_sitl.sh")
    subprocess.run(["sh", sh, "stop", str(c["instance"])], capture_output=True)
    link = APLink(c, g_port)
    relay = Relay(fc_port, onb_port)
    subprocess.run(["sh", sh, "start", str(c["instance"]), str(g_port), str(fc_port), str(speed), parm,
                    os.path.relpath(os.path.join(RUNS, label + "_sitl.log"), HERE)], check=True, capture_output=True)
    child, tx, stop_ev, seen, hold_pt, wt = None, None, None, [], np.zeros(2), None
    try:
        link.start()
        cam = C.Camera(c, ground(c))
        if apb:   # the onboard program starts with the FC, as on the drone; the camera runs from the ground up
            import onboard
            if c["cpu_model"] in ("thread", "fixed"):
                import onboard_cpu as onboard
            ctx = multiprocessing.get_context("spawn")
            rx, tx = ctx.Pipe(duplex=False)
            stop_ev, status_q = ctx.Event(), ctx.Queue()
            log_path = os.path.join(RUNS, label + "_onboard.csv")
            for old in [f for f in os.listdir(RUNS) if f.startswith(label + "_onboard")]:
                os.remove(os.path.join(RUNS, old))
            while link.n_truth < 20:
                time.sleep(0.05)
            clk = ctx.Value("d", 0.0, lock=False)
            threading.Thread(target=P.publish_clock, args=(link, clk), daemon=True).start()
            time.sleep(0.1)
            child = ctx.Process(target=onboard.main, daemon=True,
                                args=(s_path, rx, stop_ev, log_path, dict(cpu_factor=c["cpu_factor"],
                                      reanchor_on_lost=c["reanchor_on_lost"],
                                      board_s=c["board_s"] if c["cpu_model"] == "fixed" else 0.0), status_q, clk))
            child.start()
            frames = C.Frames(cam, link, c, render=True)
            threading.Thread(target=P.feed_frames, args=(frames, link, tx, c, seen), daemon=True).start()
        link.wait_until(link.now() + 8.0, timeout=60)          # AHRS and EKF alignment on the ground
        if not link.att_rows or link.n_truth < 100:
            raise RuntimeError("ArduCopter sends no ATTITUDE or SIM_STATE")
        res["truth_q_check_deg"] = round(math.degrees(max(link.q_check)), 3)
        if v == "truth":                                        # the GCS sets the EKF origin, as the autotest does
            link.m.mav.set_gps_global_origin_send(link.ts[0], int(P.HOME[0] * 1e7), int(P.HOME[1] * 1e7),
                                                  int(P.HOME_ALT * 1000), 0)
            link.yaw_off, t_feed = yaw_offset(link), link.now()
            threading.Thread(target=truth_feed, args=(link,), daemon=True).start()
        if v == "gps":                                          # a pilot arms after the GPS lock (EKF origin)
            while not any("origin set" in x[1] for x in link.texts) and link.now() < 90:
                link.wait_for(0.5)
        link.throttle = -500                                    # stick down to arm (PWM 1000)
        link.set_mode(COPTER_MODES["ALT_HOLD"])
        link.wait_for(0.5)
        link.arm()
        t_arm = link.now()
        while not link.armed and link.now() - t_arm < 5.0:
            link.wait_for(0.1)
        if not link.armed:
            raise RuntimeError("ArduCopter did not arm: " + "; ".join(x[1] for x in link.texts[-5:]))
        target, t_climb = c["height"], link.now()
        while True:                                             # climb on the throttle stick, ALT_HOLD
            err = target - link.h_now()
            if abs(err) < 0.5 or link.now() - t_climb > 120:
                break
            link.throttle = 500 + math.copysign(float(np.clip(abs(err) * 60, 120, 480)), err)
            time.sleep(0.01)
        link.throttle = 500
        link.wait_for(4.0)
        res["climb_s"] = round(link.now() - t_climb, 1)
        if c["wiggle_s"] > 0:                                   # the timing wiggle, then 4 s to settle
            t_w = link.now()
            while link.now() - t_w < c["wiggle_s"]:
                x = 2 * math.pi * c["wiggle_hz"] * (link.now() - t_w)
                link.rp = (int(c["wiggle_pwm"] * math.sin(x)), int(c["wiggle_pwm"] * math.sin(1.3 * x + 1.0)))
                time.sleep(0.02)
            link.rp = (0, 0)
            res["wiggle_t"] = (t_w, link.now())
            link.wait_for(4.0)
        link.yaw_off = yaw_offset(link)                         # after the in-flight yaw alignment
        res["ekf_yaw_minus_true_deg"] = round(math.degrees(link.yaw_off), 2)

        t_start, t_try = link.now(), -1.0
        last_now, last_wall = -1.0, time.time()
        while True:
            time.sleep(0.005)
            now = link.now()
            if now != last_now:
                last_now, last_wall = now, time.time()
            elif time.time() - last_wall > 10:
                ev(f"simulator clock stopped at {now:.2f} s")
                break
            if link.t_engage == math.inf:
                if v == "none" or link.mode == COPTER_MODES["LOITER"]:
                    link.t_engage = now
                    hp = link.truth_at(np.array([now]))
                    hold_pt = np.array([hp[0][0], hp[1][0]])
                    ev(f"hold starts, mode {link.mode}, {now - t_start:.1f} s after the Loiter request started")
                    guided_sent = False
                    wt, wn, we = P.wind_series(c, c["hold_s"] + 5)
                    if P.WINDS[c["wind"]][1] > 0:
                        link.param("SIM_WIND_TURB", c["wind_turb"])
                    ww = 0
                elif now - t_start > c["engage_wait_s"]:
                    ev("ArduCopter never accepted Loiter: " + "; ".join(x[1] for x in link.texts[-3:]))
                    break
                elif now - t_try > 0.5:
                    link.set_mode(COPTER_MODES["LOITER"])
                    t_try = now
                continue
            tt = now - link.t_engage
            if c["pilot_guided"] > 0 and not guided_sent and tt >= c["pilot_guided"]:
                link.set_mode(COPTER_MODES["GUIDED"])
                guided_sent = True
                ev("pilot selects GUIDED")
            if P.WINDS[c["wind"]][1] > 0 and ww < len(wt) and tt >= wt[ww]:
                # SIH convention of wind_series: (N, E) of the 'from' direction; SITL takes speed and direction
                link.param("SIM_WIND_SPD", float(math.hypot(wn[ww], we[ww])))
                link.param("SIM_WIND_DIR", float(math.degrees(math.atan2(we[ww], wn[ww])) % 360))
                ww += 2                                         # 5 Hz: each write also costs a PARAM_VALUE on every link
            N, E, h, _ = link.truth_at(np.array([now]))
            dist = math.hypot(N[0] - hold_pt[0], E[0] - hold_pt[1])
            ok_modes = (COPTER_MODES["LOITER"], COPTER_MODES["GUIDED"]) if c["pilot_guided"] else (COPTER_MODES["LOITER"],)
            if v != "none" and link.mode not in ok_modes and not any("left Loiter" in x[1] for x in res["events"]):
                ev(f"ArduCopter left Loiter (mode {link.mode}): " + "; ".join(x[1] for x in link.texts[-3:]))
            if c["pilot_guided"] > 0 and tt > c["pilot_guided"] + 2.0 and link.mode == COPTER_MODES["LOITER"] and \
                    not any("back in LOITER" in x[1] for x in res["events"]):
                ev("back in LOITER (the program handed back): " + "; ".join(x[1] for x in link.texts[-2:]))
            if dist > c["flyaway_m"] and v != "none":
                ev(f"FLY-AWAY: {dist:.1f} m from the hold point; ALT_HOLD, vision stopped")
                link.done = True
                link.set_mode(COPTER_MODES["ALT_HOLD"])
                break
            if h[0] < 3.0:
                ev("height below 3 m: crashed or landed")
                break
            if tt >= c["hold_s"]:
                break
        t_end = link.now()
        link.done = True
        time.sleep(0.2)
        if apb:
            res.update(P.apb_results(c, link, child, stop_ev, tx, status_q, log_path, seen, t_end))
            res.pop("_logs", None)
        if link.t_engage < math.inf:
            res.update(P.score(link, link.t_engage, t_end, hold_pt, cam))
        T = max(t_end - min(link.t_engage, t_end), 1e-9)
        if v == "truth":
            res["fix_rate_hz"] = link.sent / max(link.now() - t_feed, 1e-9)
        res["link"] = {d: dict(bytes_per_s=round(relay.bytes[d] / max(link.now() - t_arm, 1e-9)),
                               sent=dict(relay.sent[d].most_common(12)), dropped=dict(relay.dropped[d].most_common(8)))
                       for d in ("down", "up")}
        if apb:   # what the onboard program got per second over the whole flight (relay counts, ATTITUDE = 30)
            dur = max(link.now() - t_arm, 1e-9)
            res["onboard_att_hz"] = round(relay.sent["down"][30] / dur, 1)
            res["onboard_att_dropped_hz"] = round(relay.dropped["down"][30] / dur, 1)
        res["ap_text"] = [(round(t - min(link.t_engage, t), 2), x) for t, x in link.texts][-30:]
        res["hold_s_flown"] = round(T, 1)
        json.dump(res, open(os.path.join(RUNS, label + ".json"), "w"), default=str)
        np.savez_compressed(os.path.join(RUNS, label + ".npz"), truth=link.truth[:link.n_truth],
                            ekf=np.array(link.ekf), att=np.array(link.att_rows),
                            wind=np.c_[wt, wn, we] if wt is not None else np.zeros((0, 3)),
                            t_engage=link.t_engage, hold_pt=hold_pt)
        keys = ("rms", "p95", "max", "drift_m_per_min", "roll_rms_deg", "pitch_rms_deg", "judge_rms", "fix_rate_hz",
                "pct_frames_fix", "age_ms_p50", "onboard_att_hz")
        say("  ".join(f"{k} {res[k]:.3g}" for k in keys if isinstance(res.get(k), (int, float))))
        return res
    finally:
        if child is not None:
            stop_ev.set()
            child.join(timeout=5)
            if child.is_alive():
                child.terminate()
        link.close()
        relay.close()
        subprocess.run(["sh", sh, "stop", str(c["instance"])], capture_output=True)


def save(r):
    """Merge one result into results.json (a lock: cases run side by side) and rewrite results.txt."""
    with open(RESULTS + ".lock", "w") as lk:
        fcntl.flock(lk, fcntl.LOCK_EX)
        try:
            res = json.load(open(RESULTS))
        except (OSError, ValueError):
            res = {}
        res[r["case"]] = {k: v for k, v in r.items() if k != "cfg"}
        json.dump(res, open(RESULTS, "w"), indent=1, default=str)
        table(res)


def table(res):
    cols = [("rms", "drift rms m"), ("max", "drift max m"), ("judge_rms", "judge m"), ("judge_jitter_rms", "jitter m"),
            ("roll_rms_deg", "roll deg"), ("pitch_rms_deg", "pitch deg"), ("pct_frames_fix", "% fix"),
            ("fix_rate_hz", "fix Hz"), ("age_ms_p50", "age ms"), ("drift_m_per_min", "drift m/min")]
    lines = ["case".ljust(34) + "".join(n.rjust(12) for _, n in cols) + "  events"]
    for k in sorted(res):
        r = res[k]
        cells = "".join((f"{r[c]:12.3g}" if isinstance(r.get(c), (int, float)) else "-".rjust(12)) for c, _ in cols)
        ev = "; ".join(f"{t:g}s {w}" for t, w in r.get("events", [])[1:])[:120]
        lines.append(k.ljust(34) + cells + "  " + ev)
    lines += ["", "ArduCopter 4.6 SITL (Orqa mil-mavl), SITL quad model, 60 s holds. Columns as px4sim/results.txt.",
              "drift: true horizontal distance from the hold point. apb = the onboard program (apb_main run_live, fc_link)",
              f"with a 10x slower CPU, over a {LINK_BPS} B/s relay; truth = true position as the fix; gps = GNSS Loiter,",
              "no vision; none = no GNSS, no vision (ALT_HOLD). Sets: ottawa2019fall and seeds 1-9 = tuning; holdout_*",
              "sites with seeds 101+ = hold-out."]
    open(TABLE, "w").write("\n".join(lines) + "\n")
    return "\n".join(lines)


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "table":
        print(table(json.load(open(RESULTS))))
        sys.exit(0)
    kv = dict(x.split("=", 1) for x in a if "=" in x)
    a = [x for x in a if "=" not in x]
    case = dict(variant=a[0], height=float(a[1]), wind=a[2], hold_s=float(a[3]) if len(a) > 3 else 60.0)
    for k, val in kv.items():
        try:
            case[k] = json.loads(val)
        except ValueError:
            case[k] = val
    try:
        r = run_case(case)
    except Exception as e:      # a broken run is logged, not lost
        c = dict(P.DEFAULTS, **AP_DEFAULTS)
        c.update(case)
        r = dict(case=c.get("label") or f"ap_{c['variant']}_{c['height']:.0f}m_{c['wind']}_s{c['seed']}",
                 events=[(0, f"ERROR {type(e).__name__}: {e}")])
        print(r["events"], flush=True)
    save(r)
    print(json.dumps({k: v for k, v in r.items() if k not in ("cfg", "ap_params")}, default=str)[:3000])
