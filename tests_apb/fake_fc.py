#!/usr/bin/env python3
"""A small fake flight controller over MAVLink/UDP, for testing fc_link and apb_main without hardware.

- HEARTBEAT 1 Hz as PX4, ArduPilot or a generic autopilot (iNav-like telemetry).
- ATTITUDE (with rates) and LOCAL_POSITION_NED from a scenario; before SET_MESSAGE_INTERVAL they come
  at "normal mode" rates (20 Hz / 1 Hz) next to a HIGHRES_IMU stream that should be turned off.
- FC clock: t_fc = monotonic - (t_start - BOOT_S) + offset (the injected clock offset). Every message
  leaves after a random 0..jitter delay, in order, like a serial link.
- Parameters (PX4 sends int parameters as raw int bits inside the float, like real PX4), PARAM_SET,
  SET_MESSAGE_INTERVAL with COMMAND_ACK, TIMESYNC (PX4 asks at 10 Hz and answers our requests).
- Every VISION_POSITION_ESTIMATE, SET_GPS_GLOBAL_ORIGIN and TIMESYNC reply received is written to
  --log as one JSON line, with the arrival time (time.monotonic, same clock as the test process).
- While the file --pause exists, ATTITUDE is not sent (stale-attitude test).

Run by the tests: python fake_fc.py --port 14650 --t-start T --log out.jsonl [--firmware px4|ardupilot|generic]
"""
import argparse
import json
import math
import os
import random
import struct
import time

os.environ["MAVLINK20"] = "1"
import numpy as np                 # noqa: E402
from pymavlink import mavutil      # noqa: E402

M = mavutil.mavlink
BOOT_S = 7.0                       # the fake FC booted this long before the test started

PX4_PARAMS = {"EKF2_EV_CTRL": (15, 6), "EKF2_GPS_CTRL": (7, 6), "EKF2_HGT_REF": (1, 6), "EKF2_BARO_CTRL": (1, 6),
              "EKF2_RNG_CTRL": (1, 6), "EKF2_EV_NOISE_MD": (0, 6), "EKF2_EV_DELAY": (0.0, 9), "COM_ARM_WO_GPS": (1, 6),
              "EKF2_DELAY_MAX": (110.0, 9)}   # assumed: PX4's default fusion horizon (ms, float); not checked here
AP_PARAMS = {"VISO_TYPE": (0, 2), "EK3_SRC1_POSXY": (3, 2), "EK3_SRC1_VELXY": (3, 2), "EK3_SRC1_POSZ": (1, 2),
             "EK3_SRC1_VELZ": (3, 2), "EK3_SRC1_YAW": (1, 2), "GPS_TYPE": (1, 2), "VISO_DELAY_MS": (10, 4),
             "AHRS_GPS_USE": (1, 2), "EK3_GSF_RUN_MASK": (3, 2), "EK3_GSF_USE_MASK": (3, 2), "LOG_BITMASK": (176126, 6)}
AP_PARAMS.update({f"EK3_SRC{n}_{k}": (0, 2) for n in (2, 3) for k in ("POSXY", "VELXY", "POSZ", "VELZ", "YAW")})


def scenario(name):
    """(pos_fn, att_fn, h_fn) of true time t (s since t_start). 'hover50': sim_test.hover at 50 m."""
    s = np.sin
    if name == "hover50":
        pos = lambda t: np.c_[1.5 * s(0.21 * t) + 0.5 * s(0.9 * t + 1), 1.5 * s(0.17 * t + 2) + 0.5 * s(1.1 * t)]
        att = lambda t: np.radians(np.c_[4 * s(2 * np.pi * 0.7 * t) + 2 * s(2 * np.pi * 1.9 * t + 1),
                                         4 * s(2 * np.pi * 0.6 * t + 2) + 2 * s(2 * np.pi * 2.3 * t),
                                         10 * s(2 * np.pi * 0.05 * t)])
        return pos, att, lambda t: 50.0 + 1.0 * s(0.3 * np.asarray(t, float))
    if name == "bench":            # hand-held 1.5 m over a floor, nose at 0.3 rad: 1 m forward, then 1 m right
        c, k = np.cos(0.3), np.sin(0.3)
        fwd = lambda t: np.clip((np.asarray(t) - 4.0) / 2.0, 0, 1)
        rgt = lambda t: np.clip((np.asarray(t) - 9.0) / 2.0, 0, 1)
        pos = lambda t: np.c_[fwd(t) * c - rgt(t) * k, fwd(t) * k + rgt(t) * c]
        att = lambda t: np.c_[np.radians(3 * s(2 * np.pi * 0.8 * t)), np.radians(3 * s(2 * np.pi * 0.6 * t + 1)),
                              0.3 + 0.0 * np.asarray(t, float)]
        return pos, att, lambda t: 1.5 + 0.0 * np.asarray(t, float)
    if name == "hand":             # calibration (integ/calib_loop.py): turned by hand, camera 60 m from the scene
        att = lambda t: np.radians(np.c_[15 * s(2 * np.pi * 0.5 * t), 12 * s(2 * np.pi * 0.4 * t + 1),
                                         20 * s(2 * np.pi * 0.3 * t)])
        pos = lambda t: 0.2 * np.c_[np.sin(att(t)[:, 1]), -np.sin(att(t)[:, 0])]   # 0.2 m hand pivot
        return pos, att, lambda t: 60.0 + 0.0 * np.asarray(t, float)
    pos = lambda t: np.c_[0.5 * s(0.3 * t), 0.4 * s(0.2 * t)]
    att = lambda t: np.radians(np.c_[10 * s(2 * np.pi * 0.5 * t), 8 * s(2 * np.pi * 0.4 * t), 30 * s(0.2 * t)])
    return pos, att, lambda t: 2.0 + 0.0 * np.asarray(t, float)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--t-start", type=float, required=True)
    p.add_argument("--log", required=True)
    p.add_argument("--firmware", default="px4")
    p.add_argument("--offset", type=float, default=0.080)
    p.add_argument("--jitter", type=float, default=0.020)
    p.add_argument("--scenario", default="ground")
    p.add_argument("--pause", default="")
    p.add_argument("--seconds", type=float, default=60.0)
    a = p.parse_args()

    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{a.port}", source_system=1, source_component=1)
    mav = conn.mav
    pos_fn, att_fn, h_fn = scenario(a.scenario)
    boot = a.t_start - BOOT_S - a.offset          # t_fc = monotonic - boot
    autopilot = {"px4": M.MAV_AUTOPILOT_PX4, "ardupilot": M.MAV_AUTOPILOT_ARDUPILOTMEGA}.get(a.firmware, M.MAV_AUTOPILOT_GENERIC)
    params = dict(PX4_PARAMS if a.firmware == "px4" else AP_PARAMS)
    rates = {"ATTITUDE": 20.0, "LOCAL_POSITION_NED": 1.0, "HIGHRES_IMU": 50.0, "HEARTBEAT": 1.0, "TIMESYNC": 10.0}
    ids = {M.MAVLINK_MSG_ID_ATTITUDE: "ATTITUDE", M.MAVLINK_MSG_ID_LOCAL_POSITION_NED: "LOCAL_POSITION_NED",
           M.MAVLINK_MSG_ID_HIGHRES_IMU: "HIGHRES_IMU"}
    nxt = {k: time.monotonic() for k in rates}
    queue, last_send = [], 0.0
    log = open(a.log, "a")
    rng = random.Random(1)

    def enqueue(t_sample, msg):
        nonlocal last_send
        last_send = max(last_send, t_sample + rng.uniform(0, a.jitter))
        queue.append((last_send, msg))

    def pv(name):
        v, ty = params[name]
        f = struct.unpack("<f", struct.pack("<i", int(v)))[0] if a.firmware == "px4" and ty != 9 else float(v)
        return M.MAVLink_param_value_message(name.encode(), f, ty, len(params), list(params).index(name))

    def write(kind, **kw):
        log.write(json.dumps(dict(kind=kind, t_arr=time.monotonic(), **kw)) + "\n")
        log.flush()

    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        now = time.monotonic()
        for k, hz in rates.items():
            if hz <= 0 or now < nxt[k]:
                continue
            ts = nxt[k]
            nxt[k] = max(ts + 1.0 / hz, now - 0.5)
            tt = ts - a.t_start
            t_ms = int((ts - boot) * 1000)
            if k == "HEARTBEAT":
                enqueue(ts, M.MAVLink_heartbeat_message(M.MAV_TYPE_QUADROTOR, autopilot, 0, 0, M.MAV_STATE_ACTIVE, 3))
            elif k == "ATTITUDE":
                if a.pause and os.path.exists(a.pause):
                    continue
                r = att_fn(np.array([tt]))[0]
                w = (att_fn(np.array([tt + 0.005]))[0] - att_fn(np.array([tt - 0.005]))[0]) / 0.01
                enqueue(ts, M.MAVLink_attitude_message(t_ms, *r, *w))
            elif k == "LOCAL_POSITION_NED":
                x, y = pos_fn(np.array([tt]))[0]
                h_fc = 0.2 if a.scenario == "bench" else float(h_fn(tt))      # bench: the FC sits on the ground
                enqueue(ts, M.MAVLink_local_position_ned_message(t_ms, x, y, -h_fc, 0, 0, 0))
                if a.firmware == "ardupilot":     # ArduPilot's height for fc_link (always sent, apsim finding)
                    enqueue(ts, M.MAVLink_global_position_int_message(t_ms, 0, 0, 0, int(h_fc * 1000), 0, 0, 0, 0))
            elif k == "HIGHRES_IMU":
                enqueue(ts, M.MAVLink_highres_imu_message(int((ts - boot) * 1e6), *[0.0] * 13, 0))
            elif k == "TIMESYNC" and a.firmware == "px4":
                enqueue(ts, M.MAVLink_timesync_message(0, int((ts - boot) * 1e9)))
        while queue and queue[0][0] <= now:
            mav.send(queue.pop(0)[1])
        m = conn.recv_match(blocking=False)
        while m is not None:
            kind = m.get_type()
            if kind == "COMMAND_LONG" and m.command == M.MAV_CMD_SET_MESSAGE_INTERVAL:
                name = ids.get(int(m.param1))
                if name:
                    rates[name] = 0.0 if m.param2 < 0 else 1e6 / m.param2
                mav.command_ack_send(m.command, M.MAV_RESULT_ACCEPTED)
            elif kind == "PARAM_REQUEST_READ":
                name = m.param_id.rstrip("\x00")
                if name in params:
                    mav.send(pv(name))
            elif kind == "PARAM_SET":
                name = m.param_id.rstrip("\x00")
                if name in params:
                    v = m.param_value
                    if a.firmware == "px4" and params[name][1] != 9:
                        v = struct.unpack("<i", struct.pack("<f", v))[0]
                    params[name] = (v, params[name][1])
                    mav.send(pv(name))
            elif kind == "TIMESYNC":
                if m.tc1 == 0:
                    mav.timesync_send(int((time.monotonic() - boot) * 1e9), m.ts1)
                else:
                    write("timesync_reply", tc1=m.tc1, ts1=m.ts1)
            elif kind == "VISION_POSITION_ESTIMATE":
                write("vpe", usec=m.usec, x=m.x, y=m.y, z=m.z, roll=m.roll, pitch=m.pitch, yaw=m.yaw,
                      covariance=[None if math.isnan(c) else c for c in m.covariance], reset_counter=m.reset_counter,
                      sysid=m.get_srcSystem(), compid=m.get_srcComponent(),
                      angles_nan=[math.isnan(v) for v in (m.roll, m.pitch, m.yaw)])
            elif kind == "SET_GPS_GLOBAL_ORIGIN":
                write("origin", lat=m.latitude, lon=m.longitude, alt=m.altitude)
            m = conn.recv_match(blocking=False)
        time.sleep(0.0005)


if __name__ == "__main__":
    main()
