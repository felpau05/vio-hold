#!/usr/bin/env python3
"""fc_link against the fake flight controller (tests_apb/fake_fc.py, separate process, UDP localhost).

A. PX4: firmware detected; streams set (ATTITUDE 100 Hz, LOCAL_POSITION_NED 10 Hz, HIGHRES_IMU off);
   clock offset fitted with 80 ms offset and 20 ms jitter injected; attitude content in the companion
   clock; parameter check and --set-params; TIMESYNC both ways; VISION_POSITION_ESTIMATE fields.
B. ArduPilot: detected; parameter names (GPS_TYPE alias); origin; fix angles finite.
C. Unsupported: generic MAVLink autopilot, MSP Betaflight on a serial port, nothing at all.
Run: ~/.venvs/vio/bin/python tests_apb/test_fc_link.py      (about 40 s)
"""
import json
import math
import os
import pty
import subprocess
import sys
import tempfile
import threading
import time
from collections import namedtuple

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import fc_link            # noqa: E402
import gnss_rule          # noqa: E402
import settings as S      # noqa: E402
from fake_fc import BOOT_S, scenario   # noqa: E402

PY = sys.executable
fails = 0


def check(ok, text):
    global fails
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} {text}", flush=True)


def start_fake(port, log, **kw):
    t_start = time.monotonic()
    args = [PY, os.path.join(HERE, "fake_fc.py"), "--port", str(port), "--t-start", repr(t_start), "--log", log]
    for k, v in kw.items():
        args += [f"--{k.replace('_', '-')}", str(v)]
    return subprocess.Popen(args), t_start


def read_log(path, kind):
    if not os.path.exists(path):
        return []
    return [r for r in map(json.loads, open(path)) if r["kind"] == kind]


Est1 = namedtuple("Est1", "t pos var n state recovered why")            # version-1 Hold result
Est2 = namedtuple("Est2", "t pos var n state recovered why cov yaw scale")


def part_a(d):
    print("A. PX4 fake, clock offset 80 ms, arrival jitter 0-20 ms")
    log = os.path.join(d, "a.jsonl")
    fake, t_start = start_fake(14650, log, offset=0.080, jitter=0.020, seconds=40)
    s = dict(S.load("/nonexistent"), fc_device="udpin:127.0.0.1:14650", msp_device="/nonexistent")
    fc, why = fc_link.connect(s, timeout=5)
    try:
        check(fc is not None and fc.firmware == "px4", f"firmware detected: {fc and fc.firmware} {why or ''}")
        time.sleep(1.5)
        before = (fc.rate("ATTITUDE"), fc.rate("HIGHRES_IMU"), fc.rate("LOCAL_POSITION_NED"))
        ack = fc.setup_streams()
        time.sleep(3.0)
        after = (fc.rate("ATTITUDE"), fc.rate("HIGHRES_IMU"), fc.rate("LOCAL_POSITION_NED"))
        check(ack == 0 and 90 <= after[0] <= 110 and after[1] == 0 and 8 <= after[2] <= 12,
              f"streams: before ATTITUDE {before[0]:.0f} Hz, HIGHRES_IMU {before[1]:.0f}, LOCAL_POSITION_NED "
              f"{before[2]:.0f}; after ATTITUDE {after[0]:.0f} Hz, HIGHRES_IMU {after[1]:.0f}, LOCAL_POSITION_NED "
              f"{after[2]:.0f} (ack {ack})")
        # True FC->companion offset B: t_companion = t_fc + B.
        B = t_start - BOOT_S - 0.080
        tf = np.linspace(time.monotonic() - B - 3.0, time.monotonic() - B - 0.1, 50)
        err = np.array([fc.clock.to_local(t) - (t + B) for t in tf]) * 1e3
        check(fc.clock.ready() and np.abs(err).max() < 3.0,
              f"clock mapping error over the last 3 s: {err.min():+.2f} .. {err.max():+.2f} ms (limit 3 ms; "
              f"the injected jitter averages 10 ms)")
        _, att_fn, h_fn = scenario("ground")
        a = fc.att.array()
        ts = a[-150:-10:10, 0]
        e = [np.degrees(np.abs(np.interp(t, a[:, 0], a[:, i + 1]) - att_fn(np.array([t - t_start]))[0][i])).max()
             for t in ts for i in range(3)]
        check(max(e) < 0.3, f"attitude in the companion clock matches the truth: worst {max(e):.3f} deg "
                            f"(rotation up to 30 deg/s, so 1 ms = 0.03 deg)")
        check(a.shape[1] == 7 and abs(fc.hgt.rows[-1][1] - 2.0) < 1e-6,
              f"attitude rows carry rates ({a.shape[1]} columns); height {fc.hgt.rows[-1][1]:.2f} m (true 2.00)")
        lp = fc.local_pos()
        check(lp is not None, f"FC local position for re-anchor: {lp}")

        lines = fc_link.check_params(fc, s)
        bad = sorted(l.split()[2].rstrip(":") for ok, l in lines if not ok)
        check(bad == ["EKF2_DELAY_MAX", "EKF2_EV_CTRL", "EKF2_GPS_CTRL", "EKF2_HGT_REF", "EKF2_RNG_CTRL"],
              f"parameter check on PX4 v1.15 defaults: FAIL for {bad}; e.g. '{[l for ok, l in lines if not ok][0]}'")
        lines = fc_link.check_params(fc, s, write=True)
        check(all(ok for ok, _ in lines), f"--set-params: {sum(ok for ok, _ in lines)}/{len(lines)} pass; "
                                          f"e.g. '{[l for _, l in lines if 'wrote' in l][0]}'")
        lines = fc_link.check_params(fc, s)
        check(all(ok for ok, _ in lines), "parameters read back right after the write")

        fc.ping_timesync(n=10)
        time.sleep(0.3)
        replies = read_log(log, "timesync_reply")
        check(len(fc.rtts) >= 8 and len(replies) >= 10,
              f"TIMESYNC: our requests answered ({len(fc.rtts)}, RTT p90 {np.percentile(np.array(fc.rtts) * 1e3, 90):.1f} ms); "
              f"PX4 requests answered by us ({len(replies)})")

        t_fix = time.monotonic() - 0.08
        fc.send_fix(Est2(t_fix, np.array([1.5, -2.0]), 0.5, 80, "ok", False, None,
                         np.array([[0.3, 0.05], [0.05, 0.2]]), None, None))
        fc.send_fix(Est1(t_fix + 0.04, np.array([0.25, 0.75]), 0.5, 80, "ok", False, None))
        time.sleep(0.3)
        v = read_log(log, "vpe")
        ok = len(v) == 2
        if ok:
            a2, b2 = v
            ok = (a2["usec"] == int(t_fix * 1e6) and abs(a2["x"] - 1.5) < 1e-6 and abs(a2["y"] + 2.0) < 1e-6
                  and abs(a2["z"] + 2.0) < 1e-3 and all(a2["angles_nan"]) and None not in a2["covariance"]
                  and abs(a2["covariance"][0] - 0.3) < 1e-6 and abs(a2["covariance"][1] - 0.05) < 1e-6
                  and abs(a2["covariance"][6] - 0.2) < 1e-6 and a2["covariance"][11] == 0.0 and a2["reset_counter"] == 0
                  and abs(b2["covariance"][0] - 0.25) < 1e-6 and abs(b2["covariance"][6] - 0.25) < 1e-6
                  and a2["compid"] == 197)
        check(ok, f"VISION_POSITION_ESTIMATE fields: {json.dumps(v[0]) if v else 'none received'}")
        check(len(v) == 2, f"version-1 Est (no cov): covariance[0], [6] = var/2 = "
                           f"{v[1]['covariance'][0] if len(v) == 2 else '?'}")
    finally:
        if fc:
            fc.close()
        fake.terminate()
        fake.wait()


def part_b(d):
    print("B. ArduPilot fake")
    log = os.path.join(d, "b.jsonl")
    fake, _ = start_fake(14652, log, firmware="ardupilot", seconds=20)
    s = dict(S.load("/nonexistent"), fc_device="udpin:127.0.0.1:14652", msp_device="/nonexistent")
    fc, why = fc_link.connect(s, timeout=5)
    try:
        check(fc is not None and fc.firmware == "ardupilot", f"firmware detected: {fc and fc.firmware} {why or ''}")
        fc.setup_streams()
        time.sleep(4.0)
        lines = fc_link.check_params(fc, s)
        names = [l.split()[2].rstrip(":") for _, l in lines]
        bad = sorted(n for (ok, _), n in zip(lines, names) if not ok)
        check("GPS_TYPE" in names and bad == ["AHRS_GPS_USE", "EK3_GSF_RUN_MASK", "EK3_GSF_USE_MASK", "EK3_SRC1_POSXY",
                                              "EK3_SRC1_VELXY", "EK3_SRC1_VELZ", "VISO_DELAY_MS", "VISO_TYPE"],
              f"ArduPilot parameter check (GPS1_TYPE read as GPS_TYPE): FAIL for {bad}")
        g = gnss_rule.check(fc.firmware, fc.get_param)
        g_bad = sorted(l.split()[3] for ok, l in g if not ok)
        check(len(g) == 20 and g_bad == ["AHRS_GPS_USE", "EK3_GSF_RUN_MASK", "EK3_GSF_USE_MASK", "EK3_SRC1_POSXY",
                                         "EK3_SRC1_VELXY", "EK3_SRC1_VELZ"]
              and any(l.startswith("PASS GNSS rule: GPS_TYPE = 1") for _, l in g),
              f"GNSS rule on ArduCopter defaults ({len(g)} lines): FAIL for {g_bad}")
        fc_link.check_params(fc, s, write=True)
        g = gnss_rule.check(fc.firmware, fc.get_param)
        check(all(ok for ok, _ in g), f"GNSS rule after --set-params: {[l for ok, l in g if not ok] or 'all PASS'}")
        fc.set_param("GPS_TYPE", 0)
        fc.set_param("EK3_SRC3_YAW", 8)
        g_bad = sorted(l.split()[3] for ok, l in gnss_rule.check(fc.firmware, fc.get_param) if not ok)
        check(g_bad == ["EK3_SRC3_YAW", "GPS_TYPE"], f"GNSS rule: GPS off and GSF yaw in source set 3: FAIL for {g_bad}")
        fc.send_origin(45.4, -75.7, 60.0)
        fc.send_fix(Est1(time.monotonic() - 0.08, np.array([0.1, 0.2]), 0.4, 80, "ok", False, None))
        time.sleep(0.3)
        o, v = read_log(log, "origin"), read_log(log, "vpe")
        check(len(o) == 1 and o[0]["lat"] == 454000000 and o[0]["alt"] == 60000, f"SET_GPS_GLOBAL_ORIGIN: {o}")
        check(len(v) == 1 and not any(v[0]["angles_nan"]), f"ArduPilot fix carries the FC attitude: "
                                                           f"{v and (v[0]['roll'], v[0]['pitch'], v[0]['yaw'])}")
    finally:
        if fc:
            fc.close()
        fake.terminate()
        fake.wait()


def msp_responder(master, stop):
    buf = b""
    while not stop.is_set():
        try:
            buf += os.read(master, 64)
        except OSError:
            time.sleep(0.01)
            continue
        while (i := buf.find(b"$M<")) >= 0 and len(buf) >= i + 6:
            cmd = buf[i + 4]
            body = bytes([0, 1, 46]) if cmd == 1 else b"BTFL"
            chk = len(body) ^ cmd
            for x in body:
                chk ^= x
            os.write(master, b"$M>" + bytes([len(body), cmd]) + body + bytes([chk]))
            buf = buf[i + 6:]


def part_c(d):
    print("C. firmware without external vision")
    log = os.path.join(d, "c.jsonl")
    fake, _ = start_fake(14654, log, firmware="generic", seconds=10)
    s = dict(S.load("/nonexistent"), fc_device="udpin:127.0.0.1:14654", msp_device="/nonexistent")
    fc, why = fc_link.connect(s, timeout=4)
    fake.terminate()
    fake.wait()
    check(fc is None and "not supported" in why, f"generic MAVLink autopilot: {why}")

    master, slave = pty.openpty()
    stop = threading.Event()
    threading.Thread(target=msp_responder, args=(master, stop), daemon=True).start()
    s = dict(s, fc_device="udpin:127.0.0.1:14656", msp_device=os.ttyname(slave))
    t1 = time.monotonic()
    fc, why = fc_link.connect(s, timeout=2)
    stop.set()
    check(fc is None and "Betaflight" in why and "no external-vision input" in why,
          f"no heartbeat, MSP probe on a serial port: {why} ({time.monotonic() - t1:.1f} s)")
    os.close(master)
    os.close(slave)

    s = dict(s, msp_device="/nonexistent")
    fc, why = fc_link.connect(s, timeout=2)
    check(fc is None and "no MAVLink heartbeat" in why, f"nothing answers: {why}")


def main():
    t0 = time.time()
    with tempfile.TemporaryDirectory() as d:
        part_a(d)
        part_b(d)
        part_c(d)
    print(f"{'ALL PASS' if not fails else f'{fails} FAIL'} in {time.time() - t0:.0f} s")
    return fails


if __name__ == "__main__":
    sys.exit(main())
