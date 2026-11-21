#!/usr/bin/env python3
"""Closed loop: Betaflight SITL flies Position Hold on our flow and rangefinder messages.

The plant, the RC feed and the FDM (flight dynamics model) feed come from the firmware's own test harness
(src/test/sitl/sitl_harness.py in the clone at BF_SRC), imported at run time, not copied. Its plant is a crude
quad model: motor outputs -> body rates -> tilt -> acceleration with linear drag. We add wind to that drag.
Our side is msp_link.MspLink, the same code apb_main uses: it reads attitude, gyro and the FC height over MSP
and sends rangefinder + flow at 20 Hz from send_fix/send_speed. Fix = true position + noise; speed = true
speed + a constant error + noise (our VIO is not perfect). GPS is off: XY comes only from our messages.
Sequence: arm, climb on the throttle stick, switch POS HOLD + ALT HOLD on, hold for --hold seconds.
Result: true horizontal distance from the point where the hold started (RMS, max, at the end), per second.
Run: BF_SITL=<binary> BF_SRC=<clone> ~/.venvs/vio/bin/python betasim/closed_loop.py --gain 0 --seed 1
"""
import argparse
import importlib.util
import json
import math
import os
import struct
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import msp_link as M      # noqa: E402
import sitl               # noqa: E402

spec = importlib.util.spec_from_file_location("bf_harness", os.path.join(os.environ["BF_SRC"], "src/test/sitl/sitl_harness.py"))
H = importlib.util.module_from_spec(spec)
spec.loader.exec_module(H)
H.TELEMETRY_PORT = 0

ap = argparse.ArgumentParser()
ap.add_argument("--gain", type=float, default=0.0, help="msp_lock_gain: 0 = mode v1, > 0 = mode v2")
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--hold", type=float, default=60.0)
ap.add_argument("--wind", type=float, default=2.0, help="mean wind m/s toward the east")
ap.add_argument("--gust", type=float, default=1.0, help="gust sd m/s (Ornstein-Uhlenbeck, 3 s)")
ap.add_argument("--speed-bias", type=float, default=0.10, help="constant error of our speed, m/s toward the north")
ap.add_argument("--speed-sd", type=float, default=0.05, help="speed noise sd, m/s per axis")
ap.add_argument("--speed-tau", type=float, default=0.0,
                help="0: white speed noise; > 0: the speed error is a random process with this time constant (s)")
ap.add_argument("--gps", action="store_true",
                help="baseline: no flow; the firmware's own Position Hold on its virtual GPS (true position)")
ap.add_argument("--fix-sd", type=float, default=0.05)
ap.add_argument("--climb-to", type=float, default=5.0)
ap.add_argument("--extra", default="", help="more CLI lines, ';' separated")
ap.add_argument("--tag", default="")
a = ap.parse_args()
rng = np.random.default_rng(a.seed)              # wind
nrng = np.random.default_rng(a.seed + 1000)     # our fix and speed errors
tag = a.tag or f"g{a.gain:g}_s{a.seed}"
WD = os.path.join(os.environ.get("BF_WORK", "/tmp/betasim_work"), "cl_" + tag)
CFG = ["feature -GPS", "feature RANGEFINDER", "feature OPTICALFLOW", "set rangefinder_hardware = MTF02P",
       "set opticalflow_hardware = MT", "set rangefinder_max_range_cm = 1000", "set poshold_opticalflow_max_range = 1000",
       "set small_angle = 180", "set trust_mag = ON", "set debug_mode = BARO",
       "aux 0 0 0 1700 2100 0 0", "aux 1 11 1 1700 2100 0 0", "aux 2 3 1 1700 2100 0 0", "aux 3 1 2 1700 2100 0 0"] + \
      [x.strip() for x in a.extra.split(";") if x.strip()]
if a.gps:
    CFG = ["feature GPS", "set gps_provider = VIRTUAL"] + [c for c in CFG if "GPS" not in c and "OPTICALFLOW" not in c
                                                          and "RANGEFINDER" not in c and "hardware" not in c]


def status(fc):
    p = fc.request(M.MSP_STATUS_EX)
    if not p:
        return None
    n = p[15]
    return struct.unpack_from("<I", p, 17 + n)[0]          # arming disable flags


def main():
    sitl.provision(WD, CFG)
    proc = sitl.start(WD)
    rc, motors = H.RcFeed(), H.MotorFeed()
    fdm = H.FdmFeed(motors)
    wind = np.array([a.wind, 0.0])
    gust = np.zeros(2)
    step0 = fdm.model.step

    def step(dt, m):                                        # wind: drag acts on the speed through the air
        nonlocal gust
        step0(dt, m)
        if not fdm.model.on_ground():
            gust += -gust * dt / 3.0 + a.gust * math.sqrt(2 * dt / 3.0) * rng.standard_normal(2)
            w = wind + gust
            for i in range(2):
                fdm.model.vel[i] += H.K_DRAG * w[i] * dt
    fdm.model.step = step
    s = {"msp_device": f"tcp:127.0.0.1:{sitl.MSP_PORT}", "fc_baud": 115200, "att_hz": 40, "msp_lock_gain": a.gain,
         "msp_gyro_lsb_per_dps": 16.4}
    fc, log, out = None, [], {"args": vars(a), "cfg": CFG}
    stop = threading.Event()
    try:
        motors.start()
        rc.start()
        fdm.start()
        fc = M.MspLink(s)
        out["firmware"] = fc.identify()
        fc.setup_streams(wait=0.5)

        def feed():                                        # what apb_main does per good fix: send_fix, send_speed
            ou = np.zeros(2)
            while not stop.is_set():
                m = fdm.model
                n, e = m.pos[1], m.pos[0]
                if a.speed_tau > 0:
                    ou += -ou * 0.05 / a.speed_tau + a.speed_sd * math.sqrt(0.1 / a.speed_tau) * nrng.standard_normal(2)
                    err = ou
                else:
                    err = nrng.normal(0, a.speed_sd, 2)
                fc.send_fix(SimpleNamespace(t=time.monotonic(), pos=np.array([n, e]) + nrng.normal(0, a.fix_sd, 2)))
                fc.send_speed(time.monotonic(), m.vel[1] + a.speed_bias + err[0], m.vel[0] + err[1], -m.vel[2])
                time.sleep(0.05)
        if not a.gps:
            threading.Thread(target=feed, daemon=True).start()
        end = time.monotonic() + 40
        while (st := status(fc)) != 0 and time.monotonic() < end:
            time.sleep(0.5)
        out["arming_flags_before"] = st
        fc.request(205)                                    # MSP_ACC_CALIBRATION, as the harness does
        time.sleep(2.5)
        for _ in range(3):
            rc.set(4, H.RC_HIGH)
            t = time.monotonic()
            while not fc.armed and time.monotonic() - t < 6:
                time.sleep(0.2)
            if fc.armed:
                break
            rc.set(4, 1000)
            time.sleep(1.0)
        if not fc.armed:
            raise RuntimeError(f"did not arm; arming disable flags {status(fc)}")
        rc.set(2, 1600)
        t = time.monotonic()
        while fdm.model.pos[2] < a.climb_to and time.monotonic() - t < 20:
            time.sleep(0.1)
        rc.set(2, 1300)                                    # the harness value: throttle in the alt-hold deadband
        rc.set(5, H.RC_HIGH)                               # POS HOLD + ALT HOLD
        time.sleep(0.5)
        out["modes_after_switch"] = sorted(fc.mode or [])
        p0 = np.array(fdm.model.pos[:2])
        t0 = time.monotonic()
        out["engage"] = dict(t=0.0, pos_en=p0.tolist(), alt=fdm.model.pos[2])
        while time.monotonic() - t0 < a.hold:
            time.sleep(0.2)
            m = fdm.model
            d = np.array(m.pos[:2]) - p0
            dbg = fc.request(M.MSP_DEBUG, timeout=0.1)
            dbg = list(struct.unpack(f"<{len(dbg) // 2}h", dbg)) if dbg else None
            log.append(dict(t=round(time.monotonic() - t0, 2), de=round(d[0], 3), dn=round(d[1], 3), alt=round(m.pos[2], 2),
                            ve=round(m.vel[0], 3), vn=round(m.vel[1], 3), hold=bool(fc.mode and M.BOX_POSHOLD in fc.mode),
                            armed=fc.armed, fc_h=fc.hgt.rows[-1][1] if fc.hgt.rows else None, dbg=dbg))
            if not fc.armed or m.pos[2] <= 0.01:
                break
        stop.set()
        err = np.array([math.hypot(r["de"], r["dn"]) for r in log])
        span = log[-1]["t"] if log else 0
        out.update(samples=len(log), seconds=span, rms_m=float(np.sqrt(np.mean(err ** 2))) if len(err) else None,
                   max_m=float(err.max()) if len(err) else None, end_m=float(err[-1]) if len(err) else None,
                   end_dn=log[-1]["dn"] if log else None, end_de=log[-1]["de"] if log else None,
                   alt_min=min(r["alt"] for r in log) if log else None, alt_max=max(r["alt"] for r in log) if log else None,
                   hold_all=all(r["hold"] for r in log), armed_end=log[-1]["armed"] if log else None,
                   flow_sent=len(fc.sent), acks=fc.acks, errors=fc.errors, log=log)
        print(f"{tag}: {span:.0f} s held, RMS {out['rms_m']:.2f} m, max {out['max_m']:.2f} m, end {out['end_m']:.2f} m "
              f"(north {out['end_dn']:+.2f}, east {out['end_de']:+.2f}); alt {out['alt_min']:.1f}-{out['alt_max']:.1f} m; "
              f"POS HOLD on all samples {out['hold_all']}")
    finally:
        stop.set()
        for x in (rc, fdm):
            x.shutdown()
        motors.shutdown()
        if fc is not None:
            fc.close()
        sitl.stop(proc)
        os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
        with open(os.path.join(HERE, "results", f"closed_{tag}.json"), "w") as f:
            json.dump(out, f, indent=0)


main()
