#!/usr/bin/env python3
"""Item 6: attitude over the 230400-baud MAVLink link.

Part A: the attitude rate PX4 v1.15.4 actually sends on the APB link. Board defaults (DroneWuKong
Orqa_H7_PX4_Ports boards/orqa/apb/init/rc.board_defaults): MAV_0_MODE 2 (Onboard), MAV_0_RATE 0
(-> 230400/20 = 11520 B/s), LOGGER_ARGS "-m mavlink" (ULog over this link when requested).
PX4 mavlink_main.cpp v1.15.4 lines 1460-1521 set the Onboard streams; update_rate_mult() scales
every stream by (datarate * (1 - ulog share) - const) / sum(stream bandwidth), capped at 1; ULog
streaming may take up to 70 % (try_start_ulog_streaming(..., 0.7f)). UART4 TX buffer is 1500 B
(CONFIG_UART4_TXBUFSIZE in the board's default.px4board).
Payload sizes are MAVLink 2 common.xml lengths incl. extensions; 12 B header+CRC each.

Part B: the hold with attitude at those rates, with arrival-time stamps, and with ms truncation.
Part C: online timing. Hold.process needs attitude up to the last row's time (+-5 ms) when it is
called; called as soon as a frame arrives, how often does it fail with "attitude late"?

Run: ~/.venvs/vio/bin/python t7_attitude_link.py      (about 2.5 minutes)
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harness as X      # noqa: E402

S = X.S
BAUD_BPS = 230400 / 10        # 8N1 bytes per second
DATARATE = 230400 // 20       # MAV_0_RATE 0

# name: (Onboard default Hz, MAVLink2 payload bytes incl. extensions, advertised on a GNSS-off APB quad?)
ONBOARD = {
    "TIMESYNC": (10, 18, True), "HIGHRES_IMU": (50, 63, True), "LOCAL_POSITION_NED": (30, 28, True),
    "ATTITUDE": (100, 28, True), "ALTITUDE": (10, 32, True), "DISTANCE_SENSOR": (10, 39, False),
    "ESC_INFO": (10, 46, True), "ESC_STATUS": (10, 57, True), "MOUNT_ORIENTATION": (10, 20, False),
    "OBSTACLE_DISTANCE": (10, 167, False), "ODOMETRY": (30, 233, True), "ATTITUDE_QUATERNION": (50, 48, True),
    "ATTITUDE_TARGET": (10, 49, True), "AVAILABLE_MODES": (0.3, 46, True), "BATTERY_STATUS": (0.5, 54, True),
    "CURRENT_MODE": (0.5, 9, True), "ESTIMATOR_STATUS": (1, 42, True), "EXTENDED_SYS_STATE": (5, 2, True),
    "GLOBAL_POSITION_INT": (50, 28, False), "GPS_GLOBAL_ORIGIN": (1, 20, False), "HOME_POSITION": (0.5, 60, True),
    "NAV_CONTROLLER_OUTPUT": (10, 26, False), "PING": (1, 14, True), "POSITION_TARGET_GLOBAL_INT": (10, 51, False),
    "POSITION_TARGET_LOCAL_NED": (10, 51, True), "RC_CHANNELS": (20, 42, True), "SCALED_PRESSURE": (1, 16, True),
    "SERVO_OUTPUT_RAW_0": (10, 37, True), "SYS_STATUS": (5, 43, True), "SYSTEM_TIME": (1, 12, True),
    "VFR_HUD": (10, 20, True), "VIBRATION": (0.5, 32, True), "WIND_COV": (10, 40, True),
    "LINK_NODE_STATUS": (1, 36, True),
}


def part_a():
    bw = {n: hz * (pl + 12) for n, (hz, pl, adv) in ONBOARD.items() if adv}
    total = sum(bw.values())
    with_origin = total + sum(hz * (pl + 12) for n, (hz, pl, adv) in ONBOARD.items()
                              if n in ("GLOBAL_POSITION_INT", "POSITION_TARGET_GLOBAL_INT", "GPS_GLOBAL_ORIGIN"))
    print("A. PX4 v1.15.4 Onboard streams on the APB link (MAV_0_RATE 0 -> 11520 B/s)")
    print(f"   requested bandwidth of advertised streams: {total:.0f} B/s (with a global origin set: {with_origin:.0f})")
    for label, tot, ulog in (("no ULog streaming", total, 0.0), ("with origin set", with_origin, 0.0),
                             ("ULog streaming on (70 % share)", total, 0.7)):
        mult = min(1.0, DATARATE * (1 - ulog) / tot)
        print(f"   {label:32s} rate_mult {mult:.2f} -> ATTITUDE {100 * mult:5.1f} Hz, "
              f"ATTITUDE_QUATERNION {50 * mult:4.1f} Hz, HIGHRES_IMU {50 * mult:4.1f} Hz")
    top = sorted(bw.items(), key=lambda kv: -kv[1])[:5]
    print("   biggest consumers: " + ", ".join(f"{n} {b:.0f} B/s" for n, b in top))
    print(f"   UART4 TX buffer 1500 B drains in {1500 / BAUD_BPS * 1e3:.0f} ms at {BAUD_BPS:.0f} B/s; one burst of "
          f"ODOMETRY+HIGHRES_IMU+ATTITUDE_QUATERNION+LOCAL_POSITION_NED+ATTITUDE "
          f"({245 + 75 + 60 + 40 + 40} B) takes {(245 + 75 + 60 + 40 + 40) / BAUD_BPS * 1e3:.0f} ms")
    mult = min(1.0, DATARATE / total)
    return 100 * mult, 100 * min(1.0, DATARATE * 0.3 / total)


def part_b(rate_default, rate_ulog):
    print("B. hold error vs attitude feed (Ottawa photo, sim hover)")
    for h0 in (100, 200):
        g = X.Ground("ottawa2022", h0)
        pos, att_fn, h_fn = X.hover(h0)
        frames = X.frames_for(pos, att_fn, h_fn, g, noise=5.0)
        print(f"  --- {h0} m")
        for name, kw in ((f"100 Hz (sim assumption)", dict(att_hz=100)),
                         (f"{rate_default:.0f} Hz (PX4 Onboard default here)", dict(att_hz=rate_default)),
                         ("30 Hz", dict(att_hz=30)),
                         (f"{rate_ulog:.0f} Hz (ULog streaming on)", dict(att_hz=rate_ulog)),
                         ("100 Hz, time_boot_ms truncation", dict(att_hz=100, quant_ms=True)),
                         ("46 Hz, stamped on arrival (+1..21 ms)", dict(att_hz=46, jitter_s=0.020, att_lag=S.ATT_LAG + 0.001)),
                         ):
            att, hgt = X.fc_feeds(att_fn, h_fn, 20.0, **kw)
            res = X.score(X.run(frames, att, hgt, pos))
            print(f"    attitude {name:42s} {X.fmt(res)}", flush=True)


class View(X.Series):
    def __init__(self, rows):
        super().__init__(1e9)
        self.rows = rows


def part_c():
    print("C. online: Hold.process called when the frame arrives (t0 + capture latency), attitude only as it arrives")
    h0 = 100
    g = X.Ground("ottawa2022", h0)
    pos, att_fn, h_fn = X.hover(h0)
    T = 10.0
    frames = X.frames_for(pos, att_fn, h_fn, g, T=T, noise=5.0)
    rng = np.random.default_rng(5)
    for hz in (100, 46, 14):
        att_full, hgt = X.fc_feeds(att_fn, h_fn, T, att_hz=hz)
        rows = att_full.rows
        arr = np.empty(len(rows))
        last = -1e9
        for i, r in enumerate(rows):                       # UART is FIFO: arrivals keep their order
            last = max(last, r[0] + 0.002 + rng.uniform(0, 0.020))
            arr[i] = last
        for lat in (0.020, 0.035, 0.060):
            hold = X.Hold(X.K0, X.D0, X.R_BC_NADIR, S.READOUT)
            hold.reanchor(pos(np.array([0.0]))[0])
            whys, sent = {}, 0
            for t0, img in frames:
                n = int(np.searchsorted(arr, t0 + lat, side="right"))
                e = hold.process(img, t0 + S.FRAME_T_ERR, View(rows[:n]), hgt)
                if e.state == "lost":
                    hold.reanchor(pos(np.array([e.t]))[0])
                if e.var is not None:
                    sent += 1
                if e.why:
                    whys[e.why] = whys.get(e.why, 0) + 1
            print(f"   attitude {hz:5.1f} Hz, capture latency {lat * 1e3:3.0f} ms: fixes sent {sent / len(frames):4.0%}; "
                  f"failures {dict(sorted(whys.items(), key=lambda kv: -kv[1])[:2])}", flush=True)


if __name__ == "__main__":
    r_def, r_ulog = part_a()
    part_b(r_def, r_ulog)
    part_c()
