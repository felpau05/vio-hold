#!/usr/bin/env python3
"""Item 8: the PX4 v1.15.4 facts behind the flight-controller findings, read from the local clone.

The APB port (DroneWuKong/Orqa_H7_PX4_Ports) targets PX4 v1.15.4. This prints the source lines that
decide whether our external-vision (EV) fix is accepted. Needs the clone in ../../px4sim/PX4-Autopilot
(git show of tag v1.15.4; the working tree is not used).

Run: ~/.venvs/vio/bin/python t11_px4_facts.py     (a few seconds)
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "px4sim", "PX4-Autopilot"))
TAG = "v1.15.4"


def show(path):
    return subprocess.run(["git", "-C", REPO, "show", f"{TAG}:{path}"], capture_output=True, text=True,
                          check=True).stdout


def param_default(yaml_text, name):
    m = re.search(rf"^\s+{name}:\n(.*?)(?=^\s+[A-Z0-9_]+:\n|\Z)", yaml_text, re.S | re.M)
    if not m:
        return "not found"
    block = m.group(1)
    short = re.search(r"short:\s*(.*)", block)
    dflt = re.search(r"default:\s*(.*)", block)
    return f"default {dflt.group(1).strip() if dflt else '?'}  ({short.group(1).strip() if short else ''})"


def grep(text, pattern, ctx=0, label=""):
    lines = text.splitlines()
    for i, ln in enumerate(lines):
        if re.search(pattern, ln):
            for j in range(max(0, i - ctx), min(len(lines), i + ctx + 1)):
                print(f"    {label}:{j + 1}: {lines[j].rstrip()}")
            return
    print(f"    {label}: pattern {pattern!r} not found")


def main():
    if not os.path.isdir(os.path.join(REPO, ".git")):
        sys.exit(f"PX4 clone not found at {REPO}")
    y = show("src/modules/ekf2/module.yaml")
    print(f"EKF2 parameter defaults in PX4 {TAG}:")
    for p in ("EKF2_EV_CTRL", "EKF2_EV_DELAY", "EKF2_EVP_NOISE", "EKF2_EV_NOISE_MD", "EKF2_EVP_GATE",
              "EKF2_HGT_REF", "EKF2_GPS_CTRL", "EKF2_NOAID_TOUT", "EKF2_DELAY_MAX"):
        print(f"  {p:18s} {param_default(y, p)}")
    print("VISION_POSITION_ESTIMATE handler (frame and time stamp):")
    r = show("src/modules/mavlink/mavlink_receiver.cpp")
    grep(r, r"odom.timestamp_sample = _mavlink_timesync.sync_stamp\(vpe.usec\)", 8, "mavlink_receiver.cpp")
    grep(r, r"odom.position_variance\[0\] = vpe.covariance\[0\]", 2, "mavlink_receiver.cpp")
    print("Timesync: arrival time until converged; converged = 500 exchanges with RTT < 10 ms:")
    ts = show("src/lib/timesync/Timesync.cpp")
    grep(ts, r"return hrt_absolute_time\(\);", 3, "Timesync.cpp")
    th = show("src/lib/timesync/Timesync.hpp")
    grep(th, r"CONVERGENCE_WINDOW = ", 0, "Timesync.hpp")
    grep(th, r"MAX_RTT_SAMPLE = ", 0, "Timesync.hpp")
    print("EV position in a NED frame is used only after yaw alignment; with GNSS on it becomes a biased aid:")
    ep = show("src/modules/ekf2/EKF/ev_pos_control.cpp")
    grep(ep, r"case PositionFrame::LOCAL_FRAME_NED:", 15, "ev_pos_control.cpp")
    grep(ep, r"_nb_ev_pos_reset_available = 5;", 0, "ev_pos_control.cpp")
    print("EV yaw (default on) resets the EKF heading to the yaw in our message:")
    ey = show("src/modules/ekf2/EKF/ev_yaw_control.cpp")
    grep(ey, r"reset yaw to EV and set yaw_align", 2, "ev_yaw_control.cpp")
    print("EV samples farther apart than 200 ms do not start fusion:")
    c = show("src/modules/ekf2/EKF/common.h")
    grep(c, r"EV_MAX_INTERVAL\s*=", 0, "common.h")
    ec = show("src/modules/ekf2/EKF/ev_control.cpp")
    grep(ec, r"ev_sample.time_us - _ev_sample_prev.time_us\) < EV_MAX_INTERVAL", 0, "ev_control.cpp")
    print("Hold (AUTO_LOITER) needs a global position; Position mode does not:")
    m = show("src/modules/commander/ModeUtil/mode_requirements.cpp")
    grep(m, r"NAVIGATION_STATE_AUTO_LOITER, flags.mode_req_global_position", 0, "mode_requirements.cpp")
    print("Onboard-mode ATTITUDE request and bandwidth scaling, ULog share:")
    mm = show("src/modules/mavlink/mavlink_main.cpp")
    grep(mm, r'configure_stream_local\("ATTITUDE", 100.0f\);', 0, "mavlink_main.cpp")
    grep(mm, r'configure_stream_local\("ODOMETRY", 30.0f\);', 0, "mavlink_main.cpp")
    grep(mm, r"_datarate = _baudrate / 20;", 0, "mavlink_main.cpp")
    grep(mm, r"float bandwidth_mult = ", 0, "mavlink_main.cpp")
    mh = show("src/modules/mavlink/mavlink_main.h")
    grep(mh, r"MavlinkULog::try_start\(_datarate, 0.7f", 0, "mavlink_main.h")
    att = show("src/modules/mavlink/streams/ATTITUDE.hpp")
    grep(att, r"msg.time_boot_ms = att.timestamp / 1000;", 0, "ATTITUDE.hpp")


if __name__ == "__main__":
    main()
