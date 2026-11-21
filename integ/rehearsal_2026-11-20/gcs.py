#!/usr/bin/env python3
"""The pilot's mode switch, for the rehearsal: ask ArduCopter SITL (sitl_ap.sh, SERIAL0 on udp 14560) for a flight
mode, as a switch position does, and print what the FC reports.

  ~/.venvs/vio/bin/python integ/rehearsal_2026-11-20/gcs.py MODE [MODE ...]     e.g. ACRO ALT_HOLD LOITER RTL
"""
import os
import sys
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402


def main():
    m = mavutil.mavlink_connection("udpin:127.0.0.1:14560")
    m.wait_heartbeat(timeout=10)
    for name in sys.argv[1:]:
        m.set_mode(m.mode_mapping()[name])
        end, texts, mode = time.time() + 2.0, [], None
        while time.time() < end:
            msg = m.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                texts.append(msg.text)
            elif msg.get_srcComponent() == 1:
                mode = m.flightmode
        print(f"asked {name}: FC reports {mode}; FC texts: {texts or 'none'}", flush=True)


if __name__ == "__main__":
    main()
