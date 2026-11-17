#!/usr/bin/env python3
"""fc_link against real ArduCopter 4.6 firmware (SITL in container ap_sim, apsim/run_sitl.sh), not a fake:
A. firmware detected; height from GLOBAL_POSITION_INT before the EKF has a position (no LOCAL_POSITION_NED yet);
   parameter check reads the real parameter names (all PASS with params_ardupilot loaded);
   origin + MAV_CMD_DO_SET_HOME accepted; the companion HEARTBEAT reaches the GCS link (routing).
B. baud scan order on a serial device (stub FC: the scan logic only).
Skips A when Docker or the SITL build is missing. Run: ~/.venvs/vio/bin/python tests_apb/test_ap_sitl.py (40 s)
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path.insert(0, PROJ)
os.environ.setdefault("MAVLINK20", "1")
import fc_link            # noqa: E402
import settings as S      # noqa: E402
from pymavlink import mavutil   # noqa: E402

APSIM = os.path.join(PROJ, "apsim")
I, G, O = 7, 15140, 15141       # SITL instance and ports for this test only
fails = 0


def check(ok, text):
    global fails
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} {text}", flush=True)


def part_a():
    print("A. ArduCopter SITL (real firmware)")
    if not os.path.exists(os.path.join(APSIM, "ardupilot/build/sitl/bin/arducopter")):
        print("  SKIP: no SITL build (see apsim/run_sitl.sh)")
        return
    s = dict(S.load("/nonexistent"), fc_device=f"udpin:127.0.0.1:{O}", msp_device="/nonexistent")
    os.makedirs(os.path.join(APSIM, "runs"), exist_ok=True)
    parm = os.path.join(APSIM, "runs", "test_ap_sitl.parm")
    open(parm, "w").write("".join(f"{k} {v}\n" for k, v in s["params_ardupilot"].items()))
    sh = os.path.join(APSIM, "run_sitl.sh")
    subprocess.run(["sh", sh, "stop", str(I)], capture_output=True)
    subprocess.run(["sh", sh, "start", str(I), str(G), str(O), "1", "runs/test_ap_sitl.parm", "runs/test_ap_sitl.log"],
                   check=True, capture_output=True)
    gcs = mavutil.mavlink_connection(f"udpin:0.0.0.0:{G}", source_system=255)
    fc = None
    try:
        gcs.wait_heartbeat(timeout=30)
        gcs.mav.heartbeat_send(6, 8, 0, 0, 0)
        fc, why = fc_link.connect(s, timeout=20)
        check(fc is not None and fc.firmware == "ardupilot", f"firmware detected: {fc and fc.firmware} {why or ''}")
        if fc is None:
            return
        fc.setup_streams()
        time.sleep(3.0)
        check(len(fc.hgt.rows) > 10 and fc.rate("LOCAL_POSITION_NED") == 0,
              f"height before any position: {len(fc.hgt.rows)} rows from GLOBAL_POSITION_INT, "
              f"LOCAL_POSITION_NED {fc.rate('LOCAL_POSITION_NED'):.0f} Hz")
        lines = fc_link.check_params(fc, s)
        check(all(ok for ok, _ in lines) and len(lines) == len(s["params_ardupilot"]),
              "parameter check on the real names: " + "; ".join(l for ok, l in lines if not ok)[:300])
        fc.acks.clear()
        fc.send_origin(45.417, -75.715, 70.0)
        time.sleep(2.0)
        check(fc.acks.get(179) == 0, f"MAV_CMD_DO_SET_HOME after the origin: ack {fc.acks.get(179)} (0 = accepted)")
        check(any("origin set" in t for t in fc.texts), f"EKF origin set: {[t for t in fc.texts if 'origin' in t]}")
        seen, end = False, time.time() + 4
        while time.time() < end and not seen:
            m = gcs.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            seen = m is not None and m.get_srcComponent() == 197 and m.type == 18
        check(seen, "companion HEARTBEAT (ONBOARD_CONTROLLER, component 197) routed to the GCS link")
    finally:
        if fc:
            fc.close()
        gcs.close()
        subprocess.run(["sh", sh, "stop", str(I)], capture_output=True)


def part_b():
    print("B. baud scan (stub FC: answers only at 57600)")
    tried = []

    class Stub:
        def __init__(self, s):
            tried.append(s["fc_baud"])
            self.ok = s["fc_baud"] == 57600

        def wait_heartbeat(self, timeout):
            return "ardupilot" if self.ok else None

        def close(self):
            pass
    real = fc_link.FC
    fc_link.FC = Stub
    try:
        s = dict(S.load("/nonexistent"), fc_device="/dev/ttymxc2", msp_device="/nonexistent")
        fc, _ = fc_link.connect(s, timeout=0.1)
        check(fc is not None and fc.baud == 57600 and tried == [230400, 115200, 57600],
              f"tried {tried}, connected at {fc and fc.baud}")
        tried.clear()
        fc, why = fc_link.connect(dict(s, fc_device="udpin:127.0.0.1:1"), timeout=0.1)
        check(tried == [230400], f"UDP device: one try only ({tried})")
    finally:
        fc_link.FC = real


if __name__ == "__main__":
    part_b()
    part_a()
    print("ALL PASS" if fails == 0 else f"{fails} FAIL")
    sys.exit(1 if fails else 0)
