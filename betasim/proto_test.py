#!/usr/bin/env python3
"""Protocol, sign and scale proof against the Betaflight SITL. No flight: the SITL is not armed.

Run 1 (debug_mode OPTICALFLOW): identify; sensor bits; rangefinder + flow at 20 Hz for 10 s in four 2.5 s
legs (speed north, east, south, west at 1 m/s, 5 m, attitude 0); MSP_DEBUG values 0 and 1 (the flow rate the
FC computed, rad/s x 1000) and 6, 7 (after its low-pass filter); then the command rate: 300 MSP_ATTITUDE
requests at once, count the replies.
Run 2 (debug_mode RANGEFINDER): 10 s of messages; MSP_DEBUG 1 and 2 (raw and tilt-corrected height, cm).
Run 3: the flow messages stop; the debug values freeze and the FC's flow stops counting (100 ms).
The estimator's speed and position need the SITL armed: see closed_loop.py.
Out: betasim/results/proto.txt and proto.json.
Run: BF_SITL=<binary> ~/.venvs/vio/bin/python betasim/proto_test.py
"""
import json
import math
import os
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import msp_link as M      # noqa: E402
import sitl               # noqa: E402

OUT = os.path.join(HERE, "results")
WORK = os.environ.get("BF_WORK", "/tmp/betasim_work")
BASE = ["feature RANGEFINDER", "feature OPTICALFLOW", "set rangefinder_hardware = MTF02P",
        "set opticalflow_hardware = MT", "set rangefinder_max_range_cm = 1000", "set poshold_opticalflow_max_range = 1000"]
LEGS = (("north", 1.0, 0.0), ("east", 0.0, 1.0), ("south", -1.0, 0.0), ("west", 0.0, -1.0))
H = 5.0
lines, res = [], {}


def say(t):
    print(t, flush=True)
    lines.append(t)


def debug(fc):
    p = fc.request(M.MSP_DEBUG)
    return list(struct.unpack(f"<{len(p) // 2}h", p)) if p else None


def stream(fc, seconds, legs, poll):
    """Rangefinder + flow at 20 Hz; legs: [(name, vn, ve)] split evenly; poll() each 0.1 s -> rows."""
    rows, t0, last, k = [], time.monotonic(), None, 0
    while (now := time.monotonic()) - t0 < seconds:
        name, vn, ve = legs[min(int((now - t0) / seconds * len(legs)), len(legs) - 1)]
        dt = 0.05 if last is None else now - last
        mx, my = M.flow_motion(vn, ve, 0.0, H, 0.0, 0.0, 0.0, 0.0, dt)
        fc._send(M.frame_v2(M.MSP2_RANGEFINDER, M.rangefinder_payload(H)) + M.frame_v2(M.MSP2_FLOW, M.flow_payload(mx, my)))
        last, k = now, k + 1
        if k % 2 == 0:
            rows.append((round(now - t0, 3), name, poll()))
        time.sleep(max(0.0, 0.05 - (time.monotonic() - now)))
    return rows, k


def run(tag, extra):
    wd = os.path.join(WORK, tag)
    sitl.provision(wd, BASE + extra)
    p = sitl.start(wd)
    try:
        fc = M.MspLink({"msp_device": f"tcp:127.0.0.1:{sitl.MSP_PORT}", "fc_baud": 115200, "att_hz": 40})
        fw = fc.identify()
        return p, fc, fw
    except Exception:
        sitl.stop(p)
        raise


def sensor_bits(fc):
    st = fc.request(M.MSP_STATUS_EX)
    return struct.unpack_from("<H", st, 4)[0] if st else None


# ---------------- run 1: flow scale and sign
p, fc, fw = run("proto1", ["set debug_mode = OPTICALFLOW"])
try:
    say(f"[1] identify: firmware {fw}, {fc.info}; boxids include POS HOLD (11): {M.BOX_POSHOLD in fc.boxids}")
    b = sensor_bits(fc)
    say(f"[1] MSP_STATUS_EX sensor bits 0x{b:04x}: rangefinder (bit 4) {bool(b & 16)}, optical flow (bit 6) {bool(b & 64)}")
    res["identify"], res["sensor_bits"] = dict(fc.info, firmware=fw), b
    rows, n = stream(fc, 10.0, LEGS, lambda: debug(fc))
    acks = dict(fc.acks)
    say(f"[1] sent {n} rangefinder + {n} flow messages in 10 s; acknowledges: rangefinder {acks.get(0x1F01, 0)}, "
        f"flow {acks.get(0x1F02, 0)}; error replies {fc.errors}")
    exp = {name: (-ve / H * 1000, vn / H * 1000) for name, vn, ve in LEGS}   # X points left (msp_link.flow_motion)
    res["flow_rows"], summ = rows, {}
    for name, _, _ in LEGS:
        d = [r[2] for r in rows if r[1] == name and r[2]][2:]           # skip the first 0.2 s of each leg
        if not d:
            continue
        m0 = sum(x[0] for x in d) / len(d)
        m1 = sum(x[1] for x in d) / len(d)
        m6 = sum(x[6] for x in d) / len(d)
        m7 = sum(x[7] for x in d) / len(d)
        sd0 = (sum((x[0] - m0) ** 2 for x in d) / len(d)) ** .5
        sd1 = (sum((x[1] - m1) ** 2 for x in d) / len(d)) ** .5
        e = exp[name]
        summ[name] = dict(n=len(d), expect=e, debug01_mean=(m0, m1), debug01_sd=(sd0, sd1), debug67_last=d[-1][6:8],
                          debug67_mean=(m6, m7))
        say(f"[1] leg {name:5s}: expect flow (x, y) = ({e[0]:+.0f}, {e[1]:+.0f}) mrad/s; FC debug 0,1 mean "
            f"({m0:+.1f}, {m1:+.1f}) sd ({sd0:.1f}, {sd1:.1f}), n {len(d)}; after its low-pass (debug 6,7) last "
            f"({d[-1][6]:+d}, {d[-1][7]:+d})")
    res["flow_summary"] = summ
    # command rate: 300 requests without waiting
    fc.replies.clear()
    cnt0 = len(fc.arrivals.get("ATTITUDE", []))
    t0 = time.monotonic()
    fc._send(M.frame_v1(M.MSP_ATTITUDE) * 300)
    time.sleep(3.0)
    arr = [t for t in fc.arrivals.get("ATTITUDE", []) if t >= t0]
    span = arr[-1] - arr[0] if len(arr) > 1 else float("nan")
    say(f"[1] command rate: 300 MSP_ATTITUDE requests sent at once; {len(arr)} replies in 3 s; first to last "
        f"{span:.2f} s = {(len(arr) - 1) / span if len(arr) > 1 else float('nan'):.0f} replies/s")
    res["cmd_rate"] = dict(sent=300, replies=len(arr), span_s=span)
    fc.close()
finally:
    sitl.stop(p)

# ---------------- run 2: rangefinder values
p, fc, fw = run("proto2", ["set debug_mode = RANGEFINDER"])
try:
    rows, n = stream(fc, 10.0, LEGS, lambda: debug(fc))
    d = [r[2] for r in rows if r[2]][5:]
    say(f"[2] rangefinder {H * 1000:.0f} mm sent {n} times in 10 s: FC debug 1 (raw cm) last {d[-1][1]}, "
        f"debug 2 (tilt-corrected cm) last {d[-1][2]}; min/max of debug 2 over {len(d)} reads: "
        f"{min(x[2] for x in d)}/{max(x[2] for x in d)}")
    res["range"] = dict(sent_mm=H * 1000, debug=[x[:6] for x in d])
    fc.close()
finally:
    sitl.stop(p)

# ---------------- run 3: stop
p, fc, fw = run("proto3", ["set debug_mode = OPTICALFLOW"])
try:
    stream(fc, 3.0, (("north", 1.0, 0.0),), lambda: None)
    before = debug(fc)
    time.sleep(1.0)
    after = [debug(fc) for _ in range(5)]
    same = all(a[:2] == before[:2] for a in after)
    say(f"[3] flow stopped after 3 s north: debug 0,1 before {before[:2]}, 1 s later {after[-1][:2]}; unchanged: {same}. "
        "The FC keeps the last rate but stops using it after 100 ms (opticalflow.c isOpticalflowHealthy); "
        "this is not readable over MSP while disarmed.")
    res["stop"] = dict(before=before, after=after)
    fc.close()
finally:
    sitl.stop(p)

os.makedirs(OUT, exist_ok=True)
with open(os.path.join(OUT, "proto.txt"), "w") as f:
    f.write(f"betasim/proto_test.py, {time.strftime('%Y-%m-%d %H:%M')}, binary {sitl.BIN}\n" + "\n".join(lines) + "\n")
with open(os.path.join(OUT, "proto.json"), "w") as f:
    json.dump(res, f, indent=1)
