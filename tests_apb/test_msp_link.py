#!/usr/bin/env python3
"""msp_link without a simulator: frames, both checksums, the parser (split, damaged, error replies), the
acknowledge replies, the command budget, the flow arithmetic, the position lock (mode v2) and the GPS builders.

The flow and lock checks use fw_speed() and fw_drift() below: a small model of the Betaflight chain that this
test writes from the firmware source (betasim/FLOW_CHAIN.txt). It is our model and can be wrong; the SITL
runs in betasim/ test the real firmware.
Run: ~/.venvs/vio/bin/python tests_apb/test_msp_link.py      (about 2 s)
"""
import math
import os
import struct
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import msp_link as M      # noqa: E402

fails = 0


def check(ok, text):
    global fails
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} {text}", flush=True)


class FakePort:
    def __init__(self):
        self.out, self.inbox = [], b""

    def write(self, b):
        self.out.append(bytes(b))

    def read(self):
        b, self.inbox = self.inbox, b""
        if not b:
            time.sleep(0.005)
        return b

    def close(self):
        pass


def reply_v2(cmd, payload=b""):
    body = struct.pack("<BHH", 0, cmd, len(payload)) + payload
    return b"$X>" + body + bytes([M.crc8_dvb_s2(body)])


def reply_v1(cmd, payload=b"", d=b">"):
    body = bytes([len(payload), cmd]) + payload
    x = 0
    for b in body:
        x ^= b
    return b"$M" + d + body + bytes([x])


def fw_speed(mx, my, dt, h_rf, roll, pitch, yaw, gx, gy):
    """Our model of the firmware: one flow message -> (east, north) m/s. rangefinder_lidarmt.c 179-180,
    opticalflow.c 193-211 (rotation 0, flip off), rangefinder.c 360, position_estimator.c 544-558."""
    rx, ry = mx / 200 / dt, my / 200 / dt
    px, py = rx - math.radians(gx), ry - math.radians(-gy)
    h = h_rf * math.cos(roll) * math.cos(pitch)
    right, fwd = px * h * math.cos(roll), py * h * math.cos(pitch)
    return fwd * math.sin(yaw) - right * math.cos(yaw), fwd * math.cos(yaw) + right * math.sin(yaw)


print("A. frames and checksums")
check(M.crc8_dvb_s2(b"123456789") == 0xBC, "CRC-8 DVB-S2 check value of '123456789' is 0xBC")
check(M.frame_v1(2) == b"$M<\x00\x02\x02", "v1 request MSP_FC_VARIANT: $M< 0 2, XOR 2")
f = M.frame_v1(230, bytes(M.POLL))
check(f[3] == 4 and f[4] == 230 and f[5:9] == bytes([108, 102, 109, 254]) and f[9] == 4 ^ 230 ^ 108 ^ 102 ^ 109 ^ 254,
      "v1 MSP_MULTIPLE_MSP: size, command, payload, XOR")
f = M.frame_v2(M.MSP2_FLOW, M.flow_payload(-5, 7, 200))
check(f[:3] == b"$X<" and f[3] == 0 and struct.unpack_from("<HH", f, 4) == (0x1F02, 9) and len(f) == 18
      and f[-1] == M.crc8_dvb_s2(f[3:-1]), "v2 flow frame: 18 bytes, flags 0, cmd 0x1F02, size 9, CRC")
check(struct.unpack("<Bii", M.flow_payload(-5, 7, 200)) == (200, -5, 7), "flow payload <Bii")
check(struct.unpack("<Bi", M.rangefinder_payload(5.4321)) == (255, 5432), "rangefinder payload <Bi, mm")
check(struct.unpack("<Bi", M.rangefinder_payload(None))[1] < 0, "rangefinder out of range: negative")
check(len(M.frame_v2(M.MSP2_RANGEFINDER, M.rangefinder_payload(3))) == 14, "v2 rangefinder frame: 14 bytes")

print("B. parser")
a, b, c = reply_v1(108, struct.pack("<hhh", 12, -34, 270)), reply_v2(0x1F02), reply_v1(150, b"", b"!")
stream = b"\x00garbage$" + a + b + c
got = []
buf = b""
for i in range(len(stream)):                      # one byte at a time
    fr, buf = M.parse(buf + stream[i:i + 1])
    got += fr
check([g[0] for g in got] == [108, 0x1F02, 150] and got[0][1] == struct.pack("<hhh", 12, -34, 270),
      "split input, byte by byte: three frames, garbage dropped")
check(got[1][1] == b"" and got[1][2] and not got[2][2], "empty v2 acknowledge; '!' frame marked as error")
bad = bytearray(a)
bad[6] ^= 0x40
fr, rest = M.parse(bytes(bad) + b)
check([g[0] for g in fr] == [0x1F02] and rest == b"", "v1 frame with a damaged byte dropped, next frame kept")
bad = bytearray(b)
bad[-1] ^= 1
fr, rest = M.parse(bytes(bad) + a)
check([g[0] for g in fr] == [108], "v2 frame with a bad CRC dropped, next frame kept")
fr, rest = M.parse(a[:7])
check(fr == [] and rest == a[:7], "incomplete frame kept for the next read")
fr, rest = M.parse(b"$X>" + struct.pack("<BHH", 0, 1, 60000) + a)
check([g[0] for g in fr] == [108], "v2 header with an impossible size skipped")

print("C. link: acknowledges, poll decode, budget")
port = FakePort()
s = {"msp_device": "fake", "fc_baud": 115200, "att_hz": 40, "msp_lock_gain": 0.0, "msp_gyro_lsb_per_dps": 16.4}
fc = M.MspLink(s, port)
fc.boxids = [0, 1, 2, 3, 11]
multi = b""
for p in (struct.pack("<hhh", 50, -100, 90), struct.pack("<9h", 0, 0, 512, 164, -328, 0, 0, 0, 0),
          struct.pack("<ih", 512, -25), struct.pack("<8h", 0, 1009, 2100, 512, 0, 0, 0, 0)):
    multi += bytes([len(p)]) + p
st = struct.pack("<HHHI", 0, 0, 0x51, 0b10001) + bytes(10)
port.inbox = reply_v1(230, multi) + reply_v2(0x1F01) + reply_v2(0x1F02) + reply_v2(0x1F02) + reply_v1(150, st)
time.sleep(0.2)
r = fc.att.rows[-1] if fc.att.rows else (0, 0, 0, 0)
check(abs(r[1] - math.radians(5)) < 1e-9 and abs(r[2] - math.radians(10)) < 1e-9 and abs(r[3] - math.radians(90)) < 1e-9,
      "MULTIPLE_MSP: attitude roll 5.0, pitch +10.0 (nose up, sign changed), yaw 90 deg")
check(fc.gyro.rows and abs(fc.gyro.rows[-1][1] - 10) < 1e-9 and abs(fc.gyro.rows[-1][2] + 20) < 1e-9,
      "MULTIPLE_MSP: gyro 164 and -328 counts -> 10 and -20 deg/s at 16.4 counts per deg/s")
check(fc.hgt.rows and fc.hgt.rows[-1][1] == 0 and fc.vz.rows[-1][1] == 0.25,
      "disarmed: barometer height zeroed (0 m); vz +0.25 m/s down from MSP_ALTITUDE")
port.inbox = reply_v1(254, struct.pack("<8h", 0, 1009, 2100, 1012, 0, 0, 0, 0))
time.sleep(0.05)
check(fc.hgt.rows[-1][1] == 5.0, "armed: height = BARO debug value 3 minus its value at arming (1012 - 512 cm = 5.0 m)")
n0 = len(fc.hgt.rows)
port.inbox = reply_v1(254, struct.pack("<8h", 1, 2, 3, 400, 0, 0, 0, 0))
time.sleep(0.05)
check(len(fc.hgt.rows) == n0, "debug values of another debug_mode (value 1 not a pressure): no height")
check(fc.acks == {0x1F01: 1, 0x1F02: 2}, f"acknowledges counted, not decoded: {fc.acks}")
check(fc.armed and fc.mode == frozenset({0, 11}) and fc.sensors == 0x51, "STATUS_EX: armed, POS HOLD active, sensors")
bud = M.Budget(95)
n = sum(bud.take(1, 10.0 + i * 0.001) for i in range(300))
check(n == 95 and bud.take(1, 11.0005) and not bud.take(1, 11.0006), f"budget: {n} of 300 in 0.3 s, frees after 1 s")

print("D. flow arithmetic against our firmware model")
worst = 0.0
rng = np.random.default_rng(1)
for _ in range(200):
    vn, ve, yaw, h = rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-math.pi, math.pi), rng.uniform(1, 10)
    roll, pitch, gx, gy, dt = rng.uniform(-.3, .3), rng.uniform(-.3, .3), rng.uniform(-50, 50), rng.uniform(-50, 50), \
        rng.uniform(.03, .1)
    mx, my = M.flow_motion(vn, ve, yaw, h, roll, pitch, gx, gy, dt)
    e, n = fw_speed(mx, my, dt, h / (math.cos(roll) * math.cos(pitch)), roll, pitch, yaw, gx, gy)
    worst = max(worst, abs(n - vn), abs(e - ve))
check(worst < 1e-9, f"200 random cases: speed back from the model, worst error {worst:.1e} m/s")
mx, my = M.flow_motion(1.0, 0.0, 0.0, 5.0, 0, 0, 0, 0, 0.05)
check(mx == 0 and abs(my - 2.0) < 1e-12, "1 m/s north, facing north, 5 m, 50 ms: motion (0, 2.0)")
mx, my = M.flow_motion(0.0, 1.0, 0.0, 5.0, 0, 0, 0, 0, 0.05)
check(abs(mx + 2.0) < 1e-12 and abs(my) < 1e-12, "1 m/s east, facing north: motion x -2.0 (X axis points left)")

print("E. link sends: order, gates, gyro lag")
port.out.clear()
check(fc.send_speed(0, 1.0, 0.0, 0.0) is None and not port.out, "no fix: nothing sent")
fc.fix = (0.0, np.zeros(2), 5.0)
fc.gyro.add(time.monotonic(), 30.0, 0.0, 0.0)
sent = [fc.send_speed(0, 0.0, 0.0, 0.0) for _ in range(8) if time.sleep(0.02) is None]
frames, _ = M.parse(b"".join(port.out).replace(b"$X<", b"$X>"))
check([f[0] for f in frames[:2]] == [0x1F01, 0x1F02] and len(frames) == 16, "each send: rangefinder then flow")
check(all(abs(m[0]) < 1e-9 for m in sent[:6]) and all(m[0] > 0 for m in sent[6:]),
      "gyro term starts with the 7th message (the FC subtracts the sample of 6 messages earlier)")
fc.close()

print("F. position lock (mode v2) with our model: constant speed error 0.10 m/s, 60 s at 20 Hz, perfect FC hold")


def fw_drift(gain, bias=0.10, T=60.0, hz=20.0, noise=0.0, seed=0):
    """Drone true position error after T s. Our model of the firmware: its position integrates the speed we send
    (each speed held until the next message); its controller holds that position perfectly. Our fix: true
    position + noise (keyframe: no drift); our speed: true speed + bias."""
    rng = np.random.default_rng(seed)
    lock, p_true, p_fc, dt = M.Lock(gain), np.zeros(2), np.zeros(2), 1 / hz
    worst = 0.0
    for k in range(int(T * hz)):
        v_true = np.zeros(2)
        v = lock.step(k * dt, p_true + rng.normal(0, noise, 2), v_true + [bias, 0.0])
        p_fc = p_fc + v * dt
        p_true = -p_fc                       # the controller moves the drone so the FC's estimate stays at 0
        worst = max(worst, float(np.hypot(*p_true)))
    return float(np.hypot(*p_true)), worst


d1, _ = fw_drift(0.0)
d2, w2 = fw_drift(0.2)
d3, w3 = fw_drift(0.2, noise=0.3, seed=2)
check(abs(d1 - 6.0) < 0.01, f"v1 (gain 0): drift {d1:.2f} m after 60 s (0.10 m/s x 60 s)")
check(d2 < 0.26 and w2 < 0.26, f"v2 (gain 0.2/s): error {d2:.2f} m at 60 s, worst {w2:.2f} m (model bound bias/(2 gain) = 0.25 m)")
check(d3 < 1.0, f"v2 with 0.3 m fix noise (seed 2): error {d3:.2f} m at 60 s, worst {w3:.2f} m")
lk = M.Lock(1.0)
lk.step(0.0, (0, 0), (0, 0))
v = lk.step(0.05, (50.0, 0), (0, 0))
check(abs(np.hypot(*v) - M.MAX_LOCK_MPS) < 1e-9, "a 50 m jump of the fix adds at most MAX_LOCK_MPS")
v = lk.step(5.0, (60.0, 0), (0.3, 0))
check(np.allclose(v, (0.3, 0)), "after a gap the lock integral restarts at the fix: speed passes unchanged")

print("G. GPS route builders (second route; not sent by MspLink)")
p = M.gps2_payload(0.0001, -0.0002, 12.5, 1.0, -2.0, 0.5)
f = struct.unpack("<BHIBBHHHHiiiiiiHHHBBBBB", p)
check(len(p) == 52 and f[3] == 3 and f[4] == 12 and f[8] == 120 and f[9] == -2000 and f[10] == 1000 and f[11] == 1250
      and f[12:15] == (100, -200, 50), "0x1F03: 52 bytes; fix, sats, pdop, lon, lat, alt cm, vel cm/s in order")
p = M.raw_gps_payload(45.0, -75.5, 80, 1.5)
check(len(p) == 14 and struct.unpack("<BBiiHH", p) == (1, 12, 450000000, -755000000, 80, 150),
      "MSP_SET_RAW_GPS: 14 bytes, lat then lon, alt m, speed cm/s")

print(f"{'ALL PASSED' if not fails else f'{fails} FAILED'}")
sys.exit(1 if fails else 0)
