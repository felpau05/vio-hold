"""MSP (MultiWii Serial Protocol) link to a Betaflight flight controller: the second link beside fc_link.py.

Betaflight has no external-vision input. The 2026.6 branch has Position Hold from an optical flow sensor and
a rangefinder that send MSP messages (research/betaflight_route.md). This link sends our speed as the
message that such a sensor sends. betasim/FLOW_CHAIN.txt gives the firmware lines that each step mirrors.

- Reads, with the host receive time, in one MSP_MULTIPLE_MSP request at poll_hz: MSP_ATTITUDE (roll and pitch
  0.1 deg, yaw 1 deg), MSP_RAW_IMU (gyro, for the rotation part of the flow), MSP_ALTITUDE (vertical speed)
  and MSP_DEBUG with debug_mode = BARO (value 3: the barometer height, cm), zeroed at the last reading
  before arming. That barometer height is Hold's height and the rangefinder message. Not the FC's fused height (MSP_ALTITUDE): our
  rangefinder message goes into that estimate, and sending it back closes a loop; in the SITL the FC height
  then lagged the true height by 5 m and ALT HOLD climbed (betasim/results/closed_try1.json).
  MSP_STATUS_EX (armed, flight mode, sensor bits) at 5 Hz.
- Sends, for each good fix with a speed (apb_main calls send_fix, then send_speed): MSP2 0x1F01 rangefinder,
  then MSP2 0x1F02 optical flow. No fix: nothing is sent. Betaflight then stops using the flow after 100 ms,
  ends the hold 2 s later and gives the sticks back to the pilot.
- Mode v2 "position lock" (setting msp_lock_gain > 0): Betaflight integrates the flow we send, so its position
  can drift from our keyframe position. We keep the integral of what we sent and add
  gain * (our position - that integral) to the speed we send. 0 = mode v1 (speed only).
- Never changes the flight mode or a setting. The pilot's switch selects Position Hold.
"""
import math
import os
import socket
import struct
import threading
import time
from collections import deque
from types import SimpleNamespace

import numpy as np

from hold import Series

MSP_API_VERSION, MSP_FC_VARIANT, MSP_FC_VERSION, MSP_BUILD_INFO = 1, 2, 3, 5
MSP_RAW_IMU, MSP_ATTITUDE, MSP_ALTITUDE, MSP_BOXIDS = 102, 108, 109, 119
MSP_STATUS_EX, MSP_SET_RAW_GPS, MSP_MULTIPLE_MSP, MSP_DEBUG = 150, 201, 230, 254
MSP2_RANGEFINDER, MSP2_FLOW, MSP2_GPS = 0x1F01, 0x1F02, 0x1F03
BOX_ARM, BOX_POSHOLD = 0, 11
SENSOR_RANGEFINDER, SENSOR_FLOW = 1 << 4, 1 << 6          # MSP_STATUS_EX sensor bits
POLL = (MSP_ATTITUDE, MSP_RAW_IMU, MSP_ALTITUDE, MSP_DEBUG)   # one MSP_MULTIPLE_MSP request

FLOW_SCALE = 200.0     # rangefinder_lidarmt.c: rate (rad/s) = motion / 200 / dt
GYRO_LAG = 6           # opticalflow.c: the gyro it subtracts is the sample of 6 flow messages earlier
BUDGET = 95            # MSP commands per second: one command per serial task run, task at 100 Hz
MAX_LOCK_MPS = 0.5     # assumed: the most position lock adds to the speed; a bad fix must not push hard
GAP_S = 0.5            # a longer gap between flow messages restarts the lock integral and the gyro history


# ---------------------------------------------------------------- frames
def crc8_dvb_s2(data, crc=0):
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frame_v1(cmd, payload=b""):
    body = bytes([len(payload), cmd]) + payload
    x = 0
    for b in body:
        x ^= b
    return b"$M<" + body + bytes([x])


def frame_v2(cmd, payload=b""):
    body = struct.pack("<BHH", 0, cmd, len(payload)) + payload
    return b"$X<" + body + bytes([crc8_dvb_s2(body)])


def parse(buf):
    """Complete reply frames in buf -> ([(cmd, payload, ok)], rest). ok False: an error reply ('!').
    Bytes outside a frame and frames with a bad checksum are dropped; an incomplete frame stays in rest."""
    out, i, n = [], 0, len(buf)
    while True:
        i = buf.find(b"$", i)
        if i < 0:
            return out, b""
        if n - i < 3:
            return out, bytes(buf[i:])
        v, d = buf[i + 1], buf[i + 2]
        if v not in b"MX" or d not in b">!":
            i += 1
            continue
        if v == ord("M"):
            if n - i < 6:
                return out, bytes(buf[i:])
            cmd, start, end = buf[i + 4], i + 5, i + 5 + buf[i + 3]
            if n < end + 1:
                return out, bytes(buf[i:])
            x = 0
            for b in buf[i + 3:end]:
                x ^= b
            good = x == buf[end]
        else:
            if n - i < 9:
                return out, bytes(buf[i:])
            _, cmd, size = struct.unpack_from("<BHH", buf, i + 3)
            if size > 1024:                     # a damaged size field: do not wait for a frame that never ends
                i += 1
                continue
            start, end = i + 8, i + 8 + size
            if n < end + 1:
                return out, bytes(buf[i:])
            good = crc8_dvb_s2(buf[i + 3:end]) == buf[end]
        if not good:
            i += 1
            continue
        out.append((cmd, bytes(buf[start:end]), d == ord(">")))
        i = end + 1


# ---------------------------------------------------------------- message payloads
def rangefinder_payload(dist_m, quality=255):
    """0x1F01: u8 quality, i32 distance in mm (negative: out of range)."""
    return struct.pack("<Bi", quality, int(round(dist_m * 1000)) if dist_m is not None else -1)


def flow_payload(mx, my, quality=255):
    """0x1F02: u8 quality (the FC uses quality * 100 / 255, needs over 30), i32 motion x, i32 motion y."""
    return struct.pack("<Bii", quality, int(round(mx)), int(round(my)))


def gps2_payload(lat, lon, alt_m, vn, ve, vd, sats=12, pdop=120, fix=3):
    """0x1F03 (52 bytes), second route, not used by MspLink. lat/lon in degrees; speeds in m/s."""
    course = int(round(math.degrees(math.atan2(ve, vn)) % 360 * 100))
    return struct.pack("<BHIBBHHHHiiiiiiHHHBBBBB", 0, 0, 0, fix, sats, 100, 150, 50, pdop,
                       int(round(lon * 1e7)), int(round(lat * 1e7)), int(round(alt_m * 100)),
                       int(round(vn * 100)), int(round(ve * 100)), int(round(vd * 100)), course, 0,
                       0, 0, 0, 0, 0, 0)


def raw_gps_payload(lat, lon, alt_m, speed_mps, sats=12, fix=1):
    """MSP_SET_RAW_GPS (201, 14 bytes): u8 fix, u8 sats, i32 lat, i32 lon (1e-7 deg), u16 alt m, u16 speed cm/s."""
    return struct.pack("<BBiiHH", fix, sats, int(round(lat * 1e7)), int(round(lon * 1e7)),
                       int(round(alt_m)), int(round(speed_mps * 100)))


def flow_motion(vn, ve, yaw, h, roll, pitch, gx_dps, gy_dps, dt):
    """The motion counts of one 0x1F02 message, so that Betaflight computes speed (vn, ve) m/s.

    yaw: compass rad (clockwise from north); h: height above ground (m); roll, pitch: rad; gx_dps, gy_dps: the
    FC gyro x and y (deg/s) that the FC will subtract (its sample of GYRO_LAG messages earlier); dt: time since
    the previous message (the FC measures the same dt at arrival). The FC's chain, inverted:
    speed east  =  fwd * sin(yaw) - X * cos(yaw),  north = fwd * cos(yaw) + X * sin(yaw)
    X = flow_x * h_fc * cos(roll), fwd = flow_y * h_fc * cos(pitch); h_fc = rangefinder * cos(tilt)
    flow_x = raw_x - gyro_x, flow_y = raw_y + gyro_y (rad/s), raw = motion / 200 / dt.
    So the X axis points left: flow_x > 0 means the drone moves left (to the west when it faces north)."""
    fwd = vn * math.cos(yaw) + ve * math.sin(yaw)
    left = vn * math.sin(yaw) - ve * math.cos(yaw)
    fx = left / (h * math.cos(roll)) + math.radians(gx_dps)
    fy = fwd / (h * math.cos(pitch)) - math.radians(gy_dps)
    return fx * FLOW_SCALE * dt, fy * FLOW_SCALE * dt


class Budget:
    """At most n commands in any 1 s window."""

    def __init__(self, n=BUDGET):
        self.n, self.t = n, deque()

    def take(self, k, now):
        while self.t and self.t[0] <= now - 1.0:
            self.t.popleft()
        if len(self.t) + k > self.n:
            return False
        self.t.extend([now] * k)
        return True


class Lock:
    """Mode v2: the speed to send so that the integral of what we sent follows our absolute position."""

    def __init__(self, gain):
        self.gain, self.p, self.v, self.t = gain, None, np.zeros(2), None

    def step(self, t, p_abs, v):
        """t: send time; p_abs: our (north, east) now; v: our measured (vn, ve). Returns the (vn, ve) to send."""
        if self.gain <= 0:
            return np.asarray(v, float)
        if self.t is None or not 0 < t - self.t <= GAP_S:
            self.p = np.asarray(p_abs, float).copy()     # start, or after a gap: the FC re-anchors too
        else:
            self.p = self.p + self.v * (t - self.t)      # the FC holds each speed until the next message
        c = self.gain * (np.asarray(p_abs, float) - self.p)
        n = np.hypot(*c)
        if n > MAX_LOCK_MPS:
            c *= MAX_LOCK_MPS / n
        self.v, self.t = np.asarray(v, float) + c, t
        return self.v


# ---------------------------------------------------------------- transport
class Port:
    """'tcp:host:port' (the SITL) or a serial device at baud, raw 8N1. read() returns b'' on timeout."""

    def __init__(self, device, baud=115200):
        self.sock = self.fd = None
        if device.startswith("tcp:"):
            _, host, port = device.split(":")
            self.sock = socket.create_connection((host, int(port)), timeout=2.0)
            self.sock.settimeout(0.05)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        else:
            import termios
            import tty
            self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY)
            tty.setraw(self.fd)
            a = termios.tcgetattr(self.fd)
            a[4] = a[5] = getattr(termios, f"B{baud}")
            a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1            # read returns after 0.1 s
            termios.tcsetattr(self.fd, termios.TCSANOW, a)

    def write(self, b):
        if self.sock:
            self.sock.sendall(b)
        else:
            os.write(self.fd, b)

    def read(self):
        try:
            return self.sock.recv(4096) if self.sock else os.read(self.fd, 4096)
        except (socket.timeout, BlockingIOError):
            return b""

    def close(self):
        if self.sock:
            self.sock.close()
        elif self.fd is not None:
            os.close(self.fd)


# ---------------------------------------------------------------- the link
class MspLink:
    """The FC object of fc_link.py for Betaflight: the members apb_main uses, filled from MSP."""

    def __init__(self, s, port=None):
        self.s = s
        self.port = port or Port(s["msp_device"], s["fc_baud"])
        self.att, self.hgt, self.lpos, self.imu, self.vz = (Series(30.0) for _ in range(5))
        self.gyro = Series(30.0)                       # (t, gx, gy, gz) deg/s, the FC's own axes
        self.clock = SimpleNamespace(ready=lambda: bool(self.att.rows))
        self.firmware = self.sysid = self.mode = None
        self.armed, self.sensors, self.boxids, self.info, self.baro0 = False, 0, [], {}, 0
        self.arrivals, self.replies, self.acks, self.errors = {}, {}, {}, {}
        self.rtts, self.texts, self.reset_counter = deque(maxlen=1), deque(maxlen=20), 0
        self.budget, self.lock = Budget(), Lock(float(s.get("msp_lock_gain", 0.0)))
        self.lsb = float(s.get("msp_gyro_lsb_per_dps", 16.4))
        self.fix, self.t_flow, self.gyro_hist = None, None, deque(maxlen=GYRO_LAG + 1)
        self.sent = deque(maxlen=2000)                 # (t, vn, ve, vn_sent, ve_sent, h, mx, my): for tests and logs
        self._tx, self._got, self._stop, self._poll = threading.Lock(), threading.Condition(), False, None
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    # ---- receive
    def _read(self):
        buf = b""
        while not self._stop:
            try:
                data = self.port.read()
            except OSError:
                time.sleep(0.05)
                continue
            if not data:
                continue
            now = time.monotonic()
            frames, buf = parse(buf + data)
            for cmd, p, ok in frames:
                self._on_reply(cmd, p, ok, now)

    def _on_reply(self, cmd, p, ok, now):
        with self._got:
            if not ok:
                self.errors[cmd] = self.errors.get(cmd, 0) + 1
            elif cmd == MSP_MULTIPLE_MSP:
                i = 0
                for c in POLL:
                    if i >= len(p) or i + 1 + p[i] > len(p):
                        break
                    self._decode(c, p[i + 1:i + 1 + p[i]], now)
                    i += 1 + p[i]
            elif cmd in (MSP2_RANGEFINDER, MSP2_FLOW, MSP_SET_RAW_GPS, MSP2_GPS):
                self.acks[cmd] = self.acks.get(cmd, 0) + 1
            else:
                self._decode(cmd, p, now)
            self.replies[cmd] = (now, p, ok)
            self._got.notify_all()

    def _decode(self, cmd, p, now):
        self.arrivals.setdefault({MSP_ATTITUDE: "ATTITUDE", MSP_ALTITUDE: "ALTITUDE", MSP_STATUS_EX: "STATUS",
                                  MSP_RAW_IMU: "RAW_IMU"}.get(cmd, cmd), deque(maxlen=400)).append(now)
        if cmd == MSP_ATTITUDE and len(p) >= 6:
            r, pt, y = struct.unpack_from("<hhh", p)
            # Betaflight: pitch > 0 is nose down; MAVLink and Hold: nose up (betasim/results/signs.txt)
            self.att.add(now, math.radians(r / 10), -math.radians(pt / 10), math.radians(y))
        elif cmd == MSP_RAW_IMU and len(p) >= 12:
            g = struct.unpack_from("<hhh", p, 6)
            self.gyro.add(now, *(x / self.lsb for x in g))
        elif cmd == MSP_ALTITUDE and len(p) >= 6:
            self.vz.add(now, -struct.unpack_from("<h", p, 4)[0] / 100.0)
        elif cmd == MSP_DEBUG and len(p) >= 8:
            d = struct.unpack_from("<4h", p)
            if 800 <= d[1] <= 1100:                    # debug_mode BARO: value 1 is the pressure in hPa
                if not self.armed:                     # height above the point of arming, as PX4 and ArduPilot give it
                    self.baro0 = d[3]
                self.hgt.add(now, (d[3] - self.baro0) / 100.0)
        elif cmd == MSP_STATUS_EX and len(p) >= 10:
            self.sensors, flags = struct.unpack_from("<HI", p, 4)
            active = {b for i, b in enumerate(self.boxids[:32]) if flags >> i & 1}
            self.armed, self.mode = BOX_ARM in active, frozenset(active)
        elif cmd == MSP_BOXIDS:
            self.boxids = list(p)

    # ---- send
    def _send(self, frame):
        with self._tx:
            self.port.write(frame)

    def request(self, cmd, payload=b"", timeout=0.5, v2=False):
        """Send one request and wait for its reply payload; None on timeout or an error reply."""
        with self._got:
            self.replies.pop(cmd, None)
        if not self.budget.take(1, time.monotonic()):
            time.sleep(0.05)
        self._send((frame_v2 if v2 else frame_v1)(cmd, payload))
        end = time.monotonic() + timeout
        with self._got:
            while cmd not in self.replies and time.monotonic() < end:
                self._got.wait(end - time.monotonic())
            r = self.replies.get(cmd)
        return r[1] if r and r[2] else None

    def identify(self):
        """Firmware name from MSP_FC_VARIANT ('betaflight', 'inav', other or None) and the details in info."""
        for _ in range(3):
            v = self.request(MSP_FC_VARIANT)
            if v:
                break
        if not v:
            return None
        api, ver, build = self.request(MSP_API_VERSION), self.request(MSP_FC_VERSION), self.request(MSP_BUILD_INFO)
        self.info = {"variant": v[:4].decode("ascii", "replace"),
                     "api": f"{api[1]}.{api[2]}" if api and len(api) >= 3 else "?",
                     "version": ".".join(str(b) for b in ver[:3]) if ver else "?",
                     "build": build[:19].decode("ascii", "replace") if build else "?"}
        self.boxids = list(self.request(MSP_BOXIDS) or b"")
        self.firmware = {"BTFL": "betaflight", "INAV": "inav"}.get(self.info["variant"], self.info["variant"])
        return self.firmware

    def wait_heartbeat(self, timeout):
        return self.firmware or self.identify()

    def setup_streams(self, wait=1.0):
        """Starts the poll thread: MULTIPLE_MSP (attitude, gyro, height) at poll_hz, STATUS_EX at 5 Hz."""
        if self._poll is None:
            self._poll = threading.Thread(target=self._poll_loop, daemon=True)
            self._poll.start()
        time.sleep(wait)

    def _poll_loop(self):
        hz = min(float(self.s["att_hz"]), 40.0)
        k, nxt = 0, time.monotonic()
        while not self._stop:
            now = time.monotonic()
            if self.budget.take(1, now):
                if k % max(1, int(hz / 5)) == 0:
                    self._send(frame_v1(MSP_STATUS_EX))
                else:
                    self._send(frame_v1(MSP_MULTIPLE_MSP, bytes(POLL)))
                k += 1
            nxt = max(nxt + 1.0 / hz, now)
            time.sleep(max(0.0, nxt - time.monotonic()))

    def send_fix(self, e, att=None, hgt=None):
        """Betaflight takes no position: keep the fix for the position lock, and the height Hold used for it
        (hgt: the caller's Series, as in fc_link; apb_main bench passes its fixed height) for send_speed."""
        hgt = hgt if hgt is not None and hgt.rows else self.hgt
        self.fix = (e.t, np.asarray(e.pos[:2], float).copy(), float(hgt.at(e.t)) if hgt.rows else None)

    def send_speed(self, t, vn, ve, vd):
        """Rangefinder then flow message for our speed (vn, ve) m/s. Skips (sends nothing) without a fix, height,
        attitude or gyro, below 0.1 m, or over the command budget. Returns the (mx, my) sent or None."""
        now = time.monotonic()
        if self.fix is None or self.fix[2] is None or not self.att.rows or not self.gyro.rows:
            return None
        h = self.fix[2]
        _, roll, pitch, yaw = self.att.rows[-1][:4]
        if h < 0.1 or not self.budget.take(2, now):
            return None
        gap = self.t_flow is None or now - self.t_flow > GAP_S
        if gap:
            self.gyro_hist.clear()
        g = self.gyro.rows[-1]
        self.gyro_hist.append((g[1], g[2]))
        gx, gy = self.gyro_hist[0] if len(self.gyro_hist) > GYRO_LAG else (0.0, 0.0)   # what the FC subtracts
        v = self.lock.step(now, self.fix[1], (vn, ve))
        dt = 0.05 if self.t_flow is None else now - self.t_flow   # the FC divides by its own arrival gap
        mx, my = flow_motion(v[0], v[1], yaw, h, roll, pitch, gx, gy, dt)
        cos_tilt = math.cos(roll) * math.cos(pitch)
        self._send(frame_v2(MSP2_RANGEFINDER, rangefinder_payload(h / cos_tilt)) +
                   frame_v2(MSP2_FLOW, flow_payload(mx, my)))
        self.t_flow = now
        self.sent.append((now, vn, ve, float(v[0]), float(v[1]), h, mx, my))
        return mx, my

    # ---- fc_link.FC members apb_main reads
    def rate(self, kind, window=2.0):
        """Replies per second; apb_main check asks for the height stream as LOCAL_POSITION_NED (fc_link's name)."""
        a = self.arrivals.get("ALTITUDE" if kind == "LOCAL_POSITION_NED" else kind)
        now = time.monotonic()
        return sum(1 for t in list(a) if t > now - window) / window if a else 0.0

    def age(self, kind):
        a = self.arrivals.get(kind)
        return time.monotonic() - a[-1] if a else math.inf

    def local_pos(self, max_age=0.5):
        return None                                    # Betaflight sends no position over MSP

    def ping_timesync(self, n=20, gap=0.05):
        pass

    def send_origin(self, lat, lon, alt):
        pass

    def check(self):
        """[(ok, line)] for apb_main check: firmware, Position Hold box, flow and rangefinder detected."""
        out = [(self.firmware == "betaflight", f"MSP firmware {self.info}"),
               (BOX_POSHOLD in self.boxids, "POS HOLD mode (box 11) in this build" if BOX_POSHOLD in self.boxids
                else "no POS HOLD mode (box 11): this build has no Position Hold.")]
        st = self.request(MSP_STATUS_EX)
        bits = struct.unpack_from("<H", st, 4)[0] if st else 0
        out.append((bool(self.hgt.rows), f"barometer height {self.hgt.rows[-1][1]:.2f} m" if self.hgt.rows else
                    "no barometer height: CLI set debug_mode = BARO, save. Hold and the rangefinder message need it."))
        for bit, name, cli in ((SENSOR_RANGEFINDER, "rangefinder", "set rangefinder_hardware = MTF02P"),
                               (SENSOR_FLOW, "optical flow", "set opticalflow_hardware = MT")):
            out.append((bool(bits & bit), f"{name} detected" if bits & bit else
                        f"{name} not detected. CLI: {cli}, feature {name.upper().replace(' ', '')}, save."))
        return out

    def close(self):
        self._stop = True
        self._reader.join(timeout=1.0)
        if self._poll is not None:
            self._poll.join(timeout=1.0)
        self.port.close()


def connect(s):
    """(MspLink, info) when the FC on msp_device answers MSP as Betaflight; (None, info or None) otherwise."""
    dev = s["msp_device"]
    if not (dev.startswith("tcp:") or os.path.exists(dev)):
        return None, None
    try:
        fc = MspLink(s)
    except OSError:
        return None, None
    if fc.identify() == "betaflight":
        return fc, fc.info
    info = fc.info or None
    fc.close()
    return None, info
