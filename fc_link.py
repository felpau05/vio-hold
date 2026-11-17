"""MAVLink link to the flight controller.

- Firmware from HEARTBEAT (PX4 or ArduPilot). No heartbeat: an MSP probe names Betaflight or iNav,
  which have no external-vision input.
- ATTITUDE (with rates) and LOCAL_POSITION_NED (baro height, FC position) at set rates, other
  streams off (H5), real rates measured.
- FC boot time is mapped to the companion clock (time.monotonic) by a fitted offset (H1), so the
  att and hgt Series are in the same clock as the frame stamps.
- VISION_POSITION_ESTIMATE out, with covariance, as PX4 and ArduPilot need it (H2).
- Parameter check, and parameter writes only on request.
"""
import math
import os
import struct
import threading
import time
from collections import deque

os.environ.setdefault("MAVLINK20", "1")    # covariance and reset_counter are MAVLink 2 extensions
import numpy as np                         # noqa: E402
from pymavlink import mavutil              # noqa: E402

from hold import Series                    # noqa: E402

M = mavutil.mavlink
FIRMWARE = {M.MAV_AUTOPILOT_PX4: "px4", M.MAV_AUTOPILOT_ARDUPILOTMEGA: "ardupilot"}
# Streams the hold does not use; together they push ATTITUDE down to 45 Hz on this link (H5, t7).
UNUSED = (331, 31, 105, 33, 83, 85, 290, 291, 36, 65, 74, 231, 141)
PARAM_ALIAS = {"GPS1_TYPE": "GPS_TYPE"}    # ArduPilot renamed GPS_TYPE
INT_TYPES = (M.MAV_PARAM_TYPE_UINT8, M.MAV_PARAM_TYPE_INT8, M.MAV_PARAM_TYPE_UINT16, M.MAV_PARAM_TYPE_INT16,
             M.MAV_PARAM_TYPE_UINT32, M.MAV_PARAM_TYPE_INT32)


class Clock:
    """FC boot time -> companion time.

    arrival - stamp = offset + transport delay, and the delay is never negative. The minimum per
    1 s bin follows the offset; a line through the last 30 bin minima also follows crystal drift.
    The smallest transport delay stays in the mapping; the calibration absorbs it because it uses
    this same mapping.
    """

    def __init__(self, bin_s=1.0, nbins=30):
        self.bin_s, self.bins, self.cur, self.fit, self.last = bin_s, deque(maxlen=nbins), None, None, -math.inf

    def add(self, t_fc, t_arr):
        if t_fc < self.last - 1.0:              # FC rebooted: its clock started again
            self.bins.clear()
            self.cur = self.fit = None
        self.last = t_fc
        d, k = t_arr - t_fc, math.floor(t_fc / self.bin_s)
        if self.cur is None or k != self.cur[0]:
            if self.cur is not None:
                self.bins.append((self.cur[1], self.cur[2]))
                self._refit()
            self.cur = [k, t_fc, d]
        elif d < self.cur[2]:
            self.cur[1], self.cur[2] = t_fc, d

    def _refit(self):
        if len(self.bins) < 3:
            return
        t, d = np.array(self.bins).T
        t = t - t[-1]
        b, a = np.polyfit(t, d, 1)
        keep = d - (a + b * t) < 0.002          # a bin with a long stall has a high minimum: drop it
        if keep.sum() >= 3:
            b, a = np.polyfit(t[keep], d[keep], 1)
        self.fit = (self.bins[-1][0], a, b)

    def ready(self):
        return self.fit is not None

    def to_local(self, t_fc):
        if self.fit is None:
            return None
        t_ref, a, b = self.fit
        return t_fc + a + b * (t_fc - t_ref)


class FC:
    def __init__(self, s):
        self.s = s
        self.conn = mavutil.mavlink_connection(s["fc_device"], baud=s["fc_baud"], source_system=1,
                                               source_component=M.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY)
        self.att, self.hgt, self.lpos = Series(30.0), Series(30.0), Series(30.0)
        self.imu = Series(30.0)                  # SCALED_IMU: (t, ax, ay, az) in m/s^2, body FRD (att_source "imu")
        self.vz = Series(30.0)                   # the FC's own vertical speed (m/s, down) for VISION_SPEED_ESTIMATE
        self.clock = Clock()
        self.firmware = self.sysid = None
        self.compid, self.armed, self.mode = 1, False, None     # mode: the FC's custom_mode (ArduCopter: 4 GUIDED, 5 LOITER)
        self.arrivals = {}
        self.params, self.param_types, self.acks = {}, {}, {}
        self.rtts, self.texts = deque(maxlen=200), deque(maxlen=20)
        self.reset_counter = 0                   # H2: constant; Hold re-anchors without a jump
        self._tx, self._stop = threading.Lock(), False
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def _heartbeat(self):
        """Companion HEARTBEAT at 1 Hz (research/github_autopilot.md 2a, 3a): ArduPilot learns the route to us."""
        while not self._stop:
            try:
                self._send(lambda mav: mav.heartbeat_send(M.MAV_TYPE_ONBOARD_CONTROLLER, M.MAV_AUTOPILOT_INVALID,
                                                          0, 0, M.MAV_STATE_ACTIVE))
            except OSError:                      # link not up yet (serial unplugged, no UDP peer)
                pass
            time.sleep(1.0)

    # ---- receive ----
    def _read(self):
        while not self._stop:
            try:
                m = self.conn.recv_match(blocking=True, timeout=0.2)
            except Exception:                    # a broken packet or a closed socket: keep reading
                time.sleep(0.01)
                continue
            if m is None:
                continue
            now = time.monotonic()
            kind = m.get_type()
            self.arrivals.setdefault(kind, deque(maxlen=400)).append(now)
            if kind == "ATTITUDE":
                t_fc = m.time_boot_ms / 1000.0
                self.clock.add(t_fc, now)
                t = self.clock.to_local(t_fc)
                if t is not None and t > self.att.last_t():
                    self.att.add(t, m.roll, m.pitch, m.yaw, m.rollspeed, m.pitchspeed, m.yawspeed)
            elif kind == "LOCAL_POSITION_NED":
                t = self.clock.to_local(m.time_boot_ms / 1000.0)
                if t is not None and t > self.lpos.last_t():
                    self.lpos.add(t, m.x, m.y)
                    if self.firmware != "ardupilot":
                        self.hgt.add(t, -m.z)
                        self.vz.add(t, m.vz)
            elif kind == "SCALED_IMU":           # raw accelerometer (mG): the gravity reference of apb_main.ImuAtt
                t = self.clock.to_local(m.time_boot_ms / 1000.0)
                if t is not None and t > self.imu.last_t():
                    self.imu.add(t, m.xacc * 9.80665e-3, m.yacc * 9.80665e-3, m.zacc * 9.80665e-3)
            elif kind == "GLOBAL_POSITION_INT" and self.firmware == "ardupilot":
                # ArduPilot sends LOCAL_POSITION_NED only once the EKF has a horizontal position, which only
                # our fixes give, and the fixes need the height (deadlock seen in apsim). relative_alt (mm above
                # home, or baro height before home is set) comes always (GCS_Common send_global_position_int).
                t = self.clock.to_local(m.time_boot_ms / 1000.0)
                if t is not None and t > self.hgt.last_t():
                    self.hgt.add(t, m.relative_alt / 1000.0)
                    self.vz.add(t, m.vz / 100.0)
            elif kind == "HEARTBEAT":
                if m.autopilot != M.MAV_AUTOPILOT_INVALID and m.type != M.MAV_TYPE_GCS:
                    self.sysid, self.compid = m.get_srcSystem(), m.get_srcComponent()
                    self.firmware = FIRMWARE.get(m.autopilot, f"autopilot type {m.autopilot}")
                    self.armed = bool(m.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.mode = m.custom_mode
                    self.conn.mav.srcSystem = self.sysid
            elif kind == "TIMESYNC":
                if m.tc1 == 0 and self.s["answer_timesync"]:
                    self._send(lambda mav: mav.timesync_send(time.monotonic_ns(), m.ts1))
                elif m.tc1 != 0 and 0 < time.monotonic_ns() - m.ts1 < 1e9:
                    self.rtts.append(now - m.ts1 * 1e-9)     # reply to our own request
            elif kind == "PARAM_VALUE":
                name, v = m.param_id.rstrip("\x00"), m.param_value
                if self.firmware == "px4" and m.param_type in INT_TYPES:   # PX4 sends int bits in the float
                    v = struct.unpack("<i", struct.pack("<f", v))[0]
                self.params[name], self.param_types[name] = v, m.param_type
            elif kind == "COMMAND_ACK":
                self.acks[m.command] = m.result
            elif kind == "STATUSTEXT":
                self.texts.append(m.text)

    def _send(self, fn):
        with self._tx:
            fn(self.conn.mav)

    def rate(self, kind, window=2.0):
        a = self.arrivals.get(kind)
        if not a:
            return 0.0
        now = time.monotonic()
        return sum(1 for t in list(a) if t > now - window) / window

    def age(self, kind):
        a = self.arrivals.get(kind)
        return time.monotonic() - a[-1] if a else math.inf

    def wait_heartbeat(self, timeout):
        end = time.monotonic() + timeout
        while self.firmware is None and time.monotonic() < end:
            time.sleep(0.05)
        return self.firmware

    # ---- streams ----
    def setup_streams(self, wait=1.0):
        """Ask for ATTITUDE and LOCAL_POSITION_NED, turn the unused streams off. Returns {msg id: ack}."""
        want = {M.MAVLINK_MSG_ID_ATTITUDE: self.s["att_hz"], M.MAVLINK_MSG_ID_LOCAL_POSITION_NED: self.s["hgt_hz"]}
        want.update({i: 0 for i in UNUSED})
        if self.firmware == "ardupilot":         # the height source on ArduPilot (see _read)
            want[M.MAVLINK_MSG_ID_GLOBAL_POSITION_INT] = self.s["hgt_hz"]
        if self.s.get("att_source") == "imu" or self.s.get("wind_assist"):   # SCALED_IMU, 36 B (MAVLink 2): 50 Hz = 1.8 kB/s
            want[M.MAVLINK_MSG_ID_SCALED_IMU] = self.s["imu_hz"]
        self.acks.clear()
        for msg_id, hz in want.items():
            us = 1e6 / hz if hz > 0 else -1
            self._send(lambda mav: mav.command_long_send(self.sysid or 1, self.compid, M.MAV_CMD_SET_MESSAGE_INTERVAL,
                                                         0, msg_id, us, 0, 0, 0, 0, 0))
            time.sleep(0.01)
        time.sleep(wait)
        return self.acks.get(M.MAV_CMD_SET_MESSAGE_INTERVAL)

    def ping_timesync(self, n=20, gap=0.05):
        """Our own TIMESYNC requests; the round-trip times land in self.rtts."""
        for _ in range(n):
            self._send(lambda mav: mav.timesync_send(0, time.monotonic_ns()))
            time.sleep(gap)

    # ---- parameters ----
    def get_param(self, name, timeout=1.5):
        self.params.pop(name, None)
        for _ in range(2):
            self._send(lambda mav: mav.param_request_read_send(self.sysid or 1, self.compid, name.encode(), -1))
            end = time.monotonic() + timeout / 2
            while time.monotonic() < end:
                if name in self.params:
                    return self.params[name]
                time.sleep(0.01)
        return None

    def set_param(self, name, value):
        ptype = self.param_types.get(name, M.MAV_PARAM_TYPE_REAL32)
        v = float(value)
        if self.firmware == "px4" and ptype in INT_TYPES:
            v = struct.unpack("<f", struct.pack("<i", int(value)))[0]
        self._send(lambda mav: mav.param_set_send(self.sysid or 1, self.compid, name.encode(), v, ptype))
        time.sleep(0.2)
        return self.get_param(name)

    # ---- send ----
    def send_origin(self, lat, lon, alt):
        self._send(lambda mav: mav.set_gps_global_origin_send(self.sysid or 1, int(lat * 1e7), int(lon * 1e7),
                                                              int(alt * 1000), int(time.monotonic() * 1e6)))
        if self.firmware == "ardupilot":
            # Home at the origin, so GLOBAL_POSITION_INT relative_alt (our height) is above the field.
            # ArduPilot 4.6 has no handler for the SET_HOME_POSITION message; MAV_CMD_DO_SET_HOME is its way.
            time.sleep(0.2)
            self._send(lambda mav: mav.command_int_send(self.sysid or 1, self.compid, M.MAV_FRAME_GLOBAL,
                                                        M.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0, 0,
                                                        int(lat * 1e7), int(lon * 1e7), float(alt)))

    def send_fix(self, e, att=None, hgt=None):
        """VISION_POSITION_ESTIMATE for one Est whose var is not None.

        usec = Est.t (mid-frame capture time, companion clock); with TIMESYNC answered PX4 maps it.
        z = the FC's own height (NED down), finite as PX4 needs, harmless if fused. Angles: NaN for PX4
        so a default EKF2_EV_CTRL cannot fuse yaw (H2); ArduPilot gets the FC's own attitude.
        Covariance: [0] [1] [6] = the 2x2 position covariance; [11] = 0 (finite for PX4, adds nothing
        to ArduPilot's error sum). att, hgt: the caller's copies of the Series (default: the live ones).
        """
        att, hgt = att if att is not None else self.att, hgt if hgt is not None else self.hgt
        cov = getattr(e, "cov", None)
        if cov is None:
            cov = np.eye(2) * (e.var / 2)        # version-1 Hold: var is the two-axis total (H13)
        cov = np.asarray(cov, float) * self.s["cov_scale"]
        c = [0.0] * 21
        c[0], c[1], c[6] = float(cov[0][0]), float(cov[0][1]), float(cov[1][1])
        z = -float(hgt.at(e.t))
        if self.firmware == "px4":
            ang = (math.nan, math.nan, math.nan)
        else:
            a = att.array()
            ang = tuple(float(np.interp(e.t, a[:, 0], a[:, i])) for i in (1, 2, 3))
        self._send(lambda mav: mav.vision_position_estimate_send(int(e.t * 1e6), float(e.pos[0]), float(e.pos[1]),
                                                                 z, *ang, covariance=c,
                                                                 reset_counter=self.reset_counter))

    def send_speed(self, t, vn, ve, vd):
        """VISION_SPEED_ESTIMATE (NED, m/s) at time t (companion clock). ArduPilot ignores the covariance and
        uses VISO_VEL_M_NSE (AP_VisualOdom_MAV handle_vision_speed_estimate); it is filled for PX4's sake."""
        sd = self.s["speed_sd_mps"] ** 2
        self._send(lambda mav: mav.vision_speed_estimate_send(int(t * 1e6), vn, ve, vd,
                                                              covariance=[sd, 0, 0, 0, sd, 0, 0, 0, sd],
                                                              reset_counter=self.reset_counter))

    def set_mode(self, custom_mode):
        """MAV_CMD_DO_SET_MODE with an ArduPilot custom mode (5 = LOITER)."""
        self._send(lambda mav: mav.command_long_send(self.sysid or 1, self.compid, M.MAV_CMD_DO_SET_MODE, 0,
                                                     M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode, 0, 0, 0, 0, 0))

    def send_target(self, n, e, d, yaw):
        """SET_POSITION_TARGET_LOCAL_NED: position, speed 0 and a fixed heading (rad). ArduCopter GUIDED only
        (ignored in other modes). The heading must be given: without it GUIDED turned the nose toward each new
        target and the drone yawed 5 -> 110 deg in wind (run c_wa_100m_w20g40_s11, first version)."""
        mask = M.POSITION_TARGET_TYPEMASK_AX_IGNORE | M.POSITION_TARGET_TYPEMASK_AY_IGNORE | \
            M.POSITION_TARGET_TYPEMASK_AZ_IGNORE | M.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        self._send(lambda mav: mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1e3) & 0xFFFFFFFF, self.sysid or 1, self.compid, M.MAV_FRAME_LOCAL_NED, mask,
            n, e, d, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, yaw, 0.0))

    def local_pos(self, max_age=0.5):
        """The FC's own (north, east), for re-anchoring; None when it is missing or old."""
        if self.age("LOCAL_POSITION_NED") > max_age or not self.lpos.rows:
            return None
        return np.array(self.lpos.rows[-1][1:3], float)

    def close(self):
        self._stop = True
        self._reader.join(timeout=1.0)
        self.conn.close()


# ---- detection ----
def probe_msp(device, baud=230400, timeout=1.0):
    """Betaflight and iNav answer MSP, not MAVLink. Returns e.g. {'variant': 'BTFL', 'api': '1.46'} or None."""
    import termios
    import tty
    try:
        fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return None
    try:
        tty.setraw(fd)
        speed = getattr(termios, f"B{baud}", termios.B115200)
        a = termios.tcgetattr(fd)
        a[4] = a[5] = speed
        termios.tcsetattr(fd, termios.TCSANOW, a)
        for cmd in (1, 2):                       # MSP_API_VERSION, MSP_FC_VARIANT (MSP v1, no payload)
            os.write(fd, b"$M<" + bytes([0, cmd, cmd]))
        buf, out, end = b"", {}, time.monotonic() + timeout
        while time.monotonic() < end and len(out) < 2:
            try:
                buf += os.read(fd, 256)
            except BlockingIOError:
                time.sleep(0.01)
            while (i := buf.find(b"$M>")) >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]:
                n, cmd = buf[i + 3], buf[i + 4]
                body = buf[i + 5:i + 5 + n]
                if cmd == 1 and n >= 3:
                    out["api"] = f"{body[1]}.{body[2]}"
                elif cmd == 2 and n >= 4:
                    out["variant"] = body[:4].decode("ascii", "replace")
                buf = buf[i + 6 + n:]
        return out or None
    finally:
        os.close(fd)


BAUDS = (230400, 115200, 57600, 921600)      # research/github_autopilot.md 3(a): speeds to try on a serial port


def connect(s, timeout=5.0):
    """(FC, None) for PX4 or ArduPilot, else (None, plain reason). On a serial device with no heartbeat at
    fc_baud, the other BAUDS are tried; fc.baud says which one answered."""
    serial = s["fc_device"].startswith("/dev/")
    fw = None
    for baud in [s["fc_baud"]] + ([b for b in BAUDS if b != s["fc_baud"]] if serial else []):
        fc = FC(dict(s, fc_baud=baud))
        fc.baud = baud
        fw = fc.wait_heartbeat(timeout if baud == s["fc_baud"] else min(timeout, 2.5))
        if fw in ("px4", "ardupilot"):
            return fc, None
        fc.close()
        if fw is not None:
            break
    if fw is not None:
        return None, (f"the flight controller says MAVLink {fw} (iNav or Betaflight telemetry?). "
                      "This firmware has no external-vision input; not supported yet.")
    msp = probe_msp(s["msp_device"], s["fc_baud"]) if os.path.exists(s["msp_device"]) else None
    if msp:
        name = {"BTFL": "Betaflight", "INAV": "iNav"}.get(msp.get("variant"), msp.get("variant", "an MSP firmware"))
        return None, (f"the flight controller runs {name} (MSP API {msp.get('api', '?')}). "
                      "This firmware has no external-vision input; not supported yet.")
    tried = f" at {', '.join(str(b) for b in BAUDS)} baud" if serial else ""
    return None, (f"no MAVLink heartbeat on {s['fc_device']}{tried} in {timeout:.0f} s and no MSP answer on "
                  f"{s['msp_device']}. Check the cable, the baud rate and that mavlink-router runs.")


def check_params(fc, s, write=False):
    """[(ok, line)] for each needed parameter. Writes a wrong value only when write is True."""
    need = s["params_px4"] if fc.firmware == "px4" else s["params_ardupilot"]
    out = []
    for name, want in need.items():
        got = fc.get_param(name)
        if got is None and name in PARAM_ALIAS:
            name, got = PARAM_ALIAS[name], fc.get_param(PARAM_ALIAS[name])
        if got is None:
            hint = (" Older PX4 (v1.13)? Set EKF2_AID_MASK 8 and EKF2_HGT_MODE 0 by hand."
                    if name.endswith(("_CTRL", "_HGT_REF")) and name.startswith("EKF2") else
                    " This firmware may not have it: check the name for its version." if name.startswith("EKF2") else "")
            out.append((False, f"FAIL param {name}: no answer; need {want}.{hint}"))
            continue
        ok = abs(got - want) <= 1e-3 * max(1.0, abs(want))
        if not ok and write:
            got = fc.set_param(name, want)
            ok = got is not None and abs(got - want) <= 1e-3 * max(1.0, abs(want))
            out.append((ok, f"{'PASS' if ok else 'FAIL'} param {name}: wrote {want}, reads {got}."
                            + (" Reboot the flight controller." if ok else " Write refused; set it in the ground station.")))
            continue
        out.append((ok, f"PASS param {name} = {got}" if ok else
                    f"FAIL param {name} = {got}; need {want}. Run 'apb_main.py check --set-params', then reboot the FC."))
    return out
