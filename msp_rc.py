"""Stick commands to the Betaflight FC over MSP (Stage 2, section 03): bench items C, D and E.

The FC: Orqa Betaflight 2025.12.5 on the APB serial port /dev/ttymxc2, 115200 baud, MSP v1. With the engage
switch (CH10) high, the FC uses MSP_SET_RAW_RC for roll, pitch, throttle and yaw (MSP OVERRIDE + ANGLE).
Switch low: the pilot has control at once.

Safety rules of Stage 2 that this module follows:
- The FC has no command timeout: it keeps the last command. Thus a thread sends at 50 Hz from start-up,
  ready or not, and close() sends the neutral command several times before it stops.
- Before the first command after an FC power-up, the four channels read minimum with the switch on. Thus the
  first command goes out before the camera or the estimator starts (start() at program start).
- Not ready: roll, pitch and yaw 1500 and a throttle that holds height (hover_us). Never a minimum value.

Commands when ready: the move since the zero, in the body axes of now (forward, right, metres), opposed:
  roll  = 1500 - gain * right     (moved right -> roll left)
  pitch = 1500 - gain * forward   (moved forward -> pitch back; Betaflight: pitch above 1500 = nose down)
each limited to max_us. Yaw 1500. Throttle hover_us. An update older than stale_s gives the neutral command
(the estimator stopped: E1, a frozen camera, a stalled loop).

The engage switch is read with MSP_RC (cmd 105) at 10 Hz: channel 10 is index 9. take_engage_edge() returns
True once after each switch-on, so the caller can set the position zero at engage.

NOT FOR FLIGHT YET: hover_us is not measured on this aircraft, and gain is a bench value (a visible response
to a 0.5 m move). A flight needs a measured hover throttle, a height loop and a tuned position loop.
"""
import os
import struct
import threading
import time
from collections import deque

import msp_link as M

LOG_EVERY = 5            # one log row per 5 commands: 10 Hz at 50 Hz
HISTORY_N = 6000         # rows kept in memory for the replay: 10 min at 10 Hz
LOG_COLS = ("t", "wall", "ready", "fwd", "right", "roll", "pitch", "thr", "yaw",
            "fc_roll", "fc_pitch", "fc_thr", "fc_yaw", "ch10", "engaged", "zero")

MSP_RC = 105
MSP_SET_RAW_RC = 200
ALLOWED = (MSP_SET_RAW_RC, MSP_RC)   # GNSS rule proof: this module sends no other MSP command (no MSP_SET_RAW_GPS)
NEUTRAL_US = 1500
SWITCH_HIGH_US = 1800    # Betaflight MSP OVERRIDE range on this FC: 1800-2100 (rc_modes.c), values clamp to 2099
SWITCH_LOW_US = 1300     # clearly off on a real radio (about 1000). No radio reads about 1500: no gate (practice)
CH_ENGAGE = 9            # channel 10, index 9 (Stage 2: CH10 = TL2)


def rc_payload(roll, pitch, throttle, yaw, aux=(NEUTRAL_US,) * 4):
    """MSP_SET_RAW_RC payload: 8 x uint16 in the order roll, pitch, throttle, yaw, AUX1-4 (Stage 2)."""
    v = [int(round(min(2100, max(900, x)))) for x in (roll, pitch, throttle, yaw, *aux)]
    return struct.pack("<8H", *v)


def command(ready, fwd, right, gain, max_us, hover_us):
    """(roll, pitch, throttle, yaw) in microseconds. Not ready or no move: neutral sticks, hover throttle."""
    if not ready or fwd is None or right is None:
        return NEUTRAL_US, NEUTRAL_US, hover_us, NEUTRAL_US
    lim = lambda x: max(-max_us, min(max_us, x))       # noqa: E731
    return NEUTRAL_US - lim(gain * right), NEUTRAL_US - lim(gain * fwd), hover_us, NEUTRAL_US


class RcOut:
    def __init__(self, device="/dev/ttymxc2", baud=115200, hz=50.0, gain_us_per_m=100.0, max_us=100.0,
                 hover_us=1500, stale_s=0.3, say=print, port=None, log_dir=None):
        if hover_us <= 1000:
            raise ValueError(f"hover_us {hover_us}: a not-ready throttle must hold height, never minimum")
        self.port = port or M.Port(device, baud)
        # Replay (C, D): 10 Hz rows of the move, our command and the FC readback. In memory for the page,
        # and in log_dir/rc_<time>.csv, so the test can be shown after the laptop is plugged in again.
        self.hist = deque(maxlen=HISTORY_N)
        self.log = None
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
            # The board clock starts at the same time after each power-up: add the boot id, and a counter
            # if the name is still taken, so a new boot never overwrites the log of the last one.
            try:
                boot = open("/proc/sys/kernel/random/boot_id").read().strip()[:8]
            except OSError:
                boot = "noboot"
            base = os.path.join(log_dir, f"rc_{time.strftime('%H%M%S')}_{boot}")
            name, k = base + ".csv", 1
            while os.path.exists(name):
                name, k = f"{base}_{k}.csv", k + 1
            self.log = open(name, "w", buffering=1)
            self.log.write(",".join(LOG_COLS) + "\n")
        self.hz, self.gain, self.max_us, self.hover, self.stale_s, self.say = hz, gain_us_per_m, max_us, hover_us, stale_s, say
        self.lock = threading.Lock()
        self.state = (False, None, None, -1e9)          # ready, fwd, right, time
        self.sent = None                                 # last (roll, pitch, throttle, yaw)
        self.n_sent = self.n_rc = self.n_ack = 0
        self.channels, self.t_channels = None, None
        self.engaged, self._edge = False, False
        self.wait_zero = False                           # after switch-on: neutral until the caller set the new zero
        self.seen_off = False                            # a switch-on counts only after the switch was seen off
        self.on_at_start = False                         # switch ON at program start: neutral until off, then on
        self.zero_mark = False                           # the next log row marks a new zero (start of a test)
        self.switch_low = False                          # fresh reading of a switch clearly off: neutral (D-4)
        self.errors = []
        self._stop = threading.Event()
        self._threads = []

    def start(self):
        """Send the neutral command at once, then keep sending. Call this first thing at program start."""
        self._send_cmd(command(False, None, None, self.gain, self.max_us, self.hover))
        for fn in (self._write_loop, self._read_loop):
            t = threading.Thread(target=fn, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def update(self, ready, fwd_body, right_body):
        """Call once per picture with the move since the zero in the body axes of now (m), or None."""
        with self.lock:
            self.state = (bool(ready) and not self.wait_zero, fwd_body, right_body, time.monotonic())

    def mark_zero(self):
        """The position zero was set ("Zero here" or switch-on): the log marks where a test starts."""
        with self.lock:
            self.zero_mark = True

    def zero_done(self):
        """The caller set the new zero after switch-on: commands may follow the move again."""
        with self.lock:
            self.wait_zero = False

    def take_engage_edge(self):
        """True once after each switch-on (low -> high)."""
        with self.lock:
            e, self._edge = self._edge, False
        return e

    def status(self):
        with self.lock:
            age = None if self.t_channels is None else time.monotonic() - self.t_channels
            return dict(rc_sent=self.sent, rc_n=self.n_sent, rc_acks=self.n_ack, engaged=self.engaged,
                        ch10=None if not self.channels or len(self.channels) <= CH_ENGAGE else self.channels[CH_ENGAGE],
                        rc_read_age=age, rc_errors=self.errors[-1:] if self.errors else [])

    def fields(self):
        """Numbers for the page state (live view): our command, the FC readback and the switch."""
        with self.lock:
            sent, ch, eng = self.sent, self.channels, self.engaged
        f = dict(rc_roll=None, rc_pitch=None, rc_thr=None, rc_yaw=None,
                 fc_roll=None, fc_pitch=None, fc_thr=None, fc_yaw=None, engaged=eng)
        if sent:
            f.update(rc_roll=sent[0], rc_pitch=sent[1], rc_thr=sent[2], rc_yaw=sent[3])
        if ch and len(ch) >= 4:                         # MSP_RC order: roll, pitch, yaw, throttle
            f.update(fc_roll=ch[0], fc_pitch=ch[1], fc_yaw=ch[2], fc_thr=ch[3])
        return f

    def history(self, seconds=120.0):
        """Rows (dicts, LOG_COLS) of the last `seconds`, oldest first, for the replay."""
        with self.lock:
            rows = list(self.hist)
        if not rows:
            return []
        t_end = rows[-1][0]
        return [dict(zip(LOG_COLS, r)) for r in rows if r[0] >= t_end - seconds]

    def _log_row(self):
        with self.lock:
            ready, fwd, right, t = self.state
            sent, ch, eng = self.sent, self.channels, self.engaged
            zero, self.zero_mark = self.zero_mark, False
        if time.monotonic() - t > self.stale_s:
            ready = False
        fc = ch[:4] if ch and len(ch) >= 4 else (None,) * 4
        row = (round(time.monotonic(), 3), round(time.time(), 3), bool(ready),
               None if fwd is None else round(fwd, 3), None if right is None else round(right, 3),
               *(sent if sent else (None,) * 4),
               fc[0], fc[1], fc[3], fc[2],             # roll, pitch, throttle, yaw
               None if not ch or len(ch) <= CH_ENGAGE else ch[CH_ENGAGE], eng, zero)
        with self.lock:
            self.hist.append(row)
        if self.log:
            try:
                self.log.write(",".join("" if v is None else str(int(v) if isinstance(v, bool) else v) for v in row) + "\n")
            except (OSError, ValueError):
                pass

    def info(self):
        st = self.status()
        if st["rc_sent"] is None:
            return "FC commands: none sent"
        r, p, t, y = st["rc_sent"]
        sw = "-" if st["ch10"] is None else ("ON" if st["engaged"] else "off")
        old = st["rc_read_age"] is None or st["rc_read_age"] > 1.0
        with self.lock:
            ch = self.channels
        # MSP_RC order: roll, pitch, yaw, throttle (Betaflight rcData). This is the Receiver tab view.
        fc = "-" if not ch or len(ch) < 4 else f"roll {ch[0]} pitch {ch[1]} thr {ch[3]} yaw {ch[2]}"
        if self.on_at_start and self.wait_zero and st["engaged"]:
            sw += " AT START: commands neutral, turn the switch off and on"
        return (f"FC out roll {r:.0f} pitch {p:.0f} thr {t:.0f} yaw {y:.0f} · FC sees {fc} · engage switch {sw}"
                + (" (no FC reply)" if old else "") + (f" · FC error {st['rc_errors'][0]}" if st["rc_errors"] else ""))

    def close(self):
        """Neutral command 10 times (0.2 s), then stop. The FC keeps the last command it receives."""
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        for _ in range(10):
            self._send_cmd(command(False, None, None, self.gain, self.max_us, self.hover))
            time.sleep(0.02)
        try:
            self.port.close()
        except OSError:
            pass
        if self.log:
            self.log.close()

    # ---------------------------------------------------------------- internals
    def _write(self, code, payload=b""):
        if code not in ALLOWED:                          # GNSS rule: never MSP_SET_RAW_GPS or any other command
            raise ValueError(f"msp_rc sends only MSP {ALLOWED}, not {code}")
        self.port.write(M.frame_v1(code, payload))

    def _send_cmd(self, cmd):
        try:
            self._write(MSP_SET_RAW_RC, rc_payload(*cmd))
            with self.lock:
                self.sent, self.n_sent = cmd, self.n_sent + 1
        except OSError as ex:
            with self.lock:
                self.errors.append(f"write: {ex}")

    def _write_loop(self):
        period, k = 1.0 / self.hz, 0
        nxt = time.monotonic()
        while not self._stop.is_set():
            with self.lock:
                ready, fwd, right, t = self.state
                fresh = self.t_channels is not None and time.monotonic() - self.t_channels < 0.3
                if fresh and self.switch_low:
                    ready = False        # switch off: the FC ignores us, and at switch-on it must get neutral (D-4)
                if self.t_channels is None:
                    ready = False        # switch position not known yet (no MSP_RC reply): neutral
            if time.monotonic() - t > self.stale_s:
                ready = False
            self._send_cmd(command(ready, fwd, right, self.gain, self.max_us, self.hover))
            k += 1
            if k % LOG_EVERY == 0:
                self._log_row()
            if k % max(1, int(round(self.hz / 10))) == 0:
                try:
                    self._write(MSP_RC)
                except OSError as ex:
                    with self.lock:
                        self.errors.append(f"write: {ex}")
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))
            if time.monotonic() - nxt > 0.5:            # the loop fell behind (machine busy): do not burst
                nxt = time.monotonic()

    def _read_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                b = self.port.read()
            except OSError as ex:
                with self.lock:
                    self.errors.append(f"read: {ex}")
                time.sleep(0.1)
                continue
            if not b:
                continue
            frames, buf = M.parse(buf + b)
            for cmd, p, ok in frames:
                if not ok:
                    with self.lock:
                        self.errors.append(f"FC refused MSP command {cmd}")
                elif cmd == MSP_SET_RAW_RC:
                    with self.lock:
                        self.n_ack += 1
                elif cmd == MSP_RC and len(p) >= 2:
                    ch = struct.unpack(f"<{len(p) // 2}H", p[:len(p) // 2 * 2])
                    on = len(ch) > CH_ENGAGE and ch[CH_ENGAGE] >= SWITCH_HIGH_US
                    with self.lock:
                        self.switch_low = len(ch) > CH_ENGAGE and ch[CH_ENGAGE] < SWITCH_LOW_US
                        if not on:
                            self.seen_off = True
                        if on and not self.engaged:
                            self.wait_zero = True
                            self.state = (False, None, None, time.monotonic())   # old move: never at switch-on
                            if self.seen_off:
                                self._edge, self.on_at_start = True, False
                            else:
                                self.on_at_start = True  # ON at start (restart, power cut): no zero here, neutral
                        self.channels, self.t_channels, self.engaged, self.n_rc = ch, time.monotonic(), on, self.n_rc + 1
