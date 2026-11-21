#!/usr/bin/env python3
"""msp_rc against a fake Betaflight FC on a pseudo-terminal: frame layout, signs, limits, the neutral command
(not ready, stale, at start and at close), the engage switch edge and the send rate.
Run: ~/.venvs/vio/bin/python tests_apb/test_msp_rc.py      (about 4 s)"""
import os
import struct
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import msp_link as M      # noqa: E402
import msp_rc as R        # noqa: E402

FAILS = []


def check(ok, msg):
    print(("PASS " if ok else "FAIL ") + msg)
    if not ok:
        FAILS.append(msg)


class FakeFC:
    """Reads '$M<' requests on the pty master. Acks MSP_SET_RAW_RC, answers MSP_RC with self.ch."""

    def __init__(self, fd):
        self.fd, self.rc, self.ch = fd, [], [1500, 1500, 1500, 885, 1000, 2000, 1000, 1000, 1000, 1500]   # CH10 1500: no radio (practice)
        self.stop = threading.Event()
        threading.Thread(target=self.loop, daemon=True).start()

    def reply(self, cmd, p=b""):
        body = bytes([len(p), cmd]) + p
        x = 0
        for b in body:
            x ^= b
        os.write(self.fd, b"$M>" + body + bytes([x]))

    def loop(self):
        buf = b""
        while not self.stop.is_set():
            try:
                buf += os.read(self.fd, 4096)
            except OSError:
                return
            while True:
                i = buf.find(b"$M<")
                if i < 0 or len(buf) < i + 6 or len(buf) < i + 6 + buf[i + 3]:
                    break
                n, cmd = buf[i + 3], buf[i + 4]
                p = buf[i + 5:i + 5 + n]
                buf = buf[i + 6 + n:]
                if cmd == R.MSP_SET_RAW_RC:
                    self.rc.append((time.monotonic(), struct.unpack("<8H", p)))
                    self.reply(cmd)
                elif cmd == R.MSP_RC:
                    self.reply(cmd, struct.pack(f"<{len(self.ch)}H", *self.ch))


def last(fc):
    return fc.rc[-1][1] if fc.rc else None


def main():
    master, slave = os.openpty()
    fc = FakeFC(master)
    rc = R.RcOut(port=M.Port(os.ttyname(slave)), hover_us=1450, say=lambda x: None)
    t0 = time.monotonic()
    rc.start()
    time.sleep(0.05)
    check(fc.rc and fc.rc[0][0] - t0 < 0.05 and fc.rc[0][1][:4] == (1500, 1500, 1450, 1500)
          and fc.rc[0][1][4:] == (1500,) * 4, f"first command at start-up is neutral: {fc.rc[:1]}")
    cases = [((True, 0.5, 0.0), (1500, 1450), "forward 0.5 m -> pitch back"),
             ((True, -0.5, 0.0), (1500, 1550), "back 0.5 m -> pitch forward"),
             ((True, 0.0, 0.5), (1450, 1500), "right 0.5 m -> roll left"),
             ((True, 0.0, -0.5), (1550, 1500), "left 0.5 m -> roll right"),
             ((True, 3.0, -3.0), (1600, 1400), "3 m: limited to 100 us"),
             ((False, 0.5, 0.5), (1500, 1500), "not ready -> neutral"),
             ((True, None, None), (1500, 1500), "no move value (floor-track gap) -> neutral")]
    for (ready, f, r), (roll, pitch), name in cases:
        for _ in range(5):                       # keep the update fresh, as the camera loop does
            rc.update(ready, f, r)
            time.sleep(0.02)
        v = last(fc)
        check(v is not None and v[0] == roll and v[1] == pitch and v[2] == 1450 and v[3] == 1500,
              f"{name}: sent {v[:4] if v else None}")
    rc.update(True, 0.5, 0.0)
    time.sleep(0.5)                              # no update for 0.5 s: stale
    check(last(fc)[:4] == (1500, 1500, 1450, 1500), f"stale update (0.5 s) -> neutral: {last(fc)[:4]}")
    n0, ta = len(fc.rc), time.monotonic()
    time.sleep(1.0)
    hz = (len(fc.rc) - n0) / (time.monotonic() - ta)
    check(45 <= hz <= 55, f"send rate {hz:.1f} Hz")
    check(not rc.engaged and not rc.take_engage_edge(), "no radio (CH10 1500): not engaged, no edge")

    def hold(ready, f, r, s=0.3):
        t = time.monotonic()
        while time.monotonic() - t < s:
            rc.update(ready, f, r)
            time.sleep(0.02)
        return last(fc)[:2]
    fc.ch[9] = 1000
    check(hold(True, 0.5, 0.0) == (1500, 1500), "switch clearly off (CH10 1000): neutral, although ready and moved")
    fc.ch[9] = 1750
    check(hold(True, 0.5, 0.0) == (1500, 1450) and not rc.engaged, "CH10 1750: not engaged (Betaflight needs 1800)")
    fc.ch[9] = 1000
    hold(True, 0.5, 0.0, 0.5)               # the low reading arrives within 0.1 s (MSP_RC at 10 Hz)
    fc.ch[9] = 1800
    v = hold(True, 0.5, 0.0, 0.3)             # the caller has not set the new zero yet
    check(rc.engaged and v == (1500, 1500), f"switch on (1800): engaged, neutral until the new zero: {v}")
    check(rc.take_engage_edge() and not rc.take_engage_edge(), "switch on: one edge")
    early = [x[1][:2] for x in fc.rc if x[0] > time.monotonic() - 0.6]   # 0.3 s switch-on + the last 0.3 s of switch-off
    check(all(x == (1500, 1500) for x in early), "no old-move command around switch-on (last 0.6 s all neutral)")
    rc.mark_zero()
    rc.zero_done()
    check(hold(True, 0.0, 0.5) == (1450, 1500), "after the new zero: commands follow the move again")
    h = rc.history(5.0)
    check(any(r["zero"] for r in h), "the log marks the zero")
    fc.ch[9] = 1000
    time.sleep(0.3)
    check(not rc.engaged, "switch low again: not engaged")
    fc.ch[9] = 1500
    st = rc.status()
    check(st["rc_acks"] > 50 and st["rc_read_age"] is not None and st["rc_read_age"] < 0.5 and not st["rc_errors"],
          f"status: {st['rc_acks']} acks, read age {st['rc_read_age']:.2f} s, errors {st['rc_errors']}")
    print("info:", rc.info())
    check("FC sees roll 1500 pitch 1500 thr 885 yaw 1500" in rc.info(), "info shows the FC readback (Receiver tab view)")
    try:
        rc._write(201, b"\x00" * 18)                 # MSP_SET_RAW_GPS
        check(False, "MSP_SET_RAW_GPS (201) refused")
    except ValueError:
        check(True, "MSP_SET_RAW_GPS (201) refused: msp_rc sends only 200 and 105")
    for _ in range(5):
        rc.update(True, 1.0, 1.0)
        time.sleep(0.02)
    n0 = len(fc.rc)
    rc.close()
    time.sleep(0.1)
    tail = [v[:4] for _, v in fc.rc[n0:]]
    check(len(tail) >= 10 and all(v == (1500, 1500, 1450, 1500) for v in tail[-10:]),
          f"close: last 10 commands neutral ({len(tail)} after close)")
    # switch ON at program start (restart with the switch on): no edge, no zero, neutral until off then on
    fc.ch[9] = 2000
    rc2 = R.RcOut(port=M.Port(os.ttyname(slave)), hover_us=1450, say=lambda x: None).start()
    n2 = len(fc.rc)
    t = time.monotonic()
    while time.monotonic() - t < 0.5:
        rc2.update(True, 0.0, 0.5)
        time.sleep(0.02)
    tail2 = [v[:2] for _, v in fc.rc[n2:]]
    check(rc2.engaged and not rc2.take_engage_edge() and tail2 and all(v == (1500, 1500) for v in tail2)
          and "AT START" in rc2.info(), f"switch ON at start: no edge, neutral ({len(tail2)} commands, engaged {rc2.engaged}, {sorted(set(tail2))}, {rc2.info()})")
    fc.ch[9] = 1000
    time.sleep(0.3)
    fc.ch[9] = 2000
    time.sleep(0.3)
    check(rc2.take_engage_edge(), "switch ON at start, then off and on: one edge")
    rc2.zero_done()
    t = time.monotonic()
    while time.monotonic() - t < 0.3:
        rc2.update(True, 0.0, 0.5)
        time.sleep(0.02)
    check(last(fc)[:2] == (1450, 1500), f"after off-on and the new zero: commands follow ({last(fc)[:2]})")
    rc2.close()
    fc.ch[9] = 1500
    try:
        R.RcOut(port=M.Port(os.ttyname(slave)), hover_us=1000)
        check(False, "hover_us 1000 refused")
    except ValueError:
        check(True, "hover_us 1000 (minimum throttle) refused")
    fc.stop.set()
    print("ALL PASS" if not FAILS else f"{len(FAILS)} FAIL")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
