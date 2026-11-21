#!/usr/bin/env python3
"""Event-day additions from the Stage 1 package (rev 1.3), with fake cameras and a fake flight controller:
E1 mode_bench and _recording close the camera when the flight controller connection fails (as D1 for mode_run);
E2 the FC flight mode decode (ArduCopter, PX4); E3 run_live logs the mode and notes the pilot's switch (judging
area D), and asks Hold for a fresh keyframe when fresh_kf_at_engage is set; E4 the recovery demonstration
(outage_test_s: trigger file and outage_after_s); E5 Hold.request_keyframe on a still hover (new home keyframe,
no position jump, fixes go on); E6 the KF_BRIGHT gate holds a requested keyframe back while the brightness moves.
Run: ~/.venvs/vio/bin/python tests_apb/test_event.py      (about 30 s; one part: add e1 ... e6)
"""
import csv
import math
import os
import sys
import tempfile
import time
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import apb_main            # noqa: E402
import capture             # noqa: E402
import test_digital as T   # noqa: E402
import test_robust as R    # noqa: E402  (patched, frames, fake_fc)
from hold import Series    # noqa: E402

LENS_D = R.LENS_D
fails = 0


def check(ok, text):
    global fails
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} {text}", flush=True)


class Closable:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


# ---------------------------------------------------------------- E1
def e1_leaks(d):
    print("E1: the flight controller connection fails in mask/calib recording and in bench mode")
    s = T.digital_settings(log_dir=d)
    for name, call in (("_recording", lambda: apb_main._recording(s, SimpleNamespace(rec=None), 1.0, print)),
                       ("mode_bench", lambda: apb_main.mode_bench(s, SimpleNamespace(height=1.0, seconds=1), print))):
        cams, err = [], None

        def open_video(s):
            cams.append(Closable())
            return cams[-1]

        def open_fc(s, say=print):
            raise RuntimeError("no heartbeat from the flight controller (test)")

        with R.patched(apb_main, open_video=open_video, open_fc=open_fc):
            try:
                call()
            except RuntimeError as e:
                err = e
        check(err is not None and len(cams) == 1 and cams[0].closed,
              f"{name}: the error goes on ({err}) and the camera is closed: {cams and cams[0].closed}")


# ---------------------------------------------------------------- E2
def e2_modes():
    print("E2: flight mode names and which modes hold a position")
    N = SimpleNamespace
    cases = [(N(mode=None, firmware="ardupilot"), (None, False)), (N(mode=2, firmware="ardupilot"), ("ALT_HOLD", False)),
             (N(mode=5, firmware="ardupilot"), ("LOITER", True)), (N(mode=16, firmware="ardupilot"), ("POSHOLD", True)),
             (N(mode=9, firmware="ardupilot"), ("LAND", False)), (N(mode=99, firmware="ardupilot"), ("mode 99", False)),
             (N(mode=2 << 16, firmware="px4"), ("ALTCTL", False)), (N(mode=3 << 16, firmware="px4"), ("POSCTL", True)),
             (N(mode=(3 << 24) | (4 << 16), firmware="px4"), ("AUTO_LOITER", True)),
             (N(mode=(4 << 24) | (4 << 16), firmware="px4"), ("AUTO", False)), (N(mode=7, firmware="betaflight"), ("mode 7", False)),
             (SimpleNamespace(), (None, False))]
    bad = [(c.__dict__, apb_main.fc_mode(c), want) for c, want in cases if apb_main.fc_mode(c) != want]
    check(not bad, f"{len(cases)} cases decode as expected: {bad or 'all'}")


# ---------------------------------------------------------------- run_live helpers
class ModeSource(T.ListSource):
    """Frames at 60 fps (paced in real time); before frame i it sets fc.mode = modes[i]."""

    def __init__(self, frs, fc, modes):
        super().__init__(frs)
        self.fc, self.modes, self.i = fc, modes, 0

    def newest(self, timeout=0.5):
        if not self.frames:
            return super().newest(timeout)
        if self.i in self.modes:
            self.fc.mode = self.modes[self.i]
        self.i += 1
        time.sleep(1 / 60)
        now = time.monotonic()               # stamps on the monotonic clock, as the camera drivers give them: run_live
        return super().newest(timeout)._replace(t0=now - 0.04, t_grab=now)   # writes age_ms = monotonic now - t0


def fc_now(s, sent=None):
    """R.fake_fc with its level attitude and 50 m height up to t = 1e7 s, so that the monotonic stamps of
    ModeSource are covered (else run_live waits att_wait_s for attitude on each frame)."""
    fc = R.fake_fc(s, sent)
    fc.att.add(1e7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    fc.hgt.add(1e7, 50.0)
    return fc


class StubHold:
    def __init__(self):
        self.requests = 0

    def request_keyframe(self):
        self.requests += 1


class StubPipe(R.StubPipe):
    """R.StubPipe that records the stamps of the frames it got, with a Hold that counts keyframe requests."""
    steps = None

    def __init__(self, s, fc_pos):
        self.hold, self.t0s = StubHold(), []
        StubPipe.last = self

    def step(self, fr, att, hgt, gate=None):
        self.t0s.append(fr.t0)
        return super().step(fr, att, hgt, gate)


def run(s, n, modes, path, fc=None):
    fc = fc or fc_now(s)
    fc.firmware, fc.mode = "ardupilot", None
    src = ModeSource(R.frames(n), fc, modes)
    with R.patched(apb_main, Pipe=StubPipe, start_problems=lambda *a: []):
        apb_main.run_live(s, src, fc, seconds=n / 60 + 0.3, log_path=path)
    return list(csv.DictReader(open(path))), StubPipe.last


# ---------------------------------------------------------------- E3
def e3_switch(d):
    print("E3: the pilot's switch: ALT_HOLD, then LOITER, then ALT_HOLD again")
    for fresh in (False, True):
        s = T.digital_settings(log_dir=d, fresh_kf_at_engage=fresh)
        rows, pipe = run(s, 60, {0: 2, 20: 5, 40: 2}, os.path.join(d, f"e3_{fresh}.csv"))
        notes = [r["note"] for r in rows if r["note"]]
        modes = [r["mode"] for r in rows if r["t"]]
        want = ["FC mode ALT_HOLD", "engaged: FC mode LOITER" + ("; fresh keyframe requested" if fresh else ""),
                "released: FC mode ALT_HOLD"]
        check([x for x in notes if "FC mode" in x] == want and "mode" in rows[0] and set(modes) == {"ALT_HOLD", "LOITER"}
              and pipe.hold.requests == int(fresh),
              f"fresh_kf_at_engage {fresh}: notes {notes}; modes {sorted(set(modes))}; keyframe requests {pipe.hold.requests}")


# ---------------------------------------------------------------- E4
def e4_outage(d):
    print("E4: the recovery demonstration drops the video and the run goes on")
    s = T.digital_settings(log_dir=d, outage_test_s=0.3, outage_after_s=0.2)
    rows, pipe = run(s, 90, {0: 2, 10: 5}, os.path.join(d, "e4a.csv"))
    dropped = [r for r in rows if r["why"] == "recovery test: video dropped"]
    notes = [r["note"] for r in rows if "recovery test" in r["note"]]
    t_eng = next(float(r["t"]) for r in rows if "engaged" in r["note"])
    first = float(dropped[0]["t"]) if dropped else math.nan
    check(12 <= len(dropped) <= 24 and notes == ["recovery test: video dropped for 0.3 s", "recovery test: video back"]
          and 0.15 <= first - t_eng <= 0.3 and len(pipe.t0s) + len(dropped) == 89     # run_live takes frame 1 for its checks
          and not set(pipe.t0s) & {float(r["t"]) for r in dropped} and "engaged" not in notes,
          f"after the switch: {len(dropped)} frames dropped, {first - t_eng:.2f} s after it; notes {notes}; "
          f"{len(pipe.t0s)} frames went to Hold")
    trig = os.path.join(d, apb_main.OutageTest.FILE)
    open(trig, "w").close()
    rows, pipe = run(s, 60, {0: 2}, os.path.join(d, "e4b.csv"))
    dropped = [r for r in rows if r["why"] == "recovery test: video dropped"]
    check(12 <= len(dropped) <= 24 and not os.path.exists(trig) and float(dropped[0]["t"]) - float(rows[0]["t"] or 0) < 0.1,
          f"trigger file: {len(dropped)} frames dropped from the start, file deleted: {not os.path.exists(trig)}")
    open(trig, "w").close()
    rows, pipe = run(dict(s, outage_test_s=0.0), 60, {0: 2, 10: 5}, os.path.join(d, "e4c.csv"))
    dropped = [r for r in rows if r["why"] == "recovery test: video dropped"]
    check(not dropped and os.path.exists(trig) and len(pipe.t0s) == 59,
          f"outage_test_s 0 (the default): {len(dropped)} frames dropped, the file is left alone, {len(pipe.t0s)} frames to Hold")
    os.remove(trig)


# ---------------------------------------------------------------- E5, E6 (the real Hold)
def still(h=50.0):
    att, hgt = Series(1e9), Series(1e9)
    for t in np.arange(-1, 60, 0.01):
        att.add(t, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0)
    for t in np.arange(-1, 60, 0.1):
        hgt.add(t, h)
    return att, hgt


def hover(hold, base, rng, n, t_start, att, hgt, offset=lambda i: 0.0, on=None):
    """n frames of a still hover at 60 fps; offset(i) adds grey. on(i, est) sees each result. Noise 4 grey: with
    1 grey, two frames differ by less than Hold.REPEAT_DIFF after the 2x shrink and Hold refuses them as repeats."""
    for i in range(n):
        img = capture.shrink(T.noisy(base + offset(i), rng, sd=4.0), 2)
        e = hold.process(img, t_start + i / 60, att, hgt)
        if on:
            on(i, e)


def e5_request(d):
    print("E5: Hold.request_keyframe in a still hover at 50 m")
    s = T.digital_settings(lens_name=LENS_D)
    hold = apb_main.make_hold(s, (T.H // 2, T.W // 2))
    rng, base, (att, hgt) = np.random.default_rng(3), T.scene(4), still()
    out = []
    hover(hold, base, rng, 120, 1.0, att, hgt, on=lambda i, e: out.append(e))
    kf0, home0, p0 = hold.kf, hold.home, out[-1].pos.copy()
    hold.request_keyframe()
    out2 = []
    hover(hold, base, rng, 30, 3.0, att, hgt, on=lambda i, e: out2.append(e))
    good = [e for e in out2 if e.var is not None]
    jump = max(float(np.linalg.norm(e.pos - p0)) for e in good) if good else math.inf
    check(sum(e.var is not None for e in out) > 60 and kf0 is home0 and hold.kf is not kf0 and hold.home is hold.kf
          and home0 in hold.kfs and not hold.kf_wanted and len(good) >= 25 and jump < 0.05,
          f"new keyframe {hold.kf is not kf0}, it is home {hold.home is hold.kf}, old home kept {home0 in hold.kfs}; "
          f"{len(good)} of 30 fixes after the request; largest move {jump:.3f} m")


def e6_bright(d):
    print("E6: KF_BRIGHT 2 grey: a keyframe asked in the middle of an 8 grey brightness rise (1 s) waits for the end")
    rng, base, (att, hgt) = np.random.default_rng(5), T.scene(6), still()
    for kb in (0.0, 2.0):
        s = T.digital_settings(lens_name=LENS_D, hold_cfg={"kf_bright": kb})
        hold = apb_main.make_hold(s, (T.H // 2, T.W // 2))
        hover(hold, base, rng, 90, 1.0, att, hgt)
        kf0, seen = hold.kf, {}

        def on(i, e):
            if i == 29:
                hold.request_keyframe()            # after frame 29: 4 grey of the rise are past
            if hold.kf is not kf0 and "i" not in seen:
                seen["i"] = i
            seen["fix"] = seen.get("fix", 0) + (e.var is not None)
        # frames 0-59: +8 grey ramp; then steady at +8 for 60 frames
        hover(hold, base, rng, 120, 2.5, att, hgt, offset=lambda i: 8.0 * min(i, 60) / 60, on=on)
        i = seen.get("i")
        # The gate opens when the last 0.5 s span 2 grey or less: 0.25 s after the end of an 8 grey/s rise (frame 75)
        ok = (i == 30) if kb == 0 else (i is not None and 60 + 0.2 * 60 <= i <= 60 + 0.5 * 60)
        check(ok and seen.get("fix", 0) >= 100,
              f"kf_bright {kb:g}: the new keyframe came at frame {i} (ramp ends at frame 60); {seen.get('fix', 0)} of 120 fixes")


# ---------------------------------------------------------------- E7
class SendPipe(StubPipe):
    send = True


def e7_summary(d):
    print("E7: tools/log_summary.py on a run with a switch and a 0.3 s recovery test")
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))
    import log_summary
    s = T.digital_settings(log_dir=d, outage_test_s=0.3, outage_after_s=2.5)    # the drop starts after the 2 s window
    path = os.path.join(d, "e7.csv")
    with R.patched(apb_main, Pipe=SendPipe, start_problems=lambda *a: []):
        fc = fc_now(s, sent=[])
        fc.firmware, fc.mode = "ardupilot", None
        apb_main.run_live(s, ModeSource(R.frames(400), fc, {0: 2, 150: 5}), fc, seconds=400 / 60 + 0.3, log_path=path)
    r = log_summary.summary(path)
    (before, after, gap), = r["switch"] or [(0, 0, 0)]
    # send_hz 50 against 60 fps frames: about every other frame is sent (about 30 each second). The fix age is the
    # 40 ms stamp offset of ModeSource plus the loop time: 30 to 200 ms (a clock mix-up gives hours).
    ok = (len(r["switch"]) == 1 and before >= 40 and after >= 25 and gap < 0.1 and len(r["recover"]) == 1
          and r["recover"][0] is not None and r["recover"][0] < 0.1 and 0.25 <= r["gap"] <= 0.4 and r["sent_per_s"] > 20
          and r["age_p50"] is not None and 30 <= r["age_p50"] <= r["age_p95"] <= 200)
    check(ok, f"switch (sent before, after, gap s) {r['switch']}; recovery: first fix {r['recover']} s after 'video back'; "
              f"longest gap {r['gap']:.2f} s; {r['sent_per_s']:.0f} fixes/s; fix age p50 {r['age_p50']:.0f} ms, "
              f"p95 {r['age_p95']:.0f} ms")
    import contextlib
    import io
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = log_summary.main([path, path, "--csv", os.path.join(d, "e7_table.csv")])
    out = buf.getvalue().splitlines()
    check(rc == 0 and len(out) >= 4 and any(x.startswith("sent/s: median") for x in out) and not any("WARN" in x for x in out),
          f"the command line prints a table and the spread over 2 logs, no warning: {out[-3] if len(out) >= 3 else out}")
    bad = os.path.join(d, "e7_bad.csv")                     # a log whose ages mix two clocks
    rows = list(csv.DictReader(open(path)))
    with open(bad, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows([dict(x, age_ms="22000000.0") if x["age_ms"] else x for x in rows])
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        log_summary.main([bad])
    warn = [x for x in buf.getvalue().splitlines() if x.startswith("WARN")]
    check(len(warn) == 1, f"a log with fix ages of hours gets a warning: {warn}")


# ---------------------------------------------------------------- E8
def e8_gap(d):
    print("E8: GAP_JUMP_S: the first picture after a 3 s gap, 10 grey brighter, in a still hover at 50 m")
    for gj in (0.0, 0.5):
        s = T.digital_settings(lens_name=LENS_D, hold_cfg={"gap_jump_s": gj})
        hold = apb_main.make_hold(s, (T.H // 2, T.W // 2))
        rng, base, (att, hgt) = np.random.default_rng(7), T.scene(8), still()
        hover(hold, base, rng, 90, 1.0, att, hgt)
        kf0, out = hold.kf, []
        hover(hold, base, rng, 30, 1.0 + 90 / 60 + 3.0, att, hgt, offset=lambda i: 10.0, on=lambda i, e: out.append(e))
        first = out[0]
        if gj == 0:
            ok = "broken up" in (first.why or "") and first.state == "lost"
        else:
            ok = "broken" not in (first.why or "") and not any(e.state == "lost" for e in out) and hold.kf is kf0 \
                and sum(e.var is not None for e in out) >= 20
        check(ok, f"gap_jump_s {gj:g}: first frame {first.state} ({first.why}); lost in 30 frames "
                  f"{any(e.state == 'lost' for e in out)}; same keyframe {hold.kf is kf0}; {sum(e.var is not None for e in out)} fixes")


def main():
    t0 = time.time()
    only = sys.argv[1:]
    with tempfile.TemporaryDirectory() as d:
        for name, fn in (("e1", lambda: e1_leaks(d)), ("e2", e2_modes), ("e3", lambda: e3_switch(d)),
                         ("e4", lambda: e4_outage(d)), ("e5", lambda: e5_request(d)), ("e6", lambda: e6_bright(d)),
                         ("e7", lambda: e7_summary(d)), ("e8", lambda: e8_gap(d))):
            if not only or name in only:
                try:
                    fn()
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    check(False, f"{name} raised {e!r}")
    print(f"{'ALL PASS' if not fails else f'{fails} FAIL'} in {time.time() - t0:.0f} s")
    return fails


if __name__ == "__main__":
    sys.exit(main())
