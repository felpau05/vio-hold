#!/usr/bin/env python3
"""GNSS levels (settings gnss_mode, apb_main.GnssLevel), with fake cameras and a stand-in flight controller: the real
fc_link.FC send methods write real MAVLink bytes into a buffer, and the test decodes them.
G1 the setting: default "none", other values refused; G2 gnss_mode "none" sends the same MAVLink messages and commands
as the flight loop before this change (integ/gnss_ladder/apb_main_before.py) and no MAV_CMD_SET_EKF_SOURCE_SET;
G3 "setup": source set 2 requested once fixes flow, before the pilot's switch, the FC reply in the run log; G4 a switch
before the program is ready gets a WARNING note; G5 "fallback": a video drop in the hold gives set 1 with a note, no
set 2 again in that hold, set 2 after the release; G6 an FC that denies or does not answer: one request each second;
G7 settings that do not fit the level, PX4, and the profiles equal apb_main.GNSS_AP; G8 tools/use_profile.py apply with
and without gnss_setup; G9 check mode prints a PASS or FAIL line for each parameter of the level.
Run: ~/.venvs/vio/bin/python tests_apb/test_gnss.py      (about 40 s)
"""
import collections
import contextlib
import csv
import importlib.util
import io
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

os.environ.setdefault("MAVLINK20", "1")
HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path[:0] = [PROJ, HERE]
from pymavlink.dialects.v20 import ardupilotmega as mavlink   # noqa: E402

import apb_main            # noqa: E402
import fc_link             # noqa: E402
import settings as S       # noqa: E402
import test_digital as T   # noqa: E402
import test_event as E     # noqa: E402  (ModeSource, fc_now's attitude and height)
import test_robust as R    # noqa: E402

CMD = mavlink.MAV_CMD_SET_EKF_SOURCE_SET
fails = 0


def check(ok, text):
    global fails
    fails += not ok
    print(f"  {'PASS' if ok else 'FAIL'} {text}", flush=True)


def before_module():
    """The flight loop as it was before gnss_mode (a copy kept in integ/gnss_ladder)."""
    p = os.path.join(PROJ, "integ", "gnss_ladder", "apb_main_before.py")
    spec = importlib.util.spec_from_file_location("apb_main_before", p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


class WireFC(fc_link.FC):
    """fc_link.FC with no link: every send method is the real one and writes MAVLink 2 bytes into self.wire. The FC
    'replies' to MAV_CMD_SET_EKF_SOURCE_SET with reply (None: no reply), as ArduPilot does (GCS_Common.cpp 4908)."""

    def __init__(self, s, reply=0):
        base = E.fc_now(s)                                         # level attitude, 50 m height
        self.s, self.att, self.hgt, self.lpos = s, base.att, base.hgt, base.lpos
        self.clock, self.firmware, self.sysid, self.compid = base.clock, "ardupilot", 1, 1
        self.acks, self.params, self.param_types, self.armed, self.mode = {}, {}, {}, False, None
        self.reset_counter, self._tx, self.reply, self.wire = 0, threading.Lock(), reply, bytearray()
        f = SimpleNamespace(write=self.wire.extend)
        self.conn = SimpleNamespace(mav=mavlink.MAVLink(f, srcSystem=1, srcComponent=197))
        self.rate, self.age, self.local_pos, self.close = (lambda *a: 100.0), (lambda *a: 0.0), (lambda: None), (lambda: None)

    def _send(self, fn):
        n = len(self.wire)
        with self._tx:
            fn(self.conn.mav)
        for m in decode(bytes(self.wire[n:])):
            if m.get_type() == "COMMAND_LONG" and m.command == CMD and self.reply is not None:
                self.acks[CMD] = self.reply

    def messages(self):
        return decode(bytes(self.wire))


def decode(b):
    p = mavlink.MAVLink(None)
    p.robust_parsing = True
    return [m for m in (p.parse_buffer(b) or []) if m.get_type() != "BAD_DATA"]


def kinds(msgs):
    return collections.Counter(m.get_type() + (f" {m.command}" if m.get_type() == "COMMAND_LONG" else "") for m in msgs)


class SendPipe(R.StubPipe):
    send = True


class StopPipe(SendPipe):
    """Sends fixes, except while StopPipe.off is set (a camera with no ground features)."""
    off = False

    def step(self, fr, att, hgt, gate=None):
        e, ok, why = super().step(fr, att, hgt, gate)
        return (e, False, "no fix (test)") if StopPipe.off else (e, ok, why)


def run(mod, s, n, modes, path, fc, pipe=SendPipe):
    with R.patched(mod, Pipe=pipe, start_problems=lambda *a: []):
        mod.run_live(s, E.ModeSource(R.frames(n), fc, modes), fc, seconds=n / 60 + 0.3, log_path=path)
    return list(csv.DictReader(open(path)))


def notes(rows):
    return [(float(r["t"]) if r["t"] else None, r["note"]) for r in rows if r["note"] and "VISO_DELAY" not in r["note"]]


# ---------------------------------------------------------------- G1
def g1_setting(d):
    print("G1: the gnss_mode setting")
    s = S.load("/nonexistent")
    p = os.path.join(d, "bad.json")
    json.dump({"gnss_mode": "gps"}, open(p, "w"))
    try:
        S.load(p)
        err = None
    except ValueError as e:
        err = str(e)
    check(s["gnss_mode"] == "none" and err is not None and "must be one of none, setup, fallback, velocity" in err,
          f"default {s['gnss_mode']!r}; 'gps' refused: {err}")


# ---------------------------------------------------------------- G2
def g2_same_as_before(d):
    print("G2: gnss_mode none: the same MAVLink messages and commands as before the change, no source-set command")
    old = before_module()
    out = {}
    for name, mod, mode in (("before", old, None), ("none", apb_main, "none"), ("setup", apb_main, "setup")):
        s = T.digital_settings(log_dir=d, gnss_ready_s=0.5, origin_lat_lon_alt=[45.4, -75.7, 70.0])
        if mode:
            s["gnss_mode"] = mode
        fc = WireFC(s)
        rows = run(mod, s, 150, {0: 2, 100: 5}, os.path.join(d, f"g2_{name}.csv"), fc)
        out[name] = (kinds(fc.messages()), rows)
    kb, kn, ks = out["before"][0], out["none"][0], out["setup"][0]
    same_kinds = set(kb) == set(kn)
    close = all(abs(kb[k] - kn[k]) <= max(3, 0.1 * kb[k]) for k in kb)      # fix count depends on the loop timing
    cols = list(out["before"][1][0]) == list(out["none"][1][0])
    nb = [x for _, x in notes(out["before"][1])]
    nn = [x for _, x in notes(out["none"][1])]
    check(same_kinds and close and f"COMMAND_LONG {CMD}" not in kn and cols and nb == nn,
          f"before {dict(kb)}; none {dict(kn)}; same log columns {cols}; same notes {nb == nn} ({nn})")
    check(ks[f"COMMAND_LONG {CMD}"] == 1 and set(ks) - set(kn) == {f"COMMAND_LONG {CMD}"},
          f"positive control, gnss_mode setup: the only new message is COMMAND_LONG {CMD}, {ks[f'COMMAND_LONG {CMD}']} time(s)")


# ---------------------------------------------------------------- G3, G4
def g3_setup(d):
    print("G3: gnss_mode setup: set 2 requested once fixes flow, before the pilot's switch, the FC reply logged")
    s = T.digital_settings(log_dir=d, gnss_mode="setup", gnss_ready_s=1.0)
    fc = WireFC(s)
    rows = run(apb_main, s, 180, {0: 2, 120: 5}, os.path.join(d, "g3.csv"), fc)
    ns = notes(rows)
    t_req = next((t for t, x in ns if "source set 2 (our position only) requested" in x), None)
    t_ack = next((t for t, x in ns if "source set 2 (our position only): ACCEPTED" in x), None)
    t_eng = next((t for t, x in ns if x.startswith("engaged")), None)
    cmds = [m for m in fc.messages() if m.get_type() == "COMMAND_LONG" and m.command == CMD]
    t0 = float(rows[0]["t"])
    ok = (None not in (t_req, t_ack, t_eng) and 0.9 <= t_req - t0 <= 1.4 and t_req <= t_ack < t_eng and len(cmds) == 1
          and cmds[0].param1 == 2 and not any("WARNING" in x for _, x in ns))
    check(ok, f"request {None if t_req is None else round(t_req - t0, 2)} s after the first fix, reply before the switch: "
              f"{t_ack is not None and t_eng is not None and t_ack < t_eng}; commands {[c.param1 for c in cmds]}; notes {[x for _, x in ns]}")


def g4_early_switch(d):
    print("G4: the pilot's switch comes before the program is ready: a WARNING note, set 2 then follows in the hold")
    s = T.digital_settings(log_dir=d, gnss_mode="setup", gnss_ready_s=1.0)
    fc = WireFC(s)
    rows = run(apb_main, s, 150, {0: 2, 20: 5}, os.path.join(d, "g4.csv"), fc)
    ns = [x for _, x in notes(rows)]
    w = [i for i, x in enumerate(ns) if "WARNING: the hold started before the FC accepted EKF source set 2" in x]
    r = [i for i, x in enumerate(ns) if "set 2 (our position only) requested: fixes for" in x and "in a hold" in x]
    check(len(w) == 1 and len(r) == 1 and w[0] <= r[0], f"notes {ns}")


# ---------------------------------------------------------------- G5
def g5_fallback(d):
    print("G5: gnss_mode fallback: no fix for gnss_fallback_s in the hold -> set 1 (GNSS), kept until the release")
    s = T.digital_settings(log_dir=d, gnss_mode="fallback", gnss_ready_s=0.5, gnss_fallback_s=0.5,
                           outage_test_s=1.2, outage_after_s=0.5)
    fc = WireFC(s)
    rows = run(apb_main, s, 330, {0: 2, 60: 5, 240: 2}, os.path.join(d, "g5.csv"), fc)
    ns = [x for _, x in notes(rows)]
    seq = [m.param1 for m in fc.messages() if m.get_type() == "COMMAND_LONG" and m.command == CMD]
    i_fb = next((i for i, x in enumerate(ns) if "GNSS FALLBACK: no fix for" in x), None)
    i_back = next((i for i, x in enumerate(ns) if "video back" in x), None)
    i_rel = next((i for i, x in enumerate(ns) if x.startswith("released")), None)
    later2 = [i for i, x in enumerate(ns) if "set 2 (our position only) requested" in x and i_fb is not None and i > i_fb]
    ok = (seq == [2, 1, 2] and None not in (i_fb, i_back, i_rel) and i_fb < i_back < i_rel and len(later2) == 1
          and later2[0] >= i_rel and any("source set 1 (GNSS): ACCEPTED" in x for x in ns))
    check(ok, f"commands {seq} (want 2, 1, 2); fallback before video back and set 2 only after the release: {ok}; notes {ns}")


# ---------------------------------------------------------------- G6
def g6_no_reply(d):
    print("G6: an FC that denies the command or does not answer: at most one request each second, each logged")
    for reply, want in ((2, "DENIED"), (None, "no FC reply")):
        s = T.digital_settings(log_dir=d, gnss_mode="setup", gnss_ready_s=0.3)
        fc = WireFC(s, reply=reply)
        rows = run(apb_main, s, 240, {0: 2}, os.path.join(d, f"g6_{reply}.csv"), fc)
        ns = [x for _, x in notes(rows)]
        n = sum(1 for m in fc.messages() if m.get_type() == "COMMAND_LONG" and m.command == CMD)
        # 4 s of frames, ready after 0.3 s: about 3 requests (one per second), never one per frame
        check(2 <= n <= 5 and any(want in x for x in ns), f"reply {want}: {n} requests in 4 s; notes {ns[:4]}")


# ---------------------------------------------------------------- G7
def g7_problems(d):
    print("G7: settings that do not fit the level; PX4; the profiles equal apb_main.GNSS_AP")
    s = S.load("/nonexistent")
    a = apb_main.gnss_problems(s, "ardupilot")
    b = apb_main.gnss_problems(dict(s, gnss_mode="setup"), "ardupilot")
    c = apb_main.gnss_problems(dict(s, gnss_mode="setup", params_ardupilot=dict(s["params_ardupilot"], **apb_main.GNSS_AP["setup"])),
                               "ardupilot")
    p = apb_main.gnss_problems(dict(s, gnss_mode="velocity"), "px4")
    check(a == [] and len(b) == 1 and "EK3_SRC2_POSXY None (need 6)" in b[0] and c == [] and len(p) == 1 and "needs ArduPilot EKF source sets" in p[0],
          f"none: {a}; setup without the profile: {b}; with it: {c}; PX4: {p}")
    same = {}
    for lvl in ("setup", "fallback", "velocity"):
        o = json.load(open(os.path.join(PROJ, "tools", "profiles", f"gnss_{lvl}.json")))
        same[lvl] = o.get("gnss_mode") == lvl and o.get("params_ardupilot") == apb_main.GNSS_AP[lvl] and \
            set(o) == {"_why", "gnss_mode", "params_ardupilot"}
    check(all(same.values()), f"each profile owns gnss_mode and the level's params_ardupilot keys only: {same}")
    fc = R.fake_fc(T.digital_settings())
    fc.firmware = "ardupilot"
    probs = apb_main.start_problems(T.digital_settings(calibrated=True, calib_capture="digital 640x360", mask_file="",
                                                       origin_lat_lon_alt=[1, 2, 3], gnss_mode="fallback"),
                                    fc, SimpleNamespace(info={}), (360, 640))
    check(any("gnss_mode fallback needs other params_ardupilot values" in x for x in probs), f"run refuses it: {probs}")


# ---------------------------------------------------------------- G8
def g8_profiles(d):
    print("G8: tools/use_profile.py apply ... gnss_setup, then apply without it")
    path = os.path.join(d, "settings.json")
    S.save(S.load(os.path.join(PROJ, "settings.json")), path)
    tool = [sys.executable, os.path.join(PROJ, "tools", "use_profile.py"), "--settings", path, "apply"]
    r0 = subprocess.run(tool + ["outdoor_day", "justice_d"], capture_output=True, text=True)
    base = S.load(path)
    res = {}
    for lvl in ("setup", "fallback", "velocity"):
        r1 = subprocess.run(tool + ["outdoor_day", "justice_d", f"gnss_{lvl}"], capture_output=True, text=True)
        on = S.load(path)
        r2 = subprocess.run(tool + ["outdoor_day", "justice_d"], capture_output=True, text=True)
        off = S.load(path)
        res[lvl] = (r1.returncode == 0 and on["gnss_mode"] == lvl and apb_main.gnss_problems(on, "ardupilot") == []
                    and on["params_ardupilot"]["VISO_DELAY_MS"] == base["params_ardupilot"]["VISO_DELAY_MS"]
                    and r2.returncode == 0 and off == base)
    check(r0.returncode == 0 and all(res.values()), f"apply with the level fits it, apply without it gives the same file "
                                                    f"as before: {res}")
    print("    " + "\n    ".join(r1.stdout.strip().splitlines()[:16]))


# ---------------------------------------------------------------- G9
def g9_check(d):
    print("G9: check mode: a PASS or FAIL line for each parameter of the level")
    s = T.digital_settings(log_dir=d, gnss_mode="setup",
                           params_ardupilot=dict(S.DEFAULTS["params_ardupilot"], **apb_main.GNSS_AP["setup"]))
    fcp = dict(s["params_ardupilot"], GPS_TYPE=1, EK3_SRC2_POSXY=0)                       # one wrong value
    fcp.pop("GPS1_TYPE")                                                                 # older name on this FC
    fc = SimpleNamespace(firmware="ardupilot", sysid=1, rate=lambda *a: 100.0, clock=SimpleNamespace(ready=lambda: True),
                         get_param=lambda n, timeout=1.5: fcp.get(n), hgt=SimpleNamespace(rows=[(0, 0.0)]), close=lambda: None)
    out = []
    with R.patched(apb_main, open_fc=lambda s, say=print: fc, open_video=lambda s: (_ for _ in ()).throw(RuntimeError("no camera (test)"))):
        with contextlib.redirect_stdout(io.StringIO()):
            apb_main.mode_check(s, SimpleNamespace(set_params=False), say=out.append)
    lines = {k: [x for x in out if f"param {k}" in x or (k == "GPS1_TYPE" and "param GPS_TYPE" in x)] for k in apb_main.GNSS_AP["setup"]}
    one_each = all(len(v) == 1 for v in lines.values())
    bad = [x for v in lines.values() for x in v if x.startswith("FAIL")]
    level = [x for x in out if "gnss_mode setup" in x]
    check(one_each and bad == ["FAIL param EK3_SRC2_POSXY = 0; need 6. Run 'apb_main.py check --set-params', then reboot the FC."]
          and level == ["PASS gnss_mode setup: settings match the level"],
          f"{sum(len(v) for v in lines.values())} lines for {len(lines)} parameters; FAIL lines {bad}; level line {level}")


def main():
    t0 = time.time()
    only = sys.argv[1:]
    with tempfile.TemporaryDirectory() as d:
        for name, fn in (("g1", g1_setting), ("g2", g2_same_as_before), ("g3", g3_setup), ("g4", g4_early_switch),
                         ("g5", g5_fallback), ("g6", g6_no_reply), ("g7", g7_problems), ("g8", g8_profiles), ("g9", g9_check)):
            if not only or name in only:
                try:
                    fn(d)
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    check(False, f"{name} raised {e!r}")
    print(f"{'ALL PASS' if not fails else f'{fails} FAIL'} in {time.time() - t0:.0f} s")
    return fails


if __name__ == "__main__":
    sys.exit(main())
