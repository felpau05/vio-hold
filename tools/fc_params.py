#!/usr/bin/env python3
"""Save the flight controller (FC) parameters that our program changes, and put them back.

The aircraft belongs to the organizers. "apb_main.py check --set-params" writes our values into its FC (for
example GPS1_TYPE 0). Save the aircraft's own values BEFORE that step. Restore them when our tests are done.

  python3 tools/fc_params.py save      read each parameter that our settings name, write <log_dir>/fc_params_before.json
  python3 tools/fc_params.py restore   write the saved values back and read each one again
  python3 tools/fc_params.py selftest  test this tool with a stand-in FC (no hardware)
  --settings PATH                      another settings file than settings.json (as apb_main.py)

Run it on the board, propellers off, with the program stopped (tools/deploy.sh stop): the FC link has one user.
save never overwrites its file: the first save holds the aircraft's own values. A later save reads nothing and shows
PASS, so the checklist step can be repeated. Reboot the FC after restore.
"""
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
FILE = "fc_params_before.json"


# Each parameter that a GNSS level can write (apb_main.GNSS_AP; selftest compares the two lists). save reads them
# also when no level is chosen yet, so a level that the team adds later in the day can be put back too.
GNSS_NAMES = ["GPS1_TYPE", "EK3_SRC1_POSXY", "EK3_SRC1_VELXY", "EK3_SRC1_POSZ", "EK3_SRC1_VELZ", "EK3_SRC1_YAW",
              "EK3_SRC2_POSXY", "EK3_SRC2_VELXY", "EK3_SRC2_POSZ", "EK3_SRC2_VELZ", "EK3_SRC2_YAW", "EK3_SRC_OPTIONS"]


def names(s, fc):
    if fc.firmware == "px4":
        return list(s["params_px4"])
    return list(dict.fromkeys(list(s["params_ardupilot"]) + GNSS_NAMES))


def read(fc, name, alias):
    """(name the firmware answered to, value). (name, None) when the firmware has no such parameter."""
    got = fc.get_param(name)
    if got is None and name in alias:
        name, got = alias[name], fc.get_param(alias[name])
    return name, got


def save(s, fc, alias, say=print):
    path = os.path.join(s["log_dir"], FILE)
    if os.path.exists(path):
        old = json.load(open(path))
        say(f"PASS the aircraft's own values are saved already ({old.get('saved')}, {len(old['values'])} parameters). "
            f"Nothing read, nothing changed. Keep {path}. Do not rename or delete it.")
        return 0
    values = {}
    for want in names(s, fc):
        name, got = read(fc, want, alias)
        values[name] = got
        say(f"INFO {name} = {got}" if got is not None else f"INFO {name}: no answer. This firmware does not have it.")
    os.makedirs(s["log_dir"], exist_ok=True)
    with open(path, "w") as f:
        json.dump({"firmware": fc.firmware, "saved": time.strftime("%Y-%m-%d %H:%M:%S"), "values": values}, f, indent=1)
    say(f"PASS saved {sum(v is not None for v in values.values())} of {len(values)} parameters to {path}")
    return 0


def restore(s, fc, say=print):
    path = os.path.join(s["log_dir"], FILE)
    if not os.path.exists(path):
        say(f"FAIL {path} is missing. There is no saved value to restore. Ask the crew for the aircraft's parameter file.")
        return 1
    saved = json.load(open(path))
    if saved["firmware"] != fc.firmware:
        say(f"FAIL the file is from firmware {saved['firmware']}, and this FC runs {fc.firmware}. Nothing written.")
        return 1
    fails = 0
    for name, want in saved["values"].items():
        if want is None:
            say(f"INFO {name}: not saved (the firmware did not have it). Skipped.")
            continue
        got = fc.get_param(name)
        if got is not None and abs(got - want) <= 1e-3 * max(1.0, abs(want)):
            say(f"PASS {name} = {got} (already the saved value)")
            continue
        got = fc.set_param(name, want)
        ok = got is not None and abs(got - want) <= 1e-3 * max(1.0, abs(want))
        fails += not ok
        say(f"{'PASS' if ok else 'FAIL'} {name}: wrote {want}, reads {got}."
            + ("" if ok else " Write refused. Set it in the ground station."))
    say("ALL RESTORED. Reboot the flight controller." if not fails else f"{fails} FAIL. Tell the crew which parameters differ.")
    return 1 if fails else 0


def selftest():
    import tempfile

    class Fake:
        firmware = "ardupilot"

        def __init__(self):
            self.p = {"GPS_TYPE": 1.0, "EK3_SRC1_POSXY": 3.0, "VISO_TYPE": 0.0, "EK3_SRC2_POSXY": 0.0}   # the aircraft's own

        def get_param(self, name):
            return self.p.get(name)

        def set_param(self, name, value):
            self.p[name] = float(value)
            return self.p[name]

    with tempfile.TemporaryDirectory() as d:
        s = {"log_dir": d, "params_ardupilot": {"GPS1_TYPE": 0, "EK3_SRC1_POSXY": 6, "VISO_TYPE": 1, "NOT_THERE": 5}}
        fc, out, alias = Fake(), [], {"GPS1_TYPE": "GPS_TYPE"}
        assert save(s, fc, alias, out.append) == 0
        own = dict(fc.p)
        before = open(os.path.join(d, FILE)).read()
        fc.p.update(GPS_TYPE=0.0, EK3_SRC1_POSXY=6.0, VISO_TYPE=1.0)                 # what --set-params does
        assert save(s, fc, alias, out.append) == 0 and "saved already" in out[-1], out[-1]
        assert open(os.path.join(d, FILE)).read() == before, "a second save must not change the first"
        fc.p.update(EK3_SRC2_POSXY=6.0)               # a GNSS level added later: its parameter was saved at the start
        assert restore(s, fc, out.append) == 0 and fc.p == own, fc.p
        assert any("NOT_THERE: not saved" in line for line in out)
        fc.firmware = "px4"
        assert restore(s, fc, out.append) == 1, "another firmware must write nothing"
    try:
        import apb_main
        want = sorted({k for level in apb_main.GNSS_AP.values() for k in level})
        assert sorted(GNSS_NAMES) == want, (sorted(GNSS_NAMES), want)
    except ImportError as e:
        print(f"INFO GNSS_NAMES not compared with apb_main ({e})")
    print("selftest ALL PASSED")
    return 0


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "selftest":
        return selftest()
    if cmd not in ("save", "restore"):
        print(__doc__)
        return 1
    import fc_link
    import settings
    s = settings.load(sys.argv[sys.argv.index("--settings") + 1]) if "--settings" in sys.argv else settings.load()
    fc, why = fc_link.connect(s)
    if fc is None:
        print(f"FAIL flight controller: {why}")
        return 1
    try:
        return save(s, fc, fc_link.PARAM_ALIAS) if cmd == "save" else restore(s, fc)
    finally:
        fc.close()


if __name__ == "__main__":
    sys.exit(main())
