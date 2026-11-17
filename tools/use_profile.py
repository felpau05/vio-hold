#!/usr/bin/env python3
"""Settings profiles: small JSON overlays in tools/profiles/, merged onto settings.json.

  use_profile.py apply indoor goal_b ardupilot   reset every value that any overlay owns, merge these overlays
                                             in this order, check with settings.py's loader, write settings.json
  use_profile.py merge PATCH.json                merge one patch (for example from replay_tune.py) into settings.json
  use_profile.py show                            which overlays are active, and every value that is not the default
  use_profile.py diff A.json B.json              compare two settings files ("defaults" = settings.py's DEFAULTS)
  --settings PATH                            another settings file than settings.json

An overlay holds settings.py keys only. A key that starts with "_" is a note ("_why") and is never written.
For hold_cfg, params_px4 and params_ardupilot an overlay owns only the inner keys it names.
Calibration values (calib.py), the mask and the field origin are never in an overlay, so apply keeps them,
and it keeps every value no overlay names (for example a VISO_DELAY_MS set by hand).
The old file is kept as settings.json.bak; settings.json.profiles names the last apply.
"""
import argparse
import copy
import glob
import json
import os
import re
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import settings as S            # noqa: E402

PROFILES = os.path.join(HERE, "profiles")
CALIB_KEYS = {"top", "mirror", "tilt_deg", "twist_deg", "focal_scale", "readout_s", "video_delay_s", "calibrated",
              "calib_capture", "calib_info", "mask_file", "origin_lat_lon_alt"}
DICTS = {k for k, v in S.DEFAULTS.items() if isinstance(v, dict)}


def strip(d):
    return {k: strip(v) if isinstance(v, dict) else v for k, v in d.items() if not k.startswith("_")}


def leaves(d):
    """{"key": v} or {"hold_cfg.h_min": v}: one entry per value an overlay or a file sets."""
    out = {}
    for k, v in d.items():
        if k in DICTS:
            out.update({f"{k}.{kk}": vv for kk, vv in v.items()})
        else:
            out[k] = v
    return out


def check_keys(d, where, overlay):
    for k, v in d.items():
        if k not in S.DEFAULTS:
            raise ValueError(f"{where}: unknown key '{k}'. Fix the spelling or remove it.")
        if overlay and k in CALIB_KEYS:
            raise ValueError(f"{where}: '{k}' comes from calib, mask or the field, not from a profile.")
        if k in DICTS and not isinstance(v, dict):
            raise ValueError(f"{where}: '{k}' must be an object.")
    if "hold_cfg" in d:               # Hold's tuning names, read from hold.py's text: no numpy or cv2 needed here
        with open(os.path.join(os.path.dirname(HERE), "hold.py")) as f:
            body = f.read().split("\nclass Hold", 1)[1].split("\n    def ", 1)[0]
        names = set(re.findall(r"^    ([A-Z][A-Z0-9_]*) *=", body, re.M))
        bad = [k for k in d["hold_cfg"] if k.upper() not in names]
        if bad:
            raise ValueError(f"{where}: hold_cfg has unknown Hold setting(s) {bad}.")


def read(path, overlay=False):
    with open(path) as f:
        d = strip(json.load(f))
    check_keys(d, path, overlay)
    return d


def overlay_path(name):
    p = os.path.join(PROFILES, name + ".json")
    if not os.path.exists(p):
        names = sorted(os.path.basename(x)[:-5] for x in glob.glob(os.path.join(PROFILES, "*.json")))
        raise ValueError(f"no profile '{name}'. Profiles: {', '.join(names)}")
    return p


def merge(s, d):
    for k, v in d.items():
        s[k] = {**s[k], **v} if k in DICTS else copy.deepcopy(v)
    return s


def load_any(path):
    return S.load("/nonexistent") if path == "defaults" else S.load(path)


def write(s, path):
    """Check with settings.py's own loader and Hold's names, then write. Returns the old settings."""
    check_keys(s, "result", False)
    fd, tmp = tempfile.mkstemp(suffix=".json", dir=os.path.dirname(os.path.abspath(path)))
    os.close(fd)
    try:
        S.save(s, tmp)
        S.load(tmp)                                   # the program's own check: types, choices
    except ValueError as e:
        raise ValueError(str(e).replace(tmp, "the merged settings")) from None
    finally:
        os.remove(tmp)
    old = S.load(path)
    if os.path.exists(path):
        shutil.copyfile(path, path + ".bak")
    S.save(s, path)
    return old


def print_diff(a, b, la="before", lb="after"):
    fa, fb = leaves(a), leaves(b)
    n = 0
    for k in sorted(set(fa) | set(fb)):
        if fa.get(k, "(none)") != fb.get(k, "(none)"):
            print(f"  {k}: {la} {json.dumps(fa.get(k, '(none)'))} -> {lb} {json.dumps(fb.get(k, '(none)'))}")
            n += 1
    if not n:
        print("  no difference")


def cmd_apply(names, path):
    s = S.load(path)
    owned = {}
    for p in sorted(glob.glob(os.path.join(PROFILES, "*.json"))):
        owned.update(leaves(read(p, overlay=True)))
    for key in owned:                                 # back to the default first: no value is left over
        k, _, sub = key.partition(".")
        if not sub:
            s[k] = copy.deepcopy(S.DEFAULTS[k])
        elif sub in S.DEFAULTS[k]:
            s[k][sub] = S.DEFAULTS[k][sub]
        else:
            s[k].pop(sub, None)
    for name in names:
        merge(s, read(overlay_path(name), overlay=True))
    old = write(s, path)
    with open(path + ".profiles", "w") as f:          # for show: what the last apply named
        f.write(" + ".join(names) + "\n")
    print(f"applied {' + '.join(names)} -> {path}")
    print_diff(old, s)


def cmd_merge(patch, path):
    s = S.load(path)
    merge(s, read(patch))
    old = write(s, path)
    print(f"merged {patch} -> {path}")
    print_diff(old, s)


def cmd_show(path):
    s = S.load(path)
    fs = leaves(s)
    print(f"{path}")
    if os.path.exists(path + ".profiles"):
        with open(path + ".profiles") as f:
            print(f"last apply: {f.read().strip()}")
    print("profiles (ACTIVE = every value it sets is in the file now):")
    for p in sorted(glob.glob(os.path.join(PROFILES, "*.json"))):
        with open(p) as f:
            why = json.load(f).get("_why", {})
        o = leaves(read(p, overlay=True))
        hit = sum(fs.get(k, "(none)") == v for k, v in o.items())
        state = "notes only" if not o else "ACTIVE" if hit == len(o) else f"partly ({hit} of {len(o)})" if hit else "-"
        print(f"  {os.path.basename(p)[:-5]:14s} {state}")
        if state != "-":
            for k, v in o.items():
                print(f"      {k} = {json.dumps(v)}   ({why.get(k, '')})")
    print("values that differ from settings.py's defaults:")
    print_diff(S.DEFAULTS, s, "default", "now")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("cmd", choices=["apply", "merge", "show", "diff"])
    p.add_argument("args", nargs="*")
    p.add_argument("--settings", default=S.PATH)
    a = p.parse_args(argv)
    try:
        if a.cmd == "apply" and a.args:
            cmd_apply(a.args, a.settings)
        elif a.cmd == "merge" and len(a.args) == 1:
            cmd_merge(a.args[0], a.settings)
        elif a.cmd == "show" and not a.args:
            cmd_show(a.settings)
        elif a.cmd == "diff" and len(a.args) == 2:
            print(f"{a.args[0]} -> {a.args[1]}")
            print_diff(load_any(a.args[0]), load_any(a.args[1]), "A", "B")
        else:
            p.error("wrong arguments; see the help")
    except (ValueError, OSError, json.JSONDecodeError) as e:
        print(f"FAIL {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
