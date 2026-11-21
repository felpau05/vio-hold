#!/usr/bin/env python3
"""Defect: the check line "free disk" and the raw recorder use min_free_gb in two ways.
mode_check scales it from 360x288 at 25 Hz to the real picture size and rate; Raw.flush stops recording under the
unscaled value. With the no_sd profile (record_hz 10, min_free_gb 0.5) check passes at 0.44 GB free, but the recorder
stops at once under 0.5 GB. This reads both numbers from apb_main.py's own text and evaluates them.
Patch: patches/apb_main_free_disk_floor.patch (check needs at least the recorder's stop level).
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, PROJ)
sys.path.insert(0, os.path.join(PROJ, "tools"))
import settings as S        # noqa: E402
import use_profile as U     # noqa: E402

src = open(os.path.join(PROJ, "apb_main.py")).read()
need_expr = re.search(r'^    need = (.*)$', src, re.M).group(1)            # mode_check
stop_expr = re.search(r'if free < (self\.min_gb):', src).group(1)          # Raw.flush (min_gb = s["min_free_gb"])
s = S.load("/nonexistent")
for name in ("justice_d", "no_sd"):
    U.merge(s, U.read(U.overlay_path(name), overlay=True))
    shape = (s["video_size"][1] // s["video_reduce"], s["video_size"][0] // s["video_reduce"])
    rate = min(s["video_fps"], s["record_hz"]) if s["record_hz"] > 0 else s["video_fps"]
    mbs = shape[0] * shape[1] * rate / 1e6
    need = eval(need_expr, {"s": s, "mbs": mbs, "max": max})
    print(f"after {name}: record_hz {s['record_hz']}, min_free_gb {s['min_free_gb']}: check needs {need:.2f} GB "
          f"({need_expr}); the recorder stops under {s['min_free_gb']} GB ({stop_expr})"
          + ("  <- check passes where the recorder does not record" if need < s["min_free_gb"] else ""))
