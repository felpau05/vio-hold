#!/bin/sh
# Defect: "apb_main.py bench" (also mask and calib) prints a Python traceback, not one FAIL line, when the FC does not
# answer or run_live refuses to start ("will not start: not calibrated ..."). EVENT_DAY says every check prints one
# line: PASS or FAIL. Seen in the rehearsal of EVENT_DAY step 6 (steps_log.txt). Here: no FC on the port.
# Patch: patches/apb_main_fail_line_not_traceback.patch (main() turns a RuntimeError into "FAIL <reason>").
cd "$(dirname "$0")/../.." || exit 1
T=$(mktemp -d)
~/.venvs/vio/bin/python -c "import settings as S; s = S.load('/nonexistent'); s.update(camera='digital', fc_device='udpin:127.0.0.1:14599', pixel_format='GREY', video_command='$HOME/.venvs/vio/bin/python integ/rehearsal_2026-11-20/cam_stream.py integ/rehearsal_2026-11-20/runs/jd_30m/raw_sim_000.npz'); S.save(s, '$T/s.json')"
~/.venvs/vio/bin/python apb_main.py bench --settings "$T/s.json" 2>&1 | tail -4
echo "exit code $?"
rm -rf "$T"
