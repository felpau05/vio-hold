#!/bin/sh
# Defect: tools/profiles/betaflight.json (new, 2026-11-20 03:50) names keys that settings.py does not have yet
# (msp_lock_gain; its own note says betasim/patches add them). use_profile.py reads every profile on each apply and
# show, so every apply and show stops with "FAIL ... unknown key 'msp_lock_gain'". EVENT_DAY step 0 cannot run.
# Full list: profile_combos_after_betaflight_json.txt (21 of 21 combinations fail). tests_apb/test_digital: 3 FAIL.
# Patch: patches/profiles_betaflight_json_out_of_tools.patch (take the file out of tools/profiles until the settings
# patch is applied; or apply betasim/patches first).
cd "$(dirname "$0")/../.." || exit 1
T=$(mktemp -d); cp settings.json "$T/s.json"
python3 tools/use_profile.py apply outdoor_day goal_a ardupilot justice_d --settings "$T/s.json"; echo "apply exit $?"
python3 tools/use_profile.py show --settings "$T/s.json" | tail -1; echo "show exit $?"
rm -rf "$T"
