#!/bin/sh
# Turns the Betaflight link on in this project: applies the patches in betasim/patches/ (fc_link.py, apb_main.py,
# settings.py, tests_apb/run_all.sh, tests_apb/test_fc_link.py) and copies betasim/profiles/betaflight.json to
# tools/profiles/. It checks every patch first and changes nothing when one does not apply. Undo: betasim/disable.sh.
# msp_link_count_carry.patch is a proposal and is not applied here.
cd "$(dirname "$0")/.." || exit 1
P="fc_link apb_main settings run_all test_fc_link"
[ -e tools/profiles/betaflight.json ] && { echo "STOP: tools/profiles/betaflight.json exists: already enabled? Nothing changed."; exit 1; }
for p in $P; do
  git apply --check "betasim/patches/$p.patch" 2>/dev/null ||
    { echo "STOP: betasim/patches/$p.patch does not apply to this project. Nothing changed."; exit 1; }
done
for p in $P; do git apply "betasim/patches/$p.patch" || { echo "STOP: $p.patch failed while applying. Run betasim/disable.sh."; exit 1; }; done
cp betasim/profiles/betaflight.json tools/profiles/betaflight.json
echo "Betaflight link on. Next: python tools/use_profile.py apply outdoor_day betaflight"
