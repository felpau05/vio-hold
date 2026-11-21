#!/bin/sh
# Reverses betasim/enable.sh: removes tools/profiles/betaflight.json and reverses the patches.
# It checks first and changes nothing when a patch does not reverse or settings.json still uses a Betaflight key.
cd "$(dirname "$0")/.." || exit 1
P="test_fc_link run_all settings apb_main fc_link"
grep -qE '"msp_(lock_gain|gyro_lsb_per_dps)"' settings.json 2>/dev/null &&
  { echo "STOP: settings.json uses msp_lock_gain or msp_gyro_lsb_per_dps. First: python tools/use_profile.py apply <profiles without betaflight>. Nothing changed."; exit 1; }
for p in $P; do
  git apply -R --check "betasim/patches/$p.patch" 2>/dev/null ||
    { echo "STOP: betasim/patches/$p.patch does not reverse (not enabled, or the file changed since). Nothing changed."; exit 1; }
done
for p in $P; do git apply -R "betasim/patches/$p.patch" || { echo "STOP: $p.patch failed while reversing."; exit 1; }; done
rm -f tools/profiles/betaflight.json
echo "Betaflight link off."
