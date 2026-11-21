#!/bin/sh
# Runs every APB test; each writes tests_apb/out_<name>.txt. Prints the last line of each.
cd "$(dirname "$0")/.." || exit 1
PY=${PY:-$HOME/.venvs/vio/bin/python}
for t in test_basics test_justice_d test_fc_link test_run test_bench test_calib test_replay_chunks test_wind_assist test_apb_est test_ap_sitl; do
  timeout 295 "$PY" tests_apb/$t.py > tests_apb/out_$t.txt 2>&1
  echo "$t: rc=$? $(tail -1 tests_apb/out_$t.txt)"
done
timeout 295 "$PY" -c "import sim_test; sim_test.timing_test(); print('timing_test PASSED')" > tests_apb/out_timing_test.txt 2>&1
echo "sim_test.timing_test: rc=$? $(tail -1 tests_apb/out_timing_test.txt)"
