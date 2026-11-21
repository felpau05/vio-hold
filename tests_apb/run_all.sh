#!/bin/sh
# Runs every APB test; each writes tests_apb/out_<name>.txt. Prints the last line of each.
cd "$(dirname "$0")/.." || exit 1
PY=${PY:-$HOME/.venvs/vio/bin/python}
for t in test_basics test_digital test_robust test_fc_link test_run test_bench test_calib test_lens_d test_lens_store test_live test_replay_chunks test_wind_assist test_ap_sitl test_event; do
  timeout 295 "$PY" tests_apb/$t.py > tests_apb/out_$t.txt 2>&1
  echo "$t: rc=$? $(tail -1 tests_apb/out_$t.txt)"
done
timeout 295 "$PY" tests_apb/test_gnss.py > tests_apb/out_test_gnss.txt 2>&1; echo "test_gnss: rc=$? $(tail -1 tests_apb/out_test_gnss.txt)"
timeout 295 "$PY" tests_apb/test_ground_tools.py > tests_apb/out_test_ground_tools.txt 2>&1
echo "test_ground_tools: rc=$? $(tail -1 tests_apb/out_test_ground_tools.txt)"
timeout 295 "$PY" -c "import sim_test; sim_test.timing_test(); print('timing_test PASSED')" > tests_apb/out_timing_test.txt 2>&1
echo "sim_test.timing_test: rc=$? $(tail -1 tests_apb/out_timing_test.txt)"
