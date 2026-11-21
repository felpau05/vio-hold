#!/bin/sh
# B closed-loop queue (one run at a time; each under 5 min)
cd "$(dirname "$0")/../.." || exit 1
P=$HOME/.venvs/vio/bin/python
run() { lab=$1; shift; timeout 290 $P px4sim/sitl.py "$@" label=$lab > integ/avail/cl_$lab.txt 2>&1; echo "$lab rc=$?" >> integ/avail/queue_b_log.txt; }
run b_apb_100m_calm_stillstart apb 100 calm 30 still_start=true
run b_apb_100m_w20g40_imu apb 100 w20g40 60 'settings={"att_source":"imu"}'
run b_apb_100m_w20g40_holdout_quarry_s101 apb 100 w20g40 60 ground=holdout_quarry seed=101
run b_apb_100m_w20g40_holdout_field_s102 apb 100 w20g40 60 ground=holdout_field seed=102
run b_apb_100m_w20g40_holdout_water_s103 apb 100 w20g40 60 ground=holdout_water seed=103
