#!/bin/sh
# B closed-loop queue 2 (one run at a time, nothing else running)
cd "$(dirname "$0")/../.." || exit 1
P=$HOME/.venvs/vio/bin/python
run() { lab=$1; shift; timeout 290 $P px4sim/sitl.py "$@" label=$lab > integ/avail/cl_$lab.txt 2>&1; echo "$lab rc=$? $(date +%T)" >> integ/avail/queue_c_log.txt; }
run b_apb_100m_w20g40_prairie_v2 apb 100 w20g40 60 ground=prairie
run b_apb_100m_calm_stillstart_v2 apb 100 calm 30 still_start=true
for sd in 11 12 13; do
  run b_imu_100m_w20g40_s$sd apb 100 w20g40 60 seed=$sd 'settings={"att_source":"imu"}'
  run b_gyro_100m_w20g40_s$sd apb 100 w20g40 60 seed=$sd 'settings={"att_source":"gyro"}'
done
for sd in 11 12; do
  run b_imu_100m_calm_s$sd apb 100 calm 60 seed=$sd 'settings={"att_source":"imu"}'
  run b_gyro_100m_calm_s$sd apb 100 calm 60 seed=$sd 'settings={"att_source":"gyro"}'
done
run b_imu_122m_w20g40_s11 apb 122 w20g40 60 seed=11 'settings={"att_source":"imu"}'
run b_gyro_122m_w20g40_s11 apb 122 w20g40 60 seed=11 'settings={"att_source":"gyro"}'
run b_imu_122m_calm_s11 apb 122 calm 60 seed=11 'settings={"att_source":"imu"}'
run b_gyro_122m_calm_s11 apb 122 calm 60 seed=11 'settings={"att_source":"gyro"}'
run gps_100m_calm gps 100 calm 60
run gps_122m_calm gps 122 calm 60
run gps_122m_w20g40 gps 122 w20g40 60
