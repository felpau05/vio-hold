#!/bin/sh
# After the 22:12 restart: the runs that the restart cut or never started, ONE AT A TIME, in scope order.
cd "$(dirname "$0")/../.." || exit 1
P=$HOME/.venvs/vio/bin/python
LOG=integ/indoor/queue_log.txt
run() { lab=$1; shift; timeout 420 $P px4sim/sitl.py "$@" label=$lab > integ/indoor/cl_$lab.txt 2>&1; echo "$lab rc=$? $(date +%T)" >> $LOG; }
IN='settings={"min_height_m": 1.0, "hold_cfg": {"h_min": 1.0, "row_norm": true}}'
OLD='settings={"min_height_m": 1.0, "hold_cfg": {"h_min": 1.0, "row_norm": true, "live_still_pic_px": 0.3}}'
CAM="exposure_s=0.01 noise=5"
# low outdoor heights (validation)
run lo_40m_w_s12 apb 40 w20g40 60 ground=ottawa2022 seed=12
run lo_60m_calm_s13 apb 60 calm 60 ground=ottawa2022 seed=13
run lo_60m_w_s13 apb 60 w20g40 60 ground=ottawa2022 seed=13
run lo_field_20m_w_s14 apb 20 w20g40 60 ground=holdout_field seed=14
run lo_field_60m_w_s15 apb 60 w20g40 60 ground=holdout_field seed=15
# EKF2_DELAY_MAX pair at 5 m (the first dm300 run shared the CPU with an open-loop replay; dm200 crashed: sitl merge bug)
run dm300_conc_5m_s1b apb 5 calm 40 ground=floor_concrete seed=1 $CAM "$IN"
run dm200_conc_5m_s1b apb 5 calm 40 ground=floor_concrete seed=1 $CAM "$IN" 'px4={"EKF2_DELAY_MAX": 200}'
# frozen picture with fresh noise (scope 6)
run fz_in_5m_s3 apb 5 calm 40 ground=floor_concrete seed=3 $CAM "$IN" 'freeze=[15, 3]' freeze_noise=true
run fz_out_50m_s13 apb 50 calm 60 ground=ottawa2022 seed=13 'freeze=[20, 3]' freeze_noise=true
# "still" proof change, closed-loop A/B
run lv03_tiles_10m_s4 apb 10 calm 40 ground=floor_tiles seed=4 $CAM "$OLD"
run lv15_tiles_10m_s4 apb 10 calm 40 ground=floor_tiles seed=4 $CAM "$IN"
# propellers on low-texture ground (whole-picture path)
run pr_nomask_field100_s16 apb 100 w20g40 60 ground=holdout_field seed=16 props=true
run pr_mask_field100_s16 apb 100 w20g40 60 ground=holdout_field seed=16 props=true mask_corners=0.36
# 40 m calm again: the first run lost its video at 55 s when the laptop restarted
run lo_40m_calm_s12b apb 40 calm 60 ground=ottawa2022 seed=12
# open field, same seed, no props: is the masked fly-away the mask or the load?
run pr_none_field100_s16 apb 100 w20g40 60 ground=holdout_field seed=16
