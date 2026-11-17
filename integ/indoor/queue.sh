#!/bin/sh
# Indoor and low-height build: closed-loop PX4 runs, ONE AT A TIME (variant apb). Usage: sh integ/indoor/queue.sh <set>
# Indoor profile (settings overrides only): min_height_m 1, Hold h_min 1. Indoor camera: exposure 10 ms, noise 5 grey
# (both assumed: dimmer light, more gain). Calm air indoors.
cd "$(dirname "$0")/../.." || exit 1
P=$HOME/.venvs/vio/bin/python
LOG=integ/indoor/queue_log.txt
run() { lab=$1; shift; timeout 420 $P px4sim/sitl.py "$@" label=$lab > integ/indoor/cl_$lab.txt 2>&1; echo "$lab rc=$? $(date +%T)" >> $LOG; }
IN0='settings={"min_height_m": 1.0, "hold_cfg": {"h_min": 1.0}}'      # dev runs before 21:10
IN='settings={"min_height_m": 1.0, "hold_cfg": {"h_min": 1.0, "row_norm": true}}'   # indoor profile from 21:10 on
CAM="exposure_s=0.01 noise=5"
case "$1" in
first)
run in_conc_5m_s1 apb 5 calm 40 ground=floor_concrete seed=1 $CAM "$IN"
;;
dev)    # tuning floors, development seeds
run in_conc_2m_s1 apb 2 calm 40 ground=floor_concrete seed=1 $CAM "$IN0"
run in_tiles_10m_s1 apb 10 calm 40 ground=floor_tiles seed=1 $CAM "$IN0"
run in_wood_15m_s1 apb 15 calm 40 ground=floor_wood seed=1 $CAM "$IN0"
run truth_5m_s1 truth 5 calm 40 ground=floor_concrete seed=1 $CAM
run in_flick_people_5m_s2 apb 5 calm 40 ground=floor_concrete seed=2 exposure_s=0.004 noise=5 flicker_hz=120 people=3 "$IN0"
run in_sens_5m_s2 apb 5 calm 40 ground=floor_concrete seed=2 $CAM baro_wander_m=1.0 mag_yaw_deg=10 mag_yaw_dps=0.1 "$IN0"
run in_shiny_10m_s3 apb 10 calm 40 ground=floor_shiny reflect=true seed=3 $CAM "$IN0"
run in_carpet_5m_s3 apb 5 calm 40 ground=floor_carpet seed=3 $CAM "$IN0"
run in_tiles_2m_s2 apb 2 calm 40 ground=floor_tiles seed=2 $CAM "$IN0"
;;
rest)
# EKF2_DELAY_MAX A/B: settings.py params_px4 has 300 since 21:02 (control work); PX4's own default is 200
DM='px4={"EKF2_DELAY_MAX": 200}'
run dm300_conc_5m_s1 apb 5 calm 40 ground=floor_concrete seed=1 $CAM "$IN"
run dm200_conc_5m_s1 apb 5 calm 40 ground=floor_concrete seed=1 $CAM "$IN" "$DM"
run dm300_100m_w_s11 apb 100 w20g40 60 ground=ottawa2022 seed=11
run dm200_100m_w_s11 apb 100 w20g40 60 ground=ottawa2022 seed=11 "$DM"
run dm300_snow50_s1 apb 50 w20g40 60 ground=snow_tarps_pad seed=1
run dm200_snow50_s1 apb 50 w20g40 60 ground=snow_tarps_pad seed=1 "$DM"
run in_conc_2m_s1b apb 2 calm 40 ground=floor_concrete seed=1 $CAM "$IN"
run in_flick_people_5m_s2b apb 5 calm 40 ground=floor_concrete seed=2 exposure_s=0.004 noise=5 flicker_hz=120 people=3 "$IN"
# what limits the indoor hold: PX4's vision noise floor EKF2_EVP_NOISE (default 0.1 m) against 0.02 m
run in_conc_5m_s1_evp02 apb 5 calm 40 ground=floor_concrete seed=1 $CAM "$IN" 'px4={"EKF2_EVP_NOISE": 0.02}'
run truth_5m_s1_evp02 truth 5 calm 40 ground=floor_concrete seed=1 $CAM 'px4={"EKF2_EVP_NOISE": 0.02}'
# propellers in the picture corners, outdoor 50 m in wind on ottawa2022 (validation), seed 11: no props / props with
# the corner mask / props without a mask
run pr_none_50m_s11 apb 50 w20g40 60 ground=ottawa2022 seed=11
run pr_mask_50m_s11 apb 50 w20g40 60 ground=ottawa2022 seed=11 props=true mask_corners=0.36
run pr_nomask_50m_s11 apb 50 w20g40 60 ground=ottawa2022 seed=11 props=true
# HOLD-OUT indoor floors (final numbers; nothing is changed after these)
run HO_a_2m_s101 apb 2 calm 40 ground=holdout_floor_a seed=101 $CAM "$IN"
run HO_b_2m_s102 apb 2 calm 40 ground=holdout_floor_b seed=102 $CAM "$IN"
run HO_a_5m_s103 apb 5 calm 40 ground=holdout_floor_a seed=103 $CAM "$IN"
run HO_b_5m_s104 apb 5 calm 40 ground=holdout_floor_b seed=104 $CAM "$IN"
run HO_a_10m_s105 apb 10 calm 40 ground=holdout_floor_a seed=105 $CAM "$IN"
run HO_b_10m_s106 apb 10 calm 40 ground=holdout_floor_b seed=106 $CAM "$IN"
run HO_a_15m_s107 apb 15 calm 40 ground=holdout_floor_a seed=107 $CAM "$IN"
run HO_b_15m_s108 apb 15 calm 40 ground=holdout_floor_b seed=108 $CAM "$IN"
run HO_a_5m_flick_s109 apb 5 calm 40 ground=holdout_floor_a seed=109 exposure_s=0.004 noise=5 flicker_hz=120 people=3 "$IN"
run HO_b_10m_flick_s110 apb 10 calm 40 ground=holdout_floor_b seed=110 exposure_s=0.004 noise=5 flicker_hz=100 people=3 "$IN"
# snow with tarps at 50 m: more seeds (validation)
run sn_50m_s11 apb 50 w20g40 60 ground=snow_tarps_pad seed=11
run sn_50m_s12 apb 50 w20g40 60 ground=snow_tarps_pad seed=12
run sn200_50m_s11 apb 50 w20g40 60 ground=snow_tarps_pad seed=11 "$DM"
run sn200_50m_s12 apb 50 w20g40 60 ground=snow_tarps_pad seed=12 "$DM"
# low outdoor heights (validation ground ottawa2022; open grass field holdout_field is validation since 05:35)
run lo_20m_calm_s11 apb 20 calm 60 ground=ottawa2022 seed=11
run lo_20m_w_s11 apb 20 w20g40 60 ground=ottawa2022 seed=11
run lo_40m_calm_s12 apb 40 calm 60 ground=ottawa2022 seed=12
run lo_40m_w_s12 apb 40 w20g40 60 ground=ottawa2022 seed=12
run lo_60m_calm_s13 apb 60 calm 60 ground=ottawa2022 seed=13
run lo_60m_w_s13 apb 60 w20g40 60 ground=ottawa2022 seed=13
run lo_field_20m_w_s14 apb 20 w20g40 60 ground=holdout_field seed=14
run lo_field_60m_w_s15 apb 60 w20g40 60 ground=holdout_field seed=15
;;
esac
