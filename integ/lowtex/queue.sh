#!/bin/sh
# Low-texture build: closed-loop runs, ONE AT A TIME (variant apb, 60 s, w20g40 unless stated).
# Order: validation/tuning acceptance first, then HOLD-OUT last (final code, nothing changed after).
cd "$(dirname "$0")/../.." || exit 1
P=$HOME/.venvs/vio/bin/python
LOG=integ/lowtex/queue_log.txt
run() { lab=$1; shift; timeout 300 $P px4sim/sitl.py "$@" label=$lab > integ/lowtex/cl_$lab.txt 2>&1; echo "$lab rc=$? $(date +%T)" >> $LOG; }
N="noise=10 exposure_s=0.02"
case "$1" in
acc)
run lt_field_s11 apb 100 w20g40 60 ground=holdout_field seed=11
run lt_prairie_s1 apb 100 w20g40 60 ground=prairie seed=1
run lt_snow50_s1 apb 50 w20g40 60 ground=snow_tarps_pad seed=1
run lt_night50_s1 apb 50 w20g40 60 ground=night_lights seed=1 $N
run lt_night100_s1 apb 100 w20g40 60 ground=night_lights seed=1 $N
run lt_ottawa_s11 apb 100 w20g40 60 seed=11
run lt_field_s12 apb 100 w20g40 60 ground=holdout_field seed=12
run lt_field_s13 apb 100 w20g40 60 ground=holdout_field seed=13
run lt_ottawa_s12 apb 100 w20g40 60 seed=12
run lt_ottawa_s13 apb 100 w20g40 60 seed=13
run lt_prairie_s2 apb 100 w20g40 60 ground=prairie seed=2
;;
holdout)
run lt_HO_lawn_s104 apb 100 w20g40 60 ground=holdout_lawn seed=104
run lt_HO_lawn_s105 apb 100 w20g40 60 ground=holdout_lawn seed=105
run lt_HO_lawn_s106 apb 100 w20g40 60 ground=holdout_lawn seed=106
run lt_HO_quarry_s101 apb 100 w20g40 60 ground=holdout_quarry seed=101
run lt_HO_water_s103 apb 100 w20g40 60 ground=holdout_water seed=103
;;
esac
case "$1" in
recheck)
run lt_final_field_s11 apb 100 w20g40 60 ground=holdout_field seed=11
run lt_final_night100_s1 apb 100 w20g40 60 ground=night_lights seed=1 $N
run lt_final_snow50_s1 apb 50 w20g40 60 ground=snow_tarps_pad seed=1
;;
esac
