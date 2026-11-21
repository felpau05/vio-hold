#!/bin/sh
# Follow-up runs (same plant, wind 2 m/s + 1 m/s gusts, 5 m, 60 s, seeds 1 and 2; one SITL at a time).
# Q1 baselines: gps_* = the firmware's own Position Hold on its virtual GPS (true position, no flow);
#   z_* = our link with zero speed and fix error (mode v1 gain 0, mode v2 gain 0.2).
# Q2 lock gain: b_* = speed error 0.10 m/s north (constant) + 0.05 noise, gains 0.1 0.2 0.5 1.0.
# Needs BF_SITL and BF_SRC (betasim/build.sh). Out: betasim/results/closed_*.json and followup.txt.
cd "$(dirname "$0")/.." || exit 1
PY=${PY:-$HOME/.venvs/vio/bin/python}
run() { nice -n 19 timeout 300 "$PY" betasim/closed_loop.py --hold 60 "$@" 2>&1 | tail -1 >> betasim/results/followup.txt; }
: > betasim/results/followup.txt
for s in 1 2; do run --gps --seed $s --tag gps_s$s; done
for g in 0 0.2; do for s in 1 2; do
  run --gain $g --seed $s --speed-bias 0 --speed-sd 0 --fix-sd 0 --tag z_g${g}_s$s; done; done
for g in 0.1 0.2 0.5 1.0; do for s in 1 2; do run --gain $g --seed $s --tag b_g${g}_s$s; done; done
cat betasim/results/followup.txt
