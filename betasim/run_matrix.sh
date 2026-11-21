#!/bin/sh
# Closed-loop matrix: mode v1 (gain 0) and v2 (gain 0.2/s), seeds 1 and 2, 60 s hold each, one SITL at a time.
# Needs BF_SITL and BF_SRC (betasim/build.sh prints both). Out: betasim/results/closed_*.json and matrix.txt.
cd "$(dirname "$0")/.." || exit 1
PY=${PY:-$HOME/.venvs/vio/bin/python}
: > betasim/results/matrix.txt
for g in 0 0.2; do for s in 1 2; do
  nice -n 19 timeout 300 "$PY" betasim/closed_loop.py --gain $g --seed $s --hold 60 "$@" 2>&1 | tail -1 >> betasim/results/matrix.txt
done; done
cat betasim/results/matrix.txt
