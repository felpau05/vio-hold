#!/bin/sh
# Frozen-picture hand-hover runs with the liveness patch v2 (the patch now in hold.py). The earlier set has freeze
# rows only for the old code and the superseded v1 (results/hh_summary.txt). Same cases, seeds and freeze as before:
# freeze at 8 s for 6 s with fresh noise, vibration 0.2 deg. Imports hold.py from the snapshot copy gap_copy, whose
# liveness code matches integ/gap/proposed.patch, so edits in progress in the repository do not change the result.
# Load gate: another run uses SITL cases, which need a quiet computer. Each run waits until the 1-minute load is
# under 2.5, and runs at the lowest priority. One run at a time.
# Run: sh integ/gap/run_freeze_v2.sh > integ/gap/results/run_freeze_v2_log.txt 2>&1
cd /home/paul/Videos/vio-hold-attempt || exit 1
COPY=${COPY:-/tmp/gap_copy}
PY=$HOME/.venvs/vio/bin/python
grep -q LIVE_MOVED_S "$COPY/hold.py" || { echo "FAIL: $COPY/hold.py has no patch v2. Stop."; exit 1; }
for c in "JD breeze 100" "JD breeze 20" "JD calm 100" "JD calm 20" "JD still 100" "JD still 20" "PAL calm 100" "PAL still 100"; do
  set -- $c
  for seed in 11 12 13; do
    while [ "$(cut -d' ' -f1 /proc/loadavg | cut -d. -f1)" -ge 3 ] || \
          awk '{exit !($1 >= 2.5)}' /proc/loadavg; do sleep 30; done
    echo "$(date +%H:%M:%S) start $1 $2 $3 m seed $seed (load $(cut -d' ' -f1 /proc/loadavg))"
    nice -n 19 "$PY" integ/gap/hand_hover.py --camera "$1" --level "$2" --height "$3" --seed "$seed" \
      --freeze 8 --freeze_s 6 --freeze_kind noise --repo "$COPY" --tag _fix2
    echo "$(date +%H:%M:%S) done rc=$?"
  done
done
"$PY" -I integ/gap/hh_summary.py > integ/gap/results/hh_summary.txt
echo "summary written"
