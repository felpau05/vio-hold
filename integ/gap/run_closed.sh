#!/bin/sh
# One ArduCopter SITL closed-loop run with the proposed patch, from the patched copy of the repository.
# The copy runs its own container (ap_sim_gap: its run_sitl.sh is changed only in the container name, and the SITL
# binary is copied in), so the ap_sim container of the main tree is not touched. Waits for a 1-minute load under 4
# and for no other arducopter or px4 SITL. Copies the run's json, onboard log and npz to integ/gap/results/closed/.
# Run: sh integ/gap/run_closed.sh <label> <height> <wind> [key=value ...]   (as integ/justice_d/run_ap.py)
COPY=${COPY:-/tmp/gap_copy}
OUT=/home/paul/Videos/vio-hold-attempt/integ/gap/results/closed
PY=/home/paul/.venvs/vio/bin/python
mkdir -p "$OUT"
label=$1
if [ -e "$OUT/$label.json" ]; then echo "$OUT/$label.json exists: give a new label"; exit 1; fi
grep -q "LIVE_MOVE_PX = 0.5" "$COPY/hold.py" || { echo "the copy is not patched"; exit 1; }
while :; do
  busy=$(docker ps --format '{{.Names}}' | while read n; do docker exec "$n" sh -c 'ps aux' 2>/dev/null | grep -E "[a]rducopter|[p]x4" ; done | wc -l)
  load=$(cut -d' ' -f1 /proc/loadavg)
  if [ "$busy" -eq 0 ] && [ "$(echo "$load < 4" | bc)" -eq 1 ]; then break; fi
  echo "load $load, other SITL processes $busy: waiting 30 s"; sleep 30
done
uptime
cd "$COPY" && $PY integ/justice_d/run_ap.py "$@"
cp "$COPY/integ/justice_d/closed/$label.json" "$OUT/"
cp "$COPY/apsim/runs/${label}_onboard.csv" "$COPY/apsim/runs/$label.npz" "$OUT/" 2>/dev/null
cp "$COPY/apsim/runs/$label.json" "$OUT/${label}_sitl_ap.json" 2>/dev/null
