#!/bin/sh
# Runs ArduPilot closed-loop cases ONE AT A TIME from integ/control/queue.txt (one sitl_ap.py argument list per
# line; lines can be added while it runs). Done lines are counted in queue.done. A line "STOP" ends it; it also
# ends after 10 min with no new line. Log: integ/control/queue_log.txt.
cd "$(dirname "$0")/../.." || exit 1
PY=$HOME/.venvs/vio/bin/python
Q=integ/control/queue.txt; D=integ/control/queue.done; L=integ/control/queue_log.txt
touch $Q; [ -f $D ] || echo 0 > $D
idle=0
while :; do
  n=$(cat $D); line=$(sed -n "$((n+1))p" $Q)
  if [ -z "$line" ]; then
    idle=$((idle+5)); [ $idle -ge 600 ] && break; sleep 5; continue
  fi
  idle=0
  [ "$line" = "STOP" ] && { echo $((n+1)) > $D; break; }
  s=$(date +%s)
  eval "timeout 420 $PY apsim/sitl_ap.py $line" > integ/control/last_case.txt 2>&1
  rc=$?
  echo "$(date +%H:%M) rc=$rc $(( $(date +%s)-s ))s | $line | $(grep '\] rms' integ/control/last_case.txt | tail -1 | cut -c1-200)" >> $L
  lab=$(echo "$line" | sed -n 's/.*label=\([^ ]*\).*/\1/p')
  b=$(docker exec ap_sim sh -c 'ls -t /tmp/ap0/logs/*.BIN 2>/dev/null | head -1')
  [ -n "$b" ] && [ -n "$lab" ] && mkdir -p integ/control/bin && docker cp "ap_sim:$b" "integ/control/bin/$lab.BIN" >/dev/null 2>&1 \
    && docker exec ap_sim rm -f "$b"
  echo $((n+1)) > $D
done
echo "$(date +%H:%M) queue stopped" >> $L
