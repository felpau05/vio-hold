#!/bin/bash
# Wait until a replay may start (coordinator rules, 2026-11-21): no integ/kfmap/SITL_ACTIVE, 1-minute load under 2,
# no arducopter or px4 process, no docker container ap_sim* or px4_sim*. Checks every 60 s. Exit 1 after 07:30 EST.
P=/home/paul/Videos/vio-hold-attempt
while true; do
  now=$(TZ=America/New_York date +%H%M)
  if [ "$now" -ge 0730 ]; then echo "gate: 07:30 EST passed, no new replay"; exit 1; fi
  load=$(cut -d' ' -f1 /proc/loadavg)
  why=""
  [ -e $P/integ/kfmap/SITL_ACTIVE ] && why="$why SITL_ACTIVE"
  # Before main says the laptop is free (no file integ/kfgraph/GO): one replay at a time, and the load under 2.
  # With GO (the free window): up to 4 at a time (window.sh), no load test (my own replays make the load).
  if [ ! -e $P/integ/kfgraph/GO ]; then
    why="$why no-GO"                       # coordinator 06:34: without GO nothing starts
    awk -v l="$load" 'BEGIN{exit !(l>=2)}' && why="$why load=$load"
    pgrep -f "integ/kfgraph/(ol_graph|bad_graph)\.py" >/dev/null && why="$why another-kfgraph-replay"
    L=$P/integ/kfgraph/.serial_lock
    if [ -z "$why" ]; then
      [ -d $L ] && [ $(( $(date +%s) - $(stat -c %Y $L) )) -gt 15 ] && rmdir $L 2>/dev/null
      mkdir $L 2>/dev/null || why="$why claimed"
    fi
  fi
  pgrep -x arducopter >/dev/null && why="$why arducopter"
  pgrep -x px4 >/dev/null && why="$why px4"
  docker ps --format '{{.Names}}' 2>/dev/null | grep -qE '^(ap_sim|px4_sim)' && why="$why container"
  [ -z "$why" ] && exit 0
  echo "gate $(TZ=America/New_York date +%H:%M:%S): wait:$why" >&2
  sleep $(( 50 + RANDOM % 20 ))
done
