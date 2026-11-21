#!/bin/bash
# Run one command with nice -n 19, only while no simulator runs (no arducopter or px4 process, no Docker container
# named ap_sim* or px4_sim*). Waits 20 s and checks again. Usage: integ/diag10/gate_run.sh <command...>
cd /home/paul/Videos/vio-hold-attempt
while true; do
  busy=""
  pgrep -x arducopter >/dev/null && busy="arducopter"
  pgrep -f 'bin/px4( |$)' >/dev/null && busy="$busy px4"
  pgrep -x px4 >/dev/null && busy="$busy px4"
  if command -v docker >/dev/null; then
    n=$(docker ps --format '{{.Names}}' 2>/dev/null | grep -E '^(ap_sim|px4_sim)' | head -1)
    [ -n "$n" ] && busy="$busy docker:$n"
  fi
  [ -e integ/kfmap/SITL_ACTIVE ] && busy="$busy kfmap_SITL_ACTIVE"
  [ -z "$busy" ] && break
  echo "$(date +%T) wait: $busy" >&2
  sleep 20
done
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec nice -n 19 "$@"
