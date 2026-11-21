#!/bin/bash
# Run the commands of a job file (one per line) one at a time, each with nice -n 19, and only while the 1-minute load
# is under 3 (another run uses SITL on this computer). Usage: integ/lowh/run_jobs.sh jobs.txt log.txt
# Lines starting with # are skipped. Each command runs from the repository root.
cd "$(dirname "$0")/../.." || exit 1
jobs="$1"; log="$2"
while IFS= read -r cmd; do
  [ -z "$cmd" ] && continue
  case "$cmd" in \#*) continue;; esac
  while :; do
    l=$(cut -d' ' -f1 /proc/loadavg)
    awk -v l="$l" 'BEGIN{exit !(l < 3.0)}' && break
    echo "$(date +%T) load $l: wait" >> "$log"
    sleep 20
  done
  echo "$(date +%T) load $l: $cmd" >> "$log"
  nice -n 19 bash -c "$cmd" >> "$log" 2>&1
  echo "$(date +%T) exit $?" >> "$log"
done < "$jobs"
echo "$(date +%T) all done" >> "$log"
