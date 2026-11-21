#!/bin/bash
# Run the jobs of a list file one at a time (one process, nice -n 19, 1 thread), each after gate.sh.
# Usage: integ/kfgraph/run_queue.sh jobs.txt
# Lines: '#' comment; 'STEP name' prints a step marker; 'RAW cmd' runs cmd (from the repository root);
# any other line is the arguments of ol_graph.py.
P=/home/paul/Videos/vio-hold-attempt
cd $P
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
while IFS= read -r line; do
  [ -z "$line" ] && continue
  case "$line" in
    \#*) continue;;
    STEP*) echo "=== $(TZ=America/New_York date +%H:%M:%S) $line"; continue;;
  esac
  $P/integ/kfgraph/gate.sh || exit 1
  echo "=== $(TZ=America/New_York date +%H:%M:%S) RUN $line"
  left=$(( $(TZ=America/New_York date -d "07:35" +%s) - $(date +%s) ))   # every replay stops at 07:35 EST
  case "$line" in
    RAW*) eval timeout $left nice -n 19 ${line#RAW };;
    *) eval timeout $left nice -n 19 ~/.venvs/vio/bin/python integ/kfgraph/ol_graph.py $line;;
  esac
  echo "=== $(TZ=America/New_York date +%H:%M:%S) END rc=$?"
done < "$1"
echo "=== $(TZ=America/New_York date +%H:%M:%S) queue done"
