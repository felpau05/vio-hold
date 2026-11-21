#!/bin/bash
# The free window (07:15-07:30 EST): batch 1 smoke alone (stop if it fails), the solve timing on one core, then
# batches of replays, up to NPAR at the same time (1 thread each, nice -n 19), a table after each batch.
# Each job waits at gate.sh first (no SITL_ACTIVE, arducopter, px4 or container; without integ/kfgraph/GO also load < 2
# and one replay at a time); no job starts after 07:30; every job is stopped at 07:35 (timeout). Done steps are skipped.
P=/home/paul/Videos/vio-hold-attempt
K=$P/integ/kfgraph
cd $P
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
NPAR=${NPAR:-4}
now() { TZ=America/New_York date +%H:%M:%S; }
run1() {   # one job line
  line="$1"
  $K/gate.sh || return 1
  left=$(( $(TZ=America/New_York date -d "07:35" +%s) - $(date +%s) ))
  [ $left -le 5 ] && return 1
  echo "=== $(now) RUN $line"
  case "$line" in
    RAW*) eval timeout $left nice -n 19 ${line#RAW };;
    *) eval timeout $left nice -n 19 ~/.venvs/vio/bin/python integ/kfgraph/ol_graph.py $line;;
  esac
  echo "=== $(now) END rc=$? $line" | cut -c1-200
}
export -f run1 now
export K P
batch() {  # batch name file
  echo "=== $(now) BATCH $1 start"
  grep -v '^#' "$2" | grep . | xargs -d '\n' -P $NPAR -I{} bash -c 'run1 "$@"' _ {}
  echo "=== $(now) BATCH $1 done"
  nice -n 19 ~/.venvs/vio/bin/python $K/summarize.py tuning > /dev/null 2>&1
  nice -n 19 ~/.venvs/vio/bin/python $K/summarize.py validation > /dev/null 2>&1
  echo "=== $(now) TABLES written ($1)"
}
if [ ! -e $K/results/ol/kg_tun_ottawa2019fall_crop_10m_s1_smoke_relast_on.json.gz ]; then
  echo "=== $(now) BATCH 1 smoke start"
  run1 "$(grep -v '^#' $K/jobs_smoke.txt | grep .)"
  if ! ls $K/results/ol/kg_tun_ottawa2019fall_crop_10m_s1_smoke_relast_on.json.gz >/dev/null 2>&1; then
    echo "=== $(now) SMOKE FAILED: stop"; exit 1; fi
  echo "=== $(now) BATCH 1 done"
fi
if [ ! -e $K/results/test_graph_core2.txt ]; then
  $K/gate.sh && { echo "=== $(now) TIMING one core"
  OMP_NUM_THREADS=1 taskset -c 2 nice -n 19 ~/.venvs/vio/bin/python $K/test_graph.py > $K/results/test_graph_core2.log 2>&1
  cp $K/results/test_graph.txt $K/results/test_graph_core2.txt; rmdir $K/.serial_lock 2>/dev/null; }
fi
batch 2 $K/jobs_b2.txt
batch 3 $K/jobs_b3.txt
batch 4 $K/jobs_b4.txt
echo "=== $(now) window done"
