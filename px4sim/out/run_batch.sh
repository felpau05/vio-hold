#!/bin/bash
cd /home/paul/Videos/vio-hold-attempt/px4sim
run() { s=$(date +%s); timeout 295 ~/.venvs/vio/bin/python matrix.py $1 $2 $3 > out/${1}_${2}_${3}.txt 2>&1
        echo "slice $1 $2-$3 exit $? took $(( $(date +%s)-s )) s" >> out/run_batch.log; docker rm -f px4_sim >/dev/null 2>&1; }
for a in 4 6 8 10 12 14; do run first $a $((a+2)); done
for a in 0 2 4 6 8 10 12 14 16; do run more $a $((a+2)); done
echo done >> out/run_batch.log
