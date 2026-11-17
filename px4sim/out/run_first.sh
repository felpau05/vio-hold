#!/bin/bash
cd /home/paul/Videos/vio-hold-attempt/px4sim
for ab in "1 3" "4 6" "6 8" "8 10" "10 12" "12 14" "14 16"; do
  set -- $ab; s=$(date +%s)
  timeout 295 ~/.venvs/vio/bin/python matrix.py first $1 $2 > out/first_${1}_${2}.txt 2>&1
  echo "slice $1-$2 exit $? took $(( $(date +%s)-s )) s" >> out/run_first.log
done
echo done >> out/run_first.log
