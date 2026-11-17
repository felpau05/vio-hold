#!/bin/sh
# Runs closed-loop cases one per process (each under 5 min): integ/queue.sh group:index [group:index ...]
cd "$(dirname "$0")/../px4sim" || exit 1
PY=$HOME/.venvs/vio/bin/python
for gi in "$@"; do
  g=${gi%%:*}; i=${gi##*:}
  s=$(date +%s)
  timeout 295 $PY matrix.py "$g" "$i" $((i+1)) > ../integ/case_${g}_${i}.txt 2>&1
  echo "$g:$i rc=$? $(( $(date +%s)-s ))s $(grep -m1 '^\[' ../integ/case_${g}_${i}.txt | head -c 0)$(grep '\] rms' ../integ/case_${g}_${i}.txt | head -1 | cut -c1-220)" >> ../integ/queue_log.txt
done
echo "queue done: $*" >> ../integ/queue_log.txt
