#!/bin/sh
# Justice D recordings for step 7 (fly, record, replay, tune): the same PX4 SITL flights as the 2026-11-19 rehearsal,
# plus 60 m. 60 s from the switch to Position mode. Output: integ/rehearsal_2026-11-20/runs/jd_<h>m/
cd "$(dirname "$0")/../.." || exit 1
PY=$HOME/.venvs/vio/bin/python
for hr in 10:lowout_apb_jd_10m_w20g40_s1 30:apb_30m_w20g40 60:apb_60m_w20g40 100:apb_100m_w20g40; do
  h=${hr%%:*}; r=${hr#*:}
  echo "== $h m from px4sim/runs/$r.npz  $(date +%T)"
  $PY tools/sim_rec.py px4sim/runs/$r.npz integ/rehearsal_2026-11-20/runs/jd_${h}m --seconds 60 --video JUSTICE_D || exit 1
done
echo "done $(date +%T)"
