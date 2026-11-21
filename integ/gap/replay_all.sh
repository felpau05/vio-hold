#!/bin/sh
# Open-loop replay (integ/gap/replay_run.py) of every closed-loop run with a gap of 1 s or more, with hold.py as it is
# ("before") and with the proposed patch applied in the copy ("after"). One process at a time.
# Run: sh integ/gap/replay_all.sh <path to the patched copy's hold.py> > integ/gap/results/replay_all.txt
cd /home/paul/Videos/vio-hold-attempt || exit 1
PY=/home/paul/.venvs/vio/bin/python
PATCHED=$1
for r in an_apb_100m_w20g40_s11 an_apb_50m_w20g40_s11 jd_apb_100m_w20g40_s11 jd_apb_100m_w20g40_s12 jd_apb_20m_calm_s11 \
         jd_apb_50m_w20g40_s11 jdcal2_apb_100m_w20g40_s11 jdcal3_apb_100m_w20g40_s11 jdcal_apb_100m_w20g40_s11 \
         c_d140_100m_w20g40_s11 c_d140_100m_w20g40_s12 c_d140_100m_w20g40_s13; do
  $PY integ/gap/replay_run.py $r --tag _before 2>&1 | grep "replay gap" || echo "$r _before: no gap line"
  $PY integ/gap/replay_run.py $r --patch "$PATCHED" --tag _after 2>&1 | grep "replay gap" || echo "$r _after: no gap line"
done
