#!/bin/sh
# T4 and T5 closed-loop cases, one at a time (run_ap.py waits for a 1-minute load under 4 before each).
cd /home/paul/Videos/vio-hold-attempt || exit 1
PY=/home/paul/.venvs/vio/bin/python
R="$PY integ/justice_d/run_ap.py"
$R jd_apb_100m_w20g40_s12 100 w20g40 video=JUSTICE_D seed=12
$R jd_apb_100m_w20g40_s13 100 w20g40 video=JUSTICE_D seed=13
$R jd_apb_50m_w20g40_s11 50 w20g40 video=JUSTICE_D seed=11
$R jd_apb_20m_calm_s11 20 calm video=JUSTICE_D seed=11
$R jdcal_apb_100m_w20g40_s11 100 w20g40 video=JUSTICE_D seed=11 'true_lens="sim Justice D near crop"' 'cam_top="left"' 'cam_tilt=[1.2,-1.8]' cam_twist=1.5 readout_s=0.010 video_delay_s=0.046 exposure_s=0.004 'settings={"top":"left","mirror":false,"tilt_deg":[1.7345124808380734,-1.130173650164294],"twist_deg":1.5658196793151928,"focal_scale":1.0,"readout_s":0.011,"video_delay_s":0.058,"lens_file":"/home/paul/Videos/vio-hold-attempt/integ/justice_d/calib_loop_d_lens_20261118_230048.json","lens_name":"Justice D calibrated 2026-11-18 23:01"}'
$R an_apb_50m_w20g40_s11 50 w20g40 seed=11
$R an_apb_20m_calm_s11 20 calm seed=11
$R an_apb_100m_w20g40_s11 100 w20g40 seed=11
$R jdfull_apb_100m_w20g40_s11 100 w20g40 video=JUSTICE_D seed=11 'true_lens="sim Justice D near full sensor"'
echo QUEUE DONE
