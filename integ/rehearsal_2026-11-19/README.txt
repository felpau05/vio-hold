Rehearsal of EIGHT_HOURS hour 5 (replay_tune.py) on the laptop, 2026-11-19 about 04:25-04:35.
One simulator recording at each height, wind 20 gusting 40 km/h. video_delay_s set 30 ms too long (true 0.0604 s).

For each height:
  ~/.venvs/vio/bin/python tools/sim_rec.py px4sim/runs/<run>.npz OUT --seconds 60
  then edit OUT/settings.json: video_delay_s 0.0904 (10 m only: also min_height_m 3, hold_cfg {"h_min": 2})
  ~/.venvs/vio/bin/python tools/replay_tune.py "OUT/raw_sim_*.npz" --settings OUT/settings.json --jobs 6 --out OUT/tune

  10 m  run lowout_apb_jd_10m_w20g40_s1: final patch {focal_scale 1.0383 (true 1.0), hold_cfg max_pts 120,
        fb_every 3}. The delay stayed at the wrong 0.0904 (jitter 1.41-1.63 cm for every delay, the true delay
        among the worst). tune_10m.txt line 46
  30 m  run apb_30m_w20g40: final patch {video_delay_s 0.1499 (89 ms from the truth), readout_s 0.0 (true 0.0184:
        the rolling-shutter correction off), focal_scale 1.0816 (8 % off)}. tune_30m.txt line 66
  100 m run apb_100m_w20g40: final patch {video_delay_s 0.0543 (6 ms from the truth)}. tune_100m.txt line 58
Cause (design): replay_tune scores each value by fix jitter. At 10 m and 30 m a wrong value gave less jitter.
Jitter is not accuracy.

Limits: one recording at each height. sim_rec.py renders the analog camera (25 fps, analog lens) even from a
Justice D run, so these recordings do not exercise the Justice D path. replay_tune took 35 s with 6 jobs.
Change made: EIGHT_HOURS hour 5 and DECISION_CARD row 5: below 100 m, merge no replay_tune patch at all. The delay
and readout then come from the wiggle at 40 m or higher (EVENT_DAY step 7b; tested at 40 m and 100 m only,
integ/control/wiggle_calib.txt) or from the ground calibration (EVENT_DAY step 4).
