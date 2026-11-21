# Data splits: what may be used for tuning, and what may not

Every number in this project that was fitted or tuned (a threshold, a gain, a noise level, the
variance model, the calibration search ranges) is a small model. Treat it like machine learning:
tune on one set, check on a second set, and report the final result on a third set that nobody
tuned on.

## Rules
1. A file or seed in the HOLD-OUT list is never used to choose a setting. Run it only for final results.
2. If someone looks at a hold-out result and then changes a setting because of it, that data moves
   to the TUNING list, and new hold-out data is added.
3. Every reported result says which set it comes from. Report the middle and the worst of several seeds.
4. When you tune something, add a line to the log below: what, on which data, by whom, when.

## Sets
| Set | Ground sites | Real clips | Seeds |
|---|---|---|---|
| TUNING (already used to choose settings) | ottawa2019fall, suburb, prairie | clip-1 (local only, not in git) | 1 to 9 |
| VALIDATION (check during work; may guide the next change) | ottawa2022, farmland | none yet | 11 to 19 |
| HOLD-OUT (final results only) | 3 new sites, still to download: one gravel or quarry site, one open field, one built-up site with water beside it | none (the first real hold-out footage will be the event-day recordings) | 101 to 120 |

Blind trials (random hidden camera, drone and wind; see the night plan, step 4f) draw their own
values from seeds 101 to 120 when they give final results.

## Tuning log
| When | What was tuned | On which data | By |
|---|---|---|---|
| 2026-11-15 | LK minEigThreshold 3e-3 | synthetic static (std 3) and sim texture | the team |
| 2026-11-15 | mask thresholds, break-up cue z = 6 | clip-1 | the team |
| 2026-11-15 | variance model constants (honest.py) | sim runs on ottawa2019fall and other sites, seeds listed in review/estimator | the team |
| 2026-11-16 | Hold v2 thresholds (NCC gate, static-point gate, keyframe rule), compute knobs | mostly ottawa2019fall at 100 m, plus suburb and prairie; clip-1 replay | the team |
| 2026-11-16 | gyro-attitude variant for the closed loop | ottawa2019fall, 50 and 100 m, medium vehicle, seed 1 | the team |
| 2026-11-16 | Hold liveness: LIVE_STILL_PX 0.3 px still proof and the follows-the-attitude move proof (chosen, not swept); Pipe keeps its good-fix count over single failed frames | ottawa2019fall and prairie replays of runs apb_100m_calm and apb_100m_w20g40, seed 1; closed loop prairie 100 m seed 1 | the team |
| 2026-11-16 | tried and dropped (not in the code): capture sensor_noise_min 1.0 (flat picture without sensor noise = no signal), Hold MIN_STRONG 6/8 strong-point frames, MIN_PTS 15 | snow_tarps (synthetic, now tuning data) and prairie replays, seed 1 | the team |
| 2026-11-16 | Hold KF_LOW 0.8 (no forced keyframe change while n is above 0.8 x the keyframe's first n; 0.9 and 0.95 tried) | prairie replay of run apb_100m_w20g40, seed 1; checked on farmland and ottawa2022 (validation) | the team |
| 2026-11-16 | att_source default (gyro kept, imu built and compared) | closed loop ottawa2019fall 100 and 122 m, calm and w20g40, validation seeds 11-13 | the team |
| 2026-11-16 05:35 | (rule 2) holdout_field failed in closed loop on PX4 and ArduPilot, and the next build works on low-texture ground because of it. It moves to VALIDATION. New hold-out site for low texture: holdout_lawn (not yet looked at). Hold-out sites now: holdout_quarry, holdout_water, holdout_lawn. | - | the team |
| 2026-11-16 | Hold whole-picture path (low texture, a few strong marks): cubic warp instead of a homography (fisheye), DENSE_SCALE 0.25 (0.5 tried: same error, twice the compute; closed loop at 0.5 on holdout_field s11 gave 7.6 fixes/s at 116 ms per frame), DENSE_RESP 0.4, DENSE_NCC 0.25 (0.2, 0.3, 0.5 tried), DENSE_AGREE_PX 4.0 (1.0, 3.5 tried), DENSE_PTS_PX 2.0 (assumed), fallback also after a corner-path "no ground" | open-loop replay of run apb_100m_w20g40 (seed 1) on prairie (tuning), holdout_field (validation), snow_tarps_pad 50 m and night_lights 50/100 m (tuning); bad_pictures.py cases (seeds 0-2) for the bad-picture side | the team |
| 2026-11-16 | capture judge: a flat picture with MARKS_MIN 3 strong marks (MARK_GREY 25 grey at 1/4 size) is not "no signal" (both assumed, not swept) | snow_tarps_pad 50 m and night_lights 50/100 m renders | the team |
| 2026-11-16 | new synthetic site night_lights (px4sim/camera.py, with exposure_s 0.02 and noise 10): TUNING data. Its lights were made brighter (1 m core, glow 120) after the first 100 m picture showed 4 of 6 lights | night_lights renders | the team |
| 2026-11-16 | Hold whole-picture path solved in metres of drone move (per-pixel px/m from the lens), not as one pixel shift: the pixel-shift version ran away in closed loop (holdout_field seed 11: 4.7 m RMS, 17 m max, confident fixes; prairie seed 1: 1.16 m RMS) because a fisheye shows one ground move as a smaller shift off-centre | closed loop holdout_field seed 11 (validation) and prairie seed 1 (tuning); open-loop replay of that holdout_field run | the team |
| 2026-11-16 | Hold: no whole-picture fallback after "consistent points"/"points left" or when re-finding a corner keyframe; DENSE_YAW 20 deg keyframe change (assumed); capture NOISE_MIN 1.0 grey (assumed) so a noise-free NO SIGNAL screen stays "flat" | tests/run_regression.py E3 (seed 3 locked onto a moving object), E4/E6 (yaw swings), tests_apb test_basics judge case; synthetic scenes only | the team |
| 2026-11-16 | New indoor floors in px4sim/camera.py (looks assumed): floor_concrete, floor_tiles, floor_carpet, floor_wood, floor_shiny (+ reflect) are TUNING data. holdout_floor_a and holdout_floor_b are HOLD-OUT: built and run only for the final indoor table, never looked at or used to choose a setting | - | the team |
| 2026-11-16 | Indoor profile by settings overrides only: min_height_m 1.0, hold_cfg h_min 1.0 (assumed: 1 m, under the 2 m trial floor), indoor camera exposure 10 ms and noise 5 grey (assumed) | closed loop on the tuning floors, 2-15 m, seeds 1-4 | the team |
| 2026-11-16 | Hold ROW_NORM (row gain normalization against mains flicker), off by default, on in the indoor profile (hold_cfg row_norm true); placed after the picture checks (before them it let 5 of 90 broken frames through) | open-loop replay of in_conc_5m_s1 (floor_concrete, tuning) with flicker 100/120 Hz; tests/bad_pictures.py with HOLD_CFG row_norm | the team |
| 2026-11-16 | Hold LIVE_STILL_PIC_PX 1.5 (picture part of the "still" live proof; was LIVE_STILL_PX 0.3), chosen by argument, not swept | open-loop replay of in_tiles_10m_s1 (tuning) with and without frozen pictures; closed-loop A/B tiles 10 m seed 4 | the team |
| 2026-11-16 | Propeller corner mask radius 0.36 x picture height: read off the simulator's own propeller drawing (not swept) | render check only | the team |
| 2026-11-16 | PX4 EKF2_EVP_NOISE 0.02 (not set by default; tried) and EKF2_DELAY_MAX 200 vs 300 (A/B for the control work) | closed loop floor_concrete 5 m seed 1, ottawa2022 100 m w20g40 seed 11, snow_tarps_pad 50 m seeds 1, 11, 12 | the team |
| 2026-11-16 | Tilt from the picture: open-loop study only (integ/indoor/tilt_exp.py, synthetic points, seeds 11-19); not built into Hold | synthetic | the team |
| 2026-11-16 21:00 | apsim board model for the control work (not tuned, a simulator choice): every Hold.process takes a fixed 80 ms (cpu_model "fixed", the mean of the clean wall-model runs), because laptop load (average up to 41) halved the fix rate of the wall model; SITL SERIAL1_BAUD 115 to match the 11520 B/s relay | clean runs in apsim/results_clean.txt (tuning seeds 1-3) | the team |
| 2026-11-16 21:15 | ArduPilot VISO_DELAY_MS 100 -> 140 (default in settings): from ArduPilot's own log in closed loop (fixes fused 134-144 ms after capture) and apb_main.delay_advice (144); checked 100 against 140 | ottawa2019fall, 100 m w20g40, VALIDATION seeds 11-13 | the team |
| 2026-11-16 21:30 | trust: cov_scale 0.1 with VISO_POS_M_NSE 0.15 tried and dropped (worse worst case 1.47 against 0.87 m RMS, tilt wobble 9.5 against 6.9 deg); VISO_POS_M_NSE stays 0.2, cov_scale 1 | ottawa2019fall, 100 m w20g40, VALIDATION seeds 11-13; fix error measured on tuning seeds 1-3 | the team |
| 2026-11-16 21:30 | VISO_VEL_M_NSE 0.3 for the speed test: speed error of a 0.4 s fix slope 0.15-0.29 m/s | apsim runs ap_apb_*_s1-3 (TUNING seeds) | the team |
| 2026-11-16 21:30 | wiggle timing (apb_main.py timing): 8 s stick wiggle, 150 PWM at 0.7 Hz (assumed, not swept); calib.py search ranges unchanged | 100 m seed 11, 40 m seed 12 (VALIDATION), start 40 ms wrong | the team |
| 2026-11-16 21:40 | speed (send_speed, EK3_SRC1_VELXY 6, VISO_VEL_M_NSE 0.3) tried and dropped: version 1 (vz from the height slope) flew away at 100 m (EKF variance failsafe); version 2 (vz from the FC) 100 m 3.18 / 2.39 m RMS against 0.77 / 0.87 without, 40 m 0.43 / 0.47 against 0.37 / 0.28. send_speed stays false | ottawa2019fall, 100 and 40 m w20g40, VALIDATION seeds 11-12 | the team |
| 2026-11-16 21:55 | ArduCopter 4.6 position controller and EKF drag, one at a time then combined: PSC_POSXY_P 0.7/1.5, PSC_VELXY_P 3, PSC_VELXY_I 2/3, PSC_VELXY_D 0.25/1.0, PSC_JERK_XY 10, LOIT_ACC_MAX 1000, EK3_DRAG_BCOEF_X/Y 14 (assumed from the SITL quad's ref speed and angle), combinations I2+P1.5, I2+P1.5+VELP3; chosen PSC_VELXY_I 2 + PSC_POSXY_P 1.5 | stand-in feed (variant truth: true position 150 ms old, 0.1 m noise, 12 Hz) at 100 m w20g40, ottawa2019fall, VALIDATION seeds 11-12; then apb at 100 m seeds 11-13 and 40 m seeds 11-12 | the team |
| 2026-11-16 22:30 | wind assist (experiment, off by default): version 1 (target moved by the whole push) dropped, 100 m 1.70 / 2.38 m against 0.77 / 1.27 m; version 2 (only the change of the push, assist_slow_s 2 s, assist_tau_s 0.5 s, assist_max_m 1.5 m, all assumed, not swept); version 3 = version 2 with both filters started at the push of the moment (version 2 jumped upwind when GUIDED came 5 s into the hold); compared with plain Loiter, GUIDED selected 5 s into the hold | ottawa2019fall, 100 and 60 m w20g40, VALIDATION seeds 11-13 | the team |
| 2026-11-16 22:20 | profile choice fixed at 22:06 from validation, before the first hold-out result (22:11): goal A stock gains, goal B PSC_VELXY_I 2 + PSC_POSXY_P 1.5; the 40 m seed-13 run came later and agreed (0.23 against 0.49 m RMS); nothing was changed after a hold-out result. One hold-out run (c_hoD_100m_holdout_lawn_s104) was cut by the 22:12 restart and rerun; its partial number was not used | VALIDATION seeds 11-13 | the team |
| 2026-11-21 04:50 | Hold options DENSE_LOCAL (the integ/lowh patch), DENSE_TURN (the whole-picture path solves the yaw turn), DENSE_NADIR_DEG (off) and att_source gyro_fcyaw; choice of DENSE_LOCAL + DENSE_TURN + gyro_fcyaw (profile low_height) over the base, each part alone, and the 55 degree limit | open loop: integ/diag10 replays of the TUNING flights lowap 10 m seeds 1-2. Closed loop: ArduCopter SITL ottawa2019fall (TUNING) seeds 1-2 at 5 and 10 m. Check on other data: open loop VALIDATION ottawa2022 and farmland seeds 11-13 at 10 m; closed loop VALIDATION ottawa2022 seeds 11-13 at 10 m (0.74 / 0.65 / 0.94 -> 0.34 / 0.27 / 0.50 m RMS) and at 5 m, farmland seeds 11-13 at 10 m (0.32 / 0.33 / 0.55 -> 0.33 / 0.25 / 0.46 m) and seeds 11-12 at 5 m. 20 m: 1 TUNING seed (not better). 30 m: 2 TUNING seeds (no difference). 60 m: 3 TUNING seeds (worse in 2). 60 SITL runs in all (integ/kfmap/closed/ab_table.txt, pairs.txt). Rule from these runs: the profile is only for holds at 5 to 10 m. No HOLD-OUT data used | integ/kfmap, integ/diag10 (sessions of 2026-11-21) |
| 2026-11-21 04:50 | pilot hover before the switch: 15 s in place of 4 s (a procedure, no setting) | ArduCopter SITL ottawa2019fall (TUNING) seeds 1-2 at 5 and 10 m: 0.79 / 0.71 -> 0.51 / 0.41 m RMS at 10 m (integ/kfmap/closed/ab_table.txt) | integ/kfmap |
