integ/diag10: why the fix error at 10 m is about 1 m (2026-11-21, work in progress; simulation only)
=====================================================================================================

Status: IN PROGRESS. This file is updated after each step. Times are EST.

Rules kept: only files under integ/diag10/ were written. No SITL, no Docker. Replays run through gate_run.sh
(nice -n 19, OMP/OPENBLAS/MKL threads 1, cv2.setNumThreads(1), waits while arducopter/px4/ap_sim*/px4_sim* or
integ/kfmap/SITL_ACTIVE exist), at most 2 at a time (run_jobs.sh).
hold_base.py = repository hold.py copied at 03:00 (md5 051dd786..., = the A/B 'before').
hold_patch.py = hold_base.py + integ/lowh/proposed.patch (md5 eb0c0293..., = the A/B 'after').

Data split (DATA_SPLITS.md): every closed-loop SITL run at 10 m is on ottawa2019fall with seeds 1-2 = TUNING.
Diagnosis and any choice are made on those. VALIDATION = ottawa2022 / farmland with the flights
c_d140_100m_w20g40_s11..13 shifted to 10 m (seeds 11-13): final numbers only (not done yet).

Step 1. Closed-loop logs (cl_budget.py -> cl_budget.jsonl; cl_series.py)
-------------------------------------------------------------------------
lowap_apb_10m_w20g40_s1 (apsim/runs): the sent-fix error is NOT a series of jumps. It is a slow, one-way drift:
north error -0.45 m at -1.9 s, -1.03 at 4 s, -1.46 at 15 s, -1.93 at 33 s, then about -1.7 m to the end
(cl_series.py output). The EKF position follows the fix (ekf_err = fix err within 0.05 m), so a re-anchor to the
FC position keeps the error that exists; it does not cause it. Increments between sent fixes in the hold
(cl_budget.jsonl): whole-picture path steps sum -0.49 m N, keyframe events -0.17 N, re-anchors (4) -0.21 N,
corner steps -0.20 N. Re-anchor steps: RMS 0.10 m each, 4 per hold. Height the program used: 1 cm under the truth.
FC attitude against truth in the hold: roll/pitch within 0.3 deg; yaw error changes by about 2 deg.

Step 2. Open-loop replay of the SAME closed-loop flights (replay.py; results/*.json.gz; logs/jobs1.log)
-----------------------------------------------------------------------------------------------------
replay.py renders the recorded true motion of the SITL run on ottawa2019fall and runs hold_base / hold_patch.
'last' = Pipe re-anchors at Hold's last fix (as closed loop, where the FC position = our last fixes); 'truth' =
re-anchor at the true position. 'fc' = Hold gets ArduCopter ATTITUDE as sent; 'gyro' = apb_main.GyroAtt (rates
integrated, leaning on the FC attitude with tau 20 s; the onboard program's setting att_source "gyro", used in
every SITL run). Error RMS of the sent fixes over 58 s from the switch (m), TUNING:

  flight (10 m)     att    re-anchor   diag                 base    patch
  lowap s1          fc     truth       -                    0.129   0.126
  lowap s1          fc     last        -                    0.281   0.167
  lowap s1          gyro   truth       -                    0.175     -
  lowap s1          gyro   last        -                    1.003   0.952   <- reproduces closed loop (1.00 m)
  lowap s2          gyro   last        -                    0.705   0.619   <- closed loop 0.96 m
  lowap s2          fc     last        -                    0.235   0.234
  lowap s1 / s2     gyro   last        true_height          0.908 / 0.701   0.930 / 0.690
  lowap s1 / s2     gyro   last        vib0                 0.986 / 0.698   1.027 / 0.617
  lowap s1 / s2     gyro   last        exact_lens           0.951 / 0.823   1.266 / 0.648
  lowap s1          gyro   last        gs (no rolling sh.)  1.034           1.015
  lowap s1 / s2     fc     last        true_att             0.163 / 0.149   0.162 / 0.151

First result: the attitude feed is the main source. With the onboard program's GyroAtt the replay gives the
closed-loop error (1.00 / 0.71 m); with the FC attitude as sent 0.28 / 0.24 m; with the true attitude 0.16 / 0.15 m.
Height, vibration, lens and rolling shutter change the gyro result by less than the run-to-run spread.
GyroAtt yaw against the truth (gyro_err.py and the table in this file's step 3): +2.5 deg at the switch, -1.9 deg
at 30 s (s1); the FC's own yaw had a step of about 13 deg at its in-flight yaw alignment 9 s before the switch,
and GyroAtt follows the FC yaw only with tau 20 s. Roll/pitch of GyroAtt stay within 0.5 deg.

Step 3. Which part: yaw, tilt, chain, re-anchor (logs/jobs1.log, logs/jobs2.log; split.py on results/)
------------------------------------------------------------------------------------------------------
Same flights, GyroAtt feed, re-anchor at the last fix (the closed-loop case). Error RMS (m), base / patch:

  lowap s1 / s2   true_yaw  (GyroAtt roll+pitch, true yaw)       0.211 / 0.166    0.200 / 0.102
  lowap s1 / s2   kf_true   (every keyframe placed at the truth)  0.118 / 0.100    0.146 / 0.221
  lowap s1        re-anchor at the truth (chain kept)             0.175            -
  lowap s1 / s2   all ideal (true att + height, no vibration,
                  true lens, global shutter), FC feed             0.172 / 0.069    0.084 / 0.072

Result: the yaw of the attitude feed is the main cause. With the true yaw the error drops from 1.00 / 0.71 to
0.21 / 0.17 m. The keyframe changes carry it: with keyframes at the truth the same yaw error gives only 0.12 / 0.10 m.
Mechanism (measured, the lever is checked in step 4): the whole-picture path (93-96 % of the sent fixes at 10 m)
measures only a 2-D move; the turn comes from the attitude. A yaw error change since the keyframe is read as a move
perpendicular to the lean, and each keyframe change freezes that error into the next keyframe. In s1 the GyroAtt
yaw error went from +2.5 to -1.9 deg in the first 30 s (a slow follow, tau 20 s, of the FC's in-flight yaw
alignment step 9 s before the switch) and the north error went from -0.08 to -1.22 m: about 15 m of error per rad
(1.5 x the height). From 30 to 55 s the yaw error came back +0.5 deg and the error +0.15 m (same ratio).
Split of the gyro/last error (split.py): whole-picture steps sum -0.56 m N (s1) and -0.54 m N (s2); keyframe
events themselves 0.05-0.13 m; re-anchors 0.38-0.41 m (end value, the drone's move during the lost time).
Residual with ideal feeds: s1 0.17 m, nearly all from whole-picture steps (sum -0.27 m N): a geometry error of the
whole-picture path itself (not checked further).

Step 4. Proposed changes, open loop on TUNING (running: logs/jobs3.log, logs/cmds4.log)
-------------------------------------------------------------------------------------
hold_pn55.py   = hold_patch + whole-picture pixels only within 55 deg of nadir (make_variants.py nadir)
hold_pn40.py   = the same with 40 deg
hold_py.py     = hold_patch + the whole-picture path solves the yaw turn too (make_variants.py yaw)
hold_pyn55.py  = both
feed: --att gyro_fcyaw = GyroAtt roll/pitch with the FC's yaw as sent (an apb_main change, emulated in replay.py)
probe_yaw.py   = the lever (m of fix per rad of yaw error) of each version, corner and whole-picture path

Step 3b. Caution about the GyroAtt start (found 03:30)
-----------------------------------------------------
replay.py fed GyroAtt from 12 s before the switch (--att_lead 12, as integ/lowh/ol_low.py). The onboard program runs
GyroAtt from its own start, before take-off: the raw record of reh_jd_wig_10m_s31 (integ/rehearsal_2026-11-20/runs/
reh_jd_wig_10m_s31_raw) has an ATTITUDE stream with no gap over 11 ms from -23 s on, so GyroAtt never restarts.
Started 12 s before the switch, GyroAtt took the FC yaw from just before the FC's in-flight yaw alignment step
(-9 s) and its yaw error then fell by 4.4 deg in the first 30 s (s1). Started at the first ATTITUDE row (-23 s,
yaw_course.py), the change is smaller: +0.2, -1.3, -0.9, +0.4 deg at 0, 10, 30, 60 s (s1). So the steps 2-3 numbers
with 'gyro' may show a larger yaw error than the closed loop had. Reruns with --att_lead 60 (all rows): logs/cmds5.log.
Also measured: the onboard program's ATTITUDE time stamps are 4.0 ms later than the supervisor's (same samples,
residual 0.001 deg); this is a 4 ms image-to-attitude offset, small (rate x 4 ms).

Step 5. 5 m against 10 m against 20/30 m, closed loop (cl_budget.jsonl, yaw_course.py; ArduCopter SITL, ottawa2019fall,
TUNING seeds; w20g40 unless noted)
-----------------------------------------------------------------------------------------------------------------
  height  runs (ferr60 m = sent-fix error over the hold)                               whole-picture share of fixes
  5 m     A/B before 0.20, 0.22, 0.36; after 0.18, 0.17; lowap s1 0.75 (14 re-anchors), s2 0.09    0.98-0.99
  10 m    A/B before 1.06, 0.96, 1.00; after 0.98, 0.89, 0.75; lowap s1 1.00, s2 0.96;
          gn_none_10m_s2 0.84; arm_none_s2 0.73; reh_jd_wig_10m_s31 0.52                           0.92-0.96
  20 m    no wind run exists. jd_apb_20m_calm_s11 (calm, seed 11 on a TUNING ground): abs. error RMS 0.26 m
          (ferr60 not defined: no sent fix in the last 1 s before the switch)                    0.62
  30 m    gn_none_30m s1 0.13, s2 0.14; eng_30m_off 0.21, 0.26; eng_30m_on 0.16, 0.16              0.10-0.68
Open-loop at 20 m on ottawa2022: the corner path gives every fix (integ/lowh/README.txt section 1).

Files
-----
replay.py      open-loop replay of a recorded SITL flight, several hold.py copies, diagnostic feeds (see its header)
run_jobs.sh    runs a job list 2 at a time through gate_run.sh; jobs1.txt, jobs2.txt: the job lists
cl_budget.py   closed-loop split of the sent-fix error from the onboard logs (cl_budget.jsonl)
cl_series.py   closed-loop fix error and EKF error over time for one run
slope.py       scale of the measured motion (fix increments against true increments)
split.py       split of a replay's error by what happened between fixes (re-anchor, keyframe, path)
gyro_err.py    attitude error of the FC feed and of GyroAtt against the truth
rate_check.py  ATTITUDE body rates against the angle changes
att_vs_err.py  replay fix error against the attitude error of the feed

STATUS at 03:32 (the run of this study ended here; the caller asked for the report now)
--------------------------------------------------------------------------------------
DONE: steps 1, 2, 3, 3b, 5 above (all TUNING data).
QUEUED, running in the background through run_cmds.sh (2 at a time, nice 19, gated on integ/kfmap/SITL_ACTIVE):
  cmds5.txt -> logs/cmds5.log: closed-loop-like GyroAtt (--att_lead 60) baseline, true_yaw, kf_true, gyro_fcyaw,
               the fix variants (hold_pn55, hold_py, hold_pyn55), probe_yaw.py, true_tilt, fc feed.
  cmds6.txt -> logs/cmds6.log: VALIDATION (c_d140_100m_w20g40_s11-13 at 10 m, ottawa2022 and farmland) with base,
               patch, py, pyn55; heights 5 and 20 m (TUNING); gyro_fcyaw on ottawa2022.
  cmds7.txt -> logs/cmds7.log: --att_lead 60 reruns of true_height, vib0, exact_lens, gs, true_att.
  Runner PIDs at 03:32: 176378 (cmds5), 176920 (cmds6), 177605 (cmds7). To stop all: kill these PIDs and every
  process whose command line contains "integ/diag10/replay.py" or "integ/diag10/probe_yaw.py".
  Read results with: ~/.venvs/vio/bin/python integ/diag10/summ.py L60   (and summ.py VALID... by label)
NOT DONE: the fix variants have no result yet (not one replay of hold_pn55/py/pyn55 finished). No validation
number. No safety test (tests/bad_pictures.py, run_regression.py) of any variant. No closed-loop test.

03:40 correction: hold_py.py and hold_pyn55.py first crashed ("cannot reshape array of size 6 into shape (2,2)": Jc read
P[0, 2:] after the warp got two more columns). Fixed in make_variants.py (Jc = P[0, 2:6]) and both files rebuilt at
03:40. The two jobs that crashed (cmds5 lines 1-2: --att_lead 60 baseline with base, patch, pn55, py, pyn55) are
queued again as cmds8.txt -> logs/cmds8.log, after cmds7 (runner waits for PID 177605). Queued jobs of cmds5/cmds6
load the fixed files when they start.
First --att_lead 60 results (logs/cmds5.log, TUNING): true_yaw s1 0.224 / s2 0.163 m (base), 0.201 / 0.102 (patch):
the same as with --att_lead 12, so the true yaw removes most of the error in both feeds.

03:37 queue change (coordinator: yaw finding is priority 1). All older runners stopped (none had a replay running).
  cmds9.txt  -> logs/cmds9.log : FIRST. cmds8 (closed-loop-like baseline --att_lead 60 with base, patch, pn55, py,
                                 pyn55, flights s1, s2) then the 6 VALIDATION jobs at 10 m (c_d140_100m_w20g40_s11-13
                                 on ottawa2022 and farmland; base, patch, py, pyn55). Runner PID 191198.
  cmds10.txt -> logs/cmds10.log: the held-back rest of cmds5 (lines 5-13), cmds6 (lines 7-13) and cmds7; starts after
                                 runner 191198 ends. Runner PID 191658.
  Still at most 2 replays at a time: the coordinator allowed 6 until 04:20, but the user's brief says at most 2, and the
  permission system refused the 6-process start. Gate unchanged (simulators, integ/kfmap/SITL_ACTIVE).

03:37 RESULTS of cmds8 (closed-loop-like GyroAtt start, --att_lead 60; logs/cmds9.log; summ.py L60) - CORRECTION
---------------------------------------------------------------------------------------------------------------
TUNING, lowap 10 m flights, re-anchor at the last fix, error RMS (m):
  version   s1      s2       (closed loop on these flights: 1.00 / 0.96)
  base      0.279   0.200
  patch     0.269   0.136
  pn55      0.246   0.131    (patch + whole-picture pixels within 55 deg of nadir)
  py        0.191   0.213    (patch + whole-picture path solves yaw)
  pyn55     0.123   0.067    (both)
  true yaw (base / patch): s1 0.224 / 0.201, s2 0.163 / 0.102
CORRECTION of steps 2-3: with the closed-loop-like GyroAtt start, the open-loop replay gives 0.28 / 0.20 m, NOT the
closed-loop 1.0 m. The 1.00 / 0.71 m of steps 2-3 came from the 12 s GyroAtt start (a 4.4 deg yaw transient that the
closed loop did not have). So the open-loop replay does NOT reproduce the closed-loop 1 m. The yaw-lever mechanism
is real (a 4.4 deg yaw change gave 1 m), but with the closed-loop yaw course it explains about 0.1 m of the 0.28 m
(true yaw: 0.28 -> 0.22, 0.20 -> 0.16). The rest of the closed-loop 1 m is NOT explained by this study. Candidates
not tested: closed-loop feedback (the fix error moves the drone, which changes what the camera sees), the EKF
position used at re-anchors, effects that a replay of the same motion cannot show.
VALIDATION, 10 m (c_d140_100m_w20g40_s11 shifted to 10 m), error RMS (m), first seed only at 03:37:
  ground       base    patch   py      pyn55
  ottawa2022   0.417   0.612   0.065   0.081   ('points disagree' refusals: 18, 114, 78, 140)
  farmland     0.107   0.072   0.094   0.094
Seeds 12 and 13: running (logs/cmds9.log). Read all with: ~/.venvs/vio/bin/python integ/diag10/summ.py c_d140

03:38 VALIDATION complete (cmds9; c_d140_100m_w20g40_s11-13 shifted to 10 m, GyroAtt --att_lead 60, re-anchor at the
last fix; error RMS of sent fixes, m; summ.py c_d140 L60):
  ground / seed     base    patch   py      pyn55
  ottawa2022 s11    0.417   0.612   0.065   0.081
  ottawa2022 s12    0.601   0.509   0.114   0.091
  ottawa2022 s13    0.279   0.335   0.065   0.069
  farmland   s11    0.107   0.072   0.094   0.094
  farmland   s12    0.312   0.357   0.084   0.207
  farmland   s13    0.275   0.214   0.090   0.048
  median [worst]    0.296 [0.601]  0.346 [0.612]  0.087 [0.114]  0.086 [0.207]
py (the whole-picture path solves yaw) is lower than base in 6 of 6 cells and has the lowest worst case. pyn55 has
the same median but a worse worst case (farmland s12 0.207). The patch alone does not help here (median 0.35).
TUNING (cmds8) gave pyn55 0.123 / 0.067 and py 0.191 / 0.213 against base 0.279 / 0.200. The variants were chosen by
argument, not tuned; no setting was changed after these results. Not tested: safety suites, closed loop.

04:06 results of the held-back queue (cmds10; logs/cmds10.log; all TUNING unless marked; --att_lead 60; error RMS m)
------------------------------------------------------------------------------------------------------------------
Lever of a yaw error, measured directly (probe_yaw.py -> results/probe_yaw_lowap_apb_10m_w20g40_s1.json; 10 m,
lowap s1 frames, +1 deg yaw on one frame; median over the frames that gave a fix; some frames refused):
  base 14.8, patch 15.1, pn55 6.9, py 0.07, pyn55 0.02 m of fix per rad of yaw error.
  (The 'corner' lines print the same numbers: at 10 m on this ground the corner path falls back to the whole-picture
  path on these frames, so that mode did not test the corner path.)
Diagnostics, lowap s1 / s2 (baseline gyro 0.279 / 0.200):
  kf_true (keyframes at the truth)   0.096 / 0.081   -> the keyframe chain carries most of the open-loop error
  true_tilt (true roll and pitch)    0.286 / 0.284   -> tilt error of the feed is NOT a cause
  true_yaw                           0.224 / 0.163   (from 03:37)
Feeds (base / patch / py), s1 ; s2:
  fc (ATTITUDE as sent)              0.281 / 0.167 / 0.098 ; 0.235 / 0.234 / 0.089
  gyro_fcyaw (GyroAtt tilt, FC yaw)  0.270 / 0.265 / 0.259 ; 0.216 / 0.226 / 0.085
  gyro (onboard default)             0.279 / 0.269 / 0.191 ; 0.200 / 0.136 / 0.213
  -> the feed choice alone changes little; py helps in 4 of the 6 feed/flight cells.
Height, same motion shifted (base / patch / py), s1 ; s2:
  5 m    0.381 / 0.345 / 0.334 ; 0.574 / 0.226 / 0.094   (11-15 re-anchors in base)
  10 m   0.279 / 0.269 / 0.191 ; 0.200 / 0.136 / 0.213
  20 m   0.120 / 0.120 / 0.120 ; 0.184 / 0.184 / 0.184   (corner path gives every fix; no whole-picture fix)
  -> open loop: 5 m is not better than 10 m; at 20 m the corner path (which measures the turn) takes over.
VALIDATION gyro_fcyaw, ottawa2022 s11: base 0.216, patch 0.160 (gyro feed: 0.417, 0.612). Others still queued.

POWER NOTE (added by the coordinating session 0a at 06:50 EST, 2026-11-21): the laptop ran on battery from at least 04:33 EST (upower history; the first 'discharging' row found), and session 1f saw CPU power-save mode at 06:49 with 9 % battery. Runs after about 04:33 (most of the SITL A/B, the regression and the suites) ran on battery power; power-save may have started earlier than 06:49. Timing numbers (E9/H6, process() ms, test time limits) from that period can be slower than on mains power. The SITL board model uses a fixed 80 ms per Hold.process, but the SITL itself ran slower; each run's start load is in its table. The off/on comparisons ran under the same conditions for both versions.
