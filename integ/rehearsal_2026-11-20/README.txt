EVENT-DAY REHEARSAL ON SIMULATORS, JUSTICE D CAMERA PATH (2026-11-20, 03:18 to 04:15)

Words: VIO = visual-inertial odometry. FC = flight controller. APB = Auto Pilot Board. SITL = software in the loop
(a simulator). GNSS = Global Navigation Satellite System. SD = Secure Digital card. eMMC = the board's own flash
memory. VTx = video transmitter. RMS = root mean square.

How the rehearsal ran
- "The laptop": a copy of the program folder (program files, settings.json, tools). "The board": a second folder,
  reached with BOARD=local in tools/deploy.env (deploy.sh's own test mode). The project's settings.json was only read
  (md5 be7b5bea6b6c... before and after, profile_combos.txt).
- "The aircraft": ArduCopter 4.6 SITL (integ/rehearsal_2026-11-20/sitl_ap.sh, instance 7; FC link on udp 14540 as on
  the board). The pilot's switch: gcs.py (asks the FC for a mode). PX4 SITL was not used for these steps.
- "The camera": no Justice D on the laptop. check first showed "cannot open the camera at /dev/video2"; then
  video_command = cam_stream.py (GREY 1280x720 at 60 fps from a sim_rec Justice D recording, the step 2 fallback).
- Flights: apflights.py, ArduCopter SITL with the Justice D in the loop (apsim/sitl_ap.run_case), wind 20 gusting
  40 km/h, seed 31, one flight per height. Recordings for tuning: tools/sim_rec.py --video JUSTICE_D.
- Step log with clock times: steps_log.txt. Load average before each timed run: in each result file.
- The truth of the Justice D sim camera (px4sim/sitl.py VIDEO_DEFAULTS): video delay 40 ms, readout +14 ms, true lens
  "sim Justice D near crop" (118 x 64 degrees). Hand-turn tests (calib4.py): delay 58 ms, readout +10 ms.

tools/sim_rec.py: new option --video JUSTICE_D. It uses px4sim's camera model and VIDEO_DEFAULTS, the justice_d
overlay, lens_measured = camera.calib_lens (as sitl.model_lens), and Raw's record_hz (30 pictures each second).
Proof that the default output did not change: check_sim_rec_analog.py runs the committed and the new sim_rec.py on
px4sim/runs/apb_100m_w20g40.npz, 20 s: settings.json and both chunks (250 frames each) are identical in every array
(check_sim_rec_analog.txt).


1. STEPS (results: OK, FAIL, NOT CLEAR, CANNOT REHEARSE)

EVENT_DAY.txt
step | command | clock, time | result | change in the document
0 | python3 tools/use_profile.py apply outdoor_day goal_a ardupilot justice_d; show; NO_RESTART=1 tools/deploy.sh | 03:47:46, under 1 s each | OK at 03:47. FAIL since 03:50: a new tools/profiles/betaflight.json stops every apply and show (section 3, D5) | Moved to the laptop (deploy.sh does not copy tools/use_profile.py or tools/profiles to the board, so the old "run each command on the APB" failed). Added no_sd for a board with no SD card, "pull first" later in the day, and the disk numbers. Mask and calib again only after a capture change.
header | (rules) | - | NOT CLEAR | New "rules for every step": propellers off with a cable, battery indoors, board power from the battery, antennas, VTx 60 s, camera screw, orqa_dt_switcher, gusts 46 km/h, edit settings.json on the laptop only, stop the program before check/mask/calib/bench, laptop Python.
1 | python3 apb_main.py check --set-params; reboot (SITL restart, parameters kept) | 03:48:37, 9 s; reboot 2 s | OK: 6 parameters written, all param lines PASS | Pass rule named the "firmware" line, but that line never shows FAIL: an unsupported firmware fails the "flight controller" line. Added the route (board_check.sh --route), fc_device /dev/ttymxc2, Betaflight or INAV pointer, compass and level calibration, the battery and cable order.
2 | python3 apb_main.py check | 03:49:12, 8 s | FAIL (ArduPilot): "LOCAL_POSITION_NED 0 Hz" FAIL on the ground with GPS off (section 3, D2) | Pass rule exception for ArduPilot. Heading: a second pass rule (turn 90 degrees). EKF2_DELAY_MAX: params_px4 already has 300, the old rule (fix age + 100) could lower it. Origin and ATTITUDE pass rules moved here from step 7. Edits on the laptop.
2 | settings change on the laptop, then NO_RESTART=1 tools/deploy.sh (EIGHT_HOURS hour 2) | 03:49:03, 1 s | FAIL: "calibration differs (origin_lat_lon_alt)", the copy stops; FORCE=1 needed | EIGHT_HOURS hour 2 item 3: FORCE=1 after the origin.
3 | python3 apb_main.py mask | 03:50:14, 13 s | OK ("PASS mask covers 0%", no OSD in the stand-in) | None. It also saves a rec_ file: step 4 now says which rec_ file to use.
4 | python3 apb_main.py calib | - | CANNOT REHEARSE: the hand turn must move the camera and the FC together; the SITL FC sits on the ground | -
4 | python3 apb_main.py calib --rec <synthetic hand turn> (two recordings) | 03:50:34, 7 s and 5 s | OK: top left (true), delay 58 ms (true 58), readout +11 (true +10), "PASS delay 0 ms from the previous calibration" | Start-lens rule (section B).
4 laptop | tools/deploy.sh pull; python3 apb_main.py calib --rec logs/rec_<stamp>.npz; FORCE=1 NO_RESTART=1 tools/deploy.sh | 03:50:56: 0 s; fail; 7 s with the venv; 1 s | FAIL as written: python3 on this laptop has no cv2 ("No module named 'cv2'"). OK with ~/.venvs/vio/bin/python. The copy stops without FORCE=1, as written | Laptop commands use ~/.venvs/vio/bin/python.
5 | python3 apb_main.py check | 03:51:09, 8 s | FAIL (ArduPilot): "1 FAIL", the same LOCAL_POSITION_NED line; "ALL PASS" cannot happen | Exception text and the patch reference.
6 | python3 apb_main.py bench --height 1.5 (15 s, then SIGINT as Ctrl+C) | 03:51:28, 2 s; again 16 s | CANNOT REHEARSE the pass rules (hand moves). The command runs and prints lines. First try: a Python traceback, not a FAIL line (section 3, D3; cause here: the synthetic calibration's capture mode) | focal_scale edit on the laptop, then FORCE copy.
6B (new) | tools/deploy.sh; pilot moves the switch (gcs.py); tail -n 1 $(ls -t <log_dir>/run_*.csv \| head -1) | 03:51:54, 10 s; 03:52:10, about 40 s for 6 modes | OK: ACRO, ALT_HOLD, LOITER, POSHOLD, STABILIZE read right on the ground, disarmed. RTL gave "mode 11" in SITL | New step 6B (audit Table 1 row 4).
7 | the flight: apflights.py (ArduCopter SITL, wiggle 8 s, then the switch, 60 s hold) | 03:33 to 03:44, 99 to 118 s per flight | OK (section 2) | -
7 | python3 apb_main.py run --raw in an ssh session, then the session ends | 03:57:10, 35 s per test | FAIL: the program stops (A.2 below) | Item 1: tools/deploy.sh (setsid nohup). Unplug, then propellers, battery stays on.
7 | check while the program runs | 03:52:58, 7 s | FAIL: the running program lost the FC link ("attitude 0 Hz" in its run log for 39 rows) | Rule: tools/deploy.sh stop before check, mask, calib, bench.
7 | tools/deploy.sh stop | 03:53:21, 1 s | OK ("running: no"; last raw chunk saved) | Item 13.
7 | python3 apb_main.py timing logs/raw_<newest>_00N.npz | 1 to 3 s | NOT CLEAR: no rule to find the wiggle chunks, the path is not log_dir, and timing passes on hold chunks with wrong values (section 3, D1) | Item 15: a line that prints the chunks of the 14 s before the switch. Tested on 4 flights (section 2).
7 | python3 apb_main.py delay logs/run_<newest>.csv | under 1 s | NOT CLEAR: logs/ on the board is not log_dir | Item 16: <log_dir>/run_<stamp>.csv; item 17: edit on the laptop.
7 | tools/deploy.sh pull | 03:59:01, 3 s | OK | No SD card: delete the raw files after the pull.
recovery demo | - | - | CANNOT REHEARSE (time) | Pull first; start with deploy.sh.

EIGHT_HOURS.txt
hour | command | clock, time | result | change in the document
1 | ssh root@192.168.75.1 | - | CANNOT REHEARSE: no board (BOARD=local) | Battery power, three other ways in (APB manual).
1 | tools/deploy.env, NO_RESTART=1 tools/deploy.sh | 03:47:46, under 1 s | OK | Item 3 is now EVENT_DAY step 0, so the first copy has the Justice D settings (the shipped settings.json opens /dev/video0).
1 | sh tools/board_check.sh | 03:48:20, 7 s | OK as a run: 3 FAIL that are right on an x86 laptop (CPU, case A/B, no FC on its port at that moment) | Betaflight pointer.
1 | gpioset ...; date -s ... | - | CANNOT REHEARSE: no gpioset on the laptop; date -s needs root (no sudo) | The route now with board_check.sh --route; deploy.sh sets the clock at each copy (printed "clock: laptop ..., board ...").
2 | apply, show, origin, copy, steps 1 to 3 | 03:49:03 to 03:50:27 | FAIL at the copy (origin needs FORCE=1) | FORCE=1; VISO_DELAY_MS stays 140 until step 7; origin from a map, not from a GNSS receiver.
3 | steps 4, 5, 6, pull, 2b, cp good_calib.json | 03:50:34 to 03:51:44 | 2b FAIL with python3 (venv OK); 2c (lens_calib) not rehearsed: no checkerboard pictures | Venv python; step 6B; checkerboard lens is a development aid.
4 | deploy.sh start, flight, stop, pull, plot_log.py one log | 03:51:54 to 03:59:02; plot_log 1 s | OK. DECISION_CARD header "plot_log.py logs/run_*.csv" FAIL: "unrecognized arguments" with several logs | Card header: one log.
5 | tools/replay_tune.py on sim_rec recordings and on flown recordings | 45 to 67 s and 36 to 38 s (6 jobs) | NOT CLEAR / unsafe: section 2 | Merge only at 100 m or higher AND only with no wiggle result; FORCE copy after a merge.
6 | holds at 10, 30, 60, 100 m | flights above | OK (section 2) | None.
7 | tools/deploy.sh (restart, "running: yes") | 03:51:54, 10 s | OK. The timed trial sequence: CANNOT REHEARSE (no pilot) | Sequence with cable and propellers; battery change restarts.
8 | python3 tools/use_profile.py diff settings.json good_calib.json | 03:59:06, under 1 s | OK | None.

DECISION_CARD.txt (step 9): card_quotes.py checks each quoted message against the code; the quotes it cannot match are
built at run time, and each was read in the code or seen in a rehearsal output (card_quotes_before.txt, _after.txt).
Commands and options were checked with --help (apb_main.py, use_profile.py, replay_tune.py, plot_log.py,
lens_calib.py, log_summary.py, sim_rec.py) and in deploy.sh and board_check.sh. Errors found and fixed:
- Header: plot_log.py takes one log, not logs/run_*.csv. "After each change: tools/deploy.sh, then check" left the
  program running during check: now stop first.
- Row 5 and 12: logs/raw_*.npz mixes flights; one flight's chunks. Merge rules as EIGHT_HOURS hour 5.
- Row 7: EKF2_DELAY_MAX rule (300 already set). VISO_DELAY_MS from "apb_main.py delay", not the fix age.
- Row 14: the message is "under <min_height_m> m" (3 m outdoors now, not 15). The apply line dropped justice_d.
- Row 15: "Apply outdoor_night" alone dropped justice_d, the goal and the FC profile.
- Row 23: no-SD branch. Rows 6 and 16: camera screw, orqa_dt_switcher. New rows 39 to 42.

Step 8 (profile_combos.py, profile_combos.txt, run at 03:37 before betaflight.json existed): 21 combinations named by
the three documents applied with exit 0 and the Justice D camera. Two DECISION_CARD apply lines failed on a calibrated
Justice D copy: row 14 (indoor goal_b ardupilot) and row 15 (outdoor_night) set camera analog and /dev/video0 and
reset video_delay_s and readout_s to the analog defaults. Keys that two profiles set to different values: min_height_m
and hold_cfg.h_min (indoor 1.0 against outdoor_day and outdoor_night 3.0 and 2.0), record_hz (justice_d 30, no_sd
10: name no_sd last). New profile tools/profiles/no_sd.json (record_hz 10, min_free_gb 0.5): with it, check needs
0.44 GB; with justice_d alone 5.33 GB (formula in apb_main.mode_check; seen as "need 5.3 GB" in the step 1 check).
Since 03:50 every combination fails (profile_combos_after_betaflight_json.txt, 21 of 21).


2. STEP 7: FLY, RECORD, REPLAY, TUNE (true delay 40 ms, readout +14 ms, focal 1.0; wrong on purpose: delay 70 ms)

A. Recordings from PX4 SITL flights, rendered with sim_rec.py --video JUSTICE_D (the same flights as the 2026-11-19
rehearsal, plus 60 m), 60 s from the switch, 30 pictures each second. One recording per height, one run of each tool.
Error = sent fix minus truth, after the median offset (step7.py). Files: step7_<h>m.txt, runs/jd_<h>m/tune/tune.txt.
height | wrong value | replay_tune.py wrote | timing wrote | true value | error RMS: true / wrong / wrong + patch
10 m  | delay 70 ms | nothing (delay stays 70)        | FAIL 5 deg/s, nothing | 40 ms | 0.140 / 0.224 / 0.224 m
30 m  | delay 70 ms | delay 32.7 ms, focal 0.973     | FAIL 5 deg/s, nothing | 40 ms, 1.0 | 0.120 / 0.149 / 0.164 m
60 m  | delay 70 ms | focal 1.0816 (delay stays 70)  | FAIL 5 deg/s, nothing | 40 ms, 1.0 | 0.232 / 0.312 / 0.765 m
100 m | delay 70 ms | delay 41.5 ms                  | FAIL 5 deg/s, nothing | 40 ms | 0.338 / 0.481 / 0.340 m
replay_tune took 45 to 67 s with 6 jobs. Load (1 min) at its start: 3.40, 2.67, 2.96, 2.18.

B. Flights in ArduCopter SITL with the Justice D in the loop, the delay 30 ms too long during the flight, wiggle 8 s in
AltHold before the switch, 60 s hold, seed 31, one flight per height. The board model takes 80 ms per picture, so the
raw recording holds about 12 pictures each second. Files: reh_jd_wig_<h>m_s31.json, after_reh_jd_wig_<h>m_s31.txt,
timing_per_chunk_60m.txt, tune_hold_only_100m.txt. Load (1 min) at each flight start: 2.90, 3.22, 2.19, 2.82.
height | hold RMS / max with the wrong delay | timing on the chunks of EVENT_DAY step 7 item 15b | delay wrote | replay_tune (all chunks)
10 m  | 0.61 / 1.18 m | PASS 40 ms, +13 ms (chunks 000, 001)       | VISO_DELAY_MS 146 | not run
30 m  | 0.73 / 1.49 m | FAIL residual 1.28 deg (chunk 001), nothing | VISO_DELAY_MS 148 | not run
60 m  | 1.12 / 2.22 m | PASS 40 ms, +14 ms (chunks 001, 002)       | VISO_DELAY_MS 147 | delay 24.4 ms, focal 1.0225
100 m | 9.85 / 13.73 m | PASS 40 ms, +13 ms (chunks 002, 003)      | VISO_DELAY_MS 147 | delay 28.0 ms, focal 1.0145, max_pts 120, fb_every 3
True: 40 ms, +14 ms. The FC's true VISO_DELAY_MS was not measured in these runs (an earlier apsim log: 134 to 144 ms,
settings.py). timing on all chunks: 60 m FAIL (residual 1.24), 100 m PASS 36 ms. timing on each 60 m chunk alone: the
wiggle chunk PASS 38 ms; 5 of 5 hold chunks also PASS, with 12 to 52 ms and readout -29 to +42 ms.
replay_tune on the 100 m hold chunks only: delay 27.9 ms, max_pts 120.
Reading: at 60 m and 100 m one tune patch made the hold worse or was 12 ms short; the wiggle chunks gave the true
delay at 10, 60 and 100 m and refused at 30 m. One run each: these are examples, not rates.


3. DEFECTS IN FILES THAT THIS WORK MUST NOT CHANGE (scripts and patches in this folder; no patch is applied)
D1 apb_main.py, mode_timing: it uses every chunk it gets and passes on chunks of a hold in gusts with wrong values
   (60 m: 12 ms against a true 40 ms, readout -29 against +14 ms). The operator has no way to pick the wiggle chunks.
   Script: defect_timing_accepts_hold.sh (output timing_per_chunk_60m.txt).
   Patch: patches/apb_main_timing_window.patch: "timing RUN.csv RAW.npz..." keeps only the 15 s before the first
   "engaged" note. Tested: 38 ms (60 m) and 36 ms (100 m) from all chunks (patches/apb_main_timing_window_test.txt).
D2 apb_main.py, mode_check: on ArduPilot with GPS off, "FAIL LOCAL_POSITION_NED 0 Hz ... The FC sends no height" on the
   ground, every time. ArduPilot sends that message only when it has a position; the height comes from
   GLOBAL_POSITION_INT (fc_link._read). Step 5 "ALL PASS" can never pass.
   Script: defect_check_lpos_ardupilot.sh (needs ArduCopter SITL). Patch:
   patches/apb_main_check_height_stream_ardupilot.patch. Tested on SITL: "PASS GLOBAL_POSITION_INT 10 Hz"
   (patches/apb_main_check_height_stream_ardupilot_test.txt).
D3 apb_main.py, main: bench, mask and calib print a Python traceback, not one FAIL line, when the FC does not answer or
   run will not start. Script: defect_bench_traceback.sh (output defect_bench_traceback.txt).
   Patch: patches/apb_main_fail_line_not_traceback.patch.
D4 apb_main.py: check scales min_free_gb with the picture size and rate, Raw does not. With no_sd (10 Hz, 0.5 GB) check
   passes at 0.44 GB while the recorder stops under 0.5 GB. Script: defect_free_disk.py (defect_free_disk.txt).
   Patch: patches/apb_main_free_disk_floor.patch (check needs at least min_free_gb).
D5 tools/profiles/betaflight.json (new at 03:50, not mine): keys that settings.py does not have yet (msp_lock_gain).
   use_profile.py reads every profile, so every apply and show stops: EVENT_DAY step 0 cannot run, and
   tests_apb/test_digital has 3 FAIL. Script: defect_betaflight_profile.sh (defect_betaflight_profile.txt).
   Patch: patches/profiles_betaflight_json_out_of_tools.patch (take it out of tools/profiles until its settings patch
   in betasim/patches is applied).


4. THE FIVE LARGEST RISKS FOR THE EVENT DAY
R1 No profile can be applied today: tools/profiles/betaflight.json breaks use_profile.py (D5). Evidence:
   profile_combos_after_betaflight_json.txt (21 of 21 fail), run_all.txt (test_digital 3 FAIL).
R2 A wrong video delay at 100 m. With the delay 30 ms too long the 100 m hold drifted 9.85 m RMS (13.7 m max) against
   1.12 m at 60 m (B table, one flight each). The fix is the wiggle and timing before the high holds, but timing
   accepts the wrong chunks with wrong values (D1). EVENT_DAY step 7 item 15 now gives a chunk-picking line.
R3 The program stops when its start ends. A run started in an ssh session stopped at the hang-up (A.2). A battery change
   powers the board off and stops it too. The propeller rules then force a cable and propellers-off cycle before each
   restart (EIGHT_HOURS rules). Evidence: hup_test.txt; aircraft manual rules (audit Table 1 rows 2 and 3).
R4 Storage without an SD card. justice_d makes check need 5.3 GB (step 1 check line) on a 4 GB eMMC, and the program
   records raw pictures all the time it runs, also on the ground: after this rehearsal the laptop's logs/ held 2.5 GB
   (36 raw chunks from about 6 minutes of runs on the ground, 2 rec_ files, 6 run logs). Without no_sd and the delete
   rule, the recorder stops and a tuning flight has no recording.
R5 Tune patches that make the hold worse. At 60 m a replay_tune patch (focal 8 % off) made the error 2.5 times larger
   (A table); on a flown 100 m recording its delay was 12 ms short (B table). Evidence: step7_60m.txt,
   after_reh_jd_wig_100m_s31.txt. The merge rule is now: 100 m or higher, and only with no wiggle result.
Next: ArduPilot's check line D2 shows FAIL on every ground check, so an operator can learn to ignore FAIL lines.


5. TESTS (step 11, no simulator running)
sh tests_apb/run_all.sh: 04:02:08 to 04:09:56, load (1 min) 3.17 at the start, 4.24 at the end (run_all.txt).
12 of 15 groups pass (the last run: 14 of 15).
- test_live: 1 FAIL, the known "overlay only" case (21 fixes in 288 frames). Same as the last run.
- test_lens_d: 0 FAIL, 1 KNOWN LIMIT (60 m scene, fy +1.08 %). Same as the last run.
- test_digital: 3 FAIL. NEW. All three are use_profile.py apply failures: "tools/profiles/betaflight.json: unknown key
  'msp_lock_gain'" (D5).
- test_lens_store: 8 FAIL. NEW. The first is the same apply failure; the others follow from it (no profile applied,
  so calib runs with the analog lens). Not caused by no_sd.json: every FAIL line traces to the apply error.
- test_ap_sitl, test_event and the other groups pass.
tests/run_regression.py: 04:09:56 to 04:11:42 (106 s; last run 172 s). 19 PASS and 2 INFO (E5, E12), the same
result for every ID as before (regression.txt; before/regression.txt). E9 timing: mean 3.4 ms per frame (last
file: 8.7 ms). Its load at the start was 4.24, over the limit of 4 (the chained command did not wait), and another
program raised the load to 13 during it: the timing numbers are not comparable.


A.2 THE SSH HANG-UP TEST (audit item A.2)
This laptop has no ssh server (no sshd binary; no sudo to install one). sshd ends a session by closing the pseudo
terminal. hup_test.py does the same with bash in a pseudo terminal, with the program running against ArduCopter SITL
and the camera stand-in. One run each (hup_test.txt):
- python3 apb_main.py run --raw (in the foreground): STOPPED at the hang-up (no process 6 s later, run log stopped).
- nohup python3 apb_main.py run --raw &: CONTINUES (run log grew 180 rows in 3 s). Output goes to nohup.out.
- setsid nohup python3 -u apb_main.py run --raw > logs/console.txt 2>&1 < /dev/null &: CONTINUES (180 rows in 3 s).
The documents use tools/deploy.sh (it starts the program with setsid nohup, as the last line) and give that line for a
start typed on the board (EVENT_DAY step 7, item 2).


B. THE LENS CHECK (audit item B)
B.1 The Justice D simulator results use the true lens "sim Justice D near crop" (px4sim/sitl.py VIDEO_DEFAULTS):
    118 x 64 degrees (horizontal x vertical), square pixels. The program's start lens "Justice D full sensor (assumed)"
    is 152 x 85.5 degrees; "Justice D anamorphic (assumed)" is 114 x 85.5 degrees (computed from the profiles).
B.2 calib4.py: a 12 s hand turn rendered in real time with the true lens, fake FC, then apb_main calib --rec from the
    start lens. Seed 5, one run each. Load (1 min) at each start under 4 (calib4_console.txt).
    true lens   | start lens  | rate  | fx error | fy error | residual  | delay / readout (true 58 / +10 ms)
    full sensor | full sensor | 30 Hz | +0.37 %  | +0.26 %  | 0.060 deg | 58 / +11
    full sensor | anamorphic  | 30 Hz | +0.18 %  | +0.30 %  | 0.058 deg | 58 / +11
    anamorphic  | full sensor | 30 Hz | +0.23 %  | +0.31 %  | 0.056 deg | 58 / +11
    anamorphic  | anamorphic  | 30 Hz | +0.23 %  | +0.35 %  | 0.054 deg | 58 / +11
    full sensor | full sensor | 10 Hz | +0.23 %  | +0.24 %  | 0.068 deg | 58 / +11   (no_sd rate)
    anamorphic  | anamorphic  | 10 Hz | +0.28 %  | +0.31 %  | 0.064 deg | 58 / +11   (no_sd rate)
B.3 No fit stopped at the edge of its search range. "Calibrate from each start lens and keep the lower residual" picks
    the anamorphic start in both pairs (0.058 against 0.060, 0.054 against 0.056), and that fit is right in both
    cases, because all four fits found the true lens within 0.4 %. The residuals differ by 0.002 deg, which one run
    cannot separate. So the second calibration adds time and no proven gain. EVENT_DAY step 4 now says: use the
    anamorphic start only when the fit ends at the edge of its range. At 10 Hz (no_sd) the fits were as good.
B.4 Skipped (time). The sim's true lens is already 118 degrees wide, near the datasheet's 114; a closed-loop pair
    with the anamorphic lens (vertical 85.5 against 64 degrees) was not run.


FILES CHANGED
EVENT_DAY.txt, EIGHT_HOURS.txt, DECISION_CARD.txt (copies before the changes: before/), tools/sim_rec.py,
tools/profiles/no_sd.json (new), tests/regression.txt (rewritten by tests/run_regression.py; copy: regression.txt),
integ/rehearsal_2026-11-20/ (new; its runs/ folder holds 2.9 GB of recordings, git-ignored, safe to delete).
Simulator files: apsim/runs/reh_jd_wig_*, apsim/runs/rehearsal_2026-11-20*. The SITL container was stopped at 04:14.
At 04:13 another simulator (betasim/closed_loop.py) and an emulated ARM test ran on this laptop (not started here).
deploy.sh and board_check.sh changed at 03:34 and 03:36 by others (start, stop, status, --route); the documents use
those commands (md5 at 04:05: deploy.sh 909e444d6ae6, board_check.sh da0cf504bff9).

NOT DONE
- PX4 SITL for the check steps (only ArduCopter). The recovery demonstration. tools/lens_calib.py (no checkerboard
  pictures). The live calib and the bench pass rules (they need hands on a real drone). ssh to a real ssh server.
  deploy.sh boot-on (it refuses BOARD=local). B.4.
- One seed per case everywhere in sections 2 and B.
