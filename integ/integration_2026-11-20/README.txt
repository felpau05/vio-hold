INTEGRATION OF THE REHEARSAL CORRECTIONS (2026-11-20, 04:17 to about 05:00)

Words: FC = flight controller. SITL = software in the loop (a simulator). RMS = root mean square.
Source: integ/rehearsal_2026-11-20/README.txt, sections 3 and 4. Python: ~/.venvs/vio/bin/python.
Nothing is committed. The branch is stage1-justice-d. The Betaflight option stays off.

FILES IN THIS FOLDER
apb_main.diff                       apb_main.py against .snapshots/pre_agents_20261120_0320.tgz (for the reviewer)
flight_loop_unchanged.txt           old and new apb_main.py, each top-level definition compared by syntax tree
docs.diff                           the lines changed in EVENT_DAY.txt and DECISION_CARD.txt
test_ground_tools_on_old_code.txt   the new test against the old apb_main.py
defect_*.txt                        the rehearsal's defect scripts, run again after the corrections
timing_with_run_log.txt             the corrected timing on the 4 rehearsal flights
betaflight_option.txt               step 6: enable.sh and disable.sh on a copy
msp_m1.txt                          M1: tests, the Betaflight SITL matrix with and without M1, the decision
betasim_before/, betasim_noM1_rerun/, betasim_M1/   the matrix files of each version
run_all.txt, run_regression.txt, suite_times.txt, suite_load_every_15s.txt   the full suite at the end

THE FLIGHT LOOP DID NOT CHANGE
flight_loop_unchanged.txt: 56 top-level definitions are identical by syntax tree, among them run_live, Pipe, Engage,
Raw, Log, catch_up, GyroAtt, live_gate, start_problems, mode_run, replay. Changed: the module docstring, mode_check,
mode_timing, main. New: "import re". In main, "run" now goes to "return mode_run(s, args)" before the try of D3;
before, it went through modes["run"](s, args). The call is the same.

NEW TEST: tests_apb/test_ground_tools.py (no simulator, about 5 s). It is a new line in tests_apb/run_all.sh after
the loop, so betasim/patches/run_all.patch still applies. Against the old apb_main.py: 13 FAIL, every check of D1 to
D4 (test_ground_tools_on_old_code.txt). The PX4 line and the 0.6 GB line pass on both: they guard against a
regression. Against the new file: ALL PASSED.


D1 timing: APPLIED, CHANGED
Patch: "timing RUN.csv RAW.npz..." keeps the frames from 16 s to 1 s before the first "engaged" note.
a. It corrects the cause (timing had no way to know the wiggle frames) only when the operator gives the run log.
   Without the log the old path stayed: any chunks, also hold chunks. Change: the run log is required. Without it,
   or with two logs, timing shows FAIL and writes nothing.
b. Flight loop: none. mode_timing reads the run log that Log writes; Log is not changed.
c. Input the author did not test:
   - no "engaged" note: the patch shows FAIL. Kept, with a clearer reason.
   - two "engaged" notes: the window of the first. Nothing before the first switch is a hold. Kept; an INFO line now
     says "the first of N switches". A wiggle before a later switch is not used.
   - the "engaged" note on a row with no frame (t empty, a 0.5 s camera timeout): the patch skipped that note
     without a word and took the next switch, whose window can hold the first hold. Change: such a row takes the
     t of the last frame.
   - a wiggle shorter than 15 s: the window also holds calm frames, so the RMS turn rate falls. Under 20 deg/s
     timing shows FAIL and writes nothing (the existing check). Not changed.
   - raw files of another run of the same boot: no frame in the window. The patch gave calib.Rec 0 frames:
     "FAIL the recording turned 0 deg/s RMS with 0 matches ... Rock the sticks harder" (a wrong reason). Change: an
     explicit FAIL "no raw frame in the 15 s before the switch".
   - raw files of a run before a reboot: the frame clock (time.monotonic) starts again at each boot, so their times
     can fall in the window. The patch would use another flight's frames. Change: FAIL when the start stamps in the
     file names (run_<stamp>.csv, raw_<stamp>_NNN.npz) differ by more than 2 s. Files without a stamp in the name
     skip this check.
   - memory: the patch loaded every chunk's pictures before the cut. With "raw_<stamp>_*.npz" that is the whole
     run (the recorder runs also on the ground). Change: only the pictures in the window are loaded (the 4 flights:
     182 to 185 frames; 60 m: maximum resident size 157396 kB, GNU time, one run).
Test: section A of test_ground_tools.py (7 checks; calib.Rec is replaced by a spy that keeps the frames it gets).
Flights (timing_with_run_log.txt, one flight each, seed 31, the run log and all raw files, true 40 ms / +14 ms):
10 m 40 ms / +15; 30 m FAIL, settings not changed; 60 m 38 ms / +23; 100 m 36 ms / +11. The 60 m and 100 m values
are the same as the patch author's test (patches/apb_main_timing_window_test.txt).
Defect script again (defect_timing_accepts_hold.txt): each chunk alone now gives "FAIL give one run log ..."; no
chunk passes and nothing is written.

D2 check, ArduPilot height stream: APPLIED AS THE PATCH (comment shorter)
a. The cause: check measured LOCAL_POSITION_NED, which ArduPilot sends only with a position. fc_link takes the
   ArduPilot height from GLOBAL_POSITION_INT and asks for it at hgt_hz (setup_streams). check now measures that.
b. Flight loop: none (mode_check only).
c. ArduPilot with GPS: the height still comes from GLOBAL_POSITION_INT, so the line is still right. PX4: unchanged.
   Betaflight option: firmware "betaflight" keeps LOCAL_POSITION_NED, which msp_link maps to its ALTITUDE replies.
   Not covered: in flight, ArduPilot's LOCAL_POSITION_NED also gives fc.local_pos (re-anchor). check cannot test
   it on the ground, before or after.
Test: section B (a stand-in FC object with the stream rates). Defect script again on ArduCopter SITL (GPS off,
parameters wiped, one run): "PASS GLOBAL_POSITION_INT 10 Hz (asked 10)" (defect_check_lpos_ardupilot.txt; load
1.63 at the start). The SITL container was stopped after the run, and apsim/runs/rehearsal_2026-11-20_sitl.log
was put back (the run's log is defect_check_lpos_ardupilot_sitl.log here).

D3 bench, mask, calib, one FAIL line: APPLIED, CHANGED
a. The cause: main let the program's own error messages out as tracebacks. The patch caught RuntimeError for every
   mode. Change: ValueError too (the mask size error in calib and mask, a lens profile error), and check, mask,
   calib and bench only. Other errors still show a traceback, so a program bug stays visible.
b. Flight loop: the patch put "run" inside the try. mode_run already catches RuntimeError itself, so it would not
   change in practice, but it is a change in the flight start. Change: "run" is called before the try, as before.
c. Ctrl+C while mask or calib records still shows a traceback (KeyboardInterrupt; not in this defect). A ValueError
   from a program bug now shows as one FAIL line without its traceback.
Test: section C (no FC: bench, mask, calib; a mask of another size in calib). Defect script again
(defect_bench_traceback.txt): one line "FAIL no MAVLink heartbeat on udpin:127.0.0.1:14599 ...", exit code 1 (the
script's own "exit code 0" is the code of tail in its pipe).

D4 free disk: APPLIED AS THE PATCH (comment shorter)
a. The cause, within the old design: check needs min_free_gb scaled by the picture size and rate; the recorder stops
   under the unscaled min_free_gb. Now check never needs less than the recorder's stop level, so a check PASS means
   that the recorder starts. It does not promise recording time: at exactly min_free_gb free the recorder writes
   about one chunk. That was also true before for the analog default (scale 1). Not changed; EVENT_DAY step 0 now
   says what a PASS means.
b. Flight loop: none. Raw is not changed: in flight the recorder still stops under min_free_gb. What changes is only
   check's number where the scaled value is under min_free_gb: no_sd 0.44 to 0.50 GB; justice_d 5.33 GB and the
   analog default 2.0 GB stay.
c. min_free_gb 0 gives need 0, as before.
Test: section D (free disk 0.3 and 0.6 GB with the no_sd values: when check passes, Raw records). Defect script again
(defect_free_disk.txt): no_sd "check needs 0.50 GB ... the recorder stops under 0.5 GB", no mismatch flag.

M1 msp_link count carry: NOT KEPT. See msp_m1.txt.

DOCUMENTS (docs.diff)
EVENT_DAY.txt: step 0 disk line (0.44 to 0.5 GB, and what a PASS means); step 1 pass rule and step 5 (the ArduPilot
exception for LOCAL_POSITION_NED is gone; the height line is GLOBAL_POSITION_INT); step 7 item 15 (one timing line
with the run log and all raw files of the run, the new numbers, and the FAIL reasons).
DECISION_CARD.txt: row 23 (0.44 to 0.5 GB); row 42 (the new FAIL lines and what to do).
EIGHT_HOURS.txt: no line described the old timing use or named these defects as open. Not changed.

STEP 6, THE BETAFLIGHT OPTION (betaflight_option.txt)
On a copy (196 files): enable.sh applied all 5 patches, test_ground_tools and test_msp_link passed in the enabled
copy, and disable.sh restored every file (same md5). No patch file needed a correction. Done before and after M1.

FULL SUITE (step 10)
The 1-minute load was 1.27 at the start (under 3, no wait). An emulated ARM test of another job ran at low priority
the whole time. Load samples every 15 s: suite_load_every_15s.txt.
sh tests_apb/run_all.sh: 04:42:52 to 04:50:47, load 1.17 to 4.30 (run_all.txt). 16 groups: the 15 of before and
test_ground_tools.
- 14 of 16 groups with no FAIL. The run before today: 14 of 15.
- test_live: 1 FAIL, "overlay only: 21 fixes sent in 288 frames". The same known FAIL as before.
- test_lens_d: 0 FAIL, 1 KNOWN LIMIT (60 m scene, fy +1.08 %). The same as before.
- test_ground_tools: ALL PASSED. New group.
- test_digital and test_lens_store: ALL PASS. In the rehearsal run they failed (D5, tools/profiles/betaflight.json);
  that file is no longer in tools/profiles. Not changed by this work.
- test_run: 1 FAIL. NEW in this run. The check "the first note of the run log names the FC position" failed: the
  first frame came before the first height row ("height stale"), so the first note was the mode note alone
  ("FC mode main mode 0") and the re-anchor note came on the second row (test_run_fail_first_rows.csv). This is a
  race of the first frame against the first height row in the test, not a change of the program: test_run calls
  apb_main.open_fc and apb_main.run_live directly, and those and every function they call are identical by syntax
  tree (flight_loop_unchanged.txt). test_run alone again, 3 times on the new apb_main.py and 3 times on the old
  one, alternating: 6 of 6 ALL PASS, each with the first note "FC mode main mode 0; re-anchored to FC position ..."
  (test_run_repeat.txt; load 3.1 to 11.0, falling after the regression).
- All other groups and sim_test.timing_test: pass, as before.
tests/run_regression.py: 04:50:47 to 04:52:29, exit 0 (run_regression.txt; it rewrites tests/regression.txt).
19 PASS and 2 INFO (E5, E12), the same result for every ID as the rehearsal run. The load rose to 22.5 during it
(from 2.64 at its start), so its timing numbers (E9 mean 4.4 ms per frame; rehearsal run 3.4 ms) are not comparable.

NOT DONE
- A second full run_all to show that the test_run race does not repeat (only test_run alone was repeated).
- M1 with more seeds (msp_m1.txt).
- The D1 case "two wiggles, two switches": timing uses the first switch only; no option picks another.


VERSION 2: THE REVIEW (review/REVIEW.txt), 05:35 to about 06:00
Files: apb_main_v2.diff, flight_loop_unchanged_v2.txt, docs_v2.diff (both rounds against the documents before this
work), timing_cases_v2.txt, mutations_v2.txt, run_all_v2.txt, suite_v2_times.txt, suite_v2_load_every_15s.txt.
The flight loop is still unchanged: the same 56 top-level definitions are identical by syntax tree. Changed: the
module docstring, mode_check, mode_timing, main. New: import json, import re, HOLD_MODES, WIGGLE_RATIO, gap_s,
wiggle_rates. "run" still goes to mode_run(s, args) before the try.

B1 (which switch): APPLIED, CHANGED. timing uses the first "engaged" note whose 15 s before it (log t) have log rows
with no gap over 1 s, no mode in HOLD_MODES, and no height under min_height_m. Changes to the patch: the hold names
are a module constant (AP_HOLD, POSCTL, AUTO_LOITER and the Betaflight option's POS_HOLD); the FAIL lists each
switch and its reason; the gap reason names its causes (the program started later, or the camera stopped).
B2 (the window holds the wiggle): APPLIED, CHANGED. wiggle_rates: the RMS turn rate of the 8 s with the most turning
against the rest, from the FC attitude on a 0.1 s grid. Limit WIGGLE_RATIO 4, not 3: between the wrong-value windows
(1.0 to 1.8) and the flown windows that pass (60 m 5.3, 10 m 7.6, 100 m 8.7; 30 m 4.4 fails later on its fit), set
toward refusal. The comment says that it was set on the same 4 flights. "<=" so that a window with no turning at all
fails. The FAIL text gives the reasons (more than 8 s from the wiggle to the switch, or no wiggle).
S2 (frames of another run): APPLIED. Each raw frame must have a log row at t_grab - video_delay_s within 1 ms (the
setting, not the measured 70 ms). This also fails when video_delay_s changed since the flight (a second timing run on
the same flight): the FAIL says so. The name stamps may now differ by 1 s, not 2 s.
New with S2: the raw frames must cover the 15 s with no gap over 1 s (a missing or dropped chunk: FAIL; review N2).
The window still compares t_grab with the log t (review N1, about 70 ms early): moving it changed the flown values by
one frame (60 m 36 ms, 100 m 38 ms), so it is kept, with a shortcut comment.
S4: APPLIED. A json.JSONDecodeError in check, mask, calib, bench or timing gives "FAIL the lens file <path> is damaged:
...". check's own lens line names the lens file too. timing is now inside the same one-FAIL-line catch.
S3: APPLIED. The free disk line adds "<n> min of raw recording before the recorder stops" after the bracket ((free - min_free_gb) / rate). Under 10
min (and a PASS): a WARN line, not counted as a FAIL. The limit itself is not changed. The line now shows 2 decimals.
S5: the test was rewritten (tests_apb/test_ground_tools.py, 32 checks, about 6 s). The fixture log writes t = t_grab -
video_delay_s, and the expected frames are written by hand. All 9 mutations of review/evidence/mutations.txt are now
caught (the 7 open ones: m1, m2, m3, m4, m6, m7, m8), and 8 more on the new rules (x1 to x8): mutations_v2.txt.
Still open: the value of WIGGLE_RATIO (any value from 2.6 to 5.7 passes the test: fixture ratios 2.51 and 5.72); calib's own search (the spy).
S6, EVENT_DAY: step 6B item 5 (leave the switch in Altitude mode, check it); step 7 start (the switch position, no
switch test from item 1 to item 9); item 8a (climb to 40 m or higher, then still 5 s); item 8b (8 s wiggle, settle
4 s, the switch no more than 8 s after the last stick move, else wiggle again); item 15 (the rules, once per flight,
the FAIL reasons, "keep the ground calibration values, or fly the wiggle again", the run log before a restart).
Step 0 and step 2: the minutes and the WARN line. DECISION_CARD rows 23 and 42 the same.

CASES (timing_cases_v2.txt): 126 cases, 13 PASS, 0 PASS with a wrong value (more than 5 ms or 10 ms from the truth).
The flown 4: 10 m 40 ms / +15, 30 m FAIL, 60 m 38 / +23, 100 m 36 / +11 (unchanged). d_* (switch tests): 9 of 12 PASS
with the flight's own values. a_*, a2_*, b_*, c_*, e_*: 0 PASS. f_drop: 1 of 10 PASS (002 at 10 m, 40 / +17).

FULL SUITE, VERSION 2 (run_all_v2.txt, suite_v2_times.txt, suite_v2_load_every_15s.txt)
Waited 40 s for the 1-minute load to fall under 3 (4.47 at 05:59:03). sh tests_apb/run_all.sh: 05:59:43 to 06:07:28,
load 0.56 to 2.82. 15 of 16 groups with no FAIL (the first run: 14 of 16).
- test_run: ALL PASS (the first run: 1 FAIL, the race of the first frame against the first height row). Not run alone
  again: it did not fail.
- test_live: 1 FAIL "overlay only: 21 fixes sent in 288 frames", and test_lens_d: 1 KNOWN LIMIT (fy +1.08 %). The
  same as the first run and as before today.
- test_ground_tools: ALL PASSED (rewritten: 32 checks).
- Every other group and sim_test.timing_test: pass, as in the first run.
An earlier start of this run (05:48) was stopped: test_digital had 1 FAIL, because the new free disk text broke its
check for ", 30 Hz)". The minutes now come after the bracket, the line keeps ", 30 Hz)", and the run was started again
on the final apb_main.py (md5 a65d2d3d8ab4). tests/run_regression.py was not run again: hold.py did not change.

BOARD SETUP (asked after the review; commands read in tools/deploy.sh, tools/board_check.sh, tools/bundle/README.txt)
EVENT_DAY.txt: new STEP 0A (board address in tools/deploy.env, the clock line of each push and start, board_check and
"use case A" or "use case B", case A wheel install, case B "tools/deploy.sh bundle" and its PY= and BOARD_DIR= lines,
--route, the temperature limits, boot-on and boot-off). DECISION_CARD.txt: row 43 (the program stopped when the cable
came out). docs_v2.diff holds these lines too.
Bundle built again last, after the suite: sh tools/bundle/build.sh, 06:07:46 to 06:08:05, no download (the CPython file
was there, sha256 OK). tools/bundle/vio-bundle-aarch64.tar.gz: 113,834,473 bytes, sha256
5a9a80ac1b99909aaadc3b2b20f664736056fb379a0423909b815a54ddf23a41 (was fed4126a63e0...). Its apb_main.py, EVENT_DAY.txt
and DECISION_CARD.txt have the same md5 as the project files. tools/bundle/README.txt still gives the old sha256 and
size (not changed by this work).
