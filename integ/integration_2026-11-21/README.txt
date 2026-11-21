NIGHT OF 2026-11-21 (00:38 to about 02:15): GNSS LEVELS, PILOT SAFETY, AUDITS, TALK

Words: FC = flight controller. SITL = the simulator that runs the real FC firmware.
Nothing is committed. The branch is stage1-justice-d. Snapshot before the work: .snapshots/pre_agents_20261121_0038.tgz.
Nothing ran on the real board, camera or aircraft.

WHO DID WHAT (one lead session, five agents)
- GNSS levels: code, tests, 16 SITL runs.            integ/gnss_ladder/README.txt
- Pilot safety case: source reading, 14 SITL runs.   integ/pilot_safety/README.txt, PILOT_SAFETY.txt
- Sources for the opening of the talk.               talk/business_sources.md
- Lead: the talk, the checklists, the tools below, this integration.

CODE THAT CHANGED
- apb_main.py, fc_link.py, settings.py, apsim/sitl_ap.py, tests_apb/run_all.sh: the setting gnss_mode
  (none, setup, fallback, velocity). The default "none" sends the same messages as before (test_gnss G2).
  The diff: integ/gnss_ladder/flight_loop.diff.
- settings.py: one comment (the EKF origin comes from a map before the day).
- tools/deploy.sh: the file list has tools/fc_params.py. tests_apb/test_lens_store.py: its copy of the list too.
NEW FILES
- tools/fc_params.py              save the aircraft's own FC parameters, write them back. Selftest passes. On an
                                  ArduCopter SITL FC: saved 8 values, restored them after check --set-params
                                  (fc_params_sitl.sh, fc_params_sitl.txt, one run)
- tools/profiles/gnss_setup.json, gnss_fallback.json, gnss_velocity.json, att_gap.json
- tests_apb/test_gnss.py, apsim/gnss_matrix.py, apsim/safety_matrix.py and their result files
- PILOT_SAFETY.txt, ORGANIZER_QUESTIONS.md, HARDWARE_BRINGUP.md
DOCUMENTS THAT CHANGED
- EVENT_DAY.txt: step 0A item 4 (do not mix packages into a Python that has OpenCV), step 1 items 2a, 2b, 4a
  (ask the crew, save the parameters, arming), step 2 (ATTITUDE 90 Hz), step 6B (the note on mode 11),
  new step 6C (takeover demonstration for the pilot), step 7 items 5a and 10a, new step 8 (give the aircraft back).
- DECISION_CARD.txt: rows 44 (attitude gaps), 45 (GNSS level), 46 (no arming: VisOdom).
- EIGHT_HOURS.txt: new block BEFORE HOUR 1, and hour 8 item 0.
- EVIDENCE.md: areas D and E, and the bonus route for GNSS before the hold. PRESENTATION.md: the 5-minute talk on
  top, the GNSS levels, the question bank. README.md: the folder table. tools/bundle/README.txt: the new build.

FULL SUITE (run_all.txt, suite_times.txt)
sh tests_apb/run_all.sh: 01:26:43 to 01:35:05, on the final code. 17 groups.
- 15 groups with no FAIL in that run, among them test_gnss (new) and test_ap_sitl.
- test_live: 1 FAIL, "overlay only: 21 fixes sent in 288 frames". The known FAIL.
- test_lens_d: 0 FAIL, 1 KNOWN LIMIT. As before.
- test_lens_store: FAILED in that run: "tools/fc_params.py is missing here. Nothing copied." Cause: the new file in
  the deploy list was not in the test's own copy of the list. The list in the test is corrected. test_lens_store
  alone again: ALL PASS in 2 s. The full suite was not run a second time.
So the state is 16 of 17 groups with no FAIL (test_live).
tests/run_regression.py was not run: hold.py did not change.

BUNDLE
sh tools/bundle/build.sh, the last time after the last change to EVENT_DAY.txt and tools/fc_params.py. tools/bundle/vio-bundle-aarch64.tar.gz: 113,839,117 bytes, 5571 files, sha256
94dab2254a3e3ead9ea9d9be4f75e011a2b435d75005b8827ebb751409a22014. Its program files have the same md5 as the project
files. The emulated ARM test of 2026-11-20 was not run again on this build.

THE THREE FINDINGS THAT CHANGE THE DAY
1. With the default arming checks the FC does not arm with our settings ("VisOdom: not healthy"), 4 of 4 tries in
   SITL. The crew must turn off the vision arming check only (EVENT_DAY step 1 item 4a, DECISION_CARD row 46).
2. The FC does not detect a slow, wrong position. In SITL the aircraft followed a 1 m/s error with no warning.
   Only the pilot can stop it (PILOT_SAFETY.txt).
3. GNSS before the hold only (gnss_setup) works in SITL, and a 20 m GNSS error in the hold did not move the aircraft.

NOT DONE
- A second full suite run after the test_lens_store correction.
- The emulated ARM test on the new bundle.
- att_gap in closed loop. fc_params.py on a real FC.
- PX4 for the GNSS levels and for the takeover cases. Acro, RTL and the radio failsafe in SITL.
- A merge with origin/spec-justice-d: only text was taken from it (roles, questions, record sheet, Q&A questions).


SECOND PART OF THE NIGHT (03:15 to 08:15): REVIEW, LOW HEIGHT, FORWARD CAMERA, FIVE SESSIONS
Five sessions worked on this folder. Session 1f (this record) led and owned the documents, the flight program files
and the bundle. Session 0a owned hold.py and the simulators. Sessions 16, 2f and 32 checked status, rules and sources.

COMMIT 8939114 (03:25): the tested state of the first part, with the hold.py of the 02:08 bundle. Not pushed.

DOCUMENT REVIEW (an independent reader, 21 findings, all corrected)
The worst ones: the parameter save of EVENT_DAY step 1 item 2b failed on a second pass (tools/fc_params.py now
shows PASS and keeps the first file); EIGHT_HOURS put EVENT_DAY step 8 (give the aircraft back) before the scored
trial; no hour did step 6C; a board with no usable Python (case B) had no instructions for the board commands; the
GNSS-level parameters were not in the save (fc_params.py now saves them always).

LOW HEIGHT (sessions 0a and 1f; integ/diag10, integ/kfmap)
- Cause found in the simulator: at 5 to 10 m most fixes come from Hold's whole-picture path, which took the yaw
  from the attitude feed. 1 degree of yaw error moved the fix by 0.26 m at 10 m.
- hold.py (0a): options dense_local, dense_turn, dense_nadir_deg, kf_map. All off by default. With them off Hold
  gives the same fixes as before (integ/kfmap/README.txt section 3).
- apb_main.py, settings.py (1f): att_source "gyro_fcyaw" (roll and pitch from GyroAtt, the yaw of the FC as sent),
  and the interlock apb_main.turn_problems: dense_turn does not start without gyro_fcyaw. tests_apb/test_att_yaw.py.
- Profile tools/profiles/low_height.json = dense_local + dense_turn + gyro_fcyaw. Only for holds at 5 to 10 m.
  ArduCopter SITL A/B: integ/kfmap/closed/ab_table.txt. DECISION_CARD row 47 has the numbers and the limits.
- The pilot's hover before the switch: 15 s (EVENT_DAY step 7 item 8).
- A first diagnosis ("the yaw explains the full 1 m") was wrong and was withdrawn by its author within 10 minutes:
  its replay started the attitude feed at another moment than the program does.

FORWARD CAMERA (prototype, not evaluated, not in the program): fwd_cam.py, tools/fwd_rec.py, tools/fwd_check.py,
tests_apb/test_fwd_cam.py (not in run_all.sh, never run), integ/fwdcam/README.txt. Not for use in a flight today.

SOURCE CHECK OF THE TALK (session 32): the four quotes of slide 1 and the bridge quote are the same as the text of
their pages (talk/quotes_check_2026-11-21.md). The price of the comparable drone could not be confirmed.

OTHER SMALL CHANGES: tools/profiles/att_gap.json and yaw_fc.json, DECISION_CARD rows 44 to 47, EVENT_DAY step 0
(low_height), the question bank of PRESENTATION.md (items 37 and 38), CONTRIBUTION.md (one row).
