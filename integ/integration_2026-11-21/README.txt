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
