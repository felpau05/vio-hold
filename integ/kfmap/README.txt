DENSE_LOCAL, DENSE_TURN, DENSE_NADIR_DEG, KF_MAP: Hold options for low heights (2026-11-21, event day; simulation only)
==========================================================================================================================

STATUS (kept up to date during the work; times EST)
  03:10  DENSE_LOCAL built and proven (section 2). KF_MAP built (section 1b). apb_main.py: not changed (see 1c).
  03:31  test suites with the options off: done (section 3).
  03:35  DENSE_LOCAL picture tests (bad_pictures, test_live): done (section 3c).
  03:36  Lead's decision: KF_MAP STOPPED (off by default, untested in closed loop; section 5). New options DENSE_TURN (the
         whole-picture path solves the yaw turn) and DENSE_NADIR_DEG (nadir limit), ported from integ/diag10.
  03:40  DENSE_TURN and DENSE_NADIR_DEG built and proven (section 2b). Profile low_height.json: py (section 8).
  03:42  ArduCopter SITL A/B started (section 7): base, py, base with hover 15 s, (b) att_source gyro_fcyaw, py + (b).
  04:04  10 m done (section 7). SAFETY: py alone bends the FC yaw up to 11.7 deg; py + (b) best. The lead added an
         interlock (apb_main.turn_problems): dense_turn needs att_source gyro_fcyaw. py alone is not run again.
  04:07  py picture tests done (section 3c). Profile low_height.json = py + (b) (section 8); it loads (use_profile.py
         apply on a temporary settings copy).
  04:08  SITL round 3 started: 5 m (base, hover 15, (b), py + (b)), VALIDATION ottawa2022 10 m s11/s12 (base, py + (b)),
         py + (b) + hover 15 s at 10 m, 30 m and 60 m (base, py + (b)). Then the suite on the final hold.py.
  04:24  5 m done (section 7): py + (b) best (ferr 0.04/0.04 m, 0 fixes over 0.5 m wrong; base 0.43/0.39 m).
  04:32  VALIDATION ottawa2022 10 m s11/s12 done: py + (b) rms 0.34/0.27 against base 0.74/0.65 m; 0 bad fixes.
  04:34  queued after round 3: the final suite (copy 'final' = repository files of 04:33, hold.py d78329d3), then
         round 4 (more seeds: VALIDATION ottawa2022 s13, farmland s11/s12, 5 m ottawa2022 s11/s12; TUNING s3).
  04:49  60 m seed 1: py + (b) worse than base. 30 m seed 1: no harm.
  04:52  FINAL regression gate on the final hold.py: every ID the same result (section 3d). run_all 17 of 18 no FAIL.
  05:07  test_live with py + (b): overlay 22 fixes (21 off), freeze noise 11/72, 0.84 s (limit 1.0 s) (section 3d).
  05:07-05:46 round 4: 60 m s2 (worse again), VALIDATION 10 m ottawa2022 s13, farmland s11/s12, 5 m ottawa2022
         s11/s12, TUNING s3 at 5 and 10 m: py + (b) lower RMS in 11 of 12 pairs at 5-10 m, 0 bad fixes.
  (The session was paused by a usage limit from about 04:50 to 05:40; the queued runs went on.)
  05:51  FC yaw error table over the flight (section 7). 30 m seed 2: no gain, no clear harm.
  05:52-06:12 round 5: 5 m farmland s11/s12, 20 m s1 (no gain), 60 m s3 (py + (b) better; FC yaw still bent -3.4 deg).
  06:13-06:28 round 6: 5 m py + (b) + hover 15 s s1/s2, VALIDATION 10 m farmland s13, 5 m ottawa2022 s13.
  06:29  FINAL: 17 pairs at 5-10 m, py + (b) lower RMS in 16 (section 7 FINAL SUMMARY). Every SITL run of this study
         finished its 60 s hold (none failed, none skipped; 60 runs in ab_table.txt; 2 of them ran while their driver was
         stopped on purpose, at 04:02 and 04:07, so driver.log has no end line for them). No run after 06:28.
  06:30  STOPPED: no driver, runner or arducopter process; containers ap_sim_kfab and ap_sim_kf_suite removed;
         integ/kfmap/SITL_ACTIVE deleted. README FINAL. Final hold.py md5 d78329d32c349ef061282fea1192d320 (unchanged
         since 03:40; all four options off by default).

Every number here comes from simulation (px4sim/camera.py pictures, recorded or live ArduCopter SITL motion). Nothing
ran on the real board, camera or aircraft. Nothing is committed.

1. What was built
-----------------
hold.py (copy before the change: integ/kfmap/hold_before.py, md5 051dd786...; whole tree before:
.snapshots/pre_kfmap_20261121_0310.tgz). Each option is a class setting; a profile sets it through
hold_cfg (dense_local, dense_turn, dense_nadir_deg, kf_map). All four are off by default. Line numbers are those of hold.py at the time of writing (see the md5 in section 3).

a) DENSE_LOCAL (hold.py lines 355-359 setting; in _dense the 'if self.DENSE_LOCAL' blocks at about 832-841; in
   _track1 at about 927-936):
   True = integ/lowh/proposed.patch: J (px per m of drone move) at the last fix with a central step of 1 % of the
   height, and the halves gap in px of each half's own texture. False = the code before, statement for statement.

a2) DENSE_TURN (hold.py lines 360-364 setting; _dense: every 'if self.DENSE_TURN' block, lines about 765-860; _track1:
   about 938-943): the whole-picture path also solves the turn about the vertical in its LK step (3 unknowns: move
   north, east, turn), with Jy = px per rad of turn per point (central step 1e-3 rad about the vertical through the
   drone). The points move by J dl + Jy dyaw, so the fit's yaw (Est.yaw) takes the turn and a yaw error of the
   attitude feed is no longer read as a move. Port of the 'yaw' variant of integ/diag10/make_variants.py, with the
   corrected Jc = P[0, 2:6]. The halves test still compares the move only. False: the statements of the code before.
   Name: Hold already has DENSE_YAW (20 deg, the yaw change after which a whole-picture keyframe is replaced), so the new
   option is DENSE_TURN (hold_cfg dense_turn).
a3) DENSE_NADIR_DEG (hold.py lines 365-369 setting; _make_kf and _nadir about 676-694; _dense 'dz' and 'out of view'
   about 735 and 792): > 0 = the whole-picture path uses only keyframe pixels within this angle of nadir (at the
   keyframe's attitude); 'keyframe out of view' compares with the keyframe's usable share. 0 = off (the code before).
   Port of the 'nadir' variant of integ/diag10/make_variants.py. 55 with DENSE_LOCAL and DENSE_TURN = diag10 'pyn55'.
b) KF_MAP (hold.py lines 370-390 settings; 413-416 state; 427-428 reanchor; 451-549 _store_map, _reloc_log, _reloc;
   1115-1124 ramp; 1140-1141 variance; 1162-1174 pilot's switch; 1190-1191 the try). STOPPED at 03:36 (section 5):
   - reanchor() first keeps the keyframes of the chain that ends (home, older ones, last, current) as a stored map
     with their anchor positions (their G and pos), at most MAP_KEEP = 6 (the home and the 5 newest). Only a chain in
     the map's frame adds to the map: the first chain, or a chain that relocalized. A chain that started at a
     re-anchor and never relocalized is in another frame; its keyframes are dropped as before. Exception: when no
     stored keyframe can be tried at the height of the ending chain's newest keyframe (a map made in the climb), the
     ending chain replaces the map.
   - Pipe still re-anchors at the FC position at once (apb_main.py unchanged): the fixes continue with no extra gap.
   - While Hold sends 'ok' fixes on the new chain (so the picture passed the picture checks and proved it is live),
     every RELOC_EVERY = 0.5 s it tries ONE stored keyframe: the stored home, or the newest stored keyframe when the
     home was taken more than RELOC_HGT = 15 % away in height. The try is the re-find of the 'lost' path: _track with
     deep=True, then _orb_guess and _track again. It runs on a copy of the keyframe's point state, so a failed try does
     not change the stored keyframe.
   - A try passes when: the re-find gives a fit with n >= 2 MIN_PTS, inlier share >= 0.6 and spread < 0.02 h + 0.2 m
     (the 'ok' rule of Hold), fit yaw <= RELOC_YAW = 3 deg, |fit scale - 1| <= RELOC_SCALE = 0.05, and the step between
     the map position and this frame's fix < max(RELOC_STEP_M = 1.0 m, RELOC_STEP_H = 0.2 x h).
   - Acceptance: RELOC_N = 3 passing tries in a row, whose steps agree to RELOC_AGREE_M + RELOC_AGREE_H x h
     (0.15 m + 0.02 h). A refused try restarts the count.
   - On acceptance Hold switches to the stored keyframe (with the try's point state and a new reference frame), the
     map's home becomes the home again, the older map keyframes come back for revisits, and the new chain's keyframes
     are dropped. The sent position does not jump: the step is kept as an offset (ramp) that shrinks at RELOC_RATE =
     0.3 m/s (time step capped at LOST_S, so a gap without fixes gives no step). During the ramp |ramp|^2 is added to
     the per-axis variance. The jump test and the jitter estimate see the fix with the ramp of the previous frame.
   - The pilot's switch (request_keyframe, fresh_kf_at_engage) sets the hold point: the map is cleared; during a ramp
     the new home takes the sent position and the old keyframes are dropped.
   - Log: Hold.reloc_log (one row per try: time, stored keyframe time and height, step m, n, yaw, scale, map
     position, pass / refused and why, accepted) and Hold.note (one text per try, for the run log).
   - KF_MAP False: no map, no try, no ramp; the statements run are those of the code before.
   Limits marked "assumed" in hold.py: MAP_KEEP, RELOC_EVERY, RELOC_N, RELOC_AGREE_M/H, RELOC_STEP_M/H, RELOC_RATE,
   RELOC_YAW, RELOC_SCALE, RELOC_HGT. None was swept.

c) apb_main.py: NOT changed (another session owns it today). The run log shows the tries only with this one line in
   Pipe.step, after "e = self.hold.process(fr.img, fr.t0, att, hgt)":
       self.note = _join(self.note, getattr(self.hold, "note", None))   # Hold's relocalization tries (hold_cfg kf_map)
   With kf_map off Hold.note is always None and the line changes nothing. The SITL copies of section 7 have this line
   (planned; not used: KF_MAP was stopped before its SITL runs). Without it, KF_MAP works the same, but the run log
   does not show the tries.

d) tools/profiles/low_height.json (final): att_source gyro_fcyaw; hold_cfg dense_local true, dense_turn true,
   dense_nadir_deg 0, kf_map false; a note per value with its evidence file (section 8).

e) Scripts (integ/kfmap/): ol_kfmap.py (open-loop replays: integ/lowh/ol_low.py with a hold_cfg per version, forced
   losses, another place, bad pictures, repeated texture, the relocalization log); compare_rows.py (frame-by-frame
   identity); show_relocs.py, timeline.py (logs); jobs.py (the job sets); closed/scripts/kf_driver.py (SITL driver,
   from integ/lowh/closed/scripts/ab_driver.py).

2. DENSE_LOCAL: proof that the switch is the patch (Task 1)
-----------------------------------------------------------
One replay, the same 660 frames for 4 versions: ol_kfmap.py --height 5 --seed 11 --ground ottawa2022 (VALIDATION),
re-anchor case B (at Hold's last fix), versions
  before   = integ/kfmap/hold_before.py (the repository's hold.py before this work, md5 051dd786...)
  patched  = a scratch copy of it with integ/lowh/proposed.patch applied (patch -p1); kept as
             integ/kfmap/hold_patched_scratch.py, md5 eb0c0293... (the md5 of the patched file of integ/lowh/closed)
  off      = hold.py with dense_local false
  dl       = hold.py with dense_local true
compare_rows.py, every Est field (pos, var, state, why, n, sent, error), floats bit for bit
(results/proof_dense_local.txt):
  before vs off:      660 frames, 0 differ -> IDENTICAL
  patched vs dl:      660 frames, 0 differ -> IDENTICAL
  before vs patched:  659 of 660 differ (the check sees a difference)
The summaries also equal integ/lowh/results (before: 7.16 fixes/s, longest gap 2.92 s, err RMS 0.235 m, 13
re-anchors; patched: 11.2 fixes/s, 1.5 s, 0.107 m, 2 re-anchors).
Files: results/ol/kf_ottawa2022_crop_5m_s11_proof_relast_{before,patched,off,dl}.json.gz.

2b. DENSE_TURN and DENSE_NADIR_DEG: proofs
-----------------------------------------
Two replays (ol_kfmap.py, re-anchor case B, VALIDATION): ottawa2022 10 m seed 11 and farmland 5 m seed 12, 660 frames
each, the same frames for every version; compare_rows.py, bit for bit (results/proof_three_options.txt; also
proof_dense_turn.txt):
  integ/kfmap/hold_before.py           vs hold.py all options off               IDENTICAL (both replays)
  hold_patched_scratch.py (lowh patch) vs hold.py dense_local                   IDENTICAL (both)
  integ/diag10/hold_py.py ('py')       vs hold.py dense_local + dense_turn       IDENTICAL (both)
  integ/diag10/hold_pyn55.py ('pyn55') vs hold.py + dense_nadir_deg 55          IDENTICAL (both)
  hold.py off                          vs hold.py pyn55                          659 of 660 frames differ (the check works)
Summaries of these replays (fix error RMS / max m, fixes per s, longest gap s; one seed each, not a result table):
  ottawa2022 10 m s11: off 0.178 / 0.368, 10.29, 1.33; dl 0.195 / 0.339; py 0.167 / 0.274, 10.44, 1.33; pyn55 0.166 / 0.286
  farmland 5 m s12:    off 0.414 / 0.537, 9.64, 1.50; dl 0.203 / 0.342; py 0.043 / 0.159, 11.16, 1.42; pyn55 0.050 / 0.085
Files: results/ol/kf_*_proof3_relast_*.json.gz, kf_*_proofturn_relast_*.json.gz. hold.py md5 after these options:
d78329d3... (the proofs ran on it).

3. Test suites with the options off
-----------------------------------
Where: a copy of the current tree in the scratchpad (now/), so that no test output in the repository changes (tests/ and
tests_apb/out_*.txt). integ/kfmap/SITL_ACTIVE existed during each suite; each suite started at a 1-min load under 3.
One suite at a time. hold.py md5 of that run: a8418f45... (it differs from the final hold.py only inside KF_MAP code
that runs only with kf_map true; the final hold.py d78329d3 was gated again in section 3d).
Note: an earlier pair of runs at 03:12 (two suites at the same time, load 16-30) is not valid (E9 FAIL from the load);
its files were removed. During it the repository's tests_apb/out_test_basics.txt and out_test_digital.txt and
tests/regression.txt were overwritten for 3 minutes; they were put back from .snapshots/pre_kfmap_20261121_0310.tgz and
from integ/kfmap/results/regression_before_my_run.txt (same bytes, same times).

a) tests/run_regression.py (03:19:16-03:20:57, load 2.81 at the start, 10.68 at the end from its own 14 workers):
   results/suite/off1_regression.txt against the tracked tests/regression.txt (= results/regression_before_my_run.txt):
   every ID has the same result (17 PASS, 2 INFO), and every number of version 2 is the same, except:
   - E6 version-2 keyframes made [8, 7, 11, 13] -> [8, 7, 11, 14] (RMS 1.41 m, end errors and false-fix 2.2 % the same).
     This count changes between runs of the same code: the pre-change hold.py (hold_before.py) gave [8, 7, 9, 13] in
     the invalid 03:12 run, and integ/lowh/README.txt section 5b reports the same ("one v2 keyframe count").
   - version-1 numbers (E1 keyframes on static 11/3 -> 19/3, E6 v1 keyframes and prairie v1 keyframes): hold_v1 varies
     between runs (integ/lowh/README.txt 5b).
   - E9 / H6 times (15.7 -> 5.0 ms against 16.7 -> 4.4 ms; A53 estimate 50 ms against 44 ms): timing.
b) sh tests_apb/run_all.sh (03:22:37-03:31:04, load 2.68 at the start, 3.32 at the end): results/suite/off1_run_all.txt,
   outputs results/suite/off1_out/. 16 of 17 groups with no FAIL. test_live: 1 FAIL, the known 'overlay only' (21
   fixes sent in 288 frames, the same 21 fix times as the repository's out_test_live.txt of 01:33). test_lens_d: 0 FAIL,
   1 KNOWN LIMIT (as before). test_ap_sitl ALL PASS (its SITL ran in the copy's own container ap_sim_kf_suite, removed
   after). The last line of every out file equals the repository's out file of 01:27-01:35, except run times and a
   temp folder name.

d) FINAL gate run on the final hold.py (md5 d78329d3..., all options off), copy 'final' of the repository files of
   04:33 (the lead's apb_main.py 70a646ba with turn_problems, settings.py b94fefa9, tests_apb/ with test_att_yaw):
   - tests/run_regression.py, 04:49:51-04:52:25, load 2.88 at the start (9.20 at the end, its own 14 workers):
     results/suite/final_regression.txt. Every ID has the same result as the tracked tests/regression.txt (17 PASS,
     2 INFO). Numbers that differ: E1 version-1 'keyframes on static' 11/3 -> 20/3 (hold_v1, varies between runs);
     E6 version-1 keyframes [9, 11, 15, 15] -> [8, 11, 8, 12] and prairie v1 100 -> 105 (hold_v1); E6 version-2
     keyframes made [8, 7, 11, 13] -> [8, 7, 10, 13] with the same RMS 1.41 m, end errors and false-fix 2.2 % (this
     count varied in every run of this study: 13, 14 and here 10 in the third place; the pre-change hold.py gave 9 in
     the same place; integ/lowh/README.txt 5b saw the same); E9 / H6 times 16.7 -> 4.4 ms against 27.2 -> 7.0 ms
     (A53 estimate 44 -> 70 ms; timing, PASS); run time 101 -> 153 s.
   - sh tests_apb/run_all.sh, 04:53:45-05:05:33, load 2.61 / 2.50 (results/suite/final_run_all.txt, final_out/): 18
     groups now (test_att_yaw is new); 17 of 18 with no FAIL; test_live: the known 'overlay only' FAIL (21 fixes, as
     before). test_ap_sitl ALL PASS (own container ap_sim_kf_suite, removed after).
   - tests_apb/test_live.py with py + (b) (copy nowpy: HOLD_CFG dense_local + dense_turn, att_source gyro_fcyaw through
     a copy-only input HH_ATT_SOURCE of integ/gap/hand_hover.py), 05:05:33-05:07:02, load 2.50 / 3.35
     (results/suite/pic_test_live_py_gyro_fcyaw.txt): 1 FAIL, 'overlay only' with 22 fixes in 288 frames (off: 21; one
     more); hovers 0.08 s (all 6 PASS); freeze repeat 40 m 1 of 72, last fix 0.00 s (as off); freeze noise 40 m 11 of
     72 frames sent, last fix 0.84 s after the freeze (limit 1.0 s; off and py with the default gyro: 7 of 72, 0.50 s):
     PASS, but closer to the limit.

c) Picture safety with DENSE_LOCAL on (03:31-03:36, copy nowdl, one test at a time, load 2.91-2.97 at each start;
   results/suite/pic_*; test_live of the copy passes HOLD_CFG to integ/gap/hand_hover.py --hold_cfg: one line in the
   copy only, tests_apb/ of the repository is not changed):
   - tests/bad_pictures.py (static std 5/30/60, frozen, frozen + noise, NO SIGNAL, breakup; 100 m) with dense_local:
     byte-identical to the tracked tests/bad_pictures.txt. With dense_local + kf_map: identical too.
   - tests_apb/test_live.py with dense_local: 2 FAIL. 'overlay only' 21 fixes in 288 frames (the known FAIL; the same
     count as off; the last 4 fix times 19.67-19.92 s against 18.84-19.09 s off), and NEW: 'hover 10 m seed 12: longest
     time with no fix in the last 10 s is 0.33 s (limit 0.3 s)' (off: 0.08 s; hover 10 m seed 11: 0.08 s, off 0.17 s).
     This is the same result as integ/lowh/README.txt section 4 found for the patch (one frame over the limit at 12 Hz).
     freeze repeat 40 m: 1 of 72 frames sent, last fix 0.00 s after the freeze (as off); freeze noise 40 m: 7 of 72,
     0.50 s (as off). The same 2 FAIL with dense_local + kf_map.
   - The half-frozen and overlay pictures at 5 m (integ/lowh/README.txt 5a): not run again; DENSE_LOCAL is that patch
     bit for bit (section 2), so its 5a numbers apply: half-frozen pictures on the whole-picture path give more Hold
     fixes (61 -> 75 and 66 -> 113 of 360) and larger errors (0.12 -> 0.27 m, 0.32 -> 0.47 m) than off.
   - py (dense_local + dense_turn), 04:04-04:07, copy nowpy (hold.py d78329d3, the lead's apb_main/settings of 03:39),
     load 2.72 / 2.77 at the starts (results/suite/pic_py_log.txt):
     tests/bad_pictures.py: byte-identical to the tracked tests/bad_pictures.txt (results/suite/
     pic_bad_pictures_dense_localdense_turn.txt). tests_apb/test_live.py: 1 FAIL, the known 'overlay only' (21 fixes in
     288 frames, the same count as off); every hover gap 0.08 s (the 0.33 s of DENSE_LOCAL alone is gone); freeze repeat
     1 of 72, freeze noise 7 of 72, last fix 0.00 / 0.50 s after the freeze (as off)
     (results/suite/pic_test_live_dense_localdense_turn.txt).
     att_source gyro_fcyaw in these tests: tests/bad_pictures.py has no attitude filter (it gives Hold the attitude
     directly), so att_source does not apply there. test_live (integ/gap/hand_hover.py) builds apb_main.att_filter from
     settings with no input for att_source; the copies nowpy and final got a copy-only env input HH_ATT_SOURCE
     (the py + (b) run is in section 3d).

4. Open loop, VALIDATION data
-----------------------------
Not run as a matrix by this study (the lead moved the work at 03:36). The diag10 study has the open-loop validation
of py and pyn55 at 10 m (integ/diag10/README.txt section '03:38'). The proof replays of section 2b are single seeds.

5. KF_MAP: state when it was stopped (03:36)
--------------------------------------------
Built (section 1b), off by default. KF_MAP False runs the statements of the code before (the off proofs of sections 2
and 2b ran with it in the file and off). Tested only in a few open-loop replays on ottawa2019fall (TUNING) at 5 m:
- natural re-anchors (seed 11, 55 s): 1 relocalization accepted with both on (map position error 0.06 m), 2 with
  KF_MAP alone (0.05 m); error RMS both 0.094 m against DENSE_LOCAL alone 0.055 m (worse: after the switch back the
  hold stays on the old whole-picture keyframe while the drone drifts away from it), KF_MAP alone 0.114 against off
  0.138 m (results/ol/kf_ottawa2019fall_crop_5m_s11_dbg1_relast_*.json.gz).
- a forced re-anchor at 10 s with a 0.5 m offset (seed 11, 30 s): accepted at 14.1 s (3 tries 0.5 s apart, steps
  0.53/0.53/0.52 m, map position error 0.06-0.07 m), the error went from 0.5 m back to 0.06 m within the 0.3 m/s ramp;
  DENSE_LOCAL alone kept the 0.5 m (results/ol/kf_ottawa2019fall_crop_5m_s11_dbg2_relast_*.json.gz).
- One try took 2.6-7 ms on the laptop (2 threads; one 45 ms with ORB); a board estimate (x10, not measured) is 30-70 ms
  on a frame of an 80 ms budget, every 0.5 s.
NOT DONE for KF_MAP: the validation matrix, the false-acceptance tests (another place, bad pictures, repeated texture:
scripts ready, integ/kfmap/jobs.py sets place, bad, rep; tables.py), the timing at one thread, any closed-loop run. The
false-acceptance rate is NOT measured: KF_MAP must stay off. (The KF_MAP SITL copies with the Pipe note line of 1c
were deleted unused; the A/B copy of section 7 has the repository's apb_main.py.)
Design limits found: the stored home of a chain that started in the climb is at another height (tries refused by
RELOC_HGT); handled by trying the newest stored keyframe at this height, and by replacing a map that has no keyframe at
the current height (_store_map). A map stays in use while the drift is under the step limit; a larger drift is refused
forever (and costs one try every 0.5 s).

6. Compute time
---------------
Not measured on the board. Hold ms per frame in the proof replays (laptop, cv2.setNumThreads(1), load not controlled;
results/ol/*_proofturn_* and *_proof3_*): ottawa2022 10 m off 4.94 / 4.77, dl 5.14 / 5.05, py 5.63 / 5.50, pyn55 5.39;
farmland 5 m off 4.79 / 4.49, dl 4.36, py 5.51 / 5.23, pyn55 5.82 ms. So py costs about +0.5-0.9 ms per frame here.

7. Closed loop, ArduCopter SITL (the lead's A/B list)
----------------------------------------------------
Setup: apsim/low_ap_matrix.py in one copy of the tree (scratchpad closed/ab; container ap_sim_kfab; SITL instance 1),
as integ/lowh/closed/README.txt: variant apb, video JUSTICE_D, ground ottawa2019fall (TUNING), wind w20g40, tuning
seeds 1 and 2, 60 s hold in LOITER, min_height_m 3, hold_cfg h_min 2, speedup 1, board model 80 ms per Hold.process.
hold.py md5 d78329d3... and the repository's apb_main.py with fix (b) (md5 97389334...) in the copy; from 04:08 (the
runs after 5 m seed 1 base) the lead's apb_main.py 70a646ba (adds only the start check turn_problems). One SITL at a time;
each run starts at a 1-min load under 3 (integ/kfmap/closed/scripts/kf_driver.py, log integ/kfmap/closed/driver.log);
integ/kfmap/SITL_ACTIVE exists during the phase. Edits in the copy only: low_ap_matrix.py labels
lowap_apb_<h>m_w20g40_s<seed>_<version> (version: base = all off; py = hold_cfg dense_local + dense_turn; h15 = base
with case hover_s 15; b = settings att_source gyro_fcyaw; pyb = py + b; pyn55 = py + dense_nadir_deg 55), DEADLINE,
run_sitl.sh container name. In the repository: apsim/sitl_ap.py got the case key hover_s (the steady hover before the
switch, default 4.0 s = the run as before; one line, line 355; copy before: integ/kfmap/sitl_ap_before.py).
Table: integ/kfmap/closed/scripts/metrics_ab.py -> integ/kfmap/closed/ab_table.txt (definitions at its end).
FC yaw course per run: integ/kfmap/closed/scripts/yaw_course.py.

RESULTS 10 m (04:04; ab_table.txt; 30 s from the switch; seed 1 / seed 2; all starts at load 2.78-2.99):
  version           rms m      max m      gap s      ferr m     re-anchors  FC yaw-error change in 30 s (deg)
  base              0.79/0.71  1.39/1.12  1.31/1.38  0.77/0.73  2/1         1.8/2.2
  py                0.43/0.36  0.85/0.77  0.25/0.41  0.08/0.10  0/0         8.7/11.7
  base, hover 15 s  0.51/0.41  1.00/0.79  1.46/1.38  0.33/0.26  2/2         1.3/1.7
  (b) gyro_fcyaw    0.57/0.42  1.26/0.90  1.54/1.38  0.31/0.16  6/4         4.1/2.8
  py + (b)          0.42/0.33  0.84/0.74  0.41/0.16  0.11/0.04  0/0         2.2/1.6
  extnav -> switch (s): base 2.8/3.4; py 7.1/7.5; hover 15 s 13.5/19.5; (b) 6.0/6.1; py + (b) 6.0/7.6.
  FC 'in-flight yaw alignment' 8.9-9.2 s before the switch (19.9/20.7 s with the 15 s hover) in all 10 runs, yaw-error
  step -12.2 deg.
  SAFETY: py alone bends the FC yaw (yaw_course.py): s1 -2.0 deg at the switch, +6.1 at 15 s, +2.5 steady to 60 s;
  s2 -1.8, +9.4 at 20 s, +5.7 at 60 s (does not return). base: within 2 deg, toward 0. py + (b): s1 -2.1 .. -3.4 deg,
  s2 -2.0 .. -0.8 deg. Inference (not tested): GyroAtt follows the FC's -12 deg alignment step with tau 20 s; py keeps
  the yaw frame of the first keyframes (made with the lagging yaw), so the fixes come in a rotated frame and the EKF
  yaw follows them; (b) gives Hold the FC yaw after the step at once. => do not fly py without (b).
  60 s values (rms / ferr): base 0.99/1.04, 0.86/0.91; py 0.39/0.12, 0.40/0.11; hover15 0.62/0.54, 0.50/0.38;
  (b) 0.84/0.70, 0.51/0.25; py + (b) 0.40/0.15, 0.38/0.07. Longest gap over 60 s: py + (b) 1.71 / 1.31 s (in 30 s 0.41/0.16).

RESULTS 5 m (04:24; ab_table.txt; 30 s from the switch; seed 1 / seed 2; starts at load 2.8-3.0 (2.98 max)). fmax:
largest sent-fix error against the truth (relative to the switch); bad: sent fixes more than 0.5 m wrong / sent:
  version           rms m      max m      gap s      ferr m     fmax m     bad/sent          re-anchors  FC yaw chg (deg)
  base              0.73/0.55  1.45/0.86  6.35/3.01  0.43/0.39  1.13/0.58  20/131, 90/242    14/8        1.6/1.6
  base, hover 15 s  0.55/0.40  1.17/0.80  3.66/1.54  0.16/0.20  0.49/0.34  0/196, 0/259      11/5        0.6/1.3
  (b) gyro_fcyaw    0.47/0.34  1.01/0.79  4.71/2.44  0.18/0.09  0.50/0.14  1/189, 0/296      15/4        1.7/1.3
  py + (b)          0.41/0.35  0.82/0.78  1.38/1.46  0.04/0.04  0.10/0.09  0/325, 0/332      2/2         2.4/1.6
  (py alone was not run at 5 m: the lead's interlock refuses it.)
  10 m, the same two columns: base fmax 1.17/1.05, bad 209/285, 202/296; py + (b) fmax 0.20/0.09, bad 0/331, 0/363;
  hover 15 s 0.59/0.49, 22/277, 0/314; (b) 0.68/0.36, 17/244, 0/273; py 0.18/0.23, 0/355, 0/331.
  60 s (rms / ferr / fmax / bad): 5 m base 1.02/1.00/1.29/300, 0.71/0.55/0.78/286; py + (b) 0.37/0.05/0.11/0,
  0.42/0.07/0.15/0; hover 15 s 0.49/0.21/0.49/0, 0.50/0.36/0.58/126; (b) 0.47/0.32/0.50/1, 0.64/0.43/0.96/166.
  FC yaw error with py + (b) at 5 m (yaw_course.py): s1 -2.4 at the switch, -1.2 .. -3.4 deg to 60 s; s2 -2.3, -0.8 ..
  -2.3 deg: no bend. The FC's in-flight yaw alignment came 9.1-9.2 s before the switch (20.2 s with the hover), step
  -10.5 deg, in every 5 m run.

RESULTS VALIDATION, 10 m (04:32; ground ottawa2022 = a VALIDATION site of DATA_SPLITS.md, seeds 11 and 12 =
VALIDATION seeds; nothing was tuned on these runs; ab_table.txt; 30 s from the switch; seed 11 / seed 12; starts at
load 2.8-2.9):
  version     rms m      max m      gap s      ferr m     fmax m     bad/sent            re-anchors  FC yaw chg (deg)
  base        0.74/0.65  1.25/1.11  1.38/1.30  0.86/0.64  1.19/0.91  226/263, 207/289   5/3         1.6/3.2
  py + (b)    0.34/0.27  0.71/0.58  2.12/0.25  0.08/0.06  0.21/0.14  0/246, 0/275       5/0         0.7/1.6
  60 s: base rms 1.07/0.89, ferr 1.14/0.88, fmax 1.52/1.20; py + (b) 0.37/0.29, 0.15/0.05, 0.28/0.14, bad 0 and 0.
  Against the case: py + (b) seed 11 had a longer gap (2.12 s against 1.38 s) and 5 re-anchors in 30 s (base 5).
  FC yaw error with py + (b): s11 -0.6 at the switch, -0.3 .. -1.3 deg to 60 s; s12 -0.8, -0.3 .. -2.1 deg: no bend.

RESULTS flight version py + (b) + hover 15 s, 10 m (04:39; TUNING s1 / s2; starts at load 2.5 / 2.9):
  py + (b) + hover 15 s  rms 0.42/0.35  max 0.89/0.72  gap 0.12/0.17  ferr 0.05/0.06  fmax 0.14/0.14  bad 0/362, 0/361
                         re-anchors 0/0  FC yaw chg 1.3/4.4 deg  extnav -> switch 19.5/19.9 s
  (py + (b) with the 4 s hover: rms 0.42/0.33, max 0.84/0.74, gap 0.41/0.16, ferr 0.11/0.04, fmax 0.20/0.09.)
  The longer hover adds little to py + (b) at 10 m (shorter gaps, the same RMS). Against the case: in s2 the FC yaw
  error went from -0.3 deg at the switch to +3.6 deg at 20 s and stayed at +3.7 to +3.9 deg to 60 s (yaw_course.py), a
  larger bend than py + (b) had in its other 6 runs (0.7-2.4 deg in 30 s) and than base (1.6-3.2 deg).

RESULTS 30 m, seed 1 (04:46; TUNING; starts at load under 3):
  base      rms 0.45  max 0.80  gap 0.12  ferr 0.12  fmax 0.32  bad 0/359  re-anchors 0  FC yaw chg 3.2 deg
  py + (b)  rms 0.42  max 0.73  gap 0.09  ferr 0.09  fmax 0.24  bad 0/360  re-anchors 0  FC yaw chg 2.3 deg
  Seed 2 (05:51, round 5, starts at load 2.92 / 2.92):
  base      rms 0.40  max 0.77  gap 0.11  ferr 0.10  fmax 0.21  bad 0/357  FC yaw chg 2.6 deg | 60 s: rms 0.44 max 0.96 ferr 0.13
  py + (b)  rms 0.40  max 0.78  gap 0.11  ferr 0.13  fmax 0.33  bad 0/359  FC yaw chg 1.9 deg | 60 s: rms 0.47 max 1.15 ferr 0.21
  30 m, 2 seeds: no gain and no clear harm in 30 s (0.45/0.40 -> 0.42/0.40 m); over 60 s seed 2 slightly worse (0.44 ->
  0.47 m RMS, max 0.96 -> 1.15 m); no fix over 0.5 m wrong in either version. Note: the absolute sent-fix error (without removing the error at the switch) of py + (b)
  was 0.74 m at most, with 257 of 360 fixes over 0.5 m (base: 0.41 m, 0): an offset that the fixes had already at the
  switch; the hold follows the fix, so it does not move the aircraft from its hold point (rms/max above).

RESULTS round 4 (04:52-05:45; 30 s from the switch; base -> py + (b); all starts at load 2.80-2.96):
  case                          rms m        max m        gap s        ferr m       fmax m       bad (>0.5 m)/sent    FC yaw chg deg
  60 m s1 (TUNING)              0.49 -> 0.59 0.92 -> 1.04 0.10 -> 0.09 0.23 -> 0.25 0.55 -> 0.57 2/359 -> 9/359      2.2 -> 5.5
  60 m s2 (TUNING)              0.41 -> 0.44 0.88 -> 0.87 0.09 -> 0.12 0.19 -> 0.34 0.42 -> 0.73 0/361 -> 40/359     2.1 -> 7.0
  10 m s3 (TUNING)              0.62 -> 0.31 1.10 -> 0.62 1.31 -> 0.10 0.63 -> 0.04 0.82 -> 0.10 256/334 -> 0/363   3.0 -> 2.0
  5 m s3 (TUNING)               0.41 -> 0.33 0.78 -> 0.66 2.62 -> 0.41 0.19 -> 0.03 0.29 -> 0.05 0/266 -> 0/300     2.6 -> 1.3
  10 m s13 ottawa2022 (VALID.)  0.94 -> 0.50 1.48 -> 1.10 1.81 -> 0.25 0.97 -> 0.09 1.31 -> 0.23 205/240 -> 0/315   2.1 -> 2.5
  10 m s11 farmland (VALID.)    0.32 -> 0.33 0.67 -> 0.77 1.89 -> 1.32    - -> 0.06    - -> 0.12  (*) -> 0/312      4.3 -> 2.4
  10 m s12 farmland (VALID.)    0.33 -> 0.25 0.84 -> 0.51 1.31 -> 0.16 0.17 -> 0.03 0.27 -> 0.06 0/297 -> 0/365     3.9 -> 1.0
  5 m s11 ottawa2022 (VALID.)   0.51 -> 0.33 1.33 -> 0.76 4.11 -> 1.40    - -> 0.05    - -> 0.11  (*) -> 0/278      2.6 -> 1.3
  5 m s12 ottawa2022 (VALID.)   0.57 -> 0.28 1.05 -> 0.62 2.28 -> 1.41 0.51 -> 0.05 0.74 -> 0.10 105/215 -> 0/313   1.6 -> 1.6
  round 5 (05:52-06:03):
  5 m s11 farmland (VALID.)     0.55 -> 0.32 1.58 -> 0.72 3.12 -> 0.16 0.26 -> 0.03 0.85 -> 0.07 19/242 -> 0/363    2.4 -> 1.5
  5 m s12 farmland (VALID.)     0.41 -> 0.27 0.83 -> 0.61 3.04 -> 0.33 0.24 -> 0.03 0.47 -> 0.08 0/258 -> 0/360     2.5 -> 1.3
  (farmland 5 m base s11 over 60 s: rms 1.55 m, max 3.83 m, gap 7.69 s, 160 bad fixes; py + (b): 0.37, 0.77, 0.33 s, 0.)
  (*) base sent no fix in the last 1 s before the switch, so the relative fix error has no reference (absolute: farmland
  s11 fmax 0.41 m, 0 of 295 over 0.5 m; ottawa2022 5 m s11 fmax 1.19 m, 194 of 231 over 0.5 m).
  60 m: py + (b) is WORSE than base in both seeds. The FC yaw error at the switch was -7.1 / -5.3 deg (base -0.6 / -0.1):
  the FC yaw was bent during the climb, before the switch (60 m s1: -1.3 deg at -15 s, -4.5 at -5 s, -7.1 at the
  switch, back to -2.6 at 60 s). 60 s values: s1 bad fixes 32 -> 127, s2 5 -> 397, fmax 0.55 -> 0.73 and 0.52 -> 1.21 m.
  Inference (not tested): above about 30 m the whole-picture path (with the turn solve) still runs in the climb and on
  some frames, and its frame couples with the FC yaw through (b). => use low_height only for holds at 5-10 m (20 m and
  30 m: no gain; 60 m seed 3 below).

RESULT 60 m, seed 3 (06:12, round 5; gate load 2.96 / 2.92):
  base      rms 0.47  max 0.99  gap 0.09  ferr 0.27  fmax 0.66  bad 12/359  FC yaw at switch +0.6, chg 4.7
  py + (b)  rms 0.39  max 0.86  gap 0.09  ferr 0.21  fmax 0.49  bad 0/359   FC yaw at switch -3.4, chg 2.7
  60 m over 3 seeds: py + (b) worse in s1 and s2, better in s3 (RMS 0.49/0.41/0.47 -> 0.59/0.44/0.39 m; bad fixes
  2/0/12 -> 9/40/0). The FC yaw error at the switch was bent in the climb in 3 of 3 seeds with py + (b) (-7.1, -5.3,
  -3.4 deg; base -0.6, -0.1, +0.6 deg).

RESULT 20 m, seed 1 (06:07, round 5; gate load 2.98 / 2.80):
  base      rms 0.38  max 0.80  gap 0.25  ferr 0.09  fmax 0.19  bad 0/356  FC yaw chg 3.2 | 60 s: rms 0.37 ferr 0.11 fmax 0.24
  py + (b)  rms 0.44  max 0.77  gap 0.12  ferr 0.12  fmax 0.26  bad 0/357  FC yaw chg 1.4 | 60 s: rms 0.42 ferr 0.20 fmax 0.40
  20 m: no gain; RMS slightly worse (+0.06 m over 30 s, +0.05 m over 60 s), one seed.

RESULTS round 6 (06:13-06:28; gate load 2.78-2.99):
  10 m s13 farmland (VALID.)    base 0.55 / max 1.24 / gap 3.62 / ferr 0.23 / fmax 0.42 / 0 of 255 -> py + (b) 0.46 / 1.03
                                / 1.32 / 0.08 / 0.28 / 0 of 326
  5 m s13 ottawa2022 (VALID.)   base 0.76 / 1.28 / 4.19 / 0.68 / 0.92 / 136 of 191 -> py + (b) 0.47 / 0.97 / 1.40 / 0.10 /
                                0.21 / 0 of 268
  5 m py + (b) + hover 15 s     s1 0.42 / 0.80 / 1.39 / 0.07 / 0.14 / 0 of 332; s2 0.35 / 0.76 / 1.32 / 0.04 / 0.12 / 0 of 330
                                (py + (b) with 4 s: 0.41/0.35, 0.82/0.78, 1.38/1.46, 0.04/0.04): the hover adds nothing at 5 m.
                                FC yaw: s1 -0.6 at the switch, -3.8 at +30 s, -2.4 at +60 s (max |err| 4.4); s2 -0.8, -1.5, -2.6.

FINAL SUMMARY, base -> py + (b) pair by pair (integ/kfmap/closed/scripts/pairs.py -> integ/kfmap/closed/pairs.txt):
  h   ground          seed  rms30 base->pyb   max30           gap30           ferr30          fmax30          bad30 (>0.5 m)/sent        FC yaw chg30
  5   farmland          11  0.55 -> 0.32    1.58 -> 0.72    3.12 -> 0.16    0.26 -> 0.03    0.85 -> 0.07    19/242 -> 0/363        2.4 -> 1.5
  5   farmland          12  0.41 -> 0.27    0.83 -> 0.61    3.04 -> 0.33    0.24 -> 0.03    0.47 -> 0.08    0/258 -> 0/360        2.5 -> 1.3
  5   ottawa2019fall     1  0.73 -> 0.41    1.45 -> 0.82    6.35 -> 1.38    0.43 -> 0.04    1.13 -> 0.10    20/131 -> 0/325        1.6 -> 2.4
  5   ottawa2019fall     2  0.55 -> 0.35    0.86 -> 0.78    3.01 -> 1.46    0.39 -> 0.04    0.58 -> 0.09    90/242 -> 0/332        1.6 -> 1.6
  5   ottawa2019fall     3  0.41 -> 0.33    0.78 -> 0.66    2.62 -> 0.41    0.19 -> 0.03    0.29 -> 0.05    0/266 -> 0/300        2.6 -> 1.3
  5   ottawa2022        11  0.51 -> 0.33    1.33 -> 0.76    4.11 -> 1.40       - -> 0.05       - -> 0.11    -1/231 -> 0/278        2.6 -> 1.3
  5   ottawa2022        12  0.57 -> 0.28    1.05 -> 0.62    2.28 -> 1.41    0.51 -> 0.05    0.74 -> 0.10    105/215 -> 0/313        1.6 -> 1.6
  5   ottawa2022        13  0.76 -> 0.47    1.28 -> 0.97    4.19 -> 1.40    0.68 -> 0.10    0.92 -> 0.21    136/191 -> 0/268        1.5 -> 1.2
  10  farmland          11  0.32 -> 0.33    0.67 -> 0.77    1.89 -> 1.32       - -> 0.06       - -> 0.12    -1/295 -> 0/312        4.3 -> 2.4
  10  farmland          12  0.33 -> 0.25    0.84 -> 0.51    1.31 -> 0.16    0.17 -> 0.03    0.27 -> 0.06    0/297 -> 0/365        3.9 -> 1.0
  10  farmland          13  0.55 -> 0.46    1.24 -> 1.03    3.62 -> 1.32    0.23 -> 0.08    0.42 -> 0.28    0/255 -> 0/326        5.9 -> 2.6
  10  ottawa2019fall     1  0.79 -> 0.42    1.39 -> 0.84    1.31 -> 0.41    0.77 -> 0.11    1.17 -> 0.20    209/285 -> 0/331        1.8 -> 2.2
  10  ottawa2019fall     2  0.71 -> 0.33    1.12 -> 0.74    1.38 -> 0.16    0.73 -> 0.04    1.05 -> 0.09    202/296 -> 0/363        2.2 -> 1.6
  10  ottawa2019fall     3  0.62 -> 0.31    1.10 -> 0.62    1.31 -> 0.10    0.63 -> 0.04    0.82 -> 0.10    256/334 -> 0/363        3.0 -> 2.0
  10  ottawa2022        11  0.74 -> 0.34    1.25 -> 0.71    1.38 -> 2.12    0.86 -> 0.08    1.19 -> 0.21    226/263 -> 0/246        1.6 -> 0.7
  10  ottawa2022        12  0.65 -> 0.27    1.11 -> 0.58    1.30 -> 0.25    0.64 -> 0.06    0.91 -> 0.14    207/289 -> 0/275        3.2 -> 1.6
  10  ottawa2022        13  0.94 -> 0.50    1.48 -> 1.10    1.81 -> 0.25    0.97 -> 0.09    1.31 -> 0.23    205/240 -> 0/315        2.1 -> 2.5
  20  ottawa2019fall     1  0.38 -> 0.44    0.80 -> 0.77    0.25 -> 0.12    0.09 -> 0.12    0.19 -> 0.26    0/356 -> 0/357        3.2 -> 1.4
  30  ottawa2019fall     1  0.45 -> 0.42    0.80 -> 0.73    0.12 -> 0.09    0.12 -> 0.09    0.32 -> 0.24    0/359 -> 0/360        3.2 -> 2.3
  30  ottawa2019fall     2  0.40 -> 0.40    0.77 -> 0.78    0.11 -> 0.11    0.10 -> 0.13    0.21 -> 0.33    0/357 -> 0/359        2.6 -> 1.9
  60  ottawa2019fall     1  0.49 -> 0.59    0.92 -> 1.04    0.10 -> 0.09    0.23 -> 0.25    0.55 -> 0.57    2/359 -> 9/359        2.2 -> 5.5
  60  ottawa2019fall     2  0.41 -> 0.44    0.88 -> 0.87    0.09 -> 0.12    0.19 -> 0.34    0.42 -> 0.73    0/361 -> 40/359        2.1 -> 7.0
  60  ottawa2019fall     3  0.47 -> 0.39    0.99 -> 0.86    0.09 -> 0.09    0.27 -> 0.21    0.66 -> 0.49    12/359 -> 0/359        4.7 -> 2.7
  
  5-10 m: 17 pairs; py + (b) lower RMS in 16, lower max in 16, shorter longest gap in 16.
  py + (b) runs at 5-10 m (incl. hover 15 s) with sent fixes more than 0.5 m wrong (30 s / 60 s): none
  py + (b) runs at 5-10 m counted: 21; with any such fix: 0.
  (-1 = no fix in the last 1 s before the switch, so the relative error has no reference.)
  Not better at 5-10 m: 10 m farmland s11 (RMS 0.32 -> 0.33 m, max 0.67 -> 0.77 m); longer gap: 10 m ottawa2022 s11
  (1.38 -> 2.12 s). Base runs sent up to 256 of 334 fixes more than 0.5 m wrong; py + (b) none in 21 runs at 5-10 m. Loads: the driver's gate saw 2.54-2.997 at every start (ab_table.txt column 'gate load'); the
runner's own load a few seconds later was up to 3.9 (5 m farmland s11 base).

FC YAW ERROR OVER THE FLIGHT (05:51; the lead's question: does it grow with the time in the air?)
integ/kfmap/closed/scripts/yaw_table.py -> integ/kfmap/closed/yaw_table.txt (table re-run at 06:28 with all runs).
FC yaw error = ATTITUDE yaw minus the true yaw (SIM_STATE truth log), deg, '@' flight time from take-off (true height
0.3 m over its start). 'max |err|' over the 60 s hold, and from the first sent fix to the end of the hold.
  FC yaw error (deg) @ flight time from take-off (s). first fix = first sent fix of the flight.
    h ground         seed version             first fix        switch         +30 s         +60 s fix->switch s max |err| hold max |err| from fix
    5 farmland         11 base              -0.4 @  2.9s   -1.4 @ 10.6s   +0.8 @ 40.6s   +1.5 @ 70.6s           7.7            1.6                1.6
    5 farmland         11 py+(b)            -0.4 @  2.7s   -1.4 @ 10.6s   -1.9 @ 40.6s   -1.4 @ 70.6s           7.9            1.9                1.9
    5 farmland         12 base              -0.5 @  3.2s   -1.5 @ 10.6s   -2.2 @ 40.6s   -1.1 @ 70.6s           7.4            2.2                2.2
    5 farmland         12 py+(b)            -0.4 @  2.7s   -1.4 @ 10.6s   -2.2 @ 40.6s   -0.4 @ 70.6s           7.9            2.3                2.3
    5 ottawa2019fall    1 base              -2.6 @  9.2s   -2.7 @ 10.7s   -2.9 @ 40.7s   -3.3 @ 70.7s           1.4            3.8                3.8
    5 ottawa2019fall    1 base+hover15      -2.6 @  9.4s   -1.1 @ 21.6s   -1.7 @ 51.6s   -1.5 @ 81.6s          12.3            2.4                2.8
    5 ottawa2019fall    1 py+(b)            -1.0 @  3.2s   -2.4 @ 10.5s   -3.4 @ 40.5s   -3.3 @ 70.5s           7.4            3.5                3.5
    5 ottawa2019fall    1 py+(b)+hover15    -0.7 @  3.1s   -0.6 @ 21.7s   -3.8 @ 51.7s   -2.4 @ 81.7s          18.6            4.4                4.4
    5 ottawa2019fall    2 base              -2.6 @  9.1s   -2.7 @ 10.6s   -3.7 @ 40.6s   -3.5 @ 70.6s           1.5            4.1                4.1
    5 ottawa2019fall    2 base+hover15      -2.7 @  9.4s   -1.1 @ 21.6s   -2.1 @ 51.6s   -2.4 @ 81.6s          12.2            3.0                3.0
    5 ottawa2019fall    2 py+(b)            -0.9 @  3.0s   -2.3 @ 10.6s   -1.0 @ 40.6s   -2.0 @ 70.6s           7.5            2.4                2.7
    5 ottawa2019fall    2 py+(b)+hover15    -0.6 @  3.0s   -0.8 @ 21.6s   -1.5 @ 51.6s   -2.6 @ 81.6s          18.6            2.6                2.6
    5 ottawa2019fall    3 base              -1.9 @  8.0s   -2.1 @ 10.7s   -0.3 @ 40.7s   -2.0 @ 70.7s           2.7            2.1                2.1
    5 ottawa2019fall    3 py+(b)            -0.7 @  3.1s   -2.0 @ 10.7s   -0.6 @ 40.7s   -1.5 @ 70.7s           7.6            2.0                2.1
    5 ottawa2022       11 base              -0.8 @  7.9s   -0.8 @ 10.7s   -0.9 @ 40.7s   -1.0 @ 70.7s           2.8            1.7                1.7
    5 ottawa2022       11 py+(b)            -0.3 @  3.0s   -1.1 @ 10.6s   -2.3 @ 40.6s   -2.4 @ 70.6s           7.7            3.7                3.7
    5 ottawa2022       12 base              -1.1 @  8.8s   -1.1 @ 10.6s   -1.4 @ 40.6s   -1.3 @ 70.6s           1.8            1.7                1.7
    5 ottawa2022       12 py+(b)            -0.3 @  2.8s   -1.1 @ 10.5s   -1.1 @ 40.5s   -2.5 @ 70.5s           7.8            2.7                2.7
    5 ottawa2022       13 base              -0.8 @  8.8s   -0.8 @ 10.7s   -1.5 @ 40.7s   -1.0 @ 70.7s           1.9            2.1                2.1
    5 ottawa2022       13 py+(b)            -0.2 @  2.8s   -0.9 @ 10.6s   -1.0 @ 40.6s   -1.4 @ 70.6s           7.9            1.6                1.6
   10 farmland         11 base              -0.2 @  2.0s   -0.7 @ 11.0s   +3.6 @ 41.0s   +2.8 @ 71.0s           8.9            4.4                4.4
   10 farmland         11 py+(b)            -0.1 @  1.9s   -0.5 @ 11.3s   -0.7 @ 41.3s   +0.9 @ 71.3s           9.3            1.9                1.9
   10 farmland         12 base              -0.4 @  3.3s   -0.9 @  9.9s   +1.8 @ 39.9s   +1.6 @ 69.9s           6.6            3.0                3.0
   10 farmland         12 py+(b)            -0.2 @  2.1s   -1.0 @ 10.8s   -1.2 @ 40.8s   -1.1 @ 70.8s           8.7            1.4                1.4
   10 farmland         13 base              -0.1 @  2.0s   -0.7 @ 11.0s   +1.8 @ 41.0s   +1.9 @ 71.0s           9.0            5.2                5.2
   10 farmland         13 py+(b)            -0.1 @  2.0s   -0.9 @ 11.2s   +1.2 @ 41.2s   +0.4 @ 71.2s           9.1            1.6                1.6
   10 ottawa2019fall    1 base              -1.9 @  7.2s   -2.0 @ 10.2s   -0.4 @ 40.2s   -0.7 @ 70.2s           2.9            2.0                2.1
   10 ottawa2019fall    1 base+hover15      -2.0 @  7.3s   -0.2 @ 20.9s   -0.8 @ 50.9s   -0.8 @ 80.9s          13.6            1.5                2.1
   10 ottawa2019fall    1 py+(b)            -1.2 @  3.9s   -2.1 @ 10.0s   -3.0 @ 40.0s   -3.4 @ 70.0s           6.1            4.0                4.0
   10 ottawa2019fall    1 py+(b)+hover15    -0.4 @  2.1s   -0.5 @ 21.8s   -1.5 @ 51.8s   -1.0 @ 81.8s          19.7            1.6                1.7
   10 ottawa2019fall    2 base              -1.9 @  6.7s   -2.1 @ 10.3s   -0.7 @ 40.3s   -0.5 @ 70.3s           3.6            2.1                2.1
   10 ottawa2019fall    2 base+hover15      -0.5 @  2.1s   -0.2 @ 21.7s   -1.1 @ 51.7s   -0.5 @ 81.7s          19.6            1.9                2.3
   10 ottawa2019fall    2 py+(b)            -0.7 @  2.5s   -2.0 @ 10.2s   -1.5 @ 40.2s   -0.9 @ 70.2s           7.7            2.0                2.2
   10 ottawa2019fall    2 py+(b)+hover15    -0.3 @  1.9s   -0.3 @ 22.1s   +3.7 @ 52.1s   +3.8 @ 82.1s          20.1            4.1                4.1
   10 ottawa2019fall    3 base              -1.3 @  5.4s   -1.6 @ 10.3s   +1.4 @ 40.3s   +0.3 @ 70.3s           4.9            1.7                1.7
   10 ottawa2019fall    3 py+(b)            -0.5 @  2.4s   -1.6 @ 10.7s   +0.1 @ 40.7s   -0.4 @ 70.7s           8.3            1.6                1.7
   10 ottawa2022       11 base              -0.5 @  5.5s   -0.7 @ 10.0s   +0.4 @ 40.0s   +0.5 @ 70.0s           4.5            1.1                1.1
   10 ottawa2022       11 py+(b)            -0.4 @  4.0s   -0.6 @  9.9s   -1.3 @ 39.9s   -0.3 @ 69.9s           5.9            1.8                1.8
   10 ottawa2022       12 base              -0.5 @  5.6s   -0.6 @  9.9s   +1.1 @ 39.9s   +1.1 @ 69.9s           4.3            2.6                2.6
   10 ottawa2022       12 py+(b)            -0.2 @  2.4s   -0.8 @ 10.0s   -1.9 @ 40.0s   -1.1 @ 70.0s           7.6            2.4                2.4
   10 ottawa2022       13 base              -0.4 @  6.0s   -0.5 @ 10.1s   +1.7 @ 40.1s   +1.7 @ 70.1s           4.1            1.8                1.8
   10 ottawa2022       13 py+(b)            -0.1 @  2.2s   -0.5 @ 10.4s   -2.6 @ 40.4s   -1.8 @ 70.4s           8.2            3.0                3.0
   20 ottawa2019fall    1 base              -1.1 @  4.6s   -2.0 @ 12.4s   -0.5 @ 42.4s   -1.1 @ 72.4s           7.8            2.3                2.3
   20 ottawa2019fall    1 py+(b)            -0.4 @  2.2s   -0.9 @ 13.2s   -1.4 @ 43.2s   -2.1 @ 73.2s          11.0            2.3                2.3
   30 ottawa2019fall    1 base              -1.3 @  5.3s   -0.3 @ 15.6s   +0.6 @ 45.6s   +1.2 @ 75.6s          10.2            2.9                2.9
   30 ottawa2019fall    1 py+(b)            -0.5 @  2.4s   +0.6 @ 14.5s   -1.8 @ 44.5s   -1.2 @ 74.5s          12.1            1.9                1.9
   30 ottawa2019fall    2 base              -1.1 @  4.5s   +0.2 @ 15.2s   +1.3 @ 45.2s   +2.9 @ 75.2s          10.7            2.9                2.9
   30 ottawa2019fall    2 py+(b)            -0.7 @  3.2s   -0.6 @ 14.6s   -0.2 @ 44.6s   +0.5 @ 74.6s          11.5            2.5                2.5
   60 ottawa2019fall    1 base              -1.2 @  4.7s   -0.6 @ 21.2s   -0.1 @ 51.2s   +0.4 @ 81.2s          16.4            1.7                2.0
   60 ottawa2019fall    1 py+(b)            -0.5 @  2.5s   -7.1 @ 21.0s   -3.2 @ 51.0s   -2.6 @ 81.0s          18.5            7.3                7.3
   60 ottawa2019fall    2 base              -1.1 @  4.4s   -0.1 @ 21.1s   +1.8 @ 51.1s   +1.8 @ 81.1s          16.7            2.1                2.1
   60 ottawa2019fall    2 py+(b)            -0.3 @  1.9s   -5.3 @ 20.9s   +1.6 @ 50.9s   +1.2 @ 80.9s          19.0            5.4                6.4
   60 ottawa2019fall    3 base              -1.2 @  4.7s   +0.6 @ 21.1s   +0.4 @ 51.1s   -0.4 @ 81.1s          16.4            5.3                5.3
   60 ottawa2019fall    3 py+(b)            -0.3 @  1.8s   -3.4 @ 20.9s   -1.1 @ 50.9s   -2.8 @ 80.9s          19.0            4.3                4.3
Answer (simulation, ArduCopter SITL, flights of at most 82 s; 45 runs in the table):
- With time in the air: no growth seen. With py + (b) at 5-30 m the error moves by 0.0-2.4 deg between the switch and
  +30 s (4.0 deg in the hover-15 seed 2) and then stays: from +30 s to +60 s it changes by -1.4 to +1.6 deg, in both
  directions (mean |change| 0.7 deg; base 0.5 deg). Largest |error| in the 60 s hold: py + (b) 1.4-4.1 deg at 5-30 m, base 1.1-4.4 deg.
  NOT tested: flights longer than 82 s (the SITL hold is 60 s), so growth over minutes is unknown.
- 5 m with the 15 s hover (round 6): s1 -0.6 deg at the switch, -3.8 at +30 s, -2.4 at +60 s; s2 -0.8, -1.5, -2.6.
- With height: yes, in the climb. At 60 m py + (b) reached -7.1 and -5.3 deg at the switch (21 s of flight, after a
  climb of about 15 s with fixes), base -0.6 and -0.1 deg; in the hold it came back to -2.6 and +1.2 deg at +60 s. At
  5-10 m (switch after 10-11 s) and 30 m (15 s) the error at the switch is -2.4 .. +0.6 deg for both versions.
- With the time from the first sent fix to the switch: not by itself. py + (b) + 15 s hover at 10 m had 19.7 and
  20.1 s from the first fix to the switch (as long as at 60 m) and -0.5 / -0.3 deg at the switch; but seed 2 then went
  to +3.7 deg in the first 30 s of the hold and stayed (+3.8 at +60 s). So a long time with fixes before the switch at
  low height did not bend the yaw; the long climb with fixes did (3 of 3 seeds at 60 m: -7.1, -5.3, -3.4 deg).
- For a pilot time limit: the data give none (no growth within 60 s of hold). A height limit for low_height fits the
  data better than a time limit.

8. Lines for the event documents, profile, recommendation
---------------------------------------------------------
Profile tools/profiles/low_height.json (py + (b)): att_source gyro_fcyaw; hold_cfg dense_local true, dense_turn true,
dense_nadir_deg 0, kf_map false; a note per value with its evidence file, the height limit, and "never dense_turn
alone". Checked: tools/use_profile.py --settings <temporary copy of settings.json> apply justice_d outdoor_day
low_height -> settings.py loads it (att_source gyro_fcyaw, hold_cfg merged with repeat_diff 0.01 and h_min 2),
apb_main.att_filter gives GyroAtt with fc_yaw True, Hold accepts the keys. The repository's settings.json was not changed.

Recommendation (simulation only): apply low_height for a hold at 5-10 m. Evidence: 17 base / py + (b) pairs at 5 and 10 m
(TUNING and VALIDATION, section 7 FINAL SUMMARY): lower RMS in 16 of 17, no sent fix more than 0.5 m wrong in any of 21 runs,
FC yaw change in 30 s 0.7-2.6 deg in the 17 py + (b) runs, 3.8 and 4.4 deg in 2 of the 4 hover-15 runs. Above 10 m it
gives nothing: 20 m (1 seed) 0.38 -> 0.44 m, 30 m (2 seeds) 0.45/0.40 -> 0.42/0.40 m, and at 60 m it is worse in 2 of
3 seeds, with the FC yaw bent by 3-7 deg in the climb in 3 of 3. So: 5-10 m only. Keep kf_map off. The 15 s hover helps the base code (10 m 0.79/0.71 -> 0.51/0.41 m) and is harmless.

The lead wrote this study's numbers into DECISION_CARD row 47, EVENT_DAY (step 1 item 1, step 7 item 8, the profile
list), EIGHT_HOURS, EVIDENCE.md area B and DATA_SPLITS.md (04:50 rows). Lines that the later runs (04:52-06:45) change:
- '60 m: not better (0.49 -> 0.59 m, 1 seed)' (DECISION_CARD row 47, EVIDENCE.md): now 3 seeds: worse in 2, better in
  1. 30 s RMS 0.49/0.41/0.47 -> 0.59/0.44/0.39 m; sent fixes more than 0.5 m wrong 2/0/12 -> 9/40/0 (60 s: 32/5/13 ->
  127/397/0); the FC yaw error at the switch -7.1/-5.3/-3.4 deg against -0.6/-0.1/+0.6 deg (bent during the climb in
  3 of 3).
- '10 m, another site: 0.74 / 0.65 -> 0.34 / 0.27 m': now ottawa2022 seeds 11-13: 0.74/0.65/0.94 -> 0.34/0.27/0.50 m;
  farmland seeds 11-13: 0.32/0.33/0.55 -> 0.33/0.25/0.46 m (seed 11 no better); 5 m ottawa2022 seeds 11-13:
  0.51/0.57/0.76 -> 0.33/0.28/0.47 m; 5 m farmland seeds 11-12: 0.55/0.41 -> 0.32/0.27 m.
- 'With low_height it [the 15 s hover] adds nothing more, and it does no harm': true for RMS (0.42/0.35 against
  0.42/0.33 m), but in seed 2 the FC yaw error went from -0.3 to +3.8 deg and stayed there (60 s).
- 'Use low_height for holds up to 30 m' (DECISION_CARD row 47, EVENT_DAY step 1 and line 464, EIGHT_HOURS line 40,
  EVIDENCE.md 'So the profile is for holds up to 30 m'): above 10 m it gives no gain: 20 m s1 0.38 -> 0.44 m RMS (30 s),
  30 m s1/s2 0.45/0.40 -> 0.42/0.40 m (60 s: 0.42/0.44 -> 0.41/0.47 m), 60 m worse. Suggested: 'for holds at 5-10 m;
  above 10 m apply the profiles without it'.
- EVIDENCE.md 'tests/bad_pictures.py gives the same file with the options on': true; add test_live with py + (b): the
  known overlay FAIL with 22 fixes (21 off), freeze noise 11 of 72 frames sent, last fix 0.84 s after the freeze (limit
  1.0 s; off 7 of 72, 0.50 s).
- DATA_SPLITS.md row 04:50 'closed loop VALIDATION ottawa2022 seeds 11-12': now ottawa2022 seeds 11-13 and farmland
  11-13 at 10 m, ottawa2022 11-13 and farmland 11-12 at 5 m (section 7); 20 m 1, 30 m 2 and 60 m 3 TUNING seeds.

9. Not done (kept up to date)
-----------------------------
- KF_MAP: stopped by the lead at 03:36. No validation matrix, no false-acceptance test (scripts ready: jobs.py sets
  place, bad, rep; tables.py), no closed-loop run, no timing at one thread. Its false-relocalization rate is NOT
  measured. It must stay off (low_height.json: kf_map false).
- DENSE_NADIR_DEG: no closed-loop run (the lead chose py over pyn55 from the diag10 validation replays);
  pyn55 was not run in SITL.
- py alone (DENSE_TURN without att_source gyro_fcyaw) was run only at 10 m (2 seeds); the lead's interlock refuses it.
- (b) alone and base + hover 15 s: 5 and 10 m, 2 seeds each (TUNING only). Flight version (py + (b) + hover 15 s):
  5 and 10 m, 2 TUNING seeds each; not on a VALIDATION site.
- No PX4 SITL run of any option. No real camera footage. No board timing: the SITL board model is a fixed 80 ms per
  Hold.process, so the extra compute of DENSE_TURN (about +0.5-0.9 ms per frame on the laptop, single thread, proof
  replays) is not in the SITL.
- Flights longer than 82 s (the SITL hold is 60 s): growth of the FC yaw error over minutes is not tested.
- The cause of the 60 m result (FC yaw bent in the climb with py + (b)) is not analysed; one inference in section 7.
- One wind case (w20g40), one SITL vehicle, ArduCopter only; 2-3 seeds per cell.
- The open-loop validation matrix of this study (integ/kfmap/jobs.py set main) was not run; diag10 has the 10 m
  open-loop validation of py.
- tests/bad_pictures.py has no attitude filter, so att_source does not apply to it.
- Documents (EVENT_DAY, DECISION_CARD, PRESENTATION, DATA_SPLITS, EVIDENCE, EIGHT_HOURS) were not edited by this study
  (the lead edits them); the lines to change are in section 8.
- Files of this study in the repository: hold.py (the four options), apsim/sitl_ap.py (case key hover_s, one line),
  tools/profiles/low_height.json, integ/kfmap/ (scripts, results, closed/). apb_main.py, settings.py, settings.json and
  tests_apb/ are unchanged by this study (the one apb_main line of 03:05 was taken out at 03:16). The scratchpad copies
  (now, nowpy, final, closed/ab; nowdl was deleted after use) hold the SITL raw files; the ones used here are copied to
  integ/kfmap/closed/runs/ (onboard logs gzip, truth .npz, results).

POWER NOTE (added by the coordinating session 0a at 06:50 EST, 2026-11-21): the laptop ran on battery from at least 04:33 EST (upower history; the first 'discharging' row found), and session 1f saw CPU power-save mode at 06:49 with 9 % battery. Runs after about 04:33 (most of the SITL A/B, the regression and the suites) ran on battery power; power-save may have started earlier than 06:49. Timing numbers (E9/H6, process() ms, test time limits) from that period can be slower than on mains power. The SITL board model uses a fixed 80 ms per Hold.process, but the SITL itself ran slower; each run's start load is in its table. The off/on comparisons ran under the same conditions for both versions.
