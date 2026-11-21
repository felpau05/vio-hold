# Evidence map for the judges (Stage 1)

**Read first**
- Nothing has run on the real aircraft or the real APB. Results come from simulation, synthetic pictures, unit tests, or one real analog clip (local only).
- Types: UT unit test, SYN synthetic pictures, OL open-loop, CL closed-loop SITL, AN benchmark. Sites: `ottawa2019fall` tuning, `ottawa2022` validation (`DATA_SPLITS.md`).
- Not in git yet: every file of 18, 19 and 20 November that this document cites (`git status` lists them), for example `integ/justice_d/`, `integ/gap/`, `betasim/`, `msp_link.py`, the new `tests_apb/test_*.py`, `tools/lens_calib.py`, `tools/log_summary.py`, the new profiles, `apsim/drag_results.txt`, `apsim/engage_results.txt`, `px4sim/low_outage_results.txt`. Add them to git.
- Git ignores `logs/`, `runs/`, `*.npz` and `tests_apb/out_*.txt`. Copies of the ignored files that this document cites are in `evidence/`, with the same relative path (for example `evidence/tests_apb/out_test_run.txt`). Copy event-day evidence there too.
- Laptop rehearsal of the event-day documents, 20 November, with the Justice D path in ArduCopter SITL (one flight per height, seed 31): `integ/rehearsal_2026-11-20/README.txt`. Its findings are used in areas A, B and D. The corrections of `apb_main.py` that came from it, and their independent review: `integ/integration_2026-11-20/README.txt`, `integ/integration_2026-11-20/review/REVIEW.txt`.
- Offline bundle for the board (Python and libraries for 64-bit ARM, no network): `tools/bundle/README.txt`, proof in `tools/bundle/proof/`. Used in area C. It ran on emulated ARM only.
- Area F is in `CONTRIBUTION.md`: third-party software and licences, our files, experiments and decisions, and a table for changes during the event.
- Figures for the talk are in `talk/`. `talk/captions.txt` gives the source files, runs and seeds of each.

| Area | Status | Main proof | Main gap |
|---|---|---|---|
| A Sensor data, calibration | partial | `integ/justice_d/calib_loop_d_20261118_230048.txt`, `integ/rehearsal_2026-11-20/README.txt` (B) | No real camera data |
| B Estimation | partial | `integ/justice_d/summary.txt` (rows `jdcal*`) | No flight data |
| C Onboard execution | partial (timing missing) | `tools/bundle/proof/` (program runs on emulated ARM), `integ/justice_d/compute_20261118_230038.json` | No timing on the APB. Emulated time is not a speed |
| D FC integration, handover | partial | `integ/gap/results/closed_summary.txt`, `integ/integration_2026-11-20/README.txt` | No real FC. Switch positions unknown |
| E Reliability | partial | `integ/gap/results/test_live_revised.txt` | Overlay defect open |
| F Engineering contribution | partial | `CONTRIBUTION.md`, `tests/regression.txt` | Event-day change table is empty |
| G Live hold | missing | `px4sim/low_outage_results.txt` (SITL only) | Needs the aircraft |

## A. Sensor data and calibration
**Claim now**
- The digital path reads the Y plane of YUYV, UYVY, NV12 and GREY buffers and stops on a wrong buffer shape. `capture.py` (`luma`), `tests_apb/test_digital.py`, `tests_apb/test_robust.py` (D6). UT. Output: `evidence/tests_apb/out_test_digital.txt`, `evidence/tests_apb/out_test_robust.txt`.
- Lens check, rehearsal: hand-turn calibration from each of two start lenses ("full sensor" 152 x 85.5 degrees, "anamorphic" 114 x 85.5 degrees) against two true lenses, 30 Hz, seed 5, one run each. All four fits found fx within 0.37 % and fy within 0.35 %, residual 0.054 to 0.060 degrees, delay 58 ms, readout +11 ms. At 10 Hz (the `no_sd` rate) the two runs were as good. No fit stopped at its range edge. The residuals differ by 0.002 degrees, which one run cannot separate. So the second calibration is not shown to help. SYN. `integ/rehearsal_2026-11-20/README.txt` section B.
- Hand-turn calibration, no target, one run: delay 58 ms (true 58), readout 11 ms (true 10), fx +0.34 %, fy +0.23 %. Ray error: median 0.211°, max 0.426°. `integ/justice_d/calib_loop_d_20261118_230048.txt` and its `_rays_` file. SYN.
- The lens test renders with its own lens equations: fx +0.32 %, fy +0.26 % (limit 1 %) from three start profiles. `tests_apb/test_lens_d.py`. UT. Output: `evidence/tests_apb/out_test_lens_d.txt`.

**Cannot claim**
- No real Justice D picture, IMU log or timing exists. The lens profiles are assumptions.
- The `timing` command (video delay and readout from a stick wiggle) was corrected after the rehearsal and an independent review (`integ/integration_2026-11-20/README.txt`, `review/REVIEW.txt`). It now uses only the 15 s before the first switch that follows a clean log. The first correction had two blockers: it took the first "engaged" note even when it was a ground note, and it did not check that the window held a wiggle. Both are corrected. On the four rehearsal flights (one each, seed 31, true 40 ms and +14 ms): 10 m 40 ms / +15, 30 m FAIL (nothing written), 60 m 38 ms / +23, 100 m 36 ms / +11 (`integ/integration_2026-11-20/timing_with_run_log.txt`). 126 test cases: 13 PASS, none with a wrong value (`integ/integration_2026-11-20/timing_cases_v2.txt`).
- LIMIT of the wiggle check: the threshold `WIGGLE_RATIO` 4 was set on the same four flights that test it. The test accepts any value from 2.6 to 5.7. Flown windows that pass gave 5.3, 7.6 and 8.7, wrong-value windows 1.0 to 1.8, and the 30 m flight 4.4. Its safe direction is refusal: a good wiggle can be refused. One flight per height, so these are examples, not rates. `integ/integration_2026-11-20/README.txt` (B2, S5).
- The mount check failed: 0.86° against 0.5°. The fit does not find the lens centre (7 px and 8 px off).
- The start lens "Justice D full sensor (assumed)" has 152 x 85.5 degrees. The Justice D datasheet gives 114 x 86 degrees for its 1280 x 720, 16:9 picture (datasheet p.1, Stage 1 p.5 Q3). No document says that the 114 degrees belong to a 4:3 picture. `justice_lens_profiles.json`, `review/manuals_audit_2026-11-20.md` Table 1 row 9. The calibration refits the lens, but a fit can end at the edge of its search range.
- The pixel format of the digital path is not stated in any organizer document. YUYV is an assumption. The reader also takes UYVY, NV12, GREY and 3-channel BGR. It cannot read Bayer, MJPG or 10-bit formats.
- The lens fit with a checkerboard (`tools/lens_calib.py`) is a development aid. Stage 1 (p.4, p.6 Q10) says such aids may not be used in the scored evaluation. Do not count a lens value from it in the scored settings. The scored path is the hand-turn calibration.
- The laptop solves the calibration and tuning, and the values go to the board. Stage 1 does not say whether laptop-solved numbers are allowed. `CONTRIBUTION.md` Part 2 lists which tool runs where.
- Our heading check passes when the heading is within 10 degrees of north. With GPS off, the aircraft manual says the heading may be the power-up direction (aircraft manual PDF p.22). That check can pass or fail for the wrong reason.
- The board's own IMU (ICM42605, APB manual PDF p.6) is not read. The estimator reads the flight controller's IMU over MAVLink. No log has board IMU samples beside the frame stamps.
- Not found: a record that `test_digital.py` caught four planted defects, and a lens case with half the view as sky.
- `tools/replay_tune.py` is not safe below 100 m. Laptop rehearsal, one PX4 recording per height, delay set 30 ms wrong (`integ/rehearsal_2026-11-19/`). At 100 m it moved the delay to 6 ms from the truth. In the 20 November rehearsal (Justice D, delay 30 ms too long, one recording per height, one run each) it wrote a focal scale 8 % off at 60 m and the RMS error went from 0.312 to 0.765 m. On a flown 100 m recording its delay was 12 ms short. `integ/rehearsal_2026-11-20/README.txt` sections 2 and 4. Merge a tune only from 100 m or higher. At 30 m it would write delay 0.1499 s (true 0.0604), readout 0 (true 0.0184) and focal scale 1.0816 (true 1.0). At 10 m it would write focal scale 1.0383. It scores values by fix jitter, and at these heights a wrong value gives less jitter. The rehearsal used the analog camera model.

**Event-day proof**
1. Type `python3 tools/use_profile.py apply <profiles> justice_d`. This is required: the shipped `settings.json` is the analog default (camera analog, `/dev/video0`). The manual gives the Justice D on CSI 1 as `/dev/video2` (APB manual PDF p.13).
2. Check the board clock (the manual says it is wrong after a first start with only USB, APB manual PDF p.11). Type `date` and set it if it is wrong. Run names use the clock.
3. Type `python3 apb_main.py check | tee check_1.txt`. Record "video rate", "frame stamps" and "picture for Hold".
4. Type `python3 apb_main.py mask`. Record "mask covers".
5. Type `python3 apb_main.py calib`. Turn the drone by hand toward a scene more than 50 m away.
6. Record every PASS, FAIL and INFO line.
7. Do step 5 again. Record "from the previous calibration". Then do it once more from the other start lens (`lens_name` in `justice_lens_profiles.json`: the "full sensor" or the "anamorphic" profile). Keep the fit with the lower residual.
8. After a flight with the 8 s stick wiggle before the switch, type `python3 apb_main.py timing <log_dir>/run_<stamp>.csv <log_dir>/raw_<stamp>_*.npz`. The run log is required. Record the PASS line, or the FAIL reason. On a FAIL keep the ground calibration values, or fly the wiggle again. `integ/integration_2026-11-20/README.txt` (D1, B1, B2).

**Gap:** real Justice D data and timing. Needs the camera and the APB, not flight.

## B. Visual-inertial estimation
**Claim now**
- Full chain (calibrate, then hold), Justice D, 100 m, wind 20 gusting 40 km/h: 0.661, 1.615, 0.783 m RMS (`integ/justice_d/summary.txt:317-319`). After the liveness fix: 0.564 m (`integ/gap/results/closed_summary.txt`). CL, tuning site.
- True calibration, open loop, validation site: RMS median 0.114, 0.241 and 0.478 m at 20, 50 and 100 m. `integ/justice_d/summary.txt:9,21,33`. OL, a best case.
- A video delay 30 ms too long (70 ms against 40 ms), ArduCopter SITL, Justice D, wind 20 gusting 40 km/h, wiggle then 60 s hold, one flight per height, seed 31: hold RMS 0.61 m at 10 m, 0.73 m at 30 m, 1.12 m at 60 m, 9.85 m at 100 m (largest 13.73 m). One flight each, so the jump at 100 m is an example, not a rate. CL. `integ/rehearsal_2026-11-20/README.txt` section 2.B, 4 (R2).
- Scale: the bench test reads a 1 m move within 0.1 m. `tests_apb/test_bench.py`. SYN.
- Two options, off by default, unit test only, not shown in closed loop:
  - `fresh_kf_at_engage` (profile `fresh_kf.json`): a new home keyframe at the pilot's switch, no re-anchor. `hold.py` (`Hold.request_keyframe`), `tests_apb/test_event.py` E5: 0.003 m move in a still hover (`evidence/tests_apb/out_test_event.txt`). ArduCopter SITL A/B (`apsim/engage_results.txt`, 30 to 100 m, 1 to 2 seeds): no difference beyond the spread of identical runs (up to 0.25 m RMS). Keep it off. Public source of the idea: Parrot, ANAFI Ai white paper v7.4.0.0, section "Indoor flight", pp. 11–12 (a reference image taken when the drone starts a hover).
  - `kf_bright` (profile `kf_bright.json`): no keyframe change while the picture brightness changes. `hold.py` (`KF_BRIGHT`), `tests_apb/test_event.py` E6. The team records the idea source before the presentation.

**Cannot claim**
- No flight data exists. The same settings gave 1.615 and 0.783 m, so one run proves little.
- The open-loop and `jd_apb_*` rows used the true mount, delay and readout, and a lens within 0.5 %. They do not compare the Justice D with the analog camera.
- Drift, scale and rolling-shutter errors are not measured on real data.

**Event-day proof**
1. Remove the propellers.
2. Type `python3 apb_main.py bench --height 1.5`.
3. Carry the drone 1.00 m toward the nose. Record the "forward" line.
4. Carry the drone 1.00 m to the right. Record the "right" line.
5. Before each flight, type `python3 apb_main.py run --raw`.
6. After landing, type `BOARD=<address> tools/deploy.sh pull` on the laptop.
7. Type `~/.venvs/vio/bin/python tools/plot_log.py logs/run_<stamp>.csv`. Keep the PNG.

**Gap:** hover error against an outside reference. Needs the aircraft.

## C. Onboard execution
**Claim now**
- Hold.process, one laptop core (Intel Core 7 240H), two runs: median 3.22 and 3.13 ms at 640x360, 4.99 and 4.57 ms at 1280x720. Y plane and reduction: 0.28 ms. `integ/justice_d/compute_20261118_225729.json`, `compute_20261118_230038.json`. AN.
- Justice D SITL with an assumed board time of 80 ms per frame: 12.1 to 12.2 fixes/s, fix age 129 to 157 ms. `integ/justice_d/summary.txt:312-320`. CL.

**Cannot claim**
- Nothing has run on the APB. The simulators assume the board speed: `cpu_factor` 10 (`px4sim/sitl.py`), `board_s` 0.08 s (`apsim/sitl_ap.py`).
- The offline bundle (CPython 3.12.16, numpy 2.5.5, OpenCV 5.0.0.93, pymavlink 2.4.50, pyserial, 64-bit ARM, glibc 2.27 or newer) runs on emulated 64-bit ARM: qemu-user 8.2, CPU model cortex-a53 (the APB's core), no network, roots Debian 11 and 13. Imports, `apb_main.py check`, `delay`, `replay`, `timing`, `Hold.process` and these test groups pass: `test_digital`, `test_replay_chunks`, `test_wind_assist`, `test_event`, `test_lens_store`, `test_calib`, `test_lens_d`, `timing_test`. Hold, replay and timing give the same numbers as on x86_64 (`tools/bundle/proof/native_vs_emulated.txt`). The program needs the bundle only if the board has no usable Python (case B, `sh tools/board_check.sh` prints the case). AN, emulation. `tools/bundle/README.txt`, `tools/bundle/proof/`.
- Emulated time is not the board's speed. The proof files give pass or fail only. The timing claim for the APB stays missing. Emulation cannot decide 10 FAIL lines that depend on real time: `test_basics` 1 (the V4L2 query), `test_robust` 1 (D5 (a) needs 30 fixes in 1 s), `test_fc_link` 1 (clock mapping error 3.6 to 3.9 ms, limit 3 ms), `test_run` 5 and `test_bench` 3 (pictures fed in real time). Not run: `test_ap_sitl`, `test_live`. With the default qemu CPU model the tracking was wrong ("fit out of range"); the cause was not found. Nothing about the real board is proven: its Python, C library, free space, file systems, camera, serial route. Files: `tools/bundle/proof/deb11_test_*.txt`.
- The Python software on the board is not known. Stage 1 (p.5 Q1) says the OEM specifications are not a list of installed packages. The program needs Python and NumPy and OpenCV. `tools/board_check.sh` stops with FAIL if there is no Python. The bundle is the fallback.
- The board has 4 GB of flash (Stage 1 p.5 Q1). At the `justice_d` settings the "free disk" check needs 5.3 GB, so it fails without an SD card. Stage 1 does not say whether a team SD card is allowed (p.6 Q8, Q9). The profile `tools/profiles/no_sd.json` is the fallback. `review/manuals_audit_2026-11-20.md` Table 1 row 1.
- The run log has no board temperature. The board range is 0 to 70 °C and it shuts down at 95 °C (APB manual PDF p.4, p.12). `tools/board_check.sh` fails at 85 °C. No flight or bench run measured the temperature under load.

**Event-day proof**
1. Type `sh tools/board_check.sh` on the board. Record the "CPU" line and each FAIL line.
2. Type `python3 apb_main.py check` three times. Record each "Hold.process median" line.
3. Keep the run log of each flight. Columns `proc_ms` and `age_ms` give frame time and fix age.
4. Record the "free disk" line of `check`. If it fails, type `python3 tools/use_profile.py apply <profiles> justice_d no_sd`. Write down which profile ran.
5. Before and after each flight, type `cat /sys/class/thermal/thermal_zone0/temp` (millidegrees C). Write down both values.

**Gap:** timing and CPU load on the APB. Needs the APB only. Do it first.

## D. Flight-controller integration and pilot handover
**Claim now**
- The program sends VISION_POSITION_ESTIMATE with a covariance. `check` reads and sets the FC parameters. `fc_link.py`, `tests_apb/test_fc_link.py`. UT.
- Ground tools corrected after the rehearsal, each with a test (`tests_apb/test_ground_tools.py`, 32 checks; all pass, and the old `apb_main.py` fails the first version of the test 13 times): `check` on ArduPilot now measures the height stream `GLOBAL_POSITION_INT` ("PASS GLOBAL_POSITION_INT 10 Hz", ArduCopter SITL, GPS off, one run; before, `check` showed a FAIL on every ground check); `bench`, `mask`, `calib`, `check` and `timing` print one FAIL line, not a traceback; the free-disk check never needs less than the recorder's stop level; `timing` as in area A. The flight loop did not change: 56 top-level definitions, among them `run_live`, `Pipe`, `Engage`, `Raw`, are identical by syntax tree. `integ/integration_2026-11-20/README.txt`, `flight_loop_unchanged_v2.txt`, `defect_check_lpos_ardupilot.txt`, `run_all_v2.txt`. An independent review graded two items BLOCKER and both are corrected (`review/REVIEW.txt`). UT, one ArduCopter SITL run.
- The program does not change the flight mode (default settings). The program re-anchors to the FC position at start and after a loss. `apb_main.py` (`Pipe`).
- The program reads the FC mode and logs the pilot's switch: column `mode`, notes "engaged: FC mode <name>" and "released: FC mode <name>". `apb_main.py` (`fc_mode`, `Engage`), `tests_apb/test_event.py`. UT.
- Pilot exit, added 2026-11-21: the simulated pilot left the hold after 10 s. ArduCopter 4.6 SITL, 10 m and 30 m, wind 20 gusting 40 km/h, one run each. The mode changed to AltHold in 0.01 s and the height stayed within 0.05 m. With centred sticks the aircraft then drifted 36 m in 10 s, because AltHold holds no position. A takeover to Stabilize also worked. Acro was not flown. `apsim/safety_results.txt` (cases a and e), `apsim/safety_matrix.py`. CL.
- What the program can do to the aircraft, read in our code and in the ArduCopter source: with the event settings it never arms, never selects a flight mode, never sends a flight target and never overrides the sticks. It sets the EKF origin and the home point at start. `integ/pilot_safety/README.txt` section 1. The page for the pilot is `PILOT_SAFETY.txt`.
- After the liveness fix, the gap before the switch fell from 0.33–4.50 s to 0.10–0.13 s. The EKF error at the switch fell from 0.28–1.39 m to 0.01–0.51 m. `integ/gap/results/closed_summary.txt`. CL, 4 runs each.

**Cannot claim**
- No real FC received our messages yet. Disengagement, an early switch and a program stop are tested in the simulator only (2026-11-21, `apsim/safety_results.txt`). On the aircraft: the bench steps B0 to B7 of `integ/pilot_safety/README.txt` section 6 and items 5a and 10a of EVENT_DAY step 7.
- Arming, found 2026-11-21: with our parameter VISO_TYPE 1 and the default arming checks, ArduCopter 4.6 SITL refused to arm in 4 of 4 tries ("Arm: VisOdom: not healthy"), because the program sends no position on the ground. With only the vision arming check off (ARMING_CHECK 1835006) it armed, climbed and held. Every earlier simulator run had the arming checks off. The crew must make this change on the aircraft. `apsim/safety_results.txt` (runs `arm_none_s2`, `arm_gnss_s1`).
- We write flight controller parameters on an aircraft that is not ours. `tools/fc_params.py` saves the aircraft's own values first and writes them back at the end (EVENT_DAY step 1 item 2b, step 8). On an ArduCopter 4.6 SITL flight controller it saved the 8 factory values, `check --set-params` changed 6 of them, and the tool wrote them back; the values after a restart equal the saved values (`integ/integration_2026-11-21/fc_params_sitl.txt`, one run). It has not run on a real flight controller.
- Which switch position gives Loiter or Position mode is not known. The aircraft manual gives the flight-mode switch TL1: up Acro, middle Position Hold, down Return to Home. It names no Altitude position. Its Position Hold uses GPS, a magnetometer and a barometer (aircraft manual PDF p.14, p.19). Stage 1 (p.2) forbids GNSS in the scored hold. Our checklist expects an Altitude position and a Position position. `review/manuals_audit_2026-11-20.md` Table 1 row 4.
- No organizer document names the firmware on the aircraft. The APB manual, the aircraft datasheet and Stage 1 (p.5 Q2) list ArduPilot, PX4, Betaflight and iNav. The MAVLink claims here hold for ArduPilot and PX4 only. For Betaflight see "Bonus routes".
- The computer link to the flight controller is UART3 (`/dev/ttymxc2`). It reaches the flight controller only after two `gpioset` route commands (APB manual PDF p.15). No script runs them. The default `fc_device` is `udpin:127.0.0.1:14540`, which needs `mavlink-router`. The documents state neither a baud rate nor `mavlink-router`.
- Our ArduPilot parameter set does not set the failsafe actions. From the ArduCopter source (reading, not a test): after about 7 s with no fix the EKF failsafe acts, default Land (`FS_EKF_ACTION`). With drag fusion on, the same dropout starts dead reckoning, and `FS_DR_ENABLE` acts at once, default RTL. Sources: `ArduCopter/Parameters.cpp`, `ArduCopter/events.cpp` (`failsafe_deadreckon_check`), `libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp` lines 372–400 and 797, github.com/orqafpv/ardupilot commit 3b40393. The event crew must choose these actions.

**Event-day proof**
1. Remove the propellers. The propellers stay off for any step with a USB cable. Fit them only with the battery disconnected (aircraft manual PDF p.4). Start the program with the cable on and the propellers off. Then unplug the cable and fit the propellers.
2. Type `python3 apb_main.py check --set-params`. Then reboot the flight controller. If there is no heartbeat, set `fc_device` to `/dev/ttymxc2`, run the two route commands of EIGHT_HOURS.txt lines 23-26, and run `check` again.
3. Type `python3 apb_main.py check`. Record every "param" line.
4. Before take-off, ask the pilot to move the flight-mode switch through each position. Record the FC mode name of each (the run log column `mode`). If a position gives GPS Position Hold or Return to Home, do not use it for the hold.
5. Type `python3 apb_main.py run --raw`.
6. The pilot engages and then disengages the hold. Do this three times.
7. Record each switch time. Save the flight-controller log and the run log (columns `sent`, `note`, `mode`).
8. On the laptop, type `~/.venvs/vio/bin/python tools/log_summary.py logs/run_<stamp>.csv`. Record the "switch" column: fixes in the 2 s before and after each switch.
9. ArduPilot only: type `python3 apb_main.py delay logs/run_<stamp>.csv`. Record the VISO_DELAY_MS line.

**Gap:** real engage and disengage transitions. Needs the aircraft.

## E. Reliability and reproducibility
**Claim now**
- Frozen picture: the last fix came 0.00 s (repeated) and 0.50 s (fresh noise) after the freeze, limit 1.0 s. `integ/gap/results/test_live_revised.txt`. SYN.
- Static, frozen and NO SIGNAL pictures: 0 of 336 fixes sent, 3 seeds. `tests/bad_pictures.txt`. SYN, analog.
- Attitude stop: last fix 0.03 s later. Restart: first fix 0.22 m from the truth. `tests_apb/test_run.py`, `tests_apb/test_robust.py`. UT. Output: `evidence/tests_apb/out_test_run.txt`, `evidence/tests_apb/out_test_robust.txt`.
- Fault behaviour of the flight controller, added 2026-11-21. ArduCopter 4.6 SITL, 10 m and 30 m, wind 20 gusting 40 km/h, default failsafe parameters, one run for each case. `apsim/safety_results.txt`. CL.
  - The program stops in a hold: 7.2 and 7.9 s later the FC showed "EKF variance" and "EKF Failsafe" and changed to Land. The aircraft was 0.7 and 1.7 m from where it was at the stop. AltHold worked after it.
  - A position jump of 20 m: the FC refused it at once ("EKF3 lane switch") and the aircraft did not move. After 7.0 s the EKF took the jumped position, still with no movement and no failsafe.
- Hold-out sites, analog SITL, 30 m: 0.305 to 0.748 m RMS. `apsim/results.txt` (`c_hoD_30m_*`). CL.

**Cannot claim**
- Overlay-only picture: 21 fixes in 288 frames, limit 0. This defect is open.
- A slow, wrong position is not detected by the flight controller. We sent a position that ramped away at 1 m/s: no FC text and no mode change. The aircraft followed. It was 7.1 and 8.4 m away when the simulated pilot took over, 3 s after it passed 3 m. The wrong position also bent the FC's attitude estimate by up to 2.4 degrees of tilt. Only the pilot can stop this case. `apsim/safety_results.txt` (case c), 2026-11-21. CL, one run for each height.
- An attitude stream with gaps: the program stops sending after 50 ms with no attitude and needs 10 good frames again, about 0.8 s. The profile `att_gap` raises the limit to 120 ms. It is not tested in closed loop. `tools/profiles/att_gap.json`, DECISION_CARD row 44.
- Last full test run, 21 November 01:26–01:35, load about 1 to 3: 16 of 17 groups without a FAIL. `test_live` 1 FAIL (the overlay case), `test_lens_d` 1 KNOWN LIMIT. `test_gnss` is new and passes. `test_lens_store` failed in that run because the deploy file list had a new file that the test's copy did not have; with the list corrected it passes (run alone, 2 s). `integ/integration_2026-11-21/run_all.txt`.
- Hold-out lawn at 100 m: 5.81 m RMS (`c_hoD_100m_holdout_lawn_s104`).
- A long video outage: with a 20 s outage at 100 m the drone drifted 30 m from the hold point, with and without ArduPilot drag fusion (one seed each; `dr_gapX_off_s102`, `dr_gapX_on_s102`). Without drag fusion the EKF failsafe changed the mode to Land. With it, dead reckoning started at 30.3 s and the run ended in a fly-away at 39.1 s. Drag fusion did not improve normal holds either. `apsim/drag_results.txt`. The profile `tools/profiles/drag_fusion.json` is a record only, marked "NOT RECOMMENDED for the event".
- Hand-hovers with no vibration still have gaps up to 5.83 s before the switch (`integ/gap/results/hh_summary.txt`, code v2). With 0.2° vibration every case is 0.08 s.
- Frozen picture (fresh noise, 6 s) in a hand-hover, current fix v2 against the old code, 8 cases x 3 seeds: the last frozen fix came at the same time in every case, and the frozen fixes after 0.5 s changed by 0 to +2 (for example 17 / 17, 22 / 24, 111 / 111). The superseded v1 sent more (for example 39 against 22). `integ/gap/results/hh_summary.txt` (freeze rows), `integ/gap/run_freeze_v2.sh`. OL. So v2 keeps the old protection. It does not close the old hole below.
- Old hole, before any fix: in a still hover, a frozen picture with fresh noise gave fixes up to 5.96 s after the freeze (errors up to 0.52 m). `integ/gap/results/hh_summary.txt`, freeze rows "still". OL.
- Last full test run, 20 November 05:59–06:07, load 0.56 to 2.82: 15 of 16 groups without a FAIL. `test_live` 1 FAIL (the overlay case), `test_lens_d` 1 KNOWN LIMIT (fy +1.08 %). `integ/integration_2026-11-20/run_all_v2.txt`. The first run of that day (04:42) had one `test_run` FAIL, a race of the first frame against the first height row in the test. It passed in 6 of 6 repeats and in the final run (`test_run_repeat.txt`). Regression: 19 PASS, 2 INFO, as before (`integ/integration_2026-11-20/run_regression.txt`).
- Earlier test run, final code tree, idle computer (19 November 04:14–04:22, output in `evidence/tests_apb/`): 14 of 15 groups without a FAIL (`evidence/tests_apb/out_*.txt`). `test_live` 1 FAIL (the overlay case above). `test_lens_d` 1 KNOWN LIMIT (fy +1.08 %, limit 1 %). An earlier run at 03:24 saw `test_ap_sitl` fail once at load average 9. It passed alone and in both later runs.
- Dark, flat night pictures: the digital picture check refuses a picture with a texture spread under 2 grey, to stop a covered lens. It also refuses dark ground that Hold can track. A synthetic check found this. Its script is not in the repository. Not measured on the real camera.

**Event-day proof**
1. On the laptop, type `sh tests_apb/run_all.sh | tee integ/run_all_<date>.txt`. Commit that file.
2. In the bench test (B steps 1–2), cover the lens with a hand for 2 s.
3. Record that the "forward" lines stop and then start again.
4. Fly each height test three times. Keep each run log.

**Gap:** repeated real runs and real faults. Needs the aircraft.

## F. Engineering contribution
**Claim now**
- 21 review finding IDs have a before and after check: 19 PASS, 2 INFO, 0 FAIL. `tests/regression.txt`. UT.
- Third-party software with licences, our own files in groups, and one table of experiments and decisions. `CONTRIBUTION.md` Parts 1 to 3.
- Data splits and a tuning log with 31 entries. `DATA_SPLITS.md`.
- Tried and dropped, with numbers: speed messages, trust scale, wind assist version 1. `DATA_SPLITS.md` lines 52, 55 and 57. CL.

**Cannot claim**
- `tests/regression.txt` predates today's code changes.

**Proof steps**
1. Type `~/.venvs/vio/bin/python tests/run_regression.py`. Commit the new `tests/regression.txt`.
2. Fill `CONTRIBUTION.md` Part 4 as the changes happen.

**Gap:** Part 4 of `CONTRIBUTION.md` is empty until the event. No aircraft needed.

## G. Live position hold
**Claim now**
- No live hold exists. SITL only: Justice D, true calibration, 20 m, calm: 0.159 m RMS (`integ/justice_d/summary.txt:315`). CL.
- Low heights, PX4 SITL, Justice D, wind 20 gusting 40 km/h, 60 s: 5 m 0.54 and 0.51 m RMS, 10 m 0.75 and 0.65 m (2 seeds each). A perfect position gives 0.46 and 0.47 m (1 seed). Calm: 0.11 m at 5 m, 0.12 m at 10 m (1 seed). PX4 stayed in Position mode. `px4sim/low_outage_results.txt`. CL. So at low heights the autopilot's wind response causes most of the error.
- Same heights, ArduCopter SITL (`apsim/low_ap_results.txt`, tuning site, 2 seeds): 5 m 0.85 and 0.44 m RMS, 10 m 0.98 and 0.93 m. A perfect position gives 0.35 and 0.34 m (1 seed). Calm 5 m: 0.13 m. Loiter held in all 7 runs. But at 5 m the fix rate fell to 7.6 and 8.4 per s, with gaps up to 6.28 s and 14 to 16 restarts from the FC position, caused by "whole picture: halves disagree". So prefer 10 m to 5 m when the level allows it.
- Cause of those refusals, measured in open-loop replays of the same recorded flights (`integ/lowh/README.txt`, validation sites ottawa2022 and farmland, seeds 11–13). OL.
  - Main cause: camera vibration that the FC does not see (simulator 0.2°, an assumed value) moves the two picture halves 1–3 px differently. The check converts this gap at the picture centre's scale. At 5 m on the coarse photo the usable texture is near the fisheye edge, so 1–3 px read as 4–8 px. Still hover at 5 m: 6.5 px maximum with vibration, 0.16 px without. Limit: 4 px. `integ/lowh/results/shares.txt`.
  - Minor cause: the second-order error of the linear motion model. About 25 % of the refusals at 5 m on ottawa2022, none on farmland or at 10 m. `integ/lowh/results/shares.txt`.
  - The coarse photo is why the whole-picture path runs at all. On a fine synthetic texture the corner path gave every fix, with 0 refusals at 5–40 m.
- A proposed fix, not applied: `integ/lowh/proposed.patch` (the team decides). It measures the gap in pixels of each half's own texture and takes the motion map at the last fix. At 5 m on ottawa2022: refusals 114 → 15, fixes 8.5 → 10.8 per s, longest gap 2.75 → 1.42 s (median of 3 seeds, OL, `integ/lowh/results/before_after.txt`; one case re-run and matched). Costs: the fix error rose in most cases (5 m 0.10 → 0.11 m, farmland 20 m 0.21 → 0.35 m). More half-frozen pictures pass at 5 m (largest error 0.47 m). One `test_live` hover gap is 0.33 s, limit 0.3 s.
- The same patch in ArduCopter SITL (`integ/lowh/closed/README.txt`, tuning site, seeds 1–3, wind 20 gusting 40 km/h, 7 runs per version). CL.
  - 5 m: longest gap in the hold 3.34 → 1.38 s, fixes 8.1 → 10.9 per s, "halves disagree" 123 → 24, restarts 14 → 4 (medians). Every seed improved, and the ranges do not overlap.
  - Hold error: 5 m 0.48 → 0.43 m RMS, smaller than the seed spread. 10 m 1.01 → 0.85 m, lower in all 3 seeds, but n = 3. One earlier run of the same label differed by 0.4 m, so this is a lead only.
  - All 14 holds stayed in Loiter, with no EKF or failsafe text.
  - Costs: at 10 m, "points disagree" rose (31–87 → 100–108). In the calm 5 m run, "live" refusals in the hold rose from 4 to 42 (longest gap unchanged). The patched version sends its first fix during the climb. A 3.1–5.2 s gap then follows into the still hover in 5 of 7 runs. At the switch, the last fix was 0.55–1.29 s old in 6 of 7 runs (before: 0.03–0.33 s in all 7). ArduCopter accepted Loiter at once in all 14 runs.
- The outdoor profiles now allow a hold from 3 m (`min_height_m` 3, `hold_cfg` `h_min` 2). `tools/profiles/outdoor_day.json`.

**Cannot claim**
- Any flight result.
- Low-height limits: the SITL height is within 0.15 m, a real barometer is worse. The SITL ground photo is 9 to 19 times coarser than a camera pixel at 5 to 10 m. One 10 m run gave 8.0 fixes/s (under the 10/s floor) while the computer was loaded. No low-height run at night.
- The low-height cause above depends on the simulator's vibration model. The real Justice D vibration is not known.

**Event-day proof**
1. Before take-off, type `python3 apb_main.py run --raw`.
2. The pilot flies to the test point, holds still, and moves the hold switch.
3. Film the aircraft from the ground. Record the time of each switch.
4. After landing, make the plot as in B steps 6–7.
5. Repeat at the next approved height. Start at about 20 m.

**Gap:** all of G. Needs the aircraft.

## Bonus routes
- **Betaflight: partial. Proven in the Betaflight simulator only.** The firmware is the Orqa branch `orqa-betaflight_2026.6-maintenance`, commit 09ea1c692f, built as a simulator (SITL) with the MT optical-flow driver added by `betasim/build.sh`. The stock simulator build has no flow driver. Nothing ran on real Betaflight hardware. Our link is `msp_link.py` (MSP, MultiWii Serial Protocol). It sends a rangefinder message and an optical-flow message in place of GNSS. The firmware path from message to position, read in its source: `betasim/FLOW_CHAIN.txt`.
  - **Proven by the Betaflight simulator** (no flight; one run per item unless a count is given):
    - The firmware accepts our messages: 200 rangefinder and 200 flow messages in 10 s, 200 and 200 acknowledged, no error reply. It reports Betaflight 26.6.3, a POS HOLD mode, and both sensors. `betasim/results/proto.txt`.
    - Sign and scale: we sent 200 mrad/s on each of four legs (north, east, south, west). The flow rate that the firmware computed averaged +198.4, -199.8, -201.9 and +202.1 mrad/s on the matching axis, 0.0 on the other. The leg means are within 2.1 mrad/s of the 200 sent (23 reads per leg). `betasim/results/proto.txt`.
    - Height: 5000 mm sent, 500 cm read back in all 95 reads. `betasim/results/proto.txt`.
    - Closed loop: the simulator flew Position Hold on our messages only, with GPS off. Hold 60 s, wind 2 m/s with 1 m/s gusts, 5 m climb, 2 seeds per mode, 4 runs in all. The aircraft stayed in POS HOLD in every sample. CL.
      - Mode v1 (no position lock): 3.88 and 4.52 m RMS, largest distance 6.54 and 7.92 m.
      - Mode v2 (position lock, gain 0.2 per s): 1.11 and 1.24 m RMS, largest distance 2.13 and 2.29 m.
      - `betasim/results/matrix.txt`, `betasim/results/closed_g0_s1.json`, `closed_g0_s2.json`, `closed_g0.2_s1.json`, `closed_g0.2_s2.json`. Figure: `talk/betaflight_modes.png`.
    - Our onboard program connects to it: on a patched copy, `apb_main.py check` printed PASS for each flight-controller line (firmware Betaflight, attitude 35 Hz, POS HOLD mode, rangefinder, optical flow, barometer height). It printed 2 FAIL lines about video (standard, mask size), not about the flight controller. A 12 s `run_live` at a bench height of 50 m sent 136 fixes and 34 flow messages, and the firmware acknowledged 34 and 34. The simulator sat on the ground, so the log shows "no POS_HOLD". `betasim/results/apb_run.txt`.
    - Unit tests of frames, checksums, parser and link budget: all passed. `betasim/results/test_msp_link.txt`, `betasim/results/test_fc_link_patched.txt`.
  - **Proven by our own model only:**
    - The size of the mode v2 gain effect. In `tests_apb/test_msp_link.py` part F, a constant speed error of 0.10 m/s gives 6.00 m after 60 s in mode v1 and 0.25 m in mode v2. The flight controller in that test is our model with a perfect hold. `betasim/results/test_msp_link.txt`.
    - The flow arithmetic (part D of the same test) follows our reading of the firmware source, not the running firmware.
  - **Not proven:**
    - The closed loop used a synthetic fix: the true position plus 0.05 m noise, and a speed with a constant 0.10 m/s error and 0.05 m/s noise (`args` in each `closed_*.json`). It did not use our camera estimator. The plant is the crude model in the firmware's own test harness. The result shows that the firmware holds position on our messages. It does not show that our camera estimate is good enough.
    - Two seeds, 60 s each, one wind setting.
    - No real flight controller, APB, aircraft or camera. No organizer document names the event firmware (Stage 1 p.5 Q2: Stage 2 will list the builds).
    - The route stops at 10 m. In the firmware source both range settings allow at most 1000 cm (`src/main/cli/settings.c` lines 2005 and 1186) and one helper refuses a height over the limit (`src/main/flight/position_estimator.c` lines 100-111, 468, 517-525). Scaling the flow to a fixed range does not work: the rangefinder is fused into the height estimate with 15 times the weight of the barometer, and the only settings that remove it also stop the flow (`position_estimator.c` lines 445-448, 458-492, 461-465, 512-515). Source read, no simulator run above 10 m. `betasim/results/height_limit.txt`.
    - The Betaflight option is off by default. The repository code still stops at Betaflight: `fc_link.py` (`connect`) and `test_fc_link.py` part C. `betasim/enable.sh` applies the patches in `betasim/patches/` and `betasim/disable.sh` restores every file. On a copy of the project: all 5 patches applied, `test_ground_tools` and `test_msp_link` passed, and disable restored every file by checksum (`integ/integration_2026-11-20/betaflight_option.txt`).
    - The follow-up runs (same plant, 5 m, 60 s, seeds 1 and 2) show that the firmware's own Position Hold on its virtual GPS, fed the true position, was worse than our link in that simulator: 3.13 and 3.44 m RMS (`gps_s1`, `gps_s2`), against 1.26 and 1.10 m for our link with zero speed and fix error (mode v2), and 1.61 and 1.71 m (mode v1). So the numbers are not comparable with the MAVLink simulators, and the plant is crude. `betasim/results/baselines.txt`, `followup.txt`.
    - Lock gain, speed error 0.10 m/s, seeds 1 and 2 where given: gain 0.1: 1.39 and 1.44 m RMS; 0.2: 1.11 and 1.24 m (a second run of the same case gave 1.42 m, so the spread is about 0.3 m); 0.5: 1.15 m (seed 1 only; seed 2 did not arm); 1.0: 1.15 and 1.19 m; gain 0: 3.88 and 4.52 m. Gains 0.2 to 1.0 are not separated. Gain 0.2 stays. `betasim/results/gain_sweep.txt`.
    - The flow count correction for the link (carry the remainder of the counts, `betasim/patches/msp_link_count_carry.patch`) was not kept: in one of four runs the hold error was larger (1.19 against 1.11 m RMS), and a rerun of the same code moved by 0.16 and 0.68 m. One run per case cannot decide. `integ/integration_2026-11-20/msp_m1.txt`.
  - **Event-day proof:** the bench procedure `research/betaflight_bench.txt` (30 minutes, propellers off, stop conditions S1 to S6). Apply `betasim/patches/*.patch` and `python tools/use_profile.py apply outdoor_day betaflight` before.
- **GNSS-free start: partial.** SITL climbs and holds with GNSS off (`px4sim/sitl.py` docstring). The standard parameter sets switch GNSS off: `settings.py` `params_ardupilot` GPS1_TYPE 0, `params_px4` EKF2_GPS_CTRL 0. ArduCopter SITL run `dr_gap_on_s110` (`apsim/drag_results.txt`): its parameter file has GPS1_TYPE 0, and its run record has no GPS message. The flight controller printed "EKF3 IMU0 origin set" and then "EKF3 IMU0 is using external nav data". `evidence/apsim/runs/dr_gap_on_s110.parm`, `evidence/apsim/runs/dr_gap_on_s110.json`. The dataflash log of that run is not in the repository. ArduPilot needs the field origin typed by hand. Our default origin comes from a phone map (`settings.py` lines 37-39). A map position is not a GNSS receiver reading, but the audit rates it as a weak point for this route. Use a fixed origin that no receiver gave, if the firmware accepts it. `review/manuals_audit_2026-11-20.md` Table 1 row 16.
- **GNSS before the hold only (`gnss_mode` "setup"): proven in the simulator that GNSS does not aid the hold.** Added 2026-11-21. Stage 1 question 14 permits GNSS before the scored hold. With the profile `gnss_setup` the EKF uses GNSS while the pilot flies by hand. When our fixes have flowed for 2 s, the program selects EKF source set 2 (our position only) with MAV_CMD_SET_EKF_SOURCE_SET. ArduCopter 4.6 SITL, Justice D, wind 20 gusting 40 km/h, measured over the 30 s from the switch, 16 runs. `apsim/gnss_results.txt`, `integ/gnss_ladder/README.txt`. CL.
  - In all 9 runs with GNSS the FC accepted set 2 before the switch, 1.5 to 8.7 s before. The EKF position step at the change was 0.02 to 0.23 m, in AltHold. The step at the switch was 0.01 m.
  - A 20 m GNSS error during the hold did not move the aircraft: largest distance 0.98 m (a step) and 0.92 m (a ramp of 1 m/s), against 0.81 m with no error, same seed. The same ramp moved an aircraft that holds on GNSS by 19.7 m (the control).
  - Hold error against the default with no GNSS, same seeds, RMS: at 30 m 0.45 and 0.43 m against 0.46 and 0.45 m. At 10 m 0.59 and 0.42 m against 0.81 and 0.75 m. We infer that the 10 m gain comes from an EKF that had settled longer before the switch, not from GNSS in the hold. Not tested: the default with a longer hover before the switch.
  - `gnss_fallback` (one run): after 3 s with no fix in a hold the program selects GNSS again. The run log says that GNSS aids the hold from that time. Largest distance 1.52 m.
  - `gnss_velocity` does not do what its name says: ArduCopter 4.6 EKF3 fuses GNSS velocity only while GNSS is the position source (`AP_NavEKF3_PosVelFusion.cpp` lines 559-562). Not used.
  - Proof in a real flight: the run log note "FC reply to EKF source set 2 (our position only): ACCEPTED" on a row before "engaged", and event 86 before the Loiter row in the FC log.
  - Unit tests: `tests_apb/test_gnss.py` G1 to G9. G2: with the default level the program sends the same MAVLink messages as the code before the change.
  - Not proven: the simulator's GNSS is ideal (no noise). The arming checks were off. No real receiver, no PX4, one or two seeds for each case. The default stays GNSS off from power-on (GPS1_TYPE 0), and only that level is a GNSS-free start.
- **Timing: partial.** VISO_DELAY_MS 100 → 140 ms: RMS middle 1.08 → 0.77 m, worst 2.32 → 0.87 m, each seed better. `integ/control/summary.txt` (`c_base`, `c_d140`). CL, analog, 3 seeds.
- **Rolling shutter: missing.** Readout variants differ less than the seed spread: not resolved (`integ/justice_d/summary.txt` lines 33, 49, 53). Figure: `talk/rolling_shutter.png`.
- **Recovery: partial.** 3 s video outage, PX4 SITL: 0.76 m RMS, 1.49 m max (`px4sim/results.txt:17`, one run). Demonstration option: profile `tools/profiles/recovery_demo.json` drops the video for 3 s, 10 s after the switch, and logs it (`apb_main.py` `OutageTest`). `tests_apb/test_event.py` E4: after the switch the test drops 18 frames, then the run goes on and 71 frames reach Hold (`evidence/tests_apb/out_test_event.txt`). ArduCopter SITL, Justice D, 60 m, wind, 3 s drop, 2 seeds (`apsim/engage_results.txt`, `rec_60m_s1`, `rec_60m_s2`): the FC stayed in Loiter, no fly-away, first fix 1.12 s after the video came back, largest distance from the hold point 1.30 and 1.70 m. At the return the program re-anchors, and the mean fix error shifted 0.33 and 0.59 m. So claim a clean mode-level recovery, not a clean position recovery. PX4 SITL, Justice D, 3 s with no frames at 60 m (2 seeds) and 100 m (1 seed): the program restarted the camera after 2 s and sent again 0.79 to 1.05 s after the video came back. The distance from the hold point at the first new fix was 1.87, 0.36 and 1.02 m. `px4sim/low_outage_results.txt`. Option `gap_jump_s` 0.5 (profile `tools/profiles/gap_jump.json`, off by default): the first frame after a drop is no longer refused as "broken up". ArduCopter SITL, 60 m, wind, 3 s drop, 3 seeds each (`apsim/gapjump_results.txt`): with it, no refusal and no re-anchor in 3 of 3 runs, first fix 0.86 to 0.87 s after the video came back. Without it, a refusal and re-anchors in 2 of 3 runs, first fix 1.12 s in those 2 runs and 0.87 s in the third. The shift of the mean fix error with it is 0.14, 0.29 and 0.16 m. Without it: 0.33, 0.59 and 0.16 m. The ranges overlap, so the gain is not shown beyond the seed spread. The team decides whether to turn it on. Do not use it in a scored hold. Event day: `tools/log_summary.py` gives the time to the first fix (column "recover").

## Third-party tools and our own work
**Libraries** (`requirements.txt`): numpy 2.5.5, opencv-python-headless 5.0.0.93 (LK optical flow, corners, ORB, fisheye model, phase correlation), matplotlib 3.11.2, pymavlink 2.4.50, imageio-ffmpeg 0.8.1.

**External systems:** the FC estimator (PX4 EKF2 or ArduPilot EKF3) does the inertial fusion. In SITL it held position on our fixes alone (`px4sim/results.txt`, `apsim/results.txt`). PX4 SITL (`px4io/px4-sitl:latest`), ArduCopter 4.6 SITL (Orqa branch `mil-mavl`), aerial photos from USGS and the City of Ottawa (`fetch_ground.py`). That file says: do not publish the Ottawa photos.

**Related published methods (opened and checked):**
- Bristeau, Callou, Vissière, Petit, "The Navigation and Control technology inside the AR.Drone micro UAV", 18th IFAC World Congress, Milano, 2011. Section 3.1: optical flow and corner tracking on the vertical camera. Section 4.2: velocity from a rotor-drag model. https://cas.minesparis.psl.eu/~petit/papers/ifac11/PJB.pdf
- Parrot, ANAFI Ai white paper v7.4.0.0, section "Indoor flight", pp. 11–12: optical flow at 60 Hz for velocity, keypoint matching against a hover-start reference image at 15 Hz for position. https://support.parrot.com/assets/s3fs-public/2022-01/whitepaperanafiai.pdf
- ArduPilot EKF3 drag parameters EK3_DRAG_BCOEF_X, EK3_DRAG_BCOEF_Y, EK3_DRAG_MCOEF: `libraries/AP_NavEKF3/AP_NavEKF3.cpp`, github.com/ArduPilot/ardupilot (master).

**Our files:** `CONTRIBUTION.md` lists every Python and shell file that is ours, the third-party software with its licence, the experiments and decisions, and a table for changes during the event. Do not cite `research/*.md` or `review/*.md` for outside facts. Cite the primary source that the note names.
