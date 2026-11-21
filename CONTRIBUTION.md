# Our contribution (judging area F)

Stage 1 (p.7) asks teams to keep track of which work is theirs, which libraries or existing components they use, and what they change during the event. This file does that. Nothing here ran on the real aircraft or the real APB (Auto Pilot Board). Types of result: UT unit test, SYN synthetic pictures, OL open-loop replay, CL closed-loop simulator (SITL, software in the loop), AN benchmark.

## Part 1. Third-party software

Licences are read from the installed package or from the source tree on this laptop. A licence that was not read is marked "not read".

### Python libraries (`requirements.txt`)

| Library | Version | Licence (where read) | What we use | Where |
|---|---|---|---|---|
| numpy | 2.5.5 | BSD-3-Clause AND 0BSD AND MIT AND Zlib AND CC0-1.0 (package metadata, `numpy-2.5.5.dist-info/licenses/LICENSE.txt`) | Arrays and linear algebra | Every estimator, onboard, simulator and test file |
| opencv-python-headless | 5.0.0.93 | Apache 2.0 for OpenCV (package metadata, `cv2/LICENSE-3RD-PARTY.txt`). The wheel packaging is MIT (`cv2/LICENSE.txt`). Bundled third-party licences: `cv2/LICENSE-3RD-PARTY.txt` | Lucas-Kanade optical flow, corner detection, fisheye model, remap, phase correlation, camera reading (V4L2), resize, colour conversion | `hold.py`, `capture.py`, `calib.py`, `apb_main.py`, `px4sim/camera.py`, `sim_ground.py` |
| matplotlib | 3.11.2 | Matplotlib licence, based on the PSF licence (`matplotlib-3.11.2.dist-info/LICENSE`) | Plots | `tools/plot_log.py`, `sim_ground.py`, `sim_test.py`, `sim_view.py`, `talk/make_figures.py` |
| pymavlink | 2.4.50 | (L)GPL v3 for the generator. The generated code is MIT when it is embedded in a product (`pymavlink-2.4.50.dist-info/licenses/COPYING`) | MAVLink messages and connection (`mavutil`) | `fc_link.py`, `apb_main.py`, `px4sim/sitl.py`, `tests_apb/fake_fc.py` |
| imageio-ffmpeg | 0.8.1 | BSD-2-Clause (`imageio_ffmpeg-0.8.1.dist-info/licenses/LICENSE`). It bundles an FFmpeg 9.0.2 binary. That binary reports `--enable-gpl --enable-version3` in its build configuration (`ffmpeg -version`), so it is GPL version 3 | Video file reading and writing for the simulators | `sim_ground.py`, `px4sim/camera.py`, `real_video.py` |
| Python standard library | 3.12.3 | PSF licence (`/usr/lib/python3.12/LICENSE.txt`) | Everything else | All files |

Offline bundle for the board (`tools/bundle/`, built by `tools/bundle/build.sh`; the archive is not in git):

| Component | Version | Licence (where read) | Use |
|---|---|---|---|
| CPython from python-build-standalone, release 20261110 | 3.12.16, aarch64, glibc | PSF licence (`python/lib/python3.12/LICENSE.txt` in the archive) | Python for a board with no usable Python |
| lxml | 6.1.3 | BSD-3-Clause (wheel metadata) | pymavlink dependency |
| fastcrc | 0.5.0 | MIT (wheel metadata) | pymavlink dependency |
| pyserial | 3.5 | BSD (wheel metadata) | Serial port for the MSP link |
| pip | 26.3 | MIT (wheel metadata) | Install tool inside the bundle |
| libstdc++, libgcc_s, zlib from Debian 11 arm64 (`tools/bundle/syslibs/`) | 10.2.1-6, 10.2.1-6, 1.2.11 | not read | Only for a board that lacks them |
| qemu-user 8.2 | | not read | Test tool on the laptop. It is not shipped |

numpy, opencv-python-headless and pymavlink in the bundle are the aarch64 wheels of the versions in the first table.

The program that runs on the APB needs numpy, opencv-python-headless and pymavlink. matplotlib and imageio-ffmpeg run on the laptop only.

### Flight-controller firmware

| Firmware | Version | Licence (where read) | What we use | Where |
|---|---|---|---|---|
| PX4-Autopilot | `px4io/px4-sitl:latest` (image c0fd482ece15); source clone `px4sim/PX4-Autopilot` at main, commit c4b8f107ae (v1.18.0-beta1-1575) | BSD-3-Clause (`px4sim/PX4-Autopilot/LICENSE`) | Its EKF2 and controller in the SIH simulator. Parameter names and defaults read from the source | `px4sim/sitl.py`, `fc_link.py` (parameters) |
| ArduPilot ArduCopter | 4.6, Orqa branch `mil-mavl`, commit 3b40393 (`apsim/ardupilot`) | GPL v3 (`apsim/ardupilot/COPYING.txt`, header of `ArduCopter/Copter.cpp`) | Its EKF3 and Loiter in the SITL build. Parameter names read from the source | `apsim/run_sitl.sh`, `apsim/sitl_ap.py`, `fc_link.py` |
| Betaflight (Orqa fork `orqa-betaflight`) | Branch `orqa-betaflight_2026.6-maintenance`, commit 09ea1c692f | GPL v3 or any later version (`LICENSE`, `DEFAULT_LICENSE.md` of the clone) | Its simulator build with the MT optical-flow driver. Its test harness `src/test/sitl/sitl_harness.py` is imported at run time, not copied | `betasim/build.sh`, `betasim/closed_loop.py` |

The three source trees are not part of our repository. `apsim/ardupilot/` and `px4sim/PX4-Autopilot/` are git-ignored. iNav is not used.

### Simulators and build tools

| Simulator | Licence (where read) | What we use | Where |
|---|---|---|---|
| PX4 SIH in Docker, image `px4io/px4-sitl:latest` | PX4 code BSD-3-Clause (above). Other packages in the image: not read | Physics, wind and the PX4 stack | `px4sim/sitl.py` |
| ArduCopter SITL, built from the tree above in the image `ardupilot/ardupilot-dev-base:latest` | ArduPilot GPL v3 (above). The build image: not read; it only builds the binary | Physics, wind and the ArduCopter stack | `apsim/run_sitl.sh` |
| Betaflight SITL (above) | GPL v3 or later | Firmware under test, with the plant of its own harness | `betasim/` |
| Docker | not read | Runs the images | scripts above |

Our own renderers draw the camera pictures (`px4sim/camera.py`, `sim_ground.py`). They use the aerial photos below.

### Data sources

| Data | Licence or terms (where read) | What we use | Where |
|---|---|---|---|
| USGS The National Map orthophotos (prairie, farmland, suburb and four hold-out sites) | USGS states that USGS-authored or produced data are in the U.S. public domain, and that some non-USGS images on its pages are copyrighted (usgs.gov, "Copyrights and credits"). The header of `fetch_ground.py` says "public domain" for these sites | Ground pictures for the simulators | `fetch_ground.py` |
| City of Ottawa Basemap imagery, 2022 and 2019 fall | The service states no licence (`fetch_ground.py` header). We use it inside our simulator only. We do not publish it. The photos are git-ignored (`ground/*.jpg`) | Ground pictures for the simulators, at the event site | `fetch_ground.py` |
| One real test clip, name `online-reference` | Local only, not in git. The tests that need it skip. The team records its source and licence before the presentation | Tests of the picture checks | `real_video.py`, `tests/` |
| Organizer documents (Stage 1 package, APB manual, aircraft manual, datasheets) | Supplied to teams | Camera, sensor and board facts. Page numbers are in `EVIDENCE.md` | `settings.py`, `tools/profiles/justice_d.json` |

### Published methods (ideas, no code copied)

- Bristeau, Callou, Vissiere, Petit, "The Navigation and Control technology inside the AR.Drone micro UAV", 18th IFAC World Congress, 2011: optical flow and corner tracking on the vertical camera; velocity from a rotor-drag model.
- Parrot, ANAFI Ai white paper v7.4.0.0, "Indoor flight", pp. 11-12: a reference image taken when a hover starts.
- ArduPilot EKF3 drag parameters `EK3_DRAG_BCOEF_X`, `EK3_DRAG_BCOEF_Y`, `EK3_DRAG_MCOEF` (`libraries/AP_NavEKF3/AP_NavEKF3.cpp`).
- The Betaflight position estimator path, read in the firmware source (`betasim/FLOW_CHAIN.txt` lists the files and lines).

## Part 2. Our own work

Every Python and shell file that git tracks, plus the untracked files that git does not ignore. Counts per group are in the headings. `integ/rehearsal_2026-11-20/`, `integ/integration_2026-11-20/` and `tools/bundle/` hold their own scripts; each folder has a README that lists them. The estimator, the onboard program, the calibration (`calib.py`) and the link files are the code that is meant for the APB. The simulators, tests, tools and the experiment scripts run on the laptop. The calibration and tuning tools `tools/replay_tune.py` and `tools/lens_calib.py` run on the laptop, and their values are copied to the board. Stage 1 does not say whether laptop-solved numbers are allowed (`review/manuals_audit_2026-11-20.md` Table 1 row 15).

Third-party code that we changed: none. Frozen copies of our own earlier versions: `tests/hold_v1.py`, `integ/lowtex/hold_before.py`, `integ/lowtex/capture_before.py`.

### Estimator (56 files)

| File | What it does |
|---|---|
| `hold.py` | The estimator: picture, tilt and height in, position out. Keyframe anchor, rolling-shutter row times, outlier-rejecting 2-D similarity fit. |
| `integ/indoor/tilt_exp.py` | Open-loop experiment: can a joint fit of tilt and slide against the keyframe measure the change of the gyro tilt. |
| `integ/lowh/closed/scripts/ab_driver.py` | A/B driver: runs low_ap_matrix.py labels in ab_before and ab_after, one SITL at a time, interleaved. |
| `integ/lowh/closed/scripts/make_closed.py` | Before/after table from ab_before and ab_after apsim/low_ap_results.json (closed-loop A/B of proposed.patch). |
| `integ/lowh/common.py` | Shared parts of the low-height study (integ/lowh): load a hold.py by path, the Justice D camera, the grounds. |
| `integ/lowh/cubic_err.py` | Offline geometry check (no pictures): how well does hold.py's cubic whole-picture map, fitted to the 8 x 6 keyframe. |
| `integ/lowh/diag_summary.py` | Which input error makes the halves refusals at 5 m (ottawa2022, crop lens, seeds 11-13)? Diagnostic replays. |
| `integ/lowh/e6_check.py` | tests/run_regression.py E6 scene (100 m hover, ottawa2019fall, PAL, yaw swings of +-60 and +-90 deg at 0.05 Hz), hold.py. |
| `integ/lowh/final_tables.py` | Before / after tables of the low-height study (integ/lowh/results/ol). |
| `integ/lowh/jsonz.py` | Read a result file of integ/lowh: plain .json or gzip-compressed .json.gz (the stored runs are compressed). |
| `integ/lowh/make_variants.py` | Build hold.py variants for the low-height study from the repository's hold.py (text replacements, each must match once). |
| `integ/lowh/ol_low.py` | Open loop at low height: a recorded ArduCopter SITL hover in wind, replayed at a chosen height, through one or more. |
| `integ/lowh/ol_summary.py` | Tables of the open-loop flight replays (integ/lowh/results/ol/*.json). |
| `integ/lowh/probe_compare.py` | Compact comparison of probe files (integ/lowh/results/probe): per file and height, the still-hover gap (p90 / max px) |
| `integ/lowh/probe.py` | Controlled open-loop probe of Hold's whole-picture path at low height: the halves gap against the drift from. |
| `integ/lowh/probe_summary.py` | Tables of the probe results (integ/lowh/results/probe/*.json): the still-hover gap (noise floor) per height, and. |
| `integ/lowh/run_jobs.sh` | Run the commands of a job file (one per line) one at a time, each with nice -n 19, and only while the 1-minute load. |
| `integ/lowh/safety_totals.py` | Totals of the bad-picture replays (ol_low.py --bad), over seeds 11-13: per bad kind, ground, height and version. |
| `integ/lowh/shares.py` | How much of the halves refusals comes from noise and how much from geometry (drift from the keyframe)?. |
| `integ/lowtex/capture_before.py` | Frame sources for the hold, behind one call: newest(timeout) -> Frame or None. |
| `integ/lowtex/compute.py` | ms per frame on this laptop with cv2.setNumThreads(1): the whole-picture path alone (Hold._dense) and. |
| `integ/lowtex/dstats.py` | Quality numbers of the whole-picture path (Hold._dense): peak, halves gap (px), match, on good ground. |
| `integ/lowtex/hold_before.py` | Keyframe-anchored nadir position hold for a rolling-shutter camera (version 2). |
| `integ/lowtex/judge_check.py` | The capture judge (capture._Judge) on flat pictures: snow with tarps and lights at night must pass; flat. |
| `integ/lowtex/queue.sh` | Low-texture build: closed-loop runs, ONE AT A TIME (variant apb, 60 s, w20g40 unless stated). |
| `integ/lowtex/summ.py` | Table of closed-loop runs (px4sim/runs/<label>.json): drift RMS / max, fixes per second, % frames with. |
| `review/estimator/common.py` | Shared harness for the estimator review. Imports the project's own simulators unchanged. |
| `review/estimator/e1b_honest_var.py` | the proposed variance model (honest.py) on runs it was NOT tuned on. |
| `review/estimator/e1_variance.py` | is the variance Hold sends honest, and which error sources does it miss?. |
| `review/estimator/e2_bad_cases.py` | the two bad cases. |
| `review/estimator/e2_probe.py` | Probe of the two bad cases of check 2. |
| `review/estimator/e3b_rate_noise.py` | Hold.rate (max /Euler-angle rate/ from the FC attitude in a window around the frame) against. |
| `review/estimator/e3_keyframes.py` | keyframe changes. How often, why, what each one costs, and a calmer rule. |
| `review/estimator/e4b_fb_check.py` | The forward-backward check in Hold.process starts the backward LK AT the keyframe point it. |
| `review/estimator/e4_outliers.py` | inputs that break the outlier logic of Hold.process. |
| `review/estimator/e5_velocity.py` | the velocity filter  vel = 0.7 vel + 0.3 (pos - pos_prev)/dt  and its uses. |
| `review/estimator/e6_jitter.py` | frame-to-frame jitter of the sent position, its spectrum, and how much a flight. |
| `review/estimator/e7b_mount_yaw.py` | A constant tilt error between camera and FC attitude (camera-mount misalignment, or FC level. |
| `review/estimator/e7_long_hover.py` | 120 s hover at 100 m and 200 m over ottawa2019fall with a slow 5 m wander. |
| `review/estimator/e8_texture.py` | texture loss. |
| `review/estimator/e9b_low_fps.py` | does the hold still work at the frame rate the i.MX8M Plus would reach. |
| `review/estimator/e9_compute.py` | compute cost of Hold.process on this laptop, and what it means for the i.MX8M Plus. |
| `review/estimator/honest.py` | Proposed variance model for Hold, as a drop-in subclass (the position estimate is unchanged). |
| `review/estimator/scipy_free_welch.py` | Welch power spectral density without scipy (one-sided, Hann window, 50 % overlap). |
| `review/geometry/common_hover.py` | Shared open-loop hover runner for the review scripts (own renderer, project Hold). |
| `review/geometry/indep.py` | Independent geometry for the geometry review, written from first principles. |
| `review/geometry/t0_conventions.py` | Items 1, 2, 5: compare the project's euler_R, R_BC_NADIR and load_lens with own derivations. |
| `review/geometry/t1_rays_independent.py` | Item 8 (and 3, 4): independent test of Hold._rays and of the keyframe/position solve. |
| `review/geometry/t2_mount_closed_loop.py` | Critical finding: the camera mount is a hard-coded guess, and a wrong guess makes a fly-away. |
| `review/geometry/t3_focal_scale.py` | High finding: a focal-length (K scale) error gives a position error of about h * eps * (tilt change). |
| `review/geometry/t4_readout_direction.py` | Rolling-shutter direction: what if the sensor reads bottom-to-top (or the picture is flipped in memory)?. |
| `review/geometry/t5b_similarity_fix.py` | Suggested fix for the yaw/baro lever: solve position, yaw and scale together (2-D similarity fit. |
| `review/geometry/t5_yaw_baro_lever.py` | High finding: a change in FC yaw error (or baro error) since the keyframe is multiplied by. |
| `review/geometry/t6_lens_field.py` | lens scaling to one analog field, field parity, wrong-profile guard, exposure timing. |
| `review/geometry/t7_tilt_bias_yaw_swing.py` | A constant roll/pitch bias (FC level error or camera mount tilt b) does not cancel against. |
| `review/geometry/t8_sim_budget.py` | Where the known 300 m sim error comes from: sim_test's own setup, with its injected errors removed. |

### Onboard program (3 files)

| File | What it does |
|---|---|
| `apb_main.py` | The onboard program. Modes run, check, mask, calib, bench, replay, delay, timing. Sends the fixes and logs each frame. |
| `capture.py` | Camera input behind one call (newest frame): analog and digital readers, Y-plane reduction, picture checks. |
| `settings.py` | Every value that can change on the event day, in one place. |

### Calibration (9 files)

| File | What it does |
|---|---|
| `calib.py` | Event-day calibration of delay, readout, lens and mount from a 20 to 60 s recording while a person turns the drone by hand. |
| `integ/calib_loop.py` | Calibration in the loop . |
| `integ/control/wiggle_calib.py` | Video delay (and readout) from the take-off wiggle of one apsim run, with calib.py's own search. |
| `integ/justice_d/calib_loop_d.py` | Calibration in the loop for the Orqa Justice D (digital): the Justice D variant of integ/calib_loop.py. |
| `integ/justice_d/calib_rays.py` | Rays of the calibrated camera (calib_loop_d.py: new lens profile + mount) against the true camera of camera.py. |
| `review/hardware_gap/bench_calib.py` | Reference implementation of the apb_main.sweep spec, plus a mounting and focal check. |
| `review/hardware_gap/t_bench_calib.py` | Proves the event-day hand-rotation check works (items 1, 3, 4, 5). |
| `tools/lens_calib.py` | Lens calibration with a checkerboard, for the development area (not the event day). |
| `tools/replay_tune.py` | Tune timing, lens and two estimator knobs on one flight recording, without the true position. |

### Flight-controller links (3 files)

| File | What it does |
|---|---|
| `fc_link.py` | MAVLink link to the flight controller (PX4 and ArduPilot): attitude, height, vision position, parameters, mode. |
| `msp_link.py` | MSP link to a Betaflight flight controller: reads attitude, gyro and height, sends rangefinder and optical-flow messages. |
| `tests_apb/fake_fc.py` | A small fake flight controller over MAVLink/UDP, for testing fc_link and apb_main without hardware. |

### Simulators (33 files)

| File | What it does |
|---|---|
| `apsim/drag_matrix.py` | EKF3 drag fusion A/B on ArduCopter SITL: does the EKF dead-reckon through a fix gap, and does it hurt the hold?. |
| `apsim/engage_matrix.py` | Pilot-switch options on ArduCopter SITL: fresh_kf_at_engage A/B and the recovery demonstration (outage_test_s). |
| `apsim/gapjump_matrix.py` | hold_cfg {"gap_jump_s": 0.5} in the recovery demonstration on ArduCopter SITL (instance 1). |
| `apsim/low_ap_matrix.py` | Low-height holds (5 m, 10 m) on ArduCopter 4.6 SITL with the new outdoor floor: settings min_height_m 3. |
| `apsim/matrix_ap.py` | Run groups of apsim/sitl_ap.py cases, N side by side (one SITL instance each, own ports). |
| `apsim/onboard_cpu.py` | px4sim/onboard.main with a slow-board model that other programs on this laptop cannot change (much). |
| `apsim/run_sitl.sh` | ArduCopter SITL (built from apsim/ardupilot, Orqa's mil-mavl = Copter-4.6.0 + board commit), headless. |
| `apsim/sitl_ap.py` | Closed-loop test on ArduCopter 4.6 SITL (Orqa's fork, apsim/ardupilot), as px4sim/sitl.py does for PX4. |
| `betasim/apb_run_test.py` | apb_main with msp_link against the Betaflight SITL, on a patched copy of the project (APB_DIR). |
| `betasim/build.sh` | Clone and build the Betaflight SITL (software in the loop) with the MT optical flow and rangefinder drivers. |
| `betasim/closed_loop.py` | Closed loop: Betaflight SITL flies Position Hold on our flow and rangefinder messages. |
| `betasim/disable.sh` | Reverses betasim/enable.sh: removes tools/profiles/betaflight.json and reverses the patches. |
| `betasim/enable.sh` | Turns the Betaflight link on in this project: applies the patches in betasim/patches/ (fc_link.py, apb_main.py. |
| `betasim/proto_test.py` | Protocol, sign and scale proof against the Betaflight SITL. No flight: the SITL is not armed. |
| `betasim/run_followup.sh` | Follow-up Betaflight closed-loop runs (wind 2 m/s with 1 m/s gusts, 5 m, 60 s, seeds 1 and 2), one simulator at a time. |
| `betasim/run_matrix.sh` | Closed-loop matrix: mode v1 (gain 0) and v2 (gain 0.2/s), seeds 1 and 2, 60 s hold each, one SITL at a time. |
| `betasim/sitl.py` | Start and stop the Betaflight SITL (software in the loop) build for the betasim tests. |
| `betasim/summarize.py` | Table of closed-loop runs: RMS and max hold error, RMS true speed, and the main period of the north error. |
| `fetch_ground.py` | Download aerial photos (orthophotos) for sim_ground.py. |
| `integ/indoor/render_check.py` | Render the indoor tuning floors (and scene options) from a still pose, save a montage, count corners. |
| `px4sim/camera.py` | Downward camera for the closed-loop PX4 test, rendered from the TRUE pose. |
| `px4sim/check_camera.py` | Open-loop check: the independent renderer (camera.py) against hold.py, with exact inputs. |
| `px4sim/low_outage_post.py` | Post-run numbers for px4sim/low_outage_run.py results, from px4sim/runs/<label>.npz and the onboard logs. |
| `px4sim/low_outage_run.py` | Low-height holds (5 m, 10 m) and the 3 s video outage with the digital Justice D, PX4 SITL. |
| `px4sim/matrix.py` | Run a list of closed-loop cases; merge them into px4sim/results.json and px4sim/results.txt. |
| `px4sim/onboard.py` | The onboard program in its own process, as on the APB: apb_main.run_live with fc_link to the PX4 SITL. |
| `px4sim/out/run_batch.sh` | Runs a batch of PX4 closed-loop cases one after the other. |
| `px4sim/out/run_first.sh` | Runs the first group of PX4 closed-loop cases. |
| `px4sim/replay.py` | Replay a logged closed-loop run open loop, to find where the estimate error comes from. |
| `px4sim/sitl.py` | Closed-loop test: real PX4 (SIH simulator in Docker) flies, wind pushes, and the only horizontal. |
| `sim_ground.py` | Honest simulator: real aerial photos as the ground, at their true size. |
| `sim_test.py` | Synthetic test: a rolling-shutter nadir camera over textured ground, scored against truth. |
| `sim_view.py` | Pictures of what the simulated camera sees, for people (and the talk). |

### Tools (8 files)

| File | What it does |
|---|---|
| `tools/board_check.sh` | Run on the board, in the program folder:  sh tools/board_check.sh [--route] [WHEEL_FOLDER]. |
| `tools/bundle/build.sh` | Build the case B bundle (the board has no Python that we can use) |
| `tools/bundle/proof/hold_check.py` | Hold.process on synthetic pictures: a 50 m hover for 4 s at 30 frames/s (sim_test.py: rolling shutter, noise). |
| `tools/deploy.sh` | Copy the onboard program and settings.json to the board, restart it there, show the last log lines. |
| `tools/log_summary.py` | Numbers from run logs (logs/run_*.csv of "apb_main.py run", or replays) for the judges' evidence (Stage 1. |
| `tools/plot_log.py` | Quick look at one per-frame log of "apb_main.py run" (logs/run_*.csv) or of a replay: one PNG. |
| `tools/sim_rec.py` | A raw recording, as "apb_main.py run --raw" saves it, made from a closed-loop simulator run. For rehearsing. |
| `tools/use_profile.py` | Settings profiles: small JSON overlays in tools/profiles/, merged onto settings.json. |

### Tests (24 files)

| File | What it does |
|---|---|
| `tests_apb/run_all.sh` | Runs every APB test; each writes tests_apb/out_<name>.txt. Prints the last line of each. |
| `tests_apb/test_ap_sitl.py` | fc_link against real ArduCopter 4.6 firmware (SITL in container ap_sim, apsim/run_sitl.sh), not a fake. |
| `tests_apb/test_basics.py` | Small pieces: settings, capture (picture judge, field split, V4L2 queries), the height gate. |
| `tests_apb/test_bench.py` | apb_main bench mode (event-day ground tests G1/H3) in the simulator: the lens 1.5 m over a floor. |
| `tests_apb/test_calib.py` | calib.solve on synthetic hand-rotation recordings: all 8 picture orientations, with a known video. |
| `tests_apb/test_digital.py` | The digital camera path (Orqa Justice D, MIPI CSI, 1280x720 at 60 fps) with injected buffers: no camera. |
| `tests_apb/test_event.py` | Event-day additions from the Stage 1 package (rev 1.3), with fake cameras and a fake flight controller. |
| `tests_apb/test_fc_link.py` | fc_link against the fake flight controller (tests_apb/fake_fc.py, separate process, UDP localhost). |
| `tests_apb/test_ground_tools.py` | The ground-tool corrections of the 2026-11-20 rehearsal (integ/rehearsal_2026-11-20/README.txt, section 3) and. |
| `tests_apb/test_lens_d.py` | Justice D lens (digital, 1280x720, 16:9): the three assumed profiles, load_lens at 640x360. |
| `tests_apb/test_lens_store.py` | The measured lens is calibration data. It lives in settings.json under lens_measured, as the mount and the delay do. |
| `tests_apb/test_live.py` | Liveness test: Hold sends fixes in a hover by hand, and sends none on a frozen or overlay-only picture. |
| `tests_apb/test_msp_link.py` | msp_link without a simulator: frames, both checksums, the parser (split, damaged, error replies) |
| `tests_apb/test_replay_chunks.py` | Replay of a raw recording in 3 chunks must give the same fixes as the same recording in one piece. |
| `tests_apb/test_robust.py` | Camera and run code faults found by the hardware review (D1-D8), with fake cameras and a fake flight controller. |
| `tests_apb/test_run.py` | apb_main run mode end to end: simulator frames (capture.CallableSource) and the fake flight. |
| `tests_apb/test_wind_assist.py` | apb_main.WindAssist (experiment, off by default): it acts only in GUIDED, moves the target upwind of. |
| `tests/bad_pictures.py` | Bad pictures (scope 1 and 3; E1, E2, H4, E7): does Hold refuse them, and does it come back alone?. |
| `tests/compute.py` | Compute (scope 8; E9, H6): ms per Hold.process call at 360 x 240 with one OpenCV thread. |
| `tests/harness.py` | Shared code for the hold.py version 2 tests. |
| `tests/hold_v1.py` | Frozen copy of hold.py version 1 (2026-11-15), used only by tests/ as the "before" in before/after numbers. |
| `tests/interface_check.py` | Interface contract of hold.py version 2: Est fields, attitude with gyro rates, reanchor(pos, var) |
| `tests/overlay.py` | Static overlay (scope 2; E2, real video): OSD text burned into 20 % of the picture. |
| `tests/run_regression.py` | Regression runner: one line per review finding in the CORE scope. ID, PASS / FAIL / INFO, and. |

### Experiment and review scripts (84 files)

| File | What it does |
|---|---|
| `integ/avail/calm_why.py` | where the calm-air drift comes from, from a closed-loop run's own logs (no new flight). |
| `integ/avail/diag.py` | Open-loop check of the onboard pipeline (apb_main.Pipe + Hold + the tilt filter) on a logged flight. |
| `integ/avail/holdout_open.py` | Item 5, open loop: sim_ground.one (20 s hovers, sim_ground's own renderer) on the three HOLD-OUT sites. |
| `integ/avail/merge.py` | Merge the B runs (px4sim/runs/<label>.json) into px4sim/results.json and rewrite results.txt. |
| `integ/avail/queue_b.sh` | B closed-loop queue (one run at a time; each under 5 min) |
| `integ/avail/queue_c.sh` | B closed-loop queue 2 (one run at a time, nothing else running) |
| `integ/avail/queue_d.sh` | B hold-out closed-loop runs: final code and settings, HOLD-OUT sites and seeds; nothing tuned on them. |
| `integ/clock_wobble.py` | How steady is PX4 SITL's lockstep clock against wall time? From a run's npz (truth sample times. |
| `integ/control/an.py` | Analysis of apsim runs for the control work (integ/control). |
| `integ/control/make_profiles.py` | Writes apsim/profile_goal_a.json and apsim/profile_goal_b.json from the apsim results (apsim/results.json). |
| `integ/control/q.sh` | Runs ArduPilot closed-loop cases ONE AT A TIME from integ/control/queue.txt (one sitl_ap.py argument list per. |
| `integ/failure_report.py` | Failure cases in closed loop: what the drone did during and after the failure, and what the onboard. |
| `integ/fixerr.py` | Fix error against the truth for one closed-loop run (px4sim/runs/<label>.npz + its onboard CSV). |
| `integ/gap/closed_summary.py` | Closed loop before and after the proposed patch: the gap (as integ/justice_d/fix_error.py: the longest time between. |
| `integ/gap/ekf_at_switch.py` | the autopilot's position error at the switch (the Loiter target is the EKF position at that moment). |
| `integ/gap/gap_reasons.py` | which check stops the fixes in the longest gap around the hold start of each closed-loop run. |
| `integ/gap/hand_hover.py` | Q2 (open loop): the competition sequence. The pilot flies to the test position, holds the drone still by hand for. |
| `integ/gap/hh_summary.py` | Table of the hand-hover runs (integ/gap/hand_hover.py, results/hh/*.json). Code: before = old hold.py (MOVE_PX 1.5) |
| `integ/gap/hover_motion.py` | What moves in the still hover before the hold start of each closed-loop run (the input of the liveness test). |
| `integ/gap/live_diag.py` | Diagnostic twin of one tests_apb/test_live.py case (same scene changes as its worker), keeping the full. |
| `integ/gap/replay_all.sh` | Open-loop replay (integ/gap/replay_run.py) of every closed-loop run with a gap of 1 s or more, with hold.py as it. |
| `integ/gap/replay_run.py` | Open-loop replay of one recorded ArduCopter closed-loop run through the onboard pipeline (apb_main.Pipe + Hold) |
| `integ/gap/run_closed.sh` | One ArduCopter SITL closed-loop run with the proposed patch, from the patched copy of the repository. |
| `integ/gap/run_freeze_v2.sh` | Frozen-picture hand-hover runs with the liveness patch v2 (the patch now in hold.py). The earlier set has freeze. |
| `integ/indoor/fixerr.py` | Estimator error apart from the autopilot: sent fix minus the true position (turned into PX4's heading frame) |
| `integ/indoor/queue2.sh` | After queue.sh rest: closed-loop A/B of the "still" proof change, and frozen pictures with fresh noise (item 6). |
| `integ/indoor/queue3.sh` | After the 22:12 restart: the runs that the restart cut or never started, ONE AT A TIME, in scope order. |
| `integ/indoor/queue.sh` | Indoor and low-height build: closed-loop PX4 runs, ONE AT A TIME (variant apb). Usage: sh integ/indoor/queue.sh <set>. |
| `integ/indoor/summ.py` | One line per closed-loop run: summ.py integ/indoor/cl_*.txt  (RMS and worst drift in cm). |
| `integ/indoor/table.py` | Results table of every closed-loop run of this build: table.py > integ/indoor/results.txt. |
| `integ/integration_2026-11-20/review/scripts/ast_compare.py` | Compare each top-level statement of two Python files by syntax tree (ast.dump, no line numbers). |
| `integ/integration_2026-11-20/review/scripts/cases.py` | Build timing cases from the rehearsal recordings (copies and links only) and run apb_main.py timing on each. |
| `integ/integration_2026-11-20/review/scripts/clock_check.py` | Task 2a on the real files: raw t_grab minus the nearest run-log t, for every raw frame of each flight. |
| `integ/integration_2026-11-20/review/scripts/contrast.py` | For each case folder: the switch the new code uses (first 'engaged' note), then in its window [sw-16, sw-1]. |
| `integ/integration_2026-11-20/review/scripts/d4_recorder.py` | D4: with the no_sd values on the Justice D picture (640x360 for Hold, record_hz 10, min_free_gb 0.5), check. |
| `integ/integration_2026-11-20/review/scripts/look_data.py` | Script. |
| `integ/integration_2026-11-20/review/scripts/make_patches.py` | ---- B1: choose the switch whose 15 s window is a clean wiggle window ----. |
| `integ/integration_2026-11-20/review/scripts/mutate.sh` | mutate.sh NAME FILE SED_EXPR : copy base, apply one change to FILE, show the changed line, run the new test. |
| `integ/integration_2026-11-20/review/scripts/rerun.py` | Run timing on every case folder with the apb_main.py of a given folder. Fresh flight settings for each case. |
| `integ/integration_2026-11-20/review/scripts/window_stats.py` | Attitude statistics of 15 s windows (the timing window) along each rehearsal flight: RMS turn rate as calib.Rec. |
| `integ/justice_d/check_analog_same.py` | Check that the analog camera of px4sim/camera.py gives the same pictures as before the Justice D change. |
| `integ/justice_d/check_replay_same.py` | An existing analog open-loop case, before and after the Justice D change: px4sim/replay.py (re-renders the PAL. |
| `integ/justice_d/compute.py` | Compute time on this laptop, one thread (cv2.setNumThreads(1)): Hold.process per frame at 320x180, 640x360. |
| `integ/justice_d/fix_error.py` | Fix error of the onboard program in a closed-loop run: each sent fix (onboard log north, east) against the true. |
| `integ/justice_d/open_loop.py` | Open loop: the analog PAL camera against the Orqa Justice D, estimator only (Hold), no drone physics. |
| `integ/justice_d/queue_closed.sh` | T4 and T5 closed-loop cases, one at a time (run_ap.py waits for a 1-minute load under 4 before each). |
| `integ/justice_d/run_ap.py` | One ArduCopter SITL closed-loop case (apsim/sitl_ap.run_case), written to integ/justice_d/closed/<label>.json. |
| `integ/justice_d/summary.py` | Builds integ/justice_d/summary.txt from the run files in integ/justice_d/ (open/, closed/, compute_*.json. |
| `integ/queue.sh` | Runs closed-loop cases one per process (each under 5 min): integ/queue.sh group:index [group:index ...]. |
| `integ/race_test.py` | hold.py attitude race (SIM and APB reports): a reader thread appends ATTITUDE rows while Hold-side. |
| `integ/refind_timing.py` | Time Hold.process on the frame after 'lost' (the re-find of the last/home keyframe), offline. |
| `integ/rehearsal_2026-11-20/after_flight.py` | After each SITL flight of apflights.py: the commands of EVENT_DAY step 7 and EIGHT_HOURS hour 5, on. |
| `integ/rehearsal_2026-11-20/apflights.py` | Step 7 "fly" and EVENT_DAY step 7: Justice D flights on ArduCopter SITL (apsim/sitl_ap.run_case), with. |
| `integ/rehearsal_2026-11-20/calib4_all.sh` | Item B: four hand-turn calibrations (true lens x start lens), then two at record_hz 10 (no_sd). One run each, seed 5. |
| `integ/rehearsal_2026-11-20/calib4.py` | Audit item B: the hand-turn calibration (apb_main calib --rec) from each start lens, on synthetic pictures. |
| `integ/rehearsal_2026-11-20/cam_stream.py` | Stand-in for the Justice D on the laptop: a video_command that writes GREY 1280x720 frames at 60 fps to its. |
| `integ/rehearsal_2026-11-20/card_quotes.py` | Step 9 of the rehearsal: is each quoted program message of DECISION_CARD.txt in the code?. |
| `integ/rehearsal_2026-11-20/check_sim_rec_analog.py` | Proof that the --video option of tools/sim_rec.py did not change the default (analog) output. |
| `integ/rehearsal_2026-11-20/defect_bench_traceback.sh` | Defect: "apb_main.py bench" (also mask and calib) prints a Python traceback, not one FAIL line, when the FC does not. |
| `integ/rehearsal_2026-11-20/defect_betaflight_profile.sh` | Defect: tools/profiles/betaflight.json (new, 2026-11-20 03:50) names keys that settings.py does not have yet. |
| `integ/rehearsal_2026-11-20/defect_check_lpos_ardupilot.sh` | Defect: on ArduPilot, "apb_main.py check" shows "FAIL LOCAL_POSITION_NED 0 Hz ... The FC sends no height" on. |
| `integ/rehearsal_2026-11-20/defect_free_disk.py` | Defect: the check line "free disk" and the raw recorder use min_free_gb in two ways. |
| `integ/rehearsal_2026-11-20/defect_timing_accepts_hold.sh` | Defect: "apb_main.py timing" passes on raw chunks of a Position-mode hold in gusty wind and writes wrong values. |
| `integ/rehearsal_2026-11-20/gcs.py` | The pilot's mode switch, for the rehearsal: ask ArduCopter SITL (sitl_ap.sh, SERIAL0 on udp 14560) for a flight. |
| `integ/rehearsal_2026-11-20/hup_test.py` | Does "apb_main.py run" keep running when the ssh process that started it ends? (audit Table 1 row 2, item A.2) |
| `integ/rehearsal_2026-11-20/make_recs.sh` | Justice D recordings for step 7 (fly, record, replay, tune): the same PX4 SITL flights as the 2026-11-19 rehearsal. |
| `integ/rehearsal_2026-11-20/profile_combos.py` | Step 8 of the rehearsal: every profile combination that EVENT_DAY.txt, EIGHT_HOURS.txt and DECISION_CARD.txt. |
| `integ/rehearsal_2026-11-20/sitl_ap.sh` | ArduCopter SITL as the "aircraft" for the event-day rehearsal (apsim/ardupilot build, container ap_sim. |
| `integ/rehearsal_2026-11-20/step7.py` | Step 7 of the rehearsal: fly, record, replay, tune with Justice D recordings (tools/sim_rec.py --video JUSTICE_D). |
| `integ/summary.py` | Headline numbers from px4sim/results.json: per height and wind, the onboard program (apb, medium. |
| `review/hardware_gap/harness.py` | Shared code for the hardware-gap review scripts. |
| `review/hardware_gap/t10_frozen_video.py` | video stops but frames keep coming (frozen frame, or a grabber's "NO SIGNAL" screen). |
| `review/hardware_gap/t11_px4_facts.py` | the PX4 v1.15.4 facts behind the flight-controller findings, read from the local clone. |
| `review/hardware_gap/t1_timing.py` | what a frame-time error does to the hold (Ottawa photo ground, sim hover attitude). |
| `review/hardware_gap/t2_video.py` | interlaced analog frames, field order, and OSD text burned into the picture. |
| `review/hardware_gap/t3b_exposure_gain.py` | exposure time (motion blur) and automatic gain changes. |
| `review/hardware_gap/t3_readout.py` | rolling-shutter readout model wrong (true 16 ms), and readout running bottom to top. |
| `review/hardware_gap/t4_lens.py` | lens model wrong. Hover with 5 m wander, Ottawa photo ground. |
| `review/hardware_gap/t5_mount.py` | camera mounting differs from R_BC_NADIR. Ottawa photo ground, 100 m, open loop. |
| `review/hardware_gap/t6_closed_loop.py` | Items 5 and 1, closed loop: does the drone stay or fly away?. |
| `review/hardware_gap/t7_attitude_link.py` | attitude over the 230400-baud MAVLink link. |
| `review/hardware_gap/t8_height.py` | barometric height errors. |
| `review/hardware_gap/t9_cpu.py` | Compute budget: time per Hold.process call on this laptop, one thread. |
| `talk/make_figures.py` | Make the talk figures from result files that exist in the repository. No simulator starts. |

## Part 3. Experiments and decisions

Result types as above. A result with one run is marked "one run". Seeds are the ones in the proof file.

| What we tried | Result | Decision | Proof file |
|---|---|---|---|
| Hold with the true calibration, open loop, Justice D crop lens, wind 20 gusting 40 km/h | RMS median 0.114, 0.241, 0.478 m at 20, 50, 100 m (3 seeds). OL | Best case. Not a claim for the real camera | `integ/justice_d/summary.txt` lines 9, 21, 33 |
| Full chain (calibrate, then hold), closed loop, Justice D, 100 m | 0.661, 1.615, 0.783 m RMS with the same settings. CL, one run each | One run proves little. Report all three | `integ/justice_d/summary.txt` lines 317-319 |
| Liveness check refused still-hover frames, so no fix before the switch | Gap 0.33 to 4.50 s before the fix. With the "moved" proof: 0.10 to 0.13 s. EKF error at the switch 0.28-1.39 m before, 0.01-0.51 m after. Hold RMS before to after on the same 4 cases: 0.437 to 0.588, 0.654 to 0.485, 0.715 to 0.568, 1.615 to 0.564 m. Worse in one, better in three. CL | Keep the "moved" proof (code v2) | `integ/gap/results/closed_summary.txt` |
| Same fix in hand-hovers with no vibration | Gaps up to 5.83 s remain. With 0.2 degree vibration every case is 0.08 s. OL | Open limit. Not fixed | `integ/gap/results/hh_summary.txt` |
| Overlay-only picture | 21 fixes in 288 frames, limit 0. SYN | Defect is open | `integ/gap/results/test_live_revised.txt` |
| Frozen picture with fresh noise in a still hover | Fixes up to 5.96 s after the freeze, errors up to 0.52 m, before any fix. Fix v2 keeps the same protection (24 runs) but does not close this | Open limit | `integ/gap/results/hh_summary.txt` |
| Hand-turn calibration, no target, one run | Delay 58 ms (true 58), readout 11 ms (true 10), fx +0.34 %, fy +0.23 %. Mount check failed: 0.86 degrees against 0.5. SYN, one run | Keep the method. Do not claim the mount | `integ/justice_d/calib_loop_d_20261118_230048.txt` |
| In-flight stick wiggle for the delay | 62 and 64 ms (true 60.4) at 40 m and 100 m. CL, one run each | Fly the wiggle at 40 m or higher | `integ/control/wiggle_calib.txt` |
| Replay tuner on a recording below 100 m | At 30 m it would set delay 0.1499 s (true 0.0604), readout 0 (true 0.0184), focal scale 1.0816 (true 1.0). One recording per height | Use it only from 100 m or higher. Set timing from the calibration and the wiggle | `integ/rehearsal_2026-11-19/README.txt` |
| Rolling-shutter model against readout 0 and wrong sign | RMS median 0.478, 0.485, 0.499 m at 100 m, 3 seeds. Single runs 0.384 to 0.753 m. OL | Effect not resolved. No claim | `integ/justice_d/summary.txt` lines 33, 49, 53 |
| ArduPilot VISO_DELAY_MS 100 to 140 | RMS middle 1.08 to 0.77 m, worst 2.32 to 0.87 m. CL, analog, 3 seeds | Adopted 140 | `integ/control/summary.txt` (`c_base`, `c_d140`) |
| FC raw attitude against gyro attitude, ArduPilot | Raw: fly-away at 30.1 m after 16.51 s. Gyro: 0.428 m RMS. CL, one seed | Adopted gyro attitude with a 20 s lean toward the FC | `apsim/results.txt` (`ap_apb_100m_calm_s1`, `_attfc`) |
| Speed messages (send_speed) | Version 1: EKF failsafe, fly-away. Version 2: 100 m 3.18 and 2.39 m RMS against 0.77 and 0.87 m without. 7.11 m RMS in the run `c_spd_100m_w20g40_s11` | Dropped | `DATA_SPLITS.md` line 55, `apsim/results.txt` |
| Trust scale cov_scale 0.1 | Worst case 1.47 m against 0.87 m RMS | Dropped | `DATA_SPLITS.md` line 52 |
| Wind assist version 1 (target moved by the whole push) | 100 m 1.70 and 2.38 m against 0.77 and 1.27 m | Dropped. Later versions are off by default | `DATA_SPLITS.md` line 57 |
| Whole-picture path as one pixel shift | Ran away in closed loop: 4.7 m RMS, 17 m max on `holdout_field` seed 11 | Replaced by a solve in metres of drone move | `DATA_SPLITS.md` line 41 |
| Whole-picture path at scale 0.5 | Same error, twice the compute (7.6 fixes/s) | Kept scale 0.25 | `DATA_SPLITS.md` line 38 |
| Flat-picture rule sensor_noise_min 1.0, MIN_STRONG 6/8, MIN_PTS 15, KF_LOW 0.9 and 0.95 | Tried, no gain stated in the log | Dropped, not in the code | `DATA_SPLITS.md` lines 34, 35 |
| Fresh home keyframe at the pilot's switch | Unit test: 0.003 m move. ArduCopter SITL A/B, 30 to 100 m, 1 to 2 seeds: no difference beyond the spread of identical runs (up to 0.25 m RMS) | Off by default | `apsim/engage_results.txt`, `evidence/tests_apb/out_test_event.txt` |
| ArduPilot drag fusion | RMS inside the seed spread. Largest drift higher in 6 of 6 seed pairs (+0.03 to +0.69 m). 20 s outage at 100 m: 30 m drift with and without it (one seed each) | Not recommended. Profile is a record only | `apsim/drag_results.txt` |
| Recovery demonstration, ArduCopter SITL, 60 m, 3 s drop, 2 seeds | Loiter held, first fix 1.12 s after video, largest distance 1.30 and 1.70 m, fix error shift 0.33 and 0.59 m | Claim mode-level recovery only | `apsim/engage_results.txt` |
| Option gap_jump_s 0.5, 3 s drop, 3 seeds | No refusal and no re-anchor in 3 of 3 (without it 2 of 3 had them). Shift 0.14 to 0.29 m against 0.16 to 0.59 m. Ranges overlap | Off by default. The team decides | `apsim/gapjump_results.txt` |
| Low heights, PX4 SITL, wind 20 gusting 40 km/h | 5 m 0.54 and 0.51 m RMS, 10 m 0.75 and 0.65 m (2 seeds each). Perfect position: 0.46 and 0.47 m (one run each) | Allow holds from 3 m in the outdoor profiles | `px4sim/low_outage_results.txt` |
| Low heights, ArduCopter SITL, same wind | 5 m 0.85 and 0.44 m, 10 m 0.98 and 0.93 m. Perfect position 0.35 and 0.34 m. At 5 m fix gaps up to 6.28 s | Prefer 10 m to 5 m | `apsim/low_ap_results.txt` |
| Measure the halves gap in each half's own pixels (patch), 5 m | Open loop: refusals 114 to 15, longest gap 2.75 to 1.42 s (3 seeds). ArduCopter SITL: longest gap 3.34 to 1.38 s, hold RMS 0.48 to 0.43 m (inside the seed spread). Costs: more half-frozen pictures pass | In `hold.py` as `dense_local`, off by default. On only through the profile `low_height` (with `dense_turn` and `att_source` gyro_fcyaw), for holds at 5 to 10 m. With all three, ArduCopter SITL at 10 m: 0.79, 0.71, 0.62 m RMS to 0.42, 0.33, 0.31 m | `integ/lowh/results/before_after.txt`, `integ/lowh/closed/README.txt`, `integ/kfmap/closed/ab_table.txt` |
| Hold with the analog camera, hold-out sites, 30 m | 0.305 to 0.748 m RMS. Hold-out lawn at 100 m: 5.81 m | Report both | `apsim/results.txt` |
| Hold.process timing on one laptop core | 3.13 and 3.22 ms at 640x360, 4.57 and 4.99 ms at 1280x720 (2 runs). AN | No APB number exists | `integ/justice_d/compute_20261118_225729.json`, `integ/justice_d/compute_20261118_230038.json` |
| Betaflight simulator: messages, sign and scale | 200 of 200 acknowledged each. Flow rate within 2.1 mrad/s of 200. SITL, one run | Keep the link. Simulator only | `betasim/results/proto.txt` |
| Betaflight closed loop, mode v1 against mode v2 | v1 3.88 and 4.52 m RMS. v2 1.11 and 1.24 m RMS. 2 seeds each, synthetic fix | Adopted mode v2 (`msp_lock_gain` 0.2 in `tools/profiles/betaflight.json`) | `betasim/results/matrix.txt` |
| Betaflight: send the FC's fused height back as the rangefinder | It closed a loop. The FC height lagged the true height by 5 m. One 20 s run | Use the barometer debug value (`debug_mode BARO`) | `betasim/results/closed_try1.json` |
| Betaflight position source and sign | Read in the firmware source. The simulator driver stage gave the rate we sent on both axes | Keep the formula of the source | `betasim/FLOW_CHAIN.txt`, `betasim/results/proto.txt` |
| Betaflight patches to the repository code | Tests pass on a patched copy (frames, parser, link). Not applied here | Apply before the event only if the firmware is Betaflight | `betasim/patches/`, `betasim/results/test_fc_link_patched.txt` |
| Rehearsal of the event-day steps, Justice D path, ArduCopter SITL (one flight per height, seed 31) | A video delay 30 ms too long: hold RMS 1.12 m at 60 m and 9.85 m at 100 m (one flight each). Wiggle `timing` gave 40 ms at 10 m, a refusal at 30 m, 38 ms at 60 m, 36 ms at 100 m | Fly the wiggle and run `timing` before the high holds | `integ/rehearsal_2026-11-20/README.txt` section 2.B, 4 |
| Replay tuner on Justice D recordings with the delay set 30 ms wrong | 60 m: focal scale 8 % off, RMS 0.312 to 0.765 m. Flown 100 m recording: delay 12 ms short. One recording per height | Merge a tune only from 100 m or higher, and only with no wiggle result | `integ/rehearsal_2026-11-20/README.txt` sections 2.A, 2.B |
| Lens calibration from two start lenses, two true lenses, 30 and 10 Hz (seed 5, one run each) | All fits within 0.37 % (fx) and 0.35 % (fy), residual 0.054 to 0.068 degrees, delay 58 ms, readout +11 ms. Residuals differ by 0.002 degrees | Second calibration not shown to help. Use the second start lens only if a fit ends at the edge of its range | `integ/rehearsal_2026-11-20/README.txt` section B |
| Ground-tool corrections to `apb_main.py` (`check` height stream, one FAIL line, free disk, `timing`) | `tests_apb/test_ground_tools.py` 32 checks pass. The first test version failed 13 times on the old code. Flight loop unchanged (56 definitions identical by syntax tree). Full suite 15 of 16 groups without a FAIL | Applied | `integ/integration_2026-11-20/README.txt`, `run_all_v2.txt`, `flight_loop_unchanged_v2.txt` |
| First `timing` correction, independent review | Two blockers: it took a ground "engaged" note, and it did not check for a wiggle. 126 cases: 13 PASS, none with a wrong value after the second version | Rejected and corrected. Wiggle limit `WIGGLE_RATIO` 4 was set on the same four flights that test it: a limit, not a proof | `integ/integration_2026-11-20/review/REVIEW.txt`, `timing_cases_v2.txt` |
| Offline Python bundle on emulated 64-bit ARM (qemu-user 8.2, cortex-a53 model, no network) | Imports, `check`, Hold, replay, `timing` and 8 test groups pass. Same numbers as x86_64. 10 real-time FAIL lines cannot be decided by emulation. Default qemu CPU model gave wrong tracking | Use the bundle only if the board has no usable Python. No timing claim from emulation | `tools/bundle/README.txt`, `tools/bundle/proof/` |
| Betaflight: firmware's own hold on the true position, against our link (5 m, 60 s, seeds 1 and 2) | Firmware 3.13 and 3.44 m RMS. Our link, zero error, mode v2 1.26 and 1.10 m, mode v1 1.61 and 1.71 m | Not comparable with the MAVLink simulators. Plant is crude | `betasim/results/baselines.txt`, `followup.txt` |
| Betaflight lock gain 0.1, 0.2, 0.5, 1.0 (speed error 0.10 m/s) | 1.39/1.44, 1.11/1.24, 1.15 (one seed), 1.15/1.19 m RMS. Same-case spread about 0.3 m | Gain 0.2 stays. Gains 0.2 to 1.0 are not separated | `betasim/results/gain_sweep.txt` |
| Betaflight above 10 m: send a fixed range and scale the flow | Stopped at the source check: the rangefinder is fused into the height with 15 times the barometer weight, and the settings that remove it also stop the flow. No simulator run | Route stops at 10 m | `betasim/results/height_limit.txt` |
| Betaflight flow count carry (`msp_link_count_carry.patch`) | 1 of 4 runs worse (1.19 against 1.11 m RMS). A rerun without it moved by 0.16 and 0.68 m | Not kept. More seeds needed | `integ/integration_2026-11-20/msp_m1.txt` |
| Betaflight option switch | enable.sh applied 5 patches on a copy, tests passed, disable.sh restored all files | Off by default | `integ/integration_2026-11-20/betaflight_option.txt` |
| Audit of the project against the manuals | 16 disagreements and 15 unused facts listed. Examples: flight-mode switch names, SD card and 4 GB flash, propeller and battery rules | Event-day steps changed in `EVIDENCE.md` areas A, C, D | `review/manuals_audit_2026-11-20.md` |

## Part 4. Changes during the event

Write one row for each change to a file during the event day. Write the time on the laptop clock, the file path, what changed, why, and the path of the log or the picture that shows the effect.
Do not leave a change out. If you undo a change, write a new row.
Copy each proof file into `evidence/` with its relative path, and add it to git.

| Time | File | Change | Reason | Proof file |
|---|---|---|---|---|
| | | | | |
| | | | | |
| | | | | |
