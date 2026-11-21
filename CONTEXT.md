# CONTEXT.md: where the project stands (updated 2026-11-21, Stage 2, first day on the hardware)

Written so a new person (or a new Claude session) can pick up without the chat. Branch: **`stage2-bench`**.
Read `TODO.md` (what is left, by points) and `HARDWARE_BRINGUP.md` (measured facts) next.

## Stage 2 in short (package: https://github.com/Haxerus/redteam-grand-final, clone not in this repo)

- Scoring 100 + 20 bonus. **Bench A-E = 70 points**, judge carries the drone at about 1 m, props off, our laptop shows
  the live estimate; up to 3 judged sessions, 2 tries per item, passed items banked. **Flight G = 20** (levels 10/20/35/50/75 m,
  30 s within 3 m / 2 m, scored from the FC log by `hold_judge.py`), allowed only after D and E are banked, 2 flights.
  F = 10 at the presentation. Bonuses +5 each: Betaflight hold, GNSS-free start, timing/rolling-shutter evidence, recovery.
- FC: **Betaflight 2025.12.5 installed; we chose Betaflight** (no flashing risk, +5). Commands: `MSP_SET_RAW_RC` at ~50 Hz,
  used while CH10 (TL2) is high (MSP OVERRIDE + ANGLE). **No command timeout**: send safe values (roll/pitch/yaw 1500,
  hold-height throttle) continuously from start-up, never minimum values. ArduCopter 4.7.2 is optional (we do not use it).
- No GNSS while engaged; GPS stays connected for the judges. Team dev testing: walk the drone by hand first, then fly.

## Done today (commits 89b3d64 .. on stage2-bench)

- Hardware measured (`HARDWARE_BRINGUP.md`): Python 3.12 + numpy 1.26 on the board, OpenCV 4.11 put in `/data/vio/pylib`;
  Justice D 1920x1080@60 on `/dev/video2` with monotonic end-of-frame stamps; never change its size (crops the sensor);
  uncached buffers (12 ms per copy); IMU buffered at 200 Hz; barometer 4x/8 Hz; FC identity and mode switches over MSP.
- APB-only path: `v4l2_mmap.py`, `apb_sensors.py`, `apb_rec.py`, `apb_est.py` (attitude filter, barometer height, replay),
  `apb_live.py` (live position + web page :8080 + `--rec`), `tools/apb_ctl.sh` (start/stop/status). Tests: `tests_apb/test_apb_est.py`.
- Desk checks: `top = rear`; covered lens detected (fine-detail check); live page works.
- From `origin/stage1-justice-d` (Paul's pre-Stage-2 branch, taken file by file, never merge it whole): `tools/lens_calib.py`
  (fixed; OpenCV 4 only, venv `~/.venvs/vio4`), `msp_link.py` framing, `betasim/`, talk material.

## Open, in order (details in TODO.md)

1. **Lens calibration** (checkerboard, `tools/lens_calib.py`): distances scale with it; the nominal profile is probably ~40 % off.
2. First walk session with tape marks (B1/B2/B3/cover), recorded for replay.
3. MSP command link for C, D, E; auto-start (E2); then the flight controller for G.
4. Speed: the estimator runs at 13-16 fixes/s on the A53 (LK forward+backward dominates).

## Pitfalls found today

- `pkill -INT` does nothing on programs started in the background over ssh (SIGINT inherited ignored): use `apb_ctl.sh stop`.
- A `pkill -f` name pattern can match the ssh shell running it.
- `tools/lens_calib.py` gives a wrong lens under OpenCV 5: use `~/.venvs/vio4`.

---

# Before Stage 2 (2026-11-19 notes, kept for reference; facts above win)


Written at the end of a working session so a new person (or a new Claude session) can pick up without the chat.
Branch: `spec-justice-d` (4 commits on top of `main`; pushed to origin 2026-11-20; no pull request opened yet). Source of truth for the rules:
the Stage 1 participant rules (rev 1.3). Read `SPEC.md` next.

## The project

VIO position hold for a drone with no GPS, for the Contested Battlefield Aerial Operations challenge (Grand Final)
on the Orqa **M2-AI**. A downward camera + FC tilt gives a position, sent to ArduPilot or PX4 over MAVLink
(`VISION_POSITION_ESTIMATE`); the autopilot holds it. Runs on the **APB** (i.MX8M Plus, 4x A53, Yocto).
Core code: `hold.py` (estimator), `apb_main.py` (onboard program), `fc_link.py`, `capture.py`, `calib.py`, `settings.py`.
Full map: `README.md`.

## What the Stage 1 spec says (PDF rev 1.3) and what changed because of it

- Nadir camera is the **Justice D**: digital MIPI CSI, 1280x720 at 60 fps, 114x86 deg FOV, rolling shutter. The
  Justice A is analog and forward-facing and is not used. The repo had assumed the analog Justice A.
- No markers/tarps/fiducials in the solution or scored runs (dev area only). No hardware or mount changes.
- No GNSS aiding during the scored hold. Pilot flies, holds still, flips a switch to engage our hold.
- Highest altitude with a qualifying stable hold wins the flight part. Judged areas A-G. Stage 2 (firmware list,
  access, rates, approved altitudes, scoring, rangefinder?) is **not out yet**.
- Code/doc changes made (commits `14f25f3`, `5a30721`): Justice D capture path and settings (`video_kind`,
  `video_size`, `video_fps`, `video_pipeline`, `video_scale`), nominal lens profile, no "marks" exemption for the
  digital camera, `check` mode tests the Justice D, `tests_apb/test_justice_d.py`, simulators/`test_bench` pinned to
  the analog camera, docs (`SPEC.md`, `HARDWARE_BRINGUP.md`, `EVIDENCE.md`, `ORGANIZER_QUESTIONS.md`, updated
  checklists and research notes), simulator Justice D camera (`VIO_SIM_CAMERA=justice_d`, `VIO_SIM_FOCAL_ERR`).

## Environment set up in this session

- Python venv `~/.venvs/vio` (created after `python3-venv`/`pip` were installed by the user). `pypdf` was also
  installed there to read the PDF; it is not in `requirements.txt`.
- Aerial photos downloaded into `ground/` (gitignored). Ottawa photos: simulator use only, do not publish.
- `apsim/ardupilot` cloned (Orqa `mil-mavl`, gitignored) and **built** (2026-11-18): `apsim/ardupilot/build/sitl/bin/arducopter`
  (Copter 4.6.0, 8 min build in `ardupilot/ardupilot-dev-base:latest`; build command is in `apsim/run_sitl.sh`).
- Docker: the user is now in the `docker` group. It is **Docker Desktop** (4.99), whose `--network host` joins the Docker VM,
  not WSL's `127.0.0.1`, so the SITL got no heartbeat. Fixed on 2026-11-19 with Docker Desktop -> Settings -> Resources ->
  Network -> "Enable host networking" **plus** `C:\Users\aydan\.wslconfig` = `[wsl2]` / `networkingMode=mirrored` (created
  this session, outside the repo), then `wsl --shutdown`. Check: a UDP packet from the `ap_sim` container to
  `127.0.0.1` reaches a listener in WSL. Keep Docker Desktop's WSL integration ON for this distro. Rejected alternative:
  a native Docker engine inside WSL (not needed).
- Pitfall: `apsim/run_sitl.sh` reuses a running `ap_sim` container. If you start `ap_sim` by hand without
  `-v apsim:/apsim` the next run fails with "no heartbeat". Fix: `docker rm -f ap_sim` and let the script create it.

## Test and result status

- `sh tests_apb/run_all.sh`: all 9 onboard-program tests + `test_justice_d` pass (fake FC; `test_ap_sitl` does not
  need the real ArduPilot build).
- **ArduPilot closed loop confirmed (2026-11-19, analog camera):** `python apsim/sitl_ap.py apb 100 w20g40 60`: RMS 0.617 m,
  p95 0.92 m, max 0.99 m, 1.43 m/min drift, fixes in 48 % of frames (about 12 Hz), median age 177 ms. Identical to the earlier
  `apsim/results.txt` row, so older ArduPilot results stand. Other rows there: seed 2 1.35 m RMS; `att_source=fc` flies away
  (14.9 m RMS: keep `gyro`); `holdout_field` 3.3 m RMS (reported once, not re-run or tuned on).
- Review findings re-run (analog sim, outputs in `integ/review_rerun/`): E1, E2, E7 fixed; E3 much better;
  **E5 open** (a 90 deg yaw leaves 2.2-2.6 m at 100 m: do not yaw during the hold); E6 re-verified (scripts rewritten): prairie 50 m
  191 -> 34 keyframe changes, RMS 1.3-1.9 -> 0.2 m; **still weak on prairie 100 m seeds 3/5 (1.7-1.9 m, 80 % degraded)**.
- Open-loop Justice D simulation (`integ/justice_d_openloop.txt`; assumed noise/readout/fps, estimator only):
  RMS about 0.3 m at 100 m, 0.6-1.3 m at 200 m, 1-2 m at 300-400 m; 87-100 % fixes. **A 7 % focal-length error
  triples the error** (100 m: 0.35 -> ~1.0 m) and makes the variance overconfident: real lens calibration is the
  top risk.
- Older closed-loop results (`apsim/summary.txt`, `px4sim/results.txt`, `integ/summary.txt`, analog camera): holds
  within 1 m RMS / 2 m max up to 100 m in wind 20 gusting 40 and 30 gusting 55; 122 m is marginal, 200 m fails.
  Plain ground (prairie, open field) was the flyaway case in older runs; a newer prairie run held at 1.77 m RMS
  with only 34 % of fixes. `att_source` must stay `gyro` (the FC attitude flew away every time).

## What is still unknown or assumed (do not treat as facts)

- `readout_s 0.014`, `video_delay_s 0.050`, `video_scale 0.5`, `/dev/video2`, the Justice D lens (nominal, from the
  horizontal FOV; V and D FOVs imply +-7 %), sensor noise in the sim. None measured.
- Whether Python/numpy/OpenCV exist on the APB image, whether OpenCV can open the CSI sensor (maybe GStreamer),
  frame time stamps, CPU at 60 fps on the A53s, rangefinder, which firmware the event provides.
- IMU: the spec says ICM42605; `research/hardware.md` section 6 says ICM-42688 (community source). Spec wins.
- The PX4 and ArduPilot closed-loop simulators still render the old analog camera. `sim_ground.py` has a
  Justice D open-loop mode whose results depend on assumed camera parameters. Real-camera measurements are pending.

## Software to-do, in progress (2026-11-19, later session)

Goal from the user: all software done; hardware items may wait.
- DONE: `camera="justice_d"` preset for the closed-loop simulators (`px4sim/sitl.py` `JUSTICE_D`/`apply_camera`, `px4sim/camera.py`
  `VIDEO["JUSTICE_D"]`; works for `apsim/sitl_ap.py` too: `python apsim/sitl_ap.py apb 100 w20g40 60 camera=justice_d label=...`).
  Written but NOT yet run end to end: Docker Desktop was not running (needs the user to start it).
- DONE: `integ/calib_loop.py` runs for the Justice D with `VIO_SIM_CAMERA=justice_d` (output `integ/calib_loop_justice_d.txt`):
  calib finds a focal scale of 0.93 as 0.934, mount tilt within 0.2 deg, readout and delay within tolerance. So the
  top risk (focal error) is handled by `apb_main.py calib` in simulation; the real lens is still to be measured.
- `sh tests_apb/run_all.sh`: all pass except `test_ap_sitl`, which needs Docker.
- Prairie 100 m weakness was an analog-camera result; with the simulated Justice D prairie 100 m is 0.40 m RMS (open loop).
- E5 (yaw): `hold.py` already adds a mount-error x yaw-change variance term; nothing can remove an FC level bias, so the pilot does not yaw.
- TODO (needs Docker): closed-loop Justice D runs on ArduPilot (and PX4 if its container is still available), including wind,
  focal error and `test_ap_sitl`; then hold-out runs once.

## Rules to keep following

- `DATA_SPLITS.md`: tune on TUNING, check on VALIDATION, report on HOLD-OUT once. Hold-out sites
  (`holdout_quarry`, `holdout_water`, `holdout_lawn`) were **not** run this session. Log every tuned value there.
- Branch per change and pull request into `main` (README). Simulator output goes to `runs/` and `logs/` (gitignored).
- Never use markers or tarps in anything that counts as the solution.

## Next steps (in order)

1. (Done: Docker/networking fixed, ArduCopter SITL built, ArduPilot case 100 m `w20g40` confirmed.) Run one **PX4** case
   (100 m, `w20g40`; `px4sim/sitl.py`) to confirm that loop, then work the plain-ground cases (prairie 100 m seeds 3/5,
   1.7-1.9 m RMS, about 80 % degraded) on TUNING/VALIDATION sites only.
2. (Preset done, commit 141f517, not yet run end to end: needs Docker Desktop.) Run the closed-loop Justice D cases.
3. Consider a focal-length-uncertainty term in the variance. (E6 re-verification is done: `review/estimator/e3_keyframes.py`
   and `e2_bad_cases.py` were rewritten for the current `hold.py`; `review/FINDINGS.md` updated.)
4. Send `ORGANIZER_QUESTIONS.md`; fill `HARDWARE_BRINGUP.md` and `EVIDENCE.md` as hardware and Stage 2 arrive.
5. (Pushed 2026-11-20.) Open a pull request into `main`.
6. Before the event: build `tools/wheels/` (aarch64 wheels for the APB, gitignored, currently absent) and write `tools/deploy.env`.
   A readiness review on 2026-11-20 found test_ap_sitl is the only failing test and only because Docker Desktop was off.

## Open items for the team

- Fill in who wrote `hold.py` and the rest of the who-wrote-what table in `EVIDENCE.md`.
- Set `origin_lat_lon_alt` in `settings.json` at the field (ArduPilot only); run `mask` and `calib` on the real camera.
- `EVIDENCE.md` shows a trailing-newline change that is not from this work; it is not committed.
- (The 2026-11-19 work listed here earlier is committed.)
