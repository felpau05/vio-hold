# SPEC.md: the challenge rules this repo follows

Source of truth: the Stage 1 Participant Technical Package (rev 1.3, 17 Nov 2026).
Where this repo and the PDF disagree, the PDF wins. Stage 2 (event configuration, firmware list, scoring,
approved altitudes) is not out yet; every Stage 2 item below is an assumption until then.

## Rules from the spec and where the repo stands

| # | Spec says | Repo |
|---|---|---|
| R1 | Platform is the Orqa **M2-AI** with the **APB** (i.MX8M Plus, 4x A53, 4 GB RAM, Orqa Yocto BSP, SSH). Solution runs **entirely on the APB** (Q1, brief). | `apb_main.py` is the onboard program. Whether Python, numpy and OpenCV exist on the Yocto image is **unknown** (Stage 2). `tools/board_check.sh` tests it. |
| R2 | Nadir camera is the **Justice D**: digital **MIPI CSI**, rolling shutter, 1280x720 at 60 fps, 16:9, FOV 114 (H) x 86 (V) x 121.5 (D) deg (Q3). The Justice A is analog and forward-facing and is not used for the hold (brief, Q9). | `settings.py`: `video_kind="csi"`, `video_size`, `video_fps`, `video_pipeline`, `video_scale`; `capture.py` opens it; lens profile "Justice D nominal (spec FOV, uncalibrated)". Old analog code stays behind `video_kind="analog"` for old recordings and the simulators. |
| R3 | Sensors: APB ICM42605 IMU and DPS310 baro; aircraft manual also lists GPS, magnetometer, baro. Rates, timestamps and any rangefinder: Stage 2 (Q4). | Height comes from the FC baro (`LOCAL_POSITION_NED`), attitude from the FC over MAVLink. `EKF2_RNG_CTRL 0` assumes **no rangefinder**. `research/hardware.md` says ICM-42688: the spec says ICM42605; the spec wins. |
| R4 | No GNSS aiding in the scored hold. GNSS before the hold is fine; a GNSS-free start is a bonus (Q14, brief). | `GPS1_TYPE 0`, `EKF2_GPS_CTRL 0` in the param sets. `origin_lat_lon_alt` is only the ArduPilot EKF origin. |
| R5 | No physical changes, no extra cameras or sensors. Fiducials, tripods and test stands are **development only**: never in the solution, never in scored runs (Q8, Q9, Q10). | No marker logic in the solution. `capture.py` no longer lets marks rescue a flat picture for the Justice D. `DECISION_CARD.txt` no longer says "put out tarps". Mount tilt is corrected in software only (`calib`). |
| R6 | Pilot takes off, reaches the position, holds manually still; a **pilot switch** engages the team's hold. Takeoff, climb, landing and waypoints are out of scope (brief, Q6). | The program sends vision fixes from the start; the pilot's FC mode switch (Loiter/PosHold) is the engage. It never selects a mode itself. `wind_assist` (needs GUIDED) is off and not part of the solution. VIO may initialize during the manual setup flight. |
| R7 | Flight firmware: ArduPilot, PX4, Betaflight, INAV "where available"; final list in Stage 2. Betaflight is an optional bonus (Q2). | PX4 and ArduPilot supported. Betaflight/INAV are only detected over MSP (`fc_link.probe_msp`) and refused at start. |
| R8 | Score: highest altitude with a qualifying stable hold wins the flight portion; levels and thresholds in Stage 2. Judged areas A-G (sensor data and calibration, VIO estimation, onboard execution, FC integration and handover, reliability, contribution, live hold). | See "Evidence map" below. |
| R9 | Pilots are the event crew (Ted, Sujan); development flights are booked (Q5, Q6). Testing is outdoors (Q7). | `min_height_m 15`, outdoor profiles. The `indoor` profile is for bench work only. |
| R10 | Scored demos use the event evaluation setup; team development aids are excluded (evidence rules). | Keep tuning on the TUNING set and report on HOLD-OUT (`DATA_SPLITS.md`). |

## Changes made for the spec (branch `spec-justice-d`)

- `settings.py` / `settings.json`: Justice D defaults (device `/dev/video2`, 1280x720, 60 fps, whole frame, `video_scale 0.5`).
  `readout_s 0.014` and `video_delay_s 0.050` are **guesses**, not measurements (60 fps frame time is 16.7 ms).
- `capture.py`: CSI path (V4L2 with the asked size and rate, or a GStreamer pipeline), `scale_img`, no analog
  field/standard/no-signal logic for csi, and no marks exemption for flat pictures.
- `justice_lens_profiles.json`: nominal Justice D profile. The spec's three FOVs do not fit one pinhole or
  equidistant lens (focal length 480 / 643 / 693 px from V / H / D); I used the horizontal one. Run `calib` for
  `focal_scale` and replace it with a real calibration as soon as there is a camera to calibrate.
- `apb_main.py check`: Justice D frame rate and size checks instead of the PAL/NTSC check.
- Closed-loop simulators and `test_bench` pin `video_kind="analog"`, `field="top"` and the old lens: they still render
  analog fields. **The simulators do not yet model the Justice D** (digital noise, 60 fps, rolling shutter of the real sensor).
  Results in `apsim/`, `px4sim/` and `integ/` are for the old analog camera and say nothing about the Justice D.
- `tests_apb/test_justice_d.py`: new test, part of `run_all.sh`.

## Simulated Justice D results (open loop, 2026-11-18; `integ/justice_d_openloop.txt`)

`sim_ground.py` now has a Justice D camera (`VIO_SIM_CAMERA=justice_d`: 640x360, noise 2.0 grey, readout 14 ms, all assumed).
Estimator only: no drone physics, no wind, no closed loop (Docker is not installed on this machine, so the PX4 and
ArduPilot loops were not run). Seeds 1-6, 20 s hovers:

| Site | 50 m | 100 m | 200 m | 300 m | 400 m |
|---|---|---|---|---|---|
| suburb (tuning) RMS m | 0.24 | 0.32 | 0.67 | 1.05 | 1.47 |
| farmland (validation) RMS m | 0.32 | 0.35 | 0.63 | 1.90 | 2.09 |
| prairie (plain grass, tuning) RMS m | 0.19 | 0.40 | 1.34 | 2.13 | 1.72 |

- Fixes sent: 87-100 % (prairie 150 m lowest, 87.5 %). No run was lost; false-fix stayed at 0 %, except prairie 300 m (0.1 %).
- Variance is overconfident at long range on plain ground: out-50 is 55 % at prairie 300 m and 42 % at farmland 300 m (ideal 50 %: these two are the extremes).
- **A 7 % focal-length error is the biggest threat found:** at 100 m RMS goes from 0.32-0.40 m to 0.93-1.13 m, at 200 m to 1.8-3.4 m, and false-fix rises to 8-16 %. The nominal lens is only good to about 7 %, so `calib` plus the bench lens check (EVENT_DAY step 6: forward value within 1.00 +-0.03) must be done on the real camera.
- Hold-out sites were not run, so these numbers say nothing about the hold-out. Nothing was tuned on them.

## Still open (needs Stage 2 or hardware)

1. Does OpenCV on the APB Yocto image open the CSI sensor, or is a GStreamer/libcamera pipeline needed? Frame time stamps?
2. Real Justice D lens calibration, rolling-shutter readout time, camera-to-FC delay (`calib`, `bench`, `tools/replay_tune.py`).
3. CPU on the real A53 cores at 640x360 or 1280x720 and 60 fps (`apb_main.py timing`). On this x86 laptop Hold takes
   about 7.7 ms median at 640x360 (6.7 ms at the old 360x288 field, 12.3 ms at 1280x720); the A53 will be slower.
4. Which firmware and which MAVLink interface the event provides; whether a rangefinder exists; approved altitudes.
5. Wind and image detail at the approved altitudes (`min_height_m`, high-altitude behaviour: spec G area).
6. Betaflight bonus: no estimator output path exists for it.

## Evidence map (spec areas A-G)

| Area | Where the evidence lives |
|---|---|
| A sensor data and calibration | `calib.py`, `capture.py`, `tools/replay_tune.py`, `review/hardware_gap/` |
| B visual-inertial estimation | `hold.py` (rolling-shutter row-wise attitude, keyframe anchor, scale), `review/estimator/`, `review/geometry/` |
| C onboard execution | `apb_main.py timing`, `bench`, `tools/board_check.sh` (measure on the APB) |
| D FC integration and handover | `fc_link.py`, `apb_main.py check`, `tests_apb/` |
| E reliability | frozen, static, no-ground and moving-object gates in `hold.py`; `tests/bad_pictures.py`; multi-seed runs |
| F contribution | `review/FINDINGS.md`, `integ/`; list which libraries are used (OpenCV, numpy, pymavlink, ArduPilot/PX4) |
| G live hold | closed loop only so far (analog camera); real flights not done |
