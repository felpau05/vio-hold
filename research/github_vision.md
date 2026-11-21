# GitHub vision parts for GNSS-free hover (downward analog camera, rolling shutter, 4 x A53)

Date: 2026-11-16. Method: `gh api` (metadata, file lists, raw files), shallow clone of ngps_flight, WebSearch. `gh search` hit its rate limit after about 25 calls, so repo discovery by search is partial. "(search)" = from a web search summary, not opened by me. "INFERENCE" = the guess.
Licence = SPDX field from the GitHub API, plus LICENSE header where the API said NOASSERTION. Dates = last push to the default branch.

## Ranked table

| # | Repo | Licence | Last push | Stars | Lang | Verdict | Part to reuse (paths) | Needs |
|---|---|---|---|---|---|---|---|---|
| 1 | gyroflow/gyroflow https://github.com/gyroflow/gyroflow | GPL-3.0 | 2026-11-15 | 9729 | Rust | BORROW (method only, GPL) | src/core/synchronization/{rs_sync.rs,find_offset/,optical_flow/pyrlk.rs,lens_delay.rs}; src/core/gyro_source/ (per-row gyro lookup); lens DB: gyroflow/lens_profiles (CC0-1.0, 2026-11-13) | Rust, CPU. Use offline as a tool on a hand-rotation clip, not on the drone |
| 2 | AdrianEddy/telemetry-parser https://github.com/AdrianEddy/telemetry-parser | Apache-2.0 | 2026-11-09 | 321 | Rust | ADOPT (if logs are Betaflight or ArduPilot) | README lists Betaflight blackbox (.bfl .bbl .csv) and ArduPilot logs (.bin .log); outputs gyro + accel. Apache so code may be copied | Rust, CPU |
| 3 | snktshrma/ngps_flight https://github.com/snktshrma/ngps_flight | MIT | 2026-11-03 | 49 | Python/C++ | BORROW | ap_dataset_logger/src/ap_dataset_logger/camera_clock_sync.py (min-filter fit of camera clock to host clock); ap_dataset_logger/scripts/measure_camera_latency.py (screen-strobe latency test); check_dataset_sync.py; preflight_check.py; fc_params_check.py; fc_log_check.py; ap_ngps_ros2/src/ap_ngps_ros2/{statustext.py,predictor.py}; lua_scripts/srcfs_gps_fallback.lua | ROS 2 Jazzy, PyTorch/TensorRT/LightGlue in the matcher (JetPack 7.2). Helpers above are small and mostly plain Python |
| 4 | PX4/PX4-OpticalFlow https://github.com/PX4/PX4-OpticalFlow | BSD-3-Clause | 2026-09-21 | 201 | C++ | BORROW | src/flow_opencv.cpp (goodFeaturesToTrack + calcOpticalFlowPyrLK + undistortPoints, quality = tracked fraction, `limitRate`); src/flow_px4.cpp (SAD block match); include/optical_flow.hpp (image + timestamp in, dt + rad flow + quality out) | OpenCV, CPU, no ROS. Pinhole distortion only in setCameraDistortion (no fisheye model seen; INFERENCE: wrap with cv2.fisheye first) |
| 5 | HKUST-Aerial-Robotics/VINS-Mono https://github.com/HKUST-Aerial-Robotics/VINS-Mono | GPL-3.0 | 2024-08-14 | 6073 | C++ | SKIP for flight, BORROW idea | config/euroc/euroc_config.yaml has `estimate_td`, `td`, `rolling_shutter: 0/1`, `rolling_shutter_tr`. Only the Mono repo has the RS flag | ROS 1, Ceres, Eigen. Wants IMU at 100+ Hz in the same process. 4 days: no |
| 6 | HKUST-Aerial-Robotics/VINS-Fusion https://github.com/HKUST-Aerial-Robotics/VINS-Fusion | GPL-3.0 | 2024-05-23 | 4791 | C++ | SKIP | euroc_mono_imu_config.yaml has `estimate_td` only. Found no `rolling_shutter` key (grep of the config) | ROS 1, Ceres |
| 7 | rpng/open_vins https://github.com/rpng/open_vins | GPL-3.0 | 2025-11-30 | 3192 | C++ | SKIP | config/*/estimator_config.yaml: `calib_cam_timeoffset` yes; found no readout-time key upstream. A ModalAI fork documents `calib_cam_readout` (search, not opened). Config set includes uzhfpv_outdoor (FPV drone data) | ROS 1/2 optional, Eigen. Runs on small ARM in ModalAI VOXL (search) but needs real IMU stream |
| 8 | VladyslavUsenko/basalt https://github.com/VladyslavUsenko/basalt | BSD-3-Clause | 2026-03-22 | 1030 | C++ | SKIP | Only if you want a permissive VIO. Did not check RS support (not found in the reading) | CPU, no ROS; heavy to build |
| 9 | hovren/crisp https://github.com/hovren/crisp | GPL-3.0 | 2017-09-05 | 235 | Python | BORROW (method; old, GPL) | crisp/timesync.py (time offset by gyro vs video rotation), crisp/znccpyr.py (ZNCC pyramid tracker), crisp/tracking.py, crisp/ransac.py | Python 2-era, numpy, OpenCV. Unmaintained since 2017 |
| 10 | gyroflow/rollingshutter https://github.com/gyroflow/rollingshutter | MIT | 2022-08-22 | 24 | Python | ADOPT (offline tool) | rollingshutter.py (counts LED bands -> readout time); rs_blinker/ (Arduino sketch) | Python, a blinking LED. Needs 1/1000+ s shutter per README; analog camera may not allow this (INFERENCE) |
| 11 | smidm/flashvideosynchronization https://github.com/smidm/flashvideosynchronization | MIT | 2024-03-15 | 31 | Python | BORROW | flashvideosynchronization.py: flash-based sync, reports offset and time per sensor row. Video-to-video, not video-to-gyro | Python, OpenCV |
| 12 | AprilRobotics/apriltag https://github.com/AprilRobotics/apriltag | BSD-2-Clause (LICENSE.md header) | 2026-08-07 | 2621 | C | ADOPT (only if a tag target is allowed) | the C library; python wrapper pupil-labs/apriltags (NOASSERTION, not opened) | C, CPU only, no ROS |

Other repos opened (not ranked):
- thien94/vision_to_mavros, GPL-3.0, last push 2022-10-28: scripts/t265_precland_apriltags.py, apriltags3.py, t265_to_mavlink.py send LANDING_TARGET and vision messages through pymavlink. Ideas only (GPL, T265-specific).
- ArduPilot/ardupilot, GPL-3.0, 2026-11-15: libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp (accepts OPTICAL_FLOW from companion), libraries/AC_PrecLand/{AC_PrecLand_MAVLink.cpp,PosVelEKF.cpp} (target tracking filter with delay handling). Ideas only (GPL, C++ firmware).
- PX4/PX4-Autopilot, BSD-3-Clause, 2026-11-15: flow fusion lives in the ekf2 module (not opened).
- snktshrma/ap_nongps, no licence, 2025-12-03: ideas only.
- Kimera-VIO (BSD-2, 2026-08-06), rovio (BSD-style header, 2026-09-03), okvis (BSD-style, 2021), msckf_vio (Penn licence text, not read; treat as ideas only), uzh-rpg/rpg_svo (GPL, 2019), rpg_svo_pro_open (GPL, 2024), ORB-SLAM3 (GPL): none opened beyond licence; all need a real IMU stream, most need ROS.
- openmv/openmv: API says no licence field; its docs describe `find_displacement()` (phase correlation, log-polar option for rotation/scale) for hover (search). Idea: use cv2.phaseCorrelate + createHanningWindow, both in opencv (Apache-2.0, opened).

## ngps_flight: what to lift and can it run on 4 x A53 (Q2)
Seen in the clone (MIT LICENSE, last commit 2026-11-04):
- The core is a ROS 2 node (ap_ngps_ros2/src/ap_ngps_ros2/ngps_localization_node.py, 1772 lines) that matches camera frames to a satellite map with SuperPoint + LightGlue at 1-2 Hz, plus a UKF in C++ (ap_ukf) and keyframe ORB VO. README badges and requirements.txt: torch 2.11, TensorRT 10.16, JetPack 7.2. The paper (per our notes, abstract only) used Jetson Orin NX.
- ap_vo/src/ap_vo/vo_node.py is a two-frame SIFT + PnP on a ground plane at a fixed `plane_distance_m` default 50 m. It is ROS 2 and has no rolling-shutter code. grep for "rolling" or "shutter" in all .py files found nothing.
- It needs a pre-flight satellite map. Our task has none (INFERENCE: the map matcher does not apply to hover over a point).
- CPU answer: the learned matcher will not run usably on 4 x A53 without a GPU (INFERENCE; nobody in the repo tests it). The fallback path "inference_backend: pytorch" is slower still. The small non-ML helpers do run on CPU.
- Best reusable code: camera_clock_sync.py (docstring: stamp frames from the camera's own clock, fit host = slope*cam + offset using the lowest residual per bucket, constant glass-to-timestamp latency measured separately), measure_camera_latency.py (strobe the screen, find the lag; the lag value goes in `latency_offset_ms`), check_dataset_sync.py, preflight_check.py. These speak to our "timing between video and attitude" problem. For analog capture the clock source differs (no RTP); use the idea, not the file.
- Did not find any frozen-picture check or OSD mask in this repo.

## Answers by topic
1. Downward hover without GNSS: no complete open project found that holds over a point from a rolling-shutter analog camera. Pieces exist: PX4-OpticalFlow (KLT and SAD flow), ArduPilot MAV optical-flow input, OpenMV displacement matching (docs only). ArduPilot's BlueOS OpticalFlow extension (search; in ArduPilot 4.7) turns a downward camera into a flow sensor and forum posts mention high altitude use; Not opened its code. Flow gives velocity, so position drifts (forum, search).
2. GSoC / ngps_flight: see above.
3. VIO: all need a real IMU feed. VINS-Mono is the only one seen with a RS readout key in config; OpenVINS upstream has time-offset calibration but no RS key was seen; none are small enough to set up in 4 days with no hardware and no IMU data at matching rate (INFERENCE). Our notes say hover excites calibration weakly.
4. Calibration: Gyroflow (RS sync code in rs_sync.rs, lens DB under CC0), CRISP (offset from gyro vs video), gyroflow/rollingshutter (LED bands), telemetry-parser (log reader). OpenICC (urbste/OpenImuCameraCalibrator, AGPL-3.0, 2026-05-10) calibrates IMU-camera rotation and offset; search says its readout estimate "does not work well".
5. Targets: AprilRobotics/apriltag is the standard detector. ArduPilot expects LANDING_TARGET from a companion (ardupilot AC_PrecLand_MAVLink.cpp). Found no repo for LED or night targets with code.
6. Low light / analog / OSD: nothing for tracking. Only OSD tools that draw (openipc/msposd, walksnail-osd-tool; search, not opened).

## Not found
- Any open repo for keyframe-anchored hover with a rolling-shutter analog camera.
- Open code for OSD masking, frozen-frame detection, or deinterlace-aware tracking.
- LED-beacon night tracking code with a MAVLink output.
- Any Python phase-correlation hover repo (search found papers and OpenMV docs, no repo).
- Rolling-shutter support in Basalt, Kimera, ROVIO, msckf_vio: not checked in code.
- ModalAI OpenVINS fork: not opened.
