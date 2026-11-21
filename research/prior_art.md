# Prior art for camera-based position hold at 50-300 m (downward, rolling shutter, no GNSS)

Date of research: 2026-11-15. Method: web search plus page fetch plus `gh` licence checks. Statements come from the cited source unless marked "INFERENCE" (low confidence). Items that I could not find are in section 10.

Depth note: several sources were fetched only as search summaries or abstracts (large PDFs failed to load). Those items are marked "(abstract/summary only)". No paper full text was checked for numbers except where stated.

---

## 1. ArduPilot GSoC 2024 "High Altitude Non-GPS Navigation" (Sanket Sharma)

Sources:
- Start thread: https://discuss.ardupilot.org/t/gsoc-2024-high-altitude-non-gps-navigation/120253
- Wrap-up: https://discuss.ardupilot.org/t/gsoc-2024-wrapping-up-high-altitude-non-gps-navigation/122905
- Follow-on (2025): https://discuss.ardupilot.org/t/transformer-optimization-based-high-altitude-gps-denied-fusion/134181
- Code: https://github.com/snktshrma/ap_nongps (no licence field on GitHub), https://github.com/snktshrma/ngps_flight (MIT), https://github.com/snktshrma/ardupilot_gazebo_ap
- Paper (2026): https://arxiv.org/abs/2607.18936

Method (2024 GSoC):
- Not a keyframe tracker like ours. It matches live frames against a pre-flight reference map (stitched aerial or satellite image) with SIFT-FLANN (AKAZE-FLANN as alternative). Absolute position goes to ArduPilot as vision position messages.
- Wrap-up version adds a modified Lucas-Kanade optical flow (pyramid plus RANSAC) for high-rate output. Feature matching resets drift. It needs a gimbal-stabilised, bottom-facing camera. Output rate was tuned to meet the EKF minimum of about 4 Hz.
- EXIF-based geolocation reset via MAV_CMD_EXTERNAL_POSITION_ESTIMATE was under test.

Results:
- Datasets at about 75 m; tests above 100 m "similar" (the author's words). Hardware tests were planned from 50 m to above 100 m at 2-3 m/s.
- No numeric drift or accuracy figure in the 2024 threads. Simulation in Gazebo only for the original post.

Problems reported:
- Lucas-Kanade degrades during fast rotation. Feature-match correction is weak under large perspective change.
- Cloud cover (a commenter said camera-only drifts). SIFT is costly. ORB is less accurate in poor light.
- Setup issues: pypopsift segfaults (author moved to OpenCV CUDA SIFT), missing parameter file, GStreamer, a GLOBAL_POSITION_INT wait.

Continuation (2025 and 2026): YES.
- May 2025 thread: SuperPoint plus a lightweight transformer matcher gives absolute position. Local VIO (modified VINS, 100 Hz IMU plus KLT at about 20 Hz) plus a global optimisation stage. State is 4 DOF; altitude and yaw come from the EKF (baro and compass). No loop closure, because "drift can reach tens of metres" at height. Map-matching tests at 150 m. Jetson Orin NX 8 GB runs inference above 10 Hz. Adding VIO/VPS into AP_NavEKF3 caused watchdog errors. Open-source release announced 2025-10-25. A related ArduPilot PR #31134 syncs IMU data with camera triggers (needs flight validation).
- Newer release (LightGlue + SuperPoint + VINS-Fusion, external UKF) is in ngps_flight (MIT).
- arXiv 2607.18936 (abstract only): RMSE 2.94 m, worst ATE 6.04 m at 150 m AGL and 2 m/s, 60-150 m AGL, five flight sequences, 3.5x better than standalone monocular VIO, real time on Jetson Orin NX.
- 2025 slide summary (via search summary): simulated mono VIO alone gave about 20-22 m ATE. https://discourse.ros.org/t/aerial-robotics-meeting-june-19th-2025/44181
- Separate 2025 GSoC project (3D camera/LiDAR map matching): https://discuss.ardupilot.org/t/gsoc-25-non-gps-position-estimation-using-3d-camera-and-pre-generated-map-final/138513 (only the start read).

Relevance: These systems need a pre-flight map and are absolute-position systems (metres of error at 150 m). Our keyframe hold is relative with 0.4-1.3 m simulated RMS. Not the same task, so reuse is limited to the ArduPilot interface and the lessons: pace EKF input rate, send position without loop-closure jumps, timestamp at capture.

## 2. Downward-camera hold methods and low-texture ground

Findings (no head-to-head benchmark found):
- Phase correlation / Fourier-Mellin (FMT): the eFMT paper (Remote Sensing 2021) says feature-based and direct VO "usually do not perform well" on low-texture surfaces such as lawn and asphalt, and that FMT odometry is more accurate than ORB/AKAZE there. Limit: FMT cannot handle multi-depth scenes (roof plus ground). https://www.mdpi.com/2072-4292/13/5/1000 (via search summary; page returned 403).
- Phase correlation plus IMU plus sonar for absolute velocity from a downward camera: IET Image Processing 2024 (Deng et al.). https://doaj.org/article/5d854cf8fa354f6991998d141a461e3d (summary only; accuracy not read).
- IMAV 2009 phase-correlation paper says phase correlation beat other flow methods given the motion, frame rate and compute. https://www.imavs.org/papers/2009/80_EMAV09.pdf (summary only)
- MDPI Sensors 2019 hybrid: Farneback flow for velocity, multi-scale phase correlation for position. https://mdpi.com/1424-8220/19/6/1380/html (summary only)
- Dense direct, homography-based: RaD-VIO (photometric homography cost plus IMU term, rangefinder height). Claims better than other downward methods; abstract gives no numbers. https://arxiv.org/abs/1810.08704
- SVO: built for a downward camera plus IMU. No altitude drift number found. https://infoscience.epfl.ch/record/199740 ; code GPL-3.0.
- Sparse flow (ArduPilot docs) is susceptible on bare, homogeneous ground. Wind-moved grass corrupts flow (tester report in https://discuss.ardupilot.org/t/high-altitude-optical-flow-testers-needed/137124).
- Snow: no source found.
- Downward-VIO paper 2023 says drift is large when the MAV is in the air; it proposes dynamic weighting of visual residuals. https://arxiv.org/pdf/2311.09622 (summary only)

Conclusion (INFERENCE, low confidence): for low texture, whole-image methods (phase correlation, ECC/direct alignment) are the better-supported choice than sparse corners. A hybrid (corners when texture is good, correlation when not) matches what the sources suggest.

## 3. Rolling-shutter correction with gyro data

- The method is known. Karpenko, Jacobs, Baek, Levoy (Stanford 2011) correct rolling shutter and stabilise video with gyroscope data only. Their calibration, from one video of about 10 s, recovers focal length, rolling-shutter duration, gyro-to-camera delay, gyro drift and axis alignment. Demonstrated on iPhone 4. https://graphics.stanford.edu/papers/stabilization/ (summary via search; page fetch failed on certificate)
- Ovren and Forssen, ICRA 2015: joint calibration of a rolling-shutter camera and gyro (time offset and scale, relative rotation, gyro bias). They state that RS cameras need more accurate time sync and that it can only be found by modelling the RS. Code: CRISP, GPL-3.0. Assumes known readout time. https://github.com/hovren/crisp ; paper https://people.isy.liu.se/cvl/perfo/papers/ovren_icra15.pdf (not fetched)
- Karpenko-style warning: a small error in delay or shutter duration can make the correction fail (the readout is only milliseconds).
- No accuracy figure in px or ms was obtained from any source. Unknown.
- Measuring readout without lab gear:
  - Flashing LED bands: known pulse width gives band size, known frequency gives band count (Gyroflow docs). https://old-docs.gyroflow.xyz/tech/rollingshutter/ ; a hobby build at about 2 kHz counts lines as roughly ms, claimed to about 1 ms: https://blog-6de.pages.dev/posts/2025/measure-camera-sensor-readout (summary only). Measure in the exact mode used.
  - Gyro-vs-video fit: CRISP (above); Gyroflow GPL-3.0 https://github.com/gyroflow/gyroflow (RS readout is a user input; it estimates sync).
  - Online in a VIO filter: OpenVINS self-calibration estimates readout time; observable under fully excited 6-axis motion. https://arxiv.org/abs/2201.09170 (abstract only). Hovering is a weak excitation, so INFERENCE: do not expect to learn readout in hover.
  - Offline continuous-time: https://arxiv.org/abs/2108.07200 (Kalibr fork https://github.com/JzHuai0108/kalibr). Needs a calibration target and IMU.
  - A turning fan or hand-shake sweep: no source found. (A hand shake against gyro is the CRISP/Karpenko method.)
- Analog caveat (INFERENCE): the sensor readout time is not the video line time. Check where the "rolling" happens. For a CVBS camera, scan-out may follow the 15.6 kHz line time of the video standard, but I found no source.

## 4. Vibration ("jello")

- Sources are vendor blogs and forums, not measurements: gyro noise lines at about 80-350 Hz on 5-inch builds; 150-300 Hz on 2300 KV 4S; frame resonance about 100-250 Hz; motor/prop above about 250 Hz. Motor frequency = RPM/60 (15000 RPM is about 250 Hz). https://www.unmannedtechshop.co.uk/blogs/knowledge-base/fpv-drone-jello-fix-vibration-checklist-every-build , https://blog.uavmodel.com/6015-2/ , https://oscarliang.com/?p=38833
- No amplitude figures (g or pixels) found. No 7-10 inch figures found. Props of larger size run at lower RPM, so lower frequency (INFERENCE from RPM/60).
- Mitigation named: prop balancing, soft mount, dynamic notch (Betaflight), short exposure. No source tying these to tracking accuracy.
- Gyro rate: Karpenko and Ovren use a gyro stream faster than the frame rate. For vibration above about 100 Hz you need the gyro at 1 kHz or more (INFERENCE; Nyquist). On the flight controller side, the STM32 gyro logs may be filtered; ask for raw/lightly filtered gyro (INFERENCE).
- An inertial rate at the image-row time is only useful if the vibration is seen by the gyro and the camera equally. If the camera mount flexes differently from the FC (soft-mounted FC), they differ (INFERENCE).

## 5. Analog FPV video as vision source

- Interlacing: NTSC about 60 fields/s (16.7 ms), PAL 50 fields/s (20 ms). A forum thread says deinterlace before processing. https://oscarliang.com/fpv-camera-latency ; https://www.digitalfaq.com/forum/video-capture/10155-problem-capturing-fast-print.html
- Latency: camera-only measurements 25-45 ms per camera (Oscar Liang). Extra 18-20 ms (one frame) or about 40 ms (two) seen in some cameras. Capture chain example 53 ms (decoder 33 ms + USB 20 ms) https://www.sensoray.com/downloads/whitepaper_latency.pdf . Measure the full chain with a light flash.
- Processing in camera (WDR, DNR) can add delay. CVBS limits sharpness. https://oscarliang.com/fpv-camera-settings
- Auto exposure/gain, OSD burn-in: only product pages found. No source on tracker handling. Requirements (INFERENCE, low confidence): mask OSD pixels; use photometrically invariant matching (normalised cross-correlation or gradient-based); treat each field as its own sample or use one field at 50/60 Hz; time-stamp at capture and apply ArduPilot delay parameter.
- ArduPilot side: timestamps should reflect capture time; a source says sensor lag may be 10-100 ms; use VISO delay parameter; vision source needs `EK3_SRC1_POSXY=6` plus VISO_TYPE. One user found position-only messages gave no fused result and added velocity. https://discuss.ardupilot.org/t/vision-position-estimate-not-being-able-to-inspect-in-mission-planner/111582 and related threads (forum anecdotes). Check the firmware (ArduPilot or PX4) on the APB: unknown.

## 6. Attitude limit

- No source with roll/pitch accuracy for PX4 or ArduPilot EKF in hover. Searches gave only forum anecdotes (EKF3 roll/pitch drifting by about 10 degrees in one case where hover was fine; suspected accel bias or yaw error), and non-PX4 papers (RMSE roll 0.5-1.2 deg, pitch 1.5-3 deg for complementary filters). https://discuss.ardupilot.org/t/ahrs2-ekf2-ekf3-attitude-and-altitude-estimation/36001 ; https://www.ncbi.nlm.nih.gov/pmc/articles/PMC11280949/ (summary)
- The `h * e` relation is the user's own and my search did not find a source for it. 0.5 degrees = 8.7 mrad gives 0.87 m at 100 m, 2.6 m at 300 m (arithmetic, not sourced). This agrees with the sim errors of 0.4-1.3 m only if the sim attitude is near-perfect; the sim shares the geometry so it hides attitude error.
- Beating the limit with the camera: Zhao et al. (JIRS 2015) estimate velocity, attitude and IMU biases jointly from flow plus homography. https://shiyuzhao.westlake.edu.cn/13-2015JIRS.pdf (summary only). A homography between keyframe and current frame contains the relative rotation plus the plane normal; INFERENCE: the horizon is not needed, because for a flat ground the plane normal gives gravity direction as seen from the camera. That is the same information as tilt, but only relative to the ground plane (not gravity). Tilt from the ground plane equals the true tilt only when the ground is level. No source quantifies this for 100+ m.

## 7. Height from image scale

- Monocular scale is not observable without a metric reference. Systems use rangefinder (RaD-VIO), IMU (needs excitation), or barometer. https://arxiv.org/abs/1810.08704
- Observability analysis for IMU + downward camera + baro + compass: https://shiyuzhao.westlake.edu.cn/13-2015JIRS.pdf (summary only).
- Altitude from optic flow with IMU via EKF: indoor, about 2.5 cm RMS near 40 cm height (Chirarattananon 2018). https://ris.bme.cityu.edu.hk/wp-content/papercite-data/pdf/chirarattananon2018direct.pdf (summary only). Not comparable to 100+ m.
- No source for tracked-point scale change vs keyframe at 100+ m. INFERENCE (low): the scale ratio of keyframe vs current gives *relative* height h/h0 with no absolute error but with strong coupling to rotation and ground relief; the signal is dh/h, so a 1 percent scale change at 200 m is 2 m, and it is hard to measure at the pixel level. Test in sim before relying on it. Use it as a baro cross-check, not as a replacement.

## 8. Open-source projects (licences by `gh api`, 2026-11-15)

| Project | Use | Licence |
|---|---|---|
| ngps_flight (snktshrma) https://github.com/snktshrma/ngps_flight | satellite match + VIO fusion to ArduPilot | MIT |
| ap_nongps https://github.com/snktshrma/ap_nongps | GSoC 2024 code | none stated |
| ArduPilot https://github.com/ArduPilot/ardupilot | EKF3 external nav, PR #30718 optical flow above rangefinder range via SRTM | GPL-3.0 |
| ArduPilot/companion | GSoC support | GPL-3.0 |
| PX4-Autopilot, PX4-OpticalFlow | flow | BSD-3-Clause |
| OpenVINS | RS readout self-calibration | GPL-3.0 |
| VINS-Fusion / VINS-Mono | VIO | GPL-3.0 |
| Basalt | VIO/calibration | BSD-3-Clause |
| ORB-SLAM3 | SLAM | GPL-3.0 |
| rpg_svo, rpg_svo_pro_open | downward VO | GPL-3.0 |
| rovio, okvis, Kalibr | VIO / calibration | NOASSERTION (check repo) |
| CRISP | gyro/RS calibration | GPL-3.0 |
| Gyroflow | gyro stabilisation, RS | GPL-3.0 |
| rscalibration (OpenDroneMap) | RS calibration | AGPL-3.0 |
| kontiki | continuous-time | MIT |

No open-source project found that does keyframe-anchored 100-300 m hover with a low-cost rolling-shutter camera.
ArduPilot high-altitude flow thread (no drift numbers, 30 m test only): https://discuss.ardupilot.org/t/high-altitude-optical-flow-testers-needed/137124

## 9. Ranked changes the sources support

1. Use gyro-rate rotation at row time plus calibrate gyro-camera time offset and readout time with CRISP/Karpenko-style fit (Karpenko, Ovren: sync error breaks the correction). Ask the organisers' log for raw gyro rate; test sim with a 20-50 ms offset error. Strong support.
2. Add a whole-image fallback (phase correlation / ECC) for low-texture patches; keep corner tracking for textured scenes. Support: eFMT, phase-correlation papers (medium).
3. Time-stamp at capture, estimate analog chain latency with a light flash, and feed a delay parameter to the EKF. Send velocity with position if EKF does not fuse position alone. Support: ArduPilot forum, Sensoray, Oscar Liang (medium).
4. Handle interlace: use fields at their own times (or one field), and mask the OSD. Support: weak (forum plus INFERENCE).
5. Do not trust the sim's attitude: inject attitude bias and noise (0.1-1 degree), different geometry code or conventions; add a homography-based tilt cross-check against the ground plane. Support: Zhao et al. (joint estimation), no source for numbers. Height from scale: test as a cross-check only.

## 10. Unknowns

- Numeric accuracy of gyro-based RS correction (px or ms).
- Roll/pitch accuracy of PX4/ArduPilot EKF in hover.
- Vibration amplitudes; 7-10 inch frequencies.
- Whether the Justice D camera rolls at the video line time; its readout time.
- Behaviour of analog AE/AGC for a tracker; OSD handling sources.
- Quantified accuracy on grass, snow, asphalt for any method.
- Height-from-scale at 100+ m.
- Licence of ap_nongps (none stated).
- Quantified 2024 GSoC results.
- Firmware on the APB (PX4 or ArduPilot) and whether it accepts external vision.
