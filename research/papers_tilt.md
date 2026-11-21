# Papers: cutting tilt-driven drift in camera hover (research log, 2026-11-16)

## 1. Zhao, Lin, Peng, Dong, Chen, Lee - Vision-aided Estimation of Attitude, Velocity, and Inertial Measurement Bias for UAV Stabilization (J Intell Robot Syst, ~2015/2016)
URL: https://shiyuzhao.westlake.edu.cn/13-2015JIRS.pdf  (read full text, preprint)
- Setup: downward camera (works at 160x120), barometer, IMU, compass; homography fed DIRECTLY into an EKF (no decomposition). Roll/pitch enter through the plane normal term N (the third row of R, i.e. gravity direction in camera frame). Yaw is not observable from the homography.
- Key finding for us: observability of roll/pitch is WEAK when speed is slow or height is large (term T*N^T/d vanishes; two views are nearly the same bearing). "Still observable but converges slowly". Bias drift-free only when UAV speed is non-zero.
- Accuracy: real flight shown as plots only; did not extract numeric degree error from the text (not verified).
- Take: tilt from plane homography needs baseline/parallax. In a pure hover it gives little tilt information beyond what the gyro gives. Feed homography entries to the filter instead of decomposing; use as a slow bias corrector, not as a fast tilt source.

## 2. Qin and Shen - Online Temporal Calibration for Monocular Visual-Inertial Systems (IROS 2018)
URL: https://arxiv.org/abs/1808.00692  (read full text)
- Method: time offset td as a state; feature velocity on the image plane (approx. constant over a few ms) shifts each observation by td*velocity; jointly optimised with poses (VINS-Mono code).
- Accuracy: simulation offsets 5/15/30 ms, 100 trials, "low RMSE" (table numbers not extracted); real data std of estimate 0.07-0.16 ms across exposure settings vs Kalibr 0.13-0.27 ms (global shutter fisheye, 100x133 deg). VINS-Mono WITHOUT calibration: tolerance only about 6 ms on EuRoC, error grows parabolically with offset. This matches our 10-20 ms observation.
- Needs "sufficient accelerating and rotating motion" in their tests. Note: timestamp = mid-exposure, so offset = fixed delay + half exposure time (slope ~0.5 vs exposure).
- Take: offset should be modelled as fixed delay + 0.5*exposure; estimate by matching image-derived rotation/flow rate to gyro rate.

## 3. Yang, Geneva, Zuo, Huang - Online Self-Calibration for Visual-Inertial Navigation Systems: Models, Analysis and Degeneracy (IJRR/TRO ~2022/2023)
URL: https://arxiv.org/abs/2201.09170  (read full text, main sections)
- States: camera-IMU time offset td AND rolling-shutter readout time tr, extrinsics, intrinsics, in an MSCKF (OpenVINS).
- Observability result (the important part for hover): td is UNOBSERVABLE under constant local angular velocity + linear velocity, and under constant angular velocity + constant global acceleration. A hover is close to this. Readout time converged slower in planar motion "probably due to lack of motion in the beginning". tr is also tied to feature spread across rows ("hundreds of features from different rows").
- Real result: estimated readout ~30 ms for 1280x1024 (about 29 us/row). Accuracy in ms of td not given as a single figure in what was read (not verified).
- Take: do NOT expect to estimate td online during a still hover. Estimate it in a short excitation manoeuvre (roll/pitch wiggle at start, or during takeoff climb), then freeze it. Row time can be fixed from the camera's readout (PAL/NTSC analog: field time ~16.7/20 ms).

## 4. Engel, Sturm, Cremers - Camera-Based Navigation of a Low-Cost Quadrocopter (IROS 2012)
URL: https://cvai.cit.tum.de/_media/spezial/bib/engel12iros.pdf  (read full text)
- Keyframe monocular SLAM (PTAM-style) + EKF with IMU/height, AR.Drone, low-res camera. Scale estimated by ML fit against ultrasonic height/odometry.
- Hold accuracy: RMSE 4.9 cm indoor, 7.8 cm, 18.0 cm outdoor (position hold). Flight height not stated in what was read; it is a small indoor/low-altitude drone, so angle = error/height is NOT comparable to our 20-122 m (not verified, no angle given).
- Drift: odometry-only drift 2.1 m (figure) and 6 m (longer) in the test; with the keyframe map, the landing was within 15 cm.
- Take: confirms the keyframe-anchor idea removes drift; error figures show ~0.05-0.2 m at low height, so the noise floor is not the vision itself.

## 5. PX4 EKF2 source: gravity_fusion.cpp (not a paper; answers question 4)
URL: https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/src/modules/ekf2/EKF/aid_sources/gravity/gravity_fusion.cpp  (read the code)
- Verified: gravity-vector (accelerometer direction) fusion runs only if `!isHorizontalAidingActive()` and accel norm is within 0.9-1.1 g, and tilt_align is done. So as soon as external-vision position/velocity is fused (horizontal aiding), the direct tilt reference is OFF and roll/pitch can only be corrected through the position/velocity innovations. That matches our finding. Parameter: EKF2_GRAV_NOISE (default not checked), EKF2_IMU_CTRL bit for gravity.
- PX4 VIO docs (https://docs.px4.io/v1.16/en/computer_vision/visual_inertial_odometry) say that inconsistent external vs onboard data confuses the EKF and "it is more reliable to just use vision velocity" (from the search snippet only; page not read in full).
- Take: for the tilt to stay good, the fused vision measurement must be consistent (right timing, covariance not too tight). Looser EV position noise (EKF2_EVP_NOISE) or sending velocity makes the vision pull the attitude less. Not verified in any paper.

## 6. Andre and Caron - Photometric Visual Gyroscope for Full-View Spherical Camera (CVPR Workshops OmniCV 2022)
URL: https://openaccess.thecvf.com/content/CVPR2022W/OmniCV/papers/Andre_Photometric_Visual_Gyroscope_for_Full-View_Spherical_Camera_CVPRW_2022_paper.pdf  (read text of main sections)
- Direct (photometric) alignment of whole images on the unit sphere; rotation only (3 DoF), gradient-descent-like update.
- Accuracy: mean error 0.0981 deg (std 0.0442 deg) between successive images with 2.5 deg true rotation, full-resolution dual-fisheye 360-degree images, static scene, PanoraMIS dataset. With down-sampled images the mean error rises to 0.79 deg. Convergence domain stated in the text +-45 deg (hybrid), +-12.5 deg for plain PVG (from search snippet). Compute 883 ms per pair on their code.
- Conditions: pure rotation test (the text says it also works with translation for a single-axis case - from a snippet, not verified); full 360 view; high-res. Our camera is 130 deg, 360x288, so expect much worse than 0.1 deg.
- Take: dense photometric alignment on the unit sphere (after undistortion with the fisheye model) is the best-documented way to get a rotation to <0.1 deg; resolution matters: 10x fewer pixels gave 8x larger error.

## 7. Panahandeh, Hutchinson, Handel, Jansson - Planar-Based Visual Inertial Navigation: Observability Analysis and Motion Estimation (J Intell Robot Syst 2016)
URL: https://faculty.cc.gatech.edu/~seth/ResPages/pdfs/Panahandeh2016_Article_Planar-BasedVisualInertialNavi.pdf  (read abstract and observability section headings)
- IMU + monocular camera (downward-looking use case) observing features on a horizontal plane; closed-form planar measurement model; UKF.
- Result: only three unobservable directions: global translation parallel to the plane and rotation about gravity (yaw). So roll, pitch and height are observable. Plane constraint makes the vertical translation observable.
- Accuracy in degrees: not extracted (not verified).
- Take: theory supports that a planar-scene model with the IMU makes tilt observable, as long as the camera sees motion (their observability is for a system with excitation; the other paper (item 1) says weak with low speed/high height).

## 8. Photogrammetry papers on homography decomposition with long focal length (snippets only, not read in full)
URLs: https://WWW.ISPRS.org/proceedings/XXXV/congress/comm3/papers/231.pdf ; https://goobi.tib.eu/viewer/!fulltext/1663685223/17
- Search snippets say: with narrow field of view or long focal length, full decomposition (rotation + translation + normal) is poor; fixing the rotation to the known value (from a sensor) gives more stable results. This supports the FOV argument in the other direction: a 130 deg lens is the opposite regime, but no paper found measures error versus FOV (not verified).

## 9. Continuous-homography hover (Lagadic group ICRA 2012 hover from optical flow, Hua et al. observer) - snippets only
URL: https://www.irisa.fr/lagadic/pdf/2012_icra_grabe.pdf (HTTP 403 on fetch; only the search snippet seen)
- Hover from continuous homography (flow + gyro derotation, scaled velocity). Not verified: no accuracy numbers read.

## 10. Ovren and Forssen - Gyroscope-based video stabilisation with auto-calibration (ICRA 2015) - snippet only
URL: https://people.isy.liu.se/cvl/perfo/papers/ovren_icra15.pdf (download failed; snippet only)
- Estimates time offset, time scale, relative pose, gyro bias for a wide-angle rolling-shutter camera + gyro from the video itself, offline non-linear least squares. Accuracy not read (not verified).

## 11. ArduPilot docs (not papers)
- https://ardupilot.org/copter/docs/common-non-gps-to-gps.html : ExternalNav source value 6; keep EK3_SRC_OPTIONS FuseAllVelocities = 0. VISO_POS_M_NSE / VISO_YAW_M_NSE set minimum noise passed to EKF. No document found that says EKF3 tilt degrades under ExternalNav (not verified; ArduPilot has no PX4-style gravity-fusion switch that was found).
