# Control research for camera-only hover hold (written 2026-11-16)
Format per source: URL, what was read (full text / abstract / snippet only), numbers, what to take.

## S1. ArduCopter 4.6.3 source, position controller parameters (AC_PosControl.cpp, AC_Loiter.cpp)
URLs: https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.6.3/libraries/AC_AttitudeControl/AC_PosControl.cpp
      https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.6.3/libraries/AC_WPNav/AC_Loiter.cpp
Read: full source (tag Copter-4.6.3). Note: master renamed these to PSC_NE_*; 4.6 uses the names below.
- PSC_POSXY_P: default 1.0 for Copter, range 0.5-2.0 (source comment). Converts distance to desired speed.
- PSC_VELXY_P default 2.0 (range 0.1-6.0), PSC_VELXY_I default 1.0 (range 0.02-1.00), PSC_VELXY_D default 0.25 (range 0-1),
  PSC_VELXY_IMAX 1000 cm/s/s, PSC_VELXY_FLTE 5 Hz, PSC_VELXY_FLTD 5 Hz, PSC_VELXY_FF default 0.
- PSC_JERK_XY default 5 m/s/s/s (range 1-20): jerk limit of the input shaping.
- PSC_ANGLE_MAX: max lean angle (0 = use ANGLE_MAX).
- LOIT_ACC_MAX: "Higher values cause the copter to correct position errors more aggressively." Parameters.cpp conversion table in
  4.6.3 lists 500 cm/s/s as default (AC_Loiter.h LOITER_ACCEL_MAX_DEFAULT 500). LOIT_SPEED, LOIT_BRK_* only matter with pilot sticks.
Take: with no pilot input, Loiter holds via PSC_POSXY_P -> PSC_VELXY_PID -> accel. The corrections are limited by LOIT_ACC_MAX
and PSC_JERK_XY; with a 100-170 ms old fix a high POSXY_P cannot be used (see S2/S5 for delay argument). Defaults are not "tight";
there is no official statement in this source of recommended tight-hold values (not verified elsewhere yet).

## S2. ArduCopter 4.6.3 source, external-nav input path (AP_VisualOdom, AP_NavEKF3)
URLs: https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.6.3/libraries/AP_VisualOdom/AP_VisualOdom.cpp
      https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.6.3/libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp
      https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.6.3/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp
      https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.6.3/libraries/AP_NavEKF3/AP_NavEKF3.cpp
Read: full source files (grep + sections).
Facts (verified in source):
- VISO_DELAY_MS: "Visual odometry sensor delay relative to inertial measurements", range 0-250 ms. Passed as delay_ms to EKF3
  for both position (writeExtNavData) and speed (writeExtNavVelData). EKF3 stores them and "recall"s at the delayed IMU time (fusion at delayed horizon).
- VISO_POS_M_NSE default 0.2 m, range 0.1-10. The posErr in the message is clamped to a MINIMUM of this value
  (constrain_float(posErr, VISO_POS_M_NSE, 100)). Inside EKF3 R is further clamped to 0.01-10 m. So our covariance only raises noise, never lowers it below VISO_POS_M_NSE.
- VISO_VEL_M_NSE default 0.1 m/s, range 0.05-5.0. For VISION_SPEED_ESTIMATE the EKF uses the PARAMETER VISO_VEL_M_NSE as the noise, not a covariance from the message
  (handle_vision_speed_estimate calls writeExtNavVelData(vel, get_vel_noise(), ...)). Our per-sample speed covariance is ignored on this path. (Odometry MAVLink message path not checked.)
- VISO_QUAL_MIN: samples below this quality are dropped.
- EKF3 fuses extnav velocity when EK3_SRC1_VELXY = 6 (ExternalNav) (useVelXYSource(EXTNAV)); position when EK3_SRC1_POSXY = 6.
  Velocity R clamped 0.05-5 m/s, plus an acceleration-scaled term (extNavVelVarAccScale).
- EK3_DRAG_BCOEF_X/Y: default 0 (off); "Set to a positive value > 1.0 to enable. A starting value is the mass in kg divided by the frontal area."
  EK3_DRAG_MCOEF default 0; "if flying at 15 m/s ... rotor induced drag acceleration of 1.5 m/s/s, then EK3_DRAG_MCOEF = 0.1".
  EK3_DRAG_M_NSE default 0.5. Purpose: wind estimation with drag fusion (needs a speed/position aiding source; wind is estimated while moving; in a perfect hover the drag signal is small).
Take: (a) set VISO_DELAY_MS to the measured camera-to-fix age (100-170 ms), within 0-250. (b) VISO_POS_M_NSE is the floor of position trust;
  lowering it toward 0.1 only matters if our own covariance is also small. (c) Speed noise is a parameter; set VISO_VEL_M_NSE honestly.

## S3. PX4 main source: position-control and EKF2 parameter definitions
URLs: https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/src/modules/mc_pos_control/multicopter_position_control_gain_params.yaml
      https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/src/modules/ekf2/params_external_vision.yaml
      https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/src/modules/ekf2/params_drag.yaml
Read: full YAML files (main branch, fetched 2026-11-16; not pinned to a release).
- MPC_XY_P default 0.95 (0-2), m/s per m. MPC_XY_VEL_P_ACC default 1.8 (1.2-5). MPC_XY_VEL_I_ACC default 0.4 (0-60). MPC_XY_VEL_D_ACC default 0.2 (0.1-2).
- EKF2_EV_CTRL bitmask: bit0 horizontal position, bit1 vertical position, bit2 3D velocity, bit3 yaw. Default 0.
- EKF2_EV_DELAY: "Vision Position Estimator delay relative to IMU measurements", default 0, range 0-300 ms (reboot required).
- EKF2_EV_NOISE_MD: 0 = noise from message, EKF2_EVP_NOISE / EKF2_EVV_NOISE are a lower bound (defaults 0.1 m, 0.1 m/s; min 0.01). 1 = use the parameters.
- EKF2_EVP_GATE default 5 SD, EKF2_EVV_GATE default 3 SD.
- EKF2_DRAG_CTRL (bool, default 0): wind estimate from drag; EKF2_BCOEF_X/Y default 100 kg/m^2 (0-200); EKF2_MCOEF default 0.15 1/s; EKF2_DRAG_NOISE default 2.5.
  "Only use on vehicles that have their thrust aligned with the Z axis and no thrust in the XY plane."
Take: PX4 lets the message covariance set the noise (good: lets our own quality estimate matter); ArduPilot (S2) clamps from below by a parameter and ignores speed covariance.

## S4. Hentzen, Stastny, Siegwart, Brockers: "Disturbance Estimation and Rejection for High-Precision Multirotor Position Control", IROS 2019 (arXiv 1908.03166)
URL: https://arxiv.org/abs/1908.03166  (PDF read in full with pdftotext)
Setup: quadrotor on Odroid XU4, indoor Vicon (motion capture) for state; EKF force-disturbance estimator at 100 Hz (UKF max 40 Hz, nearly same accuracy); fan-made gusts 3, 10, 12 m/s while landing through the wind field.
Numbers (Table I, max horizontal deviation): 10 m/s gust: PID 17 cm; PID + disturbance compensation 10 cm (-41%); MPC 19 cm; MPC+comp 8 cm.
 12 m/s: PID 45 cm; PID+comp 23 cm (-48%); MPC+comp 17 cm (-62%); MPC+slack+comp 10 cm (-78%).
 3 m/s gust: PID and MPC without compensation have the same deviation; "with disturbance compensation, the wind disturbance is almost completely rejected" (no number in text).
 Ground effect: with PID, deviation 5-10 cm; with disturbance feed-forward within 2.5 cm.
Limits stated by authors: gust is not completely rejected because of estimator convergence time; raising estimator noise gain speeds convergence but gives high-frequency roll/pitch oscillation,
 so "the requirement of smooth control imposes an upper bound" on the estimator speed.
Caveat: position came from Vicon (low noise, low delay); estimator-based feed-forward is NOT a parameter change on ArduCopter/PX4.
Take: a disturbance feed-forward (an I-term that is fast, or a drag/wind model) roughly halves peak gust error (-41..-48% with a plain PID). On ArduCopter the closest
 parameter-only equivalents are higher PSC_VELXY_I (fast integral) and EK3 drag wind estimation; no paper tested those.

## S5. Smeur, de Croon, Chu: "Cascaded Incremental Nonlinear Dynamic Inversion for MAV Disturbance Rejection", Control Engineering Practice 2018 (arXiv 1701.07254)
URL: https://arxiv.org/abs/1701.07254   DOI 10.1016/j.conengprac.2018.01.003  (PDF read in full with pdftotext)
Setup: Parrot Bebop (small, Paparazzi autopilot), 10 m/s wind tunnel (TU Delft Open Jet Facility). Position from Optitrack but DOWN-SAMPLED TO 4 Hz to imitate GPS; control loop 512 Hz.
Numbers: wind tunnel, INDI 21 cm max position deviation on average entering/leaving the 10 m/s flow; a PID with the same P/D gains (P 0.65 (m/s)/m, D 0.20 rad/(m/s)) and an integral term 151 cm.
 Outdoor (external u-blox GPS), takeoff in wind: INDI 0.24 m average max position error vs 0.85 m for PID (Figs. 13-14). Later in the flight the average error is the same for both; some INDI runs had relatively large errors.
Author statement: because INDI uses the accelerometer, gusts are rejected the same way even with a coarse low-rate position.
Take: accelerometer-based incremental feedback gives about 7x smaller gust deviation than integral PID in this test, and works with a 4 Hz position feed (so delay/low rate does not block it). Not reachable by parameters in ArduCopter/PX4
 (no INDI in either). Closest: ArduCopter PSC_ACCZ is vertical only; there is no horizontal acceleration feedback parameter. Not verified for either autopilot.

## S6. Engelsman and Klein: "C-ZUPT: Stationarity-Aided Aerial Hovering", arXiv 2507.09344 (July 2025)
URL: https://arxiv.org/abs/2507.09344   (full PDF read with pdftotext)
What it is: zero-velocity update in hover. A window of estimated velocity and specific force is tested against thresholds (delta, window K); when "quasi-static", a zero-velocity pseudo-measurement goes into an LQG estimator. Position updates are sparse in the tests (Gamma = update ratio).
Evidence type: simulation only (a quadrotor model with process noise to emulate wind; text mentions no flight hardware). Not verified on a real vehicle.
Numbers: normalised to the unaided estimator over ten runs, at low update rates, C-ZUPT gives tracking error to 28.8% of unaided (the paper says "lower tracking error (28.8%)"; table values are normalised to the unaided result), attitude error 21.6%,
 control effort 31.9%, saturation time 76.9%. In another summary line: 30% better disturbance rejection and about 6% energy saving. At full update rate (Gamma 1.0) gain is small (3-7% in some metrics).
 Authors warn: a permissive threshold gives false positives during brief transients ("need for careful tuning"); with very sparse updates (Gamma = 0.001) both diverge.
Take: a zero-speed aid helps most when position fixes are sparse and the estimate drifts between fixes; it can HURT when stationarity is declared during a gust push or a slow drift (false positive). Our feed is 10-25 Hz, so the expected gain is small
 and the risk (declaring zero speed while the vehicle is really drifting slowly under a vision bias) is real. No flight test with vision found.

## S7. ArduCopter 4.6.3 source: Precision Loiter and PLND_* (mode_loiter.cpp, AC_PrecLand.cpp); official doc page
URLs: https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.6.3/libraries/AC_PrecLand/AC_PrecLand.cpp
      https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.6.3/ArduCopter/mode_loiter.cpp
      https://ardupilot.org/copter/docs/precision-landing-and-loiter.html  (doc page read via WebFetch summary)
Facts (verified in source): Precision Loiter is on when an RC aux switch (function 39 per doc page) is set and Loiter mode is active; if target_acquired(), the loiter setpoint becomes the precland target position
 (+ target velocity) fed into the SAME position controller (pos_control->input_pos_vel_accel_xy, update_xy_controller) - so PSC_* gains still decide how the hold behaves. If the pilot pushes > 50 cm/s/s of stick it is off.
 Params and defaults: PLND_ENABLED, PLND_TYPE (1 = MAVLink LANDING_TARGET), PLND_EST_TYPE default 1 (Kalman filter; 0 raw), PLND_ACC_P_NSE default 2.5 (range 0.5-5; "higher values weight the input from the camera more, accels less"),
 PLND_LAG default 0.02 s (range 0.02-0.25; "sensor lag, to cope with variable landing_target latency"), PLND_CAM_POS_X/Y/Z, PLND_YAW_ALIGN, PLND_ORIENT (25 = down), PLND_XY_DIST_MAX 2.5 m, PLND_ALT_MAX default 8 m (range 0-50; needs rangefinder), PLND_ALT_MIN 0.75, PLND_OPTIONS (bit 0 moving target).
 Doc page claim: "centimeter-level accuracy"; no measured figures, no height range given.
Take: for a 100-170 ms old fix set PLND_LAG to about 0.10-0.17 (max 0.25) when sending LANDING_TARGET. Not verified: how PLND_LAG interacts with the age of our own timestamps; read AC_PrecLand_Backend if used. The KF in AC_PrecLand uses accelerometer
 prediction between camera samples, so it is a second estimator with its own accel-aided smoothing (this is the only accelerometer-aided relative-position path reachable by parameters).
 No published accuracy at 20-60 m found for ArduPilot (see "not found").

## S8. Springer, Gudmundsson, Kyas: "A Precision Drone Landing System using Visual and IR Fiducial Markers and a Multi-Payload Camera", arXiv 2403.03806 (2024)
URL: https://arxiv.org/abs/2403.03806   Read: abstract only (WebFetch summary).
Reports: IR AprilTags (one active heated, one passive), day and night; successful landings from 168 m horizontal distance and 102 m altitude; average landing error 0.19 m. Gimbal camera with wide, zoom, IR sensors.
Take: large IR-contrast markers work to ~100 m altitude with a ZOOM lens; our wide fisheye (360 x 288) will not. Marker size not read (not verified).

## S9. Kim and Shin: "Night-Time UAV Precision Landing with a Multi-Spectral Helipad: IR Visual Alignment and Range Sensor-Based Descent Control" (2026 conference abstract; 2 pages)
URL: https://d2j16w31g89z0j.cloudfront.net/site/2026w/abs/0338-TMAYZ.pdf  Read: full 2-page PDF.
Reports: night trials, marker detected from d = 20.4 m to 0.5 m at about 30 FPS, confidence 0.84-0.93; landing offset 12 +- 5 cm (max 28 cm), 10/10 success vs GPS-only 163 +- 50 cm, 1/10. Align uses image-space error with an adaptive dead-zone that shrinks near the ground.
Not given: LED type, count, size, camera. Small sample (10 trials). Take: ~20 m detection with an IR-visible helipad is the only night range figure with a camera that was found; it is a landing, not a hover hold.

## S10. Lin, Jin, Chen: "Real-Time Monocular Vision System for UAV Autonomous Landing in Outdoor Low-Illumination Environments", Sensors 2021, 21, 6226
URL: https://doi.org/10.3390/s21186226   Read: full PDF text.
Passive 1 x 1 m marker, night average 5 lx at the marker, camera 1080p at 20 Hz with exposure capped at 50 ms (long exposure causes motion blur); marker detectable "with considerable accuracy" up to 15 Delta = 15 m altitude (Delta = 1 m), accuracy degrades quickly above that.
Take: an unlit 1 m marker works only to about 15 m at night; at 20-60 m with a wide lens you need active light. Exposure limit from motion blur applies to us (rolling shutter).

## S11. Khithov, Petrov, Tishchenko, Yakovlev: "Toward Autonomous UAV Landing Based on Infrared Beacons and Particle Filtering", RiTA 2015 (arXiv 1702.02735)
URL: https://arxiv.org/abs/1702.02735   Read: full text.
Reports: a 30 W NIR LED lamp near a runway visible through a narrow-band IR filter on a sunny day from 350 m (Fig. 2); advice: choose exposure so that beacons stay bright and background dark (dynamic range); beacon detection by simple threshold; fixed-wing, simulation-based evaluation, so no hover accuracy.
Take: with a narrow-band filter and short exposure, a lamp is a high-contrast point even in daylight; at night the filter is less needed but short exposure is still the way to get point-like blobs. Lamp output 30 W NIR for 350 m (daylight). No night-range-vs-lamp-size table anywhere.

## S12. Official tuning guides, what they actually say (position hold)
URLs: https://ardupilot.org/copter/docs/loiter-mode.html (read, WebFetch summary; page shows the NEWER names)
      https://ardupilot.org/copter/docs/tuning.html (read, summary)
      https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/docs/en/config_mc/pid_tuning_guide_multicopter.md (read, key lines grep'd)
      https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/docs/en/config_mc/mc_trajectory_tuning.md
ArduPilot: Loiter page says of position P and velocity P (shown there as PSC_NE_POS_P, PSC_NE_VEL_P, the post-4.6 names): "It is generally not required to adjust this." Tuning page: "Loiter does not require much tuning".
 Page says good loiter needs good GPS lock, low compass interference, low vibration; check log NTUN DesVelX vs VelX (actual velocity should track desired closely). So there is NO official recipe for a "tight" hold.
 NOTE 4.6 names are PSC_POSXY_P / PSC_VELXY_*; 4.7 and the docs use PSC_NE_POS_P, PSC_NE_VEL_*, LOIT_SPEED_MS, LOIT_ACC_MAX_M. Check which names the actual firmware shows.
PX4: the multicopter PID guide (step 14) says to tune velocity and position controllers in Position mode with "Simple position control" for step inputs; for the PID parameters: P up for responsiveness, down if overshoot; D to damp, "only as much as needed, as it amplifies noise";
 I reduces steady-state error, "if too low, the response might never reach the setpoint (e.g. in wind); if too high, slow oscillations". A badly tuned RATE controller shows in Position mode as twitches (will not hold perfectly still). The page gives no numbers for MPC_XY_*.
 The trajectory guide: setpoint params (MPC_ACC_HOR, MPC_JERK_AUTO etc.) shape setpoints and "Do not use these tuning parameters to fix bad tracking or vibration".
Community (not official), forum thread https://discuss.ardupilot.org/t/achieving-precision-loiter-stability-ac-4-5-7/129774: a 3.5-inch quad on GPS held a 30-40 cm radius in Loiter; a helper listed LOIT_ACC_MAX 600, LOIT_BRK_ACCEL 200, LOIT_BRK_DELAY 0.3, LOIT_BRK_JERK 300, PSC_VELXY_D 0.25 among many other changes, outcome unclear ("improved quite a lot, not perfect"). Anecdote.
Take: do not expect an official "tight hold" value set; the defaults are the supported tune. Treat any change as an experiment judged by SITL step response and NTUN DesVel vs Vel.

## S13. PX4 docs: external vision, EKF2 delay compensation, drag-based wind estimation (official, main branch markdown)
URLs: https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/docs/en/advanced_config/tuning_the_ecl_ekf.md
      https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/docs/en/computer_vision/visual_inertial_odometry.md
Read: relevant sections.
- EKF runs at a delayed fusion horizon; each sensor delay set by EKF2_*_DELAY; EKF2_DELAY_MAX "should be at least as large as the longest delay"; EKF2_TAU_VEL and EKF2_TAU_POS set the complementary filter that propagates the delayed state to now.
  Reducing the horizon reduces propagation error. (We need EKF2_DELAY_MAX >= EV delay; not stated what the default is in these pages.)
- EKF2_EV_DELAY: set to the difference between vision timestamp and actual capture time on the IMU clock. "Can further be tuned by varying the parameter to find the value that yields the lowest EKF innovations during dynamic maneuvers."
- EV uncertainty: from ODOMETRY covariance fields, or EKF2_EVP_NOISE / EKF2_EVV_NOISE; choose with EKF2_EV_NOISE_MD.
- Multicopter drag wind estimation: enable EKF2_DRAG_CTRL; BCOEF_X/Y and MCOEF tuned by flying "forwards/backwards/left/right/up/down between rest and maximum speed" in Position mode in still air and running mc_wind_estimator_tuning.py on the log. Parameter-only after one tuning flight.
  No accuracy numbers are given in the page.
- VIO page: "I get toilet-bowling when VIO is enabled" -> check the camera orientation/frame of velocities in ODOMETRY. "It is more reliable to just use vision velocity" when GPS and vision position disagree (GPS case only).
- Official statement on speed vs position fusion benefit: none found.
Take: for PX4 set EKF2_EV_CTRL = 5 (bits 0 and 2: position and velocity) only if our speed is genuinely independent/accurate; otherwise 1. Tune EKF2_EV_DELAY by minimum EV innovation. (The value 5 is the arithmetic from the bit list, bit0+bit2.)

## S14. Shankar, Elbaum, Detweiler: "Freyja: A Full Multirotor System for Agile and Precise Outdoor Flights" (ICRA 2021; PDF from NSF PAR)
URL: https://par.nsf.gov/servlets/purl/10297051   Read: full PDF text (pdftotext).
Setup: 45 cm diagonal, 1.2 kg quad, Pixhawk running a fork of ArduCopter as inner loop, u-blox ZED-F9P RTK GPS at 5 Hz, Odroid XU4 running an LQG outer loop with (a) trajectory feed-forward and (b) bias (disturbance) estimation. Text says the pipeline was also flown on T265 and monocular vision (not numbers for those).
Numbers: hover under ambient wind of about 5 m/s (up to 5.4 m/s): 3D error "typically over 0.5 m" with bias compensation OFF; when ON "average of about 0.125 m", the estimator converges within 2 s. Abstract: "hovering ... under dynamic wind speeds of up to 5-6 m/s while staying within 12 cm of 3D error". Calm indoor hover reference quoted: 3D error about 4 cm.
Take: a bias/disturbance estimator on the outer loop cut hover error about 4x in 5 m/s wind (0.5+ m to 0.125 m) with RTK-quality position. This is the same effect as a well-tuned integral term (PSC_VELXY_I) in ArduCopter, but faster. Needs external code on the companion computer (not a parameter). Our sim wind (20 km/h = 5.6 m/s, gusts to 40 km/h = 11 m/s) matches this regime.

## S15. Hentzen et al. and Shankar et al. together: realistic floors
Sources S4 (max deviation 17 cm at 10 m/s gust with mocap, plain PID; 45 cm at 12 m/s), S5 (INDI 21 cm vs integral PID 151 cm at 10 m/s; outdoors 0.24 vs 0.85 m peak with GPS-class position), S14 (0.125 m in 5 m/s wind with disturbance estimator + RTK).
Your numbers (perfect feed: 0.47 m RMS at 100 m in 5.6 m/s gusting 11 m/s) are consistent with S5's plain-controller regime (0.85 m peak error) and about 4x worse than S14. The ratio suggests the controller, not only the feed, sets the wind floor.
Not verified: whether the simulator's 0.47 m is dominated by the sim's wind-to-tilt model or by the ArduCopter integrator wind-up time.

## S16. Engel, Sturm, Cremers: "Accurate Figure Flying with a Quadrocopter Using Onboard Visual and Inertial Sensing", IROS-W 2012
URL: https://cvai.cit.tum.de/_media/spezial/bib/engel12vicomor.pdf   Read: full PDF text.
Setup: AR.Drone (53 x 52 cm with hull, small and light), monocular PTAM at 18 Hz, 320 x 240 video, vision delay about 130 ms (wifi + compute), IMU 200 Hz, control 100 Hz with ~60 ms command delay; EKF keeps a history of observations and
 re-computes the EKF state at the old time then predicts forward to t + control delay ("compensate for delayed and missing observations at the expense of recalculating the last cycles"); PID on predicted state.
Numbers: hold a target for 60 s: RMSE 4.9 cm (indoor), 7.8 cm, and 18.0 cm outdoors in wind (Fig. 6; low altitude). Landing within 15 cm of takeoff after 35 s with vision; raw odometry without vision drifted 2.1 m and 6 m when pushed.
Take: the same feed class as ours (18 Hz, 130 ms, 320 x 240) gave 5-18 cm RMS when the estimator handles delay properly and the controller uses the delay-compensated, forward-predicted state. That is the strongest single evidence that 100-170 ms is not the limit if VISO_DELAY_MS / EKF2_EV_DELAY are right.
 Caveat: indoor/low altitude, front/downward camera on a small drone; not at 20-100 m.

## S17. ArduPilot forum, thread "Regarding the position control: Intel T265 drifting in loiter mode" (community, includes lead developer replies)
URL: https://discuss.ardupilot.org/t/regarding-the-position-control-intel-t265-drifitng-in-loiter-mode/72161   Read: WebFetch summary of the thread (not full text).
Reported statements: ArduPilot uses covariance elements 0, 6, 11 (position) and 15, 18, 20 (angle) of VISION_POSITION_ESTIMATE; velocity noise is NOT taken from the message but from VISO_VEL_M_NSE (matches source S2);
 sending velocities (VISION_SPEED_ESTIMATE) is "quite important" for performance (developer reply); any reset_counter change makes the EKF reset to the new position; VISION_POSITION_DELTA must not be sent together with the other two.
 Drift cause there: poor tracking (vibration, few features), not controller gains. No numbers.
Companion thread https://discuss.ardupilot.org/t/quad-f330-drifts-in-loiter-mode-using-vision-position-estimates/119688: D435 + RTAB-Map at 30 Hz, VISO_DELAY 100 ms, sends position + velocity; drift reported without numbers; unresolved in the page read.
Take: no test measured with/without speed. Treat "send speed" as supported by a developer opinion and by the source (fused with VISO_VEL_M_NSE), not by data.

## S18. Shi, Fang, Schlautmann, Zintl, Dambeck, Holzapfel: "Landing Pad Design for Optical-aided Automatic Vertical Take-off and Landing", EuroGNC / CEAS-GNC-2024-047
URL: https://eurognc.ceas.org/archive/EuroGNC2024/pdf/CEAS-GNC-2024-047.pdf   Read: PDF text (skim of method and results).
Active-marker landing pad for night, 6 m side, monochrome camera with an optical band-pass filter, blob detection; seven static "finder" lights plus three blinking markers (2 Hz, 1 Hz, 0.5 Hz; duty 60/70/75%) for identification; blink state read at camera frame rate.
Evidence type: SIMULATION (X-Plane), blob error modelled as 0.5 pixel white noise. Position error about 4 m at 30 m height, centimetre level at 5 m (for a landing approach pose estimate, not a hover).
Authors state small markers (illuminated AprilTag, IR T-marker) have insufficient range for a 30 m x 30 m x 30 m volume. Take: a large footprint of lamps (metres, not centimetres) is what gives range; blink coding needs frame-rate sampling and is slow (about one period of the lowest frequency to identify). Our hold should use steady lamps.

## S19. Others read as abstract or snippet only (not relied on)
- Liu et al. AIRA, arXiv 2407.05619 (2024): IR bulb plus 3 photodiodes, <10 cm error up to 11.1 m; camera methods only 1.2 m from the same height in their comparison. Abstract only. Not useful at our range.
- Bosnak, Matko, Blazic, "Quadrocopter hovering using position-estimation information from inertial sensors and a high-delay video system", J Intell Robot Syst 67 (2012). Snippet only.
- "Delay and Dropout Tolerant State Estimation for MAVs" (ISER 2010, Springer chapter 10.1007/978-3-642-28572-1_39): snippet states a quadrotor held near a setpoint with a simple PID despite delays up to 230 ms and 16% dropout. Snippet only.
- Ordaz et al., J Intell Robot Syst 2013, doi 10.1007/s10846-012-9714-5 (Taylor-series delay predictor for quadrotor position control): snippet only.

## SUMMARY (the judgement, evidence above)
Parameter-only levers, in order:
1. Delay and noise honesty. ArduCopter: VISO_DELAY_MS = measured capture-to-EKF age (0-250), VISO_POS_M_NSE (floor of position noise; default 0.2 m) and VISO_VEL_M_NSE (default 0.1 m/s) set to what the feed really delivers.
   PX4: EKF2_EV_DELAY (0-300 ms; tune for lowest EV innovations), EKF2_DELAY_MAX >= it, EKF2_EV_NOISE_MD 0 so our covariance is used. Evidence: S2, S3, S13, S16.
2. Speed aid. ArduCopter: send VISION_SPEED_ESTIMATE and set EK3_SRC1_VELXY = 6 (ExternalNav). PX4: ODOMETRY velocity with EKF2_EV_CTRL bit 2 (value 5 with position). Evidence: S2 source, S17 developer opinion; NO measured improvement found. Risk: speed from differenced vision positions is correlated with position.
3. Gust handling in the outer loop: ArduCopter PSC_VELXY_I (default 1.0, 4.6 source range 0.02-1.00), PSC_VELXY_P, PSC_POSXY_P (default 1.0, range 0.5-2.0), LOIT_ACC_MAX (default 500 cm/s/s), PSC_JERK_XY (default 5);
   PX4 MPC_XY_VEL_I_ACC (default 0.4, range 0-60), MPC_XY_VEL_P_ACC (1.8, 1.2-5), MPC_XY_P (0.95, 0-2), MPC_XY_VEL_D_ACC (0.2, 0.1-2). Evidence: S4, S5, S14 (faster disturbance rejection halves to quarters gust error); no official tight-hold values (S12). Test only in SITL first.
Not parameter-only: INDI (S5), disturbance estimator feed-forward (S4, S14). Drag wind estimation (EK3_DRAG_*/EKF2_DRAG_CTRL) is parameter-only after a tuning flight, but no source quantifies the hold improvement.
