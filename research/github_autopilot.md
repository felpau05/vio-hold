# GitHub search: autopilot side of GNSS-free hold, and Orqa public code

Date: 2026-11-16. Method: `gh api`, `gh search`, Docker Hub API. Files were read. Nothing was run or built.
Labels: "seen" = read in a file or on a page (URL given). "inference" = the reasoning.

## 1. Orqa public code

Organization: https://github.com/orqafpv (name "Orqa FPV"). Full repo list (licence, last push, stars):

| Repo | Licence | Last push | Stars | Note |
|---|---|---|---|---|
| ardupilot | GPL-3.0 | 2026-07-29 | 0 | fork. Branches: master, h7quadcore, mil-mavl |
| PX4-Autopilot | BSD-3-Clause | 2025-04-09 | 0 | fork. Branches: main, orqa-fcs, develop_h743-3030-pro, develop_APB-initial |
| orqa-betaflight | GPL-3.0 | 2026-11-09 | 0 | fork. 19 branches, incl. PBAS |
| orqa-betaflight-config | GPL-3.0 | 2026-11-10 | 0 | target config. Folders ORQAF7 and branch orqa-H743-QUADCORE |
| orqa-inav | GPL-3.0 | 2026-08-26 | 0 | fork. Branches MRM2-build, ORQA-F405-target, ORQA-H743-WING |
| orqa-inav-configurator | GPL-3.0 | 2023-10-09 | 0 | |
| orqa-BF-config | GPL-3.0 | 2024-07-15 | 0 | |
| qgroundcontrol | none | 2025-02-14 | 0 | fork |

Seen: all 8 are forks or configs. There is no APB SDK, no DTK repo, no companion examples, no camera code, no docs repo. Other "orqa" organizations (orqastudio, orqa-technologies and others) are unrelated names (inference: no link to the drone maker found).

### Which firmware targets the APB
1. ArduPilot, branch `mil-mavl`. Head commit 3b40393b, 2026-07-28. It is Copter-4.6.0 plus ONE commit (compare Copter-4.6.0...mil-mavl: ahead 1, behind 0). The commit adds:
   - `libraries/AP_HAL_ChibiOS/hwdef/ORQAAPB/{hwdef.dat, hwdef-bl.dat, defaults.parm, README.md}`
   - `Tools/bootloaders/ORQAAPB_bl.bin/.hex`, `Tools/AP_Bootloader/board_types.txt` line `AP_HW_ORQAAPB 1441`
   - `libraries/AP_Compass/AP_Compass_IIS2MDC.cpp/.h` (new compass driver; hwdef sets `AP_COMPASS_IIS2MDC_ENABLED 1`).
   - URL: https://github.com/orqafpv/ardupilot/blob/mil-mavl/libraries/AP_HAL_ChibiOS/hwdef/ORQAAPB/hwdef.dat
2. PX4, branch `develop_APB-initial`. 3 Orqa commits (2025-02-28, 2025-03-21, 2025-04-09) on a PX4 main of 2025-02-14. Board folder `boards/orqa/h743-APB/`. Newest and last Orqa PX4 work is April 2025. ArduPilot APB work is July 2026. Inference: the APB target moved to ArduPilot. The MRM2-10 page says all four stacks are supported, so this is not proof of the event firmware.
3. Betaflight and iNav forks have H7 QuadCore and F7/F405/F722 targets. Found no APB file in them. Betaflight branch `PBAS` (Aug-Sep 2025) adds an MSP/GHST waypoint parse, `GPS_set_home_position()` mid-flight and a disabled RESCUE_DESCENT phase. It has nothing about vision.

### ArduPilot ORQAAPB hwdef (seen, hwdef.dat)
- MCU STM32H743, `APJ_BOARD_ID AP_HW_ORQAAPB`. IMUs: `IMU Invensensev3 SPI:imu1 ROTATION_YAW_180 #2-10AI` and `SPI:imu2 ROTATION_YAW_90 #2-10AI`. The comment "2-10AI" names the MRM2-10AI.
- `SERIAL_ORDER OTG1 EMPTY EMPTY USART3 UART4 EMPTY USART6 UART7 UART8 OTG2`.
- UART4 = PC10 TX / PC11 RX, `define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_MAVLink2`. This is the new line vs the plain H7 QuadCore board. It matches the PX4 branch (UART4 = IMX link). No baud default is set in the hwdef (inference: the 230400 comes from an event config, SERIAL4_BAUD=230 in a note of ours).
- USART3 RC (GHST default), USART6 MAVLink2 ("Sik"), UART7 GPS, UART8 ESC telemetry. Baro DPS310 I2C 0x77. No built-in compass (external probe). `BRD_OPTIONS 9` is the only line in defaults.parm.
- Camera switch GPIO 74 (`CAM_SW`, RELAY2). No video-capture or camera device lines (that part is Linux on the i.MX side).
- Version: Copter 4.6.0 base. The wiki says persistent EKF origin (AHRS_OPTIONS bits 3, 4) is "4.7 and later", so it is NOT in this branch (inference).

### PX4 develop_APB-initial (seen)
Files: https://github.com/orqafpv/PX4-Autopilot/tree/develop_APB-initial/boards/orqa/h743-APB
- `default.px4board`: `CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS1"`, TEL2 ttyS2, GPS1 ttyS3, TEL3 ttyS4, RC ttyS0. Modules: EKF2, LOCAL_POSITION_ESTIMATOR, LANDING_TARGET_ESTIMATOR, MC position/attitude/rate control, NAVIGATOR, UXRCE_DDS_CLIENT, `COMMON_OPTICAL_FLOW`. No vision-specific module.
- `init/rc.board_defaults`: "#IMX setup": `MAV_0_CONFIG 101` (TELEM1), `MAV_0_FORWARD 0`, `MAV_0_MODE 0` (Normal), `MAV_0_RATE 1200`, `SER_TEL1_BAUD 115200`. Also MAV_1 on TELEM2 at 57600, `SYS_AUTOSTART 4001`, GHST RC, 6S battery. NO `EKF2_EV_*`, no `EKF2_GPS_CTRL`, no vision defaults.
- `nuttx-config/.../board.h`: UART4 on PC11/PC10. `CONFIG_UART4_BAUD=115200` in the bootloader defconfig.
- Differences vs our notes: link baud 115200 and `MAV_0_MODE 0` (Normal, not Onboard) and 1200 B/s rate in Orqa's PX4 defaults. If the event drone runs PX4 with these defaults, our 230400 and Onboard-mode assumptions need a check (setting `SER_TEL1_BAUD`, `MAV_0_MODE`, `MAV_0_RATE`). Inference.
- Licence note: forks keep upstream licences. Board files carry the PX4 BSD-3 header.

Nothing public from Orqa mentions vision, `EXTERNAL VISION`, camera device names (/dev/video*) or the i.MX Linux image. The `/dev/video0`, `/dev/video2` names in our notes come from third-party code (DroneWuKong/Prismo-Calibration, no licence: ideas only).

## 2. Bridges that send vision position (conventions)

### 2a. `t265_to_mavlink.py` (thien94/vision_to_mavros)
- https://github.com/thien94/vision_to_mavros/blob/master/scripts/t265_to_mavlink.py . GPL-3.0, 295 stars, last commit 2022-10-28 (script too). 670 lines. Seen.
- Connection: `mavutil.mavlink_connection(..., source_system=1, source_component=93, baud=921600 default, autoreconnect=True, force_connected=True)`.
- Heartbeat: sent in the read loop, `MAV_TYPE_ONBOARD_CONTROLLER`, `MAV_AUTOPILOT_GENERIC`.
- `VISION_POSITION_ESTIMATE` at 30 Hz (default `vision_position_estimate_msg_hz_default = 30.0`): fields usec, x, y, z (NED, metres), roll, pitch, yaw (from the camera pose), `covariance` = diagonal: `cov_pose` for x,y,z, `cov_twist` for the angles, `reset_counter`.
  `cov_pose = linear_accel_cov * 10**(3 - tracker_confidence)`.
- `VISION_SPEED_ESTIMATE` 30 Hz (usec, vx, vy, vz, 3x3 diag covariance, reset_counter). `VISION_POSITION_DELTA` also at 30 Hz.
- Time stamp: `current_time_us = int(round(time.time() * 1000000))` = host wall clock at pose arrival. No TIMESYNC use (a `timesync_send` helper exists but is only a TODO).
- `reset_counter` starts at 1. It increases when position jumps > 0.1 m or speed jumps > 20 m/s.
- `SET_GPS_GLOBAL_ORIGIN` (target 1, lat, lon, alt) and `SET_HOME_POSITION` (x,y,z = 0, q = [1,0,0,0], approach 0,0,1), sent by a key press (and at start by the script).
- Wiki parameters (https://github.com/ArduPilot/ardupilot_wiki/blob/master/common/source/docs/common-vio-tracking-camera.rst): `SERIAL2_PROTOCOL` 2, `SERIAL2_BAUD` 921, `VISO_TYPE` 2 (T265), `EK3_SRC1_POSXY` 6, `EK3_SRC1_VELXY` 6, `EK3_SRC1_POSZ` 1 (baro), `EK3_SRC1_VELZ` 6, optional `GPS1_TYPE` 0, `EK3_SRC1_YAW` 1 (compass), `RC7_OPTION` 80 (Viso Align). "Reboot the autopilot if external navigation data is lost."

### 2b. ArduPilot receive side (source, master, pushed 2026-11-15)
- `libraries/GCS_MAVLink/GCS_Common.cpp`: `handle_common_vision_position_estimate_data`: `posErr = sqrtf(cov[0]+cov[6]+cov[11])`, `angErr = sqrtf(cov[15]+cov[18]+cov[20])`, 0 if `cov[0]` is NaN. The off-diagonal terms are ignored. One scalar error per sample.
- Same file: the sender time goes through `correct_offboard_timestamp_usec_to_ms` (UART receive time plus a lag-correction filter). Wiki: "Timestamp since system boot. This does not need to be synchronised with the autopilot's time."
- `libraries/AP_VisualOdom/AP_VisualOdom.cpp` (seen): `VISO_TYPE` values 0 None, 1 MAVLink, 2 IntelT265, 3 VOXL. `VISO_DELAY_MS` default 10 (0-250). `VISO_POS_M_NSE` default 0.2 m (lower bound; the wiki says "constrained between VISO_POS_M_NSE and 100"). `VISO_YAW_M_NSE` 0.2 rad, `VISO_VEL_M_NSE` 0.1, `VISO_QUAL_MIN` 0, `VISO_SCALE` 1, `VISO_ORIENT`, `VISO_POS_X/Y/Z`.
- `ODOMETRY` in code is accepted only if `frame_id == MAV_FRAME_LOCAL_FRD` AND `child_frame_id == MAV_FRAME_BODY_FRD`. The wiki table says "BODY_FRD (12) or LOCAL_FRD (20)" for both. The code wins (inference: the wiki is looser than the code).
- `AP_VisualOdom_MAV.cpp::handle_pose_estimate` passes the attitude to the EKF (`writeExtNavData`). With NaN angles you would feed a NaN quaternion. Our `fc_link.py` sends the FC's own attitude to ArduPilot, so it avoids this.
- Rates (https://github.com/ArduPilot/ardupilot_wiki/blob/master/dev/source/docs/mavlink-nongps-position-estimation.rst): "sent to the autopilot at 4hz or higher". Messages listed: ODOMETRY (preferred), VISION_POSITION_ESTIMATE (+ optional VISION_SPEED_ESTIMATE), VISION_POSITION_DELTA, VICON_POSITION_ESTIMATE, ATT_POS_MOCAP, GLOBAL_VISION_POSITION_ESTIMATE, GPS_INPUT.
- Parameters in that page: `EK3_SRC1_POSXY` = 6, `EK3_SRC1_VELXY` = 6 or 0, `EK3_SRC1_POSZ` = 6 or 1, `EK3_SRC1_VELZ` = 6 or 0, `EK3_SRC1_YAW` = 6 or 1. "If no GPS is attached ... the EKF origin must be set". It names `ahrs-set-origin.lua` (https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/ahrs-set-origin.lua, params `AHRS_ORIG_LAT/LON/ALT` as the script defines them).
- New command on that page: `MAV_CMD_EXTERNAL_POSITION_ESTIMATE` (43003), COMMAND_INT, frame GLOBAL, param1 transmission_time, param2 processing_time, param3 accuracy, x/y lat/lon e7, z NaN. "Rejected if the EKF is still fed GPS". EKF3 only.
- Persistent origin: `AHRS_OPTIONS` bits 3 and 4, `AHRS_ORIGIN_LAT/LON/ALT`, "4.7 and later" (https://github.com/ArduPilot/ardupilot_wiki/blob/master/common/source/docs/common-non-gps-navigation-landing-page.rst).

### 2c. ArduPilot autotest (best tested source of parameter sets)
`Tools/autotest/arducopter.py`, test `VisionPosition` (line ~6552 on master). Parameters set: `EK3_SRC1_POSXY 6, EK3_SRC1_VELXY 6, EK3_SRC1_POSZ 6, EK3_SRC1_VELZ 6, GPS1_TYPE 0, VISO_TYPE 1, SERIAL5_PROTOCOL 1`, then `reboot_sitl()`, then `mav.system_time_send(...)` (comment: "without a GPS ... AP doesn't send system_time messages. So prompt it"), then `set_origin(...)`. Vicon data comes from the SITL sim (`--serial5=sim:vicon:`, param group `SIM_VICON_*`; `SIM_VICON_TMASK` bitmask: 0 VISION_POSITION_ESTIMATE, 1 VISION_SPEED_ESTIMATE, 2 VICON_POSITION_ESTIMATE, 3 VISION_POSITION_DELTA, 4 ODOMETRY). Source file SIM_Vicon.cpp also shows message field use. Licence GPL-3.0.

### 2d. PX4
- Docs https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/ros/external_position_estimation.md : 30 Hz with covariance up to 50 Hz, `EKF2_EV_CTRL`, `EKF2_EV_DELAY`, `EKF2_EV_POS_*`, `SET_GPS_GLOBAL_ORIGIN`, frame LOCAL_FRD for ODOMETRY. This matches research/px4_vision.md.
- PX4 has no ready Python bridge in the repo. MAVSDK docs page exists (docs/en/robotics/mavsdk.md) (not read in detail).

### Our program vs these (fc_link.py, seen)
1. Time stamp. T265 stamps host wall-clock at arrival and does no sync. We stamp capture time on the companion clock and answer TIMESYNC. For ArduPilot this does not matter (it uses the jitter filter). For PX4 sync matters (px4_vision.md section 3). No conflict.
2. Heartbeat. T265 sends a companion heartbeat every loop. Found no `heartbeat_send` in our `*.py`. Effect unknown (inference: cheap to add at 1 Hz).
3. `SET_HOME_POSITION`. T265 sends it; we send only `SET_GPS_GLOBAL_ORIGIN`. ArduPilot wiki says "Set Home Here, Set EKF Origin Here" in Mission Planner. Add and test in SITL (inference).
4. `reset_counter`. T265 starts at 1 and raises it on a jump. We keep 0. Fine while the frame stays continuous.
5. Covariance. T265 fills the diagonal with scalar cov. We fill [0],[1],[6]. ArduPilot reads only [0],[6],[11] as a sum and floors with `VISO_POS_M_NSE` (0.2 m by default), so a small covariance cannot get below 0.2 m unless the parameter is lowered. PX4: `EKF2_EVP_NOISE` is the floor (px4_vision.md).
6. Component id. T265 uses source_component 93. We use `MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`. No evidence either one is required.
7. Velocity. T265 sends VISION_SPEED_ESTIMATE. We send none. With `EK3_SRC1_VELXY` 6 and no velocity, ArduPilot would have no velocity source; our notes already say use 0 (None).
8. No other difference found. Frames: NED in both.

## 3. ArduCopter SITL in Docker, closed loop, wind, GPS off, ExternalNav

Images (Docker Hub API, seen):
- `ardupilot/ardupilot-dev-base` (official, org ardupilot): tag `edge` 2026-08-07, 374 MB compressed, amd64. `ardupilot-dev-chibios` 535 MB compressed. These are BUILD images, not SITL images: you build ArduCopter inside them (seen: Dockerfile of CacheFail below does `FROM ardupilot/ardupilot-dev-base`).
- `radarku/ardupilot-sitl`: Apache-2.0, last push 2025-07-26, 1.5 GB compressed, default tag Copter-4.0.3 (README). Old.
- `orthuk/ardupilot-sitl` (repo uxduck/ardupilot-sitl-docker): no licence shown, 2026-10-24, README says Ubuntu 3.9 GB, Debian 3.65 GB; Tracker-4.5.7 version stated; the Hub tag seen is arm64 only (2024).
- Smallest: https://github.com/CacheFail/ardupilot-sitl-slim . No licence in the repo (ideas only), 2 stars, last push 2026-03-26. README: "ArduPilot SITL (Copter 4.5.7) ... reduced to 613MB". Multi-stage Dockerfile (seen in full): builder `FROM ardupilot/ardupilot-dev-base`, `git clone --depth 1 --branch Copter-4.5.7`, `./waf configure --board sitl && ./waf copter`, `strip`; final `python:3.11-slim-bookworm` + `pip install pymavlink MAVProxy future pexpect`; copy the binary + `Tools/autotest/*.py`, `pysim/`, `default_params/`, `ArduCopter/`; `touch /ardupilot/WAF_FINISHED`; CMD `sim_vehicle.py --vehicle ArduCopter --frame quad -I0 --custom-location=... --speedup 1 --no-rebuild --out=udp:host.docker.internal:14550`.
  No GPU needed (SITL is plain physics). It was not built. Disk: this laptop has 38 GB free (df).

Command sequence (mark per step):
1. (from source: CacheFail Dockerfile) Write your own two-stage Dockerfile like the one above. Pin a tag you choose (Copter-4.6.0 matches the Orqa branch; the tag exists as a ref: the Orqa branch is built on it). `docker build -t ac-sitl .`
2. (from source: sim_vehicle.py options `--no-mavproxy`, `--out`, `-w`, `--speedup`, `--custom-location`, `--model`) `docker run --rm -p 5760:5760 ac-sitl python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-rebuild --no-mavproxy -w --speedup 1` . Default SERIAL0 is TCP 5760 (seen: `mavproxy --master tcp:127.0.0.1:5760` logic, sim_vehicle.py lines ~1145-1149). Check the port in the first log lines.
3. (from source: arducopter.py VisionPosition) Set params: `GPS1_TYPE 0`, `VISO_TYPE 1`, `EK3_SRC1_POSXY 6`, `EK3_SRC1_VELXY 0` (we send no velocity; the autotest sets 6 because its Vicon sim sends velocity), `EK3_SRC1_POSZ 1` (baro; wiki T265 page), `EK3_SRC1_VELZ 0` or the autotest value 6. Reboot SITL.
4. (from source: autotest) Send `system_time`, then the EKF origin (`SET_GPS_GLOBAL_ORIGIN`). Then stream VISION_POSITION_ESTIMATE at 30 Hz from our program.
5. (from source: SITL.cpp) Wind: `SIM_WIND_SPD` (m/s), `SIM_WIND_DIR` (deg, direction wind comes from, default 180), `SIM_WIND_TURB` (m/s random variation). Also seen: `SIM_WIND_DIR_Z`, `SIM_WIND_T`, `SIM_WIND_TC`.
6. GPS off. DOC IS STALE: the wiki (https://github.com/ArduPilot/ardupilot_wiki/blob/master/dev/source/docs/SITL_simulation_parameters.rst) says `SIM_GPS_DISABLE 1`. In source `libraries/SITL/SITL.cpp` has the comment "1 was GPS_DISABLE": the old name is gone. The new name is `SIM_GPS1_ENABLE` (SIM_GPS.cpp: `AP_GROUPINFO_FLAGS("ENABLE", 1, GPSParms, enabled, 0 ...)`, group prefix "GPS1_"). Not verified how the default is set to 1 for a normal run. Safe way (from autotest): `GPS1_TYPE 0` on the autopilot side.
7. Test pass check: EKF flags via `EKF_STATUS_REPORT`, position via `LOCAL_POSITION_NED`/`GLOBAL_POSITION_INT`. Use Loiter (autotest uses LOITER then takeoff).
Forum-sourced steps: none used.

The smallest tested way (the answer to 3b is in the summary): no pre-built tested SITL image with ExternalNav exists that was found. The only tested ExternalNav + GPS-off sequence is the ArduPilot autotest `VisionPosition` (CI-run). Alternative with no build: our existing PX4 SIH container (`px4io/px4-sitl:latest`, 180 MB local) tests PX4 only.

## 4. Precision loiter and precision landing

### ArduPilot (docs + source)
Pages: https://github.com/ArduPilot/ardupilot_wiki/blob/master/copter/source/docs/precision-landing-and-loiter.rst and `dev/source/docs/mavlink-precision-landing.rst`.
- Params: `PLND_ENABLED` 1 (reboot), `PLND_TYPE` 1 (MAVLink; 2 IRLock, 3 SITL_Gazebo, 4 SITL), `PLND_YAW_ALIGN`, `PLND_CAM_POS_X/Y/Z`, `PLND_LAND_OFS_X/Y`, `PLND_EST_TYPE` (default 1 KalmanFilter), `PLND_ACC_P_NSE` 2.5, `PLND_LAG` 0.02 s, `PLND_XY_DIST_MAX`, `PLND_STRICT`, `PLND_RET_MAX`, `PLND_ALT_MIN` 0.75 m, `PLND_ALT_MAX` 8 m, `PLND_OPTIONS`, `PLND_ORIENT` (0 Forward, 4 Back, 25 Down) (source AC_PrecLand.cpp).
- Precision Loiter: "set an Auxiliary Function Switch to 39 (PrecLoiter Enable)", enter Loiter, activate the switch.
- LANDING_TARGET fields used (doc table + AC_PrecLand_MAVLink.cpp): `time_usec` (not synchronised), `frame` = MAV_FRAME_BODY_FRD (12) or MAV_FRAME_LOCAL_FRD (20) (other frame: "Plnd: Frame not supported"), `angle_x`, `angle_y` (rad), `distance` (m, "or 0 if unknown. If 0 the rangefinder's distance is used"), `x`,`y`,`z`, `position_valid`. Angle-only: `position_valid` 0, unit vector = (-tan(angle_y), tan(angle_x), 1) (source). At least 2 Hz. Measurement older than 1 s is dropped (source: `AP_HAL::millis() - time_ms <= 1000`).
- Height limits. `PLND_ALT_MAX` and `PLND_ALT_MIN` are rangefinder limits ("This needs a rangefinder to work. Set to zero to disable") (source param text). Setting both to 0 means "assume sensor always in range" (source `check_if_sensor_in_range`). So there is no fixed upper height in the code for loiter. Practical limit = our target detection at height (not in docs).
- With NO rangefinder: the target position needs a distance. Source (`construct_pos_meas_using_rangefinder`): valid if rangefinder is valid OR `distance > 0` in the message. So we must send `distance` = our height above the target (baro-based). Inference: this works, not tested.
- Needs an EKF velocity and position: `AC_PrecLand::update` builds inertial history from `get_velocity_NED` and the filter status `horiz_vel`; `ModeLoiter::precision_loiter_xy` feeds `pos_control->input_pos_vel_accel_NE_m(target_pos, ...)`. Doc note: "Precision Landing requires a valid horizontal position estimate". So the autopilot cannot hold over the target using angle only: it still needs ExternalNav (or flow) for position/velocity. Precision loiter is a correction layer in Loiter. The EKF "assumes the landing target is stationary" (doc), which fits our fixed ground point.
- Aim point only: the target is a pattern/lights on the ground. Our program would replace the IR-Lock role. Fields above are all we need. Reuse: the T265 tree has `launch/t265_precland.launch` and `scripts/t265_precland_apriltags.py` (GPL-3.0) as an example sender (not read in detail).
- SITL: `PLND_TYPE 4` (SITL) exists (param list); the dev wiki page `mavlink-precision-landing.rst` links "Testing Precision Landing in SITL" (not read).

### PX4 (docs main branch)
https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/advanced_features/precland.md
- "Precision landing is only possible with a valid global position" (info box). It is a landing procedure (mode `auto:precland`, `MAV_CMD_NAV_LAND` param2 1 or 2, `RTL_PLD_MD`), not a hold: phases horizontal approach, descent over target, final approach.
- LANDING_TARGET: PX4 "requires `LANDING_TARGET.frame` to be MAV_FRAME_LOCAL_NED and only populates the fields `x`, `y`, and `z`". So angle-only is not accepted by the doc. Params: `PLD_BTOUT` 5 s, `PLD_HACC_RAD` 0.2 m, `PLD_FAPPR_ALT` 0.1 m, `PLD_SRCH_ALT` 10 m (max 100), `PLD_SRCH_TOUT`, `LTEST_MODE`, `SENS_EN_IRLOCK`. Module `landing_target_estimator` is in the Orqa APB board config (`CONFIG_MODULES_LANDING_TARGET_ESTIMATOR=y`).
- New Vision Target Estimator (docs/en/advanced_features/vision_target_estimator.md): "PX4 v1.18", experimental, disabled in default boards. Its use is landing and precision takeoff.
- Verdict: PX4 cannot hold over our target with GNSS off.

## 5. Betaflight and iNav with a fake GPS over MSP

- Betaflight master (GPL-3.0, 11.8k stars, pushed 2026-11-15, seen in source):
  - `src/main/msp/msp.c`: `MSP_SET_RAW_GPS` reads: fix u8, numSat u8, lat u32 (1e7 deg), lon u32, alt u16 (m), groundSpeed u16 (cm/s). `MSP2_SENSOR_GPS` also handled (instance, week, msTOW, fix, sats, hAcc, vAcc, sAcc (cm), pdop, lon, lat, alt cm, vel N/E/D, course cdeg, true yaw, date/time). Both set `GPS_update |= GPS_MSP_UPDATE`.
  - `src/main/io/gps.c`: `case GPS_MSP:` accepts data; CLI table `gps_provider` = "NMEA, UBLOX, MSP, VIRTUAL" (settings.c).
  - Position hold exists on master: `src/main/flight/pos_hold_multirotor.c`, `position_estimator.c`; CLI `poshold_position_source` = AUTO / GPS_ONLY / OPTICALFLOW_ONLY; `poshold_opticalflow_quality_min`, `poshold_opticalflow_max_range`. Last change to the file 2026-11-02. Position hold needs `positionEstimatorIsValidXY()`. GPS accuracy fields (hAcc, sAcc) scale the estimator noise (position_estimator.c comments).
  - MSP2 id `MSP2_SENSOR_OPTICALFLOW 0x300B` is defined in msp_protocol_v2_betaflight.h. Did not find the handler. Unverified.
  - Feature is new (position hold in Betaflight is recent); released firmware on the drone may be older. Orqa fork `orqa-betaflight_2026.6-maintenance` exists (branch name only; not read for pos hold).
- iNav (GPL-3.0, 4.3k stars, pushed 2026-11-15, default branch maintenance-10.x):
  - `src/main/msp/msp_protocol_v2_sensor.h` ids: `MSP2_SENSOR_RANGEFINDER 0x1F01`, `MSP2_SENSOR_OPTIC_FLOW 0x1F02`, `MSP2_SENSOR_GPS 0x1F03`, COMPASS 0x1F04, BAROMETER 0x1F05, AIRSPEED, HEADTRACKER. Structs in `msp_protocol_v2_sensor_msg.h` (GPS: fields above; opflow: `uint8 quality 0-255, int32 motionX, int32 motionY`; rangefinder: quality, `int32 distanceMm`).
  - `src/main/io/opflow_msp.c` (`USE_OPFLOW_MSP`) consumes the opflow message. Position hold with flow and baro is therefore possible in principle (inference).
  - SITL: `docs/SITL/SITL.md`, `docs/development/Building SITL.md` (cmake -DSITL=ON). Sensors are "FAKE" fed by RealFlight or X-Plane. No built-in multirotor physics. We would have to write a simulator bridge (inference).
- Betaflight SITL: `src/platform/SIMULATOR` exists. Third-party bridges: demvlad/BF-SITL-X-Plane-Bridge (4 stars). Did not evaluate them.
- Projects found (all weak):
  - gamolin8v/drone-vio-bridge: MIT, 0 stars, 1 commit made 2026-11-15, pure Python, "ArduPilot, iNav (MSP GPS), Betaflight (RC override)". `src/bridges/msp_bridge.py` builds `MSP_SET_RAW_GPS` (code packs 16 bytes with a course field; Betaflight source reads 14 bytes). The file also has a duplicated `struct.pack`. Looks untested on hardware. SKIP, use only as a pointer.
  - tudelft/indiflight: GPL-3.0, 68 stars, pushed 2026-11-13. Betaflight 4.4.2 fork with INDI controller, `USE_POS_CTL`, `USE_VEL_CTL`, UART link for offboard position (`USE_GPS_PI`). Research code, "extremely experimental". Needs our own flashing. BORROW as an idea for the offboard-position link.
  - TonyJeong31/apriltag-drone-localization (MIT, 0 stars) and mop9047/iphone-based... (no licence): Betaflight RC-override position PIDs over MSP. Not a fake GPS. Ideas only.
- Limits (inference): the drone as sold may not run iNav or Betaflight with these features. Orqa forks of both exist (MRM2 targets), none with APB files.

## 6. i.MX8M Plus companion with PX4 or ArduPilot

- No public project found with vision-based hold on i.MX8M Plus + PX4/ArduPilot (searches: "imx8mp px4", "imx8mp ardupilot", "imx8mp mavlink-router", "imx8mp optical flow drone": zero repos).
- NXP NavQPlus material (seen):
  - https://github.com/NXP-Robotics/GitBook-NavQPlus (CC0-1.0, 2025-06-04, 9 stars): `navqplus-user-guide/running-mavlink-via-t1-ethernet.md` (fixed IP 10.0.0.3 vs FMUK66 10.0.0.2, netplan/connman) and `user-contributed-content/ros2/microdds.md` (micro-XRCE-DDS agent). Link is T1 Ethernet or DDS, not UART MAVLink at 230400. Different from the APB.
  - rudislabs/NavQPlus-Resources (Apache-2.0), rudislabs/navqplus-images (Apache-2.0, 2024): image/resource lists only.
  - The Orqa PX4 board files list module `UXRCE_DDS_CLIENT` (default.px4board). Inference only.
- mavlink-router: https://github.com/mavlink-router/mavlink-router (Apache-2.0, 625 stars, 2026-10-23). `examples/config.sample` seen: `[General]` section with `TcpServerPort`, `DeDuplicationPeriod`, `MavlinkDialect`. Not read the `[UartEndpoint]` block (the endpoint names `UartEndpoint`/`UdpEndpoint` come from the project's usual config; unverified here). Alternative: bluenviron/mavp2p (257 stars, 2026-11-03, not opened).

## Table of 11 most useful sources

| # | URL | Licence | Last commit | Reuse (paths) | Verdict |
|---|---|---|---|---|---|
| 1 | github.com/orqafpv/ardupilot (branch mil-mavl) | GPL-3.0 | 2026-07-28 | `libraries/AP_HAL_ChibiOS/hwdef/ORQAAPB/*` | ADOPT as evidence of target and UART4 map (read only) |
| 2 | github.com/orqafpv/PX4-Autopilot (develop_APB-initial) | BSD-3-Clause | 2025-04-09 | `boards/orqa/h743-APB/init/rc.board_defaults`, `default.px4board` | BORROW: MAV_0_* defaults and baud 115200 to check on the drone |
| 3 | github.com/ArduPilot/ardupilot (GCS_Common.cpp, AP_VisualOdom, AC_PrecLand, autotest) | GPL-3.0 | 2026-11-15 | `libraries/GCS_MAVLink/GCS_Common.cpp`, `libraries/AP_VisualOdom/`, `Tools/autotest/arducopter.py` VisionPosition | ADOPT conventions and test recipe (copy ideas, not code, unless GPL is fine) |
| 4 | github.com/thien94/vision_to_mavros | GPL-3.0 | 2022-10-28 | `scripts/t265_to_mavlink.py` | BORROW conventions: heartbeat, SET_HOME_POSITION, reset_counter |
| 5 | github.com/ArduPilot/ardupilot_wiki | Other (NOASSERTION) | 2026-11-14 | `dev/source/docs/mavlink-nongps-position-estimation.rst`, `common-vio-tracking-camera.rst`, `precision-landing-and-loiter.rst`, `mavlink-precision-landing.rst` | ADOPT as parameter source (docs only) |
| 6 | github.com/CacheFail/ardupilot-sitl-slim | none | 2026-03-26 | `Dockerfile` | BORROW idea only (no licence): own 2-stage build |
| 7 | hub.docker.com/r/ardupilot/ardupilot-dev-base | Apache-2.0 (ardupilot_dev_docker repo) | 2026-08-07 | base build image, 374 MB | ADOPT as build stage |
| 8 | github.com/PX4/PX4-Autopilot docs (`ros/external_position_estimation.md`, `advanced_features/precland.md`) | BSD-3-Clause | 2026-11-15 | the two doc files | ADOPT as reference |
| 9 | github.com/betaflight/betaflight | GPL-3.0 | 2026-11-15 | `src/main/msp/msp.c`, `flight/pos_hold_multirotor.c`, `flight/position_estimator.c` | BORROW MSP field layout; SKIP as primary path |
| 10 | github.com/iNavFlight/inav | GPL-3.0 | 2026-11-15 | `src/main/msp/msp_protocol_v2_sensor*.h`, `io/opflow_msp.c`, `docs/SITL/SITL.md` | BORROW MSP structs; SKIP as primary path |
| 11 | github.com/tudelft/indiflight | GPL-3.0 | 2026-11-13 | Betaflight fork with offboard position link | SKIP (research firmware), idea only |

Not recommended: gamolin8v/drone-vio-bridge (MIT, 1 commit, 0 stars, unverified), DroneWuKong/Orqa_H7_PX4_Ports (no licence, ideas only), DroneWuKong/Prismo-Calibration (NOASSERTION, ideas only).

## What was searched and not found
- Any Orqa repo with APB SDK, DTK, companion examples, camera or vision code, or documentation.
- An Orqa EKF2 external-vision default or any `VISO_*` default in the Orqa branches.
- Evidence of which firmware the event drone runs.
- A public i.MX8M Plus vision-hold project on PX4 or ArduPilot.
- A ready ArduCopter SITL image with ExternalNav preconfigured and a Python vision sender.
- A tested Betaflight or iNav position hold driven by a fake MSP GPS from a companion (only one new untested repo).
- Whether Betaflight handles `MSP2_SENSOR_OPTICALFLOW` (id only).
- mavlink-router UART endpoint syntax (not read).
- Docs on how the PX4 default (Normal mode, 115200) maps to the event setup: not found.
