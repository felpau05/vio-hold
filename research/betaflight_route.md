# Betaflight and INAV route for position hold without GNSS

Date: 2026-11-20. All facts come from source code read for this report. A fact marked "inference" is not read in source.

## Verdicts

- Betaflight: GO. The Orqa 2026.6 branch has an APB (Auto Pilot Board) target and a position hold that accepts optical-flow input sent over MSP (MultiWii Serial Protocol), with no GNSS receiver. The condition is that the event aircraft runs this branch or a newer one. This is not known yet.
- INAV: NOT KNOWN. The INAV source has the MSP flow, MSP rangefinder and MSP GPS paths, but the Orqa INAV fork has no APB or Orqa H7 multirotor target. The firmware that the organizer issues is not known.

## 1. Orqa repositories and the likely branch

Organization repositories (gh api orgs/orqafpv/repos): orqa-betaflight, orqa-inav, orqa-inav-configurator, orqa-BF-config, orqa-betaflight-config, PX4-Autopilot, qgroundcontrol, ardupilot.

Betaflight fork orqafpv/orqa-betaflight, branches with date of last commit:
- orqa-betaflight_2026.6-maintenance, 09ea1c692f, 2026-11-09. Newest. Contains position_estimator.c.
- orqa-betaflight_2025.12.5, f59f404fdf, 2026-08-21. Has pos_hold_multirotor.c but no position_estimator.c.
- orqa-betaflight_2025.12.1, bc5fde3a1a, 2026-07-10.
- PBAS, ccdaa548fc, 2025-09-22. master, cfc71c9378, 2026-01-22.
- Older: 4.0 to 4.4 maintenance, 4.4.1-custom_osd, 4.5.2, 4.6.x-vtxOsd, ghst-rx-num, feature branches.

Target files live in a separate repository, orqafpv/orqa-betaflight-config. Its branches orqa-betaflight-configs-2025.12.1, 2025.12.5 and 2026.6 each have configs/ORQAAPB/config.h. They also have ORQA_H7QUADCORE, ORQA_H743_MB, ORQAF405 and ORQAF7. The board name in ORQAAPB is "APB", MCU STM32H743.

Most likely firmware for the event aircraft: branch orqa-betaflight_2026.6-maintenance with target ORQAAPB. Reason: it is the newest and it is the only branch with the optical-flow position estimator. This is an inference. The organizer may flash a different build.

INAV fork orqafpv/orqa-inav: branches MRM2-6.1.1-Build, MRM2-7.1.2-buid, MRM2-build (target ORQAF405, ORQAF722, ORQAF405_SOC), ORQA-F405-target, ORQA-H743-WING (target ORQAH743WING, a wing), master (47f6afde69, 2026-08-20, merge of INAV release 9.1). Names with APB, QuadCore or MRM2-10AI: not found in any INAV branch (searched target trees of master, MRM2-*, ORQA-H743-WING).

## 2. Position hold in the 2026.6 branch

Repository orqafpv/orqa-betaflight, branch orqa-betaflight_2026.6-maintenance, commit 09ea1c692fc733a9137737ce26f16b66bb1914f7.
- src/main/flight/pos_hold_multirotor.c exists (140 lines). src/main/flight/position_estimator.c exists (756 lines).
- The ORQAAPB config.h does not set USE_GPS, USE_POSITION_HOLD or USE_OPTICALFLOW. The common header turns them on for flash size 1024 kB or more when the build is not CLOUD_BUILD: USE_ALTITUDE_HOLD, USE_POSITION_HOLD, USE_GPS, USE_RANGEFINDER, USE_OPTICALFLOW_MT (src/main/target/common_pre.h lines 289 to 315). STM32H7 flash size is 2048 kB (src/platform/STM32/mk/STM32H7.mk line 124). A cloud build may differ. This is unknown.
- The ORQAAPB config.h itself sets: USE_BARO with DPS310 on I2C2 (lines 40 to 41, 142 to 143), USE_MAG on I2C1 (line 43, 144), GPS_UART = USART7 (line 140), MSP_UART = UART4 (line 147, pins PC10 and PC11 at lines 68 to 69), serial RX GHST on USART3 (lines 138 to 139). USE_MAG_RM3100 is commented out (line 42).
- The magnetometer on the APB is an IIS2MDC in the ArduPilot hwdef. The Betaflight driver compass_lis2mdl.c line 31 says the LIS2MDL and IIS2MDC are firmware compatible. So the mag driver probably matches (inference).

## 3. Position sources the code accepts

| Source | Accepted | Where |
|---|---|---|
| gps_provider = MSP with MSP_SET_RAW_GPS (201) | Yes | msp.c lines 4014 to 4022 |
| MSP2_SENSOR_GPS (0x1F03) | Yes | msp.c lines 3986 to 4012 |
| Optical flow by MSP: MSP2 0x1F02 (name MSP2_SENSOR_OPTICALFLOW_MT) | Yes, when opticalflow_hardware = MT | msp.c lines 3980 to 3982, rangefinder_lidarmt.c lines 125 to 193 |
| Rangefinder by MSP: MSP2 0x1F01 (name MSP2_SENSOR_RANGEFINDER_LIDARMT) | Yes, when rangefinder_hardware is MTF01, MTF02, MTF01P or MTF02P | msp.c lines 3975 to 3978, rangefinder_lidarmt.c lines 63 to 110 |
| MSP2_SENSOR_OPTICALFLOW 0x300B | Id only. No handler found (grep of msp.c) | msp_protocol_v2_betaflight.h line 32 |

The poshold_position_source setting has the values AUTO, GPS_ONLY and OPTICALFLOW_ONLY (cli/settings.c line 154 and 1184). The default is AUTO (pg/pos_hold_multirotor.c line 39).

## 4. Conditions before Position Hold starts (2026.6 branch)

- Mode switch: flight mode POS_HOLD_MODE needs armed, the POS HOLD switch (or failsafe, or an autopilot mission), an accelerometer, and throttle raised once (core.c lines 1187 to 1205).
- Estimator: sensorsOk() needs positionEstimatorIsValidXY() (pos_hold_multirotor.c lines 66 to 85). XY is valid when an XY measurement arrived in the last 2 s (position_estimator.c line 94, lines 633 to 638).
- Heading: required only when the estimate uses GPS and no optical flow. With a working flow sensor and valid XY, heading is not required (position_estimator.c lines 679 to 699). Heading is valid with a calibrated mag or the GPS-course heading (imu.c lines 217 to 230).
- Fusion starts only while armed and one of these is true: POS_HOLD_MODE, a GPS fix, or healthy flow plus healthy rangefinder (position_estimator.c lines 115 to 159).
- Optical flow is used only when: sensors flow and rangefinder present; flow message younger than 100 ms (opticalflow.c line 252, drivers/opticalflow/opticalflow.h line 31); rangefinder valid within 500 ms (rangefinder.c lines 71, 383 to 386); rangefinder height from 10 cm up to the smaller of rangefinder_max_range_cm (default 400) and poshold_opticalflow_max_range (default 400) (position_estimator.c lines 96, 517 to 530); flow quality above poshold_opticalflow_quality_min (default 30 of 100) (position_estimator.c lines 330 to 343). Altitude source must not be BARO_ONLY or GPS_ONLY (lines 512 to 515).
- Flow message is rejected when the rangefinder reading is below 80 mm, or when the last rangefinder message is older than 5000 us times the device delay (50 ms for MTF01, 100 ms for MTF02 and MTF02P) (rangefinder_lidarmt.c lines 43, 56 to 61, 169 to 191).
- GPS source only: feature GPS must be on (fc/init.c lines 803 to 804). A GPS fix state must be on. Satellite count has no check in the estimator. Accuracy fields hAcc, vAcc and sAcc are stored but the estimator does not use them. It uses the pdop field only to scale noise. A pdop below 100 (1.00) counts as unknown and gets 100 times the position variance (position_estimator.c lines 89 to 91, 313 to 321).
- Home point: not needed by the estimator. The GPS origin is the first fix after XY fusion starts (position_estimator.c lines 281 to 284, 372 to 375). GPS_cosLat stays 1.0 until a home point is set (gps.c lines 2901 to 2908, 2932 to 2944). Send latitude near 0 to avoid an east-west scale error (inference from those lines).
- Arming checks: a GPS arming block applies only if GPS rescue is configured (failsafe procedure GPS_RESCUE or a GPS RESCUE switch). It needs a fix and gps_rescue_min_sats, default 8 (core.c lines 417 to 427, gps_rescue_multirotor.c line 731, pg/gps_rescue_multirotor.c line 48).
- Velocity from MSP GPS: both MSP GPS messages leave gpsSol.velned at its old value (zero). The estimator feeds that as a velocity measurement (position_estimator.c lines 389 to 392). This pulls velocity to zero in the GPS route. This is read in source.

## 5. What happens when messages stop (Betaflight)

- Flow route: XY valid ends 2 s after the last flow update (position_estimator.c line 94). Flow data stops counting after 100 ms (opticalflow.c line 252).
- When sensors fail, autopilotAngle is set to 0 and the pilot sticks control the angle again (pos_hold_multirotor.c lines 115 to 119, pid.c lines 593 to 603). Position hold gives an OSD warning, POSHOLD_FAILED (osd_warnings.c line 398). Found no failsafe start from this loss. Failsafe starts only from receiver loss.
- Estimator re-anchors the hold when sensors return (pos_hold_multirotor.c lines 106 to 113).
- GPS route: MSP GPS timeout 2500 ms (gps.c line 90, lines 1531 to 1534). Then the fix state clears (gps.c lines 1559 to 1565).
- Sanity fence: 20 m minimum (autopilot_multirotor.c line 109). A fence violation lasting 1 s stops the hold (lines 114, 502 to 512).

## 6. Table of facts

Repository for rows 1 to 22: orqafpv/orqa-betaflight, branch orqa-betaflight_2026.6-maintenance, commit 09ea1c692fc733a9137737ce26f16b66bb1914f7, unless stated.

| # | Fact | Repository, branch, commit | File and lines | Confidence |
|---|---|---|---|---|
| 1 | APB target exists, MCU STM32H743, MSP_UART = UART4, GPS_UART = USART7, DPS310 baro, USE_MAG | orqa-betaflight-config, branch orqa-betaflight-configs-2026.6, 7fe0569069931d2cb976fa8521c69ff0f11ed2fb | configs/ORQAAPB/config.h 24 to 45, 138 to 147 | read in source |
| 2 | UART4 as default MSP port came in config commit 40a04bf7ab (2026-08-19). The firmware branch pins config commit aab82696d0 (2026-07-13), which lacks it | orqa-betaflight, same branch | .gitmodules, submodule entry src/orqa-betaflight-config | read in source |
| 3 | Position hold options on by default for flash 1024 kB or more | same | src/main/target/common_pre.h 289 to 315 | read in source |
| 4 | MSP2_SENSOR_GPS decode, 52-byte payload | same | src/main/msp/msp.c 3986 to 4012 | read in source |
| 5 | MSP_SET_RAW_GPS decode, 14-byte payload | same | msp.c 4014 to 4022 | read in source |
| 6 | MT flow and range messages routed to drivers | same | msp.c 3975 to 3983 | read in source |
| 7 | MT flow decode and units | same | src/main/drivers/rangefinder/rangefinder_lidarmt.c 125 to 193 | read in source |
| 8 | MT range decode | same | rangefinder_lidarmt.c 63 to 110 | read in source |
| 9 | Sensor detect names MT, MTF01, MTF02, MTF01P, MTF02P | same | sensors/sensors.c 143 to 165, rangefinder.c 139 to 149, opticalflow.c 91 to 135 | read in source |
| 10 | Position hold entry and failure logic | same | flight/pos_hold_multirotor.c 66 to 136 | read in source |
| 11 | XY fusion rules, 2 s timeout, GPS and flow feed | same | flight/position_estimator.c 94, 115 to 159, 346 to 420, 494 to 567, 633 to 647, 679 to 699 | read in source |
| 12 | Flight mode POS_HOLD_MODE conditions | same | fc/core.c 1187 to 1205 | read in source |
| 13 | MSP GPS provider needs no serial port. Default provider for APB is UBLOX | same | io/gps.c 419 to 424, pg/gps.c 34 to 39 | read in source |
| 14 | GPS timeout 2500 ms, MSP GPS update path | same | io/gps.c 90, 1512 to 1535, 1559 to 1565, 3048 to 3056 | read in source |
| 15 | One MSP command per serial task run. Serial task at 100 Hz | same | msp/msp_serial.c 491 to 515, fc/tasks.c 371 | read in source. Rate limit of 100 commands per second is inference |
| 16 | MSP v1 and v2 frame parser, checksums | same | msp/msp_serial.c 124 to 290, 325 to 392 | read in source |
| 17 | Set commands return an empty acknowledge reply | same | msp/msp_serial.c 437 to 439, msp.c 4807 to 4829 | read in source |
| 18 | Default MSP baud is 115200 | same | io/serial.c 293 to 296 | read in source |
| 19 | MSP override of RC channels (switch BOXMSPOVERRIDE, mask msp_override_channels_mask) | same | rx/msp_override.c 28 to 41, cli/settings.c 883 | read in source |
| 20 | MSP_SET_RAW_RC needs USE_RX_MSP | same | msp.c 2924 to 2941 | read in source |
| 21 | Orqa 2025.12.5 branch: position hold needs GPS fix and a healthy compass | orqa-betaflight, branch orqa-betaflight_2025.12.5, f59f404fdfb49fe9d0a9f7d202a33743bb397bb9 | flight/pos_hold_multirotor.c 67 to 80 | read in source |
| 22 | The firmware on the aircraft is this branch | not applicable | not applicable | inference only |
| 23 | INAV: MSP GPS, flow and rangefinder compiled by default (common.h) | orqafpv/orqa-inav, master, 47f6afde694e0c063ba0f586839e024dbdb6b1a4 | src/main/target/common.h 56, 80, 93 | read in source |
| 24 | INAV: sensor message layouts | same | src/main/msp/msp_protocol_v2_sensor_msg.h 25 to 60, msp_protocol_v2_sensor.h 20 to 26 | read in source |
| 25 | INAV: MSP GPS, flow, rangefinder handlers | same | src/main/io/gps_msp.c 80 to 113, io/opflow_msp.c 74 to 87, io/rangefinder_msp.c 75 to 81, fc/fc_msp.c 4648 to 4691 | read in source |
| 26 | INAV: MSP_SET_RAW_GPS (v1, 14 bytes) | same | fc/fc_msp.c 3076 to 3101 | read in source |
| 27 | INAV: flow position correction needs flow, surface and Z valid | same | navigation/navigation_pos_estimator_flow.c 62 to 101 | read in source |
| 28 | INAV: timeouts GPS 1500 ms, baro 200 ms, surface 400 ms, flow 200 ms | same | navigation/navigation_pos_estimator_private.h 45 to 48 | read in source |
| 29 | INAV: GPS fix needs 3D fix and satellites >= gps_min_sats (default 6). MSP GPS silent for 1000 ms clears the fix | same | io/gps.c 369 to 379, 582 to 588, io/gps_private.h 27, settings.yaml gps_min_sats | read in source |
| 30 | INAV: emergency landing starts after nav_position_timeout (default 5 s) with no valid position | same | navigation/navigation.c 2783 to 2797, settings.yaml nav_position_timeout | read in source |
| 31 | INAV: arming block without GPS fix and home when a nav mode is configured. nav_extra_arming_safety default ALLOW_BYPASS | same | navigation/navigation.c 4759 to 4783 | read in source |
| 32 | INAV: allow_dead_reckoning (default OFF) lets nav run with no GPS and no valid heading | same | navigation_pos_estimator.c 472 to 486, settings.yaml inav_allow_dead_reckoning | read in source |
| 33 | INAV: no APB or Orqa H7 multirotor target | orqa-inav, all branches checked by tree listing | not found | read in tree listing |

## 7. Message layouts we must send

All numbers little-endian. Frame v2 native: bytes `$` `X` `<`, then flags (1 byte, send 0), command (2 bytes), payload size (2 bytes), payload, then 1 checksum byte. The checksum is CRC-8 DVB-S2 (polynomial 0xD5, start value 0) over flags, command, size and payload (msp_serial.c lines 240, 277, 384 to 386). Frame v1: `$` `M` `<`, size (1 byte), command (1 byte), payload, checksum = XOR of size, command and payload (msp_serial.c lines 353 to 355). Reply frames use `>`.

### Betaflight, MSP2 0x1F01 rangefinder (5 bytes)
Source: rangefinder_lidarmt.c lines 63 to 66, 104 to 110.
- u8 quality (0 to 255, not used by the driver).
- i32 distanceMm. Millimeters. Negative means out of range. The driver divides by 10 to get cm.
- Needs rangefinder_hardware = MTF02P (or MTF02, MTF01, MTF01P).

### Betaflight, MSP2 0x1F02 optical flow (9 bytes)
Source: rangefinder_lidarmt.c lines 125 to 129, 162 to 193.
- u8 quality (0 to 255). The driver makes quality = value * 100 / 255. It must be above poshold_opticalflow_quality_min (30), so send 90 or more.
- i32 motionX, i32 motionY.
- The driver computes flow rate in rad/s = motion / 200 / dt. The dt is the time between the two arrivals at the FC. So send motion = 200 * (flow angle in rad over the interval). Rotation of the airframe is subtracted later by the FC from its own gyro (opticalflow.c lines 208 to 211). So our value must contain the rotation part too: raw flow = translation flow + rotation flow (inference). Axis names: X is right, Y is forward (position_estimator.c lines 542 to 545). Sign needs a bench check.
- Needs opticalflow_hardware = MT, poshold_position_source = AUTO or OPTICALFLOW_ONLY. Send at 12 Hz or faster so the 100 ms health check passes. Send the range message first, at 20 Hz or faster for MTF01 and at 10 Hz or faster for MTF02P.

### Betaflight, MSP2 0x1F03 GPS (52 bytes, GPS route)
Source: msp.c lines 3986 to 4011. Order and size:
1. u8 instance (ignored)
2. u16 gpsWeek (ignored)
3. u32 msTOW (stored, not used by the estimator)
4. u8 fixType (any non-zero value means fix)
5. u8 satellites (send 12 or more)
6. u16 hAcc, cm (stored, not used)
7. u16 vAcc, cm (stored, not used)
8. u16 sAcc, cm (stored, not used)
9. u16 pdop, value times 100 (send 100 to 150; below 100 counts as unknown)
10. i32 longitude, 1e-7 degree
11. i32 latitude, 1e-7 degree
12. i32 altitude, cm
13. i32 velocity north, cm/s (not stored as velned: not used by estimator)
14. i32 velocity east, cm/s (same)
15. i32 velocity down, cm/s (same)
16. u16 ground course, 0.01 degree
17. u16 true yaw (ignored)
18. u16 year, u8 month, u8 day, u8 hour, u8 minute, u8 second (ignored)

### Betaflight, MSP_SET_RAW_GPS = 201 (14 bytes, v1 or v2)
Source: msp.c lines 4014 to 4022. u8 fix, u8 satellites, i32 latitude (1e-7 degree), i32 longitude, u16 altitude in meters, u16 ground speed in cm/s. No pdop, no velocity, no course.

### INAV messages (orqa-inav master 47f6afde69)
- MSP2 0x1F01 rangefinder: u8 quality, i32 distanceMm. Same layout as above (msp_protocol_v2_sensor_msg.h lines 25 to 29). Value divided by 10 to cm, zero or negative is out of range (rangefinder_msp.c lines 64 to 81). Needs rangefinder_hardware = MSP.
- MSP2 0x1F02 optical flow: u8 quality, i32 motionX, i32 motionY (same file lines 25 to 29). The FC takes dt from the message arrival times. Rate in deg/s = motion * (1e6 / dt_us) / opflow_scale (opflow.c lines 208 to 216). So we choose opflow_scale. Quality = value * 100 / 255. It must reach 35 to become valid and drops below 10 (opflow.c lines 74 to 75, 179 to 191). Needs opflow_hardware = MSP. INAV subtracts body rate too (navigation_pos_estimator_flow.c lines 86 to 89).
- MSP2 0x1F03 GPS: u8 instance, u16 week, u32 msTOW, u8 fixType (2 = 2D, 3 or more = 3D), u8 satellites, u16 hAcc cm, u16 vAcc cm, u16 hVelAcc cm/s, u16 hdop, i32 longitude, i32 latitude, i32 altitude cm, i32 velN cm/s, i32 velE cm/s, i32 velD cm/s, u16 groundCourse (0.01 degree), u16 trueYaw, u16 year, u8 month, u8 day, u8 hour, u8 minute, u8 second. Total 52 bytes. INAV uses the velocity and the accuracy (gps_msp.c lines 80 to 113). The set commands give no reply (fc_msp.c line 4691).
- MSP_SET_RAW_GPS: payload must be exactly 14 bytes, same fields as Betaflight, speed in cm/s (fc_msp.c lines 3076 to 3101).
- Needs gps_provider = MSP and enough satellites (gps_min_sats, default 6).

## 8. Reading data back (question 7)

| Need | Betaflight message | Fields and resolution | Time stamp |
|---|---|---|---|
| Attitude | MSP_ATTITUDE (108), msp.c 1401 to 1405 | i16 roll and i16 pitch in 0.1 degree, i16 yaw in whole degrees. Reply 6 bytes | None |
| Attitude, finer | MSP_ATTITUDE_QUATERNION (167), msp.c 1407 to 1419 | 4 x i16, value / 32767 | None |
| Gyro rates | MSP_RAW_IMU (102), msp.c 1223 to 1242 | 9 x i16: accelerometer raw, gyro raw, mag raw. Gyro value = dps divided by the sensor scale (2000/32768 or 4000/32768 dps per count, from gyro_init.c 815 to 817 and accgyro.h 41 to 42). The reply does not state the scale | None |
| Barometer height | MSP_ALTITUDE (109), msp.c 1422 to 1429 | i32 estimated altitude in cm, i16 vario in cm/s | None |
| Rangefinder height | MSP_SONAR_ALTITUDE (58) | i32 cm | None |
| Flight mode, sensors | MSP_STATUS_EX (150), msp.c 1168 to 1217 | u16 sensor bits (bit 3 GPS, bit 4 rangefinder, bit 6 optical flow, line 1179), u32 mode bits. Mode bit position = index among active boxes. Read MSP_BOXIDS (119) once to map. POS HOLD box id is 11 (msp_box.c line 60). The bit shows the real flight mode (msp_box.c lines 392 to 404). It does not show a sensor failure | None |
| GPS state | MSP_RAW_GPS (106), msp.c 1606 to 1616 | fix, satellites, latitude, longitude, altitude m, speed, course, pdop | None |

Betaflight debug values: MSP_DEBUG (254) returns 8 x i16 (debug.h line 25). With debug_mode = OPTICALFLOW, values 0 and 1 are the processed flow x1000, values 2 and 3 are the gyro rate x1000 (opticalflow.c lines 203 to 206).

INAV equivalents (orqa-inav master, fc/fc_msp.c): MSP_ATTITUDE same resolution (lines 679 to 683). MSP_RAW_IMU accelerometer x512 per g, gyro in whole dps (lines 519 to 533, gyro.c lines 604 to 611). MSP_ALTITUDE: i32 est. Z cm, i16 Z velocity cm/s, i32 baro cm (lines 685 to 693). MSP2_INAV_FULL_LOCAL_POSE (0x2220): attitude plus position and velocity of all three axes (lines 695 to 704). MSP2_INAV_STATUS (0x2000): arming flags and mode bits (lines 498 to 517).

No message above carries a time stamp. Use the host receive time. Expect up to one serial task period (10 ms) of delay (inference).

Rate on a serial link at 230400 baud: 8N1 gives 23040 bytes per second. Sizes in bytes: v1 request 6, v2 request 9. ATTITUDE reply 12 (v1) or 15 (v2). RAW_IMU reply 24 (v1). ALTITUDE reply 12 (v1). Set flow frame v2: 18 bytes. Set range frame v2: 14 bytes. Betaflight acknowledge reply: 9 bytes. Example load: flow 25 Hz, range 25 Hz, acknowledgements 50 Hz, ATTITUDE 40 Hz in v1: about 3.7 kB/s, about 16 percent of the link. So the byte rate is not the limit. The limit is the FC: it handles one MSP command per serial task run, and the task period is 100 Hz (msp_serial.c lines 491 to 515, tasks.c line 371, same in INAV: msp_serial.c line 475, fc_tasks.c line 491). So all commands together reach about 100 per second per port (inference). Budget per second: flow 20, range 20, ATTITUDE 40, ALTITUDE 10, STATUS 5. That is 95. RAW_IMU and ATTITUDE both at 50 Hz do not fit with the sensor messages.

## 9. INAV answers for questions 2 to 5

- Position hold exists in INAV (navigation code, navigation.c). Build options: INAV common.h enables USE_GPS, USE_GPS_PROTO_MSP, USE_RANGEFINDER_MSP, USE_OPFLOW, USE_OPFLOW_MSP, USE_GPS_FIX_ESTIMATION for every target (common.h lines 54 to 94). Orqa has no APB target, so the target file for the event aircraft is not found. A generic H743 target from upstream could be used. This is unknown.
- Sources accepted: gps_provider = MSP (values UBLOX, MSP, FAKE, settings.yaml line 45), MSP2_SENSOR_GPS, MSP_SET_RAW_GPS, flow by MSP (opflow_hardware = MSP), rangefinder by MSP (rangefinder_hardware = MSP).
- Conditions: GPS fix state (3D fix and satellites >= gps_min_sats), home point set at arming (navigation.c 3375 to 3392), valid heading (calibrated mag or GPS heading, imu.c 962 to 965), position estimate error below inav_max_eph_epv, default 1000 cm (settings.yaml). With no GPS, flow position needs flow, rangefinder and Z valid (navigation_pos_estimator_flow.c 64 to 78). Set inav_allow_dead_reckoning = ON so a missing heading does not block (navigation_pos_estimator.c 484). The arming block applies when a nav mode is configured and home or position is not healthy (navigation.c 4759 to 4783). Bypass: hold yaw high with nav_extra_arming_safety = ALLOW_BYPASS.
- Messages stop: GPS cleared after 1000 ms (gps.c 582 to 588), flow after 200 ms, rangefinder after 400 ms. With no valid position for nav_position_timeout (5 s) INAV starts emergency landing (navigation.c 2783 to 2797, 2074).
- INAV route without GNSS receiver: send MSP2 GPS with our position and velocity. INAV uses both (navigation_pos_estimator.c 704 to 726).

## 10. Betaflight SITL (software in the loop) build, do not build here

Source: orqa-betaflight branch orqa-betaflight_2026.6-maintenance, commit 09ea1c692f, files under src/platform/SIMULATOR.
- 64-bit Linux: yes. SITL.mk has empty ARCH_FLAGS, so it builds with the native gcc. Libraries: -lm -lpthread -lc -lrt (mk SITL.mk lines 20 to 32). The README states Ubuntu on x86_64. Optional static build: option SITL_STATIC.
- MSP on TCP: yes. UART n listens on TCP port 5760 + n (drivers/serial_tcp.c lines 41, 119). UART1 is port 5761. The repository test harness src/test/sitl/sitl_harness.py uses port 5761.
- Other ports (UDP): FDM state in on 9003, RC in on 9004, motor out on 9002 (sitl.c lines 202 to 204). The FDM packet is 18 doubles: time, angular rate x3, acceleration x3, quaternion x4, velocity x3, position x3, pressure (target.h lines 233 to 242).
- GPS and position hold in SITL: yes. target.h defines USE_GPS, USE_VIRTUAL_GPS, USE_ALTITUDE_HOLD, USE_POSITION_HOLD, USE_FLIGHT_PLAN (lines 116 to 122). The default provider is VIRTUAL (pg/gps.c lines 34 to 36). In SITL canUseGPSHeading is true (imu.c lines 641 to 647). Set gps_provider = MSP by CLI to test our messages. Optical flow and rangefinder in SITL: probably compiled, because flash size is 2048 (target.h line 174). This is inference.
- Commands: `git clone --depth 1 -b orqa-betaflight_2026.6-maintenance https://github.com/orqafpv/orqa-betaflight.git`, then `cd orqa-betaflight`, then `make TARGET=SITL`. Output: obj/main/betaflight_SITL.elf. Run `./obj/main/betaflight_SITL.elf`. Config is stored in eeprom.bin in the run folder.
- Tools: gcc, GNU make, libc with pthread and rt, git. No ARM compiler needed (SITL.mk: PLATFORM_SDK := none). The dyad library is in lib/main/dyad. The config submodule is needed only with CONFIG=, not with TARGET= (mk/config.mk line 17, inference from that line).
- Extra check: sitl_harness.py needs Python 3 only (imports are all standard).

## 11. Second route: stick commands

The companion computer can send MSP_SET_RAW_RC (200) with its own position controller. In Betaflight this needs USE_RX_MSP (msp.c 2924 to 2941). The MSP override mode keeps the GHST receiver and lets the pilot switch (BOXMSPOVERRIDE) take channels back (rx/msp_override.c lines 28 to 41). Risks: the controller runs on a link with unknown delay and no time stamps, so a delayed stick command can cause a crash. The override must be set up by CLI and by a mode switch, and a wrong channel mask gives the companion computer control of throttle. Betaflight keeps its failsafe on the receiver, so a stuck companion computer holds the last stick values while the receiver is fine.

## 12. Unknowns only the real aircraft can answer, with checks under 5 minutes

1. Which firmware runs. Check: connect on UART4, send MSP_API_VERSION and MSP_FC_VARIANT (1 and 2) and MSP_BUILD_INFO (5). Read variant BTFL or INAV and the date.
2. Which UART is MSP and its baud. Check: CLI `serial`. UART4 is port index 3 (identifier 3) in the line. Set baud 230400 with `serial 3 1 230400 ...` after reading the real line. Source default is 115200 (io/serial.c 293).
3. Does the build have flow and MSP rangefinder. Check: CLI `set opticalflow_hardware =` and `set rangefinder_hardware =` must list MT and MTF02P. Error "invalid name" means not built.
4. Does position hold exist. Check: CLI `set poshold_position_source =` lists AUTO, GPS_ONLY, OPTICALFLOW_ONLY. Also check mode id 11 in MSP_BOXIDS (119).
5. Does the FC accept our sensors. Check: send range and flow frames for 10 s. Read MSP_STATUS_EX: sensor bit 4 (rangefinder) and bit 6 (flow) must be 1.
6. Flow sign and rotation. Check: debug_mode = OPTICALFLOW, move the aircraft 0.5 m forward by hand and read MSP_DEBUG values 0 and 1. Fix with opticalflow_rotation and opticalflow_flip_x. Rotate in place by hand: values 2 and 3 show the gyro part the FC subtracts.
7. Real command rate limit. Check: send 300 MSP_ATTITUDE requests in 1 s without waiting. Count the replies. Expect about 100.
8. Mag health. Check: MSP_STATUS_EX sensor bit 2 and MSP_ATTITUDE yaw while turning by hand.
9. Is feature GPS on (GPS route only). Check: CLI `feature`, and send MSP_SET_RAW_GPS then read MSP_RAW_GPS (106) for the fix byte.
10. Arming blocks. Check: read arming-disable flags in MSP_STATUS_EX with the flow stream on.
11. The firmware is INAV. Check: CLI `get opflow_hardware` and `get gps_provider`, then `status` for the sensor list.

## 13. Build outline for msp_link.py (15 lines)

1. Class MspLink(port, baud=230400) opens pyserial with a read thread and a write lock.
2. Function frame_v2(cmd, payload) builds `$X<`, flags 0, cmd, size, payload and CRC-8 DVB-S2.
3. Function frame_v1(cmd, payload) builds `$M<` with the XOR checksum, for reads.
4. A parser state machine reads `$M>` and `$X>` frames, checks the checksum and fills a dict {cmd: (host_time, payload)}.
5. Betaflight ignores no frames: the parser also drops the 9-byte acknowledge replies of set commands.
6. identify(): send commands 1, 2, 5 and return "BTFL" or "INAV" so that fc_link.connect does not stop.
7. send_range(distance_mm, quality) packs `<Bi` into 0x1F01 at 20 Hz.
8. send_flow(vx, vy, height, gyro_x, gyro_y, dt) computes flow rate = v/height plus the gyro part, then motion = 200 * rate * dt (Betaflight) or the INAV scale, packs `<Bii` into 0x1F02 at 20 Hz.
9. send_gps(lat0, lon0, east, north, up, vel, hdop) packs the 52-byte 0x1F03 frame with latitude near 0 at 10 Hz. It is the second option.
10. A poll scheduler sends ATTITUDE 40 Hz, ALTITUDE 10 Hz and STATUS_EX 5 Hz, and keeps the total under 95 commands per second.
11. get_attitude() returns roll and pitch in degrees (0.1 resolution) and yaw in degrees (1 resolution) with host time stamps.
12. get_mode() maps the MSP_BOXIDS result to bit positions and returns armed and pos_hold flags.
13. Settings helper prints the CLI lines to set for each firmware (see section 12), and does not write settings.
14. A stop flag stops all sends when VIO quality drops, so the FC sees a timeout and releases the hold.
15. Test file reads a frame dump and checks the CRC and the field order against sections 7 and 8.
