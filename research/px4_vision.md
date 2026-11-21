# PX4 (and ArduPilot) position hold from an external horizontal position, GNSS off

Date of research: 2026-11-15. Method: read PX4 source and docs on GitHub (`gh api`), the MAVLink XML, ArduPilot source and docs. No code was run.

Confidence labels: "source" = a file says it. "derived" = computed from source code lines (medium confidence). "inference" = the reasoning (low confidence).

Branch note. Parameter files were read on `release/1.16`, `release/1.15`, `release/1.14`, `release/1.13` and `main`. The code walk-through (receiver, EKF, commander) used `release/1.16`. The v1.14 and v1.15 code may differ in detail. The Orqa firmware version is unknown (see Unknowns).

---

## 1. PX4 EKF2: parameters for horizontal-only external vision, baro height, no GNSS

### Recommended set for v1.14, v1.15, v1.16 (source for each name and default; the combination is the choice)

| Parameter | Value | Why | Default (1.14 / 1.15 / 1.16) |
|---|---|---|---|
| EKF2_EV_CTRL | 1 | Bit 0 only = horizontal position. Bit 1 vertical position, bit 2 3D velocity, bit 3 yaw stay off. | 15 / 15 / 0 |
| EKF2_GPS_CTRL | 0 | Turns off all GNSS fusion (lon/lat, altitude, velocity, dual-antenna heading). | 7 / 7 / 7 |
| EKF2_HGT_REF | 0 | Barometric pressure is the height reference (0 baro, 1 GPS, 2 range, 3 vision). Needs reboot. | 1 (GPS) in all three |
| EKF2_BARO_CTRL | 1 | Baro height fusion on. | 1 |
| EKF2_RNG_CTRL | 0 (if no range finder) | Range finder aiding off. At 100 m a range finder is not useful. | 1 |
| EKF2_EV_DELAY | measured, see section 3 | Delay of the vision data relative to the IMU, in ms. Range 0 to 300. Needs reboot. | 0 / 0 / 0 (it was 175 in v1.13) |
| EKF2_EV_NOISE_MD | 0 (use our variance) or 1 (use parameters) | 0: noise comes from the message, and EKF2_EVP_NOISE is a lower bound. 1: parameters only. | 0 |
| EKF2_EVP_NOISE | about 0.1 to 0.5 m (lower bound, min 0.01) | Position noise, in metres (standard deviation). | 0.1 |
| EKF2_EVP_GATE | 5 (SD) | Innovation gate for vision position (min 1). | 5.0 |
| EKF2_EV_POS_X, _Y, _Z | camera position in body frame, metres, from the centre of gravity | Offset of the camera focal point. Forward, right, down. | 0 |
| EKF2_EV_QMIN | 0 | Minimum quality from ODOMETRY.quality. Optional. | 0 |
| EKF2_NOAID_TOUT | 5000000 us (min 0.5 s, max 10 s) | Maximum inertial dead-reckoning time before the position is called invalid (see section 4). | 5 s |
| EKF2_NOAID_NOISE | 10 m | Noise of the fake "hold last position" fusion after dead reckoning time runs out. | 10 m |
| EKF2_MAG_TYPE | keep the default (Automatic) | Magnetometer is the yaw source with no GNSS. | 0 |
| COM_ARM_WO_GPS | 1 | Arming without GNSS is allowed. | 1 |
| COM_POSCTL_NAVL | 0 (Altitude) or 1 (Land) | Reaction when position control cannot run (section 4). | 0 |

Sources:
- `EKF2_EV_CTRL` bit definitions and `EKF2_EV_DELAY`, `EKF2_EV_NOISE_MD`, `EKF2_EVP_NOISE`, `EKF2_EVP_GATE`, `EKF2_EV_POS_*`, `EKF2_EV_QMIN`: https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/params_external_vision.yaml (and `main`). For v1.15 they sit in https://github.com/PX4/PX4-Autopilot/blob/release/1.15/src/modules/ekf2/module.yaml. For v1.14 they sit in https://github.com/PX4/PX4-Autopilot/blob/release/1.14/src/modules/ekf2/ekf2_params.c.
- Default of `EKF2_EV_CTRL`: 15 in v1.14 (`PARAM_DEFINE_INT32(EKF2_EV_CTRL, 15)`) and v1.15 (module.yaml `default: 15`), 0 in v1.16 and main. Source. This matters: on v1.14 and v1.15 a vision stream is fused in all four ways unless you set the parameter to 1.
- `EKF2_HGT_REF` values and default 1: https://github.com/PX4/PX4-Autopilot/blob/release/1.15/src/modules/ekf2/module.yaml
- Docs summary of the same parameters: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/ros/external_position_estimation.md and https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/advanced_config/tuning_the_ecl_ekf.md (section "External Vision System": bits 0 horizontal position, 1 vertical position, 2 velocity, 3 yaw).
- The tuning doc table "Typical configurations" lists the GNSS, baro, range and height reference settings.

### Older firmware (v1.13 and older): EKF2_AID_MASK

Source: https://github.com/PX4/PX4-Autopilot/blob/release/1.13/src/modules/ekf2/ekf2_params.c

- `EKF2_AID_MASK` bits in v1.13: 0 use GPS, 1 use optical flow, 2 inhibit IMU bias estimation, 3 vision position fusion, 4 vision yaw fusion, 5 multi-rotor drag fusion, 6 rotate external vision (EV data is in a non-NED frame and must be rotated), 7 GPS yaw fusion, 8 vision velocity fusion. Default 1 (GPS only).
- Value for our case in v1.13: `EKF2_AID_MASK = 8` (vision position only, GPS bit cleared).
- Height in v1.13: `EKF2_HGT_MODE = 0` (barometric). It has values 0 baro, 1 GPS, 2 range, 3 vision. It was renamed `EKF2_HGT_REF` in v1.14.
- In v1.14 and v1.15 `EKF2_AID_MASK` still exists but every bit is marked "Deprecated, use EKF2_GPS_CTRL / EKF2_OF_CTRL / EKF2_EV_CTRL ..." and the default is 0 (v1.14 `ekf2_params.c`). Do not use it there.
- Other v1.13 names: `EKF2_EV_NOISE_MD`, `EKF2_EVP_NOISE`, `EKF2_EVP_GATE`, `EKF2_EV_DELAY` (default 175 ms), `EKF2_EV_POS_X`, `EKF2_NOAID_TOUT` (5000000 us) all exist in v1.13.

### Other parameters that matter

- Reboot after changing parameters (docs: "Reboot the flight controller in order for parameter changes to take effect"). `EKF2_EV_DELAY` and `EKF2_HGT_REF` are `reboot_required`.
- `SDLOG_PROFILE` bit 7 (Computer Vision and Avoidance) logs the EV rates for delay tuning (docs).
- `EKF2_GPS_MODE`, `EKF2_POS_LOCK` exist in main for GNSS handling. They do not matter with `EKF2_GPS_CTRL=0`.
- If GNSS is only "degraded" and still on: EV position then fuses through a bias estimator and its variance is raised to the GNSS noise `EKF2_GPS_P_NOISE` (source, `ev_pos_control.cpp`). Our recommendation is `EKF2_GPS_CTRL=0` for the event, so a bad GNSS fix cannot pull the position.
- Start of fusion with no GNSS: the EKF resets its horizontal position to the first vision measurement (log text "starting EV position fusion, resetting state"). Source: `ev_pos_control.cpp` `startEvPosFusion`.

---

## 2. ODOMETRY or VISION_POSITION_ESTIMATE

### How PX4 handles each (source: `mavlink_receiver.cpp`, release/1.16)

https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/mavlink/mavlink_receiver.cpp

VISION_POSITION_ESTIMATE (id 102): fields `usec` (us), `x`, `y`, `z` (m), `roll`, `pitch`, `yaw` (rad), `covariance[21]` (extension, row-major upper triangle of the 6x6 pose covariance), `reset_counter` (extension). MAVLink XML: https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml
- PX4 always labels the pose as NED (`POSE_FRAME_NED`). There is no frame field.
- Variances read: `covariance[0]` (x), `[6]` (y), `[11]` (z) as position variance; `[15]`, `[18]`, `[20]` as roll, pitch, yaw variance.
- Unknown covariance: XML says "assign NaN to the first element".
- It cannot carry velocity.

ODOMETRY (id 331): fields `time_usec`, `frame_id`, `child_frame_id`, `x y z`, `q[4]` (w,x,y,z), `vx vy vz`, `rollspeed pitchspeed yawspeed`, `pose_covariance[21]`, `velocity_covariance[21]`, `reset_counter`, `estimator_type`, `quality` (int8, percent; -1 failed, 0 unknown, 1 worst, 100 best).
- `frame_id` handling in PX4: `MAV_FRAME_LOCAL_NED` (1) = NED pose; `MAV_FRAME_LOCAL_ENU` = converted to NED; `MAV_FRAME_LOCAL_FRD` (20) = FRD frame (origin fixed, x forward in the horizontal plane); `MAV_FRAME_LOCAL_FLU` = converted to FRD. Other values: the position is not copied.
- `child_frame_id` is for the velocity: `LOCAL_NED`, `LOCAL_ENU`, `LOCAL_FRD`, `LOCAL_FLU`, `BODY_FRD` (12), and the deprecated `BODY_NED`, `BODY_OFFSET_NED`.
- The EKF2 docs list ODOMETRY with `frame_id = MAV_FRAME_LOCAL_FRD` for VIO. The receiver code also accepts `LOCAL_NED`.
- `estimator_type`: PX4 accepts UNKNOWN (0, legacy), NAIVE (1), VISION (2), VIO (3) for the vision path, and MOCAP (4) for the mocap topic (EKF2 does not read the mocap topic). Other types print "unsupported estimator_type" and the message is dropped.
- IMPORTANT: if `estimator_type == MAV_ESTIMATOR_TYPE_NAIVE`, PX4 ignores all covariance fields. Use VISION (2) or VIO (3) so the covariance is used.
- Only ODOMETRY can carry velocity into EKF2 (docs: "The odometry message is the only message that can send also linear velocities to PX4").

### How to mark fields "unknown" (source: receiver and `EKF2::UpdateExtVisionSample`)

https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/EKF2.cpp

- Position: the check is "all three of x, y, z are finite". If any one is NaN, the whole position is dropped. So send a finite `z` (for example 0 or the baro height). `EKF2_EV_CTRL` bit 1 is off, so z is not fused.
- Position variance: the check is "all three variances finite". If one of `[0]`, `[6]`, `[11]` is NaN, the whole vector is replaced by `EKF2_EVP_NOISE`^2. So send a finite z variance too (for example 1.0).
- Velocity: all three must be finite, and the frame must be valid, or velocity is ignored. Send NaN for unknown velocity.
- Orientation: the quaternion must be finite, not all zero, every element at most 1, and the norm within 1e-5 of 1. Otherwise it is ignored. For ODOMETRY send NaN in `q`. For VISION_POSITION_ESTIMATE, NaN angles give a NaN quaternion, which is ignored.
- Variances are VARIANCES in m^2 (not standard deviations).

### How PX4 uses the covariance (source: `EKF2.cpp` lines in `UpdateExtVisionSample`, `ev_pos_control.cpp`)

- With `EKF2_EV_NOISE_MD = 0`: used variance = max(`EKF2_EVP_NOISE`^2, message variance). The parameter is a lower bound. If the message variance is NaN, the parameter alone is used.
- With `EKF2_EV_NOISE_MD = 1`: only `EKF2_EVP_NOISE`^2.
- The EKF then also applies max(variance, 0.01^2) (hard floor of 1 cm).
- Innovation test: innovation against (state variance + observation variance), gate = max(`EKF2_EVP_GATE`, 1) SD. A rejected sample is not fused.
- With `LOCAL_FRD` and no EV yaw fusion, the position variance is raised to at least the largest EV orientation variance (code comment: "TODO: do this properly").

### reset_counter (source: `ev_control.cpp`, `ev_pos_control.cpp`)

- `ev_reset = (sample.reset_counter != previous.reset_counter)`.
- On a change, with no GNSS, the EKF resets its horizontal position state to the new measurement (hard jump, no gate). It also resets the EV-to-EKF attitude filter.
- Use: increment it when our keyframe changes and the position frame jumps. If we keep the frame continuous, do not increment it.

### Recommendation (the choice)

- Use VISION_POSITION_ESTIMATE if we send only position. It is simple, always NED, and smaller on the wire: payload 117 bytes, MAVLink v2 frame 129 bytes (payload + 12; trailing zero bytes can be cut by v2, but NaN is not zero). ODOMETRY is 233 bytes payload, 245 bytes frame. At 30 Hz that is about 3.9 kB/s versus 7.4 kB/s (derived from the XML field sizes).
- Use ODOMETRY only if we later send velocity (optical-flow fallback, section 5). Then use `frame_id = LOCAL_NED`, `child_frame_id = LOCAL_NED`, `estimator_type = 2 (VISION)`.
- Our frame: send NED, aligned to the flight controller's own heading (see section 6). Position (0,0) is the anchor.

---

## 3. Timestamps: TIMESYNC, EKF2_EV_DELAY, a 50 ms error

### TIMESYNC (source: `Timesync.cpp`, `mavlink_timesync.cpp`, `mavlink_main.cpp`)

https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/lib/timesync/Timesync.cpp

- Message TIMESYNC (id 111): `tc1`, `ts1` in nanoseconds. Request: `tc1 = 0`, `ts1` = sender time. Response: `ts1` mirrored, `tc1` = responder time.
- PX4 replies to a request from the companion (it sets `tc1 = its time`). In onboard mode PX4 also streams TIMESYNC requests at 10 Hz (`configure_stream_local("TIMESYNC", 10.0f)`); the companion must answer with its own clock in `tc1`. MAVROS does this. A custom program must do it itself.
- Offset estimate = (ts1 + now - 2*tc1)/2, assuming equal delay both ways. A two-state filter (offset and skew) smooths it. Gains go from 0.05 to 0.003.
- Samples with round-trip time above 10 ms (`MAX_RTT_SAMPLE`) are dropped. A sample more than 100 ms from the estimate is dropped; 10 such samples in a row reset the filter ("time jump detected").
- Converged after 500 accepted samples (`CONVERGENCE_WINDOW`). At 10 Hz that is at least about 50 s (derived).
- `sync_stamp(usec)` returns `usec + offset` only when converged. If it has NOT converged, it returns PX4's own arrival time (`hrt_absolute_time()`). So before convergence the timestamp we send is ignored and the EKF sees "now".
- The timestamp we put in `usec` must be on the same clock as the companion's TIMESYNC replies.
- Risk (inference, low): on a 230400 baud link that is busy, round-trip time can exceed 10 ms, and then TIMESYNC samples are rejected and sync never converges. Keep the link lightly loaded (section 9).

### EKF2_EV_DELAY (source: `estimator_interface.cpp`, `setExtVisionData`)

https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/EKF/estimator_interface.cpp

- Sample time used by the EKF = `timestamp_sample` - `EKF2_EV_DELAY` - 0.5 * EKF step.
- `timestamp_sample` = `sync_stamp(usec)` for MAVLink input.
- So: if we stamp the true image capture time and the clocks are synced, `EKF2_EV_DELAY` should be near 0. If we stamp "now" (arrival or end-of-processing time), set `EKF2_EV_DELAY` = the delay between capture and that stamp (our 60 to 100 ms). The docs say the value is the difference between the vision timestamp and the "actual" capture time on the IMU clock, and should be tuned from logs (compare EV against the onboard estimate) and by minimizing innovations in dynamic manoeuvres.
- For a rolling shutter camera the "capture time" should be the mid-exposure of the reference row we report. Which row we report is our choice (inference).
- PX4 drops samples that arrive faster than a minimum interval (log "EV data too fast"). That interval is derived from the delay buffer length. 30 Hz should be fine (inference).

### A 50 ms timestamp error (derived and inference)

- No source gives a number. The EKF pairs each measurement with the state at the stamped time. A time error `dt` gives an innovation error of about (vehicle speed x `dt`). Example: 0.5 m/s x 0.05 s = 2.5 cm. Near hover the effect is small. During manoeuvres or wind gusts it grows linearly with speed.
- Nothing inflates the variance for a time error. A constant error in `EKF2_EV_DELAY` acts like a lag that can make the loop less damped (inference, low).
- A larger danger is the not-converged case above: the stamp is replaced by arrival time and the whole transport and processing latency is unmodelled unless `EKF2_EV_DELAY` covers it.

---

## 4. Rate, and what happens when vision stops

### Rate (source: docs)

- Docs: "The messages should be streamed at between 30Hz (if containing covariances) and 50 Hz. If the message rate is too low, EKF2 will not fuse the external vision messages." (https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/ros/external_position_estimation.md)
- Code: `EV_MAX_INTERVAL = 200 ms` (`common.h`). To START fusion, the gap between samples must be under 200 ms, and the previous and newest samples must be recent.
- Our 30 Hz is inside the documented range.

### Stop timeline (derived from `release/1.16` code; defaults)

Files: https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/EKF/aid_sources/external_vision/ev_control.cpp , `ekf_helper.cpp` (`updateHorizontalDeadReckoningstatus`), `aid_sources/fake_pos_control.cpp`, `common.h`.

| Time after the last vision sample | What happens |
|---|---|
| 0 to about 0.4 s | Nothing yet. The EKF runs on IMU and baro. Position is still reported valid. |
| about 0.4 s (2 x `EV_MAX_INTERVAL`) | Log "vision data stopped". EV position fusion is switched off (`stopEvPosFusion`). The EKF is now in inertial dead reckoning. No correction. |
| 0.5 s | As above. Position still valid. The position error grows with IMU error. There is no number for the growth. |
| 2 s | Same. Position still valid. When samples return and start conditions pass, the EKF restarts fusion and RESETS its position to the new measurement (a jump back to the anchor). |
| about 4.4 s (derived: EV drop at 0.4 s, plus `EKF2_NOAID_TOUT` 5 s minus a 1 s offset in the code) | Horizontal dead-reckon time exceeded. `xy_valid` goes false. The EKF begins "fake position" fusion at the last known position with noise `EKF2_NOAID_NOISE` = 10 m (weak). Local position is invalid. |
| 10 s | Position control has been unavailable for about 5 s. Vision restart (after 1 s probation, see below) brings it back. |

The 4.4 s figure is an estimate. It depends on `EKF2_NOAID_TOUT` (0.5 to 10 s) and on how far the EKF delayed time is behind real time (up to the largest sensor delay). Test it in SITL with a vision stream that stops (not done here).

### Mode and failsafe (source: commander code and docs)

- Mode requirements (v1.16 `ModeUtil/mode_requirements.cpp`): Position mode (POSCTL) needs `local_alt` and `local_position_relaxed` (position valid, accuracy ignored). Hold (AUTO_LOITER), Mission, RTL, Takeoff, Orbit need `local_position`, and some also `global_position` (Hold and Mission both set `mode_req_global_position`). Offboard needs only attitude, angular velocity and the offboard signal.
- Without GNSS the EKF has no global origin. Docs: use `SET_GPS_GLOBAL_ORIGIN` to set the EKF origin; "EKF will then provide a global position estimate based on origin and local frame position" (docs `external_position_estimation.md`, section "Enabling Auto Modes with a Local Position"). So Hold mode needs this message. Position mode docs prose says "Global position estimate is required", but the auto-generated requirement list in the same page says only local position (relaxed). Not tested which is true on the Orqa build.
- Validity logic (`estimatorCheck.cpp`): the local position is "invalid" if `xy_valid` is false, or the data is older than 1 s, or `eph` is above the threshold. For the hysteresis: once valid, the threshold for invalid is 2.5 x `COM_POS_FS_EPH`. After it fails, it needs 1 s without failure before it is valid again.
- `COM_POS_FS_EPH`: default 5 m, -1 disables. `COM_VEL_FS_EVH`: default 1 m/s.
- Fallback for POSCTL (`failsafe.cpp`, `checkModeFallback`): `COM_POSCTL_NAVL = 0` -> Altitude mode (then Stabilized if altitude is also lost). `= 1` -> Land, then Descend. Docs (`config/safety.md`): "Multicopters flying in a position-dependent manual mode, such as Position mode, will switch to Altitude mode if a height estimate is available".
- Autonomous modes or manual control lost: Return (needs global position and home), then Land, then Descend (docs).
- `COM_POS_FS_DELAY` (default 1 s, min 1, max 100: "position checks need to be failed this long before the failsafe activates") exists in v1.13, v1.14 and v1.15. It is NOT in the v1.16 `commander_params.c`. Source: release branches' `commander_params.c`.
- `COM_POS_LOW_EPH` (default -1 = off) and `COM_POS_LOW_ACT` trigger a warning or action when the position is valid but `eph` is above the threshold. Docs: the check only runs on hovering vehicles. Related: `COM_FAIL_ACT_T` (5 s) is the wait in Hold before a failsafe action in some failsafe paths.
- `COM_POS_FS_ACT` (Descend or Terminate) appears in docs for `main` ("PX4 v2.0") only.
- Altitude mode does NOT hold horizontal position. The drone then drifts unless the pilot corrects (inference from the mode definition).

---

## 5. Velocity from vision, and OPTICAL_FLOW_RAD from the companion

### Vision velocity

- Yes. Use ODOMETRY with finite `vx vy vz`, set `child_frame_id` (`LOCAL_NED`, `LOCAL_FRD`, `BODY_FRD`), and set `EKF2_EV_CTRL` bit 2 (add 4). Source: `EKF2.cpp` (velocity frame switch) and `ev_control.cpp`.
- Related parameters: `EKF2_EVV_NOISE` (default 0.1 m/s, min 0.01), `EKF2_EVV_GATE` (default 3 SD, min 1), `EKF2_EV_NOISE_MD`. Velocity variance comes from `velocity_covariance[0], [6], [11]`.
- Docs: ODOMETRY with `twist` in body frame must really be in body FRD, "not in inertial frame".
- For an optical-flow fallback from our own tracker, sending metres per second through ODOMETRY is better than OPTICAL_FLOW_RAD (inference): we already know the height from baro and the attitude.

### OPTICAL_FLOW_RAD from a companion

Sources: `mavlink_receiver.cpp` `handle_message_optical_flow_rad`; `VehicleOpticalFlow.cpp`; `sensor_params_flow` (SENS_FLOW_*); `optical_flow_control.cpp`; docs https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/sensor/optical_flow.md

- PX4 accepts OPTICAL_FLOW_RAD. It IGNORES the message `time_usec`: it stamps the sample with PX4's arrival time (`timestamp_sample = hrt_absolute_time()`). Delay is then set by `EKF2_OF_DELAY` (default 20 ms; 0 to 300).
- Uses: `integrated_x/y` (rad), `integration_time_us`, `quality`, `integrated_xgyro/ygyro/zgyro` (if finite, used as the sensor's own gyro), and `distance` (m, negative = unknown; if finite and no DISTANCE_SENSOR for 1 s, used as a range sample).
- MAVLink path leaves max flow rate, min and max ground distance as NaN, so the sensor module substitutes the SENS_FLOW limits: `SENS_FLOW_MAXR` 8 rad/s, `SENS_FLOW_MINHGT` 0.08 m, `SENS_FLOW_MAXHGT` 100 m (max allowed value of the parameter is 100). Release/1.16 defaults.
- Flow fusion continues only while height above ground (HAGL) is within [min, max]. So at 100 m we are at the edge; above 100 m flow is not fused (derived).
- Enable with `EKF2_OF_CTRL` (default 0 in v1.15, 1 in v1.16 `params_optical_flow.yaml`). Others: `EKF2_OF_N_MIN` 0.15 rad/s, `EKF2_OF_N_MAX` 0.5 rad/s, `EKF2_OF_QMIN` 1, `EKF2_OF_QMIN_GND` 0, `EKF2_OF_GATE` 3 SD, `EKF2_OF_DELAY` 20 ms, `EKF2_OF_GYR_SRC`, `EKF2_OF_POS_X/Y/Z`.
- Docs (`tuning_the_ecl_ekf.md`): flow is used if "Valid range finder data is available", `EKF2_OF_CTRL` is set and quality is above `EKF2_OF_QMIN`. Code detail: start also passes if a terrain estimate is valid or other horizontal aiding is active (so range is not strictly mandatory when EV position is active; inference from code, low confidence that it works at 100 m).
- Docs: "slow oscillations occur at higher altitude" above about 10 to 20 m; reduce `SENS_FLOW_SCALE` (default 1, range 0.5 to 1.5).
- Noise scale (derived): `EKF2_OF_N_MIN` 0.15 rad/s at 100 m height equals about 15 m/s of velocity noise per sample. A real hover drift of 1 m/s is 0.01 rad/s. So PX4's flow fusion has almost no weight at 100 m with default noise. Conclusion: do not rely on OPTICAL_FLOW_RAD at 100 m. Send velocity through ODOMETRY with our own variance.

---

## 6. Heading and frame alignment

- With no GNSS, heading comes from the magnetometer (`EKF2_MAG_TYPE`, default Automatic: heading fusion on ground, 3-axis in flight). The docs have no typical yaw error figure. Found no official number. Anecdotal forum report: about 5 degrees against dual-antenna GNSS heading with only the magnetometer, 2 degrees after a mag yaw rotation fix (https://discuss.px4.io/t/high-mag-accuracy-possible/48146, from a web search summary; low confidence, anecdotal).
- Docs name the symptom of a heading mismatch: "toilet-bowling" when VIO frames do not match (https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/computer_vision/visual_inertial_odometry.md). ArduPilot docs say the same about missing yaw alignment (VIO page, below).
- Code for the frames (`ev_pos_control.cpp`):
  - `LOCAL_FRAME_NED` (VISION_POSITION_ESTIMATE, or ODOMETRY with `LOCAL_NED`): used directly, and ONLY when the EKF yaw is aligned (`yaw_align`). Otherwise the sample is not fused. The measurement is assumed to be in the true north-east frame that the EKF uses.
  - `LOCAL_FRAME_FRD` with `EKF2_EV_CTRL` yaw bit (bit 3): EKF uses the EV frame as is, and heading is relative to the EV frame ("if yaw data is used the heading is with respect to the external vision frame; otherwise the heading is relative to North").
  - `LOCAL_FRAME_FRD` without the yaw bit: the EKF rotates the EV position into its own frame using a filtered difference between the EV quaternion and the EKF quaternion (`_ev_q_error_filt`). This needs a valid EV quaternion.
- Effect of a yaw error (derived and inference): the EKF integrates the IMU into north/east using its own yaw. If the position measurement is in a frame rotated by a different yaw, the velocity and position states disagree, which gives circling errors (toilet-bowl). In our design the measurement is built from the flight controller's own attitude and heading, so EV and EKF share the same yaw by construction (inference, low). Then a mag error mainly rotates the frame, and the hold target (the anchor, offset 0) stays fixed. Remaining danger: a yaw CHANGE of the EKF in flight (mag reset, "Emergency yaw reset") while our estimator keeps old keyframe geometry.
- Better to use PX4's "EV frame alignment"? Our EV has no independent yaw (its yaw is the FC's yaw), so fusing it with the yaw bit adds nothing, and sending `LOCAL_FRD` without the yaw bit just makes PX4 compute a rotation of about identity. Recommendation: send NED, keep the yaw bit off, and keep our tracker using the FC's current yaw each frame (the choice, low confidence; test in SITL with a yaw offset).
- The unit of our position frame: NED, x north, y east. A mistake in this convention is hidden in the simulator (the team notes say the simulator shares geometry with the estimator). Test it against a real PX4 SITL (see Recommendations).

---

## 7. Position-hold tuning and noise to attitude jitter

### Parameters (source: `multicopter_position_control_gain_params.yaml`, `..._limits_params.yaml`, `..._params.yaml`, main; `PositionControl.cpp` release/1.16)

| Parameter | Default | Role |
|---|---|---|
| MPC_XY_P | 0.95 (1/s), range 0 to 2 | Position error to velocity setpoint. |
| MPC_XY_VEL_P_ACC | 1.8, range 1.2 to 5 | Velocity error to acceleration setpoint. |
| MPC_XY_VEL_I_ACC | 0.4, range 0 to 60 | Integral of velocity error. |
| MPC_XY_VEL_D_ACC | 0.2, range 0.1 to 2 | Damping on measured acceleration. |
| MPC_VELD_LP | 5 Hz | Low-pass for the D term. |
| MPC_VEL_LP | 0 Hz (off) | Low-pass on the velocity estimate used by the controller. |
| MPC_VEL_NF_FRQ / MPC_VEL_NF_BW | 0 / 5 Hz | Notch filter on velocity. |
| MPC_XY_VEL_MAX | 12 m/s | Limit on horizontal velocity. |
| MPC_TILTMAX_AIR | 45 deg | Limit on tilt. |
| MPC_ACC_HOR_MAX | see docs | Horizontal acceleration limit in Position mode. |

- Control law in code: `vel_sp = (pos_sp - pos) * MPC_XY_P`; then `acc_sp = (vel_sp - vel) * MPC_XY_VEL_P_ACC + integral - D term`; then thrust vector tilt, limited to the tilt limit.
- Noise path (derived, simple model): a position error `e` gives a velocity setpoint `0.95 e` m/s, an acceleration setpoint of about `1.8 x 0.95 e` m/s^2 if the velocity estimate lags, and a tilt of about `a/g`. Example: 0.1 m of position noise gives about 0.17 m/s^2, or about 1 degree of tilt; 1 m of noise gives about 10 degrees. These are rough, because the EKF filters the noise first and the controller reads the EKF velocity, not the raw measurement.
- The EKF acts as the filter: a larger `EKF2_EVP_NOISE` (or larger reported variance) lowers the Kalman gain and smooths jitter but raises the lag. The velocity used by the position controller is an EKF state, so noise in our position reaches the controller through the position-to-velocity gain of the filter (inference).
- PX4 docs on noisy external position: Found no passage that gives advice beyond (1) the noise parameters `EKF2_EVP_NOISE`, `EKF2_EV_NOISE_MD` and (2) tuning `EKF2_EV_DELAY` from innovations. For LPE (old) the docs say reduce `LPE_VIS_XY` to track tightly. Did not find a statement about MPC gains for noisy external position. The levers that exist: raise `EKF2_EVP_NOISE`, send an honest covariance, reduce `MPC_XY_P` and `MPC_XY_VEL_P_ACC`, set `MPC_VEL_LP`, and raise `MPC_VELD_LP` filtering (the choice).

---

## 8. ArduPilot (short)

Sources: https://ardupilot.org/copter/docs/common-ekf-sources.html , https://ardupilot.org/copter/docs/common-vio-tracking-camera.html , ArduPilot source (`AP_NavEKF_Source.cpp`, `AP_VisualOdom.cpp`, `AP_VisualOdom_MAV.cpp`, `GCS_Common.cpp`, `AP_NavEKF3_PosVelFusion.cpp`, `ArduCopter/Parameters.cpp`) on `master`.

- EKF3 sources (set 1): `EK3_SRC1_POSXY = 6` (ExternalNav; values: 0 None, 3 GPS, 4 Beacon, 6 ExternalNav). `EK3_SRC1_VELXY`: default is 3 (GPS). With GPS off set it to 0 (None) if we send no velocity (6 = ExternalNav only if the sender gives velocity). `EK3_SRC1_POSZ = 1` (Baro, default; docs recommend baro with a camera). `EK3_SRC1_VELZ`: default 3 (GPS); set 0 (None). `EK3_SRC1_YAW = 1` (Compass; 6 = ExternalNav, 8 = GSF). `EK3_SRC_OPTIONS` bit 0 fuses all velocity sources, bit 1 aligns ExternalNav with optical flow, bit 3 uses one source set per core.
- `VISO_TYPE`: 0 None, 1 MAVLink, 2 IntelT265, 3 VOXL. Use 1 for a custom MAVLink sender (the T265 doc uses 2). Other parameters: `VISO_DELAY_MS` (default 10, range 0 to 250), `VISO_POS_M_NSE` (default 0.2 m, lower bound), `VISO_VEL_M_NSE` (0.1 m/s), `VISO_YAW_M_NSE` (0.2 rad), `VISO_QUAL_MIN` (default 0), `VISO_ORIENT`, `VISO_POS_X/Y/Z`, `VISO_SCALE`.
- GPS off: `GPS_TYPE = 0` (parameter now `GPS1_TYPE` in the docs).
- MAVLink input (`GCS_Common.cpp`): VISION_POSITION_ESTIMATE and ODOMETRY are both accepted. ODOMETRY is accepted ONLY with `frame_id = MAV_FRAME_LOCAL_FRD` and `child_frame_id = MAV_FRAME_BODY_FRD`; other frames are dropped. Position error sent to the EKF = sqrt(var_x + var_y + var_z) from `covariance[0] + [6] + [11]`, then limited to between `VISO_POS_M_NSE` and 100 m. NaN first element means unknown. So the z variance adds to the horizontal noise: set it small.
- Timestamp: `correct_offboard_timestamp_usec_to_ms` converts the sender's `usec` with a lag-correction filter and the UART receive time. The EKF then uses `VISO_DELAY_MS` as a fixed delay.
- External-nav position is fused in the north/east frame directly (`velPosObs` = `extNav.dataDelayed.pos`). There is no automatic rotation. The T265 docs use RC option 80 "Viso Align" to align camera yaw to EKF yaw before flight, and warn about toilet-bowling if skipped.
- EKF origin without GNSS: `SET_GPS_GLOBAL_ORIGIN` is handled (`handle_set_gps_global_origin`; the origin is set from lat, lon, altitude after range checks). Mission Planner: "Set EKF Origin Here" (T265 doc).
- Rate in the T265 script: 30 Hz for VISION_POSITION_ESTIMATE, 1 Hz for the confidence message. Docs: send at a fixed rate "so as to not flood the Autopilot".
- Failsafe: `FS_EKF_ACTION` (0 report only, 1 Land if the mode needs position, 2 AltHold if the mode needs position, 3 Land from all modes), `FS_EKF_THRESH` (0 disabled, 0.6 strict, 0.8 default, 1.0 relaxed). `EK3_NOAID_M_NSE` default 10 m exists. How long ArduPilot keeps a position after the stream stops: not checked (unknown).
- Reboot note in the T265 doc: "Reboot the autopilot if external navigation data is lost."
- Stream rates in ArduPilot: `SRn_*` stream parameters and `MAV_CMD_SET_MESSAGE_INTERVAL` (handled in `GCS_Common.cpp`). Did not map messages to stream groups (unknown).

---

## 9. Attitude and height from the flight controller, stream rates, and the 230400 baud link

### PX4 streams (source: `mavlink_main.cpp`, `configure_streams_to_default`, release/1.16)

https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/mavlink/mavlink_main.cpp

Default rates in ONBOARD mode (companion link): `TIMESYNC` 10 Hz, `HIGHRES_IMU` 50 Hz, `LOCAL_POSITION_NED` 30 Hz, `ATTITUDE` 100 Hz, `ALTITUDE` 10 Hz, `ATTITUDE_QUATERNION` 50 Hz, `ODOMETRY` 30 Hz (PX4's own estimate going out), `SCALED_PRESSURE` 1 Hz, `DISTANCE_SENSOR` 10 Hz, `GLOBAL_POSITION_INT` 50 Hz, `RC_CHANNELS` 20 Hz, `ESTIMATOR_STATUS` 1 Hz, plus about 30 more streams. NORMAL mode: `ATTITUDE` 15, `ATTITUDE_QUATERNION` 10, `ALTITUDE` 1, `LOCAL_POSITION_NED` 1, `SCALED_PRESSURE` 1. There is an EXTVISION mode (only `HIGHRES_IMU` unlimited) and EXTVISIONMIN mode (`LOCAL_POSITION_NED` 30, `ATTITUDE` 20, `ALTITUDE` 10, `TIMESYNC` 10, `ODOMETRY` 30 and others).

Fields (MAVLink XML):
- ATTITUDE (30): `time_boot_ms` (ms), roll, pitch, yaw, rollspeed, pitchspeed, yawspeed.
- ATTITUDE_QUATERNION (31): `time_boot_ms` (ms), `q1..q4`, body rates, `repr_offset_q` (extension).
- HIGHRES_IMU (105): `time_usec`, accel xyz, gyro xyz, mag xyz, `abs_pressure`, `diff_pressure`, `pressure_alt`, `temperature`, `fields_updated`, `id`. The only attitude-related message with microsecond time and raw gyro; also carries pressure altitude.
- ALTITUDE (141): `time_usec`, `altitude_monotonic`, `altitude_amsl`, `altitude_local`, `altitude_relative`, `altitude_terrain`, `bottom_clearance`.
- LOCAL_POSITION_NED (32): `time_boot_ms`, x y z, vx vy vz.
- SCALED_PRESSURE (29): `time_boot_ms`, `press_abs`, `press_diff`, `temperature`.
- Note: ATTITUDE and ATTITUDE_QUATERNION stamp in whole milliseconds (`time_boot_ms = att.timestamp/1000` in PX4 `ATTITUDE.hpp`). For rolling-shutter work at row time, 1 ms steps at 100 Hz are fine, but you must map FC boot time to the companion clock with the TIMESYNC offset (inference).
- Barometric height: `LOCAL_POSITION_NED.z` is the EKF height (fused). `ALTITUDE.altitude_local` is up-positive. `HIGHRES_IMU.pressure_alt` is the raw baro altitude. Which one the team should use for the height scale: the EKF height (LOCAL_POSITION_NED / ALTITUDE) follows the same baro the FC uses. Not tested.

### How to set rates (source: `mavlink_receiver.cpp`, MAVLink XML)

- `MAV_CMD_SET_MESSAGE_INTERVAL` (command 511): param1 = message id, param2 = interval in microseconds (-1 disable, 0 default rate), param3 = index. Sent with COMMAND_LONG. PX4 handles it in `mavlink_receiver.cpp` (`set_message_interval`). Example: 100 Hz ATTITUDE is `param1 = 30`, `param2 = 10000`.
- On the FC side the profile is set by `MAV_n_MODE` (Onboard etc.) and the max data rate by `MAV_n_RATE`. The MAVLink instance option `-r` is "Maximum sending data rate in B/s (if 0, use baudrate / 20)".

### What a 230400 baud link carries

- 8N1 serial: 10 bits per byte, so 23,040 bytes per second at most in each direction. UART is full duplex, so the two directions do not share the budget (inference from how UART works).
- PX4's default send budget on its transmit side: baud / 20 = 11,520 B/s at 230400 (`_datarate = _baudrate / 20`, `mavlink_main.cpp`). PX4 scales stream rates down when the sum exceeds this (`bandwidth_mult`). The default onboard stream set would exceed it, so choose streams explicitly.
- Frame size = payload + 12 bytes (MAVLink v2: 10 header + 2 checksum; +13 if signed). From XML field sums:

| Message | Payload | v2 frame | At rate | Bytes per second |
|---|---|---|---|---|
| ATTITUDE | 28 | 40 | 100 Hz | 4,000 |
| ATTITUDE_QUATERNION | 48 (32 without extension) | 60 | 100 Hz | 6,000 |
| HIGHRES_IMU | 63 | 75 | 50 Hz | 3,750 |
| ALTITUDE | 32 | 44 | 10 Hz | 440 |
| LOCAL_POSITION_NED | 28 | 40 | 30 Hz | 1,200 |
| TIMESYNC | 18 | 30 | 10 Hz each way | 300 |
| VISION_POSITION_ESTIMATE | 117 | 129 | 30 Hz | 3,870 (companion to FC) |
| ODOMETRY | 233 | 245 | 30 Hz | 7,350 (companion to FC) |
| OPTICAL_FLOW_RAD | 44 | 56 | 30 Hz | 1,680 |

Example plan (the choice): FC to companion: ATTITUDE 100 Hz + ALTITUDE 10 Hz + TIMESYNC 10 Hz + HEARTBEAT = about 4.8 kB/s, which is about 42% of the PX4 send budget (11.5 kB/s) and 21% of the raw line rate. Companion to FC: VISION_POSITION_ESTIMATE 30 Hz + TIMESYNC replies + heartbeat = about 4.3 kB/s, 19% of the raw line rate. All other streams must be switched off (`MAV_CMD_SET_MESSAGE_INTERVAL` with -1), or `MAV_n_MODE` set to a minimal profile. v2 frames with trailing zero bytes get shorter, so these are upper bounds.
- If we need 200 Hz attitude for rolling shutter, ATTITUDE needs 8 kB/s, which is 70% of the PX4 budget. Use `HIGHRES_IMU`'s gyro at 50 Hz plus ATTITUDE at 100 Hz, and interpolate (inference).

---

## Unknowns (could not find or not tested)

1. The Orqa M2-AI firmware: PX4 or something else, which version, whether it has EKF2 external vision enabled, which serial port and baud rate, and whether the MAVLink instance is in Onboard mode. Not found in this research.
2. An official typical yaw error of the PX4 magnetometer. Only anecdotal forum values.
3. The exact time from "vision stops" to "position invalid" on a real build. The 4.4 s figure is derived from reading the code and not run in SITL.
4. Whether Position mode on v1.14 and v1.15 needs a global position (docs prose says yes; v1.16 requirement list says local relaxed only). Hold mode needs global position in v1.16 code, so use `SET_GPS_GLOBAL_ORIGIN`.
5. How much position error grows during 0.5 s, 2 s, 10 s of dead reckoning with a real IMU. No number.
6. Whether PX4 optical-flow fusion works at 100 m height with no range finder. Code suggests possible start through terrain estimate; not tested; noise default too large to matter.
7. Which `ATTITUDE`-family rates the FC can actually deliver over the Orqa UART with the full default stream set; whether TIMESYNC round-trip stays under 10 ms on that link.
8. ArduPilot: time until position is lost after the stream stops, SRn stream-group membership of each message, and whether `EK3_POS_I_GATE` is the gate used for ExternalNav position.
9. Effect of a 50 ms timestamp error: only derived, no measurement.
10. PX4 behaviour of `EKF2_EV_CTRL` with the `LOCAL_FRD` frame and the Orqa build; not tested.

---

## Recommendations for the build and the event day

1. Send VISION_POSITION_ESTIMATE at 30 Hz: `usec` = capture time (mid-exposure of the reference row) on the companion monotonic clock in microseconds; x, y = north, east in metres relative to the anchor; z = any finite number (for example baro-based or 0); roll/pitch/yaw = attitude from the FC (or NaN); `covariance[0]` = `covariance[6]` = our variance in m^2, `covariance[11]` = a finite value (for example 1.0), the other entries 0 or NaN-free; `reset_counter` only changes on a real frame jump.
2. Answer TIMESYNC requests from the FC (tc1 = our clock in ns). Check `timesync_status` (PX4 topic) and watch for convergence (about 50 s at 10 Hz). Until converged, PX4 ignores our timestamp.
3. Start the event with the PX4 set: `EKF2_EV_CTRL=1`, `EKF2_GPS_CTRL=0`, `EKF2_HGT_REF=0`, `EKF2_BARO_CTRL=1`, `EKF2_RNG_CTRL=0`, `EKF2_EV_NOISE_MD=0`, `EKF2_EVP_NOISE=0.1..0.3`, `EKF2_EV_DELAY` = measured, `EKF2_EV_POS_X/Y/Z` = camera offset. On v1.14 and v1.15 `EKF2_EV_CTRL` defaults to 15, so it must be set. Reboot after changes. On v1.13 use `EKF2_AID_MASK=8`, `EKF2_HGT_MODE=0`.
4. Set `EKF2_EV_DELAY` from logs: set `SDLOG_PROFILE` bit 7, compare vision and onboard position in a log, and pick the value that gives the smallest innovations. Our 60 to 100 ms is the starting guess only if we stamp "now"; use about 0 if we stamp true capture time with converged TIMESYNC.
5. Keep our estimator in the FC's own heading frame, send NED, keep the EV yaw bit off. Test a deliberate yaw offset in SITL.
6. Make our program drop a sample (do not send) when tracking quality is low; do not send a very large variance as a substitute. PX4 restarts fusion with a hard position reset after a gap of more than 0.4 s, so a short gap is safe and a long gap is a position jump to our anchor.
7. Plan for stop events: after about 0.4 s PX4 stops fusing; after about 4 to 5 s with defaults the position is invalid and Position mode falls to Altitude mode (`COM_POSCTL_NAVL=0`). Consider `EKF2_NOAID_TOUT` larger (max 10 s) only if the judges' mode allows; decide with the Orqa firmware owner.
8. Mode: ask the organizers which flight mode the drone holds in (Position, Hold, Offboard). Hold needs a global origin: send `SET_GPS_GLOBAL_ORIGIN` once if GNSS is off.
9. Link budget: set only needed streams via `MAV_CMD_SET_MESSAGE_INTERVAL` (ATTITUDE 100 Hz, ALTITUDE 10 Hz, TIMESYNC 10 Hz) and disable the rest. Keep TIMESYNC round-trip under 10 ms.
10. Test in PX4 SITL with a real PX4 EKF2: closed loop, a vision stream that stops for 0.5, 2, 10 s, a yaw offset, a 50 ms timestamp shift, and covariance NaN cases. This removes the "simulator and estimator share the same geometry" blind spot for the frame conventions.
11. For the optical-flow fallback, send velocity through ODOMETRY (EV_CTRL bit 2, add 4), not OPTICAL_FLOW_RAD.

---

## Source URLs

PX4 (GitHub, read with `gh api`):
- https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/ros/external_position_estimation.md
- https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/advanced_config/tuning_the_ecl_ekf.md
- https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/computer_vision/visual_inertial_odometry.md
- https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/sensor/optical_flow.md
- https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/config/safety.md
- https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/flight_modes_mc/position.md
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/params_external_vision.yaml
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/module.yaml
- https://github.com/PX4/PX4-Autopilot/blob/release/1.15/src/modules/ekf2/module.yaml
- https://github.com/PX4/PX4-Autopilot/blob/release/1.14/src/modules/ekf2/ekf2_params.c
- https://github.com/PX4/PX4-Autopilot/blob/release/1.13/src/modules/ekf2/ekf2_params.c
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/params_optical_flow.yaml
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/EKF2.cpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/EKF/common.h
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/EKF/estimator_interface.cpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/EKF/ekf_helper.cpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/EKF/aid_sources/external_vision/ev_control.cpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/EKF/aid_sources/external_vision/ev_pos_control.cpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/EKF/aid_sources/fake_pos_control.cpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/ekf2/EKF/aid_sources/optical_flow/optical_flow_control.cpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/mavlink/mavlink_receiver.cpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/mavlink/mavlink_main.cpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/mavlink/mavlink_timesync.cpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/lib/timesync/Timesync.cpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/lib/timesync/Timesync.hpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/commander/commander_params.c
- https://github.com/PX4/PX4-Autopilot/blob/release/1.15/src/modules/commander/commander_params.c
- https://github.com/PX4/PX4-Autopilot/blob/release/1.14/src/modules/commander/commander_params.c
- https://github.com/PX4/PX4-Autopilot/blob/release/1.13/src/modules/commander/commander_params.c
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/commander/failsafe/failsafe.cpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/commander/HealthAndArmingChecks/checks/estimatorCheck.cpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/commander/ModeUtil/mode_requirements.cpp
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/mc_pos_control/PositionControl/PositionControl.cpp
- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_pos_control/multicopter_position_control_gain_params.yaml
- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_pos_control/multicopter_position_control_limits_params.yaml
- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_pos_control/multicopter_position_control_params.yaml
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/sensors/vehicle_optical_flow/VehicleOpticalFlow.cpp
- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/sensors/sensor_params_flow.yaml (v1.16: `sensor_params_flow.c`)
- https://github.com/PX4/PX4-Autopilot/blob/release/1.16/src/modules/mavlink/streams/ATTITUDE.hpp

MAVLink:
- https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml (VISION_POSITION_ESTIMATE, ODOMETRY, TIMESYNC, OPTICAL_FLOW_RAD, MAV_FRAME, MAV_ESTIMATOR_TYPE, MAV_CMD_SET_MESSAGE_INTERVAL, ATTITUDE and others)

ArduPilot:
- https://ardupilot.org/copter/docs/common-ekf-sources.html
- https://ardupilot.org/copter/docs/common-vio-tracking-camera.html
- https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF/AP_NavEKF_Source.cpp
- https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_VisualOdom/AP_VisualOdom.cpp
- https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp
- https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS_Common.cpp
- https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp
- https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Parameters.cpp

Other (anecdotal, from a web search summary, low confidence):
- https://discuss.px4.io/t/high-mag-accuracy-possible/48146
- https://docs.px4.io/main/en/config/compass
