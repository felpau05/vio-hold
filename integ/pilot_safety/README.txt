PILOT SAFETY CASE: SOURCES AND EVIDENCE (phase 1 written 2026-11-21 00:45-01:10; phase 2 01:30-02:40 local)

This file backs each line of PILOT_SAFETY.txt (the one page for the event pilot, Ted). It also holds the parameter
list for the event crew (section 5), the bench demonstration (section 6), the PX4 notes (section 3) and the simulator
plan (section 7). Phase 2: the arming test and the simulator results are in section 8. Where section 8 changes a
label of sections 0-6, section 8 wins.

Nobody has run anything on the real aircraft. Every statement carries one of these labels:
  [simulator test]          ArduCopter 4.6 SITL or PX4 SITL with the team's program (file and line given)
  [read in the source]      file and line given. ArduPilot paths are under apsim/ardupilot (Orqa branch, commit
                            3b40393, "ArduCopter V4.6.0", ArduCopter/version.h:9). PX4 paths are under
                            px4sim/PX4-Autopilot (v1.18.0-beta1-1575-gc4b8f107ae).
  [unit test]               tests_apb/ or tests/
  [not tested on the aircraft]
Line numbers in apb_main.py and fc_link.py are those of 01:00. Another agent edited both files at that time
(setting gnss_mode), so a line can move. The function names stay.
"Our position" = the fixes (VISION_POSITION_ESTIMATE) that apb_main.py sends. With the event parameters
(GPS1_TYPE 0, EK3_SRC1_POSXY 6) they are the FC's only horizontal position.


=====================================================================================================================
0. TAKEOVER PATHS THAT CAN FAIL OR HAVE NO EVIDENCE (read this first)
=====================================================================================================================
F1. The FC does not notice a slow, believable wrong position. Loiter then flies the aircraft away, with no warning.
    - A fix passes the EKF3 gate when its innovation is under 5 sigma (EK3_POS_I_GATE default 500,
      libraries/AP_NavEKF3/AP_NavEKF3.cpp:34; test AP_NavEKF3_PosVelFusion.cpp:813-818). A slow ramp passes every time,
      the EKF follows it, and Loiter holds the wrong point. [read in the source]
    - The EKF failsafe needs a variance over FS_EKF_THRESH or a lost position (ArduCopter/ekf_check.cpp:52-54,
      142-160). A consistent wrong position gives neither. [read in the source]
    - Seen in the simulator: apsim/results.txt:3 (ap_apb_100m_calm_s1_attfc, 30.1 m at 16.5 s) and :8
      (ap_apb_100m_w20g40_s1_attfc, 30.0 m at 5.3 s). The aircraft stayed in Loiter (no "left Loiter" event). Cause: the old
      attitude option att_source "fc" (now "gyro", CONTRIBUTION.md:356). [simulator test]
    - Only the pilot can stop it: a switch to AltHold, Stabilize or Acro. Case c of apsim/safety_matrix.py measures
      it at 10 m and 30 m (phase 2).
F2. The stock "Return to Home" switch position flies on our position.
    - RTL needs a position (ArduCopter/mode.h:1421). With our fixes, the FC accepts RTL and flies to HOME. The program
      sets HOME to settings origin_lat_lon_alt with MAV_CMD_DO_SET_HOME at every start (fc_link.py:238-247, called
      from apb_main.py:649-650). That point is the EKF origin, the (0, 0) of our frame. [read in the source]
    - Where (0, 0) is on the field: where the program anchored its first fix (apb_main.py:169-176, Pipe._reanchor;
      before the FC has a position it anchors to Hold's own start position; that value was not checked). If our frame drifted or the fixes are
      wrong, RTL flies to a wrong place. RTL first climbs to RTL_ALT (default 15 m, ArduCopter/Parameters.cpp:105).
      [read in the source; where (0, 0) lands is not tested on the aircraft]
    - With no fix, the FC refuses RTL in the air and stays in its mode (mode.cpp:322-327). [read in the source]
    - So "Return to Home" is never a takeover with GNSS off. Ask the crew to remove it from the switch (section 5).
F3. A radio failsafe starts RTL in every flight mode, also in Acro.
    - FS_THR_ENABLE default 1 = RTL (ArduCopter/Parameters.cpp:229, defines.h:118; SITL copter.parm:2 also 1).
      ArduCopter/events.cpp:13-73 picks RTL with no exception for manual modes. set_mode_RTL_or_land_with_pause
      (events.cpp:380-392) gives RTL when our position is valid, else LAND. [read in the source]
    - So after a radio loss the aircraft flies to HOME on our position (F2). Ask FS_THR_ENABLE 3 (Land). No test.
F4. A refused position-hold request stays refused.
    - Armed, a mode that needs a position is refused when the FC has none: text "Mode change to LOITER failed:
      requires position" and a sad tone (ArduCopter/mode.cpp:191-199, 322-327). The FC stays in its old mode.
    - The FC reads the switch only when its position changes (libraries/RC_Channel/RC_Channel.cpp:590-620). Fixes
      that start later do not start Loiter. The pilot must move the switch out and back. [read in the source]
    - Whether the refusal text reaches Ted's goggles or OSD: [not tested on the aircraft].
F5. On the ground, disarmed, the FC accepts every mode (mode.cpp:290: "allow switching to any mode if disarmed").
    A ground switch test shows the mode names but not the refusal. The arming check catches it at arm time:
    "Need Position Estimate" (ArduCopter/AP_Arming.cpp:435-460). This check runs even with ARMING_CHECK 0
    (mandatory_checks, AP_Arming.cpp:643-646). [read in the source]
F6. Nothing in the source caps how far wrong fixes can bend the EKF attitude that Stabilize, AltHold and Acro use.
    - Copter forces the EKF as the attitude source (libraries/AP_AHRS/AP_AHRS.cpp:211-214, FLAG_ALWAYS_USE_EKF).
      With that flag the EKF3 stays in use while it has no numeric fault (AP_AHRS.cpp:1970-1983). The DCM fallback
      exists only for fixed wing and ground vehicles (AP_AHRS.cpp:2001-2004). So there is NO fallback to DCM for a
      copter. [read in the source]
    - A lane change (ekf_check.cpp:73-77) does not help: every EKF lane gets the same fixes. [read in the source]
    - What still protects the attitude: the gate rejects large jumps (F1); the compass gives the yaw (EK3_SRC1_YAW 1,
      settings.py:60-61); the rate loop uses the gyro (libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:212).
    - Could not determine: how many degrees of tilt error a 1 m/s ramp, a 20 m jump or an EKF position reset gives.
      Cases a, c, d and e of apsim/safety_matrix.py measure it (EKF ATTITUDE against the true attitude).
F7. A 20 m jump: the EKF refuses it first, then may accept it.
    - The jump fails the gate (PosVelFusion.cpp:813-816). After 7 s with no accepted fix (posRetryTimeNoVel_ms 7000,
      AP_NavEKF3.h:474; AP_NavEKF3_Control.cpp:373-386, 396-401) the EKF sets posTimeout and resets its position to
      the next fix, the jumped one (PosVelFusion.cpp:834-855). [read in the source]
    - In the same seconds the FC can lose "position OK" and start the EKF failsafe (section 2.3). Which comes first
      is not clear from the source. Case d measures it. [not tested]
F8. The program can stop for good.
    - A Python error restarts the program after 1 s (apb_main.py:1296-1315, mode_run loop) and it re-anchors to the
      FC position (apb_main.py:169-176). A stopped camera ends run_live after 2 s and restarts it (apb_main.py:633,
      676). [unit test: tests_apb/test_robust.py D2, lines 153-171]
    - A hung program (no error) never restarts. No watchdog exists. [read in the source]
    - A kill of the process (or an ssh start without setsid nohup) stops it for good
      (integ/rehearsal_2026-11-20/README.txt:178-185, hang-up test). [simulator test]
    - The APB is one board with the flight controller (STM32H743) and the computer (i.MX8M Plus)
      (research/hardware.md:18, 73). A loss of board power stops the FC too. No takeover then exists. Whether a
      reboot of the i.MX resets the FC is NOT KNOWN (bench step B7). [not tested on the aircraft]
F9. The FC refuses to arm, and the easy fix removes every arming check. NOW A SIMULATOR TEST (section 8.1).
    - VISO_TYPE 1 (our parameter set) with the default ARMING_CHECK (bit 18 VisualOdometry) refuses to arm unless a
      vision message came in the last 300 ms: "VisOdom: not healthy" (libraries/AP_Arming/AP_Arming.cpp:
      1939-1953, libraries/AP_VisualOdom/AP_VisualOdom.cpp:277-298, AP_VisualOdom_Backend.cpp:36,
      AP_VisualOdom.h:31). [read in the source]
    - The program sends no fix below min_height_m (3 m outdoors; apb_main.py:186-188). On the ground it sends none.
    - [simulator test] ARMING_CHECK 1: "Arm: VisOdom: not healthy", arming refused in ALT_HOLD and in STABILIZE, with
      the default level and with gnss_setup. With only the vision check off (ARMING_CHECK 1835006) it armed, climbed
      to 10 m and held. Section 8.1.
    - Risk: someone sets ARMING_CHECK 0 to fly, which also removes the battery, compass, INS and RC checks. Ask the
      crew to clear the vision check only (section 5).
F10. Takeover paths with NO evidence at all (no simulator run, no aircraft). PHASE 2: section 8.2 now covers in SITL
    the release to AltHold and Stabilize, a program stop, a ramp and a jump at 10 m and 30 m. Still no evidence:
    Acro, RTL, the radio failsafe, and everything on the aircraft. The phase 1 list:
    - the release: Loiter to AltHold, Stabilize or Acro, with the flight measured afterwards (the harness switches
      ALT_HOLD at 30 m and then ends the run, apsim/sitl_ap.py:415-419 at 01:05; another agent edits that file);
    - a program stop during a hold at 10-30 m (one 100 m video-outage run exists, apsim/drag_results.txt:2);
    - a wrong ramp and a jump in closed loop; a radio failsafe with our position; the RTL switch position;
    - arming with ARMING_CHECK on; every path on the real aircraft.


=====================================================================================================================
1. WHAT THE PROGRAM CAN DO TO THE AIRCRAFT
=====================================================================================================================
1.1 Messages that `apb_main.py run` sends (all through fc_link.FC._send, fc_link.py:171-173) [read in the source]
  Message                                   Where                                   When                   Changes flight?
  HEARTBEAT (ONBOARD_CONTROLLER)            fc_link.py:101-109                      1 Hz, always           no
  COMMAND_LONG SET_MESSAGE_INTERVAL         fc_link.py:193-208 (open_fc,            at each start          no (telemetry
                                            apb_main.py:794-803)                                           rates, this link)
  TIMESYNC (answer)                         fc_link.py:156-158                      when the FC asks        no
  SET_GPS_GLOBAL_ORIGIN                     fc_link.py:238-240 (apb_main.py:650)    at each start          yes: gives the EKF
                                                                                                           an origin, which
                                                                                                           also arms the EKF
                                                                                                           check (ekf_check
                                                                                                           .cpp:35-39)
  COMMAND_INT MAV_CMD_DO_SET_HOME           fc_link.py:241-247 (ArduPilot only)     at each start, also    YES: moves the RTL
                                                                                    after a restart in     target (F2). Same
                                                                                    flight                 point every time.
  VISION_POSITION_ESTIMATE                  fc_link.py:249-273 (apb_main.py:701)    each good fix          YES: the FC's only
                                                                                                           horizontal position
  VISION_SPEED_ESTIMATE                     fc_link.py:275-281 (apb_main.py:708)    only send_speed true   YES (default off)
  COMMAND_LONG MAV_CMD_DO_SET_MODE LOITER   fc_link.py:283-286 (apb_main.py:605)    only wind_assist true  YES (default off)
                                                                                    AND the FC in GUIDED
  SET_POSITION_TARGET_LOCAL_NED             fc_link.py:295-303 (apb_main.py:629)    only wind_assist true  YES (default off)
                                                                                    AND the FC in GUIDED
  COMMAND_LONG MAV_CMD_SET_EKF_SOURCE_SET   fc_link.py:288-293 (apb_main.py:333-380 only gnss_mode not     YES: switches the
  (42007)                                   GnssLevel, called at apb_main.py:666)   "none" (default none,  EKF between GNSS
                                                                                    settings.py:152)       and our position
  `apb_main.py bench` sends the same set as run (it runs run_live), with fixes at a fixed height also on the ground,
  and stops the fixes when the FC arms (apb_main.py:691, 1277-1279).

1.2 Messages that `apb_main.py check --set-params` sends on the ground [read in the source]
  HEARTBEAT, SET_MESSAGE_INTERVAL, TIMESYNC requests (PX4 only, fc_link.py:210-214), PARAM_REQUEST_READ
  (fc_link.py:217-226) for each parameter, and PARAM_SET (fc_link.py:228-235) for each parameter that differs
  (fc_link.py:376-399, called at apb_main.py:885). It writes settings params_ardupilot (settings.py:60-61):
  VISO_TYPE 1, EK3_SRC1_POSXY 6, EK3_SRC1_VELXY 0, EK3_SRC1_POSZ 1, EK3_SRC1_VELZ 0, EK3_SRC1_YAW 1, GPS1_TYPE 0,
  VISO_DELAY_MS 140. A profile can add keys (tools/profiles/drag_fusion.json adds FS_DR_ENABLE 0; the gnss_*
  profiles add GPS1_TYPE 1 and EK3_SRC2_*). PX4: params_px4 (settings.py:50-52), with COM_ARM_WO_GPS 1.
  All of these change what the FC does: they choose the EKF sources. GPS1_TYPE 0 removes GNSS for the whole flight.
  Phase 1 noted a conflict: the team writes FC parameters itself. EVENT_DAY.txt now asks the crew first (step 1
  item 2a), saves the aircraft's own values with tools/fc_params.py save (item 2b) and writes them back after the
  last test (step 8, tools/fc_params.py restore). fc_params.py saves only the names our settings write; a vision
  arming change made by the crew is the crew's to undo.

1.3 The four claims, checked [read in the source: grep of apb_main.py, fc_link.py, msp_link.py at 01:00]
  - "never arms": TRUE. No MAV_CMD_COMPONENT_ARM_DISARM in the code.
  - "never changes the flight mode": TRUE ONLY WITH wind_assist false (the default, settings.py). With wind_assist
    true, WindAssist sends MAV_CMD_DO_SET_MODE LOITER while the FC is in GUIDED (apb_main.py:571-631, line 605).
  - "never sends a position, velocity or attitude target": TRUE ONLY WITH wind_assist false. With it true, the
    program sends SET_POSITION_TARGET_LOCAL_NED (position, zero velocity, heading) in GUIDED (apb_main.py:629).
    No attitude target exists anywhere. NOTE: VISION_POSITION_ESTIMATE is not a target, but in Loiter it moves the
    aircraft just as much (F1).
  - "never overrides the pilot's sticks": TRUE. No RC_CHANNELS_OVERRIDE or MANUAL_CONTROL in the code.
  - gnss_mode (added by another agent at 00:5x): default "none" sends nothing new (apb_main.py:356-357). Any other
    value sends MAV_CMD_SET_EKF_SOURCE_SET. In "fallback" the program itself switches the EKF to GNSS during a hold
    after gnss_fallback_s (3 s) with no fix (apb_main.py:369-372). That changes what the aircraft does.
  - Betaflight: msp_link.py is not imported by apb_main.py; if used it sends only MSP rangefinder and optical-flow
    sensor messages, never a mode or a setting (msp_link.py:1-20). Off.


=====================================================================================================================
2. ARDUCOPTER BEHAVIOUR (source apsim/ardupilot)
=====================================================================================================================
2.1 Modes and the position [read in the source, ArduCopter/mode.h; "requires_GPS" means any horizontal position,
    ours included]
  No position needed: STABILIZE (mode.h:1624), ACRO (:422), ALT_HOLD (:477), LAND (:1239; it uses a position if it
  has one when it starts, mode_land.cpp:7), SPORT (:1597), FLOWHOLD (:974), GUIDED_NOGPS (:1215).
  Position needed: LOITER (:1285), POSHOLD (:1334), RTL (:1421), SMART_RTL (:1537), GUIDED (:1062), BRAKE (:840),
  CIRCLE (:870), DRIFT (:900), THROW (:1749), ZIGZAG (:1918), FOLLOW (:1869), AUTO (:511, by mission item).
  Mode numbers (mode.h:77-95): 0 STABILIZE, 1 ACRO, 2 ALT_HOLD, 5 LOITER, 6 RTL, 9 LAND, 11 DRIFT, 16 POSHOLD,
  21 SMART_RTL. (EVENT_DAY.txt:254 says "RTL gave mode 11" in SITL. RTL is 6; 11 is DRIFT. The program prints "RTL"
  for 6 (apb_main.py:215-216). A position that shows "mode 11" or "mode 21" also needs a position. Do not use it.)

2.2 Position hold selected with no position [read in the source]
  Armed: refused, "Mode change to <mode> failed: requires position", tone, old mode kept (mode.cpp:191-199,
  322-327). position_ok() is false while the EKF failsafe is on, else it needs horiz_pos_abs or horiz_pos_rel without
  constant-position mode (ArduCopter/system.cpp:227-285). Disarmed: accepted (mode.cpp:290). The switch must move
  again (F4). From a manual-throttle mode (Acro, Stabilize) to AltHold or Loiter the FC also refuses when it has no
  height estimate (mode.cpp:331-337). Leaving Loiter for AltHold, Stabilize or Acro has no refusal check in flight
  (mode.cpp:295-357: the checks are runup, landed throttle, position, height, fence).

2.3 The fixes stop during Loiter: time line [read in the source] [simulator test where marked]
  t = 0     Last fix. The EKF predicts from the IMU. No velocity source (EK3_SRC1_VELXY 0), so the error grows fast.
  t = 7 s   No accepted fix for posRetryTimeNoVel_ms 7000 (AP_NavEKF3.h:474) -> posTimeout (AP_NavEKF3_Control.cpp:
            373-386, 396-401) -> horiz_pos_abs and horiz_pos_rel false (AP_NavEKF3_Control.cpp:767, 779-780).
  t = 7-8 s ekf_check (10 Hz) sees no position: 10 failed checks = 1 s (ekf_check.cpp:11, 52-90). At check 9 it asks
            for a lane change (ekf_check.cpp:73-77). Text "EKF variance" (ekf_check.cpp:85-88).
  t = 8 s   EKF failsafe (ekf_check.cpp:167-206). Loiter needs a position, so FS_EKF_ACTION acts:
            1 = LAND (default, config.h:103, Parameters.cpp:388), 2 = ALT_HOLD, 3 = LAND also in Stabilize/Acro
            (defines.h:137-139). With a radio failsafe on, AltHold becomes LAND (ekf_check.cpp:193-195).
            Text "EKF Failsafe: changed to LAND Mode". In Stabilize, Acro or AltHold the default does nothing
            (ekf_check.cpp:185-187).
  t = 15 s  No aiding for tiltDriftTimeMax_ms 15000 (AP_NavEKF3.h:472) -> constant-position mode
            (AP_NavEKF3_Control.cpp:361-395).
  Variance path (also possible): two of compass, velocity and position test ratios over FS_EKF_THRESH (default 0.8,
  config.h:106, Parameters.cpp:395), or velocity over twice it, for 1 s (ekf_check.cpp:114-163). The ratios are
  low-pass filtered at FS_EKF_FILT (default 5 Hz, config.h:114, Parameters.cpp:1233, system.cpp:201-203).
  FS_EKF_THRESH 0 disables the whole check (ekf_check.cpp:42-48).
  [simulator test] 100 m, hold-out site, w20g40, forced 20 s video outage: "EKF Failsafe: changed to LAND Mode" 8.1 s
  into the gap, 30 m from the hold point 14.7 s into the gap (apsim/drag_results.txt:2, EVIDENCE.md:142). One run.
  What the pilot can do after the failsafe:
  - In LAND with no position (mode_land.cpp:93-132): a 4 s pause (LAND_WITH_DELAY_MS, config.h:327-328), then a
    descent. It drifts with the wind. LAND_REPOSITION default 1 (config.h:325) lets the pilot lean it
    (mode_land.cpp:105-110). The pilot can select AltHold, Stabilize or Acro at once. Loiter, PosHold and RTL are
    refused while the EKF failsafe is on (system.cpp:229-232).
  - In AltHold (FS_EKF_ACTION 2): it keeps its height and drifts with the wind. The pilot flies it.
  - The FC never returns to Loiter by itself. The failsafe clears when the checks pass again (ekf_check.cpp:92-104).

2.4 Wrong fixes [read in the source]
  - Slow ramp: F1. No variance flag, no failsafe, no lane change. Only the pilot stops it.
  - 20 m jump: F7. While the EKF refuses the fixes, the position test ratio is high but the velocity ratio is not
    (no velocity source), so the variance path needs the compass ratio too (ekf_check.cpp:142-160). The loss of
    position after 7 s is the likely trigger. Not tested.
  - Lane change: every lane gets the same fixes, so a lane change does not reject our data (ekf_check.cpp:73-77).

2.5 The attitude: F6. [read in the source] Could not determine the size of the effect. Case a (normal hold), c, d
  and e measure the EKF attitude error against the truth after the takeover.

2.6 Acro [read in the source]
  Acro needs no position (mode.h:422). It still uses the EKF attitude: the attitude controller compares its target
  with the AHRS attitude (AC_AttitudeControl.cpp:607), and ACRO_TRAINER (default 2 "limited", Parameters.cpp:439)
  levels the aircraft and limits the angle (mode_acro.cpp:132-175). With ACRO_TRAINER 0 a constant attitude offset
  does not change the stick response. The gyro is bias-corrected by the EKF (AC_AttitudeControl.cpp:212).
  A failsafe can still take Acro away: the radio failsafe (F3) and FS_EKF_ACTION 3.
  Throttle in Acro and Stabilize: mid stick = the learned hover throttle (mode.cpp:928-948).

2.7 RTL and the failsafes that use a position [read in the source]
  - RTL: F2. No position: refused in the air; LAND when a failsafe asked for RTL (events.cpp:380-392).
  - Radio failsafe FS_THR_ENABLE: F3. On the ground it disarms (events.cpp:47-51, 454-472).
  - Battery: BATT_FS_LOW_ACT and BATT_FS_CRT_ACT default 0 = warning only
    (libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp:122, 134). 2 (RTL) or 3/4 (SmartRTL) would fly on our
    position (events.cpp:99-121, 476-490).
  - GCS failsafe FS_GCS_ENABLE default 0 = off (Parameters.cpp:165). It watches only the heartbeat of the GCS system
    id (events.cpp:126-160); the program sends with the FC's own system id (fc_link.py:155), so it is not the GCS.
  - FS_OPTIONS default 16 = "GCS continue if pilot control" (Parameters.cpp:943).
  - Dead reckoning FS_DR_ENABLE default 2 = RTL, FS_DR_TIMEOUT 30 s (Parameters.cpp:1048, 1055; events.cpp:319-378).
    It acts only when the EKF flags dead reckoning, which needs drag or airspeed fusion (AP_NavEKF3_Control.cpp:766,
    797). Our parameter set has no drag fusion, so it never acts. With the drag_fusion profile it would start RTL on
    a dead-reckoned position. The profile sets FS_DR_ENABLE 0, so Loiter keeps flying on dead reckoning for 30 s
    (ekf_has_relative_position stays true, system.cpp:259-285). [simulator test] dr_gapX_on_s102: stayed in Loiter
    and flew 30 m away 19.1 s into a 20 s gap (apsim/drag_results.txt:3). The profile is marked NOT RECOMMENDED.
  - Fence: the actions were not read. A fence action RTL would fly on our position like RTL. A circle or polygon
    fence also needs a position to arm (ArduCopter/AP_Arming.cpp:448-463), and on the ground we send none. Ask the crew.

2.8 Our parameters and documents: what weakens control or a failsafe
  - settings.py params_ardupilot sets no failsafe parameter and does not touch the manual modes. [read in the source]
  - GPS1_TYPE 0 (settings.py:61) removes GNSS for the whole flight. Nobody can switch GNSS on in the air; the GPS
    type needs a restart (AP_GPS_Params.cpp:29, quoted at apb_main.py:312). [read in the source]
  - EK3_SRC1_POSXY 6: Loiter, PosHold, RTL and every failsafe RTL use our position. [read in the source]
  - tools/profiles/drag_fusion.json: FS_DR_ENABLE 0. Weakens a failsafe. Never apply it (EVENT_DAY.txt:364-366
    already says so).
  - wind_assist (settings, default false): sends mode changes and position targets in GUIDED. Keep it off.
  - recovery_demo profile: drops the video for 3 s on purpose. Never in a scored hold (EVENT_DAY.txt:341). With
    outage_test_s 8 or more it would trigger the EKF failsafe (section 2.3).
  - VISO_TYPE 1 with the default arming check may block arming (F9).
  - EVENT_DAY.txt step 1 writes FC parameters (section 1.2 conflict). Step 6B expects positions for Altitude and
    Position mode, but the stock switch is Acro / Position Hold / Return to Home (review/manuals_audit_2026-11-20.md
    Table 1 row 4). Step 6B line 254: "mode 11" (section 2.1).
  - Profiles outdoor_day, outdoor_night, ardupilot, justice_d, px4, gap_jump, goal_a, goal_b: no FC failsafe key.

2.9 The "Return to Home" switch position with GNSS off: F2 and F3. Risk: an uncommanded flight to a point that is
  only as good as our frame, with a climb to 15 m first, or (no fix) a refusal that leaves the aircraft where it was.


=====================================================================================================================
3. PX4 (shorter; source px4sim/PX4-Autopilot)
=====================================================================================================================
  - Position mode with no valid position falls back to Altitude mode, and Altitude mode with no height falls back
    to Stabilized. The pilot can always take over (src/modules/commander/failsafe/failsafe.cpp:747-751, 784-796).
    [read in the source]
  - Manual control loss: NAV_RCL_ACT default 2 = Return (src/modules/commander/commander_params.yaml:420-441).
    Return flies on our position, as ArduCopter RTL does. [read in the source]
  - COM_POS_FS_ACT (loss of position in autonomous modes) default 0 = Descend; COM_POS_FS_EPH default 5 m
    (commander_params.yaml:350-375). COM_POS_LOW_EPH default -1 = off (:716-746). COM_LOW_BAT_ACT default 0 =
    warning (:146-165). [read in the source]
  - EKF2_EVP_GATE default 5 sigma (src/modules/ekf2/params_external_vision.yaml:60); EKF2_NOAID_TOUT 5 s of dead
    reckoning (src/modules/ekf2/module.yaml:76). A slow ramp passes the gate, as in F1. [read in the source]
  - Our params_px4 sets COM_ARM_WO_GPS 1 and EKF2_GPS_CTRL 0 (settings.py:50-52).
  - [simulator test] PX4 fell back to Altitude mode and drifted 30 m in wind with no pilot action:
    px4sim/results.txt:19 (prairie, 100 m, left Position mode at 1 s) and :40 (hold-out field, 100 m, 17 s).
  - NOT CHECKED for PX4: the mode-refusal message, the time from the last fix to the Altitude fallback, any guard of
    the attitude against wrong vision data, the stick-movement takeover rule (COM_RC_OVERRIDE), the battery and
    datalink actions in detail. No PX4 run of the takeover cases is planned.


=====================================================================================================================
4. EVIDENCE ALREADY IN THE REPOSITORY
=====================================================================================================================
4.1 Every simulated fly-away or long drift found (the harness calls 30 m a FLY-AWAY, selects ALT_HOLD and ends)
  file:line                               case / height                       cause                                what would stop it
  apsim/results.txt:3                     ap_apb_100m_calm_s1_attfc, 100 m    old att_source "fc": wrong fixes,     pilot (FC silent).
                                                                              Loiter, no FC text                    option removed
  apsim/results.txt:8                     ap_apb_100m_w20g40_s1_attfc         same                                  same
  apsim/results.txt:90                    c_spd_100m_w20g40_s11, 100 m        send_speed with a wrong vertical     EKF failsafe LAND at
                                                                              speed (Speed class, apb_main.py:     25.2 s, then drift;
                                                                              536-568 notes)                       pilot. Option off
  apsim/drag_results.txt:2                dr_gapX_off_s102, 100 m, hold-out   20 s forced video outage             EKF failsafe LAND
                                                                                                                   8.1 s in; pilot
  apsim/drag_results.txt:3                dr_gapX_on_s102                     same, drag fusion on: dead           nothing in the FC
                                                                              reckoning, stayed in Loiter          (FS_DR_ENABLE 0);
                                                                                                                   pilot
  apsim/results_loaded_first_run.txt:4,   ap_apb_100m_w20g40_holdout_field    1.2 % fixes on low texture; LAND     pilot
  apsim/summary.txt:3                     _s102, 100 m                        at 9 s (mode 9)
  px4sim/results.txt:19                   apb_100m_w20g40_prairie (PX4)       3 % fixes; Altitude mode at 1 s      pilot
  px4sim/results.txt:40                   b_apb_100m_w20g40_holdout_field     Altitude mode at 17 s                pilot
                                          _s102 (PX4)
  All are at 100 m. At 5 m and 10 m, ArduCopter held 60 s in 6 of 6 wind runs (apsim/low_ap_results.txt; profile
  outdoor_day _why). [simulator test]
4.2 Tests of the switch-off (release) path
  - tests_apb/test_event.py:145-157 (e3_switch): the run log shows "engaged" and "released" notes. It checks the
    log only, with a stub FC. [unit test]
  - No simulator run measured the flight after Loiter -> AltHold, Stabilize or Acro. integ/integration_2026-11-20/
    review/scripts/cases.py:206-212 builds synthetic run logs for timing only.
4.3 Tests of a program stop, a video stop or a frozen picture
  - Program stop: tests_apb/test_robust.py D1, D2 (lines 153-171: a stopped camera restarts run_live) [unit test];
    attitude stop: last fix 0.03 s later (EVIDENCE.md:136, tests_apb/test_run.py) [unit test]; ssh hang-up stops the
    program (integ/rehearsal_2026-11-20/README.txt:178-185) [simulator test]. No run measured the FC after a
    program stop in a hold (case b).
  - Video stop: 3 s outage, PX4, 100 m: 0.76 m RMS, 1.49 m max (px4sim/results.txt:17-18); ArduCopter, 60 m:
    max 1.30 and 1.70 m, first fix 1.1 s after video back (apsim/engage_results.txt:12-13, 22-23); 20 s outage:
    apsim/drag_results.txt:2-3. [simulator test]
  - Frozen picture: last fix 0.00 s (repeat) and 0.50 s (fresh noise) after the freeze (integ/gap/results/
    test_live_revised.txt); 0 of 336 fixes on static, frozen and NO SIGNAL pictures (tests/bad_pictures.txt);
    old hole: fixes up to 5.96 s after a freeze in a still hover (integ/gap/results/hh_summary.txt); overlay-only
    picture: 21 fixes, open defect (EVIDENCE.md:134-145). [synthetic pictures / unit test]
4.4 Takeover paths with no evidence: F10.


=====================================================================================================================
5. PARAMETERS WE ASK THE EVENT CREW FOR (we ask; the crew sets them; ArduCopter 4.6 names and defaults from source)
=====================================================================================================================
  Parameter          Ask            Default (source)                     Why / what the default does
  FS_EKF_ACTION      2 (AltHold)    1 Land (config.h:103)                Land with GNSS off has no position: it waits 4 s,
                     (1 is OK if                                         then descends wherever the wind carries it. AltHold
                     Ted prefers)                                        keeps the height in the mode Ted flies anyway.
  FS_EKF_THRESH      0.8 (default)  0.8 (config.h:106)                   Never 0: 0 turns the EKF check off (ekf_check.cpp:42).
  FS_EKF_FILT        5 (default)    5 Hz (config.h:114)                  Keep.
  FS_THR_ENABLE      3 (Land)       1 RTL (Parameters.cpp:229)           Default RTL flies to HOME on our position in every
                                                                         mode (F2, F3). Land does not travel.
  BATT_FS_LOW_ACT    0 or 1         0 warning (AP_BattMonitor_Params     2, 3, 4 (RTL, SmartRTL) fly on our position.
  BATT_FS_CRT_ACT    1 (Land)       .cpp:122, 134)
  FS_GCS_ENABLE      0 (default)    0 (Parameters.cpp:165)               If the crew uses it: action Land, not RTL.
  FS_DR_ENABLE       1 (Land)       2 RTL (Parameters.cpp:1048)          No effect without drag fusion. Never 0 with drag
                                                                         fusion (tools/profiles/drag_fusion.json).
  FS_OPTIONS         16 (default)   16 (Parameters.cpp:943)              Keep.
  FLTMODE_CH,        a map with     manual: TL1 = Acro / Position Hold   Remove Return to Home (F2). One position for Acro or
  FLTMODE1-6         Acro or        (GPS) / Return to Home               Stabilize (Ted's takeover), one AltHold, one Loiter.
                     Stabilize,
                     AltHold, Loiter
  ARMING_CHECK       1835006        1 = all (libraries/AP_Arming/        ArduCopter 4.6: every check bit 1-20 except bit 18
  (4.6)              (all checks    AP_Arming.cpp:163; bits AP_Arming.h  (VisualOdometry). Tested in SITL: arms, section 8.1.
                     but vision)    :25-45)                              Never 0.
  ARMING_SKIPCHK     262144         0 = skip none                        Same request on ArduPilot 4.7 and later (the event
  (4.7 and later)    (skip vision   (ArduPilot master libraries/         firmware can be one of them): ARMING_CHECK was
                     only)          AP_Arming/AP_Arming.cpp, read        replaced by this "checks to skip" mask, "PARAMETER_
                                    2026-11-21 on github)                CONVERSION - Added: Dec-2025 for ArduPilot-4.7". Not in
                                                                         the local 4.6 tree. Not tested. Never -1 (skips all).
  RTL_ALT            crew choice    1500 cm (Parameters.cpp:105)         Only matters if RTL stays reachable.
  ACRO_TRAINER       Ted's choice   2 limited (Parameters.cpp:439)       Levels Acro on the EKF attitude (2.6).
  Fence              no RTL action  not read                             A fence RTL flies on our position.
  The EKF source set (VISO_TYPE 1, EK3_SRC1_POSXY 6, VELXY 0, POSZ 1, VELZ 0, YAW 1, GPS1_TYPE 0, VISO_DELAY_MS) is
  our request too (settings.py:60-61). (Phase 1 suggested an EKF-source switch, RCx_OPTION 90. Withdrawn:
  integ/gnss_ladder/README.txt section 5 says not to configure one together with the program. Not tested.)


=====================================================================================================================
6. BENCH DEMONSTRATION FOR TED (propellers OFF, 10 minutes, before the first flight)
=====================================================================================================================
Do this after EVENT_DAY steps 1-6. Do not repeat steps 1, 6 or 6B: step B1 IS step 6B. Battery connected, laptop
cable connected, propellers off (EVENT_DAY rules). The crew's ground station shows the FC messages and the mode. All
pass rules below are expectations from the source; none ran on the aircraft. If a step fails, do not fly.

B0 (1 min) Parameters. The crew reads in the ground station: FS_EKF_ACTION, FS_THR_ENABLE, BATT_FS_LOW_ACT,
   BATT_FS_CRT_ACT, FS_DR_ENABLE, ARMING_CHECK, FLTMODE1-6. Ted sees the values. Pass: the values of section 5,
   or values Ted accepts.
B1 (3 min) Each switch position gives the expected mode name = EVENT_DAY step 6B items 1-6, with Ted watching.
   Pass (6B): one position ALT_HOLD (or Ted's ACRO/STABILIZE), one LOITER or POSHOLD, no RTL, no "mode 11/21".
B2 (1 min) Position hold is refused with no fix. Disarmed, the FC accepts any mode (mode.cpp:290), so the arming
   check shows the refusal. Team: on the laptop, type `tools/deploy.sh stop` (no program, no fix). Ted: switch to
   Position Hold, then try to arm. Pass: the FC does not arm. With the vision check on, the text is
   "Arm: VisOdom: not healthy" (simulator test, section 8.1). With the vision check off, expect "Need Position
   Estimate" (ArduCopter/AP_Arming.cpp:455-459; not tested). If the motors arm: disarm at once, do not fly.
   With a gnss_setup or gnss_fallback profile and a GNSS lock, Position Hold arms on GNSS: skip B2 then.
B3 (2 min) The program stops during a hold (hang). Team: on the board, type `python3 apb_main.py bench --height 1.5`
   and hold the lens 1.5 m above a textured floor until the lines start. Ted: switch to Position Hold.
   Team, in a second ssh window: `pkill -STOP -f "apb_main.py bench"` (the program freezes and sends nothing).
   Pass: the bench lines stop; 5 to 15 s later the ground station shows "EKF variance" (ekf_check.cpp:85-88); the
   mode does not change (disarmed: no action, ekf_check.cpp:173-176). Then: `pkill -CONT -f "apb_main.py bench"`.
   Note for Ted: a frozen program never restarts by itself (F8). In the air the FC goes to Land (or AltHold with
   FS_EKF_ACTION 2) about 8 s after the last fix (section 2.3).
B4 (1 min) The camera is covered. Bench still running, switch in Position Hold. Team: hold a hand on the lens for
   15 s (step 6 item 10 holds it only 2 s). Pass: the lines stop within 0.5 s; "EKF variance" appears 5 to 15 s
   later (wait 30 s after B3: the FC repeats this text at most every 30 s, ekf_check.cpp:15, 85); the lines start
   again within 2 s after the hand is gone.
B5 (1 min) The program is killed. Team: `pkill -KILL -f "apb_main.py bench"`. Pass: the bench output ends and does
   not come back (no restart of a killed process). Ted knows: same FC reaction as B3 and B4.
B6 (1 min) The switch gives manual control at once. Team: start bench again; Ted: Position Hold, then Acro (or
   AltHold), then Position Hold, then Acro. Pass: the ground station shows each new mode within 1 s of each move.
   The run log of the bench (`<log_dir>/run_*.csv`) shows "engaged" and "released" notes; on the laptop
   `~/.venvs/vio/bin/python tools/log_summary.py logs/run_<stamp>.csv` after `tools/deploy.sh pull` lists them.
B7 (1 min, optional) Board computer reset. Team: on the board, type `reboot`. Ted watches the FC (OSD, ground
   station link over the radio, motor-arm tone). Pass: the FC keeps running while the i.MX restarts. If the FC
   restarts too, a computer crash in flight would stop the FC: tell Ted and the crew before any flight (F8).
   "Cable lost": the FC link is an internal UART on the same board (research/hardware.md:20, 75). No cable can be
   pulled; B3 and B5 give the same FC view (no messages).
What the bench cannot show (it needs an armed aircraft), and how the first low flight covers it:
   - The refusal of Position Hold in the air: at 2 m in AltHold the program sends nothing (min_height_m 3). Ted
     selects Position Hold. Pass: the FC stays in AltHold ("Mode change to LOITER failed: requires position").
     Ted moves the switch back. (mode.cpp:322-327)
   - The release: at 10 m, 10 s after Loiter holds, Ted selects AltHold, then Acro. Pass: the mode changes at once
     and the aircraft answers the sticks normally.
   - The EKF failsafe in the air (optional, only if Ted agrees, 10 m, clear area): the team sets outage_test_s 12
     (the recovery_demo profile uses 3) so the program sends no fix for 12 s, 10 s after the switch. Expect Land
     (or AltHold with FS_EKF_ACTION 2) about 8 s into the drop. Ted selects AltHold at once.
   - Everything with a radio failsafe or RTL: not shown. Do not test it.


=====================================================================================================================
7. SIMULATOR PLAN (apsim/safety_matrix.py; results in section 8)
=====================================================================================================================
  ArduCopter 4.6 SITL, variant apb (the real onboard program), video JUSTICE_D, tuning site ottawa2019fall, wind
  w20g40, tuning seed 1 (DATA_SPLITS.md), profile outdoor_day (min_height_m 3, h_min 2), SITL instance 1, SITL
  default failsafe parameters (FS_EKF_ACTION 1). 10 core runs: cases a-e at 10 m and 30 m. 2 spare runs
  (saf_b_10m_s2, saf_c_10m_s2). Each run stops within 300 s. A run starts only at a 1-minute load under 3.
    a  normal hold, AltHold 10 s in: attitude error, speed, height change in the next 10 s.
    b  the program goes silent 10 s in (all messages, as a power loss): time to the FC mode change, the new mode,
       distance flown; AltHold 1 s after the FC change.
    c  ramp 1 m/s: AltHold 3 s after the true distance passes 3 m; largest distance; attitude error after.
    d  20 m jump: same takeover rule; FC reaction (reset or failsafe).
    e  as c with Stabilize, then Acro (harness throttle from the true height), then AltHold.
  Hooks: the script wraps sitl_ap.APLink and the onboard fc_link.FC methods; it does not edit sitl_ap.py.
  Results will go to apsim/safety_results.json and .txt and into this section.
  Phase 2 results: section 8.


=====================================================================================================================
8. PHASE 2 RESULTS (2026-11-21, ArduCopter 4.6 SITL, apsim/safety_matrix.py)
=====================================================================================================================
Files: apsim/safety_results.json and apsim/safety_results.txt (tables); per run apsim/runs/<label>.json, .npz
(truth, EKF, attitude), .parm (the FC parameters), _onboard_*.csv (the program's run log), _settings.json, _sitl.log;
console apsim/logs/saf_<label>.txt. The first arming run before any change of the script is kept as arm_none_s1.

8.1 ARMING WITH THE DEFAULT ARMING CHECKS (task 1, 3 runs)
Setup: sitl_ap.run_case with ARMING_CHECK 1 (the firmware default, written into the .parm on top of the harness's
ARMING_CHECK 0), the program's params_ardupilot (default level), the program running as in a flight (variant apb,
Justice D, outdoor_day floor min_height_m 3). The aircraft is on the ground, so the program sends no fix. Times are
in sim seconds after the FC start. COMMAND_ACK result 0 = ACCEPTED, 4 = FAILED.
- arm_none_s1 (harness not yet corrected): refused in ALT_HOLD (12.7 s) and STABILIZE (16.7 s) with
  "Arm: VisOdom: not healthy" and three harness texts: "Throttle below failsafe" and "Radio failsafe on" (run_case
  holds the throttle at PWM 500, under FS_THR_VALUE 975) and "Accels/Gyros inconsistent" (the IMUs must agree for
  10 s, libraries/AP_Arming/AP_Arming.cpp:419-460; the harness armed too early). With ARMING_CHECK 1835006 the
  vision text went away and only the harness texts stayed. The script then set the throttle to PWM 1000 (a real
  transmitter at low stick) and the first arm attempt to 35 s after the FC start.
- arm_none_s2 (default level):
    30.45  "PreArm: VisOdom: not healthy" (the periodic pre-arm report)
    36.01  ALT_HOLD, arm command: ACK 4, "Arm: VisOdom: not healthy"
    40.02  STABILIZE, arm command: ACK 4, "Arm: VisOdom: not healthy"
    43.03  ARMING_CHECK set to 1835006 (all checks except the vision check)
    46.03  ALT_HOLD, arm command: ACK 0, "Arming motors"
    50.38  "EKF3 IMU0 origin set" (the program's SET_GPS_GLOBAL_ORIGIN); 51.73 in-flight yaw alignment
    52.96  "EKF3 IMU0 is using external nav data", "initial pos NED = 0.0,-0.0,-7.0 (m)" (first fixes above 3 m)
    62.1   LOITER. 30 s hold at 10 m, w20g40: RMS 1.01 m, max 1.57 m. No FC text in the hold: no warning.
- arm_gnss_s1 (profile gnss_setup, GPS1_TYPE 1; the harness arms only after "EKF3 IMU0 is using GPS"):
    30.45  "PreArm: VisOdom: not healthy"; 41.62 "EKF3 IMU0 is using GPS"
    42.13  "Radio Failsafe - Disarming", 42.65 "Radio Failsafe Cleared" (harness throttle before the correction acts)
    45.61  ALT_HOLD: ACK 4, "Arm: VisOdom: not healthy"; 49.63 STABILIZE: ACK 4, the same text
    55.65  ARMING_CHECK 1835006, ALT_HOLD: ACK 0, "Arming motors"
    Run log: "FC reply to EKF source set 2 (our position only): ACCEPTED" 2.0 s before "engaged: FC mode LOITER".
    30 s hold at 10 m: RMS 0.47 m, max 0.93 m. No extra pre-arm text with GNSS in SITL (its GNSS is ideal,
    integ/gnss_ladder/README.txt section 5).
Result [simulator test]: with the vision check on, the FC refuses to arm in every mode, because the program sends no
fix on the ground. Ted sees "PreArm: VisOdom: not healthy" in the ground station, and each arm attempt fails with
"Arm: VisOdom: not healthy". The smallest change that arms: the crew turns off only the vision check.
  ArduCopter 4.6: ARMING_CHECK 1835006 = bits 1-20 except bit 18 (bits libraries/AP_Arming/AP_Arming.h:25-45,
  parameter and default 1 AP_Arming.cpp:163, check_enabled AP_Arming.cpp:284-290, vision check AP_Arming.cpp:
  1939-1953). Tested: arms, climbs to 10 m, holds, no warning.
  ArduPilot 4.7 and later: ARMING_SKIPCHK 262144 (bit 18 only). The local 4.6 tree has no second name. Found in the
  ArduPilot master source (github ArduPilot/ardupilot, libraries/AP_Arming/AP_Arming.cpp, read 2026-11-21):
  AP_GROUPINFO("SKIPCHK", 13, AP_Arming, checks_to_skip, 0), bitmask "18:VisualOdometry", and the conversion
  "PARAMETER_CONVERSION - Added: Dec-2025 for ArduPilot-4.7 / ARMING_CHECK -> ARMING_SKIPCHK". The event
  firmware can be 4.7 or later. It then has ARMING_SKIPCHK (not known). Not tested. Never ARMING_CHECK 0 or
  ARMING_SKIPCHK -1: they turn off every non-mandatory check.
Limits: SITL sensors are ideal; on the real aircraft other checks can fail too (compass, RC, GPS configuration).
Not tested: arming in Loiter with the vision check off (expected "Need Position Estimate", section 6 B2).
A program change instead (for the coordinator to decide): send fixes on the ground so that the vision check passes.
Risks: the EKF gets a position from a picture under 3 m (Hold refuses heights under h_min 2 m, so the fix would be a
fixed or invented value); Loiter could then start at 2 m, which removes the refusal of EVENT_DAY step 7 item 5a
(the proof that the hold cannot start before the program is ready); a fixed ground fix would also hold the EKF still
while the aircraft drifts in the climb. Not recommended. The crew's one-bit change is smaller and keeps every other
check.

8.2 THE TAKEOVER MATRIX (task 2: cases a-e at 10 m and 30 m, seed 1; spare case f at 10 m)
ArduCopter 4.6 SITL, variant apb (the real program, fixed 80 ms board model), Justice D, tuning site ottawa2019fall,
wind w20g40 (20 km/h from 270 deg, gusts to 40 km/h, turbulence 1 m/s), outdoor_day floor, SITL default failsafe
parameters (FS_EKF_ACTION 1 = Land), ARMING_CHECK 0 (harness). One run per cell: one seed, one site, one wind.
THE HARNESS PILOT KEEPS THE STICKS CENTRED AFTER THE TAKEOVER (only case e holds the height on the throttle). A real
pilot flies against the wind; the drift numbers after a takeover show what happens if he does not.
Runs 01:42-02:00, 1-minute load under 3 at each start; each run under 90 s wall time. Table: apsim/safety_results.txt.
Times in s after the failure starts (case a: after the takeover). d = true horizontal distance from the hold point.
tilt / yaw = largest error of the EKF attitude (ATTITUDE) against the true attitude (SIM_STATE).

  case (both heights)       FC texts and modes                     numbers 10 m / 30 m
  a normal hold, AltHold    none; ALT_HOLD at once                 d at takeover 0.54 / 0.09 m. Next 10 s, sticks centred:
    10 s into the hold                                             drift 35.9 / 36.0 m, speed up to 6.45 / 6.46 m/s,
                                                                   height within 0.05 / 0.05 m. Tilt error 0.32 / 0.56 deg
                                                                   (0.22 / 0.21 before), yaw 1.2 / 4.7 deg.
  b program silent          "EKF3 lane switch 1", "EKF primary      FC change after 7.17 / 7.93 s, to LAND. d at the change
    (no message at all)     changed:1", "EKF variance", "EKF       0.40 / 1.88 m; flown since the stop 0.72 / 1.67 m. AltHold
                            Failsafe: changed to LAND Mode";       1 s later: drift to the 60 m harness limit in 9.7 / 10 s,
                            "EKF3 IMU0 stopped aiding" at          up to 7.6 / 7.4 m/s; height within 0.06 / 0.07 m. Tilt
                            14.3 / 15.0 s                          0.35 / 0.60 deg. EKF position error by the takeover 1.55 /
                                                                   3.0 m.
  c ramp 1 m/s              NO FC TEXT, no mode change (Loiter     d passed 3 m at 4.37 / 2.99 s. Takeover 3 s later at
                            flew it away)                          7.11 / 8.37 m, aircraft moving 1.0 / 0.67 m/s. EKF error
                                                                   7.0 / 7.0 m. Tilt error during the ramp up to 1.62 / 2.43
                                                                   deg, after 1.31 / 1.30 deg; yaw error after 9.5 / 9.8 deg.
                                                                   AltHold drift 59 / 58 m in 10 s; height within 0.14 /
                                                                   0.57 m.
  d jump 20 m, removed by   "EKF3 lane switch 1", "EKF primary     10 m: the program re-anchored 0.58 s after the jump (its
    the program's re-anchor changed:1" at 0.15 / 0.12 s; nothing   frame then matched the FC again); EKF error at most 1.4 m.
                            else; no failsafe                      30 m: no re-anchor until 9.45 s; the EKF RESET to the
                                                                   jumped position at 6.98 s (EKF error 20.8 m) and the
                                                                   aircraft did not move (d 1.85 m at 25 s). Takeover at
                                                                   25 s (d never passed 3 m): drift 25 / 23 m in 10 s.
  e ramp, then STABILIZE,   no FC text                             Stabilize at 7.54 / 7.97 m (3.7 / 2.9 s to 3 m). Height
    then ACRO                                                      held within 0.42 / 0.42 m (throttle model). Drift with
                                                                   the wind reached the harness limit of 60 m 9.5 / 9.4 s
                                                                   later, so ACRO WAS NOT FLOWN (the harness selected
                                                                   ALT_HOLD at 60 m). Tilt error after 1.35 / 1.32 deg,
                                                                   yaw 11.7 / 4.8 deg.
  f jump 20 m that the      see 8.3: rejected, then accepted after  10 m only: EKF reset at 7.0 s and again at 22.5 s (40.9 m),
    re-anchor keeps (10 m)  7 s twice; "EKF3 lane switch" only     the aircraft did not move (d 1.70 m at 25 s).

What this means for Ted, path by path (new labels):
- The release (Loiter -> AltHold): [simulator test] accepted at once (0.01 s) at 10 m and 30 m, height within
  0.05 m. With centred sticks the aircraft runs with the wind: 36 m in 10 s, up to 6.5 m/s, in 20 km/h gusting 40.
  The pilot must fly against the wind at once. (Before: [read in the source].)
- Loiter -> Stabilize: [simulator test] accepted at once; the height model held 0.4 m. Loiter -> Acro: NOT flown;
  still [read in the source].
- Program stop (all messages): [simulator test] EKF failsafe and LAND 7.2 s (10 m) and 7.9 s (30 m) after the last
  message, as the source predicted (7 s + up to 1 s). Until then the simulated aircraft stayed within 2 m (ideal
  IMU; a real IMU drifts more). This CONTRADICTS the phase 1 page line "the aircraft drifts" for the first 7 s; the
  page now says what the simulator showed. The texts Ted can see: "EKF variance", "EKF Failsafe: changed to LAND
  Mode". After the takeover, AltHold works normally.
- Slow wrong position: [simulator test] confirmed: no FC text and no mode change while the EKF follows a 1 m/s error
  for 7 s; the aircraft moves away at the ramp speed. Only the pilot stops it.
- Jump: [simulator test] the FC rejects the jumped fixes at once ("EKF3 lane switch 1") and does not move. At 30 m
  the EKF accepted the jump 7.0 s later by a position reset, without moving the aircraft (the hold now runs in a
  frame 20 m off, so HOME and RTL are 20 m off). No failsafe. The source line "it can accept it or start the
  failsafe" is now "it accepted it after 7 s; no failsafe" for a jump that the program does not remove.
- Attitude (F6): [simulator test] tilt error up to 2.4 deg during a 1 m/s ramp and about 1.3 deg after the takeover;
  heading error up to 11.7 deg. AltHold and Stabilize still held height and attitude with centred sticks. Not
  measured: a longer or faster error, Acro, a real IMU.
- RTL, radio failsafe, battery failsafe: still [read in the source] only.
Limits: one seed per cell; one site; one wind; ideal SITL sensors; the harness pilot does not fly against the wind;
case e reached the 60 m harness limit before Acro.

8.3 SPARE RUN: A JUMP THAT THE PROGRAM KEEPS (case f, 10 m, seed 1; used because case d's jump was removed by the
program's own re-anchor at 10 m, a property of the harness hook, not of the FC)
  The 20 m offset stays in every sent fix, also after a re-anchor (local_pos not corrected).
  0.10 s   "EKF3 lane switch 1", "EKF primary changed:1": the FC rejects the jumped fixes. The aircraft does not move.
  7.01 s   The EKF resets its position to the jumped fixes (EKF error over 10 m), as in d at 30 m (6.98 s).
  15.61 s  The program re-anchors to the FC position (now 20 m off) and so adds the offset a second time.
  16.69 s  "EKF3 lane switch 0", "EKF primary changed:0": the second jump is rejected.
  22.45 s  The EKF resets again: EKF error 40.9 m.
  25 s     Takeover (the true distance never passed 3 m): d 1.70 m. AltHold drift 23.5 m in 10 s, tilt error 0.32 deg.
  No EKF failsafe, no mode change, no other text. [simulator test]
  Meaning for Ted: a position jump does not move the aircraft in Loiter (in the sim). The only FC sign is "EKF3 lane
  switch" / "EKF primary changed". The frame, and with it HOME and the RTL target, can move by the size of the jump
  each time. One more reason not to use RTL. A slow error (case c) is the dangerous one.
