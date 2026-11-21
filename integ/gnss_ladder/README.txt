HOW MUCH GNSS: FOUR LEVELS (2026-11-21, 00:41 to 01:35)

Words: FC = flight controller. EKF = the FC's state estimator. SITL = the simulator that runs the real FC firmware.
Program = apb_main.py. Fix = one position that the program sends. Hold = the time after the pilot's switch selects
Loiter. Set 1, set 2 = EKF source sets (parameters EK3_SRC1_*, EK3_SRC2_*). RMS = root mean square.
Source: apsim/ardupilot (Orqa's branch of ArduCopter 4.6, commit 3b40393). Line numbers are in that tree.
Python: ~/.venvs/vio/bin/python. Nothing is committed. The branch is stage1-justice-d.
The default stays gnss_mode "none": GNSS off from power-on, the same messages as before (test G2).

FILES IN THIS FOLDER
README.txt              this file
flight_loop.diff        the change: apb_main.py, fc_link.py, settings.py, apsim/sitl_ap.py, tests_apb/run_all.sh
apb_main_before.py      apb_main.py before the change. tests_apb/test_gnss.py G2 runs it as the reference.
sitl_ap_before.py       apsim/sitl_ap.py before the change
tests.txt               the output of the four tests
results.txt             a copy of apsim/gnss_results.txt (the SITL table)


SUMMARY (ArduCopter 4.6 SITL, Justice D, tuning site, wind w20g40, 30 s from the switch, 16 runs)
- gnss_setup WORKS. The FC accepted "set 2 = our position only" 1.5 to 8.7 s before the switch in all 9 runs with
  GNSS. EKF step at the change 0.02 to 0.23 m (in AltHold), at the switch 0.01 m. Hold at 30 m equal to gnss_none
  (0.45 / 0.81 and 0.43 / 0.84 m against 0.46 / 0.83 and 0.45 / 1.00 m, rms / max). At 10 m lower than gnss_none
  (0.59 / 0.99 and 0.42 / 0.71 against 0.81 / 1.36 and 0.75 / 1.10), inferred from a longer settled EKF before the
  switch, not from GNSS in the hold.
- GNSS DID NOT AID THE HOLD. A 20 m GNSS error in the hold (step or 1 m/s ramp) did not move the set-2 EKF or
  aircraft (largest 0.98 m, 0.81 m without the error). The same ramp moved a GNSS-only aircraft 19.7 m.
- gnss_fallback WORKS: video dropped 6 s, set 1 (GNSS) 3.0 s after the last fix, FC ACCEPTED, EV 85 in the FC log,
  largest distance 1.52 m. That hold is GNSS-aided from 13.1 s, and the log says so.
- gnss_velocity DOES NOT DO WHAT ITS NAME SAYS on ArduCopter 4.6: EKF3 fuses GNSS velocity only while GNSS is
  the position source (AP_NavEKF3_PosVelFusion.cpp 559-562). In SITL it held like gnss_setup and the EKF fused no
  velocity in the hold (constant XKF3 innovations). Keep it out of the plan, or tell the judges it is setup.
- Two traps found. (1) The pilot must arm only after "EKF3 IMU0 is using GPS". In the one run where the pilot
  armed 2 s after the GNSS was found, the EKF did not use GNSS in 21 s of flight, and Loiter was refused until
  set 2 (run gn_setup_10m_s1_nolock). (2) The FC sends no STATUSTEXT at a set change. The proof is the run log
  plus the FC's BIN log (section 4).
- PX4: parameters written, NOT TESTED. The program refuses gnss_mode setup, fallback, velocity on PX4.


1. THE SOURCE: ANSWERS WITH FILE AND LINE

Q1. Values of EK3_SRCn_*.
  POSXY and VELXY: 0 none, 3 GPS, 4 beacon, 5 optical flow (VELXY only), 6 external nav, 7 wheel encoder (VELXY
  only). POSZ: 0 none, 1 baro, 2 range finder, 3 GPS, 4 beacon, 6 external nav. VELZ: 0 none, 3 GPS, 4 beacon,
  6 external nav. YAW: 0 none, 1 compass, 2 GPS, 3 GPS with compass fallback, 6 external nav, 8 GSF.
  libraries/AP_NavEKF/AP_NavEKF_Source.h 17-46 (enums), AP_NavEKF_Source.cpp 29-59 (parameter values).
  Our position arrives as VISION_POSITION_ESTIMATE with VISO_TYPE 1 and is "external nav" (6).
  Defaults in the firmware: SRC1 = GPS for POSXY, VELXY, VELZ, baro for POSZ, compass for YAW
  (AP_NavEKF_Source.cpp 31, 38, 45, 52, 59). SRC2_POSXY default is 0 = none (AP_NavEKF_Source.cpp 67).

Q2. Active set after power-on. The command.
  After power-on set 1 is active. active_source_set has no start value (AP_NavEKF_Source.h 133). The object is part
  of the AHRS singleton, which has static storage, so the value is 0 = set 1. Only an RC switch with RCx_OPTION 90
  can select another set at boot (RC_Channel.cpp 674). The team has no such switch.
  MAV_CMD_SET_EKF_SOURCE_SET (42007) exists. GCS_Common.cpp 5603-5604 calls handle_command_set_ekf_source_set,
  GCS_Common.cpp 4908-4918. param1 1, 2 or 3 selects set 1, 2 or 3. The FC replies ACCEPTED for 1 to 3, DENIED
  for other values. A COMMAND_LONG is changed into the COMMAND_INT handler (GCS_Common.cpp 5115).
  Seen in SITL: our COMMAND_LONG got COMMAND_ACK ACCEPTED in every GNSS run (run log notes).

Q3. Position and velocity at a set change.
  The EKF sees the change of the POSXY source and sets posxy_source_reset (AP_NavEKF3_PosVelFusion.cpp 547-552).
  With the new source = external nav, the next fix resets the EKF horizontal position to our fix:
  ResetPositionNE(extNav pos) (AP_NavEKF3_PosVelFusion.cpp 626-636). With the new source = GPS, it resets to the
  GPS position (AP_NavEKF3_PosVelFusion.cpp 606-623). The velocity is not reset. The height does not change when
  POSZ is the same in both sets (both 1 = baro here).
  No alignment step is needed for external nav: readyToUseExtNav needs only tilt alignment and fix data
  (AP_NavEKF3_Control.cpp 609-620). The FC does NOT align our position to its own position for VISO_TYPE 1:
  align_inactive_sources calls align_position_to_ahrs (AP_NavEKF_Source.cpp 250-291), but the MAVLink back end
  has the empty default (AP_VisualOdom_Backend.h 53). Only VISO_TYPE 2 and 3 (T265, VOXL) align
  (AP_VisualOdom_IntelT265.cpp 223). So the step at the change is (our fix - EKF position). The program's
  re-anchor to the FC position (Pipe._reanchor, "re-anchored to FC position") keeps that step small: with GNSS on,
  LOCAL_POSITION_NED exists before the first fix. Measured in SITL: 0.02 to 0.23 m (section 2, stepSet2).
  In Loiter, a reset does not move the aircraft: AC_PosControl::handle_ekf_xy_reset moves the target by the same
  step (AC_PosControl.cpp 1496-1512). Normally the change comes before the switch, in AltHold, which does not use
  the horizontal position.
  GNSS velocity with our position (gnss_velocity): EKF3 fuses GPS velocity only inside the branch where POSXY is
  GPS (AP_NavEKF3_PosVelFusion.cpp 559-562). There is no other place that fuses GPS velocity (the only other
  velocity fusion is external nav velocity, line 591). So EK3_SRC2_VELXY 3 with EK3_SRC2_POSXY 6 fuses NO GNSS
  velocity. The GSF yaw estimator also runs only when POSXY is GPS (AP_NavEKF3_Control.cpp 809-813, 830-834).

Q4. What the FC sends or logs at a set change.
  No STATUSTEXT for the MAVLink command (GCS_Common.cpp 4908-4918). Only the RC switch path sends "Using EKF
  Source Set %u" (RC_Channel.cpp 1647). The FC writes an event to its log (BIN): EV Id 85, 86, 87 = sources set
  to primary, secondary, tertiary (AP_Logger.h 94-96, written in AP_NavEKF_Source.cpp 151-164). EKF3 also logs a
  DAL event for replay (AP_NavEKF3.cpp 1110-1116). The proof is therefore: the COMMAND_ACK in our run log, and
  EV 86 before MODE 5 in the BIN log (section 4).

Q5. GNSS type parameter.
  GPS1_TYPE (AP_GPS.cpp 274: subgroup "1_"; AP_GPS_Params.cpp 24-30: "TYPE"). The old GPS_TYPE is converted
  (AP_GPS.cpp 131). fc_link.PARAM_ALIAS reads GPS_TYPE when GPS1_TYPE has no answer. GPS1_TYPE has
  "@RebootRequired: True" (AP_GPS_Params.cpp 29): a change needs an FC restart. The EK3_SRC parameters have no
  reboot flag. The set itself changes in flight with the command.

Q6. Pre-arm checks and failsafes with GNSS on.
  Arming: the GPS checks run only when the flight mode needs GPS (Loiter, PosHold ...), a fence needs it or
  super simple is on (ArduCopter/AP_Arming.cpp 344-374). In Stabilize or AltHold the pilot can arm with no GNSS
  lock. With GPS1_TYPE 1 the general checks can still report "GPS 1: Bad fix" or "not healthy"
  (libraries/AP_Arming/AP_Arming.cpp 663, 669, inside gps_checks) when a mode needs GPS. Not tested on hardware:
  SITL runs with ARMING_CHECK 0.
  "GPS glitching" blocks arming in a GPS mode (ArduCopter/AP_Arming.cpp 475). In flight, the EKF raises
  gps_glitching only while POSXY is GPS (AP_NavEKF3_Control.cpp 790), and Copter then sends "GPS Glitch or Compass
  error" (ArduCopter/events.cpp 303-310). In set 2 this cannot happen. The EKF failsafe is the same in all levels.
  Seen in SITL: without a GNSS position, Loiter is refused: "Mode change to LOITER failed: requires position"
  until the first fix is fused (run gn_setup_10m_s1_nolock).

Q7. EKF origin.
  The program sends SET_GPS_GLOBAL_ORIGIN and MAV_CMD_DO_SET_HOME with origin_lat_lon_alt at the start of run
  (apb_main.run_live, fc_link.FC.send_origin). EKF3 refuses a second origin (AP_NavEKF3_Control.cpp 696-700).
  GPS sets the origin only when there is none (AP_NavEKF3_Measurements.cpp 671-681). So the first one wins. In
  SITL both orders happened: the program first (gn_setup_10m_s1_nolock, "origin set" 0.15 s before the program's
  first note) and GPS first (gn_setup_30m_s1, "origin set" and "Field Elevation Set: 70m" 26 s before the program
  started). Home: Copter moves home to the EKF position while disarmed (ArduCopter/commands.cpp 4-20) and again
  at arming (ArduCopter/AP_Arming.cpp 712-722), so the program's DO_SET_HOME does not stay. The program's height
  is GLOBAL_POSITION_INT relative_alt, height above home (fc_link), so it is the height above the arming point
  in all levels.
  What must change: nothing for the hold. A wrong origin_lat_lon_alt only moves the local frame. The program still
  needs origin_lat_lon_alt in all levels (start_problems is not changed). Inferred: if GNSS never locks, the
  program's origin is the one the EKF uses, as in gnss_none.


2. RESULTS IN ARDUCOPTER 4.6 SITL

How: apsim/gnss_matrix.py, results apsim/gnss_results.json and gnss_results.txt (copy: results.txt here).
Variant apb (the program, board model 80 ms per frame, 230400-baud relay), video JUSTICE_D, tuning site
ottawa2019fall, tuning seeds 1 and 2, wind w20g40, 40 s holds. Every run has the outdoor_day floor (min_height_m 3,
hold_cfg h_min 2). In the GNSS levels the simulated pilot arms after "EKF3 IMU0 is using GPS" (new wait in
apsim/sitl_ap.py). One run at a time, 1-minute load under 3 at each start, SITL instance 1. 16 simulator runs.
Run files: apsim/runs/<run name>.* (.json, .npz, _onboard.csv = the run log, .BIN = the FC log).
Columns, over the 30 s from the switch:
  rms, max   true horizontal distance from the position at the switch (m)
  stepSet2   largest EKF position step within 1 s of the FC's ACCEPTED reply to set 2, less the true motion (m)
  set2 at    time of that reply (s after the switch, negative = before)
  stepSw     the same step at the pilot's switch (m)
  herr       largest |true height - height at the switch| (m)
  stepSet1, set1 at, maxAft1   the fallback to set 1: step, time, largest distance from the switch position after it
  GPS        yes = the EKF used GNSS before the change; no = no GNSS yet; only = GNSS-only Loiter, no program

level     h m seed  GPS   rms   max stepSet2 set2 at stepSw  herr stepSet1 set1 at maxAft1  run file (apsim/runs/)
none       10    1    -  0.81  1.36        -       -   0.01  0.08        -       -       -  gn_none_10m_s1.*
none       10    2    -  0.75  1.10        -       -   0.01  0.07        -       -       -  gn_none_10m_s2.*
setup      10    1  yes  0.59  0.99     0.04    -4.0   0.01  0.04        -       -       -  gn_setup_10m_s1.*
setup      10    1   no  0.92  1.71     0.00    -1.2   0.05  0.21        -       -       -  gn_setup_10m_s1_nolock.*
setup      10    2  yes  0.42  0.71     0.02    -1.5   0.01  0.04        -       -       -  gn_setup_10m_s2.*
velocity   10    1  yes  0.50  1.03     0.02    -1.9   0.01  0.03        -       -       -  gn_velocity_10m_s1.*
none       30    1    -  0.46  0.83        -       -   0.01  0.06        -       -       -  gn_none_30m_s1.*
none       30    2    -  0.45  1.00        -       -   0.01  0.10        -       -       -  gn_none_30m_s2.*
setup      30    1  yes  0.45  0.81     0.20    -8.7   0.01  0.09        -       -       -  gn_setup_30m_s1.*
setup      30    2  yes  0.43  0.84     0.22    -8.4   0.01  0.07        -       -       -  gn_setup_30m_s2.*
fallback   30    1  yes  0.76  1.52     0.02    -8.0   0.01  0.10     0.10    13.1    1.52  gn_fallback_30m_s1.*
velocity   30    1  yes  0.57  0.93     0.21    -8.0   0.01  0.07        -       -       -  gn_velocity_30m_s1.*
setup      30    1  yes  0.47  0.98     0.15    -7.6   0.01  0.07        -       -       -  gn_setup_30m_s1_glitch.*
setup      30    1  yes  0.52  0.92     0.23    -8.1   0.01  0.09        -       -       -  gn_setup_30m_s1_ramp.*
gps        30    1 only  0.54  1.07        -       -   0.02  0.09        -       -       -  gn_gps_30m_s1_glitch.*
gps        30    1 only 10.95 19.69        -       -   0.02  0.10        -       -       -  gn_gps_30m_s1_ramp.*

What the rows show:
a. gnss_setup works. In every run with GNSS the FC accepted set 2 before the switch: 1.5 to 8.7 s before. The EKF
   step at the change was 0.02 to 0.23 m, in AltHold. The step at the switch was 0.01 m. Height error 0.10 m or less.
b. gnss_setup against gnss_none, same seed (rms / max m):
     10 m seed 1   0.59 / 0.99  against  0.81 / 1.36
     10 m seed 2   0.42 / 0.71  against  0.75 / 1.10
     30 m seed 1   0.45 / 0.81  against  0.46 / 0.83
     30 m seed 2   0.43 / 0.84  against  0.45 / 1.00
   Spread between runs: the same set-2 configuration and seed at 30 m gave 0.45, 0.47 (glitch run) and 0.57
   (velocity run, the same EKF inputs, Q3). At 30 m there is no difference beyond that spread.
   At 10 m gnss_setup was lower in both seeds. Cause (from the FC texts, inferred): in gnss_none at 10 m the EKF
   started to use our fixes only 2.5 s and 0.95 s before the switch ("EKF3 IMU0 is using external nav data"),
   because the climb is short and the simulated pilot hovers 4 s. At 30 m that came 10.4 s before. In gnss_setup
   the EKF used GNSS 15 s before the switch, so its velocity and bias states had settled. So the gain is a longer
   settled EKF before the switch, not GNSS in the hold. Not tested: gnss_none with a longer hover at 10 m. It may
   give the same gain without GNSS.
c. With no GNSS lock (gn_setup_10m_s1_nolock, the first run, before the wait was added): set 1 had no position.
   Loiter was refused ("requires position") until set 2. Then the EKF started on our fixes. The level then acts as
   gnss_none. Hold 0.92 / 1.71 m.
d. GNSS error during the hold, 20 m north, the same seed (30 m seed 1). The BIN log shows the error at the FC:
   GPS Lat moved 20 m at 10.0 s (gn_setup_30m_s1_glitch).
   Rows: true distance from the switch position, rms / max m, and the span of the EKF north position (m), before and
   after the error starts.
                               before         after          EKF N span before / after
     step at 10 s:
       set 2 (setup)           0.31 / 0.59    0.54 / 0.98    0.36 / 0.72     gn_setup_30m_s1_glitch
       set 2, no error         0.36 / 0.64    0.49 / 0.81    0.50 / 0.67     gn_setup_30m_s1 (same windows)
       GNSS-only Loiter        0.47 / 0.79    0.57 / 1.07    0.47 / 20.63    gn_gps_30m_s1_glitch
     ramp 1 m/s from 5 s to 25 s (20 m at 25 s):
       set 2 (setup)           0.22 / 0.42    0.56 / 0.92    0.27 / 0.67     gn_setup_30m_s1_ramp
       set 2, no error         0.29 / 0.43    0.48 / 0.81    0.44 / 0.67     gn_setup_30m_s1 (same windows)
       GNSS-only Loiter        0.31 / 0.43   12.00 / 19.69   0.21 / 0.94     gn_gps_30m_s1_ramp
   On set 2 neither error moved the EKF position or the aircraft beyond the run without an error. No FC text.
   GNSS-only, step: the EKF position jumped 20.6 m ("GPS Glitch or Compass error" at 11.3 s, "Glitch cleared" at
   29.9 s), but the aircraft stayed: ArduCopter moves the Loiter target with each EKF position reset
   (AC_PosControl.cpp 1496-1512). So a step shows that the EKF used the GNSS, but it does not move the aircraft.
   GNSS-only, ramp: the EKF followed the GNSS and the aircraft flew 19.7 m away from the hold point while the EKF
   thought it held (EKF span 0.94 m). This is the control: the injection moves an aircraft that uses GNSS.
e. gnss_fallback works (gn_fallback_30m_s1). Video dropped at 10.0 s for 6 s. At 13.0 s: "GNSS FALLBACK: no fix for
   3.0 s in the hold". FC reply ACCEPTED at 13.1 s. BIN: EV 85 at 13.1 s. EKF step 0.10 m. Distance from the switch
   position: 0.37 m at the drop, 0.65 m at the change, then up to 1.52 m (13 to 16 s), 1.50 m at 30 s. The same seed
   without a drop (gn_setup_30m_s1): largest 0.81 m. The program did not select set 2 again in that hold.
f. gnss_velocity: 30 m 0.57 / 0.93, 10 m 0.50 / 1.03. It held like gnss_setup. The FC log shows that it fused no
   GNSS velocity (g).
g. FC-side evidence, EKF core 0 velocity innovations (XKF3 IVN, IVE in the BIN log, 300 samples in the 30 s):
   in every set-2 hold, also gnss_velocity, one constant value (no horizontal velocity fused). In GNSS-aided holds
   they change: GNSS-only control 6 and 10 distinct values, gn_fallback_30m_s1 (after set 1) 6 and 14,
   GNSS-only ramp 52 and 14. Weak alone (the ideal SITL GNSS gives tiny innovations), but it agrees with Q3.
h. No FC STATUSTEXT came at any set change (all runs). The run log and the BIN log have the change (section 4).


3. EVENT DAY: HOW TO SELECT A LEVEL

All levels: deploy the code and settings.json together. The old program refuses a settings.json that has
gnss_mode ("unknown key"). Replace <profiles> with the profiles you use today (for example outdoor_day justice_d).

gnss_none (default, today):
  1. python3 tools/use_profile.py apply <profiles>
  2. ~/.venvs/vio/bin/python apb_main.py check --set-params
  3. Restart the FC if check wrote a parameter. Do check again. Each param line must show PASS.
  Tell the pilot: nothing changes.

gnss_setup:
  1. python3 tools/use_profile.py apply <profiles> gnss_setup
  2. ~/.venvs/vio/bin/python apb_main.py check --set-params
  3. Restart the FC. GPS1_TYPE needs a restart.
  4. Do check again. The 12 GNSS param lines and "PASS gnss_mode setup: settings match the level" must show.
  5. Start run as usual.
  Tell the pilot:
  1. Wait for "EKF3 IMU0 is using GPS" in the ground station before you arm. In SITL it came 42 s after the FC
     started, in each run (gps_in_use_s in apsim/runs/gn_*.json).
  2. Arm in Stabilize or AltHold. Climb to the hold height.
  3. Hover steady for 10 s before you select Loiter. The program needs about 2 s of fixes plus the time to its
     first fix. In SITL the FC accepted set 2 between 1.5 and 8.7 s before
     the switch (the simulated pilot hovers only 4 s).
  4. If the team calls "GNSS warning": select AltHold, hover steady, then select Loiter again.
  The team: watch the run log if you have a link to the board:
  tail -f logs/run_*.csv | grep -E "source set|WARNING|engaged|released"
  Before the switch you must see "FC reply to EKF source set 2 (our position only): ACCEPTED".

gnss_fallback:
  Steps 1 to 5 as gnss_setup, with gnss_fallback instead of gnss_setup.
  Tell the pilot: as gnss_setup. If the video fails for gnss_fallback_s (3 s) in a hold, the FC holds on GNSS.
  That hold is not a no-GNSS hold. After the video comes back, select AltHold, then Loiter again. The program
  selects set 2 again only after the release.

gnss_velocity:
  Steps 1 to 5 as gnss_setup, with gnss_velocity instead of gnss_setup.
  Tell the pilot and the judges: demonstration only, not a scored hold. On ArduCopter 4.6 it does not do what
  its name says: the EKF does not fuse GNSS velocity while our position is the position source (Q3). In SITL it
  held like gnss_setup.

To go back to gnss_none: apply the profiles without gnss_*, check --set-params, restart the FC (GPS1_TYPE 0).
use_profile.py removes the EK3_SRC2_* keys from settings.json. The FC keeps its EK3_SRC2_* values, but set 2
is never selected in gnss_none, so they do nothing.

PX4 (NOT TESTED, no PX4 SITL run was made). EKF2 has no source sets and no command to change them.
The program refuses gnss_mode setup, fallback and velocity on PX4 (start_problems: "needs ArduPilot EKF source
sets"). The parameter set that would give the four levels (EKF2_GPS_CTRL bits: 1 lon/lat, 2 altitude,
4 3D velocity, 8 dual antenna heading; px4sim/PX4-Autopilot src/modules/ekf2/params_gnss.yaml 5-19):
  none:      EKF2_GPS_CTRL 0, EKF2_EV_CTRL 1 (today's params_px4)
  setup:     EKF2_GPS_CTRL 5 (lon/lat + velocity) at power-on, EKF2_EV_CTRL 1. The program would have to write
             EKF2_GPS_CTRL 0 with PARAM_SET before the switch. Inferred from gps_control.cpp 205, 443: EKF2 reads
             the bit at each update, so the change acts in flight. Not implemented.
  fallback:  as setup, and PARAM_SET EKF2_GPS_CTRL 5 when fixes stop in the hold. Not implemented.
  velocity:  EKF2_GPS_CTRL 4 (velocity only), EKF2_EV_CTRL 1, from power-on. No action of the program. PX4 EKF2,
             unlike ArduPilot EKF3, fuses GNSS velocity with external position (inferred, not tested).


4. THE LOG LINES THAT PROVE THAT GNSS DID NOT AID A HOLD

a. Run log (logs/run_<stamp>.csv, column "note"):
   "FC reply to EKF source set 2 (our position only): ACCEPTED" on a row before "engaged: FC mode LOITER", and
   between "engaged" and "released" no row with "source set 1", "GNSS FALLBACK" or "WARNING".
b. FC log (BIN, on the FC's SD card): EV Id 86 (sources set to secondary) before MODE Mode 5 (Loiter), and no
   EV Id 85 before the next MODE row. Read it with:
   ~/.venvs/vio/bin/mavlogdump.py --types EV,MODE <log>.BIN
   Example, run gn_setup_30m_s1: EV 86 at TimeUS 52156629, MODE 5 at TimeUS 60859813 (8.7 s later).
c. FC log PARM rows: EK3_SRC2_POSXY 6, EK3_SRC2_VELXY 0, EK3_SRC2_VELZ 0, EK3_SRC2_POSZ 1, EK3_SRC_OPTIONS 0:
   ~/.venvs/vio/bin/mavlogdump.py --types PARM <log>.BIN | grep -E "EK3_SRC|GPS1_TYPE"
d. The behaviour test in SITL (section 2, glitch table): a 20 m GNSS error in the hold did not move the
   aircraft on set 2, and moved the GNSS-only aircraft.
The FC sends no STATUSTEXT at the change (Q4). A ground station message list therefore shows nothing.


5. NOT TESTED, ONLY INFERRED, AND THE LIMITS OF THE SIMULATOR

Simulator limits:
- The SITL GNSS is ideal: SIM_GPS_NOISE 0 (no noise, no wander), 10 satellites, 5 Hz, no multipath, no outage
  (libraries/SITL/SITL.cpp 757-808). A real GNSS wanders by metres. So the step at the change to set 2 (stepSet2)
  is measured with a perfect GNSS position. Inferred: on the drone the step is (our drift since the first fix) +
  (GNSS wander since the first fix), because the program re-anchors to the FC (GNSS) position at its first fix.
  It happens in AltHold, so it does not move the aircraft. In the hold it does not matter: set 2 uses no GNSS.
- The SITL origin, the SITL home and the program's origin_lat_lon_alt are the same point.
- ARMING_CHECK 0 in SITL. The real pre-arm checks with GPS1_TYPE 1 are not tested (Q6).
- The time to "EKF3 IMU0 is using GPS" (42 s after the FC started, in SITL) is the SITL time. A real receiver
  needs its cold-start time on top.
- Only the tuning site (ottawa2019fall) and tuning seeds 1 and 2, wind w20g40, 40 s holds, 30 s windows.
  One run per case. At 30 m the differences between gnss_none and gnss_setup are inside the run-to-run spread.
- The board model and the 230400-baud relay are the ones of the other apsim runs (fixed 80 ms per frame).
- Run gn_setup_10m_s1_nolock (the first run) used GnssLevel before two small edits: the limit of one request
  each second (after a DENIED or a missing reply) and the plain-English WARNING and FALLBACK texts. Neither edit
  acts in a normal flow (request, ACCEPTED). Runs gn_none_10m_s1 and gn_setup_30m_s1 also had the old texts.
  Neither text occurs in them. All other runs used the final code.

Not tested:
- PX4 (no PX4 SITL run). Betaflight is out of scope.
- Real FC hardware, a real GNSS receiver, a real SD-card log.
- The ArduCopter build of the event aircraft, if it is newer than 4.6. Its source-set code may differ.
- A restart of run_live in flight (the camera restart of mode_run). Inferred: the new GnssLevel does not know the
  active set, so it sends set 2 again when fixes flow. The command is idempotent: the EKF resets only when the
  POSXY source changes (AP_NavEKF3_PosVelFusion.cpp 549).
- An RC switch with RCx_OPTION 90 (EKF source set) together with the program. Do not configure one.
- gnss_fallback at 10 m. A normal fix gap at 10 m can reach 3 s (apsim/low_ap_results.txt), the same as
  gnss_fallback_s. Inferred: at 10 m a normal gap can start a fallback. Then that hold is GNSS-aided.

Only from the source (not by a run): Q1, Q2 (start value), Q4 (no STATUSTEXT), Q5, Q6, the PX4 parameter set,
and that gnss_velocity fuses no GNSS velocity (Q3). The runs agree with the source where they touch it: COMMAND_ACK
ACCEPTED, EV 86 in the BIN log, no STATUSTEXT at the change, the position step, and constant XKF3 velocity
innovations in the hold (section 2).


6. THE CHANGE AND THE TESTS

The flight-loop change (flight_loop.diff, apb_main.py part): one constructor and one call in run_live.
    eng, outage, gnss = Engage(s), OutageTest(s), GnssLevel(s)
    ...
    drop, onote = outage.step(time.monotonic(), eng.t_engaged)
    onote = _join(onote, gnss.step(fc, time.monotonic(), last_send, eng.t_engaged))
GnssLevel.step returns None at once for gnss_mode "none" or a firmware other than ArduPilot. _join(onote, None)
is onote. So the log rows and the messages of gnss_none do not change (test G2 compares them with the old file).
Other parts: GNSS_AP (the parameter table of each level), gnss_problems (start_problems refuses a level whose
parameters are not the profile's: set 2 with POSXY 0 would leave the EKF with no position), one check line,
fc_link.FC.set_source_set, settings gnss_mode / gnss_ready_s / gnss_fallback_s, and in apsim/sitl_ap.py a wait
for "is using GPS" before arming in a GNSS level (gnss_mode "none" and the other variants do not reach it).
Profiles: tools/profiles/gnss_setup.json, gnss_fallback.json, gnss_velocity.json.

Tests (tests.txt):
  tests_apb/test_gnss.py      ALL PASS in 32 s (new, no simulator; a stand-in FC whose real fc_link send methods
                              write MAVLink 2 bytes that the test decodes)
    G1 the setting: default "none", "gps" refused by settings.load
    G2 gnss_mode "none" against apb_main_before.py, same frames and FC: the same MAVLink messages
       (SET_GPS_GLOBAL_ORIGIN 1, COMMAND_INT 1, VISION_POSITION_ESTIMATE 75 in both), the same log columns and notes,
       no COMMAND_LONG 42007. Positive control: gnss_mode "setup" adds exactly one new message, COMMAND_LONG 42007.
    G3 setup: set 2 requested 1.0 s after the first fix, FC reply logged before the switch, no WARNING
    G4 switch before the program is ready: one WARNING note, then set 2 "in a hold"
    G5 fallback: commands 2, 1, 2. Set 1 after the drop, not set 2 again in that hold, set 2 after the release
    G6 an FC that denies or does not answer: about one request each second (4 in 4 s), each one logged
    G7 gnss_problems for "none", setup without the profile, PX4; the profiles equal apb_main.GNSS_AP;
       start_problems refuses a level with the wrong parameters
    G8 tools/use_profile.py apply <profiles> gnss_<level> on a copy of settings.json, then apply without it: the
       level's parameters are in, VISO_DELAY_MS is kept, and the file is the same as before the apply
    G9 check mode with a stand-in FC (GPS_TYPE name, one wrong value): one line for each of the 12 parameters,
       one FAIL for EK3_SRC2_POSXY, "PASS gnss_mode setup: settings match the level"
  tests_apb/test_event.py     ALL PASS in 26 s
  tests_apb/test_run.py       ALL PASS in 28 s
  tests_apb/test_fc_link.py   ALL PASS in 20 s
  The full tests_apb/run_all.sh was not run (as asked). It has one new line for test_gnss.py.
  check against a live SITL FC was not run. G9 covers the check lines with a stand-in FC.


7. ADDED 2026-11-21 07:15 TO 07:35: THE LEVELS TOGETHER WITH THE PROFILE low_height
Why: the low-height A/B of integ/kfmap ran only with gnss_mode none. A GNSS level with low_height was not tested.
How: apsim/gnss_matrix.py, label suffix _lh (att_source gyro_fcyaw, hold_cfg dense_local and dense_turn on top of
the level). The same site, wind, seeds and measures as section 2. The laptop was on mains power.
level     h m seed   rms   max stepSet2 set2 at stepSw  herr stepSet1 set1 at maxAft1  run file (apsim/runs/)
none       10    1  0.44  0.87        -       -   0.00  0.18        -       -       -  gn_none_10m_s1_lh.*
none       10    2  0.41  0.90        -       -   0.00  0.10        -       -       -  gn_none_10m_s2_lh.*
setup      10    1  0.42  0.79     0.11    -4.9   0.01  0.04        -       -       -  gn_setup_10m_s1_lh.*
setup      10    2  0.38  0.84     0.09    -3.5   0.01  0.04        -       -       -  gn_setup_10m_s2_lh.*
setup       5    1  0.42  0.83     0.02    -5.6   0.01  0.06        -       -       -  gn_setup_5m_s1_lh.*
setup       5    2  0.40  0.74     0.03    -5.5   0.01  0.05        -       -       -  gn_setup_5m_s2_lh.*
fallback   10    1  0.76  1.55     0.11    -5.3   0.01  0.03     0.32    13.0    1.55  gn_fallback_10m_s1_lh.*
What the rows show: gnss_setup with low_height holds as well as no GNSS with low_height (inside the spread between
runs). The change to set 2 came before the switch in each run. The fallback worked as in section 2 e: video dropped
at 10.0 s, "GNSS FALLBACK" at 12.9 s, FC ACCEPTED at 13.0 s, EKF step 0.32 m, largest distance 1.55 m.
Not tested: 30 m and higher with low_height and a GNSS level (low_height is not for those heights), more than 2
seeds, a GNSS error in the hold with low_height.

