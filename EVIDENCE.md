# Evidence and presentation log

The spec says credit follows demonstrated work, and claims alone score nothing (areas A-G, Stage 1 p.3-4).
Keep this file current: one line per result, with the file that proves it and what it does NOT prove.
Scored runs use the event evaluation setup; team development aids do not count (so no markers or tripods in the evidence).

## Who wrote what (area F: separate your work from existing tools)

| Part | Ours or existing |
|---|---|
| Hold estimator (`hold.py`): keyframe anchor, per-row rolling-shutter rays, similarity fit, gates, variance model | ours (fill in who) |
| Onboard program, FC link, calibration, tools | ours |
| OpenCV (LK, ORB, phase correlation), numpy, pymavlink | existing libraries |
| ArduPilot / PX4 (EKF fusion, position hold) | existing firmware; Orqa's `mil-mavl` ArduPilot branch for SITL |
| Prior art studied (research/): Prismo, DroneWuKong ports, OpenVINS candidate | existing; say what was only read |

## Results

| Area | Result | File | Camera | Proves | Does not prove |
|---|---|---|---|---|---|
| A | | | | | |
| B | Open-loop and closed-loop hold at 100 m, wind 20 gust 40 (ArduCopter SITL): tuning site RMS 1.56 m; 2 of 3 hold-out sites held | `apsim/summary.txt` | analog (simulated) | the estimator and control loop in simulation | the Justice D, real flight, the open-field case (flew away) |
| C | | | | | |
| D | Onboard-program tests 9/9 pass (fake flight controller) | `tests_apb/out_*.txt` | simulated | message flow, calibration, replay logic | a real FC |
| E | | | | | |
| G | none yet | | | | |

## Presentation outline (spec p.4: sensor fusion, rolling shutter, APB compute, response when VIO degrades)

1. **Sensor fusion:** gyro-integrated attitude pulled toward the FC attitude, camera rays rotated per row, baro height for scale; fix sent as `VISION_POSITION_ESTIMATE` with covariance.
2. **Rolling shutter:** each image row uses the attitude at its own exposure time (`hold.py`); readout time and delay are calibrated (`calib.py`). Bonus route: measured improvement (needs a with/without comparison on real Justice D data).
3. **APB compute:** Hold cost per frame, fixes per second, CPU headroom (HARDWARE_BRINGUP.md row 8).
4. **Degraded VIO:** the gates (frozen, static, no ground, moving object, attitude mismatch), what is sent when a gate fails (nothing), how it re-anchors, what the pilot does.
5. **Limits and failures:** the hold-out field flyaway, analog-only simulation, unverified Justice D numbers. Say them plainly.
