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
| A | Calibration loop recovers a 0.93 focal scale as 0.934, mount tilt within 0.2 deg, readout and delay within tolerance | `integ/calib_loop_justice_d.txt` | Justice D (simulated, assumed noise) | the calibration method works when the sim model is right | the real lens, real timing, real data quality |
| B | Open-loop and closed-loop hold at 100 m, wind 20 gust 40 (ArduCopter SITL): tuning site RMS 1.56 m; 2 of 3 hold-out sites held | `apsim/summary.txt` | analog (simulated) | the estimator and control loop in simulation | the Justice D, real flight, the open-field case (flew away) |
| C | Hold takes about 7.7 ms median per 640x360 frame on a laptop | `SPEC.md` open item 3 | n/a (x86 laptop) | rough cost | anything about the A53 cores: run `apb_main.py timing` on the APB |
| D | Onboard-program tests 9/9 pass (fake flight controller) | `tests_apb/out_*.txt` | simulated | message flow, calibration, replay logic | a real FC |
| B | Open loop Justice D sim: RMS about 0.3 m at 100 m, 0.6-1.3 m at 200 m; a 7 % focal error triples it | `integ/justice_d_openloop.txt` | Justice D (simulated, assumed) | estimator accuracy vs height and the focal-error sensitivity | no drone physics, wind or real camera; hold-out not run |
| E | Gates for frozen, static, no-ground, moving-object and attitude mismatch; `att_source=fc` flies away so `gyro` is fixed | `tests/bad_pictures.py`, `review/FINDINGS.md`, `apsim/results.txt` | simulated | bad inputs are refused in simulation | behaviour on real degraded flight |
| G | none yet | | | | |

## Presentation outline (spec p.4: sensor fusion, rolling shutter, APB compute, response when VIO degrades)

1. **Sensor fusion:** gyro-integrated attitude pulled toward the FC attitude, camera rays rotated per row, baro height for scale; fix sent as `VISION_POSITION_ESTIMATE` with covariance.
2. **Rolling shutter:** each image row uses the attitude at its own exposure time (`hold.py`); readout time and delay are calibrated (`calib.py`). Bonus route: measured improvement (needs a with/without comparison on real Justice D data).
3. **APB compute:** Hold cost per frame, fixes per second, CPU headroom (HARDWARE_BRINGUP.md row 8).
4. **Degraded VIO:** the gates (frozen, static, no ground, moving object, attitude mismatch), what is sent when a gate fails (nothing), how it re-anchors, what the pilot does.
5. **Limits and failures:** the analog hold-out field flyaway, analog closed-loop results, and Justice D open-loop results based on assumed camera parameters.