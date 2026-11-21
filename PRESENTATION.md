# PRESENTATION.md: 5-minute talk, 2-minute Q&A, and the question bank

The spec tells us what the talk must cover (Stage 1, p.4): **sensor-fusion approach, treatment of rolling shutter,
use of APB compute, response when VIO degrades.** It also says: separate your work from existing tools, show measured
results and limitations, and failures with a clear account still earn credit. Claims without evidence do not.
Judges score evidence, so the talk is a guided tour of `EVIDENCE.md`.

Rule for every number on a slide: say **where it came from** (real flight / real recording / simulation) in the same breath.
Everything before the event is simulated with an assumed camera. Never let a simulated number sound like a flight result.

## Structure (5:00, six slides, about 50 seconds each)

| Time | Slide | What to say | Evidence on the slide |
|---|---|---|---|
| 0:00-0:30 | 1. The problem | A cheap FPV rolling-shutter camera and an IMU, no GNSS, must hold a spot. VIO gets harder with height: features shrink, parallax drops, drift builds. We asked how far this hardware can be pushed. | One photo of the M2-AI; the target in one line: highest altitude, stable hold, everything on the APB |
| 0:30-1:20 | 2. Approach (sensor fusion) | Down-looking Justice D plus FC attitude plus baro height. We track ground points, rotate each ray by the attitude, solve the horizontal move in metres, and send `VISION_POSITION_ESTIMATE` with a variance. The autopilot's EKF fuses it and holds. The pilot's switch is the engage. | One architecture diagram (camera -> Hold -> MAVLink -> EKF -> position hold). Mark which parts are ours and which are ArduPilot/PX4/OpenCV |
| 1:20-2:10 | 3. Rolling shutter | A rolling shutter exposes row by row, so under motion each row sees a different attitude. We give every row its own attitude (from gyro time stamps) and calibrate the readout time and camera delay in flight with a roll/pitch wiggle. | **The with/without comparison** (readout 0 vs calibrated) on real data. If real data is missing, the simulated one, labelled |
| 2:10-2:50 | 4. APB compute | Where the time goes. Hold costs about 7.7 ms per frame at 640x360 on a laptop; on the A53: `[fill in: ms median/p95, fixes/s, CPU %]`. Why we process at half resolution and when we step up. | A small table from `apb_main.py timing` on the real APB |
| 2:50-3:40 | 5. When VIO degrades | Gates for frozen frames, static overlay, no ground, moving objects, attitude mismatch. When a gate trips we **send nothing** rather than a confident wrong position; the EKF coasts. We re-find the anchor after an outage (0.11-0.18 s after a 3 s blackout in simulation). Variance grows with height and with calibration error. | One log plot: fixes stop, then resume. Plus the honest-variance finding (we caught our own overconfident variance) |
| 3:40-4:30 | 6. Results and limits | `[fill in: altitude ladder table: level, held yes/no, RMS/max drift, repeats]`. Then what failed and why (one line per failure). Then the simulated results labelled as simulated. | Ladder table from real flights; hold-out result stated once |
| 4:30-5:00 | Close | Our contribution in one sentence; the main limit (focal-length sensitivity, no yaw during hold); what we would do with another week. | One line each |

### Slide 6 content to prepare now (simulated, label it)
- ArduPilot closed loop, analog camera, 100 m, wind 20 gusting 40: **RMS 0.62 m, max 0.99 m**, drift 1.4 m/min, about 12 fixes/s (`apsim/results.txt`).
- Justice D open-loop sim: RMS about 0.3 m at 100 m, 0.6-1.3 m at 200 m, 1-2 m at 300-400 m (`integ/justice_d_openloop.txt`). Assumed noise and delay.
- **Focal-length error finding:** a 7 % error triples the error (0.35 -> about 1.0 m at 100 m), so we built in-flight calibration, which recovered a 0.93 scale as 0.934 in simulation (`integ/calib_loop_justice_d.txt`).
- Things that failed and what we learned: the FC's own attitude flew away every time (so we integrate the gyro, `att_source=gyro`); the first open-field run flew away (so the whole-picture low-texture path exists); speed-feed made it worse (dropped, kept in the tuning log).
- Test discipline: tuned on one set of sites, validated on a second, hold-out reported once (`DATA_SPLITS.md`).

### Delivery
- Open with the question, not the team name. Speak to the four required topics in the order above so judges can tick them.
- One idea per slide, big numbers, no paragraphs. Put the ownership split (ours vs OpenCV/numpy/pymavlink/ArduPilot/PX4) in the footer of slide 2.
- Put one real-flight artifact on screen (a plot or a short clip) rather than only tables. If a flight failed, show it anyway with the cause.
- Practise to 4:40 so there is slack. Two people: one speaks slides 1-3, the other 4-6, one runs the clicker. Hand off on a number.
- Have `EVIDENCE.md`, the altitude ladder log and one backup plot open on the laptop for questions.

## Q&A (2 minutes: about 3 questions). How to answer
1. Answer in the first sentence. Then one number or one file as proof. Stop. Two minutes is short.
2. Say "we measured X; we did not measure Y" instead of hedging. Honesty about limits is scored (area F).
3. If you do not know: say what you would measure to find out.
4. Route a question to whoever owns that part; do not talk over each other.

## Question bank (prepared answers; fill the brackets with real numbers tomorrow)

### A. Technical: estimation and rolling shutter
1. **How do you handle the rolling shutter?** Every image row gets the attitude at its own exposure time; readout time and camera delay are calibrated in flight from a gentle roll/pitch wiggle. With/without comparison: `[real replay: drift with readout 0 vs calibrated]`.
2. **How do you keep metric scale?** Baro height from the FC, because the camera points straight down and horizontal motion scales with height. No rangefinder assumed. Limit: baro drift and noise; we treat height error as part of the variance.
3. **Why not OpenVINS, ORB-SLAM, VINS-Fusion?** They need a global-shutter camera, a calibrated camera-IMU extrinsic, a lot of CPU, and they target full 6-DoF motion. Our task is a fixed down-looking hold, so a plane-motion estimator with a keyframe anchor is cheaper, more robust to a bad shutter, and runs on four A53 cores.
4. **How do you deal with drift?** Hold is anchored to a keyframe, not integrated frame to frame, so error does not accumulate while the keyframe stays in view. Keyframe changes add variance, which we measured and modelled. `[real: drift m/min]`.
5. **What happens at altitude when features shrink?** Fewer, blurrier corners and less parallax. We step resolution up (`video_scale`) when CPU allows, fall back to a whole-picture method on low texture, and the variance rises so the EKF trusts us less. Limits: simulated error grows from 0.3 m at 100 m to 1-2 m at 300-400 m.
6. **Why integrate the gyro instead of using the FC's attitude?** In closed-loop simulation using the FC attitude alone flew away every time (14.9 m RMS vs 0.62 m with `gyro`; `apsim/results.txt`). We do not have a verified root cause: say so. We integrate the gyro and pull it slowly toward the FC attitude.
7. **How do you calibrate the camera-IMU time offset?** Wiggle test fits the delay and readout from image motion versus gyro; two runs must agree within 3 ms. In simulation a 40 ms start error held 4.5-5.7 m at 100 m; after the wiggle 0.64 m.
8. **What about yaw?** The pilot must not yaw during the hold. A 90 deg yaw leaves 2.2-2.6 m error at 100 m in simulation (E5), because a mount or level error times a yaw change moves the fix. We add variance for it; we do not remove it.
9. **How is your variance honest?** We found our early variance was about 1.7x too small, which made the EKF overconfident. We rebuilt the model per axis (anchor error, frame error, mount-times-yaw, timing) and checked false-fix rate and NEES on hold-out-style runs. (`review/FINDINGS.md` E4.)
10. **What is the biggest weakness?** Focal-length error (7 % triples error) and flat ground at high altitude. We mitigate with in-flight calibration and the no-ground gate, and we say so openly.

### B. Technical: onboard execution and integration
11. **Does it really run on the APB?** `[real: apb_main.py check and timing output]`. If it ran only partly, say exactly which parts.
12. **What is the CPU load / latency?** `[ms median/p95 per frame, fixes/s, CPU %]` at `video_scale 0.5` and `1.0`. Laptop reference: 7.7 ms at 640x360.
13. **How do you share the board with other services?** A reader thread takes every camera buffer so the driver queue never holds old frames (`capture.py`), and each fix carries its age: `[real: fix age ms]`. Competition from other services: `[measure with timing while the aircraft services run]`.
14. **How do you integrate with the FC and the pilot?** MAVLink `VISION_POSITION_ESTIMATE` with covariance; ArduPilot/PX4 EKF does the hold. The pilot's mode switch is the engage and disengage; we never select a mode ourselves, so handover is whatever the pilot flips. Wind assist and speed feed were tried and dropped.
15. **Why ArduPilot (or PX4)?** `[Stage 2 firmware]`. We support both and ran closed loop on both in simulation; Betaflight/INAV have no external position input path in our program.
16. **How does it initialise without GNSS?** It starts a keyframe in the manual setup flight; the pilot steadies the aircraft and engages. `[real: GNSS-free start log, if done]`.

### C. Reliability and reproducibility (area E)
17. **How do you know when the estimate is bad?** Gates (frozen picture, repeated frame, static overlay, no ground, moving object, attitude mismatch) and a variance that grows; on failure we send nothing.
18. **What did you do about moving objects, clouds, shadows, glare?** Per-axis 2.5-sigma gate with re-centring; large object (27-30 % of the picture) gave 1.1 m RMS vs 29.6 m before in simulation.
19. **Is it reproducible?** `[real: three repeats at the best level]` plus multi-seed simulation: 3 seeds on the tuning site, hold-out reported once.
20. **Did you test on data you did not tune on?** Yes: tuning, validation and hold-out sets with a written log (`DATA_SPLITS.md`). One hold-out site failed once; we moved it to validation per our own rule and added a new one. That rule is in the doc.
21. **What happens if the camera dies or the link drops?** Fixes stop; the EKF coasts and the pilot takes over by switching mode. Recovery re-finds the anchor in 0.11-0.18 s after a 3 s blackout in simulation.

### D. Contribution and honesty (area F)
22. **What is yours and what did you use?** Ours: Hold estimator (`hold.py`), onboard program, FC link, calibration, tools, tests, simulator integration. Existing: OpenCV (LK, ORB, phase correlation), numpy, pymavlink, ArduPilot/PX4, Orqa's `mil-mavl` branch, public aerial photos for simulation. We read prior art (Prismo, DroneWuKong, OpenVINS) but did not copy it. `[fill in who wrote which part: EVIDENCE.md]`.
23. **How much is simulation and how much real?** Be exact: `[N real flights, M hours of real recordings]`; everything else simulated with an assumed camera model and labelled.
24. **Did the simulation match the real camera?** `[real vs sim comparison: noise, readout, delay]`; if not measured, say we assumed it and which values calibration then replaced.
25. **What did not work?** The FC attitude (flew away), speed feed (worse at 100 m), the first open-field case (flyaway, fixed with the whole-picture path), 122 m marginal and 200 m failed in earlier analog runs. Each has a cause and a tuning-log line.

### E. Business / operations (speculative: this is a student hackathon, answer with the value, not a product claim)
26. **Where could this be used?** Any small UAS that loses or distrusts GNSS: jamming or spoofing, urban canyons, under bridges, indoor-to-outdoor inspection, search and rescue in tunnels or forests, disaster response, agriculture. A hold-over-a-point capability is the building block for hovering inspection and relay missions.
27. **Why does a cheap rolling-shutter camera matter?** Because most small UAS fly FPV-class cameras. Global-shutter and lidar are cost and weight; a software-only method needs no new hardware and no new supply chain.
28. **How is it better than an optical-flow sensor?** Flow sensors need low height, drift, and give velocity only. We give position over a keyframe at much higher altitude (`[measured max altitude]`) with a calibrated lens and an honest variance.
29. **What about cost and scalability?** No new hardware: a camera already on the aircraft and a computer already in the autopilot board. Per-aircraft cost is the calibration flight. The method is portable across any FC that accepts external vision.
30. **What would it take to make this operational?** Real flight hours across seasons and terrain, certified failure handling (a safe fallback instead of "send nothing"), a no-yaw or yaw-compensated estimator, vibration and temperature calibration, a rangefinder or a second height source, and ground-station integration.
31. **What are the safety implications?** A confident wrong position is the dangerous case, so the design bias is: refuse a fix rather than send a wrong one, keep the pilot's switch as the authority, and keep a variance the autopilot can distrust. We measured false-fix rates, not just RMS.
32. **How would it scale to a fleet?** Calibration is per aircraft; the code is identical. Logs and a replay tool give a repeatable tuning loop. A fleet would need a calibration-quality check at pre-flight, which `apb_main.py check` begins.
33. **What is the dual-use / ethics angle?** Keep it brief and neutral: a stability aid for any small UAS that lowers the skill needed to hover in GNSS-poor places. Do not claim any operational or military use; this is a competition entry.

### F. Gotchas
34. **Why should we believe your numbers?** Every number cites a file and says whether it is simulated or real; hold-out data was run once; failures are in the log.
35. **What would you do with one more week?** Real-camera noise and readout characterisation, a focal-length term in the variance, yaw compensation (E5), and a flight campaign over several surfaces and weather.
36. **What if the Stage 2 firmware were different?** The estimator is firmware-independent; only the message path changes. PX4 and ArduPilot are tested; Betaflight/INAV would need a new output path.
37. **Why didn't you reach a higher altitude?** `[fill in: the limiting factor measured: image detail, wind, CPU, drift]`. Name one cause and one next step.
38. **How would you test this faster?** Closed-loop SITL with the Justice D preset plus raw recordings replayed through the same estimator.

## Last-minute checklist for the talk
- [ ] Every slide number labelled real or simulated.
- [ ] Ladder table with three repeats at the best level; failures listed.
- [ ] One log plot and one with/without rolling-shutter number.
- [ ] Ownership split on slide 2.
- [ ] Timed twice; backup plots open; who answers what decided.
