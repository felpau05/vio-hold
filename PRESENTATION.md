# The talk: 5 minutes, then 2 minutes of questions

Written 2026-11-21 for the presenters. It follows the brief the team got for the final: business value, the idea, the approach, the decisions, why a hold is not yet proven on the Orqa M2-AI and what it takes, and the trade-offs. It also covers the four topics that Stage 1 (page 4) requires: sensor fusion, rolling shutter, APB compute, and the response when VIO degrades.

Three rules for every sentence on stage:
1. Say where a number comes from in the same breath: "in simulation", "on the bench today", "in flight today".
2. Nothing had flown before today. Do not let a simulated number sound like a flight result.
3. A `[TODAY: ...]` slot holds a number that only today's work can give. Fill it or cut the sentence. Never guess it.

## The six slides

| Time | Slide | The one point | Covers |
|---|---|---|---|
| 0:00-0:40 | 1. Why it matters | Small drones lose GNSS first. A software-only hold keeps them useful. | Business value |
| 0:40-1:15 | 2. The idea | A hold is a narrower problem than VIO. Anchor to one picture of the ground. | Idea |
| 1:15-2:20 | 3. How it works | Camera + attitude + height gives a position; the flight controller holds it. | Sensor fusion, rolling shutter, APB compute, degradation |
| 2:20-3:10 | 4. Decisions and their price | Each choice bought something and cost something. | Decisions, trade-offs |
| 3:10-4:15 | 5. Where it stands on the M2-AI | What ran today, what is simulation, and why the rest is not proven yet. | Results, limits, "why not yet" |
| 4:15-5:00 | 6. What it takes | The short list that turns this into a feature of the aircraft. | What is needed, close |

Aim for 4:40 when you rehearse. The script below is about 650 words.

## Script and slide content

### Slide 1. Why it matters (0:00-0:40)
**On the slide:** one line, large: "No GNSS. Same camera. Same board. Still holds." Under it, three small facts with their sources.

**Say:**
"Satellite navigation is the first thing a small drone loses. IATA data, published with EASA, show 220 percent more GPS-loss events on airliners in 2024 than in 2021. In Ukraine, RUSI estimated in 2023 that ten thousand drones were lost every month, in a war where it calls Russian electronic warfare potent. When GNSS goes, a small drone cannot hold its position, and the operator loses the mission or the aircraft.

Canada is already asking for the answer. A National Defence IDEaS challenge for small drones, this year, lists navigation by visual odometry as the fallback when GNSS degrades.

Our answer is software only. It runs on the camera and the computer that the aircraft already carries. No new sensor, no new supply chain."

**Sources (`talk/business_sources.md`):** row 1a (EASA and IATA press release, 2025-06-18), rows 2a and 2b (RUSI, "Meatgrinder", 2023-05-19), row 4b (National Defence, IDEaS challenge "True north precision", 2026-06-11). Each of the four quotes was compared with the text of its page on 2026-11-21 and is the same: `talk/quotes_check_2026-11-21.md`.
**Say it exactly:** the 220 % is IATA's flight data. The 10,000 a month is a 2023 estimate, not a number of today. The IDEaS challenge is mainly about laser rangefinding on small drones; visual odometry is one of its listed outcomes, and the 2.1 million dollars is the funding of the whole challenge.
**Do not say:** "ten thousand drones lost to jamming". RUSI gives the losses and says the electronic warfare is potent. It does not say that jamming caused all of them. Do not claim a camera price gap: no price source was verified.

### Slide 2. The idea (0:40-1:15)
**On the slide:** two pictures side by side: the reference picture at the switch, and a later picture with the measured offset drawn as an arrow in metres.

**Say:**
"General visual-inertial odometry is hard on this hardware: a rolling-shutter FPV camera, no hardware time sync, a small processor. But the task is narrower than general VIO. The pilot steadies the aircraft and moves a switch. From then on we must stay over that one point.

So we do not track a trajectory. We keep a reference picture of the ground, and many times a second we measure how far the aircraft is from that picture, in metres. An error that is measured against one picture does not add up over time while that picture stays in view."

**Backing:** `hold.py` (keyframe, `robust_fit`). A commercial aircraft does the same with a reference image taken when a hover starts: Parrot, ANAFI Ai white paper v7.4.0.0, "Indoor flight", pp. 11-12.

### Slide 3. How it works (1:15-2:20)
**On the slide:** one diagram: Justice D camera -> Hold (ours) -> position + uncertainty over MAVLink -> flight controller EKF -> position hold. Colour our parts. Footer: "Ours: estimator, onboard program, FC link, calibration, tests, simulator harness. Existing: OpenCV, NumPy, pymavlink, ArduPilot, PX4." Four small labels on the diagram, one for each required topic.

**Say:**
"Fusion. We track ground points. We turn each point's ray by the aircraft's attitude, scale it with the barometric height, and solve for the horizontal offset. We send that position, with its uncertainty, to the flight controller. Its EKF fuses it with the IMU and flies the hold. The pilot's switch engages and releases. Our program never flies the aircraft.

Rolling shutter. Every image row is exposed at a different time, so every row gets the attitude of its own moment. We measure the camera delay and the readout time on the ground with a hand turn, and again in flight with a stick wiggle.

Compute. One Python process on a 640 by 360 grey picture. About 3 milliseconds a frame on one laptop core. On the APB today: `[TODAY: Hold.process median ms and fixes per second from "apb_main.py check"]`.

Degradation. When the picture is frozen, covered, or disagrees with the attitude, we send nothing. No position is safer than a wrong position. The flight controller's failsafe and the pilot take over."

**Backing:** sections 1 to 4 below, each number with its file.
**Honest line, if a judge asks about the rolling-shutter gain:** in our simulation the gain is smaller than the spread between runs (0.478 m with the model, 0.485 m without, single runs 0.384 to 0.753 m). We do not claim a gain. `[TODAY: replay one real recording with readout 0 and with the measured readout]`.

### Slide 4. Decisions and their price (2:20-3:10)
**On the slide:** a table with three columns: Decision, What it bought, What it cost.

| Decision | What it bought | What it cost |
|---|---|---|
| Send a position to the flight controller's EKF. No filter of our own on the APB. | The FC's tested fusion and failsafes. A small compute load. One interface for ArduPilot and PX4. | No full 6-axis estimate. We depend on the FC's attitude and on timing. |
| Anchor to a reference picture. | No drift that adds up while the picture is in view. | A new anchor after a loss. The pilot must not yaw in the hold. |
| Height from the barometer gives the scale. | No added sensor (the rules allow none). | A 1 m height error is small at 100 m and large at 5 m. |
| Refuse a doubtful fix. | The FC never gets a confident wrong position from a frozen or covered picture. | Gaps: up to 6.3 s without a fix at 5 m in simulation. |
| Our own attitude from the gyro rates. | With the FC's raw attitude the simulated aircraft flew away (30 m in 16.5 s). With ours: 0.43 m RMS. | We must keep our attitude aligned to the FC's. |
| GNSS: `[TODAY: the level we flew, see "GNSS levels" below]`. | | |

**Say:**
"Five decisions shaped this, and each had a price. We give the flight controller a position and let its EKF do the fusion. That keeps its tested failsafes and a small compute load, and it costs us a full six-axis estimate. We take the scale from the barometer, because the rules allow no added sensor. A one-metre height error is small at a hundred metres and large at five. We refuse a doubtful fix. In simulation that cost gaps of up to six seconds at five metres. That is the price of never sending a confident wrong position.

And some things we tried and dropped, because the measurement said no: speed messages made the hold worse, drag fusion did not help, and our own replay tuner picked wrong values at low height."

**Backing:** "What did not work and why" below, items 6, 7, 8 and 9.

### Slide 5. Where it stands on the M2-AI (3:10-4:15)
**On the slide:** two columns. Left, "On the aircraft today": a ladder of milestones with a tick or a cross and one number each. Right, "In simulation (real firmware in the loop)": the height table. A bold label on each column.

Left column, fill in during the day (one line each, with the log file):
- Board: program deployed and restarted on the APB `[TODAY]`
- Camera: Justice D frames per second and time stamps `[TODAY]`
- FC link: attitude rate, fix age, position accepted by the EKF `[TODAY]`
- Calibration: delay, readout, focal scale, two runs that agree `[TODAY]`
- Bench, propellers off: carry 1 m, read 1 m; cover the lens, fixes stop `[TODAY]`
- Flight: hold for `[TODAY]` s at `[TODAY]` m, RMS `[TODAY]` m, with `[TODAY: GNSS level]`

Right column (simulation with the real firmware in the loop, wind 20 gusting 40 km/h, Justice D camera model, 60 s holds, RMS distance from the hold point, each number is one run). The figure is `talk/hold_vs_height.png`.

| Height | PX4 | ArduCopter | A perfect position (the autopilot alone) |
|---|---|---|---|
| 5 m | 0.51, 0.54 m | 0.44, 0.85 m | 0.35 to 0.46 m |
| 10 m | 0.65, 0.75 m | 0.93, 0.98 m | 0.34 to 0.47 m |
| 50 m | no run | 0.62 m | no run |
| 100 m | no run | 0.44, 0.65, 0.72 m | no run |

- The largest distance in these runs was 0.80 to 1.65 m.
- At 5 and 10 m a perfect position already gives 0.34 to 0.47 m, so the autopilot's response to wind is a large part of the error there.
- 3 s video outage at 60 m: the FC stayed in its hold mode, first fix 1.1 s after the video came back (ArduCopter, 2 runs).
- Files: `px4sim/low_outage_results.txt`, `apsim/low_ap_results.txt`, `integ/justice_d/summary.txt` lines 312 to 316, `apsim/engage_results.txt`.

**Say, if a hold flew:**
"Before today, nothing had flown. Everything was simulation, with the real ArduCopter and PX4 firmware in the loop. Today on the aircraft: `[TODAY: the left column in two sentences, ending with the hold]`. In simulation, in a twenty-kilometre-an-hour wind, the same code stays between half a metre and one metre from the point, RMS, from five metres up to a hundred."

**Say, if no hold flew:**
"Before today, nothing had flown. Today we reached `[TODAY: the last milestone with a tick]`. A full hold on the M2-AI is not proven yet, and we can say exactly why: `[TODAY: the one or two blockers we hit, each with its measurement]`. In simulation, with the real firmware in the loop and a twenty-kilometre-an-hour wind, the same code stays between half a metre and one metre from the point, RMS, from five metres up to a hundred. The gap between those two columns is the work that is left."

### Slide 6. What it takes (4:15-5:00)
**On the slide:** a short list titled "From hackathon to a feature of the aircraft". Then the closing line.

`[TODAY: replace or confirm each item with what the day showed. The list below is what we expect before the first contact with the aircraft.]`
1. A known time offset between a camera frame and the IMU. Today we measure it. A hardware time stamp removes the need.
2. A height source that is better than a barometer near the ground.
3. A flight controller build with a documented, tested input for an outside position, and failsafe settings that the crew has agreed.
4. Flight hours over different ground, light and wind, with an independent position reference.
5. A hold that tolerates yaw.

**Say:**
"What does it take to make this a feature of the M2-AI? A known time offset between the camera and the IMU. A height source that is better than a barometer near the ground. A flight controller build with a tested input for an outside position. And flight hours with an independent reference. `[TODAY: adjust to what the day showed.]`

Most of that is integration and flying, not invention. A low-cost camera, the board that is already on the aircraft, and no satellites. Thank you."

## GNSS levels (decide on the day, say which one flew)
Stage 1, question 14: GNSS before the scored hold is permitted. GNSS must not aid the scored hold. A start with no GNSS at all is a bonus route.

| Level (profile) | What the flight controller uses | Scored hold? | What we have |
|---|---|---|---|
| No GNSS from power-on (default) | Our position only, from the first fix | Yes, and it keeps the bonus route "GNSS-free start" | Simulated holds at 5 to 100 m |
| GNSS before the hold only (`gnss_setup`) | GNSS while the pilot flies by hand. Our position only from 1.5 to 8.7 s before the switch | Yes (Stage 1, question 14). No bonus | 9 simulated runs. A 20 m GNSS error in the hold did not move the aircraft (0.98 m against 0.81 m with no error). The same error moved an aircraft on GNSS by 19.7 m |
| GNSS as a fallback (`gnss_fallback`) | As above. After 3 s with no fix in a hold, GNSS again | Only until the fallback acts. The log says when | 1 simulated run: largest distance 1.52 m |
| GNSS velocity (`gnss_velocity`) | The same as "before the hold only": ArduCopter 4.6 does not fuse the GNSS velocity | Do not use | Read in the ArduCopter source, and 2 simulated runs |

How to choose on the day: `DECISION_CARD.txt` row 45. Results and commands: `integ/gnss_ladder/README.txt`. All of it is simulation with an ideal GNSS. Say on stage which level flew.

**One sentence for slide 4, for the level that flew:**
- No GNSS: "The satellite receiver was off from power-on."
- Before the hold only: "GNSS was on for the take-off. Our program switched the flight controller to our position alone before the pilot's switch, and the log shows it. In simulation a 20-metre GNSS error in the hold did not move the aircraft."
- Fallback: say the second sentence, then: "If our picture fails for three seconds, the flight controller takes GNSS back, and the log marks that hold as GNSS-aided."

## Delivery
- Two speakers at most. Hand over on a number, not on a slide title.
- Open with the first sentence of slide 1, not with the team name. Put the team name and the members on slide 1 in small type.
- One real artifact on the screen from today: a log plot (`tools/plot_log.py`) or a short clip. If a flight failed, show it with its cause.
- Keep `EVIDENCE.md` and one backup plot open for the questions.
- Rehearse twice with a timer. Cut words, not slides.

## The 2 minutes of questions
Two minutes is about three questions.
1. Answer in the first sentence. Give one number or one file as proof. Stop.
2. Say "we measured X, we did not measure Y". Do not hedge.
3. If you do not know, say what you would measure to find out.
4. The person who owns the part answers.

The question bank is at the end of this file.

# Backing detail: the four required topics, with the file behind every number

Before today, nothing had flown. All numbers come from simulation, synthetic pictures or unit tests. Types: SYN synthetic, OL open-loop, CL closed-loop SITL, AN benchmark.

## 1. Sensor-fusion approach
**Message.** Our program measures the drone's position against a ground picture (the keyframe). The flight controller's EKF fuses that position with its IMU.

**How**
- The attitude turns the ray of each tracked point. The baro height scales it. A 2-D similarity fit with outlier rejection gives the position. `hold.py` (`Hold._rays`, `robust_fit`).
- The fix goes out as VISION_POSITION_ESTIMATE with a 2x2 covariance. `fc_link.py`.
- Hold's attitude integrates the FC gyro rates and leans toward the FC attitude with a 20 s time constant. `apb_main.py` (`GyroAtt`), `settings.py` (`att_tau_s`).
- Option, off by default: a fresh home keyframe at the pilot's switch. `hold.py` (`Hold.request_keyframe`). In SITL it made no difference beyond the run-to-run spread (`apsim/engage_results.txt`). Parrot's ANAFI Ai does the same with a reference image taken when a hover starts (Parrot, ANAFI Ai white paper v7.4.0.0, "Indoor flight", pp. 11–12).
- Reason (PX4): EKF2 stops gravity (tilt) fusion while external-vision position aiding is active. Source: `src/modules/ekf2/EKF/aid_sources/gravity/gravity_fusion.cpp` and `src/modules/ekf2/EKF/estimator_interface.cpp` (github.com/PX4/PX4-Autopilot, branch main).

**Numbers**
- Full chain (calibrate, hold), Justice D, 100 m, wind 20 gusting 40 km/h: 0.661, 1.615, 0.783 m RMS. `integ/justice_d/summary.txt:317-319`. After the liveness fix: 0.564 m. `integ/gap/results/closed_summary.txt`. CL, tuning site.
- Low heights, PX4 SITL, Justice D, wind 20 gusting 40 km/h: 5 m 0.51–0.54 m RMS, 10 m 0.65–0.75 m (2 seeds each). A perfect position gives 0.46–0.47 m (1 seed). At low height the autopilot's wind response causes most of the error, not our estimate. `px4sim/low_outage_results.txt`. CL. ArduCopter at the same heights: 0.44–0.98 m RMS against 0.34–0.35 m with a perfect position, with fix gaps up to 6.3 s at 5 m. `apsim/low_ap_results.txt`. CL.
- With the FC's raw attitude, ArduCopter SITL flew away (30.1 m at 16.51 s). With gyro attitude: 0.428 m RMS. `apsim/results.txt` (`ap_apb_100m_calm_s1`, `_attfc`). CL, one seed, analog.

**Figures.** `talk/fusion.png`: the run log of one closed-loop run, made by `tools/plot_log.py` from `integ/gap/results/closed/gapfix_jdcal_apb_100m_w20g40_s11_onboard.csv`. `talk/hold_vs_height.png`: hold error against height for PX4, ArduCopter and a perfect position (simulation, each mark one run). `talk/make_figures.py` makes both. Sources, runs and seeds: `talk/captions.txt`.

**Judge question.** "Why no full VIO filter on the APB?" Answer: the FC EKF already fuses the IMU. We send a position with a covariance and keep our compute small. We did not compare this with a tightly coupled filter.

## 2. Treatment of rolling shutter
**Message.** Hold turns each image row with the attitude at that row's exposure time. The calibration measures the video delay and the readout time, with its sign.

**How**
- Row r gets the time t0 + r / rows × readout. A negative readout means bottom to top. `hold.py` (`Hold._rays`).
- Hold waits until the attitude covers the last row. `apb_main.py` (`run_live`).
- `calib` fits delay and readout from a hand turn. `calib.py` (`solve`).
- `timing` measures them again from an in-flight stick wiggle. `apb_main.py` (`mode_timing`).

**Numbers**
- One calibration run: readout 11 ms (true 10), delay 58 ms (true 58). `integ/justice_d/calib_loop_d_20261118_230048.txt`. SYN.
- In-flight wiggle, analog SITL: delay 62 and 64 ms (true 60.4). `integ/control/wiggle_calib.txt`. CL.
- Benefit: not resolved. At 100 m, RMS median 0.478 m with the model, 0.485 m with readout 0, 0.499 m with the wrong sign. Single runs span 0.384–0.753 m. `integ/justice_d/summary.txt` lines 33, 49, 53. OL.
- Real Justice D readout: not measured. Event day: `calib` and `timing` measure it.

**Figure.** `talk/rolling_shutter.png`: bar chart of the three readout variants per seed (`integ/justice_d/summary.txt` lines 33, 49, 53 and the per-seed rows above each). The seed spread is larger than the effect.

**Judge question.** "How much does the rolling-shutter model help?" Answer: in our simulation, less than the spread between seeds. We cannot claim a gain yet.

## 3. Use of APB compute
**Message.** Hold works on a 640x360 grey picture in one Python process. We timed it on a laptop only. Event day: `check` times it on the APB.

**How**
- A reader thread keeps only the newest frame. `capture.py` (`DigitalSource`).
- Hold gets the Y plane, reduced 2x, with no colour conversion. `capture.py` (`luma`, `shrink`).
- LK 15x15 with 2 pyramid levels, at most 150 points on a 6x4 grid. `hold.py` (`Hold` constants).
- `check` times Hold on the board and fails under 10 fixes/s. `apb_main.py` (`hold_timing`).

**Numbers**
- One laptop core (Intel Core 7 240H): Hold.process median 3.13–3.22 ms at 640x360, 4.57–4.99 ms at 1280x720. Y plane and reduction: 0.28 ms. `integ/justice_d/compute_20261118_225729.json`, `compute_20261118_230038.json`. AN.
- APB: not measured. The offline Python bundle runs on emulated 64-bit ARM (Cortex-A53 model, no network): imports, `check`, Hold and replay pass, with the same numbers as on x86_64. Emulated time is not a speed, so we give no APB time. `tools/bundle/README.txt`.
- With an assumed 80 ms per frame, Justice D SITL gives 12.1–12.2 fixes/s, fix age 129–157 ms. `apsim/sitl_ap.py` (`board_s`), `integ/justice_d/summary.txt:312-320`. CL.

**Demonstration.** Run `python3 apb_main.py check` on the APB. Show the "Hold.process median" line.

**Judge question.** "What runs on the four cores?" Answer: Hold runs in the main loop. The camera reader and the MAVLink reader have their own threads. We did not measure the core load on the APB yet.

## 4. Response when VIO degrades
**Message.** The program sends nothing when in doubt. The flight controller then follows its own settings.

**How**
- Picture checks: repeated buffer, black or covered lens. `capture.py` (`_DigitalJudge`).
- Hold checks: match quality, frozen picture, motion against attitude, and a proof that the picture is live. `hold.py` (`Hold._track1`, `Hold.process`).
- Ten good fixes in a row before sending. Re-anchor to the FC position after a loss. `apb_main.py` (`Pipe`).
- Stop on stale attitude, on height under `min_height_m` (3 m with the outdoor profiles, 15 m by default), and on 2 s without a frame. `apb_main.py` (`live_gate`, `run_live`).
- Each engage and release of the pilot's switch is logged. A recovery demonstration option drops the video for 3 s after the switch. `apb_main.py` (`Engage`, `OutageTest`), profile `recovery_demo.json`.

**Numbers**
- Frozen picture: last fix 0.00 s and 0.50 s after the freeze, limit 1.0 s. `integ/gap/results/test_live_revised.txt`. SYN.
- The liveness fix keeps the old frozen-picture protection: 24 hand-hover freeze runs, same last frozen fix as the old code in every case. `integ/gap/results/hh_summary.txt`. OL.
- Static, frozen and NO SIGNAL pictures: 0 of 336 fixes. `tests/bad_pictures.txt`. SYN.
- 3 s video outage, PX4 SITL: 0.76 m RMS, 1.49 m max. `px4sim/results.txt:17`. CL, analog, one run.
- Recovery demonstration, unit test: first fix 0.0 s after the video came back. `tests_apb/test_event.py` E4. UT.
- Recovery demonstration, ArduCopter SITL, 60 m, wind, 3 s drop, 2 seeds: the FC stayed in Loiter, first fix 1.12 s after the video came back, largest distance 1.30 and 1.70 m. The re-anchor shifted the mean fix error by 0.33 and 0.59 m. `apsim/engage_results.txt`. CL.
- Real FC response: not measured. Event day: cover the lens in the bench test, and fly the recovery demonstration. `tools/log_summary.py` gives the time to the first fix.

**Figure.** `talk/outage_view.png` (a copy of `integ/judge_outage_21s.png`): simulated camera view during the outage.

**Judge question.** "What does the drone do when VIO fails?" Answer: our program stops sending, and the FC failsafe settings act. In ArduCopter, after about 7 s with no fix the EKF failsafe lands by default (`FS_EKF_ACTION`). If drag fusion is on, the dead-reckoning failsafe (`FS_DR_ENABLE`) acts at once, default RTL. The event crew sets these actions. We read this in the ArduCopter source (github.com/orqafpv/ardupilot, `ArduCopter/events.cpp`, `AP_NavEKF3_Control.cpp`) and saw the Land case in SITL only.

## What did not work and why
1. **No fixes just before the switch.** The liveness check refused still-hover frames. SITL gaps reached 4.50 s (Justice D) and 5.28 s (analog) (`integ/gap/results/gap_reasons.txt`). A "moved" proof cut them to 0.10–0.13 s (`integ/gap/results/closed_summary.txt`). Hand-hovers without vibration still show gaps up to 5.83 s (`integ/gap/results/hh_summary.txt`, code v2).
2. **An overlay-only picture still gets fixes.** 21 fixes in 288 frames, as with the old code. The review traced it to the old "still" proof. `integ/gap/results/test_live_revised.txt`, `integ/gap/results/live/overlay_40m_s11_old.json`, `_patchv1.json`, `_v2.json`, `_v3.json`.
3. **The calibration mount check failed.** 0.86° against 0.5° (one run). The fit does not find the lens centre (7 px and 8 px off). `integ/justice_d/calib_loop_d_20261118_230048.txt`.
4. **One run proves little.** The same settings gave 1.615 and 0.783 m RMS. `integ/justice_d/summary.txt:317-318`.
5. **A frozen picture in a still hover is hard to see.** Picture and attitude both stay still, so they agree. Before any fix, fixes continued up to 5.96 s after a freeze with fresh noise. The errors stayed under 0.52 m because the drone did not move. `integ/gap/results/hh_summary.txt` (freeze rows "still"). OL.
6. **ArduPilot drag fusion did not help.** Normal 60 s holds in wind: RMS changed less than the seed spread (60 m: off 0.53 / 0.42 m, on 0.52 / 0.47 m), and the largest drift was higher with drag fusion in all 6 seed pairs (+0.03 to +0.69 m, from the rounded table). A 20 s video outage at 100 m (one seed): both runs drifted 30 m from the hold point, without drag fusion after the EKF failsafe changed the mode to Land, with it after dead reckoning started at 30.3 s. The SITL drag model has the same form as the EKF model, so a real flight is likely worse. `apsim/drag_results.txt`. CL. The team records the idea source before the presentation.
7. **Our replay tuner chose wrong values at low height.** It scores a value by fix jitter. In a rehearsal at 30 m it would set the readout to 0 and the focal scale 8 % wrong, because those values gave less jitter. At 100 m it found the delay within 6 ms. One recording per height. `integ/rehearsal_2026-11-19/`. We now set timing from the ground calibration and the in-flight wiggle. The wiggle found the delay within 1.6 ms at 40 m and 3.6 ms at 100 m in SITL (`integ/control/wiggle_calib.txt`). Below 40 m it is not tested, so we fly the wiggle at 40 m or higher also for a lower hold. The delay does not depend on the height.
8. **Speed messages made the hold worse.** ArduPilot hit an EKF failsafe and flew away (7.11 m RMS). We dropped the feature. `apsim/results.txt` (`c_spd_100m_w20g40_s11`), `DATA_SPLITS.md` line 55.
9. **Fix gaps at 5 m.** ArduCopter SITL lost fixes for up to 6.3 s at 5 m (`apsim/low_ap_results.txt`). Open-loop replays found the main cause: camera vibration that the FC does not see moves the two picture halves 1–3 px differently. The check measures that gap at the picture centre's scale, where it reads 4–8 px. A fix that measures the gap in each half's own pixels cut the refusals at 5 m from 114 to 15 and the longest gap from 2.75 to 1.42 s (3 seeds, OL). But it raised the fix error in most cases and lets more half-frozen pictures through. In ArduCopter SITL (3 seeds, CL) it cut the longest 5 m gap from 3.34 to 1.38 s with no safety event, and the hold error changed less than the seed spread. Not applied. The simulator's vibration (0.2°) is assumed. `integ/lowh/README.txt`, `integ/lowh/closed/README.txt`.
10. **The first `timing` correction was rejected by an independent review.** After the rehearsal, `timing` got a rule that used only the 15 s before the first switch. The review found two blockers. It took the first "engaged" note even when that note was a ground check, so a good wiggle was refused. It did not check that the window held a wiggle, so a hold in gusts passed with a wrong delay. Both are corrected: the program picks the first switch with a clean log before it, and it refuses a window whose strongest 8 s turn less than 4 times the rest. That limit was set on the same four flights that test it (one flight per height, seed 31), so it is a limit, not a proof. On those flights `timing` gave 40 ms at 10 m, a refusal at 30 m, 38 ms at 60 m and 36 ms at 100 m (true 40 ms). `integ/integration_2026-11-20/README.txt`, `integ/integration_2026-11-20/review/REVIEW.txt`, `integ/integration_2026-11-20/timing_with_run_log.txt`.
11. **The flow-count correction for the Betaflight link was not kept.** The link rounds the flow to whole counts, so a small speed becomes 0. A patch carries the remainder (`betasim/patches/msp_link_count_carry.patch`). In one of four Betaflight simulator runs the hold error was larger with it (1.19 against 1.11 m RMS). A rerun of the same code without it moved by 0.16 and 0.68 m, so one run per case cannot decide. The patch is not applied. `integ/integration_2026-11-20/msp_m1.txt`.
12. **A video delay 30 ms too long at 100 m.** One flight per height (seed 31, ArduCopter SITL, Justice D): hold RMS 1.12 m at 60 m and 9.85 m at 100 m. Our answer is the wiggle and `timing` before a high hold. `integ/rehearsal_2026-11-20/README.txt` section 2.B.

## Bonus route: Betaflight (simulator only)
**Message.** The Betaflight firmware has no external-vision input. It has an optical-flow position hold. Our program sends a rangefinder message and a flow message over MSP (MultiWii Serial Protocol), in place of GNSS. We proved the link in the Betaflight simulator only. Nothing ran on real hardware.

**Numbers.** Firmware: Orqa branch `orqa-betaflight_2026.6-maintenance`, commit 09ea1c692f, simulator build with the flow driver added.
- The firmware accepted 200 of 200 rangefinder and 200 of 200 flow messages in 10 s, with no error reply. Its own flow rate was within 2.1 mrad/s of the 200 mrad/s sent, on each of four legs. `betasim/results/proto.txt`.
- Closed loop, simulator, GPS off, 60 s hold, 2 seeds per mode. Mode v1 (no position lock): 3.88 and 4.52 m RMS. Mode v2 (position lock, gain 0.2 per s): 1.11 and 1.24 m RMS. The aircraft stayed in POS HOLD in every sample. `betasim/results/matrix.txt`. CL. Figure: `talk/betaflight_modes.png`.
- The closed loop used a synthetic fix (the true position plus noise, a speed with a constant 0.10 m/s error), not our camera estimate. The plant is the crude model of the firmware's test harness.
- The route stops at 10 m: the firmware's range settings allow at most 1000 cm, and no setting keeps the flow and removes the rangefinder from the height estimate (firmware source, `betasim/results/height_limit.txt`). The event firmware is not known.
- The firmware's own Position Hold on a true virtual GPS position gave 3.13 and 3.44 m RMS in the same simulator. Our link with zero error gave 1.26 and 1.10 m. The plant is crude, so do not compare these numbers with the MAVLink simulators. `betasim/results/baselines.txt`.
- The Betaflight option is off by default (`betasim/enable.sh` turns it on, `disable.sh` turns it off).

**Judge question.** "Does it work on the aircraft?" Answer: not tested. The simulator shows that the firmware holds position on our messages. The bench procedure for the event day is `research/betaflight_bench.txt`.

## Open points from the audit against the manuals
- The flight-mode switch: the aircraft manual gives up Acro, middle Position Hold (GPS), down Return to Home. We do not know which position gives our hold mode. We print each position's mode name before the first flight.
- The board has 4 GB of flash. The `justice_d` settings need 5.3 GB free for raw recording. Without an SD card we use the profile `no_sd`.
- The checkerboard lens fit is a development aid. The scored path is the hand-turn calibration.
- Source: `review/manuals_audit_2026-11-20.md` Tables 1 to 3, and the manual pages in `EVIDENCE.md` areas A, C and D.

# Question bank for the 2 minutes of questions

The questions come from Aydan's list (`PRESENTATION.md` on the branch `origin/spec-justice-d`). The answers use the numbers of this branch, each with its file. "SIM" marks a simulation result. Fill each `[TODAY: ...]` slot or say "we did not measure that".

## A. Estimation and rolling shutter
1. **How do you handle the rolling shutter?** Each image row gets the attitude of its own exposure time. We measure the readout time and the camera delay on the ground with a hand turn, and in flight with a stick wiggle at 40 m or higher. The gain is not resolved in simulation: 0.478 m with the model, 0.485 m without, single runs 0.384 to 0.753 m (`integ/justice_d/summary.txt` lines 33, 49, 53, SIM). `[TODAY: one real recording replayed with readout 0 and with the measured readout]`
2. **How do you keep the metric scale?** The camera looks straight down, so ground motion in the picture scales with the height. The height comes from the flight controller's barometer. A 1 m height error is 1 % at 100 m and 20 % at 5 m. `[TODAY: does the aircraft have a rangefinder, and does the FC report it]`
3. **Why not OpenVINS, ORB-SLAM or VINS-Fusion?** Those systems estimate a full 6-axis path and expect a calibrated camera with a known time offset to the IMU. Our task is one fixed point under a camera that looks down, so we measure the offset from one reference picture. That is less code and less compute. We did not run those systems on this hardware, so we do not say that they fail on it.
4. **How do you deal with drift?** The position is measured against a reference picture, so the error does not add up while that picture is in view. After a loss the program takes a new anchor at the flight controller's position. In simulation that moved the mean fix error by 0.33 and 0.59 m (2 runs, `apsim/engage_results.txt`, SIM). `[TODAY: drift in metres per minute]`
5. **What happens higher up, when the ground detail shrinks?** Fewer corners. A whole-picture method takes over on plain ground, and the uncertainty that we send grows. Simulated up to 100 m: 0.44 to 0.72 m RMS (3 runs, `integ/justice_d/summary.txt` lines 312 to 314, SIM). We have no Justice D run above 100 m.
6. **Why your own attitude and not the flight controller's?** With the FC's raw attitude the simulated aircraft flew away: 30.1 m at 16.5 s. With the gyro rates integrated and pulled slowly toward the FC attitude: 0.43 m RMS (`apsim/results.txt`, rows `ap_apb_100m_calm_s1` and `_attfc`, one run each, SIM). One cause that we read in the PX4 source: EKF2 stops its gravity (tilt) fusion while outside-position aiding is active (`gravity_fusion.cpp`).
7. **How do you find the time offset between camera and IMU?** Two ways. A hand turn on the ground: 58 ms found, 58 ms true, readout 11 ms against 10 ms (one synthetic run, `integ/justice_d/calib_loop_d_20261118_230048.txt`). A stick wiggle in flight: 40 ms at 10 m, a refusal at 30 m, 38 ms at 60 m, 36 ms at 100 m against a true 40 ms (4 simulated flights, `integ/integration_2026-11-20/timing_with_run_log.txt`, SIM). `[TODAY: the two values on the aircraft]`
8. **What about yaw?** The pilot must not yaw in the hold. A tilt error between camera and FC does not cancel when the aircraft yaws: with 1 degree of tilt error, a 90 degree yaw left 3.6 m at 100 m, against 2.5 m predicted (`review/FINDINGS.md`, E5, SIM). A slow full turn at the hold would measure that error. We did not build that.
9. **Is the uncertainty that you send honest?** Our first variance was too small by about 1.7 times in sigma. A review found it. The corrected model gave 0.7 % false fixes on 102 runs that we did not tune on (`review/FINDINGS.md`, E4, SIM).
10. **What is the weakest point?** Three. The scale at low height, because the barometer is coarse there. Plain ground at low height: gaps up to 6.3 s without a fix at 5 m (`apsim/low_ap_results.txt`, SIM). And timing: a delay 30 ms too long gave 9.85 m RMS at 100 m in one simulated flight (`integ/rehearsal_2026-11-20/README.txt`, SIM). That is why we measure the delay on the day.

## B. Onboard execution and integration
11. **Does it run on the APB?** `[TODAY: the "check" output on the board]`. Before today it ran on an emulated 64-bit ARM processor only: imports, `check`, Hold and replay pass (`tools/bundle/README.txt`). An emulator gives no speed.
12. **What are the processing time and the latency?** One laptop core: 3.1 to 3.2 ms a frame at 640 x 360 (`integ/justice_d/compute_20261118_225729.json`). On the APB: `[TODAY: median and 95th percentile in ms, fixes per second, fix age]`.
13. **How do you share the board with the other services?** A reader thread keeps only the newest camera frame, so no old frame waits in a queue. Each fix carries its age. `[TODAY: the same numbers with the pilot video running]`
14. **How does it work with the flight controller and the pilot?** We send VISION_POSITION_ESTIMATE with a covariance over MAVLink. The FC's EKF fuses it and the FC flies the hold. The pilot's mode switch engages and releases the hold. With our event settings the program never arms, never selects a flight mode, never sends a flight target and never overrides the sticks. At start-up it sets the EKF origin and the home point. On the ground, with the crew's agreement, `check --set-params` writes the parameters that select our position as the source; `tools/fc_params.py` saves the aircraft's own values first and puts them back at the end (`integ/pilot_safety/README.txt` section 1, read in the source).
15. **Which firmware?** `[TODAY: the issued build]`. ArduCopter and PX4 both flew our fixes in closed-loop simulation. For Betaflight we built a route over MSP with rangefinder and optical-flow messages. It works in the Betaflight simulator only, and only to 10 m (`betasim/results/`).
16. **How does it start without GNSS?** By default the GNSS is off from power-on. The EKF origin is a map coordinate that we typed in. The program sends fixes above the minimum height before the pilot moves the switch. `[TODAY: which GNSS level flew]`

## C. Reliability
17. **How do you know that an estimate is bad?** Picture checks: a repeated frame, a black or covered lens. Estimator checks: match quality, a frozen picture, picture motion against attitude. Ten good fixes in a row before we send. Static, frozen and no-signal pictures gave 0 sent fixes of 336 (`tests/bad_pictures.txt`, synthetic).
18. **Where does that fail?** One known hole: a flat picture with only overlay text still gave 21 fixes in 288 frames (`integ/gap/results/test_live_revised.txt`). It is open.
19. **Is it repeatable?** `[TODAY: the repeats at the best level]`. In simulation one run proves little: the same settings gave 0.78 and 1.62 m RMS (`integ/justice_d/summary.txt` lines 317 to 318, SIM). So we give the range of several runs.
20. **Did you test on data that you did not tune on?** Yes. Tuning, validation and hold-out sites and seeds are listed in `DATA_SPLITS.md`, with a log of every tuned value.
21. **What happens when the camera or the board fails?** The fixes stop. ArduCopter keeps the hold mode and drifts. In simulation, 7 to 8 s after our program stopped, ArduCopter showed "EKF Failsafe" and changed to Land, and the aircraft was still within 2 m (2 runs, `apsim/safety_results.txt`, SIM). The crew can set AltHold in place of Land. The pilot does not wait for that: in simulation the switch to AltHold took 0.01 s. `[TODAY: the bench steps B3 to B5 of PILOT_SAFETY.txt on the real aircraft]` After a 3 s video outage in simulation the FC stayed in its hold mode, and the first fix came 1.1 s after the video came back (2 runs, `apsim/engage_results.txt`, SIM).

## D. Contribution
22. **What is yours and what did you use?** Ours: the estimator (`hold.py`), the onboard program, the FC link, the calibration, the tools, the tests and the simulator harness. Existing: OpenCV, NumPy, pymavlink, ArduPilot (Orqa's branch), PX4 and public aerial photos. The list with licences is `CONTRIBUTION.md`.
23. **How much is simulation?** `[TODAY: the number of flights and the minutes of real recordings]`. Everything before today is simulation or synthetic pictures, and each number says so.
24. **Did the simulation match the real camera?** `[TODAY: measured delay, readout and lens against the assumed 40 ms, 16 ms and lens]`
25. **What did not work?** Show the backup slide. Speed messages made the hold worse. Drag fusion gave no gain. Our replay tuner chose wrong values below 100 m. The gain of the rolling-shutter model is not resolved.

## E. Business and use
26. **Where is this useful?** For any small aircraft that loses GNSS or cannot trust it: under jamming or spoofing, and close to structures. The National Academies write that a drone under a bridge deck is likely to lose GPS (`talk/business_sources.md` row 6a). A hold over a point is the base for inspection, relay and observation.
27. **Why does a low-cost rolling-shutter camera matter?** The challenge brief says that inexpensive FPV-class cameras are what most small aircraft in the field fly with. A method that needs no new sensor needs no new supply chain.
28. **How is it different from an optical-flow sensor?** A flow sensor gives a speed, so the position drifts, and it needs a rangefinder, which limits the height. Our Betaflight route uses flow messages and stops at 10 m for that reason. We give a position against a reference picture, simulated up to 100 m.
29. **What does it cost for each aircraft?** No added hardware. One lens calibration for each camera type and one timing measurement for each installation. For comparison, a commercial visual-navigation development drone starts at 3,199.99 US dollars (`talk/business_sources.md` row 3c; check the page). Do not claim a camera price gap: we verified no camera price.
30. **What does an operational version need?** Slide 6.
31. **What about safety?** The dangerous case is a confident wrong position. So the program refuses a fix when in doubt, and the pilot's switch stays the authority. One limit we state openly: the flight controller does not detect a slow, wrong position. We sent one that moved away at 1 m/s in simulation: no warning, and the aircraft was 7 to 8 m away when the pilot took over. So the pilot is the monitor (`PILOT_SAFETY.txt`, `apsim/safety_results.txt`, SIM). Before the first flight we showed the pilot each takeover path on the bench, propellers off: `[TODAY: which bench steps passed]`.

## F. Hard questions
32. **Why should we believe your numbers?** Each number names its file and its kind. The hold-out data ran once. The failures are in the list. An independent review rejected our first timing correction, and we fixed its two findings (`integ/integration_2026-11-20/review/REVIEW.txt`).
33. **Did GNSS help your hold?** `[TODAY: the GNSS level that flew, and the log line that shows the flight controller's position source]`
34. **Why not a higher altitude?** `[TODAY: the limit that we measured, with one cause and one next step]`
35. **What would you do with one more week?** Fly with an independent position reference. Then, in this order: a yaw-tolerant hold, a better height source near the ground, and the hot loop in C++.
36. **What if the firmware were different?** The estimator does not depend on the firmware. Only the message path changes: MAVLink for ArduPilot and PX4, MSP for Betaflight.

