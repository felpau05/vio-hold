# TODO before the presentation

Stage 2 scoring: bench A–E (70 pts) is scored with the judge carrying the drone at about 1 m; flight G (20) needs
D and E banked first; F (10) and the timing bonus (+5) at the presentation. We chose **Betaflight** (MSP).
Dev testing: **walk the drone by hand first**, then ask for flight time. Facts measured on the APB: `CONTEXT.md`
(to update) and commit `89b3d64`.

Legend: **[you]** needs hands on the drone · **[code]** software · **[ask]** organizers

---

## 1. Walk test before lunch (critical path)

Goal: carry the drone (props off, battery in, laptop unplugged) and see the live position on the laptop.
A walk can also just be **recorded** now and replayed later, so calibration can follow the walk if needed.

- [x] **[you] Desk checks, 1 min:** cover the lens 3 s (E1 detection on the real sensor); slide the drone
      10 cm forward and back (which image edge is the front); hold it still 5 s at the end.
      Done: top = rear; a hand over the lens is bright and blurry, not dark: now caught by a detail check.
- [x] **[code] Live mode on the APB:** camera + APB IMU + barometer → `Pipe` → position, live at 30 fps
      (`apb_est` inputs, `v4l2_mmap` frames). Check that the A53 keeps up (process time per frame).
      Done: `apb_live.py` (`--rec LABEL` also records). 13-16 fps with the estimator (LK forward+backward
      is most of it); camera and recording at 30 fps.
- [x] **[code] Live display:** web page served by the APB (`http://192.168.75.1:8080`), large READY / NOT READY,
      forward / right (m), distance from zero, height change, **Zero here** button. Refresh about 5 Hz.
- [ ] **[code] Mask** for the drone parts visible at the left and right edges of the picture.
- [x] **[code] Front direction** (`top`, `mirror`) from the slide check.
- [ ] **[you] Battery check:** the APB keeps running with the USB cable unplugged (C needs it).
- [ ] **[you] Distance marks:** tape or floor tiles with known spacing, a tape measure. Ask the organizers for tape.
- [ ] **[you + code] First walk session** (start each on the floor 5 s, then lift to about 1 m):
      still 30 s · 1.0 m forward and back · 1.5 m sideways · 3 m loop back to the start · same loop with the lens
      covered 3 s · lift 0.5 m. Record every walk (`apb_rec.py`) for replay.

## 2. Distances right (before any judged session)

- [ ] **[you] Lens calibration recording:** 30 s, drone upside down, camera at the ceiling, slow turns in every
      direction around the lens. Repeat outdoors later to confirm.
- [ ] **[code] `calib` on apb_rec recordings:** focal length, lens centre, camera-to-IMU rotation (`imu_R_sb`),
      camera delay, rolling-shutter readout. Today's lens profile is a spec-sheet guess and the visible fisheye
      circle says it is far off (about 300 px focal, not 482, at 960 px width).
- [ ] **[code] Height for B3:** the barometer wanders about 0.2 m in this room (the FC barometer sees the same);
      measure the 0.5 m lift with the picture scale too. Measure the lens-on-floor height (assumed 5 cm).
- [ ] **[code] Return to start (B2) and recovery bonus:** re-find the start view after a 3 m loop and after the
      lens was covered. Test on the loop recordings; add ORB matching to the home view if the current re-find fails.
- [ ] **[code] Replay the walk recordings** against the known distances: B1 within 0.25 m, B2 within 0.3 m,
      B3 within 0.15 m, cover → not ready within 1 s. Tune on some walks, check on others (`DATA_SPLITS.md`).

## 3. Bench items C, D, E (needed before we may fly)

- [ ] **[code] MSP link from the APB:** send `MSP_SET_RAW_RC` at 50 Hz **from start-up**; not ready = roll,
      pitch, yaw 1500 and a hold-height throttle, never minimum values. Read CH10 (engage), RC, attitude, altitude.
- [ ] **[you + code] Confirm the channel order** on the Betaflight Receiver tab, props off, before any real
      command (Stage 2 says roll, pitch, throttle, yaw; `MSP_RC` reads back roll, pitch, yaw, throttle).
- [ ] **[code] C:** with the switch on, roll and pitch oppose a 0.5 m move (left, right, forward, back).
- [ ] **[code] E1:** covered lens → not ready within 1 s, safe values while covered, normal only after uncovering.
- [ ] **[code] E2:** auto-start from `/data` after a power-cycle, safe values from the first command, ready
      within 60 s (cold-boot camera start included: the video node boots in another format).
- [ ] **[code] GNSS:** our code never requests GPS; a short note and the code path to show the judge.
- [ ] **[you] Rehearse A–E** with the exact Stage 2 bench steps before booking a session (3 sessions, 2 tries per item).

## 4. Flight (after D and E are banked)

- [ ] **[code] Position and altitude controller:** position → target angles (roll/pitch µs) in ANGLE mode;
      barometer → throttle, starting from the pilot's throttle at engage. Simple angle-mode model to tune it offline.
- [ ] **[you] Props-off check** of the full solution, then development flights at level 1 (10 m).
- [ ] **[code] Score our own flight logs** with `hold_judge.py` (3 m / 2 m for 30 s).
- [ ] **[code] GNSS-free start bonus (+5):** no GNSS from power-up to landing; declare it before take-off, keep
      the code and logs to show. Our path already never reads GPS: make it provable (no GPS request in the MSP code).
- [ ] **[you + code] Climb the ladder:** G pays 12 / 14 / 16 / 18 / 20 for 10 / 20 / 35 / 50 / 75 m. Each level
      after a full 30 s hold. Check the estimate at height (fewer floor details, larger height error) on the way up.
- [ ] Betaflight integration bonus (+5) comes with any qualifying hold on Betaflight: nothing extra to do.

## 5. Presentation (F 10 pts + timing bonus up to 5)

- [ ] **[code] Timing and rolling-shutter evidence:** measured frame stamps (EOF, 16.67 ms), camera delay and
      readout from calibration, IMU stamp fix, uncached-buffer finding; plots.
- [ ] **[code] Results:** walk replays against tape distances, bench rehearsal results, flight logs scored with
      `hold_judge.py`.
- [ ] **[you] Story and architecture slides**; update `PRESENTATION.md`.

## 6. If time allows (low points per hour; only after 1–5)

- [ ] 60 fps through the ISI hardware scaler (today 30 fps; uncached buffers cost 12 ms per full-size copy).
      Helps accuracy while turning and the timing story, but 30 fps is enough to score.
- [ ] FC gyro and barometer over MSP as backup inputs if an APB sensor fails or freezes (the APB barometer froze
      for over a minute twice in the drift log).
- [ ] Gyro bias against temperature (board at 56–64 °C); refresh the bias whenever the drone is still.
- [ ] Closed-loop simulator with a Betaflight angle-mode model (beyond the simple model needed for section 4).
- [ ] Hold-out evaluation runs and a written error budget for the presentation.
- [ ] Update `CONTEXT.md`, `SPEC.md`, `HARDWARE_BRINGUP.md` with the Stage 2 and APB findings.
- [ ] ArduCopter fallback (flash, tune) only if the Betaflight path fails; it costs flight time and has no bonus.
- [ ] Forward Justice A: not scored, did not stream through `/dev/video3` (passthrough); skip unless needed.

## Ask the organizers

- [ ] Is a radio supplied or bound for bench sessions? (None was bound at first check.)
- [ ] May the drone power up on the floor (barometer zero) before the judge lifts it?
- [ ] Is a systemd service in `/data` allowed for auto-start (E2)?
- [ ] Are our own Python packages in `/data` fine (OpenCV 4.11 unpacked there; no system changes)?
- [ ] Tape for floor marks in the dev area?
