# TOMORROW.md: event-day plan (rewritten 2026-11-20 after reading every doc)

Goal: **win the flight part (highest altitude with a qualifying stable hold) and bank evidence in areas A-G.**
Presentation and Q&A: `PRESENTATION.md`. Commands and pass rules: `EVENT_DAY.txt`. What to record: `HARDWARE_BRINGUP.md`.
"If X then change Y": `DECISION_CARD.txt` (ignore row 5's "goal B" and row 15 unless it is dark). Rules: `SPEC.md`.

## Rules that never bend
- No markers, tarps or fiducials in the solution or any scored run (dev area only). No hardware or mount changes. No extra sensors.
- No GNSS aiding in the scored hold. No yaw during the hold (E5: 90 deg yaw leaves 2.2-2.6 m at 100 m in simulation).
- Use profile **goal_a** (highest altitude). Never goal_b. `att_source` stays `gyro`.
- One change at a time. Copy every settings file that flew well: `cp settings.json good_HHMM.json`.
- Pilots (Ted, Sujan) fly. If the drone drifts, the pilot goes back to AltHold/Altitude at once.
- Every number goes into `EVIDENCE.md` with the file that proves it, failures included. Claims alone score nothing.

## Roles (adjust to team size)
- **Lead / Stage 2 reader:** decides firmware, talks to organizers and pilots, owns the clock.
- **Board person:** SSH, `board_check`, `check`, deploy, logs pull.
- **Calibration person:** lens intrinsics (checkerboard on a tripod, dev area), `calib`, `bench`.
- **Analyst:** plots, `replay_tune`, ablations, fills `EVIDENCE.md` tables.
- **Scribe / presenter-prep:** timestamps every action and result; builds slides from `EVIDENCE.md` as results arrive.

## Tonight (before bed)
- [ ] Docker Desktop on; `sh tests_apb/run_all.sh` all pass (test_ap_sitl needs Docker). Last check: 9/10 pass, Docker was off.
- [ ] If time: one closed-loop Justice D run (`python apsim/sitl_ap.py apb 100 w20g40 60 camera=justice_d label=jd1`). It has never run.
- [ ] Build `tools/wheels/` for aarch64 (gitignored, missing) so pip failure does not stop you. Put it on the USB stick.
- [ ] Checkerboard printed on something rigid and flat; tape measure; phone compass; USB stick; laptop charger; ethernet cable.
- [ ] Commit and push the doc changes; open the PR so the branch is the thing everyone has.
- [ ] Send `ORGANIZER_QUESTIONS.md` to the event thread if not sent.
- [ ] Draft the 6 slides with simulated numbers (PRESENTATION.md) so tomorrow is only swapping in real data.
- [ ] Sleep.

## Block 0: Stage 2 (first 20-30 min, whole team reads)
Write the answers into `SPEC.md` "Still open" and strike `ORGANIZER_QUESTIONS.md`:
firmware list, APB access and deploy method, Python/numpy/OpenCV on the image, camera acquisition, timestamps,
rangefinder, approved altitudes, hold definition (radius, duration, drift), scoring and bonus routes, evidence rules,
flight booking. **Book pilot flights immediately.**
- Choose ArduPilot or PX4 (both supported; Betaflight/INAV are not). Prefer ArduPilot if offered (Orqa `mil-mavl` branch is what we simulated).
- Apply the profile: `python3 tools/use_profile.py apply outdoor_day goal_a <px4|ardupilot>`.
- Decide **which flights are HOLD-OUT now** (DATA_SPLITS.md): reserve the last altitude ladder flights; never tune on them.

## Block 1: Board and link (60 min). Nothing works without it
1. Connect, find the board address, write `tools/deploy.env` (`BOARD=<address>`), then `NO_RESTART=1 tools/deploy.sh`.
2. On board: `sh tools/board_check.sh`. Fix each FAIL (wheels, `tools/wheels/README.txt`). Record versions.
3. **Props off.** Power the drone. `python3 apb_main.py check --set-params`, then **reboot the FC**, then `check` again. Every `param` line PASS.
4. **ArduPilot only:** set `origin_lat_lon_alt` in `settings.json` (field lat, lon, alt: phone GPS is fine, it is only the EKF origin). Without it Loiter is never accepted. Redeploy.
5. Check: ATTITUDE 100 Hz or more (else set the `SRn_` rates of the APB port to 0), fix age under 150 ms (record it), TIMESYNC passes.
6. Compass: point the nose north with the phone compass; heading within 10 degrees.
7. Confirm GPS is off/ignored in the FC (`GPS1_TYPE 0`, `EKF2_GPS_CTRL 0` or the firmware's equivalent). The brief says GNSS may be degraded rather than absent, so it must not leak in.
**Stop rule:** firmware is px4 or ardupilot, `board_check` passes, params PASS. If not, ask organizers; do not go on.

## Block 2: Camera and first real data (45 min)
1. `ls /dev/video*`, `v4l2-ctl --list-devices`, `v4l2-ctl -d /dev/videoN --all`. Fix `video_device` or write a GStreamer `video_pipeline` (HARDWARE_BRINGUP row 3).
2. `apb_main.py check`: about 60 frames/s, 1280x720, stamps from "driver". Record jitter (ms).
3. `apb_main.py run --raw`, hand-carry over textured ground. Copy the `.npz` frames to the laptop and keep them. **First real Justice D data.**
4. Log the data set in the DATA_SPLITS tuning log (what it is, which set).
**Stop rule:** a stable picture with timestamps. If the camera will not open, fix that before anything else.

## Block 3: Calibration, props off (75 min, run in parallel)
- **Person A (dev area):** checkerboard on a tripod, Justice D intrinsics. Replace the nominal profile in `justice_lens_profiles.json`. Record RMS reprojection error and whether the lens is equidistant.
- **Person B (EVENT_DAY steps 3-6):**
  1. `mask` (10 s, tilt by hand): PASS, covers under 50 %.
  2. `calib` twice outdoors with scenery over 50 m away. Two delays within 3 ms. Focal scale 0.98 to 1.02.
  3. `bench --height 1.5`: carry 1.00 m forward: +0.90 to +1.10 (lens check +0.97 to +1.03; if not, multiply `focal_scale` by the value), right -0.10 to +0.10; carry right; tilt 15 deg: change under 0.10 m; hand on the lens 2 s: lines stop within 0.5 s and restart.
  4. `timing`: CPU at `video_scale` 0.5 and 1.0 on the A53, fixes per second.
- Scribe records every number in `HARDWARE_BRINGUP.md` and `EVIDENCE.md`. `cp settings.json good_calib.json`.
**Stop rule:** focal scale 0.98-1.02, bench in range. If not, do not fly (DECISION_CARD rows 1, 2, 6). Focal error is the biggest threat found: a 7 % error triples the position error.

## Block 4: First flight (with pilots, props on)
1. Start `tools/deploy.sh` (runs `run --raw`). **PX4: wait 60 s** before selecting Position mode (time sync).
2. Pilot: AltHold, climb above 15 m (we send nothing below `min_height_m`), still 5 s, no yaw.
3. Timing wiggle at 20 m or more, clear area: rock roll/pitch about 30 % each way every 1.5 s for 8 s, settle 4 s (EVENT_DAY 7b).
4. Pilot selects Loiter/Position; hold 30 s; hands off. Pilot lands.
5. Pull logs: `tools/deploy.sh pull`; `python tools/plot_log.py logs/run_<newest>.csv`; `python3 apb_main.py timing logs/raw_*_00N.npz`.
6. ArduPilot: `python3 apb_main.py delay logs/run_<newest>.csv`, set `VISO_DELAY_MS` to the number printed, redo parameters. PX4: `EKF2_DELAY_MAX` = fix age + 100.
7. `python tools/replay_tune.py "logs/raw_<stamp>_*.npz"`; merge a patch only if it helped: `use_profile.py merge logs/tune/tune_patch.json`. At most two tune loops.
**Pass:** drone stays over the point, 10+ fixes/s through the hold, fix age under 150 ms, `sent = 1` after the switch.
**If worse after a change:** `cp good_calib.json settings.json`, redeploy.

## Block 5: Altitude ladder (the scoring part)
1. Use the approved levels from Stage 2, lowest first. One 30 s (or the defined duration) hold per level, pull and plot each flight.
2. At each level record: fixes/s, fix age, fraction degraded, max and RMS drift from the log, wind (ask the pilots), height.
3. If a level fails: DECISION_CARD rows 3 and 5. Options: try `video_scale` 1.0 (more detail, more CPU: check CPU first), fly over more textured ground (choosing the spot is not a marker), re-run `calib`.
4. **Keep the last good level repeatable**: the spec scores reproduction across runs, not a single lucky one. Fly the best level 3 times and note all three.
5. Record every failure with a one-line cause (the presentation uses them).
**Stop rule:** the highest level where the drone stays over the same point with 10+ fixes/s for the full hold, three times.

## Block 6: Rehearsal, evidence, freeze (last 90 min)
1. Dress rehearsal: power, program, 60 s wait, Altitude mode, climb, Position mode, hold, land. No laptop work in flight. Time every step.
2. Restart the program once on the ground (`tools/deploy.sh`); the log shows "running: yes".
3. **Freeze code.** `tools/deploy.sh pull`; copy `settings.json`, `logs/`, `good_*.json` to the USB stick. `python3 tools/use_profile.py diff settings.json good_<last>.json`: only the calibration differs.
4. If the demo is after dark: run `calib` in night light and the bench test again (DECISION_CARD row 15).
5. Finalize slides with real numbers; rehearse the 5 minutes twice with a timer.

## Bonus routes (only if the base hold is solid; Stage 2 sets the points)
- **Rolling-shutter improvement (measured):** replay one real recording with `readout_s` set to 0 versus the calibrated value, compare drift. One replay command, a number for the slides.
- **GNSS-free initialization:** run one flight with GPS disabled from boot and show the hold still works. Mostly a procedure and a log.
- **Demonstrated recovery:** cover the lens briefly (on the ground) or fly through a texture-poor patch and show the log recovering (fixes stop, then resume). `integ/` and E7 say 0.11-0.18 s recovery after a 3 s blackout in simulation.
- Betaflight: skip (no estimator output path).

## Do not do
- No new features after Block 4 begins unless a flight showed the need. Features cost less than they pay only before the day.
- Do not tune on the hold-out flights. Do not change several settings between flights.
- Do not claim real Justice D numbers you have not measured. Say "simulated, assumed camera" for everything before the event.
