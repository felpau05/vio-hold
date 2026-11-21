# Start here: VIO position hold for the Red Team Hack Grand Final

One page for the team. Written 2026-11-21, 07:20, before the event.

**What this is.** A program that holds a drone over one point with no GPS. A camera looks down. The program measures how far the drone is from a reference picture of the ground, in metres, and sends that position to the flight controller. The flight controller holds it.

**The state, honestly.** Nothing has run on the real board, camera or aircraft. Every number here comes from a simulator that runs the real flight controller firmware, from synthetic pictures, or from unit tests.

---

## 1. First things to do at the hackathon

Do these in this order. Details: `EIGHT_HOURS.txt` (the plan for the day) and `EVENT_DAY.txt` (each step with its pass rule).

1. **Read Stage 2 together.** Write each answer next to its question in `ORGANIZER_QUESTIONS.md`.
2. **Ask the crew three things at once.**
   - "Will you turn off the vision arming check?" With our settings the flight controller does not arm (4 of 4 tries in the simulator: "VisOdom: not healthy"). ArduCopter 4.6: `ARMING_CHECK 1835006`. ArduCopter 4.7 or later: `ARMING_SKIPCHK 262144` (not tested). Never `ARMING_CHECK 0`.
   - "May our program write flight controller parameters?" It turns the GPS off for the autopilot.
   - "Which mode does each switch position give, and what are the failsafe actions?" The list we ask for is in `EVENT_DAY.txt` step 6C, item 2.
3. **Book the pilot (Ted) for two times.** First 10 minutes with propellers off for the takeover demonstration. Then the first hover.
4. **Choose the profiles.** Low-height event, no GPS: `outdoor_day goal_a ardupilot justice_d low_height`.
5. **Save the aircraft's own parameters before we change them.** On the board: `python3 tools/fc_params.py save`.
6. **Show Ted the page `PILOT_SAFETY.txt`** and do the takeover demonstration (`EVENT_DAY.txt` step 6C). Ted decides if the aircraft flies.
7. **First flight.** Ted climbs in AltHold, holds still (5 s with `low_height`, 15 s without), then moves the switch to Loiter. Hold for 10 s, then back to AltHold.
8. **After the scored trial, give the aircraft back:** `python3 tools/fc_params.py restore`.

Fly this branch (`stage1-justice-d`). Do not merge the branch `origin/spec-justice-d` today: after a merge the program does not start.

---

## 2. How the solution works

```
  Justice D camera (looks down, 1280x720, 60 frames each second, rolling shutter)
        | frames
        v
  +---------------------- APB board (Linux): our program, apb_main.py ---------------------+
  | capture.py   keeps only the newest frame, grey picture, reduced to 640x360             |
  |      v                                                                                 |
  | hold.py      offset from the reference picture, in metres, with its uncertainty        |
  |              (each image row uses the attitude of its own exposure time)               |
  |      v                                                                                 |
  | Pipe         sends only after 10 good fixes. In doubt it sends nothing.                |
  |              It sends nothing below 3 m.                                               |
  |      v                                                                                 |
  | fc_link.py   VISION_POSITION_ESTIMATE over MAVLink, about 10 to 12 times each second   |
  |                                                                                        |
  | helpers:  GyroAtt    our attitude, from the flight controller's gyro rates             |
  |           Engage     reads the flight mode and logs the pilot's switch                 |
  |           GnssLevel  the optional GPS levels (off by default)                          |
  |           Raw, Log   recording of pictures, and the run log of each flight             |
  +---------------^------------------------------------------------+-----------------------+
                  | attitude 100 Hz, height, flight mode           | position
                  |                                                v
  +---------------------- Flight controller (ArduCopter, or PX4) --------------------------+
  | EKF: fuses our position with its IMU and barometer  ->  Loiter: holds that position    |
  +---------------^------------------------------------------------------------------------+
                  | sticks and mode switch (AltHold = manual, Loiter = our hold)
               Pilot (Ted)

  LAPTOP:  tools/deploy.sh       copy the program, start, stop, pull the logs
           tools/use_profile.py  choose the settings
           tools/plot_log.py     look at a flight
           apsim/, px4sim/       simulators with the real firmware       tests_apb/  tests
```

Four ideas carry the design:
- **Anchor to one picture.** The error does not add up over time while that picture is in view.
- **The flight controller does the fusion.** We send a position. Its tested EKF and failsafes stay in charge.
- **Refuse when in doubt.** A frozen, covered or inconsistent picture gives no fix. No position is safer than a wrong one.
- **The pilot's switch is the authority.** With the event settings the program never arms, never changes the flight mode and never sends a flight target.

---

## 3. What we have

| Piece | Where | State |
|---|---|---|
| Estimator (picture + attitude + height gives a position) | `hold.py` | Simulated at 5 to 100 m |
| Onboard program (`run`, `check`, `mask`, `calib`, `bench`, `timing`, `delay`, `replay`) | `apb_main.py` | Runs on a laptop and on emulated ARM. Not on the board yet |
| Camera input, calibration, flight controller link, settings | `capture.py`, `calib.py`, `fc_link.py`, `settings.py` | Unit tests and simulator |
| Low-height setting | profile `low_height` | New on 2026-11-21. Simulator only. For holds at 5 to 10 m |
| GPS levels | profiles `gnss_setup`, `gnss_fallback` | Simulator only |
| Save and restore of the aircraft's parameters | `tools/fc_params.py` | Passed against a simulated flight controller |
| Page for the pilot and the takeover demonstration | `PILOT_SAFETY.txt`, `EVENT_DAY.txt` step 6C | Simulator and source reading |
| Offline bundle for the board (our own Python) | `tools/bundle/` | Built 2026-11-21 07:06. A copy from 02:08 is the fallback |
| Betaflight route | `msp_link.py`, `betasim/` | Betaflight simulator only, up to 10 m. Off |
| Talk: 5 minutes and 38 prepared questions | `PRESENTATION.md`, slides at https://claude.ai/artifact/Pecdjb7iqTNMe6V1H2Abt7 | Fill the `[TODAY]` slots |
| Forward-camera reference, keyframe map | `fwd_cam.py`, `hold.py` option `kf_map` | Prototypes. Not tested. Do not use today |

---

## 4. How to use it

**On the laptop** (in this folder):

| To do this | Type |
|---|---|
| Choose the settings | `python3 tools/use_profile.py apply outdoor_day goal_a ardupilot justice_d low_height` |
| See which settings are active | `python3 tools/use_profile.py show` |
| Copy the program to the board, no start | `NO_RESTART=1 tools/deploy.sh` |
| Copy and start the program for a flight | `tools/deploy.sh` |
| Stop it, or see if it runs | `tools/deploy.sh stop` , `tools/deploy.sh status` |
| Get the logs and the board's settings | `tools/deploy.sh pull` |
| Look at a flight | `~/.venvs/vio/bin/python tools/plot_log.py logs/run_<stamp>.csv` |
| Board with no usable Python | `tools/deploy.sh bundle` |

**On the board** (ssh root@192.168.75.1, in the program folder):

| To do this | Type |
|---|---|
| Check Python and the packages | `sh tools/board_check.sh` |
| Save, and later restore, the aircraft's parameters | `python3 tools/fc_params.py save` , `python3 tools/fc_params.py restore` |
| Check everything, and write our parameters | `python3 apb_main.py check` , `python3 apb_main.py check --set-params` |
| Mask the overlay text, then calibrate | `python3 apb_main.py mask` , `python3 apb_main.py calib` |
| Ground test, propellers off | `python3 apb_main.py bench --height 1.5` |

**Which profile when:**

| Profile | Use it |
|---|---|
| `outdoor_day`, `goal_a`, `ardupilot`, `justice_d` | Always (the base for the event) |
| `low_height` | Holds at 5 to 10 m. No gain from 10 to 30 m. **Never above 30 m** |
| `gnss_setup` | Only as plan B: GPS for the take-off, our position alone before the switch |
| `gnss_fallback` | Practice flights only, if the pilot wants GPS as a safety net |
| `no_sd` | A board with no SD card. Name it last |
| `att_gap` | Only when the run log shows many "attitude stale" rows |

**After each flight, look at four things** in the plot of the run log:
1. Fixes each second: 10 or more through the hold.
2. Fix age: under 150 ms.
3. The reasons for frames that were not sent. Many "attitude stale" rows: `DECISION_CARD.txt` row 44. Many "halves disagree" rows: row 47.
4. The notes: "engaged" and "released" at the right times, and no "WARNING" or "GNSS FALLBACK".

When something goes wrong, find what you see in `DECISION_CARD.txt`. It gives the likely cause and the one change to make.

---

## 5. Test results

All from the ArduCopter simulator unless a line says otherwise. Wind 20 gusting 40 km/h. Distance from the hold point over the 30 s from the switch.

**Hold error by height**

| Height | Old settings: typical / largest | With `low_height`: typical / largest |
|---|---|---|
| 5 m | 0.4 to 0.7 m / up to 1.5 m | 0.3 to 0.4 m / up to 0.8 m |
| 10 m | 0.6 to 0.9 m / up to 1.5 m | 0.25 to 0.5 m / up to 1.1 m |
| 20 to 30 m | 0.4 to 0.45 m / up to 0.8 m | no gain |
| 60 m | 0.4 to 0.5 m / up to 0.9 m | worse in 2 of 3 runs: do not use |
| 100 m (60 s holds) | 0.45 to 0.7 m / up to 1.4 m | do not use |

- With no wind: about 0.1 m at 5 to 20 m, and 0.4 to 0.5 m at 50 to 100 m.
- A perfect position gives 0.34 to 0.47 m at 5 to 10 m in this wind. So part of the error is the autopilot's answer to gusts.

**The low-height setting (60 simulator runs, 2026-11-21)**
- Lower error in 16 of 17 pairs of runs at 5 and 10 m, on three sites.
- At 10 m about half: 0.79, 0.71, 0.62 m became 0.42, 0.33, 0.31 m. On a site that it was not tuned on: 0.74, 0.65, 0.94 m became 0.34, 0.27, 0.50 m.
- At 10 m the old settings sent about 7 of 10 fixes more than 0.5 m wrong. With `low_height`: none, in 21 runs.
- The cause that it removes: one degree of heading error moved the fix by 0.26 m at 10 m.
- The pilot's hover alone (15 s in place of 4 s, old settings): 0.79 and 0.71 m became 0.51 and 0.41 m.

**GPS levels**
- `gnss_setup`: the flight controller accepted "our position only" 1.5 to 8.7 s before the switch in all 9 runs. A 20 m GPS error in the hold did not move the aircraft (0.98 m against 0.81 m with no error). The same error moved an aircraft that holds on GPS by 19.7 m.
- `gnss_fallback`: after 3 s with no fix the flight controller held on GPS. Largest distance 1.52 m. One run.
- GPS levels together with `low_height` (7 runs, 2026-11-21 07:20): `gnss_setup` holds as well as no GPS. 10 m: 0.42 and 0.38 m against 0.44 and 0.41 m with no GPS (same seeds). 5 m: 0.42 and 0.40 m. The flight controller accepted "our position only" 3.5 to 5.6 s before the switch, with a position step of 0.02 to 0.11 m. `gnss_fallback` with a 6 s video loss: GPS came back 3 s into the loss, and the largest distance was 1.55 m (1 run).

**When things fail (the pilot's page has these)**
- Our program stops in a hold: the flight controller changes to Land 7 to 8 s later. The aircraft was still within 2 m.
- A slow, wrong position: no warning. The aircraft was 7.1 and 8.4 m away when the pilot took over. Only the pilot can stop this.
- The pilot's switch to AltHold took 0.01 s. AltHold does not hold the position: with centred sticks the aircraft drifted 36 m in 10 s.
- A 3 s video loss at 60 m: the first fix came 1.1 s after the video came back.

**Other measurements**
- Processing time: about 3 ms for each frame on one laptop core. Not measured on the board.
- Calibration (synthetic): delay 58 ms found, 58 ms true. Readout 11 ms found, 10 ms true.
- The gain of the rolling-shutter model: not resolved. The difference is smaller than the spread between runs.

**Test suite on the final code:** 17 of 18 groups with no FAIL (`sh tests_apb/run_all.sh`). The one FAIL is known: a picture with only overlay text still gets 21 fixes in 288 frames.

**Not tested:** anything on the real hardware, PX4 for the new options, holds longer than 60 s with `low_height`, more than one wind case for the new options, Acro and Return to Home in the simulator.

---

## 6. The repository at a glance

**To fly**

| Path | What it is |
|---|---|
| `apb_main.py` | The onboard program |
| `hold.py` | The estimator |
| `capture.py`, `calib.py`, `fc_link.py` | Camera input, calibration, flight controller link |
| `settings.py`, `settings.json` | Every setting with its reason. The active values |
| `tools/profiles/` | Ready sets of settings |
| `tools/deploy.sh`, `tools/board_check.sh`, `tools/fc_params.py` | Copy and start. Check the board. Save and restore parameters |
| `tools/bundle/` | Our own Python for the board, if the board has none that we can use |

**For the day**

| Path | What it is |
|---|---|
| `START_HERE.md` | This page: the overview for the team |
| `EIGHT_HOURS.txt` | The plan, one block for each hour, each with a stop rule |
| `EVENT_DAY.txt` | Each step with its commands and its pass rule |
| `DECISION_CARD.txt` | "If you see X, the cause is Y, change Z". 47 rows |
| `PILOT_SAFETY.txt` | One page for the pilot |
| `ORGANIZER_QUESTIONS.md`, `HARDWARE_BRINGUP.md` | Questions for the first contact. The sheet for each measured value |
| `tools/plot_log.py`, `tools/log_summary.py`, `tools/use_profile.py` | Look at a flight. Choose settings |

**For the judges**

| Path | What it is |
|---|---|
| `PRESENTATION.md`, `talk/` | The talk, the question bank, the figures, the checked sources |
| `EVIDENCE.md` | The proof for each judging area A to G, with the file behind each number |
| `CONTRIBUTION.md` | What is ours, what is third-party, and the decisions we made |
| `DATA_SPLITS.md` | Which data is for tuning and which is for testing |

**Simulators and tests**

| Path | What it is |
|---|---|
| `apsim/` | ArduCopter in the loop (Orqa's branch). Result tables `*_results.txt` |
| `px4sim/` | PX4 in the loop |
| `betasim/` | Betaflight simulator for the MSP route |
| `sim_ground.py`, `sim_test.py` | Open-loop simulator on aerial photos (estimator only) |
| `tests_apb/`, `tests/` | The test suite (`sh tests_apb/run_all.sh`) and the regression |

**Records of what we tried** (the code comments cite these)

| Path | What it is |
|---|---|
| `integ/integration_2026-11-21/` | The record of the last night |
| `integ/kfmap/`, `integ/diag10/` | The low-height work: the A/B table and the diagnosis |
| `integ/gnss_ladder/`, `integ/pilot_safety/` | GPS levels. The pilot's safety case |
| `integ/justice_d/`, `integ/lowh/`, `integ/gap/` | Justice D results. Low-height gaps. The gap before the switch |
| `integ/rehearsal_*/`, `integ/integration_2026-11-20/` | Rehearsals of the event day in the simulator |
| `integ/fwdcam/`, `integ/kfgraph/` | Prototypes: forward camera, pose graph. Not for today |
| `research/`, `review/` | Notes on hardware and papers. Review findings (IDs such as E4, H9 in the code) |
