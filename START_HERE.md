# Start here: VIO position hold, Red Team Hack Grand Final

A camera looks down. The program measures how far the drone is from a reference picture of the ground, in metres, and sends that position to the flight controller, which holds it. No GPS.

**Nothing has run on the real board, camera or aircraft.** Every number is from a simulator with the real flight controller firmware, or from unit tests.

## 1. First things to do at the hackathon

1. Read Stage 2 together. Write the answers into `ORGANIZER_QUESTIONS.md`.
2. Ask the crew:
   - to turn off the vision arming check. With our settings the flight controller does not arm. ArduCopter 4.6: `ARMING_CHECK 1835006`. Never `ARMING_CHECK 0`.
   - if our program may write flight controller parameters (it turns the GPS off for the autopilot).
   - which mode each switch position gives, and for the failsafe values of `EVENT_DAY.txt` step 6C.
3. Book the pilot (Ted) twice: 10 minutes with propellers off for the takeover demonstration, then the first hover.
4. Apply the profiles: `python3 tools/use_profile.py apply outdoor_day goal_a ardupilot justice_d low_height`
5. On the board, before any parameter change: `python3 tools/fc_params.py save`
6. Show Ted `PILOT_SAFETY.txt` and do `EVENT_DAY.txt` step 6C with him. Ted decides if the aircraft flies.
7. First flight: Ted climbs in AltHold, holds still for 5 s, then selects Loiter. Hold 10 s, then back to AltHold.
8. After the scored trial: `python3 tools/fc_params.py restore`

The plan for the day is `EIGHT_HOURS.txt`. Each step with its pass rule is `EVENT_DAY.txt`. Fixes by symptom are `DECISION_CARD.txt`.
Fly this branch (`stage1-justice-d`). Do not merge `origin/spec-justice-d` today: the program then does not start.

## 2. How it works

```
  Justice D camera (looks down, rolling shutter)
        | frames
        v
  +------------- APB board: our program, apb_main.py ---------------+
  | capture.py   newest frame, grey, 640x360                        |
  | hold.py      offset from the reference picture, in metres       |
  | Pipe         10 good fixes first. In doubt: send nothing        |
  | fc_link.py   position over MAVLink, 10 to 12 times each second  |
  +-----------^---------------------------------+-------------------+
              | attitude, height, flight mode   | position
  +-----------+--- Flight controller (ArduCopter) v-----------------+
  | EKF fuses our position with its IMU  ->  Loiter holds it        |
  +-----------^-----------------------------------------------------+
              | sticks, mode switch (AltHold = manual, Loiter = our hold)
           Pilot (Ted)
```

- The position is measured against one picture, so the error does not add up over time.
- The flight controller does the fusion and keeps its failsafes. The program never arms, never changes the mode and never sends a flight target.
- A frozen, covered or inconsistent picture gives no fix. No position is safer than a wrong one.

## 3. How to use it

| Where | To do this | Type |
|---|---|---|
| Laptop | Choose settings / show them | `python3 tools/use_profile.py apply <profiles>` / `... show` |
| Laptop | Copy to the board, no start | `NO_RESTART=1 tools/deploy.sh` |
| Laptop | Copy and start for a flight | `tools/deploy.sh` |
| Laptop | Stop / status / get the logs | `tools/deploy.sh stop` / `status` / `pull` |
| Laptop | Look at a flight | `~/.venvs/vio/bin/python tools/plot_log.py logs/run_<stamp>.csv` |
| Board | Check Python and packages | `sh tools/board_check.sh` |
| Board | Check all / write our parameters | `python3 apb_main.py check` / `check --set-params` |
| Board | Mask, calibrate, ground test | `python3 apb_main.py mask` / `calib` / `bench --height 1.5` |

| Profile | Use it |
|---|---|
| `outdoor_day goal_a ardupilot justice_d` | Always |
| `low_height` | Holds at 5 to 10 m. No gain from 10 to 30 m. **Never above 30 m** |
| `gnss_setup` | Plan B: GPS for the take-off, our position alone before the switch |
| `gnss_fallback` | Practice flights only: GPS again if our picture fails for 3 s |
| `no_sd` | No SD card in the board. Name it last |

After each flight, look at the plot: 10 or more fixes each second, fix age under 150 ms, the reasons for unsent frames, and the notes ("engaged", "released", no "WARNING").

## 4. Test results (ArduCopter simulator, wind 20 gusting 40 km/h, the 30 s from the switch)

| Height | Old settings: typical / largest | With `low_height` |
|---|---|---|
| 5 m | 0.4 to 0.7 m / 1.5 m | 0.3 to 0.4 m / 0.8 m |
| 10 m | 0.6 to 0.9 m / 1.5 m | 0.25 to 0.5 m / 1.1 m |
| 20 to 30 m | 0.4 to 0.45 m / 0.8 m | no gain |
| 60 to 100 m | 0.4 to 0.7 m / 1.4 m | worse: do not use |

- No wind: about 0.1 m at 5 to 20 m, 0.4 to 0.5 m at 50 to 100 m.
- `low_height`: lower error in 16 of 17 pairs of runs at 5 and 10 m on three sites. At 10 m the old settings sent 7 of 10 fixes more than 0.5 m wrong, `low_height` none.
- GPS for the take-off (`gnss_setup`) holds as well as no GPS. A 20 m GPS error in the hold did not move the aircraft. The same error moved an aircraft that holds on GPS by 19.7 m.
- Our program stops: the flight controller changes to Land 7 to 8 s later, with the aircraft still within 2 m.
- A slow, wrong position gives no warning. The aircraft was 7 to 8 m away when the pilot took over. Only the pilot can stop this.
- The pilot's switch to AltHold took 0.01 s. AltHold does not hold the position.
- Test suite: 17 of 18 groups with no FAIL. The known FAIL: a picture with only overlay text still gets fixes.
- Not tested: the real hardware, PX4 for the new options, holds longer than 60 s with `low_height`.

More numbers, each with its file: `EVIDENCE.md`.

## 5. The repository

| Path | What it is |
|---|---|
| `apb_main.py`, `hold.py` | The onboard program. The estimator |
| `capture.py`, `calib.py`, `fc_link.py`, `settings.py` | Camera, calibration, flight controller link, settings |
| `tools/` | `deploy.sh`, `use_profile.py`, `plot_log.py`, `fc_params.py`, `board_check.sh`, `profiles/`, `bundle/` (our own Python for the board) |
| `EIGHT_HOURS.txt`, `EVENT_DAY.txt`, `DECISION_CARD.txt` | The day: plan, steps, fixes |
| `PILOT_SAFETY.txt` | One page for the pilot |
| `ORGANIZER_QUESTIONS.md`, `HARDWARE_BRINGUP.md` | First questions. The sheet for measured values |
| `PRESENTATION.md`, `talk/` | The 5-minute talk, 38 prepared questions, figures, checked sources |
| `EVIDENCE.md`, `CONTRIBUTION.md`, `DATA_SPLITS.md` | For the judges: proof by area, what is ours, tuning against test data |
| `apsim/`, `px4sim/`, `betasim/` | Simulators: ArduCopter, PX4, Betaflight |
| `tests_apb/`, `tests/` | Tests: `sh tests_apb/run_all.sh` |
| `integ/` | Records of each experiment. Newest: `integration_2026-11-21/`, `kfmap/` (low height), `gnss_ladder/`, `pilot_safety/` |
| `research/`, `review/` | Notes on hardware and papers. Review findings |

Not for today (prototypes, not tested): `fwd_cam.py`, the option `kf_map`, `integ/kfgraph/`, the Betaflight route.
