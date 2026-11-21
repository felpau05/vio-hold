# VIO position hold

Hold a drone in place with no GPS. A downward camera watches the ground. Our program turns the
picture motion, plus the tilt from the flight controller (FC), into a position. It sends that
position to the autopilot (ArduPilot or PX4) over MAVLink, and the autopilot holds it.

Target: the Contested Battlefield Aerial Operations challenge. The aircraft is the Orqa M2-AI (MRM2-10AI).
The program runs on its APB (Auto Pilot Board, an i.MX8M Plus computer next to the FC) and reads
the downward camera, the Orqa Justice D. The Justice D is digital (MIPI CSI, 1280 x 720, 60 fps, rolling shutter).
The forward camera is the analog Justice A. The code also reads an analog camera (setting `camera`).
The event pilots fly the aircraft. The pilot starts our position hold with a switch. Team members do not fly.
The scored hold uses no GNSS. The rules do not permit marks on the ground, added cameras or added sensors.
No part has run on the real board.

## Folders

| Path | What it is |
|---|---|
| `hold.py` | The estimator: picture + tilt + height in, position out |
| `apb_main.py` | The onboard program: camera, FC link, Hold, sends positions. Modes: `run`, `check`, `mask`, `calib`, `bench`, `replay`, `delay`, `timing` |
| `fc_link.py`, `capture.py`, `calib.py`, `settings.py` | FC link (MAVLink), camera input, camera calibration, settings |
| `settings.json`, `tools/profiles/` | Settings; ready profiles (indoor, outdoor day/night, goal A/B, ArduPilot, PX4, `justice_d`) |
| `tools/profiles/justice_d.json` | Overlay for the Justice D: `camera`, `video_size`, `video_fps`, `video_reduce`, `record_hz` and start values |
| `tools/lens_calib.py` | Lens calibration with a checkerboard, for the development area only |
| `integ/justice_d/` | Justice D simulation results: `summary.txt` and the scripts behind it |
| `sim_ground.py`, `fetch_ground.py` | Open-loop simulator on real aerial photos (estimator only, no drone physics) |
| `msp_link.py`, `betasim/` | Betaflight route: MSP link (rangefinder and optical-flow messages) and its Betaflight simulator tests. Proven in the simulator only |
| `EVIDENCE.md`, `CONTRIBUTION.md`, `PRESENTATION.md` | For the judges: proof by area A to G, our contribution and licences (area F), talk outline |
| `evidence/`, `talk/` | Copies of git-ignored proof files (same relative path), and the talk figures with `talk/make_figures.py` and `talk/captions.txt` |
| `px4sim/` | Closed loop: PX4 flies in a simulator, and its only horizontal position comes from us |
| `apsim/` | Closed loop with ArduCopter 4.6 (Orqa's branch) |
| `tests/`, `tests_apb/` | Tests and regression runs |
| `integ/` | Experiments and their results (the code comments cite these files) |
| `review/` | Review findings (IDs like E4, G2, H1 in the code) and the scripts behind them |
| `research/` | Notes: hardware, prior work, papers, simulators |
| `tools/` | Event-day tools: deploy, board check, log plot, replay tuning, profiles, lens calibration |
| `EVENT_DAY.txt`, `EIGHT_HOURS.txt`, `DECISION_CARD.txt` | Event-day checklist, 8-hour plan, "if X then change Y" card |
| `PILOT_SAFETY.txt`, `integ/pilot_safety/` | For the pilot: how to take over, what the aircraft does in each failure, and the bench demonstration with propellers off. The evidence and the parameter list for the crew |
| `ORGANIZER_QUESTIONS.md`, `HARDWARE_BRINGUP.md` | Questions for the first contact with the pilot and the organizers. The record sheet for the first session on the real hardware |
| `tools/fc_params.py` | Saves the aircraft's own flight controller parameters before our change, and puts them back |
| `tools/profiles/gnss_*.json`, `integ/gnss_ladder/` | GNSS levels for the day: none (default), before the hold only, fallback, velocity. Simulator results |
| `DATA_SPLITS.md` | Which data is for tuning and which is for testing. Read it before you tune a number |

## Setup (Linux)

The scripts expect the Python environment at `~/.venvs/vio`.

```sh
python3 -m venv ~/.venvs/vio
~/.venvs/vio/bin/pip install -r requirements.txt
~/.venvs/vio/bin/python fetch_ground.py ground     # aerial photos for the simulators (once)
```

Closed-loop simulators need Docker.

- PX4: `px4sim/sitl.py` pulls the `px4io/px4-sitl` image by itself.
- ArduPilot: clone and build Orqa's branch once, inside `apsim/`:

```sh
git clone -b mil-mavl https://github.com/orqafpv/ardupilot.git apsim/ardupilot
cd apsim/ardupilot && git submodule update --init --recursive
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v $PWD:/ardupilot -w /ardupilot \
  ardupilot/ardupilot-dev-base:latest sh -c './waf configure --board sitl && ./waf copter -j16'
```

## Run

```sh
sh tests_apb/run_all.sh                                        # all onboard-program tests
~/.venvs/vio/bin/python sim_ground.py --heights 50,100         # open loop on aerial photos
~/.venvs/vio/bin/python apsim/sitl_ap.py apb 100 w20g40        # ArduPilot closed loop, 100 m, wind 20 gusting 40
~/.venvs/vio/bin/python px4sim/sitl.py apb 100 w20g40          # PX4 closed loop
```

Latest result tables: `apsim/results.txt`, `px4sim/results.txt`, `integ/summary.txt`, `integ/justice_d/summary.txt`.
`apsim/summary.txt` is an old summary. Use `apsim/results.txt`.

## Working together

- Make a branch for each change and open a pull request into `main`.
- Simulator runs write to `runs/` and `logs/`. Git ignores them, so share a result by its summary
  file, not the raw log.
- One test clip (`clip-1`) is local only and not in git. The tests that need it skip.
