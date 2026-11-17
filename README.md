# VIO position hold

Hold a drone in place with no GPS. A downward camera watches the ground. Our program turns the
picture motion, plus the tilt from the flight controller (FC), into a position. It sends that
position to the autopilot (ArduPilot or PX4) over MAVLink, and the autopilot holds it.

Target: the Contested Battlefield Aerial Operations challenge. The drone is most likely an Orqa MRM2-10AI.
The program runs on its APB (Auto Pilot Board, an i.MX8M Plus computer next to the FC) and reads
the Justice analog camera.

## Folders

| Path | What it is |
|---|---|
| `hold.py` | The estimator: picture + tilt + height in, position out |
| `apb_main.py` | The onboard program: camera, FC link, Hold, sends positions. Modes: `run`, `check`, `mask`, `calib`, `bench`, `replay`, `delay`, `timing` |
| `fc_link.py`, `capture.py`, `calib.py`, `settings.py` | FC link (MAVLink), camera input, camera calibration, settings |
| `settings.json`, `tools/profiles/` | Settings; ready profiles (indoor, outdoor day/night, goal A/B, ArduPilot, PX4) |
| `sim_ground.py`, `fetch_ground.py` | Open-loop simulator on real aerial photos (estimator only, no drone physics) |
| `px4sim/` | Closed loop: PX4 flies in a simulator, and its only horizontal position comes from us |
| `apsim/` | Closed loop with ArduCopter 4.6 (Orqa's branch) |
| `tests/`, `tests_apb/` | Tests and regression runs |
| `integ/` | Experiments and their results (the code comments cite these files) |
| `review/` | Review findings (IDs like E4, G2, H1 in the code) and the scripts behind them |
| `research/` | Notes: hardware, prior work, papers, simulators |
| `tools/` | Event-day tools: deploy, board check, log plot, replay tuning, profiles |
| `EVENT_DAY.txt`, `EIGHT_HOURS.txt`, `DECISION_CARD.txt` | Event-day checklist, 8-hour plan, "if X then change Y" card |
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

Latest result tables: `apsim/summary.txt`, `px4sim/results.txt`, `integ/summary.txt`.

## Working together

- Make a branch for each change and open a pull request into `main`.
- Simulator runs write to `runs/` and `logs/`. Git ignores them, so share a result by its summary
  file, not the raw log.
- One test clip (`clip-1`) is local only and not in git. The tests that need it skip.
