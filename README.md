# VIO position hold

Hold a drone in place with no GPS. A downward camera watches the ground. Our program turns the
picture motion, plus the tilt from an IMU, into a position.

Target: the Contested Battlefield Aerial Operations challenge (Grand Final), on the Orqa M2-AI. The program runs
on its APB (Auto Pilot Board, an i.MX8M Plus computer next to the FC) and reads the downward **Justice D**
(digital MIPI CSI, 1920x1080 at 60 fps on the event aircraft, strong fisheye).

**Stage 2 (2026-11-21):** the FC runs Betaflight 2025.12.5, commanded over MSP (`MSP_SET_RAW_RC` while CH10 is
on). The Stage 2 path uses only the APB's camera, IMU and barometer: `apb_live.py`. What is done and what is left:
`TODO.md`. Measured hardware facts: `HARDWARE_BRINGUP.md`. Rules: the Stage 1 package (rev 1.3) and the Stage 2
participant package; `SPEC.md` lists the Stage 1 rules. The older MAVLink path (ArduPilot / PX4,
`apb_main.py run`) and the simulators stay for reference.

## Folders

| Path | What it is |
|---|---|
| `SPEC.md` | The challenge rules (from the PDF), how the repo follows them, what is still open |
| `HARDWARE_BRINGUP.md` | First session on the APB and the Justice D: what to measure and write down |
| `EVIDENCE.md` | Evidence log, who-wrote-what and presentation outline for the judges |
| `ORGANIZER_QUESTIONS.md` | Open questions to send to the organizers |
| `hold.py` | The estimator: picture + tilt + height in, position out |
| `apb_live.py` | **Stage 2 live program on the APB**: camera + APB IMU + barometer -> position; web page on :8080; `--rec` records |
| `apb_est.py` | APB attitude filter, barometer height, replay of APB recordings (`apb_est.py replay rec/x_*.npz`) |
| `apb_sensors.py`, `v4l2_mmap.py`, `apb_rec.py` | APB IMU/barometer (IIO), Justice D capture (V4L2 mmap, 1920x1080 only), recorder |
| `tools/apb_ctl.sh` | On the board: `sh /data/vio/apb_ctl.sh start [--rec LABEL] / stop / status` |
| `tools/lens_calib.py` | Checkerboard fisheye calibration (run with OpenCV 4: `~/.venvs/vio4`) |
| `msp_link.py`, `betasim/` | MSP framing (to reuse for `MSP_SET_RAW_RC`); Betaflight SITL closed loop (from the stage1-justice-d branch) |
| `TODO.md` | Everything left before the presentation, by points |
| `apb_main.py` | The onboard program: camera, FC link, Hold, sends positions. Modes: `run`, `check`, `mask`, `calib`, `bench`, `replay`, `delay`, `timing` |
| `fc_link.py`, `capture.py`, `calib.py`, `settings.py` | FC link (MAVLink), camera input, camera calibration, settings |
| `gnss_rule.py` | The event GNSS rule as FC parameter values. `run` does not start when the FC breaks it |
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

Closed-loop simulators need Docker (Docker Desktop must be running). The simulators render the old analog camera by
default; add `camera=justice_d` (closed loop) or `VIO_SIM_CAMERA=justice_d` (open loop) for the simulated Justice D.
Its noise, readout and delay are assumed, not measured (see `SPEC.md`). Old result tables are analog.

- PX4: `px4sim/sitl.py` pulls the `px4io/px4-sitl` image by itself.
- ArduPilot: clone and build Orqa's branch once, inside `apsim/`:

```sh
git clone -b mil-mavl https://github.com/orqafpv/ardupilot.git apsim/ardupilot
cd apsim/ardupilot && git submodule update --init --recursive
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v $PWD:/ardupilot -w /ardupilot \
  ardupilot/ardupilot-dev-base:latest sh -c './waf configure --board sitl && ./waf copter -j16'
```

## Run

On the APB (copy the files to `/data/vio`; OpenCV is in `/data/vio/pylib`):

```sh
ssh root@192.168.75.1 sh /data/vio/apb_ctl.sh start             # live position, page http://192.168.75.1:8080
ssh root@192.168.75.1 sh /data/vio/apb_ctl.sh start --rec walk1 # same, and record for replay
ssh root@192.168.75.1 sh /data/vio/apb_ctl.sh stop
~/.venvs/vio/bin/python apb_est.py replay 'rec/walk1_*.npz'     # replay a recording on the laptop
```

On the laptop:

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
