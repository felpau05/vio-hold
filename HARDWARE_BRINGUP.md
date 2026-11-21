# Bring-up record: first session on the M2-AI and the APB

Nothing in this repository has run on the real board, camera or flight controller. This sheet turns each unknown into a measured fact. Write every result in the last column, with the time and the log file. These lines are the evidence for judging areas A (sensor data) and C (onboard execution).

Where a number can be measured twice, measure it twice and write both values: a result that repeats is worth more than one good value.

The idea of this sheet and rows 2, 4, 10 and 13 come from Aydan's `HARDWARE_BRINGUP.md` (branch `origin/spec-justice-d`). The commands are the ones of this branch (`EVENT_DAY.txt`).

| # | What to record | How | Result (value, time, file) |
|---|---|---|---|
| 1 | Python, numpy and OpenCV versions on the board. Case A or case B. | `sh tools/board_check.sh` (EVENT_DAY step 0A) | |
| 2 | Which device is the Justice D. Its pixel formats, sizes and frame rates. | `v4l2-ctl --list-devices`, then `v4l2-ctl -d /dev/video2 --all` | |
| 3 | Frames per second and picture size that the program gets. | `python3 apb_main.py check` (step 2), the video lines | |
| 4 | Where the frame time stamps come from (driver or read time), and their jitter in ms. | the same `check`, the stamp line | |
| 5 | Flight controller firmware and version. The route and baud rate that answered. | `check`, the "firmware" line (step 1) | |
| 6 | The aircraft's own parameter values before our change. | `python3 tools/fc_params.py save` (step 1, item 2b) | |
| 7 | ATTITUDE rate in Hz, fix age in ms, time sync. | `check` (step 2) | |
| 8 | Gaps in the attitude stream: the count of "attitude stale" rows in a 60 s run on the ground. | start with `tools/deploy.sh`, wait 60 s, stop, pull, `tools/plot_log.py` | |
| 9 | Hold time for each frame on the board (median and 95th percentile, ms) and fixes per second. The same with the pilot video on. | `check`, the "Hold.process" line | |
| 10 | Which IMU and barometer the flight controller reports. Barometer noise at rest. Is a rangefinder fitted? | ground station, and question 5 of `ORGANIZER_QUESTIONS.md` | |
| 11 | Board temperature at rest and after a 3 minute run. | `cat /sys/class/thermal/thermal_zone0/temp` (millidegrees C) | |
| 12 | Calibration: video delay, readout time with its sign, measured lens, mount tilt. Two runs that agree within 3 ms. | `python3 apb_main.py calib`, twice (step 4) | |
| 13 | Vibration: gyro noise and picture noise with the motors on, propellers off and on. | the crew's procedure, then the run log | |
| 14 | Bench: 1.00 m carried forward reads how much. Tilt 15 degrees changes it by how much. A hand on the lens stops the fixes after how long. | `python3 apb_main.py bench --height 1.5` (step 6) | |
| 15 | The flight mode that each switch position gives. | step 6B | |
| 16 | The takeover demonstration for the pilot: each bench step B0 to B7 and its result. | EVENT_DAY step 6C (the page for the pilot is `PILOT_SAFETY.txt`) | |

When a value changes a setting, add a line to the tuning log in `DATA_SPLITS.md` (what, which data, who, when).
