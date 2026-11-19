# Hardware bring-up: first session on the M2-AI / APB

Nothing in this repo has run on the real APB or the real Justice D yet. This list turns the unknowns in
`SPEC.md` into measured facts. Write every result in the "Result" column; those lines are your evidence for areas A and C.
Follow the event crew's procedures: bench work and flights go through them (spec, Q6, Q7).

| # | Do | Pass / what to record | Result |
|---|---|---|---|
| 1 | SSH in (OEM manual). `sh tools/board_check.sh` | Python 3, numpy, OpenCV (with V4L2 and GStreamer), pymavlink present. If not: tools/wheels/README.txt. Record versions. | |
| 2 | `ls /dev/video*`, `v4l2-ctl --list-devices`, `v4l2-ctl -d /dev/videoN --all` | Which device is the Justice D (repo guess: /dev/video2). Record pixel formats, sizes, frame rates. | |
| 3 | `python3 apb_main.py check` | "Justice D delivers ~60 frames/s", picture 1280x720, stamps from "driver". If it cannot open: try a GStreamer pipeline and put it in `video_pipeline` (settings.json), e.g. `v4l2src device=/dev/video2 ! video/x-raw,width=1280,height=720,framerate=60/1 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink drop=true max-buffers=2` (an example: check the real caps with the commands in row 2). | |
| 4 | Record frames: `python3 apb_main.py run --raw` (hover or hand-carry) | Raw .npz files in logs/ for replay on the laptop. Keep them; they are the first real Justice D data (HOLD-OUT rules: DATA_SPLITS.md). | |
| 5 | Frame time stamps | Do stamps come from the driver ("driver") or from read time ("read")? Frame interval jitter (ms). | |
| 6 | `python3 apb_main.py calib` twice (EVENT_DAY step 4) | `video_delay_s` and `readout_s` (sign too), `focal_scale`, mount tilt. Two runs agree within 3 ms. | |
| 7 | Lens: real calibration of the Justice D (board only on a tripod in the development area, never on the aircraft) | Replace the nominal profile "Justice D nominal (spec FOV, uncalibrated)" in justice_lens_profiles.json. Record RMS reprojection error and whether the lens is really equidistant. | |
| 8 | `python3 apb_main.py timing` and `check` | Hold.process median and p95 (ms) at video_scale 0.5 and 1.0 on the A53. Fixes per second. CPU load of the other aircraft services at the same time (spec area C). Laptop numbers: SPEC.md. | |
| 9 | `python3 apb_main.py bench --height 1.5` (EVENT_DAY step 6) | Forward/right moves within 0.10 m. | |
| 10 | IMU and baro | Which IMU and baro the FC reports; attitude rate over MAVLink; baro noise at rest. Is there a rangefinder? (SPEC.md open item 4.) | |
| 11 | Firmware | Which firmware is installed, and which MAVLink port/rate the event gives (check shows it). | |
| 12 | Vibration | Gyro and frame noise with motors on, props off and on (crew's procedure). | |

When a value changes a setting, add a line to the tuning log in `DATA_SPLITS.md` (what, which data, by whom, when).
