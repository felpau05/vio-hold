# Event hardware research (2026-11-15)

Method: web search/fetch plus `gh` reads. Nothing found for "M2-AI" by that name; closest product is the Orqa MRM2-10 / MRM2-10AI (inference, low confidence).

## 1. Orqa M2-AI (probably MRM2-10 / MRM2-10AI)
- No source uses the name "M2-AI". Press coverage of the "MRM2-10AI" (Eurosatory, June 2026) is the nearest match. (inference)
- MRM2-10 official page (https://orqafpv.com/products/mrm2-10): carbon-fibre airframe, 10-inch dual-blade props, ORQA 2814 880 kV motors, 70 A 4-in-1 ESC (AM32), payload up to 3.5 kg, FC ORQA H7 QuadCore (STM32H743, ICM-42688), firmware support ArduPilot/iNav/Betaflight/PX4 + MAVLink, camera "ORQA Justice Analog" 1200 TVL with switchable 4:3/16:9, 5.8 GHz analog VTX up to 5 W. No weight, diagonal, battery, thrust or flight time on the page.
- Distributor (https://aalan.hr/uav-mrm2-10/271): frame weight 478 g, battery 22.2 V 7000 mAh 6S 95C 868 g (sold separately), about 29 min flight time without payload, motors 2814 880 kV.
- MRM2-10AI press release (https://www.prnewswire.com/in/news-releases/orqa-unveils-the-mrm2-10ai-at-eurosatory-a-tactical-drone-designed-to-dominate-in-the-age-of-electronic-warfare-302797302.html): AI-ready onboard compute, a new "Auto Pilot Board" supporting vision-based terminal guidance for GNSS-denied operation. No weight/flight time.
- Take-off weight, motor-to-motor diagonal, top thrust: NOT FOUND. A rough take-off weight can only be summed (frame 478 g + battery 868 g + motors/ESC/FC/APB/camera/VTX); not stated by any source.

## 2. Flight controller and firmware
- Orqa H7 QuadCore, STM32H743VIH6, 480 MHz. Orqa says PX4, ArduPilot, Betaflight, iNav are supported (MRM2-10 page). Which one the event drone runs is NOT FOUND.
- The DroneWuKong PX4 port targets PX4 v1.15.4 (https://github.com/DroneWuKong/Orqa_H7_PX4_Ports, build-config.json). Orqa's own fork has branches develop_APB-initial and develop_h743-3030-pro (https://github.com/orqafpv/PX4-Autopilot). Repo says hardware-unvalidated.
- A comment in the repo cites a field config `orqa_mrm2_10f_arducopter.param` (SERIAL4_BAUD=230) from a private repository, so ArduCopter was used on an MRM2-10F in the field. Second-hand; low confidence.

## 3. APB
- Name: Orqa "DTK APB". Orqa support page calls it a single board with a main controller and an onboard computer; H7-series STM32 FC; i.MX8M Plus companion (search summary of https://orqafpv.freshdesk.com/support/solutions/articles/48001274269-dtk-apb; page itself returned 404 to the fetch). The press release calls the new board "Auto Pilot Board" (so APB = Auto Pilot Board, medium confidence; no source spells it out for the DTK board).
- Port repo README (https://github.com/DroneWuKong/Orqa_H7_PX4_Ports): i.MX8M Plus = 4x Cortex-A53 + M7, 2.25 TOPS NPU, H.265 encode, dual MIPI-CSI. OS: Orqa "Scarthgap v1.0.5" Yocto SDK (Linux); microSD is Linux-owned at /sdcard (reference/orqa-apb/README.md).
- Link: FC UART4 (PC10/PC11, "IMX", TEL1, /dev/ttyS1) to i.MX UART3 /dev/ttymxc2, 230400 baud. mavlink-router v4 bridges to udp 127.0.0.1:14540 (rc.board_defaults). Repo says 230400 is "field-confirmed" and 921600 caused link loss after reflashes. FC side: MAV_0_CONFIG=101, MAV_0_MODE=2 (Onboard), MAV_0_RATE=0 (auto = baud/20 = 11520 B/s). MAVLink version: mavlink-router; Prismo scripts parse MAVLink v2 frames. TEL2 (USART6) 57600 for SiK/gimbal.
- Camera into Linux side: Prismo code comment: /dev/video0 = analog capture, /dev/video2 = MIPI downward-facing (apb/scripts/wingman_vio.py). Chip for analog capture not named.
- Serial map APB: USART3 RC, UART4 TEL1 (IMX), USART6 TEL2, UART7 GPS, UART8 ESC telemetry.

## 4. Justice D camera
- Official pages only say: "ORQA Justice (Analog)", 1200 TVL, 4:3/16:9 switchable (https://orqafpv.com/products/mrm2-10, https://orqafpv.com/products/mrm1-5). Sensor model, FOV, lens, shutter, field rate, PAL/NTSC: NOT FOUND. Rolling-shutter readout time: NOT FOUND, so no datasheet lookup was possible.
- Justice A data (earlier lens calibration): analog standard "unconfirmed"; capture chain Justice A -> analog VTX -> UVC receiver -> OBS (1280x720, cropped to 960x720 for 4:3), and Orqa goggles capture 1280x960. Default OSD model uses 132 deg (Caddx Ratel 2) and was a poor fit (15.5 px RMS).

## 5. Justice A vs D
Not documented anywhere reachable. No source mentions "Justice D". Open question stays open.

## 6. DroneWuKong repos
Orqa_H7_PX4_Ports:
- Targets: orqa_h7quadcore, h7wingcore, apb (FC side), f405pro. Board ID 1204 (README) vs 1013 (SESSION_CONTEXT, older).
- IMUs: SPI1 MPU6000 then ICM-42688-P, CS PA4, rotation R12; SPI4 ICM-42688-P CS PE11 R14 (APB: ICM-42605 or ICM-42688P on SPI4). Baro DPS310 I2C2 0x77; mag QMC5883 I2C1. IMU_GYRO_RATEMAX 2000. Actual IMU sample rates not stated.
- Defaults: EKF2_IMU_CTRL 7 (gyro, accel, gravity fusion), SYS_AUTOSTART 4001, BAT 6S, RC on USART3.
- No EKF2 vision parameters (EKF2_EV_*, EKF2_HGT_REF) found in the repo. Logging: attitude 10 Hz, local position 5 Hz via MAVLink ULog.
- No VIO setup here at all.
Prismo-Calibration (https://github.com/DroneWuKong/Prismo-Calibration; proprietary license):
- apb/scripts/wingman_vio.py: V4L2 capture at 320x180, 20 fps, FAST + Lucas-Kanade, default /dev/video2, assumed 70 deg HFOV, PX4 IMU over UDP 14541, output UDP 14560. Complementary filter (alpha 0.98). "orqa_csi" focal estimate 250 px. This is optical flow, not full VIO.
- apb/scripts/wingman_vio_native.py: adapter for headless OpenVINS (commit 69488123ed93...), needs an estimator YAML and manifest; RAW_IMU at 100 Hz minimum (min_imu_hz 100), MAVLink UDP 14566. It says no guessed intrinsics and no barometric scale. It is a candidate; the README does not claim a validated flight setup.
- No PX4 EKF2 vision parameters.

## 7. Justice A lens calibration (earlier measurements)
- Justice A, 4:3. Round 1: USB/OBS 4376 frames 1280x720, crop [160,0,1120,720] to 960x720. Round 2: Orqa goggles 356 frames 1280x960, no crop. Interlace state not stated.
- Model: equidistant fisheye, theta = atan2(r,z), R = 1 + k1 t^2 + k2 t^4; u = cx + fx*xd, v = cy + fy*yd.
- Goggles native 1280x960: fx 713.302, fy 742.971, cx 651.156, cy 486.606, k1 -0.035077, k2 -0.002254. RMS 0.985 px on 12 held-out views (same recording). 960x720: fx 534.977, fy 557.229, cx 488.242, cy 364.829.
- Round 1 (960x720): fx 525.198, fy 546.794, cx 480.735, cy 363.852, k1 -0.024775, k2 -0.005190. Ray coverage about 54 to 59 deg half-angle.
- OSD sampling assumes 52 us active width, 288 lines PAL / 240 NTSC.
- Fx != fy by about 4% suggests non-square sampling from the analog chain (inference).

## Sources
- https://orqafpv.com/products/mrm2-10 ; https://orqafpv.com/products/mrm1-5 ; https://orqafpv.com/enterprise
- https://aalan.hr/uav-mrm2-10/271
- https://www.prnewswire.com/in/news-releases/orqa-unveils-the-mrm2-10ai-at-eurosatory-a-tactical-drone-designed-to-dominate-in-the-age-of-electronic-warfare-302797302.html
- https://orqafpv.freshdesk.com/support/solutions/articles/48001274269-dtk-apb (search snippet only)
- https://github.com/DroneWuKong/Orqa_H7_PX4_Ports (README.md, SESSION_CONTEXT.md, boards/orqa/apb/init/rc.board_defaults, reference/orqa-apb/README.md, companion/apb/README.md)
- https://github.com/DroneWuKong/Prismo-Calibration (apb/scripts/wingman_vio.py, wingman_vio_native.py, apb/docs/*)
- https://github.com/orqafpv/PX4-Autopilot (branch list only)

## Stage 1 package and OEM manuals (2026-11-18)
Sources: the organizer's Stage 1 package (2026-11-17), the APB manual v1.7.2, the Justice D datasheet v1.0, the Justice A datasheet, the aircraft manual v1.0.

Stage 1 package (organizer, 2026-11-17)
- The aircraft is the Orqa M2-AI (MRM2-10AI). The solution runs on its APB. It uses the downward camera, the Justice D.
- The Justice D is digital: MIPI CSI, 1280 x 720, 60 fps, rolling shutter. The forward camera is the analog Justice A.
- The event pilots fly (takeoff, landing, safety). Team members do not fly. The pilot flies to the test position and holds the aircraft still. Then the pilot moves a switch that starts the team's hold. Teams give test requests to the pilots.
- The scored hold has no GNSS aiding. GNSS is permitted before the scored hold. A start with no GNSS is an optional bonus.
- No physical change to the aircraft. No added cameras or sensors. Development aids (fiducial markers, tripods, test stands) are permitted only in the engineering and development area. They are not permitted in the scored evaluation and not in the solution.
- The highest altitude with a qualifying stable hold wins the flight part.
- Stage 2 will give the firmware list, the board access, the sensor data details, the altitude levels, the engagement procedure and the points.

APB manual v1.7.2
- NXP i.MX8M Plus, four Cortex-A53 cores. STM32H743 flight controller. 4 GB RAM, 4 GB eMMC. Micro-SD up to 512 GB at /sdcard (pages 3, 11).
- CSI 1 is /dev/video2, set for Orqa cameras, fixed 1280x720@60. CSI 2 is /dev/video3, digital or analog through an ADV7282 chip (PAL 720x576, 50 fps). The device tree selects it: tool orqa_dt_switcher, then a restart (pages 12, 15).
- The link to the flight controller is UART3, /dev/ttymxc2, behind a switch. `gpioset -t0 -c gpiochip3 13=1` and `gpioset -t0 -c gpiochip3 0=0` select the flight controller route (page 14). The manual gives no baud rate and no protocol.
- Access: the USB-C port of the i.MX is a network device. The board address is 192.168.75.1. `ssh root@192.168.75.1` (page 10). The board clock is wrong after the first start until the board gets network time (page 10).
- Temperature: `cat /sys/class/thermal/thermal_zone0/temp` in millidegrees C. The fan holds about 62 C. The board stops at 95 C (page 11).
- An ICM42605 IMU and a DPS310 barometer connect to the Linux processor (page 5).
- The manual shows `gst-launch-1.0` on the board (page 16). It does not state if Python or OpenCV is on the board.

Justice D datasheet v1.0
- Sensor "Sony 1/2.8 inch CMOS", lens F2.0, 1280x720, 60 fps, MIPI CSI. Field of view 114 / 86 / 121.5 degrees (horizontal / vertical / diagonal). OSD: "Voltage, Name, Timer".
- It does not state the shutter type, the readout time or the latency.
- The Justice A datasheet gives the same sensor, lens and angles for a 4:3 picture. The three angles do not fit one lens model on a 16:9 picture, so the datasheet cannot give the lens. Calibrate the lens on the real camera.

Aircraft manual v1.0
- Transmitter switch TL1 (channel 9) selects the flight mode: up = Acro, middle = Position Hold, down = Return to Home (page 18).
- The stock aircraft has the Justice D at the front on a mount with 0 to 30 degrees of tilt, and no downward camera. The event aircraft is different.
- The manual refers to "Betaflight Configurator". It does not state the firmware.
- 10-inch propellers, "substantial vibrations" (page 22). Takeoff is rated for wind gusts under 46.3 km/h (datasheet page 2).

What the new facts confirm in the earlier sections
- Section 1: the M2-AI is the MRM2-10AI (the open question is closed by the organizer).
- Section 3: the i.MX8M Plus with four Cortex-A53 cores, /sdcard, and the link to /dev/ttymxc2 (UART3).
- Section 3: /dev/video2 is the MIPI camera input (CSI 1).
- Section 3 and section 6: the DPS310 barometer and an ICM42605 IMU (APB variant).
- Section 4: "ORQA Justice" has a 1/2.8 inch sensor. The lens needs a calibration on the real camera.

What the new facts contradict or change
- Section 4: the Justice D is digital (MIPI CSI), not analog. Its title, "ORQA Justice (Analog)", holds for the Justice A.
- Section 4: the field of view, the lens and the sensor are now known (114 / 86 / 121.5 degrees). The shutter type, readout time and latency stay NOT FOUND.
- Section 3: /dev/video0 as the analog capture is not confirmed. The manual names /dev/video2 (CSI 1) and /dev/video3 (CSI 2, analog or digital, ADV7282).
- Section 3: the 230400 baud of the link comes from a repo, not from the manual. The manual gives no baud rate. The two sources do not disagree, but the manual does not confirm the value.
- Section 5: the open question is closed. The Justice D is the digital downward camera and the Justice A is the analog forward camera.
- Section 2: the firmware stays NOT FOUND. The aircraft manual names "Betaflight Configurator". Stage 2 gives the firmware list.
- Sections 1 and 3: the stock aircraft has no downward camera. The event aircraft is different (aircraft manual).
