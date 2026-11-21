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
- A comment in the repo cites a field config `orqa_mrm2_10f_arducopter.param` (SERIAL4_BAUD=230) from a private repo (Ai-Project), so ArduCopter was used on an MRM2-10F in the field. Second-hand; low confidence.

## 3. APB
- Name: Orqa "DTK APB". Orqa support page calls it a single board with a main controller and an onboard computer; H7-series STM32 FC; i.MX8M Plus companion (search summary of https://orqafpv.freshdesk.com/support/solutions/articles/48001274269-dtk-apb; page itself returned 404 to my fetch). The press release calls the new board "Auto Pilot Board" (so APB = Auto Pilot Board, medium confidence; no source spells it out for the DTK board).
- Port repo README (https://github.com/DroneWuKong/Orqa_H7_PX4_Ports): i.MX8M Plus = 4x Cortex-A53 + M7, 2.25 TOPS NPU, H.265 encode, dual MIPI-CSI. OS: Orqa "Scarthgap v1.0.5" Yocto SDK (Linux); microSD is Linux-owned at /sdcard (reference/orqa-apb/README.md).
- Link: FC UART4 (PC10/PC11, "IMX", TEL1, /dev/ttyS1) to i.MX UART3 /dev/ttymxc2, 230400 baud. mavlink-router v4 bridges to udp 127.0.0.1:14540 (rc.board_defaults). Repo says 230400 is "field-confirmed" and 921600 caused link loss after reflashes. FC side: MAV_0_CONFIG=101, MAV_0_MODE=2 (Onboard), MAV_0_RATE=0 (auto = baud/20 = 11520 B/s). MAVLink version: mavlink-router; Prismo scripts parse MAVLink v2 frames. TEL2 (USART6) 57600 for SiK/gimbal.
- Camera into Linux side: Prismo code comment: /dev/video0 = analog capture, /dev/video2 = MIPI downward-facing (apb/scripts/wingman_vio.py). Chip for analog capture not named.
- Serial map APB: USART3 RC, UART4 TEL1 (IMX), USART6 TEL2, UART7 GPS, UART8 ESC telemetry.

## 0. UPDATE 2026-11-18: the Stage 1 spec (rev 1.3) overrides the guesses below
- Platform name is **M2-AI** (spec); "MRM2-10AI" is the press name. The earlier "inference, low confidence" match is confirmed in practice.
- APB (OEM manual): NXP i.MX8M Plus, 4x Cortex-A53, STM32H743 FC, 4 GB LPDDR4, 4 GB eMMC, 2D/3D graphics, NPU 2.25 TOPS, Orqa Yocto BSP, SSH documented. Preinstalled packages (Python, OpenCV, ROS 2) are NOT promised; Stage 2 gives the deployment environment.
- **Justice D = nadir camera**: digital **MIPI CSI**, 1280x720 at 60 fps, 16:9, FOV 114 deg H x 86 deg V x 121.5 deg D, used as a **rolling shutter**. **Justice A = forward-facing, analog**. The hold uses the Justice D only; extra cameras and sensors are forbidden.
- Sensors named by the spec: ICM42605 IMU and DPS310 barometer on the APB (the aircraft manual also lists GPS, magnetometer, baro). Rates, timestamp conventions, camera/IMU timing and any rangefinder: Stage 2. (Section 6's ICM-42688 is the DroneWuKong port's guess; the spec wins.)
- Firmware: ArduPilot, PX4, Betaflight, INAV "where available"; list in Stage 2. Betaflight is an optional bonus.
- Not yet known: sensor model and exposure, readout time, how Linux exposes the CSI sensor (the Prismo note in section 3 says /dev/video2), frame time stamps.

## 4. Justice D camera (OLD: written before the spec; the "Justice (Analog)" pages below describe the Justice A)
- Official pages only say: "ORQA Justice (Analog)", 1200 TVL, 4:3/16:9 switchable (https://orqafpv.com/products/mrm2-10, https://orqafpv.com/products/mrm1-5). Sensor model, FOV, lens, shutter, field rate, PAL/NTSC: NOT FOUND. Rolling-shutter readout time: NOT FOUND, so no datasheet lookup was possible.
- Justice A data (earlier lens calibration): analog standard "unconfirmed"; capture chain Justice A -> analog VTX -> UVC receiver -> OBS (1280x720, cropped to 960x720 for 4:3), and Orqa goggles capture 1280x960. Default OSD model uses 132 deg (Caddx Ratel 2) and was a poor fit (15.5 px RMS).

## 5. Justice A vs D
Answered by the Stage 1 spec (section 0 above): A = forward analog, D = nadir digital MIPI CSI. Section 7's lens
calibration is for the Justice A and does not apply to the Justice D.

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
