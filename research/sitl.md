# PX4 SITL for a closed-loop test of the position estimator

Date of research: 2026-11-15. Local PX4 clone: `main`, `v1.18.0-beta1-1575-gc4b8f107ae`.

Tags used for each item: **[docs]** from official docs, **[src]** from source code, **[forum]** from a forum or issue,
**[tested]** run on this laptop on 2026-11-15 (Docker 29, no GPU, `--network host`, pymavlink 2.4.50 in a scratch venv).
Anything marked "inference" is my reasoning, not a source statement.

## 0. Answer in short

1. Use **SIH** (item a). It is the smallest and the only one with a documented wind parameter. Run it from the ready image `px4io/px4-sitl` (53 MB download, 6 MB of RAM while running). **[tested]**
2. SIH gives true pose, real PX4 EKF2 and controllers, simulated IMU, baro and mag noise, GNSS you can turn off, and wind you can change in flight with PARAM_SET. **[tested]**
3. The wind is only a velocity in the first-order drag term. It has no gust model, no turbulence, no vertical wind and no wind torque. You build gusts in your Python script by changing `SIH_WIND_N` and `SIH_WIND_E` over time. **[src][tested]**
4. Gazebo Harmonic runs headless in Docker with no GPU (150 MB RAM, about 0.6 CPU core). **[tested]** But its truth stream through PX4 is only about 50 Hz, and the wind changes only by editing the world SDF. A runtime wind topic had no visible effect in my test. **[tested]**
5. jMAVSim is gone from PX4 `main` (removed 2026-09-26). Gazebo Classic is community-supported and Ubuntu 22.04 only. Neither is a good choice.
6. Control from the host with **pymavlink**, not MAVSDK-Python. On PyPI the name `mavsdk` now means a different package (v4 native binding). The gRPC package is `mavsdk-grpc`. You need raw MAVLink anyway (ground truth, vision messages). **[docs][tested]**
7. Ground truth at 200 Hz or more: `HIL_STATE_QUATERNION` through `MAV_CMD_SET_MESSAGE_INTERVAL`. At the default 250 Hz sim step I measured 185 to 200 messages per second (about 25 percent of steps are skipped). At `IMU_INTEG_RATE=400` I measured about 297 per sim second. **[tested]**

## 1. Comparison for this job

| | (a) SIH | (b) Gazebo Harmonic `gz sim`, x500 | (c) Gazebo Classic | (d) jMAVSim |
|---|---|---|---|---|
| Install size | Image `px4io/px4-sitl`: 51 MB compressed, 53 MB content, 180 MB on disk. `.deb` about 10 MB. [docs][tested] | Image `px4io/px4-sitl-gazebo`: 667 to 688 MB compressed, 730 MB content, 3.1 GB on disk. `.deb` about 30 MB plus Gazebo packages. [docs][tested] | Gazebo 11 apt packages (`gazebo libgazebo11 libgazebo-dev`), old `px4-dev-simulation-jammy` image is 1.35 GB compressed (2024). [docs] | Java plus a jar. [docs] |
| Headless in Docker, no GPU | Yes. It has no renderer at all. [docs][tested] | Yes. `HEADLESS=1` starts `gz sim -r -s` (server only). Started in about 30 s, no EGL error, 153 MB RAM. [src][tested] | `HEADLESS=1` supported. [docs] Not tested. | `HEADLESS=1` supported in v1.17 docs. [docs] Not tested. |
| Wind and gusts | `SIH_WIND_N`, `SIH_WIND_E` (m/s), changeable in flight. No gust model. [docs][src][tested] | World SDF `<wind>` plus `WindEffects` system. `windy` world works on x500. Runtime topic did not work in my test. [src][tested][forum] | `libgazebo_wind_plugin.so` in the world file. Has gust start, duration, mean, variance. [docs] | Keyboard keys in the GUI only (README). Not usable headless. [docs] |
| Ground truth | uORB `*_groundtruth`, MAVLink `HIL_STATE_QUATERNION`. [src][tested] | gz topic `/world/<w>/pose/info` (about 50 to 60 Hz), same `HIL_STATE_QUATERNION` path. [src][tested] | Same MAVLink path. Simulator feeds `HIL_STATE_QUATERNION` into PX4. [src] | Same idea. [src] |
| Change mass, size, thrust, drag | `SIH_*` parameters, set at runtime or with `PX4_PARAM_*` env vars. [src][tested] | Edit model SDF (`x500_base/model.sdf`, motor plugin constants), restart. [src] | Edit model SDF. | Edit Java source or parameters. |
| Known problems | Only quad is "stable". Linear thrust and drag models. Sensor noise is small (see section 2). No wind turbulence. [docs][src] | Pose truth is slow. Wind effect on x500 not documented. Forum users report "no wind". Velocity and angular rate in truth are finite differences. [forum][src] | Downgraded to community support, not maintained by core team. Ubuntu 22.04 only. [docs] | Removed from PX4 `main` by commit `a23449cbe` on 2026-09-26. Still in v1.17. [src] |

Conclusion: SIH for the estimator test. Keep Gazebo only if you need a rendered camera, which you do not.

## 2. SIH parameters

All in `src/modules/simulation/simulator_sih/sih_params.yaml` **[src]**. Defaults and units as in that file.

| Need | Parameter | Default | Unit | Note |
|---|---|---|---|---|
| Mass | `SIH_MASS` | 1.0 | kg | |
| Inertia | `SIH_IXX`, `SIH_IYY`, `SIH_IZZ` | 0.025, 0.025, 0.03 | kg m^2 | Cross terms `SIH_IXY`, `SIH_IXZ`, `SIH_IYZ` default 0 |
| Arm length | `SIH_L_ROLL`, `SIH_L_PITCH` | 0.2, 0.2 | m | Roll arm and pitch arm |
| Max thrust per propeller | `SIH_T_MAX` | 5.0 | N | Doc text: "about 5 times the mass of the quadrotor" |
| Max propeller torque | `SIH_Q_MAX` | 0.1 | Nm | Yaw authority |
| Linear drag | `SIH_KDV` | 1.0 | N/(m/s) | Force `D = -KDV * V`. Doc: free-fall speed `V = 10*MASS/KDV` |
| Angular damping | `SIH_KDW` | 0.025 | Nm/(rad/s) | |
| Thrust time constant | `SIH_T_TAU` | 0.05 | s | |
| Wind (from north, from east) | `SIH_WIND_N`, `SIH_WIND_E` | 0.0, 0.0 | m/s | Added in v1.18 |
| Start position | `SIH_LOC_LAT0`, `SIH_LOC_LON0`, `SIH_LOC_H0`, `SIH_LOC_YAW0` | 47.397742, 8.545594, 489.4, 0 | deg, deg, m, rad | Env vars `PX4_HOME_LAT`, `PX4_HOME_LON`, `PX4_HOME_ALT`, `PX4_HOME_YAW` set them [src] |
| Distance sensor | `SIH_DISTSNSR_EN`, `_MIN`, `_MAX`, `_NOI`, `_OVR` | 1, 0, 100, 0.01, -1 | m | |

Force model for the quad **[src]** (`sih.cpp`, `generate_force_and_torques`):
- Thrust is `-SIH_T_MAX * (u0+u1+u2+u3)` on body Z (linear in the motor signals).
- Moments are `L_ROLL*T_MAX*(...)`, `L_PITCH*T_MAX*(...)`, `Q_MAX*(...)`.
- Drag force is `-KDV * R * v_apparent`, where `v_apparent = v + v_wind`. Torque damping is `-KDW * w_B`.

Match the controller to the vehicle:
- The docs say the `SIH_*` and `CA_*` parameters must describe the same vehicle. **[docs]**
- With a linear thrust model, hover motor signal is `mass*9.81/(4*SIH_T_MAX)`. With the defaults this is 0.49. Set `MPC_THR_HOVER` to this value. This is my derivation from the source, not a documented rule.

Wind **[src][tested]**:
- Version: added to SIH in **v1.18** (PR #26467, merged 2026-03-05, release note "Add option to set wind velocity"). v1.17.0 has no `SIH_WIND_*` parameters (checked in the v1.17.0 tag). The v1.18.0-beta1 and v1.18.0-rc1 tags have them. The local clone has them.
- Sign: "wind from north". The code sets `v_apparent = v_ground + v_wind`, so `SIH_WIND_N=5` pushes the drone to the south. Measured: with Position mode and truth fed back, a step to `SIH_WIND_N=5` gave a peak excursion of -1.88 m (N), then recovery to about 0 within 12 s. Setting it back to 0 gave an excursion of +2.37 m.
- Wind can change during flight. `Sih::parameters_updated()` re-reads `SIH_WIND_N` and `SIH_WIND_E` whenever a `parameter_update` arrives. The docs say the same: "The parameters can also be changed during flight to simulate changing wind". I changed them with MAVLink `PARAM_SET` during flight.
- Wind only enters the drag force. There is no vertical wind, no turbulence model, no wind torque. Gusts are what your script makes them (for example a sum of steps or a filtered random process written to `SIH_WIND_N/E` at 5 to 20 Hz).
- Force at hover for a given wind is `KDV * wind` (first-order). With default KDV=1.0 and mass 1 kg, 5 m/s wind equals 5 N, about 27 degrees of tilt. Choose `SIH_KDV` to match the drag you want.
- Alternative: `px4-param set SIH_WIND_E 2` inside the container also worked. **[tested]**

Sensor noise in SIH-as-SITL **[src]**:
- IMU (in `sih.cpp`): Gaussian, accel 0.1 m/s^2 per axis, gyro 0.01 rad/s per axis. (The higher noise branch is disabled with `if (false && ...)` in this version.) Earth rate is added to the gyro.
- Barometer (`sensor_baro_sim`): ISA pressure from the true altitude, plus Gaussian noise of 1 Pa RMS (about 0.08 m), plus drift (`_baro_drift_pa_per_sec` is 0.0 by default). Offsets `SIM_BARO_OFF_P`, `SIM_BARO_OFF_T`.
- Magnetometer (`sensor_mag_sim`): Gaussian noise 0.02, 0.02, 0.03 on the field vector, offsets `SIM_MAG_OFFSET_X/Y/Z`.
- GNSS (`sensor_gps_sim`, 8 Hz): Markov position noise (amplitude 0.8 m) and velocity noise. Not needed when GNSS is off.
- These sensor modules read the true state (`vehicle_*_groundtruth`) and add noise. They start only if `SENS_EN_BAROSIM`, `SENS_EN_MAGSIM`, `SENS_EN_GPSSIM` are 1 (set by the airframe `10040_sihsim_quadx`). **[src]**
- The barometer noise is much smaller than a real drone's barometer under propeller wash. For a fair test of the height input, add extra noise yourself or lower `EKF2_BARO_NOISE` trust. (Inference.)

## 3. Gazebo Harmonic: wind and headless

Headless **[docs][src][tested]**:
- `HEADLESS=1` skips the GUI. `px4-rc.gzsim` always starts `gz sim --verbose=1 -r -s <world>` (server only); the GUI `gz sim -g` is started only if `HEADLESS` is unset.
- The image `px4io/px4-sitl-gazebo` ran with `HEADLESS=1` on this laptop with no GPU and no display: Gazebo 8.15.0, started in about 30 s, 153 MB RAM, 60 to 65 percent of one core, real-time factor 1.00 (sim time 5.98 s in 5.98 s wall).
- `PX4_GZ_SIM_RENDER_ENGINE=ogre` exists for VM rendering problems. Not needed here. The x500 has no camera.

Wind in the world SDF **[src]**:
- The `windy` world (`PX4-gazebo-models/worlds/windy.sdf`) contains `<wind><linear_velocity>5 2 0</linear_velocity></wind>` (ENU, x = east, y = north). Select it with `PX4_GZ_WORLD=windy` or `make px4_sitl gz_x500_windy`.
- PX4 loads `gz-sim-wind-effects-system` and `gz-sim-apply-link-wrench-system` in `gz_bridge/server.config`.
- `WindEffects` applies a force `mass * k * (wind_velocity - link_velocity)` to each link that has wind enabled. In my test x500 was affected, although the x500 SDF has no `enable_wind` text (so the default seems to be "on" for these links; I did not find out why).
- Mean and gust controls of `WindEffects` (from the system header in gz-sim8): `<horizontal><magnitude><time_for_rise>`, `<sin><amplitude_percent>`, `<sin><period>`, `<noise>`; `<horizontal><direction>...`; `<vertical><time_for_rise>`, `<vertical><noise>`; `<force_approximation_scaling_factor>`. Put these inside a `WindEffects` plugin element in the world SDF. PX4's `server.config` loads the system with no children, so you must either add the plugin element in your own world file or edit `server.config`. I did not test the gust elements.

What I tested **[tested]**:
- Control run, default world, Altitude mode, no wind: after 12 s the drone had moved 0.1 m.
- Windy world, same script: the drone drifted to N=8.5 m, E=21.8 m in 14 s (ratio 2.56, wind ratio is 5/2), then to about 5 m/s speed. Position mode with truth-fed vision held the drone still with 1.5 deg roll and 3.4 deg pitch.
- Runtime wind topic: `gz topic -t /world/windy/wind -m gz.msgs.Wind -p 'linear_velocity:{x:0,y:10,z:0} enable_wind:true'` gave no visible change in 20 s. The forum reports the same ("publishing to wind_info changed the topic values but the vehicle did not respond") [forum]. I did not find the cause.
- Runtime force instead: `gz topic -t /world/default/wrench/persistent -m gz.msgs.EntityWrench -p 'entity: {name: "x500_0", type: MODEL}, wrench: {force: {x: 3, y: 0, z: 0}}'` moved the drone east at once (velocity 2.4 m/s after 3 s, 7.7 m/s after 9 s in Altitude mode). The `ApplyLinkWrench` system is in PX4's `server.config`. My `.../wrench/clear` message did not stop it (wrong syntax, not investigated). Sending another persistent wrench with zero force may replace it (untested).

Other: forum threads say wind support for Harmonic multicopters is incomplete ("new Gz sim does not support simulating wind", answered 2025-12-03; PR #26018 merged 2025-12-01 added an airspeed plugin with wind effects for planes). [forum]

## 4. Turn GNSS off, arm, take off

GNSS off, any simulator **[src][docs]**:
- `SYS_HAS_GPS=0` (reboot required): the sensors module does not start the GPS pipeline at all. Airframe `4021_gz_x500_flow` uses `SYS_HAS_GPS 0`, `SIM_GPS_USED 0`, `EKF2_GPS_CTRL 0` together ("Disabling Sim GPS").
- `EKF2_GPS_CTRL=0` (bitmask: 1 lon/lat, 2 altitude, 4 velocity, 8 dual antenna; default 7).
- `SIM_GPS_USED=0`: `sensor_gps_sim` then reports `fix_type = 0`. With 4 or more it reports a 3D fix.
- SIH only: `SENS_EN_GPSSIM=0` (reboot required) stops the simulated GPS driver. I read this in `px4-rc.sihsim`, I did not test it.
- Gazebo: the GNSS comes from the `navsat` sensor in the model. `SYS_HAS_GPS=0` hides it from PX4.
- Height reference: `EKF2_HGT_REF=0` (barometer). Default is 1 (GPS).
- `COM_ARM_WO_GPS`: default 1 (allow arming without GNSS with a warning, for modes that need position). `SYS_HAS_GPS=0` removes the check completely.
- Vision input: `EKF2_EV_CTRL` default is 0. Bits: 1 horizontal position, 2 vertical position, 4 3D velocity, 8 yaw. Noise `EKF2_EVP_NOISE` default 0.1 m. `EKF2_EV_DELAY` default 0 ms. `EKF2_EV_NOISE_MD` default 0 (use the variance in the message, parameter as lower bound).
- A ready example of GNSS-off SIH: airframe `10046_sihsim_quadx_flow` (`param bitclear EKF2_SENS_EN 3`, `EKF2_HGT_REF 0`). In v1.18 there is a new `EKF2_SENS_EN` bitmask that replaces `EKF2_EN_BOOT` [docs: release notes 1.18]. Use `EKF2_GPS_CTRL=0` and `SYS_HAS_GPS=0` to stay portable across versions.

What each mode needs (`mode_requirements.cpp`) **[src]**:
- Stabilized: attitude and manual control only. No altitude, no position.
- Altitude: plus local altitude (barometer is enough).
- Position: plus `local_position_relaxed` (needs horizontal aiding, here the vision input).
- Offboard: attitude, angular velocity and an offboard signal only. No position needed.
- Takeoff (auto), Hold/Loiter, Mission: need local and global position for multicopters. Do not use them without GNSS.

Measured sequence in SIH with GNSS off **[tested]** (`PX4_PARAM_SYS_HAS_GPS=0 PX4_PARAM_SIM_GPS_USED=0 PX4_PARAM_EKF2_GPS_CTRL=0 PX4_PARAM_EKF2_HGT_REF=0`):
1. Send `MANUAL_CONTROL` at 25 Hz all the time (SITL default `COM_RC_IN_MODE=1`, MAVLink only). If the stream stops for a few seconds PX4 lands the drone (I saw this when a blocking `docker exec` paused my loop).
2. `MAV_CMD_DO_SET_MODE` to Altitude (main mode 2). Accepted.
3. `MAV_CMD_COMPONENT_ARM_DISARM` param1=1. Accepted with no GNSS and no vision. In SIH no GCS heartbeat was needed.
4. `MANUAL_CONTROL` z=900 for 10 s gave +10.4 m. z=500 holds. (z is 0 to 1000, 500 is centre.)
5. Set `EKF2_EV_CTRL=1`, stream `VISION_POSITION_ESTIMATE` at 30 Hz, then switch to Position mode (main mode 3). Accepted, held position, and followed wind steps.

In Gazebo (x500, no GNSS) **[tested]**: arming was refused with "Preflight Fail: No connection to the GCS" until my script sent GCS heartbeats (`MAV_TYPE_GCS`, 1 Hz). After that, arm in Altitude mode worked and the climb worked.

The test above fed true position back as the vision message. It only checks the plumbing. It is not a test of your estimator.

## 5. Ground truth from outside PX4

Facts **[src]**:
- SIH publishes `vehicle_attitude_groundtruth`, `vehicle_angular_velocity_groundtruth`, `vehicle_local_position_groundtruth`, `vehicle_global_position_groundtruth` on every sim step (250 Hz at defaults).
- There is no MAVLink stream called `GROUND_TRUTH`. Use `HIL_STATE_QUATERNION` (id 115). Its fields are `time_usec` (PX4 clock, taken from the attitude truth timestamp), attitude quaternion, body rates, `lat`, `lon` (1e-7 deg), `alt` (mm), `vx, vy, vz` (cm/s), `xacc, yacc, zacc`. There is no local x, y in it.
- The MAVLink `LOCAL_POSITION_NED` stream is the estimate, not the truth.
- Convert lat, lon to metres with the SIH origin `SIH_LOC_LAT0`, `SIH_LOC_LON0`, `SIH_LOC_H0`. Resolution is about 1.1 cm in lat (1e-7 deg), 1 mm in altitude. Use a local-tangent conversion. I used R=6371000 m and a cosine factor. Error from this approximation is small near the origin but not zero. For exact values use `px4-listener vehicle_local_position_groundtruth` which gives x, y, z directly.
- Default SIH has `HIL_STATE_QUATERNION` at 25 Hz on UDP 19450/19410 only (the display link). Other links have it off. [src: `px4-rc.mavlink`]

Commands **[tested]**:
- MAVLink, from the host (works on any link, here the 14540 link):
  ```python
  m.mav.command_long_send(sysid, compid, 511, 0, 115, 4000, 0,0,0,0,0)  # MAV_CMD_SET_MESSAGE_INTERVAL, msg 115, 4000 us
  ```
  The stream goes to the link that sent the command.
- PX4 shell: `mavlink stream -r 200 -s HIL_STATE_QUATERNION -u 14580` (the `-u` is the local port of the link, 14580 for the onboard link). In the image: `docker exec px4t /opt/px4/bin/px4-mavlink stream -u 14580 -s HIL_STATE_QUATERNION -r 200`. The command ran with no error. I did not check the resulting rate.
- uORB, one sample: `docker exec px4t /opt/px4/bin/px4-listener vehicle_local_position_groundtruth -n 1` (works, gives x, y, z, vx, vy, vz, heading, `ref_lat`, `ref_lon`). The image has `px4-<module>` client programs in `/opt/px4/bin`.

Measured rate **[tested]**:
| Case | Requested | Delivered |
|---|---|---|
| SIH, sim 250 Hz (default), 1x | 250, 500, 1000 Hz | 184 to 201 per sim second. Steps between messages are 4 ms (about 70 percent) or 8 ms (about 30 percent). So the stream drops samples. |
| SIH, `IMU_INTEG_RATE=400`, speed 5 requested | 500 Hz | 297 per sim second (sim step 2.5 ms). Steps are 2.5 ms, 5 ms, 7.5 ms. |
| Gazebo x500, 4 ms physics step | 250 Hz | about 74 to 76 per second (mostly 20 ms steps). |

Reasons:
- SIH: the MAVLink loop (`MAVLINK_MIN_INTERVAL` 1500 us) is not locked to every sim step. **[src]** The stream sends the latest sample and skips others.
- Gazebo: `/world/<w>/pose/info` is a `SceneBroadcaster` topic with `dynamic_pose_hertz` default 60. **[src]** PX4's `GZBridge::poseInfoCallback` reads this topic and creates the truth. So truth through PX4 is limited to about 60 Hz. For more, add a `gz-sim-pose-publisher-system` (`<update_frequency>`) or `gz-sim-odometry-publisher-system` (`<odom_publish_frequency>`) to the model and read it with gz-transport. [src, not tested]
- In Gazebo the truth velocity and rates are finite differences of the pose. [src]

Advice for the rolling-shutter renderer: take the SIH truth at 300 to 400 Hz, keep `time_usec`, and interpolate the pose (slerp for attitude) at each row time. Do not assume a fixed step.

Logging: the default SITL ulog stores groundtruth topics with an interval limit (10 to 100 ms in `logged_topics.cpp`). Do not use it for 200 Hz truth. [src] I did not test a full-rate log profile.

## 6. Docker

Official images **[docs][tested via Docker Hub API 2026-11-15]**:

| Image | What | Size (compressed, amd64) | Tags seen |
|---|---|---|---|
| `px4io/px4-sitl` | PX4 built with SIH, Ubuntu 24.04, no Gazebo. Entrypoint `/opt/px4/bin/px4-entrypoint.sh`, `PX4_SIM_MODEL=sihsim_quadx`. | 51 MB (53 MB content, 180 MB on disk) | `latest`, `v1.18.0-rc1`, `v1.18.0-beta1`, `v1.18.0-beta2`, `v1.17.0-alpha1-...`. No `v1.17.0` tag. `latest` was `v1.18.0-beta1-660-g61907962c9` (2026-09-11). |
| `px4io/px4-sitl-gazebo` | PX4 plus Gazebo Harmonic | 667 to 688 MB (3.1 GB on disk) | same scheme |
| `px4io/px4-sitl-ros2`, `px4io/px4-sitl-gazebo-ros2` | Above plus ROS 2 Jazzy, DDS agent | about 540 MB | same |
| `px4io/px4-dev` | Build toolchain: Ubuntu 24.04, ARM and Xtensa cross compilers, cmake, ninja, ccache, Python deps. Built with `ubuntu.sh --no-sim-tools`. No Gazebo. | 1.9 to 2.0 GB | `v1.17.0` (2012 MB), `v1.18.0-rc1` (1903 MB) |
| `px4io/px4-dev-ros2`, `px4io/px4-dev-ros2-gazebo` | ROS toolchains | not measured | `main-jazzy`, `main-humble`, `v1.18.0-jazzy` |
| `px4io/px4-dev-simulation-jammy` and older `-focal`, `-bionic` | Legacy (last update 2024-06-12), for Gazebo Classic era | 1.35 GB | `latest` |

Notes:
- `px4-sitl` image has no `python3` and no `px4-dev` tools. It only runs PX4. Modify parameters with `-e PX4_PARAM_<NAME>=<value>` (documented in `simulation/index.md`, implemented in `rcS`) or with `docker exec ... px4-param set`.
- A source build is only needed if you change PX4 code. For SIH you can avoid it.

### 6.1 Run the ready image (tested, recommended)

```sh
docker pull px4io/px4-sitl:latest          # 51 MB, took 7 s here

docker run -d --name px4sih --network host \
  -e PX4_SIM_SPEED_FACTOR=1 \
  -e PX4_PARAM_SYS_HAS_GPS=0 -e PX4_PARAM_SIM_GPS_USED=0 \
  -e PX4_PARAM_EKF2_GPS_CTRL=0 -e PX4_PARAM_EKF2_HGT_REF=0 \
  -e PX4_PARAM_EKF2_EV_CTRL=1 \
  px4io/px4-sitl:latest -d
```

- The trailing `-d` is required. It is the PX4 option "daemon mode, don't start pxh shell". Without it, a container started with `docker run -d` and no TTY loops on `pxh>` and writes about 4 MB of log per second to Docker (I saw 183 MB in 45 s and 100 percent CPU). Same for the Gazebo image. **[tested]** Use `-it` and no `-d` for an interactive shell instead.
- `--network host` (Linux only) is the simplest. PX4 sends to `127.0.0.1:14540` and `127.0.0.1:14550` and listens on 14580 and 18570. Bridge networking with `-p 14540:14540/udp` does not carry the PX4-to-host direction, because PX4 sends to the container's own localhost. (Inference from `px4-rc.mavlink`. The doc says to use `--network host` on Linux or `--add-host=host.docker.internal:host-gateway`, which makes the entrypoint rewrite the target address.) **[docs]**
- Ports (from `px4-rc.mavlink` and docs): PX4 sends to 14550 (GCS) and 14540 (offboard API); PX4 listens on 18570 (GCS link), 14580 (offboard link); 19410 and 19450 are the SIH display link; 8888 is DDS.
- On the host: `mavutil.mavlink_connection('udpin:0.0.0.0:14540')` then `wait_heartbeat()`. Target system 1, component 0 as I measured.
- Logs on host: add `-v $PWD/px4-logs:/root/.local/share/px4/rootfs/fs/log` (from the Dockerfile comment).

Change the vehicle (SIH), for example a 2 kg drone with 4 propellers of 12 N max:
```sh
  -e PX4_PARAM_SIH_MASS=2.0 -e PX4_PARAM_SIH_T_MAX=12.0 -e PX4_PARAM_MPC_THR_HOVER=0.41 \
  -e PX4_PARAM_SIH_IXX=0.03 -e PX4_PARAM_SIH_IYY=0.03 -e PX4_PARAM_SIH_IZZ=0.05 \
  -e PX4_PARAM_SIH_KDV=0.5
```
(Numbers are examples only. `0.41` is `2.0*9.81/(4*12)`.)

### 6.2 Source build with the source mounted (for PX4 code changes)

The local clone is `main`. In `main` the SIH target is `px4_sitl_sih`. In v1.16 and v1.17 it is `make px4_sitl sihsim_quadx` [docs: v1.17 SIH page, v1.18 page].

```sh
cd /home/paul/Videos/vio-hold-attempt/px4sim/PX4-Autopilot
mkdir -p ~/.ccache
docker run --rm -it --network host \
  --user "$(id -u):$(id -g)" -e CCACHE_DIR=$HOME/.ccache -v $HOME/.ccache:$HOME/.ccache \
  -v "$PWD:$PWD" -w "$PWD" \
  px4io/px4-dev:v1.18.0-rc1 \
  bash -c 'git config --global --add safe.directory "*"; make px4_sitl_sih'

# run it headless from the same image, from the build tree
docker run -d --name px4src --network host --user "$(id -u):$(id -g)" \
  -v "$PWD:$PWD" -w "$PWD/build/px4_sitl_sih" \
  -e HEADLESS=1 -e PX4_SIM_MODEL=sihsim_quadx \
  px4io/px4-dev:v1.18.0-rc1 ./bin/px4 -d ./etc
```
- This sequence follows `Tools/docker_run.sh` (docs: `test_and_ci/docker.md`) and `Tools/simulation/sitl_multiple_run.sh` (`px4 -i <n> -d <build>/etc`). I did **not** run the build here. Build time and extra dependencies for the main branch were not measured. The docs say `px4_sitl` (not `_sih`) also builds Gazebo libraries, so use `px4_sitl_sih`.
- `HEADLESS=1` has no effect on SIH. SIH never opens a window. It matters only for Gazebo.
- The `px4-dev` image is 1.9 GB compressed and has a home-directory `ccache` mount in `docker_run.sh` for repeat builds.
- If `make px4_sitl_sih` in the container fails on a missing tool, run `Tools/setup/ubuntu.sh --no-sim-tools` as in the Dockerfile. (Untested.)

### 6.3 Gazebo Harmonic image (only if needed)

```sh
docker run -d --name px4gz --network host \
  -e HEADLESS=1 -e PX4_GZ_WORLD=windy \
  -e PX4_PARAM_SYS_HAS_GPS=0 -e PX4_PARAM_SIM_GPS_USED=0 \
  -e PX4_PARAM_EKF2_GPS_CTRL=0 -e PX4_PARAM_EKF2_HGT_REF=0 \
  px4io/px4-sitl-gazebo:latest -d
```
Needs GCS heartbeats from your script to arm. Inside the image the `gz` command and `px4-*` clients exist.

## 7. Simulation speed, lockstep, time base

Facts **[src][docs]**:
- `PX4_SIM_SPEED_FACTOR` is read in `Sih::lockstep_loop()`. The sim step is `1e6 / min(IMU_GYRO_RATEMAX, IMU_INTEG_RATE)` microseconds, limited to 500..5000 us. At defaults (`IMU_GYRO_RATEMAX` 400, `px4-rc.simulator` sets `IMU_INTEG_RATE` 250) this is 250 Hz (4000 us). The sim sets the PX4 clock (`px4_clock_settime`) every step. The real-time wait is `sim_interval / speed_factor`.
- `rcS` multiplies `COM_DL_LOSS_T` (x10), `COM_RC_LOSS_T`, `COM_OF_LOSS_T`, `COM_DISARM_PRFLT` by the speed factor, so the link-loss timers stay in wall time. I saw `COM_DL_LOSS_T set to 50` at factor 5.
- Lockstep waits only for PX4's own components (`px4_lockstep_wait_for_components`) and for new actuator outputs. It does **not** wait for UDP messages from your program. (Inference from reading the code.)
- Gazebo: `px4-rc.gzsim` calls the `set_physics` service with `real_time_factor` and `max_step_size`. `GZBridge::clockCallback` sets the PX4 clock on each sim step.

Measured **[tested]**:
- SIH at factor 1: sim time 3.91 s in 3.99 s wall.
- SIH, factor 5 requested, 400 Hz sim step: sim time advanced 12.12 s in 5.00 s wall, so 2.4x. The container used 23 percent of one core. The limit is not CPU, so the real factor depends on the loop and sleep granularity. Always measure the achieved factor. `simulator_sih status` prints "Achieved speedup".

Time base for your program **[src]**:
- Use PX4's simulated clock: `HIL_STATE_QUATERNION.time_usec`, or `SYSTEM_TIME.time_boot_ms` (boot time). Not your wall clock. Pace your loop by these timestamps (for example, send a vision message when the truth time passes the next frame time). Then you stay in step at any speed factor.
- Vision messages: PX4 uses `Timesync::sync_stamp(usec)`. If MAVLink `TIMESYNC` has not converged, it **ignores your timestamp and uses its own arrival time** (`hrt_absolute_time()`). If it has converged, it uses `usec + offset`. So (a) if you do not answer `TIMESYNC`, your messages are stamped with arrival time in the sim clock. This is correct only if your latency is small in sim time. At a speed factor of N, a wall-clock delay of D ms is N times D ms of sim time. (b) Set `EKF2_EV_DELAY` for the camera delay (analog camera plus processing). Send `time_usec` of the camera frame in the sim clock.
- Wall-clock pacing at speed >1 breaks the step relation. A fixed 30 fps wall timer would make 2.4 sim frames per wall frame at 2.4x. Use sim time.
- Quantisation: SIH time stamps are multiples of the sim step (4000 us at defaults).

## 8. pymavlink or MAVSDK-Python

Use **pymavlink**. Reasons:
1. MAVSDK-Python changed names. README **[docs]**: the gRPC wrapper is now `mavsdk-grpc` (import `mavsdk_grpc`). On PyPI, `mavsdk` is now a different native binding (version 4.2.0, 2026-11-12) with a different API. Old tutorials will break. The gRPC version starts a `mavsdk_server` process.
2. You need raw messages in any case: `HIL_STATE_QUATERNION` for truth, `VISION_POSITION_ESTIMATE` or `ODOMETRY` for your estimator, `MANUAL_CONTROL`, `PARAM_SET` for wind. MAVSDK has no plugin for reading `HIL_STATE_QUATERNION`. (Inference from the plugin list I know. I did not check MAVSDK v4.)
3. pymavlink 2.4.50 (2026-09-24) installs as one 7 MB wheel, no server process. **[tested]**

Working pattern **[tested]**:
```python
from pymavlink import mavutil
mav = mavutil.mavlink
m = mavutil.mavlink_connection('udpin:0.0.0.0:14540')
m.wait_heartbeat()
T = (m.target_system, m.target_component)

# ground truth stream, 250 Hz request
m.mav.command_long_send(*T, 511, 0, 115, 4000, 0, 0, 0, 0, 0)

# parameter set: INT32 parameters need the integer bits packed into a float
import struct
def set_int(n, v): m.mav.param_set_send(*T, n.encode(), struct.unpack('<f', struct.pack('<i', v))[0], mav.MAV_PARAM_TYPE_INT32)
def set_f(n, v):   m.mav.param_set_send(*T, n.encode(), v, mav.MAV_PARAM_TYPE_REAL32)
set_int('EKF2_EV_CTRL', 1); set_f('SIH_WIND_N', 3.0)

# modes (pymavlink knows PX4 main modes: MANUAL 1, ALTCTL 2, POSCTL 3, OFFBOARD 6, STABILIZED 7)
m.set_mode_px4('ALTCTL', 0, 0)          # sends MAV_CMD_DO_SET_MODE
m.mav.command_long_send(*T, mav.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)

# every 40 ms: MANUAL_CONTROL(target, x, y, z, r, buttons); z=500 hold, z>500 climb
m.mav.manual_control_send(T[0], 0, 0, 900, 0, 0)

# vision message at camera rate, stamped with the sim time from HIL_STATE_QUATERNION
m.mav.vision_position_estimate_send(time_usec, north, east, down, roll, pitch, yaw, [float('nan')] + [0]*20, 0)

# GCS heartbeat once per second (needed to arm in Gazebo)
m.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
```
Gotchas **[tested]**:
- `PARAM_VALUE` and `PARAM_SET` for INT32 parameters carry the integer in the float bits. Reading `EKF2_EV_CTRL` returned `4.2e-45` for the value 3. Decode with `struct`.
- `set_mode_px4(mode, custom_mode, custom_sub_mode)` needs all three arguments even when `mode` is a name.
- Keep sending `MANUAL_CONTROL` without gaps. A long blocking call stops the stream and PX4 lands the drone.
- Read the mode from `HEARTBEAT.custom_mode`: Altitude is `0x20000`, Position `0x30000`.
- MAVSDK 4 `mocap`/odometry plugins exist in the older gRPC wrapper. I did not test either.

## 9. Sources

PX4 docs and source (main unless noted):
- SIH docs: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/sim_sih/index.md
- SIH parameters: https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/simulator_sih/sih_params.yaml
- SIH source: https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/simulator_sih/sih.cpp and `sih.hpp` in the same folder
- v1.17.0 SIH parameters (no wind): https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/simulation/simulator_sih/sih_params.c
- Simulation overview, `PX4_PARAM_*`, speed factor, comparison: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/index.md
- Prebuilt packages and containers: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/px4_sitl_prebuilt_packages.md
- Quickstart: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/px4_simulation_quickstart.md
- Container builds: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/dev_setup/sitl_container_builds.md
- Docker dev containers: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/test_and_ci/docker.md
- `Dockerfile.sih`, `Dockerfile.gazebo`, entrypoint: https://github.com/PX4/PX4-Autopilot/tree/main/Tools/packaging/containers
- Dev Dockerfile: https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/Dockerfile and `Tools/docker_run.sh`
- Multi-instance script (`px4 -i N -d <build>/etc`): https://github.com/PX4/PX4-Autopilot/blob/main/Tools/simulation/sitl_multiple_run.sh
- Startup scripts: https://github.com/PX4/PX4-Autopilot/tree/main/ROMFS/px4fmu_common/init.d-posix (`rcS`, `px4-rc.simulator`, `px4-rc.sihsim`, `px4-rc.gzsim`, `px4-rc.mavlink`, `airframes/10040_sihsim_quadx`, `10046_sihsim_quadx_flow`, `4021_gz_x500_flow`)
- Simulated sensors: https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation (`sensor_gps_sim`, `sensor_baro_sim`, `sensor_mag_sim`)
- Gazebo bridge: https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/gz_bridge/GZBridge.cpp and `server.config`
- Gazebo docs in PX4: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/sim_gazebo_gz/index.md and `worlds.md`
- Gazebo Classic docs: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/sim_gazebo_classic/index.md
- jMAVSim v1.17 docs: https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/docs/en/sim_jmavsim/index.md ; removal commit: https://github.com/PX4/PX4-Autopilot/commit/a23449cbe ; jMAVSim README: https://github.com/PX4/jMAVSim/blob/master/README.md
- Community simulators: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/community_supported_simulators.md
- Release notes 1.18: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/releases/1.18.md ; SIH wind PR: https://github.com/PX4/PX4-Autopilot/pull/26467 ; SIH build target PR: https://github.com/PX4/PX4-Autopilot/pull/26672
- MAVLink stream `HIL_STATE_QUATERNION`: https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/streams/HIL_STATE_QUATERNION.hpp
- MAVLink main loop interval and `SET_MESSAGE_INTERVAL`: https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_main.h and `mavlink_receiver.cpp`
- Time sync: https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/timesync/Timesync.cpp
- Mode requirements: https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp
- Estimator arming checks: https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/HealthAndArmingChecks/checks/estimatorCheck.cpp
- Parameters: `src/lib/systemlib/system_params.yaml` (`SYS_HAS_GPS`), `src/modules/ekf2/params_gnss.yaml`, `params_external_vision.yaml`, `module.yaml` (`EKF2_HGT_REF`), `src/modules/commander/commander_params.yaml` (`COM_ARM_WO_GPS`, `COM_RC_IN_MODE`), `src/modules/sensors/vehicle_imu/imu_parameters.yaml` (`IMU_INTEG_RATE`)
- Logger topics: https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/logger/logged_topics.cpp

Gazebo:
- PX4 gazebo models (windy world, x500): https://github.com/PX4/PX4-gazebo-models/blob/main/worlds/windy.sdf and `models/x500/model.sdf`, `models/x500_base/model.sdf`
- `WindEffects`: https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/wind_effects/WindEffects.hh and `WindEffects.cc`
- `ApplyLinkWrench`: https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/apply_link_wrench/ApplyLinkWrench.hh
- `SceneBroadcaster` (`dynamic_pose_hertz` 60): https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/scene_broadcaster/SceneBroadcaster.cc
- `OdometryPublisher`, `PosePublisher`: https://github.com/gazebosim/gz-sim/tree/gz-sim8/src/systems

Docker Hub (API calls made 2026-11-15):
- https://hub.docker.com/v2/repositories/px4io/px4-sitl/tags , https://hub.docker.com/v2/repositories/px4io/px4-sitl-gazebo/tags , https://hub.docker.com/v2/repositories/px4io/px4-dev/tags , https://hub.docker.com/v2/repositories/px4io/

Forum:
- https://discuss.px4.io/t/unable-to-simulate-wind-via-make-px4-sitl-gz-x500-windy/45696
- https://discuss.px4.io/t/wind-effects-in-gazebo/48053
- PR 26018 (gz airspeed plugin with wind): https://github.com/PX4/PX4-Autopilot/pull/26018

Python:
- https://pypi.org/project/pymavlink/ , https://pypi.org/project/mavsdk/ , https://github.com/mavlink/MAVSDK-Python/blob/main/README.md
- pymavlink `mavutil.py` (PX4 mode map): https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py

## 10. Cleanup note

I pulled `px4io/px4-sitl:latest` and `px4io/px4-sitl-gazebo:latest` (3.3 GB on disk together) and created a scratch venv with pymavlink in the session scratchpad. All test containers are removed. I did not edit any existing file.
