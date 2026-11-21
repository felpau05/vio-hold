# Review findings (2026-11-15, three reviewers)

> **STATUS 2026-11-18, re-run (analog simulator; outputs in integ/review_rerun/):** E1 fixed (e8_texture: no fixes over
> static of std 3-30, fading ground stops with 0 confident-wrong fixes at 100 m). E2 fixed (e4_outliers: frozen frame
> sends nothing, static OSD 0.34 m). E3 much better (30 % object: RMS 1.10 m, 3.7 % false-fix, was 29.6 m). E7 fixed
> (e5_velocity: back in 0.11-0.18 s after a 3 s blackout, no re-anchor help). E5 still open: a 90 deg yaw turn leaves
> 2.2-2.6 m of error at 100 m (e4_outliers). E6 re-verified 2026-11-18 (e3_keyframes.py and e2_bad_cases.py rewritten for
> the new Hold; the old KfRuleHold comparison and forced-change test are gone): prairie 50 m keyframe changes 191-199 -> 34,
> RMS 1.3-1.9 -> 0.18-0.28 m; ottawa 100 m with +-30/60/90 deg yaw RMS 0.82/1.24/2.10 -> 0.54/0.94/1.32 m (changes 8/13/18, more
> than before but each cheaper). NOT fully fixed: prairie 100 m seeds 3 and 5 still 1.7-1.9 m RMS, 77-85 % degraded, 34-57 changes
> (seed 1: 0.46 m, 1 change). The 'spread' column in e2_bad_cases B is approximate (ATT_NOISE no longer exists).
> Earlier note, read from the code: hold.py now cites these IDs in its
> fixes: E1 (NCC_MIN, NCC_FRAME), E2 (STILL_PX, GRID, repeated-frame check), E3 (per-axis 2.5-sigma gate, GRID),
> E4 (honest variance model, per axis), E6 (KF_GAP, KF_LOW), E7 (t and vel cleared when lost). E5 is only partly
> covered (BETA_UNC adds variance; no boresight yaw calibration exists). Every other ID is unchecked. The findings
> were written for the analog Justice A; see SPEC.md for the Justice D change. Re-run the proof scripts before
> citing a finding as fixed or open.
Each finding has a proof script in its lens folder. IDs are stable: use them in commits, tests and reports.

# Lens: estimator

## E1 [critical] No working 'no ground' test: analog static (std >= 5) and faded ground give keyframes and confident wrong fixes

- Where: hold.py:24 (LK minEigThreshold=3e-3), hold.py:139-157 _new_kf texture self-test, hold.py:178-190 (no per-point match-quality test)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e8_texture.py ; ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e4b_fb_check.py`

**Problem.** The only 'is there ground?' test is LK's minEigThreshold, tuned to reject std-3 static. At std 5 (sim_ground's own NOISE) and above, static passes. Hold makes keyframes on static and sends fixes whose position follows the attitude, with small variance. The same happens as ground contrast fades: the hold does not stop until contrast is 0-6 %, and it sends wrong fixes all the way down. sim_test's blackout test passes only because its static is std 3. Real analog snow is std 30-60. The EKF gets confident wrong positions exactly when the video link drops.

**Evidence.** e8_texture.py part C (5 s blackout at 100 m, caller re-anchors; columns: static std / keyframes made on static / fixes sent of 135 / 'ok' / max err m / sent sigma per axis m / false-fix):
  3: 0 / 0 / 0 / - / - / -
  5: 9 / 29 / 10 / 13.83 / 0.40 / 100%
  10: 1 / 124 / 1 / 31.96 / 0.97 / 87.9%
  30: 4 / 75 / 0 / 31.47 / 1.70 / 86.7%
Part A, 10 % contrast: noise 5 at 100 m: 100 % sent, RMS 2.34 m, 71 keyframe changes. Noise 10: RMS 8.75 m (100 m) / 17.09 m (200 m), false-fix 79.7 % / 86.1 %.
Part B, fade to 0 % over 15 s: the hold stops at 0-6 % contrast. In its last 2 s: mean error 1.7-10.1 m, max 9.5-21.9 m, sigma 0.5-1.9 m, false-fix up to 72.4 %, up to 42 confident-wrong fixes, and 13-18 fixes sent after contrast reached 0.
e4b_fb_check.py: 'static std 10: 300 of 300 corners pass Hold's texture self-test'. Patch NCC median: ground 0.96 (5th percentile 0.73), static std 10: 0.06 (95th percentile 0.21), std 30: 0.08 (95th percentile 0.22), ground at 10 % contrast: 0.10.

**Suggested fix.** Add a per-point NCC gate between the keyframe patch (11x11) and the tracked patch. Reject points with NCC < 0.6, and fail the frame ('no ground') when fewer than MIN_PTS pass. Also treat the frame as failed when the median NCC is below 0.5. Change sim_test's blackout to std 30-60 snow, plus a std-5 case, so the test covers real static.

## E2 [critical] Frozen frame or static overlay (OSD, dirt, airframe in view) gives 'ok'/'degraded' fixes tied to attitude; never 'lost'

- Where: hold.py:165-219 Hold.process (no check that the picture changed or that tracked motion matches the attitude-predicted motion)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e4_outliers.py`

**Problem.** Points that stay still in the image (a repeated capture buffer, burned-in OSD text, a crosshair, dirt) look perfectly consistent to LK and to the forward-backward check. The fix becomes C = G - g(att): it moves with the drone's tilt (h·tilt, about 10 m at 100 m for 6°), not with its position. Hold keeps sending with sigma 0.2-1 m. In closed loop the controller can cancel this 'error' by holding a small tilt, which means sustained acceleration (flyaway), unless the EKF gate rejects the measurement.

**Evidence.** e4_outliers.py, ottawa2019fall 100 m, event at t = 5 s (RMS after / max after / sent after / confident-wrong (> 2 m and outside 99 %) / states ok-deg-lost):
  frozen frame + 0.5 m/s drift: 10.63 / 22.58 / 100% / 288 / 44-256-0
  static OSD, ground at 40 % contrast: RMS 8.76 m already BEFORE the drift; after: 8.07 / 17.60 / 100% / 17 / 82-218-0
  frozen-frame trace: t=6 s err 15.33 m, sigma 0.25 m; t=9 s err 22.58 m, sigma 0.52 m (states 'degraded', still sent)
  static OSD + grid-bucketed corners (proposed fix): RMS 0.32 m, max 0.81 m, 0 confident-wrong.

**Suggested fix.** (1) Fail the frame if the capture buffer is identical to the previous one (hash or the grabber's frame counter). (2) Reject points whose image displacement from the keyframe stays below about 0.5 px while the attitude-predicted displacement is several px. (3) Mask known overlay regions. (4) Pick corners per grid cell (the 6x4 bucketing in e4_outliers.py) so that no region owns the keyframe.

## E3 [high] Large textured moving object pulls the fix: loose inlier gate plus mean of inliers

- Where: hold.py:140 goodFeaturesToTrack (global top-300), hold.py:193-200 (median, gate max(3*1.4826*median(r), 0.003h+0.05), pos = mean of inliers)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e4_outliers.py`

**Problem.** goodFeaturesToTrack gives most corners to the strongest texture. Even with grid bucketing (27 % of points on the object), the gate accepts the object's cluster. 3·1.4826·median(r) on a 2-D radius is about 5.2 sigma for a Gaussian, and a contaminated median makes it larger. The fix is then the MEAN of the inliers, so it is dragged toward the object. The fix follows the object at its speed with a small variance.

**Evidence.** e4_outliers.py, object covering ~27 % of the picture (saved frame e4_big_object.png) moving 3 m/s: RMS before 3.42 m, after 29.57 m, max 39.01 m, 296 confident-wrong, false-fix 100 %. With grid-bucketed corners: still 29.44 m / 296. Debug at t = 2.51 s: 136 tracked = 136 inliers (88 ground with median err 1.19 m, 48 object with median err 7.32 m), fix error 3.75 m.

**Suggested fix.** Gate per axis at about 2.5 sigma, with sigma = 1.4826·MAD of each axis. Re-centre on the inlier median and re-gate once or twice. Use the median, or the mean of the tight core, as the fix. Pick corners per grid cell. Optionally require the inlier set to hold at least 60 % of tracked points, or mark the frame degraded and inflate its variance.

## E4 [high] Sent variance is too small by ~1.7x in sigma: one attitude error counted, two present; several sources missing

- Where: hold.py:209-211 (var formula), hold.py:98 ATT_NOISE, hold.py:111-112 reanchor (anchor var 0)
- Simulator can catch it: True
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e1_variance.py ; ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e1b_honest_var.py`

**Problem.** Every fix carries two independent attitude errors. The first is the anchor frame's attitude error, frozen into G and inherited by all later fixes, also across keyframe changes. The second is the current frame's error. The formula counts one 0.15° term, and sim_ground reads var as a trace, so the per-axis allowance is 0.106°. Measured per axis, anchor and frame are 0.12-0.14° each. Missing entirely: tilt-bias change since the anchor (constant in the sim, not on real FCs), mount-or-level error × yaw change, and vibration and timing beyond the white noise. The keyframe-switch term is wrong the other way: each switch re-adds the whole fix variance, but the measured increment is only about spread²/n. The scalar's convention (trace or per-axis) is ambiguous and will be guessed in apb_main.

**Evidence.** e1_variance.py part A (6 seeds × 20 s), pooled false-fix: 12.9 % (100 m), 14.0 % (200 m), 19.8 % (300 m), 23.7 % (400 m); per-seed range 3-51 %. Per-axis error/h: anchor 0.116-0.139°, frame 0.118-0.133°, against Hold's 0.106°.
Part B ablation at 300 m (16 seeds, anchor RMS): all 1.26 m; no vibration 1.03; no FC noise 1.04; no timing error 1.02; no constant bias 1.03; all off 0.14.
e1b_honest_var.py (102 held-out runs, other sites, ±60° yaw, prairie 50 m), ALL: Hold false-fix 15.8 %, out-50 74.9 %, NEES 4.83 -> honest.py 0.7 %, 50.0 %, 1.92 (ideal 1 %, 50 %, 2.0). Weak spots: prairie 50 m 6.1 %, ±60° yaw at 100 m NEES 2.97.
e7_long_hover.py, 120 s with 0.2° tilt-bias wander: Hold false-fix 51.7-91.3 % -> honest.py 0.0-0.3 %.

**Suggested fix.** Use the model in /home/paul/Videos/vio-hold-attempt/review/estimator/honest.py (drop-in subclass), per axis: (SIG_TH·h_anchor)² + Σ_switches spread²/(2n) + carried_EKF_var + (SIG_TH·h)² + 2(SIG_BIAS·h)²(1 - e^(-Δt/τ)) + (2h·BETA_UNC·sin(|Δψ|/2))²/2 + spread²/(2n) + HGT term. Set SIG_TH online from the jitter: std(first difference per axis)/(√2·h). Inflate the switch term about 3x (n_eff ≈ n/3). Return a 2x2 per-axis covariance, not an ambiguous scalar. Make reanchor() take the EKF variance.

## E5 [high] Camera-mount / FC-level tilt error does not cancel when the drone yaws: error 2·h·beta·sin(dpsi/2)

- Where: hold.py:19 R_BC_NADIR (assumed exact), hold.py:114-122 _rays; no boresight calibration or estimate anywhere
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e7b_mount_yaw.py`

**Problem.** A constant tilt error beta between the camera and the FC attitude is stored in every keyframe's G pointing one way. It comes from organizer camera-mount tolerance or FC level bias. After a yaw change dpsi, it points another way, so the fix is off by 2·h·beta·sin(dpsi/2). Nothing estimates beta, and the variance has no term for it. The sim's own ATT_BIAS (0.58° tilt) already shows this as the 20 s yaw-period cycle in the long-hover error (e7_long_hover.png), and it explains the 1.5-1.8 m offset after the 90° turn in e4.

**Evidence.** e7b_mount_yaw.py (beta = 1° roll; columns: h / yaw change / error after turn / predicted / false-fix):
  100 m / 90° / 3.61 m / 2.47 m / 100 %
  100 m / 180° / 4.97 m / 3.49 m / 100 %
  200 m / 90° / 6.03 m / 4.94 m / 100 %
  200 m / 180° / 9.48 m / 6.98 m / 100 %
  control beta = 0, 200 m / 180°: 2.05 m (from the 8 keyframe changes during the turn).

**Suggested fix.** Calibrate the boresight on the event day: one slow 360° yaw at hold. The fix traces a circle of radius h·beta, so fitting it gives beta to about 0.05°. Apply the result as a correction to R_BC. Alternatively, estimate beta online from the yaw-correlated fix error. Add the s_yaw term to the variance. Keep yaw fixed during the scored window.

## E6 [high] Keyframe rule makes a random walk on marginal texture; KF_RATE preference never acts

- Where: hold.py:217-218 (n < 2*MIN_PTS or n < 0.5*n0 and rate < KF_RATE), hold.py:130-136 (rate from differenced noisy Euler angles), hold.py:185 (failed points keep garbage prev_px)
- Simulator can catch it: True
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e3_keyframes.py ; ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e2_bad_cases.py`

**Problem.** When inliers hover near 40, `n < 2*MIN_PTS` fires every few frames. Each change hands over the mean error of a few noisy points, and the fix random-walks. n0 counts corners passing the self-test (about 267), not usable inliers (about 44), so the n0/2 test is meaningless. Hold.rate is the max of differenced noisy Euler angles, which stays above KF_RATE (0.3 rad/s) almost always. Switching therefore waits for n<40, the moment with the fewest and worst points (also during yaw turns). Yaw swings force changes because LK cannot follow rotated patches. The prairie-50 m 'bad case' is this rule, triggered by the simulator's upsampled 0.586 m/px photo.

**Evidence.** e2_bad_cases.py, prairie 50 m: n0 267, LK-ok 64-65, inliers 43-45, keyframe changes 191-199 in 20 s, error 0.25 -> 2.99 m (seed 3). Controls with 0 changes: prairie 100 m (80 inliers, RMS 0.40 m), ottawa 50 m (231 inliers, 0.20 m), synthetic detail at 50 m (183 inliers, 0.22 m).
e3_keyframes.py A: ottawa 50-400 m with ±10° yaw: 0 changes. 100 m, changes / RMS: ±30° yaw 3 / 0.82 m; ±60° 11 / 1.24 m; ±90° 15 / 2.10 m. Every one from n<40; 0 % of frames with rate < KF_RATE (median 0.95 rad/s).
e3b_rate_noise.py: calm ±1° hover with 0.1° noise gives median rate 0.53 rad/s and 1.7 % of frames below KF_RATE; with no noise, 100 %.
B: per-change sigma 0.04-0.07 m at 100 m and 0.08-0.15 m at 300 m (≈ √(spread²/n)).
C, calmer rule on prairie 50 m: changes 191 -> 59, RMS 1.29 -> 0.56 m; seed 3: 199 -> 64, 1.92 -> 1.08 m, end error 2.99 -> 0.80 m.

**Suggested fix.** Use KfRuleHold (e3_keyframes.py). Switch when inliers fall below half of this keyframe's FIRST tracked inlier count, or below MIN_PTS+10, and at most once per second unless forced. Gate on gyro rates (MAVLink ATTITUDE rollspeed, pitchspeed, yawspeed), not differenced Euler angles. For yaw and revisits, keep about 8 old keyframes. Predict each one's points by projecting G with the current pose, warp its picture by the attitude difference before LK, and use the oldest keyframe that still gives at least 2·MIN_PTS inliers. Re-seed points whose tracking failed.

## E7 [high] After 'lost' without external re-anchor, Hold never recovers (jump-gate livelock on stale t and vel)

- Where: hold.py:159-163 _fail (sets kf=None, keeps t/vel/pos), hold.py:202-205 jump gate, hold.py:152 new keyframe anchored at stale pos
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e5_velocity.py`

**Problem.** On loss only kf is cleared. The next keyframe is anchored at the old pos, but the gate predicts pos + vel·(now - t), with a stale t and the noisy filtered vel (2-10 m/s). After a gap of 1-3 s the prediction is metres off, so every frame fails 'jump'. After 10 failures the state goes 'lost' again with an even larger dt, forever. This only works if the caller always re-anchors. sim_ground does not, and apb_main.py is unwritten. Permanent loss of vision means the FC dead-reckons and drifts.

**Evidence.** e5_velocity.py part B (3 s blackout while moving 1 m/s; columns: h / mode / back after / sent after / why not sent):
  20 m / own pos (sim_ground) / never / 0 % / {'jump': 90}
  50 m / own pos / never / 0 % / {'jump': 90}
  100 m / own pos / never / 0 % / {'jump': 90}
  20-100 m / re-anchor (sim_test) / 0.18 s / 97 % / mean err 0.33-0.42 m

**Suggested fix.** In _fail, when it declares lost, also set self.t = None and self.vel[:] = 0. That way a keyframe made after a loss is never gated against a stale extrapolation. Make reanchor(pos, var) take the EKF variance.

## E8 [medium] Claimed sim accuracy and both 'bad cases' are single-seed draws; 400 m bad case is one unlucky anchor

- Where: sim_ground.py:118-133 one() / sim_test.py:24 rng (one 20 s run per height, single shared RNG)
- Simulator can catch it: True
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e2_bad_cases.py`

**Problem.** Each 20 s run contains essentially ONE draw of the dominant error, the anchor-frame attitude error. So a run's RMS and false-fix mostly measure that one draw. The reported table varies about ±40 % with the seed. The '400 m bad case' is seed 5 and nothing else: no tracking loss, no keyframe changes. Later tuning against single runs will chase noise.

**Evidence.** e1_variance.py A, RMS per seed at 300 m: 1.06 1.49 1.06 1.40 1.85 1.09. False-fix per seed at 400 m: 8 % 31 % 13 % 26 % 51 % 13 %. Seed 5 is the worst at every height.
e2_bad_cases.py A, 400 m seed 5: RMS 2.59 m, false-fix 50.8 %, lost 0 %, 0 keyframe changes, constant offset 2.19 m. Correlation of the offset with h × FC attitude noise on the anchor frame: 0.99 (seed 5 noise -0.147°, -0.144°, the largest).
Baseline run today (baseline_sim_ground_ottawa2019fall.txt): 400 m RMS 1.40 m, false-fix 7.0 %.

**Suggested fix.** Report pooled results over at least 8-12 seeds per case (C.pmap in common.py runs them in parallel). Report the anchor offset and the jitter separately.

## E9 [medium] Compute: ~18 ms/frame on one 5.2 GHz laptop core -> ~0.2 s on one A53; 30 fps not realistic as written

- Where: hold.py:24 LK (21x21, maxLevel 3), hold.py:140 (300 corners), hold.py:178-182 (3 LK calls, each rebuilding both pyramids)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e9_compute.py ; ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e9b_low_fps.py`

**Problem.** 93 % of the time is in the three calcOpticalFlowPyrLK calls per frame. Geekbench 5 single-core: i.MX8M Plus about 146-161 against the Core 7 240H at about 1686-1862, so about 11x (multi-core about 18-20x). As written that means about 195 ms mean and about 340 ms worst on one A53 (about 5 fps), or about 55 ms using all four cores. The hold still works at 6 fps, but with a lower update rate and more latency. A leaner setting keeps accuracy and makes 30 fps plausible. Not measured on the target.

**Evidence.** e9_compute.py, Hold.process per frame (mean / p95 / max ms): as written, 1 thread 17.7 / 29.6 / 31.2; 4 threads 5.1 / 5.9 / 6.5; LK 15x15 + 2 levels, 1 thread 8.1 / 8.7 / 10.3; plus 120 corners, 1 thread 3.9 / 4.1 / 4.6 (RMS 0.27 m against 0.28 m); 4 threads 1.9. cProfile: calcOpticalFlowPyrLK 4.209 s of 4.522 s. Series.array() rebuild 0.53 ms. e9b_low_fps.py: 6 fps at 100 m gives RMS 0.33-0.43 m with 99 % of fixes sent. Sources: browser.geekbench.com/v5/cpu/7106154, browser.geekbench.com/v5/cpu/19042077, notebookcheck.net Intel-Core-7-240H page.

**Suggested fix.** Use winSize 15x15, maxLevel 2 and about 120 bucketed corners. Build the keyframe pyramid once with buildOpticalFlowPyramid. Drop the prev->cur LK call and predict p1 from attitude and G instead. Keep Series in a preallocated numpy ring buffer. Benchmark on the APB as soon as one is available.

## E10 [medium] 'degraded' fixes are sent with the same variance as 'ok'

- Where: hold.py:209-213 (var computed before state; state does not change var), hold.py:27-29 docstring
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e4_outliers.py`

**Problem.** The docstring says 'degraded (send, larger covariance)', but the variance does not depend on state. Frames flagged degraded because of large scatter or few points go out as confident as good ones. That is how most of the frozen-frame garbage was sent.

**Evidence.** e4_outliers.py, frozen frame: 256 'degraded' fixes after t = 5 s, errors 2.5-22.6 m with sent sigma 0.25-1.05 m per axis; false-fix 97 %.

**Suggested fix.** When degraded, multiply var by a factor (for example 4-10), or send nothing.

## E11 [medium] Velocity filter is 4-17x noisier than the motion it compensates and has no effect on accuracy

- Where: hold.py:206-207 (vel = 0.7 vel + 0.3 dpos/dt), uses at hold.py:152, 192, 204
- Simulator can catch it: True
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e5_velocity.py`

**Problem.** Differencing 30 Hz fixes with about 0.24 m per-frame noise gives velocity noise of several m/s. The EMA is stable (pole 0.7), but the result is useless for the 8 ms row-time compensation. Forcing vel to zero or to the true value gives identical results. Its only real effect is harm: the jump-gate extrapolation after a loss (livelock above). It is also a trap if anyone sends it to the FC as a velocity.

**Evidence.** e5_velocity.py A, velocity noise RMS: 2.30 m/s (100 m), 4.86 m/s (200 m), 10.00 m/s (400 m), against true speed 0.59 m/s. RMS for Hold / vel=0 / vel=true: 0.38 / 0.38 / 0.38 (100 m), 0.76 / 0.76 / 0.76 (200 m), 1.81 / 1.81 / 1.81 (400 m).

**Suggested fix.** Drop the vel term, or take velocity from the FC EKF (LOCAL_POSITION_NED) if it is needed. Reset it on loss.

## E12 [low] Frame jitter 0.24/0.50 m per axis (100/200 m); FC would wander ~0.15/0.31 m RMS; pre-filters do not reduce real motion

- Where: output of Hold.process (no smoothing; EKF R taken from var)
- Simulator can catch it: True
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/estimator/e6_jitter.py`

**Problem.** The fix error is mostly white, with peaks at the attitude frequencies (0.7, 2 Hz) and at vibration aliases (97 Hz -> 7 Hz, 140 Hz -> 10 Hz at 30 fps). In a closed-loop model (IMU-driven Kalman filter plus PX4-style cascade) the drone wanders 0.15 m (100 m) and 0.31 m (200 m) RMS from estimator noise alone. An EMA or moving average cuts jitter 2.5-4.5x but leaves the motion unchanged, because the EKF already low-passes. An honest R cuts commanded tilt by about 20 %. Much of the jitter comes from the sim's white attitude noise; a real FC is smoother but drifts.

**Evidence.** e6_jitter.py: jitter 0.241 m (100 m), 0.498 m (200 m); white-noise ratio 0.83. Closed loop at 100 m: wander 0.150 m and tilt 0.354° (Hold var), 0.156 m and 0.280° (honest). At 200 m: 0.300 m / 0.573° and 0.310 m / 0.469°. Pre-filters at 100 m (jitter, delay, wander): none 0.241 m, 0 ms, 0.156 m; EMA 0.5: 0.098 m, 33 ms, 0.155 m; MA-3: 0.081 m, 33 ms, 0.155 m; EMA 0.3: 0.055 m, 78 ms, 0.154 m. PSD plot: e6_jitter_psd.png.

**Suggested fix.** Send honest per-axis R and let the EKF filter. If a pre-filter is wanted, the smallest useful one is EMA a = 0.5 or MA-3 (33 ms delay); add that delay to EKF2_EV_DELAY. Check that the motor vibration frequency is not near a multiple of the frame rate.

### estimator: verified OK

- Cloud shadow (60 m soft disc, 55 % darker, 8 m/s) and a sweeping shadow edge at 100 m: RMS 0.39 m / 0.40 m, 0 confident-wrong fixes (e4_outliers.py)
- 3 m repeated grid of bright dots at 100 m: no period slip, RMS 0.31 m (e4_outliers.py)
- Real climb 10 m/s from 100 to 150 m: 1 keyframe change, max error 1.31 m (e4_outliers.py)
- Baro +10 m glitch for 2 s: 61 frames 'degraded', max error 1.05 m (e4_outliers.py)
- The forward-backward check rejects wrong matches on real texture: 0 % pass at +1.5 px and +3 px (e4b_fb_check.py). My first suspicion that it was ineffective is disproved.
- 120 s hover with 5 m wander and the sim's attitude model: error bounded, no growth, 0 keyframe changes; inliers decay only 232 -> 179 (e7_long_hover.py, e7_long_hover.png)
- The velocity EMA is numerically stable and continuous across keyframe changes (e5_velocity.py)
- With a re-anchor after 'lost' (sim_test behaviour), fixes return 0.18 s after light returns, and nothing is sent on std-3 static (e5_velocity.py, e8_texture.py)
- The hold works at 6 and 15 fps in the sim with 99-100 % of fixes sent (e9b_low_fps.py)
- Ground contrast 25-100 % with video noise 5-10: accuracy unchanged (e8_texture.py)
- A keyframe handover cancels the switch frame's own attitude error; the measured per-change sigma is about sqrt(spread^2/n) (e3_keyframes.py B)
- Proposed variance model honest.py on 102 held-out runs: false-fix 0.7 %, 50.0 % outside the 50 % bound, NEES 1.92 (e1b_honest_var.py)
- Calmer keyframe rule (KfRuleHold): prairie 50 m changes 191 -> 59, RMS 1.29 -> 0.56 m (e3_keyframes.py C)
- Grid-bucketed corners fix the static-OSD case (RMS 8.8 -> 0.32 m) without hurting the baseline (e4_outliers.py)
- The baseline sim_ground ottawa2019fall run reproduces the known 0.46 / 0.76 / 1.37 m at 100 / 200 / 300 m (baseline_sim_ground_ottawa2019fall.txt)

### estimator: not checked

- Real EKF2/EKF3 innovation gating, resets and fallback when fed the frozen-frame or static fixes. The closed-loop consequence is inferred, and only a linear Kalman filter plus PX4-style cascade was modelled (e6_jitter.py).
- Geometry conventions shared by the simulator and the estimator (euler_R, R_BC_NADIR, fisheye model, Justice A vs D lens): a shared error cancels and stays hidden.
- Real FC attitude-error spectra (the sim uses white 0.1° noise plus a constant bias; I added only a 0.2° Gauss-Markov wander)
- Rolling-shutter readout-time error, analog field/interlace timing and capture timestamp jitter
- Vibration aliasing when the motor frequency is near a multiple of the frame rate (aliases are visible in the PSD but not tested)
- Timing on the actual i.MX8M Plus: only a Geekbench-based scale factor of about 11x single-core; the Python overhead on an in-order core is not measured
- Thread safety of hold.Series between the capture and MAVLink threads, and the unwritten apb_main.py (how var is sent: trace or per-axis)
- Cause of the per-point scatter of about 2 px (spread 0.8-1 m at 50-100 m), far above the code comment's ~0.3 % of height
- Wind, drone dynamics and closed-loop interaction of estimate and control in the image simulator
- Small moving objects (single cars, people) at low height, and moving grass from prop wash
- Per-experiment outputs are in review/estimator/*.txt.

# Lens: geometry

## G1 [critical] Camera mount is a hard-coded guess; a 180 deg, 90 deg or mirrored mount causes a fly-away that no simulator can show

- Where: hold.py:17-19 (R_BC_NADIR), used at hold.py:118; sim_test.py:28 and sim_ground.py:49 render with the same matrix
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t2_mount_closed_loop.py && ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t1_rays_independent.py`

**Problem.** R_BC_NADIR is correct only for 'image top toward the nose, picture not mirrored'. The organizers mount the Justice D, and nobody has confirmed that orientation. If the camera is turned 180 deg, the estimated move is -1 x the true move. If it is turned 90 deg, the estimated move is rotated by 90 deg. If the picture is mirrored (an OSD option on many analog cameras), one axis has the wrong sign. In each case the position loop has positive feedback. The tilt compensation also adds the tilt instead of removing it: 10 deg of pitch at 100 m gives a fake move of 32 m. Hold shows 'degraded' or 'lost' but continues to send fixes. Nothing in the code checks the mount.

**Evidence.** t2 (closed loop with full images, 50 m, start 1.1 m off). True distance from the hold point at 2/4/6/8/10/12 s:
  top->nose (code assumption): 0.6 0.2 0.1 0.0 0.0 0.0 m
  top->tail (180 deg): 3.7 10.5 20.4 32.1 45.0 58.8 m (ok 34, degraded 162, lost 146)
  top->right wing (90 deg): 2.2 9.1 17.6 26.8 38.3 49.0 m
  top->left wing (-90 deg): 2.6 10.2 19.3 29.6 43.5 57.8 m
  mirrored L/R: 3.1 8.3 14.6 21.6 27.3 32.4 m
t1 part D, estimated-move matrices: 180 deg [[-1,0],[0,-1]]; 90 deg [[0,1],[-1,0]]; mirrored, eigenvalues (+1,-1). Pitch +10 deg with no move gives a reported move of 32.03 m (180 deg) and 22.77 m (90 deg) at 100 m.

**Suggested fix.** 1. Make the mount a config value: rotation 0/90/180/270 plus mirror.
2. Before the first flight, do event-day tests T2 and T3. Remove the propellers. Hold the lens 1.50 m above a textured floor and feed Hold a fixed 1.50 m height.
   - T2: move the drone 1.0 m toward the nose, then 1.0 m toward the right wing. The fix must move 1.0 +/- 0.1 m along each direction, and sideways by less than 0.1 m. If the test fails, do not fly.
   - T3: keep the lens over one mark. Tilt the nose down 15 deg, then the right wing down 15 deg. The fix must move less than 0.10 m. A wrong mount gives about 0.80 m.
3. Add a runtime guard. In the first seconds of flight, correlate the EV velocity with the acceleration that the FC tilt commands. If the correlation is negative or rotated, stop all EV output.

## G2 [high] K scale (focal length) for the APB 360x240 field is not verified; a 2.5 % error gives about 1.2 m RMS at 300 m

- Where: hold.py:32-40 (load_lens); sim_test.py:25 (the same K for render and estimate)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t3_focal_scale.py`

**Problem.** When the drone tilts, each pixel shift is converted to an angle with fx and fy. A scale error eps leaves an angle error of eps x tilt. The position error is about h * eps * (tilt now - tilt at the keyframe), and it does not cancel against the keyframe.
The calibration is from a Justice A, through a USB grabber, at 960x720. load_lens assumes that the APB field shows the same picture extent. These differences can each change the scale by 2-3 % or more:
- Analog sampling: 720 samples per line against about 704 active samples.
- The grabber's crop or overscan.
- PAL (288 rows) against NTSC (240 rows).
- Justice A against Justice D.
Both simulators render and estimate with the same K, so they cannot show the error.

**Evidence.** t3: attitude, height and row timing are exact. The tilt wobble is +/-6 deg (RMS 4.5 deg). Only K differs.
  h 300 m: eps 0 -> rms 0.04 m; +1.0 % -> 0.46 m; +2.5 % -> 1.15 m (worst 2.18); -2.5 % -> 1.28 m (worst 2.40)
  h 100 m: eps 0 -> 0.02 m; +1.0 % -> 0.15 m; +2.5 % -> 0.37 m; -2.5 % -> 0.43 m

**Suggested fix.** 1. Calibrate the lens again through the APB capture path, at the field size that Hold receives (for example, a checkerboard captured on the APB).
2. On the event day, do test T6. Hold the camera level 1.50 m above two marks 1.00 m apart. The pixel distance must be 126.4 px along x and 117.0 px along y, +/- 1 px. A 2.5 % error gives 129.6 px and 119.9 px.
3. Check that the image size is 360x240. If it has 288 rows, the signal is PAL and K must be rescaled.

## G3 [high] A yaw or baro error change since the keyframe is multiplied by the centroid of the tracked points (up to 156 m), not by the hold displacement

- Where: hold.py:192-200 (position = mean of C_i = G_i - g_i); hold.py:209-210 (the variance has no term for this)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t5_yaw_baro_lever.py && ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t5b_similarity_fix.py`

**Problem.** Derivation:
- Yaw. With yaw error delta at the keyframe and delta' now, the mean of C_i has the error (R(delta) - R(delta'))*c + (R(delta') - I)*d. Here c is the mean camera-to-point ground offset at the keyframe, and d is the true move since the keyframe.
- Baro. With height error e at the keyframe and e' now, the error is (e/h)*d + ((e - e')/h)*c.
In a hold, d is a few metres. But c includes the keyframe tilt (h*tan(theta)) and any asymmetry in where the features are. Water, a car park or a shadow on one side moves c to about a quarter of the footprint (156 m at 300 m). The event site is beside the Ottawa River.
Without GNSS, the FC yaw drifts on gyro bias, or it follows the motor-current magnetometer error. This is likely about 1 deg over a keyframe's life.
The covariance contains only HGT_ERR/h*|d|, so these fixes look confident. Both simulators feed a constant yaw bias, so they never show the error.
With a keyframe change, the error that remains is (R(delta2) - I)(c2 - c1). It cancels if the new keyframe sees the same features. It stays when the change happens because the old points left one side of the image, so the error then does a random walk.

**Evidence.** t5, still hover, exact except for the injected FC error:
  h 300 m, water on the west half: |centroid| 156.0 m. Yaw error 0->1 deg gives 2.52 m (predicted 2.72). Baro error 0->1 m gives 0.49 m (predicted 0.52). No FC error gives 0.02 m.
  h 100 m, same shore: |centroid| 52.0 m. Yaw 1 deg gives 0.84 m. Baro 1 m gives 0.49 m.
  h 300 m, textured everywhere: |centroid| 32.6 m. Yaw 1 deg gives 0.34 m.
  A constant yaw error of 5 deg with a 5 m move gives 0.44 m (predicted 0.44). With 15 deg it gives 1.31 m (predicted 1.31).
  Keyframe change in a still hover: 0.18 m, against 0.23 m with no change. Here c2 = c1, so the error mostly cancels, as the derivation predicts.
t5b fix (2-D similarity fit), 300 m shore: yaw 1 deg goes from 2.59 m to 0.04 m. Baro 1 m goes from 0.50 m to 0.04 m. With no error it is 0.03 m before and 0.04 m after.

**Suggested fix.** 1. Replace 'pos = mean(C_i)' with a 2-D similarity (Umeyama) fit. Find C, s and phi that minimise sum |G_i + vel*(tref - t_i) - (C + s*R(phi)*g_i)|^2 over the inliers. Then pos = C. The code is in t5b_similarity_fix.py (HoldSim.process).
2. Send phi to the FC as a visual EV yaw, so that the FC heading also stops drifting.
3. If you keep the mean, add (dpsi^2 + (dh/h)^2) * |c|^2 to the variance.

## G4 [high] t0 must be the mid-exposure time of row 0; auto exposure moves it by several ms, which gives 0.75-1.8 m RMS at 300 m

- Where: hold.py:116 and hold.py:166 (docstring: 't0: time the first image row was read'); sim_test.timing_test; the latency sweep in apb_main (not written yet)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t6_lens_field.py`

**Problem.** Each row gets the attitude at its read time. The correct time is the middle of that row's exposure: read time - exposure/2.
Analog FPV cameras use auto exposure. Indoors the exposure is about 1/60 s, so the shift is about 8 ms. In daylight it is about 1 ms, so the shift is about 0.5 ms. A latency calibrated with the indoor hand-rotation bench test can therefore be wrong outdoors by several ms.
The error is about h*omega*dt per frame, and the keyframe keeps its own share. The simulators hide this. The net timing error in sim_test is only FRAME_T_ERR - ATT_LAG = 2 - 3 = -1 ms, and it never changes.

**Evidence.** t6 (c), exact except for t0:
  h 300 m: t0 off by 2 ms -> rms 0.75 m (worst 1.43). Off by 5 ms -> rms 1.83 m (worst 3.52).
  h 100 m: 2 ms -> 0.25 m. 5 ms -> 0.62 m (worst 1.19).
t8: sim_test at 300 m with the net timing error at -3 ms instead of -1 ms goes from 1.02 to 1.74 m.

**Suggested fix.** 1. Lock the camera exposure if the OSD allows it, and turn off WDR.
2. If you cannot lock it, do the latency and readout sweep outdoors in the flight light (event-day test T5). Rock the drone +/-15 deg at 1-2 Hz for 20 s. Sweep the latency from 0 to 150 ms and the readout from -35 to +35 ms.
3. Do the sweep again on the flight log.
4. Change the docstring so that t0 means 'mid-exposure of row 0'.

## G5 [high] Wrong claim: 'RMS 1.0-1.3 m at 300 m' measures the assumed input errors, not the method

- Where: Known simulator results; sim_test.py:21-23 (ATT_BIAS, ATT_NOISE, ATT_LAG, FRAME_T_ERR)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t8_sim_budget.py`

**Problem.** With exact inputs the geometry gives 0.04 m RMS at 300 m. The sim number is the sum of assumed inputs:
- attitude noise 0.1 deg
- a roll/pitch bias of 0.58 deg with a +/-10 deg yaw swing
- a net timing error of -1 ms
- a yaw error that never changes
The real values of these inputs are not measured. The error sources that the simulators never vary (K scale, yaw or baro wander x centroid, exposure timing, mount tilt) each add 0.5-2.6 m at 300 m. Later work (choice of hold height, the pitch) may build on the 300 m number.

**Evidence.** t8, sim_test's own 300 m hover:
  as shipped: 1.02 m
  net timing 0 (FRAME_T_ERR = ATT_LAG): 0.84 m
  net timing -3 ms: 1.74 m
  roll/pitch bias 0: 0.90 m
  timing 0 and bias 0: 0.68 m (the remainder is about the 0.1 deg attitude-noise floor)
With exact inputs, my renderer feeding Hold gives 0.04 m at 300 m and 0.02 m at 100 m (t3 and t4 baselines).

**Suggested fix.** Do not quote the 300 m number as a capability. Report an error budget:
h*(att_noise (+) eps*tilt (+) omega*dt (+) b*dpsi) (+) |c|*(dpsi (+) dh/h)
Measure each term on the event day (tests T5, T6 and T11, and the FC yaw drift in a 60 s static log). Add yaw drift, a K scale error, exposure-dependent timing and a mount tilt to the simulators as separate knobs.

## G6 [medium] Readout direction is assumed top-to-bottom; a reversed sensor is worse than no rolling-shutter model, and the sweep must allow a negative readout

- Where: hold.py:116 (t = t0 + row/rows*readout)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t4_readout_direction.py`

**Problem.** If the sensor reads bottom-to-top, or the ISP flips the picture in memory, every row time is wrong by up to +/-readout. Most of the mean cancels, because the errors are antisymmetric about the middle row. The yaw rate and one-sided point sets do not cancel.
Hold already handles the reversed case exactly with readout = -R and t0 = the read time of the top row. Only the sweep must search negative values.
A camera turned 180 or 90 deg does not change the timing formula, because rows are counted in image coordinates. Only R_bc must change.

**Evidence.** t4 at 300 m: correct model rms 0.04 m. Reversed sensor with the code top-first: 0.39 m (worst 1.10). Reversed with a global model: 0.23 m. Reversed with readout -16 ms: 0.04 m.
At 100 m: 0.02 / 0.13 / 0.09 / 0.02 m.
t1 part C: per-point errors up to 4.04 m at 100 m and 1.5 rad/s. The mean error is 0.56 m with a yaw rate. Readout -16 ms gives an error of 0.0000 m.
t1 part B: with a 180 or 90 deg mount and the matching R_bc, the errors are 0.0000 m.

**Suggested fix.** Let the apb_main sweep search readout values from about -35 ms to +35 ms. A field from a 30 Hz sensor can take nearly 33 ms. Store the sign in the config. Event-day test T5 gives the sign. At 1.5 m and 1.5 rad/s, the skew is about 4.7 px, which the sweep can resolve.

## G7 [medium] A constant roll/pitch bias (FC level trim or camera mount tilt) does not cancel when the drone yaws

- Where: Hold has no boresight term (R_bc is exact nadir); sim_test.py:22 ATT_BIAS
- Simulator can catch it: True
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t7_tilt_bias_yaw_swing.py`

**Problem.** A fixed body-frame tilt b moves every ground offset by about h*b in a body direction. Against the keyframe, this cancels only while the yaw stays the same. The error is h*b*2*sin(dpsi/2).
sim_test already contains 0.58 deg of this with a +/-10 deg yaw swing. A real bracket plus FC level trim can reach 1-2 deg.

**Evidence.** t7 at 300 m with a +/-10 deg yaw swing: bias 0 -> 0.05 m. Bias (0.5,-0.3) -> 0.47 m (worst 0.75). Bias 1.5 deg -> 1.13 m (worst 1.67).
With bias (0.5,-0.3) and no yaw swing: 0.05 m.
At 100 m: 0.16 m and 0.38 m.

**Suggested fix.** Do event-day test T11:
1. Hang a plumb line from the lens to a floor mark, with the lens 1.5 m above the floor.
2. Run Hold._rays on the pixel of that mark, with the live attitude.
3. The horizontal offset must be less than 0.026 m (1 deg, which is 3.4 px in x and 3.2 px in y). If it is larger, add the tilt to R_bc.
You can also estimate the boresight in flight from a slow 360 deg yaw at hover.

## G8 [medium] An analog field is a line decimation: load_lens's half-pixel terms leave the field parities 0.25 row off, and alternating parities jump by 0.5 row

- Where: hold.py:37-38 ((c + 0.5)*s - 0.5)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t6_lens_field.py`

**Problem.** (c + 0.5)*s - 0.5 is correct for an area resize. A field row r is frame line 2r (even field) or 2r+1 (odd field). The correct value for each parity is cy_f = ((cy_c + 0.5)/s - 0.5 - p)/2, with s = H_calib/(2*H_field). This is +/-0.25 row from load_lens.
A constant parity is only a small boresight error. If apb_main feeds both fields (60 Hz), the image jumps by 0.5 row (2.74 mrad) on every frame.

**Evidence.** t6 (a): fy = 182.3 px/rad. A 0.5 row jump is 0.27 m at 100 m and 0.82 m at 300 m.
Full images at 300 m: one parity gives rms 0.04 m. Alternating parities give rms 0.75 m, with a median frame-to-frame jump of 1.08 m.
At 100 m: 0.25 m and 0.36 m.

**Suggested fix.** Use one field parity only, or set cy for each parity with the formula above. Do event-day test T7: in 2 s of a static scene, the frame-to-frame vertical shift must be less than 0.05 row and must not alternate by 0.5 row. Also check whether the USB grabber that made the calibration deinterlaced with weave or bob.

## G9 [low] load_lens ignores the 'projection' field and sends the Brown 'frame' profile through cv2.fisheye

- Where: hold.py:34
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/geometry/t6_lens_field.py`

**Problem.** If someone selects 'Round 2 Metashape Frame' (projection 'frame', k1 = -0.30), cv2.fisheye reads its coefficients as an equidistant model, and every ray angle is wrong.

**Evidence.** t6 (b): with 'Round 2 independent k2', a 40.0 deg ray reads as 40.0 deg. With 'Round 2 Metashape Frame', the same ray reads as 48.1 deg.

**Suggested fix.** In load_lens, assert p['projection'] == 'equidistant_fisheye'. Also reject a nonzero p1, p2 or b2, which cv2.fisheye drops.

### geometry: verified OK

- euler_R (hold.py:48-56): every entry equals Rz(yaw)*Ry(pitch)*Rx(roll), body FRD to NED, MAVLink ATTITUDE ZYX. The max difference from a quaternion built from the physical meaning of the angles is 1.6e-15 over 2000 random attitudes. Pitch +10 puts the nose up (D -0.174). Roll +10 puts the right wing down (D +0.174). Yaw +90 puts the nose east. det = 1 (t0_conventions.py).
- R_BC_NADIR: columns are camera x = body right, camera y = body back, camera z = body down, det +1. It equals the mount built from 'image top toward the nose' with x = y cross z. 'c @ R_bc.T' equals R_bc @ c (t0).
- Simple case: level, nose north, h 10 m, camera moves 1 m north. The ground point moves DOWN the image by +18.2 px. With yaw 90 it moves right by +19.6 px. The code reads 'down' as a move north, which is correct (t0).
- Ground offset g = h*b_xy/b_z = ground point minus camera. C = G - g. The MAX_OFF_NADIR test uses the angle from the vertical.
- _new_kf G = pos + vel*(t - tref) + g, and process C = G - g + vel*(tref - t). Both are exact for a constant-velocity camera (6 m/s north, 6 m/s east with yaw 40, 4 m/s with a pitch rate): the G error is 0.0000 m and the pos error is 0.0000 m (t1 part A).
- Independent test (item 8, t1_rays_independent.py): my quaternions, mount, fisheye and rolling-shutter projector put known ground points into pixels, and Hold._rays recovers them. Cases: level, roll +12, pitch +12, yaw +60, (8,-10,135), north 6 m/s, east 6 m/s, roll rate 1.5 rad/s, pitch rate -1.5 rad/s + north, yaw rate 1.5 rad/s. The worst per-point error is 0.002 cm and the row-time error is 0.000 ms.
- Full-image independent pipeline (my renderer feeding Hold.process) with exact inputs: rms 0.02 m at 100 m and 0.04 m at 300 m (t3 and t4 baselines).
- Row-time formula: row 0 is read at t0 and row 239 at t0 + 239/240*readout. tref is half a row (33 us) late, which does not matter. sim_test.run gives the global model a mid-frame stamp (t0 + FRAME_T_ERR + READOUT/2), which is a fair comparison. The rows used for timing are raw distorted rows, which is correct.
- A camera turned 180 or 90 deg needs no change to the timing formula, only to R_bc (t1 part B: 0.0000 m errors). Hold accepts a negative readout, which models a bottom-to-top sensor exactly (t1 part C, t4).
- load_lens: the scaling and half-pixel terms are correct for an area resize. The same ray has the same (u+0.5)/w and (v+0.5)/h at 960x720 and 360x240 (t0).
- Fisheye model: Metashape's fisheye (r0 -> atan(r0) = theta, polynomial in theta) and OpenCV fisheye (polynomial in theta) are the same model for the selected profile (p1 = p2 = b2 = 0; fx != fy covers B1). My forward model equals cv2.fisheye.projectPoints to 1e-13 px. The undistortPoints round trip is exact up to 60 deg off axis.
- Height derivation (item 6): a constant baro offset dh gives the error (dh/h)*d, which the HGT_ERR term covers. Terrain relief z gives offsets k*g_true with k = h/(h-z) at any tilt, so C_est = C_kf + k*d. The relief error is (z/(h-z))*d + z*(h_now - h_kf)/h^2*g. Example: a 20 m roof, dh 5 m, g 150 m and h 100 m give 1.5 m for those points only. A lever arm L of about 5 cm gives |dR*L| < 3 cm. Hold reports the camera position, so set the EV lever arm in the FC. Earth curvature is 1 cm at 380 m.
- Yaw derivation (item 7): a constant yaw error only rotates the move from the anchor, error 2*sin(d/2)*|d| (t5: 0.44 m for 5 deg x 5 m, predicted 0.44). A yaw error change dpsi since the keyframe gives dpsi*|c| (finding 3). With a keyframe change, the remaining error is (R(d2) - I)(c2 - c1).
- The 2-D similarity-fit fix removes the yaw and baro lever and adds no noise (t5b: 2.59 -> 0.04 m, 0.50 -> 0.04 m, 0.03 -> 0.04 m).

### geometry: not checked

- Conventions the simulators cannot catch, because the simulators share them with the estimator, and the event-day test for each. Setup for all tests: remove the propellers, hold the lens 1.50 m above a textured floor, and feed Hold a fixed 1.50 m height.
- T1 ATTITUDE: nose down -> pitch decreases. Right side down -> roll increases. Turn clockwise seen from above -> yaw increases. The values are in radians.
- T2 mount and mirror: a 1.0 m move toward the nose, then toward the right wing, gives 1.0 +/- 0.1 m along each direction.
- T3 tilt: tilt 15 deg over a fixed mark. The fix moves < 0.10 m. A wrong mount gives 0.80 m.
- T4 yaw sign: turn 90 deg clockwise, then move 1 m north. North increases by 1.0 +/- 0.1 m.
- T5 latency, readout sign and exposure: do this outdoors. Rock the drone +/-15 deg at 1-2 Hz for 20 s. Sweep the latency from 0 to 150 ms and the readout from -35 to +35 ms.
- T6 K scale: two marks 1.00 m apart give 126.4 px (x) and 117.0 px (y), +/- 1 px. The image size must be 360x240, not 288 rows.
- T7 field parity: the shift between frames of a static scene must be < 0.05 row.
- T8 height: lift 1 m, and the height feed increases by +1.0 m (LOCAL_POSITION_NED.z is positive down; relative_alt is in mm).
- T9 output axes: a move north increases LOCAL_POSITION_NED.x.
- T10 clock: the latency from T5 is positive and between 20 and 120 ms.
- T11 boresight: Hold._rays on the pixel of the plumb-line mark gives < 0.026 m.
- Report content is in this file.
- apb_main.py does not exist. These are unknown: the field selection and parity, the resize method (area or decimation, which gives a -0.25 px column offset), the timestamp source, the FC clock sync, the EV message type and frame, and the N/E axis order.
- Which firmware the M2-AI FC runs (PX4, ArduPilot or Orqa), and which attitude message reaches the APB (ATTITUDE, ATTITUDE_QUATERNION or MSP, which has different units and signs).
- Justice D facts: NTSC or PAL, readout time, exposure range, WDR, and the OSD mirror/flip settings. Whether its lens equals the Justice A that was calibrated.
- How the 960x720 USB capture maps to the APB decoder's active picture (crop, overscan, 704 or 720 samples). Whether the grabber deinterlaced with weave or bob.
- How the FC EKF behaves with a wrong-signed EV (innovation gating and reset to EV). t2 assumes that the controller follows the EV directly.
- Real terrain relief in images (my renderer uses flat ground; relief is a derivation only), motion blur, vibration aliasing in the rolling shutter, wind and drone dynamics, and LK tracking quality on real ground. These are outside the geometry lens.
- Non-geometry logic in Hold: inlier thresholds, state machine, jump gate, LOST_AFTER, covariance tuning, and Series thread safety.

# Lens: hardware_gap

## H1 [critical] Frame-to-attitude time offset is unmeasured; apb_main.sweep does not exist (20-40 ms errors give 4-8x error open loop and 8-42 m excursions closed loop)

- Where: hold.py:165-166 (Hold.process t0 contract); sim_test.py:21-23,81 (FRAME_T_ERR = 2 ms assumed); sim_test.py:132-149 (timing_test needs apb_main.sweep)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t1_timing.py && ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t6_closed_loop.py && ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t_bench_calib.py`

**Problem.** Hold needs t0 = first-row read time in the attitude clock. Every sim result assumes a 2 ms error. On the board, the frame stamp comes from a V4L2 analog capture: exposure 0-20 ms, field readout ~16 ms, second field +16.7 ms, ISP 0-1 field, USB grabber ~1 frame, driver stamps at start or end of frame, OpenCV queue 0-N frames. Attitude comes from PX4 time_boot_ms (1 ms truncation) over a UART with 0-65 ms TX queue, and the FC and i.MX crystals drift 1-3 ms per minute. Without a measurement, the first-day guess will be 20-40 ms off. The covariance does not grow with this error, so 89-99 % of fixes fall outside the 99 % bound.

**Evidence.** t1_timing.py (Ottawa photo, sim hover; roll/pitch rate 24.8/25.8 deg/s RMS, 60 deg/s peak). 100 m: 0 ms 0.34 m RMS; +10 ms 0.74 m; +20 ms 1.51 m (false-fix 89 %); -20 ms 2.88 m (99 %); +40 ms 2.87 m; random +-16.7 ms per frame (stamp at read()) 0.98 m, 89 % sent. 200 m: 0 ms 0.82; +10 ms 1.80; +20 ms 3.11; -20 ms 6.24; +40 ms 6.36; random 2.42 m. Rule of thumb h x w_rms x dt = 1.25 m at 100 m / 20 ms. Closed loop t6_closed_loop.py (toy PX4-like cascade, alpha-beta EKF stand-in, 100 m): nominal max 0.5 m; stamp 20 ms early max 8.0 m; stamp 40 ms late max 42.1 m (FLY-AWAY > 20 m). t_bench_calib.py: reference sweep passes the timing_test assertion: 'sweep found latency 58 ms (expected 57), readout 18 ms (true 16) -> timing_test assertion PASSES'; far scene with 0.2 m hand pivot: latency 70-72 ms (true 72), readout 13-17 ms (true 16).

**Suggested fix.** Write apb_main.sweep to this spec (working reference: review/hardware_gap/bench_calib.py). Signature sweep(path, R_bc) -> (latency_s, readout_s), latency = t_grab - t_first_row, so live t0 = t_grab - latency. Recording .npz: imgs exactly as fed to Hold (same field split and size); t_grab made by the SAME live stamping code; att [t, roll, pitch, yaw] mapped to the companion clock by the SAME live mapping; K, D. Tracking: keyframe tracks (LK from the previous frame, refined to the keyframe, forward-backward check < 0.5 px, new keyframe after 60 frames or < 25 points). Frame-to-frame pairs fail: they see only the change of rotation rate (first version missed by 9 ms and 16 ms). Cost: median angle between the NED rays of each match in keyframe and frame, each ray rotated with the attitude at its own row time. Search latency 0-250 ms step 10 x readout 0-40 ms step 5, refine +-8/+-4 ms step 2. Also try all 8 picture orientations and fail if the best is not R_bc; refuse recordings under 20 deg/s RMS or with fewer than 2 axes. Note lat - ro/2 is the well-measured quantity. Event-day procedure (5 min, no gear): 1. Remove the propellers. 2. Power the drone. Make sure the FC streams attitude. 3. Start apb_main --record 20. 4. Hold the drone on its side so that the camera points at buildings or trees more than 50 m away. 5. Turn the drone by hand for 20 s, +-15-20 deg about all three axes, at about 0.5 Hz. 6. Run sweep. 7. Do steps 3-6 again. The latencies must agree within 3 ms. Live code: fit the FC-to-companion clock offset and drift continuously (minimum filter on arrival - time_boot), keep stamps monotonic (np.interp in Series.at/attitude_R silently accepts non-monotonic stamps), stamp frames with the V4L2 driver timestamp, not time at read(). Add a timing term (h x attitude rate x timing sigma) to the variance.

## H2 [critical] PX4 v1.15.4 defaults will reject, corrupt or double-count the external-vision fix (EV yaw on, GNSS height, arrival-time stamps, NED needs mag, Hold mode needs global position)

- Where: apb_main.py (not written) and the APB PX4 parameters; PX4 v1.15.4 ekf2/module.yaml, mavlink_receiver.cpp:1245-1261, Timesync.cpp:124-133, ev_pos_control.cpp:67-82,219, ev_yaw_control.cpp:147-149, common.h:70, mode_requirements.cpp:103
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t11_px4_facts.py`

**Problem.** 1) EKF2_EV_CTRL default 15 fuses EV yaw from the VISION_POSITION_ESTIMATE roll/pitch/yaw. yaw=0 resets the EKF heading to 0, which rotates the frame Hold itself uses. Sending the FC's own yaw fuses the EKF's own output. 2) VPE is always NED. EKF2 fuses a NED EV position only after yaw alignment, which with GNSS off needs a working magnetometer (QMC5883L). No alignment means no fusion. 3) EKF2_GPS_CTRL 7 and EKF2_HGT_REF 1 (GNSS): with GNSS degraded but connected, EV becomes a biased relative aid with variance floored at the GNSS noise, and the height that scales every ray comes from GNSS. 4) VPE stamp = sync_stamp(usec): arrival time until 500 TIMESYNC exchanges with RTT < 10 ms. A 460 B stream burst on this link already takes 20 ms, so convergence is uncertain. EKF2_EV_DELAY default 0, so 60-300 ms of pipeline delay goes uncompensated, and if timesync converges mid-flight the stamp meaning changes. 5) AUTO_LOITER (Hold) requires global position, so it is unavailable without GNSS or an origin. 6) EV samples more than 200 ms apart do not start fusion. 7) After 1 s without fusion EKF2 resets to EV, at most 5 times in the air, then drops EV. Position goes invalid after 5 s and Position mode falls back to Altitude mode, which drifts with the wind.

**Evidence.** t11_px4_facts.py output: 'EKF2_EV_CTRL default 15', 'EKF2_EV_DELAY default 0', 'EKF2_HGT_REF default 1', 'EKF2_GPS_CTRL default 7', 'EKF2_DELAY_MAX default 200'; 'mavlink_receiver.cpp:1245: odom.timestamp_sample = _mavlink_timesync.sync_stamp(vpe.usec);' '1247: odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;'; 'Timesync.cpp:131: return hrt_absolute_time();' 'Timesync.hpp:76: CONVERGENCE_WINDOW = 500' 'Timesync.hpp:88: MAX_RTT_SAMPLE = 10_ms'; 'ev_pos_control.cpp:68: if (_control_status.flags.yaw_align) {' ... '80: continuing_conditions_passing = false;'; 'ev_yaw_control.cpp:149: resetQuatStateYaw(...)'; 'common.h:70: EV_MAX_INTERVAL = 200e3'; 'mode_requirements.cpp:103: ...AUTO_LOITER, flags.mode_req_global_position'. Not provable in the simulators; derivation from source.

**Suggested fix.** Parameters: EKF2_EV_CTRL 1 (horizontal position only), EKF2_GPS_CTRL 0, EKF2_HGT_REF 0 (baro). Send VPE z, roll, pitch, yaw = NaN, covariance[0] = covariance[6] = var/2. Time: either answer TIMESYNC in apb_main (and make sure no other process on mavlink-router answers) and stamp usec in the companion clock, or do not answer and set EKF2_EV_DELAY to the measured total delay (raise EKF2_DELAY_MAX if over 200 ms). Fly Position mode, not Hold. Keep reset_counter constant (Hold already has no jump at keyframe changes). Ground detection before take-off: listener estimator_status_flags (cs_ev_yaw 0, cs_yaw_align 1, cs_gps 0, cs_baro_hgt 1); heading within 10 deg of a phone compass; listener vehicle_visual_odometry (rate >= 10 Hz, timestamp - timestamp_sample equals the expected delay); bench EV test: hold the drone 1.5 m above textured ground with a height override, carry it 1 m toward the nose, local x must rise by about 1 m, then 1 m right, local y must rise; listener estimator_aid_src_ev_pos innovation_rejected 0, test_ratio < 1; cover the lens 2 s and kill apb_main: no position jump on return/restart (re-anchor to LOCAL_POSITION_NED).

## H3 [critical] Picture orientation is assumed (image top = nose); a picture turned 90/180 deg or mirrored makes the drone fly away

- Where: hold.py:17-19 (R_BC_NADIR); shared by sim_test.py:28 and sim_ground.py:49, so neither simulator can show it
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t5_mount.py && ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t6_closed_loop.py && ~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t_bench_calib.py`

**Problem.** The organizers mount the camera, and top-toward-tail is as likely as top-toward-nose. The camera menu can also mirror or flip the picture. A turned or mirrored picture reverses or crosses the measured motion and also doubles the tilt error instead of removing it. In a position loop that is positive feedback. Both project simulators render and estimate with the same R_BC_NADIR, so this can never appear there.

**Evidence.** t5_mount.py (open loop, 100 m): nominal 0.32 m RMS gain [[+0.93 -0.07] [+0.09 +1.04]]; mirrored 14.98 m RMS gain [[+0.51 +0.22] [+1.12 -3.94]]; turned 90 deg 16.89 m RMS; turned 180 deg 27.48 m RMS, 63 % sent. t6_closed_loop.py: 'picture turned 180 deg max distance 1556.2 m ... 5s: 34.7' ; 'picture mirrored left-right ... 5s: 27.0' ; 'picture turned 90 deg ... 5s: 37.7' -> all FLY-AWAY (>20 m) within 5 s (toy controller: the sign of the loop, not PX4 numbers). t_bench_calib.py finds each case: 'picture turned 180 -> found turn 180 deg ... residual 0.147 deg vs next-best orientation 8.491 deg', 'mirrored -> mirror True ... 0.161 vs 8.127', 'turned 90 -> found turn 90 ... 0.150 vs 7.882'.

**Suggested fix.** Ground test, under 2 minutes: 1. Show the live picture. 2. Pitch the nose down by hand. The picture must move toward the top edge. 3. Roll the right side down. The picture must move toward the right edge. 4. Yaw the nose to the right. The picture must turn counter-clockwise. 5. If any of steps 2-4 fails, stop and correct R_bc before you fly. 6. Run the 20-second hand-rotation recording and bench_calib.Bench(rec).full_check(). 7. Accept only if the best orientation's residual is less than 1/5 of the next best. In the sim step 6 also gives tilts within 0.7 deg and focal scale within 0.5 %. Make apb_main.sweep refuse to run if the best orientation is not the configured R_bc.

## H4 [critical] A frozen frame or a grabber 'NO SIGNAL' screen is not detected; Hold keeps sending fixes that miss the drift

- Where: hold.py:176-219 (no check that the picture moves or is fresh); hold.py:139-147 (a new keyframe can be made on 'NO SIGNAL' text)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t10_frozen_video.py`

**Problem.** Analog grabbers and V4L2 drivers often repeat the last frame or show a blue 'NO SIGNAL' screen when the signal drops (loose connector, vibration, EMI near a 5 W VTX). LK on an unchanged picture returns zero motion, which Hold reads as 'the camera only tilted', so it reports positions that follow attitude only. With the loop closed, the EKF holds that fake position while the wind moves the drone. Likelihood is moderate, consequence is a slow fly-away.

**Evidence.** t10_frozen_video.py (100 m, drone drifts 0.5 m/s east, video bad 5-10 s): 'frozen frame (same buffer repeated) during the bad video: fixes sent 100% ('ok' 9%, lost 0%); error of the last fix sent 3.77 m (drift in 5 s = 2.5 m)'; 'frozen frame + fresh sensor noise ... fixes sent 100% ... 4.11 m'; 'grabber NO SIGNAL screen ... fixes sent 100% ('ok' 0%, lost 0%) ... 4.47 m'.

**Suggested fix.** Refuse a frame when the median image motion is under 0.1 px while the attitude changed more than 0.3 deg since the previous frame; when the frame bytes or the V4L2 sequence number repeat; when VIDIOC_ENUMINPUT status has V4L2_IN_ST_NO_SIGNAL. On refusal stop sending at once (do not resend the last fix) and require 10 good frames before sending again. Test on the ground: unplug the video for 2 s; the fixes must stop within 0.5 s.

## H5 [high] Attitude reaches the APB at 45 Hz (13.5 Hz with ULog streaming), queued up to 65 ms; frames processed on arrival fail 'attitude late'

- Where: hold.py:124-127 (_check_inputs needs attitude to t0+readout-5 ms); sim_test.py:56-63 (fc_feeds: 100 Hz, perfect stamps, filled before the run); APB rc.board_defaults MAV_0_MODE 2, MAV_0_RATE 0, LOGGER_ARGS -m mavlink; default.px4board UART4_TXBUFSIZE 1500
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t7_attitude_link.py`

**Problem.** PX4 Onboard mode requests ATTITUDE at 100 Hz, but MAV_0_RATE 0 gives 230400/20 = 11520 B/s while the advertised Onboard streams ask about 25600 B/s (ODOMETRY alone 7350 B/s). update_rate_mult() scales every stream by 0.45. ULog streaming over the same link may take 70 % of the budget. Arrival times jitter by the 20 ms stream bursts. The sim fills the attitude series before it runs, so 'attitude late' never fires there; online, a frame processed as soon as it arrives often has no attitude for its last rows yet, and each rejection counts toward LOST_AFTER.

**Evidence.** t7_attitude_link.py A: 'requested bandwidth of advertised streams: 25604 B/s'; 'no ULog streaming rate_mult 0.45 -> ATTITUDE 45.0 Hz'; 'with origin set ... 40.8 Hz'; 'ULog streaming on (70 % share) rate_mult 0.13 -> ATTITUDE 13.5 Hz'; 'UART4 TX buffer 1500 B drains in 65 ms'. B (100 m): 100 Hz 0.34 m, 45 Hz 0.39 m, 30 Hz 0.62 m, 13 Hz 1.10 m, time_boot_ms truncation 0.28 m, 46 Hz stamped on arrival 1.64 m (false-fix 94 %); 200 m: 13 Hz 2.56 m, arrival stamps 2.44 m. C (online): 100 Hz / 20 ms capture latency: fixes sent 3 %; 100 Hz / 35 ms: 82 %; 46 Hz / 20 ms: 0 %; 46 Hz / 35 ms: 49 %; 14 Hz / 60 ms: 50 %; failures all 'attitude late'.

**Suggested fix.** Trim the streams on /dev/ttyS1 in extras.txt (mavlink stream -d /dev/ttyS1 -s ODOMETRY -r 0, same for HIGHRES_IMU, ATTITUDE_QUATERNION, GLOBAL_POSITION_INT) or set MAV_0_RATE 23040; keep 230400 baud (the port repo says 921600 caused link loss). Never stream ULog on this link during the hold. Stamp attitude with time_boot_ms through a continuously fitted offset, not arrival time. In apb_main, queue each frame until attitude covers its last row + 5 ms, with a 100 ms limit, before calling Hold.process. Check on the ground: mavlink status shows tx rate mult 1.0 and ATTITUDE >= 90 Hz at the companion.

## H6 [high] Hold.process takes 29 ms mean on a fast laptop core; the i.MX8M Plus A53 is likely several times slower, so fixes may come at 3-10 Hz and PX4 refuses gaps over 200 ms

- Where: hold.py:178-182 (three LK passes on up to 300 points per frame), hold.py:140 (goodFeaturesToTrack 300), hold.py:73-77 (Series.array rebuild)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t9_cpu.py`

**Problem.** The frame budget is 33 ms (30 Hz) or 16.7 ms (per field). On this laptop one thread already needs 29 ms mean and 39 ms p95. The APB companion is 4x Cortex-A53; public single-core scores suggest about 10x slower per thread (not measured here). At 3-10 Hz, EV gaps approach PX4's EV_MAX_INTERVAL 200 ms (fusion does not start), and a FIFO frame queue grows latency without bound, which feeds finding 1.

**Evidence.** t9_cpu.py (Intel Core 7 240H, cv2.setNumThreads(1), run alone): 'Hold.process 360x240: mean 29.08 ms, p95 38.69 ms, max 109.48 ms (budget 33.3 ms per frame at 30 Hz, 16.7 ms per field at 60 Hz)'; 'one LK pass on 300 points 6.20 ms (process() runs 3 LK passes per frame)'; 'Series.array() rebuild with 4000 rows: 1.80 ms'. A53 factor is an estimate, not measured.

**Suggested fix.** Time Hold.process on the APB before any other tuning. If it is over 30 ms: track about 120 points, winSize 15x15, maxLevel 2, run the backward check every third frame, keep the attitude Series short. Always process the newest frame (V4L2 queue of 2, drop stale frames). Send at >= 10 Hz or PX4 will not start EV fusion.

## H7 [high] Lens profile is a Justice A captured through a different chain; 5-10 % focal or 30 px principal-point error costs 2-5x

- Where: hold.py:32-40 (load_lens assumes the APB image shows the same picture extent as the 960x720 UVC capture); sim_test.py:25 and sim_ground (true and model lens are the same, so the sims cannot show it)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t4_lens.py`

**Problem.** The Round 1 profile came from a UVC receiver at 1280x720 cropped to 960x720. The APB's analog decoder samples and crops overscan its own way. The brief says Justice D, not A, and Orqa Justice cameras switch 4:3/16:9 (about 25-33 % horizontal scale). A focal error also scales the tilt correction, so the error is height x tilt x focal error on top of displacement x focal error.

**Evidence.** t4_lens.py (5 m wander). 100 m: exact 0.30 m; focal +-2 % 0.36-0.45; +-5 % 0.67-0.85; +-10 % 1.20-1.69; fy:=fx (square pixels) 0.69; principal point +10 px @960 0.55; +30 px 1.32. 200 m: exact 0.72; +-5 % 1.63-1.73; +-10 % 2.87-3.40; fy:=fx 1.79; +30 px 2.70. True lens 'Nominal Justice' (fx -8 %) 1.33 m; 'Generic 132 deg' (fx -21 %) 3.08 m (false-fix 98 %).

**Suggested fix.** Calibrate on the APB capture chain at the exact size and field mode. 5-minute check: 1. Run the hand-rotation recording. 2. Read the focal scale from full_check (sim: found x0.953 for true x0.95); fit fx and fy separately. 3. If it is outside 0.98-1.02, do step 4 or 5. 4. Show an A3 checkerboard at 0.4-0.8 m in 15-20 positions covering the centre and four corners; run cv2.fisheye.calibrate with k1-k4 fixed at the Round 1 values, fit fx, fy, cx, cy; accept RMS < 1 px. 5. Or put two marks 4.00 m apart, hold the drone level 2.00 m above their midpoint, grab a frame: the marks are 90.0 deg apart; compare with the model angle. Principal point and small mount tilt act the same way on the hold; the rotation check measures their sum.

## H8 [high] A few degrees of mounting tilt or twist (and FC attitude bias) rotate with yaw and grow with height

- Where: hold.py:19 (exact R_BC_NADIR assumed), hold.py:114-122 (_rays); sim_test.py:22 ATT_BIAS acts like a camera tilt
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t5_mount.py`

**Problem.** A constant tilt adds an offset h x tan(tilt) that rotates with yaw and changes with height. A twist about the optical axis leaks roll into pitch (error ~ h x tilt x twist). The FC's own roll/pitch bias acts exactly like a camera tilt, and no rotation-only check can see it. These do not fly away but multiply the error.

**Evidence.** t5_mount.py (100 m): tilt 3 deg 0.71 m with yaw +-10 deg, 2.72 m with yaw +-45 deg; tilt 5 deg 5.24 m with yaw +-45 deg; twist 5 deg 1.26 m (nominal 0.32); nominal mount with yaw +-45 deg 0.67 m (sim FC bias 0.5/-0.3 deg); climb 100->175 m in 15 s: nominal last-second error 0.44 m, tilt 3 deg 2.12 m. t_bench_calib.py finds 'tilt roll +2.33 pitch -2.03' for true +3/-2.

**Suggested fix.** Measure tilt and twist with full_check and put the result into R_bc. Hold the heading constant during the hold (no pilot yaw). Anchor at the final height and do not change height during the judged hold.

## H9 [high] Near-zero or negative baro height makes Hold report moves backwards with state 'ok'; h = 0.0 raises ZeroDivisionError

- Where: hold.py:195 (hm = mean height, no minimum), hold.py:209-210 (self.HGT_ERR / hm), hold.py:122 (rays scaled by h)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t8_height.py`

**Problem.** Relative baro on the pad is near 0 or negative, and in ground effect it reads low by around a metre. Hold scales every offset by the baro height, so the reported move is (baro height / true height) x the true move, negative below zero, while the state stays 'ok'. A Position-mode take-off with EV active can push the drone the wrong way. With a height of exactly 0.0 the program crashes.

**Evidence.** t8_height.py B (true height 3 m, move 2 m east): 'baro +3.0 m: reported/true = +1.02'; '+2.0 m: +0.65'; '+1.0 m: +0.33'; '+0.5 m: +0.16'; 'baro -0.5 m (true 3): reported east move / true move = -0.19; state 'ok' on 93% of frames'; 'baro exactly 0.0 m: Hold.process raises ZeroDivisionError (float division by zero) -> the program dies'.

**Suggested fix.** Do not anchor and do not send below a minimum height (for example 15 m, and at least 20 x the baro noise). Guard hm (return 'lost' if hm < h_min). Take off in Altitude mode and switch to Position mode above the gate.

## H10 [high] Interlaced capture: wrong field order is a 16.7 ms stamp error (5x); woven frame with the 16 ms model is 3x

- Where: sim_test.py:20 (W, H, FPS = one 360x240 field at 30 Hz); apb_main capture code (not written)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t2_video.py`

**Problem.** A V4L2 analog grabber delivers woven 720x480 (NTSC, fields 1/59.94 s apart) or 720x576 (PAL, 1/50 s) frames. The sims never build interlaced frames. Comb artifacts did not break LK at these rates; the timing of each field is what matters.

**Evidence.** t2_video.py A (100 m, 12 s): one field width halved (sim assumption) 0.54 m; woven 720x480 as one picture, readout 16 ms 1.15 m; woven, readout 33 ms 0.32 m; driver blend-deinterlace stamped mid-way 0.39 m; 'field B used, stamped as field A (field order swapped)' 1.80 m (false-fix 94 %).

**Suggested fix.** Read the standard and size from the driver (VIDIOC_QUERYSTD, G_FMT), set W, H, field period and K from them (no hard-coded 360x240/30). Read the field order from v4l2_format.fmt.pix.field; stamp field 2 one field period after field 1. Either split fields with their own t0, or feed the woven frame with a readout model spanning both fields (33 ms NTSC). Turn off driver deinterlacing and scaling. Mask black borders, VBI/head-switching lines and OSD regions (the APB FC runs an analog OSD: atxxxx start -s; check whether the APB captures before or after it).

## H11 [medium] Exposure is a hidden, light-dependent time offset (stamp at readout vs mid-exposure)

- Where: hold.py:116 (row time = t0 + y/rows x readout, no exposure term)
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t3b_exposure_gain.py`

**Problem.** Each row records light over the exposure before its readout. Stamping at readout adds an offset of exposure/2, and auto exposure moves it by up to about 8 ms as light changes. Blur itself was harmless.

**Evidence.** t3b_exposure_gain.py (100 m): exposure 16.7 ms stamped at readout 0.88 m (false-fix 73 %), stamped at mid-exposure 0.23 m; 8 ms: 0.35 vs 0.28 m. Auto gain harmless: +-15 % 0.34 m, +-30 % 0.44 m, step to x0.6 0.50 m, step and back 0.43 m.

**Suggested fix.** Run the timing sweep outdoors in event light so it absorbs the exposure offset. If the Justice D menu allows a fixed shutter, set it short (removes blur and the varying offset).

## H12 [medium] The readout value matters much less than the mid-frame time; findings.md 'a wrong readout value is worse than none' does not hold here

- Where: /home/paul/Videos/findings.md section 8 and open question 3; sim_test.py:21 READOUT
- Simulator can catch it: True
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t3_readout.py`

**Problem.** With the mid-frame time right, any readout model 0-33 ms stays within run-to-run scatter. With the first-row time right, a wrong readout shifts the mid-frame and costs up to 5x. Later work should spend event-day time on the total offset, not on the readout.

**Evidence.** t3_readout.py (true 16 ms). 100 m first-row stamp right: 0 ms 1.70, 8 ms 0.79, 16 ms 0.29, 24 ms 0.44, 33 ms 0.77 m; mid-frame right: 0.34, 0.31, 0.29, 0.27, 0.28 m. 200 m first-row: 2.64, 1.37, 0.68, 0.96, 1.61; mid-frame: 0.78, 0.73, 0.68, 0.64, 0.59 m. Bottom-to-top readout: model +16 0.50 m, correct -16 0.28 m, 0 ms mid 0.46 m. Scatter between runs of the same setup is up to about 1.7x.

**Suggested fix.** Correct findings.md: measure lat - readout/2 (the mid-frame time) with the sweep; the readout itself is second order. Hold supports a reversed readout with negative readout_s and t0 = top-row time.

## H13 [medium] Hold's variance is a two-axis total and is overconfident; with any real-hardware error 60-99 % of fixes fall outside the 99 % bound

- Where: hold.py:209-211 (var), sim_test.py:102 (2 e^2 / var vs chi-square(2) shows var is the 2-axis total)
- Simulator can catch it: True
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t1_timing.py`

**Problem.** apb_main must put var/2 per axis into VPE covariance[0] and [6]. Even nominal fixes are outside the 99 % bound 2-15 % of the time, and the timing, lens, mount and interlace errors push that to 60-99 %. PX4 gates at 5 sigma; repeated rejections lead to 1-second resets and then EV is dropped after 5.

**Evidence.** false-fix column: nominal 1.6-15 % (t1, t4, t5); +20 ms 89 %; focal -10 % 91 %; twist 5 deg 83 %; field order swapped 94 %; arrival-time attitude 94 %.

**Suggested fix.** Send var/2 per axis. Add a timing term (h x attitude rate x timing sigma) and a floor of about 0.5 % of height. Check on the bench: estimator_aid_src_ev_pos test_ratio < 1, innovation_rejected 0.

## H14 [medium] Process and safety are undefined: no start gate, no stale-input stop, restart must re-anchor, and the failsafe is a wind drift

- Where: apb_main.py (not written); hold.py:109-112 (reanchor), hold.py:165-219
- Simulator can catch it: False
- Repro: `none`

**Problem.** Nothing defines what the program does at start, on a crash, on video loss or MAVLink loss. Without position, PX4 Position mode falls back to Altitude mode (COM_POSCTL_NAVL default 0) after EKF2_NOAID_TOUT 5 s, geofence and RTL do not work, and the drone drifts with the wind. A restarted program that starts at (0, 0) makes a position jump.

**Evidence.** Derivation from PX4 v1.15.4 (t11_px4_facts.py) and findings 6 and 9 (t10, t8).

**Suggested fix.** Start: 1. Wait for attitude >= 40 Hz with a fitted clock offset. 2. Wait for frames at the expected size, rate and field order. 3. Refuse 'NO SIGNAL' and frozen frames. 4. Wait for baro height above the minimum. 5. Wait for attitude rate < KF_RATE. 6. Re-anchor to LOCAL_POSITION_NED. 7. Send only after 10 good frames. Crash: systemd restart within 2 s, re-anchor to the FC estimate. Video or MAVLink loss (no attitude for 50 ms): stop sending at once. Pilot: one switch to Altitude/Stabilized, kill switch, no yaw during the hold; take off upwind. Pre-flight checklist (27 min): 1 (2 min) load the parameter file, check EKF2_EV_CTRL 1, EKF2_GPS_CTRL 0, EKF2_HGT_REF 0, EKF2_EV_DELAY, trimmed streams. 2 (2 min) start apb_main in check mode, read standard, size, field order, frame rate, attitude rate, clock offset. 3 (1 min) tilt test (nose down -> picture up; right side down -> picture right; nose right -> picture counter-clockwise). 4 (3 min) hand-rotation recording, full_check, write down latency, readout, orientation, tilts, focal. 5 (2 min) repeat; latency within 3 ms, tilts within 0.5 deg. 6 (3 min) lens check if focal outside 0.98-1.02. 7 (2 min) heading vs phone compass at two headings, EKF flags. 8 (1 min) load the values, restart apb_main. 9 (3 min) bench EV test at 1.5 m with height override: carry 1 m toward the nose, then 1 m right; local x then y rise about 1 m; test_ratio < 1. 10 (2 min) cover the lens 2 s, unplug video 2 s: fixes stop within 0.5 s; kill apb_main: restarts in 2 s with no jump. 11 (2 min) RC switches Position/Altitude/Stabilized/kill. 12 (4 min) take off in Altitude mode, climb above the gate, switch to Position, hold 30 s, check drift, then climb to target height with no yaw.

## H15 [low] Barometric height errors are second order in a hover

- Where: hold.py:120-122 (rays scaled by hgt.at(t)); sim_test.py:61-62
- Simulator can catch it: False
- Repro: `~/.venvs/vio/bin/python /home/paul/Videos/vio-hold-attempt/review/hardware_gap/t8_height.py`

**Problem.** Error = scale error x displacement from the anchor, so with small wander a few percent of baro error costs little. Gust dynamic pressure (10 m/s is about 60 Pa, about 5 m) is still about 5 % at 100 m. The real height risks are the GNSS height source (finding 2) and the low-height case (finding 9).

**Evidence.** t8_height.py A (5 m wander). 100 m: exact 0.35; scale +-2..+-10 % 0.34-0.75; ground 5 m lower 0.32; drift 1 m/min at 5 min 0.56. 200 m: exact 0.70; scale 0.78-1.14; ground 5 m lower 0.96; drift 0.67 m.

**Suggested fix.** Use baro (EKF2_HGT_REF 0) via LOCAL_POSITION_NED.z or ALTITUDE.altitude_local; do not send our own z; keep wander small and anchor at the hold height.

### hardware_gap: verified OK

- R_BC_NADIR matches its comment: image x = body right, image y = body back, optical axis = body down (hold.py:17-19).
- euler_R is the ZYX body-to-NED rotation PX4 uses to fill ATTITUDE (matrix::Eulerf of the attitude quaternion, PX4 v1.15.4 ATTITUDE.hpp:72-76).
- A new keyframe inherits the current position, so the reported position has no step at a keyframe change; no EKF reset_counter change is needed for it (hold.py:151-155).
- A negative readout_s with t0 = top-row time handles a sensor that reads bottom to top (t3_readout.py: 0.28 m).
- The rotation-only hand check recovers latency (within 2 ms), readout (within 3 ms), picture orientation and mirror (residual 0.15 deg vs >= 7.9 deg), tilts (within 0.7 deg) and focal scale (within 0.5 %) in the simulator; it passes the sim_test.timing_test tolerance (t_bench_calib.py).
- LK tracking survives auto-gain hunting of +-30 % and a 40 % brightness step (t3b_exposure_gain.py).
- PX4-style fixed OSD text and black capture borders did not change the drift the hold sees at 200 and 300 m (t2_video.py B).
- Motion blur from a 16.7 ms exposure does not hurt when the stamp is at mid-exposure (0.23 m, t3b).
- A woven 720x480 frame fed as one picture works as well as one field when the readout model spans both fields (0.32 m, t2 A).
- MAVLink time_boot_ms truncation to 1 ms has no measurable effect (t7 B).
- Attitude at 45 Hz instead of 100 Hz costs little at 100 m (0.39 vs 0.34 m) when stamped with time_boot_ms (t7 B).
- Height scale errors up to +-10 % cost at most about 2x with 5 m wander (t8 A).
- PX4 v1.15.4 maps VPE covariance[0], [6], [11] to x, y, z variance and floors it at EKF2_EVP_NOISE 0.1 m (mavlink_receiver.cpp:1259-1261, ev_pos_control.cpp:140-141).
- Harness baseline reproduces the known sim results: Ottawa 2022 0.43 m at 100 m and 0.72 m at 200 m.
- All 14 scripts run from any directory and finish in under 2.5 minutes each when run alone (out_*.txt hold the last outputs).

### hardware_gap: not checked

- Report content is in this file.
- Anything on real hardware: the analog decoder behind /dev/video0, its stamp semantics, field order and standard (NTSC or PAL), the Justice D lens, readout, exposure range and capture latency.
- CPU time on the i.MX8M Plus Cortex-A53 (only the laptop was timed; the about-10x factor is from public benchmark scores).
- PX4 EKF2 in closed loop with our fix: t6_closed_loop.py uses a toy controller and an alpha-beta filter, not PX4 SITL; its distances show sign and order of magnitude only.
- Whether the event drone runs PX4 or ArduPilot (a port-repo comment cites an ArduCopter param file; ArduPilot uses VISO_TYPE, VISO_DELAY_MS, EK3_SRC1_POSXY instead).
- The real FC attitude error spectrum; the sim uses white 0.1 deg noise per sample plus a constant bias, and longer readout models scoring better at 200 m suggests that noise model sets part of the error floor.
- Magnetometer quality near the museum, in-flight yaw resets, and their effect on the NED frame.
- Wind and gusts at 300 m, and large steady tilts against the 55 deg off-nadir cut.
- Snow, water, moving cars and the Ottawa River in view at 300 m.
- Real analog artifacts (dot crawl, chroma noise, ringing) against the LK minEigThreshold tuned on sim noise.
- Interference from the 5 W VTX on the analog capture, and USB grabber resets under vibration.
- Python GIL jitter of a MAVLink reader thread next to the image thread.
- research/prior_art.md and research/site_weather.md (other reviewers' files) were not read.
