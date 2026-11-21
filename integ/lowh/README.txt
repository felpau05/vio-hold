Low-height failure of the whole-picture path: cause, fix, tests (2026-11-19, simulation only)
=========================================================================================

Every number here comes from simulation: px4sim/camera.py pictures and recorded ArduCopter SITL motion, open loop.
No SITL was run for this study. Seeds 11-13 (VALIDATION) only. Grounds: the ottawa2022 and farmland photos
(VALIDATION sites) and lowh_fine, a new synthetic fine texture made here (common.py; 6 mm per texture pixel).
Camera: the Justice D of the SITL runs (true lens "sim Justice D near crop", 317 px/rad at 640 x 360); Hold gets
camera.calib_lens of it. Section 4 also has the full-sensor lens (241 px/rad), the team's start profile.

1. The failure, reproduced in open loop (T1)
--------------------------------------------
Tool: ol_low.py. Motion and FC feeds as integ/justice_d/open_loop.py: the recorded ArduCopter hover in wind 20 gusting
40 km/h (apsim/runs/c_d140_100m_w20g40_s11..13.npz; it drifts 0.9-1.3 m in 55 s), height curve moved to 5-40 m,
Hold at 12 Hz, the rules of apb_main.Pipe emulated (re-anchor after 'lost', send after 10 good fixes).
Files: results/ol/*_t1_before.json.gz, results/t1_summary.txt. Repository hold.py, crop lens, seeds 11 / 12 / 13:

  ground      height  'halves disagree' in 55 s   fixes/s           re-anchors   fixes from the whole-picture path
  ottawa2022   5 m    130 / 96 / 114               7.2 / 8.8 / 8.5   13 / 7 / 7   382/394, 461/483, 463/466
  ottawa2022  10 m     21 / 41 / 28                10.3 / 9.6 / 8.6  3 / 7 / 6    543/566, 518/529, 469/474
  ottawa2022  20 m, 40 m: 0 refusals, 11.8 fixes/s, corner path only
  farmland    (0.59 m per photo pixel) whole-picture path at every height: 42-68 refusals at 5 m, 22-105 at 10 m,
              8-53 at 20 m, 0-13 at 40 m
  lowh_fine   5-40 m: 0 refusals, 11.8 fixes/s, corner path only

This matches the SITL runs (apsim/low_ap_results.txt: 131 and 125 refusals at 5 m, 7.6 and 8.4 fixes/s, n = 48).
PX4 SITL at 5 m has the same main refusal (px4sim/runs/lowout_apb_jd_5m_w20g40_s1/s2: 'whole picture: halves
disagree' 206 and 97 times in the hold, calm 5 m: 64; at 10 m: 22 and 66, plus 'whole picture and points disagree' 109
and 44). PX4 did better because it processed more frames (15.6-16.0 per s above 4 m, ArduPilot 12.3) and sent a larger
share (73 % and 87 %, ArduPilot 58 % and 64 %); the share refused by the halves test was 18 % and 10 % (ArduPilot 18 %
and 18 %).

2. Cause (T1, T2), with evidence
--------------------------------
The given hypothesis needs a correction first: the whole-picture path is not one image shift. hold.py _dense solves the
drone move in metres with a per-pixel J (px per m). The halves gap is |Jc (dl_top - dl_bottom)|, with Jc in px per m at
the picture centre. Files: results/probe/*.json.gz (probe.py: one keyframe, then single frames with the drone moved by d
or kept still), results/probe_summary_before.txt, results/probe_compare.txt, results/shares.txt, results/diag_5m.txt,
results/cubic_err.txt.

a) Noise, made large by the unit of the gap (the main cause). Camera vibration that the FC does not see (simulator: 0.2
   deg at 140 and 97 Hz, rolling shutter) moves the two halves by 1-3 px differently. Without vibration the still-hover
   gap is 0.01-0.16 px on ottawa2022 and lowh_fine (0.85 px max on farmland at 5 m): texture and sensor noise alone do
   not matter. With vibration the still gap is 2.0-2.2 px max on fine ground at every height (2.8 px on the photos at
   10-40 m), but 6.5 px (7.8 px with a 3 / -5 deg tilt) at 5 m on the ottawa2022 photo. Reason: the gap is converted to
   px at the picture centre's scale, but a fisheye moves its edge less per metre. At 5 m the photo (0.29 m per photo
   pixel = 14 picture px) is smooth near nadir, so the texture that the LK step uses lies near the edge, where a 1-3 px
   shift is a large move in metres; at the centre's scale it reads 4-8 px. Measured in px of each half's own texture
   (common.local_gap), the same still frames give at most 1.9 px (2.9 px with tilt) on every ground and height.
   Replays, repository hold.py, 5 m ottawa2022, calls with less than 0.05 h of drift from the keyframe (1085 calls):
   13.5 % over 4 px at the centre's scale, 0.0 % in own-texture px (median ratio 0.39).
   Diagnostic replays (results/diag_5m.txt, 5 m ottawa2022, 3 seeds): 340 refusals with the normal feeds, 340 with the
   true height, 306 with the true attitude, 137 without vibration, 100 with all of these and the true lens.
b) Geometry (the given hypothesis, corrected). The warp is linear in the drone move, with J taken at the KEYFRAME position
   from a 1 m FORWARD step. Without vibration, the measured gap follows the prediction of exactly that model (probe.py
   geom_gap, no pictures): lowh_fine 5 m, back direction (the worst one for a forward step), d/h 0.1: 3.7 px measured,
   3.5 predicted; d/h 0.2: 8.7 / 9.3 px. The second-order part depends on d/h only (the same at 5 and 40 m); the 1 m
   step adds a part that matters only at low height (it changes J by up to 6.5 % at 5 m, 3.3 % at 10 m, 0.8 % at
   40 m, 0.3 % at 100 m). Geometry is what is left with ideal feeds above (100 refusals). In the replays the drift from the keyframe stays small (median 0.04 h, p90
   0.11 h at 5 m), because re-anchors renew the keyframe. Refusal rate below / above 0.05 h of drift (shares.txt):
   ottawa2022 5 m 13 % / 24 %, 10 m 5 % / 3 %; farmland 5 m 10 % / 5 %, 10 m 12 % / 8 %. So geometry is about 25-30 %
   of the refusals at 5 m on ottawa2022, and none on farmland or at 10 m. It grows after the noise fix, because
   keyframes then live longer (drift from the keyframe at 5 m: p90 0.17-0.21 h).
c) A third term, found and not fixed: the cubic map is fitted to keyframe points within 55 deg off nadir and
   extrapolated beyond. There its error is 1-7 px (median) at d/h 0.1-0.2 (results/cubic_err.txt). 12 % of the crop-lens
   picture and 37 % of the full-sensor picture lie beyond 55 deg.

Answers for the two causes of the brief:
- Geometric cause: real (b), but a minor share of the refusals; the main cause is the noise in (a).
- Coarse simulator photo: it is why the whole-picture path runs at all at 5-10 m. On ottawa2022 the corner path ends
  with 'N points tracked' (its corners are mostly sensor-noise corners on a smooth picture); on lowh_fine the corner path
  gives every fix and nothing is refused at 5-40 m (fixes 11.8 /s, error RMS 0.03-0.04 m at 5 m). The coarse photo also
  makes the noise cause large, because it puts the texture near the picture edge. Real low-texture ground (short grass,
  snow with a few marks) can do the same; this study has no real footage of it.

3. The fix (proposed.patch; hold.py of the copy; 2 places, 7 code lines replace 4)
----------------------------------------------------------------------------------
- _track1, whole-picture branch: J (px per m of drone move) is taken at the last fix (Hold.pos) with a central step of
  1 % of the height, not at the keyframe position with a 1 m forward step. The start of the warp is not changed.
  Why: the warp is linear in the move, and each frame solves only the move since the reference frame, so J must be the
  derivative where the drone is now. The image motion depends on d / h, so the step is a share of h; a central step
  has no second-order error. 1 % is not tuned: steps of 0.1 % and 5 % of h give the same J to 0.003 % and 0.08 %.
  Limit: this helps only when the reference frame follows the drone (it does on most frames in the replays: every
  'move' live proof moves it). In the probe, where the reference stays at the keyframe, J alone does not remove the
  drift dependence (lowh_fine, d/h 0.2: 3 of 12 'halves' refusals at each height, as before). Its measured benefit in
  the replays is the fix error against the unit change alone: 5 m ottawa2022 0.11 [0.17] against 0.15 [0.28] m,
  full-sensor lens 0.20 [0.31] against 0.25 [0.30] m, farmland 0.08 [0.09] against 0.10 [0.12] m; at 10 m mixed.
- _dense: the halves gap is measured in px of each half's own texture: sqrt(2 d^T (P_top + P_bottom)^-1 d) / s, with
  d = dl_top - dl_bottom, P = A^-1 x (gradient energy of the half) / 2 in m^2 per px^2, s = DENSE_SCALE. It equals the
  old |Jc d| when J is the same over the picture (checked numerically: 1.012 against 1.010 px).
  Why: the vibration error is a px error in the textured part of the picture, so it must be compared with the limit in
  the same px.
- DENSE_AGREE_PX stays 4.0 (not tuned). Its noise floor in the new unit, still hover with the simulator's 0.2 deg
  vibration, 5-40 m, three grounds, with and without tilt: at most 2.9 px.
Tried and not chosen (make_variants.py; results/ol/*_var_*, *_g2_*, results/probe/v_*):
- 'geo', the first try: also START the warp at the last fix. It broke tests/run_regression.py E6 (100 m, +-90 deg yaw
  swings): RMS 1.41 -> 1.65 m (v1 1.45 m) = FAIL; validation seeds 11-13: 1.51 / 1.23 / 1.70 -> 1.71 / 1.53 / 1.64 m.
  The old start carries the reference frame's offset; without it, fixes on a revisited whole-picture keyframe were
  0.04-0.45 m worse. With J only (the patch), E6 gives 1.37 m (PASS), and seeds 11-13 give 1.51 / 1.23 / 1.70 m as
  the repository (results/e6_check.txt; the unit change alone does not change E6).
- the unit change alone ('local'): the same refusal gain, but a larger fix error at 5 m (0.15 [0.28] m against 0.11).
- 'mask55' (whole-picture path only within 55 deg, where the cubic map is fitted): alone 114 -> 66 refusals at 5 m;
  combined with the fix it kept the gain, accepted fewer half-frozen frames and changed the fix error both ways.

4. Before and after (T4), results/before_after.txt
--------------------------------------------------
'after' = the copy's hold.py (variant geo2_local, the same file). Median over seeds 11-13, worst seed in brackets.
A: re-anchor at the true position after 'lost' (a perfect FC position: it resets the fix error to 0 at each re-anchor,
so it favours the version that loses more often). B: re-anchor at Hold's own last fix (apb_main.Pipe without an FC
position); refusals and fix rates are the same as in A.

  ground / lens / h      halves refusals   fixes per s       longest gap s     err RMS m (A)     err RMS m (B)
  ottawa2022 crop  5 m   114 [130] -> 15 [41]  8.47 [7.16] -> 10.80 [10.71]  2.75 [2.92] -> 1.42 [1.50]  0.10 [0.10] -> 0.11 [0.17]  0.34 [0.48] -> 0.28 [0.51]
  ottawa2022 crop 10 m    28 [41] -> 8 [11]    9.62 [8.62] -> 9.27 [8.85]    1.50 [2.42] -> 1.42 [2.50]  0.10 [0.13] -> 0.11 [0.19]  0.44 [0.48] -> 0.46 [0.48]
  ottawa2022 crop 20, 40 m: no whole-picture call: identical (0 refusals, 11.82 fixes/s, 0.84 s)
  farmland   crop  5 m    46 [68] -> 7 [8]     9.16 [9.15] -> 10.55 [10.35]  1.50 [1.58] -> 1.50 [1.58]  0.07 [0.07] -> 0.08 [0.09]  0.27 [0.41] -> 0.20 [0.27]
  farmland   crop 10 m    85 [105] -> 33 [45]  9.16 [8.62] -> 10.95 [10.56]  1.50 [1.75] -> 1.42 [1.50]  0.10 [0.13] -> 0.18 [0.20]  0.27 [0.41] -> 0.19 [0.54]
  farmland   crop 20 m    22 [53] -> 20 [26]  11.22 [10.04] -> 11.04 [10.78] 1.42 [1.58] -> 1.42 [1.50]  0.21 [0.30] -> 0.35 [0.38]  0.30 [0.43] -> 0.38 [0.59]
  farmland   crop 40 m     2 [13] -> 0 [10]     9.33 [8.44] -> 10.07 [8.44]   1.67 [2.09] -> 1.83 [2.09]  0.31 [0.38] -> 0.22 [0.38]  -
  lowh_fine  crop 5-40 m: no whole-picture call: identical (0 refusals, 11.82 fixes/s, 0.84 s, 0.04-0.18 m)
  ottawa2022 full  5 m   122 [130] -> 23 [50]  6.84 [5.96] -> 10.80 [9.36]   2.83 [3.00] -> 1.42 [1.50]  0.14 [0.18] -> 0.20 [0.31]
  ottawa2022 full 10 m    54 [55] -> 10 [16]   7.67 [7.49] -> 9.38 [8.53]    2.08 [3.00] -> 2.00 [2.08]  0.11 [0.13] -> 0.17 [0.17]
Re-anchors (Hold lost), median: ottawa2022 crop 5 m 7 -> 2; full 5 m 16 -> 3; full 10 m 11 -> 5; farmland 10 m 6 -> 2.

Results that do not support the fix, or limit it:
- ottawa2022 10 m: the fix rate does not rise (9.62 -> 9.27 /s). The frames that pass the halves test are then refused
  by 'whole picture and points disagree' (DENSE_PTS_PX 2.0, an assumed limit; offsets 2.1-2.4 px median). Not changed.
- farmland 20 m: refusals 22 -> 20 only, fixes 11.22 -> 11.04 /s, error 0.21 -> 0.35 m (A).
- The fix error in A rises in most cells (0.10 -> 0.11 m at 5 m, 0.10 -> 0.18 m at farmland 10 m, 0.14 -> 0.20 m with
  the full-sensor lens at 5 m). One cause is the perfect re-anchor in A (see B). Another: the whole-picture path's error
  depends on its history. Example: farmland 20 m seed 11, first try against before: the two versions differ first at
  3.7 s by one refused frame, and after that the error differs by a nearly constant 0.2 m. With 3 seeds such
  differences cannot be told from noise.
- Longer-lived whole-picture keyframes: after the fix, Hold loses less often, so a whole-picture keyframe lives longer.
  Its LK corners die out (a corner that LK loses is not tracked again), and then the 'moved' live proof is not
  available; tests_apb/test_live.py hover 10 m seed 12 had a 0.33 s gap from 'waiting for image motion that proves the
  picture is live' (limit 0.3 s; repository hold.py 0.08 s). With
  the unit change alone the same test passes (0.17 s); with the first try, seed 11 failed (0.33 s). It is one frame
  over the limit at 12 Hz, in one of six hover cases.

5. Safety (T4)
--------------
5a. Bad pictures in the flight replays (ol_low.py --bad: 10 s bad window inside 30 s, 3 seeds, the same frames for every
version; results/ol/*_g2_bad_*, results/safety_totals.txt). 'Hold fixes' = frames where Hold gave a fix (360 frames per
cell), 'sent' = after the Pipe's 10 good fixes, 'max err' = the largest error of those Hold fixes.
  case (ottawa2022; farmland was not tested with bad pictures)                 Hold fixes  sent      max err m   'halves'
  bottom half frozen, 5 m                    whole-picture path                   61 -> 75    6 -> 16   0.12 -> 0.27  166 -> 138
  top half frozen, 5 m                       whole-picture path                   66 -> 113   12 -> 21  0.32 -> 0.47  144 -> 123
  static text on the top half of a flat grey picture, 5 m                         16 -> 18    4 -> 4    0.69 -> 0.64    6 -> 0
  the same, 40 m                                                                   18 -> 18    4 -> 4    4.30 -> 4.41    0 -> 0
  static text on the top half, live ground below, 5 m                              13 -> 13    4 -> 4    0.36 -> 0.36    9 -> 9
  the same, 40 m (corner path)                                                    219 -> 219  169 -> 169 3.35 -> 3.35   8 -> 8
  bottom / top half frozen, 40 m (corner path)                                    274 / 315 both, 267 / 306 sent, 5.37 / 4.12 m
  lowh_fine, every bad kind, 5 and 40 m (corner path; first-try runs, results/ol/*_var_bad_*): the same before and after
So the patch still refuses most half-frozen pictures on the whole-picture path, but fewer. The limit now means 4 px of
missed image motion in the textured part of the picture. Before it was 4 px at the centre's scale, which at 5 m on the
coarse photo was about 1.5 px of real image motion (median ratio 0.39), below the 1-3 px vibration noise; that is why
it refused both the noise and the half-frozen pictures. The fixes that passed had errors up to 0.47 m at 5 m.
Found, not caused by the patch: the corner path accepts most half-frozen pictures, using the live half (errors mostly
0.1-0.4 m, but 3-8 m at 40 m in 'degraded' frames), and the flat picture with static text gave 13 Hold fixes (4 sent)
in one seed at both heights, before and after (the known 'overlay only' weakness of tests_apb/test_live.py).

5b. Test suites (worker pools capped at 2 processes by a sitecustomize in the scratchpad; the results do not depend on
the pool size). 'patched' = a copy whose hold.py is byte-identical to the copy's hold.py; 'base' = a copy with the
repository's hold.py.
- tests/bad_pictures.py: patched, base and the tracked tests/bad_pictures.txt are identical.
- tests/run_regression.py: patched = base on every line except E1 and E6 v1 numbers (hold_v1 runs vary between runs)
  and E6 v2: RMS 1.41 -> 1.37 m, false-fix 2.2 % -> 0.7 % (PASS); E9 timing. All lines PASS, as in base. Base itself
  differs from the tracked tests/regression.txt in E6 (v1 numbers and one v2 keyframe count, RMS the same), E9 timing.
- tests_apb/test_basics.py: patched = base: 1 FAIL 'mask not written' in both copies (the copies only; the
  repository's out file says ALL PASS). Not caused by the patch.
- tests_apb/test_run.py, test_event.py: ALL PASS.
- tests_apb/test_live.py: base: 1 FAIL (the known 'overlay only': 21 fixes). patched: 2 FAIL: 'overlay only' (21 fixes,
  as base) and hover 10 m seed 12 (0.33 s, limit 0.3 s; see section 4).

6. Limits, and what was not checked
-----------------------------------
- Simulation only, open loop. No closed-loop SITL run with the patch (not allowed here); its effect on the EKF and the
  hold is unknown. The replays re-anchor at the true position (A) or at the last fix (B), not at an EKF position.
- One camera vibration model (0.2 deg, 140 and 97 Hz, assumed in the simulator). The real Justice D vibration is not
  known. The 4 px limit keeps a 1.4x margin over the simulated noise floor in the new unit.
- Two photo grounds and one synthetic texture; no real low-texture footage. The coarse photo makes the whole-picture
  path run; on real ground at 5 m the corner path may run instead.
- 3 seeds; the fix-error differences between versions are of the size of the history effect (section 4).
- Not fixed: the cubic map beyond 55 deg (2c); the 2.0 px 'points disagree' limit at 10 m; long-lived whole-picture
  keyframes (section 4); rolling-shutter motion of the drone during the readout (about 0.5 px at 5 m and 1 m/s).
- The halves test refuses fewer half-frozen pictures on the whole-picture path after the patch (5a).
- Suggested next step (not done): renew a whole-picture keyframe when its drift passes a share of the height or its
  corners are used up, and test 'mask55' with it.

Line for the tuning log of DATA_SPLITS.md (not added here: this study may write only integ/lowh):
| 2026-11-19 | Hold whole-picture path: J at the last fix with a central step of 1 % of h; halves gap in px of each half's
own texture (DENSE_AGREE_PX 4.0 unchanged); choice among the variants geo, geo2, local, mask55 and their combinations |
open-loop replays and probes on ottawa2022 and farmland (VALIDATION) and lowh_fine (new synthetic), seeds 11-13;
tests/run_regression.py E6 (seeds 1, 3) and the same scene on seeds 11-13 | low-height study (integ/lowh) |

7. Files
--------
common.py          shared parts: load a hold.py by path, camera, grounds, lowh_fine texture, local_gap, instrument
probe.py           controlled frames (still, or moved by d), results/probe/
probe_summary.py   tables of the probe files; probe_compare.py: compact comparison; shares.py: noise against geometry
ol_low.py          flight replays: several hold.py versions on the same frames, bad pictures, diagnostic feeds
ol_summary.py      tables of the replays; final_tables.py: the before/after table; safety_totals.py: bad-picture totals
diag_summary.py    the diagnostic replays (true height, true attitude, no vibration)
make_variants.py   builds the hold.py variants (geo, geo2, local, mask55 and combinations) from the repository's hold.py
                   (geo2+local is the patch, byte for byte)
e6_check.py        the E6 scene of tests/run_regression.py per seed, for the hold.py of a project copy
cubic_err.py       offline check of the cubic map against the true fisheye displacement
run_jobs.sh        runs job lists one at a time with nice -n 19, only while the 1-minute load is under 3
jsonz.py           reads the stored runs (.json or .json.gz)
proposed.patch     the change to hold.py against the repository's current hold.py (patch -p1 < proposed.patch)
results/           every run in its own file (results/ol/*.json.gz, results/probe/*.json.gz; gzip), and the .txt tables
                   named above. Run names: _t1 = T1 (repository hold.py); _var = variant comparison incl. the first try;
                   _g2 = the patch (geo2_local) against before, the first try and local; _relast = re-anchor at the
                   last fix; _diag = diagnostic feeds; _bad_<kind> = bad pictures.
Example: ~/.venvs/vio/bin/python integ/lowh/ol_low.py --hold before=hold.py --hold after=/path/copy/hold.py --height 5
         --seed 11 --ground ottawa2022 --tag _x   (then ol_summary.py --tag _x)
