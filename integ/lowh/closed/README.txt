Closed-loop A/B of integ/lowh/proposed.patch on ArduCopter 4.6 SITL at 5 m and 10 m (2026-11-19, 07:04-07:34)
===============================================================================================================

Every number here comes from a SITL run that finished. Simulation only. This fills the gap in integ/lowh/README.txt
section 6 ("no closed-loop run"). 14 runs: 7 before, 7 after. None failed. No retry was necessary.

1. Setup
--------
- Runner: apsim/low_ap_matrix.py run <label>, a wrapper of apsim/engage_matrix.py and apsim/sitl_ap.py. Variant apb,
  video JUSTICE_D, ground ottawa2019fall (tuning site), seeds 1, 2, 3 (tuning seeds), 60 s hold in LOITER, settings
  min_height_m 3, hold_cfg h_min 2, SITL instance 1, speedup 1, board model fixed at 80 ms per Hold.process.
  No hold-out site. No seed from 101 to 120.
- Two copies of the repository tree in the scratchpad: ab_before and ab_after.
  ab_before/hold.py is byte-identical to the repository's hold.py (md5 051dd786...).
  ab_after/hold.py is the repository's hold.py with proposed.patch applied (patch -p1). It is byte-identical to
  scratchpad/lowh_copy/hold.py (md5 eb0c0293...). Both checks passed before the first run (scripts/hold_md5.txt,
  scripts/hold_after_vs_repo.diff).
- Edits, the same in both copies (scripts/copy_edits.diff):
  a) apsim/low_ap_matrix.py: DEADLINE "06:05" -> "09:30". Nothing else. The SUITE_FILE / SUITE_AFTER gate did not
     block (tests_apb/out_timing_test.txt has the time 04:22, and no run_all.sh process ran), so it stays on.
  b) apsim/run_sitl.sh: the container name ap_sim -> ap_sim_ab_before and ap_sim_ab_after. Nothing else.
     Reason: run_sitl.sh mounts the apsim folder of the copy that starts the container at /apsim, and the SITL
     reads its .parm file and writes its log there. The running container ap_sim mounts the repository's apsim.
     With ap_sim, the SITL of a copy would read the .parm from the repository's apsim/runs and write its log
     there. The two containers were stopped after the runs. ap_sim was not touched.
  Apart from hold.py and the container name, the two copies are identical.
- The copies leave out .git, apsim/runs, apsim/logs, px4sim/runs, px4sim/PX4-Autopilot, tools/wheels, integ,
  videos and the old apsim/low_ap_results.*. Their apsim/ardupilot holds only build/sitl/bin/arducopter and
  Tools/autotest/default_params/copter.parm, copied from the repository.
- Path check, before the runs: in each copy, PROJ, RUNS and the output files of engage_matrix.py and
  low_ap_matrix.py, PROJ of sitl_ap.py and RUNS of px4sim/sitl.py are inside the copy. After the first run, each
  copy had its own __pycache__/hold.cpython-312.pyc. The repository's file kept its time (04:02).
  The settings file and the .parm file of before s1 are equal to those of the earlier repository run
  of the same label, except for the paths.
- Order: one SITL at a time, before then after for each label (scripts/ab_driver.py, log ab_driver.log).
  Gate before each start: 1-min load under 2.5, no arducopter process, no other simulator script, no px4_sim
  container. The 1-min load at the start was 0.11-2.50 (all 14 under 2.5, the highest 2.499). During a run the
  load rose to 2.7-6.5. No other SITL ran at the end of any run, and at one check between runs (07:25) the CPU
  was 97 % idle. Inference: the rise came from the run itself.

2. Before/after table
---------------------
rms, max: true horizontal distance from the hold point over the 60 s hold (m).
fix Hz: sent fixes per s in the hold. gap: longest time in the hold without a sent fix (s).
halves: rows refused with 'whole picture: halves disagree' in the hold.
points: rows refused with 'whole picture and points disagree' in the hold.
re-anc: re-anchors in the hold (Hold lost, then re-anchored). >1s: number of gaps over 1 s in the hold.
FC: LOITER = ArduCopter stayed in LOITER for the full 60 s.
EKF text: FC STATUSTEXT in the hold that contains failsafe, EKF, variance, lane, aiding, position or Vision.
load: 1-min load at the start (16 cores).

  case                  ver      rms   max  fix Hz   gap  halves points re-anc  >1s  FC      EKF text  load
  5 m w20g40   s1       before  0.46  0.87    8.1   3.34    123      0     14   10  LOITER  none      0.1
                        after   0.43  0.80   10.9   1.38     24      0      4    4  LOITER  none      2.5
  5 m w20g40   s2       before  0.48  0.95    7.1   5.09    153      0     21   11  LOITER  none      2.4
                        after   0.48  1.17   12.1   0.53      5      0      0    0  LOITER  none      2.4
  5 m w20g40   s3       before  0.62  1.40    9.0   1.82     81      0     10   10  LOITER  none      2.5
                        after   0.42  1.04   10.6   1.38     38      0      5    5  LOITER  none      2.4
  5 m w20g40   median   before  0.48  0.95    8.1   3.34    123      0     14   10
                        after   0.43  1.04   10.9   1.38     24      0      4    4
  10 m w20g40  s1       before  1.06  1.64   10.2   1.54     16     62      5    4  LOITER  none      2.5
                        after   0.94  1.44    9.5   1.30      5    108      4    4  LOITER  none      2.5
  10 m w20g40  s2       before  1.01  1.60   10.4   1.38     41     31      4    4  LOITER  none      2.4
                        after   0.85  1.33    9.7   1.42     14    103      2    2  LOITER  none      2.3
  10 m w20g40  s3       before  1.01  1.57    7.8   1.63     62     87      9    8  LOITER  none      2.4
                        after   0.74  1.15    9.6   1.54     12    100      6    5  LOITER  none      2.5
  10 m w20g40  median   before  1.01  1.60   10.2   1.54     41     62      5    4
                        after   0.85  1.33    9.6   1.42     12    103      4    4
  5 m calm     s1       before  0.11  0.17   11.6   1.54     18      0      2    2  LOITER  none      2.5
                        after   0.07  0.14   11.2   1.46      8      0      1    1  LOITER  none      2.3

Seed ranges (min-max over s1-s3), before against after:
  5 m:  gap 1.82-5.09 / 0.53-1.38 s, halves 81-153 / 5-38, fix Hz 7.1-9.0 / 10.6-12.1, re-anc 10-21 / 0-5:
        no overlap, and each seed improves.
        rms 0.46-0.62 / 0.42-0.48 m, max 0.87-1.40 / 0.80-1.17 m: the ranges overlap. The median rms difference
        (0.05 m) is smaller than the before seed spread (0.16 m).
  10 m: rms 1.01-1.06 / 0.74-0.94 m, max 1.57-1.64 / 1.15-1.44 m: no overlap, each seed lower.
        halves 16-62 / 5-14: no overlap. points 31-87 / 100-108: no overlap, higher after.
        gap 1.38-1.63 / 1.30-1.54 s, fix Hz 7.8-10.4 / 9.5-9.7: the ranges overlap.

Other facts per run (low_ap_results_*.txt and the onboard logs). ferr60: RMS of the sent-fix error against the
truth over the hold, relative to the error at the switch (engage_matrix.fix_err). 'live': rows refused with
'waiting for image motion that proves the picture is live'. first / last: the first sent fix of the flight and the
last sent fix before the switch, in s after the switch. longest flight gap: tools/log_summary.py (ground to landing).

  case            ver     ferr60  'live' -10..0 s  'live' in hold  first   last   longest flight gap (start)
  5 m w20g40 s1   before   0.20        47              10         -7.75  -0.03   6.90 s (-7.7 s)
                  after    0.18        27               0         -7.88  -0.57   3.33 s (-5.1 s)
  5 m w20g40 s2   before   0.22        49               8         -1.19  -0.22   5.20 s (54.8 s, in the hold)
                  after    0.17        29               0         -7.75  -1.00   3.10 s (-5.6 s)
  5 m w20g40 s3   before   0.36        47              12         -0.79  -0.06   1.82 s (0.1 s)
                  after      -         34               1         -7.68  -1.29   3.77 s (-5.6 s)
  10 m w20g40 s1  before   1.06        16               0         -0.74  -0.24   1.53 s (15.0 s)
                  after    0.98         9               0         -6.64  -0.79   5.03 s (-6.6 s)
  10 m w20g40 s2  before   0.96        11               0         -2.07  -0.05   1.38 s (34.6 s)
                  after    0.89         9               0         -3.46  -0.55   1.42 s (11.7 s)
  10 m w20g40 s3  before   1.00         7               2         -3.91  -0.33   1.63 s (42.1 s)
                  after    0.75        15               0         -2.43  -0.08   1.53 s (5.6 s)
  5 m calm s1     before   0.05        45               4         -1.21  -0.07   1.55 s (2.0 s)
                  after    0.04        47              42         -7.67  -1.00   5.22 s (-6.7 s)
ferr60 '-' (after 5 m s3): the program sent no fix in the last 1 s before the switch, so fix_err has no reference.
The 5.20 s gap of before 5 m s2 is in the hold, but the runner's gap column says 5.09 s. The runner adds each
fix's age to its time and stops at the end of the hold. tools/log_summary.py does not.

3. Verdict
----------
1) Gaps at 5 m: yes. In every seed the patch shortened the longest gap in the hold (median 3.34 -> 1.38 s) and cut
   the 'halves disagree' refusals (123 -> 24), the re-anchors (14 -> 4) and the gaps over 1 s (10 -> 4). It also
   raised the fix rate (8.1 -> 10.9 Hz). The before and after ranges do not overlap.
2) Hold error: at 5 m, no change beyond the seed spread (rms median 0.48 -> 0.43 m, the ranges overlap). At 10 m the
   after runs were lower in all 3 seeds (rms 1.01 -> 0.85 m, max 1.60 -> 1.33 m, the ranges do not overlap). But
   n = 3 per version, the run-to-run spread of one label is not measured, and the open-loop replays showed no gain at
   10 m. A rank test on 3 against 3 fully separated runs gives two-sided p = 0.1. Treat the 10 m result as a lead.
3) Safety events: none. All 14 holds stayed in LOITER for 60 s. No EKF or failsafe text in any hold. No fly-away.
   The true height stayed within 4.60-4.78 m and 9.68-9.86 m. All 14 runs (both versions) show 'Radio Failsafe -
   Disarming' and 'Radio Failsafe Cleared' at -15.1 to -13.5 s, on the ground before arming. This comes from the
   test harness, not from the hold.

4. Results that do not support the patch, or limit it
------------------------------------------------------
- 10 m: the 'whole picture and points disagree' refusals rise from 31-87 to 100-108 per hold. The fix rate at 10 m
  drops in 2 of 3 seeds (median 10.2 -> 9.6 Hz). The longest gap does not change (1.54 -> 1.42 s, ranges overlap).
  This agrees with the open-loop result in integ/lowh/README.txt section 4 (DENSE_PTS_PX 2.0 is not changed).
- 5 m calm (one seed): the 'live' refusals in the hold rise from 4 to 42. The longest gap stays the same
  (1.54 -> 1.46 s). This agrees with the open-loop concern about longer-lived whole-picture keyframes
  (README section 4, tests_apb/test_live.py hover 10 m seed 12).
- Before the switch (not in the hold metrics): the after version sends its first fix earlier, during the climb at
  about 3.4 m (5 m: -7.9 to -7.7 s, before: -7.8 to -0.8 s). In every run the climb reaches the hold height minus
  0.5 m at -4.05 s, and a 4 s still settle in ALT_HOLD follows. In 5 of 7 after runs, a gap of 3.1-5.2 s starts
  at -6.7 to -5.1 s (the end of the climb) and runs into the settle. In the four 5 m runs most rows in it are
  'live' refusals (20-44). In 10 m s1 most are 'halves' (15) and 'points' (12) refusals, 'live' 5. The
  before version has this in 1 of 7 runs (5 m s1, first fix at -7.75 s, gap 6.9 s). It has the same 'live'
  refusals at 5 m (45-49 rows from -10 to 0 s). In its other three 5 m runs the first fix came at -1.2 to -0.8 s,
  so these refusals delayed the first fix and made no gap between fixes. Result: the last fix before the switch
  was 0.55-1.29 s old in 6 of 7 after runs, against 0.03-0.33 s in all 7 before runs. ArduCopter accepted LOITER
  at once in all 14 runs. Inference, not tested: a pilot switch inside such a gap starts the hold with no fresh fix
  for some seconds.
- Run-to-run spread: the earlier repository run of the same label before 5 m s1 (04:24, the same hold.py, the
  same settings and .parm, load 1.4 at the start and 8.3 at the end) gave rms 0.85 m, max 1.65 m, gap 6.28 s and
  131 'halves' refusals. This run gave 0.46 m, 0.87 m, 3.34 s and 123. That old run is not used as a comparison
  (its load rose during the run). But it shows that one label can change by about 0.4 m rms between runs.
  Inference: at speedup 1 the SITL follows the wall clock, so thread timing makes each run take its own path
  after the first different frame. The hold-error differences above are of that size. The gap and refusal counts
  at 5 m change much more than that.
- One ground photo (ottawa2019fall), one camera vibration model, 3 seeds, one wind case. Simulation only.

5. Failed or skipped runs
-------------------------
None. All 14 runs finished at the first attempt (rc 0, 60.0 s hold, wall time 88-89 s each). No retry, no
timeout, no run skipped. The s3 labels (lowap_apb_5m_w20g40_s3, lowap_apb_10m_w20g40_s3) ran for both versions.

6. Files (in integ/lowh/closed/)
--------------------------------
README.txt                   this file
ab_table.txt                 the before/after table from scripts/make_closed.py, with the gaps over 1 s per run
low_ap_results_before.txt    apsim/low_ap_matrix.py table of ab_before (per-run FC modes, notes, FC texts, summaries)
low_ap_results_after.txt     the same for ab_after
low_ap_results_before.json   the runner's results of ab_before (all fields)
low_ap_results_after.json    the same for ab_after
onboard_summaries/           tools/log_summary.py output per run (<ver>_<label>.txt)
onboard_csv/                 the onboard log of each run, gzip (<ver>_<label>_onboard.csv.gz)
ab_driver.log                start, end, load and gate waits of each run
scripts/ab_driver.py         the driver (gate, order, retry rule)
scripts/make_closed.py       the table script
scripts/copy_edits.diff      the edits in the copies (low_ap_matrix.py DEADLINE, run_sitl.sh container name)
scripts/hold_after_vs_repo.diff, scripts/hold_md5.txt   the patched hold.py against the repository's, md5 sums
The two copies (with apsim/runs/*: npz truth, json, kf.csv, settings, parm, SITL logs) stay in the work folder of the runs
scratchpad, ab_before and ab_after. A cleanup can remove them.

To run the A/B again:
1. Make two copies of the repository tree (rsync, with the exclusions of section 1).
2. Copy apsim/ardupilot/build/sitl/bin/arducopter and Tools/autotest/default_params/copter.parm into each copy.
3. In ab_after, run: patch -p1 < integ/lowh/proposed.patch. Compare the md5 of hold.py with scripts/hold_md5.txt.
4. Apply the edits of scripts/copy_edits.diff in each copy.
5. Put scripts/ab_driver.py and scripts/make_closed.py next to the two copies.
6. Run: ~/.venvs/vio/bin/python ab_driver.py <label> [<label> ...]
7. Run: ~/.venvs/vio/bin/python make_closed.py
