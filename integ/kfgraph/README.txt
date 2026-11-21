integ/kfgraph: keyframe pose graph back end for the VIO position hold (prototype, outside hold.py)
=====================================================================================================
Status: STOPPED 06:36 EST (2026-11-21). Code ready; measured on TUNING data only (smoke + check); NO validation run.

Files
  kf_graph.py        the graph (numpy only): nodes = keyframe anchors (x, y, yaw), home node fixed (gauge), edges from
                     frames matched to two keyframes, current frame solved with the nodes; Gauss-Newton, Huber weights,
                     chi-square gate (3 dof, 99.9 %: 16.27) per block, per-edge gate before an edge measurement is added.
  test_graph.py      synthetic checks of kf_graph.py (no pictures) -> results/test_graph.txt
  hold_snapshot.py   hold.py of .snapshots/pre_kfmap_20261121_0310.tgz, byte for byte (= integ/kfmap/hold_before.py)
  hold_graph.py      copy of hold_snapshot.py with the option KF_GRAPH (False by default). On: every good frame is also
                     tracked against up to GRAPH_K = 2 other window keyframes (least recently tried first; the graph's
                     home first when it was not tried for 1 s); the graph gives the fix; the graph survives reanchor(),
                     so a new chain (prior at its anchor, sigma 0.3 m + 2 m/s x time since the last fix) is tied back
                     to the old keyframes when one is seen again. Safety: the jump test on the frame's own fix; when the
                     graph gates this frame's own keyframe measurement, or moves the fix more than 1 m + 0.05 h from the
                     frame's own fix, the frame's own fix is sent (as graph off) and no edges are added from it.
  ol_graph.py        replay runner: integ/lowh/ol_low.py's pictures, feeds, Pipe emulation and bad pictures; several Hold
                     versions on the same frames; adds false fixes (m2 > 9.21), time per process(), graph info per frame.
                     --split tuning uses ottawa2019fall/suburb/prairie with flights ap_apb_100m_w20g40_s1..3 (TUNING).
  gate.sh            waits until a replay may start (no integ/kfmap/SITL_ACTIVE, load < 2, no arducopter/px4/containers)
  run_queue.sh       runs a job list one replay at a time (nice -n 19, 1 thread, gate before each)
  window.sh          the run plan for the free window (07:15-07:30 EST): batch 1 smoke alone (stops if it fails), solve
                     timing pinned to one core, then batches of up to 4 replays at the same time (1 thread each,
                     nice -n 19), tables after each batch. Log: results/window.log
  jobs_smoke.txt     batch 1: 20 s of the tuning flight s1, ottawa2019fall 10 m, snapshot / off / on
  jobs_b2.txt        batch 2: tuning (ottawa2019fall 5/10/20 m, seeds 1-2) and validation 10 m (ottawa2022, farmland,
                     seeds 11-13, re-anchor at the last fix and at the truth)
  jobs_b3.txt        batch 3: validation 5 and 20 m, then bad pictures (bad_graph.py off / on; ol_low.py kinds at 5 m)
  jobs_b4.txt        batch 4 (if time is left): lowh_fine ground 5/10/20 m and the full-sensor lens at 5/10 m
  jobs_all.txt, run_queue.sh: the first, serial plan (not used any more)
  inspect_run.py     one run, versions side by side over time
  summarize.py       off/on tables -> results/table_<split>.txt, and the off == snapshot check
  bad_graph.py       tests/bad_pictures.py cases with hold_graph off / on -> results/bad_pictures_<version>.txt

Step 1 (done 03:16): kf_graph.py + synthetic check (results/test_graph.txt). Synthetic data only, noise values assumed:
  9 keyframes, chain error 3 cm per change, frame matched to 3 keyframes. Frame error RMS: graph 0.017 m, single
  keyframe (Hold's way) 0.099 m. With one wrong edge (1.5 m, 10 frames) and wrong frame matches (1 m, every 4th frame):
  graph 0.018 m, 15 blocks gated. Solve time 1.2 ms median, measured while the laptop load was 24 (not a clean timing).

Replays: every replay waited at the gate from 03:21 EST (integ/kfmap/SITL_ACTIVE, then the SITL A/B of integ/kfmap,
load 2-8). Coordinator decision (about 03:53): no replay while SITL runs. The queue (jobs_all.txt) starts by itself when the
gate opens. Results of each step are written below as soon as the step ends.

Step 2 (done 05:46, approved code check, 1 process, 1 thread): ol_graph.py on 8 s of the TUNING flight s1,
  ottawa2019fall 10 m, re-anchor at the last fix. Files: results/ol/kg_tun_ottawa2019fall_crop_10m_s1_check_relast_*.json.gz
  - graph off == snapshot on all 96 frames (sent, error vector, state, reason, n).
  - graph on: runs without error; 2.3 keyframes measured per frame (extra tracks 96 of 126 good); no gated block.
    error RMS 0.141 -> 0.082 m, max 0.324 -> 0.152 m, false fixes 9 -> 3, fixes/s 6.75 = 6.75 (5 re-anchors in 8 s).
    Time per process(): 4.9 -> 10.7 ms mean (p95 7.9 -> 17.2 ms); graph solve 0.70 ms median, 1.85 ms max.
  - How: before the first loss both versions have the same error (0.02 -> 0.09 m, growing). After each re-anchor
    at the last fix, the graph ties the new chain back to the old keyframes (correction up to 0.14 m), so the error
    falls back (0.28 -> 0.14 m) while graph off keeps the wrong re-anchor (0.31-0.32 m).
  - 8 s of one tuning flight only: not a result, a code check.

What each job should show, and how to read it (written before the window, 05:50)
- Expected: the graph helps after a loss (re-anchor at the last fix = the 'relast' runs), because it ties the new
  chain back to old keyframes. It does NOT remove the error that all keyframes share (attitude, height, lens): in the
  check, before the first loss, off and on had the same error. So:
  * relast runs: error RMS and max lower with the graph where Hold loses often (5 and 10 m: 3-13 re-anchors per 55 s
    in integ/lowh), about equal at 20 m (0-1 re-anchors).
  * truth runs (re-anchor at the true position): the graph can make the error LARGER: it pulls the new chain back to
    the old map, which carries its own error, while graph off starts from a perfect position. A real FC position
    after a loss is not perfect, so 'truth' favours graph off.
  * fixes/s and longest gap: about equal. The graph does not give a fix when the current keyframe fails (not built).
  * time per frame: about 2x (2 more keyframes tracked per frame). Graph solve: about 1 ms.
  * false fixes (m2 > 9.21 under the sent covariance): watch them; the graph's covariance is smaller (it drops the
    hand-over sum), so more false fixes would mean the covariance is too small.
- How to read: results/table_validation.txt (summarize.py): median over seeds 11-13 and [worst seed], off -> on.
  A cell 'helps' only when the median and the worst seed both improve. The tuning table is results/table_tuning.txt.
  Safety: results/bad_pictures_off.txt against results/bad_pictures_on.txt (same columns as tests/bad_pictures.txt),
  and the kg_val_*_bad_* runs (bad_window in each file): any case with more sent fixes or a larger max error with the
  graph on is a finding.

Step 3 (06:31, gate open early: SITL_ACTIVE gone, load < 2) smoke: 20 s of the TUNING flight s1, ottawa2019fall 10 m,
  re-anchor at the last fix. Files: results/ol/kg_tun_ottawa2019fall_crop_10m_s1_smoke_relast_{snap,off,on}.json.gz
  - off == snapshot on all 240 frames.
  - off -> on: error RMS 0.273 -> 0.130 m, max 0.492 -> 0.345 m, false fixes 123 -> 26 of 168 sent, fixes/s 8.4 = 8.4,
    longest gap 2.42 = 2.42 s, 6 re-anchors in 20 s. process() 7.2 -> 16.6 ms mean (p95 12.5 -> 24.0 ms), load 1.7.
    Graph: 196 frames, all used; 1.9 keyframes per frame; extra tracks 177 of 372 good; solve 1.2 ms median, 1.9 max.
  Incident 06:31-06:33: window.sh started 4 replays at once before the approved window (my gate had no load test
  inside a batch). I stopped 6 of them within 1 min (no files) and changed gate.sh: without integ/kfgraph/GO it now
  allows one replay at a time and load < 2. Also found and fixed a bug in hold_graph.py _graph_step (cand.remove
  compared keyframe dicts by value: ValueError; now by identity). The tuning 10 m s1 run crashed on it (no file).
  Solve timing pinned to core 2 (results/test_graph_core2.txt): 2.0 ms median, 2.25 ms p99; it ran while those
  replays started, so it is not a clean timing.
  06:33: coordinator: no replay at all until integ/kfgraph/GO exists (the lead's timed suite runs 06:31-06:45);
  gate.sh changed: without GO nothing starts. Nothing of kfgraph ran from 06:33 until GO.
  06:33:51-06:34:00: one more replay (tuning 10 m s2) started without GO: a gate.sh process that had started before
  the no-GO change still ran the old code. Killed after about 9 s (no file). The coordinator stopped the runner at
  06:34; window.sh restarts only after the coordinator's GO. Kill list sent to main (06:35).

FINAL STATE (06:36 EST; the agent run ended here, before the GO window)
Done
  - kf_graph.py (graph, numpy only), synthetic check results/test_graph.txt (+ results/test_graph_core2.txt).
  - hold_graph.py (KF_GRAPH off by default). Off == snapshot hold.py on every frame of both replays run (96 + 240 frames).
  - ol_graph.py, bad_graph.py, summarize.py, inspect_run.py, window.sh, gate.sh (no start without integ/kfgraph/GO),
    job lists jobs_smoke.txt, jobs_b2.txt (tuning + validation 10 m), jobs_b3.txt (validation 5/20 m + bad pictures),
    jobs_b4.txt (lowh_fine, full lens).
  - TUNING data only, one flight (s1, ottawa2019fall 10 m, re-anchor at the last fix): 8 s check and 20 s smoke
    (Step 2, Step 3 above): graph on lowered error RMS 0.141 -> 0.082 and 0.273 -> 0.130 m, false fixes 9 -> 3 and
    123 -> 26; fixes/s equal; process() about 2.2x slower (4.9 -> 10.7 and 7.2 -> 16.6 ms mean).
Not done
  - No VALIDATION replay (ottawa2022, farmland, seeds 11-13): no off/on table (results/table_validation.txt absent).
  - No full tuning replay (5/10/20 m, seeds 1-2): the runs were stopped (gate incidents) and left no file.
  - No safety run: tests/bad_pictures.py cases (bad_graph.py) and ol_low.py bad kinds at 5 m not run.
  - No clean timing on an idle core; no APB (Cortex-A53) timing. Scale in the node state: not added, not tested.
  - Not built: a graph fix when the current keyframe fails; merge with KF_MAP's keyframe store.
To run later (one command, after GO exists): integ/kfgraph/window.sh > integ/kfgraph/results/window.log 2>&1
  then: python integ/kfgraph/summarize.py validation ; python integ/kfgraph/summarize.py tuning

POWER NOTE (added by the coordinating session 0a at 06:50 EST, 2026-11-21): the laptop ran on battery from at least 04:33 EST (upower history; the first 'discharging' row found), and session 1f saw CPU power-save mode at 06:49 with 9 % battery. Runs after about 04:33 (most of the SITL A/B, the regression and the suites) ran on battery power; power-save may have started earlier than 06:49. Timing numbers (E9/H6, process() ms, test time limits) from that period can be slower than on mains power. The SITL board model uses a fixed 80 ms per Hold.process, but the SITL itself ran slower; each run's start load is in its table. The off/on comparisons ran under the same conditions for both versions.
