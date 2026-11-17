"""px4sim/onboard.main with a slow-board model that other programs on this laptop cannot change (much).

px4sim/onboard.py waits (cpu_factor - 1) x the WALL time of Hold.process. When other simulations load
the laptop (load average 35-41 on 16 cores at 20:53 on 2026-11-16), the wall time doubles (proc 173 ms
against 83 ms unloaded) and the emulated board gets twice as slow. Here the wait makes the total wall
time cpu_factor x the THREAD CPU time of Hold.process, which load hardly changes. Unloaded the two
models would agree on a synthetic texture (wall 2.5 ms, thread 2.4 ms per frame), but on the sim's
ground pictures OpenCV works on other threads too: at 100 m the thread time x 10 gave 25 ms per frame
(run c_base_100m_w20g40_s11, discarded) against 64-83 ms for the wall model in the clean runs
(apsim/results_clean.txt, 100 m and 50 m). WALL_PER_THREAD scales it back to the same board.
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
WALL_PER_THREAD = 2.8   # measured: unloaded wall / thread time of Hold.process on sim ground (64-83 ms / 25 ms x 10 ~ 2.5-3.3)


def main(settings_path, conn, stop, log_path, opts, status_q, clk):
    """opts["board_s"] > 0: every Hold.process takes board_s seconds (fixed board model). Else the
    thread-time model above. Thread time is not load-proof either: with both hyperthreads of a core busy
    it grew 2.4x (run c_base_100m_w20g40_s12 at 20:59: 220 ms per frame against 93 ms for seed 11)."""
    sys.path.insert(0, PROJ)
    sys.path.insert(0, os.path.join(PROJ, "px4sim"))
    import hold
    import onboard
    f, board_s = float(opts.get("cpu_factor", 10.0)), float(opts.get("board_s", 0.0))
    orig = hold.Hold.process

    def slow(self, *a):
        w0, c0 = time.perf_counter(), time.thread_time()
        r = orig(self, *a)
        want = board_s if board_s > 0 else f * WALL_PER_THREAD * (time.thread_time() - c0)
        rest = want - (time.perf_counter() - w0)
        if rest > 0:
            time.sleep(rest)
        return r
    if f > 1:
        hold.Hold.process = slow
    onboard.main(settings_path, conn, stop, log_path, dict(opts, cpu_factor=1.0), status_q, clk)
