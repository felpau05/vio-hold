#!/usr/bin/env python3
"""Checks of kf_graph.py on synthetic data (no pictures): truth recovery, the robust loss and gate against one wrong
edge, and the time of one solve. Run: nice -n 19 ~/.venvs/vio/bin/python integ/kfgraph/test_graph.py
Writes integ/kfgraph/results/test_graph.txt.

The synthetic keyframes: 9 nodes (home + 8) along a 2 m hover drift; each stored anchor pos_k carries a chain error
(random walk 3 cm per change, assumed), each local frame a yaw error (0.3 deg, assumed). Measurements: the true frame
seen in each local frame with noise sig_m (2 cm, 0.2 deg; assumed). Every frame is matched to 3 of the 9 keyframes.
"""
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kf_graph as KG   # noqa: E402

OUT = []


def say(s):
    print(s, flush=True)
    OUT.append(s)


def R(a):
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def make(rng, n=9):
    """True anchors, stored anchors (with chain error) and true local-frame yaws."""
    true_a = np.cumsum(rng.normal(0, 0.25, (n, 2)), 0)
    true_a[0] = 0
    chain = np.cumsum(rng.normal(0, 0.03, (n, 2)), 0)
    chain[0] = 0
    th = rng.normal(0, math.radians(0.3), n)
    th[0] = 0
    return true_a, true_a + chain, th


def local(true_a, pos, th, k, p, psi):
    """The frame (true world p, psi) in local frame k: C = pos_k + R(-th_k)(p - a_k), phi = psi - th_k."""
    return pos[k] + R(-th[k]) @ (p - true_a[k]), psi - th[k]


def run(seed, wrong_edge=False, n_frames=60):
    rng = np.random.default_rng(seed)
    true_a, pos, th = make(rng)
    g = KG.Graph(window=8)
    sm, sp = 0.02, math.radians(0.2)
    S = np.diag([sm ** 2, sm ** 2, sp ** 2])
    for k in range(9):
        g.add_node(k, pos[k], pos[k], 0.0)
    # history: frames near the anchors, each matched to 3 keyframes -> edges
    for f in range(200):
        p = true_a[rng.integers(0, 9)] + rng.normal(0, 0.3, 2)
        ks = rng.choice(9, 3, replace=False)
        ms = {}
        for k in ks:
            C, phi = local(true_a, pos, th, k, p, 0.0)
            ms[k] = (C + rng.normal(0, sm, 2), phi + rng.normal(0, sp), S)
        for i in ks:
            for j in ks:
                if i < j:
                    g.add_pair(i, ms[i], j, ms[j])
    if wrong_edge:   # one frame matched wrongly: keyframe 5's measurement 1.5 m off, 10 times (a repeated false match)
        for _ in range(10):
            p = true_a[5]
            C4, phi4 = local(true_a, pos, th, 4, p, 0.0)
            C5, phi5 = local(true_a, pos, th, 5, p, 0.0)
            g.add_pair(4, (C4, phi4, S), 5, (C5 + np.array([1.5, 0.0]), phi5, S))
    errs, errs_single, ms_t = [], [], []
    for f in range(n_frames):
        p = true_a[rng.integers(0, 9)] + rng.normal(0, 0.3, 2)
        ks = rng.choice(9, 3, replace=False)
        meas = []
        for k in ks:
            C, phi = local(true_a, pos, th, k, p, 0.0)
            meas.append((int(k), C + rng.normal(0, sm, 2), phi + rng.normal(0, sp), S))
        if wrong_edge and f % 4 == 0:     # and one wrong frame measurement in a quarter of the frames
            k, C, phi, S_ = meas[1]
            meas[1] = (k, C + np.array([0.0, 1.0]), phi, S_)
        t = time.perf_counter()
        r = g.solve(meas)
        ms_t.append(1e3 * (time.perf_counter() - t))
        errs.append(float(np.linalg.norm(r["p"] - p)))
        k, C, phi, _ = meas[0]
        errs_single.append(float(np.linalg.norm(C - p)))     # Hold's way: the local fix taken as the world position
    return np.array(errs), np.array(errs_single), np.array(ms_t), g


def main():
    import cv2   # noqa: F401  (not used; keeps the BLAS thread count as in Hold's process)
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    say("kf_graph.py synthetic checks (assumed noise: chain 3 cm per change, local yaw 0.3 deg, meas 2 cm / 0.2 deg)")
    for wrong in (False, True):
        E, E1, T = [], [], []
        for seed in range(1, 6):
            e, e1, t, g = run(seed, wrong)
            E.append(e), E1.append(e1), T.append(t[1:])
        E, E1, T = np.concatenate(E), np.concatenate(E1), np.concatenate(T)
        say(f"  {'with a wrong edge and wrong frame matches' if wrong else 'clean':45s}: frame error RMS graph "
            f"{np.sqrt(np.mean(E ** 2)):.3f} m (max {E.max():.3f}) | single keyframe {np.sqrt(np.mean(E1 ** 2)):.3f} m "
            f"(max {E1.max():.3f}) | solve {np.median(T):.2f} ms median, {np.percentile(T, 99):.2f} ms p99, "
            f"gated blocks {g.stats['blocks_gated']}, edge frames gated {g.stats['edge_gated']}")
    open(os.path.join(HERE, "results", "test_graph.txt"), "w").write("\n".join(OUT) + "\n")


if __name__ == "__main__":
    main()
