#!/usr/bin/env python3
"""Replay of a raw recording in 3 chunks must give the same fixes as the same recording in one piece.

Raw.flush saves each 10 s chunk with 1 s of attitude and height before and after its frames, so
neighbouring chunks share rows. Before the fix, replay fed the shared attitude rows through GyroAtt
twice: the tilt bent at each chunk start (first version: 2.39 m against truth on 60 s, 0.36 m merged).
Synthetic data: a moving texture with a rolling and pitching attitude (non-zero body rates).
"""
import math
import os
import sys
import tempfile

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import apb_main  # noqa: E402
import settings as S  # noqa: E402


def recording(s, seconds=6.0, fps=25.0):
    h, w = 288, 360
    rng = np.random.default_rng(3)
    tex = cv2.GaussianBlur(rng.normal(128, 40, (h + 200, w + 200)).astype(np.float32), (0, 0), 2.0)
    tex = np.clip((tex - tex.mean()) * 4 + 128, 0, 255).astype(np.uint8)
    tg = np.arange(0, seconds, 1 / fps) + 10.0
    imgs = []
    for i in range(len(tg)):
        dx, dy = int(60 + 20 * math.sin(i / 15)), int(60 + 20 * math.cos(i / 19))
        imgs.append(tex[dy:dy + h, dx:dx + w].copy())
    ta = np.arange(tg[0] - 2, tg[-1] + 2, 0.01)
    roll, pitch = 0.03 * np.sin(2.1 * ta), 0.02 * np.sin(1.3 * ta)
    att = np.c_[ta, roll, pitch, 0.1 + 0 * ta, 0.063 * np.cos(2.1 * ta), 0.026 * np.cos(1.3 * ta), 0 * ta]
    th = np.arange(tg[0] - 2, tg[-1] + 2, 0.1)
    hgt = np.c_[th, 50.0 + 0 * th]
    K, D = apb_main.lens(s, (h, w), focal=False)
    return dict(imgs=np.array(imgs), t_grab=tg, att=att, hgt=hgt, lpos=np.zeros((0, 3)), K=K, D=D)


def rows(path):
    import csv
    return [(r["t"], r["state"], r["n"], r["north"], r["east"], r["var"], r["sent"], r["why"])
            for r in csv.DictReader(open(path))]


def main():
    s = S.load("/nonexistent")
    s.update(mask_file="", min_height_m=5.0, good_before_send=3)
    rec = recording(s)
    d = tempfile.mkdtemp(prefix="replay_chunks_")
    whole = os.path.join(d, "whole.npz")
    tg = rec["t_grab"]
    span = lambda a: a[(a[:, 0] >= tg[0] - 1.0) & (a[:, 0] <= tg[-1] + 1.0)]      # noqa: E731
    np.savez(whole, **dict(rec, att=span(rec["att"]), hgt=span(rec["hgt"])))       # the chunks merged
    chunks = []
    for k in range(3):                     # as Raw.flush: frames of 2 s, rows from 1 s before to 1 s after
        m = (tg >= tg[0] + 2 * k) & (tg < tg[0] + 2 * (k + 1))
        lo, hi = tg[m][0] - 1.0, tg[m][-1] + 1.0
        cut = lambda a: a[(a[:, 0] >= lo) & (a[:, 0] <= hi)]      # noqa: E731
        p = os.path.join(d, f"raw_{k:03d}.npz")
        np.savez(p, **dict(rec, imgs=rec["imgs"][m], t_grab=tg[m], att=cut(rec["att"]), hgt=cut(rec["hgt"])))
        chunks.append(p)
    p1, c1 = apb_main.replay(s, [whole], log_path=os.path.join(d, "whole.csv"))
    p2, c2 = apb_main.replay(s, chunks, log_path=os.path.join(d, "chunks.csv"))
    a, b = rows(p1), rows(p2)
    same = a == b
    print(f"{'PASS' if c1['sent'] > 20 else 'FAIL'} the whole recording gives fixes: {c1}")
    print(f"{'PASS' if same else 'FAIL'} 3 chunks give the same {len(a)} log rows as the whole recording ({c2})")
    if not same:
        diff = [i for i in range(min(len(a), len(b))) if a[i] != b[i]]
        print(f"  first different row {diff[:1]}: {a[diff[0]] if diff else ''} | {b[diff[0]] if diff else ''}")
    ok = same and c1["sent"] > 20
    print("ALL PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
