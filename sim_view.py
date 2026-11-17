#!/usr/bin/env python3
"""Pictures of what the simulated camera sees, for people (and the talk).

Run:  python sim_view.py      -> sim_view.png
"""
import os

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import sim_test as S
from hold import LK

fig, ax = plt.subplots(2, 3, figsize=(13, 6.8))

# Row 1: the camera at three heights.
for a, h0 in zip(ax[0], (10, 100, 300)):
    pos, att_fn, h_fn = S.hover(h0)
    a.imshow(S.render(5.0, pos, att_fn, h_fn, 8 * h0), cmap="gray", vmin=0, vmax=255)
    a.set_title(f"Camera at {h0} m")

# Row 2: the anchor at 100 m, the same points 3 s later, and grey static.
pos, att_fn, h_fn = S.hover(100)
anchor = S.render(5.0, pos, att_fn, h_fn, 800)
later = S.render(8.0, pos, att_fn, h_fn, 800)
px = cv2.goodFeaturesToTrack(anchor, 300, 0.01, 7)
p2, s, _ = cv2.calcOpticalFlowPyrLK(anchor, later, px, None, **LK)
ok = s.ravel() == 1
a0, a1 = px.reshape(-1, 2)[ok], p2.reshape(-1, 2)[ok]

ax[1, 0].imshow(anchor, cmap="gray", vmin=0, vmax=255)
ax[1, 0].plot(a0[:, 0], a0[:, 1], ".", color="#2ea043", ms=4)
ax[1, 0].set_title(f"Anchor picture: {len(a0)} ground points")

ax[1, 1].imshow(later, cmap="gray", vmin=0, vmax=255)
for (x0, y0), (x1, y1) in zip(a0, a1):
    ax[1, 1].plot([x0, x1], [y0, y1], "-", color="#f0b400", lw=1)
ax[1, 1].plot(a1[:, 0], a1[:, 1], ".", color="#2ea043", ms=4)
ax[1, 1].set(xlim=(0, S.W), ylim=(S.H, 0), title="3 s later: each point's shift from the anchor")

static = np.clip(np.full((S.H, S.W), 90.) + S.rng.normal(0, 3, (S.H, S.W)), 0, 255).astype(np.uint8)
c = cv2.goodFeaturesToTrack(static, 300, 0.01, 7)
_, cs, _ = cv2.calcOpticalFlowPyrLK(static, static, c, None, **LK)
ax[1, 2].imshow(static, cmap="gray", vmin=0, vmax=255)
ax[1, 2].plot(c[:, 0, 0], c[:, 0, 1], "x", color="#d1242f", ms=4)
ax[1, 2].set_title(f"Grey static: {len(c)} corners, {int(cs.sum())} pass the texture test")

for a in ax.ravel():
    a.set_xticks([])
    a.set_yticks([])
fig.suptitle("Simulated nadir camera (360 x 240, one analog field). Top row: the simulated ground "
             "grows with height, so every height looks alike. That is the shortcut to fix.", fontsize=11)
fig.tight_layout()
fig.savefig(os.path.join(S.HERE, "sim_view.png"), dpi=110)
print("wrote sim_view.png")
