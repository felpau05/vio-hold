#!/usr/bin/env python3
"""Stand-in for the Justice D on the laptop: a video_command that writes GREY 1280x720 frames at 60 fps to its
standard output, from a Justice D sim recording (tools/sim_rec.py --video JUSTICE_D, 640x360 at 30 Hz).

  settings: "video_command": "<venv python> integ/rehearsal_2026-11-20/cam_stream.py RAW.npz", "pixel_format": "GREY"

Each recorded picture is enlarged 2x and sent twice, the second time with +-1 grey of fresh noise (the digital
capture refuses bit-identical buffers). The clip repeats. The pictures do not follow the simulator's attitude:
this source tests the capture path (check, mask, run start, the hang-up test), not the hold.
"""
import sys
import time

import cv2
import numpy as np


def main():
    imgs = np.load(sys.argv[1])["imgs"]
    rng = np.random.default_rng(1)
    out, t, k = sys.stdout.buffer, time.monotonic(), 0
    try:
        while True:
            big = cv2.resize(imgs[(k // 2) % len(imgs)], (1280, 720), interpolation=cv2.INTER_LINEAR)
            if k % 2:
                big = np.clip(big.astype(np.int16) + rng.integers(-1, 2, big.shape), 0, 255).astype(np.uint8)
            t += 1 / 60
            time.sleep(max(0.0, t - time.monotonic()))
            out.write(big.tobytes())
            out.flush()
            k += 1
    except (BrokenPipeError, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()
