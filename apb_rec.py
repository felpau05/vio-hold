"""Walk-test recorder for the APB: Justice D + APB IMU + barometer into 10 s chunks, for replay on the laptop.

  PYTHONPATH=/data/vio/pylib python3 apb_rec.py [--seconds 120] [--fps 30] [--label b1_1m_fwd] [--out /data/vio/rec]

Chunk file <out>/<label>_<time>_<k>.npz:
  imgs     N x 540 x 960 uint8   grey, the 1920x1080 Y plane halved (cv2.INTER_AREA)
  t_frame  N                     driver stamp, CLOCK_MONOTONIC, END of frame (s)
  seq      N                     driver frame sequence (gaps = frames the driver dropped)
  gyro     M x 4                 t, x, y, z (rad/s, sensor axes, no bias removed)
  accel    M x 4                 t, x, y, z (m/s^2)
  baro     K x 2                 t, pressure (Pa)
  soc_c    scalar                SoC temperature at the chunk end
IMU and baro rows overlap 1 s with the neighbouring chunks. Ctrl+C stops cleanly and restores the sensors.
Stops by itself when /data has less than --min-free-mb left.
"""
import argparse
import os
import shutil
import sys
import threading
import time

import cv2
import numpy as np

import apb_sensors as A
from v4l2_mmap import Camera


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=120)
    ap.add_argument("--fps", type=float, default=30, help="frames kept per second (camera runs at 60)")
    ap.add_argument("--label", default="walk")
    ap.add_argument("--out", default="/data/vio/rec")
    ap.add_argument("--imu-hz", type=int, default=200)
    ap.add_argument("--min-free-mb", type=float, default=300)
    a = ap.parse_args()
    A.stop_on_signals()
    os.makedirs(a.out, exist_ok=True)
    base = os.path.join(a.out, f"{a.label}_{time.strftime('%H%M%S')}")

    cam = Camera()
    gyro, acc = A.open_imu(a.imu_hz)
    baro = A.Baro()
    savers, k = [], 0
    imgs, ts, seqs = [], [], []
    t_start = time.monotonic()
    t_keep = -1.0
    p0 = None
    last_print = 0.0
    print(f"recording to {base}_*.npz, {a.fps:g} fps, Ctrl+C to stop", flush=True)

    def flush():
        nonlocal imgs, ts, seqs, k
        if not imgs:
            return
        lo, hi = ts[0] - 1.0, ts[-1] + 1.0
        cut = lambda rows: np.array([r for r in rows if lo <= r[0] <= hi])     # noqa: E731
        data = dict(imgs=imgs, t_frame=np.array(ts), seq=np.array(seqs), gyro=cut(gyro.since(lo)),
                    accel=cut(acc.since(lo)), baro=cut(baro.since(lo)), soc_c=np.array(A.soc_temp_c()))
        path = f"{base}_{k:03d}.npz"
        k += 1

        def save(path=path, data=data):     # stacking 300 frames here, not in the capture loop (a 157 ms stall)
            data["imgs"] = np.stack(data["imgs"])
            np.savez(path, **data)
        th = threading.Thread(target=save, daemon=False)
        th.start()
        savers.append(th)
        imgs, ts, seqs = [], [], []

    try:
        while time.monotonic() - t_start < a.seconds:
            r = cam.read(newest=False, keep=lambda t: t - t_keep >= 1.0 / a.fps - 0.004)
            if r is None:
                print("WARN no frame in 0.5 s", flush=True)
                continue
            y, t, seq = r
            t_keep = t
            imgs.append(cv2.resize(y, (y.shape[1] // 2, y.shape[0] // 2), interpolation=cv2.INTER_AREA))
            ts.append(t)
            seqs.append(seq)
            if ts[-1] - ts[0] >= 10.0:
                flush()
                free = shutil.disk_usage(a.out).free / 1e6
                if free < a.min_free_mb:
                    print(f"STOP only {free:.0f} MB free in {a.out}", flush=True)
                    break
            now = time.monotonic()
            if now - last_print >= 1.0:
                last_print = now
                b = baro.since(now - 1.5)
                p = np.mean([x[1] for x in b]) if b else float("nan")
                if p0 is None and b:
                    p0 = p
                g = gyro.since(now - 1.0)
                print(f"{now - t_start:5.0f} s  frame age {(now - t) * 1e3:4.0f} ms  gyro {len(g):3d}/s  "
                      f"acc {len(acc.since(now - 1.0)):3d}/s  baro {len(b) / 1.5:3.1f}/s  "
                      f"dh {A.pa_to_m(p - p0) if p0 else 0:+.2f} m  grey {y[::8, ::8].mean():3.0f}  "
                      f"SoC {A.soc_temp_c():.0f} C", flush=True)
    except KeyboardInterrupt:
        print("stopping", flush=True)
    finally:
        try:
            flush()
        finally:
            cam.close()
            gyro.close()
            acc.close()
            baro.close()
            for th in savers:
                th.join()
    print(f"saved {k} chunks: {base}_000..{k - 1:03d}.npz", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
