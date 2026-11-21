#!/usr/bin/env python3
"""Forward-camera recorder (Justice A, analog). It runs as its own process, never inside apb_main.py.

  tools/fwd_rec.py --device /dev/video1 [--hz 5] [--dir logs] [--min-free-gb 2.0] [--field top] [--seconds N]

It opens one video device, keeps at most --hz pictures each second (default 5), stamps each picture in the
time.monotonic() clock (the program's clock), and writes 10 s chunks <dir>/fwd_<stamp>_NNN.npz with the keys of the
program's raw recorder (apb_main.Raw): imgs, t_grab, att, hgt, lpos, K, D. att, hgt and lpos are empty: this process
has no FC link (the program owns the FC port). Extra keys: camera="forward", device, t_read, info.
t_grab: the driver's buffer time (CLOCK_MONOTONIC, the clock of time.monotonic()) when it is plausible, else the read
time. t_read: time.monotonic() right after the buffer came. Pictures are stored as the program stores its analog
pictures (capture.split_field: one field, width halved), unless --field frame.

It stops cleanly on Ctrl+C (SIGINT) and on SIGTERM: it writes the last chunk and releases the device. It stops by
itself when its disk has less than --min-free-gb free, and when no picture comes for 10 s.
CPU: it dequeues every buffer with grab() (no decoding) and decodes only the kept pictures; it runs at nice 10.
Not tested on the board. Run the bench procedure (integ/fwdcam/BENCH.txt) before any use near a flight.
"""
import argparse
import math
import os
import shutil
import signal
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, PROJ)

import cv2               # noqa: E402
import numpy as np       # noqa: E402

CHUNK_S = 10.0
NO_FRAME_S = 10.0


class Saver:
    """One save thread at most; a chunk that comes while the last one is still being written is dropped."""

    def __init__(self):
        self.th, self.dropped, self.saved, self.error = None, 0, 0, None

    def save(self, path, data, wait=False):
        if self.th is not None and self.th.is_alive():
            if wait:
                self.th.join()
            else:
                self.dropped += 1
                print(f"fwd_rec: dropped a chunk: the disk still writes the last one ({path})", flush=True)
                return
        self.th = threading.Thread(target=self._run, args=(path, data), daemon=False)
        self.th.start()
        if wait:
            self.th.join()

    def _run(self, path, data):
        try:
            np.savez(path, **data)
            self.saved += 1
        except OSError as e:
            self.error = str(e)
            print(f"fwd_rec: cannot write {path}: {e}", flush=True)


def lens_for(w, h, name):
    try:
        import fwd_cam
        L = fwd_cam.Lens.profile(w, h, name)
        return L.K, L.D
    except Exception as e:                       # the recording is still useful without a lens
        print(f"fwd_rec: no lens profile stored ({e})", flush=True)
        return np.zeros((3, 3)), np.zeros(4)


def main(argv=None):
    ap = argparse.ArgumentParser(description="forward-camera recorder (own process)")
    ap.add_argument("--device", default="/dev/video1")
    ap.add_argument("--hz", type=float, default=5.0, help="pictures kept each second")
    ap.add_argument("--dir", default=os.path.join(PROJ, "logs"))
    ap.add_argument("--min-free-gb", type=float, default=2.0, help="stop under this much free disk (the program's own "
                    "min_free_gb is 2.0: keep at least that)")
    ap.add_argument("--field", default="top", choices=("top", "bottom", "frame"))
    ap.add_argument("--lens", default="Round 1 independent k2")
    ap.add_argument("--seconds", type=float, default=0.0, help="0: until Ctrl+C")
    a = ap.parse_args(argv)

    try:
        os.nice(10)
    except OSError:
        pass
    cv2.setNumThreads(1)
    import capture                                # read only: split_field and v4l2_info, the program's conventions
    os.makedirs(a.dir, exist_ok=True)
    free = shutil.disk_usage(a.dir).free / 1e9
    if free < a.min_free_gb:
        print(f"fwd_rec: not started: {a.dir} has {free:.1f} GB free, less than {a.min_free_gb:g} GB", flush=True)
        return 2
    info = capture.v4l2_info(a.device)
    if "error" in info:
        print(f"fwd_rec: cannot open {a.device}: {info['error']}", flush=True)
        return 2
    cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"fwd_rec: cannot open {a.device} with OpenCV V4L2", flush=True)
        return 2
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    stop = {"why": None}

    def on_signal(sig, _frm):
        stop["why"] = "Ctrl+C" if sig == signal.SIGINT else "SIGTERM"

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    base = os.path.join(a.dir, f"fwd_{time.strftime('%Y%m%d_%H%M%S')}")
    saver, k, n_kept, n_grab = Saver(), 0, 0, 0
    imgs, tg, tr = [], [], []
    slot, t_last = None, time.monotonic()
    t_end = time.monotonic() + a.seconds if a.seconds > 0 else math.inf
    stamp_kind = "driver"
    K = D = None

    def flush(wait=False):
        nonlocal imgs, tg, tr, k
        if not imgs:
            return True
        free = shutil.disk_usage(a.dir).free / 1e9
        if free < a.min_free_gb:
            stop["why"] = f"disk nearly full ({free:.1f} GB free, less than {a.min_free_gb:g} GB)"
            imgs, tg, tr = [], [], []
            return False
        e = np.zeros((0, 7))
        data = dict(imgs=np.array(imgs), t_grab=np.array(tg), t_read=np.array(tr), att=e, hgt=np.zeros((0, 2)),
                    lpos=np.zeros((0, 4)), K=K, D=D, camera=np.array("forward"), device=np.array(a.device),
                    info=np.array(repr(info)), stamp=np.array(stamp_kind))
        saver.save(f"{base}_{k:03d}.npz", data, wait=wait)
        k += 1
        imgs, tg, tr = [], [], []
        return True

    print(f"fwd_rec: recording {a.device} ({info}) at {a.hz:g} pictures/s to {base}_NNN.npz. Ctrl+C stops.", flush=True)
    try:
        while stop["why"] is None and time.monotonic() < t_end:
            ok = cap.grab()
            t_read = time.monotonic()
            if not ok:
                if t_read - t_last > NO_FRAME_S:
                    stop["why"] = f"no picture for {NO_FRAME_S:g} s (camera, cable or decoder input?)"
                time.sleep(0.01)
                continue
            n_grab += 1
            t_last = t_read
            t_buf = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            if 0.0 <= t_read - t_buf < 1.0:
                t = t_buf
            else:
                t, stamp_kind = t_read, "read"
            s_now = math.floor(t * a.hz)
            if s_now == slot:
                continue
            slot = s_now
            ok, bgr = cap.retrieve()
            if not ok:
                continue
            g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
            g = capture.split_field(g, a.field, info.get("field", "interlaced"))
            if K is None:
                K, D = lens_for(g.shape[1], g.shape[0], a.lens)
            imgs.append(g)
            tg.append(t)
            tr.append(t_read)
            n_kept += 1
            if t - tg[0] >= CHUNK_S and not flush():
                break
    finally:
        flush(wait=True)
        if saver.th is not None:
            saver.th.join()
        cap.release()
    why = stop["why"] or "time is up"
    print(f"fwd_rec: stopped ({why}). {n_kept} pictures kept of {n_grab} buffers, {saver.saved} chunks written, "
          f"{saver.dropped} dropped, stamps: {stamp_kind}. Files: {base}_NNN.npz", flush=True)
    return 0 if saver.error is None else 1


if __name__ == "__main__":
    sys.exit(main())
