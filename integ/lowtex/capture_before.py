"""Frame sources for the hold, behind one call: newest(timeout) -> Frame or None.

Frame.img is the picture exactly as Hold gets it (grey; one field, width halved, unless field is
"frame"). Frame.t_grab is the frame's stamp in the companion clock (time.monotonic). Frame.t0 =
t_grab - video_delay_s is the mid-exposure time of row 0; calib.py measures video_delay_s through
this same code (H1). Frame.why is None for a usable picture, else the reason not to use it:
a repeated frame, a flat "no signal" picture, or the driver's no-signal flag (H4).

Sources: V4L2Source (analog grabber through OpenCV), FileSource (our .npz recordings or a video
file), CallableSource (the simulator).
"""
import fcntl
import os
import struct
import threading
import time
from collections import namedtuple

import cv2
import numpy as np

Frame = namedtuple("Frame", "img t0 t_grab seq why")

# V4L2 ioctls (linux/videodev2.h). _IOC(dir, 'V', nr, size); struct v4l2_format is 208 bytes on 64-bit.
_FMT_SIZE = 208 if struct.calcsize("P") == 8 else 204
_IOC = lambda d, nr, size: (d << 30) | (size << 16) | (ord("V") << 8) | nr      # noqa: E731
VIDIOC_G_FMT, VIDIOC_G_STD, VIDIOC_QUERYSTD = _IOC(3, 4, _FMT_SIZE), _IOC(2, 23, 8), _IOC(2, 63, 8)
VIDIOC_G_INPUT, VIDIOC_ENUMINPUT = _IOC(2, 38, 4), _IOC(3, 26, 80)
V4L2_IN_ST_NO_SIGNAL = 0x2
STD_525_60 = 0xF900          # PAL-M, PAL-60, NTSC variants: 60 fields/s
FIELDS = {0: "any", 1: "none", 2: "top", 3: "bottom", 4: "interlaced", 5: "seq_tb", 6: "seq_bt", 7: "alternate",
          8: "interlaced_tb", 9: "interlaced_bt"}


def _ioctl(fd, req, buf):
    try:
        fcntl.ioctl(fd, req, buf)
        return True
    except OSError:
        return False


def v4l2_info(dev, query=False):
    """Driver facts: size, field order, standard, no-signal flag. query=True senses the standard
    (VIDIOC_QUERYSTD); some drivers refuse it while streaming, so call it before opening."""
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
    except OSError as e:
        return {"error": str(e)}
    try:
        out = {}
        fmt = bytearray(_FMT_SIZE)
        struct.pack_into("<I", fmt, 0, 1)                       # V4L2_BUF_TYPE_VIDEO_CAPTURE
        if _ioctl(fd, VIDIOC_G_FMT, fmt):
            w, h, _, field = struct.unpack_from("<4I", fmt, 8)
            out.update(width=w, height=h, field=FIELDS.get(field, str(field)))
        std = bytearray(8)
        if _ioctl(fd, VIDIOC_QUERYSTD if query else VIDIOC_G_STD, std):
            v = struct.unpack("<Q", std)[0]
            out["standard"] = None if v == 0 else ("NTSC" if v & STD_525_60 else "PAL")
        idx = bytearray(4)
        if _ioctl(fd, VIDIOC_G_INPUT, idx):
            inp = bytearray(80)
            inp[0:4] = idx
            if _ioctl(fd, VIDIOC_ENUMINPUT, inp):
                out["no_signal"] = bool(struct.unpack_from("<I", inp, 56)[0] & V4L2_IN_ST_NO_SIGNAL)
        return out
    finally:
        os.close(fd)


def split_field(g, field, layout="interlaced"):
    """One field of a woven frame, width halved so pixels stay near square (as in sim_test)."""
    # shortcut: a single-field buffer (V4L2_FIELD_ALTERNATE) passes through, so top and bottom fields
    # alternate and the picture jumps half a row (G8); read the buffer's field flag if check shows "alternate".
    if field == "frame" or g.shape[0] <= 320:
        return g
    h = g.shape[0]
    if layout == "seq_tb":
        top, bottom = g[:h // 2], g[h // 2:]
    elif layout == "seq_bt":
        bottom, top = g[:h // 2], g[h // 2:]
    else:
        top, bottom = g[0::2], g[1::2]
    f = top if field == "top" else bottom
    return cv2.resize(f, (g.shape[1] // 2, f.shape[0]), interpolation=cv2.INTER_AREA)


class _Judge:
    """Repeated and flat pictures (H4). A refused picture is not kept as the comparison frame."""

    def __init__(self, s):
        self.s, self.prev = s, None

    def __call__(self, img, no_signal=False):
        if no_signal:
            return "no signal (driver)"
        small = cv2.resize(img, (max(1, img.shape[1] // 8), max(1, img.shape[0] // 8)), interpolation=cv2.INTER_AREA)
        small = small.astype(np.float32)
        if np.median(np.abs(small - np.median(small))) < self.s["no_signal_mad"]:
            return "flat picture (no signal?)"
        prev, self.prev = self.prev, img
        if prev is not None and prev.shape == img.shape and cv2.absdiff(img, prev).mean() < self.s["repeat_mad"]:
            return "repeated frame"
        return None


class V4L2Source:
    """Analog grabber. A thread reads every buffer so the driver queue never holds old frames;
    newest() returns only the latest one (H6). Stamp: the driver's buffer time (CLOCK_MONOTONIC)
    when it is plausible, else the read time (then self.stamp says "read")."""

    def __init__(self, s):
        self.s, self.dev = s, s["video_device"]
        sensed = v4l2_info(self.dev, query=True).get("standard")
        self.cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open video device {self.dev}. Check the grabber and video_device in settings.json.")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        self.info = v4l2_info(self.dev)              # the format OpenCV set up
        if sensed:
            self.info["standard"] = sensed           # the incoming signal wins over the configured one
        self.judge, self.stamp, self.seq, self.got = _Judge(s), "driver", 0, 0
        self._latest, self._cv, self._stop = None, threading.Condition(), False
        self._t = threading.Thread(target=self._read, daemon=True)
        self._t.start()

    def _read(self):
        while not self._stop:
            ok, bgr = self.cap.read()
            t_read = time.monotonic()
            if not ok:
                time.sleep(0.01)
                continue
            t_buf = self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            if 0.0 <= t_read - t_buf < 1.0:
                t = t_buf
            else:   # shortcut: read time jitters by up to a frame (H1); fix the driver stamp if check shows "read"
                t, self.stamp = t_read, "read"
            with self._cv:
                self.seq += 1
                self._latest = (bgr, t, self.seq)
                self._cv.notify_all()

    def newest(self, timeout=0.5):
        with self._cv:
            if not self._cv.wait_for(lambda: self._latest is not None and self._latest[2] > self.got, timeout):
                return None
            bgr, t, seq = self._latest
        self.got = seq
        g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        img = split_field(g, self.s["field"], self.info.get("field", "interlaced"))
        no_sig = seq % 5 == 0 and v4l2_info(self.dev).get("no_signal", False)
        return Frame(img, t - self.s["video_delay_s"], t, seq, self.judge(img, no_sig))

    def close(self):
        self._stop = True
        self._t.join(timeout=1.0)
        self.cap.release()


class FileSource:
    """Frames in order from our .npz recording (imgs, t_grab: pictures as Hold gets them) or a video
    file (stamp = frame index / fps; the field is split as live)."""

    def __init__(self, path, s):
        self.s, self.judge, self.i = s, _Judge(s), 0
        if path.endswith(".npz"):
            d = np.load(path)
            self.imgs, self.tg, self.cap = d["imgs"], np.asarray(d["t_grab"], float), None
        else:
            self.cap = cv2.VideoCapture(path)
            self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 25.0

    def newest(self, timeout=0.0):
        if self.cap is None:
            if self.i >= len(self.imgs):
                return None
            img, t = self.imgs[self.i], float(self.tg[self.i])
        else:
            ok, bgr = self.cap.read()
            if not ok:
                return None
            img = split_field(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), self.s["field"])
            t = self.i / self.fps
        self.i += 1
        return Frame(img, t - self.s["video_delay_s"], t, self.i, self.judge(img))

    def close(self):
        if self.cap is not None:
            self.cap.release()


class CallableSource:
    """fn() -> (picture as Hold gets it, t_grab) or None. For the simulator."""

    def __init__(self, fn, s):
        self.fn, self.s, self.judge, self.seq = fn, s, _Judge(s), 0
        self.info, self.stamp = {}, "sim"

    def newest(self, timeout=0.5):
        r = self.fn()
        if r is None:
            return None
        self.seq += 1
        img, t = r
        return Frame(img, t - self.s["video_delay_s"], t, self.seq, self.judge(img))

    def close(self):
        pass


def standard(info, fps):
    """PAL or NTSC from the driver, else from the measured frame rate, else None."""
    if info.get("standard"):
        return info["standard"]
    if fps:
        per_frame = fps / 2 if info.get("field") == "alternate" else fps
        return "PAL" if 22 < per_frame < 27 else "NTSC" if 27 < per_frame < 32 else None
    return None


def capture_mode(s, info, shape):
    """The capture facts a calibration depends on; run refuses to start when they changed."""
    return f"{info.get('standard') or '?'} field={s['field']} {shape[1]}x{shape[0]}"
