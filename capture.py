"""Frame sources for the hold, behind one call: newest(timeout) -> Frame or None.

Frame.img is the picture exactly as Hold gets it (grey; one field, width halved, unless field is
"frame"). Frame.t_grab is the frame's stamp in the companion clock (time.monotonic). Frame.t0 =
t_grab - video_delay_s is the mid-exposure time of row 0; calib.py measures video_delay_s through
this same code (H1). Frame.why is None for a usable picture, else the reason not to use it:
a repeated frame, a flat "no signal" picture, or the driver's no-signal flag (H4).

Sources: V4L2Source (analog grabber through OpenCV), DigitalSource (MIPI CSI camera such as the Orqa
Justice D: whole frames, shrunk by video_reduce; through OpenCV or a shell command, CommandCap),
FileSource (our .npz recordings or a video file), CallableSource (the simulator).
"""
import atexit
import fcntl
import os
import signal
import struct
import subprocess
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


MARK_GREY = 25.0     # grey; a strong mark differs this much (and 6 robust sigma) from the picture's median at 1/4 size.
                     # assumed: a tarp on snow differs by about 145 grey, a point light at night by 50 or more
MARKS_MIN = 3        # this many separate strong marks make a flat picture usable (tarps on snow, lights at night)
NOISE_MIN = 1.0      # grey; ... but only with sensor noise (row to row): a generated NO SIGNAL screen has none.
                     # assumed: live analog video carries about 3.7 grey (measured on the real video)


def sensor_noise(img):
    """Robust sigma of the row-to-row noise (grey)."""
    return 1.4826 * float(np.median(np.abs(np.diff(img.astype(np.int16), axis=0)))) / np.sqrt(2)


def strong_marks(img):
    """Number of separate compact marks that stand out of a flat picture (1/4 size)."""
    q = cv2.resize(img, (max(1, img.shape[1] // 4), max(1, img.shape[0] // 4)), interpolation=cv2.INTER_AREA).astype(np.float32)
    med = float(np.median(q))
    thr = max(MARK_GREY, 6 * 1.4826 * float(np.median(np.abs(q - med))))
    return cv2.connectedComponents((np.abs(q - med) > thr).astype(np.uint8), connectivity=8)[0] - 1


class _Judge:
    """Repeated and flat pictures (H4). A refused picture is not kept as the comparison frame. A flat picture
    with MARKS_MIN strong marks (tarps on snow, lights at night) and sensor noise is not refused: Hold's own
    gates (frozen, static, match quality) then judge it."""

    def __init__(self, s):
        self.s, self.prev = s, None

    def __call__(self, img, no_signal=False):
        if no_signal:
            return "no signal (driver)"
        small = cv2.resize(img, (max(1, img.shape[1] // 8), max(1, img.shape[0] // 8)), interpolation=cv2.INTER_AREA)
        small = small.astype(np.float32)
        if np.median(np.abs(small - np.median(small))) < self.s["no_signal_mad"] and \
                (strong_marks(img) < MARKS_MIN or sensor_noise(img) < NOISE_MIN):
            return "flat picture (no signal?)"
        prev, self.prev = self.prev, img
        if prev is not None and prev.shape == img.shape and cv2.absdiff(img, prev).mean() < self.s["repeat_mad"]:
            return "repeated frame"
        return None


BLACK_GREY = 64.0    # grey; a flat picture with its median under this is black (covered lens), even with OSD text.
                     # Was 24: a covered lens with OSD text at 24 to 60 grey passed, and Hold then sent fixes on the
                     # text (tests_apb/test_robust.py D7). A dark flat picture with only lights (no ground texture) is
                     # refused too. assumed: a covered lens shows under 64 grey (black level 16 plus gain) at any gain


class _DigitalJudge(_Judge):
    """A digital picture has little noise: two fresh frames of a still hover can differ by less than repeat_mad,
    so only a bit-identical picture is a repeat (a frozen stream). It has no NO SIGNAL screen and no noise to
    tell one apart. A covered lens is black, or flat without MARKS_MIN strong marks (the camera's OSD text on
    black has marks, so black is refused with or without marks)."""

    def __call__(self, img, no_signal=False):
        small = cv2.resize(img, (max(1, img.shape[1] // 8), max(1, img.shape[0] // 8)), interpolation=cv2.INTER_AREA)
        small = small.astype(np.float32)
        med = float(np.median(small))
        if np.median(np.abs(small - med)) < self.s["no_signal_mad"] and (med < BLACK_GREY or strong_marks(img) < MARKS_MIN):
            return "black or flat picture (lens covered?)"
        prev, self.prev = self.prev, img
        if prev is not None and prev.shape == img.shape and cv2.norm(img, prev, cv2.NORM_INF) == 0:
            return "repeated buffer (frozen stream)"
        return None


def judge(s):
    return _DigitalJudge(s) if s.get("camera") == "digital" else _Judge(s)   # .get: a simulator can pass a partial dict


CLOSE_WAIT_S = 12.0  # s; close() waits this long for the reader to leave cap.read() (OpenCV's V4L2 read waits up to 10 s)


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
        """Release the capture only after the reader thread stopped: a release while the reader is inside
        cap.read() can crash the process with a real driver."""
        self._stop = True
        self._t.join(timeout=CLOSE_WAIT_S)
        if self._t.is_alive():
            print(f"the camera reader did not stop in {CLOSE_WAIT_S:g} s, so the program did not release the camera. "
                  "If no camera frames arrive, restart the program.", flush=True)
            return
        self.cap.release()


PACKED_Y2 = ("UYVY", "VYUY")                           # packed 4:2:2 with Y in the second byte (YUYV, YVYU: first)
PLANAR_420 = ("NV12", "NV21", "YU12", "YV12", "I420")  # Y plane on top, then h/2 rows of chroma


def luma(buf, fourcc, size=None):
    """The grey (Y) picture of a raw buffer in OpenCV's layout, with no BGR step: YUYV and UYVY (h, w, 2), NV12
    and other 4:2:0 (h * 3/2, w), GREY (h, w). A 3-channel buffer is BGR (a GStreamer appsink, or the driver
    converted it). size: the driver's (w, h). A buffer of another shape raises an error: an OpenCV build that gives
    YUYV as one channel (h, 2w) would otherwise give Hold a wrong picture."""
    if size is not None:
        w, h = size
        want = raw_shape(fourcc or "GREY", w, h)
        if buf.shape not in ((h, w, 3), want):
            fmt = want is None or buf.shape[:2] == (h, w) or buf.size == np.prod(want)   # the right bytes, wrong layout
            raise RuntimeError(f"the camera buffer has shape {buf.shape}, but pixel format {fourcc or '?'} at {w}x{h} "
                               f"needs shape {want or '(unknown)'}. " +
                               ("Set pixel_format to the format that the camera gives (YUYV, UYVY, NV12 or GREY), or use "
                                "video_command." if fmt else "Set video_size to the size that the camera gives."))
    if buf.ndim == 3 and buf.shape[2] == 3:
        return cv2.cvtColor(buf, cv2.COLOR_BGR2GRAY)
    if buf.ndim == 3 and buf.shape[2] == 2:
        return buf[:, :, 1 if fourcc in PACKED_Y2 else 0]
    if buf.ndim == 3:
        buf = buf[:, :, 0]
    if buf.shape[0] < 2:                               # OpenCV gives an unknown format as one row of bytes
        raise RuntimeError(f"cannot read pixel format '{fourcc}' as grey. Set pixel_format to YUYV, UYVY, NV12 or GREY.")
    return buf[:buf.shape[0] * 2 // 3] if fourcc in PLANAR_420 else buf


def shrink(g, k):
    """g reduced by the whole factor k (video_reduce) with INTER_AREA: a digital picture as Hold gets it."""
    return cv2.resize(g, (g.shape[1] // k, g.shape[0] // k), interpolation=cv2.INTER_AREA) if k > 1 else np.ascontiguousarray(g)


def raw_shape(fourcc, w, h):
    """One raw frame in luma's layout: GREY (h, w), packed 4:2:2 (h, w, 2), 4:2:0 (h * 3/2, w). None: unknown size."""
    if fourcc == "GREY":
        return h, w
    if fourcc in ("YUYV", "YVYU") + PACKED_Y2:
        return h, w, 2
    return (h * 3 // 2, w) if fourcc in PLANAR_420 else None


class CommandCap:
    """video_command: a shell command that writes raw frames to its standard output, read with the
    cv2.VideoCapture calls of the reader thread. A frame is video_size in pixel_format (raw_shape). A byte
    stream has no buffer time: get(CAP_PROP_POS_MSEC) is 0, so the stamp is the read time. The command runs
    in a process group of its own, so release() also stops the programs that its shell starts. atexit calls
    release(), so the command does not stay alive when the program stops with an error."""

    def __init__(self, s):
        w, h = map(int, s["video_size"])
        self.fourcc, self.wh = s["pixel_format"].upper(), (w, h)
        self.shape = raw_shape(self.fourcc, w, h)
        if self.shape is None:
            raise RuntimeError(f"cannot cut the video_command output into frames: pixel_format is '{self.fourcc}'. "
                               "Set pixel_format to the format that video_command writes (YUYV, UYVY, NV12 or GREY).")
        self.size, self.ended = int(np.prod(self.shape)), False
        self.proc = subprocess.Popen(s["video_command"], shell=True, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                     start_new_session=True)
        atexit.register(self.release)

    def read(self):
        buf = None if self.ended else bytearray(self.size)        # a new buffer: newest() hands out the last one
        if buf is None or self.proc.stdout.readinto(buf) < self.size:
            self.ended = True                                      # end of the output: the command stopped
            return False, None
        return True, np.frombuffer(buf, np.uint8).reshape(self.shape)

    def get(self, prop):
        return {cv2.CAP_PROP_FRAME_WIDTH: self.wh[0], cv2.CAP_PROP_FRAME_HEIGHT: self.wh[1],
                cv2.CAP_PROP_FOURCC: cv2.VideoWriter_fourcc(*self.fourcc)}.get(prop, 0.0)

    def release(self):
        atexit.unregister(self.release)
        try:                         # SIGKILL to the group: a TERM handler could keep the camera and the pipe open
            os.killpg(self.proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.proc.wait()


class DigitalSource(V4L2Source):
    """Digital camera (Orqa Justice D on CSI 1, 1280x720 progressive at 60 fps). The same reader thread, newest()
    and stamp rule as V4L2Source. Raw buffers (CAP_PROP_CONVERT_RGB off): Hold gets their Y values, shrunk by
    video_reduce with INTER_AREA. No field split, no PAL/NTSC, no driver no-signal flag.
    cap: the object that gives the buffers (cv2.VideoCapture interface; CommandCap for video_command); a test
    passes a fake one."""

    def __init__(self, s, cap=None):
        self.s, self.dev, pipe = s, s["video_device"], s["video_pipeline"]
        if cap is None and s["video_command"]:          # priority over video_pipeline
            cap, pipe = CommandCap(s), ""
        if cap is None:
            cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER) if pipe else cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
            if not cap.isOpened():
                raise RuntimeError(f"cannot open the camera at {'video_pipeline' if pipe else self.dev}. Check the "
                                   f"camera cable and {'video_pipeline' if pipe else 'video_device'} in settings.json.")
            if not pipe:     # the pipeline sets its own size, rate and format
                if s["pixel_format"]:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*s["pixel_format"]))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, s["video_size"][0])
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, s["video_size"][1])
                cap.set(cv2.CAP_PROP_FPS, s["video_fps"])
                cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)       # raw YUV: Y is the grey picture, no BGR conversion
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        self.cap = cap
        code = int(cap.get(cv2.CAP_PROP_FOURCC))
        drv = code.to_bytes(4, "little").decode("ascii", "replace").strip("\0 ") if code > 0 else ""
        self.fourcc = (s["pixel_format"] if pipe else drv or s["pixel_format"]).upper()
        self.info = {"width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                     "fourcc": self.fourcc or "?"}
        self.judge, self.seq, self.got = _DigitalJudge(s), 0, 0
        self.stamp = "read" if isinstance(cap, CommandCap) else "driver"
        self._latest, self._cv, self._stop = None, threading.Condition(), False
        self._t = threading.Thread(target=self._read, daemon=True)
        self._t.start()

    def newest(self, timeout=0.5):
        with self._cv:
            if not self._cv.wait_for(lambda: self._latest is not None and self._latest[2] > self.got, timeout):
                return None
            buf, t, seq = self._latest
        self.got = seq
        self.info.setdefault("buffer", "x".join(map(str, buf.shape)))     # check mode shows the raw layout
        size = (self.info["width"] or self.s["video_size"][0], self.info["height"] or self.s["video_size"][1])  # 0: no driver size
        img = shrink(luma(buf, self.fourcc, size), self.s["video_reduce"])
        return Frame(img, t - self.s["video_delay_s"], t, seq, self.judge(img))

    def close(self):
        if not isinstance(self.cap, CommandCap):
            return super().close()
        self._stop = True        # the reader waits on the command's pipe: stopping the command ends the pipe, so the
        self.cap.release()       # read returns at once (a pipe, not a driver: safe while the reader is in read())
        self._t.join(timeout=CLOSE_WAIT_S)


class FileSource:
    """Frames in order from our .npz recording (imgs, t_grab: pictures as Hold gets them) or a video
    file (stamp = frame index / fps; prepared as live: analog split into one field, digital reduced by video_reduce)."""

    def __init__(self, path, s):
        self.s, self.judge, self.i = s, judge(s), 0      # a digital recording needs the digital picture checks
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
            g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            img = shrink(g, self.s["video_reduce"]) if self.s["camera"] == "digital" else split_field(g, self.s["field"])
            t = self.i / self.fps
        self.i += 1
        return Frame(img, t - self.s["video_delay_s"], t, self.i, self.judge(img))

    def close(self):
        if self.cap is not None:
            self.cap.release()


class CallableSource:
    """fn() -> (picture as Hold gets it, t_grab) or None. For the simulator."""

    def __init__(self, fn, s):
        self.fn, self.s, self.judge, self.seq = fn, s, judge(s), 0   # a digital picture needs the digital checks:
        self.info, self.stamp = {}, "sim"                             # the analog ones refused 29 of 30 still frames (integ/justice_d)

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
    """The capture facts a calibration depends on; run refuses to start when they changed.
    The analog string has no camera word, so analog calibrations made before the digital path stay valid."""
    if s["camera"] == "digital":     # a GStreamer pipeline adds its own delay; video_command stamps at the read
        how = " command" if s["video_command"] else " gstreamer" if s["video_pipeline"] else ""
        return (f"digital {info.get('width') or '?'}x{info.get('height') or '?'}@{s['video_fps']:g}"
                f"{how} reduce={s['video_reduce']} {shape[1]}x{shape[0]}")
    return f"{info.get('standard') or '?'} field={s['field']} {shape[1]}x{shape[0]}"
