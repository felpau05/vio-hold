"""Justice D capture straight from V4L2 (mmap, multiplanar), without OpenCV's VideoCapture.

Why not cv2.VideoCapture: it sets the capture size, and on the APB a size set on /dev/video2 propagates to
the sensor subdev (dummy_csi2_sensor) and crops the picture (measured 2026-11-21: 960x540 gave the top-left
quarter). This reader only ever asks for SENSOR_W x SENSOR_H, the size the organizers' boot script sets.

Frames: NV12; the first plane block is the grey (Y) picture, so no colour conversion is needed.
Stamp: the driver's buffer time, CLOCK_MONOTONIC, end of frame (flags ts-monotonic, ts-src-eof, measured).
"""
import ctypes
import mmap
import os
import select
import time

import numpy as np

SENSOR_W, SENSOR_H = 1920, 1080
NV12 = int.from_bytes(b"NV12", "little")
MPLANE = 9                 # V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
MEM_MMAP = 1
VIDEO_MAX_PLANES = 8
TS_MASK, TS_MONO = 0xE000, 0x2000
TSRC_MASK, TSRC_EOF = 0x70000, 0x00000


class Plane(ctypes.Structure):
    _fields_ = [("bytesused", ctypes.c_uint32), ("length", ctypes.c_uint32), ("m", ctypes.c_ulong),
                ("data_offset", ctypes.c_uint32), ("reserved", ctypes.c_uint32 * 11)]


class Timeval(ctypes.Structure):
    _fields_ = [("sec", ctypes.c_long), ("usec", ctypes.c_long)]


class Buffer(ctypes.Structure):
    _fields_ = [("index", ctypes.c_uint32), ("type", ctypes.c_uint32), ("bytesused", ctypes.c_uint32),
                ("flags", ctypes.c_uint32), ("field", ctypes.c_uint32), ("timestamp", Timeval),
                ("timecode", ctypes.c_uint8 * 16), ("sequence", ctypes.c_uint32), ("memory", ctypes.c_uint32),
                ("planes", ctypes.POINTER(Plane)), ("length", ctypes.c_uint32), ("reserved2", ctypes.c_uint32),
                ("request_fd", ctypes.c_int32)]


class ReqBufs(ctypes.Structure):
    _fields_ = [("count", ctypes.c_uint32), ("type", ctypes.c_uint32), ("memory", ctypes.c_uint32),
                ("capabilities", ctypes.c_uint32), ("flags", ctypes.c_uint8), ("reserved", ctypes.c_uint8 * 3)]


class PlaneFmt(ctypes.Structure):
    _fields_ = [("sizeimage", ctypes.c_uint32), ("bytesperline", ctypes.c_uint32), ("reserved", ctypes.c_uint16 * 6)]


class PixMp(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("width", ctypes.c_uint32), ("height", ctypes.c_uint32), ("pixelformat", ctypes.c_uint32),
                ("field", ctypes.c_uint32), ("colorspace", ctypes.c_uint32), ("plane_fmt", PlaneFmt * 8),
                ("num_planes", ctypes.c_uint8), ("flags", ctypes.c_uint8), ("enc", ctypes.c_uint8),
                ("quantization", ctypes.c_uint8), ("xfer_func", ctypes.c_uint8), ("reserved", ctypes.c_uint8 * 7)]


class Format(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32), ("pad", ctypes.c_uint32), ("pix_mp", PixMp),
                ("rest", ctypes.c_uint8 * (200 - ctypes.sizeof(PixMp)))]


def _iowr(nr, size, d=3):
    return (d << 30) | (size << 16) | (ord("V") << 8) | nr


VIDIOC_G_FMT = _iowr(4, ctypes.sizeof(Format))
VIDIOC_S_FMT = _iowr(5, ctypes.sizeof(Format))
VIDIOC_REQBUFS = _iowr(8, ctypes.sizeof(ReqBufs))
VIDIOC_QUERYBUF = _iowr(9, ctypes.sizeof(Buffer))
VIDIOC_QBUF = _iowr(15, ctypes.sizeof(Buffer))
VIDIOC_DQBUF = _iowr(17, ctypes.sizeof(Buffer))
VIDIOC_STREAMON = _iowr(18, 4, d=1)
VIDIOC_STREAMOFF = _iowr(19, 4, d=1)

_libc = ctypes.CDLL(None, use_errno=True)
_libc.ioctl.argtypes = [ctypes.c_int, ctypes.c_ulong, ctypes.c_void_p]


def _ioctl(fd, req, obj):
    if _libc.ioctl(fd, req, ctypes.byref(obj)) != 0:
        e = ctypes.get_errno()
        raise OSError(e, f"ioctl 0x{req:x}: {os.strerror(e)}")


class Camera:
    """Open, set NV12 at the sensor size (never another size), stream. read() -> (grey HxW view copy, stamp, seq)."""

    def __init__(self, dev="/dev/video2", nbuf=4):
        assert ctypes.sizeof(Format) == 208 and ctypes.sizeof(Buffer) == 88 and ctypes.sizeof(Plane) == 64
        self.fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
        self.maps, self.streaming = [], False
        try:
            f = Format(type=MPLANE)
            _ioctl(self.fd, VIDIOC_G_FMT, f)
            p = f.pix_mp
            if (p.width, p.height, p.pixelformat) != (SENSOR_W, SENSOR_H, NV12):
                f = Format(type=MPLANE)
                f.pix_mp.width, f.pix_mp.height, f.pix_mp.pixelformat = SENSOR_W, SENSOR_H, NV12
                f.pix_mp.num_planes = 1
                _ioctl(self.fd, VIDIOC_S_FMT, f)
                p = f.pix_mp
            if (p.width, p.height, p.pixelformat) != (SENSOR_W, SENSOR_H, NV12):
                raise RuntimeError(f"{dev} gives {p.width}x{p.height}, not {SENSOR_W}x{SENSOR_H} NV12")
            self.w, self.h, self.stride, self.nplanes = p.width, p.height, p.plane_fmt[0].bytesperline, p.num_planes
            rb = ReqBufs(count=nbuf, type=MPLANE, memory=MEM_MMAP)
            _ioctl(self.fd, VIDIOC_REQBUFS, rb)
            self._planes = [(Plane * VIDEO_MAX_PLANES)() for _ in range(rb.count)]
            for i in range(rb.count):
                b = self._buf(i)
                _ioctl(self.fd, VIDIOC_QUERYBUF, b)
                pl = self._planes[i][0]
                self.maps.append(mmap.mmap(self.fd, pl.length, mmap.MAP_SHARED, mmap.PROT_READ, offset=pl.m))
                _ioctl(self.fd, VIDIOC_QBUF, b)
            _ioctl(self.fd, VIDIOC_STREAMON, ctypes.c_int(MPLANE))
            self.streaming = True
        except Exception:
            self.close()
            raise
        self.ts_flags = None

    def _buf(self, i=0):
        b = Buffer(index=i, type=MPLANE, memory=MEM_MMAP, length=VIDEO_MAX_PLANES)
        b.planes = ctypes.cast(self._planes[i], ctypes.POINTER(Plane))
        return b

    def read(self, timeout=0.5, newest=True, keep=None):
        """Next frame's grey plane (a copy, h x w uint8), its stamp (s, CLOCK_MONOTONIC, end of frame) and sequence.
        newest: drop older queued frames and return the latest. keep(stamp) -> False hands a frame straight
        back to the driver without copying it (the buffers are uncached: a 1920x1080 copy costs ~12 ms on the
        APB, a hand-back 0.03 ms). None on timeout."""
        got = None
        end = time.monotonic() + timeout
        while True:
            wait = 0.0 if got is not None else max(0.0, end - time.monotonic())
            r, _, _ = select.select([self.fd], [], [], wait)
            if not r:
                if got is None and time.monotonic() < end:
                    continue
                break
            b = self._buf()
            try:
                _ioctl(self.fd, VIDIOC_DQBUF, b)
            except BlockingIOError:
                continue
            if keep is not None and not keep(b.timestamp.sec + b.timestamp.usec * 1e-6):
                _ioctl(self.fd, VIDIOC_QBUF, self._buf(b.index))
                continue
            if got is not None:
                _ioctl(self.fd, VIDIOC_QBUF, self._buf(got[0].index))
            got = (b,)
            if not newest:
                break
        if got is None:
            return None
        b = got[0]
        self.ts_flags = b.flags & (TS_MASK | TSRC_MASK)
        y = np.frombuffer(self.maps[b.index], np.uint8, count=self.stride * self.h).reshape(self.h, self.stride)
        img = y[:, :self.w].copy()
        t = b.timestamp.sec + b.timestamp.usec * 1e-6
        seq = b.sequence
        _ioctl(self.fd, VIDIOC_QBUF, self._buf(b.index))
        return img, t, seq

    def stamp_kind(self):
        if self.ts_flags is None:
            return "unknown"
        mono = (self.ts_flags & TS_MASK) == TS_MONO
        eof = (self.ts_flags & TSRC_MASK) == TSRC_EOF
        return ("monotonic" if mono else "other clock") + (", end of frame" if eof else ", start of exposure")

    def close(self):
        if self.streaming:
            try:
                _ioctl(self.fd, VIDIOC_STREAMOFF, ctypes.c_int(MPLANE))
            except OSError:
                pass
            self.streaming = False
        for m in self.maps:
            try:
                m.close()
            except BufferError:        # a numpy view still points into it; the OS unmaps at exit
                pass
        self.maps = []
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


if __name__ == "__main__":       # quick check: python3 v4l2_mmap.py [seconds]
    import sys
    cam = Camera()
    fps = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    n, t0, last, seqs, tk = 0, time.monotonic(), None, [], [-1.0]

    def keep(t):
        return t - tk[0] >= 1.0 / fps - 0.004

    try:
        while time.monotonic() - t0 < float(sys.argv[1] if len(sys.argv) > 1 else 5):
            r = cam.read(newest=False, keep=keep)
            if r is not None:
                tk[0] = r[1]
            if r is None:
                print("timeout")
                continue
            img, t, seq = r
            seqs.append(seq)
            n += 1
            last = (t, time.monotonic(), img.mean())
        dt = time.monotonic() - t0
        span = seqs[-1] - seqs[0]
        print(f"kept {n} frames in {dt:.1f} s = {n / dt:.1f} fps (asked {fps:g}); driver {span / dt:.1f} fps; "
              f"largest sequence step {max(np.diff(seqs)) if n > 1 else 0}; stamps: {cam.stamp_kind()}, "
              f"last stamp age {(last[1] - last[0]) * 1e3:.1f} ms, mean grey {last[2]:.0f}, {cam.w}x{cam.h} stride {cam.stride}")
    finally:
        cam.close()
