"""APB IMU (ICM-42605 gyro + accel) and barometer (DPS310) through Linux IIO, in the camera's clock.

IMU: buffered mode (/dev/iio:deviceN), one thread per device. The timestamp clock is set to monotonic, the
camera's clock (frame stamps are CLOCK_MONOTONIC). Rows: (t, x, y, z) in rad/s or m/s^2, sensor axes.
Baro: in_pressure_input (blocking until a new sample), oversampling 4 at 8 Hz (measured 2026-11-21: about
6 cm per sample; the driver default 1x at 1 Hz gave 20 cm). Rows: (t, pressure Pa).

Everything this changes is runtime sysfs state (no boot or device-tree change); close() puts it back.
Measured on the event APB: device0 dps310, device1 icm42605-gyro, device2 icm42605-accel; the gyro has a
steady offset of about (-17.5, -17.1, +14.8) deg/s, so remove a still-time bias before use.
"""
import glob
import os
import threading
import time
from collections import deque

import numpy as np

IIO = "/sys/bus/iio/devices"


def find(name):
    for d in glob.glob(f"{IIO}/iio:device*"):
        try:
            if open(f"{d}/name").read().strip() == name:
                return d
        except OSError:
            pass
    return None


def _rd(p):
    return open(p).read().strip()


def _wr(p, v):
    with open(p, "w") as f:
        f.write(str(v))


def _scan_layout(d):
    """Enabled scan elements in index order -> numpy record dtype. Each element is aligned to its own storage
    size and the record to its largest element (IIO rule). Byte order is per element: the ICM-42605 sends
    be:s16 axes and an le:s64 timestamp (measured)."""
    els = []
    for en in glob.glob(f"{d}/scan_elements/*_en"):
        if _rd(en) != "1":
            continue
        base = en[:-3]
        idx = int(_rd(base + "_index"))
        t = _rd(base + "_type")                       # e.g. be:s16/16>>0
        endian, rest = t.split(":")
        sign, bits = rest[0], rest[1:].split("/")
        store = int(bits[1].split(">>")[0])
        if ">>" in bits[1] and int(bits[1].split(">>")[1]) != 0 or int(bits[0]) != store:
            raise RuntimeError(f"{base}: packed or shifted samples ({t}) are not handled")
        els.append((idx, os.path.basename(base).replace("in_", ""), endian, sign, store // 8))
    els.sort()
    names, formats, offsets, off = [], [], [], 0
    for _, name, endian, sign, n in els:
        off += (-off) % n
        names.append(name)
        formats.append((">" if endian == "be" else "<") + ("i" if sign == "s" else "u") + str(n))
        offsets.append(off)
        off += n
    align = max([e[4] for e in els] or [1])
    off += (-off) % align
    return np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": off})


class ImuStream:
    """One IIO IMU device (gyro or accel) in buffered mode. rows: deque of (t, x, y, z) in SI units."""

    def __init__(self, name, chan, hz=200, keep_s=30.0, start=True):
        self.d = find(name)
        if self.d is None:
            raise RuntimeError(f"no IIO device named {name}")
        self.name, self.chan = name, chan
        self.orig = {}
        self.rows = deque(maxlen=int(keep_s * (hz or 200) * 1.2))
        self.lock = threading.Lock()
        self.count, self.err = 0, None
        self._stop = False
        self.fd = None
        try:
            self._set("buffer/enable", 0)
            for f in glob.glob(f"{self.d}/scan_elements/*_en"):
                want = 1 if (os.path.basename(f).startswith(f"in_{chan}_") or "timestamp" in f) else 0
                self._set(os.path.relpath(f, self.d), want)
            self._set("current_timestamp_clock", "monotonic")
            if hz is not None:
                self._set("sampling_frequency", hz)
            self._set("buffer/length", 256)
            self.hz = float(_rd(f"{self.d}/sampling_frequency"))
            self.scale = float(_rd(f"{self.d}/in_{chan}_scale"))
            self.dtype = _scan_layout(self.d)
            self.size = self.dtype.itemsize
            if not {f"{chan}_x", f"{chan}_y", f"{chan}_z", "timestamp"} <= set(self.dtype.names):
                raise RuntimeError(f"{name}: scan elements {self.dtype.names} lack {chan} x/y/z or timestamp")
        except BaseException:
            self._restore()
            raise
        self._t = None
        if start:
            self.start()

    def set_rate(self, hz):
        self._set("sampling_frequency", hz)
        self.hz = float(_rd(f"{self.d}/sampling_frequency"))

    def start(self):
        """Enable the buffer and read it. For both gyro and accel use open_imu: they share the chip's FIFO,
        and a device already streaming when the other one starts at a new rate keeps the old stamp spacing
        (measured: 19.6 ms stamps for 5 ms data, 15 s ahead after 5 s)."""
        try:
            self._set("buffer/enable", 1)
            self.fd = os.open(f"/dev/{os.path.basename(self.d)}", os.O_RDONLY)
        except BaseException:
            self._restore()
            raise
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _set(self, rel, v):
        p = f"{self.d}/{rel}"
        if rel not in self.orig:
            self.orig[rel] = _rd(p)
        if _rd(p) != str(v):
            _wr(p, v)

    def _run(self):
        buf = b""
        while not self._stop:
            try:
                buf += os.read(self.fd, self.size * 64)
            except OSError as e:
                self.err = str(e)
                time.sleep(0.01)
                continue
            n = len(buf) // self.size
            if not n:
                continue
            r = np.frombuffer(buf, self.dtype, count=n)
            c = self.chan
            out = np.column_stack([r["timestamp"] * 1e-9, r[f"{c}_x"] * self.scale, r[f"{c}_y"] * self.scale,
                                   r[f"{c}_z"] * self.scale])
            buf = buf[n * self.size:]
            with self.lock:
                self.rows.extend(map(tuple, out.tolist()))
                self.count += n

    def since(self, t):
        with self.lock:
            return [r for r in self.rows if r[0] > t]

    def close(self):
        self._stop = True
        try:
            _wr(f"{self.d}/buffer/enable", 0)       # wakes the blocking read
        except OSError:
            pass
        if self._t is not None:
            self._t.join(timeout=1.0)
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        self._restore()

    def _restore(self):
        """Put back every sysfs value this changed; the buffer must be off while the others are written."""
        try:
            _wr(f"{self.d}/buffer/enable", 0)
        except OSError:
            pass
        for rel, v in self.orig.items():
            if rel != "buffer/enable":
                try:
                    _wr(f"{self.d}/{rel}", v)
                except OSError:
                    pass
        try:
            _wr(f"{self.d}/buffer/enable", self.orig.get("buffer/enable", "0"))
        except OSError:
            pass


class Baro:
    """DPS310 pressure at ov x hz. rows: deque of (t, Pa). height(t0, t1) helpers live in the caller."""

    KEYS = ("in_pressure_oversampling_ratio", "in_pressure_sampling_frequency")

    def __init__(self, ov=4, hz=8, keep_s=60.0):
        self.d = find("dps310")
        if self.d is None:
            raise RuntimeError("no IIO device named dps310")
        self.orig = {k: _rd(f"{self.d}/{k}") for k in self.KEYS}
        _wr(f"{self.d}/{self.KEYS[0]}", ov)
        _wr(f"{self.d}/{self.KEYS[1]}", hz)
        self.rows = deque(maxlen=int(keep_s * hz * 1.5))
        self.lock = threading.Lock()
        self.count, self.err = 0, None
        self._stop = False
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    SKIP = 3            # readings dropped after the rate change (measured: the first one can be thousands of Pa off)
    JUMP_PA = 200.0     # ~17 m: a reading this far from the median of the last 9 is a glitch, not motion

    def _run(self):
        prev, skip, recent = None, self.SKIP, deque(maxlen=9)
        self.rejected = 0
        while not self._stop:
            try:
                p = float(_rd(f"{self.d}/in_pressure_input")) * 1000.0
            except OSError as e:
                self.err = str(e)
                time.sleep(0.05)
                continue
            t = time.monotonic()
            if p == prev:                       # a read can return the same sample twice
                continue
            prev = p
            if skip:
                skip -= 1
                continue
            if len(recent) >= 3 and abs(p - float(np.median(recent))) > self.JUMP_PA:
                self.rejected += 1
                continue
            recent.append(p)
            with self.lock:
                self.rows.append((t, p))
                self.count += 1

    def temp_c(self):
        return float(_rd(f"{self.d}/in_temp_input")) / 1000.0      # blocks up to ~1 s at the default temp rate

    def since(self, t):
        with self.lock:
            return [r for r in self.rows if r[0] > t]

    def close(self):
        self._stop = True
        self._t.join(timeout=1.5)
        for k, v in self.orig.items():
            try:
                _wr(f"{self.d}/{k}", v)
            except OSError:
                pass


def open_imu(hz=200):
    """(gyro, accel) streams with correct stamps: start both at the rate they have, then change the rate while
    both run (the only order of the ones tried on the APB that gave right stamp spacing on both, 2026-11-21).
    Rows from before the change are dropped."""
    g = ImuStream("icm42605-gyro", "anglvel", hz=None, start=False)
    try:
        a = ImuStream("icm42605-accel", "accel", hz=None, start=False)
    except BaseException:
        g.close()
        raise
    try:
        g.start()
        a.start()
        time.sleep(0.2)
        g.set_rate(hz)
        a.set_rate(hz)
        time.sleep(0.3)
        for s in (g, a):
            with s.lock:
                s.rows.clear()
    except BaseException:
        g.close()
        a.close()
        raise
    return g, a


def pa_to_m(dp_pa, p_pa=101325.0, t_c=20.0):
    """Height change for a pressure change (hydrostatic, small changes): dh = -dp * R T / (g M p)."""
    return -dp_pa * 287.05 * (t_c + 273.15) / (9.80665 * p_pa)


def soc_temp_c():
    try:
        return float(_rd("/sys/class/thermal/thermal_zone0/temp")) / 1000.0
    except OSError:
        return float("nan")


if __name__ == "__main__":      # quick check: python3 apb_sensors.py [seconds]
    import sys
    sec = float(sys.argv[1]) if len(sys.argv) > 1 else 5
    g, a = open_imu()
    b = Baro()
    try:
        t0 = time.monotonic()
        time.sleep(sec)
        for s in (g, a):
            r = np.array(s.since(0))
            dt = np.diff(r[:, 0]) if len(r) > 2 else np.array([0])
            print(f"{s.name}: {s.count} rows, {len(r) / sec:.0f}/s (set {s.hz:g} Hz), stamp age "
                  f"{(time.monotonic() - r[-1, 0]) * 1e3:.1f} ms, dt median {np.median(dt) * 1e3:.2f} ms max "
                  f"{dt.max() * 1e3:.1f} ms, mean {np.round(r[:, 1:].mean(0), 4)}, sd {np.round(r[:, 1:].std(0), 4)}"
                  + (f", error {s.err}" if s.err else ""))
        r = np.array(b.since(0))
        print(f"baro: {len(r) / sec:.1f}/s, sd {r[:, 1].std():.2f} Pa = {abs(pa_to_m(r[:, 1].std())):.3f} m, "
              f"rejected {b.rejected}" + (f", error {b.err}" if b.err else ""))
    finally:
        g.close(); a.close(); b.close()
        print("restored:", _rd(f"{g.d}/current_timestamp_clock"), _rd(f"{g.d}/sampling_frequency"),
              _rd(f"{b.d}/in_pressure_sampling_frequency"))
