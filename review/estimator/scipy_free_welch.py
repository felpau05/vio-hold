"""Welch power spectral density without scipy (one-sided, Hann window, 50 % overlap)."""
import numpy as np


def welch(x, fs, nper):
    x = np.asarray(x, float)
    w = np.hanning(nper)
    step = nper // 2
    segs = [x[i:i + nper] for i in range(0, len(x) - nper + 1, step)]
    p = np.mean([np.abs(np.fft.rfft((s - s.mean()) * w)) ** 2 for s in segs], axis=0)
    p /= fs * np.sum(w ** 2)
    p[1:-1] *= 2
    return np.fft.rfftfreq(nper, 1 / fs), p
