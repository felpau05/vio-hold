#!/usr/bin/env python3
"""Bad pictures (scope 1 and 3; E1, E2, H4, E7): does Hold refuse them, and does it come back alone?

ottawa2019fall at 100 m, sim_ground settings, the drone drifts 0.5 m/s east (so a frozen fix is
wrong). The video is bad from 5 s to 10 s. Nobody calls reanchor(): Hold must recover by itself.
  static std 5 / 30 / 60   analog snow (grey 90 + noise)
  frozen picture           the capture repeats the 5 s buffer
  frozen + fresh noise     the 5 s picture with new sensor noise (std 3) each frame
  NO SIGNAL screen         a grabber's blue screen with text (t10_frozen_video.py)
  breakup bursts           3 broken frames every 0.5 s: a band of rows shifted, impulse noise, brightness step
Also E7: a 3 s blackout (static std 30) while flying 1 m/s north, at 20, 50 and 100 m.
Run: ~/.venvs/vio/bin/python tests/bad_pictures.py      (about 60 s) -> tests/bad_pictures.txt
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2                  # noqa: E402
import numpy as np          # noqa: E402

import harness as X         # noqa: E402

BAD, T = (5.0, 10.0), 14.0
CASES = ["static std 5", "static std 30", "static std 60", "frozen picture", "frozen + fresh noise",
         "NO SIGNAL screen", "breakup bursts"]


def make(v1):
    import hold_v1
    return (hold_v1.Hold if v1 else X.Hold)(X.S.K, X.S.D, X.S.R_BC_NADIR, X.S.READOUT)


def no_signal():
    im = np.full((X.S.H, X.S.W), 60, np.uint8)
    cv2.putText(im, "NO SIGNAL", (95, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 235, 2, cv2.LINE_AA)
    cv2.putText(im, "AV1  NTSC", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, 235, 1, cv2.LINE_AA)
    return im


def breakup(img, rng):
    """One broken analog frame: a band of rows slips sideways, gets impulse noise and a brightness step."""
    out = img.astype(np.int16)
    y0 = int(rng.integers(0, img.shape[0] - 40))
    band = np.roll(out[y0:y0 + 40], int(rng.integers(5, 30)), axis=1) + int(rng.integers(20, 60))
    salt = rng.random(band.shape) < 0.2
    band[salt] = rng.integers(0, 256, salt.sum())
    out[y0:y0 + 40] = band
    return np.clip(out, 0, 255).astype(np.uint8)


def job(case, seed, h0=100, v1=False, reanchor_err=None):
    """v1: run tests/hold_v1.py instead. reanchor_err: the caller re-anchors after 'lost' (sim_test style)."""
    rng = np.random.default_rng(100 + seed)
    held, broken = {}, set()
    ns = no_signal()

    def mod(t0, img):
        if not BAD[0] <= t0 < BAD[1]:
            return img
        if case.startswith("static"):
            sd = float(case.split()[-1])
            return np.clip(90 + rng.normal(0, sd, img.shape), 0, 255).astype(np.uint8)
        if case == "frozen picture":
            return held.setdefault("img", img)
        if case == "frozen + fresh noise":
            return np.clip(held.setdefault("img", img) + rng.normal(0, 3, img.shape), 0, 255).astype(np.uint8)
        if case == "NO SIGNAL screen":
            return np.clip(ns + rng.normal(0, 2, ns.shape), 0, 255).astype(np.uint8)
        if case == "breakup bursts" and (t0 - BAD[0]) % 0.5 < 0.11:
            broken.add(round(t0, 3))
            return breakup(img, rng)
        return img

    p, a, hf = X.S.hover(h0)
    pos = lambda t: p(t) + np.c_[np.zeros(len(t)), 0.5 * np.asarray(t)]
    frames, att, hgt, pos = X.scene(h0=h0, T=T, seed=seed, pos_fn=pos, img_mod=mod)
    rec = X.run(frames, att, hgt, pos, hold=make(v1), reanchor_err=reanchor_err)
    t_frame = np.array([t for t, _ in frames])
    late = (rec.t >= BAD[0] + 0.5) & (rec.t < BAD[1])
    after = rec.t >= BAD[1]
    back = rec.t[after & rec.sent]
    on_broken = np.array([round(t, 3) in broken for t in t_frame])
    kf_before = rec.kfs[rec.t < BAD[0] + 0.5].max()
    return dict(case=case, seed=seed, sent_late=int(np.sum(late & rec.sent)), frames_late=int(late.sum()),
                kf_late=int(rec.kfs[rec.t < BAD[1]].max() - kf_before),
                back=float(back[0] - BAD[1]) if len(back) else np.nan,
                rms_after=rec.stats(after).get("rms", np.nan), conf_wrong=rec.stats().get("conf_wrong", 0),
                sent_broken=int(np.sum(on_broken & rec.sent)), broken=int(on_broken.sum()),
                sent_bad=int(np.sum((rec.t >= BAD[0]) & (rec.t < BAD[1]) & rec.sent)),
                rms_bad=rec.stats((rec.t >= BAD[0]) & (rec.t < BAD[1])).get("rms", np.nan))


def job_e7(h0, seed, v1=False):
    """E7: 3 s of static (std 30) from 5 s while flying 1 m/s north; no outside re-anchor."""
    rng = np.random.default_rng(200 + seed)
    dark = (5.0, 8.0)

    def mod(t0, img):
        return np.clip(90 + rng.normal(0, 30, img.shape), 0, 255).astype(np.uint8) if dark[0] <= t0 < dark[1] else img

    pos = lambda t: np.c_[np.clip(np.asarray(t) - 4.0, 0, 5.0), np.zeros(len(t))]      # 1 m/s north from 4 to 9 s
    frames, att, hgt, pos = X.scene(h0=h0, T=12.0, seed=seed, pos_fn=pos, img_mod=mod)
    rec = X.run(frames, att, hgt, pos, hold=make(v1))
    after = rec.t >= dark[1]
    back = rec.t[after & rec.sent]
    s = rec.stats(after)
    return dict(h=h0, seed=seed, back=float(back[0] - dark[1]) if len(back) else np.nan,
                sent_after=s["sent_share"], err_after=s.get("rms", np.nan), conf_wrong=rec.stats().get("conf_wrong", 0))


def main():
    res = X.pmap(job, [dict(case=c, seed=s) for c in CASES for s in (1, 2, 3)])
    e7 = X.pmap(job_e7, [dict(h0=h, seed=s) for h in (20, 50, 100) for s in (1, 2)])
    lines = [f"Bad video from {BAD[0]:.0f} to {BAD[1]:.0f} s at 100 m, drone drifting 0.5 m/s east, 3 seeds per case, "
             "no reanchor() call.",
             " case                    fixes 5-5.5 s  fixes sent 5.5-10 s  keyframes made 5.5-10 s  fixes back after (s, per seed)  "
             "RMS 10-14 s (m)  confident-wrong  fixes from broken frames"]
    for c in CASES:
        rs = [r for r in res if r["case"] == c]
        lines.append(f" {c:22s}  {sum(r['sent_bad'] - r['sent_late'] for r in rs):4d}           "
                     f"{sum(r['sent_late'] for r in rs):4d} of {sum(r['frames_late'] for r in rs):4d}"
                     f"          {sum(r['kf_late'] for r in rs):3d}                     "
                     f"{' '.join(f'{r['back']:.2f}' for r in rs):24s}       {np.nanmean([r['rms_after'] for r in rs]):5.2f}"
                     f"            {sum(r['conf_wrong'] for r in rs):3d}           "
                     f"{sum(r['sent_broken'] for r in rs)} of {sum(r['broken'] for r in rs)}")
    lines.append("E7: 3 s of static (std 30) from 5 s while flying 1 m/s north; no reanchor() call; 2 seeds")
    lines.append(" h(m)  fixes back after (s)  sent after  RMS after (m)  confident-wrong")
    for h in (20, 50, 100):
        rs = [r for r in e7 if r["h"] == h]
        lines.append(f" {h:4d}  {' '.join(f'{r['back']:.2f}' for r in rs):20s}  {np.mean([r['sent_after'] for r in rs]):6.1%}"
                     f"      {np.nanmean([r['err_after'] for r in rs]):5.2f}           {sum(r['conf_wrong'] for r in rs)}")
    out = "\n".join(lines)
    print(out)
    open(os.path.join(X.HERE, "bad_pictures.txt"), "w").write(out + "\n")


if __name__ == "__main__":
    main()
