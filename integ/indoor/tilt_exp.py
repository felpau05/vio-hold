"""Open-loop experiment: can a joint fit of tilt and slide against the keyframe measure the change of the gyro tilt
error since the keyframe (what turns into position error: h x that change)?

Truth: flat ground, camera.py's own lens functions with the TRUE lens; the estimator side fits with the MODEL lens
(focal scale off by focal_err), an attitude whose tilt error changed by dtilt since the keyframe, a frame time stamp
off by time_err at a body rate of rate_dps, LK noise 0.1 px (assumed), points spread over the picture inside 55 deg.
Unknowns per frame: slide (N, E), tilt change (roll, pitch), yaw, height scale. Then a SLOW corrector: the mean of the
tilt estimate over n_avg frames. Compared: position error of the plain fit (tilt from the gyro, as Hold does now)
and of the fit with the corrector. Seeds 11-19 (validation). Run: ~/.venvs/vio/bin/python integ/indoor/tilt_exp.py
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "px4sim"))
import camera as C  # noqa: E402

W, H = 360, 288
K_T, D_T = C.lens("Round 1 Metashape fisheye", W, H)     # true lens (sim default)
DEG = math.pi / 180


def rot(rx, ry, rz):
    """Small rotation: body-frame turn by (roll, pitch, yaw) as a matrix (exact Rodrigues)."""
    v = np.array([rx, ry, rz], float)
    a = np.linalg.norm(v)
    if a < 1e-15:
        return np.eye(3)
    k = v / a
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * Kx + (1 - math.cos(a)) * Kx @ Kx


def pixels(P, pos, h, R, K, D):
    """Pixels of ground points P (N x 2) from camera at (pos, height h) with camera->NED rotation R."""
    v = np.c_[P - pos, np.full(len(P), h)] @ R          # NED -> camera (R columns: camera axes in NED)
    u, w = C.project(v, K, D)
    return np.c_[u, w]


def rays_ground(px, pos, h, R, K, D):
    r = C.unproject(px[:, 0], px[:, 1], K, D) @ R.T
    return pos + h * r[:, :2] / r[:, 2:3]


def fit(px, G, h, R_est, K, D, tilt=True):
    """Gauss-Newton on (dn, de, droll, dpitch, dyaw, dscale): G seen at px from pos with attitude R_est o turn."""
    p = np.zeros(6)
    free = [0, 1, 2, 3, 4, 5] if tilt else [0, 1, 4, 5]
    for _ in range(6):
        def f(q):
            R = R_est @ rot(q[2], q[3], q[4])
            return (pixels(G, q[:2], h * (1 + q[5]), R, K, D) - px).ravel()
        r0 = f(p)
        J = np.zeros((len(r0), 6))
        for k in free:
            dq = np.zeros(6)
            dq[k] = 1e-6 if k >= 2 else 1e-4
            J[:, k] = (f(p + dq) - r0) / dq[k]
        Jf = J[:, free]
        step = np.linalg.lstsq(Jf, -r0, rcond=None)[0]
        p[free] += step
        if np.abs(step).max() < 1e-9:
            break
    return p


def keyframe(rng, h, focal_err, n_pts=100, lk=0.1):
    """Ground points seen in the keyframe (true P, the estimator's G from the model lens and an attitude with
    tilt error e0), and the model lens."""
    K_M = K_T.copy()
    K_M[0, 0] *= 1 + focal_err
    K_M[1, 1] *= 1 + focal_err
    Rc = C.mount()                                         # camera -> body (nadir, top = front)
    lean = rng.normal(0, 2 * DEG, 2)
    Rb0 = rot(lean[0], lean[1], 0)
    e0 = rng.normal(0, 0.15 * DEG, 2)
    R0, R0e = Rb0 @ Rc, Rb0 @ rot(e0[0], e0[1], 0) @ Rc
    th = np.sqrt(rng.uniform(0, 1, 4 * n_pts)) * 55 * DEG
    ph = rng.uniform(0, 2 * np.pi, 4 * n_pts)
    dirs = np.c_[np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)] @ R0.T
    P = h * dirs[:, :2] / dirs[:, 2:3]
    px0 = pixels(P, np.zeros(2), h, R0, K_T, D_T)
    ok = (px0[:, 0] > 5) & (px0[:, 0] < W - 5) & (px0[:, 1] > 5) & (px0[:, 1] < H - 5)
    P, px0 = P[ok][:n_pts], px0[ok][:n_pts]
    px0 = px0 + rng.normal(0, lk, px0.shape)
    G = rays_ground(px0, np.zeros(2), h, R0e, K_M, D_T)
    return P, G, e0, K_M, Rc


def frame(rng, h, P, G, e0, K_M, Rc, time_err, rate_dps, e1, slide_m, lk=0.1):
    """One later frame: slid by slide_m (random direction), a new lean, tilt error e1, stamp error x body rate.
    Returns true slide, the estimator's attitude R1e, the pixels, and the true tilt change since the keyframe."""
    d = rng.normal(0, 1, 2)
    pos1 = slide_m * d / np.linalg.norm(d)
    lean1 = rng.normal(0, 2 * DEG, 2)
    Rb1 = rot(lean1[0], lean1[1], 0)
    w = rng.normal(0, 1, 2)
    et = rate_dps * DEG * w / np.linalg.norm(w) * time_err
    R1, R1e = Rb1 @ Rc, Rb1 @ rot(e1[0] + et[0], e1[1] + et[1], 0) @ Rc
    px1 = pixels(P, pos1, h, R1, K_T, D_T) + rng.normal(0, lk, (len(P), 2))
    return pos1, R1e, px1, (e1 + et) - e0


def main():
    out = []
    say = lambda s: (out.append(s), print(s, flush=True))   # noqa: E731
    say("Per keyframe: tilt error changes by 0.15 deg (random direction); 50 frames, slide 0-2 m, lean 2 deg, rate 3 deg/s.")
    say("h_m focal time_ms | plain fit pos err cm rms | joint fit tilt-change err deg: bias sd (per frame) | "
        "slow corrector (mean tilt of 50 frames) pos err cm rms | wins?")
    for h in (20.0, 60.0, 100.0):
        for fe, te in ((0.0, 0.0), (0.02, 0.005), (0.05, 0.01), (-0.05, 0.01)):
            plain, corr, terr = [], [], []
            for seed in range(11, 20):
                rng = np.random.default_rng(seed)
                P, G, e0, K_M, Rc = keyframe(rng, h, fe)
                a = rng.uniform(0, 2 * np.pi)
                e1 = e0 + 0.15 * DEG * np.array([math.cos(a), math.sin(a)])
                fr = [frame(rng, h, P, G, e0, K_M, Rc, te, 3.0, e1, rng.uniform(0, 2.0)) for _ in range(50)]
                tilts = []
                for pos1, R1e, px1, tdt in fr:
                    qp = fit(px1, G, h, R1e, K_M, D_T, tilt=False)
                    qj = fit(px1, G, h, R1e, K_M, D_T, tilt=True)
                    plain.append(np.hypot(*(qp[:2] - pos1)))
                    tilts.append(qj[2:4])
                    terr.append((qj[2:4] - tdt) / DEG)
                c = np.mean(tilts, 0)                       # the slow corrector's tilt change
                for pos1, R1e, px1, tdt in fr:
                    qc = fit(px1, G, h, R1e @ rot(c[0], c[1], 0), K_M, D_T, tilt=False)
                    corr.append(np.hypot(*(qc[:2] - pos1)))
            terr = np.array(terr)
            r = lambda x: 100 * float(np.sqrt(np.mean(np.square(x))))   # noqa: E731
            say(f"{h:5.0f} {fe:+.2f} {1e3 * te:4.0f} | {r(plain):6.1f} | bias {np.round(terr.mean(0), 3)} sd "
                f"{np.round(terr.std(0), 3)} | {r(corr):6.1f} | {'yes' if r(corr) < r(plain) else 'no'}")
    open(os.path.join(HERE, "tilt_exp.txt"), "w").write("\n".join(out) + "\n")


if __name__ == "__main__":
    main()
