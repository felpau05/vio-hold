#!/usr/bin/env python3
"""fwd_cam.py (forward camera as an attitude reference), tools/fwd_check.py and tools/fwd_rec.py.

SYNTHETIC PICTURES ONLY: integ/fwdcam/fwd_render.py draws the forward camera over ground/suburb.jpg (a TUNING site)
with its own lens equations and quaternions (review/geometry/indep.py); fwd_cam uses cv2.fisheye and its own Euler
code. Hidden from the code: the true lens ("Round 1 Metashape fisheye"; the code uses "Round 1 independent k2"), the
mount (22 deg up, 1.2 deg turned, 0.8 deg yawed), the FC error at the reference (0.8 deg per axis), and a gyro drift.
Seeds: tuning seeds 1-3 only (the report in integ/fwdcam uses seeds 4-6).

  A lens round trip            B mount built two ways, mount solve
  C horizon on clean pictures  D refusals (ground only, flat grey, noise, a rough skyline with text)
  E FwdRef over a hover at 30 m with noise: tilt change and yaw change errors
  F heading_change: the yaw the gyro missed   G tools/fwd_check.py end to end on a synthetic run
  H tools/fwd_rec.py refuses a missing device and a full disk
Limits in the asserts are ASSUMED from 3 single-picture checks (integ/fwdcam/README.txt). This test has NOT RUN yet
(the simulator lock blocked it on 2026-11-21). They are not a proof for the real camera.
Run: ~/.venvs/vio/bin/python tests_apb/test_fwd_cam.py   (about 1 minute; 2 library threads)
Not in run_all.sh (the lead adds it).
"""
import csv
import math
import os
import sys
import tempfile

os.environ.setdefault("OMP_NUM_THREADS", "2")
import cv2                 # noqa: E402
import numpy as np         # noqa: E402

cv2.setNumThreads(2)
HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path.insert(0, PROJ)
sys.path.insert(0, os.path.join(PROJ, "tools"))
sys.path.insert(0, os.path.join(PROJ, "integ", "fwdcam"))
import fwd_cam as F        # noqa: E402
import fwd_render as FR    # noqa: E402

W, H = 640, 480
TRUE_LENS = "Round 1 Metashape fisheye"
MOUNT = (22.0, 1.2, 0.8)
FAILS = []
d2r, r2d = math.radians, math.degrees


def check(name, ok, detail):
    print(f"{'PASS' if ok else 'FAIL'} {name}: {detail}", flush=True)
    if not ok:
        FAILS.append(name)


def renderer(seed=1, mount=MOUNT, **kw):
    K, D = FR.I.field_K(FR.I.lens_profile(TRUE_LENS), W, H)
    return FR.FwdRenderer(K, D, FR.fwd_mount_true(*mount), W, H, TEX, seed=seed, **kw)


def up_err_deg(hz, att, mount=MOUNT):
    n_true = FR.fwd_mount_true(*mount).T @ F.euler_to_R(*att).T @ F.UP_NED
    return r2d(math.acos(min(1.0, float(hz.n @ n_true))))


def test_lens():
    L = F.Lens.profile(W, H)
    uu, vv = np.meshgrid(np.linspace(0, W - 1, 17), np.linspace(0, H - 1, 13))
    uv = np.stack([uu, vv], -1)
    back = L.project(L.rays(uv))
    e = float(np.abs(back - uv).max())
    check("A lens round trip", e < 0.01, f"largest pixel error {e:.4f} px")
    Lf = F.Lens.profile(W, H // 2)
    r_full, r_field = L.rays(np.array([[100.0, 101.0]])), Lf.rays(np.array([[100.0, 50.25]]))
    e = r2d(math.acos(min(1.0, float(r_full[0] @ r_field[0]))))
    check("A one-field lens", e < 0.01, f"row 101 of the frame = row 50.25 of the field: {e:.4f} deg apart")


def test_mount():
    for a, r in ((0, 0), (22, 1.2), (30, -3)):
        e = float(np.abs(F.mount_R(d2r(a), d2r(r)) - FR.fwd_mount_true(a, r, 0)).max())
        check(f"B mount {a} deg up, {r} deg turned", e < 1e-9, f"largest difference {e:.1e}")
    att = (d2r(3), d2r(-4), d2r(70))
    n_c = FR.fwd_mount_true(18, -2, 0).T @ F.euler_to_R(*att).T @ F.UP_NED
    al, rho = F.solve_mount(n_c, att[0], att[1])
    check("B mount solve", abs(r2d(al) - 18) < 1e-6 and abs(r2d(rho) + 2) < 1e-6,
          f"solved {r2d(al):.6f} deg up, {r2d(rho):.6f} deg turned (true 18, -2)")


def test_horizon_clean():
    L = F.Lens.profile(W, H)
    for h in (10, 100):
        att = (d2r(4), d2r(-3), d2r(30))
        img = renderer(seed=1, noise=3.7).frame((0.0, 0.0), h, att)
        hz = F.find_horizon(img, L)
        e = up_err_deg(hz, att) if hz.n is not None else 99
        check(f"C horizon {h} m", hz.ok and e < 0.5,
              f"ok {hz.ok} ({hz.why}), quality {hz.quality:.2f}, normal {e:.3f} deg from the true local level "
              f"(includes the horizon dip and the lens mismatch)")


def test_refusals():
    L = F.Lens.profile(W, H)
    att = (d2r(2), d2r(1), 0.0)
    img = renderer(seed=2, mount=(-60.0, 0.0, 0.0), noise=3.7).frame((0.0, 0.0), 30, att)
    hz = F.find_horizon(img, L)
    check("D refuses ground only (camera 60 deg down)", not hz.ok, f"why: {hz.why}")
    rng = np.random.default_rng(3)
    flat = np.clip(128 + rng.normal(0, 3.7, (H, W)), 0, 255).astype(np.uint8)
    hz = F.find_horizon(flat, L)
    check("D refuses a flat grey picture", not hz.ok, f"why: {hz.why}")
    noise = cv2.GaussianBlur(rng.uniform(0, 255, (H, W)).astype(np.float32), (0, 0), 3)
    noise = cv2.normalize(noise, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    hz = F.find_horizon(noise, L)
    check("D refuses texture with no horizon", not hz.ok, f"why: {hz.why}")
    att = (d2r(3), d2r(-2), d2r(10))
    img = renderer(seed=2, noise=3.7, haze_m=2000.0, skyline=True, text=True).frame((0.0, 0.0), 5, att)
    hz = F.find_horizon(img, L)
    e = up_err_deg(hz, att) if hz.n is not None else 99
    check("D rough skyline at 5 m: refuses or is right", (not hz.ok) or e < 0.5,
          f"ok {hz.ok} ({hz.why}), normal {e:.2f} deg from level")


def hover_run(h, seed, n_pics, **kw):
    rend = renderer(seed=seed, **kw)
    pos_fn, att_fn, _, off = FR.hover(h, seed)
    rend.centre = off
    rng = np.random.default_rng(seed + 500)
    bias = np.radians([0.05, 0.05, 0.3]) * rng.choice([-1, 1], 3)
    fc_err = np.radians(0.8) * rng.choice([-1, 1], 2)
    ref = F.FwdRef(F.Lens.profile(W, H // 2 if kw.get("field") else H), height_m=float(h))
    t_ref = a0 = None
    er, ep, ey, n_tilt, n_yaw = [], [], [], 0, 0
    for k in range(n_pics):
        t = 1.0 + k / 5.0
        a = att_fn(np.array([t]))[0]
        img = rend.frame(pos_fn(np.array([t]))[0], h, a)
        if t_ref is None:
            if ref.set_reference(img, t, (a[0] + fc_err[0], a[1] + fc_err[1], a[2])) is None:
                t_ref, a0 = t, a
            continue
        g = a + bias * (t - t_ref)
        st = ref.update(img, t, tuple(g))
        if st.tilt_ok:
            n_tilt += 1
            er.append(r2d(F.wrap(st.droll - F.wrap(a[0] - a0[0]))))
            ep.append(r2d(st.dpitch - (a[1] - a0[1])))
        if st.yaw_ok:
            n_yaw += 1
            ey.append(r2d(F.wrap(st.dyaw - F.wrap(a[2] - a0[2]))))
    return np.array(er), np.array(ep), np.array(ey), n_tilt, n_yaw, n_pics - 1


def test_fwdref():
    er, ep, ey, nt, ny, n = hover_run(30, 2, 41, noise=3.7)
    tilt = np.hypot(er, ep)
    t95 = float(np.percentile(tilt, 95)) if len(tilt) else 99
    y95 = float(np.percentile(np.abs(ey), 95)) if len(ey) else 99
    check("E FwdRef 30 m tilt change", nt >= 0.9 * n and t95 < 0.2,
          f"{nt}/{n} pictures with a horizon, tilt error p50 {np.median(tilt):.3f} p95 {t95:.3f} deg")
    check("E FwdRef 30 m yaw change", ny >= 0.9 * n and y95 < 0.3,
          f"{ny}/{n} pictures with a rotation, yaw error p50 {np.median(np.abs(ey)):.3f} p95 {y95:.3f} deg "
          f"(gyro drift 0.3 deg/s removed)")


def test_heading_change():
    rend = renderer(seed=3, noise=3.7)
    L = F.Lens.profile(W, H)
    a0 = np.array([d2r(2), d2r(-3), d2r(40)])
    a1 = a0 + np.array([d2r(1), d2r(0.5), d2r(5)])
    i0, i1 = rend.frame((0.0, 0.0), 100, a0), rend.frame((0.3, 0.2), 100, a1)
    gyro1 = a1 - np.array([0, 0, d2r(1.0)])                       # the gyro missed 1 deg of yaw
    R_bc = F.mount_R(*F.solve_mount(F.find_horizon(i0, L).n, a0[0], a0[1]))
    rem, rot = F.heading_change(i0, i1, L, R_bc, tuple(a0), tuple(gyro1), band_deg=F.yaw_band_deg(100))
    ok = rem is not None and abs(r2d(rem) - 1.0) < 0.1
    check("F heading_change", ok, f"remaining yaw {r2d(rem) if rem is not None else float('nan'):.3f} deg "
          f"(true 1.000), {rot.n_inl} far points, {rot.why}")


def test_fwd_check():
    import fwd_check
    tmp = tempfile.mkdtemp(prefix="fwdchk_")
    rend = renderer(seed=1, noise=3.7, field=True)
    pos_fn, att_fn, _, off = FR.hover(30, 1)
    rend.centre = off
    T0 = 1000.0                                                    # monotonic clock of the board
    ts = T0 + np.arange(0, 12, 0.2)
    imgs = np.array([rend.frame(pos_fn(np.array([t - T0]))[0], 30, att_fn(np.array([t - T0]))[0]) for t in ts])
    L = F.Lens.profile(W, H // 2)
    delay = 0.06
    np.savez(os.path.join(tmp, "fwd_x_000.npz"), imgs=imgs, t_grab=ts + delay, K=L.K, D=L.D)
    ta = T0 - 1 + np.arange(0, 14, 0.02)                           # FC attitude at 50 Hz, with rates
    A = att_fn(ta - T0)
    fc_bias = np.radians([0.0, 0.0, 0.0])
    rates = np.gradient(A, ta, axis=0)                              # Euler rates; body rates below
    rows = []
    for k, t in enumerate(ta):
        r, p, y = A[k]
        dr, dp, dy = rates[k]
        P = dr - math.sin(p) * dy
        Q = math.cos(r) * dp + math.sin(r) * math.cos(p) * dy
        Rr = -math.sin(r) * dp + math.cos(r) * math.cos(p) * dy
        rows.append([t, r + fc_bias[0], p + fc_bias[1], y, P, Q, Rr])
    np.savez(os.path.join(tmp, "raw_x_000.npz"), att=np.array(rows), imgs=np.zeros((0, 2, 2), np.uint8),
             t_grab=np.zeros(0), hgt=np.zeros((0, 2)), lpos=np.zeros((0, 4)), K=np.eye(3), D=np.zeros(4))
    with open(os.path.join(tmp, "run_x.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "state", "n", "north", "east", "var", "sent", "why", "proc_ms", "age_ms", "height", "note",
                    "mode"])
        for t in T0 + np.arange(0, 12, 0.1):
            note = "engaged: FC mode LOITER" if abs(t - T0 - 1.0) < 0.05 else (
                "released: FC mode ALT_HOLD" if abs(t - T0 - 11.0) < 0.05 else "")
            w.writerow([f"{t:.4f}", "track", 50, 0, 0, 0.01, 1, "", 5.0, 50.0, 30.0, note, "LOITER"])
    out_txt = os.path.join(tmp, "out.txt")
    import contextlib
    with open(out_txt, "w") as fo, contextlib.redirect_stdout(fo):
        rc = fwd_check.main(["--fwd", os.path.join(tmp, "fwd_*.npz"), "--run", os.path.join(tmp, "run_x.csv"),
                             "--raw", os.path.join(tmp, "raw_*.npz"), "--delay-s", str(delay)])
    txt = open(out_txt).read()
    print(txt.strip())
    lines = [ln for ln in txt.splitlines() if ln.startswith("hold ")]
    ok = rc == 0 and len(lines) == 1 and "NO RESULT" not in lines[0]
    p95 = float(lines[0].split("p95 ")[1].split(" ")[0]) if ok else 99
    check("G fwd_check one hold", ok and p95 < 0.3, f"rc {rc}, {len(lines)} hold line(s), tilt FC-camera p95 {p95}")


def test_fwd_rec():
    import fwd_rec
    rc = fwd_rec.main(["--device", "/dev/video_does_not_exist", "--dir", tempfile.mkdtemp(prefix="fwdrec_")])
    check("H fwd_rec missing device", rc == 2, f"rc {rc}")
    rc = fwd_rec.main(["--device", "/dev/null", "--dir", tempfile.mkdtemp(prefix="fwdrec_"), "--min-free-gb", "1e9"])
    check("H fwd_rec full disk", rc == 2, f"rc {rc}")


if __name__ == "__main__":
    TEX = FR.ground_texture("suburb")
    for fn in (test_lens, test_mount, test_horizon_clean, test_refusals, test_fwdref, test_heading_change,
               test_fwd_check, test_fwd_rec):
        fn()
    print(f"{len(FAILS)} FAILED: {FAILS}" if FAILS else "test_fwd_cam PASSED (synthetic pictures only)")
    sys.exit(1 if FAILS else 0)
