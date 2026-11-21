"""Live position on the APB from its own sensors, with a web display for the bench tests and walk tests.

  PYTHONPATH=/data/vio/pylib python3 apb_live.py [--port 8080] [--fps 30] [--seconds N] [--rec LABEL]

Open http://192.168.75.1:8080 on the laptop. The page shows READY / NOT READY, forward and right (m, body axes at
the zero), distance from the zero, height change, and why it is not ready. Buttons: "Zero here" (position and
height zero at the current place) and "Floor zero" (the drone sits on the floor now: barometer zero).

Start-up: put the drone down still (gyro bias, 2 s) on the floor (barometer zero, first 2 s), then lift it.
Not ready while: no gyro bias yet, lens covered or blurred, picture flat, height unknown, the estimator not
locked (Pipe needs good_before_send good fixes in a row), or the barometer silent for over HGT_HOLD_S.
Inputs are the APB camera, IMU and barometer only: no FC, no GNSS.
"""
import argparse
import json
import math
import os
import shutil
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

import apb_est as E
import apb_lift as LF
import apb_sensors as A
import capture
import settings as S
from hold import Series, attitude_R, make_R_bc
from v4l2_mmap import Camera

COVER_FINE = 3.0      # fine-detail energy below this: lens covered or blurred. Desk test 2026-11-21: hand 1.3-1.7,
                      # sliding (motion blur) 4.9 at worst, normal 16-19
LIFT_H0_M = 1.0      # m; lens height assumed at "Zero here" for the camera height change (B3), when --height is not set.
                     # The judge holds the drone about 1 m up: a true 0.77-1.43 m keeps a 0.5 m lift within B3's 0.15 m.
CAM_STALL_S = 5.0     # s with no picture (after the first one): the camera watchdog stops the program (exit 1)
CAM_FIRST_S = 30.0    # s to wait for the first picture (the Justice D format is set about 8 s after power-up)
LIFT_MAX_ERR = 20     # floor-track errors (each one a gap) before it turns off for the run
HGT_HOLD_S = 5.0      # s; the last barometer height is carried forward this long (the APB barometer froze >1 min twice)
ATT_WAIT_S = 0.015    # s; wait at most this long for the gyro to cover a frame


def fine_detail(img):
    """Mean |picture - blurred picture| in the centre half: blur and a lens cover kill this first.
    uint8 OpenCV ops (the float numpy version cost 6.5 ms per frame on the APB)."""
    h, w = img.shape
    c = np.ascontiguousarray(img[h // 4:3 * h // 4, w // 4:3 * w // 4])
    return float(cv2.mean(cv2.absdiff(c, cv2.GaussianBlur(c, (0, 0), 2.0)))[0])


class Recorder:
    """apb_rec.py-format chunks (replay with apb_est.py replay) written from the live program: every frame the
    grabber gets, plus the IMU and barometer rows. Saving runs in its own thread."""

    MIN_FREE_MB = 300

    def __init__(self, base, gyro, acc, baro, fps=15.0, say=print):
        self.base, self.gyro, self.acc, self.baro, self.fps, self.say = base, gyro, acc, baro, fps, say
        self.imgs, self.ts, self.seqs, self.k, self.savers = [], [], [], 0, []
        self.t_keep, self.full = -1.0, False

    def add(self, img, t, seq):
        if self.full or t - self.t_keep < 1.0 / self.fps - 0.004:
            return
        self.t_keep = t
        self.imgs.append(img)
        self.ts.append(t)
        self.seqs.append(seq)
        if self.ts[-1] - self.ts[0] >= 10.0:
            self.flush()

    def flush(self):
        if not self.imgs:
            return
        lo, hi = self.ts[0] - 1.0, self.ts[-1] + 1.0
        cut = lambda rows: np.array([r for r in rows if lo <= r[0] <= hi])     # noqa: E731
        data = dict(imgs=self.imgs, t_frame=np.array(self.ts), seq=np.array(self.seqs), gyro=cut(self.gyro.since(lo)),
                    accel=cut(self.acc.since(lo)), baro=cut(self.baro.since(lo)), soc_c=np.array(A.soc_temp_c()))
        path = f"{self.base}_{self.k:03d}.npz"
        self.k += 1
        self.imgs, self.ts, self.seqs = [], [], []

        def save():
            data["imgs"] = np.stack(data["imgs"])
            np.savez(path, **data)
        th = threading.Thread(target=save, daemon=False)
        th.start()
        self.savers.append(th)
        free = shutil.disk_usage(os.path.dirname(self.base)).free / 1e6
        if free < self.MIN_FREE_MB + 160:           # room for one more chunk (~75-160 MB) above the floor
            self.full = True
            self.say(f"recording stopped: {free:.0f} MB free (live mode goes on)")

    def close(self):
        self.flush()
        for th in self.savers:
            th.join()


class Grabber:
    """Camera reads and halving in their own thread: the copy out of the uncached driver buffer (~12 ms) then
    overlaps the processing of the previous frame. get() returns the newest frame not returned before.
    rec: a Recorder that gets every frame (the estimator may skip some when it runs slower than the camera)."""

    def __init__(self, cam, fps, rec=None):
        self.cam, self.fps, self.rec = cam, fps, rec
        self.cv = threading.Condition()
        self.latest, self.got, self.stop, self.err = None, -1, False, None
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def _run(self):
        tk = -1.0
        try:
            while not self.stop:
                r = self.cam.read(newest=True, keep=lambda t: t - tk >= 1.0 / self.fps - 0.004)
                if r is None:
                    continue
                y, t, seq = r
                tk = t
                img = cv2.resize(y, (y.shape[1] // 2, y.shape[0] // 2), interpolation=cv2.INTER_AREA)
                if self.rec is not None:
                    self.rec.add(img, t, seq)
                with self.cv:
                    self.latest = (img, t, seq)
                    self.cv.notify_all()
        except Exception as ex:                         # noqa: BLE001  the main loop sees the dead thread (watchdog)
            self.err = f"{type(ex).__name__}: {ex}"

    def get(self, timeout=0.5):
        with self.cv:
            if not self.cv.wait_for(lambda: self.latest is not None and self.latest[2] != self.got, timeout):
                return None
            self.got = self.latest[2]
            return self.latest

    def close(self):
        self.stop = True
        self.t.join(timeout=1.0)


class Shared:
    """State the web page reads and the buttons write."""

    def __init__(self):
        self.lock = threading.Lock()
        self.state = {"ready": False, "why": "starting"}
        self.zero_req = False
        self.floor_req = False
        self.frame = None           # newest picture (reference; the loop makes a new array each frame), for /frame.jpg

    def jpeg(self, width=480, quality=70):
        """The newest picture as a small grey JPEG, or None. Runs in the HTTP thread: cv2 releases the GIL."""
        img = self.frame
        if img is None:
            return None
        h, w = img.shape[:2]
        small = cv2.resize(img, (width, max(1, round(h * width / w))), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes() if ok else None

    def get(self):
        with self.lock:
            d = dict(self.state)
            t_put = getattr(self, "t_put", None)
        d["state_age_s"] = None if t_put is None else time.monotonic() - t_put     # a stalled loop shows on the page
        return d

    def put(self, **kw):
        with self.lock:
            self.state.update(kw)
            self.t_put = time.monotonic()


PAGE = """<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>VIO live</title><style>
:root{--bg:#f4f4f2;--fg:#1d1d1b;--mut:#6b6b66;--ok:#1f7a3a;--bad:#b3261e;--card:#fff}
@media (prefers-color-scheme:dark){:root{--bg:#141413;--fg:#ececea;--mut:#9a9a95;--ok:#4cc477;--bad:#ff6b5e;--card:#1f1f1d}}
body{margin:0;background:var(--bg);color:var(--fg);font:16px system-ui,sans-serif}
main{max-width:900px;margin:0 auto;padding:16px}
#ready{font-size:44px;font-weight:700;padding:14px 18px;border-radius:10px;color:#fff;background:var(--bad)}
#ready.ok{background:var(--ok)} #why{color:var(--mut);margin:8px 2px 16px;min-height:1.2em}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:12px}
.c{background:var(--card);border-radius:10px;padding:12px 14px}.l{color:var(--mut);font-size:14px}
.v{font-size:40px;font-weight:700;font-variant-numeric:tabular-nums}
button{font-size:18px;padding:12px 18px;margin:16px 8px 0 0;border-radius:8px;border:1px solid var(--mut);background:var(--card);color:var(--fg)}
#info{color:var(--mut);font-size:13px;margin-top:16px;font-variant-numeric:tabular-nums}
#streams{font-size:16px;margin-top:14px;font-variant-numeric:tabular-nums}
#cam{display:block;width:100%;max-width:480px;aspect-ratio:16/9;margin-top:14px;border-radius:8px;background:var(--card)}
</style></head><body><main>
<div id="ready">NOT READY</div><div id="why"></div>
<div class="grid">
<div class="c"><div class="l">Forward (m)</div><div class="v" id="fwd">-</div></div>
<div class="c"><div class="l">Right (m)</div><div class="v" id="right">-</div></div>
<div class="c"><div class="l">Distance from zero (m)</div><div class="v" id="dist">-</div></div>
<div class="c"><div class="l">Height change (m)</div><div class="v" id="dh">-</div></div>
</div>
<button onclick="post('zero')">Zero here</button><button onclick="post('floor')">Floor zero (drone on the floor)</button>
<img id="cam" alt="live Justice D picture"><div id="streams"></div><div id="info"></div></main><script>
// live picture: the next request starts only after the last one arrived (no pile-up on a slow link), about every 0.5 s
(function(){const c=document.getElementById('cam');let t0=0;
function next(){const w=Math.max(0,500-(Date.now()-t0));setTimeout(()=>{t0=Date.now();c.src='/frame.jpg?t='+t0;},w);}
c.onload=next;c.onerror=()=>setTimeout(next,1000);next();})();
const f=(x,d=2)=>x===null||x===undefined?'-':(x>=0?'+':'')+x.toFixed(d);
async function post(p){try{await fetch('/'+p,{method:'POST'})}catch(e){}}
// One request at a time (no old answer after a newer one), 1 s timeout: when the APB goes away (power cut, USB
// gone, a route through Wi-Fi) the page shows NO CONNECTION and no numbers, never an old READY (review E2-03).
// state_age_s: the APB loop has not written the state for this long (a stalled loop): NOT READY.
const id=(k)=>document.getElementById(k);
async function tick(){const ac=new AbortController();const to=setTimeout(()=>ac.abort(),1000);
try{const s=await (await fetch('/state',{cache:'no-store',signal:ac.signal})).json();clearTimeout(to);
const stale=s.state_age_s!=null&&s.state_age_s>1.0, rdy=!!s.ready&&!stale;
const r=id('ready');r.textContent=rdy?'READY':'NOT READY';r.className=rdy?'ok':'';
id('why').textContent=rdy?'':(stale?`APB program loop stalled (${s.state_age_s.toFixed(1)} s)`:(s.why||''));
document.getElementById('fwd').textContent=f(s.fwd);document.getElementById('right').textContent=f(s.right);
document.getElementById('dist').textContent=s.dist==null?'-':s.dist.toFixed(2);document.getElementById('dh').textContent=f(s.dh);
const n1=(x,d=1)=>x==null?'-':x.toFixed(d), sg=(x)=>x==null?'-':(x>=0?'+':'')+x.toFixed(1);
document.getElementById('streams').textContent=`Camera: ${n1(s.fps)} fps, frame age ${n1(s.age_ms,0)} ms · IMU: gyro ${n1(s.gyro_hz,0)} Hz, accel ${n1(s.acc_hz,0)} Hz, age ${n1(s.imu_age_ms,0)} ms, roll ${sg(s.roll_deg)} pitch ${sg(s.pitch_deg)} deg, turn ${n1(s.turn_dps)} deg/s`;
document.getElementById('info').textContent=`${s.pos_info??''} · height change from ${s.dh_src??'-'} · barometer change ${f(s.dh_baro)} m · height ${s.h==null?'-':s.h.toFixed(2)} m · baro ${s.baro_age==null?'none':s.baro_age.toFixed(1)+' s ago'} (${s.baro_n??'-'} rows, ${s.baro_rej??'-'} rejected) · ${s.state||'-'} · ${s.n??'-'} points · ${s.fps?.toFixed(1)??'-'} fps · process ${s.proc_ms?.toFixed(0)??'-'} ms · frame age ${s.age_ms?.toFixed(0)??'-'} ms · SoC ${s.soc?.toFixed(0)??'-'} °C · up ${s.up?.toFixed(0)??'-'} s`;
}catch(e){clearTimeout(to);const r=id('ready');r.textContent='NO CONNECTION';r.className='';
id('why').textContent='no connection to the APB';for(const k of ['fwd','right','dist','dh'])id(k).textContent='-';
id('streams').textContent='';}
finally{setTimeout(tick,200);}}
tick();
</script></body></html>"""


def serve(shared, port):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            b = body if isinstance(body, bytes) else body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path.startswith("/state"):
                self._send(200, json.dumps(shared.get()), "application/json")
            elif self.path in ("/", "/index.html"):
                self._send(200, PAGE, "text/html; charset=utf-8")
            elif self.path.startswith("/frame.jpg"):
                jpg = shared.jpeg()
                if jpg is None:
                    self._send(503, "no picture yet", "text/plain")
                else:
                    self._send(200, jpg, "image/jpeg")
            else:
                self._send(404, "not found", "text/plain")

        def do_POST(self):
            if self.path == "/zero":
                with shared.lock:
                    shared.zero_req = True
            elif self.path == "/floor":
                with shared.lock:
                    shared.floor_req = True
            else:
                return self._send(404, "not found", "text/plain")
            self._send(200, "ok", "text/plain")

    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def run(s, port=8080, fps=30.0, seconds=None, log_dir="/data/vio/logs", say=print, fixed_height=None,
        rec_label=None, rec_dir="/data/vio/rec", use_lift=True, pos_src="hold", rc=None):
    """fixed_height (desk tests only): use this lens height (m) instead of the barometer.
    rec_label: also record every frame + IMU + barometer to rec_dir/<label>_<time>_<k>.npz."""
    from apb_main import Log, Pipe, lens
    s = E.bench_settings(s)
    shared = Shared()
    srv = serve(shared, port)
    os.makedirs(log_dir, exist_ok=True)
    log = Log(os.path.join(log_dir, f"live_{time.strftime('%H%M%S')}.csv"))
    cam = Camera()
    gyro, acc = A.open_imu(200)
    baro = A.Baro()
    say(f"live: http://192.168.75.1:{port}  log {log.path}")
    try:
        att_f, bh = E.ApbAtt(s), E.BaroHeight(s)
        att, hgt = Series(30.0), Series(30.0)
        pipe, judge = Pipe(s, lambda t: None), capture._Judge(s)
        lift, R_bc = None, make_R_bc(s["top"], s["mirror"], tuple(s["tilt_deg"]), s["twist_deg"])
        h_lift0 = fixed_height if fixed_height else LIFT_H0_M
        lift_err = []
        odo_yaw = None                                  # yaw at the floor-track zero: forward/right axes

        lift_off = []                                   # set once: the floor track is off for this run

        def guarded(fn):
            # An error in the floor track must never stop the loop, and must never restart the position at 0
            # (a new LiftScale starts at 0: seen in the C test of 2026-11-21 as resets with no zero). An error is a
            # gap that keeps the last values; after LIFT_MAX_ERR errors the floor track turns off for this run.
            nonlocal lift
            if fn is None or fn is False:
                return None
            try:
                return fn()
            except Exception as ex:                     # noqa: BLE001
                lift_err.append(f"{type(ex).__name__}: {ex}")
                if len(lift_err) <= 3 or len(lift_err) % 50 == 0:
                    say(f"floor track error {len(lift_err)}: {lift_err[-1]}")
                if lift is not None and len(lift_err) < LIFT_MAX_ERR:
                    try:
                        lift._gap()                     # keep the last values; the next good picture goes on
                    except Exception:                   # noqa: BLE001
                        pass
                else:
                    lift = None
                    if not lift_off:
                        lift_off.append(True)
                        say(f"floor track off after {len(lift_err)} errors: {lift_err[-1]}")
                return None
        rec = None
        if rec_label:
            os.makedirs(rec_dir, exist_ok=True)
            rec = Recorder(os.path.join(rec_dir, f"{rec_label}_{time.strftime('%H%M%S')}"), gyro, acc, baro, say=say)
            say(f"recording to {rec.base}_*.npz")
        grab = Grabber(cam, fps, rec)
        t_last_frame = None                             # monotonic time of the newest picture (camera watchdog)
        t_start = time.monotonic()
        last_g = last_b = -math.inf
        last_h = None                                   # (t, h) of the newest barometer height
        floor_window = []                               # (t, Pa) for the floor zero
        floor_until = time.monotonic() + 2.0            # first 2 s: on the floor
        zero = None                                     # dict(pos, c, s) of the position zero
        pos_zero_pending = False                        # "Zero here" pressed with no fix: zero at the next fix
        h_zero = None                                   # lens height at the zero (B3). Needs no fix.
        fps_t = deque(maxlen=60)
        imu_n = deque()                                 # (t, gyro rows, accel rows) for the IMU rates (area A)
        while seconds is None or time.monotonic() - t_start < seconds:
            r = grab.get()
            if r is None:
                # Camera watchdog (E2): a dead camera thread, or no picture for CAM_STALL_S (CAM_FIRST_S before the
                # first one: the Justice D format is set about 8 s after power-up), ends the program with an error.
                # finally then makes the FC output neutral, the process exits with code 1, and the boot unit
                # restarts it. Without this the page would show "no picture" until someone restarts by hand.
                waited = time.monotonic() - (t_last_frame if t_last_frame is not None else t_start)
                alive = getattr(getattr(grab, "t", None), "is_alive", lambda: True)()
                why_cam = (f"camera thread stopped ({getattr(grab, 'err', None) or 'no error text'})" if not alive else
                           f"no picture from the camera for {waited:.0f} s"
                           if waited > (CAM_STALL_S if t_last_frame is not None else CAM_FIRST_S) else None)
                if why_cam:
                    shared.put(ready=False, why=why_cam + ": the program stops (the boot unit restarts it)")
                    raise RuntimeError(why_cam)
                shared.put(ready=False, why="no picture from the camera")
                continue
            t_last_frame = time.monotonic()
            img, t_eof, seq = r
            shared.frame = img                          # for /frame.jpg (area A); encoded only on request
            t1 = time.monotonic()
            t0 = t_eof - s["video_delay_s"]
            # IMU: wait briefly until the gyro covers the frame, then feed every new row once
            need = t0 + max(s["readout_s"], 0.0) + 0.006
            while True:
                g = gyro.since(last_g)
                if (g and g[-1][0] >= need) or time.monotonic() - t1 > ATT_WAIT_S:
                    break
                time.sleep(0.002)
            if g:
                a = [x for x in acc.since(last_g - 0.05) if x[0] <= g[-1][0]]
                for row in att_f.update(np.array(g), np.array(a) if a else np.zeros((0, 4))):
                    if row[0] > att.last_t():
                        att.add(*row)
                last_g = g[-1][0]
            # barometer: floor zero, then heights; carry the last height forward while it is silent
            b = baro.since(last_b)
            if b:
                last_b = b[-1][0]
            with shared.lock:
                floor_req, shared.floor_req = shared.floor_req, False
                zero_req, shared.zero_req = shared.zero_req, False
            if fixed_height is not None:
                hgt.add(t0, fixed_height)
                bh.p0 = bh.p0 or 1.0                    # no floor zero needed
            elif floor_req:
                bh.p0, floor_window, floor_until = None, [], time.monotonic() + 1.5
            if fixed_height is not None:
                pass
            elif bh.p0 is None:
                floor_window += b
                if time.monotonic() >= floor_until and floor_window:
                    bh.zero(float(np.mean([x[1] for x in floor_window])))
                    bh.rows = list(floor_window)
                    say(f"floor zero set at {bh.p0:.1f} Pa")
            else:
                for row in bh.update(b):
                    hgt.add(*row)
                    last_h = row
            if last_h is not None and t_eof - last_h[0] < HGT_HOLD_S and hgt.last_t() < t0:
                hgt.add(t0, last_h[1])                  # silent barometer: hold the last height a while
            # picture checks, then the estimator
            fd = fine_detail(img)
            why = judge(img) or (f"lens covered or blurred (detail {fd:.1f})" if fd < COVER_FINE else None)
            pic_bad = why is not None
            if att_f.bias is None:
                why = why or "hold the drone still (gyro bias)"
            elif bh.p0 is None:
                why = why or "measuring the floor (barometer zero)"
            fr = capture.Frame(img, t0, t_eof, seq, why)
            e, ok, why2 = pipe.step(fr, att, hgt)
            proc = (time.monotonic() - t1) * 1e3
            fps_t.append(t_eof)
            h = float(hgt.at(t0)) if hgt.rows else None
            st = dict(ready=bool(ok), why=why2, state=e.state if e else None, n=e.n if e else None, h=h,
                      proc_ms=proc, age_ms=(time.monotonic() - t_eof) * 1e3, soc=A.soc_temp_c(),
                      up=time.monotonic() - t_start,
                      fps=(len(fps_t) - 1) / (fps_t[-1] - fps_t[0]) if len(fps_t) > 2 else None,
                      baro_age=time.monotonic() - last_b if last_b > -math.inf else None,
                      baro_n=baro.count, baro_rej=getattr(baro, "rejected", 0))
            # area A: the IMU stream next to the camera stream (rates from row counts, newest row age, attitude)
            now = time.monotonic()
            imu_n.append((now, getattr(gyro, "count", 0), getattr(acc, "count", 0)))
            while len(imu_n) > 2 and now - imu_n[0][0] > 1.5:
                imu_n.popleft()
            dt_n = now - imu_n[0][0]
            g_rows = getattr(gyro, "rows", None)
            g_last = g_rows[-1] if g_rows else None
            a_last = att.rows[-1] if att.rows else None
            st.update(gyro_hz=(imu_n[-1][1] - imu_n[0][1]) / dt_n if dt_n > 0.3 else None,
                      acc_hz=(imu_n[-1][2] - imu_n[0][2]) / dt_n if dt_n > 0.3 else None,
                      imu_age_ms=(now - g_last[0]) * 1e3 if g_last else None,
                      turn_dps=math.degrees(math.sqrt(g_last[1] ** 2 + g_last[2] ** 2 + g_last[3] ** 2)) if g_last else None,
                      roll_deg=math.degrees(a_last[1]) if a_last else None,
                      pitch_deg=math.degrees(a_last[2]) if a_last else None)
            # height zero: at "Zero here" (at once, fix or not), else at the first position zero.
            # The height change is shown on every frame: a lift can stop the fix, and B3 needs no fix.
            # Floor track (apb_lift): height change (B3) and move (B1/B2) from the picture; restarts at "Zero here".
            if lift is None and use_lift and not lift_off:
                lift = guarded(lambda: LF.LiftScale(*lens(s, img.shape), img.shape, h0=h_lift0))
            if zero_req:
                pos_zero_pending, h_zero = True, h
                guarded(lift and lift.zero)
                odo_yaw = None
            hold_pos = None
            if e is not None and e.var is not None:
                yaw = float(np.interp(t0, *np.array(att.rows)[-200:, [0, 3]].T))
                if zero is None or pos_zero_pending:
                    zero = dict(pos=e.pos[:2].copy(), c=math.cos(yaw), s=math.sin(yaw))
                    h_zero = h if h_zero is None else h_zero
                    pos_zero_pending = False
                d = e.pos[:2] - zero["pos"]
                hold_pos = (float(d[0] * zero["c"] + d[1] * zero["s"]), float(-d[0] * zero["s"] + d[1] * zero["c"]))
            floor_pos = floor_body = None
            if lift is not None:
                if not att.rows:
                    guarded(lift.zero)                  # no attitude yet: do not mix camera axes with level axes
                elif pic_bad:
                    guarded(lift.bad_picture)
                else:
                    tm = t0 + 0.5 * s["readout_s"]
                    guarded(lambda: lift.update(img, attitude_R(att, np.array([tm]))[0] @ R_bc))
                    if lift is not None and lift.disp is not None:
                        yaw_now = float(np.interp(tm, *np.array(att.rows)[-200:, [0, 3]].T))
                        if odo_yaw is None:
                            odo_yaw = yaw_now
                        c, sn = math.cos(odo_yaw), math.sin(odo_yaw)
                        d = lift.disp
                        floor_pos = (float(d[0] * c + d[1] * sn), float(-d[0] * sn + d[1] * c))
                        # the same move in the body axes of NOW (for commands: C, D)
                        c, sn = math.cos(yaw_now), math.sin(yaw_now)
                        floor_body = (float(d[0] * c + d[1] * sn), float(-d[0] * sn + d[1] * c))
            # floor_ok: the floor track has a value for this picture (no gap, picture good). In --pos floor mode the
            # READY banner follows it, so a covered lens shows NOT READY at once (E1).
            st.update(floor_ok=floor_pos is not None,
                      fwd_body=None if floor_body is None else floor_body[0],
                      right_body=None if floor_body is None else floor_body[1])
            if pos_src == "floor":
                st.update(ready=floor_pos is not None,
                          why=None if floor_pos is not None else (why or "floor track: no picture fit now"))
            show = floor_pos if pos_src == "floor" else hold_pos
            if show is not None:
                st.update(fwd=show[0], right=show[1], dist=float(math.hypot(*show)))
            elif pos_src == "floor":
                st.update(fwd=None, right=None, dist=None)      # a floor-track gap: unknown, not the old value
            other = hold_pos if pos_src == "floor" else floor_pos
            st["pos_info"] = (f"position from {'floor track' if pos_src == 'floor' else 'Hold'}"
                              + (f" ({lift.n} points, {lift.gaps} gaps)" if pos_src == "floor" and lift is not None else "")
                              + f" · {'Hold' if pos_src == 'floor' else 'floor track'} "
                              + ("-" if other is None else f"F {other[0]:+.2f} R {other[1]:+.2f}"))
            dh_baro = None if h is None or h_zero is None else float(h - h_zero)
            if lift is not None and lift.ratio is not None:
                st.update(dh=float(h_lift0 * (lift.ratio - 1.0)), dh_src=f"camera ({lift.n} points)", dh_baro=dh_baro)
            else:
                st.update(dh=dh_baro, dh_baro=dh_baro, dh_src="barometer" + (
                    f" (camera height off: {lift_err[-1]})" if lift_off else
                    f" (camera errors {len(lift_err)}: {lift_err[-1]})" if lift_err else
                    " (camera: no picture fit now)" if lift else ""))
            if rc is not None:                          # FC commands (C, D, E): msp_rc.py, a thread at 50 Hz
                rc.update(st["ready"], st.get("fwd_body"), st.get("right_body"))
                if zero_req:                            # "Zero here" or switch-on was applied in this frame
                    rc.mark_zero()
                if getattr(rc, "zero_next", False):     # the zero of the switch-on was set in this frame
                    rc.zero_done()
                    rc.zero_next = False
                if rc.take_engage_edge():               # engage switch on: zero here, as the page button
                    with shared.lock:
                        shared.zero_req = True
                    rc.zero_next = True                 # neutral (msp_rc.wait_zero) until that zero is set
                st["pos_info"] = st.get("pos_info", "") + " · " + rc.info()
                st.update(rc.fields())
            shared.put(**st)
            log.row(fr, e, ok, why2, proc, st["age_ms"], h, pipe.note)
    except KeyboardInterrupt:
        pass
    finally:
        if rc is not None:
            rc.close()                                  # neutral command 10 times: the FC keeps the last one
        log.close()
        if "grab" in locals():
            grab.close()
        if locals().get("rec") is not None:
            rec.close()
        cam.close()
        gyro.close()
        acc.close()
        baro.close()
        srv.shutdown()
        srv.server_close()                              # free the port at once (a restart in the same process)
    return log.path


def main():
    # nohup makes stdout a file, which Python buffers in blocks: a crash or an error message would stay unseen
    # in apb_live_now.out until exit (C test 2026-11-21). One line at a time instead.
    try:
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)
    except (AttributeError, ValueError):
        pass
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--seconds", type=float)
    ap.add_argument("--height", type=float, help="desk tests only: fixed lens height (m), no barometer")
    ap.add_argument("--no-lift", action="store_true", help="no floor track (camera height change and move); "
                    "the barometer and Hold only")
    ap.add_argument("--pos", choices=("hold", "floor"), default="hold",
                    help="source of forward/right/distance on the page: Hold, or the floor track (apb_lift, B1/B2). "
                         "The info line shows the other one.")
    ap.add_argument("--rec", metavar="LABEL", help="also record for replay (15 fps, ~7.5 MB/s; stops by itself near 300 MB free)")
    ap.add_argument("--rc", action="store_true", help="send stick commands to the Betaflight FC (msp_rc.py, "
                    "bench items C, D, E). Needs --pos floor. NOT FOR FLIGHT: hover throttle and gain are bench values")
    ap.add_argument("--rc-hover", type=int, default=1500, help="throttle (us) when not ready or ready; never minimum")
    ap.add_argument("--rc-gain", type=float, default=100.0, help="us of roll/pitch per metre of move (limit 100 us)")
    a = ap.parse_args()
    rc = None
    if a.rc:
        if a.pos != "floor":
            ap.error("--rc needs --pos floor (the commands use the floor track)")
        import msp_rc
        rc = msp_rc.RcOut(hover_us=a.rc_hover, gain_us_per_m=a.rc_gain, log_dir="/data/vio/logs").start()   # neutral at once (Stage 2 safety)
    A.stop_on_signals()
    run(S.load(), a.port, a.fps, a.seconds, fixed_height=a.height, rec_label=a.rec, use_lift=not a.no_lift,
        pos_src=a.pos, rc=rc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
