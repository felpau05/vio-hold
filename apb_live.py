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
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

import apb_est as E
import apb_sensors as A
import capture
import settings as S
from hold import Series
from v4l2_mmap import Camera

COVER_FINE = 3.0      # fine-detail energy below this: lens covered or blurred. Desk test 2026-11-21: hand 1.3-1.7,
                      # sliding (motion blur) 4.9 at worst, normal 16-19
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

    def __init__(self, base, gyro, acc, baro):
        self.base, self.gyro, self.acc, self.baro = base, gyro, acc, baro
        self.imgs, self.ts, self.seqs, self.k, self.savers = [], [], [], 0, []

    def add(self, img, t, seq):
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
        self.latest, self.got, self.stop = None, -1, False
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def _run(self):
        tk = -1.0
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

    def get(self):
        with self.lock:
            return dict(self.state)

    def put(self, **kw):
        with self.lock:
            self.state.update(kw)


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
</style></head><body><main>
<div id="ready">NOT READY</div><div id="why"></div>
<div class="grid">
<div class="c"><div class="l">Forward (m)</div><div class="v" id="fwd">-</div></div>
<div class="c"><div class="l">Right (m)</div><div class="v" id="right">-</div></div>
<div class="c"><div class="l">Distance from zero (m)</div><div class="v" id="dist">-</div></div>
<div class="c"><div class="l">Height change (m)</div><div class="v" id="dh">-</div></div>
</div>
<button onclick="post('zero')">Zero here</button><button onclick="post('floor')">Floor zero (drone on the floor)</button>
<div id="info"></div></main><script>
const f=(x,d=2)=>x===null||x===undefined?'-':(x>=0?'+':'')+x.toFixed(d);
async function post(p){try{await fetch('/'+p,{method:'POST'})}catch(e){}}
async function tick(){try{const s=await (await fetch('/state',{cache:'no-store'})).json();
const r=document.getElementById('ready');r.textContent=s.ready?'READY':'NOT READY';r.className=s.ready?'ok':'';
document.getElementById('why').textContent=s.ready?'':(s.why||'');
document.getElementById('fwd').textContent=f(s.fwd);document.getElementById('right').textContent=f(s.right);
document.getElementById('dist').textContent=s.dist==null?'-':s.dist.toFixed(2);document.getElementById('dh').textContent=f(s.dh);
document.getElementById('info').textContent=`height ${s.h==null?'-':s.h.toFixed(2)} m · ${s.state||'-'} · ${s.n??'-'} points · ${s.fps?.toFixed(1)??'-'} fps · process ${s.proc_ms?.toFixed(0)??'-'} ms · frame age ${s.age_ms?.toFixed(0)??'-'} ms · SoC ${s.soc?.toFixed(0)??'-'} °C · up ${s.up?.toFixed(0)??'-'} s`;
}catch(e){document.getElementById('why').textContent='no connection to the APB';document.getElementById('ready').className='';}}
setInterval(tick,200);tick();
</script></body></html>"""


def serve(shared, port):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            b = body.encode()
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
        rec_label=None, rec_dir="/data/vio/rec"):
    """fixed_height (desk tests only): use this lens height (m) instead of the barometer.
    rec_label: also record every frame + IMU + barometer to rec_dir/<label>_<time>_<k>.npz."""
    from apb_main import Log, Pipe
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
        rec = None
        if rec_label:
            os.makedirs(rec_dir, exist_ok=True)
            rec = Recorder(os.path.join(rec_dir, f"{rec_label}_{time.strftime('%H%M%S')}"), gyro, acc, baro)
            say(f"recording to {rec.base}_*.npz")
        grab = Grabber(cam, fps, rec)
        t_start = time.monotonic()
        last_g = last_b = -math.inf
        last_h = None                                   # (t, h) of the newest barometer height
        floor_window = []                               # (t, Pa) for the floor zero
        floor_until = time.monotonic() + 2.0            # first 2 s: on the floor
        zero = None                                     # dict(pos, h, c, s) of the position zero
        fps_t = deque(maxlen=60)
        while seconds is None or time.monotonic() - t_start < seconds:
            r = grab.get()
            if r is None:
                shared.put(ready=False, why="no picture from the camera")
                continue
            img, t_eof, seq = r
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
                      fps=(len(fps_t) - 1) / (fps_t[-1] - fps_t[0]) if len(fps_t) > 2 else None)
            if e is not None and e.var is not None:
                yaw = float(np.interp(t0, *np.array(att.rows)[-200:, [0, 3]].T))
                if zero is None or zero_req:
                    zero = dict(pos=e.pos[:2].copy(), h=h, c=math.cos(yaw), s=math.sin(yaw))
                d = e.pos[:2] - zero["pos"]
                st.update(fwd=float(d[0] * zero["c"] + d[1] * zero["s"]),
                          right=float(-d[0] * zero["s"] + d[1] * zero["c"]), dist=float(np.hypot(*d)),
                          dh=None if h is None or zero["h"] is None else float(h - zero["h"]))
            elif zero_req and zero is not None:
                with shared.lock:
                    shared.zero_req = True              # no fix now: zero at the next one
            shared.put(**st)
            log.row(fr, e, ok, why2, proc, st["age_ms"], h, pipe.note)
    except KeyboardInterrupt:
        pass
    finally:
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
    return log.path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--seconds", type=float)
    ap.add_argument("--height", type=float, help="desk tests only: fixed lens height (m), no barometer")
    ap.add_argument("--rec", metavar="LABEL", help="also record every frame for replay (about 15 MB/s)")
    a = ap.parse_args()
    run(S.load(), a.port, a.fps, a.seconds, fixed_height=a.height, rec_label=a.rec)
    return 0


if __name__ == "__main__":
    sys.exit(main())
