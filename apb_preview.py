"""Camera preview page for calibration rounds: the Justice D picture with the checkerboard found in it and a
coverage map of where board corners have landed so far. Optionally records the frames for tools/lens_calib.py.

  PYTHONPATH=/data/vio/pylib python3 apb_preview.py [--rec LABEL] [--board 10x7] [--port 8081] [--fps 10]

Open http://192.168.75.1:8081. Green dots: the board found in this frame. Cell tint: how many corners have
landed in that part of the picture (red none, yellow a few, green many). Recording: 960x540 grey frames at
--fps into /data/vio/rec/<label>_<time>_<k>.npz (imgs, t_frame, seq), stops by itself near 300 MB free.
The detection here is a quick one for the preview; lens_calib.py runs the accurate one on the recording.
"""
import argparse
import os
import shutil
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

import apb_sensors as A
from apb_live import Grabber
from v4l2_mmap import Camera

GRID = (8, 5)          # coverage cells across, down


class FrameRec:
    """Frames only (lens calibration needs no IMU): chunks of 100 frames, saved in a thread."""

    def __init__(self, base, min_free_mb=300, say=print):
        self.base, self.min_free, self.say = base, min_free_mb, say
        self.imgs, self.ts, self.seqs, self.k, self.th, self.full = [], [], [], 0, [], False

    def add(self, img, t, seq):
        if self.full:
            return
        self.imgs.append(img)
        self.ts.append(t)
        self.seqs.append(seq)
        if len(self.imgs) >= 100:
            self.flush()

    def flush(self):
        if not self.imgs:
            return
        data = dict(imgs=self.imgs, t_frame=np.array(self.ts), seq=np.array(self.seqs))
        path = f"{self.base}_{self.k:03d}.npz"
        self.k += 1
        self.imgs, self.ts, self.seqs = [], [], []

        def save():
            data["imgs"] = np.stack(data["imgs"])
            np.savez(path, **data)
        t = threading.Thread(target=save, daemon=False)
        t.start()
        self.th.append(t)
        if shutil.disk_usage(os.path.dirname(self.base)).free / 1e6 < self.min_free + 60:
            self.full = True
            self.say("recording stopped: disk nearly full")

    def close(self):
        self.flush()
        for t in self.th:
            t.join()


PAGE = """<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Camera preview</title><style>
:root{--bg:#f4f4f2;--fg:#1d1d1b;--mut:#6b6b66}
@media (prefers-color-scheme:dark){:root{--bg:#141413;--fg:#ececea;--mut:#9a9a95}}
body{margin:0;background:var(--bg);color:var(--fg);font:16px system-ui,sans-serif}
main{max-width:1000px;margin:0 auto;padding:16px}img{width:100%;height:auto;border-radius:8px;display:block}
#s{margin:10px 2px;color:var(--mut);font-variant-numeric:tabular-nums}
button{font-size:16px;padding:8px 14px;border-radius:8px;margin-top:8px}
</style></head><body><main><img id="f" alt="camera"><div id="s"></div>
<button onclick="fetch('/reset',{method:'POST'})">Clear coverage</button></main><script>
const f=document.getElementById('f'),s=document.getElementById('s');let busy=false;
function next(){if(busy)return;busy=true;const i=new Image();i.onload=()=>{f.src=i.src;busy=false};i.onerror=()=>{busy=false};
i.src='/frame.jpg?'+Date.now();}
setInterval(next,250);
setInterval(async()=>{try{const j=await (await fetch('/state')).json();
s.textContent=`board views ${j.views} · this frame: ${j.found?'BOARD FOUND':'no board'} · ${j.fps.toFixed(1)} preview fps`+(j.rec?` · recording ${j.rec}`:'');}catch(e){s.textContent='no connection';}},500);
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rec", metavar="LABEL")
    ap.add_argument("--board", default="10x7", help="inner corners across x down")
    ap.add_argument("--port", type=int, default=8081)
    ap.add_argument("--fps", type=float, default=10.0, help="camera frames kept (and recorded) per second")
    ap.add_argument("--seconds", type=float)
    a = ap.parse_args()
    A.stop_on_signals()
    board = tuple(int(x) for x in a.board.lower().split("x"))
    cv2.ocl.setUseOpenCL(False)      # the APB's GPU OpenCL path logs cl_mem errors and falls back anyway
    cam = Camera()
    rec = None
    if a.rec:
        os.makedirs("/data/vio/rec", exist_ok=True)
        rec = FrameRec(f"/data/vio/rec/{a.rec}_{time.strftime('%H%M%S')}")
        print(f"recording to {rec.base}_*.npz", flush=True)
    grab = Grabber(cam, a.fps, rec)
    lock = threading.Lock()
    st = {"jpg": b"", "views": 0, "found": False, "fps": 0.0, "rec": a.rec or ""}
    cover = np.zeros(GRID[::-1], int)
    reset = [False]

    class H(BaseHTTPRequestHandler):
        def log_message(self, *x):
            pass

        def _send(self, body, ctype):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            with lock:
                if self.path.startswith("/frame.jpg"):
                    return self._send(st["jpg"], "image/jpeg")
                if self.path.startswith("/state"):
                    import json
                    return self._send(json.dumps({k: v for k, v in st.items() if k != "jpg"}).encode(), "application/json")
            self._send(PAGE.encode(), "text/html; charset=utf-8")

        def do_POST(self):
            reset[0] = True
            self._send(b"ok", "text/plain")

    srv = ThreadingHTTPServer(("0.0.0.0", a.port), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"preview: http://192.168.75.1:{a.port}", flush=True)
    t_start, times = time.monotonic(), []
    try:
        while a.seconds is None or time.monotonic() - t_start < a.seconds:
            r = grab.get()
            if r is None:
                continue
            img = r[0]
            h, w = img.shape
            if reset[0]:
                cover[:] = 0
                st["views"] = 0
                reset[0] = False
            small = cv2.resize(img, (w // 2, h // 2), interpolation=cv2.INTER_AREA)
            ok, c = cv2.findChessboardCornersSB(small, board)
            v = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            if ok:
                c = c.reshape(-1, 2) * 2
                for x, y in c:
                    cover[min(GRID[1] - 1, int(GRID[1] * y / h)), min(GRID[0] - 1, int(GRID[0] * x / w))] += 1
                st["views"] += 1
            tint = np.zeros_like(v)
            for j in range(GRID[1]):
                for i in range(GRID[0]):
                    n = cover[j, i]
                    col = (0, 0, 200) if n == 0 else (0, 200, 220) if n < 60 else (0, 180, 0)
                    y0, y1, x0, x1 = j * h // GRID[1], (j + 1) * h // GRID[1], i * w // GRID[0], (i + 1) * w // GRID[0]
                    tint[y0:y1, x0:x1] = col
                    cv2.rectangle(v, (x0, y0), (x1 - 1, y1 - 1), (90, 90, 90), 1)
            v = cv2.addWeighted(v, 0.78, tint, 0.22, 0)
            if ok:
                for x, y in c:
                    cv2.circle(v, (int(x), int(y)), 3, (0, 255, 0), -1)
            cv2.putText(v, "BOARD FOUND" if ok else "no board", (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 255, 0) if ok else (0, 0, 255), 2)
            jpg = cv2.imencode(".jpg", v, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
            times.append(time.monotonic())
            times = times[-20:]
            with lock:
                st.update(jpg=jpg, found=bool(ok),
                          fps=(len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0)
    except KeyboardInterrupt:
        pass
    finally:
        grab.close()
        if rec:
            rec.close()
            print(f"saved {rec.k} chunks: {rec.base}_*.npz", flush=True)
        cam.close()
        srv.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
