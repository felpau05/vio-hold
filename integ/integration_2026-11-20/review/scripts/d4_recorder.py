"""D4: with the no_sd values on the Justice D picture (640x360 for Hold, record_hz 10, min_free_gb 0.5), check
needs 0.50 GB. Start the real Raw recorder at exactly that free space; the disk shrinks by what Raw writes.
How long does it record? (EVENT_DAY step 7: the program starts at item 1, the wiggle comes minutes later.)"""
import os, shutil, sys
from types import SimpleNamespace
import numpy as np
sys.path.insert(0, os.environ["PROJECT"])
import apb_main
from hold import Series
from collections import namedtuple

out = sys.argv[1]
shape, hz, min_gb = (360, 640), 10, 0.5
mbs = shape[0] * shape[1] * hz / 1e6
need = max(min_gb * mbs / (360 * 288 * 25 / 1e6), min_gb)          # apb_main.py line 915
print(f"check needs {need:.2f} GB (scaled value {min_gb * mbs / (360 * 288 * 25 / 1e6):.2f} GB); raw {mbs:.2f} MB/s")
start_free = need * 1e9
real = shutil.disk_usage
def used():
    return sum(os.path.getsize(os.path.join(out, f)) for f in os.listdir(out))
shutil.disk_usage = lambda p: SimpleNamespace(free=start_free - used())
s = dict(apb_main.S.load("/nonexistent"), log_dir=out, record_hz=hz, min_free_gb=min_gb)
raw = apb_main.Raw(s, SimpleNamespace(att=Series(), hgt=Series(), lpos=Series()), shape)
Fr = namedtuple("Fr", "img t_grab")
img = np.random.default_rng(0).integers(0, 255, shape, np.uint8)
stopped = None
for k in range(int(300 * 30)):                 # 300 s of a 30 fps stream
    t = k / 30.0
    raw.add(Fr(img, t))
    if raw.saver is not None:
        raw.saver.join()
    if raw.off and stopped is None:
        stopped = t
print(f"chunks saved: {raw.k}; recorder stopped at t {stopped} s of 300 s; free at the end "
      f"{shutil.disk_usage(out).free / 1e9:.3f} GB")
shutil.disk_usage = real
