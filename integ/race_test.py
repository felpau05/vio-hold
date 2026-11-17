"""hold.py attitude race (SIM and APB reports): a reader thread appends ATTITUDE rows while Hold-side
code calls attitude_R and unwrapped_yaw. Before the fix this raised 'fp and xp are not of the same
length'. Run: ~/.venvs/vio/bin/python integ/race_test.py [hold module path]"""
import importlib.util
import math
import os
import sys
import threading
import time

import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "hold.py")
spec = importlib.util.spec_from_file_location("hold_under_test", path)
hold = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hold)

att = hold.Series(30.0)
stop = False


def writer():
    t = 0.0
    while not stop:
        t += 0.001
        att.add(t, 0.01, 0.02, math.sin(t), 0.0, 0.0, 0.1)
        time.sleep(0.0002)        # about 2-4 kHz: far faster than the 100 Hz ATTITUDE stream


th = threading.Thread(target=writer, daemon=True)
th.start()
time.sleep(0.05)
errors, n, end = 0, 0, time.time() + 5
while time.time() < end:
    try:
        tt = att.last_t() - 0.01
        hold.attitude_R(att, np.array([tt, tt - 0.005]))
        n += 1
    except ValueError:
        errors += 1
stop = True
print(f"{os.path.basename(path)}: attitude_R calls {n}, ValueError {errors}: {'PASS' if errors == 0 and n > 200 else 'FAIL'}")
