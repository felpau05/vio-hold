#!/usr/bin/env python3
"""A/B driver: runs low_ap_matrix.py labels in ab_before and ab_after, one SITL at a time, interleaved.
Gate before each run: 1-min load < 2.5, no arducopter process, no px4_sim container. Retry once on failure.
Log: ab_driver.log (one JSON line per event)."""
import json
import os
import signal
import subprocess
import sys
import time

SP = os.path.dirname(os.path.abspath(__file__))
PY = os.path.expanduser("~/.venvs/vio/bin/python")
COPIES = ["ab_before", "ab_after"]
LOG = os.path.join(SP, "ab_driver.log")
LOAD_MAX = 2.5
LAST_START = "09:08"     # no new run starts after this local time (each run takes about 90 s; report after)
TIMEOUT_S = 420          # a run that takes longer is called hung


def log(**kw):
    kw["t"] = time.strftime("%H:%M:%S")
    line = json.dumps(kw, default=str)
    print(line, flush=True)
    open(LOG, "a").write(line + "\n")


def arducopters():
    out = []
    for p in os.listdir("/proc"):
        if p.isdigit():
            try:
                a = open(f"/proc/{p}/cmdline", "rb").read().split(b"\0")
            except OSError:
                continue
            if a and a[0].endswith(b"arducopter"):
                out.append((p, b" ".join(a).decode(errors="replace")))
    return out


def sitl_py():
    """Other simulator scripts (any copy) still alive."""
    out = []
    me = os.getpid()
    for p in os.listdir("/proc"):
        if p.isdigit() and int(p) != me:
            try:
                a = open(f"/proc/{p}/cmdline", "rb").read()
            except OSError:
                continue
            if any(k in a for k in (b"sitl_ap", b"low_ap_matrix", b"engage_matrix", b"px4sim/sitl.py",
                                    b"drag_matrix", b"gapjump_matrix", b"run_all.sh")) and b"ab_driver" not in a:
                out.append((p, a.replace(b"\0", b" ").decode(errors="replace")[:160]))
    return out


def px4_busy():
    r = subprocess.run(["docker", "ps", "-q", "-f", "name=^px4_sim$"], capture_output=True, text=True)
    return bool(r.stdout.strip())


def gate():
    t0 = time.time()
    while True:
        if time.strftime("%H:%M") >= LAST_START:
            return None
        ac, py, px, l1 = arducopters(), sitl_py(), px4_busy(), os.getloadavg()[0]
        if not ac and not py and not px and l1 < LOAD_MAX:
            return dict(load=os.getloadavg(), waited_s=round(time.time() - t0))
        if int(time.time() - t0) % 60 < 6:
            log(ev="wait", load=round(l1, 2), arducopter=ac, sim_py=py, px4_sim=px)
        time.sleep(5)


def result(copy, label):
    try:
        r = json.load(open(os.path.join(SP, copy, "apsim", "low_ap_results.json")))[label]
    except (OSError, ValueError, KeyError):
        return None, "no entry in low_ap_results.json"
    err = [e for e in (r.get("events") or []) if "ERROR" in str(e[1])]
    if err:
        return r, f"error event: {err}"
    if not r.get("hold_s_flown") or r["hold_s_flown"] < 59.5:
        return r, f"hold_s_flown {r.get('hold_s_flown')}; events {r.get('events')}"
    return r, None


def run_one(copy, label, attempt):
    g = gate()
    if g is None:
        log(ev="skip", copy=copy, label=label, why=f"after {LAST_START}")
        return "skipped"
    d = os.path.join(SP, copy)
    if attempt == 1:
        cmd = [PY, os.path.join(d, "apsim", "low_ap_matrix.py"), "run", label]
        out = open(os.path.join(SP, f"drv_{copy}_{label}.txt"), "w")
    else:   # run() skips a label already in the results: call one() directly, own log file
        cmd = [PY, os.path.join(d, "apsim", "low_ap_matrix.py"), "one", label]
        out = open(os.path.join(d, "apsim", "logs", f"lowap_{label}_retry.txt"), "w")
    log(ev="start", copy=copy, label=label, attempt=attempt, load=g["load"], waited_s=g["waited_s"])
    t0 = time.time()
    p = subprocess.Popen(cmd, cwd=d, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        rc = p.wait(timeout=TIMEOUT_S)
    except subprocess.TimeoutExpired:
        rc = "timeout"
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        p.wait()
    subprocess.run(["sh", os.path.join(d, "apsim", "run_sitl.sh"), "stop", "1"], capture_output=True)
    r, bad = result(copy, label)
    log(ev="end", copy=copy, label=label, attempt=attempt, rc=rc, wall_s=round(time.time() - t0), bad=bad,
        rms=r and r.get("rms"), load_start_runner=r and r.get("load_start"),
        hold_md5=None)
    if bad:
        # keep the failed record before a retry overwrites it
        fp = os.path.join(d, "apsim", f"failed_{label}_a{attempt}.json")
        json.dump(dict(rc=rc, bad=bad, record=r), open(fp, "w"), indent=1, default=str)
        return "failed"
    return "ok"


def main(labels):
    for label in labels:
        for copy in COPIES:
            st = run_one(copy, label, 1)
            if st == "failed":
                st = run_one(copy, label, 2)
            log(ev="final", copy=copy, label=label, status=st)
            if st == "skipped":
                return


if __name__ == "__main__":
    main(sys.argv[1:])
