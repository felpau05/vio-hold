#!/usr/bin/env python3
"""Does "apb_main.py run" keep running when the ssh session that started it ends? (audit Table 1 row 2, item A.2)

This laptop has no ssh server (no sshd binary, and no sudo to install one), so the test makes the same event
without ssh: sshd ends a session by closing the pseudo-terminal master. The kernel then sends SIGHUP to the
session leader (the shell), the shell sends SIGHUP to its jobs, and the pseudo-terminal stops taking output.
Here: bash -i in a new pseudo-terminal, the start command typed into it, then the master closed.

  hup_test.py BOARD_DIR LOG_DIR PY
For each start command: is the program still running 6 s after the hang-up, and does its run log still grow?
"""
import glob
import os
import pty
import subprocess
import select
import sys
import time

WAIT_S = 20.0


def newest_log(log_dir):
    logs = glob.glob(os.path.join(log_dir, "run_*.csv"))
    return max(logs, key=os.path.getmtime) if logs else None


def running(marker):
    r = subprocess.run(["pgrep", "-f", marker], capture_output=True, text=True)
    return [int(x) for x in r.stdout.split()]


def one(board, log_dir, label, cmd, marker):
    pid, fd = pty.fork()
    if pid == 0:
        os.chdir(board)
        os.execvp("bash", ["bash", "--norc", "--noprofile", "-i"])
    seen = []
    os.write(fd, (cmd + "\n").encode())
    end = time.monotonic() + WAIT_S
    while time.monotonic() < end:             # read the terminal in this thread: a reader thread would keep the
        if select.select([fd], [], [], 0.2)[0]:   # master open after os.close, and no hang-up would happen
            seen.append(os.read(fd, 4096))
    before = running(marker)
    log = newest_log(log_dir)
    os.close(fd)                                  # the session ends: the pseudo-terminal hangs up
    os.waitpid(pid, 0)
    time.sleep(6.0)
    after = running(marker)
    n1 = sum(1 for _ in open(log)) if log else 0
    time.sleep(3.0)
    n2 = sum(1 for _ in open(log)) if log else 0
    out = b"".join(seen).decode(errors="replace").strip().splitlines()
    print(f"[{label}] start command: {cmd}", flush=True)
    print(f"  before the hang-up: program pids {before}; run log {log}", flush=True)
    print(f"  6 s after the hang-up: program pids {after}; run log rows {n1} -> {n2} in 3 s", flush=True)
    print(f"  RESULT: {'CONTINUES' if after and n2 > n1 else 'STOPPED'}", flush=True)
    print("  terminal (last 3 lines): " + " | ".join(x.strip() for x in out[-3:]), flush=True)
    for p in after:                               # clean up for the next test (Ctrl+C, as deploy.sh stops it)
        subprocess.run(["kill", "-INT", str(p)])
    time.sleep(3.0)
    for p in running(marker):
        subprocess.run(["kill", "-TERM", str(p)])
    time.sleep(1.0)


def main():
    board, log_dir, py = sys.argv[1:4]
    run = f"{py} apb_main.py run --raw"
    marker = "apb_main.py run --raw"
    tests = [("in the foreground, as EVENT_DAY step 7 item 3 said before 2026-11-20", run),
             ("nohup, in the background", f"nohup {run} &"),
             ("setsid nohup, output to logs/console.txt", f"setsid nohup {run} > logs/console.txt 2>&1 < /dev/null &")]
    for label, cmd in tests:
        one(board, log_dir, label, cmd, marker)


if __name__ == "__main__":
    main()
