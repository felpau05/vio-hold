"""Start and stop the Betaflight SITL (software in the loop) build for the betasim tests.

BF_SITL: the SITL binary (betasim/build.sh prints it). Each run gets a fresh folder with its own eeprom.bin,
written by the binary's own --config option from the CLI lines given. The process runs at nice 19.
"""
import os
import socket
import subprocess
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.environ.get("BF_SITL", "")
MSP_PORT = 5761                 # UART1: TCP 5760 + 1 (drivers/serial_tcp.c)


def provision(workdir, lines):
    os.makedirs(workdir, exist_ok=True)
    cfg = os.path.join(workdir, "config.txt")
    with open(cfg, "w") as f:
        f.write("\n".join(lines) + "\n")
    if os.path.exists(os.path.join(workdir, "eeprom.bin")):
        os.remove(os.path.join(workdir, "eeprom.bin"))
    r = subprocess.run(["nice", "-n", "19", BIN, "--config", cfg], cwd=workdir, capture_output=True, text=True,
                       timeout=60)
    with open(os.path.join(workdir, "provision.log"), "w") as f:
        f.write(r.stdout + r.stderr)
    if r.returncode != 0 or not os.path.exists(os.path.join(workdir, "eeprom.bin")):
        raise RuntimeError(f"provision failed rc={r.returncode}: {r.stdout[-500:]} {r.stderr[-500:]}")
    return r.stdout


def port_open():
    try:
        socket.create_connection(("127.0.0.1", MSP_PORT), timeout=0.3).close()
        return True
    except OSError:
        return False


def start(workdir):
    """Popen of the SITL; returns once its MSP port accepts a connection."""
    end = time.monotonic() + 15
    while port_open() and time.monotonic() < end:      # a previous SITL still holds the port
        time.sleep(0.3)
    log = open(os.path.join(workdir, "sitl.log"), "a")
    p = subprocess.Popen(["nice", "-n", "19", BIN], cwd=workdir, stdout=log, stderr=log)
    end = time.monotonic() + 20
    while time.monotonic() < end:
        if p.poll() is not None:
            raise RuntimeError(f"SITL exited rc={p.returncode}; see {workdir}/sitl.log")
        if port_open():
            time.sleep(0.5)
            return p
        time.sleep(0.2)
    stop(p)
    raise RuntimeError("SITL MSP port did not open in 20 s")


def stop(p):
    if p is None or p.poll() is not None:
        return
    p.terminate()
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        p.kill()
        p.wait()
