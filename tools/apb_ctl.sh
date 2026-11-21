#!/bin/sh
# Start, stop and check the live program on the APB. Copy to /data/vio and run there, or from the laptop:
#   ssh root@192.168.75.1 sh /data/vio/apb_ctl.sh start [extra apb_live.py arguments, e.g. --rec walk1]
#   ssh root@192.168.75.1 sh /data/vio/apb_ctl.sh stop
#   ssh root@192.168.75.1 sh /data/vio/apb_ctl.sh status
# Stops by process ID (a name pattern can match the shell that runs pkill itself) with SIGTERM, which
# apb_live.py turns into a clean stop that puts the IMU and barometer settings back.
D=/data/vio
PID=$D/live.pid
cd "$D" || exit 1
running() { [ -f "$PID" ] && kill -0 "$(cat "$PID")" 2>/dev/null; }

case "$1" in
start)
    shift
    if running; then echo "already running (pid $(cat "$PID")). Stop it first."; exit 1; fi
    mkdir -p logs
    PYTHONPATH=$D/pylib nohup python3 apb_live.py "$@" > logs/live_now.out 2>&1 &
    echo $! > "$PID"
    sleep 3
    if running; then echo "started (pid $(cat "$PID")): http://192.168.75.1:8080"; else echo "FAILED to start:"; tail -5 logs/live_now.out; exit 1; fi
    ;;
stop)
    if ! running; then echo "not running"; rm -f "$PID"; exit 0; fi
    kill -TERM "$(cat "$PID")"
    i=0
    while running && [ $i -lt 20 ]; do sleep 0.5; i=$((i + 1)); done
    if running; then echo "still running after 10 s: kill -9 $(cat "$PID"), then restore the sensors (see apb_sensors.py)"; exit 1; fi
    rm -f "$PID"
    echo "stopped"
    ;;
status)
    if running; then echo "running (pid $(cat "$PID"))"; tail -2 logs/live_now.out; else echo "not running"; fi
    ;;
*)
    echo "usage: sh apb_ctl.sh start [apb_live.py args] | stop | status"; exit 2
    ;;
esac
