#!/bin/sh
# ArduCopter SITL as the "aircraft" for the event-day rehearsal (apsim/ardupilot build, container ap_sim, as
# apsim/run_sitl.sh). SERIAL1 sends to udp 127.0.0.1:14540, where settings fc_device udpin:127.0.0.1:14540 listens
# (the board's mavlink-router port). SERIAL0 goes to 14560 (gcs.py: mode changes and checks).
#   sitl_ap.sh start [wipe]   start; "wipe" = factory parameters (first power-on). Without it the parameters stay
#                             (a reboot of the flight controller keeps what check --set-params wrote).
#   sitl_ap.sh stop
set -e
APSIM=$(cd "$(dirname "$0")/../../apsim" && pwd)
I=7
if [ "$1" = stop ]; then
  docker exec ap_sim pkill -f "arducopter .* -I $I " >/dev/null 2>&1 || true
  exit 0
fi
W=""; [ "${2:-}" = wipe ] && W="-w"
mkdir -p "$APSIM/runs"
printf 'ARMING_CHECK 0\nSIM_WIND_SPD 0\n' > "$APSIM/runs/rehearsal_2026-11-20.parm"
if [ "$(docker inspect -f '{{.State.Running}}' ap_sim 2>/dev/null)" != "true" ]; then
  docker rm -f ap_sim >/dev/null 2>&1 || true
  docker run -d --rm --name ap_sim --network host -u "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$APSIM":/apsim ardupilot/ardupilot-dev-base:latest sleep infinity >/dev/null
fi
docker exec -d ap_sim sh -c "mkdir -p /tmp/ap$I && cd /tmp/ap$I && exec /apsim/ardupilot/build/sitl/bin/arducopter \
  -S $W --model quad --speedup 1 -I $I --home 45.417,-75.715,70,0 \
  --defaults /apsim/ardupilot/Tools/autotest/default_params/copter.parm,/apsim/runs/rehearsal_2026-11-20.parm \
  --serial0 udpclient:127.0.0.1:14560 --serial1 udpclient:127.0.0.1:14540 > /apsim/runs/rehearsal_2026-11-20_sitl.log 2>&1"
echo "ArduCopter SITL instance $I started ${W:+(parameters wiped)}"
