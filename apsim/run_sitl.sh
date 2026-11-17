#!/bin/sh
# ArduCopter SITL (built from apsim/ardupilot, Orqa's mil-mavl = Copter-4.6.0 + board commit), headless,
# in the container ap_sim (--network host). Several instances can run side by side (-I k).
#   apsim/run_sitl.sh start <instance> <gcs_port> <onboard_port> <speedup> <parm file under apsim/> <log under apsim/>
#   apsim/run_sitl.sh stop <instance>
# SERIAL0 = UDP to <gcs_port> (the supervisor), SERIAL1 = UDP to <onboard_port> (the onboard program,
# through apsim's 230400-baud relay). Build once:
#   docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v $PWD/apsim/ardupilot:/ardupilot -w /ardupilot \
#     ardupilot/ardupilot-dev-base:latest sh -c './waf configure --board sitl && ./waf copter -j16'
set -e
APSIM=$(cd "$(dirname "$0")" && pwd)
IMAGE=ardupilot/ardupilot-dev-base:latest
if [ "$1" = "stop" ]; then
  docker exec ap_sim pkill -f "arducopter .* -I $2 " >/dev/null 2>&1 || true
  exit 0
fi
I=$2; G=$3; O=$4; SPD=$5; PARM=$6; LOG=$7
if [ "$(docker inspect -f '{{.State.Running}}' ap_sim 2>/dev/null)" != "true" ]; then
  docker rm -f ap_sim >/dev/null 2>&1 || true
  docker run -d --rm --name ap_sim --network host -u "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$APSIM":/apsim "$IMAGE" sleep infinity >/dev/null
fi
docker exec -d ap_sim sh -c "mkdir -p /tmp/ap$I && cd /tmp/ap$I && exec /apsim/ardupilot/build/sitl/bin/arducopter \
  -S -w --model quad --speedup $SPD -I $I --home 45.417,-75.715,70,0 \
  --defaults /apsim/ardupilot/Tools/autotest/default_params/copter.parm,/apsim/$PARM \
  --serial0 udpclient:127.0.0.1:$G --serial1 udpclient:127.0.0.1:$O > /apsim/$LOG 2>&1"
