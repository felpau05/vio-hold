#!/bin/sh
# Defect: on ArduPilot, "apb_main.py check" shows "FAIL LOCAL_POSITION_NED 0 Hz ... The FC sends no height" on the
# ground with GPS off (GPS1_TYPE 0), so EVENT_DAY step 5 ("ALL PASS") can never pass. ArduPilot sends
# LOCAL_POSITION_NED only once its EKF has a position, which only our fixes give (fc_link._read says so and takes
# the height from GLOBAL_POSITION_INT). The height works: "INFO height now" is read.
# Needs: ArduCopter SITL (integ/rehearsal_2026-11-20/sitl_ap.sh), no other program on udp 14540.
# Patch: patches/apb_main_check_height_stream_ardupilot.patch (measure GLOBAL_POSITION_INT on ArduPilot).
cd "$(dirname "$0")/../.." || exit 1
PY=$HOME/.venvs/vio/bin/python
T=$(mktemp -d)
$PY -c "import settings as S; s = S.load('/nonexistent'); s['video_device'] = '/nonexistent'; S.save(s, '$T/s.json')"
sh integ/rehearsal_2026-11-20/sitl_ap.sh start wipe; sleep 8
$PY apb_main.py check --set-params --settings $T/s.json | grep -E "firmware|param GPS1_TYPE"
sh integ/rehearsal_2026-11-20/sitl_ap.sh stop; sleep 2; sh integ/rehearsal_2026-11-20/sitl_ap.sh start; sleep 8   # reboot
$PY apb_main.py check --settings $T/s.json | grep -E "POSITION|height now"
sh integ/rehearsal_2026-11-20/sitl_ap.sh stop
rm -rf "$T"
