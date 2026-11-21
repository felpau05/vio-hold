#!/bin/sh
# tools/fc_params.py against a live ArduCopter SITL flight controller (integ/rehearsal_2026-11-20/sitl_ap.sh):
# save the factory values, let "check --set-params" write ours, restart, restore, restart, read again.
# The values after the restore must equal the saved values.
cd "$(dirname "$0")/../.." || exit 1
PY=$HOME/.venvs/vio/bin/python
R=integ/rehearsal_2026-11-20
T=$(mktemp -d)
cp apsim/runs/rehearsal_2026-11-20_sitl.log "$T/old_sitl.log" 2>/dev/null
$PY -c "import settings as S; s = S.load('/nonexistent'); s['video_device'] = '/nonexistent'; s['log_dir'] = '$T/a'; S.save(s, '$T/a.json'); s['log_dir'] = '$T/b'; S.save(s, '$T/b.json')"
sh $R/sitl_ap.sh start wipe; sleep 8
echo "--- save (factory values)";             $PY tools/fc_params.py save --settings $T/a.json
echo "--- save again (must refuse)";          $PY tools/fc_params.py save --settings $T/a.json; echo "exit $?"
echo "--- check --set-params (our values)";   $PY apb_main.py check --set-params --settings $T/a.json | grep -E "param "
sh $R/sitl_ap.sh stop; sleep 2; sh $R/sitl_ap.sh start; sleep 8          # reboot, parameters kept
echo "--- restore";                           $PY tools/fc_params.py restore --settings $T/a.json; echo "exit $?"
sh $R/sitl_ap.sh stop; sleep 2; sh $R/sitl_ap.sh start; sleep 8          # reboot
echo "--- read again after the restore";      $PY tools/fc_params.py save --settings $T/b.json | tail -1
$PY -c "
import json
a = json.load(open('$T/a/fc_params_before.json'))['values']; b = json.load(open('$T/b/fc_params_before.json'))['values']
bad = {k: (a[k], b.get(k)) for k in a if a[k] != b.get(k)}
print('PASS the values after the restore equal the saved values:', a) if not bad else print('FAIL differ:', bad)"
sh $R/sitl_ap.sh stop
[ -f "$T/old_sitl.log" ] && cp "$T/old_sitl.log" apsim/runs/rehearsal_2026-11-20_sitl.log
rm -rf "$T"
