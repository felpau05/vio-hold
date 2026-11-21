#!/bin/sh
# Item B: four hand-turn calibrations (true lens x start lens), then two at record_hz 10 (no_sd). One run each, seed 5.
cd "$(dirname "$0")/../.." || exit 1
PY=$HOME/.venvs/vio/bin/python
FULL="Justice D full sensor (assumed)"; ANA="Justice D anamorphic (assumed)"
run() {
  while [ "$(cut -d' ' -f1 /proc/loadavg | cut -d. -f1)" -ge 4 ]; do echo "load $(cut -d' ' -f1 /proc/loadavg): wait 20 s"; sleep 20; done
  $PY integ/rehearsal_2026-11-20/calib4.py "$1" "$2" "$3" 5
}
run "$FULL" "$FULL" 30; run "$FULL" "$ANA" 30; run "$ANA" "$FULL" 30; run "$ANA" "$ANA" 30
run "$FULL" "$FULL" 10; run "$ANA" "$ANA" 10
