#!/bin/sh
# Copy the onboard program and settings.json to the board, restart it there, show the last log lines.
#
#   BOARD=192.168.1.20 tools/deploy.sh          push: copy, restart "apb_main.py run --raw", show the log
#   BOARD=192.168.1.20 tools/deploy.sh pull     copy the board's settings.json, its mask and new logs here
#   BOARD=... tools/deploy.sh -n [pull]         dry run: print every step, change nothing
#
# Environment, or KEY=value lines in tools/deploy.env (the environment wins):
#   BOARD       board address (required). BOARD=local: BOARD_DIR on this machine (for tests).
#   BOARD_USER  default root              BOARD_DIR  default /home/root/vio
#   PY          Python on the board, default python3
#   RUN_ARGS    default "run --raw"       WAIT       seconds before the log is shown, default 10
#   NO_RESTART=1  copy only               FORCE=1    skip the "flying" and "calibration differs" stops
#   NO_RSYNC=1    use tar over ssh even when rsync is there        SSH_OPTS  extra ssh options
#
# Push never copies the mask (the board makes it) and never copies anything outside FILES.
# It stops when the board's calibration differs from this laptop's: run "pull" first.
set -eu
cd "$(dirname "$0")/.."
if [ -f tools/deploy.env ]; then       # plain KEY=value lines, no quotes; a value set in the environment wins
  while IFS='=' read -r k v; do
    case $k in ''|\#*|*[!A-Z_]*) continue ;; esac
    eval "[ -n \"\${$k:-}\" ] || $k=\$v"
  done < tools/deploy.env
fi
DRY=""; CMD=push
for a in "$@"; do case $a in -n|--dry-run) DRY=1 ;; push|pull) CMD=$a ;; *) echo "FAIL unknown argument $a"; exit 2 ;; esac; done
BOARD=${BOARD:-}; BOARD_USER=${BOARD_USER:-root}; BOARD_DIR=${BOARD_DIR:-/home/root/vio}; PY=${PY:-python3}
RUN_ARGS=${RUN_ARGS:-run --raw}; WAIT=${WAIT:-10}; SSH_OPTS=${SSH_OPTS:-"-o ConnectTimeout=5"}
[ -n "$BOARD" ] || { echo "FAIL set BOARD to the board's address, e.g. BOARD=192.168.1.20 tools/deploy.sh"; exit 2; }
D=$BOARD_DIR

remote() {      # one shell command on the board
  if [ "$BOARD" = local ]; then sh -c "$1"; else ssh $SSH_OPTS "$BOARD_USER@$BOARD" "$1"; fi
}
step() {        # print the step; run it unless this is a dry run
  echo "+ $1"; shift
  [ -n "$DRY" ] || "$@"
}
die() { echo "FAIL $*"; exit 1; }
json_get() {    # json_get FILE KEY
  python3 -c 'import json,sys; v=json.load(open(sys.argv[1])).get(sys.argv[2]); print("" if v is None else v)' "$1" "$2"
}

if [ "$BOARD" != local ] && [ -z "$DRY" ]; then
  remote true || die "no ssh to $BOARD_USER@$BOARD. Check the cable or network, the address and the user."
fi
if [ -z "${NO_RSYNC:-}" ] && command -v rsync >/dev/null && remote "command -v rsync" >/dev/null 2>&1; then
  COPY=rsync; else COPY=tar; fi
echo "board $BOARD_USER@$BOARD:$D, copy with $COPY"
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT

if [ "$CMD" = pull ]; then
  pull_settings() {
    remote "cat '$D/settings.json'" > "$TMP/settings.json" || die "the board has no $D/settings.json"
    python3 -c 'import sys; sys.path.insert(0, "."); import settings as S; S.load(sys.argv[1])' "$TMP/settings.json" \
      || die "the board's settings.json does not load here (another program version?). Nothing changed."
    cp settings.json settings.json.bak
    cp "$TMP/settings.json" settings.json
    M=$(json_get settings.json mask_file)
    case $M in
      "") ;;
      /*|*..*) echo "mask $M is outside the program folder; not copied" ;;
      *) remote "cat '$D/$M'" > "$TMP/mask" && mv "$TMP/mask" "$M" && echo "copied mask $M" ;;
    esac
  }
  step "copy $D/settings.json and its mask here (old settings.json kept as settings.json.bak)" pull_settings
  [ -n "$DRY" ] || mkdir -p logs
  if [ "$COPY" = rsync ]; then
    R="$D/logs/"; [ "$BOARD" = local ] || R="$BOARD_USER@$BOARD:$D/logs/"
    step "rsync new logs: $R -> logs/" rsync -a --ignore-existing -e "ssh $SSH_OPTS" "$R" logs/
  else
    NEW=""
    for f in $(remote "ls '$D/logs' 2>/dev/null" || true); do [ -e "logs/$f" ] || NEW="$NEW $f"; done
    pull_tar() { remote "cd '$D/logs' && tar cf - $NEW" | tar xf - -C logs; }
    if [ -n "$NEW" ]; then step "tar new logs over ssh:$NEW" pull_tar; else echo "no new logs"; fi
  fi
  [ -n "$DRY" ] || { echo "newest logs here:"; ls -t logs | head -5; }
  exit 0
fi

# ---------------------------------------------------------------- push
LENS=$(json_get settings.json lens_file)
FILES="apb_main.py hold.py fc_link.py capture.py calib.py settings.py settings.json $LENS
EVENT_DAY.txt DECISION_CARD.txt tools/board_check.sh"
for f in $FILES; do
  case $f in
    .clip-1|*/.clip-1|ground/*|videos/*|real/*|px4sim/PX4-Autopilot/*|apsim/ardupilot/*|/*|*..*)
      die "$f is on the never-copy list. Nothing copied." ;;
  esac
  [ -f "$f" ] || die "$f is missing here. Nothing copied."
done
python3 -c 'import sys; sys.path.insert(0, "."); import settings as S; S.load("settings.json")' \
  || die "settings.json does not load. Fix it first: python3 tools/use_profile.py show"

if [ -z "$DRY" ] && [ -z "${FORCE:-}" ]; then
  if remote "cat '$D/settings.json'" > "$TMP/board.json" 2>/dev/null && [ -s "$TMP/board.json" ]; then
    DIFF=$(python3 - "$TMP/board.json" settings.json <<'EOF'
import json, sys
a, b = (json.load(open(p)) for p in sys.argv[1:3])
keys = ("top", "mirror", "tilt_deg", "twist_deg", "focal_scale", "readout_s", "video_delay_s", "calibrated",
        "calib_capture", "calib_info", "mask_file", "origin_lat_lon_alt")
print(" ".join(k for k in keys if a.get(k) != b.get(k)))
EOF
)
    [ -z "$DIFF" ] || die "the board's calibration differs from this laptop's ($DIFF). Run 'tools/deploy.sh pull', apply your changes again, then push. FORCE=1 overwrites the board's values."
  fi
  FLY=$(remote "f=\$(ls -t '$D'/logs/run_*.csv 2>/dev/null | head -1); [ -n \"\$f\" ] || exit 0
    age=\$(( \$(date +%s) - \$(stat -c %Y \"\$f\") ))
    [ \$age -lt 5 ] && tail -n 25 \"\$f\" | awk -F, '\$7==1{n++} END{if(n) print n}'" || true)
  [ -z "$FLY" ] || die "the program on the board is sending positions now (the drone may be flying). Land first. FORCE=1 skips this stop."
fi

step "mkdir -p $D/logs" remote "mkdir -p '$D/logs'"
if [ "$COPY" = rsync ]; then
  R="$D/"; [ "$BOARD" = local ] || R="$BOARD_USER@$BOARD:$D/"
  step "rsync -a --relative $(echo $FILES) $R" rsync -a --relative -e "ssh $SSH_OPTS" $FILES "$R"
else
  copy_tar() { tar cf - $FILES | remote "tar xf - -C '$D'"; }
  step "tar cf - $(echo $FILES) | ssh ... tar xf - -C $D" copy_tar
fi
[ -z "${NO_RESTART:-}" ] || { echo "copied; not restarted (NO_RESTART)"; exit 0; }

# "[a]pb" matches apb_main.py but not this pattern's own text, so pkill spares the shell that runs it.
# Local tests add the folder, so other programs on this laptop are safe.
PAT="[a]pb_main.py"; [ "$BOARD" = local ] && PAT="$D/[a]pb_main.py"
# Launcher: SIGINT stops it like Ctrl+C (Raw recording flushed, log closed); a background job would ignore it.
LAUNCH="import runpy, signal, sys; signal.signal(signal.SIGINT, signal.default_int_handler); sys.argv = ['apb_main.py'] + sys.argv[1:]; runpy.run_path('$D/apb_main.py', run_name='__main__')"
step "stop the running program (SIGINT, then SIGTERM after 5 s)" remote "
  pkill -INT -f '$PAT' || true
  for i in 1 2 3 4 5 6 7 8 9 10; do pgrep -f '$PAT' >/dev/null || break; sleep 0.5; done
  pkill -TERM -f '$PAT' || true"
step "start: cd $D && $PY -u apb_main.py $RUN_ARGS > logs/console.txt" remote "
  cd '$D' && nohup $PY -u -c \"$LAUNCH\" $RUN_ARGS > logs/console.txt 2>&1 < /dev/null &"
[ -n "$DRY" ] && exit 0
echo "waiting $WAIT s for the program to start ..."
sleep "$WAIT"
remote "cd '$D'; echo '--- logs/console.txt (last 15 lines)'; tail -n 15 logs/console.txt
  f=\$(ls -t logs/run_*.csv 2>/dev/null | head -1); [ -z \"\$f\" ] || { echo \"--- \$f (last 5 rows)\"; tail -n 5 \"\$f\"; }
  echo '--- running:'; pgrep -f '$PAT' >/dev/null && echo yes || echo 'NO: the program stopped. Read the console lines above.'"
