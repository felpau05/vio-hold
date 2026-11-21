#!/bin/sh
# Copy the onboard program and settings.json to the board, restart it there, show the last log lines.
#
#   BOARD=192.168.75.1 tools/deploy.sh          push: set the board clock, copy, restart "apb_main.py run --raw"
#                                               (detached, as start), show the log
#   BOARD=192.168.75.1 tools/deploy.sh pull     copy the board's settings.json, its mask and new logs here
#                                               (logs from its log_dir, e.g. /sdcard/logs, and from logs/)
#   BOARD=... tools/deploy.sh start             set the board clock, restart the run detached from ssh: it runs on
#                                               when the cable is out. Its output: logs/console.txt on the board
#   BOARD=... tools/deploy.sh stop | status     stop the run (as Ctrl+C) | show if it runs and its last log lines
#   BOARD=... tools/deploy.sh boot-on | boot-off  start the run at each power-on (systemd unit vio-run), or not.
#                                               Off by default.
#   BOARD=... tools/deploy.sh bundle            case B (no usable Python on the board): copy tools/bundle's Python
#                                               and program to the first of /data /sdcard /usb /home/root with
#                                               400 MB free (800 MB on a FAT or exFAT card), not noexec,
#                                               then run "apb_main.py check" with the bundle's Python
#   BOARD=... tools/deploy.sh -n [command]      dry run: print every step, change nothing
#
# Environment, or KEY=value lines in tools/deploy.env (the environment wins):
#   BOARD       board address (required): 192.168.75.1 over the USB-C cable, 192.168.1.75 with a USB-Ethernet
#               adapter (APB manual). BOARD=local: BOARD_DIR on this machine (for tests).
#   BOARD_USER  default root              BOARD_DIR  default /home/root/vio
#   PY          Python on the board, default python3 (bundle: the bundle's python3.12)
#   RUN_ARGS    default "run --raw"       WAIT       seconds before the log is shown, default 10
#   NO_RESTART=1  copy only               FORCE=1    skip the "flying" and "calibration differs" stops
#   NO_RSYNC=1    use tar over ssh even when rsync is there        SSH_OPTS  extra ssh options
#   BUNDLE_DIR  bundle: the folder on the board (default: the first with enough space, see above)
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
for a in "$@"; do case $a in -n|--dry-run) DRY=1 ;; push|pull|start|stop|status|boot-on|boot-off|bundle) CMD=$a ;;
  *) echo "FAIL unknown argument $a"; exit 2 ;; esac; done
BOARD=${BOARD:-}; BOARD_USER=${BOARD_USER:-root}; BOARD_DIR=${BOARD_DIR:-/home/root/vio}; PY=${PY:-python3}
RUN_ARGS=${RUN_ARGS:-run --raw}; WAIT=${WAIT:-10}; SSH_OPTS=${SSH_OPTS:-"-o ConnectTimeout=5"}
[ -n "$BOARD" ] || { echo "FAIL set BOARD to the board's address, e.g. BOARD=192.168.75.1 tools/deploy.sh"; exit 2; }
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
board_logs() {  # board_logs SETTINGS: its log_dir as a path on the board; run writes its logs there
  L=$(python3 -c 'import sys; sys.path.insert(0, "."); import settings as S; print(S.load(sys.argv[1])["log_dir"])' "$1" 2>/dev/null) || L=logs
  case $L in /*) echo "$L" ;; *) echo "$D/$L" ;; esac
}

if [ "$BOARD" != local ] && [ -z "$DRY" ]; then
  remote true || die "no ssh to $BOARD_USER@$BOARD. Check the cable or network, the address and the user."
fi
if [ -z "${NO_RSYNC:-}" ] && command -v rsync >/dev/null && remote "command -v rsync" >/dev/null 2>&1; then
  COPY=rsync; else COPY=tar; fi
echo "board $BOARD_USER@$BOARD:$D, copy with $COPY"
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT

# ---------------------------------------------------------------- the program on the board
# "[a]pb" matches apb_main.py but not this pattern's own text, so pkill spares the shell that runs it.
# Local tests add the folder, so other programs on this laptop are safe.
PAT="[a]pb_main.py"; [ "$BOARD" = local ] && PAT="$D/[a]pb_main.py"
# Launcher: SIGINT stops it like Ctrl+C (Raw recording flushed, log closed); a background job would ignore it.
LAUNCH="import runpy, signal, sys; signal.signal(signal.SIGINT, signal.default_int_handler); sys.argv = ['apb_main.py'] + sys.argv[1:]; runpy.run_path('$D/apb_main.py', run_name='__main__')"
board_run_logs() {  # the board's log_dir (its run logs), from its settings.json; $D/logs when it has none
  if remote "cat '$D/settings.json'" > "$TMP/board.json" 2>/dev/null && [ -s "$TMP/board.json" ]; then
    board_logs "$TMP/board.json"; else echo "$D/logs"; fi
}
not_flying() {  # not_flying LOG_DIR: stop here when its newest run log is fresh and shows sent positions
  [ -z "$DRY" ] && [ -z "${FORCE:-}" ] || return 0
  FLY=$(remote "f=\$(ls -t '$1'/run_*.csv 2>/dev/null | head -1); [ -n \"\$f\" ] || exit 0
    age=\$(( \$(date +%s) - \$(stat -c %Y \"\$f\") ))
    [ \$age -lt 5 ] && tail -n 25 \"\$f\" | awk -F, '\$7==1{n++} END{if(n) print n}'" || true)
  [ -z "$FLY" ] || die "the program on the board is sending positions now (the drone may be flying). Land first. FORCE=1 skips this stop."
}
set_clock() {   # APB manual: with only the USB link the board clock is wrong. Log names and dates use it.
  echo "clock: laptop $(date '+%Y-%m-%d %H:%M:%S %Z'), board $(remote "date '+%Y-%m-%d %H:%M:%S %Z'")"
  [ "$BOARD" = local ] || remote "date -u -s '$(date -u '+%Y-%m-%d %H:%M:%S')' >/dev/null"
}
show() {        # show LOG_DIR: the console, the newest run log in LOG_DIR, and if the program runs
  remote "cd '$D'; echo '--- logs/console.txt (last 15 lines)'; tail -n 15 logs/console.txt
    f=\$(ls -t '$1'/run_*.csv 2>/dev/null | head -1); [ -z \"\$f\" ] || { echo \"--- \$f (last 5 rows)\"; tail -n 5 \"\$f\"; }
    echo '--- running:'; pgrep -f '$PAT' >/dev/null && echo yes || echo 'NO: the program does not run. Read the console lines above.'"
}
stop_run() {
  step "stop the running program (SIGINT, then SIGTERM after 5 s)" remote "
    pkill -INT -f '$PAT' || true
    for i in 1 2 3 4 5 6 7 8 9 10; do pgrep -f '$PAT' >/dev/null || break; sleep 0.5; done
    pkill -TERM -f '$PAT' || true"
}
restart() {     # restart LOG_DIR: stop the program, start it detached, show the log after WAIT s
  stop_run
  # setsid + nohup, output to a file, no input: the program runs on when the ssh session or the cable ends.
  step "start: cd $D && setsid nohup $PY -u apb_main.py $RUN_ARGS > logs/console.txt" remote "
    cd '$D' || exit 1; N=nohup; command -v setsid >/dev/null && N='setsid nohup'
    \$N $PY -u -c \"$LAUNCH\" $RUN_ARGS > logs/console.txt 2>&1 < /dev/null &"
  [ -n "$DRY" ] && exit 0
  echo "waiting $WAIT s for the program to start ..."
  sleep "$WAIT"
  show "$1"
}

case $CMD in
  status) show "$(board_run_logs)"; exit 0 ;;
  stop)
    not_flying "$(board_run_logs)"; stop_run
    [ -n "$DRY" ] || remote "pgrep -f '$PAT' >/dev/null && echo 'running: yes (it did not stop)' || echo 'running: no'"
    exit 0 ;;
  start)
    RL=$(board_run_logs); not_flying "$RL"
    step "set the board clock from this laptop" set_clock
    restart "$RL"; exit 0 ;;
  boot-on|boot-off)
    [ "$BOARD" != local ] || [ -n "$DRY" ] || die "$CMD changes how the board starts. Not on this laptop: add -n to see the steps."
    remote "[ -d /run/systemd/system ]" || { echo "the board has no systemd: no start at power-on. After each power-on type: BOARD=$BOARD tools/deploy.sh start"; exit 1; }
    U=/etc/systemd/system/vio-run.service
    if [ "$CMD" = boot-off ]; then
      step "systemctl disable vio-run; delete $U" remote "systemctl disable vio-run 2>/dev/null; rm -f $U; systemctl daemon-reload"
      exit 0
    fi
    cat > "$TMP/unit" <<EOF
[Unit]
Description=apb_main.py $RUN_ARGS at power-on (tools/deploy.sh boot-on; boot-off removes it)

[Service]
WorkingDirectory=$D
ExecStart=/bin/sh -c 'mkdir -p logs && exec $PY -u apb_main.py $RUN_ARGS >> logs/console.txt 2>&1'
# SIGINT stops it as Ctrl+C does: the raw recording is saved. A crash or a refusal to start starts it again.
KillSignal=SIGINT
TimeoutStopSec=10
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
    cat "$TMP/unit"
    put_unit() { remote "cat > $U && systemctl daemon-reload && systemctl enable vio-run" < "$TMP/unit"; }
    step "write $U and enable it: the run starts at each power-on" put_unit
    exit 0 ;;
  bundle)
    TGZ=tools/bundle/vio-bundle-aarch64.tar.gz
    [ -f "$TGZ" ] || die "no $TGZ here. Build it: sh tools/bundle/build.sh"
    # The bundle: 5570 files, 359 MB, 366 MB on a Linux disk. On a FAT or exFAT card each file takes a whole
    # cluster (up to 128 KB): up to 800 MB. A mount with noexec cannot run it.
    BD=${BUNDLE_DIR:-$(remote 'for p in /data /sdcard /usb /home/root; do
        [ -d "$p" ] || continue
        grep -q " $p [^ ]* [^ ]*noexec" /proc/mounts && continue
        need=400000; grep -qE " $p (vfat|exfat|msdos) " /proc/mounts && need=800000
        [ "$(df -Pk "$p" 2>/dev/null | awk "NR==2 {print \$4}")" -ge $need ] 2>/dev/null && { echo "$p"; break; }
      done' || true)}
    [ -n "$BD" ] || die "no folder on the board with 400 MB free (800 MB on a FAT card); tried /data /sdcard /usb /home/root. Set BUNDLE_DIR."
    B=$BD/vio-bundle
    [ "$PY" != python3 ] || PY=$B/python/bin/python3.12
    step "set the board clock from this laptop" set_clock
    put_bundle() { remote "mkdir -p '$BD' && tar xzof - -C '$BD' || echo 'WARN tar reported errors (a FAT card keeps no owners or modes). The check below shows if Python runs.'" < "$TGZ"; }
    if [ -z "${FORCE:-}" ] && remote "[ -f '$B/vio/settings.json' ]"; then
      echo "$B is on the board already: not copied (FORCE=1 copies it again and overwrites the board's settings.json)"
    else step "copy $TGZ and extract it into $B" put_bundle; fi
    step "check: cd $B/vio && $PY -u apb_main.py check" remote "cd '$B/vio' && $PY -u apb_main.py check" || true
    echo "next: put these two lines into tools/deploy.env. Then push, start, stop and status use the bundle:"
    echo "  PY=$PY"; echo "  BOARD_DIR=$B/vio"
    exit 0 ;;
esac

if [ "$CMD" = pull ]; then
  # Read the board's settings first (also in a dry run): its log_dir tells where the logs are.
  remote "cat '$D/settings.json'" > "$TMP/settings.json" || die "the board has no $D/settings.json"
  python3 -c 'import sys; sys.path.insert(0, "."); import settings as S; S.load(sys.argv[1])' "$TMP/settings.json" \
    || die "the board's settings.json does not load here (another program version?). Nothing changed."
  pull_settings() {
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
  # Run writes its logs in log_dir (the digital camera: /sdcard/logs). logs/ holds the launcher's console.txt.
  LD=$(board_logs "$TMP/settings.json"); DIRS="$D/logs"; [ "$LD" = "$D/logs" ] || DIRS="$DIRS $LD"
  for L in $DIRS; do
    remote "[ -d '$L' ]" || { echo "no folder $L on the board"; continue; }
    if [ "$COPY" = rsync ]; then
      R="$L/"; [ "$BOARD" = local ] || R="$BOARD_USER@$BOARD:$L/"
      step "rsync new logs: $R -> logs/" rsync -a --ignore-existing -e "ssh $SSH_OPTS" "$R" logs/
    else
      NEW=""
      for f in $(remote "ls '$L' 2>/dev/null" || true); do [ -e "logs/$f" ] || NEW="$NEW $f"; done
      pull_tar() { remote "cd '$L' && tar cf - $NEW" | tar xf - -C logs; }
      if [ -n "$NEW" ]; then step "tar new logs from $L over ssh:$NEW" pull_tar; else echo "no new logs in $L"; fi
    fi
  done
  [ -n "$DRY" ] || { echo "newest logs here:"; ls -t logs | head -5; }
  exit 0
fi

# ---------------------------------------------------------------- push
LENS=$(json_get settings.json lens_file)
FILES="apb_main.py hold.py fc_link.py capture.py calib.py settings.py settings.json $LENS
EVENT_DAY.txt DECISION_CARD.txt tools/board_check.sh tools/fc_params.py"
for f in $FILES; do
  case $f in
    .clip-1|*/.clip-1|ground/*|videos/*|real/*|px4sim/PX4-Autopilot/*|apsim/ardupilot/*|/*|*..*)
      die "$f is on the never-copy list. Nothing copied." ;;
  esac
  [ -f "$f" ] || die "$f is missing here. Nothing copied."
done
python3 -c 'import sys; sys.path.insert(0, "."); import settings as S; S.load("settings.json")' \
  || die "settings.json does not load. Fix it first: python3 tools/use_profile.py show"

BL="$D/logs"
if [ -z "$DRY" ] && [ -z "${FORCE:-}" ]; then
  if remote "cat '$D/settings.json'" > "$TMP/board.json" 2>/dev/null && [ -s "$TMP/board.json" ]; then
    BL=$(board_logs "$TMP/board.json")       # the running program writes its run log there
    DIFF=$(python3 - "$TMP/board.json" settings.json <<'EOF'
import json, sys
a, b = (json.load(open(p)) for p in sys.argv[1:3])
keys = ("top", "mirror", "tilt_deg", "twist_deg", "focal_scale", "readout_s", "video_delay_s", "calibrated",
        "calib_capture", "calib_info", "mask_file", "origin_lat_lon_alt", "lens_measured")
print(" ".join(k for k in keys if a.get(k) != b.get(k)))
EOF
)
    [ -z "$DIFF" ] || die "the board's calibration differs from this laptop's ($DIFF). Run 'tools/deploy.sh pull', apply your changes again, then push. FORCE=1 overwrites the board's values."
  fi
  not_flying "$BL"
fi

step "set the board clock from this laptop" set_clock
step "mkdir -p $D/logs" remote "mkdir -p '$D/logs'"
if [ "$COPY" = rsync ]; then
  R="$D/"; [ "$BOARD" = local ] || R="$BOARD_USER@$BOARD:$D/"
  step "rsync -a --relative $(echo $FILES) $R" rsync -a --relative -e "ssh $SSH_OPTS" $FILES "$R"
else
  copy_tar() { tar cf - $FILES | remote "tar xf - -C '$D'"; }
  step "tar cf - $(echo $FILES) | ssh ... tar xf - -C $D" copy_tar
fi
[ -z "${NO_RESTART:-}" ] || { echo "copied; not restarted (NO_RESTART)"; exit 0; }
restart "$(board_logs settings.json)"
