#!/bin/sh
# Start the live program at power-up (Stage 2 bench item E2). Uses apb_ctl.sh to start and stop it.
# Copy this file and vio-live.service to /data/vio on the APB. Then, on the APB:
#   sh /data/vio/vio_boot.sh install     check the files, put the unit in /etc/systemd/system, enable it
#                                        (needs the organizers' yes: Stage 2 section 02, boot configuration)
#   sh /data/vio/vio_boot.sh remove      disable the unit, stop the program, delete the unit
#   sh /data/vio/vio_boot.sh check       the files that apb_live.py needs, the start arguments, md5 values
#   sh /data/vio/vio_boot.sh status      unit state, program state, md5 values, time since power-up
#   sh /data/vio/vio_boot.sh restart     check, then restart the program through the unit (after a copy or a boot.env change)
#   sh /data/vio/vio_boot.sh args        show the start arguments that the unit uses
#   sh /data/vio/vio_boot.sh clean-pid   (the unit runs this) delete a pid file from before the power cut
# Start arguments: /data/vio/boot.env, one line, for example: VIO_ARGS=--height 1.0 --pos floor --rc
D=/data/vio
UNIT=vio-live.service
PID=$D/apb_live.pid
DEFAULT_ARGS="--height 1.0 --pos floor --rc"
# apb_live.py and its local imports (found on the laptop with an import scan, 2026-11-21), and the data it reads.
# An older apb_live.py does not know --rc: the program then stops at boot, and E2 fails.
NEED="apb_live.py apb_lift.py msp_rc.py msp_link.py apb_est.py apb_main.py apb_sensors.py calib.py capture.py
fc_link.py hold.py settings.py v4l2_mmap.py settings.json apb_ctl.sh vio_boot.sh $UNIT"

fail() { echo "FAIL $*"; exit 1; }

clean_pid() {
    [ -f "$PID" ] || return 0
    p=$(cat "$PID" 2>/dev/null)
    if [ -n "$p" ] && [ -r "/proc/$p/cmdline" ] && tr '\0' '\n' < "/proc/$p/cmdline" | head -1 | grep -q python \
        && tr '\0' '\n' < "/proc/$p/cmdline" | grep -q 'apb_live\.py$'; then
        fail "apb_live.py runs now as pid $p, outside the unit. Stop it first: sh $D/apb_ctl.sh stop"
    fi
    rm -f "$PID"
    echo "removed an old pid file (pid $p is not apb_live.py)"
}

cur_args() {
    # The same value that systemd reads from boot.env (EnvironmentFile), else the unit default.
    a=""
    [ -f "$D/boot.env" ] && a=$(grep '^VIO_ARGS=' "$D/boot.env" | tail -1 | cut -d= -f2- | sed 's/^"\(.*\)"$/\1/')
    [ -n "$a" ] && echo "$a" || echo "$DEFAULT_ARGS"
}

args() {
    if [ -f "$D/boot.env" ] && grep -q '^VIO_ARGS=' "$D/boot.env"; then
        echo "start arguments (boot.env): $(cur_args)"
    else
        echo "start arguments (unit default, no boot.env): $(cur_args)"
    fi
}

md5s() {
    for f in apb_live.py apb_lift.py msp_rc.py; do
        [ -f "$D/$f" ] && printf '%s %s  ' "$(md5sum "$D/$f" | cut -c1-8)" "$f"
    done
    echo "(compare with md5sum on the laptop)"
}

py_check() {
    # The modules that apb_live.py imports at start (module level, followed through local files) must exist,
    # and settings.load() must accept settings.json. Executes no local module except settings (and what it imports).
    # Catches: a copied settings.py or apb_main.py that imports gnss_rule with no gnss_rule.py on the board,
    # and a settings.json that settings.py refuses (for example params_ardupilot GPS1_TYPE 0 under the GNSS rule).
    (cd "$D" && PYTHONPATH="$D/pylib" "${PY:-python3}" - <<'PYEOF'
import ast, importlib.util, os, sys
sys.path.insert(0, os.getcwd())

def top_imports(path):
    """Module-level imports. Skip 'if __name__ == ...' blocks and try-blocks that catch ImportError (optional)."""
    out = []
    def walk(body):
        for n in body:
            if isinstance(n, ast.Import):
                out.extend(a.name.split(".")[0] for a in n.names)
            elif isinstance(n, ast.ImportFrom) and n.module and not n.level:
                out.append(n.module.split(".")[0])
            elif isinstance(n, ast.If):
                if "__name__" in ast.dump(n.test):
                    continue
                walk(n.body); walk(n.orelse)
            elif isinstance(n, ast.Try):
                optional = any("ImportError" in ast.dump(h) or "ModuleNotFoundError" in ast.dump(h) or h.type is None
                               for h in n.handlers)
                if not optional:
                    walk(n.body)
    walk(ast.parse(open(path).read()).body)
    return out

seen, todo, missing = set(), ["apb_live", "msp_rc"], []
while todo:
    m = todo.pop()
    if m in seen:
        continue
    seen.add(m)
    if importlib.util.find_spec(m) is None:
        missing.append(m)
        continue
    if os.path.exists(m + ".py"):
        todo.extend(top_imports(m + ".py"))
bad = False
if missing:
    print("FAIL Python modules not found on the board: " + " ".join(sorted(missing))
          + ". Copy them (for example gnss_rule.py with a merged settings.py or apb_main.py).")
    bad = True
try:
    import settings as S
    S.load()
    print("PASS settings.py loads settings.json")
except Exception as ex:
    print(f"FAIL settings.load(): {type(ex).__name__}: {ex}")
    bad = True
sys.exit(1 if bad else 0)
PYEOF
    )
}

check_files() {
    bad=0
    for f in $NEED; do
        [ -f "$D/$f" ] || { echo "FAIL $D/$f is missing. Copy it from the laptop."; bad=1; }
    done
    py_check || bad=1
    for w in $(cur_args); do
        case $w in
        --*) grep -q -- "\"$w\"" "$D/apb_live.py" 2>/dev/null \
                 || { echo "FAIL apb_live.py does not know the start argument $w. Copy the new apb_live.py, apb_lift.py and msp_rc.py together."; bad=1; } ;;
        esac
    done
    md5s
    [ $bad = 0 ] && echo "PASS files and start arguments" || return 1
}

case "$1" in
install)
    [ "$(id -u)" = 0 ] || fail "run as root"
    check_files || fail "install stopped. Correct the lines above."
    cp "$D/$UNIT" "/etc/systemd/system/$UNIT" || fail "copy to /etc/systemd/system"
    systemctl daemon-reload
    systemctl enable "$UNIT" || fail "systemctl enable $UNIT"
    sync                                # a new file that is not on the disk can be empty after a power cut
    echo "PASS installed and enabled. The program starts at the next power-up."
    args
    echo "To start it now: stop a manual run (sh $D/apb_ctl.sh stop), then: sh $D/vio_boot.sh restart"
    echo "To remove: sh $D/vio_boot.sh remove"
    ;;
remove)
    systemctl disable --now "$UNIT" 2>/dev/null
    rm -f "/etc/systemd/system/$UNIT"
    systemctl daemon-reload
    sync
    echo "removed. The program does not start at power-up."
    ;;
check)
    check_files || exit 1
    args
    ;;
restart)
    check_files || fail "restart stopped. Correct the lines above."
    sync
    if ! systemctl restart "$UNIT"; then
        systemctl status "$UNIT" --no-pager | tail -8
        systemctl stop "$UNIT"          # Restart=on-failure would try again every 3 s with no limit
        fail "restart. The unit is stopped. Read: tail -40 $D/logs/apb_live_now.out"
    fi
    sleep 2
    sh "$0" status
    ;;
status)
    echo "unit: $(systemctl is-enabled "$UNIT" 2>/dev/null || echo not-installed), $(systemctl is-active "$UNIT" 2>/dev/null)"
    sh "$D/apb_ctl.sh" status
    echo "last program output (logs/apb_live_now.out):"
    tail -3 "$D/logs/apb_live_now.out" 2>/dev/null
    args
    md5s
    echo "time since power-up: $(cut -d' ' -f1 /proc/uptime) s"
    df -h /data | tail -1 | awk '{print "free on /data: " $4}'
    ;;
args)
    args
    ;;
clean-pid)
    clean_pid
    ;;
*)
    echo "usage: sh $D/vio_boot.sh install | remove | check | status | restart | args | clean-pid"
    exit 2
    ;;
esac
