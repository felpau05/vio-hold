#!/bin/sh
# Run on the board, in the program folder:  sh tools/board_check.sh [WHEEL_FOLDER]
# Prints PASS or FAIL for each part the onboard program needs, and the exact fix for each FAIL.
# WHEEL_FOLDER: the "wheels" folder on the USB stick (found by itself under /run/media, /media, /mnt).
cd "$(dirname "$0")/.." || exit 1
PY=${PY:-python3}
fails=0
pass() { echo "PASS $*"; }
fail() { echo "FAIL $*"; fails=$((fails + 1)); }
info() { echo "INFO $*"; }

W=${1:-}
[ -n "$W" ] || W=$(ls -d /run/media/*/wheels /media/*/wheels /mnt/*/wheels tools/wheels 2>/dev/null | head -1)
[ -n "$W" ] || W="<USB stick>/wheels"

# ---- machine
ARCH=$(uname -m)
[ "$ARCH" = aarch64 ] && pass "CPU architecture $ARCH" || fail "CPU architecture $ARCH: the wheels are for aarch64 only."
PART=$(grep -m1 "CPU part" /proc/cpuinfo 2>/dev/null | awk '{print $NF}')
[ "$PART" = 0xd03 ] && PART="$PART (Cortex-A53)"
MODEL=$(cat /sys/firmware/devicetree/base/model 2>/dev/null | tr -d '\0')
GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)
MHZ=$(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq 2>/dev/null)
info "CPU: $(nproc 2>/dev/null || grep -c ^processor /proc/cpuinfo) cores, part ${PART:-?}, max ${MHZ:-?} kHz, governor ${GOV:-?}, board ${MODEL:-?}"
[ "$GOV" = powersave ] && info "governor powersave makes Hold slower. For the flight: echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
LIBC=$(ldd --version 2>&1 | head -1 | grep -o '[0-9]*\.[0-9]*$')
info "glibc ${LIBC:-?} (numpy for Python 3.11 and 3.12 needs 2.28 or newer)"

# ---- Python
if ! command -v "$PY" >/dev/null; then
  fail "no $PY on this board. Nothing else can run. The Yocto image needs python3."
  echo "$fails FAIL"; exit 1
fi
V=$("$PY" -c 'import sys; print("%d.%d" % sys.version_info[:2])')
case $V in 3.10) T=cp310 ;; 3.11) T=cp311 ;; 3.12) T=cp312 ;; *) T="" ;; esac
[ -n "$T" ] && pass "Python $V (wheel folder $T)" || fail "Python $V: no wheel set for it (only 3.10, 3.11, 3.12). See tools/wheels/README.txt, last part."
PKGS="numpy opencv-python-headless pymavlink pyserial"
if "$PY" -m pip --version >/dev/null 2>&1; then
  FIX="$PY -m pip install --no-index --find-links $W/$T $PKGS"
  info "pip is here"
else
  FIX="$PY $W/$T/pip-26.3-py3-none-any.whl/pip install --no-index --find-links $W/$T $PKGS"
  info "no pip: the fix below runs pip from its wheel"
fi

# ---- packages
if v=$("$PY" -c 'import numpy; print(numpy.__version__)' 2>/dev/null); then pass "numpy $v"
else fail "numpy does not import."; echo "     fix: $FIX"; fi

if v=$("$PY" -c 'import cv2; print(cv2.__version__)' 2>/dev/null); then
  if "$PY" -c 'import cv2, sys; import cv2.videoio_registry as r; sys.exit(0 if r.hasBackend(cv2.CAP_V4L2) else 1)' 2>/dev/null \
     || "$PY" -c 'import cv2; print(cv2.getBuildInformation())' 2>/dev/null | grep -qi "v4l/v4l2: *YES"; then
    pass "cv2 $v with V4L2 video capture"
  else
    fail "cv2 $v has no V4L2 capture (a Yocto OpenCV without v4l?). The analog camera cannot open."
    echo "     fix: $FIX --force-reinstall   (then check again; the wheel has V4L2)"
  fi
else fail "cv2 (OpenCV) does not import."; echo "     fix: $FIX"; fi

if v=$("$PY" -c 'import os; os.environ["MAVLINK20"] = "1"; from pymavlink import mavutil; mavutil.mavlink.MAVLink_vision_position_estimate_message; import pymavlink; print(getattr(pymavlink, "__version__", "?"))' 2>/dev/null); then
  pass "pymavlink $v (MAVLink 2)"
else fail "pymavlink does not import."; echo "     fix: $FIX"; fi

DEV=$("$PY" -c 'import json; print(json.load(open("settings.json"))["fc_device"])' 2>/dev/null)
case $DEV in
  /dev/*) "$PY" -c 'import serial' 2>/dev/null && pass "pyserial (fc_device $DEV is a serial port)" \
          || { fail "pyserial does not import, and fc_device $DEV is a serial port."; echo "     fix: $FIX"; } ;;
  *) info "fc_device ${DEV:-?}: pyserial not needed" ;;
esac

# ---- the program itself
if out=$("$PY" -c 'import apb_main, settings; settings.load()' 2>&1); then pass "apb_main.py imports and settings.json loads"
else fail "apb_main.py or settings.json: $(echo "$out" | tail -1)"; fi

VID=$("$PY" -c 'import json; print(json.load(open("settings.json"))["video_device"])' 2>/dev/null)
[ -e "${VID:-/dev/video0}" ] && info "video device ${VID:-/dev/video0} is there" || info "video device ${VID:-/dev/video0} is missing: check the grabber, or video_device in settings.json"
pgrep -f mavlink-route >/dev/null 2>&1 && info "mavlink-router runs" || info "mavlink-router does not run (needed when fc_device is udpin:...)"
info "free disk here: $(df -h . | awk 'NR==2 {print $4}')"

[ $fails -eq 0 ] && echo "ALL PASS" || echo "$fails FAIL"
exit $fails
