#!/bin/sh
# Run on the board, in the program folder:  sh tools/board_check.sh [--route] [WHEEL_FOLDER]
# Prints PASS or FAIL for each part the onboard program needs, and the exact fix for each FAIL.
# It also prints one line "use case A" (the board's python3 + tools/wheels) or "use case B" (tools/bundle).
# --route: first run the two gpioset commands of the APB manual (UART3 /dev/ttymxc2 to the flight controller).
# WHEEL_FOLDER: the "wheels" folder on the USB stick (found by itself under /run/media, /media, /mnt, /usb).
# PY: the Python that runs the program, default python3 (case B: the bundle's python/bin/python3.12).
cd "$(dirname "$0")/.." || exit 1
PY=${PY:-python3}
fails=0
pass() { echo "PASS $*"; }
fail() { echo "FAIL $*"; fails=$((fails + 1)); }
info() { echo "INFO $*"; }
warn() { echo "WARN $*"; }
ROUTE_CMDS="gpioset -t0 -c gpiochip3 13=1; gpioset -t0 -c gpiochip3 0=0"

ROUTE=""; [ "${1:-}" = --route ] && { ROUTE=1; shift; }
W=${1:-}
[ -n "$W" ] || W=$(ls -d /run/media/*/wheels /media/*/wheels /mnt/*/wheels /usb/wheels /usb/*/wheels tools/wheels 2>/dev/null | head -1)
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
if ls /lib/ld-musl-* >/dev/null 2>&1; then LIBC=musl; GV=""
else
  GV=$( { getconf GNU_LIBC_VERSION; ldd --version; /lib/libc.so.6; /lib64/libc.so.6; /lib/aarch64-linux-gnu/libc.so.6; } 2>/dev/null \
        | head -1 | grep -o '[0-9][0-9]*\.[0-9][0-9]*' | tail -1)
  LIBC="glibc ${GV:-?}"
fi
info "C library: $LIBC (case A with Python 3.11 or 3.12, and case B, need glibc 2.27 or newer; Python 3.10 needs 2.17)"
PYS=$(for d in $(echo "$PATH" | tr : ' '); do ls "$d"/python3* 2>/dev/null; done | grep -o 'python3[.0-9]*$' | sort -u | tr '\n' ' ')
info "Python programs on this board: ${PYS:-none}"
MISS=""
for l in libstdc++.so.6 libgcc_s.so.1 libz.so.1; do
  ls /lib/$l /usr/lib/$l /lib64/$l /usr/lib64/$l /lib/*-linux-gnu/$l /usr/lib/*-linux-gnu/$l 2>/dev/null | grep -q . || MISS="$MISS $l"
done
[ -z "$MISS" ] && info "libstdc++, libgcc_s and libz are here (OpenCV and numpy need them)" \
  || warn "missing:$MISS. Case B: start the bundle's Python with LD_LIBRARY_PATH set to the bundle's syslibs folder (tools/bundle/README.txt)."
for p in / /data /sdcard /usb; do
  m=$(awk -v p="$p" '$2 == p {print $3 ($4 ~ /(^|,)noexec(,|$)/ ? ", noexec" : "")}' /proc/mounts 2>/dev/null | tail -1)
  if [ -n "$m" ]; then info "free disk on $p: $(df -h "$p" | awk 'NR==2 {print $4}') ($m)"
  elif [ -d "$p" ]; then info "$p is a folder on the / disk (not a mount)"
  else info "no $p"; fi
done

# ---- case A or case B. Case A: python3 3.10 to 3.12 with the standard modules the program imports,
# a writable site-packages with 350 MB free (the wheels take about 260 MB). Case B: aarch64 and glibc 2.27.
ge() { awk -v a="$1" -v b="$2" 'BEGIN { split(a, x, "."); split(b, y, "."); exit !(x[1] > y[1] || (x[1] == y[1] && x[2] >= y[2])) }'; }
MODS="argparse,atexit,csv,errno,fcntl,glob,hashlib,json,logging,platform,select,shutil,signal,socket,ssl,subprocess,termios,threading,traceback,zipfile"
SV=$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null)
NEED=2.27; [ "$SV" = 3.10 ] && NEED=2.17
WHY_A=$(
  [ "$ARCH" = aarch64 ] || { echo "the CPU is $ARCH"; exit; }
  [ "$LIBC" != musl ] || { echo "the C library is musl"; exit; }
  case $SV in 3.10|3.11|3.12) ;; "") echo "no python3"; exit ;; *) echo "python3 is $SV (wheels: 3.10, 3.11, 3.12)"; exit ;; esac
  ge "$GV" $NEED || { echo "glibc ${GV:-?} is older than $NEED"; exit; }
  e=$(python3 -c "import $MODS" 2>&1) || { echo "python3 has no $(echo "$e" | tail -1 | sed 's/.*No module named //')"; exit; }
  PL=$(python3 -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')
  [ -w "$PL" ] || { echo "$PL is not writable"; exit; }
  [ "$(df -Pk "$PL" | awk 'NR==2 {print $4}')" -ge 350000 ] || echo "less than 350 MB free at $PL"
)
WHY_B=$(
  [ "$ARCH" = aarch64 ] || { echo "the CPU is $ARCH"; exit; }
  [ "$LIBC" != musl ] || { echo "the C library is musl (OpenCV has no musl wheel, so there is no musl bundle)"; exit; }
  ge "$GV" 2.27 || echo "glibc ${GV:-?} is older than 2.27"
)
if [ -z "$WHY_A" ]; then echo "use case A"
elif [ -z "$WHY_B" ]; then info "not case A: $WHY_A"; echo "use case B"
else fail "neither case A ($WHY_A) nor case B ($WHY_B)"; fi

# ---- Python
if ! command -v "$PY" >/dev/null; then
  fail "no $PY on this board. Use case B: tools/bundle/README.txt."
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
    fail "cv2 $v has no V4L2 capture (a Yocto OpenCV without v4l?). The camera cannot open."
    echo "     fix: $FIX --force-reinstall   (then check again; the wheel has V4L2)"
  fi
  PIPE=$("$PY" -c 'import settings; print(settings.load()["video_pipeline"])' 2>/dev/null)
  if "$PY" -c 'import cv2, sys; import cv2.videoio_registry as r; sys.exit(0 if r.hasBackend(cv2.CAP_GSTREAMER) else 1)' 2>/dev/null \
     || "$PY" -c 'import cv2; print(cv2.getBuildInformation())' 2>/dev/null | grep -qi "gstreamer: *YES"; then
    info "cv2 $v has GStreamer: video_pipeline can be used"
  elif [ -n "$PIPE" ]; then
    fail "cv2 $v has no GStreamer, but video_pipeline is set. Empty video_pipeline in settings.json to use V4L2."
  else
    info "cv2 $v has no GStreamer: keep video_pipeline empty (V4L2)"
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

# ---- digital camera (Orqa Justice D) and board facts. APB manual: CSI 1 = /dev/video2, CSI 2 = /dev/video3.
for d in /dev/video2 /dev/video3; do
  [ -e $d ] && info "$d is there" || info "$d is missing"
done
if command -v v4l2-ctl >/dev/null; then
  for d in /dev/video2 /dev/video3; do
    [ -e $d ] && v4l2-ctl -d $d --list-formats-ext 2>&1 | sed "s|^|INFO $d: |"
  done
else info "no v4l2-ctl: the pixel formats are not shown. Read them in the video line of 'apb_main.py check'."; fi
# Flight controller route: UART3 (/dev/ttymxc2) reaches the FC only after two gpioset commands (APB manual).
[ -e /dev/ttymxc2 ] && info "/dev/ttymxc2 is there" || info "/dev/ttymxc2 is missing"
command -v gpioset >/dev/null || info "no gpioset on this board: the route commands ($ROUTE_CMDS) cannot run."
if [ -n "$ROUTE" ]; then
  sh -c "$ROUTE_CMDS" && info "ran: $ROUTE_CMDS" || fail "the route commands failed: $ROUTE_CMDS"
fi
if why=$("$PY" -c 'import fc_link, settings
fc, why = fc_link.connect(settings.load())
print(why or "firmware %s, baud %s" % (fc.firmware, fc.baud)); fc and fc.close(); raise SystemExit(fc is None)' 2>&1); then
  pass "flight controller: $why"
else
  fail "flight controller: $(echo "$why" | tail -1)"
  [ -n "$ROUTE" ] || echo "     next: connect UART3 to the FC, then check again: sh tools/board_check.sh --route   (runs: $ROUTE_CMDS)"
fi
IIO=""
for f in /sys/bus/iio/devices/*/name; do [ -r "$f" ] && IIO="$IIO $(cat "$f")"; done
[ -n "$IIO" ] && info "IIO devices:$IIO (icm42605 = IMU, dps310 = barometer: Linux can read them)" \
  || info "no IIO devices: Linux cannot read the ICM42605 IMU or the DPS310 barometer. The program uses the flight controller's IMU."
T=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null)
if [ -n "$T" ]; then
  C=$((T / 1000))
  # APB manual: board range 0 to 70 C, shutdown at 95 C. Fail limit 85 C (assumed: 10 C margin for a flight).
  if [ "$C" -ge 85 ]; then fail "board temperature $C C: the board stops at 95 C. Cool the board before the flight."
  elif [ "$C" -ge 70 ] || [ "$T" -lt 0 ]; then warn "board temperature $C C is outside the board range 0 to 70 C (APB manual)."
  else info "board temperature $C C (range 0 to 70 C, the board stops at 95 C)"; fi
else info "no board temperature in /sys/class/thermal/thermal_zone0"; fi
Y=$(date +%Y)
[ "$Y" -ge 2026 ] && info "clock $(date '+%Y-%m-%d %H:%M')" \
  || info "clock year $Y is before 2026: the board has no network time. Log names get a wrong date. Connect the network, or set the clock: date -s 'YYYY-MM-DD hh:mm'"
if grep -q " /sdcard " /proc/mounts 2>/dev/null; then
  info "SD card at /sdcard. Raw recording of the digital camera takes about 14 MB/s: set log_dir to /sdcard/logs."
else info "no SD card at /sdcard: logs and raw recordings go to the 4 GB eMMC (/data keeps its files after an update)."; fi

[ $fails -eq 0 ] && echo "ALL PASS" || echo "$fails FAIL"
exit $fails
