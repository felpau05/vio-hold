#!/bin/sh
# Clone and build the Betaflight SITL (software in the loop) with the MT optical flow and rangefinder drivers.
# BF_BUILD_DIR: the folder for the clone and the binary (not the project folder). Needs git, gcc, make.
# The stock SITL build has no rangefinder and no optical flow: common_pre.h turns them on only when SITL is not
# defined. EXTRA_FLAGS=-DUSE_OPTICALFLOW_MT adds the MT flow driver; common_post.h then adds the MT rangefinder,
# USE_RANGEFINDER and USE_OPTICALFLOW. The ORQAAPB target gets the same drivers from common_pre.h.
# Tested 2026-11-20: commit 09ea1c692f, gcc 13.3, 1 to 2 minutes with -j2.
set -e
: "${BF_BUILD_DIR:?set BF_BUILD_DIR to a work folder}"
BRANCH=orqa-betaflight_2026.6-maintenance
WANT=09ea1c692fc733a9137737ce26f16b66bb1914f7
mkdir -p "$BF_BUILD_DIR"
cd "$BF_BUILD_DIR"
[ -d orqa-betaflight ] || git clone --depth 1 -b "$BRANCH" https://github.com/orqafpv/orqa-betaflight.git
cd orqa-betaflight
HEAD=$(git rev-parse HEAD)
[ "$HEAD" = "$WANT" ] || echo "NOTE: clone is at $HEAD, the tests ran on $WANT"
nice -n 19 make -j2 TARGET=SITL EXTRA_FLAGS="-DUSE_OPTICALFLOW_MT" > "$BF_BUILD_DIR/build.log" 2>&1 || { tail -20 "$BF_BUILD_DIR/build.log"; exit 1; }
rm -f "$BF_BUILD_DIR/betaflight_SITL_flow.elf"   # a running SITL keeps its own copy
cp obj/main/betaflight_SITL.elf "$BF_BUILD_DIR/betaflight_SITL_flow.elf"
nm "$BF_BUILD_DIR/betaflight_SITL_flow.elf" | grep -q mtOpticalflowGetData || { echo "FAIL: no MT flow driver in the binary"; exit 1; }
echo "export BF_SITL=$BF_BUILD_DIR/betaflight_SITL_flow.elf BF_SRC=$BF_BUILD_DIR/orqa-betaflight"
