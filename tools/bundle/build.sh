#!/bin/sh
# Build the case B bundle (the board has no Python that we can use):
#   sh tools/bundle/build.sh        ->  tools/bundle/vio-bundle-aarch64.tar.gz
# vio-bundle/python: CPython 3.12 for 64-bit ARM Linux with glibc (python-build-standalone, relocatable),
#                    with numpy, OpenCV, pymavlink and pyserial from tools/wheels/cp312.
# vio-bundle/vio:    the onboard program: the files that "tools/deploy.sh" push copies.
# vio-bundle/syslibs: libstdc++.so.6, libgcc_s.so.1, libz.so.1, for a board that does not have them.
# Needs on this laptop: tools/wheels/cp312, a Python 3.12 with pip (PIP_PY), and the CPython file in tools/bundle.
# The CPython file is downloaded once (curl) when it is not here. After that, the build needs no internet.
set -eu
cd "$(dirname "$0")/../.."
REL=20261110
PBS=cpython-3.12.16+$REL-aarch64-unknown-linux-gnu-install_only.tar.gz
SHA=4792f81750282597fb2a3389fe18b945ed2ba6aad5e45ad406d122c58cb33081
OUT=tools/bundle/vio-bundle-aarch64.tar.gz
PIP_PY=${PIP_PY:-$HOME/.venvs/vio/bin/python}

[ -f "tools/bundle/$PBS" ] || curl -fL -o "tools/bundle/$PBS" \
  "https://github.com/astral-sh/python-build-standalone/releases/download/$REL/$(echo "$PBS" | sed 's/+/%2B/')"
echo "$SHA  tools/bundle/$PBS" | sha256sum -c -
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
mkdir "$T/vio-bundle"
tar xzf "tools/bundle/$PBS" -C "$T/vio-bundle"
P=$T/vio-bundle/python
# No symbolic links (a FAT SD card cannot hold them): run python/bin/python3.12.
# No libpython .so (63 MB): python3.12 has it built in, and only programs that embed Python load it.
find "$P" -type l -delete
rm -f "$P"/lib/libpython3*.so*
"$PIP_PY" -m pip install --quiet --no-index --find-links tools/wheels/cp312 --only-binary=:all: \
  --implementation cp --python-version 3.12 --platform manylinux2014_aarch64 --platform manylinux_2_17_aarch64 \
  --platform manylinux_2_28_aarch64 --target "$P/lib/python3.12/site-packages" \
  numpy opencv-python-headless pymavlink pyserial
rm -rf "$P/lib/python3.12/site-packages/bin"     # pymavlink tools, with this laptop's Python in their first line
# Only for a board without them: libstdc++, libgcc_s, libz (Debian 11 arm64). Used with LD_LIBRARY_PATH only.
cp -r tools/bundle/syslibs "$T/vio-bundle/"
BOARD=local BOARD_DIR="$T/vio-bundle/vio" NO_RESTART=1 NO_RSYNC=1 sh tools/deploy.sh
tar czf "$OUT" -C "$T" --owner=0 --group=0 vio-bundle
echo "bundle $OUT: $(du -k "$OUT" | cut -f1) KB, $(du -sk "$T/vio-bundle" | cut -f1) KB after extraction"
sha256sum "$OUT"
