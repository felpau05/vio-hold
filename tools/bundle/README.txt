CASE B BUNDLE: OUR OWN PYTHON FOR THE BOARD (no internet needed)

Words: APB = Auto Pilot Board. FC = flight controller. glibc = GNU C Library. musl = another C library.
ELF = Executable and Linkable Format. qemu = Quick Emulator. FAT = File Allocation Table (an SD card file system).

WHICH CASE
On the board, in the program folder, type: sh tools/board_check.sh
It prints one line: "use case A" or "use case B".
Case A: the board's python3 is 3.10, 3.11 or 3.12, it has the standard modules that the program imports,
        and its site-packages folder is writable with 350 MB free. Install tools/wheels (tools/wheels/README.txt).
Case B: every other 64-bit ARM (aarch64) board with glibc 2.27 or newer. Use this bundle.
A board with musl: no case works. See MUSL below.

WHAT IS IN THE BUNDLE
tools/bundle/vio-bundle-aarch64.tar.gz (not in git: "sh tools/bundle/build.sh" makes it)
  vio-bundle/python    CPython 3.12.16 for aarch64 Linux with glibc, relocatable. Source: python-build-standalone
                       release 20261110, asset cpython-3.12.16+20261110-aarch64-unknown-linux-gnu-install_only.
                       With numpy 2.5.5, opencv-python-headless 5.0.0.93 (V4L2 yes, GStreamer no), pymavlink 2.4.50
                       (with lxml 6.1.3 and fastcrc 0.5.0) and pyserial 3.5, from tools/wheels/cp312.
                       No matplotlib and no imageio-ffmpeg: the onboard program does not import them.
  vio-bundle/vio       the onboard program: the files that "tools/deploy.sh" push copies, with settings.json.
  vio-bundle/syslibs   libstdc++.so.6, libgcc_s.so.1, libz.so.1 from Debian 11 arm64 (libstdc++6 10.2.1-6,
                       libgcc-s1 10.2.1-6, zlib1g 1.2.11.dfsg-2+deb11u2). Only for a board without them.
The Python program is vio-bundle/python/bin/python3.12. There is no "python3" name: the bundle has no
symbolic links, so a FAT card can hold it.

Facts of the bundle built on 2026-11-21 at 07:06:
  .tar.gz size          113,848,696 bytes (109 MiB)
  after extraction      358.6 MB in 5571 files (one more than before: tools/fc_params.py). 366 MB on a Linux disk. Up to 800 MB on an exFAT card.
  sha256                bbbaf507301660e727e900e26e9c84d6f1a4262adb75f2b6699a6a2e9463d77e
  lowest glibc          2.27 (numpy). python3.12 and OpenCV need 2.17 only. Details: proof/glibc_and_libs.txt
  from the board        glibc (libc, libm, libdl, libpthread, librt, libutil, ld-linux-aarch64.so.1), and
                        libstdc++.so.6, libgcc_s.so.1, libz.so.1 (or the copies in syslibs)
A new build has another sha256, because the files get new dates.
This build holds the program files of 2026-11-21 07:06 (the same md5 as the project files, checked for apb_main.py,
fc_link.py, settings.py, hold.py, EVENT_DAY.txt, DECISION_CARD.txt and the two tools). After a later code change,
build again: sh tools/bundle/build.sh (no internet needed). Python and its packages stay the same.
The proof files in proof/ are from the build of 2026-11-20. The emulated ARM test was not run again on this build.
This build has the estimator options of 2026-11-21 (hold.py d78329d3, all off by default) and att_source gyro_fcyaw.
The bundle of 02:08, before those changes, is kept as vio-bundle-aarch64_tested_0208.tar.gz in this folder.
"tools/deploy.sh" push always copies the current program files.

EVENT DAY, CASE A (the board's python3 is 3.10 to 3.12; no internet)
  1. BOARD=192.168.75.1 NO_RESTART=1 tools/deploy.sh
  2. ssh root@192.168.75.1 "cd /home/root/vio && sh tools/board_check.sh"
     It must print "use case A" and the Python version.
  3. scp -r tools/wheels/cp312 root@192.168.75.1:/tmp/          (cp310 or cp311 for that Python)
  4. ssh root@192.168.75.1 "python3 -m pip install --no-index --find-links /tmp/cp312 numpy opencv-python-headless pymavlink pyserial"
     If pip says "externally-managed-environment", add --break-system-packages. No pip: tools/wheels/README.txt, STEP 3.
  5. ssh root@192.168.75.1 "cd /home/root/vio && sh tools/board_check.sh && python3 apb_main.py check"

EVENT DAY, CASE B (no Python on the board that we can use)
  1. BOARD=192.168.75.1 tools/deploy.sh bundle
     It sets the board clock. It selects the first of /data, /sdcard, /usb, /home/root that has 400 MB free
     (800 MB on a FAT or exFAT card) and is not mounted noexec. It copies the bundle there, extracts it, and runs
     "apb_main.py check" with the bundle's Python. It ends with two lines, for example:
       PY=/data/vio-bundle/python/bin/python3.12
       BOARD_DIR=/data/vio-bundle/vio
  2. Put BOARD=192.168.75.1 and these two lines into tools/deploy.env.
     After that, tools/deploy.sh (push), start, stop, status and pull use the bundle.
  3. If check says "libstdc++.so.6" (or libgcc_s.so.1, libz.so.1) "cannot open shared object file",
     change the PY line in tools/deploy.env to:
       PY=env LD_LIBRARY_PATH=/data/vio-bundle/syslibs /data/vio-bundle/python/bin/python3.12
     Then type "tools/deploy.sh bundle" again. It does not copy again. It runs check again.
  4. board_check with the bundle's Python:
       ssh root@192.168.75.1 "cd /data/vio-bundle/vio && PY=/data/vio-bundle/python/bin/python3.12 sh tools/board_check.sh"
     If step 3 was necessary, put LD_LIBRARY_PATH=/data/vio-bundle/syslibs in front of PY=.
  Another folder: BUNDLE_DIR=/some/folder tools/deploy.sh bundle
  A second "tools/deploy.sh bundle" does not copy again, because the board's settings.json holds its calibration.
  FORCE=1 copies again and overwrites that settings.json.
  On a FAT or exFAT card, tar can print errors about owners and modes. The files are there. The check shows if Python runs.
  Without the laptop script (USB stick): tar xzf vio-bundle-aarch64.tar.gz -C /data
     then: cd /data/vio-bundle/vio && ../python/bin/python3.12 apb_main.py check

MAKE THE BUNDLE AGAIN (on the laptop)
  sh tools/bundle/build.sh
  It needs tools/wheels/cp312, tools/bundle/syslibs, a Python 3.12 with pip (~/.venvs/vio/bin/python, or PIP_PY=...),
  and tools/bundle/cpython-3.12.16+20261110-aarch64-unknown-linux-gnu-install_only.tar.gz.
  When that file is missing, the script downloads it from GitHub (internet needed once). It checks its sha256.
  The script copies the program with tools/deploy.sh (BOARD=local), so the bundle holds the current program files.
  After a code change, push with tools/deploy.sh as usual. A new bundle is necessary only for a new board.

MUSL
  The bundle needs glibc. On a musl board (board_check prints "C library: musl"), python3.12 does not start:
  the shell says "not found", because the board has no /lib/ld-linux-aarch64.so.1.
  The wheels in tools/wheels do not install either: they are manylinux wheels, and manylinux means glibc.
  python-build-standalone has a musl Python (aarch64-unknown-linux-musl), and numpy, pymavlink, lxml and fastcrc
  have musllinux wheels. But PyPI has no musllinux wheel of opencv-python-headless, for any version.
  A musl bundle needs OpenCV built from source. That takes more than 20 minutes: not done.
  Yocto uses glibc by default. No manual states the C library of the board.

PROOF (tools/bundle/proof/)
No board was available. The checks ran on this laptop under qemu-user 8.2 (qemu-aarch64-static, taken from
the Ubuntu archive with apt-get download and dpkg -x: no install, no root, no binfmt change). CPU model: cortex-a53,
the core of the i.MX8M Plus. Root file systems: arm64 Docker Hub images, exported: Debian 11 (glibc 2.31),
Debian 12 (2.36), Debian 13 (2.41), and a root that holds only glibc (from Debian 11).
Each run had its own network namespace with loopback only (no internet), nice 19, one test at a time.
An emulated time is not the board's speed. These files give pass or fail only. Processing times were removed.

  caseA_cp310.txt caseA_cp311.txt caseA_cp312.txt
      Case A. pip installs tools/wheels/cp31X with no index and no network into Python 3.10, 3.11, 3.12.
      numpy, cv2 (V4L2 yes), pymavlink and the onboard modules import. Hold.process passes.
      cp312 also with pip run from its wheel (a board with no pip).                              PASS
  deb11_imports.txt deb13_imports.txt
      The bundle on glibc 2.31 and 2.41: numpy, cv2 (V4L2 yes, GStreamer no), pymavlink (MAVLink 2),
      pyserial and the onboard modules import, settings.json loads.                              PASS
  glibconly_imports.txt glibconly_hold.txt
      The same on a root with glibc only, with LD_LIBRARY_PATH=syslibs. Without syslibs, python3.12
      does not start there (libgcc_s.so.1 missing).                                              PASS
  deb11_help.txt deb13_help.txt            apb_main.py --help                                     PASS
  deb11_check.txt deb13_check.txt          apb_main.py check with no camera and no FC. It runs to the
      end. FAIL lines only for calibration, FC, video and mask, as expected without hardware.    runs
  deb11_delay/replay/timing.txt, deb13_...  apb_main.py delay, replay and timing on a recorded wiggle
      flight (apsim/runs/c_wig_40m_w20g40_s12).                                                   PASS
  deb11_hold.txt deb13_hold.txt            hold_check.py: Hold.process on 120 synthetic pictures.  PASS
  native_vs_emulated.txt   The same numbers as on this x86_64 laptop: hold_check identical;
      replay 245 rows, positions within 0.0001 m; timing identical.
  glibc_and_libs.txt       readelf: the glibc version that each file needs; libraries from the board.
  deb11_test_*.txt         The test groups of tests_apb/run_all.sh on Debian 11 (results below).
  deb11_board_check.txt deb11_board_check_route.txt
      board_check.sh with the bundle's Python: package and program lines PASS. The FC line FAILs
      (no FC) and prints the --route step. The machine lines describe this laptop.
  board_check_decision_logic.txt   The "use case" line for 5 stand-in boards on this laptop.
  deploy_bundle_local.txt  deploy.sh bundle with BOARD=local: copy, extract (the same 5570 files and sha256),
      check; a second call does not copy; with no folder it stops with a FAIL message.
  deploy_start_stop_status_local.txt   deploy.sh push, start (in a terminal session that then ends),
      status (still running, own session), stop (SIGINT), status.
  systemd_user_unit.txt    The boot-on unit settings as a user unit: it runs; systemctl stop sends SIGINT;
      it ends with no restart.

Test groups (Debian 11 root):
  test_digital, test_replay_chunks, test_wind_assist, test_event, test_lens_store,
  test_calib, test_lens_d, sim_test.timing_test                                             ALL PASS
  test_basics    1 FAIL: the V4L2 query of this laptop's /dev/video0. qemu 8.2 does not pass V4L2
                 requests. All other checks PASS, with the mask from the real analog video file.
  test_robust    1 FAIL: D5 (a) needs 30 fixes in 1 s of real time. All other checks PASS.
  test_fc_link   1 FAIL: clock mapping error 3.6 to 3.9 ms, limit 3 ms (real-time jitter).
  test_run       5 FAIL, test_bench 3 FAIL: these tests feed pictures in real time. The emulation
                 processed too few pictures per second. In test_run the error check (under 1 m RMS),
                 the frozen-picture and stopped-attitude checks PASS.
  These 10 FAIL lines depend on real time. Emulation cannot decide them. Run them on the board.
  test_lens_d gives slightly other lens numbers on aarch64 than on x86_64 (the tracker finds a few other
  points). Every check passes on both; its last case is a KNOWN LIMIT on x86_64 and a PASS here
  (native_vs_emulated.txt). Hold, replay and timing give the same numbers on both.
  Debian 13 root: test_basics (the same 1 FAIL), test_digital, test_replay_chunks, test_wind_assist,
  test_event, test_lens_store ALL PASS (deb13_test_*.txt).
  Not run: test_ap_sitl (needs the ArduPilot SITL simulator), test_live (imports px4sim/sitl.py, the PX4
  simulator package).

qemu with its default CPU model ("max", with SVE) gave wrong tracking: "fit out of range" on each frame.
With cortex-a53 and cortex-a76 the numbers equal those of x86_64. The cause was not found (qemu's SVE
emulation, or an SVE code path in a library). The board's Cortex-A53 has no SVE.

NOT PROVEN
  Anything on the real board: its Python, C library, free space, file systems, camera, serial route, FC.
  The board's speed. Real-time behaviour (the 10 FAIL lines above).
  boot-on and boot-off on a real systemd system (only the unit text, a dry run and a user unit ran here).
  The board clock set (date -u -s was checked for parsing only: BusyBox and GNU date, no root here).
  A run that goes on after a real ssh session ends (a local terminal session ended instead).
  If the board's logind kills the processes of a closed session (KillUserProcesses=yes), start can stop
  when ssh ends; boot-on avoids that. Extraction onto a FAT or exFAT card. musl.
