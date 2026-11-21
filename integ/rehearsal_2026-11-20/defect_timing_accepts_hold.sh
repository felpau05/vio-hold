#!/bin/sh
# Defect: "apb_main.py timing" passes on raw chunks of a Position-mode hold in gusty wind and writes wrong values.
# It has no way to know which chunks hold the wiggle. Flight: integ/rehearsal_2026-11-20/apflights.py 31 60
# (ArduCopter SITL, Justice D, 60 m, w20g40, seed 31). Truth: delay 40 ms, readout +14 ms. Chunk 002 = the wiggle,
# chunks 005-009 = the hold. Each chunk alone, on a fresh copy of the flight's settings.
# Output: timing_per_chunk_60m.txt. Patch: patches/apb_main_timing_window.patch (timing takes the run log and keeps
# only the 15 s before the pilot's switch).
cd "$(dirname "$0")/../.." || exit 1
L=reh_jd_wig_60m_s31
T=$(mktemp)
for f in integ/rehearsal_2026-11-20/runs/${L}_raw/raw_*.npz; do
  cp apsim/runs/${L}_settings.json "$T"
  echo "== $(basename "$f")"
  ~/.venvs/vio/bin/python apb_main.py timing "$f" --settings "$T" | head -1
done
rm -f "$T"
