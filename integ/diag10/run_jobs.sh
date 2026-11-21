#!/bin/bash
# Run the replay jobs of a file (one argument line per job, each with its own --hold options), 2 at a time, each
# through gate_run.sh (nice 19, 1 thread per library, waits while a simulator or integ/kfmap/SITL_ACTIVE exists).
# Usage: run_jobs.sh jobs.txt
cd "$(dirname "$0")/../.."
cat "$1" | xargs -P 2 -I{} bash -c 'integ/diag10/gate_run.sh ~/.venvs/vio/bin/python integ/diag10/replay.py {} 2>&1 | grep -v Warn'
