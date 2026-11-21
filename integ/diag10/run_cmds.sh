#!/bin/bash
# Run the commands of a file (one per line, run from the repository root), N at a time (default 2), each through
# gate_run.sh. Usage: run_cmds.sh cmds.txt [pid to wait for first, or 0] [N]
cd "$(dirname "$0")/../.."
[ -n "$2" ] && [ "$2" != 0 ] && while kill -0 "$2" 2>/dev/null; do sleep 10; done
N=${3:-2}
cat "$1" | xargs -P "$N" -I{} bash -c 'integ/diag10/gate_run.sh {} 2>&1 | grep -v Warn'
