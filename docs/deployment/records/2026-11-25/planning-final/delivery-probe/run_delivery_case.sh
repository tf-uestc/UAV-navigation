#!/bin/bash
set -eu -o pipefail
label="${1:?unique experiment label}"
scenario="${2:?blocked or dynamic}"
case "$scenario" in blocked|dynamic) ;; *) exit 2 ;; esac
out="$UAV_ACCEPTANCE_DIR/flights/$label"
test ! -e "$out" || { echo "Existing flight ID"; exit 2; }
probe="$(cd "$(dirname "$0")" && pwd -P)"
run="$UAV_ACCEPTANCE_BUILD/UAV-navigation/tools/deployment/run_acceptance_case.sh"
runner="$(mktemp "$probe/run-case-XXXXXX.sh")"
trap 'rm -f -- "$runner"' EXIT
export UAV_READONLY_PLANNING_PROBE="$probe/planning_probe.py"
/usr/bin/python3 - "$run" "$runner" <<'PY'
from pathlib import Path
import sys
source, target = map(Path, sys.argv[1:])
data = source.read_text()
needle = '/usr/bin/python3 -u tools/deployment/validate_planning_sitl.py "$scenario"'
assert data.count(needle) == 1, 'Unknown validation runner'
target.write_text(data.replace(needle, '/usr/bin/python3 -u "$UAV_READONLY_PLANNING_PROBE" "$scenario"'))
PY
set +e
bash "$runner" "$label" "$scenario" 2.4 1.6 0 0
status=$?
set -e
test -s "$out/result/trace.json" || exit 2
sha256sum "$run" "$probe/planning_probe.py" "$probe/grade_delivery.py" > "$out/probe-sha256.txt"
/usr/bin/python3 "$probe/grade_delivery.py" "$out" | tee "$out/native-delivery.json"
exit "$status"
