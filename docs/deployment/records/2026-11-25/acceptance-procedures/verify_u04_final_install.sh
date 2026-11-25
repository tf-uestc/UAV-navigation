#!/usr/bin/env bash
set -e -o pipefail

experiment=/root/autodl-tmp/uav/validation/goal-20261125
previous="$experiment/isolated/u04-final03"
out="$experiment/isolated/u04-final04"
test -s "$previous/map_case.sh"
commit=$(git -C "$experiment/install04/UAV-navigation" rev-parse HEAD)
test "$commit" = 6b58825f8e715d6ac902303b812976ed20693e6e
mkdir "$out"
mkdir "$out/guards"
printf '%s\n' "$commit" > "$out/commit.txt"
sed -e s,install03,install04,g -e s,u04-final03,u04-final04,g -e s,11318,11319,g "$previous/map_case.sh" > "$out/map_case.sh"
for yaw in 0 90 180 -90; do
    bash "$out/map_case.sh" known-start "$yaw" > "$out/known-start-$yaw.console" 2>&1
done
bash "$out/map_case.sh" readiness 90 > "$out/readiness-90.console" 2>&1
/usr/bin/python3 - "$out/guards" <<'PY'
import json
import pathlib
import sys

guards = pathlib.Path(sys.argv[1])
files = sorted(guards.glob('*.json'))
if len(files) != 5:
    raise ValueError('Expected four headings and a readiness replay')
for file in files:
    data = json.loads(file.read_text())
    print(file.name, data.get('passed'), len(data['checks']))
    if not data.get('passed') or not all(check['passed'] for check in data['checks']):
        sys.exit(1)
PY
