#!/usr/bin/env bash
# Build isolated installs from this checkout and the pinned EGO patches.
set -e -o pipefail
out="$(realpath "$1")"
ego_source="$(realpath "$2")"
px4_source="$(realpath "$3")"
secret_env="$(realpath "$4")"
project="$(cd "$(dirname "$0")/../.." && pwd)"
test -d "$out"
test -z "$(find "$out" -mindepth 1 -maxdepth 1 -print -quit)"
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH VIRTUAL_ENV CONDA_PREFIX LD_PRELOAD
export LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64:/usr/lib/x86_64-linux-gnu
source /opt/ros/noetic/setup.bash
git -C "$project" worktree add --detach "$out/UAV-navigation" HEAD
git -C "$ego_source" worktree add --detach "$out/ego-planner" bfda51284c8c1b476043255a8145ef925a3778a5
/usr/bin/python3 - "$project" "$out" <<'PY'
import hashlib,json,shutil,subprocess,sys
from pathlib import Path
b=Path(sys.argv[1]);d=Path(sys.argv[2])
files=subprocess.check_output(['git','-C',str(b),'ls-files','--modified','--others','--exclude-standard','-z']).decode().strip('\0').split('\0')
manifest={}
for name in files:
 if not (name.startswith(('ws/','tools/','external/','simulation/','tests/')) or name=='.gitignore'):continue
 p=b/name
 if not p.is_file() or p.name=='.env':continue
 dest=d/'UAV-navigation'/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
 manifest[name]=hashlib.sha256(p.read_bytes()).hexdigest()
(d/'source-overlay-sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
PY
cd "$out/ego-planner"
for patch in managed-integration.patch known-space-guard.patch dependency-manifest.patch depth-pixel.patch;do
 git apply --check "$out/UAV-navigation/external/ego-planner/$patch"
 git apply "$out/UAV-navigation/external/ego-planner/$patch"
done
packages="$(/usr/bin/python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as E
mapping={}
for p in Path('src').rglob('package.xml'):
 x=E.parse(p).getroot();mapping[x.findtext('name')]=x
selected=set();todo=['ego_planner']
while todo:
 n=todo.pop()
 if n in selected or n not in mapping:continue
 selected.add(n);x=mapping[n]
 for tag in ('depend','build_depend','build_export_depend','run_depend','exec_depend'):
  todo.extend(z.text.strip() for z in x.findall(tag) if z.text)
print(';'.join(sorted(selected)))
PY
)"
printf '%s\n' "$packages" > "$out/ego-selected-packages.txt"
catkin_make --build "$out/ego-build" install -j3 -l4 \
 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_DEVEL_PREFIX="$out/ego-devel" \
 -DCMAKE_INSTALL_PREFIX="$out/ego-install" -DCATKIN_WHITELIST_PACKAGES="$packages"
source "$out/ego-install/setup.bash"
cd "$out/UAV-navigation/ws"
catkin_make --build "$out/business-build" install -j3 -l4 \
 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_DEVEL_PREFIX="$out/business-devel" \
 -DCMAKE_INSTALL_PREFIX="$out/business-install"
{
 printf 'export UAV_RUNTIME_HOME=%q\n' "$out"
 printf 'export UAV_ROOT=%q\n' "$out"
 printf 'export UAV_PROJECT_DIR=%q\n' "$out/UAV-navigation"
 printf 'export UAV_EGO_SETUP=%q\n' "$out/ego-install/setup.bash"
 printf 'export UAV_BUSINESS_SETUP=%q\n' "$out/business-install/setup.bash"
 printf 'export UAV_EGO_INSTALL=%q\n' "$out/ego-install"
 printf 'export UAV_BUSINESS_INSTALL=%q\n' "$out/business-install"
 printf 'export PX4_AUTOPILOT_DIR=%q\n' "$px4_source"
 printf 'export UAV_SECRET_ENV=%q\n' "$secret_env"
 printf 'export UAV_GUI=1\n'
} > "$out/runtime.env"
chmod 600 "$out/runtime.env"
UAV_RUNTIME_CONFIG="$out/runtime.env" bash "$out/UAV-navigation/tools/deployment/uav_runtime.sh" check
