#!/usr/bin/env bash
# Bootstrap only the dependencies used by the single-AUV launch, without sudo.
set -eo pipefail
BASELINE_PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -e "$BASELINE_PROJECT/.deps" ]]; then
    # rosidl/CMake file(STRINGS) cannot handle Chinese build paths on this system.
    BASELINE_CACHE="${BASELINE_DEPENDENCY_CACHE:-$HOME/.cache/baseline_robot/dependencies}"
    mkdir -p "$BASELINE_CACHE"
    ln -s "$BASELINE_CACHE" "$BASELINE_PROJECT/.deps"
fi
BASELINE_CACHE="$(realpath "$BASELINE_PROJECT/.deps")"
mkdir -p "$BASELINE_CACHE/archives" "$BASELINE_CACHE/src"
touch "$BASELINE_CACHE/COLCON_IGNORE"

python3 - "$BASELINE_PROJECT" "$BASELINE_CACHE" <<'PY'
import json
from pathlib import Path
import subprocess
import sys
import yaml

project, cache = map(Path, sys.argv[1:])
if not str(cache).isascii():
    sys.exit(f'Dependency cache must have an ASCII-only path: {cache}')
run = lambda *args, **kwargs: subprocess.run(args, check=True, **kwargs)
packages = json.loads((project / 'scripts/debian_dependencies.json').read_text())
available = {}
for deb in (cache / 'archives').glob('*.deb'):
    fields = subprocess.check_output(
        ['dpkg-deb', '-f', str(deb), 'Package', 'Version'], text=True).splitlines()
    available[tuple(line.split(': ', 1)[1] for line in fields)] = deb
for name, version in packages.items():
    if (name, version) not in available:
        run('apt-get', 'download', f'{name}={version}', cwd=cache / 'archives')
for deb in (cache / 'archives').glob('*.deb'):
    run('dpkg-deb', '-x', str(deb), str(cache / 'root'))

repos = yaml.safe_load((project / 'scripts/dependencies.repos').read_text())
for relative, spec in repos['repositories'].items():
    destination = cache / relative
    if not destination.exists():
        run('git', 'clone', spec['url'], str(destination))
        run('git', '-C', str(destination), 'checkout', '--detach', spec['version'])
        run('git', '-C', str(destination), 'submodule', 'update', '--init', '--recursive')
    # Existing source trees are never reset or overwritten.
for patch in sorted((project / 'scripts/dependency_patches').glob('*.patch')):
    destination = cache / 'src' / patch.stem
    applied = subprocess.run(
        ['git', '-C', str(destination), 'apply', '--reverse', '--check', str(patch)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
    if not applied:
        run('git', '-C', str(destination), 'apply', '--check', str(patch))
        run('git', '-C', str(destination), 'apply', str(patch))
PY

BASELINE_SKIP_PROJECT=1 source "$BASELINE_PROJECT/scripts/env.sh"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
export MAKEFLAGS="${MAKEFLAGS:--j4 -l8}"
colcon --log-base "$BASELINE_CACHE/log" build \
    --base-paths "$BASELINE_CACHE/src" \
    --build-base "$BASELINE_CACHE/build" --install-base "$BASELINE_CACHE/install" \
    --symlink-install --parallel-workers 2 \
    --packages-select acomms_msgs stonefish_ros2 world_of_stonefish mvp_msgs \
    mvp_control behavior_interface mvp_helm bhv_teleoperation bhv_direct_control \
    bhv_path_following mvp_localization_utilities \
    --cmake-args -DBUILD_TESTING=OFF -DOSQP-CPP_BUILD_TESTS=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    "-DCMAKE_MODULE_PATH=$BASELINE_CACHE/root/usr/share/cmake/geographiclib" \
    "-DCMAKE_INCLUDE_PATH=$BASELINE_CACHE/root/usr/include" \
    "-DCMAKE_LIBRARY_PATH=$BASELINE_CACHE/root/usr/lib/x86_64-linux-gnu" \
    "-DFETCHCONTENT_SOURCE_DIR_OSQP-CPP=$BASELINE_CACHE/vendor/osqp-cpp" \
    "-DFETCHCONTENT_SOURCE_DIR_OSQP=$BASELINE_CACHE/vendor/osqp" \
    "-DFETCHCONTENT_SOURCE_DIR_ABSEIL-CPP=$BASELINE_CACHE/vendor/abseil-cpp"
