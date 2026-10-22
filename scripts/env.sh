#!/usr/bin/env bash
# Source this file in each terminal; optional dependencies live in this project.
BASELINE_ROBOT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
# VS Code's Snap GTK modules are incompatible with system Qt/ROS executables.
if [[ "${GTK_PATH:-}" == /snap/* ]]; then
    unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
fi
if [[ "${GIO_MODULE_DIR:-}" == */snap/* ]]; then
    unset GIO_MODULE_DIR
fi
BASELINE_DEPS_ROOT="$BASELINE_ROBOT_ROOT/.deps"
BASELINE_ROS_DEPS="$BASELINE_DEPS_ROOT/root/opt/ros/jazzy"
BASELINE_NATIVE_DEPS="$BASELINE_DEPS_ROOT/root/usr/lib/x86_64-linux-gnu"
if [[ -d "$BASELINE_ROS_DEPS/share/ament_index" ]]; then
    export AMENT_PREFIX_PATH="$BASELINE_ROS_DEPS${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export CMAKE_PREFIX_PATH="$BASELINE_ROS_DEPS:$BASELINE_DEPS_ROOT/root/usr${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$BASELINE_ROS_DEPS/lib:$BASELINE_NATIVE_DEPS:/usr/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export PATH="$BASELINE_ROS_DEPS/bin${PATH:+:$PATH}"
    for BASELINE_PYTHON_DEPS in "$BASELINE_ROS_DEPS"/lib/python*/site-packages "$BASELINE_DEPS_ROOT/root/usr/lib/python3/dist-packages"; do
        if [[ -d "$BASELINE_PYTHON_DEPS" ]]; then
            export PYTHONPATH="$BASELINE_PYTHON_DEPS${PYTHONPATH:+:$PYTHONPATH}"
        fi
    done
fi
if [[ -f "$BASELINE_DEPS_ROOT/install/local_setup.bash" ]]; then
    source "$BASELINE_DEPS_ROOT/install/local_setup.bash"
fi
if [[ "${BASELINE_SKIP_PROJECT:-0}" != 1 && -f "$BASELINE_ROBOT_ROOT/install/local_setup.bash" ]]; then
    source "$BASELINE_ROBOT_ROOT/install/local_setup.bash"
fi
