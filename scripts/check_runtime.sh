#!/usr/bin/env bash
# Portable deployment preflight. Run after sourcing the target workspace.
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 unavailable. Source /opt/ros/humble/setup.bash." >&2
  exit 2
fi
if ! command -v ldd >/dev/null 2>&1; then
  echo "FAIL: ldd unavailable on this Linux host." >&2
  exit 2
fi

prefix="$(ros2 pkg prefix simple_loop_closure 2>/dev/null)" || {
  echo "FAIL: simple_loop_closure not found; source the target workspace install/setup.bash." >&2
  exit 2
}
executable="$prefix/lib/simple_loop_closure/simple_loop_closure_node"
if [[ ! -x "$executable" ]]; then
  echo "FAIL: executable missing: $executable" >&2
  exit 3
fi

echo "Machine: $(uname -m)"
echo "ROS_DISTRO: ${ROS_DISTRO:-unset}"
echo "Package prefix: $prefix"
echo "Executable: $executable"

for mode in lio livo; do
  yaml="$prefix/share/simple_loop_closure/config/cohemo_${mode}.yaml"
  if [[ ! -f "$yaml" ]]; then
    echo "FAIL: missing mode config: $yaml" >&2
    exit 3
  fi
  echo "Config $mode: OK"
done

deps="$(ldd "$executable" 2>&1)" || {
  printf '%s\n' "$deps" >&2
  echo "FAIL: dynamic-link dependency inspection failed." >&2
  exit 4
}
if grep -Eq '=>[[:space:]]+not found|not a dynamic executable' <<<"$deps"; then
  printf '%s\n' "$deps" >&2
  echo "FAIL: missing runtime shared libraries; do not launch SimpleLoopClosure." >&2
  exit 4
fi
echo "PASS: dynamic dependencies resolved on this machine."
echo "NOTE: this does not prove LiDAR, DDS, GTSAM runtime correctness, or UGV validation."
