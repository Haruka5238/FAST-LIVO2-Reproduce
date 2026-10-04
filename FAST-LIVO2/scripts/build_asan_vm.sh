#!/usr/bin/env bash

set -Eeuo pipefail

workspace="${1:-/home/fastlivo/codex_asan_ws_20260710}"
if [[ ! -d "$workspace/src" ]]; then
  echo "ASan catkin workspace not found: $workspace" >&2
  exit 2
fi

set +u
source /opt/ros/noetic/setup.bash
source /home/fastlivo/catkin_ws/devel/setup.bash
set -u
cd "$workspace"

catkin_make \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-sanitize-recover=address -fno-omit-frame-pointer -g" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address" \
  -DCMAKE_SHARED_LINKER_FLAGS="-fsanitize=address" \
  -j1
