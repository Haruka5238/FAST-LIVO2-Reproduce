#!/usr/bin/env bash

set -Eeuo pipefail

if [[ $# -lt 4 || $# -gt 6 ]]; then
  echo "Usage: $0 MODE BAG DURATION RESULT_ROOT [RATE] [COMPRESSED_IMAGE]" >&2
  exit 2
fi

mode="$1"
bag="$2"
duration="$3"
result_root="$4"
rate="${5:-1.0}"
compressed_image="${6:-false}"

current_ws="${FAST_LIVO_CURRENT_WS:-/home/fastlivo/codex_catkin_ws_20260710}"
baseline_ws="${FAST_LIVO_BASELINE_WS:-/home/fastlivo/codex_baseline_ws_20260710}"
project_dir="${current_ws}/src/FAST-LIVO2"
result_dir="${result_root}/${mode}"
stop_int_seconds="${FAST_LIVO_STOP_INT_SECONDS:-8}"
stop_term_seconds="${FAST_LIVO_STOP_TERM_SECONDS:-4}"
drain_seconds="${FAST_LIVO_DRAIN_SECONDS:-5}"

case "$mode" in
  stock_baseline|all_off|legacy_3x3|coupled_6x6|full) ;;
  *) echo "Unknown mode: $mode" >&2; exit 2 ;;
esac

if [[ ! -f "$bag" ]]; then
  echo "Bag not found: $bag" >&2
  exit 2
fi
if [[ ! "$duration" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] ||
   ! awk -v value="$duration" 'BEGIN { exit !(value > 0.0) }'; then
  echo "DURATION must be a positive finite decimal: $duration" >&2
  exit 2
fi
if [[ ! "$rate" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] ||
   ! awk -v value="$rate" 'BEGIN { exit !(value > 0.0) }'; then
  echo "RATE must be a positive finite decimal: $rate" >&2
  exit 2
fi
if [[ "$compressed_image" != "true" && "$compressed_image" != "false" ]]; then
  echo "COMPRESSED_IMAGE must be true or false" >&2
  exit 2
fi
for value in "$stop_int_seconds" "$stop_term_seconds" "$drain_seconds"; do
  if [[ ! "$value" =~ ^[0-9]+$ ]]; then
    echo "Process timeout values must be non-negative integers" >&2
    exit 2
  fi
done

if [[ -e "$result_dir" && ! -d "$result_dir" ]]; then
  echo "Result path exists and is not a directory: $result_dir" >&2
  exit 2
fi
if [[ -d "$result_dir" ]]; then
  shopt -s nullglob dotglob
  existing_entries=("$result_dir"/*)
  shopt -u nullglob dotglob
  if (( ${#existing_entries[@]} > 0 )); then
    echo "Result directory is not empty; refusing to mix runs: $result_dir" >&2
    exit 7
  fi
fi

mkdir -p "$result_dir/ros_home" "$result_dir/ros_log"
export ROS_HOME="$result_dir/ros_home"
export ROS_LOG_DIR="$result_dir/ros_log"
export ROS_MASTER_URI="http://127.0.0.1:11311"

stage="setup"
start_epoch="$(date +%s)"
player_exit="not_started"
node_exit="not_started"
record_exit="not_started"
republisher_exit="not_started"
roscore_exit="not_started"
player_timed_out="false"
node_timed_out="false"
record_timed_out="false"
republisher_alive_through_run="not_requested"
health_failure="none"
contract_version="3"
use_sim_time="true"
bag_realpath=""
bag_bytes=""
bag_sha256=""
source_config_sha256=""
camera_config_sha256=""
mapping_binary_sha256=""
runtime_bundle_sha256=""
parameters_sha256=""
player_pid=""
node_pid=""
record_pid=""
republish_pid=""
roscore_pid=""

declare -a child_pids=()
declare -A child_roles=()
declare -A child_reaped=()
declare -A child_codes=()
declare -A child_escalated=()

register_child()
{
  local role="$1"
  local pid="$2"
  child_pids+=("$pid")
  child_roles["$pid"]="$role"
  child_reaped["$pid"]="false"
  child_escalated["$pid"]="false"
}

child_running()
{
  local pid="$1"
  local state
  if ! kill -0 "$pid" 2>/dev/null; then
    return 1
  fi
  state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d '[:space:]' || true)"
  [[ -z "$state" || "${state:0:1}" != "Z" ]]
}

reap_child()
{
  local pid="$1"
  local rc
  if [[ "${child_reaped[$pid]:-false}" == "true" ]]; then
    return 0
  fi
  if wait "$pid"; then
    rc=0
  else
    rc=$?
  fi
  child_codes["$pid"]="$rc"
  child_reaped["$pid"]="true"
}

wait_bounded()
{
  local pid="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))
  while child_running "$pid"; do
    if (( SECONDS >= deadline )); then
      return 124
    fi
    sleep 0.2
  done
  reap_child "$pid"
  return 0
}

stop_child()
{
  local pid="$1"
  if [[ -z "$pid" || "${child_reaped[$pid]:-false}" == "true" ]]; then
    return 0
  fi
  if ! child_running "$pid"; then
    reap_child "$pid"
    return 0
  fi

  kill -INT -- "-$pid" 2>/dev/null || true
  if wait_bounded "$pid" "$stop_int_seconds"; then
    return 0
  fi
  child_escalated["$pid"]="true"
  kill -TERM -- "-$pid" 2>/dev/null || true
  if wait_bounded "$pid" "$stop_term_seconds"; then
    return 0
  fi
  kill -KILL -- "-$pid" 2>/dev/null || true
  wait_bounded "$pid" 3
}

refresh_child_statuses()
{
  if [[ -n "$player_pid" ]]; then player_exit="${child_codes[$player_pid]:-not_reaped}"; fi
  if [[ -n "$node_pid" ]]; then node_exit="${child_codes[$node_pid]:-not_reaped}"; fi
  if [[ -n "$record_pid" ]]; then record_exit="${child_codes[$record_pid]:-not_reaped}"; fi
  if [[ -n "$republish_pid" ]]; then republisher_exit="${child_codes[$republish_pid]:-not_reaped}"; fi
  if [[ -n "$roscore_pid" ]]; then roscore_exit="${child_codes[$roscore_pid]:-not_reaped}"; fi
}

cleanup_children()
{
  local index
  local cleanup_failed=0
  for ((index=${#child_pids[@]} - 1; index >= 0; index--)); do
    if ! stop_child "${child_pids[$index]}"; then
      cleanup_failed=1
    fi
  done
  refresh_child_statuses
  return "$cleanup_failed"
}

write_status()
{
  local script_rc="$1"
  local exit_stage="$2"
  local success="$3"
  local end_epoch="$4"
  local tmp="$result_dir/status.txt.tmp"
  printf '%s\n' \
    "mode=$mode" \
    "bag=$bag" \
    "duration=$duration" \
    "rate=$rate" \
    "compressed_image=$compressed_image" \
    "contract_version=$contract_version" \
    "use_sim_time=$use_sim_time" \
    "bag_realpath=$bag_realpath" \
    "bag_bytes=$bag_bytes" \
    "bag_sha256=$bag_sha256" \
    "source_config_sha256=$source_config_sha256" \
    "camera_config_sha256=$camera_config_sha256" \
    "mapping_binary_sha256=$mapping_binary_sha256" \
    "runtime_bundle_sha256=$runtime_bundle_sha256" \
    "parameters_sha256=$parameters_sha256" \
    "stage=$exit_stage" \
    "success=$success" \
    "script_exit=$script_rc" \
    "player_exit=$player_exit" \
    "node_exit=$node_exit" \
    "record_exit=$record_exit" \
    "republisher_exit=$republisher_exit" \
    "roscore_exit=$roscore_exit" \
    "player_timed_out=$player_timed_out" \
    "node_timed_out=$node_timed_out" \
    "record_timed_out=$record_timed_out" \
    "republisher_alive_through_run=$republisher_alive_through_run" \
    "health_failure=$health_failure" \
    "wall_seconds=$((end_epoch - start_epoch))" \
    >"$tmp"
  mv -f "$tmp" "$result_dir/status.txt"
}

on_exit()
{
  local rc=$?
  local exit_stage="$stage"
  local cleanup_ok="true"
  local success="false"
  local end_epoch
  trap - EXIT INT TERM
  set +e
  if ! cleanup_children; then
    cleanup_ok="false"
    if (( rc == 0 )); then rc=6; fi
  fi
  refresh_child_statuses
  if (( rc == 0 )) && [[ "$cleanup_ok" == "true" ]]; then success="true"; fi
  end_epoch="$(date +%s)"
  write_status "$rc" "$exit_stage" "$success" "$end_epoch"
  exit "$rc"
}

trap on_exit EXIT
trap 'stage="interrupted"; exit 130' INT
trap 'stage="terminated"; exit 143' TERM

# Catkin-generated setup files probe optional variables that may be unset.
# Keep errexit/pipefail active, but suspend nounset only while sourcing them.
set +u
source /opt/ros/noetic/setup.bash
if [[ "$mode" == "stock_baseline" ]]; then
  source "${baseline_ws}/devel/setup.bash"
  runtime_ws="$baseline_ws"
  mapping_binary="${baseline_ws}/devel/lib/fast_livo/fastlivo_mapping"
else
  source "${current_ws}/devel/setup.bash"
  runtime_ws="$current_ws"
  mapping_binary="${current_ws}/devel/lib/fast_livo/fastlivo_mapping"
fi
set -u

if [[ ! -x "$mapping_binary" ]]; then
  stage="mapping_binary_missing"
  echo "Mapping binary not found: $mapping_binary" >&2
  exit 2
fi

stage="input_contract"
bag_realpath="$(realpath -e -- "$bag")"
bag_bytes="$(stat -c '%s' -- "$bag_realpath")"
bag_sha256="$(sha256sum -- "$bag_realpath" | awk '{print $1}')"
source_config_sha256="$(sha256sum -- "$project_dir/config/avia.yaml" | awk '{print $1}')"
camera_config_sha256="$(sha256sum -- "$project_dir/config/camera_pinhole.yaml" | awk '{print $1}')"
mapping_binary_sha256="$(sha256sum -- "$mapping_binary" | awk '{print $1}')"
runtime_artifacts=(
  "$mapping_binary"
  "${runtime_ws}/devel/lib/liblaser_mapping.so"
  "${runtime_ws}/devel/lib/liblio.so"
  "${runtime_ws}/devel/lib/libvio.so"
  "${runtime_ws}/devel/lib/libpre.so"
  "${runtime_ws}/devel/lib/libimu_proc.so"
)
for artifact in "${runtime_artifacts[@]}"; do
  if [[ ! -f "$artifact" ]]; then
    stage="runtime_artifact_missing"
    echo "Runtime artifact not found: $artifact" >&2
    exit 2
  fi
done
runtime_manifest="$result_dir/runtime_manifest.txt"
: >"$runtime_manifest"
for artifact in "${runtime_artifacts[@]}"; do
  printf '%s %s\n' "$(basename -- "$artifact")" "$(sha256sum -- "$artifact" | awk '{print $1}')" \
    >>"$runtime_manifest"
done
runtime_bundle_sha256="$(sha256sum -- "$runtime_manifest" | awk '{print $1}')"

stage="ros_master_check"
if rostopic list >/dev/null 2>&1; then
  echo "A ROS master is already running at ${ROS_MASTER_URI}; refusing to mix experiments" >&2
  exit 3
fi

stage="roscore_start"
setsid roscore >"$result_dir/roscore.log" 2>&1 &
roscore_pid=$!
register_child roscore "$roscore_pid"
for _ in $(seq 1 100); do
  if rosparam list >/dev/null 2>&1; then break; fi
  if ! child_running "$roscore_pid"; then break; fi
  sleep 0.1
done
if ! rosparam list >/dev/null 2>&1; then
  stage="roscore_not_ready"
  echo "roscore did not become ready" >&2
  exit 4
fi

rosparam set /use_sim_time true

stage="parameter_load"
rosparam load "$project_dir/config/avia.yaml" /
rosparam load "$project_dir/config/camera_pinhole.yaml" /laserMapping
rosparam set /debug/verbose_runtime_log false
rosparam set /pcd_save/pcd_save_en false
rosparam set /pcd_save/colmap_output_en false
rosparam set /image_save/img_save_en false
rosparam set /publish/dense_map_en false
rosparam set /publish/pub_effect_point_en false
rosparam set /evo/pose_output_en false

adaptive_switches=(
  degeneracy_aware_en coupled_degeneracy_en normal_anisotropy_weight_en
  normal_balance_en incidence_weight_en robust_kernel_en scan_distortion_weight_en
  lidar_return_quality_en lidar_range_adaptive_en lidar_range_candidate_source_en
  map_write_gate_en dynamic_object_filter_en plane_quality_weight_en vio_quality_en
  vio_cov_fusion_en visual_lifecycle_en visual_view_weight_en cross_modal_gate_en
  temporal_degrade_en ref_patch_adaptive_en motion_degrade_en vibration_degrade_en
  preprocess_adaptive_en preprocess_intensity_quality_en lio_hard_outlier_reject_en
  map_write_recovery_en
)

declare -A expected_switches=()
default_switch_value="false"
if [[ "$mode" == "full" ]]; then default_switch_value="true"; fi
for key in "${adaptive_switches[@]}"; do
  expected_switches["$key"]="$default_switch_value"
done
if [[ "$mode" == "legacy_3x3" ]]; then
  expected_switches[degeneracy_aware_en]="true"
  expected_switches[coupled_degeneracy_en]="false"
elif [[ "$mode" == "coupled_6x6" ]]; then
  expected_switches[degeneracy_aware_en]="true"
  expected_switches[coupled_degeneracy_en]="true"
fi

stage="parameter_override"
for key in "${adaptive_switches[@]}"; do
  rosparam set "/adaptive/${key}" "${expected_switches[$key]}"
done
for key in "${adaptive_switches[@]}"; do
  actual="$(rosparam get "/adaptive/${key}")"
  actual="${actual,,}"
  if [[ "$actual" != "${expected_switches[$key]}" ]]; then
    stage="parameter_verification_failed"
    echo "Parameter verification failed for /adaptive/${key}: expected ${expected_switches[$key]}, got $actual" >&2
    exit 5
  fi
done
rosparam dump "$result_dir/parameters.yaml"
parameters_sha256="$(sha256sum -- "$result_dir/parameters.yaml" | awk '{print $1}')"

if [[ "$compressed_image" == "true" ]]; then
  stage="republisher_start"
  setsid rosrun image_transport republish compressed in:=/left_camera/image raw out:=/left_camera/image \
    >"$result_dir/republish.log" 2>&1 &
  republish_pid=$!
  register_child republisher "$republish_pid"
  sleep 1
  if ! child_running "$republish_pid"; then
    reap_child "$republish_pid"
    republisher_alive_through_run="false"
    stage="republisher_not_ready"
    echo "Compressed-image republisher exited before replay" >&2
    exit 5
  fi
  republisher_alive_through_run="true"
fi

stage="mapping_start"
if [[ "${FAST_LIVO_GDB:-false}" == "true" ]]; then
  setsid gdb --batch -ex run -ex "thread apply all bt" --args "$mapping_binary" \
    >"$result_dir/node.log" 2>&1 &
else
  setsid /usr/bin/time -v -o "$result_dir/node_time.txt" "$mapping_binary" \
    >"$result_dir/node.log" 2>&1 &
fi
node_pid=$!
register_child mapping "$node_pid"

node_ready="false"
for _ in $(seq 1 150); do
  if rosnode ping -c 1 /laserMapping >/dev/null 2>&1; then
    node_ready="true"
    break
  fi
  if ! child_running "$node_pid"; then break; fi
  sleep 0.1
done
if [[ "$node_ready" != "true" ]]; then
  if ! child_running "$node_pid"; then reap_child "$node_pid"; fi
  stage="mapping_not_ready"
  echo "Mapping node did not become ready" >&2
  exit 5
fi

stage="recorder_start"
setsid rosbag record --buffsize=128 --chunksize=256 -O "$result_dir/output.bag" \
  /aft_mapped_to_init /fast_livo2/lio_diag /fast_livo2/vio_diag \
  >"$result_dir/record.log" 2>&1 &
record_pid=$!
register_child recorder "$record_pid"
sleep 1
if ! child_running "$record_pid"; then
  reap_child "$record_pid"
  stage="recorder_not_ready"
  echo "rosbag recorder exited before replay" >&2
  exit 5
fi

stage="replay"
setsid /usr/bin/time -v -o "$result_dir/player_time.txt" \
  rosbag play --quiet --clock --rate "$rate" --duration "$duration" "$bag" \
  >"$result_dir/player.log" 2>&1 &
player_pid=$!
register_child player "$player_pid"

if [[ -n "${FAST_LIVO_PLAYER_TIMEOUT:-}" ]]; then
  player_timeout="${FAST_LIVO_PLAYER_TIMEOUT}"
  if [[ ! "$player_timeout" =~ ^[1-9][0-9]*$ ]]; then
    stage="invalid_player_timeout"
    echo "FAST_LIVO_PLAYER_TIMEOUT must be a positive integer" >&2
    exit 2
  fi
else
  player_timeout="$(awk -v d="$duration" -v r="$rate" 'BEGIN { value=d/r+60.0; if (value < 60.0) value=60.0; print int(value+0.999) }')"
fi

player_deadline=$((SECONDS + player_timeout))
while child_running "$player_pid"; do
  if ! child_running "$node_pid"; then
    reap_child "$node_pid"
    health_failure="mapping_exited_during_replay"
    stage="$health_failure"
    break
  fi
  if ! child_running "$record_pid"; then
    reap_child "$record_pid"
    health_failure="recorder_exited_during_replay"
    stage="$health_failure"
    break
  fi
  if [[ -n "$republish_pid" ]] && ! child_running "$republish_pid"; then
    reap_child "$republish_pid"
    republisher_alive_through_run="false"
    health_failure="republisher_exited_during_replay"
    stage="$health_failure"
    break
  fi
  if ! child_running "$roscore_pid"; then
    reap_child "$roscore_pid"
    health_failure="roscore_exited_during_replay"
    stage="$health_failure"
    break
  fi
  if (( SECONDS >= player_deadline )); then
    player_timed_out="true"
    health_failure="player_timeout"
    stage="$health_failure"
    break
  fi
  sleep 0.2
done

if child_running "$player_pid"; then
  stop_child "$player_pid" || true
else
  reap_child "$player_pid"
fi
refresh_child_statuses

if [[ "$health_failure" == "none" && "$player_exit" == "0" ]]; then
  stage="drain"
  for ((elapsed=0; elapsed < drain_seconds * 5; elapsed++)); do
    if ! child_running "$node_pid" || ! child_running "$record_pid"; then
      health_failure="processing_exited_during_drain"
      break
    fi
    if [[ -n "$republish_pid" ]] && ! child_running "$republish_pid"; then
      republisher_alive_through_run="false"
      health_failure="republisher_exited_during_drain"
      break
    fi
    sleep 0.2
  done
fi
if [[ -n "$republish_pid" ]] && ! child_running "$republish_pid"; then
  reap_child "$republish_pid"
  republisher_alive_through_run="false"
  if [[ "$health_failure" == "none" ]]; then
    health_failure="republisher_exited_before_shutdown"
  fi
fi

stage="mapping_stop"
if child_running "$node_pid"; then
  rosnode kill /laserMapping >/dev/null 2>&1 || true
fi
if ! wait_bounded "$node_pid" "$stop_int_seconds"; then
  node_timed_out="true"
  stop_child "$node_pid" || true
fi

stage="recorder_stop"
if ! stop_child "$record_pid"; then
  record_timed_out="true"
fi
if [[ "${child_escalated[$record_pid]:-false}" == "true" ]]; then
  record_timed_out="true"
fi
refresh_child_statuses
if [[ ! -s "$result_dir/output.bag" && "$health_failure" == "none" ]]; then
  health_failure="output_bag_missing_or_empty"
fi

if [[ "$health_failure" != "none" || "$player_timed_out" == "true" ||
      "$node_timed_out" == "true" || "$record_timed_out" == "true" ||
      "$player_exit" != "0" || "$node_exit" != "0" || "$record_exit" != "0" ]]; then
  stage="run_failed"
  exit 6
fi

stage="complete"
exit 0
