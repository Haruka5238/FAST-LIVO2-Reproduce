#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${FASTLIVO_WS:-/home/fastlivo/codex_rc_ws_20260712}"
BAG_PATH="${1:-/home/fastlivo/lab_mid360_12s_uncompressed.bag}"
OUTPUT_DIR="${2:-/home/fastlivo/codex_r1_smoke_20260714/run}"
PLAY_RATE="${3:-1.0}"
IMG_EN="${4:-1}"
HEALTH_CONFIG="${WORKSPACE}/src/fast_livo2_health_monitor/config/health_monitor.yaml"
DRAIN_TIMEOUT_SECONDS="${FASTLIVO_DRAIN_TIMEOUT_SECONDS:-300}"

if [[ ! -r "${BAG_PATH}" ]]; then
  echo "bag is not readable: ${BAG_PATH}" >&2
  exit 2
fi
if [[ ! -r "${WORKSPACE}/devel/setup.bash" ]]; then
  echo "workspace is not built: ${WORKSPACE}" >&2
  exit 2
fi
if [[ ! -r "${HEALTH_CONFIG}" ]]; then
  echo "health monitor config is missing: ${HEALTH_CONFIG}" >&2
  exit 2
fi
if [[ "${IMG_EN}" != "0" && "${IMG_EN}" != "1" ]]; then
  echo "img_en must be 0 (LIO) or 1 (LIVO): ${IMG_EN}" >&2
  exit 2
fi

mkdir -p "${OUTPUT_DIR}" "${OUTPUT_DIR}/ros_home" "${OUTPUT_DIR}/ros_log"
export ROS_HOME="${OUTPUT_DIR}/ros_home"
export ROS_LOG_DIR="${OUTPUT_DIR}/ros_log"

source /opt/ros/noetic/setup.bash
source "${WORKSPACE}/devel/setup.bash"
set -u

EXPECTED_LIDAR_MESSAGES="$(rosbag info "${BAG_PATH}" 2>/dev/null |
  awk '$1 == "/livox/lidar" && $3 == "msgs" {print $2; exit}')"

declare -a CHILD_PIDS=()

start_child()
{
  "$@" &
  CHILD_PIDS+=("$!")
}

cleanup()
{
  local index
  local pid
  set +e
  for ((index=${#CHILD_PIDS[@]} - 1; index >= 0; --index)); do
    pid="${CHILD_PIDS[index]}"
    if kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null
    fi
  done
  sleep 1
  for ((index=${#CHILD_PIDS[@]} - 1; index >= 0; --index)); do
    pid="${CHILD_PIDS[index]}"
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null
    fi
  done
  for ((index=${#CHILD_PIDS[@]} - 1; index >= 0; --index)); do
    pid="${CHILD_PIDS[index]}"
    wait "${pid}" 2>/dev/null
  done
}
trap cleanup EXIT INT TERM

roscore >"${OUTPUT_DIR}/roscore.log" 2>&1 &
CHILD_PIDS+=("$!")
MASTER_PID="${CHILD_PIDS[-1]}"
for _ in $(seq 1 100); do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done
if ! kill -0 "${MASTER_PID}" 2>/dev/null || ! rosparam list >/dev/null 2>&1; then
  echo "ROS master failed to start" >&2
  exit 3
fi

rosparam set /use_sim_time true
rosparam load "${HEALTH_CONFIG}" /fast_livo2_health_monitor
rosparam set /fast_livo2_health_monitor/csv_path "${OUTPUT_DIR}/health.csv"

roslaunch fast_livo mapping_mid360.launch rviz:=false img_en:="${IMG_EN}" \
  >"${OUTPUT_DIR}/mapping.log" 2>&1 &
CHILD_PIDS+=("$!")
MAPPING_PID="${CHILD_PIDS[-1]}"

rosrun fast_livo2_health_monitor health_monitor_node \
  __name:=fast_livo2_health_monitor \
  >"${OUTPUT_DIR}/health_monitor.log" 2>&1 &
CHILD_PIDS+=("$!")
HEALTH_PID="${CHILD_PIDS[-1]}"

for _ in $(seq 1 200); do
  nodes="$(rosnode list 2>/dev/null || true)"
  if grep -qx '/laserMapping' <<<"${nodes}" &&
     grep -qx '/fast_livo2_health_monitor' <<<"${nodes}"; then
    break
  fi
  sleep 0.1
done
if ! kill -0 "${MAPPING_PID}" 2>/dev/null ||
   ! kill -0 "${HEALTH_PID}" 2>/dev/null; then
  echo "mapping or health monitor failed during startup" >&2
  exit 4
fi

MAPPING_NODE_PID="$(rosnode info /laserMapping 2>/dev/null | sed -n 's/^Pid: //p' | head -n 1)"
if [[ ! "${MAPPING_NODE_PID}" =~ ^[0-9]+$ ]] ||
   ! kill -0 "${MAPPING_NODE_PID}" 2>/dev/null; then
  echo "cannot resolve the laserMapping process PID" >&2
  exit 4
fi

(
  echo "wall_time_s,elapsed_s,cpu_percent,rss_kib,vsz_kib"
  while kill -0 "${MAPPING_NODE_PID}" 2>/dev/null; do
    wall_time="$(date +%s.%N)"
    ps -p "${MAPPING_NODE_PID}" -o etimes=,%cpu=,rss=,vsz= 2>/dev/null |
      awk -v wall="${wall_time}" 'NF == 4 {print wall "," $1 "," $2 "," $3 "," $4}'
    sleep 1
  done
) >"${OUTPUT_DIR}/mapping_resources.csv" 2>"${OUTPUT_DIR}/mapping_resources.stderr" &
CHILD_PIDS+=("$!")

rostopic echo -p /fast_livo2/lio_diag >"${OUTPUT_DIR}/lio_diag.csv" 2>"${OUTPUT_DIR}/lio_diag.stderr" &
CHILD_PIDS+=("$!")
rostopic echo -p /fast_livo2/vio_diag >"${OUTPUT_DIR}/vio_diag.csv" 2>"${OUTPUT_DIR}/vio_diag.stderr" &
CHILD_PIDS+=("$!")
rostopic echo /fast_livo2_health >"${OUTPUT_DIR}/health_status.log" 2>"${OUTPUT_DIR}/health_status.stderr" &
CHILD_PIDS+=("$!")
rosbag record -O "${OUTPUT_DIR}/telemetry.bag" \
  /aft_mapped_to_init \
  /fast_livo2/lio_diag \
  /fast_livo2/vio_diag \
  /fast_livo2_health \
  /fast_livo2_health/advice \
  /fast_livo2_health/diagnostics \
  >"${OUTPUT_DIR}/record.log" 2>&1 &
CHILD_PIDS+=("$!")

sleep 2
set +e
rosbag play --clock --rate "${PLAY_RATE}" "${BAG_PATH}" \
  >"${OUTPUT_DIR}/play.log" 2>&1
PLAY_STATUS=$?
set -e

DRAIN_STATUS="not_required"
DRAIN_LIO_ROWS=0
if [[ "${IMG_EN}" == "0" && "${EXPECTED_LIDAR_MESSAGES}" =~ ^[0-9]+$ ]]; then
  # At accelerated replay, roscpp can have already accepted all messages while
  # the estimator still owns a bounded internal backlog. Do not kill a healthy
  # mapper three seconds after the publisher exits; wait for diagnostic coverage.
  DRAIN_STATUS="complete"
  DRAIN_TARGET=$((EXPECTED_LIDAR_MESSAGES > 10 ? EXPECTED_LIDAR_MESSAGES - 10 : 1))
  DRAIN_DEADLINE=$(( $(date +%s) + DRAIN_TIMEOUT_SECONDS ))
  while kill -0 "${MAPPING_PID}" 2>/dev/null; do
    DRAIN_LIO_ROWS="$(awk 'END {print (NR > 0 ? NR - 1 : 0)}' "${OUTPUT_DIR}/lio_diag.csv" 2>/dev/null || echo 0)"
    if [[ "${DRAIN_LIO_ROWS}" =~ ^[0-9]+$ ]] && ((DRAIN_LIO_ROWS >= DRAIN_TARGET)); then
      break
    fi
    if (( $(date +%s) >= DRAIN_DEADLINE )); then
      DRAIN_STATUS="timeout"
      break
    fi
    sleep 1
  done
fi
sleep 3

rosnode list >"${OUTPUT_DIR}/nodes_after_play.txt" 2>&1 || true
rostopic list -v >"${OUTPUT_DIR}/topics_after_play.txt" 2>&1 || true
ps -o pid,ppid,stat,etime,%cpu,rss,cmd -p "${MAPPING_NODE_PID}","${HEALTH_PID}" \
  >"${OUTPUT_DIR}/processes_after_play.txt" 2>&1 || true

{
  echo "workspace=${WORKSPACE}"
  echo "fast_livo=$(rospack find fast_livo)"
  echo "health_monitor=$(rospack find fast_livo2_health_monitor)"
  echo "bag=${BAG_PATH}"
  echo "play_rate=${PLAY_RATE}"
  echo "img_en=${IMG_EN}"
  echo "play_status=${PLAY_STATUS}"
  echo "mapping_node_pid=${MAPPING_NODE_PID}"
  echo "expected_lidar_messages=${EXPECTED_LIDAR_MESSAGES:-unknown}"
  echo "drain_status=${DRAIN_STATUS}"
  echo "drain_lio_rows=${DRAIN_LIO_ROWS}"
  echo "mapping_alive=$(kill -0 "${MAPPING_PID}" 2>/dev/null && echo 1 || echo 0)"
  echo "health_alive=$(kill -0 "${HEALTH_PID}" 2>/dev/null && echo 1 || echo 0)"
} >"${OUTPUT_DIR}/run_summary.txt"

exit "${PLAY_STATUS}"
