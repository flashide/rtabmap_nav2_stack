#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-3}"

DEMO_BAG_DEFAULT_FILE="$ROOT_DIR/bags/demo_mapping/demo_mapping.db3"
DEMO_BAG_DEFAULT_DIR1="$ROOT_DIR/bags/demo_mapping_bag"
DEMO_BAG_DEFAULT_DIR2="$ROOT_DIR/bags/demo_mapping"

usage() {
  cat <<USAGE
Usage:
  $(basename "$0") demo-up [--gui]
  $(basename "$0") demo-play [bag_path] [--no-loop]
  $(basename "$0") real-up
  $(basename "$0") status
  $(basename "$0") verify-lidar-only

Examples:
  bash scripts/check_mapping.sh demo-up
  bash scripts/check_mapping.sh demo-play
  bash scripts/check_mapping.sh demo-play ~/rtabmap_nav2_stack/bags/demo_mapping/demo_mapping.db3
  bash scripts/check_mapping.sh real-up
  bash scripts/check_mapping.sh status
  bash scripts/check_mapping.sh verify-lidar-only
USAGE
}

source_env() {
  set +u
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  source "$ROOT_DIR/install/setup.bash"
  set -u
}

has_topic_data() {
  local topic="$1"
  timeout "${WAIT_TIMEOUT_SEC}s" ros2 topic echo --once "$topic" >/dev/null 2>&1
}

topic_type() {
  local topic="$1"
  ros2 topic type "$topic" 2>/dev/null | tr -d '\r' || true
}

node_info() {
  local node="$1"
  ros2 node info "$node" 2>/dev/null | tr -d '\r' || true
}

print_status() {
  echo "[INFO] Root       : $ROOT_DIR"
  echo "[INFO] ROS distro : $ROS_DISTRO_NAME"

  echo
  echo "[STEP] Nodes (rtabmap/livox/mock)"
  ros2 node list 2>/dev/null | grep -E 'rtabmap|livox|mock' || true

  echo
  echo "[STEP] Topics (map/info/clock/lidar/odom/scan)"
  ros2 topic list -t 2>/dev/null | grep -Ei '(^/map$|/info$|/clock$|lidar|point|scan|odom|rtabmap)' || true

  echo
  echo "[STEP] Quick topic checks"
  local t
  for t in /clock /map /info /jn0/base_scan /sensors/lidar/points_deskewed /odometry/local; do
    local ty
    ty="$(topic_type "$t")"
    if [[ -z "$ty" ]]; then
      echo "[MISS] $t"
      continue
    fi
    if has_topic_data "$t"; then
      echo "[OK]   $t ($ty)"
    else
      echo "[WAIT] $t ($ty)"
    fi
  done
}

run_verify_lidar_only() {
  source_env

  local namespace="${NAMESPACE:-rtabmap}"
  local slam_node="/${namespace}/rtabmap"
  local odom_node="/${namespace}/icp_odometry"
  local lidar_topic="${LIDAR_TOPIC:-/sensors/lidar/points_deskewed}"
  local odom_topic="${ODOM_TOPIC:-/odometry/local}"
  local map_topic="${MAP_TOPIC:-/map}"
  local slam_info
  local odom_info

  echo "[STEP] Verifying lidar-only mapping"
  echo "[INFO] slam_node=$slam_node"
  echo "[INFO] odom_node=$odom_node"
  echo "[INFO] lidar_topic=$lidar_topic"
  echo "[INFO] odom_topic=$odom_topic"
  echo "[INFO] map_topic=$map_topic"

  slam_info="$(node_info "$slam_node")"
  if [[ -z "$slam_info" ]]; then
    echo "[ERROR] RTAB-Map SLAM node not found: $slam_node" >&2
    echo "[HINT] Start mapping first: bash scripts/check_mapping.sh real-up" >&2
    return 2
  fi

  if ! grep -Fq "$lidar_topic" <<< "$slam_info"; then
    echo "[ERROR] SLAM node is not subscribing to lidar topic: $lidar_topic" >&2
    echo "[HINT] Check LIDAR_TOPIC or current RTAB-Map launch arguments." >&2
    return 3
  fi

  if ! grep -Fq "$odom_topic" <<< "$slam_info"; then
    echo "[WARN] SLAM node info does not show odom topic: $odom_topic"
    echo "[HINT] This can happen if odometry is provided internally by icp_odometry."
  fi

  if grep -Eiq 'rgbd_image|camera_info|image_rect|image_raw|depth/image|left/image|right/image|/rgb\b|/depth\b|/stereo\b' <<< "$slam_info"; then
    echo "[ERROR] SLAM node still subscribes to image-related topics." >&2
    echo "$slam_info"
    return 4
  fi

  odom_info="$(node_info "$odom_node")"
  if [[ -n "$odom_info" ]]; then
    if grep -Eiq 'rgbd_image|camera_info|image_rect|image_raw|depth/image|left/image|right/image|/rgb\b|/depth\b|/stereo\b' <<< "$odom_info"; then
      echo "[ERROR] ICP odometry node still subscribes to image-related topics." >&2
      echo "$odom_info"
      return 5
    fi
    if ! grep -Fq "$lidar_topic" <<< "$odom_info"; then
      echo "[WARN] ICP odometry node info does not show lidar topic: $lidar_topic"
    fi
  else
    echo "[INFO] ICP odometry node not running: $odom_node"
    echo "[INFO] This is acceptable if external odometry is being used."
  fi

  if [[ -z "$(topic_type "$map_topic")" ]]; then
    echo "[WARN] Map topic not found yet: $map_topic"
  elif has_topic_data "$map_topic"; then
    echo "[OK]   map topic has data: $map_topic"
  else
    echo "[WAIT] map topic exists but no data yet: $map_topic"
  fi

  if [[ -z "$(topic_type "$lidar_topic")" ]]; then
    echo "[ERROR] Lidar topic not found: $lidar_topic" >&2
    return 6
  elif has_topic_data "$lidar_topic"; then
    echo "[OK]   lidar topic has data: $lidar_topic"
  else
    echo "[ERROR] Lidar topic exists but no data yet: $lidar_topic" >&2
    return 7
  fi

  echo "[OK] Lidar-only mapping verification passed."
  echo "[OK] RTAB-Map is using point cloud input without RGB/Depth fusion."
}

resolve_demo_bag_path() {
  if [[ -n "${1:-}" ]]; then
    echo "$1"
    return 0
  fi

  if [[ -f "$DEMO_BAG_DEFAULT_FILE" ]]; then
    echo "$DEMO_BAG_DEFAULT_FILE"
    return 0
  fi
  if [[ -d "$DEMO_BAG_DEFAULT_DIR1" ]]; then
    echo "$DEMO_BAG_DEFAULT_DIR1"
    return 0
  fi
  if [[ -d "$DEMO_BAG_DEFAULT_DIR2" ]]; then
    echo "$DEMO_BAG_DEFAULT_DIR2"
    return 0
  fi

  echo "$DEMO_BAG_DEFAULT_FILE"
}

run_demo_up() {
  local gui="false"
  if [[ "${1:-}" == "--gui" ]]; then
    gui="true"
  fi

  source_env
  echo "[STEP] Starting demo mapping launch"
  echo "[INFO] GUI=$gui"
  ros2 launch rtabmap_demos robot_mapping_demo.launch.py rviz:="$gui" rtabmap_viz:="$gui"
}

run_demo_play() {
  local bag_path_arg="${1:-}"
  local loop="true"
  if [[ "${2:-}" == "--no-loop" || "${1:-}" == "--no-loop" ]]; then
    loop="false"
    [[ "${1:-}" == "--no-loop" ]] && bag_path_arg=""
  fi

  source_env

  local bag_path
  bag_path="$(resolve_demo_bag_path "$bag_path_arg")"

  if [[ -d "$bag_path" ]]; then
    echo "[STEP] Playing rosbag directory"
    echo "[INFO] path=$bag_path"
    ros2 bag reindex "$bag_path" >/dev/null 2>&1 || true
    ros2 bag info "$bag_path" || true
    if [[ "$loop" == "true" ]]; then
      ros2 bag play "$bag_path" --clock -l
    else
      ros2 bag play "$bag_path" --clock
    fi
    return 0
  fi

  if [[ -f "$bag_path" ]]; then
    echo "[STEP] Playing rosbag sqlite file"
    echo "[INFO] path=$bag_path"
    ros2 bag info -s sqlite3 "$bag_path" || true
    if [[ "$loop" == "true" ]]; then
      ros2 bag play -s sqlite3 "$bag_path" --clock -l
    else
      ros2 bag play -s sqlite3 "$bag_path" --clock
    fi
    return 0
  fi

  echo "[ERROR] Bag path not found: $bag_path" >&2
  echo "[HINT] Provide path manually, e.g.:" >&2
  echo "       bash scripts/check_mapping.sh demo-play ~/rtabmap_nav2_stack/bags/demo_mapping/demo_mapping.db3" >&2
  return 2
}

run_real_up() {
  source_env
  echo "[STEP] Starting real mapping stack"
  exec bash "$ROOT_DIR/scripts/start_all_mapping.sh"
}

main() {
  local cmd="${1:-}"
  case "$cmd" in
    demo-up)
      shift
      run_demo_up "${1:-}"
      ;;
    demo-play)
      shift
      run_demo_play "${1:-}" "${2:-}"
      ;;
    real-up)
      run_real_up
      ;;
    status)
      source_env
      print_status
      ;;
    verify-lidar-only)
      run_verify_lidar_only
      ;;
    -h|--help|help|"")
      usage
      ;;
    *)
      echo "[ERROR] Unknown command: $cmd" >&2
      usage
      return 1
      ;;
  esac
}

main "$@"
