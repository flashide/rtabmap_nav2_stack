#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
JOBS="${JOBS:-8}"
WORKERS="${WORKERS:-4}"
HEAVY_JOBS="${HEAVY_JOBS:-1}"
CLEAN_BUILD="${CLEAN_BUILD:-1}"
INSTALL_DEPS="${INSTALL_DEPS:-0}"
BUILD_TYPE="${BUILD_TYPE:-Release}"

if [[ $# -ge 1 ]]; then
  JOBS="$1"
  WORKERS="$JOBS"
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  echo "[ERROR] ROS distro not found: /opt/ros/${ROS_DISTRO_NAME}/setup.bash" >&2
  exit 1
fi

detect_opencv_dir() {
  local candidates=()
  local pc_dir=""

  if [[ -n "${RTABMAP_OPENCV_DIR:-}" ]]; then
    candidates+=("${RTABMAP_OPENCV_DIR}")
  fi
  if [[ -n "${OpenCV_DIR:-}" ]]; then
    candidates+=("${OpenCV_DIR}")
  fi

  if command -v pkg-config >/dev/null 2>&1; then
    pc_dir="$(pkg-config --variable=pcfiledir opencv4 2>/dev/null || true)"
    if [[ -n "$pc_dir" ]]; then
      candidates+=("${pc_dir%/pkgconfig}/cmake/opencv4")
    fi
  fi

  candidates+=(
    "/usr/lib/aarch64-linux-gnu/cmake/opencv4"
    "/usr/lib/x86_64-linux-gnu/cmake/opencv4"
    "/usr/local/lib/cmake/opencv4"
    "/usr/local/lib64/cmake/opencv4"
    "/usr/lib/cmake/opencv4"
  )

  local d
  for d in "${candidates[@]}"; do
    if [[ -n "$d" && -f "$d/OpenCVConfig.cmake" ]]; then
      echo "$d"
      return 0
    fi
  done

  local found
  found="$(find /usr /usr/local -type f -path '*/cmake/opencv4/OpenCVConfig.cmake' 2>/dev/null | head -n 1 || true)"
  if [[ -n "$found" ]]; then
    dirname "$found"
    return 0
  fi

  return 1
}

cd "$ROOT_DIR"

echo "[INFO] Root        : $ROOT_DIR"
echo "[INFO] ROS distro  : $ROS_DISTRO_NAME"
echo "[INFO] Jobs        : $JOBS"
echo "[INFO] Workers     : $WORKERS"
echo "[INFO] Heavy jobs  : $HEAVY_JOBS"
echo "[INFO] Build type  : $BUILD_TYPE"

# ROS setup scripts may reference unset vars when nounset is enabled.
set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

# Avoid stale pkg-config overrides from previous sessions.
unset PKG_CONFIG_PATH || true
unset PKG_CONFIG_LIBDIR || true

export RTABMAP_OPENCV_DIR="$(detect_opencv_dir || true)"
if [[ -z "$RTABMAP_OPENCV_DIR" ]]; then
  echo "[ERROR] OpenCVConfig.cmake not found. Set RTABMAP_OPENCV_DIR manually." >&2
  exit 11
fi
export OpenCV_DIR="$RTABMAP_OPENCV_DIR"
echo "[INFO] OpenCV_DIR   : $OpenCV_DIR"

if [[ "$INSTALL_DEPS" == "1" ]]; then
  echo "[STEP] Installing dependencies with rosdep"
  rosdep update
  rosdep install --from-paths src third_party/rtabmap-0.23.4 --ignore-src -r -y
fi

if [[ "$CLEAN_BUILD" == "1" ]]; then
  echo "[STEP] Cleaning build/install/log and third_party RTABMap cache"
  rm -rf build install log
  rm -rf third_party/rtabmap-0.23.4/build_local third_party/rtabmap-0.23.4/install
fi

echo "[STEP] Building RTABMap 0.23.4 (third_party)"
JOBS="$JOBS" RTABMAP_OPENCV_DIR="$RTABMAP_OPENCV_DIR" bash "$ROOT_DIR/scripts/build_rtabmap_0234.sh"

# Make local RTABMap discoverable for all downstream find_package(RTABMap 0.23.4)
source "$ROOT_DIR/scripts/use_rtabmap_0234_env.sh"

if [[ -z "${RTABMap_DIR:-}" || ! -f "${RTABMap_DIR}/RTABMapConfig.cmake" ]]; then
  echo "[ERROR] Invalid RTABMap_DIR: ${RTABMap_DIR:-<empty>}" >&2
  exit 2
fi
RTABMAP_CONFIG_VERSION_FILE="${RTABMap_DIR}/RTABMapConfigVersion.cmake"
if [[ ! -f "$RTABMAP_CONFIG_VERSION_FILE" ]]; then
  echo "[ERROR] Version file not found: $RTABMAP_CONFIG_VERSION_FILE" >&2
  exit 3
fi
if ! grep -Eq 'PACKAGE_VERSION[[:space:]]+"0\.23\.4"' "$RTABMAP_CONFIG_VERSION_FILE"; then
  echo "[ERROR] RTABMap version is not 0.23.4: $RTABMAP_CONFIG_VERSION_FILE" >&2
  exit 4
fi
echo "[INFO] Using RTABMap_DIR: $RTABMap_DIR"

HAVE_LOCAL_NAV2=0
TOTAL_STAGES=5
if [[ -f "$ROOT_DIR/src/navigation2/navigation2/package.xml" ]]; then
  HAVE_LOCAL_NAV2=1
  TOTAL_STAGES=6
fi

OVERRIDES=(
  rtabmap_conversions rtabmap_costmap_plugins rtabmap_demos rtabmap_examples
  rtabmap_launch rtabmap_msgs rtabmap_odom rtabmap_python rtabmap_ros
  rtabmap_rviz_plugins rtabmap_slam rtabmap_sync rtabmap_util rtabmap_viz
)

if [[ "$HAVE_LOCAL_NAV2" == "1" ]]; then
  OVERRIDES+=(
    costmap_queue dwb_core dwb_critics dwb_msgs dwb_plugins
    nav_2d_msgs nav_2d_utils
    nav2_amcl nav2_behavior_tree nav2_behaviors nav2_bringup nav2_bt_navigator
    nav2_collision_monitor nav2_common nav2_constrained_smoother nav2_controller
    nav2_core nav2_costmap_2d nav2_dwb_controller nav2_graceful_controller
    nav2_lifecycle_manager nav2_map_server nav2_mppi_controller nav2_msgs
    nav2_navfn_planner nav2_planner nav2_regulated_pure_pursuit_controller
    nav2_rotation_shim_controller nav2_route nav2_rviz_plugins
    nav2_simple_commander nav2_smac_planner nav2_smoother
    nav2_theta_star_planner nav2_util nav2_velocity_smoother
    nav2_voxel_grid nav2_waypoint_follower navigation2
  )
fi

DRIVER_PACKAGES=(
  livox_ros_driver2 fast_lio
)

BASE_PACKAGES=(
  rtabmap rtabmap_msgs rtabmap_costmap_plugins rtabmap_python rtabmap_conversions
)

HEAVY_PACKAGES=(
  rtabmap_sync rtabmap_viz rtabmap_rviz_plugins
)

REST_PACKAGES=(
  rtabmap_util rtabmap_odom rtabmap_slam rtabmap_launch rtabmap_examples rtabmap_demos rtabmap_ros
)

APP_PACKAGES=(
  rtsp_camera_bridge robot_bringup
)

NAV2_PACKAGES=()
if [[ "$HAVE_LOCAL_NAV2" == "1" ]]; then
  # Keep runtime Nav2 packages in the full build.
  # Intentionally skip nav2_system_tests to avoid pulling test-only dependencies
  # such as Gazebo / launch_testing into the normal workspace build.
  NAV2_PACKAGES=(
    nav2_common nav2_msgs nav2_util nav2_core nav2_costmap_2d nav2_voxel_grid
    nav_2d_msgs nav_2d_utils dwb_msgs costmap_queue dwb_core dwb_critics dwb_plugins
    nav2_amcl nav2_behavior_tree nav2_behaviors nav2_bt_navigator
    nav2_collision_monitor nav2_constrained_smoother nav2_controller
    nav2_dwb_controller nav2_graceful_controller nav2_lifecycle_manager
    nav2_map_server nav2_mppi_controller nav2_navfn_planner nav2_planner
    nav2_regulated_pure_pursuit_controller nav2_rotation_shim_controller
    nav2_route nav2_rviz_plugins nav2_simple_commander nav2_smac_planner
    nav2_smoother nav2_theta_star_planner nav2_velocity_smoother
    nav2_waypoint_follower nav2_bringup navigation2
  )
fi

CMAKE_ARGS=(
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
  -DOpenCV_DIR="$OpenCV_DIR"
)

COMMON_ARGS=(
  --symlink-install
  --cmake-clean-cache
  --allow-overriding "${OVERRIDES[@]}"
)

echo "[STEP] Build stage 1/$TOTAL_STAGES: drivers and odometry (parallel)"
export MAKEFLAGS="-j${JOBS} -l${JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"
colcon build \
  --executor parallel \
  --parallel-workers "$WORKERS" \
  "${COMMON_ARGS[@]}" \
  --cmake-args "${CMAKE_ARGS[@]}" \
  --packages-select "${DRIVER_PACKAGES[@]}"

echo "[STEP] Build stage 2/$TOTAL_STAGES: RTABMap base packages (parallel)"
export MAKEFLAGS="-j${JOBS} -l${JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"
colcon build \
  --executor parallel \
  --parallel-workers "$WORKERS" \
  "${COMMON_ARGS[@]}" \
  --cmake-args "${CMAKE_ARGS[@]}" \
  --packages-select "${BASE_PACKAGES[@]}"

echo "[STEP] Build stage 3/$TOTAL_STAGES: RTABMap heavy packages (low parallel)"
export MAKEFLAGS="-j${HEAVY_JOBS} -l${HEAVY_JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="$HEAVY_JOBS"
colcon build \
  --executor sequential \
  --parallel-workers 1 \
  "${COMMON_ARGS[@]}" \
  --cmake-args "${CMAKE_ARGS[@]}" \
  --packages-select "${HEAVY_PACKAGES[@]}"

echo "[STEP] Build stage 4/$TOTAL_STAGES: RTABMap remaining packages (parallel)"
export MAKEFLAGS="-j${JOBS} -l${JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"
colcon build \
  --executor parallel \
  --parallel-workers "$WORKERS" \
  "${COMMON_ARGS[@]}" \
  --cmake-args "${CMAKE_ARGS[@]}" \
  --packages-select "${REST_PACKAGES[@]}"

if [[ "$HAVE_LOCAL_NAV2" == "1" ]]; then
  echo "[STEP] Build stage 5/$TOTAL_STAGES: Navigation2 packages (parallel)"
  export MAKEFLAGS="-j${JOBS} -l${JOBS}"
  export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"
  colcon build \
    --executor parallel \
    --parallel-workers "$WORKERS" \
    "${COMMON_ARGS[@]}" \
    --cmake-args "${CMAKE_ARGS[@]}" \
    --packages-select "${NAV2_PACKAGES[@]}"

  echo "[STEP] Build stage 6/$TOTAL_STAGES: application packages (parallel)"
else
  echo "[INFO] Local Navigation2 source not found at src/navigation2, skipping Nav2 source build."
  echo "[STEP] Build stage 5/$TOTAL_STAGES: application packages (parallel)"
fi
export MAKEFLAGS="-j${JOBS} -l${JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"
colcon build \
  --executor parallel \
  --parallel-workers "$WORKERS" \
  "${COMMON_ARGS[@]}" \
  --cmake-args "${CMAKE_ARGS[@]}" \
  --packages-select "${APP_PACKAGES[@]}"

echo "[DONE] Build finished."
echo "[NEXT] source \"$ROOT_DIR/install/setup.bash\""
