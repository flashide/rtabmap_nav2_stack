#!/bin/bash
# 启动 RTSP 相机桥接节点
# 用法：可编辑下方 RTSP_URL 后执行；或通过环境变量覆盖：
#   RTSP_URL='rtsp://...' ./start_rtsp_bridge.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# 默认 RTSP 地址（密码中的 @ 需写成 %40）
RTSP_URL="${RTSP_URL:-rtsp://admin:Basic%402021@192.168.168.25:554/cam/realmonitor?channel=1&subtype=0}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/sensors/camera/rgb/image_rect}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/sensors/camera/rgb/camera_info}"
FRAME_ID="${FRAME_ID:-camera_link}"
CAMERA_INFO_URL="${CAMERA_INFO_URL:-}"
FORCE_MONO="${FORCE_MONO:-false}"
TARGET_FPS="${TARGET_FPS:-0.0}"
WIDTH="${WIDTH:-0}"
HEIGHT="${HEIGHT:-0}"
FOCAL_LENGTH_PX="${FOCAL_LENGTH_PX:-0.0}"

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE-$(command -v python3)}"
set +u
source /opt/ros/humble/setup.bash
[ -f "$WS_ROOT/install/setup.bash" ] && source "$WS_ROOT/install/setup.bash"
set -u

echo "Starting rtsp_camera_bridge ..."
echo "  rtsp_url=$RTSP_URL"
echo "  image_topic=$IMAGE_TOPIC"
echo "  camera_info_topic=$CAMERA_INFO_TOPIC"

launch_args=(
  "image_topic:=$IMAGE_TOPIC"
  "camera_info_topic:=$CAMERA_INFO_TOPIC"
  "frame_id:=$FRAME_ID"
  "force_mono:=$FORCE_MONO"
  "target_fps:=$TARGET_FPS"
  "width:=$WIDTH"
  "height:=$HEIGHT"
  "focal_length_px:=$FOCAL_LENGTH_PX"
)

if [[ -n "$RTSP_URL" ]]; then
  launch_args+=("rtsp_url:=$RTSP_URL")
fi
if [[ -n "$CAMERA_INFO_URL" ]]; then
  launch_args+=("camera_info_url:=$CAMERA_INFO_URL")
fi

ros2 launch rtsp_camera_bridge rtsp_camera_bridge.launch.py "${launch_args[@]}"
