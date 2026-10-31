#!/usr/bin/env bash
export IGN_GAZEBO_RESOURCE_PATH=~/airobot_ws/src/user_code/models:$IGN_GAZEBO_RESOURCE_PATH
set -eo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
setup_file="${ROS_WORKSPACE:-$HOME/airobot_ws}/install/setup.bash"

if [[ ! -f "$setup_file" ]]; then
  echo "ROS workspace setup not found: $setup_file" >&2
  exit 1
fi
source "$setup_file"

for executable in ros2 ign setsid; do
  if ! command -v "$executable" >/dev/null 2>&1; then
    echo "Required command not found: $executable. Run inside the ROS container." >&2
    exit 1
  fi
done

export DISPLAY="${DISPLAY:-:0}"
# Override with LIBGL_ALWAYS_SOFTWARE=0 to use hardware rendering.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export CAMERAS="${CAMERAS:-1}"
pids=()
cleanup() {
  trap - EXIT INT TERM
  echo "Stopping simulation and camera bridges..."
  for pid in "${pids[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  # Give ROS launch time to shut down its child processes.
  for attempt in {1..5}; do
    alive=false
    for pid in "${pids[@]}"; do
      if kill -0 -- "-$pid" 2>/dev/null; then alive=true; fi
    done
    if [[ "$alive" == false ]]; then break; fi
    sleep 1
  done
  for pid in "${pids[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Starting camera/car world from $project_root"
echo "Desktop: http://localhost:6080 | Stop: Ctrl+C"
setsid ros2 launch \
  "$project_root/camera/launch/crane_plus_with_camera.launch.py" &
pids+=("$!")
if [[ "$CAMERAS" == 1 ]]; then
  bridge_args=()
  # for camera in top left right wrist; do
  for camera in top wrist; do
    bridge_args+=(
      "/camera_${camera}/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image"
      "/camera_${camera}/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo"
    )
  done
  setsid ros2 run ros_gz_bridge parameter_bridge "${bridge_args[@]}" &
  pids+=("$!")
else
  echo "Camera is set to off"
fi

# If either launcher exits, shut down the companion process too.
set +e
wait -n "${pids[@]}"
status=$?
set -e
echo "A simulation launcher exited (status $status)." >&2
exit "$status"
