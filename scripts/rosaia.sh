#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$REPO_ROOT/ros2_ws"

source_ros() {
  # ROS setup scripts legitimately reference variables that may be unset.
  set +u
  source /opt/ros/jazzy/setup.bash
  if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
    source "$WORKSPACE/install/setup.bash"
  fi
  set -u
}

usage() {
  sed -n '/^Commands:/,$p' <<'EOF'
Usage: scripts/rosaia.sh COMMAND [ARGS]

Commands:
  build                 Build all ROS 2 packages with symlink install
  test                  Run package tests and print their results
  bridge                Start the Bluno serial bridge
  vision                Start the RealSense RGB-D ArUco tracker
  recorder [NAME]       Record synchronized CSV data under data/sessions
  excitation [ARGS]     Generate a non-actuating step/Fourier reference CSV
  train [ARGS]          Train one MLP from complete recorded sessions
  jacobian [ARGS]       Publish learned Jacobians from encoder positions
  controller            Start the non-actuating shape controller
  view                  Open the annotated image with rqt_image_view
  status                Show devices, relevant nodes and topic rates
  stop                  Publish a zero command once
  preview               Run the direct single-camera ArUco preview
EOF
}

command="${1:-help}"
shift || true

case "$command" in
  build)
    source_ros
    cd "$WORKSPACE"
    colcon build --symlink-install
    ;;
  test)
    source_ros
    cd "$WORKSPACE"
    colcon test --event-handlers console_direct+
    colcon test-result --verbose
    ;;
  bridge)
    source_ros
    exec ros2 launch bluno_motor_bridge bluno_motor_bridge.launch.py "$@"
    ;;
  vision)
    source_ros
    exec ros2 launch aruco_shape_tracker aruco_shape_tracker.launch.py "$@"
    ;;
  recorder)
    source_ros
    session_name="${1:-$(date +%Y%m%d_%H%M%S)}"
    cd "$REPO_ROOT"
    exec ros2 launch rosaia_data_acquisition session_recorder.launch.py \
      session_name:="$session_name" output_root:="$REPO_ROOT/data/sessions"
    ;;
  excitation)
    source_ros
    cd "$REPO_ROOT"
    exec ros2 run rosaia_learning generate_excitation "$@"
    ;;
  train)
    source_ros
    cd "$REPO_ROOT"
    exec ros2 run rosaia_learning train_mlp "$@"
    ;;
  jacobian)
    source_ros
    exec ros2 launch rosaia_learning jacobian_publisher.launch.py "$@"
    ;;
  controller)
    source_ros
    exec ros2 launch rosaia_shape_control shape_controller.launch.py "$@"
    ;;
  view)
    source_ros
    if ! command -v rqt_image_view >/dev/null 2>&1; then
      echo "rqt_image_view is not installed (package: ros-jazzy-rqt-image-view)." >&2
      exit 2
    fi
    exec rqt_image_view /aruco/annotated/compressed
    ;;
  status)
    source_ros
    echo "Serial device:"
    ls -l /dev/serial/by-id/usb-2341_0043-if00 2>/dev/null || echo "  not found"
    echo "RGB devices:"
    v4l2-ctl --list-devices 2>/dev/null || echo "  v4l2-ctl unavailable"
    echo "Relevant ROS nodes:"
    ros2 node list 2>/dev/null | grep -E 'bluno|aruco|shape|recorder' || true
    echo "Relevant ROS topics:"
    ros2 topic list 2>/dev/null | grep -E 'motor|encoder|aruco|arm_|shape' || true
    ;;
  stop)
    source_ros
    ros2 topic pub --once /motor_command std_msgs/msg/Int16MultiArray \
      '{data: [0, 0]}'
    ;;
  preview)
    cd "$REPO_ROOT"
    exec python3 scripts/single_aruco_preview.py "$@"
    ;;
  help|-h|--help)
    usage
    ;;
  *)
    echo "Unknown command: $command" >&2
    usage >&2
    exit 2
    ;;
esac
