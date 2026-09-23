#!/usr/bin/env bash
set -euo pipefail

# Edit these values directly, or override them from the environment:
# SIGNED_PWM=-140 DURATION=1.5 ./scripts/run_constant_pwm_test.sh
AXIS="${AXIS:-0}"
SIGNED_PWM="${SIGNED_PWM:--105}"
DURATION="${DURATION:-0.3}"
KICK_PWM="${KICK_PWM:--125}"
KICK_DURATION="${KICK_DURATION:-0.05}"
PUBLISH_RATE="${PUBLISH_RATE:-100.0}"
DRY_RUN="${DRY_RUN:-0}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$REPO_ROOT/ros2_ws"

set +u
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

if [[ "$AXIS" != "0" && "$AXIS" != "1" ]]; then
  echo "AXIS must be 0 or 1" >&2
  exit 2
fi
if (( SIGNED_PWM == 0 || SIGNED_PWM < -255 || SIGNED_PWM > 255 )); then
  echo "SIGNED_PWM must be in [-255, -1] or [1, 255]" >&2
  exit 2
fi
if ros2 node list 2>/dev/null | grep -qx '/encoder_velocity_controller'; then
  echo "Stop /encoder_velocity_controller before an open-loop PWM test." >&2
  exit 3
fi

commands=(0 0)
commands[$AXIS]="$SIGNED_PWM"
duty_percent="$(awk -v pwm="$SIGNED_PWM" \
  'BEGIN { if (pwm < 0) pwm = -pwm; printf "%.1f", 100.0 * pwm / 255.0 }')"

echo "Constant open-loop PWM test"
echo "  axis:          $AXIS"
echo "  signed PWM:    $SIGNED_PWM / 255"
echo "  duty cycle:    $duty_percent %"
echo "  duration:      $DURATION s"
if [[ "$KICK_DURATION" != "0" && "$KICK_DURATION" != "0.0" ]]; then
  echo "  initial kick:  $KICK_PWM / 255 for $KICK_DURATION s"
fi
echo "  publish rate:  $PUBLISH_RATE Hz"

if [[ "$DRY_RUN" == "1" ]]; then
  exit 0
fi

stop_motor() {
  ros2 topic pub --once /motor_command std_msgs/msg/Int16MultiArray \
    '{data: [0, 0]}' >/dev/null 2>&1 || true
}
trap stop_motor EXIT INT TERM

echo "Initial motor state:"
timeout 3 ros2 topic echo /motor_state --once --field data || true

python3 "$REPO_ROOT/scripts/constant_pwm_test.py" \
  --axis "$AXIS" \
  --pwm "$SIGNED_PWM" \
  --duration "$DURATION" \
  --kick-pwm "$KICK_PWM" \
  --kick-duration "$KICK_DURATION" \
  --rate "$PUBLISH_RATE"

stop_motor
sleep 0.25
echo "Final motor state:"
timeout 3 ros2 topic echo /motor_state --once --field data || true
trap - EXIT INT TERM
