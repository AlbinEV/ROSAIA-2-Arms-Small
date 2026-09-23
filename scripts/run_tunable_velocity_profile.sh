#!/usr/bin/env bash
set -euo pipefail

# Edit these values directly, or override them from the environment, e.g.:
# PEAK_VELOCITY=30 MOVE_DURATION=4 ./scripts/run_tunable_velocity_profile.sh
PROFILE_TYPE="${PROFILE_TYPE:-step}"
AXIS="${AXIS:-0}"
MOVE_DURATION="${MOVE_DURATION:-5.0}"
RISE_TIME="${RISE_TIME:-1.0}"
FALL_TIME="${FALL_TIME:-1.0}"
PEAK_VELOCITY="${PEAK_VELOCITY:-25.0}"
HOME_DWELL="${HOME_DWELL:-1.0}"
PUBLISH_RATE="${PUBLISH_RATE:-100.0}"
DRY_RUN="${DRY_RUN:-0}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "$PROFILE_TYPE" == "step" ]]; then
  nominal_excursion="$(awk -v duration="$MOVE_DURATION" \
    -v peak="$PEAK_VELOCITY" \
    'BEGIN { printf "%.2f", peak * duration }')"
elif [[ "$PROFILE_TYPE" == "trapezoid" ]]; then
  nominal_excursion="$(awk -v duration="$MOVE_DURATION" \
    -v rise="$RISE_TIME" -v fall="$FALL_TIME" \
    -v peak="$PEAK_VELOCITY" \
    'BEGIN { printf "%.2f", peak * (duration - 0.5 * (rise + fall)) }')"
else
  echo "PROFILE_TYPE must be step or trapezoid" >&2
  exit 2
fi

echo "Velocity profile"
echo "  type:               $PROFILE_TYPE"
echo "  axis:               $AXIS"
echo "  outward duration:   $MOVE_DURATION s"
if [[ "$PROFILE_TYPE" == "trapezoid" ]]; then
  echo "  rise / fall:        $RISE_TIME / $FALL_TIME s"
fi
echo "  peak velocity:      $PEAK_VELOCITY count/s"
echo "  nominal excursion:  $nominal_excursion count"
echo "  home dwell:         $HOME_DWELL s"
echo "  publish rate:       $PUBLISH_RATE Hz"

if [[ "$DRY_RUN" == "1" ]]; then
  exit 0
fi

exec "$REPO_ROOT/scripts/rosaia.sh" profile \
  --type "$PROFILE_TYPE" \
  --axis "$AXIS" \
  --duration "$MOVE_DURATION" \
  --rise-time "$RISE_TIME" \
  --fall-time "$FALL_TIME" \
  --maximum-velocity "$PEAK_VELOCITY" \
  --home-dwell "$HOME_DWELL" \
  --rate "$PUBLISH_RATE" \
  "$@"
