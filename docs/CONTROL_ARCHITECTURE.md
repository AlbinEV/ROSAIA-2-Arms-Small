# ROSAIA-derived visual control architecture

## State and actuation

Each arm has one base marker, three moving markers, and one primary motor. Its
visual state is

```text
x = [x1, y1, x2, y2, x3, y3]^T in R^6
```

and its generalized coordinate is the scalar encoder position `q`. Unlike the
historical two-motor ROSAIA hardware, there is no coupled antagonistic command.
The per-arm differential model is therefore

```text
x_dot = J(q) q_dot,     J(q) in R^(6x1).
```

The two-arm system can be treated as two independent scalar-input models first;
cross-arm terms can be introduced later only if measurements show mechanical
coupling.

## Perception contract

`aruco_shape_tracker` detects `DICT_4X4_50` markers in synchronized RealSense
RGB-D frames and publishes three ordered metric points per arm as
`sensor_msgs/PointCloud`. The two base-marker centres define a shared robot
orientation; each arm retains its own base origin. A 3-D constant-velocity
Kalman filter stabilizes measured points, while incomplete or prediction-only
frames are not published for training. Plane calibration will fix the final
two-dimensional projection convention.

## Identification data

At each accepted sample, store one timestamped row containing:

```text
q, q_dot, command, x[6], x_dot[6], marker validity, camera timestamp
```

Identification excitations are bounded single-axis velocity profiles. There is
no `coupled_motor` or `coupled_gain`. Samples with incomplete marker sets,
non-monotonic timestamps, or stale motor telemetry are excluded rather than
filled with an old pose.

The first model can reproduce the one-DOF DeJaN structure: learn marker
positions as a differentiable function of `q`, then obtain `J(q)` by automatic
differentiation. A finite-difference/local ridge Jacobian is the baseline used
to validate signs and scaling before enabling the learned controller.

The implemented `rosaia_data_acquisition` recorder preserves the complete
motor telemetry stream, the latest commanded encoder-velocity reference and
one visual-sample table. Offline preprocessing can
therefore estimate timing offsets and interpolate encoder position at camera
timestamps instead of treating callback arrival order as exact synchronization.
The calibration, excitation, hysteresis-test, split, and validation protocol is
defined in `TRAINING_PIPELINE.md`.

## Resolved-rate control

For error `e = x_goal - x`, the damped scalar pseudoinverse gives

```text
q_dot_ref = gain * (J^T e) / (J^T J + damping^2).
```

The reference is saturated in encoder counts per second and passed to a
separate motor-velocity loop that produces bounded PWM. The visual controller
must not publish raw PWM directly: separating the encoder velocity loop keeps
the learned Jacobian kinematic and reduces dependence on load, friction, and
supply-current limiting.

The command becomes zero when the visual state or encoder feedback is stale,
the Jacobian norm is too small, a configured encoder limit is reached, or the
bridge reports a fault.

## Implementation sequence

1. Base-relative ArUco state extraction and timestamp/validity contract.
2. Timestamp-aware acquisition recorder (implemented), then encoder velocity estimation.
3. Low-amplitude bounded excitation profiles for each motor independently.
4. Finite-difference Jacobian baseline and offline replay.
5. Differentiable learned model and comparison against the baseline.
6. Closed-loop visual control with zero-command supervision.
7. Stereo metric reconstruction without changing the controller interface.
