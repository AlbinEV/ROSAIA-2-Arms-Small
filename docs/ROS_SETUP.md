# ROS 2 setup decisions

This note records what is implemented, what was learned from the available
ROSAIA material, and which details remain intentionally unresolved.

## Scope of the first milestone

The initial package implements only the host-side serial boundary:

- receive two logical primary-motor commands from ROS;
- map axis 0 to Makerfabs M2 and axis 1 to M1 in one location;
- command both physical primary motors independently;
- encode `CMD` and `STOP` packets;
- parse encoder/current `STATE` telemetry;
- publish raw measurements and link status;
- stop on stale commands, disconnect, shutdown, or invalid configuration.

The RealSense RGB-D ArUco frontend is implemented as the independent
`aruco_shape_tracker` package. Timestamp-aware CSV acquisition is implemented
in `rosaia_data_acquisition`. Offline step/Fourier profile generation, MLP
training, timestamp interpolation, portable model export, and runtime Jacobian
publication are implemented in `rosaia_learning`. Dataset reports, automatic
timing-offset estimation, and encoder velocity control remain subsequent
components.

## ROS interfaces

All names are relative so that the complete stack can later be placed in a ROS
namespace.

| Direction | Topic | Type | Meaning |
|---|---|---|---|
| subscribe | `motor_command` | `std_msgs/Int16MultiArray` | `[arm_1_primary, arm_2_primary]`, raw signed commands |
| publish | `encoder_counts` | `std_msgs/Int64MultiArray` | raw counts for axes 0 and 1 |
| publish | `motor_current` | `std_msgs/Float32MultiArray` | filtered current for axes 0 and 1 in amperes |
| publish | `motor_state` | `std_msgs/String` | complete telemetry packet as JSON |
| publish | `bluno/status` | `std_msgs/String` | bridge health as JSON |
| publish | `/arm_1/aruco_state` | `sensor_msgs/PointCloud` | three ordered base-relative visual points |
| publish | `/arm_2/aruco_state` | `sensor_msgs/PointCloud` | three ordered base-relative visual points |
| publish | `/arm_N/aruco_points_3d_raw` | `sensor_msgs/PointCloud` | raw points in the camera optical frame |
| publish | `/aruco/status` | `std_msgs/String` | detection completeness and observed IDs |
| subscribe | `/arm_N/shape_goal` | `sensor_msgs/PointCloud` | ordered desired three-marker state |
| subscribe | `/arm_N/jacobian` | `std_msgs/Float64MultiArray` | learned or baseline 6x1 Jacobian |
| publish | `/arm_N/encoder_velocity_reference` | `std_msgs/Float64` | bounded scalar velocity reference; never raw PWM |
| subscribe | `/motor_state`, `/motor_command`, `/arm_N/aruco_state` | existing types | recorder inputs for synchronized experiment sessions |
| subscribe | `/encoder_counts` | `std_msgs/Int64MultiArray` | MLP inference input after configured encoder sign |
| publish | `/arm_N/jacobian` | `std_msgs/Float64MultiArray` | analytic derivative of the validated MLP at current `q` |
| publish | `/jacobian_model/status` | `std_msgs/String` | model loading, freshness, and training-range state |

These simple message types are appropriate for hardware bring-up. Before the
learning/control layer is implemented, replace the arrays/JSON with explicit
project message definitions so units, timestamps, flags, and axis identity are
part of the type contract.

## Relationship to original ROSAIA

The available historical ROSAIA material uses a base marker plus three moving
markers and exposes motor velocity commands and joint states. It also contains
logic built around coupled primary/antagonistic actuation. Only the visual
measurement concept and general separation of concerns should be carried over.
The coupled two-motor policy is not valid for this one-primary-motor-per-arm
platform.

The planned future data flow is:

```text
camera -> ArUco poses -> base-relative 3-marker arm state
       -> Jacobian identification / learned model
       -> bounded primary velocity per arm
       -> motor command adapter -> Bluno bridge
```

## Unknowns and gates

Do not unlock motion until these values have been measured or decided:

- stable serial device path;
- safe initial PWM limit;
- physical motor and encoder signs;
- encoder CPR and gearbox ratio;
- ACS712 variant, zero calibration, sensitivity, and safe-current limit;
- axis-1 motor and encoder signs and gearbox ratio;
- ROS marker IDs, camera frames, and exact reference-marker convention;
- controller command units at the learning-layer boundary.

The hardware contract is provisional. If a bench observation conflicts with
it, stop, record the observation, and update the contract before changing code.
