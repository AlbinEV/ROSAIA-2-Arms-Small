# Agent handoff — ROSAIA two-arm training

Updated: 2026-09-24, Europe/Rome.

## Objective

Continue identification, visualization and inference for two independent
single-actuator deformable arms. Each arm is observed by three moving ArUco
markers; markers 0 and 1 define the frozen base frames. The intended final UI
shows both current shapes, accepts desired shapes, projects them onto the
reachable one-dimensional manifolds, and closes the loop using vision,
encoders and current sensing.

Treat the operator as a robotics researcher. Report measurements and
trade-offs directly. Never start a motor trajectory without stating the exact
axis/profile, but routine bounded tests explicitly requested by the operator
may proceed without unnecessary confirmation loops.

## Current physical and ROS state

- Nominal motor supply: **5.6 V transformer**. The old velocity maps were
  identified at 6.75 V and must not be used at 5.6 V.
- Current encoder state at handoff: `[0, 0]`.
- Applied/requested PWM: `[0, 0]`.
- Firmware flag: `16` only (`CURRENT_SCALE_UNKNOWN`, expected).
- ArUco IDs 0–7 and RGB-D are valid; both arms are complete.
- Camera serial: `152122077644`.
- Dashboard: <http://127.0.0.1:8765>, goal publication disabled.
- Active nodes expected: `/aruco_shape_tracker`, `/bluno_motor_bridge`,
  `/rosaia_dashboard`.
- Do not start `encoder_velocity_controller`: its checked-in schedules are
  explicitly historical 6.75 V values.

Check state before continuing:

```bash
curl -fsS http://127.0.0.1:8765/api/state | jq '{
  encoder:.motor.encoder_counts,
  applied:.motor.applied_commands,
  requested:.motor.requested_commands,
  current:.motor.current_a_host,
  flags:.motor.flags,
  arms:.vision.complete_arms
}'
```

## Hardware conventions

| Item | Arm 1 | Arm 2 |
|---|---:|---:|
| Logical axis | 0 | 1 |
| Shield output | M2 | M1 |
| Encoder pins | D2/D3 | D11/D12 |
| Operational encoder sign | +1 | -1 |
| Motor command sign for contraction | -1 | -1 |
| Provisional operational range | 0–300 | 0–300 |

Important: the `STEP_AXIS` signed-count field selects **motor direction** and
only its magnitude is the raw-encoder displacement bound. Therefore an
increasing operational-q step uses
`command_to_q_sign * operational_step`, not `encoder_sign * operational_step`.
This convention is implemented in `scripts/quasistatic_encoder_sweep.py`.

Small bounded sweeps use firmware-local steps of at most 5 encoder counts.
The host script aborts on stale telemetry, firmware faults or step timeout.

## Valid datasets

Raw sessions are under ignored `data/sessions/`; do not modify them.

### Arm 1

- Training:
  `data/sessions/arm1_shape_train_5v6_20260924_01`
  - audit ready; 1610 visual rows;
  - q range 0–220;
  - no unexpected flags;
  - current range −1.851…+1.407 A.
- Validation candidate:
  `data/sessions/arm1_shape_validation_5v6_20260924_01`
  - audit ready; 3114 visual rows;
  - q range 0–217;
  - no unexpected flags in accepted samples;
  - contained one interrupted outbound step at q≈200 followed by bounded
    completion at PWM 130. Prefer collecting an additional completely clean
    validation cycle before approving online control.

Arm-1 deformation is nearly flat through q≈140. Approximate tip displacement
from home in the training session:

| q | tip Δx | tip Δy |
|---:|---:|---:|
| 160 | 4.42 mm | 1.32 mm |
| 180 | 9.93 mm | 2.23 mm |
| 200 | 20.90 mm | 3.21 mm |
| 220 | 30.68 mm | 3.95 mm |

### Arm 2

- Training:
  `data/sessions/arm2_shape_train_5v6_20260924_02`
  - audit ready; 1883 visual rows;
  - q range 0–216;
  - no unexpected flags;
  - current range −2.444…+1.407 A.
- Validation:
  `data/sessions/arm2_shape_validation_5v6_20260924_02`
  - audit ready; 1693 visual rows;
  - q range 0–216;
  - no unexpected flags;
  - current range −1.259…+1.629 A.

Arm 2 is strongly observable across the full range. In the diagnostic map,
tip displacement at q≈216 was approximately Δx=−148.8 mm, Δy=−65.7 mm.

## Sessions that must not be used for final training

- `arm1_bounded_pilot_5v6_20260924_01`: wrong STEP sign diagnostic.
- `arm1_bounded_pilot_5v6_20260924_02`: contraction timeout at q≈20.
- `arm1_bounded_train_5v6_20260924_01`: release timeout flag 1.
- `arm2_bounded_pilot_5v6_20260924_01`: release timeouts.
- `arm2_shape_train_5v6_20260924_01`: contraction timeout near q≈193.
- `arm2_shape_validation_5v6_20260924_01`: multiple breakaway probes/timeouts.

Calibration-only sessions remain useful as evidence but not as clean
train/validation trajectories.

## Models and findings

Arm-1 useful candidate:

```text
data/models/arm1_shape_5v6_v1
```

Metrics:

- converged: true;
- train q: 0–220;
- validation q: 0–216.84;
- validation RMSE: 0.585 mm;
- validation P95: 1.220 mm;
- validation maximum component error: 4.553 mm;
- extrapolation fraction: 0.

This model is **not approved**. At home, pure shape projection can select a q
inside the flat pretension region (observed q*=138), because q=0…140 produces
almost the same visual shape. Goal projection therefore needs an encoder prior
or continuity regularization before online use. A sensible formulation is

```text
argmin_q ||f(q) - x_goal||² + lambda_q (q - q_current)²
```

with the shape term dominant once meaningful deformation begins.

The earlier short-range model `arm1_bounded_5v6_v2` has RMSE 0.286 mm but is
not useful for inversion: the endpoint shape change over q=0…116 is only about
0.46 mm.

## Immediate next actions

1. Train arm 2 from the clean sessions:

```bash
scripts/rosaia.sh train \
  --arm arm_2 \
  --train-session data/sessions/arm2_shape_train_5v6_20260924_02 \
  --validation-session data/sessions/arm2_shape_validation_5v6_20260924_02 \
  --hidden 32,32 --max-iterations 8000 \
  --q-bin-width 5 --maximum-samples-per-session-bin 50 \
  --minimum-q 0 --maximum-q 220 \
  --output data/models/arm2_shape_5v6_v1
```

2. Inspect convergence, RMSE/P95/max, extrapolation and Jacobian smoothness.
   Do not promote based on RMSE alone.
3. Restart the dashboard with both candidates for passive inference:

```bash
scripts/rosaia.sh dashboard \
  arm_1_model:=$PWD/data/models/arm1_shape_5v6_v1 \
  arm_2_model:=$PWD/data/models/arm2_shape_5v6_v1
```

4. Add q-current regularization to
   `rosaia_dashboard/projection.py` and pass the selected axis encoder q from
   `dashboard_node.py`. Keep `enable_goal_publish=false`.
5. Collect a clean second arm-1 validation trajectory using a contraction PWM
   schedule (`0:110,190:130`) and release PWM 105, if repeatability permits.
6. Validate local Jacobian sign/smoothness and offline goal projection before
   any controller output is enabled.

## Sweep commands known to work

Arm 1, extended range:

```bash
scripts/rosaia.sh sweep --axis 0 --encoder-sign 1 \
  --command-to-q-sign -1 --pwm 110 --pwm-schedule 0:110,190:130 \
  --reverse-pwm 105 \
  --points 0,20,40,60,80,100,120,140,160,180,200,220 \
  --step-counts 5 --dwell 1 --tolerance 5 --step-timeout 3 \
  --maximum-q 240
```

Arm 2, robust extended range:

```bash
scripts/rosaia.sh sweep --axis 1 --encoder-sign -1 \
  --command-to-q-sign -1 --pwm 180 --reverse-pwm 120 \
  --points 0,20,40,60,80,100,120,140,160,180,200,220 \
  --step-counts 5 --dwell 1 --tolerance 5 --step-timeout 3 \
  --maximum-q 240
```

Always start the recorder before the sweep and audit the completed session.

## Repository state

- Remote: `git@github.com:AlbinEV/ROSAIA-2-Arms-Small.git`
- Branch: `main`
- Baseline commit before this handoff: `5bb788c`. The handoff implementation
  is the following commit on `main`; use `git log -2 --oneline` to identify it.
- Implementation work covered by the handoff commit:
  - q-range filtering in `rosaia_learning/train.py`;
  - corrected/adaptive bounded sweep tool;
  - this handoff document and training-guide edits.
- The handoff was tested before commit. Re-run these checks after any change:

```bash
scripts/rosaia.sh build
scripts/rosaia.sh test
git diff --check
git status --short
```

Raw `data/` artifacts are intentionally ignored and remain local.
