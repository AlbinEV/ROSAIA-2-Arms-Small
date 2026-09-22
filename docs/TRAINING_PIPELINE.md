# Calibration and learning pipeline

## Objective and model boundary

Each deformable arm has one motor coordinate `q` and a six-component visual
state

```text
x = [x1, y1, x2, y2, x3, y3].
```

The first identification target is the per-arm differential map

```text
x_dot = J(q) q_dot,       J(q) in R^(6x1).
```

The arms are identified separately. Simultaneous motion is useful only after
both independent models pass offline validation; it is then used to test for
camera, electrical, or mechanical cross-coupling.

The tracker now supplies metric RGB-D coordinates with a shared robot
orientation and one origin per base marker. Plane calibration must estimate
the arm-plane offset/normal and freeze that convention with each dataset and
model artifact. The learned interface remains the six in-plane coordinates.

## Mandatory calibration gates

Training data collection starts only after a calibration manifest records the
following values for both axes.

1. **Marker contract:** reference and moving IDs, their physical order, full
   visibility rate, and stationary image noise.
2. **Motor/encoder contract:** command sign, raw encoder sign, contraction
   sign, home definition, usable minimum/maximum count, gearbox ratio, and
   encoder counts per motor/output revolution where available.
3. **Electrical contract:** ADC zero distribution with outputs disabled,
   current-sensor variant/sensitivity, loaded current distribution, and the
   configured supply/firmware limits.
4. **Velocity actuation:** breakaway PWM in both directions, steady `q_dot`
   versus PWM, reversal behaviour, and a bounded low-level velocity-loop
   configuration.
5. **Timing:** camera frame interval/jitter, serial telemetry interval/jitter,
   dropped packets, and the camera-to-telemetry delay used during offline
   interpolation.

Every calibration session is immutable. Its raw data and a small JSON manifest
identify hardware wiring, firmware revision, camera serial/resolution, marker
mapping, supply voltage/current limit, and Git revision. Derived parameters are
stored separately so raw observations are never overwritten.

Start each calibration record from
`config/calibration_manifest.template.yaml`; set `eligible_for_training: true`
only after every acceptance field has been verified from recorded data.

## Dataset acquisition

`rosaia_data_acquisition/session_recorder` writes one session directory:

```text
data/sessions/<session>/
├── metadata.json       acquisition configuration and schema version
├── telemetry.csv      every motor telemetry packet
└── samples.csv        one row per complete per-arm ArUco observation
```

`telemetry.csv` is the authoritative high-rate stream. `samples.csv` contains
the most recent telemetry next to each visual observation for immediate
inspection, but the training preprocessing stage should interpolate encoder
position onto `camera_time_ns` using the raw stream after applying the measured
time offset. This interpolation is implemented in `rosaia_learning`; samples
outside the common camera/telemetry interval are rejected rather than clamped.
Both files contain encoder data; they also preserve raw current
ADC and calibrated milliamps. Until the sensor sensitivity is calibrated,
`current_ma` remains unavailable and `adc_raw` is used only diagnostically.

Rows are rejected online when any of the three moving markers is missing, the
shape is non-finite, motor telemetry is absent, or the latest telemetry exceeds
the configured age limit. Offline preprocessing additionally rejects firmware
faults, encoder discontinuities, non-monotonic clocks, saturation intervals,
and samples outside the calibrated range.

## Excitation design

Use independent, bounded trajectories, beginning with one arm stationary:

1. slow contraction/extension ramps across the usable range;
2. repeated triangular sweeps to quantify hysteresis and repeatability;
3. velocity plateaus at several magnitudes in both directions;
4. band-limited Fourier/multisine references only after the velocity loop is
   validated;
5. simultaneous two-arm sequences solely as a cross-coupling validation set.

Coverage should be evaluated in `(q, q_dot, direction)` bins rather than by row
count. Stationary dwell segments are retained to estimate visual noise and
relaxation, but they must not dominate the training loss.

## Baselines and learned model

The implementation order is deliberately incremental.

### Baseline A: local finite difference

Estimate each component of `J` in position bins using robust local regression
of `x_dot` on `q_dot`. This establishes sign, scale, observability, and a
controller baseline without a neural network.

### Learned model: MLP forward kinematics

Fit a smooth model

```text
x_hat = f_theta(q)
J_hat(q) = d f_theta(q) / d q
```

The implemented model is a small `tanh` MLP trained with scikit-learn. Its
weights are exported as numeric NPZ arrays, and runtime inference computes the
Jacobian analytically by applying the chain rule through every layer. The
initial input is encoder position only. Network capacity is increased only when
held-out sessions justify it.

The excitation CLI generates non-actuating reference CSV files:

```bash
scripts/rosaia.sh excitation --type step --axis 0 \
  --levels 0,5,10,0,-5,-10,0 --dwell 2 \
  --output data/excitation_profiles/arm1_step.csv

scripts/rosaia.sh excitation --type fourier --axis 0 \
  --frequencies 0.05,0.11,0.19 --maximum-velocity 10 \
  --duration 60 --output data/excitation_profiles/arm1_fourier.csv
```

These profiles are not sent to hardware until the encoder-velocity loop and
operational ranges are calibrated.

### Hysteresis decision

A deformable, tendon-driven arm may not have a single-valued `x=f(q)`. Compare
states at matched `q` during contraction and extension. If their separation is
larger than stationary visual noise and repeatability error, replace the static
model with a stateful or direction-conditioned model, for example

```text
x_hat(t) = f_theta(q(t), q_dot(t), direction(t), x(t-1)).
```

This decision is measurement-driven; direction/history are not added merely to
improve training-set error.

## Validation protocol

Split by complete trajectories and acquisition sessions, never by randomly
shuffling adjacent frames. Report per marker and per state component:

- forward-model RMSE and 95th-percentile error;
- Jacobian error against the local-regression baseline;
- contraction versus extension error;
- one-step and rollout error;
- resolved-rate tracking in offline replay;
- command saturation, limit hits, and invalid-frame fraction.

The model becomes eligible for online control only if it outperforms the local
baseline on held-out trajectories, preserves the measured Jacobian sign, and
the replayed controller remains within calibrated position and velocity bounds.

## Online control integration

The learned model publishes a fresh six-value Jacobian for each arm. The shape
controller produces a bounded encoder-velocity reference. A separate calibrated
velocity loop converts `q_dot_ref` to PWM using encoder feedback. No learned
component writes PWM directly. Missing/stale vision, stale Jacobian, firmware
faults, range violations, or loss of serial communication all resolve to zero
velocity and then `STOP` at the bridge/firmware boundary.
