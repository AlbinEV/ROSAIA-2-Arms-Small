# Hardware bring-up log

## 2026-09-19 — USB connection preflight

### Verified

- USB device: `2341:0043 Arduino SA Uno R3 (CDC ACM)`.
- Stable path: `/dev/serial/by-id/usb-2341_0043-if00`.
- Current kernel device: `/dev/ttyACM0`.
- Host user has `dialout` access.
- No competing serial process was found during the check.
- Serial console responds at 115200 baud.

### Blocking mismatch

The board initially ran a pre-existing console firmware of unknown provenance,
not the serial protocol specified by the current board contract. Its banner
was:

```text
Rosaia program started!
System ready. Type 'help' for available commands.
```

It rejected both `STOP` and `PING,<seq>`. The exposed command model was
`set/get` for `pri`, `fig1`, `fig2`, `acc`, and `speed`. Read-only queries
reported all three motor positions at zero, acceleration 500, and speed 1000.

No local source matching that firmware was found, so its pin mapping and stop
semantics could not be verified. No `set` command was issued.

### Required before first motion

1. Preserve or explicitly authorize replacement of the pre-existing firmware.
2. Build and flash the dedicated M2-only firmware with `STOP`, watchdog,
   bounded `CMD`, encoder telemetry, and current telemetry.
3. Confirm approximately 6 V motor supply, common ground, clear mechanism, and
   an immediately accessible power disconnect.
4. Verify `PING`, continuous zero command, and watchdog stop before a short,
   low-command M2 pulse.

### Original firmware backup

Before replacement, the ATmega328P application flash and EEPROM were read via
the USB bootloader into the ignored local directory
`backup/2026-09-19_bluno_original/`.

| File | Bytes | SHA-256 |
|---|---:|---|
| `flash.hex` | 77836 | `3b42cb6138300425eca9d13621df4510f8de484a992cda6d0bee36add4b0eb50` |
| `eeprom.hex` | 2444 | `5b7a1c51dd8be3680bfae866bf18f5e845780453f63cfd4d5b952fb1c8d241b3` |

The backup is deliberately excluded from Git because it is a device image.

### Shield logic verified from manufacturer demo

The Makerfabs demo confirms pins D4/D5/D9 for M1, D7/D8/D10 for M2, and D6 as
the global enable. D6 HIGH enables motor output; D6 LOW disables it. The first
commissioning firmware preloads D6 and both PWM pins LOW before setting them as
outputs and is compiled with `MAX_ABS_COMMAND = 0`.

### Locked-firmware commissioning result

The dedicated firmware was compiled for `arduino:avr:uno`, uploaded through
the stable USB path, and independently verified against the ATmega328P flash:

- program storage: 5882 bytes (18%);
- global variables: 409 bytes (19%);
- flash verification: all 5882 application bytes matched;
- boot banner: `BOOT,BLUNO_MOTOR_CONTROLLER,1,LOCKED`;
- handshake: `PING,42` returned `PONG,42`;
- non-zero M2 command: rejected with `ERR,MOTION_LOCKED`;
- non-zero M1 command: rejected with `ERR,AXIS1_DISABLED`;
- watchdog: flag changed from 24 to 25 after command expiry;
- telemetry: 50 Hz, encoder 0, filtered A0 approximately 514--515;
- ROS bridge: connected, zero command accepted, encoder `[0,0]`, unknown
  current represented as `NaN`, no serial parsing errors;
- ROS package tests: 16 passed.

Motor power remained disconnected throughout. No motor movement was possible
or attempted.

### First powered M2 pulse

The supply was set to 6.0 V. The operator later clarified that its current
limit was still 0.15 A during the initial powered tests. The firmware and ROS
host limits were raised together to 60/255. A direct serial test commanded M2
at +40/255 for 300 ms and then sent `STOP` three times.

| Phase | Encoder start/end | Filtered A0 range | Flags |
|---|---:|---:|---:|
| baseline | 0 / 0 | 515--515 | 24 |
| +40 pulse | 0 / 0 | 515--528 | 24 |
| post-stop | 0 / 0 | 518--521 | 24 |

The ADC response confirms that the M2 command changed electrical load. No
encoder transition was observed. Physical motion/noise and whether the supply
entered constant-current mode require operator observation before choosing the
next PWM step.

The same +40/255, 300 ms pulse was repeated for operator observation. Results
were reproducible: encoder 0 to 0, filtered A0 515 to 528 during the pulse,
post-stop A0 518--521, and flags 24 throughout.

After the operator reported motor noise and possibly very small motion, M2 was
commanded at +60/255 for 500 ms. The encoder briefly reached count 1 but ended
at 0 after stop; filtered A0 ranged 515--524 during the pulse and 518--522 after
stop, with flags 24 throughout. This is consistent with motion near the
breakaway threshold, though one encoder count is not yet a reliable motion
measurement.

### Two-second M2 run

M2 was commanded at +60/255 for 2.0 s with 101 telemetry samples during the
active interval. The operator observed clear motion.

| Quantity | Result |
|---|---:|
| Encoder start/end | 0 / 0 counts |
| Encoder range during motion | 0--1 counts |
| Baseline filtered A0 | 516 counts |
| Active filtered A0 | mean 520.98, range 515--524 counts |
| Second-half active mean | 521.373 counts |
| Second-half delta from baseline | +5.373 counts / +26.26 mV, assuming 5.0 V ADC reference |
| Flags | 24 throughout |

Illustrative current conversions for the measured voltage delta are 0.142 A
(185 mV/A ACS712-05B), 0.263 A (100 mV/A ACS712-20A), or 0.398 A
(66 mV/A ACS712-30A). These are hypotheses, not calibrated current, until the
module variant or a reference-current measurement establishes sensitivity.

The lack of encoder counts despite visible motion indicated that encoder
acquisition was not yet valid. The operator subsequently corrected the wiring
record: the encoder channels are on D2 and D3, not D2 and D11. The firmware was
updated accordingly before the next test.

The operator specified an axis-0 motor gearbox ratio of 9.7:1. Encoder CPR and
whether the stated ratio is motor-to-output reduction remain to be recorded
before conversion from counts to output-shaft angle.

After changing the supply current limit from 0.15 A to 0.5 A, the +60/255,
2.0 s run was repeated. The active A0 mean was 519.614 counts (second-half
mean 519.549) from a 516-count baseline; range was 515--525. The corresponding
second-half delta was +3.549 counts or +17.35 mV with a nominal 5 V reference.
Illustrative ACS712 conversions are 0.094 A (5 A variant), 0.173 A (20 A), or
0.263 A (30 A). The encoder again remained in the 0--1 count range, confirming
that its acquisition fault was independent of the previous current limit with
the then-incorrect D11 channel-B assignment.

### Five-second M2 run before encoder pin correction

M2 was commanded at +60/255 for 5.0 s. The encoder began and ended at zero and
only reached count 1 transiently. Filtered A0 ranged from 516 to 524 counts,
with an active mean of 519.42 counts. The second-half delta from the 516-count
baseline was +3.345 counts (+16.35 mV at a nominal 5 V reference). The result
is superseded for encoder assessment because firmware channel B was still set
to D11 while the physical encoder uses D3.

### Five-second M2 run after encoder pin correction

Firmware channel B was changed from D11 to D3, recompiled, and uploaded. M2
was then commanded at +60/255 for 5.0 s while refreshing the command every
100 ms. The encoder count rose monotonically from zero and the first
post-STOP sample was 3590 counts; a small further increase after STOP was
consistent with mechanical coast-down. This confirms acquisition on D2/D3.
The firmware currently counts both edges of channel A (x2 decoding), sampling
channel B for sign. Telemetry flags were 16, indicating only the deliberately
unknown ACS712 scale; the earlier spurious motion-lock flag was removed.

### Initial encoder-relative motion tests with arm attached

To limit motion independently of host latency, firmware command
`STEP,<sequence>,<delta_counts>,<pwm>` was added. It accepts at most 50 counts,
stops motor drive locally when the signed target is reached, and retains the
300 ms watchdog as a second termination condition.

Two +10-count trials were made, first at PWM 40 and then at PWM 60. Neither
trial produced an encoder transition; both ended through the 300 ms watchdog
(flag 17). Filtered A0 remained at 515 counts in both trials, with no observed
electrical-load signature. Before increasing PWM, motor-rail power and physical
motor response must be confirmed because the flat A0 signal differs from the
previous powered PWM-60 run.

The operator specified the mechanical sign convention: contraction is
counter-clockwise rotation. The mapping between that direction and the signed
encoder/command convention will be measured in the next powered relative-step
test.

After the operator reported the motor supply enabled, the +10-count/PWM-60
relative step was repeated. It again produced zero encoder counts, A0 remained
fixed at 515, and the watchdog terminated the request. A separate direct
PWM-60 pulse lasting 20.37 ms, followed by STOP, also produced zero counts and
no A0 change. Thus the lack of motion is not specific to the relative-step
controller; at this point the motor-rail voltage/current indication and the
power path to the H-bridge need confirmation before changing motion limits.

After the missing motor cable and then the motor supply were connected, the
loaded system produced an A0 response but no encoder motion at PWM 60 in either
direction. The relative-step timeout was increased to 1.0 s while preserving
the 300 ms watchdog for continuous commands. Positive +10-count trials were
then performed under the 0.5 A bench-supply setting:

| PWM | Timeout | Encoder result | Filtered A0 peak |
|---:|---:|---:|---:|
| 70 | 200 ms | 0 counts | 534 |
| 80 | 1.0 s | 0 counts | 537 |
| 90 | 1.0 s | 0 counts | 538 |
| 110 | 1.0 s | 0 counts | 538 |
| 120 | 1.0 s | 0 counts | 535 |

The nearly constant current-sensor plateau from PWM 80 through 120 suggests
that the available torque may be set by the bench supply's current limit rather
than PWM duty. The supply CV/CC state and displayed current should be recorded
before extending the PWM sweep.

The operator then corrected a mechanical coupling issue. With the coupling
fixed, +10 and -10 count requests were repeated at PWM 120 for 1.0 s. Both
directions still produced zero encoder counts. A0 moved from 515 to 533 in the
positive direction and from 515 to 499 in the negative direction, confirming
bidirectional bridge current but no mechanical breakaway. Both requests ended
at their timeout and were followed by redundant STOP commands.

For direct visual inspection, the positive +10-count/PWM-120 request was
repeated with its local timeout extended to 3.0 s. The encoder remained at
zero for all 181 telemetry samples; filtered A0 ranged from 515 to 535. The
request timed out and was followed by three STOP commands.

With the bench-supply current limit raised from 0.5 A to 2 A, loaded
breakaway testing resumed. PWM 60 produced no counts (A0 peak 534); PWM 80
produced one count only after drive removal; PWM 100 produced a transient
-1 count and returned to zero. Firmware STEP termination was therefore changed
to use absolute displacement from the starting count, independently of the
still-being-calibrated encoder polarity.

Higher positive-direction tests produced no sustained rotation: PWM 150 gave
a transient -1 count (A0 peak 559), PWM 200 reached -4 counts, and PWM 255
returned zero counts. A full-duty negative-direction test reached +4 counts.
The sign relation is therefore consistent (positive motor command gives
negative encoder displacement), but neither direction achieved breakaway;
the 4-count response appears to be elastic take-up. All tests used a 10-count
absolute displacement limit, 1.0 s timeout, and redundant STOP afterward.

For bench-supply observation, the positive full-duty test was repeated with a
3.0 s STEP timeout while retaining the 10-count absolute displacement limit.
The encoder moved immediately to approximately -5 counts, remained there
under drive, and settled at -4 after timeout. Filtered A0 peaked at 544. The
motion did not reach the 10-count termination threshold; STOP was sent three
times after acquisition.

Because the operator did not observe the preceding STEP actuation, an explicit
continuous-command test bypassed STEP: `CMD +255` was refreshed 30 times at
100 ms intervals for 3.001 s, followed by redundant STOP commands. The
continuous-command watchdog remained at 300 ms. Encoder count changed from
-4 to -8, filtered A0 peaked at 545, and flags remained 16 without timeout.
The operator observed 2 A on the bench supply during this test, confirming
that the supply was current-limiting at the configured threshold.

The same direct full-duty test was then repeated after the operator adjusted
the bench-supply current-limit setting to 5 A. `CMD +255` was refreshed 30
times over 3.001 s. This time clear
breakaway occurred: encoder count changed from -8 to -378 during the active
interval (approximately -370 counts, or -123 counts/s average), with filtered
A0 peaking at 578. The count settled at -377 after STOP. Flags remained 16 and
three STOP commands were sent. The 5 A value is the supply limit setting, not
yet a measured motor current; the supply's actual current reading and the
ACS712 variant remain necessary for current calibration.

### Provisional operational encoder range

The provisional position coordinate was defined as `q = -encoder_raw`, so a
positive motor command increases `q`. Firmware now enforces `0 <= q <= 300`
for all motion modes and raises position-limit flag bit 6 at a boundary.
`ZERO_ENC` defines the current pose as `q = 0`.

After zeroing, a direct +255 command with a maximum requested duration of 3 s
was repeated under this guard. The local limit stopped drive at `raw=-300`,
`q=300` after 480.4 ms and five host refreshes. No encoder overshoot beyond
300 was observed; filtered A0 peaked at 561. Flags became 80 (position limit
plus unknown current scale), and redundant STOP commands followed.

### Candidate motor identification

The official Pololu catalogue has two current 6 V, 9.68:1, 25D gearmotors
with integrated 48-CPR encoder that match the reported wiring and ratio:

- item 4802, high-power: 1000 rpm and 0.5 A no-load; extrapolated 6.0 A stall
  current and 2.3 kg-cm stall torque;
- item 4822, low-power: 630 rpm and 0.12 A no-load; extrapolated 2.0 A stall
  current and 1.3 kg-cm stall torque.

The observed dependence on a 5 A supply-limit setting makes the HP version the
leading hypothesis, but it is not yet confirmed. An older no-end-cap HP model,
item 2271, has similar geometry and 6.5 A extrapolated stall current.

Pololu's 48 CPR value assumes x4 quadrature decoding. The current firmware
interrupts on both edges of channel A only, so its effective resolution is
24 counts per motor revolution and approximately 232.32 counts per gearbox
output revolution. Conditional on this identification, the provisional
0--300 count range spans about 1.29 output revolutions (465 degrees). Moving to
x4 decoding later would require doubling the numerical range to preserve the
same physical travel.

## RGB camera bring-up

An Intel RealSense D435, serial `814113021800`, was detected on USB 3 through a
Genesys Logic USB 3.2 hub. Its RGB UVC interface is `/dev/video8` and advertises
YUYV resolutions through 1920x1080 at 30 fps; 1280x720 at 30 fps was selected
for the initial RGB-only test.

No frame could be acquired: every librealsense or V4L2 stream-open attempt
caused the complete upstream USB hub to disconnect and enumerate again. The
kernel repeatedly removed and recreated `/dev/video4` through `/dev/video9`;
there was no userspace process holding the RGB node. Camera testing should
resume with the D435 connected directly to a USB 3 port or after replacing the
hub/cable.

After connecting the D435 directly to the host USB 3 root port, it enumerated
stably at 5 Gbit/s. RGB-only capture from `/dev/video8` succeeded at
1280x720, YUYV, 30 fps. Auto-exposure and white balance settled after roughly
2 s. The initial view showed the lab shelves and a chair, with a nearby beige
surface occluding approximately the lower half of the image. A live RGB-only
preview was then started without enabling depth or infrared streams.

### Dual RGB maximum-throughput test

A second D435 was connected directly to the same USB 3 root controller on a
separate 5 Gbit/s port. Stable RGB mappings are:

| Serial | USB path | RGB node |
|---|---|---|
| `814113021800` | `usb-0:3` | `/dev/video8` |
| `201623028735` | `usb-0:1` | `/dev/video14` |

Both RGB streams were run simultaneously as uncompressed YUYV at 1920x1080,
30 fps, with depth and infrared disabled. Each delivered exactly 300 frames
over 10 s at 30 fps with no reported drops, corrupt frames, or USB resets.
This is approximately 995 Mbit/s per stream, or 1.99 Gbit/s total payload.

The first camera's lower field of view was substantially occluded by a nearby
beige surface. The second camera faced a wall/window region with a saturated
light reflection. Two live 1080p30 RGB previews were opened for mechanical
aiming; useful ArUco testing requires repositioning both views toward the arm
workspace and removing the occlusion/glare.

### Initial dual-camera ArUco detection

OpenCV 4.13 detected the installed markers as `DICT_4X4_50`. A live RGB-only
1080p30 dual-camera preview with subpixel corner refinement and ID overlays was
started. Camera `814113021800` detected IDs 2 and 3 consistently and ID 17
intermittently; camera `201623028735` detected ID 2. The remaining markers were
not visible/detectable in the initial arrangement, so camera/marker placement
still requires adjustment before pose reconstruction.

The operator then specified the actual marker set: IDs 2, 3, and 4 from
`DICT_4X4_50`, with 30x30 mm physical size. The preview was restricted to
these expected IDs, eliminating intermittent false decodes such as 17 and 37.
After repositioning, both cameras detected all three expected IDs in some
frames, but visibility was still intermittent. A fixed-pose detection-rate
measurement is required after the cameras and arm stop moving.
## 2026-09-20 — Two-axis firmware activation

With the external motor rail disconnected, the connected Bluno was updated
from firmware protocol version 1 to version 2. The verified allocation is:

```text
axis 0 -> shield M2, encoder D2/D3, current A0
axis 1 -> shield M1, encoder D11/D12 (PCINT3/4), current A1
```

Axis 1 uses a pin-change interrupt on encoder channel A and samples channel B,
matching the x2 decoding method used for axis 0. Both fields in `CMD` are now
active; the 300 ms watchdog, STOP command and independent provisional 0--300
encoder limits remain enabled. The legacy bounded `STEP` command remains
axis-0-only.

The firmware compiled for `arduino:avr:uno` using 7800 bytes of flash and 425
bytes of RAM, uploaded successfully on `/dev/ttyACM0`, and reported:

```text
BOOT,BLUNO_MOTOR_CONTROLLER,2,MAX_PWM,255
STATE,...,0,0,514,513,-2147483648,-2147483648,16
PONG,42
```

Thus both ACS712 raw channels are live and near their zero-current midpoint.
Encoder-2 polarity and count response still require a manual-shaft test before
powered axis-1 motion.

The initial A2/A3 allocation was superseded after an all-pin transition scan.
With motor 1 stationary and motor 2 rotated manually, the scanner measured
`D2=0`, `D3=0`, `D11=135`, and `D12=136`, with zero transitions on D13 and
A2--A5. Encoder 2 is therefore assigned to D11/D12. The briefly reported
D5/D6 wiring was rejected because those pins are respectively M1 direction 2
and the shield's global enable.

A powered 300 ms axis-1 pulse at `+60/255` produced no breakaway but changed
A1 from 517 to 537 ADC counts. At `+120/255`, raw encoder 1 increased from 0
to 48 counts (49-count peak), establishing that positive M1 command is the
non-contraction direction. The axis-1 operational convention is therefore
`q1 = -encoder1_raw`, with negative M1 command increasing contraction. Limit
checks use an explicit command-to-operational-direction sign so this reversed
command convention is handled independently from encoder polarity.

## 2026-09-21 — ArUco state and single-input control scaffold

The first ROSAIA-derived perception/control boundary was implemented without
connecting it to motor commands. `aruco_shape_tracker` detects `DICT_4X4_50`
markers from the D435 RGB node and maps the three ordered moving-marker centers
into the projective frame of each base marker. The provisional configurable
mapping is base 0 with markers 2/3/4 for arm 1 and base 1 with markers 5/6/7
for arm 2.

A live test on RealSense `201623028735` detected IDs 0--7 in one frame and
published complete `/arm_1/aruco_state` and `/arm_2/aruco_state` point clouds.
The large normalized scale of the second group means the arm-2 association
must be physically confirmed before data acquisition.

`rosaia_shape_control` implements the one-actuator damped resolved-rate law
and publishes only encoder-velocity references. It publishes zero when the
state, goal, or 6x1 Jacobian is absent, stale, invalid, or degenerate. It has
no connection to the raw PWM topic. Ten geometry/control unit tests passed,
and all three ROS packages built successfully under Jazzy.

## 2026-09-22 — Single-camera RGB-D reconstruction

RealSense D435 `152122077644` was tested at synchronized 1280x720x30 RGB and
Z16 depth, with depth aligned to colour. In a stationary 10.44 s acquisition,
all eight markers had valid robust depth whenever detected; complete two-arm
frames were available in 97.08% of frames. Unfiltered standard deviations were
approximately 0.1--0.6 mm laterally and 0.4--1.5 mm in depth.

The tracker now deprojects robust inner-marker depth into metric 3-D and runs a
constant-velocity Kalman filter per marker. It publishes a training state only
when both base markers and all three markers of that arm have current RGB-D
measurements; it never substitutes prediction-only points. Raw camera-frame
3-D points remain available on `/arm_N/aruco_points_3d_raw`.

A first implementation used PnP orientation from each 30 mm base marker. The
live test exposed the planar-pose ambiguity clearly: arm 2 acquired a false
tilt reaching -0.46 m in local z despite coherent raw depths. This was rejected.
The final tested frame instead uses the long 3-D baseline between base IDs 0
and 1 as common x, camera-down orthogonalized against it as y, and a separate
origin for each arm. Live values then became symmetric and physically
plausible: distal y was 0.547 m and 0.539 m, with local z -0.027 m and -0.045 m
for arms 1 and 2 respectively. The ROS state rate increased from about 22.5 Hz
to 28.6 Hz by skipping annotated JPEG encoding when no viewer is subscribed.
Arm-plane offset/normal and final Kalman covariances remain calibration outputs.

Before home/range acquisition, firmware protocol version 3 and recorder schema
version 3 were prepared. Firmware telemetry now includes both commands actually
present at the PWM outputs after local limit/watchdog logic. The recorder takes
the first valid encoder pair as the explicit session zero, writes it into
metadata and every visual row, and the learning loader subtracts it during
timestamp alignment. This removes the previous ambiguity caused by USB reset
and by logging the requested ROS command after firmware had already stopped.

### First powered arm-1 range cycle

The powered calibration used 6.75 V with a 7 A aggregate supply limit shared
by both motors. At home, power-off and power-on ADC baselines were identical:
A0=516 and A1=514, with both encoders stationary. An axis-0 contraction request
bounded to 5 counts at PWM 120 did not break away in 2.98 s; raw encoder motion
was only 0--2 counts and filtered A0 fell to 480 from a 515 baseline.

At PWM 200, negative M2 command produced visible contraction. A subsequent
cycle moved raw axis 0 from -2 to +12 and an opposite PWM-160 bounded move
returned it exactly to raw zero. The operator confirmed the physical
contraction. Therefore arm 1 uses contraction coordinate `q0=encoder0_raw`,
with negative command increasing q0. Firmware position-limit signs and all ROS
recorder/model configurations were updated accordingly; arm 2 remains
provisional at `q1=-encoder1_raw`.

A non-actuating arm-1 velocity reference was added for the requested cycle:
10 s outward with a 3 s rise, 4 s plateau and 3 s fall, peak 30 counts/s
(210-count outward area), 1 s dwell, then a sign-reversed 10 s return. The
complete cycle contains 1051 samples at 50 Hz, lasts 21 s and integrates to
zero net encoder displacement. Execution awaits the encoder-velocity loop; it
is intentionally not mapped directly to PWM because measured breakaway is
strongly nonlinear.

### First closed-loop arm-1 velocity cycle

The 21 s trapezoidal reference was executed from encoder home with axis 1 held
at zero. The first invocation exposed a ROS node API name collision in the
profile player (`publishers` is a read-only `Node` property); no reference was
published and no motor moved. Renaming the member to `reference_publishers`
fixed the player before the powered run.

The completed run reached `q0=256` counts and returned to `q0=1`, while axis 1
remained at zero throughout. The applied axis-0 PWM and ADC ranges were
-166--+149 and 478--555 counts respectively; the idle ADC baseline was about
516. Firmware flags were 16 (unknown current scale) except for 75 samples with
flag 80, which is flag 16 plus the expected lower position-limit clamp after
the return reached home. The controller subsequently published zero PWM.

RGB-D recording remained complete for both arms at approximately 21.7 Hz per
arm during the run. At peak encoder contraction, the three arm-1 marker
displacement norms relative to the initial home sample were approximately
41.7, 92.0 and 21.2 mm. The session is stored locally as
`data/sessions/velocity_arm1_trapezoid01_20260922` (ignored by Git). Before
collecting the training excitation set, an explicit velocity-reference field
was added to recorder schema 4. The controller was also updated with
direction-change integral reset and conditional-integration anti-windup. An
initial 2-count directional guard removed command chatter but allowed inertial
coast to q=-3 and q=303; the measured 6-count coast led to a 10-count guard at
both position limits.

The follow-up cycle with the 10-count guard reached raw operational `q0=300`
(302 session-relative counts because the recorder started at raw -2), stopped
without chatter, and left axis 1 at zero. Release stopped at raw q0=10 by
design. Firmware-bounded calibration moves then returned it to raw q0=2. This
also reconfirmed that STEP delta sign selects motor direction, not operational
q direction: positive axis-0 STEP releases and negative STEP contracts.
