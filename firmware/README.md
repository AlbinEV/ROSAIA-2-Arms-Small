# Bluno firmware

`bluno_motor_controller/bluno_motor_controller.ino` is the dedicated firmware
for the Bluno/ATmega328P and Makerfabs H-Bridge Motor Shield.

## Current bench-test build

The checked-in build limits the magnitude of every command:

```cpp
constexpr int16_t MAX_ABS_COMMAND = 255;
```

Axis 0 is assigned to shield channel M2 and axis 1 to M1. Both axes accept
independent signed commands. Current in amperes is unavailable until the
ACS712 variant and sensitivity are measured; raw A0/A1 ADC telemetry remains
available for both channels.

Axis 0 encoder wiring is channel A on D2 and channel B on D3. The firmware
counts both edges of channel A and samples channel B to determine direction.
Axis 1 uses channel A on D11/PCINT3 and channel B on D12/PCINT4; a pin-change
interrupt on D11 provides the same x2 decoding scheme.

For small bench motions, `STEP,<sequence>,<delta_counts>,<pwm>` performs a
relative encoder move locally on the Bluno. The checked-in build accepts at
most 50 counts per request and PWM at most 255 during loaded breakaway
calibration. It removes motor drive as soon as the requested count magnitude is reached,
applies a 3 s step timeout, and retains the independent 300 ms watchdog for
continuous commands.

`STEP_AXIS,<sequence>,<axis>,<delta_counts>,<pwm>` provides the same bounded
raw-encoder move for axis 0 or axis 1. Unlike `CMD`, this calibration primitive
does not apply the provisional operational-position bounds: its own 50-count
displacement limit and 3 s timeout terminate the movement. This allows a known
relative return move after the USB serial connection has reset both counters.

During sign calibration, the sign of `delta_counts` selects motor direction
and its magnitude sets the displacement limit. Termination uses the absolute
encoder displacement from the starting count, so an unknown encoder polarity
cannot make the motion run past the requested count magnitude.

The provisional axis-0 operational coordinate is `q0 = -encoder0_raw`.
Axis 1 uses `q1 = -encoder1_raw`: its negative motor command produces
contraction and increases `q1`.
Firmware enforces `0 <= q <= 300` independently on both axes. `ZERO_ENC`
defines the current mechanical pose as zero for both encoders. The legacy
`STEP` primitive remains axis-0-only; `STEP_AXIS` addresses either axis and
continuous `CMD` packets drive both.

## Local toolchain

Arduino CLI and the AVR core are installed in ignored local directories. Build
from the repository root with:

```bash
.tools/arduino-cli compile \
  --config-file .arduino-cli.yaml \
  --fqbn arduino:avr:uno \
  --warnings all \
  firmware/bluno_motor_controller
```

Upload only while motor power is disconnected:

```bash
.tools/arduino-cli upload \
  --config-file .arduino-cli.yaml \
  --port /dev/serial/by-id/usb-2341_0043-if00 \
  --fqbn arduino:avr:uno \
  firmware/bluno_motor_controller
```

## Preserved original image

The pre-existing application flash and EEPROM are stored under the ignored
directory `backup/2026-09-19_bluno_original/`. Their checksums are recorded in
`docs/BRINGUP_LOG.md`. Do not restore them while the motor rail is powered,
because their pin mapping and stop behavior are unknown.
