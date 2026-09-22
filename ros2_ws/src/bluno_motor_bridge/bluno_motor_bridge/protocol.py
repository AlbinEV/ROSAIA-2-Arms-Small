"""Pure-Python codec for the line-oriented Bluno serial protocol."""

from dataclasses import asdict, dataclass


AXIS_COUNT = 2
MIN_FIRMWARE_COMMAND = -255
MAX_FIRMWARE_COMMAND = 255
CURRENT_UNAVAILABLE_MA = -(2**31)


class ProtocolError(ValueError):
    """Raised when a serial packet violates the protocol contract."""


@dataclass(frozen=True)
class StatePacket:
    """Decoded telemetry produced by the Bluno firmware."""

    sequence: int
    mcu_time_us: int
    encoder_counts: tuple[int, int]
    raw_adc: tuple[int, int]
    current_ma: tuple[int, int]
    flags: int

    def as_dict(self) -> dict:
        """Return a JSON-serializable representation."""
        return asdict(self)


def encode_command(sequence: int, commands: tuple[int, int]) -> bytes:
    """Encode one command after strict type and range validation."""
    if len(commands) != AXIS_COUNT:
        raise ProtocolError(f'expected {AXIS_COUNT} commands, got {len(commands)}')
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ProtocolError('sequence must be a non-negative integer')

    checked = []
    for command in commands:
        if isinstance(command, bool) or not isinstance(command, int):
            raise ProtocolError('commands must be integers')
        if not MIN_FIRMWARE_COMMAND <= command <= MAX_FIRMWARE_COMMAND:
            raise ProtocolError(f'command {command} is outside [-255, 255]')
        checked.append(command)

    return f'CMD,{sequence},{checked[0]},{checked[1]}\n'.encode('ascii')


def encode_stop() -> bytes:
    """Encode the unconditional stop command."""
    return b'STOP\n'


def parse_state(line: str | bytes) -> StatePacket:
    """Decode a STATE packet and reject malformed or out-of-range fields."""
    if isinstance(line, bytes):
        try:
            line = line.decode('ascii')
        except UnicodeDecodeError as exc:
            raise ProtocolError('packet is not ASCII') from exc

    fields = line.strip().split(',')
    if len(fields) != 10 or fields[0] != 'STATE':
        raise ProtocolError('expected STATE with 9 numeric fields')

    try:
        values = tuple(int(value, 10) for value in fields[1:])
    except ValueError as exc:
        raise ProtocolError('STATE contains a non-integer field') from exc

    sequence, mcu_time_us, enc0, enc1, adc0, adc1, ma0, ma1, flags = values
    if sequence < 0 or mcu_time_us < 0:
        raise ProtocolError('sequence and MCU time must be non-negative')
    if not (0 <= adc0 <= 1023 and 0 <= adc1 <= 1023):
        raise ProtocolError('ADC value is outside the ATmega328 10-bit range')
    if flags < 0:
        raise ProtocolError('flags must be non-negative')

    return StatePacket(
        sequence=sequence,
        mcu_time_us=mcu_time_us,
        encoder_counts=(enc0, enc1),
        raw_adc=(adc0, adc1),
        current_ma=(ma0, ma1),
        flags=flags,
    )
