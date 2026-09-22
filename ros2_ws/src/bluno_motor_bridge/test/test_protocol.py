"""Unit tests for the ROS-independent serial protocol codec."""

from bluno_motor_bridge.protocol import (
    CURRENT_UNAVAILABLE_MA,
    encode_command,
    encode_stop,
    parse_state,
    ProtocolError,
)
import pytest


def test_encode_command() -> None:
    assert encode_command(152, (80, -40)) == b'CMD,152,80,-40\n'
    assert encode_stop() == b'STOP\n'


@pytest.mark.parametrize(
    'sequence,commands',
    [(-1, (0, 0)), (0, (256, 0)), (0, (0, -256)), (0, (0,)), (0, (1.2, 0))],
)
def test_encode_command_rejects_invalid_values(sequence, commands) -> None:
    with pytest.raises(ProtocolError):
        encode_command(sequence, commands)


def test_parse_state() -> None:
    packet = parse_state(b'STATE,152,38124930,12584,-3402,522,507,410,-120,0\r\n')
    assert packet.sequence == 152
    assert packet.mcu_time_us == 38124930
    assert packet.encoder_counts == (12584, -3402)
    assert packet.raw_adc == (522, 507)
    assert packet.current_ma == (410, -120)
    assert packet.flags == 0


def test_current_unavailable_sentinel_is_valid_telemetry() -> None:
    packet = parse_state(
        'STATE,1,2,0,0,512,0,-2147483648,-2147483648,24'
    )
    assert packet.current_ma == (
        CURRENT_UNAVAILABLE_MA,
        CURRENT_UNAVAILABLE_MA,
    )


@pytest.mark.parametrize(
    'line',
    [
        '',
        'DEBUG,hello',
        'STATE,1,2,3',
        'STATE,1,2,3,4,1024,0,0,0,0',
        'STATE,1,2,3,4,0,-1,0,0,0',
        'STATE,a,2,3,4,0,0,0,0,0',
    ],
)
def test_parse_state_rejects_malformed_packets(line) -> None:
    with pytest.raises(ProtocolError):
        parse_state(line)
