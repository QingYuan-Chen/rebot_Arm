"""Exercise the installed ABI through virtual serial ports, never real hardware."""
from __future__ import annotations

from contextlib import contextmanager
import os
import select
import threading
import time

import pytest

motorbridge = pytest.importorskip("motorbridge")
pytestmark = pytest.mark.skipif(
    os.name != "posix" or motorbridge.__version__ != "0.4.7+rebotarm.1",
    reason="requires the verified v0.4.7 rebotarm.1 wheel and POSIX virtual serial ports",
)


def packet(*, can_id=0x17, dlc=8, status=0, motor_id=7, position=32767):
    payload = bytes([(status << 4) | motor_id]) + position.to_bytes(2, "big")
    payload += bytes.fromhex("7ff7ff201e")
    return bytes([0xAA, 0x11, dlc]) + can_id.to_bytes(4, "little") + payload + b"\x55"


@contextmanager
def virtual_motor():
    import pty

    master, slave = pty.openpty()
    path = os.ttyname(slave)
    assert path.startswith("/dev/pts/")
    controller = motorbridge.Controller.from_dm_serial(path, 921600)
    motor = controller.add_damiao_motor(7, 0x17, "4310")
    try:
        yield master, controller, motor
    finally:
        motor.close()
        controller.close()
        os.close(slave)
        os.close(master)


def wait_sequence(controller, motor, wanted):
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        controller.poll_feedback_once()
        state, sequence = motor.get_state_with_sequence()
        if sequence >= wanted:
            return state, sequence
        time.sleep(0.001)
    pytest.fail(f"virtual feedback did not reach sequence {wanted}")


@pytest.mark.parametrize("change", [
    {"can_id": 0x123}, {"motor_id": 6}, {"dlc": 0}, {"dlc": 1},
    {"dlc": 9}, {"dlc": 0x48}, {"dlc": 0x88},
])
def test_malformed_frames_do_not_advance_feedback_identity(change):
    with virtual_motor() as (master, controller, motor):
        os.write(master, packet())
        _, baseline = wait_sequence(controller, motor, 1)
        os.write(master, packet(position=32961, **change))
        os.write(master, packet(position=33334))
        state, sequence = wait_sequence(controller, motor, baseline + 1)
        time.sleep(0.01)
        controller.poll_feedback_once()
        state, sequence = motor.get_state_with_sequence()
        assert sequence == baseline + 1
        assert state.pos == pytest.approx(0.21610546112060547)
        assert state.arbitration_id == 0x17
        assert state.can_id == 7


@pytest.mark.parametrize("status", [0, 1, 8, 13, None])
def test_zero_requires_new_requested_disabled_status(status):
    with virtual_motor() as (master, controller, motor):
        # Even a valid cached disabled sample must not authorize the next zero.
        os.write(master, packet())
        wait_sequence(controller, motor, 1)
        stop = threading.Event()
        transmitted = bytearray()
        errors = []

        def respond():
            try:
                answered = False
                while not stop.is_set():
                    readable, _, _ = select.select([master], [], [], 0.01)
                    if not readable:
                        continue
                    transmitted.extend(os.read(master, 4096))
                    if len(transmitted) >= 30 and not answered:
                        assert transmitted[21:24] == bytes([7, 0, 0xCC])
                        answered = True
                        if status is not None:
                            os.write(master, packet(status=status))
            except BaseException as exc:
                errors.append(exc)

        thread = threading.Thread(target=respond)
        thread.start()
        try:
            if status == 0:
                motor.set_zero_position()
            else:
                with pytest.raises(motorbridge.CallError):
                    motor.set_zero_position()
            time.sleep(0.03)
        finally:
            stop.set()
            thread.join(timeout=1)
        assert not thread.is_alive()
        assert not errors
        assert len(transmitted) % 30 == 0
        payloads = [bytes(transmitted[i + 21:i + 29]) for i in range(0, len(transmitted), 30)]
        assert payloads.count(bytes.fromhex("fffffffffffffffe")) == int(status == 0)
        assert all(p == bytes.fromhex("0700cc0000000000") or p == bytes.fromhex("fffffffffffffffe") for p in payloads)


def test_c_abi_handle_ownership_errors_and_reconnect():
    """Opaque ABI handles map directly to the local driver's used operations."""
    import ctypes
    import pty
    from motorbridge.abi import CState, get_abi
    lib=get_abi().lib
    master,slave=pty.openpty()
    try:
        for _ in range(2):
            ctrl=lib.motor_controller_new_dm_serial(os.ttyname(slave).encode(),921600)
            assert ctrl
            motor=lib.motor_controller_add_damiao_motor(ctrl,7,0x17,b'4310')
            assert motor
            try:
                state,seq=CState(),ctypes.c_uint64()
                assert lib.motor_handle_get_state_with_sequence(motor,ctypes.byref(state),ctypes.byref(seq))==0
                assert state.has_value==0 and seq.value==0
                os.write(master,packet())
                deadline=time.monotonic()+.5
                while not state.has_value and time.monotonic()<deadline:
                    assert lib.motor_controller_poll_feedback_once(ctrl)==0
                    assert lib.motor_handle_get_state_with_sequence(motor,ctypes.byref(state),ctypes.byref(seq))==0
                    time.sleep(.001)
                assert state.has_value and seq.value==1
                assert lib.motor_handle_send_pos_vel(motor,0.,.3)==0
                # Invalid mode returns an error rather than silently accepting it.
                assert lib.motor_handle_ensure_mode(motor,255,1)!=0
                assert lib.motor_last_error_message()
            finally:
                lib.motor_handle_free(motor)
                assert lib.motor_controller_close_bus(ctrl)==0
                lib.motor_controller_free(ctrl)
    finally:
        os.close(slave);os.close(master)
