from __future__ import annotations

from dataclasses import dataclass

import pytest
import serial

from .protocol import GatekeeperClient, NUM_CHANNELS, detect_port


def calibration_snapshot(client: GatekeeperClient, saved: bool) -> tuple[tuple[int, ...], tuple[int, ...]]:
    qualifier = "SAVED_" if saved else ""
    zero = tuple(
        client.query_int(f"GET_{qualifier}ZERO_SCALE_CAL", channel)
        for channel in range(NUM_CHANNELS)
    )
    full = tuple(
        client.query_int(f"GET_{qualifier}FULL_SCALE_CAL", channel)
        for channel in range(NUM_CHANNELS)
    )
    return zero, full


@dataclass(frozen=True)
class DeviceState:
    dac_voltages: tuple[float, ...]
    conversion_times: tuple[float, ...]
    chopping: bool
    saved_calibration: tuple[tuple[int, ...], tuple[int, ...]]
    live_calibration: tuple[tuple[int, ...], tuple[int, ...]]


def capture_state(client: GatekeeperClient) -> DeviceState:
    return DeviceState(
        dac_voltages=tuple(
            client.query_float("GET_DAC", channel) for channel in range(NUM_CHANNELS)
        ),
        conversion_times=tuple(
            client.query_float("GET_CONVERT_TIME", channel)
            for channel in range(NUM_CHANNELS)
        ),
        chopping=client.query("GET_CHOP") == "true",
        saved_calibration=calibration_snapshot(client, saved=True),
        live_calibration=calibration_snapshot(client, saved=False),
    )


def restore_state(client: GatekeeperClient, state: DeviceState):
    client.command_no_reply("SET_CHOP", state.chopping)
    for channel, conversion_time in enumerate(state.conversion_times):
        client.query("CONVERT_TIME", channel, conversion_time)
    for channel, voltage in enumerate(state.dac_voltages):
        client.query("SET", channel, voltage)


@pytest.fixture(scope="session")
def gatekeeper(request):
    port = request.config.getoption("--port") or detect_port()
    if port is None:
        pytest.skip("No Gatekeeper USB CDC device detected")
    try:
        client = GatekeeperClient(port)
    except serial.SerialException as exc:
        pytest.fail(f"Cannot open Gatekeeper on {port}: {exc}")

    state = capture_state(client)
    if state.saved_calibration != state.live_calibration:
        client.close()
        pytest.fail("Live ADC calibration does not match saved calibration before tests")

    try:
        yield client
    finally:
        restoration_error = None
        try:
            restore_state(client, state)
            saved_after = calibration_snapshot(client, saved=True)
            live_after = calibration_snapshot(client, saved=False)
            if saved_after != state.saved_calibration:
                restoration_error = "Saved ADC calibration changed during tests"
            elif live_after != state.live_calibration:
                restoration_error = "Live ADC calibration changed during tests"
        except Exception as exc:
            restoration_error = f"Failed to restore hardware state: {exc}"
        finally:
            client.close()
        if restoration_error:
            pytest.fail(restoration_error)
