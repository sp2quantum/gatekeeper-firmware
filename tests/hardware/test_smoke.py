import importlib.util
from pathlib import Path

import pytest

from .conftest import calibration_snapshot
from .protocol import NUM_CHANNELS


pytestmark = pytest.mark.hardware


def _load_post_flash_health_checks():
    path = (
        Path(__file__).resolve().parents[2]
        / "firmware_uploader"
        / "post_flash_health_checks.py"
    )
    spec = importlib.util.spec_from_file_location("post_flash_health_checks", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.post_flash_health_checks
def test_read_only_post_flash_health_checks(gatekeeper):
    health_checks = _load_post_flash_health_checks()

    def send(command):
        name, *args = command.split(",")
        return gatekeeper.query(name, *args)

    results = health_checks.run_post_flash_health_checks(send)
    assert results["environment"] == "GATEKEEPER"
    assert results["serial_number"].startswith("DA_")


def test_registry_and_argument_validation(gatekeeper):
    registry = gatekeeper.query_multiline("PRINT_FUNCTIONS")
    required = {
        "DAC_LED_BUFFER_RAMP",
        "TIME_SERIES_BUFFER_RAMP",
        "2D_DAC_LED_BUFFER_RAMP",
        "2D_TIME_SERIES_BUFFER_RAMP",
        "TIME_SERIES_ADC_READ",
        "AWG_WITH_ADC",
        "BOXCAR_BUFFER_RAMP",
    }
    registered = {
        line.split(",", 1)[0].strip()
        for line in registry.splitlines()
        if "," in line and not line.startswith("Available")
    }
    assert required <= registered

    for name, args in (
        ("SET", (0.5, 0)),
        ("SET", (-1, 0)),
        ("SET", (0, 1e100)),
        ("AWG_BUFFER_RAMP", (1, 1, 20_000_000, 0, 0)),
    ):
        assert gatekeeper.query_raw(name, *args).startswith("FAILURE:")


def test_initialize_and_reset_preserve_adc_calibration(gatekeeper):
    saved_before = calibration_snapshot(gatekeeper, saved=True)
    live_before = calibration_snapshot(gatekeeper, saved=False)

    assert gatekeeper.query("INITIALIZE") == "INITIALIZATION COMPLETE"
    assert calibration_snapshot(gatekeeper, saved=True) == saved_before
    assert calibration_snapshot(gatekeeper, saved=False) == live_before

    assert gatekeeper.command_no_reply("RESET", wait=0.25) == ""
    assert calibration_snapshot(gatekeeper, saved=True) == saved_before
    assert calibration_snapshot(gatekeeper, saved=False) == live_before


def test_filter_word_respects_chopping_mode(gatekeeper):
    gatekeeper.command_no_reply("SET_CHOP", 0)
    assert gatekeeper.query_raw("CONVERT_TIME_FW", 0, 2).startswith("FAILURE:")
    assert float(gatekeeper.query("CONVERT_TIME_FW", 0, 3)) > 0
    assert gatekeeper.query("GET_CHOP") == "false"

    gatekeeper.command_no_reply("SET_CHOP", 1)
    assert float(gatekeeper.query("CONVERT_TIME_FW", 0, 2)) > 0
    assert gatekeeper.query("GET_CHOP") == "true"


def test_all_peripheral_channels_respond(gatekeeper):
    assert [gatekeeper.query_int("GET_REVISION_REG", board) for board in range(2)] == [34, 34]
    for channel in range(NUM_CHANNELS):
        assert -10.1 <= gatekeeper.query_float("GET_DAC", channel) <= 10.1
        assert -10.5 <= gatekeeper.query_float("GET_ADC", channel) <= 10.5
