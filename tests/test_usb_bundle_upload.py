import contextlib
import importlib.util
import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "platformio_tools"
    / "usb_bundle_upload.py"
)
SPEC = importlib.util.spec_from_file_location("usb_bundle_upload", SCRIPT)
bundle = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bundle)


class FakePersistence:
    def __init__(self, environment="GATEKEEPER", serial="DA_2026_ABC"):
        self.environment = environment
        self.serial = serial

    def wait_for_device_ready(self, _env, expected_serial_number=None):
        return "COM1"

    @contextlib.contextmanager
    def open_command_port(self, port):
        yield port

    def send_command(self, _serial_port, command):
        fixed = {
            "NOP": "NOP",
            "*RDY?": "READY",
            "GET_ENVIRONMENT": self.environment,
            "SERIAL_NUMBER": self.serial,
            "*IDN?": "GateKeeper DAC/ADC",
            "GET_FIRMWARE_VERSION": "test-version",
        }
        if command in fixed:
            return fixed[command]
        name, channel = command.split(",", 1)
        if name == "GET_REVISION_REG":
            return "34"
        if name in ("GET_DAC", "GET_ADC"):
            return "0.0"
        if name in ("GET_SAVED_ZERO_SCALE_CAL", "GET_ZERO_SCALE_CAL"):
            return str(0x800000 + int(channel))
        if name in ("GET_SAVED_FULL_SCALE_CAL", "GET_FULL_SCALE_CAL"):
            return str(0x500000 + int(channel))
        raise AssertionError(f"Unexpected command: {command}")


class BasicFirmwareVerificationTests(unittest.TestCase):
    def test_accepts_expected_identity(self):
        port = bundle.verify_basic_firmware(
            FakePersistence(), object(), "DA_2026_ABC"
        )
        self.assertEqual(port, "COM1")

    def test_rejects_wrong_environment(self):
        with self.assertRaisesRegex(RuntimeError, "environment"):
            bundle.verify_basic_firmware(
                FakePersistence(environment="OTHER"), object(), "DA_2026_ABC"
            )


if __name__ == "__main__":
    unittest.main()
