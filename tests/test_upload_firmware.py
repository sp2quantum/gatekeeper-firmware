import sys
import unittest
from pathlib import Path
from unittest import mock


UPLOADER_DIR = Path(__file__).resolve().parents[1] / "firmware_uploader"
sys.path.insert(0, str(UPLOADER_DIR))

import upload_firmware as uploader  # noqa: E402


class UploadConfirmationTests(unittest.TestCase):
    def test_calibration_backup_supplies_the_serial_without_an_extra_read(self):
        state = {"serial_number": "DA_2025_XYZ"}
        with (
            mock.patch.object(
                uploader, "serial_with_current_year", return_value="DA_2026_XYZ"
            ),
            mock.patch.object(uploader, "open_command_port") as open_port,
        ):
            serial_number = uploader.resolve_serial_number("COM8", state)

        self.assertEqual(serial_number, "DA_2026_XYZ")
        open_port.assert_not_called()

    def test_unreadable_serial_requires_confirmation_before_using_default(self):
        with (
            mock.patch.object(
                uploader,
                "default_serial_with_current_year",
                return_value="DA_2026_ABC",
            ),
            mock.patch.object(
                uploader,
                "open_command_port",
                side_effect=OSError("unresponsive"),
            ),
            mock.patch("builtins.input", return_value=""),
        ):
            with self.assertRaisesRegex(SystemExit, "serial number was not overridden"):
                uploader.resolve_serial_number("COM8")

        with (
            mock.patch.object(
                uploader,
                "default_serial_with_current_year",
                return_value="DA_2026_ABC",
            ),
            mock.patch.object(
                uploader,
                "open_command_port",
                side_effect=OSError("unresponsive"),
            ),
            mock.patch("builtins.input", return_value="yes"),
        ):
            self.assertEqual(uploader.resolve_serial_number("COM8"), "DA_2026_ABC")

    def test_unreadable_calibration_requires_confirmation_before_continuing(self):
        with (
            mock.patch.object(
                uploader,
                "backup_device_state",
                side_effect=OSError("unresponsive"),
            ),
            mock.patch("builtins.input", return_value="no"),
        ):
            with self.assertRaisesRegex(
                SystemExit, "calibration data was not overridden"
            ):
                uploader.backup_calibration_if_available("COM8")

        with (
            mock.patch.object(
                uploader,
                "backup_device_state",
                side_effect=OSError("unresponsive"),
            ),
            mock.patch("builtins.input", return_value="y"),
        ):
            self.assertIsNone(uploader.backup_calibration_if_available("COM8"))

    def test_positional_serial_suffix_is_rejected(self):
        with mock.patch.object(sys, "argv", ["upload_firmware.py", "XYZ"]):
            with self.assertRaises(SystemExit) as raised:
                uploader.main()
        self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
