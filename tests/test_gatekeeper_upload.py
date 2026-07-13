import contextlib
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


UPLOADER_DIR = Path(__file__).resolve().parents[1] / "firmware_uploader"
sys.path.insert(0, str(UPLOADER_DIR))

import gatekeeper_upload as upload  # noqa: E402


def valid_calibration_state():
    return {
        "schema_version": 1,
        "saved_at": "2026-07-10T00:00:00+00:00",
        "source_environment": "GATEKEEPER",
        "serial_number": "DA_2026_ABC",
        "firmware_version": "test",
        "device_id": "GateKeeper 1.0",
        "dac_channel_count": 8,
        "adc_channel_count": 8,
        "dac_offsets": [0.0] * 8,
        "dac_gains": [1.0] * 8,
        "adc_zero_scale": [0x800000] * 8,
        "adc_full_scale": [0x200000] * 8,
    }


class SerialNumberTests(unittest.TestCase):
    def test_suffix_is_padded_and_restricted_to_safe_ascii(self):
        with mock.patch.object(upload, "current_serial_prefix", return_value="DA_2026"):
            self.assertEqual(upload.serial_from_suffix("7"), "DA_2026_007")
            self.assertEqual(upload.serial_from_suffix("A_b"), "DA_2026_A_b")
            for suffix in ("", "ABCD", "A/B", "é"):
                with self.subTest(suffix=suffix):
                    with self.assertRaises(RuntimeError):
                        upload.serial_from_suffix(suffix)

    def test_binary_patch_requires_one_complete_marker(self):
        marker = upload.SERIAL_MARKER
        field = marker + b"DA_2025_OLD\x00"
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "firmware.bin"
            path.write_bytes(b"prefix" + field + b"suffix")
            original_size = path.stat().st_size

            upload.patch_binary_serial(path, "DA_2026_NEW")

            self.assertEqual(path.stat().st_size, original_size)
            self.assertEqual(upload.read_binary_serial(path), "DA_2026_NEW")

            path.write_bytes(field + field)
            with self.assertRaisesRegex(RuntimeError, "exactly one"):
                upload.patch_binary_serial(path, "DA_2026_NEW")

            path.write_bytes(marker)
            with self.assertRaisesRegex(RuntimeError, "truncated"):
                upload.patch_binary_serial(path, "DA_2026_NEW")

    def test_dfu_read_ranges_cover_the_linked_flash_regions(self):
        self.assertEqual(upload.M7_READ_ADDRESS, "0x08040000:0x000C0000")
        self.assertEqual(upload.M4_READ_ADDRESS, "0x08100000:0x00100000")


class CalibrationTests(unittest.TestCase):
    def test_validation_rejects_values_firmware_cannot_apply(self):
        state = valid_calibration_state()
        upload.validate_calibration_state(state)

        bad_states = []
        for key, index, value in (
            ("dac_gains", 0, 0.0),
            ("dac_offsets", 0, float("nan")),
            ("adc_zero_scale", 0, -1),
            ("adc_full_scale", 0, 0),
            ("adc_full_scale", 0, 0x1000000),
        ):
            candidate = json.loads(json.dumps(state))
            candidate[key][index] = value
            bad_states.append((key, value, candidate))

        for key, value, candidate in bad_states:
            with self.subTest(key=key, value=value):
                with self.assertRaises(RuntimeError):
                    upload.validate_calibration_state(candidate)

        malformed = valid_calibration_state()
        malformed["serial_number"] = 123
        with self.assertRaises(RuntimeError):
            upload.validate_calibration_state(malformed)

    def test_restore_updates_live_and_saved_adc_calibration(self):
        state = valid_calibration_state()
        serial = object()
        commands = []

        @contextlib.contextmanager
        def fake_port(_port):
            yield serial

        def record_command(actual_serial, command, timeout=None):
            self.assertIs(actual_serial, serial)
            commands.append(command)
            return "OK"

        with (
            mock.patch.object(upload, "open_command_port", fake_port),
            mock.patch.object(upload, "wait_for_ready"),
            mock.patch.object(upload, "send_command", record_command),
            mock.patch.object(upload.time, "sleep"),
        ):
            upload.restore_calibration("COM1", state)

        self.assertTrue(any(c.startswith("SET_ZERO_SCALE_CAL,") for c in commands))
        self.assertTrue(any(c.startswith("SET_FULL_SCALE_CAL,") for c in commands))
        self.assertFalse(any(c.startswith("SET_SAVED_") for c in commands))

    def test_timeout_context_is_safe_when_posix_signals_are_unavailable(self):
        with (
            mock.patch.object(upload.signal, "SIGALRM", create=True) as sigalrm,
            mock.patch.object(upload.signal, "setitimer", create=True) as setitimer,
            mock.patch.object(upload.threading, "current_thread"),
        ):
            # A non-main thread deliberately takes the portable no-signal path.
            upload.threading.current_thread.return_value = object()
            with upload.operation_timeout(1, "test"):
                pass
            sigalrm.assert_not_called()
            setitimer.assert_not_called()


if __name__ == "__main__":
    unittest.main()
