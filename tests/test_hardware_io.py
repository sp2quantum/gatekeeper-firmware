import unittest
from unittest import mock

from tests.hardware.protocol import GatekeeperClient, GatekeeperResponseError


class ChunkedSerial:
    def __init__(self, chunks):
        self.chunks = [bytearray(chunk) for chunk in chunks]

    def read(self, size):
        if not self.chunks:
            return b""
        chunk = self.chunks[0]
        result = bytes(chunk[:size])
        del chunk[:size]
        if not chunk:
            self.chunks.pop(0)
        return result


def reader_for(chunks):
    reader = GatekeeperClient.__new__(GatekeeperClient)
    reader.serial = ChunkedSerial(chunks)
    return reader


class BinaryResponseTests(unittest.TestCase):
    def test_failure_prefix_may_arrive_one_byte_at_a_time(self):
        reader = reader_for([b"F", b"AI", b"LURE:", b" bad\n"])
        with self.assertRaisesRegex(GatekeeperResponseError, "FAILURE: bad"):
            reader._read_binary_or_failure(16, timeout=0.2)

    def test_binary_starting_with_f_is_not_mistaken_for_failure(self):
        reader = reader_for([b"F", b"Z\x00\x01"])
        data = reader._read_binary_or_failure(4, timeout=0.2)
        self.assertEqual(data, b"FZ\x00\x01")

    def test_short_binary_response_times_out(self):
        reader = reader_for([b"\x01\x02"])
        monotonic_values = iter([0.0, 0.0, 1.0])
        with mock.patch(
            "tests.hardware.protocol.time.monotonic",
            side_effect=lambda: next(monotonic_values, 1.0),
        ):
            with self.assertRaises(TimeoutError):
                reader._read_binary_or_failure(4, timeout=0.5)


if __name__ == "__main__":
    unittest.main()
