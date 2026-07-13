from __future__ import annotations

import math
import time
from typing import Iterable

import numpy as np
import serial
import serial.tools.list_ports


GATEKEEPER_VID = 0x2341
GATEKEEPER_PID = 0x0266
NUM_CHANNELS = 8

BLOCKED_COMMANDS = {
    "HARD_RESET",
    "HARD_RESET_CALIBRATION",
    "CALIBRATE_ADC_CHANNEL_ZERO_SCALE",
    "CALIBRATE_ALL_ADC_CHANNELS_ZERO_SCALE",
    "CALIBRATE_ADC_CHANNEL_FULL_SCALE",
    "CALIBRATE_ALL_ADC_CHANNELS_FULL_SCALE",
    "SET_SAVED_ZERO_SCALE_CAL",
    "SET_SAVED_FULL_SCALE_CAL",
    "SET_ZERO_SCALE_CAL",
    "SET_FULL_SCALE_CAL",
    "SET_OSG",
    "DAC_CH_CAL",
}


class GatekeeperResponseError(RuntimeError):
    pass


def detect_port() -> str | None:
    ports = [
        port.device
        for port in serial.tools.list_ports.comports()
        if port.vid == GATEKEEPER_VID and port.pid == GATEKEEPER_PID
    ]
    if len(ports) == 1:
        return ports[0]
    if not ports:
        return None
    raise RuntimeError(f"Multiple Gatekeepers detected: {ports}")


def _argument(value) -> str:
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    if isinstance(value, (float, np.floating)):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"Non-finite command argument: {value}")
        return f"{value:.9g}"
    return str(value)


def command_text(name: str, *args) -> str:
    return name if not args else name + "," + ",".join(_argument(x) for x in args)


class GatekeeperClient:
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.serial = serial.Serial(
            port, baudrate=baudrate, timeout=0.03, write_timeout=1.0
        )
        time.sleep(0.25)
        self.serial.reset_input_buffer()

    def close(self):
        self.serial.close()

    def _write(self, name: str, *args):
        upper_name = name.upper()
        if upper_name in BLOCKED_COMMANDS:
            raise RuntimeError(f"Test suite refuses destructive command {upper_name}")
        self.serial.write((command_text(name, *args) + "\n").encode("ascii"))
        self.serial.flush()

    def _read_line(self, timeout: float = 2.0) -> str:
        deadline = time.monotonic() + timeout
        data = bytearray()
        while time.monotonic() < deadline:
            byte = self.serial.read(1)
            if byte:
                data.extend(byte)
                if byte == b"\n":
                    return data.decode("utf-8", errors="replace").strip()
        raise TimeoutError("Timed out waiting for Gatekeeper response")

    def query_raw(self, name: str, *args, timeout: float = 2.0) -> str:
        self.serial.reset_input_buffer()
        self._write(name, *args)
        return self._read_line(timeout)

    def start_command(self, name: str, *args):
        self.serial.reset_input_buffer()
        self._write(name, *args)

    def query(self, name: str, *args, timeout: float = 2.0) -> str:
        response = self.query_raw(name, *args, timeout=timeout)
        if response.startswith("FAILURE:"):
            raise GatekeeperResponseError(
                f"{command_text(name, *args)} returned {response}"
            )
        return response

    def command_no_reply(self, name: str, *args, wait: float = 0.15):
        self.serial.reset_input_buffer()
        self._write(name, *args)
        time.sleep(wait)
        response = self._drain_text(timeout=0.25)
        if response.startswith("FAILURE:"):
            raise GatekeeperResponseError(
                f"{command_text(name, *args)} returned {response}"
            )
        return response

    def query_float(self, name: str, *args, timeout: float = 2.0) -> float:
        return float(self.query(name, *args, timeout=timeout))

    def query_int(self, name: str, *args, timeout: float = 2.0) -> int:
        return int(self.query(name, *args, timeout=timeout))

    def query_multiline(self, name: str, *args, timeout: float = 2.0) -> str:
        self.serial.reset_input_buffer()
        self._write(name, *args)
        return self._drain_text(timeout=timeout)

    def _drain_text(self, timeout: float = 0.5, idle: float = 0.08) -> str:
        deadline = time.monotonic() + timeout
        last_data = time.monotonic()
        data = bytearray()
        while time.monotonic() < deadline:
            chunk = self.serial.read(max(1, self.serial.in_waiting))
            if chunk:
                data.extend(chunk)
                last_data = time.monotonic()
            elif data and time.monotonic() - last_data >= idle:
                break
        return data.decode("utf-8", errors="replace").strip()

    def _read_binary_or_failure(self, byte_count: int, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        data = bytearray()
        prefix = b"FAILURE:"
        while time.monotonic() < deadline and (
            len(data) < byte_count or prefix.startswith(data)
        ):
            prefix_candidate = prefix.startswith(data)
            chunk = self.serial.read(1 if prefix_candidate else byte_count - len(data))
            if not chunk:
                continue
            data.extend(chunk)
            if data.startswith(prefix):
                while time.monotonic() < deadline and not data.endswith(b"\n"):
                    data.extend(self.serial.read(1))
                raise GatekeeperResponseError(
                    data.decode("utf-8", errors="replace").strip()
                )
        if len(data) != byte_count:
            raise TimeoutError(f"Read {len(data)} of {byte_count} binary bytes")
        return bytes(data)

    def binary_command(
        self,
        name: str,
        args: Iterable,
        frames: int,
        channels: int,
        timeout: float = 10.0,
    ) -> np.ndarray:
        self.serial.reset_input_buffer()
        self._write(name, *list(args))
        try:
            raw = self._read_binary_or_failure(frames * channels * 4, timeout)
        except (TimeoutError, GatekeeperResponseError):
            self.stop()
            raise
        trailer = self._drain_text()
        if trailer.startswith("FAILURE:"):
            raise GatekeeperResponseError(trailer)
        return np.frombuffer(raw, dtype="<f4").astype(float).reshape(frames, channels)

    def time_series_adc_read(
        self,
        channels: list[int],
        conversion_us: int,
        duration_us: int,
        timeout: float = 10.0,
    ) -> tuple[float, np.ndarray]:
        self.serial.reset_input_buffer()
        self._write(
            "TIME_SERIES_ADC_READ",
            len(channels),
            *channels,
            conversion_us,
            duration_us,
        )
        try:
            period_raw = self._read_binary_or_failure(4, timeout)
        except (TimeoutError, GatekeeperResponseError):
            self.stop()
            raise
        period_us = float(np.frombuffer(period_raw, dtype="<f4")[0])
        if not math.isfinite(period_us) or period_us <= 0:
            raise GatekeeperResponseError(f"Invalid sample period {period_us}")
        frames = int(duration_us // int(period_us))
        try:
            raw = self._read_binary_or_failure(
                frames * len(channels) * 4, timeout
            )
        except (TimeoutError, GatekeeperResponseError):
            self.stop()
            raise
        trailer = self._drain_text()
        if trailer.startswith("FAILURE:"):
            raise GatekeeperResponseError(trailer)
        data = np.frombuffer(raw, dtype="<f4").astype(float)
        return period_us, data.reshape(frames, len(channels))

    def stop(self):
        self.serial.write(b"stop\n")
        self.serial.flush()
        return self._drain_text(timeout=2.0)
