#!/usr/bin/env python3
"""Characterize GateKeeper buffer-ramp timing guards.

This script talks directly to the USB CDC port and checks two things:

1. Safe normal-command timing choices complete without watchdog/overflow text.
2. Unsafe normal-command timing choices fail before streaming binary data.

It intentionally does not use the *_SUDO commands for the main assertions,
because the purpose is to verify the safety behavior users get by default.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

import serial
from serial.tools import list_ports


GATEKEEPER_VID = 0x2341
GATEKEEPER_PID = 0x0266
NUM_DAC_CHANNELS = 8
NUM_ADC_CHANNELS = 8
NUM_CHANNELS_PER_ADC_BOARD = 4


DAC_LED_TABLE = [
    [0, 120, 200, 300, 400],
    [120, 120, 220, 320, 420],
    [200, 220, 240, 320, 420],
    [300, 320, 320, 340, 440],
    [400, 420, 420, 440, 460],
]
TIME_SERIES_1D_TABLE = [
    [0, 80, 80, 100, 160],
    [80, 80, 80, 80, 160],
    [80, 80, 80, 180, 140],
    [100, 120, 120, 200, 240],
    [160, 160, 300, 240, 160],
]
TIME_SERIES_2D_TABLES = {
    "normal": [
        [0, 80, 80, 80, 80],
        [80, 80, 80, 80, 280],
        [80, 80, 80, 120, 160],
        [80, 80, 80, 120, 160],
        [80, 280, 160, 160, 160],
    ],
    "retrace": [
        [0, 80, 80, 80, 80],
        [80, 80, 80, 80, 280],
        [80, 80, 80, 80, 160],
        [80, 80, 80, 120, 160],
        [80, 280, 160, 160, 160],
    ],
    "snake": [
        [0, 80, 80, 80, 80],
        [80, 80, 80, 80, 260],
        [80, 80, 80, 80, 160],
        [80, 120, 80, 120, 220],
        [80, 280, 160, 160, 270],
    ],
}
BOXCAR_BY_MAX_ADC_PER_BOARD = {1: 300, 2: 300, 3: 500, 4: 800}


WATCHDOG_MARKERS = (
    "spi_missteps",
    "dac_spi_missteps",
    "adc_spi_missteps",
    "adc_conversion_missteps",
    "Voltage output buffer overflow",
    "Ramp timing misstep",
)


@dataclass
class CaseResult:
    name: str
    command: str
    expectation: str
    passed: bool
    message: str
    params: dict[str, Any]
    bytes_read: int = 0
    expected_bytes: int = 0
    samples: int = 0
    metric: dict[str, Any] | None = None


def detect_port() -> str:
    matches = [
        port.device
        for port in list_ports.comports()
        if port.vid == GATEKEEPER_VID and port.pid == GATEKEEPER_PID
    ]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise SystemExit("No GateKeeper USB CDC port found")
    raise SystemExit(f"Multiple GateKeeper ports found: {matches}; pass --port")


def command_text(name: str, *args: Any) -> str:
    return name if not args else name + "," + ",".join(format_arg(a) for a in args)


def format_arg(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.9g}"
    return str(value)


class GateKeeper:
    def __init__(self, port: str) -> None:
        self.ser = serial.Serial(port, baudrate=115200, timeout=0.03, write_timeout=1)
        time.sleep(0.25)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        self.ser.close()

    def read_idle(self, idle_s: float = 0.08, timeout_s: float = 1.0) -> bytes:
        deadline = time.monotonic() + timeout_s
        last = time.monotonic()
        out = bytearray()
        while time.monotonic() < deadline:
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                out.extend(chunk)
                last = time.monotonic()
            elif out and time.monotonic() - last >= idle_s:
                break
        return bytes(out)

    def query_line(self, name: str, *args: Any, timeout_s: float = 2.0) -> str:
        self.ser.reset_input_buffer()
        self.ser.write((command_text(name, *args) + "\n").encode("ascii"))
        self.ser.flush()
        deadline = time.monotonic() + timeout_s
        buf = bytearray()
        while time.monotonic() < deadline:
            b = self.ser.read(1)
            if b:
                buf.extend(b)
                if b == b"\n":
                    break
        return buf.decode("utf-8", errors="replace").strip()

    def write_command(self, name: str, *args: Any) -> None:
        self.ser.reset_input_buffer()
        self.ser.write((command_text(name, *args) + "\n").encode("ascii"))
        self.ser.flush()

    def command_no_reply(self, name: str, *args: Any, wait_s: float = 0.1) -> str:
        self.write_command(name, *args)
        time.sleep(wait_s)
        return self.read_idle(timeout_s=0.4).decode("utf-8", errors="replace").strip()

    def stop(self) -> str:
        self.ser.write(b"STOP\n")
        self.ser.flush()
        return self.read_idle(timeout_s=2.0).decode("utf-8", errors="replace").strip()

    def read_response(self, expected_bytes: int, timeout_s: float) -> tuple[bytes, str]:
        deadline = time.monotonic() + timeout_s
        raw = bytearray()
        while len(raw) < expected_bytes and time.monotonic() < deadline:
            chunk = self.ser.read(expected_bytes - len(raw))
            if not chunk:
                continue
            raw.extend(chunk)
            if raw.startswith(b"FAILURE:") and b"\n" in raw:
                return b"", raw.decode("utf-8", errors="replace").strip()
            if raw.startswith(b"FAILURE:"):
                while time.monotonic() < deadline and not raw.endswith(b"\n"):
                    b = self.ser.read(1)
                    if b:
                        raw.extend(b)
                return b"", raw.decode("utf-8", errors="replace").strip()
        trailer = self.read_idle(timeout_s=0.8).decode("utf-8", errors="replace").strip()
        return bytes(raw), trailer

    def read_exact_binary(self, expected_bytes: int, timeout_s: float) -> bytes:
        deadline = time.monotonic() + timeout_s
        raw = bytearray()
        while len(raw) < expected_bytes and time.monotonic() < deadline:
            chunk = self.ser.read(expected_bytes - len(raw))
            if chunk:
                raw.extend(chunk)
        return bytes(raw)


def adc_board_counts(adc_channels: list[int]) -> tuple[int, int]:
    counts = [0, 0]
    for ch in adc_channels:
        counts[ch // NUM_CHANNELS_PER_ADC_BOARD] += 1
    return counts[0], counts[1]


def max_adc_per_board(adc_channels: list[int]) -> int:
    return max(adc_board_counts(adc_channels))


def table_lookup(table: list[list[int]], adc_channels: list[int]) -> int:
    a, b = adc_board_counts(adc_channels)
    return table[a][b]


def max_board_conversion_sum(adc_channels: list[int], actual_by_channel: dict[int, float]) -> float:
    sums = [0.0, 0.0]
    for ch in adc_channels:
        sums[ch // NUM_CHANNELS_PER_ADC_BOARD] += actual_by_channel[ch]
    return max(sums)


def time_series_minimum(
    table: list[list[int]], adc_channels: list[int], actual_by_channel: dict[int, float]
) -> int:
    base = table_lookup(table, adc_channels)
    max_single = max(actual_by_channel[ch] for ch in adc_channels)
    if max_single <= 90.0:
        return base
    return max(base, math.ceil(1.2 * max_board_conversion_sum(adc_channels, actual_by_channel)))


def dac_led_minimum(adc_channels: list[int], actual_by_channel: dict[int, float]) -> int:
    base = table_lookup(DAC_LED_TABLE, adc_channels)
    max_single = max(actual_by_channel[ch] for ch in adc_channels)
    if max_single <= 90.0:
        return base
    return max(base, math.ceil(1.2 * max_board_conversion_sum(adc_channels, actual_by_channel)))


def dac_only_minimum(dac_channels: list[int]) -> int:
    return 20 if len(dac_channels) <= 4 else 40


def awg_with_adc_minimum(
    dac_channels: list[int], adc_channels: list[int], actual_by_channel: dict[int, float]
) -> int:
    base = dac_led_minimum(adc_channels, actual_by_channel)
    if len(dac_channels) >= 8:
        return base + 40
    if len(dac_channels) >= 4 and 200 <= base < 300:
        return base + 20
    return base


def parse_floats(raw: bytes) -> list[float]:
    usable = len(raw) - (len(raw) % 4)
    if usable <= 0:
        return []
    return list(struct.unpack("<" + "f" * (usable // 4), raw[:usable]))


def finite_and_in_range(values: Iterable[float]) -> bool:
    vals = list(values)
    return bool(vals) and all(math.isfinite(v) and abs(v) <= 10.5 for v in vals)


def has_bad_trailer(message: str) -> bool:
    return any(marker in message for marker in WATCHDOG_MARKERS)


def set_all_dacs(gk: GateKeeper, voltage: float = 0.0) -> None:
    gk.stop()
    gk.read_idle(idle_s=0.15, timeout_s=0.5)
    gk.ser.reset_input_buffer()
    for ch in range(NUM_DAC_CHANNELS):
        gk.ser.write((command_text("SET", ch, voltage) + "\n").encode("ascii"))
    gk.ser.flush()
    time.sleep(0.2)
    gk.read_idle(idle_s=0.15, timeout_s=3.0)


def set_conversions(gk: GateKeeper, request_us: int) -> dict[int, float]:
    gk.ser.reset_input_buffer()
    for ch in range(NUM_ADC_CHANNELS):
        gk.ser.write((command_text("CONVERT_TIME", ch, request_us) + "\n").encode("ascii"))
    gk.ser.flush()

    lines: list[str] = []
    deadline = time.monotonic() + 8.0
    current = bytearray()
    while len(lines) < NUM_ADC_CHANNELS and time.monotonic() < deadline:
        b = gk.ser.read(1)
        if not b:
            continue
        current.extend(b)
        if b == b"\n":
            text = current.decode("utf-8", errors="replace").strip()
            current.clear()
            if text:
                lines.append(text)
    if len(lines) != NUM_ADC_CHANNELS:
        raise RuntimeError(
            f"Expected {NUM_ADC_CHANNELS} CONVERT_TIME responses for request "
            f"{request_us}, got {len(lines)}: {lines}"
        )
    return {ch: float(lines[ch]) for ch in range(NUM_ADC_CHANNELS)}


def expect_failure(
    gk: GateKeeper, name: str, command: str, args: list[Any], params: dict[str, Any]
) -> CaseResult:
    gk.write_command(command, *args)
    message = gk.read_idle(timeout_s=1.2).decode("utf-8", errors="replace").strip()
    passed = message.startswith("FAILURE:")
    return CaseResult(name, command, "failure", passed, message, params, 0, 0)


def expect_binary_success(
    gk: GateKeeper,
    name: str,
    command: str,
    args: list[Any],
    expected_floats: int,
    params: dict[str, Any],
    timeout_s: float = 6.0,
) -> CaseResult:
    gk.write_command(command, *args)
    raw, trailer = gk.read_response(expected_floats * 4, timeout_s=timeout_s)
    values = parse_floats(raw)
    ok = (
        len(raw) == expected_floats * 4
        and not trailer.startswith("FAILURE:")
        and not has_bad_trailer(trailer)
        and finite_and_in_range(values)
    )
    return CaseResult(
        name,
        command,
        "success",
        ok,
        trailer,
        params,
        len(raw),
        expected_floats * 4,
        expected_floats,
        {"max_abs": max(abs(v) for v in values) if values else None},
    )


def expect_continuous_success(
    gk: GateKeeper, name: str, command: str, args: list[Any], params: dict[str, Any]
) -> CaseResult:
    gk.write_command(command, *args)
    time.sleep(0.12)
    message = gk.stop()
    passed = not has_bad_trailer(message) and "overflow" not in message.lower()
    return CaseResult(name, command, "continuous", passed, message, params)


def build_cases(quick: bool) -> list[tuple[list[int], list[int], int]]:
    cases = [
        ([0], [0], 82),
        ([0, 2, 4], [0, 4], 200),
        ([0, 1, 2, 3], [0, 1, 4, 5], 500),
        (list(range(8)), list(range(8)), 82),
        (list(range(8)), list(range(8)), 200),
        (list(range(8)), list(range(8)), 500),
    ]
    return cases[:3] if quick else cases


def run_suite(gk: GateKeeper, quick: bool) -> list[CaseResult]:
    results: list[CaseResult] = []
    gk.stop()
    gk.ser.reset_input_buffer()
    print(gk.query_line("INIT", timeout_s=4.0))
    gk.command_no_reply("SET_CHOP", 1)
    set_all_dacs(gk, 0.0)

    for dac_channels, adc_channels, conversion_request in build_cases(quick):
        actual = set_conversions(gk, conversion_request)
        set_all_dacs(gk, 0.0)
        prefix = (
            f"dac{len(dac_channels)}_adc{len(adc_channels)}_conv{conversion_request}"
        )
        common = {
            "dac_channels": dac_channels,
            "adc_channels": adc_channels,
            "conversion_request_us": conversion_request,
            "conversion_actual_us": {str(k): actual[k] for k in adc_channels},
        }

        # DAC-led 1D.
        steps = 9
        dac_led_min = dac_led_minimum(adc_channels, actual)
        safe_interval = dac_led_min
        unsafe_interval = max(1, dac_led_min - 1)
        args = [len(dac_channels), len(adc_channels), steps, 1, unsafe_interval, 20]
        args += dac_channels + [-0.8] * len(dac_channels) + [0.8] * len(dac_channels) + adc_channels
        results.append(expect_failure(gk, prefix + "_dac_led_unsafe", "DAC_LED_BUFFER_RAMP", args, {**common, "dac_interval_us": unsafe_interval, "dac_settling_time_us": 20}))
        args = [len(dac_channels), len(adc_channels), steps, 1, safe_interval, 20]
        args += dac_channels + [-0.8] * len(dac_channels) + [0.8] * len(dac_channels) + adc_channels
        results.append(expect_binary_success(gk, prefix + "_dac_led_safe", "DAC_LED_BUFFER_RAMP", args, steps * len(adc_channels), {**common, "dac_interval_us": safe_interval, "dac_settling_time_us": 20}))
        set_all_dacs(gk, 0.0)

        # DAC-led 2D.
        fast, slow = 5, 2
        args = [len(dac_channels), len(adc_channels), fast, slow, unsafe_interval, 20, 0, 0, 1]
        args += dac_channels + [-0.5] * len(dac_channels) + [1.0] * len(dac_channels) + [0.2] * len(dac_channels) + adc_channels
        results.append(expect_failure(gk, prefix + "_dac_led_2d_unsafe", "2D_DAC_LED_BUFFER_RAMP", args, {**common, "dac_interval_us": unsafe_interval}))
        args = [len(dac_channels), len(adc_channels), fast, slow, safe_interval, 20, 0, 0, 1]
        args += dac_channels + [-0.5] * len(dac_channels) + [1.0] * len(dac_channels) + [0.2] * len(dac_channels) + adc_channels
        results.append(expect_binary_success(gk, prefix + "_dac_led_2d_safe", "2D_DAC_LED_BUFFER_RAMP", args, fast * slow * len(adc_channels), {**common, "dac_interval_us": safe_interval}))
        set_all_dacs(gk, 0.0)

        # Time-series 1D.
        ts_min = time_series_minimum(TIME_SERIES_1D_TABLE, adc_channels, actual)
        ts_unsafe = max(1, ts_min - 1)
        ts_safe = ts_min
        ts_steps = 31
        ts_dac_interval = max(1000, ts_safe * 8)
        args = [len(dac_channels), len(adc_channels), ts_steps, ts_dac_interval, ts_unsafe]
        args += dac_channels + [-1.0] * len(dac_channels) + [1.0] * len(dac_channels) + adc_channels
        results.append(expect_failure(gk, prefix + "_time_series_unsafe", "TIME_SERIES_BUFFER_RAMP", args, {**common, "dac_interval_us": ts_dac_interval, "adc_interval_us": ts_unsafe}))
        frames = (ts_steps * ts_dac_interval) // ts_safe
        args = [len(dac_channels), len(adc_channels), ts_steps, ts_dac_interval, ts_safe]
        args += dac_channels + [-1.0] * len(dac_channels) + [1.0] * len(dac_channels) + adc_channels
        results.append(expect_binary_success(gk, prefix + "_time_series_safe", "TIME_SERIES_BUFFER_RAMP", args, frames * len(adc_channels), {**common, "dac_interval_us": ts_dac_interval, "adc_interval_us": ts_safe}, timeout_s=8.0))
        set_all_dacs(gk, 0.0)

        # Time-series 2D normal.
        ts2_min = time_series_minimum(TIME_SERIES_2D_TABLES["normal"], adc_channels, actual)
        ts2_unsafe = max(1, ts2_min - 1)
        ts2_safe = ts2_min
        ts2_dac_interval = max(800, ts2_safe * 6)
        fast, slow = 9, 2
        args = [len(dac_channels), len(adc_channels), fast, slow, ts2_dac_interval, ts2_unsafe, 0, 0]
        args += dac_channels + [-0.5] * len(dac_channels) + [1.0] * len(dac_channels) + [0.2] * len(dac_channels) + adc_channels
        results.append(expect_failure(gk, prefix + "_time_series_2d_unsafe", "2D_TIME_SERIES_BUFFER_RAMP", args, {**common, "dac_interval_us": ts2_dac_interval, "adc_interval_us": ts2_unsafe}))
        frames_per_line = (fast * ts2_dac_interval) // ts2_safe
        args = [len(dac_channels), len(adc_channels), fast, slow, ts2_dac_interval, ts2_safe, 0, 0]
        args += dac_channels + [-0.5] * len(dac_channels) + [1.0] * len(dac_channels) + [0.2] * len(dac_channels) + adc_channels
        results.append(expect_binary_success(gk, prefix + "_time_series_2d_safe", "2D_TIME_SERIES_BUFFER_RAMP", args, frames_per_line * slow * len(adc_channels), {**common, "dac_interval_us": ts2_dac_interval, "adc_interval_us": ts2_safe}, timeout_s=8.0))
        set_all_dacs(gk, 0.0)

        # AWG_WITH_ADC.
        waveform = [-0.8, -0.2, 0.4, 0.8]
        awg_min = awg_with_adc_minimum(dac_channels, adc_channels, actual)
        args = [len(dac_channels), len(adc_channels), len(waveform), max(1, awg_min - 1), 1]
        args += dac_channels + adc_channels + waveform * len(dac_channels)
        results.append(expect_failure(gk, prefix + "_awg_with_adc_unsafe", "AWG_WITH_ADC", args, {**common, "dac_interval_us": max(1, awg_min - 1)}))
        args = [len(dac_channels), len(adc_channels), len(waveform), awg_min, 1]
        args += dac_channels + adc_channels + waveform * len(dac_channels)
        results.append(expect_binary_success(gk, prefix + "_awg_with_adc_safe", "AWG_WITH_ADC", args, len(waveform) * len(adc_channels), {**common, "dac_interval_us": awg_min}, timeout_s=5.0))
        set_all_dacs(gk, 0.0)

        # Boxcar.
        box_min = BOXCAR_BY_MAX_ADC_PER_BOARD[max_adc_per_board(adc_channels)]
        box_unsafe = max(1, box_min - 1)
        box_steps, measures, averages = 3, 2, 1
        args = [len(dac_channels), len(adc_channels), box_steps, measures, averages, box_unsafe]
        args += dac_channels + [-0.4] * len(dac_channels) + [0.4] * len(dac_channels) + [0.4] * len(dac_channels) + [-0.4] * len(dac_channels) + adc_channels
        results.append(expect_failure(gk, prefix + "_boxcar_unsafe", "BOXCAR_BUFFER_RAMP", args, {**common, "adc_conversion_time_us": box_unsafe}))
        args = [len(dac_channels), len(adc_channels), box_steps, measures, averages, box_min]
        args += dac_channels + [-0.4] * len(dac_channels) + [0.4] * len(dac_channels) + [0.4] * len(dac_channels) + [-0.4] * len(dac_channels) + adc_channels
        expected = 2 * box_steps * measures * averages * len(adc_channels)
        results.append(expect_binary_success(gk, prefix + "_boxcar_safe", "BOXCAR_BUFFER_RAMP", args, expected, {**common, "adc_conversion_time_us": box_min}, timeout_s=8.0))
        set_all_dacs(gk, 0.0)

        # TIME_SERIES_ADC_READ.
        args = [len(adc_channels)] + adc_channels + [60, 2000]
        results.append(expect_failure(gk, prefix + "_adc_read_unsafe", "TIME_SERIES_ADC_READ", args, {**common, "conversion_time_us": 60}))
        read_duration = 3000
        args = [len(adc_channels)] + adc_channels + [conversion_request, read_duration]
        gk.write_command("TIME_SERIES_ADC_READ", *args)
        period_raw = gk.read_exact_binary(4, timeout_s=2.0)
        if period_raw.startswith(b"FAILURE:"):
            trailer = period_raw.decode("utf-8", errors="replace")
        else:
            trailer = ""
        if len(period_raw) == 4 and not trailer.startswith("FAILURE:"):
            sample_period = struct.unpack("<f", period_raw)[0]
            frames = int(read_duration / sample_period)
            raw, trailer = gk.read_response(frames * len(adc_channels) * 4, timeout_s=5.0)
            values = parse_floats(raw)
            ok = (
                len(raw) == frames * len(adc_channels) * 4
                and not trailer.startswith("FAILURE:")
                and not has_bad_trailer(trailer)
                and finite_and_in_range(values)
            )
            results.append(CaseResult(prefix + "_adc_read_safe", "TIME_SERIES_ADC_READ", "success", ok, trailer, {**common, "sample_period_us": sample_period, "duration_us": read_duration}, len(raw), frames * len(adc_channels) * 4, frames * len(adc_channels)))
        else:
            results.append(CaseResult(prefix + "_adc_read_safe", "TIME_SERIES_ADC_READ", "success", False, trailer, {**common, "duration_us": read_duration}, len(period_raw), 4))

        # AWG_BUFFER_RAMP is continuous, so validate guard and short run/STOP.
        dac_min = dac_only_minimum(dac_channels)
        waveform = [-0.5, 0.0, 0.5, 0.0]
        args = [len(dac_channels), len(waveform), max(1, dac_min - 1)]
        args += dac_channels + waveform * len(dac_channels)
        results.append(expect_failure(gk, prefix + "_awg_buffer_unsafe", "AWG_BUFFER_RAMP", args, {**common, "dac_interval_us": max(1, dac_min - 1)}))
        args = [len(dac_channels), len(waveform), dac_min]
        args += dac_channels + waveform * len(dac_channels)
        results.append(expect_continuous_success(gk, prefix + "_awg_buffer_safe", "AWG_BUFFER_RAMP", args, {**common, "dac_interval_us": dac_min}))
        set_all_dacs(gk, 0.0)

    return results


def write_outputs(results: list[CaseResult], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    serializable = [r.__dict__ for r in results]
    (output_dir / "buffer_ramp_safety_results.json").write_text(
        json.dumps(serializable, indent=2), encoding="utf-8"
    )
    with (output_dir / "buffer_ramp_safety_results.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = [
            "name",
            "command",
            "expectation",
            "passed",
            "message",
            "bytes_read",
            "expected_bytes",
            "samples",
            "params",
            "metric",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for result in results:
            row = result.__dict__.copy()
            row["params"] = json.dumps(row["params"], sort_keys=True)
            row["metric"] = json.dumps(row["metric"], sort_keys=True)
            writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()

    port = args.port or detect_port()
    output = Path(
        args.output_dir
        or Path("artifacts")
        / ("buffer_ramp_safety_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    )

    gk = GateKeeper(port)
    try:
        results = run_suite(gk, args.quick)
    finally:
        try:
            set_all_dacs(gk, 0.0)
            set_conversions(gk, 500)
        finally:
            gk.close()

    write_outputs(results, output)
    failures = [r for r in results if not r.passed]
    print(json.dumps({"output": str(output), "cases": len(results), "failures": len(failures)}, indent=2))
    if failures:
        for failure in failures:
            print(f"FAIL {failure.name}: {failure.message}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
