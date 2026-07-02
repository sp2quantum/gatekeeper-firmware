#!/usr/bin/env python3
r"""Find minimum stable timings for 2D buffer-ramp commands.

Run with the PlatformIO Python environment, for example:

  C:\Users\Kapitza\.platformio\penv\Scripts\python.exe ^
    platformio_tools\characterize_2d_ramp_timings.py ^
    --port COM8 --dac-counts 1..4 --adc-counts 1..4

The script tests both 2D_DAC_LED_BUFFER_RAMP and
2D_TIME_SERIES_BUFFER_RAMP by default.  ADC channel selections are grouped by
board load, so two ADC channels on board 0 (2/0) are reported separately from
one channel on each board (1/1).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - this is for operator feedback.
    raise SystemExit(
        "pyserial is required. Run this with the PlatformIO Python environment."
    ) from exc


GATEKEEPER_VID = 0x2341
GATEKEEPER_PID = 0x0266
SERIAL_BAUD = 115200
NUM_CHANNELS_PER_DAC_BOARD = 4
NUM_DAC_BOARDS = 2
NUM_ADC_BOARDS = 2
NUM_CHANNELS_PER_ADC_BOARD = 4
NUM_DAC_CHANNELS = NUM_DAC_BOARDS * NUM_CHANNELS_PER_DAC_BOARD
NUM_ADC_CHANNELS = NUM_ADC_BOARDS * NUM_CHANNELS_PER_ADC_BOARD


@dataclass(frozen=True)
class RampSpec:
    name: str
    command: str


@dataclass(frozen=True)
class AdcLayout:
    board_counts: tuple[int, ...]
    channels: tuple[int, ...]

    @property
    def label(self) -> str:
        return "/".join(str(count) for count in self.board_counts)


@dataclass
class TrialResult:
    passed: bool
    bytes_read: int
    expected_bytes: int
    message: str
    values: list[float]
    min_r2: float | None = None
    mean_slope: float | None = None
    shape_ok: bool | None = None


RAMPS = {
    "dac-led-2d": RampSpec("dac-led-2d", "2D_DAC_LED_BUFFER_RAMP"),
    "time-series-2d": RampSpec("time-series-2d", "2D_TIME_SERIES_BUFFER_RAMP"),
}


def parse_count_list(value: str, maximum: int, label: str) -> list[int]:
    counts: list[int] = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        if ".." in part:
            start_text, end_text = part.split("..", 1)
            start = int(start_text)
            end = int(end_text)
            counts.extend(range(start, end + 1))
        else:
            counts.append(int(part))
    unique = sorted(set(counts))
    if not unique:
        raise argparse.ArgumentTypeError(f"{label} must not be empty")
    for count in unique:
        if count < 1 or count > maximum:
            raise argparse.ArgumentTypeError(
                f"{label} count {count} is outside 1..{maximum}"
            )
    return unique


def parse_int_list(value: str, maximum: int, label: str) -> list[int]:
    if value.strip().lower() in {"", "none"}:
        return []
    channels = [int(part.strip()) for part in value.split(",") if part.strip()]
    for channel in channels:
        if channel < 0 or channel >= maximum:
            raise argparse.ArgumentTypeError(
                f"{label} channel {channel} is outside 0..{maximum - 1}"
            )
    return channels


def auto_port() -> str:
    candidates = []
    for port in list_ports.comports():
        if port.vid == GATEKEEPER_VID and port.pid == GATEKEEPER_PID:
            candidates.append(port.device)
    if not candidates:
        raise SystemExit(
            "No GateKeeper serial port found. Pass --port COMx explicitly."
        )
    if len(candidates) > 1:
        raise SystemExit(
            "Multiple GateKeeper serial ports found: "
            + ", ".join(candidates)
            + ". Pass --port explicitly."
        )
    return candidates[0]


def adc_board(channel: int) -> int:
    return channel // NUM_CHANNELS_PER_ADC_BOARD


def adc_layouts_for_count(count: int) -> list[AdcLayout]:
    layouts: list[AdcLayout] = []

    def rec(board: int, remaining: int, counts: list[int]) -> None:
        if board == NUM_ADC_BOARDS:
            if remaining == 0:
                channels: list[int] = []
                for board_index, board_count in enumerate(counts):
                    base = board_index * NUM_CHANNELS_PER_ADC_BOARD
                    channels.extend(range(base, base + board_count))
                layouts.append(AdcLayout(tuple(counts), tuple(channels)))
            return
        max_here = min(NUM_CHANNELS_PER_ADC_BOARD, remaining)
        for board_count in range(max_here, -1, -1):
            counts.append(board_count)
            rec(board + 1, remaining - board_count, counts)
            counts.pop()

    rec(0, count, [])
    return layouts


def command_line(serial_port: serial.Serial, command: str, timeout_s: float) -> str:
    serial_port.reset_input_buffer()
    serial_port.write((command + "\n").encode("ascii"))
    deadline = time.time() + timeout_s
    data = bytearray()
    while time.time() < deadline:
        byte = serial_port.read(1)
        if not byte:
            continue
        data.extend(byte)
        if byte == b"\n":
            break
    return data.decode("utf-8", errors="replace").strip()


def drain(serial_port: serial.Serial, quiet_s: float) -> bytes:
    deadline = time.time() + quiet_s
    data = bytearray()
    while time.time() < deadline:
        chunk = serial_port.read(4096)
        if chunk:
            data.extend(chunk)
            deadline = time.time() + quiet_s
    return bytes(data)


def make_scan_vectors(num_dac_channels: int) -> tuple[list[float], list[float], list[float]]:
    start = [0.0] * num_dac_channels
    fast = [0.0] * num_dac_channels
    slow = [0.0] * num_dac_channels

    start[0] = -1.0
    fast[0] = 2.0
    if num_dac_channels > 1:
        slow[1] = 0.5
    return start, fast, slow


def expected_frames(
    ramp: RampSpec,
    num_steps_fast: int,
    num_steps_slow: int,
    dac_interval_us: int,
    adc_interval_us: int,
    retrace: bool,
    snake: bool,
) -> int:
    scans_per_slow_step = 2 if (retrace and not snake) else 1
    if ramp.name == "dac-led-2d":
        return num_steps_fast * num_steps_slow * scans_per_slow_step

    frames_per_scan = (num_steps_fast * dac_interval_us) // adc_interval_us
    return frames_per_scan * num_steps_slow * scans_per_slow_step


def build_ramp_command(
    ramp: RampSpec,
    dac_channels: Sequence[int],
    adc_channels: Sequence[int],
    interval_us: int,
    args: argparse.Namespace,
) -> tuple[str, int]:
    start, fast, slow = make_scan_vectors(len(dac_channels))
    retrace = 1 if args.retrace else 0
    snake = 1 if args.snake else 0

    if ramp.name == "dac-led-2d":
        command_args: list[float | int] = [
            len(dac_channels),
            len(adc_channels),
            args.num_fast,
            args.num_slow,
            interval_us,
            args.settling_us,
            retrace,
            snake,
            args.adc_averages,
        ]
        dac_interval_us = interval_us
        adc_interval_us = interval_us
    else:
        command_args = [
            len(dac_channels),
            len(adc_channels),
            args.num_fast,
            args.num_slow,
            interval_us if args.time_series_dac_interval_us is None else args.time_series_dac_interval_us,
            interval_us if args.time_series_adc_interval_us is None else args.time_series_adc_interval_us,
            retrace,
            snake,
        ]
        dac_interval_us = int(command_args[4])
        adc_interval_us = int(command_args[5])

    command_args.extend(dac_channels)
    command_args.extend(start)
    command_args.extend(fast)
    command_args.extend(slow)
    command_args.extend(adc_channels)

    frames = expected_frames(
        ramp,
        args.num_fast,
        args.num_slow,
        dac_interval_us,
        adc_interval_us,
        args.retrace,
        args.snake,
    )
    command = ramp.command + "," + ",".join(str(value) for value in command_args)
    return command, frames * len(adc_channels) * 4


def read_ramp_output(
    serial_port: serial.Serial,
    command: str,
    expected_bytes: int,
    timeout_s: float,
    quiet_s: float,
) -> tuple[bytes, str]:
    serial_port.reset_input_buffer()
    serial_port.write((command + "\n").encode("ascii"))
    data = bytearray()
    deadline = time.time() + timeout_s
    while len(data) < expected_bytes and time.time() < deadline:
        chunk = serial_port.read(min(4096, expected_bytes - len(data)))
        if not chunk:
            continue
        data.extend(chunk)
        if data.startswith(b"FAILURE") or b"FAILURE:" in data[:160]:
            break

    trailer = drain(serial_port, quiet_s)
    message = trailer.decode("utf-8", errors="replace").strip()
    if not message:
        prefix = bytes(data[:160]).decode("utf-8", errors="ignore")
        if "FAILURE" in prefix:
            message = prefix.strip()
    return bytes(data), message


def unpack_values(data: bytes, expected_bytes: int) -> list[float]:
    if len(data) < expected_bytes:
        return []
    sample_count = expected_bytes // 4
    return list(struct.unpack("<" + "f" * sample_count, data[:expected_bytes]))


def shape_metrics(
    values: Sequence[float],
    adc_channels: Sequence[int],
    tracked_adc_channel: int,
    frames: int,
    num_fast: int,
    num_slow: int,
    retrace: bool,
    snake: bool,
    min_r2_threshold: float,
) -> tuple[bool | None, float | None, float | None]:
    if tracked_adc_channel not in adc_channels:
        return None, None, None
    if len(values) != frames * len(adc_channels):
        return False, None, None

    adc_index = adc_channels.index(tracked_adc_channel)
    tracked = [
        values[frame * len(adc_channels) + adc_index] for frame in range(frames)
    ]

    scans_per_slow_step = 2 if (retrace and not snake) else 1
    expected_scans = num_slow * scans_per_slow_step
    if frames % expected_scans != 0:
        return False, None, None
    frames_per_scan = frames // expected_scans
    if frames_per_scan < 2:
        return False, None, None

    xs = [-1.0 + 2.0 * i / (frames_per_scan - 1) for i in range(frames_per_scan)]
    r2s: list[float] = []
    slopes: list[float] = []
    for scan in range(expected_scans):
        row = tracked[scan * frames_per_scan : (scan + 1) * frames_per_scan]
        mx = sum(xs) / len(xs)
        my = sum(row) / len(row)
        sxx = sum((x - mx) ** 2 for x in xs)
        syy = sum((y - my) ** 2 for y in row)
        sxy = sum((x - mx) * (y - my) for x, y in zip(xs, row))
        if sxx == 0.0:
            return False, None, None
        slope = sxy / sxx
        fitted = [my + slope * (x - mx) for x in xs]
        rss = sum((y - fit) ** 2 for y, fit in zip(row, fitted))
        r2 = 1.0 - rss / syy if syy > 1e-12 else 0.0
        r2s.append(r2)
        slopes.append(slope)

    finite = all(math.isfinite(value) for value in tracked)
    min_r2 = min(r2s)
    mean_slope = sum(slopes) / len(slopes)
    shape_ok = finite and min_r2 >= min_r2_threshold and abs(mean_slope) > 0.2
    return shape_ok, min_r2, mean_slope


def run_trial(
    serial_port: serial.Serial,
    ramp: RampSpec,
    dac_channels: Sequence[int],
    adc_layout: AdcLayout,
    interval_us: int,
    args: argparse.Namespace,
) -> TrialResult:
    command, expected_bytes = build_ramp_command(
        ramp, dac_channels, adc_layout.channels, interval_us, args
    )
    if args.dry_run:
        print(command)
        return TrialResult(True, expected_bytes, expected_bytes, "dry-run", [])

    data, message = read_ramp_output(
        serial_port, command, expected_bytes, args.ramp_timeout_s, args.quiet_s
    )
    failure_text = any(
        token in message
        for token in ("FAILURE", "misstep", "source=spi", "overflow", "Invalid")
    )
    values = unpack_values(data, expected_bytes)
    frames = expected_bytes // (4 * len(adc_layout.channels))
    shape_ok, min_r2, mean_slope = shape_metrics(
        values,
        adc_layout.channels,
        args.tracked_adc_channel,
        frames,
        args.num_fast,
        args.num_slow,
        args.retrace,
        args.snake,
        args.min_r2,
    )
    passed = len(data) == expected_bytes and not failure_text
    if shape_ok is False:
        passed = False
    return TrialResult(
        passed=passed,
        bytes_read=len(data),
        expected_bytes=expected_bytes,
        message=message,
        values=values,
        min_r2=min_r2,
        mean_slope=mean_slope,
        shape_ok=shape_ok,
    )


def interval_candidates(args: argparse.Namespace) -> list[int]:
    return list(range(args.interval_min_us, args.interval_max_us + 1, args.interval_step_us))


def find_minimum_interval(
    serial_port: serial.Serial,
    ramp: RampSpec,
    dac_channels: Sequence[int],
    adc_layout: AdcLayout,
    args: argparse.Namespace,
) -> tuple[int | None, TrialResult | None, TrialResult | None]:
    last_failure: TrialResult | None = None
    for interval_us in interval_candidates(args):
        trial_results = [
            run_trial(serial_port, ramp, dac_channels, adc_layout, interval_us, args)
            for _ in range(args.repeats)
        ]
        if all(result.passed for result in trial_results):
            return interval_us, trial_results[-1], last_failure
        last_failure = trial_results[-1]
        time.sleep(args.between_trials_s)
    return None, None, last_failure


def set_conversion_times(
    serial_port: serial.Serial, channels: Iterable[int], conversion_us: float, timeout_s: float
) -> dict[int, str]:
    actual: dict[int, str] = {}
    for channel in sorted(set(channels)):
        command_line(serial_port, f"CONVERT_TIME,{channel},{conversion_us}", timeout_s)
        actual[channel] = command_line(serial_port, f"GET_CONVERT_TIME,{channel}", timeout_s)
    return actual


def print_table(rows: Sequence[dict[str, object]]) -> None:
    headers = [
        "ramp",
        "dac_n",
        "adc_n",
        "adc_layout",
        "dac_channels",
        "adc_channels",
        "min_interval_us",
        "bytes",
        "shape",
        "min_r2",
        "mean_slope",
        "last_failure",
    ]
    widths = {
        header: max(len(header), *(len(str(row.get(header, ""))) for row in rows))
        for header in headers
    }
    print(" | ".join(header.ljust(widths[header]) for header in headers))
    print(" | ".join("-" * widths[header] for header in headers))
    for row in rows:
        print(" | ".join(str(row.get(header, "")).ljust(widths[header]) for header in headers))


def write_outputs(rows: Sequence[dict[str, object]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "2d_ramp_timing_table.csv"
    json_path = output_dir / "2d_ramp_timing_table.json"
    if rows:
        with csv_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    json_path.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(f"\nWrote {csv_path}")
    print(f"Wrote {json_path}")


def make_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Characterize minimum stable 2D ramp timings."
    )
    parser.add_argument("--port", default=None, help="Serial port, e.g. COM8.")
    parser.add_argument(
        "--ramps",
        default="dac-led-2d,time-series-2d",
        help="Comma-separated ramp names: dac-led-2d,time-series-2d.",
    )
    parser.add_argument("--dac-counts", default="2", help="Counts or ranges, e.g. 1,2,4 or 1..4.")
    parser.add_argument("--adc-counts", default="1,2", help="Counts or ranges, e.g. 1,2 or 1..4.")
    parser.add_argument(
        "--dac-channel-base",
        type=int,
        default=0,
        help="First DAC channel for generated DAC channel lists.",
    )
    parser.add_argument("--num-fast", type=int, default=64)
    parser.add_argument("--num-slow", type=int, default=8)
    parser.add_argument("--conversion-us", type=float, default=82.0)
    parser.add_argument("--settling-us", type=int, default=2)
    parser.add_argument("--adc-averages", type=int, default=1)
    parser.add_argument("--interval-min-us", type=int, default=60)
    parser.add_argument("--interval-max-us", type=int, default=180)
    parser.add_argument("--interval-step-us", type=int, default=1)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--retrace", action="store_true")
    parser.add_argument("--snake", action="store_true")
    parser.add_argument(
        "--tracked-adc-channel",
        type=int,
        default=0,
        help="ADC channel expected to track DAC0 for shape validation. "
        "Rows without this ADC channel are timing-only.",
    )
    parser.add_argument(
        "--min-r2",
        type=float,
        default=0.985,
        help="Minimum row-wise R^2 for shape validation when tracked ADC is present.",
    )
    parser.add_argument(
        "--time-series-dac-interval-us",
        type=int,
        default=None,
        help="Pin DAC interval for time-series ramps. Default sweeps it.",
    )
    parser.add_argument(
        "--time-series-adc-interval-us",
        type=int,
        default=None,
        help="Pin ADC interval for time-series ramps. Default sweeps it.",
    )
    parser.add_argument("--command-timeout-s", type=float, default=4.0)
    parser.add_argument("--ramp-timeout-s", type=float, default=10.0)
    parser.add_argument("--quiet-s", type=float, default=0.4)
    parser.add_argument("--between-trials-s", type=float, default=0.1)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("test_outputs") / "2d_ramp_timing_sweep",
    )
    parser.add_argument("--skip-initialize", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main() -> int:
    parser = make_arg_parser()
    args = parser.parse_args()

    ramp_names = [name.strip() for name in args.ramps.split(",") if name.strip()]
    unknown = [name for name in ramp_names if name not in RAMPS]
    if unknown:
        parser.error("unknown ramp(s): " + ", ".join(unknown))
    dac_counts = parse_count_list(args.dac_counts, NUM_DAC_CHANNELS, "DAC")
    adc_counts = parse_count_list(args.adc_counts, NUM_ADC_CHANNELS, "ADC")
    if args.dac_channel_base < 0:
        parser.error("--dac-channel-base must be nonnegative")
    if args.repeats < 1:
        parser.error("--repeats must be at least 1")

    port = args.port or auto_port()
    rows: list[dict[str, object]] = []

    serial_port: serial.Serial | None = None
    if not args.dry_run:
        serial_port = serial.Serial(port, SERIAL_BAUD, timeout=0.02, write_timeout=2)
        serial_port.dtr = True
        serial_port.rts = True
        time.sleep(1.0)
        serial_port.reset_input_buffer()
        print("*IDN? ->", command_line(serial_port, "*IDN?", args.command_timeout_s))
        if not args.skip_initialize:
            print("INITIALIZE ->", command_line(serial_port, "INITIALIZE", args.command_timeout_s))

    try:
        for adc_count in adc_counts:
            layouts = adc_layouts_for_count(adc_count)
            for layout in layouts:
                actual_conversion = {}
                if serial_port is not None:
                    actual_conversion = set_conversion_times(
                        serial_port,
                        layout.channels,
                        args.conversion_us,
                        args.command_timeout_s,
                    )
                for dac_count in dac_counts:
                    dac_channels = tuple(
                        range(args.dac_channel_base, args.dac_channel_base + dac_count)
                    )
                    if dac_channels[-1] >= NUM_DAC_CHANNELS:
                        parser.error(
                            f"DAC channel list {dac_channels} exceeds "
                            f"0..{NUM_DAC_CHANNELS - 1}"
                        )
                    for ramp_name in ramp_names:
                        ramp = RAMPS[ramp_name]
                        if serial_port is None:
                            command, expected_bytes = build_ramp_command(
                                ramp,
                                dac_channels,
                                layout.channels,
                                args.interval_min_us,
                                args,
                            )
                            print(command)
                            interval, trial, last_failure = (
                                args.interval_min_us,
                                TrialResult(
                                    passed=True,
                                    bytes_read=expected_bytes,
                                    expected_bytes=expected_bytes,
                                    message="dry-run",
                                    values=[],
                                ),
                                None,
                            )
                        else:
                            interval, trial, last_failure = find_minimum_interval(
                                serial_port, ramp, dac_channels, layout, args
                            )

                        row = {
                            "ramp": ramp.name,
                            "dac_n": dac_count,
                            "adc_n": adc_count,
                            "adc_layout": layout.label,
                            "dac_channels": ",".join(str(ch) for ch in dac_channels),
                            "adc_channels": ",".join(str(ch) for ch in layout.channels),
                            "actual_conversion_us": ";".join(
                                f"{ch}:{actual_conversion.get(ch, '')}"
                                for ch in layout.channels
                            ),
                            "min_interval_us": interval if interval is not None else "FAIL",
                            "bytes": (
                                f"{trial.bytes_read}/{trial.expected_bytes}"
                                if trial is not None
                                else ""
                            ),
                            "shape": (
                                "timing-only"
                                if trial is not None and trial.shape_ok is None
                                else str(trial.shape_ok)
                                if trial is not None
                                else ""
                            ),
                            "min_r2": (
                                f"{trial.min_r2:.6f}"
                                if trial is not None and trial.min_r2 is not None
                                else ""
                            ),
                            "mean_slope": (
                                f"{trial.mean_slope:.6f}"
                                if trial is not None and trial.mean_slope is not None
                                else ""
                            ),
                            "last_failure": (
                                last_failure.message[:180]
                                if last_failure is not None and last_failure.message
                                else ""
                            ),
                        }
                        rows.append(row)
                        print(
                            f"{ramp.name} dac={dac_count} adc={adc_count} "
                            f"layout={layout.label} -> {row['min_interval_us']} us"
                        )
    finally:
        if serial_port is not None:
            serial_port.close()

    print()
    print_table(rows)
    write_outputs(rows, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
