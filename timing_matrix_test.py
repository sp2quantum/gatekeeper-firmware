#!/usr/bin/env python3
"""Measure GateKeeper ramp timing limits by DAC count and ADC-card split."""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from hardware_functionality_test import GateKeeperSerial, NUM_CHANNELS, detect_port


ADC_BOARD_CHANNELS = ([0, 1, 2, 3], [4, 5, 6, 7])


def cmd_args(values: list[Any]) -> str:
    return ",".join(str(v) for v in values)


def adc_channels_for_split(adc0: int, adc1: int) -> list[int]:
    return list(ADC_BOARD_CHANNELS[0][:adc0]) + list(ADC_BOARD_CHANNELS[1][:adc1])


def parse_missteps(message: str) -> dict[str, int]:
    out: dict[str, int] = {}
    for key in ("dac_spi_missteps", "adc_spi_missteps", "adc_conversion_missteps"):
        match = re.search(rf"{key}=(\d+)", message)
        if match:
            out[key] = int(match.group(1))
    return out


class TimingMatrix:
    def __init__(self, port: str, output_dir: Path, quick: bool = False) -> None:
        self.gk = GateKeeperSerial(port)
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.quick = quick
        self.rows: list[dict[str, Any]] = []

    def close(self) -> None:
        self.gk.close()

    def set_conversion_all(self, conversion_us: float) -> None:
        for ch in range(NUM_CHANNELS):
            self.gk.query_line("CONVERT_TIME", ch, conversion_us, timeout=1.0)

    def set_dacs_all(self, voltage: float) -> None:
        for ch in range(NUM_CHANNELS):
            self.gk.query_line("SET", ch, voltage, timeout=1.0)

    def read_ramp(
        self, command: str, args: list[Any], expected_floats: int, timeout: float
    ) -> tuple[bool, np.ndarray, str]:
        self.gk.ser.reset_input_buffer()
        self.gk.write_command(command, *args)
        deadline = time.monotonic() + timeout
        raw = bytearray()
        failure = bytearray()
        expected_bytes = expected_floats * 4
        while len(raw) < expected_bytes and time.monotonic() < deadline:
            chunk = self.gk.ser.read(expected_bytes - len(raw))
            if not chunk:
                continue
            if not raw and chunk.startswith(b"FAILURE:"):
                failure.extend(chunk)
                break
            raw.extend(chunk)
        if failure:
            while time.monotonic() < deadline and not failure.endswith(b"\n"):
                b = self.gk.ser.read(1)
                if b:
                    failure.extend(b)
            return False, np.array([], dtype=float), failure.decode(errors="replace").strip()
        if len(raw) != expected_bytes:
            stop_text = self.stop()
            return False, np.array([], dtype=float), f"timeout bytes={len(raw)}/{expected_bytes} {stop_text}".strip()
        msg = self.gk.read_idle_text(timeout=0.4, idle=0.06)
        data = np.frombuffer(bytes(raw), dtype="<f4").astype(float)
        return not msg.startswith("FAILURE:"), data, msg

    def stop(self) -> str:
        return self.gk.stop_worker()

    @staticmethod
    def data_sensible(data: np.ndarray, num_adc: int) -> bool:
        if data.size == 0 or data.size % num_adc != 0:
            return False
        arr = data.reshape((-1, num_adc))
        return bool(np.all(np.isfinite(arr)) and np.max(np.abs(arr)) < 10.5)

    def record(
        self,
        ramp: str,
        mode: str,
        num_dac: int,
        adc0: int,
        adc1: int,
        timing_name: str,
        timing_value: int | None,
        passed: bool,
        message: str,
        extra: dict[str, Any] | None = None,
    ) -> None:
        row = {
            "ramp": ramp,
            "mode": mode,
            "num_dac": num_dac,
            "adc0": adc0,
            "adc1": adc1,
            "num_adc": adc0 + adc1,
            "max_adc_per_card": max(adc0, adc1),
            "timing_name": timing_name,
            "timing_value_us": timing_value,
            "passed": passed,
            "message": message,
        }
        row.update(parse_missteps(message))
        if extra:
            row.update(extra)
        self.rows.append(row)
        status = "PASS" if passed else "FAIL"
        print(
            f"[{status}] {ramp}/{mode} N={num_dac} adc={adc0}/{adc1} "
            f"{timing_name}={timing_value} {message}",
            flush=True,
        )

    def find_min(self, candidates: list[int], run_once) -> tuple[int | None, str]:
        last_message = ""
        for candidate in candidates:
            passed, message = run_once(candidate)
            last_message = message
            if passed:
                return candidate, message
        return None, last_message

    def sweep_dac_led(self, ramp: str, mode: str, num_dac: int, adc0: int, adc1: int) -> int | None:
        adc_channels = adc_channels_for_split(adc0, adc1)
        dac_channels = list(range(num_dac))
        settling = 20
        max_depth = max(adc0, adc1)
        total_adc = adc0 + adc1
        if max_depth >= 4:
            start, stop = 360, 920
        else:
            predicted = 85 * max_depth + 12 * total_adc + 12 * num_dac
            start = max(40, ((predicted - 80) // 20) * 20)
            stop = min(820, ((predicted + 180) // 20) * 20)
        candidates = list(range(start, stop + 1, 20))
        if self.quick:
            candidates = list(range(max(80, start), min(700, stop) + 1, 20))

        def run_once(interval: int) -> tuple[bool, str]:
            if ramp == "DAC_LED_BUFFER_RAMP":
                steps = 31
                args = [num_dac, len(adc_channels), steps, 1, interval, settling]
                args += dac_channels + [-0.8] * num_dac + [0.8] * num_dac + adc_channels
                expected = steps * len(adc_channels)
                command = "DAC_LED_BUFFER_RAMP"
            else:
                steps_fast, steps_slow = 11, 3
                retrace = 1 if mode == "retrace" else 0
                snake = 1 if mode == "snake" else 0
                scans = 2 if retrace and not snake else 1
                args = [
                    num_dac,
                    len(adc_channels),
                    steps_fast,
                    steps_slow,
                    interval,
                    settling,
                    retrace,
                    snake,
                    1,
                ]
                args += dac_channels + [-0.6] * num_dac + [1.2] * num_dac + [0.3] * num_dac + adc_channels
                expected = steps_fast * steps_slow * scans * len(adc_channels)
                command = "2D_DAC_LED_BUFFER_RAMP"
            ok, data, msg = self.read_ramp(command, args, expected, timeout=3.0)
            return ok and self.data_sensible(data, len(adc_channels)), msg

        best, msg = self.find_min(candidates, run_once)
        if best is None:
            best, msg = self.find_min(list(range(80, 901, 40)), run_once)
        self.record(ramp, mode, num_dac, adc0, adc1, "dac_interval", best, best is not None, msg, {"settling_us": settling})
        return best

    def sweep_time_series(self, ramp: str, mode: str, num_dac: int, adc0: int, adc1: int) -> int | None:
        adc_channels = adc_channels_for_split(adc0, adc1)
        dac_channels = list(range(num_dac))
        dac_interval = 400 if ramp == "2D_TIME_SERIES_BUFFER_RAMP" else 300
        base_end = 460 if mode == "snake" else 360
        candidates = list(range(80, base_end + 1, 20))
        if self.quick:
            candidates = list(range(120, min(361, base_end + 1), 20))

        def run_once(adc_interval: int) -> tuple[bool, str]:
            if ramp == "TIME_SERIES_BUFFER_RAMP":
                steps = 61
                args = [num_dac, len(adc_channels), steps, dac_interval, adc_interval]
                args += dac_channels + [-0.8] * num_dac + [0.8] * num_dac + adc_channels
                expected = max(1, (steps * dac_interval) // adc_interval) * len(adc_channels)
                command = "TIME_SERIES_BUFFER_RAMP"
            else:
                steps_fast, steps_slow = 21, 3
                retrace = 1 if mode == "retrace" else 0
                snake = 1 if mode == "snake" else 0
                scans = 2 if retrace and not snake else 1
                args = [
                    num_dac,
                    len(adc_channels),
                    steps_fast,
                    steps_slow,
                    dac_interval,
                    adc_interval,
                    retrace,
                    snake,
                ]
                args += dac_channels + [-0.6] * num_dac + [1.2] * num_dac + [0.3] * num_dac + adc_channels
                expected_per_line = max(1, (steps_fast * dac_interval) // adc_interval)
                expected = expected_per_line * steps_slow * scans * len(adc_channels)
                command = "2D_TIME_SERIES_BUFFER_RAMP"
            ok, data, msg = self.read_ramp(command, args, expected, timeout=4.0)
            return ok and self.data_sensible(data, len(adc_channels)), msg

        best, msg = self.find_min(candidates, run_once)
        if best is None:
            best, msg = self.find_min(list(range(base_end + 40, 701, 40)), run_once)
        self.record(ramp, mode, num_dac, adc0, adc1, "adc_interval", best, best is not None, msg, {"dac_interval_us": dac_interval})
        return best

    def sweep_awg_with_adc(self, num_dac: int, adc0: int, adc1: int) -> int | None:
        adc_channels = adc_channels_for_split(adc0, adc1)
        dac_channels = list(range(num_dac))
        waveform = np.linspace(-0.8, 0.8, 24).round(6).tolist()
        max_depth = max(adc0, adc1)
        total_adc = adc0 + adc1
        if max_depth >= 4:
            start, stop = 360, 1000
        else:
            predicted = 85 * max_depth + 12 * total_adc + 12 * num_dac
            start = max(60, ((predicted - 80) // 20) * 20)
            stop = min(900, ((predicted + 220) // 20) * 20)
        candidates = list(range(start, stop + 1, 20))
        if self.quick:
            candidates = list(range(max(100, start), min(800, stop) + 1, 20))

        def run_once(interval: int) -> tuple[bool, str]:
            args = [num_dac, len(adc_channels), len(waveform), interval, 1]
            args += dac_channels + adc_channels + waveform * num_dac
            ok, data, msg = self.read_ramp("AWG_WITH_ADC", args, len(waveform) * len(adc_channels), timeout=4.0)
            return ok and self.data_sensible(data, len(adc_channels)), msg

        best, msg = self.find_min(candidates, run_once)
        if best is None:
            best, msg = self.find_min(list(range(100, 1001, 40)), run_once)
        self.record("AWG_WITH_ADC", "normal", num_dac, adc0, adc1, "dac_interval", best, best is not None, msg)
        return best

    def sweep_boxcar(self, num_dac: int, adc0: int, adc1: int) -> int | None:
        adc_channels = adc_channels_for_split(adc0, adc1)
        dac_channels = list(range(num_dac))
        candidates = [82, 120, 160, 200, 300, 500, 800, 1200, 1600, 2200, 2600]

        def run_once(conversion: int) -> tuple[bool, str]:
            args = [num_dac, len(adc_channels), 5, 3, 1, conversion]
            args += dac_channels + [-0.4] * num_dac + [0.4] * num_dac + [0.4] * num_dac + [-0.4] * num_dac + adc_channels
            expected = 2 * 5 * 1 * 3 * len(adc_channels)
            ok, data, msg = self.read_ramp("BOXCAR_BUFFER_RAMP", args, expected, timeout=6.0)
            missteps = parse_missteps(msg)
            timing_ok_without_conversion = (
                missteps.get("dac_spi_missteps", 0) == 0
                and missteps.get("adc_spi_missteps", 0) == 0
            )
            return (ok or timing_ok_without_conversion) and self.data_sensible(data, len(adc_channels)), msg

        best, msg = self.find_min(candidates, run_once)
        self.record(
            "BOXCAR_BUFFER_RAMP",
            "normal",
            num_dac,
            adc0,
            adc1,
            "adc_conversion",
            best,
            best is not None,
            msg,
            {"pass_ignores_adc_conversion_missteps": True},
        )
        return best

    def run(self) -> None:
        self.gk.command_no_reply("SET_CHOP", 1)
        self.set_conversion_all(82)
        self.set_dacs_all(0.0)
        dac_counts = list(range(1, 9))
        adc_splits = [(a, b) for a in range(5) for b in range(5) if a + b > 0]
        if self.quick:
            dac_counts = [1, 2, 4, 8]
        for num_dac in dac_counts:
            for adc0, adc1 in adc_splits:
                self.sweep_dac_led("DAC_LED_BUFFER_RAMP", "normal", num_dac, adc0, adc1)
                for mode in ("normal", "retrace", "snake"):
                    self.sweep_dac_led("2D_DAC_LED_BUFFER_RAMP", mode, num_dac, adc0, adc1)
                self.sweep_time_series("TIME_SERIES_BUFFER_RAMP", "normal", num_dac, adc0, adc1)
                for mode in ("normal", "retrace", "snake"):
                    self.sweep_time_series("2D_TIME_SERIES_BUFFER_RAMP", mode, num_dac, adc0, adc1)
                self.sweep_awg_with_adc(num_dac, adc0, adc1)
                self.sweep_boxcar(num_dac, adc0, adc1)
                self.write_outputs()
        self.set_dacs_all(0.0)
        self.set_conversion_all(500)

    def write_outputs(self) -> None:
        json_path = self.output_dir / "timing_matrix.json"
        json_path.write_text(json.dumps(self.rows, indent=2), encoding="utf-8")
        csv_path = self.output_dir / "timing_matrix.csv"
        if not self.rows:
            return
        fieldnames = sorted({key for row in self.rows for key in row})
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()

    port = args.port or detect_port()
    output = Path(args.output_dir or Path("test_outputs") / ("timing_matrix_" + datetime.now().strftime("%Y%m%d_%H%M%S"))).resolve()
    matrix = TimingMatrix(port, output, quick=args.quick)
    try:
        matrix.run()
    finally:
        try:
            matrix.set_dacs_all(0.0)
            matrix.set_conversion_all(500)
        except Exception:
            pass
        matrix.write_outputs()
        matrix.close()
    print(output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
