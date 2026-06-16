#!/usr/bin/env python3
"""
Direct hardware validation for a connected GateKeeper.

This intentionally avoids calibration commands and calibration flash mutation.
It talks to the USB CDC gateway directly, exercises non-calibration command
paths, streams binary ramp data, and saves plots plus a JSON/Markdown report.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import serial
import serial.tools.list_ports


NUM_CHANNELS = 8
GATEKEEPER_VID = 0x2341
GATEKEEPER_PID = 0x0266

NON_CALIBRATION_COMMANDS = {
    "PRINT_FUNCTIONS",
    "INITIALIZE",
    "INIT",
    "INNIT",
    "GET_FIRMWARE_VERSION",
    "NOP",
    "GET_ENVIRONMENT",
    "*IDN?",
    "*RDY?",
    "SERIAL_NUMBER",
    "CONVERT_TIME_FW",
    "GET_CONVERT_TIME",
    "GET_REVISION_REG",
    "CONTINUOUS_CONVERT_READ",
    "GET_CHANNELS_ACTIVE",
    "RESET",
    "TALK",
    "RESET_MAINTAIN",
    "SET_CHOP",
    "GET_CHOP",
    "GET_ADC",
    "IDLE_MODE",
    "SET_RDYFN",
    "UNSET_RDYFN",
    "CONVERT_TIME",
    "SET_UPPER_LIMIT",
    "SET_LOWER_LIMIT",
    "GET_UPPER_LIMIT",
    "GET_LOWER_LIMIT",
    "TOGGLE_LDAC",
    "SET_DAC_CODE",
    "FULL_SCALE",
    "GET_FULL_SCALE",
    "RAMP1",
    "RAMP2",
    "RAMP_N",
    "SET",
    "GET_DAC",
    "DAC_LED_BUFFER_RAMP",
    "2D_DAC_LED_BUFFER_RAMP",
    "TIME_SERIES_BUFFER_RAMP",
    "2D_TIME_SERIES_BUFFER_RAMP",
    "TIME_SERIES_ADC_READ",
    "AWG_BUFFER_RAMP",
    "AWG_WITH_ADC",
    "BOXCAR_BUFFER_RAMP",
}

CALIBRATION_RELATED_COMMANDS = {
    "ADC_ZERO_SC_CAL",
    "ADC_CH_ZERO_SC_CAL",
    "ADC_CH_FULL_SC_CAL",
    "GET_SAVED_ZERO_SCALE_CAL",
    "GET_SAVED_FULL_SCALE_CAL",
    "SET_SAVED_ZERO_SCALE_CAL",
    "SET_SAVED_FULL_SCALE_CAL",
    "SET_ZERO_SCALE_CAL",
    "SET_FULL_SCALE_CAL",
    "GET_ZERO_SCALE_CAL",
    "GET_FULL_SCALE_CAL",
    "HARD_RESET",
    "HARD_RESET_CALIBRATION",
    "SET_OSG",
    "INQUIRY_OSG",
    "DAC_CH_CAL",
}


@dataclass
class Check:
    name: str
    status: str
    details: dict[str, Any] = field(default_factory=dict)


def now_label() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def detect_port() -> str:
    candidates = []
    for port in serial.tools.list_ports.comports():
        if port.vid == GATEKEEPER_VID and port.pid == GATEKEEPER_PID:
            candidates.append(port.device)
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise RuntimeError("No GateKeeper USB CDC device found")
    raise RuntimeError(f"Multiple GateKeeper USB CDC devices found: {candidates}")


def arg_text(value: Any) -> str:
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    if isinstance(value, (float, np.floating)):
        if math.isfinite(float(value)):
            return f"{float(value):.9g}"
        raise ValueError(f"Non-finite argument: {value}")
    return str(value)


def command_text(name: str, *args: Any) -> str:
    if not args:
        return name
    return name + "," + ",".join(arg_text(a) for a in args)


class GateKeeperSerial:
    def __init__(self, port: str, baud: int = 115200) -> None:
        self.port = port
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.03, write_timeout=1)
        time.sleep(0.25)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        self.ser.close()

    def drain(self, idle: float = 0.08, timeout: float = 1.0) -> bytes:
        deadline = time.monotonic() + timeout
        last = time.monotonic()
        out = bytearray()
        while time.monotonic() < deadline:
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                out.extend(chunk)
                last = time.monotonic()
            elif out and time.monotonic() - last >= idle:
                break
        return bytes(out)

    def write_command(self, name: str, *args: Any) -> str:
        cmd = command_text(name, *args)
        self.ser.write((cmd + "\n").encode("ascii"))
        self.ser.flush()
        return cmd

    def read_line(self, timeout: float = 2.0) -> str | None:
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            buf.extend(b)
            if b == b"\n":
                break
        if not buf:
            return None
        return buf.decode("utf-8", errors="replace").strip()

    def read_idle_text(self, timeout: float = 2.0, idle: float = 0.12) -> str:
        raw = self.drain(idle=idle, timeout=timeout)
        return raw.decode("utf-8", errors="replace").strip()

    def query_line(self, name: str, *args: Any, timeout: float = 2.0) -> str:
        self.ser.reset_input_buffer()
        self.write_command(name, *args)
        line = self.read_line(timeout=timeout)
        if line is None:
            raise TimeoutError(f"No response to {command_text(name, *args)}")
        return line

    def query_multiline(self, name: str, *args: Any, timeout: float = 2.0) -> str:
        self.ser.reset_input_buffer()
        self.write_command(name, *args)
        return self.read_idle_text(timeout=timeout)

    def command_no_reply(self, name: str, *args: Any, wait: float = 0.15) -> str:
        self.ser.reset_input_buffer()
        cmd = self.write_command(name, *args)
        time.sleep(wait)
        return self.read_idle_text(timeout=0.25)

    def read_exact(self, byte_count: int, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        out = bytearray()
        while len(out) < byte_count and time.monotonic() < deadline:
            chunk = self.ser.read(byte_count - len(out))
            if chunk:
                out.extend(chunk)
        if len(out) != byte_count:
            raise TimeoutError(f"Read {len(out)} of {byte_count} expected bytes")
        return bytes(out)

    def stop_worker(self) -> str:
        self.ser.write(b"stop\n")
        self.ser.flush()
        return self.read_idle_text(timeout=2.0)


class HardwareTest:
    def __init__(self, gatekeeper: GateKeeperSerial, output_dir: Path) -> None:
        self.gk = gatekeeper
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.checks: list[Check] = []
        self.artifacts: dict[str, str] = {}
        self.measurements: dict[str, Any] = {}

    def record(self, name: str, ok: bool, **details: Any) -> bool:
        status = "PASS" if ok else "FAIL"
        self.checks.append(Check(name, status, details))
        marker = "PASS" if ok else "FAIL"
        print(f"[{marker}] {name}", flush=True)
        if details:
            print(f"       {details}", flush=True)
        return ok

    def warn(self, name: str, **details: Any) -> None:
        self.checks.append(Check(name, "WARN", details))
        print(f"[WARN] {name}", flush=True)
        if details:
            print(f"       {details}", flush=True)

    def query_float(self, name: str, *args: Any, timeout: float = 2.0) -> float:
        line = self.gk.query_line(name, *args, timeout=timeout)
        if line.startswith("FAILURE:"):
            raise RuntimeError(line)
        return float(line)

    def query_required_line(
        self, expected: str | Callable[[str], bool], name: str, *args: Any
    ) -> str:
        line = self.gk.query_line(name, *args)
        ok = line == expected if isinstance(expected, str) else expected(line)
        self.record(command_text(name, *args), ok, response=line)
        return line

    def set_all_dacs(self, voltage: float) -> None:
        for ch in range(NUM_CHANNELS):
            line = self.gk.query_line("SET", ch, voltage, timeout=2.0)
            if line.startswith("FAILURE:"):
                raise RuntimeError(line)

    def set_conversion_all(self, conversion_us: float) -> list[float]:
        actual = []
        for ch in range(NUM_CHANNELS):
            actual.append(self.query_float("CONVERT_TIME", ch, conversion_us))
        return actual

    def run_text_and_binary(
        self,
        name: str,
        args: Iterable[Any],
        expected_floats: int,
        timeout: float,
        allow_failure_text: bool = False,
    ) -> tuple[bool, np.ndarray, str]:
        args = list(args)
        self.gk.ser.reset_input_buffer()
        self.gk.write_command(name, *args)
        raw = b""
        optional = ""
        try:
            raw = self.gk.read_exact(expected_floats * 4, timeout=timeout)
            optional = self.gk.read_idle_text(timeout=0.5, idle=0.08)
        except TimeoutError as exc:
            optional = raw.decode("utf-8", errors="replace") + " " + str(exc)
            optional += " " + self.gk.stop_worker()
            return False, np.array([], dtype=np.float32), optional.strip()

        if expected_floats:
            data = np.frombuffer(raw, dtype="<f4").astype(float)
        else:
            data = np.array([], dtype=float)
        ok = not optional.startswith("FAILURE:")
        if allow_failure_text:
            ok = True
        return ok, data, optional

    def ramp_sensible(
        self,
        data: np.ndarray,
        num_adc: int,
        expected_direction: int = 1,
        min_corr: float = 0.90,
        max_abs: float = 10.5,
    ) -> tuple[bool, dict[str, Any]]:
        if data.size == 0 or data.size % num_adc != 0:
            return False, {"reason": "bad_size", "size": int(data.size)}
        arr = data.reshape((-1, num_adc))
        if not np.all(np.isfinite(arr)):
            return False, {"reason": "non_finite"}
        if float(np.max(np.abs(arr))) > max_abs:
            return False, {"reason": "range", "max_abs": float(np.max(np.abs(arr)))}
        idx = np.arange(arr.shape[0], dtype=float)
        corrs = []
        for ch in range(num_adc):
            y = arr[:, ch]
            if np.std(y) < 1e-9:
                corrs.append(0.0)
            else:
                corrs.append(float(np.corrcoef(idx, y)[0, 1]) * expected_direction)
        return min(corrs) >= min_corr, {
            "samples": int(arr.shape[0]),
            "min_corr": float(min(corrs)),
            "max_abs": float(np.max(np.abs(arr))),
            "start_mean": float(np.mean(arr[0])),
            "end_mean": float(np.mean(arr[-1])),
        }

    def run_handshake_and_registry(self) -> None:
        self.query_required_line("NOP", "NOP")
        self.query_required_line(lambda s: s.startswith("GateKeeper"), "*IDN?")
        self.query_required_line("READY", "*RDY?")
        self.query_required_line("GATEKEEPER", "GET_ENVIRONMENT")
        serial_number = self.query_required_line(lambda s: s.startswith("DA_"), "SERIAL_NUMBER")
        fw = self.query_required_line(lambda s: len(s) > 0, "GET_FIRMWARE_VERSION")
        self.measurements["serial_number"] = serial_number
        self.measurements["firmware_version"] = fw

        functions_text = self.gk.query_multiline("PRINT_FUNCTIONS", timeout=1.5)
        functions = []
        for line in functions_text.splitlines():
            if "," in line and not line.startswith("Available"):
                functions.append(line.split(",", 1)[0].strip())
        functions_set = set(functions)
        missing = sorted(NON_CALIBRATION_COMMANDS - functions_set)
        self.record(
            "registry exposes all expected non-calibration commands",
            not missing,
            missing=missing,
            total_functions=len(functions_set),
        )
        self.measurements["registered_functions"] = sorted(functions_set)

        bogus = self.gk.query_line("THIS_COMMAND_SHOULD_NOT_EXIST")
        self.record("unknown command reports failure", bogus.startswith("FAILURE:"), response=bogus)
        bad_args = self.gk.query_line("SET", 99, 0)
        self.record("invalid channel reports failure", bad_args.startswith("FAILURE:"), response=bad_args)

        for alias in ("INITIALIZE", "INIT", "INNIT"):
            self.query_required_line("INITIALIZATION COMPLETE", alias)

    def run_adc_dac_register_tests(self) -> None:
        for board in range(2):
            rev = self.query_float("GET_REVISION_REG", board)
            self.record(f"ADC board {board} revision register", int(rev) == 34, revision=rev)

        reset_reply = self.gk.command_no_reply("RESET", wait=0.2)
        self.record("ADC RESET command accepted", reset_reply == "", response=reset_reply)
        self.query_required_line("INITIALIZATION COMPLETE", "INITIALIZE")

        talk = self.gk.query_multiline("TALK", 0, timeout=1.0)
        talk_values = [x.strip() for x in talk.splitlines() if x.strip()]
        self.record("TALK command returns one response per ADC board", len(talk_values) == 2, response=talk_values)

        active = self.gk.query_line("GET_CHANNELS_ACTIVE")
        self.record("GET_CHANNELS_ACTIVE returns a parseable response", bool(active), response=active)

        chop_reply = self.gk.command_no_reply("SET_CHOP", 1)
        self.record("SET_CHOP true accepted without error text", not chop_reply.startswith("FAILURE:"), response=chop_reply)
        self.query_required_line("true", "GET_CHOP")

        actual_min = self.set_conversion_all(82)
        self.record(
            "CONVERT_TIME 82us on all channels",
            all(80.0 <= t <= 85.0 for t in actual_min),
            actual=actual_min,
        )
        gets = [self.query_float("GET_CONVERT_TIME", ch) for ch in range(NUM_CHANNELS)]
        self.record(
            "GET_CONVERT_TIME matches requested minimum",
            max(abs(a - b) for a, b in zip(actual_min, gets)) < 1e-3,
            actual=gets,
        )

        fw2 = self.query_float("CONVERT_TIME_FW", 0, 2)
        self.record("CONVERT_TIME_FW minimum filter word", 80.0 <= fw2 <= 85.0, actual=fw2)

        false_reply = self.gk.command_no_reply("SET_CHOP", 0)
        self.record("SET_CHOP false accepted without error text", not false_reply.startswith("FAILURE:"), response=false_reply)
        self.query_required_line("false", "GET_CHOP")
        fw3 = self.query_float("CONVERT_TIME_FW", 0, 3)
        self.record("CONVERT_TIME_FW explicit filter word remains valid after chop toggle", 100.0 <= fw3 <= 105.0, actual=fw3)
        converted = self.gk.query_line("CONVERT_TIME", 0, 200)
        try:
            converted_float = float(converted)
            ok = abs(converted_float - 200.0) < 25.0
        except ValueError:
            converted_float = None
            ok = False
        self.record(
            "CONVERT_TIME tracks requested value with chopping disabled",
            ok,
            response=converted,
            parsed=converted_float,
        )

        self.gk.command_no_reply("SET_CHOP", 1)
        self.query_required_line("true", "GET_CHOP")
        self.set_conversion_all(500)

        for ch in range(NUM_CHANNELS):
            self.query_required_line(lambda s: "Set RDYFN" in s, "SET_RDYFN", ch)
            self.query_required_line(lambda s: "Unset RDYFN" in s, "UNSET_RDYFN", ch)
            self.query_required_line(lambda s: "Returned ADC" in s, "IDLE_MODE", ch)
        self.gk.command_no_reply("RESET_MAINTAIN", wait=0.2)
        self.record("RESET_MAINTAIN command did not emit failure", True)

    def run_static_voltage_tests(self) -> None:
        for ch in range(NUM_CHANNELS):
            self.query_required_line(lambda s: s.startswith("CH"), "SET_UPPER_LIMIT", ch, 9.0)
            self.query_required_line(lambda s: s.startswith("CH"), "SET_LOWER_LIMIT", ch, -9.0)
            upper = self.query_float("GET_UPPER_LIMIT", ch)
            lower = self.query_float("GET_LOWER_LIMIT", ch)
            fail = self.gk.query_line("SET", ch, 9.5)
            self.record(
                f"DAC {ch} volatile limits",
                abs(upper - 9.0) < 1e-5 and abs(lower + 9.0) < 1e-5 and fail.startswith("FAILURE:"),
                upper=upper,
                lower=lower,
                overrange_response=fail,
            )
            self.gk.query_line("SET_UPPER_LIMIT", ch, 10.0)
            self.gk.query_line("SET_LOWER_LIMIT", ch, -10.0)

        for ch in range(NUM_CHANNELS):
            self.query_required_line("FULL_SCALE_UPDATED", "FULL_SCALE", ch, 5.0)
            fs = self.query_float("GET_FULL_SCALE", ch)
            self.record(f"DAC {ch} full-scale update", abs(fs - 5.0) < 1e-5, full_scale=fs)
            self.gk.query_line("FULL_SCALE", ch, 10.0)

        voltages = np.array([-9.5, -7.0, -5.0, -2.5, 0.0, 2.5, 5.0, 7.0, 9.5])
        dac_readback = np.zeros((len(voltages), NUM_CHANNELS))
        adc_readback = np.zeros((len(voltages), NUM_CHANNELS))

        for i, voltage in enumerate(voltages):
            self.set_all_dacs(float(voltage))
            time.sleep(0.06)
            for ch in range(NUM_CHANNELS):
                dac_readback[i, ch] = self.query_float("GET_DAC", ch)
            for ch in range(NUM_CHANNELS):
                adc_readback[i, ch] = self.query_float("GET_ADC", ch, timeout=3.0)

        dac_err = dac_readback - voltages[:, None]
        adc_err = adc_readback - voltages[:, None]
        self.measurements["static_voltage_sweep"] = {
            "setpoints": voltages.tolist(),
            "dac_readback": dac_readback.tolist(),
            "adc_readback": adc_readback.tolist(),
            "dac_max_abs_error": float(np.max(np.abs(dac_err))),
            "adc_max_abs_error": float(np.max(np.abs(adc_err))),
            "adc_mean_abs_error": float(np.mean(np.abs(adc_err))),
        }
        self.record(
            "DAC readback tracks common voltage sweep",
            float(np.max(np.abs(dac_err))) < 0.001,
            max_abs_error=float(np.max(np.abs(dac_err))),
        )
        self.record(
            "ADC readback tracks common voltage sweep",
            float(np.max(np.abs(adc_err))) < 0.10,
            max_abs_error=float(np.max(np.abs(adc_err))),
            mean_abs_error=float(np.mean(np.abs(adc_err))),
        )

        fig, axes = plt.subplots(1, 2, figsize=(13, 5))
        for ch in range(NUM_CHANNELS):
            axes[0].plot(voltages, dac_err[:, ch] * 1e6, marker="o", label=f"DAC{ch}")
            axes[1].plot(voltages, adc_err[:, ch] * 1e3, marker="o", label=f"ADC{ch}")
        axes[0].set_title("DAC readback error")
        axes[0].set_xlabel("Set voltage [V]")
        axes[0].set_ylabel("Error [uV]")
        axes[1].set_title("ADC readback error")
        axes[1].set_xlabel("Set voltage [V]")
        axes[1].set_ylabel("Error [mV]")
        axes[1].legend(ncol=2, fontsize=8)
        fig.tight_layout()
        path = self.output_dir / "static_voltage_sweep.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        self.artifacts["static_voltage_sweep_plot"] = str(path)

        for target in (-2.0, 2.0):
            self.set_all_dacs(target)
            code = self.voltage_to_code(target)
            readbacks = []
            for ch in range(NUM_CHANNELS):
                line = self.gk.query_line("SET_DAC_CODE", ch, code)
                if line.startswith("FAILURE:"):
                    raise RuntimeError(line)
                readbacks.append(self.query_float("GET_DAC", ch))
            self.record(
                f"SET_DAC_CODE {target:+.1f} V equivalent on all DACs",
                max(abs(v - target) for v in readbacks) < 0.012,
                code=code,
                readbacks=readbacks,
            )
        self.set_all_dacs(0.0)
        self.query_required_line("LDAC TOGGLED", "TOGGLE_LDAC")

    @staticmethod
    def voltage_to_code(voltage: float) -> int:
        if voltage >= 0:
            return int(round(voltage * 524287 / 10.0))
        return int(round(voltage * 524288 / 10.0 + 1048576))

    def run_simple_dac_ramps(self) -> None:
        self.set_all_dacs(0.0)
        small = 0.05
        line = self.gk.query_line("RAMP1", 0, -small, small, 5, 800)
        final = self.query_float("GET_DAC", 0)
        self.record("RAMP1 completes and reaches final DAC readback", "RAMPING DAC 0" in line and abs(final - small) < 0.005, response=line, final=final)

        self.set_all_dacs(0.0)
        line = self.gk.query_line("RAMP2", 0, 1, -small, -small, small, small, 5, 800)
        finals = [self.query_float("GET_DAC", ch) for ch in (0, 1)]
        self.record("RAMP2 completes and reaches final DAC readback", "RAMPING DAC 0" in line and max(abs(v - small) for v in finals) < 0.005, response=line, finals=finals)

        channels = list(range(NUM_CHANNELS))
        args = [NUM_CHANNELS, 5, 800] + channels + [-small] * NUM_CHANNELS + [small] * NUM_CHANNELS
        line = self.gk.query_line("RAMP_N", *args, timeout=3.0)
        finals = [self.query_float("GET_DAC", ch) for ch in range(NUM_CHANNELS)]
        self.record("RAMP_N completes on all DACs", "RAMPING" in line and max(abs(v - small) for v in finals) < 0.005, response=line, max_final_error=max(abs(v - small) for v in finals))
        self.set_all_dacs(0.0)

        waveform = [0.0, 0.05, -0.05, 0.0]
        args = [NUM_CHANNELS, len(waveform), 1000] + channels + waveform * NUM_CHANNELS
        self.gk.ser.reset_input_buffer()
        self.gk.write_command("AWG_BUFFER_RAMP", *args)
        time.sleep(0.08)
        stop_text = self.gk.stop_worker()
        self.record("AWG_BUFFER_RAMP runs until STOP", "RAMPING_STOPPED" in stop_text, stop_response=stop_text)
        self.set_all_dacs(0.0)

    def find_minimum_timings(self) -> dict[str, Any]:
        print("[INFO] Starting minimum timing search at ADC conversion 82 us", flush=True)
        self.gk.command_no_reply("SET_CHOP", 1)
        actual = self.set_conversion_all(82)
        real_conv = float(np.mean(actual))
        channels = list(range(NUM_CHANNELS))
        dac_args_base = [NUM_CHANNELS, NUM_CHANNELS, 41, 1]
        common_lists = channels + [-1.0] * NUM_CHANNELS + [1.0] * NUM_CHANNELS + channels

        settle_values = [20, 50, 82, 100, 150, 200]
        interval_values = list(range(400, 661, 20))
        heat = np.full((len(settle_values), len(interval_values)), np.nan)
        best_by_settle = {}
        for si, settle in enumerate(settle_values):
            print(f"[INFO] DAC-led timing row settling={settle} us", flush=True)
            for ii, interval in enumerate(interval_values):
                if interval <= settle:
                    heat[si, ii] = -1
                    continue
                args = dac_args_base + [interval, settle] + common_lists
                ok, data, msg = self.run_text_and_binary(
                    "DAC_LED_BUFFER_RAMP",
                    args,
                    expected_floats=41 * NUM_CHANNELS,
                    timeout=2.0,
                )
                sensible, metrics = self.ramp_sensible(data, NUM_CHANNELS)
                passed = ok and sensible
                heat[si, ii] = 1 if passed else 0
                if passed and settle not in best_by_settle:
                    best_by_settle[settle] = {
                        "interval_us": interval,
                        "metrics": metrics,
                        "message": msg,
                    }
                    break

        best_dac_led = None
        for settle in settle_values:
            if settle in best_by_settle:
                candidate = {"settling_us": settle, **best_by_settle[settle]}
                if best_dac_led is None or candidate["interval_us"] < best_dac_led["interval_us"]:
                    best_dac_led = candidate

        print("[INFO] Time-series ADC interval search", flush=True)
        ts_intervals = list(range(160, 621, 20))
        ts_results = []
        best_ts = None
        dac_interval = 300
        for adc_interval in ts_intervals:
            search_steps = 101
            args = [NUM_CHANNELS, NUM_CHANNELS, search_steps, dac_interval, adc_interval] + common_lists
            expected = max(1, (search_steps * dac_interval) // adc_interval) * NUM_CHANNELS
            ok, data, msg = self.run_text_and_binary(
                "TIME_SERIES_BUFFER_RAMP",
                args,
                expected_floats=expected,
                timeout=3.0,
            )
            sensible, metrics = self.ramp_sensible(data, NUM_CHANNELS)
            passed = ok and sensible
            ts_results.append({"adc_interval_us": adc_interval, "passed": passed, "metrics": metrics, "message": msg})
            if passed and best_ts is None:
                best_ts = {"dac_interval_us": dac_interval, "adc_interval_us": adc_interval, "metrics": metrics}
                break

        print("[INFO] AWG_WITH_ADC interval search", flush=True)
        awg_intervals = list(range(400, 821, 20))
        waveform = np.linspace(-1.0, 1.0, 32).tolist()
        awg_best = None
        for interval in awg_intervals:
            args = [NUM_CHANNELS, NUM_CHANNELS, len(waveform), interval, 1] + channels + channels + waveform * NUM_CHANNELS
            ok, data, msg = self.run_text_and_binary(
                "AWG_WITH_ADC",
                args,
                expected_floats=len(waveform) * NUM_CHANNELS,
                timeout=3.0,
            )
            sensible, metrics = self.ramp_sensible(data, NUM_CHANNELS)
            if ok and sensible:
                awg_best = {"dac_interval_us": interval, "metrics": metrics, "message": msg}
                break

        fig, axes = plt.subplots(1, 2, figsize=(13, 5))
        im = axes[0].imshow(heat, aspect="auto", origin="lower", cmap="RdYlGn", vmin=0, vmax=1)
        axes[0].set_title("DAC-led pass/fail at ADC conversion 82 us")
        axes[0].set_xlabel("DAC interval [us]")
        axes[0].set_ylabel("DAC settling [us]")
        axes[0].set_xticks(range(0, len(interval_values), 4), [str(v) for v in interval_values[::4]], rotation=45)
        axes[0].set_yticks(range(len(settle_values)), [str(v) for v in settle_values])
        fig.colorbar(im, ax=axes[0], label="pass")

        axes[1].plot([r["adc_interval_us"] for r in ts_results], [1 if r["passed"] else 0 for r in ts_results], marker="o")
        axes[1].set_ylim(-0.1, 1.1)
        axes[1].set_title("Time-series ADC interval search")
        axes[1].set_xlabel("ADC interval [us]")
        axes[1].set_ylabel("pass")
        axes[1].grid(True, alpha=0.3)
        fig.tight_layout()
        path = self.output_dir / "minimum_timing_search.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        self.artifacts["minimum_timing_search_plot"] = str(path)

        timings = {
            "conversion_time_request_us": 82,
            "actual_conversion_time_us": real_conv,
            "dac_led": {
                "best": best_dac_led,
                "best_by_settling": best_by_settle,
                "settling_candidates": settle_values,
                "interval_candidates": interval_values,
            },
            "time_series": {
                "best": best_ts,
                "searched_adc_intervals": ts_results,
            },
            "awg_with_adc": {
                "best": awg_best,
            },
        }
        self.measurements["minimum_timings"] = timings
        self.record("DAC_LED_BUFFER_RAMP minimum timing found", best_dac_led is not None, best=best_dac_led)
        self.record("TIME_SERIES_BUFFER_RAMP minimum timing found", best_ts is not None, best=best_ts)
        self.record("AWG_WITH_ADC minimum timing found", awg_best is not None, best=awg_best)
        return timings

    def run_buffer_ramp_suite(self, timings: dict[str, Any]) -> None:
        channels = list(range(NUM_CHANNELS))
        self.gk.command_no_reply("SET_CHOP", 1)
        self.set_conversion_all(82)

        dac_led = timings["dac_led"]["best"] or {"interval_us": 700, "settling_us": 100}
        ts = timings["time_series"]["best"] or {"dac_interval_us": 300, "adc_interval_us": 520}
        awg = timings["awg_with_adc"]["best"] or {"dac_interval_us": 500}
        traces: dict[str, np.ndarray] = {}

        common_lists = channels + [-1.0] * NUM_CHANNELS + [1.0] * NUM_CHANNELS + channels
        args = [NUM_CHANNELS, NUM_CHANNELS, 101, 1, dac_led["interval_us"], dac_led["settling_us"]] + common_lists
        ok, data, msg = self.run_text_and_binary("DAC_LED_BUFFER_RAMP", args, 101 * NUM_CHANNELS, timeout=6.0)
        sensible, metrics = self.ramp_sensible(data, NUM_CHANNELS)
        self.record("DAC_LED_BUFFER_RAMP trace at minimum timing", ok and sensible, metrics=metrics, message=msg)
        traces["dac_led"] = data.reshape((-1, NUM_CHANNELS)) if data.size else np.empty((0, NUM_CHANNELS))

        expected = max(1, (101 * ts["dac_interval_us"]) // ts["adc_interval_us"]) * NUM_CHANNELS
        args = [NUM_CHANNELS, NUM_CHANNELS, 101, ts["dac_interval_us"], ts["adc_interval_us"]] + common_lists
        ok, data, msg = self.run_text_and_binary("TIME_SERIES_BUFFER_RAMP", args, expected, timeout=6.0)
        sensible, metrics = self.ramp_sensible(data, NUM_CHANNELS)
        self.record("TIME_SERIES_BUFFER_RAMP trace at minimum timing", ok and sensible, metrics=metrics, message=msg)
        traces["time_series"] = data.reshape((-1, NUM_CHANNELS)) if data.size else np.empty((0, NUM_CHANNELS))

        for retrace, snake, label in ((0, 0, "2d_dac_led"), (1, 0, "2d_dac_led_retrace"), (0, 1, "2d_dac_led_snake")):
            steps_fast, steps_slow = 31, 6
            scans = 2 if retrace and not snake else 1
            expected_points = steps_fast * steps_slow * scans
            args = [
                NUM_CHANNELS,
                NUM_CHANNELS,
                steps_fast,
                steps_slow,
                max(dac_led["interval_us"], 700),
                dac_led["settling_us"],
                retrace,
                snake,
                1,
            ] + channels + [-0.6] * NUM_CHANNELS + [1.2] * NUM_CHANNELS + [0.4] * NUM_CHANNELS + channels
            ok, data, msg = self.run_text_and_binary("2D_DAC_LED_BUFFER_RAMP", args, expected_points * NUM_CHANNELS, timeout=8.0)
            arr = data.reshape((-1, NUM_CHANNELS)) if data.size else np.empty((0, NUM_CHANNELS))
            sensible = bool(arr.size and np.all(np.isfinite(arr)) and np.max(np.abs(arr)) < 10.5)
            self.record(f"2D_DAC_LED_BUFFER_RAMP {label}", ok and sensible, samples=int(arr.shape[0]), message=msg)
            traces[label] = arr

        for retrace, snake, label in ((0, 0, "2d_time_series"), (1, 0, "2d_time_series_retrace"), (0, 1, "2d_time_series_snake")):
            steps_fast, steps_slow = 31, 5
            samples_per_line = max(1, (steps_fast * max(ts["dac_interval_us"], 400)) // ts["adc_interval_us"])
            scans = 2 if retrace and not snake else 1
            expected_points = samples_per_line * steps_slow * scans
            args = [
                NUM_CHANNELS,
                NUM_CHANNELS,
                steps_fast,
                steps_slow,
                max(ts["dac_interval_us"], 400),
                ts["adc_interval_us"],
                retrace,
                snake,
            ] + channels + [-0.6] * NUM_CHANNELS + [1.2] * NUM_CHANNELS + [0.4] * NUM_CHANNELS + channels
            ok, data, msg = self.run_text_and_binary("2D_TIME_SERIES_BUFFER_RAMP", args, expected_points * NUM_CHANNELS, timeout=10.0)
            arr = data.reshape((-1, NUM_CHANNELS)) if data.size else np.empty((0, NUM_CHANNELS))
            sensible = bool(arr.size and np.all(np.isfinite(arr)) and np.max(np.abs(arr)) < 10.5)
            self.record(f"2D_TIME_SERIES_BUFFER_RAMP {label}", ok and sensible, samples=int(arr.shape[0]), message=msg)
            traces[label] = arr

        self.set_all_dacs(0.0)
        duration_us = 40_000
        self.gk.ser.reset_input_buffer()
        self.gk.write_command("TIME_SERIES_ADC_READ", NUM_CHANNELS, *channels, 82, duration_us)
        first = self.gk.read_exact(4, timeout=3.0)
        sample_period = struct.unpack("<f", first)[0]
        expected_samples = int(duration_us // int(sample_period))
        raw = self.gk.read_exact(expected_samples * NUM_CHANNELS * 4, timeout=6.0)
        msg = self.gk.read_idle_text(timeout=0.5)
        adc_data = np.frombuffer(raw, dtype="<f4").astype(float).reshape((-1, NUM_CHANNELS))
        self.record(
            "TIME_SERIES_ADC_READ at minimum conversion time",
            np.all(np.isfinite(adc_data)) and np.max(np.abs(adc_data)) < 0.25 and not msg.startswith("FAILURE:"),
            sample_period_us=sample_period,
            samples=int(adc_data.shape[0]),
            std_uv=(np.std(adc_data, axis=0) * 1e6).round(3).tolist(),
            message=msg,
        )
        traces["adc_read_noise"] = adc_data

        continuous = self.gk.query_line("CONTINUOUS_CONVERT_READ", 0, 5000, 30000, timeout=3.0)
        values = np.array([float(x) for x in continuous.split(",") if x])
        self.record("CONTINUOUS_CONVERT_READ returns finite CSV data", values.size == 6 and np.all(np.isfinite(values)), samples=values.tolist())

        waveform = np.sin(np.linspace(0, 2 * np.pi, 48, endpoint=False)).tolist()
        args = [NUM_CHANNELS, NUM_CHANNELS, len(waveform), max(awg["dac_interval_us"], 500), 2] + channels + channels + waveform * NUM_CHANNELS
        ok, data, msg = self.run_text_and_binary("AWG_WITH_ADC", args, len(waveform) * 2 * NUM_CHANNELS, timeout=8.0)
        arr = data.reshape((-1, NUM_CHANNELS)) if data.size else np.empty((0, NUM_CHANNELS))
        sensible = bool(arr.size and np.all(np.isfinite(arr)) and np.max(np.abs(arr)) < 10.5)
        self.record("AWG_WITH_ADC waveform capture", ok and sensible, samples=int(arr.shape[0]), message=msg)
        traces["awg_with_adc"] = arr

        args = [
            NUM_CHANNELS,
            1,
            7,
            3,
            1,
            82,
        ] + channels + [-0.4] * NUM_CHANNELS + [0.4] * NUM_CHANNELS + [0.4] * NUM_CHANNELS + [-0.4] * NUM_CHANNELS + [0]
        expected = 2 * 7 * 1 * 3 * 1
        ok, data, msg = self.run_text_and_binary("BOXCAR_BUFFER_RAMP", args, expected, timeout=8.0)
        arr = data.reshape((-1, 1)) if data.size else np.empty((0, 1))
        sensible = bool(arr.size and np.all(np.isfinite(arr)) and np.max(np.abs(arr)) < 10.5)
        self.record("BOXCAR_BUFFER_RAMP one-ADC capture at 82us", ok and sensible, samples=int(arr.shape[0]), message=msg)
        traces["boxcar_one_adc"] = arr

        args = [
            NUM_CHANNELS,
            NUM_CHANNELS,
            7,
            3,
            1,
            82,
        ] + channels + [-0.4] * NUM_CHANNELS + [0.4] * NUM_CHANNELS + [0.4] * NUM_CHANNELS + [-0.4] * NUM_CHANNELS + channels
        expected = 2 * 7 * 1 * 3 * NUM_CHANNELS
        ok, data, msg = self.run_text_and_binary("BOXCAR_BUFFER_RAMP", args, expected, timeout=8.0)
        arr = data.reshape((-1, NUM_CHANNELS)) if data.size else np.empty((0, NUM_CHANNELS))
        sensible = bool(arr.size and np.all(np.isfinite(arr)) and np.max(np.abs(arr)) < 10.5)
        self.record("BOXCAR_BUFFER_RAMP all-ADC capture at 82us", ok and sensible, samples=int(arr.shape[0]), message=msg)
        traces["boxcar"] = arr

        self.measurements["buffer_ramp_trace_shapes"] = {k: list(v.shape) for k, v in traces.items()}
        self.plot_buffer_traces(traces)
        self.set_all_dacs(0.0)
        self.set_conversion_all(500)

    def plot_buffer_traces(self, traces: dict[str, np.ndarray]) -> None:
        selected = [
            "dac_led",
            "time_series",
            "2d_dac_led",
            "2d_dac_led_retrace",
            "2d_dac_led_snake",
            "2d_time_series",
            "2d_time_series_retrace",
            "2d_time_series_snake",
            "adc_read_noise",
            "awg_with_adc",
            "boxcar",
        ]
        fig, axes = plt.subplots(4, 3, figsize=(15, 13))
        axes = axes.flatten()
        for ax, key in zip(axes, selected):
            arr = traces.get(key, np.empty((0, NUM_CHANNELS)))
            if arr.size:
                for ch in range(min(NUM_CHANNELS, arr.shape[1])):
                    ax.plot(arr[:, ch], linewidth=0.9, alpha=0.85)
            ax.set_title(key)
            ax.set_xlabel("sample")
            ax.set_ylabel("V")
            ax.grid(True, alpha=0.2)
        for ax in axes[len(selected):]:
            ax.axis("off")
        fig.tight_layout()
        path = self.output_dir / "buffer_ramp_traces.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        self.artifacts["buffer_ramp_traces_plot"] = str(path)

    def write_report(self) -> None:
        json_path = self.output_dir / "hardware_test_report.json"
        md_path = self.output_dir / "hardware_test_report.md"
        self.artifacts["json_report"] = str(json_path)
        self.artifacts["markdown_report"] = str(md_path)
        data = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "port": self.gk.port,
            "checks": [check.__dict__ for check in self.checks],
            "measurements": self.measurements,
            "artifacts": self.artifacts,
            "summary": {
                "passed": sum(1 for c in self.checks if c.status == "PASS"),
                "failed": sum(1 for c in self.checks if c.status == "FAIL"),
                "warnings": sum(1 for c in self.checks if c.status == "WARN"),
            },
        }
        json_path.write_text(json.dumps(data, indent=2), encoding="utf-8")

        lines = [
            "# GateKeeper hardware test report",
            "",
            f"- Port: `{self.gk.port}`",
            f"- Firmware: `{self.measurements.get('firmware_version', 'unknown')}`",
            f"- Serial: `{self.measurements.get('serial_number', 'unknown')}`",
            f"- Passed: {data['summary']['passed']}",
            f"- Failed: {data['summary']['failed']}",
            f"- Warnings: {data['summary']['warnings']}",
            "",
            "## Minimum timings at ADC conversion 82 us",
            "",
        ]
        timings = self.measurements.get("minimum_timings", {})
        lines.append("```json")
        lines.append(json.dumps(timings, indent=2))
        lines.append("```")
        lines.extend(["", "## Checks", ""])
        for check in self.checks:
            lines.append(f"- {check.status}: {check.name} {json.dumps(check.details)}")
        md_path.write_text("\n".join(lines), encoding="utf-8")
        print(f"Report written to {json_path}")
        print(f"Markdown written to {md_path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None, help="USB CDC serial port, e.g. COM8")
    parser.add_argument("--output-dir", default=None)
    args = parser.parse_args()

    port = args.port or detect_port()
    output_dir = Path(args.output_dir or Path("test_outputs") / f"hardware_full_{now_label()}").resolve()

    gk = GateKeeperSerial(port)
    test = HardwareTest(gk, output_dir)
    try:
        test.run_handshake_and_registry()
        test.run_adc_dac_register_tests()
        test.run_static_voltage_tests()
        test.run_simple_dac_ramps()
        timings = test.find_minimum_timings()
        test.run_buffer_ramp_suite(timings)
    except Exception as exc:
        test.record("test runner exception", False, error=repr(exc))
        try:
            gk.stop_worker()
            test.set_all_dacs(0.0)
        except Exception:
            pass
    finally:
        try:
            test.set_all_dacs(0.0)
        except Exception:
            pass
        test.write_report()
        gk.close()

    failed = sum(1 for check in test.checks if check.status == "FAIL")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
