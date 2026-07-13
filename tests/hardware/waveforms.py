from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class WaveformMetrics:
    max_abs_error: float
    rmse_by_channel: np.ndarray
    best_expected_channel: np.ndarray


def linear_ramp(starts, ends, steps: int) -> np.ndarray:
    starts = np.asarray(starts, dtype=float)
    ends = np.asarray(ends, dtype=float)
    fractions = np.linspace(0.0, 1.0, steps)[:, None]
    return starts + fractions * (ends - starts)


def raster_scan(
    start,
    fast_axis,
    slow_axis,
    steps_fast: int,
    steps_slow: int,
    retrace: bool = False,
    snake: bool = False,
) -> np.ndarray:
    start = np.asarray(start, dtype=float)
    fast_axis = np.asarray(fast_axis, dtype=float)
    slow_axis = np.asarray(slow_axis, dtype=float)
    scans_per_slow_step = 2 if retrace and not snake else 1
    points = []
    for slow_step in range(steps_slow):
        for scan in range(scans_per_slow_step):
            reverse = (retrace and not snake and scan == 1) or (
                snake and slow_step % 2 == 1
            )
            fractions = np.linspace(1.0, 0.0, steps_fast) if reverse else np.linspace(0.0, 1.0, steps_fast)
            slow_fraction = slow_step / (steps_slow - 1) if steps_slow > 1 else 0.0
            for fast_fraction in fractions:
                points.append(
                    start + slow_fraction * slow_axis + fast_fraction * fast_axis
                )
    return np.asarray(points)


def sampled_steps(points: np.ndarray, samples_per_step: int) -> tuple[np.ndarray, np.ndarray]:
    if samples_per_step < 1:
        raise ValueError("samples_per_step must be positive")
    expected = np.repeat(np.asarray(points, dtype=float), samples_per_step, axis=0)
    phase = np.arange(expected.shape[0]) % samples_per_step
    stable = (phase >= 1) & (phase < samples_per_step - 1)
    return expected, stable


def waveform_metrics(measured: np.ndarray, expected: np.ndarray) -> WaveformMetrics:
    measured = np.asarray(measured, dtype=float)
    expected = np.asarray(expected, dtype=float)
    if measured.shape != expected.shape:
        raise AssertionError(f"shape {measured.shape} != expected {expected.shape}")
    if measured.ndim != 2 or measured.shape[0] < 2:
        raise AssertionError(f"waveform must be a 2D array with multiple samples: {measured.shape}")
    if not np.all(np.isfinite(measured)):
        raise AssertionError("waveform contains non-finite samples")

    error = measured - expected
    rmse_by_channel = np.sqrt(np.mean(error * error, axis=0))
    matching_rmse = np.empty((measured.shape[1], expected.shape[1]))
    for measured_channel in range(measured.shape[1]):
        for expected_channel in range(expected.shape[1]):
            delta = measured[:, measured_channel] - expected[:, expected_channel]
            matching_rmse[measured_channel, expected_channel] = np.sqrt(
                np.mean(delta * delta)
            )
    return WaveformMetrics(
        max_abs_error=float(np.max(np.abs(error))),
        rmse_by_channel=rmse_by_channel,
        best_expected_channel=np.argmin(matching_rmse, axis=1),
    )


def assert_tracks_expected(
    measured: np.ndarray,
    expected: np.ndarray,
    *,
    max_abs_error: float = 0.08,
    max_rmse: float = 0.035,
    require_channel_mapping: bool = True,
) -> WaveformMetrics:
    metrics = waveform_metrics(measured, expected)
    if metrics.max_abs_error > max_abs_error:
        raise AssertionError(
            f"maximum error {metrics.max_abs_error:.6f} V exceeds {max_abs_error:.6f} V"
        )
    if np.max(metrics.rmse_by_channel) > max_rmse:
        raise AssertionError(
            f"channel RMSE {metrics.rmse_by_channel.tolist()} exceeds {max_rmse:.6f} V"
        )
    if require_channel_mapping:
        expected_mapping = np.arange(measured.shape[1])
        if not np.array_equal(metrics.best_expected_channel, expected_mapping):
            raise AssertionError(
                "channel mapping mismatch: "
                f"best expected channels {metrics.best_expected_channel.tolist()}"
            )
    return metrics
