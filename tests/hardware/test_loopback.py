import time

import numpy as np
import pytest

from .protocol import NUM_CHANNELS
from .waveforms import (
    assert_tracks_expected,
    linear_ramp,
    raster_scan,
    sampled_steps,
)


pytestmark = [pytest.mark.hardware, pytest.mark.loopback]
CHANNELS = list(range(NUM_CHANNELS))
STARTS = np.array([-1.4, -1.0, -0.6, -0.2, 0.2, 0.6, 1.0, 1.4])
ENDS = np.array([1.1, 0.1, -0.9, 1.4, -1.2, 0.8, -0.3, -1.5])


def set_dacs(client, values):
    for channel, voltage in enumerate(values):
        client.query("SET", channel, float(voltage))


def read_adcs(client):
    return np.array(
        [client.query_float("GET_ADC", channel, timeout=3.0) for channel in CHANNELS]
    )


def set_conversion_times(client, requested_us=200):
    return np.array(
        [client.query_float("CONVERT_TIME", channel, requested_us) for channel in CHANNELS]
    )


@pytest.mark.parametrize(
    "targets",
    [
        STARTS,
        ENDS,
        np.array([0.75, -1.25, 1.5, -0.5, 1.0, -1.5, 0.25, -0.9]),
    ],
)
def test_static_indexed_loopback(gatekeeper, targets):
    set_dacs(gatekeeper, targets)
    time.sleep(0.06)
    measured = read_adcs(gatekeeper)
    error = measured - targets
    assert np.all(np.isfinite(measured))
    assert np.max(np.abs(error)) < 0.08, error.tolist()
    assert np.sqrt(np.mean(error * error)) < 0.035


def test_point_to_point_ramps(gatekeeper):
    set_dacs(gatekeeper, np.zeros(NUM_CHANNELS))
    response = gatekeeper.query("RAMP_N", NUM_CHANNELS, 21, 800, *CHANNELS, *STARTS, *ENDS, timeout=4.0)
    assert "RAMPING" in response
    time.sleep(0.03)
    measured = read_adcs(gatekeeper)
    assert np.max(np.abs(measured - ENDS)) < 0.08


def test_dac_led_ramp_tracks_all_channels(gatekeeper):
    set_conversion_times(gatekeeper)
    steps = 41
    args = [NUM_CHANNELS, NUM_CHANNELS, steps, 2, 3500, 500]
    args += CHANNELS + STARTS.tolist() + ENDS.tolist() + CHANNELS
    measured = gatekeeper.binary_command(
        "DAC_LED_BUFFER_RAMP", args, steps, NUM_CHANNELS, timeout=8.0
    )
    assert_tracks_expected(measured, linear_ramp(STARTS, ENDS, steps))


def test_time_series_ramp_tracks_stable_samples(gatekeeper):
    set_conversion_times(gatekeeper)
    steps, samples_per_step = 25, 5
    dac_interval, adc_interval = 6000, 1200
    args = [NUM_CHANNELS, NUM_CHANNELS, steps, dac_interval, adc_interval]
    args += CHANNELS + STARTS.tolist() + ENDS.tolist() + CHANNELS
    measured = gatekeeper.binary_command(
        "TIME_SERIES_BUFFER_RAMP",
        args,
        steps * samples_per_step,
        NUM_CHANNELS,
        timeout=10.0,
    )
    expected, stable = sampled_steps(
        linear_ramp(STARTS, ENDS, steps), samples_per_step
    )
    assert_tracks_expected(measured[stable], expected[stable])


@pytest.mark.parametrize("retrace,snake", [(False, False), (True, False), (False, True)])
def test_dac_led_2d_scan_modes(gatekeeper, retrace, snake):
    set_conversion_times(gatekeeper)
    start = STARTS * 0.6
    fast = np.array([0.9, -0.7, 0.6, 0.8, -0.6, 0.5, -0.8, 0.7])
    slow = np.array([0.3, 0.4, -0.3, 0.2, 0.5, -0.4, 0.25, -0.35])
    steps_fast, steps_slow = 9, 4
    expected = raster_scan(start, fast, slow, steps_fast, steps_slow, retrace, snake)
    args = [NUM_CHANNELS, NUM_CHANNELS, steps_fast, steps_slow, 3500, 500, retrace, snake, 2]
    args += CHANNELS + start.tolist() + fast.tolist() + slow.tolist() + CHANNELS
    measured = gatekeeper.binary_command(
        "2D_DAC_LED_BUFFER_RAMP", args, len(expected), NUM_CHANNELS, timeout=10.0
    )
    assert_tracks_expected(measured, expected)


@pytest.mark.parametrize("retrace,snake", [(False, False), (True, False), (False, True)])
def test_time_series_2d_scan_modes(gatekeeper, retrace, snake):
    set_conversion_times(gatekeeper)
    start = STARTS * 0.6
    fast = np.array([0.9, -0.7, 0.6, 0.8, -0.6, 0.5, -0.8, 0.7])
    slow = np.array([0.3, 0.4, -0.3, 0.2, 0.5, -0.4, 0.25, -0.35])
    steps_fast, steps_slow, samples_per_step = 9, 3, 4
    points = raster_scan(start, fast, slow, steps_fast, steps_slow, retrace, snake)
    expected, stable = sampled_steps(points, samples_per_step)
    args = [NUM_CHANNELS, NUM_CHANNELS, steps_fast, steps_slow, 6000, 1500, retrace, snake]
    args += CHANNELS + start.tolist() + fast.tolist() + slow.tolist() + CHANNELS
    measured = gatekeeper.binary_command(
        "2D_TIME_SERIES_BUFFER_RAMP", args, len(expected), NUM_CHANNELS, timeout=12.0
    )
    assert_tracks_expected(measured[stable], expected[stable])


def test_awg_with_adc_tracks_channel_major_waveforms(gatekeeper):
    set_conversion_times(gatekeeper)
    steps, cycles = 32, 2
    phase = np.arange(NUM_CHANNELS) * 0.47
    theta = np.linspace(0, 2 * np.pi, steps, endpoint=False)[:, None]
    waveform = 0.12 * np.arange(NUM_CHANNELS)[None, :] - 0.42
    waveform = waveform + 0.65 * np.sin(theta + phase[None, :])
    args = [NUM_CHANNELS, NUM_CHANNELS, steps, 3500, cycles]
    args += CHANNELS + CHANNELS + waveform.T.reshape(-1).tolist()
    measured = gatekeeper.binary_command(
        "AWG_WITH_ADC", args, steps * cycles, NUM_CHANNELS, timeout=10.0
    )
    expected = np.tile(waveform, (cycles, 1))
    assert_tracks_expected(
        measured, expected, max_abs_error=0.12, max_rmse=0.055
    )


def test_dac_only_awg_stops_cleanly(gatekeeper):
    steps = 16
    waveform = linear_ramp(STARTS * 0.4, ENDS * 0.4, steps)
    args = [NUM_CHANNELS, steps, 1000]
    args += CHANNELS + waveform.T.reshape(-1).tolist()
    gatekeeper.start_command("AWG_BUFFER_RAMP", *args)
    time.sleep(0.08)
    response = gatekeeper.stop()
    assert "RAMPING_STOPPED" in response
    readback = np.array(
        [gatekeeper.query_float("GET_DAC", channel) for channel in CHANNELS]
    )
    for channel in CHANNELS:
        assert np.min(np.abs(waveform[:, channel] - readback[channel])) < 0.01


def test_boxcar_preserves_low_high_sequence(gatekeeper):
    set_conversion_times(gatekeeper, 500)
    steps, measures = 5, 3
    low_start = STARTS * 0.5
    low_end = low_start + 0.35
    high_start = ENDS * 0.45
    high_end = high_start - 0.3
    args = [NUM_CHANNELS, NUM_CHANNELS, steps, measures, 1, 2000]
    args += CHANNELS + low_start.tolist() + low_end.tolist()
    args += high_start.tolist() + high_end.tolist() + CHANNELS
    measured = gatekeeper.binary_command(
        "BOXCAR_BUFFER_RAMP", args, 2 * steps * measures, NUM_CHANNELS, timeout=12.0
    )
    low = linear_ramp(low_start, low_end, steps)
    high = linear_ramp(high_start, high_end, steps)
    expected = np.vstack(
        [np.repeat(values[None, :], measures, axis=0) for pair in zip(low, high) for values in pair]
    )
    assert_tracks_expected(
        measured, expected, max_abs_error=0.12, max_rmse=0.055
    )


def test_streamed_adc_read_matches_indexed_outputs(gatekeeper):
    targets = np.array([-1.25, -0.9, -0.55, -0.2, 0.2, 0.55, 0.9, 1.25])
    set_dacs(gatekeeper, targets)
    time.sleep(0.06)
    period_us, measured = gatekeeper.time_series_adc_read(
        CHANNELS, conversion_us=500, duration_us=60_000, timeout=10.0
    )
    assert period_us > 0
    assert measured.shape[0] >= 2
    assert np.all(np.isfinite(measured))
    means = np.mean(measured, axis=0)
    assert np.max(np.abs(means - targets)) < 0.08
    assert np.max(np.std(measured, axis=0)) < 0.01


def test_continuous_conversion_tracks_loopback(gatekeeper):
    target = 0.875
    gatekeeper.query("SET", 0, target)
    time.sleep(0.06)
    response = gatekeeper.query("CONTINUOUS_CONVERT_READ", 0, 5000, 40_000, timeout=4.0)
    samples = np.array([float(value) for value in response.split(",")])
    assert samples.shape == (8,)
    assert np.all(np.isfinite(samples))
    assert np.max(np.abs(samples - target)) < 0.08
