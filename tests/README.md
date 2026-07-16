# Gatekeeper tests

`pytest` is the only test runner. With no device connected, hardware tests skip.

```sh
python -m pytest -m "not hardware"  # uploader/protocol checks
python -m pytest -m post_flash_health_checks  # read-only connected checks
python -m pytest                    # full DAC i -> ADC i loopback suite
```

Device detection is automatic; use `--port=COM8` to select a specific unit.

The hardware fixture restores DAC outputs, ADC conversion times, and chopping
mode, then verifies that saved and live ADC calibration are unchanged. The test
client refuses calibration writes, `HARD_RESET`, and `HARD_RESET_CALIBRATION`.

`loopback` tests change DAC outputs within ±1.5 V. Post-flash health checks are
read-only and require no DAC-to-ADC connections.
