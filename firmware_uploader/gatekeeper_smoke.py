import math


EXPECTED_ENVIRONMENT = "GATEKEEPER"
NUM_ADC_BOARDS = 2
NUM_CHANNELS = 8


def _expect_equal(actual, expected, label):
    if actual != expected:
        raise RuntimeError(f"{label}: expected {expected!r}, got {actual!r}")


def _parse_float(value, label):
    try:
        parsed = float(value)
    except ValueError as exc:
        raise RuntimeError(f"{label}: invalid numeric response {value!r}") from exc
    if not math.isfinite(parsed):
        raise RuntimeError(f"{label}: non-finite response {value!r}")
    return parsed


def run_smoke_checks(send_command, expected_serial_number=None):
    results = {}

    results["nop"] = send_command("NOP")
    results["ready"] = send_command("*RDY?")
    results["environment"] = send_command("GET_ENVIRONMENT")
    results["serial_number"] = send_command("SERIAL_NUMBER")
    results["identity"] = send_command("*IDN?")
    results["firmware_version"] = send_command("GET_FIRMWARE_VERSION")

    _expect_equal(results["nop"], "NOP", "NOP")
    _expect_equal(results["ready"], "READY", "ready status")
    _expect_equal(results["environment"], EXPECTED_ENVIRONMENT, "environment")
    if expected_serial_number:
        _expect_equal(
            results["serial_number"], expected_serial_number, "serial number"
        )
    if not results["identity"].startswith("GateKeeper"):
        raise RuntimeError(f"Unexpected identity: {results['identity']!r}")
    if not results["firmware_version"]:
        raise RuntimeError("Firmware version response is empty")

    revisions = [int(send_command(f"GET_REVISION_REG,{board}")) for board in range(NUM_ADC_BOARDS)]
    if revisions != [34] * NUM_ADC_BOARDS:
        raise RuntimeError(f"Unexpected ADC revision registers: {revisions}")
    results["adc_revisions"] = revisions

    dac_readbacks = []
    adc_readbacks = []
    for channel in range(NUM_CHANNELS):
        dac = _parse_float(send_command(f"GET_DAC,{channel}"), f"DAC {channel}")
        adc = _parse_float(send_command(f"GET_ADC,{channel}"), f"ADC {channel}")
        if abs(dac) > 10.1:
            raise RuntimeError(f"DAC {channel} readback out of range: {dac}")
        if abs(adc) > 10.5:
            raise RuntimeError(f"ADC {channel} readback out of range: {adc}")
        dac_readbacks.append(dac)
        adc_readbacks.append(adc)

    results["dac_readbacks"] = dac_readbacks
    results["adc_readbacks"] = adc_readbacks
    return results
