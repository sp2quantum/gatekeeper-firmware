import json
import math
import re
import shutil
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import serial
import serial.tools.list_ports


SERIAL_BAUD = 115200
SERIAL_TIMEOUT_S = 2
ARDUINO_GIGA_DFU_DEVICE_ID = "2341:0366"
M7_ADDRESS = "0x08040000"
M4_ADDRESS = "0x08100000"
M7_FLASH_SIZE = 0x000C0000
M4_FLASH_SIZE = 0x00100000
M7_READ_ADDRESS = f"{M7_ADDRESS}:0x{M7_FLASH_SIZE:08X}"
M4_READ_ADDRESS = f"{M4_ADDRESS}:0x{M4_FLASH_SIZE:08X}"
M7_LEAVE_ADDRESS = f"{M7_ADDRESS}:leave"
M4_LEAVE_ADDRESS = f"{M4_ADDRESS}:leave"
DFU_AUTO_WAIT_S = 8
DFU_MANUAL_WAIT_S = 60
READY_RETRY_COUNT = 60
READY_RETRY_DELAY_S = 0.5
CALIBRATION_WRITE_DELAY_S = 0.05
FLOAT_VERIFY_ABS_TOL = 1e-6
CHANNELS_PER_DAC_BOARD = 4
CHANNELS_PER_ADC_BOARD = 4
DAC_BOARD_COUNT = 2
ADC_BOARD_COUNT = 2
DAC_CHANNEL_COUNT = DAC_BOARD_COUNT * CHANNELS_PER_DAC_BOARD
ADC_CHANNEL_COUNT = ADC_BOARD_COUNT * CHANNELS_PER_ADC_BOARD
SUPPORTED_ENVIRONMENT = "GATEKEEPER"
SERIAL_PATTERN = re.compile(r"DA_\d{4}_[A-Za-z0-9_]{3}$")
SERIAL_SUFFIX_PATTERN = re.compile(r"[A-Za-z0-9_]{1,3}$")
NO_CALIBRATION_UPLOAD_SERIAL_PATTERN = re.compile(r"DA_\d{4}____$")
SERIAL_MARKER = b"__SERIAL_NUMBER__"
SERIAL_FIELD_LENGTH = 12
DEFAULT_SERIAL_SUFFIX = "ABC"
DEFAULT_CALIBRATION_PATH = Path("calibration_data.json")
POST_FLASH_PORT_RETRY_COUNT = 30


def log(message):
    print(f"[firmware-uploader] {message}")


def find_executable(*names):
    for name in names:
        path = shutil.which(name)
        if path:
            return path
    return None


def find_dfu_util():
    path = find_executable("dfu-util")
    if path:
        return path

    candidates = [
        Path.home() / ".platformio" / "packages" / "tool-dfuutil" / "bin" / "dfu-util",
        Path.home() / ".platformio" / "packages" / "tool-dfuutil-arduino" / "dfu-util",
        Path.home()
        / ".platformio"
        / "packages"
        / "tool-stm32duino"
        / "dfu-util"
        / "dfu-util",
    ]
    for candidate in candidates:
        platform_candidates = [candidate]
        if candidate.suffix.lower() != ".exe":
            platform_candidates.append(candidate.with_suffix(".exe"))
        for platform_candidate in platform_candidates:
            if platform_candidate.is_file():
                return str(platform_candidate)
    return None


def dfu_list_output(dfu_util):
    result = subprocess.run(
        [dfu_util, "-l", "-d", ARDUINO_GIGA_DFU_DEVICE_ID],
        check=False,
        capture_output=True,
        text=True,
    )
    return f"{result.stdout}\n{result.stderr}"


def dfu_device_present(dfu_util):
    return "Found DFU" in dfu_list_output(dfu_util)


def wait_for_dfu_device(dfu_util, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if dfu_device_present(dfu_util):
            return True
        time.sleep(0.5)
    return False


def trigger_dfu_mode(
    dfu_util,
    port,
    log_func=log,
    manual_prompt="Double-tap reset now to enter the bootloader.",
):
    log_func(f"Triggering DFU mode from {port}...")
    try:
        with serial.Serial(port, 1200, timeout=1):
            pass
    except (serial.SerialException, OSError) as exc:
        log_func(f"1200-baud touch failed: {exc}")

    if wait_for_dfu_device(dfu_util, DFU_AUTO_WAIT_S):
        return

    log_func(f"DFU did not appear after the 1200-baud touch. {manual_prompt}")
    if not wait_for_dfu_device(dfu_util, DFU_MANUAL_WAIT_S):
        raise RuntimeError("Timed out waiting for USB DFU mode.")


def dfu_download(dfu_util, address, firmware_path, log_func=log):
    command = [
        dfu_util,
        "-d",
        ARDUINO_GIGA_DFU_DEVICE_ID,
        "-a",
        "0",
        "-s",
        address,
        "-D",
        str(firmware_path),
    ]
    log_func("$ " + " ".join(command))
    subprocess.run(command, check=True)


def dfu_upload(dfu_util, address, output_path, log_func=log):
    command = [
        dfu_util,
        "-d",
        ARDUINO_GIGA_DFU_DEVICE_ID,
        "-a",
        "0",
        "-s",
        address,
        "-U",
        str(output_path),
    ]
    log_func("$ " + " ".join(command))
    subprocess.run(command, check=True)


def current_serial_prefix():
    return f"DA_{datetime.now().year}"


def serial_with_current_year(serial_number):
    if not SERIAL_PATTERN.fullmatch(serial_number or ""):
        raise RuntimeError(f"Invalid serial number '{serial_number}'.")
    suffix = serial_number.rsplit("_", 1)[1]
    return f"{current_serial_prefix()}_{suffix}"


def default_serial_with_current_year():
    return f"{current_serial_prefix()}_{DEFAULT_SERIAL_SUFFIX}"


def serial_from_suffix(suffix):
    if not SERIAL_SUFFIX_PATTERN.fullmatch(suffix or ""):
        raise RuntimeError(
            "Serial suffix must contain 1-3 ASCII letters, digits, or underscores."
        )
    return f"{current_serial_prefix()}_{suffix.zfill(3)}"


KNOWN_GIGA_VIDS_PIDS = {
    (0x2341, 0x0266),
}


def parse_vid_pid_from_hwid(hwid):
    if not hwid:
        return None, None
    match = re.search(r"VID(?:[:_=]|_)?([0-9A-Fa-f]{4}).*?PID(?:[:_=]|_)?([0-9A-Fa-f]{4})", hwid)
    if not match:
        return None, None
    return int(match.group(1), 16), int(match.group(2), 16)


def is_giga_port(port):
    vid, pid = port.vid, port.pid
    if vid is None or pid is None:
        vid, pid = parse_vid_pid_from_hwid(port.hwid)
    return (vid, pid) in KNOWN_GIGA_VIDS_PIDS


def port_matches_serial(port, expected_serial_number):
    if not expected_serial_number:
        return True
    return (
        port.serial_number == expected_serial_number
        or expected_serial_number in (port.device or "")
        or expected_serial_number in (port.hwid or "")
    )


def list_giga_ports():
    return [port for port in serial.tools.list_ports.comports() if is_giga_port(port)]


def format_port(port):
    parts = [port.device]
    if port.description:
        parts.append(port.description)
    if port.hwid:
        parts.append(port.hwid)
    return " | ".join(parts)


def choose_giga_port(port=None, prompt="Select Arduino GIGA port"):
    if port:
        return port

    ports = list_giga_ports()
    if not ports:
        available = [p.device for p in serial.tools.list_ports.comports()]
        print(f"No Arduino GIGA serial ports found. Available ports: {available}")
        return None

    print("Detected Arduino GIGA serial ports:")
    for index, candidate in enumerate(ports, start=1):
        print(f"  {index}. {format_port(candidate)}")

    if len(ports) == 1:
        selected = ports[0].device
        print(f"Using only detected Arduino GIGA port: {selected}")
        return selected

    while True:
        selection = input(f"{prompt} [1-{len(ports)}]: ").strip()
        try:
            selected_index = int(selection)
        except ValueError:
            print("Please enter a number from the list.")
            continue
        if 1 <= selected_index <= len(ports):
            return ports[selected_index - 1].device
        print("Selection out of range.")


def open_command_port(port, timeout=SERIAL_TIMEOUT_S):
    return serial.Serial(port, SERIAL_BAUD, timeout=timeout, write_timeout=timeout)


def read_line(ser, timeout=None):
    old_timeout = ser.timeout
    if timeout is not None:
        ser.timeout = timeout
    try:
        response = ser.readline()
    finally:
        if timeout is not None:
            ser.timeout = old_timeout

    if not response:
        raise RuntimeError("Timed out waiting for a serial response.")
    return response.decode(errors="replace").strip()


def send_command(ser, command, timeout=None):
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(f"{command}\n".encode("ascii"))
    ser.flush()
    response = read_line(ser, timeout=timeout)
    if response.startswith("FAILURE"):
        raise RuntimeError(f"Command '{command}' failed: {response}")
    return response


def wait_for_ready(ser):
    for _ in range(READY_RETRY_COUNT):
        try:
            if send_command(ser, "*RDY?") == "READY":
                return
        except RuntimeError:
            pass
        time.sleep(READY_RETRY_DELAY_S)
    raise RuntimeError("Device never reported READY.")


def wait_for_giga_port(expected_serial_number=None):
    for _ in range(POST_FLASH_PORT_RETRY_COUNT):
        ports = list_giga_ports()
        if expected_serial_number:
            for candidate in ports:
                if port_matches_serial(candidate, expected_serial_number):
                    return candidate.device
        elif ports:
            return ports[0].device
        time.sleep(READY_RETRY_DELAY_S)
    if expected_serial_number:
        raise RuntimeError(f"Timed out waiting for serial port {expected_serial_number}.")
    raise RuntimeError("Timed out waiting for Arduino GIGA serial port.")


def get_dac_channel_count_for_environment(environment):
    if environment != SUPPORTED_ENVIRONMENT:
        raise RuntimeError(f"Unsupported device environment '{environment}'.")
    return DAC_CHANNEL_COUNT


def read_dac_float_stream(ser, active_dac_channel_count):
    min_count = active_dac_channel_count * 2
    max_count = DAC_CHANNEL_COUNT * 2
    values = []
    deadline = time.monotonic() + SERIAL_TIMEOUT_S

    while len(values) < min_count:
        line = read_line(ser, timeout=SERIAL_TIMEOUT_S)
        for token in line.split():
            try:
                values.append(float(token))
            except ValueError as exc:
                raise RuntimeError(f"Could not parse float response '{line}'.") from exc
        deadline = time.monotonic() + 0.2

    while len(values) < max_count and time.monotonic() < deadline:
        try:
            line = read_line(ser, timeout=0.05)
        except RuntimeError:
            continue
        for token in line.split():
            try:
                values.append(float(token))
            except ValueError as exc:
                raise RuntimeError(f"Could not parse float response '{line}'.") from exc
        deadline = time.monotonic() + 0.2

    if len(values) % 2 != 0:
        raise RuntimeError(f"Expected offset/gain pairs, got {len(values)} values.")
    return values


def read_dac_calibration(ser, dac_channel_count):
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(b"INQUIRY_OSG\n")
    ser.flush()
    values = read_dac_float_stream(ser, dac_channel_count)
    response_channel_count = len(values) // 2
    if response_channel_count < dac_channel_count:
        raise RuntimeError(
            f"Expected at least {dac_channel_count} DAC calibration channels, "
            f"got {response_channel_count}."
        )
    offsets = values[:response_channel_count]
    gains = values[response_channel_count:]
    return offsets[:dac_channel_count], gains[:dac_channel_count]


def read_adc_calibration_with_commands(ser, zero_cmd, full_cmd):
    adc_zero_scale = []
    adc_full_scale = []
    for channel in range(ADC_CHANNEL_COUNT):
        adc_zero_scale.append(int(send_command(ser, f"{zero_cmd},{channel}")))
    for channel in range(ADC_CHANNEL_COUNT):
        adc_full_scale.append(int(send_command(ser, f"{full_cmd},{channel}")))
    return adc_zero_scale, adc_full_scale


def read_adc_calibration(ser):
    try:
        return read_adc_calibration_with_commands(
            ser, "GET_SAVED_ZERO_SCALE_CAL", "GET_SAVED_FULL_SCALE_CAL"
        )
    except RuntimeError:
        return read_adc_calibration_with_commands(
            ser, "GET_ZERO_SCALE_CAL", "GET_FULL_SCALE_CAL"
        )


def make_calibration_state(
    source_environment,
    serial_number,
    firmware_version,
    device_id,
    dac_offsets,
    dac_gains,
    adc_zero_scale,
    adc_full_scale,
):
    return {
        "schema_version": 1,
        "saved_at": datetime.now(timezone.utc).isoformat(),
        "source_environment": source_environment,
        "serial_number": serial_number,
        "firmware_version": firmware_version,
        "device_id": device_id,
        "dac_channel_count": len(dac_offsets),
        "adc_channel_count": ADC_CHANNEL_COUNT,
        "dac_offsets": dac_offsets,
        "dac_gains": dac_gains,
        "adc_zero_scale": adc_zero_scale,
        "adc_full_scale": adc_full_scale,
    }


def validate_calibration_state(state):
    if not isinstance(state, dict):
        raise RuntimeError("Calibration state must be a JSON object.")
    required_keys = {
        "schema_version",
        "saved_at",
        "source_environment",
        "serial_number",
        "firmware_version",
        "device_id",
        "dac_channel_count",
        "adc_channel_count",
        "dac_offsets",
        "dac_gains",
        "adc_zero_scale",
        "adc_full_scale",
    }
    missing = sorted(required_keys - set(state))
    if missing:
        raise RuntimeError(f"Calibration state is missing keys: {', '.join(missing)}")
    if state["schema_version"] != 1:
        raise RuntimeError(
            f"Unsupported calibration schema_version: {state['schema_version']}"
        )
    if (
        isinstance(state["adc_channel_count"], bool)
        or not isinstance(state["adc_channel_count"], int)
        or state["adc_channel_count"] != ADC_CHANNEL_COUNT
    ):
        raise RuntimeError(
            f"Unsupported ADC channel count: {state['adc_channel_count']}"
        )
    dac_channel_count = state["dac_channel_count"]
    if (
        isinstance(dac_channel_count, bool)
        or not isinstance(dac_channel_count, int)
        or not 1 <= dac_channel_count <= DAC_CHANNEL_COUNT
    ):
        raise RuntimeError(f"Unsupported DAC channel count: {dac_channel_count}")
    if (
        not isinstance(state["source_environment"], str)
        or state["source_environment"] != SUPPORTED_ENVIRONMENT
    ):
        raise RuntimeError(
            f"Unsupported source environment: {state['source_environment']}"
        )
    if not isinstance(state["serial_number"], str) or not SERIAL_PATTERN.fullmatch(
        state["serial_number"]
    ):
        raise RuntimeError(f"Invalid calibration serial: {state['serial_number']}")
    if not isinstance(state["dac_offsets"], list) or not isinstance(
        state["dac_gains"], list
    ):
        raise RuntimeError("DAC calibration values must be lists.")
    if len(state["dac_offsets"]) != dac_channel_count:
        raise RuntimeError("DAC offset count does not match dac_channel_count.")
    if len(state["dac_gains"]) != dac_channel_count:
        raise RuntimeError("DAC gain count does not match dac_channel_count.")
    for label, values in (
        ("DAC offset", state["dac_offsets"]),
        ("DAC gain", state["dac_gains"]),
    ):
        for channel, value in enumerate(values):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise RuntimeError(f"{label} {channel} is not numeric.")
            if not math.isfinite(value):
                raise RuntimeError(f"{label} {channel} is not finite.")
    for channel, gain in enumerate(state["dac_gains"]):
        if abs(gain) < 1e-6:
            raise RuntimeError(f"DAC gain {channel} is too close to zero.")
    if not isinstance(state["adc_zero_scale"], list) or not isinstance(
        state["adc_full_scale"], list
    ):
        raise RuntimeError("ADC calibration values must be lists.")
    if len(state["adc_zero_scale"]) != ADC_CHANNEL_COUNT:
        raise RuntimeError("ADC zero-scale count does not match adc_channel_count.")
    if len(state["adc_full_scale"]) != ADC_CHANNEL_COUNT:
        raise RuntimeError("ADC full-scale count does not match adc_channel_count.")
    for label, values, minimum in (
        ("ADC zero scale", state["adc_zero_scale"], 0),
        ("ADC full scale", state["adc_full_scale"], 1),
    ):
        for channel, value in enumerate(values):
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or not minimum <= value <= 0xFFFFFF
            ):
                raise RuntimeError(
                    f"{label} {channel} must be an integer from "
                    f"{minimum} to 16777215."
                )


def get_available_path(path):
    path = Path(path)
    if not path.exists():
        return path

    parent = path.parent
    stem = path.stem
    suffix = path.suffix
    index = 2
    while True:
        candidate = parent / f"{stem}{index}{suffix}"
        if not candidate.exists():
            return candidate
        index += 1


def load_calibration_state(path):
    state = json.loads(Path(path).read_text())
    validate_calibration_state(state)
    return state


def write_calibration_state_file(path, state):
    validate_calibration_state(state)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(state, indent=2) + "\n")
    saved_state = json.loads(path.read_text())
    if saved_state != state:
        raise RuntimeError(f"Calibration backup file verification failed: {path}")
    return path


def write_adc_calibration_with_commands(ser, zero_cmd, full_cmd, state):
    for channel, value in enumerate(state["adc_zero_scale"]):
        send_command(ser, f"{zero_cmd},{channel},{value}")
        time.sleep(CALIBRATION_WRITE_DELAY_S)

    for channel, value in enumerate(state["adc_full_scale"]):
        send_command(ser, f"{full_cmd},{channel},{value}")
        time.sleep(CALIBRATION_WRITE_DELAY_S)


def restore_calibration(port, state):
    validate_calibration_state(state)
    with open_command_port(port) as ser:
        wait_for_ready(ser)

        for channel, (offset, gain) in enumerate(
            zip(state["dac_offsets"], state["dac_gains"])
        ):
            send_command(ser, f"SET_OSG,{channel},{offset:.8f},{gain:.8f}")
            time.sleep(CALIBRATION_WRITE_DELAY_S)

        # These commands update both the live ADC registers and the persisted
        # calibration section. Writing only SET_SAVED_* leaves stale live
        # calibration in place until the next device reboot.
        write_adc_calibration_with_commands(
            ser,
            "SET_ZERO_SCALE_CAL",
            "SET_FULL_SCALE_CAL",
            state,
        )


def verify_calibration(port, state):
    validate_calibration_state(state)
    with open_command_port(port) as ser:
        wait_for_ready(ser)
        actual_offsets, actual_gains = read_dac_calibration(
            ser, len(state["dac_offsets"])
        )
        actual_zero_scale, actual_full_scale = read_adc_calibration(ser)
        live_zero_scale, live_full_scale = read_adc_calibration_with_commands(
            ser, "GET_ZERO_SCALE_CAL", "GET_FULL_SCALE_CAL"
        )
        actual_environment = send_command(ser, "GET_ENVIRONMENT")
        actual_serial = send_command(ser, "SERIAL_NUMBER")

    if actual_environment != state["source_environment"]:
        raise RuntimeError(
            "Environment changed across upload: "
            f"expected {state['source_environment']}, got {actual_environment}"
        )
    if actual_serial != state["serial_number"]:
        raise RuntimeError(
            "Serial number changed across upload: "
            f"expected {state['serial_number']}, got {actual_serial}"
        )

    for channel, (expected, actual) in enumerate(
        zip(state["dac_offsets"], actual_offsets)
    ):
        if abs(expected - actual) > FLOAT_VERIFY_ABS_TOL:
            raise RuntimeError(
                f"DAC {channel} offset mismatch: expected {expected:.8f}, got {actual:.8f}"
            )

    for channel, (expected, actual) in enumerate(
        zip(state["dac_gains"], actual_gains)
    ):
        if abs(expected - actual) > FLOAT_VERIFY_ABS_TOL:
            raise RuntimeError(
                f"DAC {channel} gain mismatch: expected {expected:.8f}, got {actual:.8f}"
            )

    for channel, (expected, actual) in enumerate(
        zip(state["adc_zero_scale"], actual_zero_scale)
    ):
        if expected != actual:
            raise RuntimeError(
                f"ADC {channel} zero scale mismatch: expected {expected}, got {actual}"
            )

    for channel, (expected, actual) in enumerate(
        zip(state["adc_full_scale"], actual_full_scale)
    ):
        if expected != actual:
            raise RuntimeError(
                f"ADC {channel} full scale mismatch: expected {expected}, got {actual}"
            )
    for label, expected_values, actual_values in (
        ("live zero scale", state["adc_zero_scale"], live_zero_scale),
        ("live full scale", state["adc_full_scale"], live_full_scale),
    ):
        for channel, (expected, actual) in enumerate(
            zip(expected_values, actual_values)
        ):
            if expected != actual:
                raise RuntimeError(
                    f"ADC {channel} {label} mismatch: "
                    f"expected {expected}, got {actual}"
                )


def backup_device_state(port):
    with open_command_port(port) as ser:
        if firmware_hangs_on_noop(ser):
            return {
                "skip": True,
                "skip_reason": "firmware_hangs_on_nop",
            }

        wait_for_ready(ser)
        source_environment = send_command(ser, "GET_ENVIRONMENT")
        if source_environment != SUPPORTED_ENVIRONMENT:
            return {
                "skip": True,
                "skip_reason": f"unsupported_environment_{source_environment}",
            }
        serial_number = send_command(ser, "SERIAL_NUMBER")
        firmware_version = send_command(ser, "GET_FIRMWARE_VERSION")
        device_id = send_command(ser, "*IDN?")
        if is_no_calibration_upload_serial(serial_number):
            return {
                "skip": True,
                "skip_reason": "no_calibration_upload_serial",
            }

        if not SERIAL_PATTERN.fullmatch(serial_number or ""):
            raise RuntimeError(
                "Could not determine a valid serial number from the current firmware."
            )
        serial_number = serial_with_current_year(serial_number)

        dac_offsets, dac_gains = read_dac_calibration(
            ser, get_dac_channel_count_for_environment(source_environment)
        )
        adc_zero_scale, adc_full_scale = read_adc_calibration(ser)

    return make_calibration_state(
        source_environment,
        serial_number,
        firmware_version,
        device_id,
        dac_offsets,
        dac_gains,
        adc_zero_scale,
        adc_full_scale,
    )


def patch_binary_serial(binary_path, serial_number):
    if not SERIAL_PATTERN.fullmatch(serial_number or ""):
        raise RuntimeError(f"Invalid serial number '{serial_number}'.")

    binary_path = Path(binary_path)
    data = binary_path.read_bytes()
    marker_count = data.count(SERIAL_MARKER)
    if marker_count != 1:
        raise RuntimeError(
            f"Expected exactly one serial marker in {binary_path}, "
            f"found {marker_count}."
        )
    index = data.find(SERIAL_MARKER)

    field_start = index
    field_end = field_start + len(SERIAL_MARKER) + SERIAL_FIELD_LENGTH
    if field_end > len(data):
        raise RuntimeError(f"Serial field is truncated in {binary_path}.")
    replacement = SERIAL_MARKER + serial_number.encode("ascii")
    replacement = replacement.ljust(field_end - field_start, b"\x00")
    data = data[:field_start] + replacement + data[field_end:]
    binary_path.write_bytes(data)


def read_binary_serial(binary_path):
    data = Path(binary_path).read_bytes()
    if data.count(SERIAL_MARKER) != 1:
        return None
    index = data.find(SERIAL_MARKER)
    serial_start = index + len(SERIAL_MARKER)
    serial_end = serial_start + SERIAL_FIELD_LENGTH
    if serial_end > len(data):
        return None
    try:
        serial_number = data[serial_start:serial_end].rstrip(b"\x00").decode("ascii")
    except UnicodeDecodeError:
        return None
    return serial_number if SERIAL_PATTERN.fullmatch(serial_number) else None


def binary_contains_serial_marker(binary_path):
    return SERIAL_MARKER in Path(binary_path).read_bytes()


def is_no_calibration_upload_serial(serial_number):
    return bool(NO_CALIBRATION_UPLOAD_SERIAL_PATTERN.fullmatch(serial_number or ""))


def firmware_hangs_on_noop(ser):
    try:
        return send_command(ser, "NOP", timeout=1) != "NOP"
    except RuntimeError:
        return True


    log("Calibration and serial verification passed after upload.")
