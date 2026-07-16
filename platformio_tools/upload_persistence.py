"""PlatformIO-only state persistence around GateKeeper firmware uploads."""

import importlib.util
import json
import signal
import threading
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
UTIL_PATH = ROOT_DIR / "firmware_uploader" / "util.py"


def _load_firmware_uploader_util():
    spec = importlib.util.spec_from_file_location("firmware_uploader_util", UTIL_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(
            f"Could not load firmware uploader utilities from {UTIL_PATH}"
        )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# The development bundle uploader historically consumes one combined helper module.
# Re-export release-safe utilities here while keeping PlatformIO-specific code out of
# the directory shipped to end users.
_util = _load_firmware_uploader_util()
for _name in dir(_util):
    if not _name.startswith("_"):
        globals()[_name] = getattr(_util, _name)


CALIBRATION_DOWNLOAD_TIMEOUT_S = 10
CALIBRATION_UPLOAD_TIMEOUT_S = 10
CALIBRATION_VERIFY_TIMEOUT_S = 10
DEFAULT_CALIBRATION_BACKUP_DIR_NAME = "calibration_backups"
M4_ENVIRONMENTS = {
    "gatekeeper_m4_usb_gateway",
    "gatekeeper_m4_usb_gateway_stlink",
}


def log(message):
    print(f"[upload-persistence] {message}")


class CalibrationOperationTimeout(RuntimeError):
    pass


@contextmanager
def operation_timeout(seconds, operation_name):
    if (
        threading.current_thread() is not threading.main_thread()
        or not hasattr(signal, "SIGALRM")
        or not hasattr(signal, "setitimer")
    ):
        yield
        return

    def handle_timeout(_signum, _frame):
        raise CalibrationOperationTimeout(
            f"Timed out {operation_name} after {seconds} seconds."
        )

    previous_handler = signal.getsignal(signal.SIGALRM)
    signal.signal(signal.SIGALRM, handle_timeout)
    previous_timer = signal.setitimer(signal.ITIMER_REAL, seconds)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous_handler)
        if previous_timer[0] > 0:
            signal.setitimer(signal.ITIMER_REAL, previous_timer[0], previous_timer[1])


def get_state_path(env):
    return Path(env.subst("$BUILD_DIR")) / "upload_state.json"


def get_binary_path(env):
    return Path(env.subst("$BUILD_DIR")) / f"{env.subst('${PROGNAME}')}.bin"


def get_project_root(env):
    return Path(env.subst("$PROJECT_DIR")).parent


def get_calibration_backup_dir(env):
    configured = env.get("CALIBRATION_BACKUP_DIR")
    if configured:
        return Path(env.subst(configured)).expanduser()
    return get_project_root(env) / DEFAULT_CALIBRATION_BACKUP_DIR_NAME


def get_calibration_backup_path(env, state):
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    serial_number = state["serial_number"].replace("/", "_")
    backup_dir = get_calibration_backup_dir(env)
    backup_path = backup_dir / f"calibration_{serial_number}_{timestamp}.json"
    if not backup_path.exists():
        return backup_path

    index = 2
    while True:
        candidate = backup_dir / f"calibration_{serial_number}_{timestamp}_{index}.json"
        if not candidate.exists():
            return candidate
        index += 1


def is_m4_project(env):
    return env.subst("$PIOENV") in M4_ENVIRONMENTS


def find_giga_port(env=None, expected_serial_number=None):
    ports = list(serial.tools.list_ports.comports())
    if env is not None:
        upload_port = env.subst("$UPLOAD_PORT")
        if upload_port and upload_port != "$UPLOAD_PORT":
            existing = next(
                (port for port in ports if port.device == upload_port), None
            )
            if existing:
                if is_giga_port(existing) and port_matches_serial(
                    existing, expected_serial_number
                ):
                    log(f"Using configured upload port: {upload_port}")
                    return upload_port
                log(
                    f"Configured upload port {upload_port} is present but not a "
                    "recognized GIGA device; scanning actual ports"
                )
            else:
                log(
                    f"Configured upload port {upload_port} not present; "
                    "scanning actual ports"
                )

    for port in ports:
        if is_giga_port(port) and port_matches_serial(port, expected_serial_number):
            log(f"Found GIGA port {port.device} ({port.hwid})")
            return port.device

    log(
        "No GIGA serial port found. "
        f"Available ports: {[port.device for port in ports]}"
    )
    return None


def wait_for_device_ready(env=None, port=None, expected_serial_number=None):
    last_error = None
    if port is not None and expected_serial_number is None:
        log(f"Waiting for same port to become ready: {port}")
        for _ in range(READY_RETRY_COUNT):
            try:
                with open_command_port(port) as command_port:
                    wait_for_ready(command_port)
                return port
            except Exception as exc:  # noqa: BLE001
                last_error = exc
                time.sleep(READY_RETRY_DELAY_S)
        raise RuntimeError(f"Device never became ready after upload: {last_error}")

    for _ in range(READY_RETRY_COUNT):
        current_port = find_giga_port(env, expected_serial_number)
        if current_port is None and port is not None:
            current_port = port
        if current_port is None:
            time.sleep(READY_RETRY_DELAY_S)
            continue

        try:
            with open_command_port(current_port) as command_port:
                wait_for_ready(command_port)
            return current_port
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            time.sleep(READY_RETRY_DELAY_S)

    if last_error is not None:
        raise RuntimeError(f"Device never became ready after upload: {last_error}")
    raise RuntimeError("Arduino GIGA not found after upload.")


def comparable_calibration_state(state):
    comparable = dict(state)
    comparable.pop("saved_at", None)
    return comparable


def remove_older_duplicate_calibration_files(backup_path, state):
    backup_dir = backup_path.parent
    if not backup_dir.exists():
        return []

    comparable_state = comparable_calibration_state(state)
    removed_paths = []
    for candidate in backup_dir.glob("*.json"):
        if candidate == backup_path:
            continue
        try:
            candidate_state = json.loads(candidate.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        try:
            validate_calibration_state(candidate_state)
        except RuntimeError:
            continue
        if comparable_calibration_state(candidate_state) != comparable_state:
            continue
        candidate.unlink()
        removed_paths.append(candidate)
    return removed_paths


def write_calibration_backup_file(env, state):
    backup_path = get_calibration_backup_path(env, state)
    write_calibration_state_file(backup_path, state)

    removed_paths = remove_older_duplicate_calibration_files(backup_path, state)
    if removed_paths:
        log(
            "Removed older duplicate calibration backup(s): "
            + ", ".join(str(path) for path in removed_paths)
        )
    log(f"Calibration backup saved to {backup_path}.")
    return backup_path


def save_state(env, state):
    state_path = get_state_path(env)
    state_path.parent.mkdir(parents=True, exist_ok=True)
    state_path.write_text(json.dumps(state, indent=2))


def load_state(env):
    state_path = get_state_path(env)
    if not state_path.exists():
        raise RuntimeError(f"Upload state file not found: {state_path}")
    return json.loads(state_path.read_text())


def delete_state(env):
    state_path = get_state_path(env)
    if state_path.exists():
        state_path.unlink()


def run_pre_upload(env):
    port = find_giga_port(env)
    if port is None:
        raise RuntimeError("Arduino GIGA not found before upload.")

    try:
        with operation_timeout(
            CALIBRATION_DOWNLOAD_TIMEOUT_S, "downloading calibration data"
        ):
            state = backup_device_state(port)
    except CalibrationOperationTimeout as exc:
        log(f"{exc} Proceeding with firmware upload without calibration backup.")
        save_state(
            env,
            {
                "skip": True,
                "skip_reason": "calibration_download_timeout",
                "source_port": port,
            },
        )
        return
    state["source_port"] = port
    if state.get("skip"):
        log(f"Skipping calibration/serial persistence: {state.get('skip_reason')}")
        save_state(env, state)
        return

    backup_path = write_calibration_backup_file(env, state)
    state["calibration_backup_path"] = str(backup_path)
    log(
        "Backed up device state "
        f"(env={state['source_environment']}, serial={state['serial_number']})."
    )

    if is_m4_project(env) and state["source_environment"] != SUPPORTED_ENVIRONMENT:
        raise RuntimeError(
            f"Connected board environment {state['source_environment']} does not match "
            f"selected PlatformIO environment {SUPPORTED_ENVIRONMENT}."
        )

    binary_path = get_binary_path(env)
    if binary_contains_serial_marker(binary_path):
        patch_binary_serial(binary_path, state["serial_number"])
        log(f"Patched {binary_path.name} with serial {state['serial_number']}.")

    save_state(env, state)


def run_post_upload(env):
    state = load_state(env)
    if state.get("skip"):
        log(
            "Skipping post-upload calibration/serial verification: "
            f"{state.get('skip_reason')}"
        )
        delete_state(env)
        return

    port = wait_for_device_ready(
        env,
        port=state.get("source_port"),
        expected_serial_number=state.get("serial_number"),
    )
    log(f"Restoring calibration on {port}...")
    try:
        with operation_timeout(
            CALIBRATION_UPLOAD_TIMEOUT_S, "uploading calibration data"
        ):
            restore_calibration(port, state)
        with operation_timeout(
            CALIBRATION_VERIFY_TIMEOUT_S, "verifying calibration data"
        ):
            verify_calibration(port, state)
    except Exception:
        backup_path = state.get("calibration_backup_path")
        if backup_path:
            log(f"Calibration backup remains saved at {backup_path}.")
        raise
    delete_state(env)
    log("Calibration and serial verification passed after upload.")
