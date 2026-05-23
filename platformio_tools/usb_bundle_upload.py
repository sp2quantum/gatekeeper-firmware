#!/usr/bin/env python3
import argparse
import importlib.util
import os
import shutil
import subprocess
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
M4_PROJECT_DIR = ROOT_DIR / "m4"
M7_PROJECT_DIR = ROOT_DIR / "m7"

DEFAULT_ROOT_BUILD_NAME = "gatekeeper_firmware"
DEFAULT_M4_ENV = "gatekeeper_m4_usb_gateway"
DEFAULT_M7_ENV = "gatekeeper_m7_worker"
EXPECTED_ENVIRONMENT = "GATEKEEPER"

def log(message):
    print(f"[usb-bundle-upload] {message}", flush=True)


class UploadEnv:
    def __init__(self, project_dir, pioenv, upload_port=None, backup_dir=None):
        self.project_dir = Path(project_dir)
        self.pioenv = pioenv
        self.upload_port = upload_port
        self.backup_dir = backup_dir

    def subst(self, value):
        if value == "$PROJECT_DIR":
            return str(self.project_dir)
        if value == "$PIOENV":
            return self.pioenv
        if value == "$UPLOAD_PORT":
            return self.upload_port or "$UPLOAD_PORT"
        if value == "$BUILD_DIR":
            return str(self.project_dir / ".pio" / "build" / self.pioenv)

        result = value
        replacements = {
            "$PROJECT_DIR": str(self.project_dir),
            "$PIOENV": self.pioenv,
            "$UPLOAD_PORT": self.upload_port or "$UPLOAD_PORT",
            "$BUILD_DIR": str(self.project_dir / ".pio" / "build" / self.pioenv),
            "${PROGNAME}": "firmware",
            "$PROGNAME": "firmware",
        }
        for key, replacement in replacements.items():
            result = result.replace(key, replacement)
        return result

    def get(self, key, default=None):
        if key == "CALIBRATION_BACKUP_DIR" and self.backup_dir is not None:
            return str(self.backup_dir)
        return default


def load_upload_persistence():
    helper_path = ROOT_DIR / "firmware_uploader" / "gatekeeper_upload.py"
    spec = importlib.util.spec_from_file_location(
        "gatekeeper_upload", helper_path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def find_executable(*names):
    for name in names:
        path = shutil.which(name)
        if path:
            return path
    scripts_dir = Path(sys.executable).resolve().parent
    extensions = [""]
    if os.name == "nt":
        extensions.extend(
            os.environ.get("PATHEXT", ".EXE;.BAT;.CMD").split(os.pathsep)
        )
    for name in names:
        name_path = Path(name)
        candidates = [scripts_dir / name_path]
        if not name_path.suffix:
            candidates.extend(
                scripts_dir / f"{name}{extension.lower()}"
                for extension in extensions
            )
            candidates.extend(
                scripts_dir / f"{name}{extension.upper()}"
                for extension in extensions
            )
        for candidate in candidates:
            if candidate.is_file():
                return str(candidate)
    return None


def run(command, cwd=None, env=None):
    log("$ " + " ".join(str(part) for part in command))
    subprocess.run(command, cwd=cwd, env=env, check=True)


def platformio_env(build_dir=None):
    if build_dir is None:
        return None
    child_env = os.environ.copy()
    child_env["PLATFORMIO_BUILD_DIR"] = str(build_dir)
    return child_env


def build_platformio_env(project_dir, pioenv, build_dir=None):
    pio = find_executable("pio", "platformio")
    if pio is None:
        raise RuntimeError("PlatformIO executable not found.")
    run([pio, "run", "-e", pioenv], cwd=project_dir, env=platformio_env(build_dir))


def clean_platformio_env(project_dir, pioenv, build_dir=None):
    pio = find_executable("pio", "platformio")
    if pio is None:
        raise RuntimeError("PlatformIO executable not found.")
    run(
        [pio, "run", "-e", pioenv, "-t", "clean"],
        cwd=project_dir,
        env=platformio_env(build_dir),
    )


def firmware_bin(project_dir, pioenv, build_dir=None):
    path = (build_dir or (project_dir / ".pio" / "build")) / pioenv / "firmware.bin"
    if not path.exists():
        raise RuntimeError(f"Built firmware binary not found: {path}")
    return path


def resolve_upload_selection(args):
    args.m4_env = args.m4_env or DEFAULT_M4_ENV
    args.m7_env = args.m7_env or DEFAULT_M7_ENV
    if (
        args.root_build_name is None
        and args.m4_env == DEFAULT_M4_ENV
        and args.m7_env == DEFAULT_M7_ENV
    ):
        args.root_build_name = DEFAULT_ROOT_BUILD_NAME
    return EXPECTED_ENVIRONMENT


def bundle_build_dirs(args):
    if args.root_build_name is None:
        return None, None

    root_build_dir = ROOT_DIR / ".pio" / "build" / args.root_build_name
    return (
        root_build_dir / "m4",
        root_build_dir / "m7",
    )


def build_bundle(args):
    m4_build_dir, real_m7_build_dir = bundle_build_dirs(args)
    if not args.skip_build:
        if not args.skip_m4_build:
            build_platformio_env(M4_PROJECT_DIR, args.m4_env, m4_build_dir)
        if not args.skip_real_m7_build:
            build_platformio_env(M7_PROJECT_DIR, args.m7_env, real_m7_build_dir)
        refresh_intellisense()

    m4_bin = firmware_bin(M4_PROJECT_DIR, args.m4_env, m4_build_dir)
    real_m7_bin = firmware_bin(M7_PROJECT_DIR, args.m7_env, real_m7_build_dir)
    return m4_bin, real_m7_bin


def clean_bundle(args):
    m4_build_dir, real_m7_build_dir = bundle_build_dirs(args)
    clean_platformio_env(M4_PROJECT_DIR, args.m4_env, m4_build_dir)
    clean_platformio_env(M7_PROJECT_DIR, args.m7_env, real_m7_build_dir)


def refresh_intellisense():
    script_path = ROOT_DIR / "platformio_tools" / "update_compile_commands.py"
    if not script_path.exists():
        return

    try:
        run(
            [
                sys.executable,
                str(script_path),
            ]
        )
    except subprocess.CalledProcessError as exc:
        log(f"IntelliSense refresh failed after build: {exc}")


def patch_binary_if_supported(persistence, binary_path, serial_number):
    if persistence.binary_contains_serial_marker(binary_path):
        persistence.patch_binary_serial(binary_path, serial_number)
        log(f"Patched {binary_path.name} with serial {serial_number}.")
        return True
    return False


def upload_serial_number(persistence, state):
    if state.get("skip"):
        serial_number = state.get("serial_number")
        if serial_number:
            return persistence.serial_with_current_year(serial_number)
        return persistence.default_serial_with_current_year()
    return state["serial_number"]


def verify_basic_firmware(persistence, upload_env, expected_serial_number):
    ready_port = persistence.wait_for_device_ready(
        upload_env, expected_serial_number=expected_serial_number
    )
    with persistence.open_command_port(ready_port) as ser:
        nop = persistence.send_command(ser, "NOP")
        ready = persistence.send_command(ser, "*RDY?")
        environment = persistence.send_command(ser, "GET_ENVIRONMENT")
        serial_number = persistence.send_command(ser, "SERIAL_NUMBER")

    if nop != "NOP":
        raise RuntimeError(f"NOP check failed after upload: {nop}")
    if ready != "READY":
        raise RuntimeError(f"READY check failed after upload: {ready}")
    if expected_serial_number and serial_number != expected_serial_number:
        raise RuntimeError(
            "Serial number changed across upload: "
            f"expected {expected_serial_number}, got {serial_number}"
        )

    log(
        "Basic firmware verification passed "
        f"on {ready_port} (env={environment}, serial={serial_number})."
    )
    return ready_port


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Upload a complete GateKeeper firmware bundle to Arduino GIGA "
            "over USB DFU."
        )
    )
    parser.add_argument(
        "--port",
        help="Serial port to trigger DFU from. If omitted, a connected GIGA is detected.",
    )
    parser.add_argument(
        "--root-build-name",
        default=None,
        help="Root .pio/build subdirectory name for bundled PlatformIO builds.",
    )
    parser.add_argument("--m4-env")
    parser.add_argument("--m7-env")
    parser.add_argument(
        "--skip-real-m7-build",
        action="store_true",
        help="Use the already-built real M7 binary. PlatformIO's usb_upload target uses this.",
    )
    parser.add_argument(
        "--skip-m4-build",
        action="store_true",
        help="Use the already-built M4 binary. PlatformIO's M4 upload target uses this.",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Use already-built M4 and M7 binaries.",
    )
    parser.add_argument(
        "--build-only",
        action="store_true",
        help="Build the complete firmware bundle without uploading.",
    )
    parser.add_argument(
        "--clean-only",
        action="store_true",
        help="Clean the complete firmware bundle without uploading.",
    )
    parser.add_argument(
        "--no-restore",
        action="store_true",
        help="Do not restore or verify calibration after upload.",
    )
    parser.add_argument(
        "--calibration-backup-dir",
        type=Path,
        help="Optional calibration backup directory.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    expected_environment = resolve_upload_selection(args)
    persistence = load_upload_persistence()
    upload_env = UploadEnv(
        M7_PROJECT_DIR,
        args.m7_env,
        upload_port=args.port,
        backup_dir=args.calibration_backup_dir,
    )

    if args.build_only:
        build_bundle(args)
        log(
            "Firmware bundle build complete "
            f"(m4={args.m4_env}, m7={args.m7_env})."
        )
        return

    if args.clean_only:
        clean_bundle(args)
        log(
            "Firmware bundle clean complete "
            f"(m4={args.m4_env}, m7={args.m7_env})."
        )
        return

    dfu_util = persistence.find_dfu_util()
    if dfu_util is None:
        raise RuntimeError("dfu-util not found.")

    port = args.port or persistence.find_giga_port(upload_env)
    if port is not None:
        log(f"Backing up current calibration/serial state from {port}...")
        state = persistence.backup_device_state(port)
        state["source_port"] = port
        if state.get("skip"):
            log(
                "Current firmware did not provide calibration/serial state; "
                f"continuing without persistence ({state.get('skip_reason')})."
            )
        else:
            backup_path = persistence.write_calibration_backup_file(upload_env, state)
            state["calibration_backup_path"] = str(backup_path)
            log(
                "Backed up device state "
                f"(env={state['source_environment']}, serial={state['serial_number']})."
            )
    elif persistence.dfu_device_present(dfu_util):
        log("Found Arduino GIGA already in USB DFU mode.")
        state = {
            "skip": True,
            "skip_reason": "already_in_dfu",
            "source_port": None,
        }
    else:
        log(
            "No Arduino GIGA serial port or DFU device found. "
            "Double-tap reset now to enter the bootloader."
        )
        if not persistence.wait_for_dfu_device(dfu_util, persistence.DFU_MANUAL_WAIT_S):
            raise RuntimeError("Arduino GIGA not found as serial or USB DFU.")
        state = {
            "skip": True,
            "skip_reason": "manual_dfu",
            "source_port": None,
        }

    if (
        expected_environment is not None
        and not state.get("skip")
        and state.get("source_environment") != expected_environment
    ):
        raise RuntimeError(
            "Connected board environment "
            f"{state['source_environment']} does not match requested upload "
            f"environment {expected_environment}."
        )

    m4_bin, real_m7_bin = build_bundle(args)

    serial_number = upload_serial_number(persistence, state)
    state["serial_number"] = serial_number
    patch_binary_if_supported(persistence, m4_bin, serial_number)
    patch_binary_if_supported(persistence, real_m7_bin, serial_number)

    if port is not None:
        persistence.trigger_dfu_mode(
            dfu_util,
            port,
            log_func=log,
            manual_prompt="Double-tap reset now if this board is running older firmware.",
        )
    elif not persistence.dfu_device_present(dfu_util):
        raise RuntimeError("Arduino GIGA USB DFU device disappeared before upload.")
    else:
        log("Using existing Arduino GIGA USB DFU device.")

    log("Uploading M4 without leaving DFU...")
    persistence.dfu_download(
        dfu_util, persistence.M4_ADDRESS, m4_bin, log_func=log
    )
    log("Uploading M7 and leaving DFU...")
    persistence.dfu_download(
        dfu_util, persistence.M7_LEAVE_ADDRESS, real_m7_bin, log_func=log
    )

    if args.no_restore:
        log("USB bundle upload complete.")
        return

    if state.get("skip"):
        verify_basic_firmware(persistence, upload_env, serial_number)
        log("USB bundle upload and basic verification complete.")
        return

    ready_port = persistence.wait_for_device_ready(
        upload_env, expected_serial_number=state["serial_number"]
    )
    log(f"Restoring calibration on {ready_port}...")
    persistence.restore_calibration(ready_port, state)
    persistence.verify_calibration(ready_port, state)
    log("USB bundle upload, calibration restore, and verification complete.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001
        print(f"[usb-bundle-upload] ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
