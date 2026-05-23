#!/usr/bin/env python3
import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
M4_PROJECT_DIR = ROOT_DIR / "m4"
M7_PROJECT_DIR = ROOT_DIR / "m7"
M4_ENV = "gatekeeper_m4_usb_gateway"
M7_ENV = "gatekeeper_m7_worker"
INCLUDES_RESPONSE_RE = re.compile(
    r"(?P<iprefix>(?:^|\s)-iprefix(?:\s+)?(?P<prefix>\"[^\"]+\"|\S+))"
    r"\s+@(?P<response>\"[^\"]+\"|\S*includes\.txt)"
)


def log(message):
    print(f"[intellisense] {message}", flush=True)


def find_platformio():
    for name in ("pio", "platformio"):
        path = shutil.which(name)
        if path:
            return path
    scripts_dir = Path(sys.executable).resolve().parent
    extensions = [""]
    if sys.platform == "win32":
        extensions.extend(
            os.environ.get("PATHEXT", ".EXE;.BAT;.CMD").split(os.pathsep)
        )
    for name in ("pio", "platformio"):
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
    raise RuntimeError("PlatformIO executable not found.")


def run_compiledb(platformio, project_dir, env_name):
    log(f"Refreshing {project_dir.name} compile database ({env_name})...")
    subprocess.run(
        [platformio, "run", "-e", env_name, "-t", "compiledb"],
        cwd=project_dir,
        check=True,
    )


def _resolve_path(path, directory):
    path = Path(path)
    if path.is_absolute():
        return path
    return (directory / path).resolve()


def _include_path_flag(path):
    return "-I" + str(path)


def _unquote(value):
    if len(value) >= 2 and value[0] == value[-1] == '"':
        return value[1:-1]
    return value


def _expand_response_file(path, directory, iprefix):
    response_path = _resolve_path(path, directory)
    expanded = []
    for line in response_path.read_text().splitlines():
        token = line.strip()
        if not token:
            continue
        if token.startswith("-iwithprefixbefore"):
            suffix = token.removeprefix("-iwithprefixbefore")
            if "\\" in str(iprefix):
                suffix = suffix.replace("/", "\\")
            token = _include_path_flag(str(iprefix).rstrip("/\\") + suffix)
        expanded.append(token)
    return expanded


def expand_compile_command(command, directory):
    def replace(match):
        iprefix = match.group("iprefix")
        prefix = _unquote(match.group("prefix"))
        response = _unquote(match.group("response"))
        includes = _expand_response_file(response, directory, prefix)
        return iprefix + " " + " ".join(includes)

    return INCLUDES_RESPONSE_RE.sub(replace, command)


def normalized_entries(compile_db_path):
    data = json.loads(compile_db_path.read_text())
    entries = []
    for entry in data:
        normalized = dict(entry)
        directory = Path(normalized["directory"])
        file_path = Path(normalized["file"])
        if not file_path.is_absolute():
            normalized["file"] = str((directory / file_path).resolve())
        if "output" in normalized:
            output_path = Path(normalized["output"])
            if not output_path.is_absolute():
                normalized["output"] = str((directory / output_path).resolve())
        if "command" in normalized:
            normalized["command"] = expand_compile_command(
                normalized["command"], directory
            )
        entries.append(normalized)
    return entries


def merge_compile_databases():
    compile_databases = [
        M4_PROJECT_DIR / "compile_commands.json",
        M7_PROJECT_DIR / "compile_commands.json",
    ]
    missing = [path for path in compile_databases if not path.exists()]
    if missing:
        missing_list = ", ".join(str(path) for path in missing)
        raise RuntimeError(f"Missing compile database: {missing_list}")

    entries = []
    for path in compile_databases:
        entries.extend(normalized_entries(path))

    output_path = ROOT_DIR / "compile_commands.json"
    output_path.write_text(json.dumps(entries, indent=2) + "\n")
    log(
        "Wrote combined compile database "
        f"{output_path} ({len(entries)} entries)."
    )


def write_vscode_cpp_config():
    vscode_dir = ROOT_DIR / ".vscode"
    vscode_dir.mkdir(exist_ok=True)
    config_path = vscode_dir / "c_cpp_properties.json"
    config = {
        "configurations": [
            {
                "name": "GateKeeper Firmware",
                "compileCommands": "${workspaceFolder}/compile_commands.json",
                "compilerPath": (
                    "${userHome}/.platformio/packages/"
                    "toolchain-gccarmnoneeabi/bin/arm-none-eabi-g++"
                ),
                "intelliSenseMode": "gcc-arm",
                "cStandard": "gnu11",
                "cppStandard": "gnu++14",
            }
        ],
        "version": 4,
    }
    config_path.write_text(json.dumps(config, indent=4) + "\n")
    log(f"Wrote VSCode C++ config {config_path}.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Build a root compile_commands.json for M4/M7 IntelliSense."
    )
    parser.add_argument(
        "--no-compiledb",
        action="store_true",
        help="Only merge existing M4/M7 compile databases.",
    )
    parser.add_argument(
        "--no-vscode-config",
        action="store_true",
        help="Do not write .vscode/c_cpp_properties.json.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.no_compiledb:
        platformio = find_platformio()
        run_compiledb(platformio, M4_PROJECT_DIR, M4_ENV)
        run_compiledb(platformio, M7_PROJECT_DIR, M7_ENV)

    merge_compile_databases()
    if not args.no_vscode_config:
        write_vscode_cpp_config()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001
        print(f"[intellisense] ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
