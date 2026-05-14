#!/usr/bin/env python3
import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path


VARIANTS = {
    "GIGA": "disable_m7_usb_cdc",
    "GENERIC_STM32H747_M4": "disable_m4_usb_cdc",
}

SERIAL_CDC_DEFINE_RE = re.compile(r"(?m)^[ \t]*#define[ \t]+SERIAL_CDC\b.*\n?")
SERIAL_CDC_UNDEF_RE = re.compile(r"(?m)^[ \t]*#undef[ \t]+SERIAL_CDC\b")
M7_USB_CDC_BLOCK_RE = re.compile(
    r"(?m)^[ \t]*#if[ \t]+!defined\(GATEKEEPER_DISABLE_M7_USB_CDC\)\n"
    r"[ \t]*#define[ \t]+SERIAL_CDC\b.*\n"
    r"[ \t]*#endif\n?"
)
M4_USB_CDC_BLOCK_RE = re.compile(
    r"(?m)^[ \t]*#if[ \t]+defined\(GATEKEEPER_ENABLE_M4_USB_CDC\)\n"
    r"[ \t]*#define[ \t]+SERIAL_CDC\b.*\n"
    r"[ \t]*#else\n"
    r"[ \t]*#undef[ \t]+SERIAL_CDC\n"
    r"[ \t]*#endif\n?"
)


def _file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _tree_fingerprint(path):
    entries = []
    for child in sorted(path.rglob("*")):
        rel = child.relative_to(path).as_posix()
        if child.is_dir():
            entries.append([rel, "dir"])
            continue
        entries.append([rel, "file", _file_sha256(child)])
    encoded = json.dumps(entries, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _metadata(source_variant, transform_name):
    return {
        "source": str(source_variant.resolve()),
        "source_fingerprint": _tree_fingerprint(source_variant),
        "generator": _file_sha256(Path(__file__)),
        "transform": transform_name,
    }


def _metadata_matches(stamp_path, metadata):
    if not stamp_path.exists():
        return False
    try:
        return json.loads(stamp_path.read_text()) == metadata
    except (OSError, json.JSONDecodeError):
        return False


def _rewrite_pins(variant_dir, rewrite):
    pins_path = variant_dir / "pins_arduino.h"
    pins_path.write_text(rewrite(pins_path.read_text()))


def _disable_m7_usb_cdc(text):
    text = M7_USB_CDC_BLOCK_RE.sub("", text)
    return SERIAL_CDC_DEFINE_RE.sub("", text)


def _disable_m4_usb_cdc(text):
    text = M4_USB_CDC_BLOCK_RE.sub("#undef SERIAL_CDC\n", text)
    text = SERIAL_CDC_DEFINE_RE.sub("", text)
    if not SERIAL_CDC_UNDEF_RE.search(text):
        text = text.rstrip() + "\n\n#undef SERIAL_CDC\n"
    return text


TRANSFORMS = {
    "disable_m7_usb_cdc": _disable_m7_usb_cdc,
    "disable_m4_usb_cdc": _disable_m4_usb_cdc,
}


def generate_all(repo_root=None, framework_dir=None):
    repo_root = Path(repo_root or Path(__file__).resolve().parents[1])
    framework_dir = Path(
        framework_dir or Path.home() / ".platformio/packages/framework-arduino-mbed"
    )
    upstream_variants = framework_dir / "variants"
    variant_root = repo_root / "platformio_variants"
    output_root = variant_root / "generated"

    if not upstream_variants.exists():
        raise FileNotFoundError(f"Arduino-mbed variants not found: {upstream_variants}")

    output_root.mkdir(parents=True, exist_ok=True)
    updated = []

    for variant, transform_name in VARIANTS.items():
        source_variant = upstream_variants / variant
        output_variant = output_root / variant

        if not source_variant.exists():
            raise FileNotFoundError(f"Upstream variant not found: {source_variant}")

        metadata = _metadata(source_variant, transform_name)
        stamp_path = output_variant / ".gatekeeper_variant.json"
        if output_variant.exists() and _metadata_matches(stamp_path, metadata):
            continue

        if output_variant.exists():
            shutil.rmtree(output_variant)
        shutil.copytree(source_variant, output_variant)
        _rewrite_pins(output_variant, TRANSFORMS[transform_name])
        stamp_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
        updated.append(variant)

    return updated


def main():
    parser = argparse.ArgumentParser(
        description="Generate GateKeeper PlatformIO variants from Arduino-mbed."
    )
    parser.add_argument("--repo-root", type=Path, default=None)
    parser.add_argument("--framework-dir", type=Path, default=None)
    args = parser.parse_args()

    updated = generate_all(repo_root=args.repo_root, framework_dir=args.framework_dir)
    if updated:
        print("Generated PlatformIO variant(s): " + ", ".join(updated))


if __name__ == "__main__":
    main()
