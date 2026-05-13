#!/usr/bin/env python3
import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path


VARIANTS = {
    "GIGA": ["GIGA.pins_arduino.patch"],
    "GENERIC_STM32H747_M4": ["GENERIC_STM32H747_M4.pins_arduino.patch"],
}


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


def _metadata(source_variant, patch_paths):
    return {
        "source": str(source_variant.resolve()),
        "source_fingerprint": _tree_fingerprint(source_variant),
        "patches": {
            patch.name: _file_sha256(patch)
            for patch in patch_paths
        },
    }


def _metadata_matches(stamp_path, metadata):
    if not stamp_path.exists():
        return False
    try:
        return json.loads(stamp_path.read_text()) == metadata
    except (OSError, json.JSONDecodeError):
        return False


def _apply_patch(variant_dir, patch_path):
    subprocess.run(
        ["patch", "-p1", "--batch", "--silent", "-i", str(patch_path)],
        cwd=variant_dir,
        check=True,
    )


def generate_all(repo_root=None, framework_dir=None):
    repo_root = Path(repo_root or Path(__file__).resolve().parents[1])
    framework_dir = Path(
        framework_dir or Path.home() / ".platformio/packages/framework-arduino-mbed"
    )
    upstream_variants = framework_dir / "variants"
    variant_root = repo_root / "platformio_variants"
    output_root = variant_root / "generated"
    patch_root = variant_root / "patches"

    if not upstream_variants.exists():
        raise FileNotFoundError(f"Arduino-mbed variants not found: {upstream_variants}")

    output_root.mkdir(parents=True, exist_ok=True)
    updated = []

    for variant, patch_names in VARIANTS.items():
        source_variant = upstream_variants / variant
        output_variant = output_root / variant
        patch_paths = [patch_root / name for name in patch_names]

        if not source_variant.exists():
            raise FileNotFoundError(f"Upstream variant not found: {source_variant}")
        for patch_path in patch_paths:
            if not patch_path.exists():
                raise FileNotFoundError(f"Variant patch not found: {patch_path}")

        metadata = _metadata(source_variant, patch_paths)
        stamp_path = output_variant / ".gatekeeper_variant.json"
        if output_variant.exists() and _metadata_matches(stamp_path, metadata):
            continue

        if output_variant.exists():
            shutil.rmtree(output_variant)
        shutil.copytree(source_variant, output_variant)
        for patch_path in patch_paths:
            _apply_patch(output_variant, patch_path)
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
