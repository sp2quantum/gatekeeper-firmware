# PlatformIO Variants

The checked-in files here are the source of truth for GateKeeper variant changes.
The actual PlatformIO variant directories are generated under `generated/` from
the installed Arduino-mbed framework package.

Generation copies these upstream variants:

- `variants/GIGA`
- `variants/GENERIC_STM32H747_M4`

Then it applies the USB CDC transforms in
`platformio_tools/generate_platformio_variants.py`.

The generated directories are ignored by git and recreated automatically by the
PlatformIO pre-build script.
