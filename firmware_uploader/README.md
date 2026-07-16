# GateKeeper Firmware Installation Guide

This directory has an easy-to-use uploader for compiled GateKeeper firmware releases. If there's no `firmware` directory present, download it from the [latest firmware release](https://github.com/sp2quantum/gatekeeper-firmware/releases).

Run the task-specific scripts described below. `util.py` is their shared internal
helper and is not an upload command.

## 1. Make sure you have `dfu-util` installed

- **Linux (Debian)**: `sudo apt install dfu-util`
- **MacOS**: `brew install dfu-util`
- **Windows**:
  - Download the `dfu-util` to your local system, e.g., under `D:\dfu-util`.
  - Rename it to `dfu-util.exe`.
  - Append the path of the `dfu-util.exe` to the system environment variable PATH.

## 2. Run `pip install -r requirements.txt`

## 3. Plug in Arduino Giga

## 4. Run `python3 upload_firmware.py`

The uploader attempts to auto-detect the GateKeeper port; you can pass
`--port <port>` if you would like to select one explicitly.

The uploader preserves the device's serial number and calibration data. If it
cannot read either one, it asks for confirmation before replacing that data.

## 5. Feel free to email [markzakharyan@sp2quantum.com](mailto:markzakharyan@sp2quantum.com) if something isn't working
