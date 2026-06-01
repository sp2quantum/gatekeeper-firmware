# GateKeeper Firmware

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Firmware for sp2 Quantum GateKeeper. New features include precise timings (<300ns error) for SPI comms, new buffer ramp options, and data acquisition being in-tandem with transmission to LabRAD.

The firmware is designed to be easily extensible, as new peripherals can simply be added with little consideration for how other peripherals are written, as operations are all added independently to a centralized registry.

**If you have any questions**, feel free to email [markzakharyan@sp2quantum.com](mailto:markzakharyan@sp2quantum.com)

<!--
## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [License](#license)
-->

## Installation

1. Unzip Firmware_Package.zip (from the releases tab)

2. Make sure you have `dfu-util` installed

  - **Linux (Debian)**: `sudo apt install dfu-util`
  - **MacOS**: `brew install dfu-util`
  - **Windows**:
    - Download [dfu-util](https://dfu-util.sourceforge.net) to your local system, e.g., under `D:\dfu-util`.
    - Rename it to `dfu-util.exe`.
    - Append the path of the `dfu-util.exe` to the system environment variable PATH.

3. Run `pip install -r requirements.txt`

4. Plug in Arduino Giga

5. Run `python3 upload_firmware.py`

You can also build/upload from source:

This firmware has one root PlatformIO entrypoint plus separate M4 and M7 core projects. The root default environment is `gatekeeper_firmware`, so normal development uploads do not need an explicit `-e` selection. From the repo root, run:

```sh
pio run -t upload
```

This uploads the complete firmware bundle over USB DFU: USB gateway M4, then worker M7 firmware. To build the same bundle without uploading, use:

```sh
pio run
```

The `m4/` and `m7/` folders remain normal PlatformIO projects for core-specific development and debugging.

## Usage

***Firmware docs are still in progress***

Note for vim users: If you have any issues with linting/LSP with (neo)vim then try running `python3 platformio_tools/update_compile_commands.py` from the firmware root.

**New features of this firmware include:**

- **Function Registry / user IO handling completely separate from all peripheral logic**
  - This means you can add/modify peripheral logic without regard for how the firmware processes commands. Simply write your logic and register commands with the Function Registry
- **Precise timings**
  - Framework to trigger events at a specified frequency, with error <300ns. We use this to communicate with the DAC/ADC at very precise intervals in buffer ramps
- **Dual Core**
  - Utilizes both M4 and M7 cores of the Arduino Giga, which allows parallel data collection and transmission to LabRAD, which saves a *substantial* amount of time during long buffer ramps (~25% faster).
- **New native buffer ramp options**
  - DAC-led ramp now allows for precise control over DAC settling time
  - Time series buffer ramp allows for spectral analysis of data after collection with LabRAD
  - Native 2D buffer ramp along any arbitrary axis in DAC voltage phase space
  - Boxcar buffer ramp

### Known Issues

- There are currently no known issues! Please let me know if you find a bug.

### Extending Firmware

Add a command by writing a normal `OperationResult` function and placing
`COMMAND` right below it. Argument parsing and argument count checking are
inferred from the function signature.

```cpp
#include "FunctionRegistry/FunctionRegistryHelpers.h"

OperationResult setThing(int channel, float value) {
  return OperationResult::Success();
}
COMMAND("SET_THING", setThing)
```

For variable-length arguments, use `List<T, N>` where `N` is the earlier scalar
argument that gives the list length. `List<float, 0, 2>` reads `arg0 * arg2`
values.

```cpp
using FunctionRegistryParsing::List;

OperationResult exampleRamp(int numDacs, List<int, 0>& channels,
                            List<float, 0>& voltages) {
  return OperationResult::Success();
}
COMMAND("EXAMPLE_RAMP", exampleRamp)
```

Use `ON_SETUP(func)` for work that happens once at boot, before user commands are
handled: pin modes, hardware objects, default runtime state, etc. Use
`ON_SETUP_PLATFORM` for lower-level platform setup and `ON_SETUP_CALIBRATION` for
applying saved calibration after hardware setup. For example:

```cpp
void setup() {
  DACLimits::initializeLimits();
  initChannelDefaults();
  pinMode(ldac, OUTPUT);
  FastGpio::digitalWrite(ldac, true);
}
ON_SETUP(setup)
```

Use `ON_INITIALIZE(func)` for work that should happen only when the user sends
`INITIALIZE`/`INIT`, such as taking a peripheral out of a safe startup state. Below is an example from `DACController.cpp`. `ON_SETUP` configures the pins and
`INITIALIZE` takes the AD5791 DACs out of tri-state and sets them to zero.

```cpp
void initialize() {
  for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
    byte buf[3] = {kWriteControlRegisterCommand, 0, kUnclampDacFromGround};
    comms[i].transferDAC(buf, 3);
    setVoltage(i, 0.0);
  }
}
ON_INITIALIZE(initialize)
```

If a peripheral has saved calibration, define its private calibration struct in
that controller and register it with `CALIBRATION_SECTION`. The root calibration
service handles flash persistence and default reset; `main.cpp` does not need to
know the peripheral exists. For example:

```cpp
struct DacCalibrationData {
  float gain[NUM_DAC_CALIBRATION_CHANNELS];
  float offset[NUM_DAC_CALIBRATION_CHANNELS];
};

void setDacCalibrationDefaults(void* section) { /* ... */ }
bool validateDacCalibration(const void* section) { return true; }

CALIBRATION_SECTION("DAC", DacCalibrationData, setDacCalibrationDefaults,
                    validateDacCalibration)
```

### Precise Timings (Hardware Timer)

Precise timings were achieved by using separate hardware timers for the DAC and ADC and configuring the Arduino Giga to trigger an interrupt service routine (ISR) when a timer's register exceeds a certain value. All timing-related things are handled in `TimingUtil.h`, which mostly contains register configurations to setup the hardware timers properly. TIM1 is used for the DAC and TIM8 is used for the ADC.

After setting up a timer using TimingUtil (for example, by calling `setupTimersTimeSeries(dac_period_us, adc_period_us)`), `TimingUtil::dacFlag` and `TimingUtil::adcFlag` are set to true constantly after a given period. What's expected is a loop to constantly check if one of these flags is `true`, execute something, and set that flag to false.

**Important:** None of the timers in TimingUtil automatically stop, and the function that calls a TimingUtil function must recognize when the timers have triggered enough times and call `TimingUtil.disableDacInterrupt()` and/or `TimingUtil.disableAdcInterrupt()`

The two modes we have for precise timings are Time Series, and DAC-Led. Time Series simply triggers both the `dacFlag` and `adcFlag` at their respective intervals, starting at the same time. DAC-Led triggers both the `dacFlag` and `adcFlag` at the same frequency (`dac_interval_us`), with a phase difference between them (`dac_settling_time_us`)

### SPI Comms

#### Simultaneity

When two events occur at the same time, the following behavior occurs:

- DAC SPI is sent before ADC SPI.
- ADC channels on separate ADC boards can convert simultaneously.
- ADC channels on the same ADC board are serviced sequentially in channel order.
- Timing checks should be based on the slowest active ADC board: sum the conversion times for channels selected on each board, then use the maximum board total plus firmware/SPI overhead.

### Dual Core Comms

This firmware utilizes both the M4 and M7 cores of the Arduino Giga R1. By default, the M7 core is booted and the M4 is initialized *on the M7*.

The purpose of using a dual-core approach is to separate USB host communication from peripheral timing work. The M4 owns USB CDC and forwards host commands through shared memory. The M7 boots the M4, owns the DAC/ADC hardware, and executes the peripheral workload. This keeps USB/serial handling away from timing-critical SPI execution.

If you Google the docs for how to implement dual-core communications between the M4 and M7 cores, it tells you to use RPC. **We do not do this!** RPC is slow for our purposes and completely screws up our precise timings for the same reason why Serial messes them up. Instead, we manually initialize the M4 core ourselves (instead of with RPC) and use a circular shared memory buffer that both the M4 and M7 have access to. We initialize the M4 core ourselves (instead of with `RPC.begin()`).

We have comms pipelines for char arrays (cstrings), float arrays, and `VoltagePacket` arrays (defined below). By "comm pipeline", I mean a one-direction transfer of data of a particular type. For instance, we have *separate* circular buffers for floats going from M4 to M7 and floats going from M7 to M4. Each circular buffer works like this:

![Multiprocessing diagram](docs/images/data_transfer.png)
This is an example for how ADC voltages are collected and transmitted to LabRAD mid ramp. There's another identical buffer for data going the other direction --from the M7 to the M4.

A `VoltagePacket` is simply a float that is transmitted to LabRAD over serial as four bytes, most significant bit first. This is significantly faster than printing to serial the voltage float converted to a char array, which is why we use this in buffer ramps. Technically, this is one extra byte than in the old firmware, which sent raw DAC data and had LabRAD calculate the voltage. Since we have async data transfer now, we don't need to worry about this extra byte --the added simplicity is worth it.

### Buffer Ramps

#### Minimum Timing Tables

These limits were measured on a real GateKeeper with all ADC conversion times set
to the hardware minimum request of 82us (82.03125us actual), with
`dac_settling_time_us = 20` for DAC-led ramps. Calibration commands were not
run. Rows are the number of selected ADC channels on ADC card 0 and columns are
the number of selected ADC channels on ADC card 1, so `1/1` and `2/0` are
intentionally different cases. The firmware hardcodes these limits in
`BufferRampCommon`.

DAC count did not change the DAC-led minimum in spot checks for N = 1, 2, 4, and
8 DAC channels. These values apply to `DAC_LED_BUFFER_RAMP` and
`2D_DAC_LED_BUFFER_RAMP` with normal, retrace, or snake scanning.

| ADC card 0 \ ADC card 1 | 0 | 1 | 2 | 3 | 4 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0 | - | 120 | 200 | 300 | 400 |
| 1 | 120 | 120 | 220 | 320 | 420 |
| 2 | 200 | 220 | 240 | 320 | 420 |
| 3 | 300 | 320 | 320 | 340 | 440 |
| 4 | 400 | 420 | 420 | 440 | 460 |

`TIME_SERIES_BUFFER_RAMP` minimum `adc_interval_us`:

| ADC card 0 \ ADC card 1 | 0 | 1 | 2 | 3 | 4 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0 | - | 80 | 80 | 100 | 160 |
| 1 | 80 | 80 | 80 | 80 | 160 |
| 2 | 80 | 80 | 80 | 180 | 140 |
| 3 | 100 | 120 | 120 | 200 | 240 |
| 4 | 160 | 160 | 300 | 240 | 160 |

`2D_TIME_SERIES_BUFFER_RAMP` minimum `adc_interval_us`, normal mode:

| ADC card 0 \ ADC card 1 | 0 | 1 | 2 | 3 | 4 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0 | - | 80 | 80 | 80 | 80 |
| 1 | 80 | 80 | 80 | 80 | 280 |
| 2 | 80 | 80 | 80 | 120 | 160 |
| 3 | 80 | 80 | 80 | 120 | 160 |
| 4 | 80 | 280 | 160 | 160 | 160 |

`2D_TIME_SERIES_BUFFER_RAMP` minimum `adc_interval_us`, retrace mode:

| ADC card 0 \ ADC card 1 | 0 | 1 | 2 | 3 | 4 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0 | - | 80 | 80 | 80 | 80 |
| 1 | 80 | 80 | 80 | 80 | 280 |
| 2 | 80 | 80 | 80 | 80 | 160 |
| 3 | 80 | 80 | 80 | 120 | 160 |
| 4 | 80 | 280 | 160 | 160 | 160 |

`2D_TIME_SERIES_BUFFER_RAMP` minimum `adc_interval_us`, snake mode:

| ADC card 0 \ ADC card 1 | 0 | 1 | 2 | 3 | 4 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0 | - | 80 | 80 | 80 | 80 |
| 1 | 80 | 80 | 80 | 80 | 260 |
| 2 | 80 | 80 | 80 | 80 | 160 |
| 3 | 80 | 120 | 80 | 120 | 220 |
| 4 | 80 | 280 | 160 | 160 | 270 |

`AWG_WITH_ADC` uses the DAC-led table as its base `dac_interval_us`, with a
DAC-count overhead for heavier DAC updates:

| DAC channels | Additional interval |
| ---: | ---: |
| 1-3 | 0 |
| 4-7 | +20us when the base interval is 200-280us |
| 8 | +40us |

`BOXCAR_BUFFER_RAMP` enforces a conservative `adc_conversion_time_us` floor
based on the busiest ADC card. The ramp data were sensible at lower values, but
the timing watchdog showed ADC timing pressure near the lower and phase-sensitive
settings; the firmware therefore checks DAC/ADC SPI missteps and ignores the
persistent ADC-conversion watchdog bit for boxcar cleanup.

| Max selected ADC channels on one card | Minimum `adc_conversion_time_us` |
| ---: | ---: |
| 1 | 300 |
| 2 | 300 |
| 3 | 500 |
| 4 | 800 |

## License

This project is licensed under the [MIT License](LICENSE).
