```
    ____  _           _____ __          __        __               
   / __ \(_)________ / ___// /_  ____ _/ /_____  / /   ____  ____ _
  / /_/ / / ___/ __ \\__ \/ __ \/ __ `/ //_/ _ \/ /   / __ \/ __ `/
 / ____/ / /__/ /_/ /__/ / / / / /_/ / ,< /  __/ /___/ /_/ / /_/ / 
/_/   /_/\___/\____/____/_/ /_/\__,_/_/|_|\___/_____/\____/\__, /  
                                                          /____/
```
[![DOI](https://zenodo.org/badge/1434099138.svg)](https://doi.org/10.5281/zenodo.23661399)

# PicoShakeLog

## Introduction

This project is released under the Academic Free License v3.0. See the `LICENSE` file for the full license text.

**Author:** Frédéric MARÇON

**Project purpose:**
PicoShakeLog is an open-source firmware for a high-resolution **6-axis motion and vibration logger** (3-axis accelerometer + 3-axis gyroscope) with anti-alias filtering (AAF), based on the TDK Invensense ICM-42688-P™ IMU. The project aims to measure tremors, vibrations and shocks for human healthcare research and transport applications across frequencies below 500 Hz. It is designed to be low-cost and easily reproducible.

**Typical applications:**

- **Human tremor and movement assessment (research):** wearable actimeter or tremor recorder for clinical and neuroscience studies (e.g. essential tremor, Parkinsonian tremor, physiological tremor), motor symptom monitoring, rehabilitation and ergonomics.
- **Pharmaceutical and healthcare logistics:** shock and vibration monitoring of shipments (cold chain, Good Distribution Practice context), qualification of transport packaging and routes.

**Keywords:** wearable actimeter, tremor monitoring, vibration logger, shock logger, inertial measurement unit (IMU), medical devices, pharmaceutical transport, cold chain, Raspberry Pi Pico 2 W, RP2350, ICM-42688-P.

**Hardware used:**

- The **TDK Invensense ICM-42688-P™** sensor is a 6-DOF device (accelerometer + gyroscope) with 20-bit (in fact 18 and 19 bits) resolution and an integrated anti-alias filter. It measures accelerations up to ±16 g and angular rates up to ±2000 °/s.
- The **Raspberry Pi Pico 2 W™** development board is based on the RP2350™ microcontroller. It is a dual-core device (Arm Cortex-M33 / RISC-V) running up to 133 MHz with 520 KiB of SRAM, 4 MiB of on-board flash, and a radio subsystem (Infineon CYW43439™) providing 2.4 GHz Wi‑Fi (802.11n) and Bluetooth 5.2.

**Disclaimer:**

This software and associated hardware are provided for research and prototyping purposes only. The authors have applied methodological rigor to ensure data accuracy and robustness but cannot guarantee reliability in all contexts. By using this project you accept the following conditions:

- **No medical advice or certification:** This project is not a certified medical device and does not replace professional medical diagnosis or equipment. Do not use this product for clinical decision-making or patient care without appropriate certification (e.g., CE marking) and supervision by qualified professionals.
- **No warranty:** Measurements, results and processing provided by this firmware/hardware are supplied "as-is." The authors and contributors make no warranties regarding accuracy, reliability or fitness for a particular purpose and are not liable for direct or indirect damages resulting from use of the project.
- **Compliance responsibility:** Users and integrators are responsible for ensuring regulatory and safety compliance for their intended use (standards, certifications, testing). Do not deploy this device in regulated contexts (medical, transport, safety) without appropriate qualification and validation.
- **Technical limits:** The integrated sensor measures accelerations up to ±16 g and is not suitable for shocks exceeding this range. Performance (bandwidth, noise, accuracy) depends on mechanical mounting, calibration, environment and software processing.
- **No use in critical systems:** Do not use this hardware/software as the sole component of a safety‑critical system or for decisions where failure could cause significant harm.

For applications requiring certification, validation or quality assurance, contact the authors or perform the necessary regulatory tests prior to deployment.

Note: you are free to modify the code; however, if you are not familiar with embedded firmware development or the project internals, exercise caution before changing low-level code or algorithmic processing.

### Flashing the firmware

1. Download the UF2 file (for example `PicoShakeLog-v0.2.0-rc.1.uf2`) from the project's GitHub Releases page: https://github.com/marconfr/PicoShakeLog/releases
2. Enter BOOTSEL mode: hold the BOOTSEL button while connecting the Pico to USB — a `RPI-RP2` drive will appear on your computer.
3. Copy the UF2 file onto the `RPI-RP2` drive; the board will reboot and run the new firmware.

Tip: you can also use `picotool` (provided with the Pico SDK) to load binaries from the command line.

### Building and flashing the firmware (VS Code)

Prerequisites: Visual Studio Code and the Raspberry Pi / Pico extension (C/C++ build tools, CMake). For full Pico development environment setup, see the official Raspberry Pi getting-started guide: https://www.raspberrypi.com/documentation/microcontrollers/pico/getting-started/

Quick procedure:

- Open the workspace in VS Code.
- Before opening the project, download the repository archive (green "Code" → "Download ZIP") or clone the repository:

```bash
git clone https://github.com/marconfr/PicoShakeLog.git
cd PicoShakeLog
```
- Install the Pico/C/C++ extension and ensure the Pico SDK is available (e.g. `~/.pico-sdk`).
- Run the `Compile Project` task (Terminal → Run Task → `Compile Project`); the binary and UF2 file will be generated in the `build/` folder (e.g. `build/PicoShakeLog.uf2`).
- To flash from VS Code, copy `build/PicoShakeLog.uf2` to the `RPI-RP2` drive (BOOTSEL) or use the provided tasks (`Run Project` / `Flash`) if configured.

### Wiring (ICM-42688-P ↔ Raspberry Pi Pico 2 W)

The firmware talks to the IMU over **SPI0 at 10 MHz**. Pin numbers are defined in [include/psl_config.h](include/psl_config.h). Wire the ICM-42688-P (or a breakout board exposing its SPI pins) as follows:

| ICM-42688-P signal | Pico 2 W GPIO | Pico 2 W physical pin | Role |
|---|---|---|---|
| `SDO` (AP_SDO) | GP16 (SPI0 RX) | 21 | MISO |
| `CS` (AP_CS) | GP17 | 22 | Chip select |
| `SCLK` (AP_SCLK) | GP18 (SPI0 SCK) | 24 | SPI clock |
| `SDI` (AP_SDI) | GP19 (SPI0 TX) | 25 | MOSI |
| `INT1` | GP13 | 17 | FIFO watermark interrupt (**required**) |
| `INT2` | GP14 | 19 | Reserved (defined, not used by the firmware) |
| `VDD` / `VDDIO` | 3V3(OUT) | 36 | 3.3 V supply |
| `GND` | GND | 23 (or 38) | Ground |

**Sensor documentation and boards:**

- TDK InvenSense ICM-42688-P product page: https://www.invensense.tdk.com/en-us/products/6-axis/icm-42688-p
- Datasheet (DS-000347): https://www.invensense.tdk.com/download-resource/ds-000347-icm-42688-p-datasheet
- Evaluation and breakout boards (availability and part numbers change, check before buying; the TDK evaluation board is usually listed as `EV_ICM-42688-P`)

Notes:

- Use **3.3 V only**; the ICM-42688-P is not 5 V tolerant.
- Keep SPI wires short (a few centimetres) at 10 MHz, and share a good ground. Use the 4-wire SPI mode, not I²C: if your breakout board has an I²C/SPI selector, set it to SPI.
- Mount the sensor rigidly on the part being measured: mechanical coupling directly affects the measured vibration.
- To use other pins, edit `SPI_*` in `psl_config.h`; the SPI pins must belong to the same SPI peripheral (`SPI_PORT`).

## Usage

The firmware outputs debug messages over USB serial. To view the serial output use a serial terminal client configured to **115200 baud, 8N1**. 

On macOS/Linux the port looks like `/dev/tty.*` or `/dev/ttyUSB*`; on Windows `COMx`. Search for a "serial terminal client" for your operating system, for instance `screen` or `minicom`

## How to cite

> Marçon, F. (2026). *marconfr/PicoShakeLog: PicoShakeLog v0.2.0-rc.1* [Computer software]. Zenodo. https://doi.org/10.5281/zenodo.23661400

```bibtex
@software{marcon_picoshakelog_2026,
  author    = {Marçon, Frédéric},
  title     = {{marconfr/PicoShakeLog: PicoShakeLog v0.2.0-rc.1}},
  year      = {2026},
  publisher = {Zenodo},
  doi       = {10.5281/zenodo.23661400},
  url       = {https://doi.org/10.5281/zenodo.23661400}
}
```

## Trademarks

The names, logos and trademarks mentioned in this project (for example TDK™, Raspberry Pi™, Pimoroni™, ICM-42688-P™, RP2350™, CYW43439™) are the property of their respective owners. They are mentioned here solely to identify compatible hardware, components or services and do not imply any affiliation, endorsement or sponsorship by the trademark owners. Any other use of these trademarks requires permission from the respective owners.

All cited trademarks remain the property of their respective holders.


