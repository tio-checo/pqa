# PQA — Three-phase power quality analyzer

Hardware designs, ESP32-S3 firmware, and MATLAB tools for a three-phase power
quality analyzer. The firmware acquires three-phase voltage and current signals
through a Microchip MCP3913 six-channel ADC. PQA calculates electrical measurements, serves
them over a local HTTP API, and uploads measurements to ThingSpeak over HTTPS.

## Features

- Six ADC channels: three currents (`I1`–`I3`) and three voltages (`U1`–`U3`).
- Nominal sampling rate of 2,048 samples/s per channel, with 4,096 samples per
  acquisition block (approximately two seconds).
- DC offset removal, RMS voltage/current, zero-crossing frequency estimation,
  phase shifts, and per-phase active and reactive power.
- Phase-based power factor calculated as `cos(voltage–current phase shift)`.
- 2,048-point FFT processing with diagnostic output on the serial console.
- JSON measurement and CSV raw-sample endpoints over Wi-Fi.
- SNTP time synchronization and timestamp storage in NVS.
- ThingSpeak uploads of voltage, current, frequency, and total active power.

Power calculations use RMS values and the estimated phase shift. The firmware
does not calculate true power factor from instantaneous power under distorted
waveforms, and does not currently expose FFT results through the HTTP API.

## Hardware

The supplied `fw/pqa/sdkconfig` targets an **ESP32-S3 with 8 MB flash and octal PSRAM**.
Acquisition buffers, FFT buffers, and the CSV response buffer use external RAM.
Adjust the configuration to match your module before flashing.

The ADC configuration assumes a **4.194304 MHz crystal**, oversampling ratio 512,
and gain 1 on all channels. GPIO assignments are defined in `fw/pqa/main/mcp3913.c` and
`fw/pqa/main/pqa.c`:

| Signal | ESP32-S3 GPIO |
| --- | --- |
| MCP3913 MISO | 12 |
| MCP3913 MOSI | 13 |
| MCP3913 SPI clock | 11 |
| MCP3913 chip select | 10 |
| MCP3913 data-ready interrupt | 7 |
| ADC debug output | 17 |
| Heartbeat LED (active high) | 6 |

SPI uses `SPI2_HOST` at 10 MHz. ADC channels 0–2 map to `I1`, `I2`, and `I3`;
channels 3–5 map to `U1`, `U2`, and `U3`.

## Configuration

Before building, review these settings:

| Setting | Location |
| --- | --- |
| Wi-Fi SSID and password | `EXAMPLE_ESP_WIFI_SSID` and `EXAMPLE_ESP_WIFI_PASS` in `fw/pqa/main/pqa.c` |
| ThingSpeak write API key | `THINGSPEAK_API_KEY` in `fw/pqa/main/pqa.c` |
| ADC board calibration selection | `MCP3913_BOARD_SN` in `fw/pqa/main/include/board.h` (currently `2`) |
| ADC offset and gain calibration | `adc_offcal` and `adc_gaincal` in `fw/pqa/main/mcp3913.c` |
| Voltage/current scaling and current-sensor phase correction | `fw/pqa/main/daq.c` |
| SNTP servers | `initialize_sntp()` in `fw/pqa/main/time_sync.c` |
| HTTPS trust certificate | `fw/pqa/main/server_root_cert.pem`, embedded by `fw/pqa/main/CMakeLists.txt` |
| Target, flash, PSRAM, and console settings | `fw/pqa/sdkconfig` / `idf.py menuconfig` |

Wi-Fi credentials and the ThingSpeak key are currently source constants; replace
them with your own values. They are not configured through `menuconfig`.
Calibration values and scaling factors are specific to the measurement hardware.
The current conversion assumes 1,000 mV / 20 A sensors.

## Build and flash

Use **ESP-IDF 5.5.1**, the version recorded in the supplied `fw/pqa/sdkconfig`.
The component manifest currently declares an older minimum IDF version.

1. Install ESP-IDF 5.5.1 and its ESP32-S3 tools using the ESP-IDF installation
   instructions for your operating system.
2. Activate the ESP-IDF environment. For a shell-based installation:

   ```bash
   . /path/to/esp-idf/export.sh
   ```

3. From the repository root, enter the firmware directory, add the cJSON
   dependency required with ESP-IDF 5.5.1, then configure and build:

   ```bash
   cd fw/pqa
   idf.py add-dependency "espressif/cjson^1.7.19"
   idf.py menuconfig
   idf.py build
   ```

   The supplied configuration already selects `esp32s3`. If generating a new
   configuration, use `idf.py set-target esp32s3`, then configure flash and PSRAM
   to match the hardware.

4. Connect the board and flash it, replacing `PORT` with its serial device:

   ```bash
   idf.py -p PORT flash monitor
   ```

   The configured UART console runs at 115,200 baud. Exit the monitor with
   **Ctrl+]**.

The ESP-IDF Component Manager resolves dependencies from
`fw/pqa/main/idf_component.yml`. The repository currently declares ESP-DSP with a
wildcard version and does not include a dependency lockfile. Keep the generated
lockfile when reproducing a working build.

If ESP-DSP 1.7.0 reports missing `std::sin` or `std::cos` declarations, it needs
`<cmath>` included when compiling its C++ sources. A local build workaround is
to append this to `fw/pqa/CMakeLists.txt` after `project(pqa)`:

```cmake
idf_component_get_property(esp_dsp_lib espressif__esp-dsp COMPONENT_LIB)
target_compile_options(${esp_dsp_lib} PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-include>"
    "$<$<COMPILE_LANGUAGE:CXX>:cmath>")
```

## Run and retrieve measurements

At boot, the firmware connects to Wi-Fi and synchronizes time before starting
acquisition and the HTTP server. The current implementation always attempts SNTP
synchronization; failure aborts startup rather than restoring the saved NVS time.
Allow access to the configured SNTP servers (`time.windows.com` and `pool.ntp.org`).

Find the device address in the serial log (`got ip: ...`). Once acquisition is
running, query the device from the same network, replacing `DEVICE_IP`:

```bash
curl "http://DEVICE_IP/api/v1/values"
curl "http://DEVICE_IP/api/v1/samples.csv" -o samples.csv
```

The HTTP server uses port 80. The current API has no authentication or TLS.
Static web-page serving is disabled, so `/` does not provide a dashboard.

### Measurement JSON

`GET /api/v1/values` returns an object with these fields:

| Field | Meaning | Unit |
| --- | --- | --- |
| `ts` | Time at completion of measurement processing, in UTC | ISO 8601 (`YYYY-MM-DDTHH:MM:SSZ`) |
| `f` | Frequency estimated from `U3` | Hz |
| `u1`, `u2`, `u3` | Per-phase RMS voltage | V |
| `i1`, `i2`, `i3` | Per-phase RMS current | A |
| `fi1`, `fi2`, `fi3` | Corrected voltage–current phase shift for each phase | degrees |
| `p` | Sum of the three calculated active powers | W |

### Raw sample CSV

`GET /api/v1/samples.csv` returns up to 4,096 rows with columns
`i1, i2, i3, u1, u2, u3`. Values are signed ADC codes, before software DC offset
removal and conversion to volts or amperes. Each row contains one sample from
each channel. The handler copies from a mutex-protected acquisition snapshot.

### ThingSpeak

Uploads are enabled in `app_main()`. Fields 1–3 contain RMS voltages, fields 4–6
contain RMS currents, field 7 contains the `U3` frequency, and field 8 contains
total calculated active power. Acquisition requests an upload initially and
then every eight acquisition blocks, nominally about 16 seconds. The initial
request is signaled before the first block's calculations finish.

## Source layout

| Path | Purpose |
| --- | --- |
| `fw/pqa/main/pqa.c` | Startup, Wi-Fi, heartbeat LED, and ThingSpeak HTTPS client |
| `fw/pqa/main/mcp3913.c` | SPI ADC driver, calibration, and interrupt-driven acquisition |
| `fw/pqa/main/daq.c` | Sample processing, electrical calculations, and FFT diagnostics |
| `fw/pqa/main/rest_server.c` | HTTP JSON and CSV endpoints |
| `fw/pqa/main/time_sync.c` | SNTP synchronization and NVS timestamp storage |
| `fw/pqa/main/include/` | Board selection, channel definitions, and module interfaces |
| `fw/pqa/main/server_root_cert.pem` | Trust certificate for outgoing HTTPS requests |
| `fw/pqa/sdkconfig` | ESP-IDF project configuration |
| `fw/pqa/managed_components/` | Dependencies managed by ESP-IDF (generated) |
| `fw/pqa/build/` | Generated build output |
| `hw/pqa/` | KiCad schematic, PCB layout, and project files |
| `matlab/` | Frequency-estimation helper, RC-filter analysis, and dashboard UI prototype |

The MATLAB dashboard currently creates gauges and labels; it does not fetch
live device measurements. `pqa_freq.m` estimates frequency from sample data, and
`pqa_rc_filter.m` plots the analog RC-filter response.

## Verification

There is no project-specific automated test suite in this source tree. After
building and flashing, check the serial log for Wi-Fi connection, successful time
synchronization, ADC acquisition, and HTTP server startup. Confirm that both API
endpoints respond, then compare measurements against a known reference using the
calibration appropriate to your hardware.
