# MES 301 Wind Tunnel

ESP32-WROOM-32 firmware and a SPIFFS-hosted dashboard for two ESP-10 load
cells. The signal path is ESP-10 -> AD623 -> ADS1115 -> ESP32. Lift and drag
are read as differential ADS1115 channels by default.

## Files and settings

- `src/main.cpp` calls the input, processing, and server modules.
- `src/ESP10_Inputs.cpp` uses Adafruit_ADS1X15 and asynchronous conversion
  polling with a timeout.
- `src/Calculations.cpp` filters each input independently. Force stays
  unavailable until valid coefficients are supplied.
- `src/WebServer.cpp` provides the Wi-Fi, SPIFFS, JSON, test-control, and CSV
  endpoints.
- `data/index.html` is the dashboard uploaded to SPIFFS.
- `include/Config.h` holds hardware, sampling, and calibration settings.

Create `include/WiFiSecrets.h` locally (it is ignored by Git):

```cpp
#pragma once
#define WIFI_SSID "your-wifi-name"
#define WIFI_PASSWORD "your-wifi-password"
```

Without this local file, the firmware still builds and starts the web server,
but does not connect to Wi-Fi.

The default ADC configuration is address `0x48`, SDA 21, SCL 22, gain
`GAIN_SIXTEEN` (±0.256 V full scale), and differential A0-A1/A2-A3. To test
single-ended output wiring, set `UseSingleEndedInputs` to `true` and select
the lift/drag channels in `Config.h`.

**Check analog voltage compatibility before connecting the AD623 outputs.**
The ADS1115 inputs must remain between ground and the ADS1115 supply, and at
`GAIN_SIXTEEN` the measured differential signal must remain within ±0.256 V.
Common-mode voltage also matters. The AD623 supply rails, gain, reference, and
output range were not provided, so compatibility cannot be confirmed from
software; verify the amplifier output with a meter before connection.

Calibration coefficients are intentionally unset. After calibrating each
sensor with known loads, set `CalibrationConfigured`, tare counts, and
Newtons-per-count in `Config.h`. Both scales must be finite and nonzero or
force output remains unavailable. The dashboard calibration action reports
that calibration data is missing; it does not invent or save calibration
values. Actual airspeed and fan control are not implemented; no PWM is
generated, and actual speed remains unknown.

## Build, upload, and run

In the PlatformIO terminal, from this project directory:

```text
pio run
pio run -t upload
pio run -t uploadfs
pio device monitor
```

Upload the filesystem after changing `data/index.html`. The serial monitor
runs at 115200 baud. With Wi-Fi credentials configured, open the printed ESP32
IP address in a browser. Use **Start Test** and **Stop Test** from the shared
header on any dashboard page. Test settings are applied when the test starts;
choose a team (1–5), enter a duration from 1–60 seconds and a logging rate
from 1–50 Hz, or select one of the provided presets. The measurements graph
shows filtered output voltage in millivolts. CSV contains only samples
collected during a test.

API: `GET /`, `GET /api/status`, `GET /api/data`, `POST /api/start`,
`POST /api/stop`, `POST /api/calibration`, and `GET /api/export.csv`.
