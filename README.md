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
- `src/WindTunnelControl.cpp` interpolates the supplied speed/RPM and
  RPM/voltage measurements and provides simulation-only tunnel state.
- `data/index.html` is the dashboard uploaded to SPIFFS.
- `include/Config.h` holds hardware, sampling, calibration, speed-range, and
  stabilization settings.

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
values.

## Wind-tunnel control (simulation only)

The dashboard's existing speed slider requests a target in the configured
20–40 mph range, with 0.5 mph steps. **Apply Speed** calculates the estimated
RPM and voltage without starting the tunnel. Start Test runs the test and
simulation timer; Stop Test stops both. In simulation mode no DAC command is
sent, the DAC is reported unavailable, actual speed remains unknown, and
steady state is never claimed.

Interpolation uses the airspeed/RPM and RPM/control-voltage measurements
provided in the project request. It does not extrapolate. Therefore not every
speed in the configured range is currently calculable: the first airspeed
reference is 20.54 mph. The last reference point, 39.98 mph, maps to 3209 RPM,
above the voltage table's 3183 RPM maximum, so 40 mph is also unavailable.
For example, 30 mph calculates to approximately 2415 RPM and 6.85 V; 20 mph
and 40 mph are rejected pending additional measurements.
`MinTunnelSpeedMph`, `MaxTunnelSpeedMph`, `TunnelSpeedStepMph`, and
`TunnelStabilizationPeriodMs` are in `include/Config.h`. The stabilization
timer is not evidence that the tunnel reached the requested speed.

The PCM5122 is an I2S audio DAC, not a verified industrial 0–10 V control
output. The exact module schematic, conditioning circuit, and H-6910 analog
input/safe-stop specifications were not provided, so compatibility and safe
motor-stop behavior cannot be confirmed. The DAC driver functions are
intentionally disabled and `TunnelPhysicalOutputEnabled` is `false`.
Do not enable physical output until the exact board, conditioned output,
polarity/range/startup behavior, and tunnel input requirements have been
independently verified. Keep the tunnel's hardware emergency stop available.

Tunnel API: `POST /api/tunnel/set-speed` with
`{"targetSpeedMph":30}`, `POST /api/tunnel/start`, `POST /api/tunnel/stop`,
and `GET /api/tunnel/status`. Existing test, measurement, calibration, and CSV
routes remain available.

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
`POST /api/stop`, `POST /api/calibration`, and `GET /api/export.csv`, plus the
tunnel endpoints listed above.

## Software verification

- PlatformIO firmware build: passed.
- PlatformIO SPIFFS filesystem build: passed.
- Speed/RPM/voltage interpolation: 25, 30, and 35 mph are within the supplied
  measurements; 20 mph is below the first airspeed point, and the 39.98 mph
  reference exceeds the voltage table's RPM range, so 20 and 40 correctly
  require more calibration data.
- Physical DAC output, fan control, actual wind speed, and steady-state
  feedback require hardware verification and were not tested or enabled.
