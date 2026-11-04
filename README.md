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
  RPM/voltage measurements and controls the DFR0971 GP8403 DAC on its
  separate I2C bus.
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

## Wind-tunnel control and DFR0971 DAC

The dashboard's existing speed slider requests a target in the configured
20–40 mph range, with 0.5 mph steps. **Apply Speed** calculates the estimated
RPM and voltage without sending a DAC command. Start Test runs the test and
simulation timer; Stop Test stops both. The default is simulation mode:
`TunnelHardwareControlEnabled`, `TunnelSafetyPrerequisitesVerified`,
`TunnelSafeStopVerified`, and `DacBenchTestEnabled` are all false. In this
mode no DAC command is sent and no physical wind-tunnel control is possible.
Actual speed remains unknown, and steady state is never claimed.

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

The DAC is the **DFRobot DFR0971 (GP8403)**, a 12-bit, 0–10 V I2C DAC. The
official `DFRobot_GP8403` library is included through PlatformIO and accepts a
custom `TwoWire` pointer. The ADS1115 remains on the existing `Wire` bus at
GPIO21/GPIO22, address `0x48`. The DAC uses a separate `TwoWire(1)` bus on
GPIO16/GPIO17, address `0x58`, and VOUT0 only. Its driver uses the library's
millivolt-valued voltage setter; the library maps that to its 12-bit command.
The firmware never calls the library's nonvolatile `store()` method.

On a classic ESP32-WROOM-32, GPIO16 and GPIO17 can be used as general-purpose
pins when no other peripheral uses them. They are used for PSRAM on some
ESP32-WROVER modules, so verify the module marking and board schematic before
wiring. The current project has no other GPIO16/17 users. Connect the DAC
digital `SDA`, `SCL`, `+`/VCC, and `-`/GND to GPIO16, GPIO17, a regulated
supply supported by the DAC board and ESP32 power budget, and common ground.
The DFRobot example powers it at 3.3 V; confirm the DAC board's power demand
and your ESP32 board's available 3.3 V current first. Never connect VOUT0 or
VOUT1 to an ESP32 GPIO or the ADS1115.

Keep the DAC's analog VOUT0 disconnected from the H-6910 until the tunnel's
analog input range, polarity, common reference, enable circuit, and safe-stop
behavior have been confirmed from its documentation and independently
tested. A 0 V DAC command is **not** treated as a verified fan stop. Physical
control is blocked unless the hardware enable, safety prerequisites, and
verified safe-stop configuration are all explicitly enabled in `Config.h`.
The hardware emergency stop must remain independent of the ESP32.

### Isolated DAC bench test

1. Leave the H-6910 disconnected from both DAC output channels.
2. Verify GPIO16/17 and power/ground wiring, then set
   `DacBenchTestEnabled = true` in `include/Config.h` (leave
   `TunnelHardwareControlEnabled = false`).
3. Build and upload the firmware. Startup only initializes and configures
   the DAC; it does not automatically command a test voltage.
4. Open **Test Setup**, select 0, 2, 5, 7, or 9 V in **DAC Bench Test**, and
   press **Set VOUT0**. Measure VOUT0 relative to DAC GND with a multimeter.
5. Compare the meter reading with the displayed command. The firmware cannot
   verify actual analog output voltage; record the measured values yourself.
6. Press **Return to 0 V** and confirm the reading with the meter. Keep the
   H-6910 disconnected; 0 V here is only a bench command and is not certified
   as the tunnel's safe-stop condition.
7. Set `DacBenchTestEnabled = false` after bench testing.

Bench mode is deliberately disabled in the committed configuration and only
exposes the endpoint/UI when enabled. It cannot be used concurrently with
hardware tunnel mode, and a normal tunnel test is blocked until bench mode is
returned to 0 V and stopped.

Tunnel API: `POST /api/tunnel/set-speed` with
`{"targetSpeedMph":30}`, `POST /api/tunnel/start`, `POST /api/tunnel/stop`,
`GET /api/tunnel/status`, and the config-gated bench routes
`POST /api/tunnel/bench-output` with `{"voltage":5}` and
`POST /api/tunnel/bench-stop`. Status distinguishes calculated voltage,
commanded voltage, and voltage measurement verification. Commanding the DAC
does not verify its physical output.

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
- DFR0971 output voltage, DAC wiring, H-6910 operation, actual wind speed, and
  steady-state feedback require physical hardware verification; none were
  tested. DAC bench mode and wind-tunnel hardware mode remain disabled.
