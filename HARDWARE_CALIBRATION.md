# Wind tunnel hardware and calibration

## Firmware modules

`src/main.cpp` coordinates the four firmware modules:

1. `src/UserInterface.cpp` serves `data/index.html` from SPIFFS and implements the JSON/CSV API.
2. `src/WindTunnelControl.cpp` manages test settings, timing, sample logging, and the optional fan-driver PWM.
3. `src/Calculations.cpp` converts ADC readings through the AD623 gain/reference model into independently calibrated lift and drag forces.
4. `src/Inputs.cpp` configures the ADS1115, acquires both channels, checks readings, and prints serial diagnostics.

Shared declarations and editable hardware settings are in `include/`. The source files are compiled by PlatformIO; the HTML is uploaded separately with `pio run -t uploadfs`.

## AD623ANZ and ADS1115 wiring

The two AD623ANZ amplifiers are separate channels:

- Lift load cell -> AD623 #1 -> ADS1115 A0 in single-ended mode.
- Drag load cell -> AD623 #2 -> ADS1115 A2 in single-ended mode.
- ESP32 GPIO 21 -> ADS1115 SDA.
- ESP32 GPIO 22 -> ADS1115 SCL.
- ADS1115 I2C address: `0x48`.
- Join the analog signal references as required by the circuit. Connect grounds only as appropriate for the power and signal design.

Single-ended mode is selected initially in `include/Config.h`. Confirm that the actual board is wired this way before powering it. If the ADS1115 still uses differential pairs, set `ADC_INPUT_MODE` to `Differential` and wire lift to A0/A1 and drag to A2/A3. In differential mode, set the two `*_ADC_NEGATIVE_REFERENCE_VOLTS` values to the voltage actually present on each negative ADC input. The calculation reconstructs the AD623 output relative to ground before subtracting the amplifier REF voltage.

Each AD623 needs a valid supply, gain resistor, REF connection, and output connection. Verify the input common-mode range and output swing from the AD623 datasheet for the selected supply, sensor, gain, and reference. Do not connect an AD623 output to the ADS1115 if it can exceed the ADS1115's permitted input voltage. The ADS1115 PGA full-scale setting does **not** extend the absolute voltage range of its analog pins. The ESP32, ADS1115, and amplifier circuit must share a safe signal reference.

## Editable configuration

Edit `include/Config.h`:

- `LIFT_AD623_RG_OHMS` and `DRAG_AD623_RG_OHMS`: enter the measured/installed gain resistor values independently. They default to zero, which explicitly leaves gain unconfigured.
- `LIFT_AD623_REFERENCE_KNOWN` / `DRAG_AD623_REFERENCE_KNOWN`: set true only after verifying the respective REF pin voltage.
- `LIFT_AD623_REFERENCE_VOLTS` / `DRAG_AD623_REFERENCE_VOLTS`: enter measured REF voltage in volts.
- `ADC_INPUT_MODE`: choose `SingleEnded` or `Differential` to match wiring.
- `ADS1115_GAIN`: selects the ADS1115 PGA range. It currently defaults to `GAIN_SIXTEEN`, which is a differential full-scale range of ±0.256 V (not ±4.096 V). A single-ended AD623 output above 0.256 V will saturate at this setting even if it is still within the ADS1115 supply range. Select a wider range, such as `GAIN_ONE` (±4.096 V), if the measured signal requires it and the inputs remain within the ADS1115's absolute pin limits.
- `FAN_PWM_ENABLED`: remains false until the ESP32 output, fan driver, and speed-control relationship have been confirmed. Never drive a fan or motor directly from an ESP32 GPIO. If enabled, the firmware outputs PWM on `FAN_PWM_PIN`; the setpoint-to-duty mapping is open-loop and is not an airspeed measurement.

AD623 gain is computed independently for each amplifier:

```text
G = 1 + (100,000 / RG)
RG = 100,000 / (G - 1)
```

The input-referred signal is calculated as `(AD623 output voltage - AD623 REF voltage) / G`. For differential ADC mode, the configured ADS1115 negative-input reference is added back to the measured differential before applying this equation.

## Startup and diagnostics

The firmware continues to start Wi-Fi/AP fallback and the web/API server if the ADS1115 is missing. Missing filesystem content or ADC hardware is reported explicitly; no synthetic measurements are generated. Serial Monitor runs at 115200 baud and retains the raw ADC and voltage diagnostic output. Saturated ADC counts and out-of-supply single-ended readings are marked invalid and are not logged as forces.

## Zeroing and force calibration

1. Verify the two amplifier gain resistors, REF pin voltages, ADC mode, wiring, ADC gain, and safe ADC input voltage in `include/Config.h`.
2. Upload firmware after changing configuration: `pio run -t upload`.
3. Open the dashboard and select **Zero Load Cells** with both sensors unloaded. Zero offsets are saved in ESP32 preferences.
4. Apply a known force to the lift cell and drag cell. Enter each force independently in newtons in **Run Calibration**. For a known mass, calculate its applied force as `mass_kg * 9.80665`.
5. Calibration coefficients are independently calculated in N per AD623-input-referred volt and saved in ESP32 preferences. The dashboard's force values and CSV use only calibrated newton measurements; an uncalibrated or invalid channel is not represented as a force measurement.
6. Remove/reapply loads to check repeatability, sign, zero return, and saturation. Re-zero after changing the load-cell mounting or no-load baseline.

Changing the configured AD623 resistor/reference, ADC mode, or ADS1115 gain invalidates the saved calibration. Re-zero and recalibrate both channels after such changes. A force calibration requires a valid ADC reading and a nonzero response from that channel.

## Web API

- `GET /api/status`
- `GET /api/data`
- `POST /api/start`
- `POST /api/stop`
- `POST /api/zero`
- `POST /api/calibration` with `liftKnownForceN` and/or `dragKnownForceN`
- `GET /api/export.csv`

When Wi-Fi credentials are configured in `include/Config.h` and unavailable, the ESP32 starts the configured fallback access point. Connect to it and open the AP IP printed in Serial Monitor. Edit only `data/index.html` for dashboard appearance changes, then upload the filesystem with `pio run -t uploadfs`.
