#include "WindTunnelControl.h"

#include <Arduino.h>
#include <DFRobot_GP8403.h>
#include <math.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/semphr.h>

#include "Config.h"

namespace {
struct SpeedRpmPoint {
  float mph;
  float rpm;
};

struct RpmVoltagePoint {
  float rpm;
  float voltage;
};

constexpr SpeedRpmPoint SpeedRpmData[] = {
    {20.54f, 1662.0f}, {22.78f, 1841.0f}, {25.03f, 2019.0f},
    {27.27f, 2198.0f}, {29.51f, 2376.0f}, {31.76f, 2554.0f},
    {34.00f, 2732.0f}, {36.24f, 2910.0f}, {38.48f, 3090.0f},
    {39.98f, 3209.0f},
};

constexpr RpmVoltagePoint RpmVoltageData[] = {
    {329.0f, 1.05f},  {347.0f, 1.1f},   {496.0f, 1.5f},
    {677.0f, 2.0f},   {858.0f, 2.5f},   {1039.0f, 3.0f},
    {1218.0f, 3.5f},  {1397.0f, 4.0f},  {1577.0f, 4.5f},
    {1756.0f, 5.0f},  {1933.0f, 5.5f},  {2111.0f, 6.0f},
    {2290.0f, 6.5f},  {2470.0f, 7.0f},  {2646.0f, 7.5f},
    {2825.0f, 8.0f},  {3003.0f, 8.5f},  {3183.0f, 9.0f},
};

constexpr size_t SpeedRpmDataCount =
    sizeof(SpeedRpmData) / sizeof(SpeedRpmData[0]);
constexpr size_t RpmVoltageDataCount =
    sizeof(RpmVoltageData) / sizeof(RpmVoltageData[0]);

portMUX_TYPE controlMux = portMUX_INITIALIZER_UNLOCKED;
WindTunnelControlStatus controlStatus = {
    25.0f, NAN, NAN, 0, false, false,
    false, false, false, false, false, false, false, NAN};
uint32_t controlStartedAt = 0;
SemaphoreHandle_t dacMutex = nullptr;
TwoWire dacWire(1);
DFRobot_GP8403 dac(&dacWire, Config::DacI2cAddress);
bool dacBusStarted = false;

bool interpolate(float input, float &output, float firstX, float lastX,
                 const float *xValues, const float *yValues, size_t count) {
  if (!isfinite(input) || input < firstX || input > lastX || count < 2) {
    return false;
  }

  for (size_t i = 1; i < count; ++i) {
    if (input <= xValues[i]) {
      const float fraction =
          (input - xValues[i - 1]) / (xValues[i] - xValues[i - 1]);
      output = yValues[i - 1] +
               fraction * (yValues[i] - yValues[i - 1]);
      return isfinite(output);
    }
  }
  return false;
}

void invalidateTarget() {
  portENTER_CRITICAL(&controlMux);
  controlStatus.calculationAvailable = false;
  controlStatus.estimatedFanRpm = NAN;
  controlStatus.calculatedVoltage = NAN;
  portEXIT_CRITICAL(&controlMux);
}
}  // namespace

bool initializeDAC() {
  if (dacMutex == nullptr || xSemaphoreTake(dacMutex, portMAX_DELAY) != pdTRUE) {
    return false;
  }

  if (isDACConnected()) {
    xSemaphoreGive(dacMutex);
    return true;
  }

  if (!dacBusStarted) {
    dacBusStarted = dacWire.begin(Config::DacI2cSdaPin,
                                  Config::DacI2cSclPin);
  }
  const bool initialized =
      dacBusStarted && dac.begin() == 0;
  if (initialized) {
    dac.setDACOutRange(DFRobot_GP8403::eOutputRange10V);
    dacWire.beginTransmission(Config::DacI2cAddress);
  }
  const bool connected =
      initialized && dacWire.endTransmission() == 0;
  portENTER_CRITICAL(&controlMux);
  controlStatus.dacConnected = connected;
  portEXIT_CRITICAL(&controlMux);
  xSemaphoreGive(dacMutex);
  return connected;
}

bool setDACVoltage(float voltage) {
  if (!isfinite(voltage) || voltage < 0.0f ||
      voltage > Config::DacOutputRangeVolts || !isDACConnected() ||
      dacMutex == nullptr || xSemaphoreTake(dacMutex, portMAX_DELAY) != pdTRUE) {
    return false;
  }

  const uint16_t millivolts =
      static_cast<uint16_t>(lroundf(voltage * 1000.0f));
  // The GP8403 library converts millivolts to its 12-bit code for the selected range.
  dac.setDACOutVoltage(millivolts, Config::DacOutputChannel);

  // The library setter returns void, so this confirms bus presence, not output voltage.
  dacWire.beginTransmission(Config::DacI2cAddress);
  const bool connected = dacWire.endTransmission() == 0;
  portENTER_CRITICAL(&controlMux);
  controlStatus.dacConnected = connected;
  controlStatus.commandAccepted = connected;
  controlStatus.outputVoltageVerified = false;
  if (connected) {
    controlStatus.commandedDacVoltage = millivolts / 1000.0f;
  }
  portEXIT_CRITICAL(&controlMux);
  xSemaphoreGive(dacMutex);
  return connected;
}

bool disableDACOutput() {
  if (!Config::TunnelHardwareControlEnabled ||
      !Config::TunnelSafetyPrerequisitesVerified ||
      !Config::TunnelSafeStopVerified ||
      !isfinite(Config::TunnelSafeStopVoltage) ||
      Config::TunnelSafeStopVoltage < 0.0f ||
      Config::TunnelSafeStopVoltage > Config::DacOutputRangeVolts) {
    return false;
  }
  return setDACVoltage(Config::TunnelSafeStopVoltage);
}

bool isDACConnected() {
  portENTER_CRITICAL(&controlMux);
  const bool connected = controlStatus.dacConnected;
  portEXIT_CRITICAL(&controlMux);
  return connected;
}

float getCommandedDACVoltage() {
  portENTER_CRITICAL(&controlMux);
  const float voltage = controlStatus.commandedDacVoltage;
  portEXIT_CRITICAL(&controlMux);
  return voltage;
}

bool setDACBenchTestVoltage(float voltage, const char **error) {
  constexpr float testVoltages[] = {0.0f, 2.0f, 5.0f, 7.0f, 9.0f};
  bool supportedVoltage = false;
  for (float testVoltage : testVoltages) {
    if (fabsf(voltage - testVoltage) < 0.001f) {
      supportedVoltage = true;
      break;
    }
  }
  if (!Config::DacBenchTestEnabled ||
      Config::TunnelHardwareControlEnabled) {
    if (error != nullptr) {
      *error = "DAC bench-test mode is disabled in Config.h.";
    }
    return false;
  }
  if (!supportedVoltage) {
    if (error != nullptr) {
      *error = "Bench voltage must be one of 0, 2, 5, 7, or 9 V.";
    }
    return false;
  }
  if (getWindTunnelControlStatus().running) {
    if (error != nullptr) {
      *error = "Stop the test before using DAC bench-test mode.";
    }
    return false;
  }
  if (!isDACConnected() && !initializeDAC()) {
    if (error != nullptr) {
      *error = "DFR0971 is not connected on its configured I2C bus.";
    }
    return false;
  }
  if (!setDACVoltage(voltage)) {
    if (error != nullptr) {
      *error = "Could not communicate with the DFR0971.";
    }
    return false;
  }
  portENTER_CRITICAL(&controlMux);
  controlStatus.benchTestActive = true;
  portEXIT_CRITICAL(&controlMux);
  if (error != nullptr) {
    *error = nullptr;
  }
  return true;
}

bool stopDACBenchTest(const char **error) {
  if (!Config::DacBenchTestEnabled ||
      Config::TunnelHardwareControlEnabled) {
    if (error != nullptr) {
      *error = "DAC bench-test mode is disabled in Config.h.";
    }
    return false;
  }
  if (!setDACVoltage(0.0f)) {
    if (error != nullptr) {
      *error = "Could not send the 0 V bench-test command.";
    }
    return false;
  }
  portENTER_CRITICAL(&controlMux);
  controlStatus.benchTestActive = false;
  portEXIT_CRITICAL(&controlMux);
  if (error != nullptr) {
    *error = nullptr;
  }
  return true;
}

float mphToRPM(float mph) {
  if (!isfinite(mph) || mph < Config::MinTunnelSpeedMph ||
      mph > Config::MaxTunnelSpeedMph) {
    return NAN;
  }

  float speeds[SpeedRpmDataCount];
  float rpms[SpeedRpmDataCount];
  for (size_t i = 0; i < SpeedRpmDataCount; ++i) {
    speeds[i] = SpeedRpmData[i].mph;
    rpms[i] = SpeedRpmData[i].rpm;
  }

  float rpm = NAN;
  if (mph < speeds[0]) {
    const float fraction = (mph - speeds[0]) / (speeds[1] - speeds[0]);
    rpm = rpms[0] + fraction * (rpms[1] - rpms[0]);
    return isfinite(rpm);
  }
  return interpolate(mph, rpm, SpeedRpmData[0].mph,
                     SpeedRpmData[SpeedRpmDataCount - 1].mph, speeds, rpms,
                     SpeedRpmDataCount)
             ? rpm
             : NAN;
}

float rpmToVoltage(float rpm) {
  float rpms[RpmVoltageDataCount];
  float voltages[RpmVoltageDataCount];
  for (size_t i = 0; i < RpmVoltageDataCount; ++i) {
    rpms[i] = RpmVoltageData[i].rpm;
    voltages[i] = RpmVoltageData[i].voltage;
  }

  float voltage = NAN;
  return interpolate(rpm, voltage, RpmVoltageData[0].rpm,
                     RpmVoltageData[RpmVoltageDataCount - 1].rpm, rpms,
                     voltages, RpmVoltageDataCount)
             ? voltage
             : NAN;
}

float mphToVoltage(float mph) {
  const float rpm = mphToRPM(mph);
  return isfinite(rpm) ? rpmToVoltage(rpm) : NAN;
}

void initializeWindTunnelControl() {
  dacMutex = xSemaphoreCreateMutex();
  portENTER_CRITICAL(&controlMux);
  controlStatus.targetSpeedMph = 25.0f;
  controlStatus.estimatedFanRpm = NAN;
  controlStatus.calculatedVoltage = NAN;
  controlStatus.stabilizationElapsedMs = 0;
  controlStatus.calculationAvailable = false;
  controlStatus.dacConnected = false;
  controlStatus.outputEnabled = false;
  controlStatus.running = false;
  controlStatus.stabilizationTimerComplete = false;
  controlStatus.steadyStateVerified = false;
  controlStatus.commandAccepted = false;
  controlStatus.outputVoltageVerified = false;
  controlStatus.benchTestActive = false;
  controlStatus.commandedDacVoltage = NAN;
  controlStartedAt = 0;
  portEXIT_CRITICAL(&controlMux);

  const bool shouldInitializeDac =
      Config::TunnelHardwareControlEnabled || Config::DacBenchTestEnabled;
  bool dacReady = shouldInitializeDac && initializeDAC();
  if (dacMutex == nullptr) {
    Serial.println("ERROR: Could not create the DAC bus mutex.");
  }
  if (shouldInitializeDac && !dacReady) {
    Serial.println("ERROR: DFR0971 DAC initialization failed.");
  }
  if (dacReady && Config::TunnelHardwareControlEnabled &&
      Config::TunnelSafetyPrerequisitesVerified &&
      Config::TunnelSafeStopVerified &&
      !setDACVoltage(Config::TunnelSafeStopVoltage)) {
    dacReady = false;
    Serial.println("ERROR: Could not command the configured DAC safe-stop state.");
  }
  portENTER_CRITICAL(&controlMux);
  controlStatus.dacConnected = dacReady;
  portEXIT_CRITICAL(&controlMux);
}

void updateWindTunnelControl() {
  portENTER_CRITICAL(&controlMux);
  if (controlStatus.running) {
    const uint32_t elapsed = millis() - controlStartedAt;
    controlStatus.stabilizationElapsedMs =
        elapsed < Config::TunnelStabilizationPeriodMs
            ? elapsed
            : Config::TunnelStabilizationPeriodMs;
    controlStatus.stabilizationTimerComplete =
        elapsed >= Config::TunnelStabilizationPeriodMs;
    controlStatus.steadyStateVerified = false;
  }
  portEXIT_CRITICAL(&controlMux);
}

bool setWindTunnelTargetSpeed(float mph, const char **error) {
  if (!isfinite(mph) || mph < Config::MinTunnelSpeedMph ||
      mph > Config::MaxTunnelSpeedMph ||
      fabsf(mph / Config::TunnelSpeedStepMph -
            roundf(mph / Config::TunnelSpeedStepMph)) > 0.001f) {
    if (error != nullptr) {
      *error = "Target speed must be from 4 to 40 mph in 0.5 mph steps.";
    }
    return false;
  }

  const float rpm = mphToRPM(mph);
  const float voltage = mphToVoltage(mph);
  if (!isfinite(rpm) || !isfinite(voltage)) {
    invalidateTarget();
    if (error != nullptr) {
      *error =
          "Target speed is outside the supplied measured data range; "
          "additional calibration is required.";
    }
    return false;
  }

  portENTER_CRITICAL(&controlMux);
  if (controlStatus.running || controlStatus.benchTestActive) {
    portEXIT_CRITICAL(&controlMux);
    if (error != nullptr) {
      *error =
          "Stop the current test or DAC bench test before applying a target.";
    }
    return false;
  }
  controlStatus.targetSpeedMph = mph;
  controlStatus.estimatedFanRpm = rpm;
  controlStatus.calculatedVoltage = voltage;
  controlStatus.calculationAvailable = true;
  portEXIT_CRITICAL(&controlMux);
  return true;
}

bool updateWindTunnelTargetSpeedDuringTest(float mph, const char **error) {
  if (!isfinite(mph) || mph < Config::MinTunnelSpeedMph ||
      mph > Config::MaxTunnelSpeedMph ||
      fabsf(mph / Config::TunnelSpeedStepMph -
            roundf(mph / Config::TunnelSpeedStepMph)) > 0.001f) {
    if (error != nullptr) {
      *error = "Target speed must be from 4 to 40 mph in 0.5 mph steps.";
    }
    return false;
  }

  const float rpm = mphToRPM(mph);
  const float voltage = mphToVoltage(mph);
  if (!isfinite(rpm) || !isfinite(voltage)) {
    if (error != nullptr) {
      *error =
          "Target speed is outside the supplied measured data range; "
          "additional calibration is required.";
    }
    return false;
  }

  portENTER_CRITICAL(&controlMux);
  if (!controlStatus.running || controlStatus.benchTestActive) {
    portEXIT_CRITICAL(&controlMux);
    if (error != nullptr) {
      *error = "A speed sweep target can only change during an active test.";
    }
    return false;
  }
  const bool updatePhysicalOutput = Config::TunnelHardwareControlEnabled;
  const bool dacConnected = controlStatus.dacConnected;
  portEXIT_CRITICAL(&controlMux);

  if (updatePhysicalOutput && (!dacConnected || !setDACVoltage(voltage))) {
    if (error != nullptr) {
      *error = "Could not update the physical tunnel output for the new target.";
    }
    return false;
  }

  portENTER_CRITICAL(&controlMux);
  controlStatus.targetSpeedMph = mph;
  controlStatus.estimatedFanRpm = rpm;
  controlStatus.calculatedVoltage = voltage;
  controlStatus.calculationAvailable = true;
  portEXIT_CRITICAL(&controlMux);
  if (error != nullptr) {
    *error = nullptr;
  }
  return true;
}

bool startWindTunnelControl(const char **error) {
  portENTER_CRITICAL(&controlMux);
  if (controlStatus.benchTestActive) {
    portEXIT_CRITICAL(&controlMux);
    if (error != nullptr) {
      *error = "Stop DAC bench-test mode before starting a tunnel test.";
    }
    return false;
  }
  if (!controlStatus.calculationAvailable) {
    portEXIT_CRITICAL(&controlMux);
    if (error != nullptr) {
      *error = "Apply a supported target speed before starting.";
    }
    return false;
  }

  if (!controlStatus.running && controlStatus.outputEnabled) {
    portEXIT_CRITICAL(&controlMux);
    if (error != nullptr) {
      *error = "A previous DAC stop command failed; check the output before retrying.";
    }
    return false;
  }

  if (Config::TunnelHardwareControlEnabled) {
    if (!Config::TunnelSafetyPrerequisitesVerified ||
        !Config::TunnelSafeStopVerified) {
      portEXIT_CRITICAL(&controlMux);
      if (error != nullptr) {
        *error =
            "Hardware control is locked until tunnel input and safe-stop "
            "requirements are verified in Config.h.";
      }
      return false;
    }
    const float voltage = controlStatus.calculatedVoltage;
    const bool dacConnected = controlStatus.dacConnected;
    portEXIT_CRITICAL(&controlMux);
    if (!dacConnected || !setDACVoltage(voltage)) {
      if (error != nullptr) {
        *error = "Physical output is not available; tunnel start was blocked.";
      }
      return false;
    }
    portENTER_CRITICAL(&controlMux);
    controlStatus.outputEnabled = true;
  }

  if (!controlStatus.running) {
    controlStartedAt = millis();
    controlStatus.stabilizationElapsedMs = 0;
    controlStatus.stabilizationTimerComplete = false;
    controlStatus.steadyStateVerified = false;
  }
  controlStatus.running = true;
  portEXIT_CRITICAL(&controlMux);

  if (error != nullptr) {
    *error = nullptr;
  }
  return true;
}

bool stopWindTunnelControl(const char **error) {
  portENTER_CRITICAL(&controlMux);
  const bool outputEnabled = controlStatus.outputEnabled;
  controlStatus.running = false;
  controlStatus.stabilizationElapsedMs = 0;
  controlStatus.stabilizationTimerComplete = false;
  controlStatus.steadyStateVerified = false;
  portEXIT_CRITICAL(&controlMux);

  if (outputEnabled && !disableDACOutput()) {
    if (error != nullptr) {
      *error = "Safe-stop command failed; the DAC output state is unknown.";
    }
    return false;
  }
  portENTER_CRITICAL(&controlMux);
  controlStatus.outputEnabled = false;
  portEXIT_CRITICAL(&controlMux);
  if (error != nullptr) {
    *error = nullptr;
  }
  return true;
}

WindTunnelControlStatus getWindTunnelControlStatus() {
  portENTER_CRITICAL(&controlMux);
  WindTunnelControlStatus status = controlStatus;
  portEXIT_CRITICAL(&controlMux);
  return status;
}
