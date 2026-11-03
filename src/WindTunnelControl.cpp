#include "WindTunnelControl.h"

#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

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
    false, false, false, false};
uint32_t controlStartedAt = 0;

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

bool initializeDAC() { return false; }

bool setDACVoltage(float) { return false; }

bool disableDACOutput() { return false; }

void invalidateTarget() {
  portENTER_CRITICAL(&controlMux);
  controlStatus.calculationAvailable = false;
  controlStatus.estimatedFanRpm = NAN;
  controlStatus.calculatedVoltage = NAN;
  portEXIT_CRITICAL(&controlMux);
}
}  // namespace

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
  const bool dacReady =
      Config::TunnelPhysicalOutputEnabled && initializeDAC();
  portENTER_CRITICAL(&controlMux);
  controlStatus.targetSpeedMph = 25.0f;
  controlStatus.estimatedFanRpm = NAN;
  controlStatus.calculatedVoltage = NAN;
  controlStatus.stabilizationElapsedMs = 0;
  controlStatus.calculationAvailable = false;
  controlStatus.dacConnected = dacReady;
  controlStatus.outputEnabled = false;
  controlStatus.running = false;
  controlStatus.stabilizationTimerComplete = false;
  controlStatus.steadyStateVerified = false;
  controlStartedAt = 0;
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
      *error = "Target speed must be from 20 to 40 mph in 0.5 mph steps.";
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
  if (controlStatus.running) {
    portEXIT_CRITICAL(&controlMux);
    if (error != nullptr) {
      *error = "Stop the current test before applying a different target.";
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

bool startWindTunnelControl(const char **error) {
  portENTER_CRITICAL(&controlMux);
  if (!controlStatus.calculationAvailable) {
    portEXIT_CRITICAL(&controlMux);
    if (error != nullptr) {
      *error = "Apply a supported target speed before starting.";
    }
    return false;
  }

  if (Config::TunnelPhysicalOutputEnabled) {
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

void stopWindTunnelControl() {
  portENTER_CRITICAL(&controlMux);
  const bool outputEnabled = controlStatus.outputEnabled;
  controlStatus.running = false;
  controlStatus.outputEnabled = false;
  controlStatus.stabilizationElapsedMs = 0;
  controlStatus.stabilizationTimerComplete = false;
  controlStatus.steadyStateVerified = false;
  portEXIT_CRITICAL(&controlMux);

  if (outputEnabled && Config::TunnelPhysicalOutputEnabled) {
    disableDACOutput();
  }
}

WindTunnelControlStatus getWindTunnelControlStatus() {
  portENTER_CRITICAL(&controlMux);
  WindTunnelControlStatus status = controlStatus;
  portEXIT_CRITICAL(&controlMux);
  return status;
}
