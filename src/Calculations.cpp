#include "Calculations.h"

#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include "Config.h"

namespace {
struct MovingAverage {
  float values[Config::MovingAverageSamples] = {};
  uint8_t next = 0;
  uint8_t count = 0;
  float sum = 0.0f;

  float add(float value) {
    if (count == Config::MovingAverageSamples) {
      sum -= values[next];
    } else {
      ++count;
    }
    values[next] = value;
    sum += value;
    next = (next + 1) % Config::MovingAverageSamples;
    return sum / count;
  }
};

MovingAverage liftRawFilter;
MovingAverage dragRawFilter;
MovingAverage liftVoltsFilter;
MovingAverage dragVoltsFilter;
MeasurementValues measurements = {};
portMUX_TYPE measurementMux = portMUX_INITIALIZER_UNLOCKED;
bool calibrationLoaded = false;
uint32_t lastInputSequence = 0;
}  // namespace

void initializeMeasurements() {
  calibrationLoaded =
      Config::CalibrationConfigured &&
      isfinite(Config::LiftTareCounts) && isfinite(Config::DragTareCounts) &&
      isfinite(Config::LiftNewtonsPerCount) &&
      isfinite(Config::DragNewtonsPerCount) &&
      Config::LiftNewtonsPerCount != 0.0f &&
      Config::DragNewtonsPerCount != 0.0f;

  measurements.calibrationLoaded = calibrationLoaded;
  if (Config::CalibrationConfigured && !calibrationLoaded) {
    Serial.println("Calibration configuration is invalid; force is unavailable.");
  }
}

void updateMeasurements() {
  if (!isAdsConnected()) {
    liftRawFilter = MovingAverage{};
    dragRawFilter = MovingAverage{};
    liftVoltsFilter = MovingAverage{};
    dragVoltsFilter = MovingAverage{};
    portENTER_CRITICAL(&measurementMux);
    measurements.hasSample = false;
    portEXIT_CRITICAL(&measurementMux);
    return;
  }

  const LoadCellReadings input = getLoadCellReadings();
  if (input.sequence == lastInputSequence || !input.lift.valid ||
      !input.drag.valid) {
    return;
  }
  lastInputSequence = input.sequence;

  MeasurementValues next = {};
  next.liftRaw = liftRawFilter.add(input.lift.raw);
  next.dragRaw = dragRawFilter.add(input.drag.raw);
  next.liftVolts = liftVoltsFilter.add(input.lift.volts);
  next.dragVolts = dragVoltsFilter.add(input.drag.volts);
  next.hasSample = true;
  next.sequence = input.sequence;
  next.calibrationLoaded = calibrationLoaded;
  if (calibrationLoaded) {
    next.liftNewtons =
        (next.liftRaw - Config::LiftTareCounts) * Config::LiftNewtonsPerCount;
    next.dragNewtons =
        (next.dragRaw - Config::DragTareCounts) * Config::DragNewtonsPerCount;
  }

  portENTER_CRITICAL(&measurementMux);
  measurements = next;
  portEXIT_CRITICAL(&measurementMux);
}

MeasurementValues getMeasurementValues() {
  MeasurementValues copy;
  portENTER_CRITICAL(&measurementMux);
  copy = measurements;
  portEXIT_CRITICAL(&measurementMux);
  return copy;
}

bool isCalibrationLoaded() { return calibrationLoaded; }
