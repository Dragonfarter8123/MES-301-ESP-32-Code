#pragma once

#include <stdint.h>

#include "ESP10_Inputs.h"

struct MeasurementValues {
  float liftRaw;
  float dragRaw;
  float liftVolts;
  float dragVolts;
  float liftNewtons;
  float dragNewtons;
  bool hasSample;
  bool calibrationLoaded;
  uint32_t sequence;
};

void initializeMeasurements();
void updateMeasurements();
MeasurementValues getMeasurementValues();
bool isCalibrationLoaded();
