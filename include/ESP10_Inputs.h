#pragma once

#include <Arduino.h>
#include <stdint.h>

struct SensorReading {
  int16_t raw;
  float volts;
  bool valid;
};

struct LoadCellReadings {
  SensorReading lift;
  SensorReading drag;
  uint32_t sequence;
};

bool initializeLoadCellInputs();
void updateLoadCellInputs();
bool isAdsConnected();
String getAdsError();
LoadCellReadings getLoadCellReadings();
SensorReading readLiftSensor();
SensorReading readDragSensor();
