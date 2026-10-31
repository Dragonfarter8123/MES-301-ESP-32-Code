#pragma once

#include <Arduino.h>
#include <stdint.h>

struct ChannelSample {
  int16_t raw = 0;
  float adcVolts = 0.0f;
  float inputEquivalentVolts = 0.0f;
  float forceNewtons = 0.0f;
  bool adcValid = false;
  bool forceValid = false;
  const char* error = "not sampled";
};

struct MeasurementRecord {
  uint32_t elapsedMs = 0;
  float speedSetpointMph = 0.0f;
  float liftNewtons = 0.0f;
  float dragNewtons = 0.0f;
};
