#pragma once

#include <stdint.h>

struct WindTunnelControlStatus {
  float targetSpeedMph;
  float estimatedFanRpm;
  float calculatedVoltage;
  uint32_t stabilizationElapsedMs;
  bool calculationAvailable;
  bool dacConnected;
  bool outputEnabled;
  bool running;
  bool stabilizationTimerComplete;
  bool steadyStateVerified;
};

void initializeWindTunnelControl();
void updateWindTunnelControl();
bool setWindTunnelTargetSpeed(float mph, const char **error);
bool startWindTunnelControl(const char **error);
void stopWindTunnelControl();
WindTunnelControlStatus getWindTunnelControlStatus();

float mphToRPM(float mph);
float rpmToVoltage(float rpm);
float mphToVoltage(float mph);
