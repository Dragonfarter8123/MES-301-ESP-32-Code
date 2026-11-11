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
  bool commandAccepted;
  bool outputVoltageVerified;
  bool benchTestActive;
  float commandedDacVoltage;
};

struct I2cBusScanResult {
  bool busReady;
  bool scanError;
  bool overflow;
  uint8_t deviceCount;
  uint8_t deviceAddresses[32];
};

void initializeWindTunnelControl();
void updateWindTunnelControl();
bool setWindTunnelTargetSpeed(float mph, const char **error);
bool updateWindTunnelTargetSpeedDuringTest(float mph, const char **error);
bool startWindTunnelControl(const char **error);
bool stopWindTunnelControl(const char **error);
bool initializeDAC();
bool scanDACI2cBus(I2cBusScanResult &result);
bool setDACVoltage(float voltage);
bool disableDACOutput();
bool isDACConnected();
float getCommandedDACVoltage();
bool setDACBenchTestVoltage(float voltage, const char **error);
bool stopDACBenchTest(const char **error);
WindTunnelControlStatus getWindTunnelControlStatus();

float mphToRPM(float mph);
float rpmToVoltage(float rpm);
float mphToVoltage(float mph);
