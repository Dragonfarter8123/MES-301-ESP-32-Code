#pragma once

#include <Adafruit_ADS1X15.h>

#if __has_include("WiFiSecrets.h")
#include "WiFiSecrets.h"
#else
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#endif

namespace Config {
constexpr uint8_t AdsAddress = 0x48;
constexpr int I2cSdaPin = 21;
constexpr int I2cSclPin = 22;
constexpr adsGain_t AdsGain = GAIN_ONE;  // +/- 4.096 V full scale.
constexpr uint16_t AdsDataRate = RATE_ADS1115_860SPS;
constexpr uint32_t InputSamplingIntervalMs = 0;
constexpr uint32_t ConversionTimeoutMs = 40;
constexpr uint16_t MaxLoggingSampleRateHz = 50;
constexpr uint8_t DftWindowSamples = 64;
constexpr uint8_t DftRetainedFrequencyBins = 1;

// Differential mode reads A0-A1 for lift and A2-A3 for drag.
constexpr bool UseSingleEndedInputs = false;
constexpr uint8_t LiftSingleEndedChannel = 0;
constexpr uint8_t DragSingleEndedChannel = 2;

// Legacy direct-count conversion; the dashboard uses per-team SPIFFS curves.
constexpr bool CalibrationConfigured = false;
constexpr float LiftTareCounts = 0.0f;
constexpr float DragTareCounts = 0.0f;
constexpr float LiftNewtonsPerCount = 0.0f;
constexpr float DragNewtonsPerCount = 0.0f;

constexpr uint32_t MaxTestDurationSeconds = 3600;
constexpr uint16_t MaxCsvSamples = 3000;

constexpr float MinTunnelSpeedMph = 4.0f;
constexpr float MinTunnelEndSpeedMph = 20.0f;
constexpr float MaxTunnelSpeedMph = 40.0f;
constexpr float TunnelSpeedStepMph = 0.5f;
constexpr uint32_t TunnelStabilizationPeriodMs = 8200;

constexpr int DacI2cSdaPin = 18;
constexpr int DacI2cSclPin = 19;
constexpr uint8_t DacI2cAddress = 0x58;
constexpr uint8_t DacOutputChannel = 0;
constexpr float DacOutputRangeVolts = 10.0f;

// Tunnel hardware control stays disabled; this branch enables isolated DAC tests.
constexpr bool TunnelHardwareControlEnabled = false;
constexpr bool TunnelSafetyPrerequisitesVerified = false;
constexpr bool TunnelSafeStopVerified = false;
constexpr float TunnelSafeStopVoltage = 0.0f;
constexpr bool DacBenchTestEnabled = true;
}  // namespace Config
