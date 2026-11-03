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
constexpr adsGain_t AdsGain = GAIN_SIXTEEN;  // +/- 0.256 V full scale.
constexpr uint16_t AdsDataRate = RATE_ADS1115_128SPS;
constexpr uint32_t InputSamplingIntervalMs = 20;
constexpr uint32_t ConversionTimeoutMs = 40;
constexpr uint8_t MovingAverageSamples = 5;

// Differential mode reads A0-A1 for lift and A2-A3 for drag.
constexpr bool UseSingleEndedInputs = false;
constexpr uint8_t LiftSingleEndedChannel = 0;
constexpr uint8_t DragSingleEndedChannel = 2;

// Set these only after measuring and validating a real calibration.
constexpr bool CalibrationConfigured = false;
constexpr float LiftTareCounts = 0.0f;
constexpr float DragTareCounts = 0.0f;
constexpr float LiftNewtonsPerCount = 0.0f;
constexpr float DragNewtonsPerCount = 0.0f;

constexpr uint32_t MaxTestDurationSeconds = 60;
constexpr uint16_t MaxCsvSamples = 3000;

constexpr float MinTunnelSpeedMph = 20.0f;
constexpr float MaxTunnelSpeedMph = 40.0f;
constexpr float TunnelSpeedStepMph = 0.5f;
constexpr uint32_t TunnelStabilizationPeriodMs = 8200;
constexpr bool TunnelPhysicalOutputEnabled = false;
}  // namespace Config
