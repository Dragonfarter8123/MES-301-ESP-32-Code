#include "ESP10_Inputs.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include "Config.h"

namespace {
Adafruit_ADS1115 ads;
LoadCellReadings readings = {{0, 0.0f, false}, {0, 0.0f, false}, 0};
bool adsConnected = false;
char adsError[96] = "ADS1115 has not been initialized";
portMUX_TYPE inputStatusMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t conversionStartedAt = 0;
uint32_t lastPairCompletedAt = 0;
uint32_t lastBeginAttemptAt = 0;
enum class ConversionState { WaitingToStart, WaitingForLift, WaitingForDrag };
ConversionState conversionState = ConversionState::WaitingToStart;

void setAdsError(const char *message) {
  portENTER_CRITICAL(&inputStatusMux);
  strncpy(adsError, message, sizeof(adsError) - 1);
  adsError[sizeof(adsError) - 1] = '\0';
  portEXIT_CRITICAL(&inputStatusMux);
}

void setAdsConnected(bool connected) {
  portENTER_CRITICAL(&inputStatusMux);
  adsConnected = connected;
  portEXIT_CRITICAL(&inputStatusMux);
}

uint16_t liftMux() {
  return Config::UseSingleEndedInputs
             ? static_cast<uint16_t>(ADS1X15_REG_CONFIG_MUX_SINGLE_0 +
                                     Config::LiftSingleEndedChannel)
             : ADS1X15_REG_CONFIG_MUX_DIFF_0_1;
}

uint16_t dragMux() {
  return Config::UseSingleEndedInputs
             ? static_cast<uint16_t>(ADS1X15_REG_CONFIG_MUX_SINGLE_0 +
                                     Config::DragSingleEndedChannel)
             : ADS1X15_REG_CONFIG_MUX_DIFF_2_3;
}

void startConversion(uint16_t mux, ConversionState state) {
  ads.startADCReading(mux, false);
  conversionStartedAt = millis();
  conversionState = state;
}

void conversionTimedOut() {
  setAdsConnected(false);
  setAdsError("ADS1115 conversion timed out");
  readings.lift.valid = false;
  readings.drag.valid = false;
  conversionState = ConversionState::WaitingToStart;
}
}  // namespace

bool initializeLoadCellInputs() {
  Wire.begin(Config::I2cSdaPin, Config::I2cSclPin);
  lastBeginAttemptAt = millis();
  if (!ads.begin(Config::AdsAddress, &Wire)) {
    setAdsConnected(false);
    setAdsError("ADS1115 not detected at I2C address 0x48");
    Serial.println(getAdsError());
    return false;
  }

  ads.setGain(Config::AdsGain);
  ads.setDataRate(Config::AdsDataRate);
  setAdsConnected(true);
  setAdsError("");
  Serial.println("ADS1115 detected; gain range is +/- 0.256 V.");
  Serial.println(Config::UseSingleEndedInputs
                     ? "ADC inputs: single-ended"
                     : "ADC inputs: differential lift A0-A1, drag A2-A3");
  return true;
}

void updateLoadCellInputs() {
  if (!isAdsConnected()) {
    if (millis() - lastBeginAttemptAt >= 5000U) {
      initializeLoadCellInputs();
    }
    return;
  }

  const uint32_t now = millis();
  if (conversionState == ConversionState::WaitingToStart) {
    if (now - lastPairCompletedAt >= Config::InputSamplingIntervalMs) {
      startConversion(liftMux(), ConversionState::WaitingForLift);
    }
    return;
  }

  if (ads.conversionComplete()) {
    const int16_t raw = ads.getLastConversionResults();
    if (conversionState == ConversionState::WaitingForLift) {
      readings.lift = {raw, ads.computeVolts(raw), true};
      startConversion(dragMux(), ConversionState::WaitingForDrag);
    } else {
      readings.drag = {raw, ads.computeVolts(raw), true};
      ++readings.sequence;
      lastPairCompletedAt = now;
      conversionState = ConversionState::WaitingToStart;
      setAdsConnected(true);
      setAdsError("");
    }
    return;
  }

  if (now - conversionStartedAt >= Config::ConversionTimeoutMs) {
    conversionTimedOut();
  }
}

bool isAdsConnected() {
  portENTER_CRITICAL(&inputStatusMux);
  const bool connected = adsConnected;
  portEXIT_CRITICAL(&inputStatusMux);
  return connected;
}

String getAdsError() {
  char copy[sizeof(adsError)];
  portENTER_CRITICAL(&inputStatusMux);
  memcpy(copy, adsError, sizeof(copy));
  portEXIT_CRITICAL(&inputStatusMux);
  copy[sizeof(copy) - 1] = '\0';
  return String(copy);
}

LoadCellReadings getLoadCellReadings() { return readings; }

SensorReading readLiftSensor() { return readings.lift; }

SensorReading readDragSensor() { return readings.drag; }
