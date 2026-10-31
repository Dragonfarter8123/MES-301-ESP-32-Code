#include "Inputs.h"

#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "Config.h"
#include "Calculations.h"

namespace {

Adafruit_ADS1115 ads;
volatile bool adsAvailable = false;
SemaphoreHandle_t adcMutex = nullptr;
uint32_t lastDiagnosticMs = 0;

int16_t readRaw(bool lift) {
  if (Config::ADC_INPUT_MODE == Config::AdcInputMode::SingleEnded) {
    const uint8_t channel = lift ? Config::LIFT_ADC_POSITIVE_CHANNEL
                                 : Config::DRAG_ADC_POSITIVE_CHANNEL;
    return ads.readADC_SingleEnded(channel);
  }

  return lift ? ads.readADC_Differential_0_1()
              : ads.readADC_Differential_2_3();
}

bool readChannel(bool lift, ChannelSample& sample) {
  sample = ChannelSample();
  if (!adcMutex || xSemaphoreTake(adcMutex, portMAX_DELAY) != pdTRUE) {
    sample.error = "ADS1115 read lock unavailable";
    return false;
  }
  if (!adsAvailable) {
    sample.error = "ADS1115 unavailable";
    xSemaphoreGive(adcMutex);
    return false;
  }

  Wire.beginTransmission(Config::ADS1115_ADDRESS);
  if (Wire.endTransmission() != 0) {
    adsAvailable = false;
    sample.error = "ADS1115 I2C communication failed";
    xSemaphoreGive(adcMutex);
    return false;
  }

  sample.raw = readRaw(lift);
  sample.adcVolts = ads.computeVolts(sample.raw);
  if (abs(sample.raw) >= Config::ADC_SATURATION_COUNTS) {
    sample.error = "ADS1115 saturated";
    xSemaphoreGive(adcMutex);
    return false;
  }

  if (Config::ADC_INPUT_MODE == Config::AdcInputMode::SingleEnded &&
      (sample.adcVolts < 0.0f ||
       sample.adcVolts > Config::ADS1115_SUPPLY_VOLTS)) {
    sample.error = "ADC input outside supply range";
    xSemaphoreGive(adcMutex);
    return false;
  }

  sample.adcValid = true;
  sample.error = nullptr;
  xSemaphoreGive(adcMutex);
  return true;
}

}  // namespace

namespace Inputs {

bool begin() {
  Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN);
  adcMutex = xSemaphoreCreateMutex();
  if (!adcMutex) {
    Serial.println("ERROR: Could not create ADS1115 access lock.");
    adsAvailable = false;
    return false;
  }

  Serial.println("\nESP32 + ADS1115 Dual Load Cell Test");
  Serial.println("-----------------------------------");
  adsAvailable = ads.begin(Config::ADS1115_ADDRESS, &Wire);
  if (!adsAvailable) {
    Serial.println("ERROR: ADS1115 not detected.");
    Serial.println("Check VIN, GND, SDA, and SCL.");
    return false;
  }

  ads.setGain(Config::ADS1115_GAIN);
  ads.setDataRate(Config::ADS1115_DATA_RATE);
  Serial.println("ADS1115 detected!");
  if (Config::ADC_INPUT_MODE == Config::AdcInputMode::SingleEnded) {
    Serial.println("Lift AD623 output: A0 | Drag AD623 output: A2");
  } else {
    Serial.println("Lift: A0-A1 | Drag: A2-A3");
  }
  Serial.print("ADS1115 full-scale: +/-");
  Serial.print(fullScaleVolts(), 3);
  Serial.println(" V");
  Serial.println();
  return true;
}

bool available() {
  return adsAvailable;
}

bool readLift(ChannelSample& sample) {
  return readChannel(true, sample);
}

bool readDrag(ChannelSample& sample) {
  return readChannel(false, sample);
}

void printDiagnosticsIfDue() {
  if (millis() - lastDiagnosticMs < Config::DIAGNOSTIC_INTERVAL_MS) {
    return;
  }
  lastDiagnosticMs = millis();

  ChannelSample lift;
  ChannelSample drag;
  Calculations::readLift(lift);
  Calculations::readDrag(drag);

  Serial.print("Lift raw: ");
  Serial.print(lift.raw);
  Serial.print(Config::ADC_INPUT_MODE == Config::AdcInputMode::Differential
                   ? " | Lift differential: "
                   : " | Lift ADC voltage: ");
  if (lift.adcValid) {
    Serial.print(lift.adcVolts, 6);
    Serial.print(" V");
  } else {
    Serial.print(lift.error ? lift.error : "invalid reading");
  }
  Serial.print(" || Drag raw: ");
  Serial.print(drag.raw);
  Serial.print(Config::ADC_INPUT_MODE == Config::AdcInputMode::Differential
                   ? " | Drag differential: "
                   : " | Drag ADC voltage: ");
  if (drag.adcValid) {
    Serial.print(drag.adcVolts, 6);
    Serial.print(" V");
  } else {
    Serial.print(drag.error ? drag.error : "invalid reading");
  }

  Serial.print(" | AD623 input-equivalent (V): ");
  if (lift.adcValid && drag.adcValid &&
      Calculations::amplifierConfigured(true) &&
      Calculations::amplifierConfigured(false)) {
    Serial.print(lift.inputEquivalentVolts, 9);
    Serial.print(" / ");
    Serial.print(drag.inputEquivalentVolts, 9);
  } else {
    Serial.print("not configured or invalid");
  }
  Serial.println();
}

const char* modeName() {
  return Config::ADC_INPUT_MODE == Config::AdcInputMode::SingleEnded
             ? "single-ended"
             : "differential";
}

const char* gainName() {
  switch (Config::ADS1115_GAIN) {
    case GAIN_TWOTHIRDS: return "GAIN_TWOTHIRDS";
    case GAIN_ONE: return "GAIN_ONE";
    case GAIN_TWO: return "GAIN_TWO";
    case GAIN_FOUR: return "GAIN_FOUR";
    case GAIN_EIGHT: return "GAIN_EIGHT";
    case GAIN_SIXTEEN: return "GAIN_SIXTEEN";
  }
  return "unknown";
}

float fullScaleVolts() {
  switch (Config::ADS1115_GAIN) {
    case GAIN_TWOTHIRDS: return 6.144f;
    case GAIN_ONE: return 4.096f;
    case GAIN_TWO: return 2.048f;
    case GAIN_FOUR: return 1.024f;
    case GAIN_EIGHT: return 0.512f;
    case GAIN_SIXTEEN: return 0.256f;
  }
  return 0.256f;
}

}  // namespace Inputs
