#pragma once

#include <Adafruit_ADS1X15.h>
#include <stdint.h>

namespace Config {

static const char* const WIFI_SSID = "";
static const char* const WIFI_PASSWORD = "";
static const char* const FALLBACK_AP_SSID = "WindTunnel-ESP32";
static const char* const FALLBACK_AP_PASSWORD = "WindTunnel301";
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;

constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t ADS1115_ADDRESS = 0x48;
constexpr adsGain_t ADS1115_GAIN = GAIN_SIXTEEN;

enum class AdcInputMode : uint8_t {
  SingleEnded,
  Differential
};

// Single-ended: AD623 outputs to A0/A2. Differential: signal/reference pairs A0-A1/A2-A3.
constexpr AdcInputMode ADC_INPUT_MODE = AdcInputMode::SingleEnded;
constexpr uint8_t LIFT_ADC_POSITIVE_CHANNEL = 0;
constexpr uint8_t DRAG_ADC_POSITIVE_CHANNEL = 2;
constexpr float ADS1115_SUPPLY_VOLTS = 3.3f;
constexpr uint16_t ADS1115_DATA_RATE = RATE_ADS1115_860SPS;
constexpr int16_t ADC_SATURATION_COUNTS = 32760;
constexpr uint32_t DIAGNOSTIC_INTERVAL_MS = 1000;

// Enter the installed gain resistors. Zero deliberately means "not configured".
constexpr float LIFT_AD623_RG_OHMS = 0.0f;
constexpr float DRAG_AD623_RG_OHMS = 0.0f;

// Enter each measured AD623 REF voltage and mark it known after wiring is verified.
constexpr bool LIFT_AD623_REFERENCE_KNOWN = false;
constexpr bool DRAG_AD623_REFERENCE_KNOWN = false;
constexpr float LIFT_AD623_REFERENCE_VOLTS = 0.0f;
constexpr float DRAG_AD623_REFERENCE_VOLTS = 0.0f;

// In differential mode, enter the voltage on each ADS1115 negative input.
constexpr float LIFT_ADC_NEGATIVE_REFERENCE_VOLTS = 0.0f;
constexpr float DRAG_ADC_NEGATIVE_REFERENCE_VOLTS = 0.0f;

// A safe MOSFET/driver interface and its airflow calibration must be confirmed first.
constexpr bool FAN_PWM_ENABLED = false;
constexpr int FAN_PWM_PIN = 25;
constexpr uint8_t FAN_PWM_CHANNEL = 0;
constexpr uint32_t FAN_PWM_FREQUENCY_HZ = 20000;

}  // namespace Config
