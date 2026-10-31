#include "Calculations.h"

#include <Preferences.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "Config.h"
#include "Inputs.h"

namespace {

Preferences preferences;
bool preferencesAvailable = false;
bool zeroKnown = false;
float liftZeroInputVolts = 0.0f;
float dragZeroInputVolts = 0.0f;
float liftForceNPerInputVolt = 0.0f;
float dragForceNPerInputVolt = 0.0f;
SemaphoreHandle_t calculationMutex = nullptr;
SemaphoreHandle_t calibrationOperationMutex = nullptr;

class CalculationLock {
 public:
  CalculationLock()
      : _locked(calculationMutex &&
                xSemaphoreTake(calculationMutex, portMAX_DELAY) == pdTRUE) {}
  ~CalculationLock() {
    if (_locked) {
      xSemaphoreGive(calculationMutex);
    }
  }
  bool locked() const { return _locked; }

 private:
  bool _locked;
};

class CalibrationOperationLock {
 public:
  CalibrationOperationLock()
      : _locked(calibrationOperationMutex &&
                xSemaphoreTake(calibrationOperationMutex, portMAX_DELAY) == pdTRUE) {}
  ~CalibrationOperationLock() {
    if (_locked) {
      xSemaphoreGive(calibrationOperationMutex);
    }
  }
  bool locked() const { return _locked; }

 private:
  bool _locked;
};

float gainResistor(bool lift) {
  return lift ? Config::LIFT_AD623_RG_OHMS : Config::DRAG_AD623_RG_OHMS;
}

bool referenceKnown(bool lift) {
  return lift ? Config::LIFT_AD623_REFERENCE_KNOWN
              : Config::DRAG_AD623_REFERENCE_KNOWN;
}

float referenceVolts(bool lift) {
  return lift ? Config::LIFT_AD623_REFERENCE_VOLTS
              : Config::DRAG_AD623_REFERENCE_VOLTS;
}

float negativeInputReferenceVolts(bool lift) {
  return lift ? Config::LIFT_ADC_NEGATIVE_REFERENCE_VOLTS
              : Config::DRAG_ADC_NEGATIVE_REFERENCE_VOLTS;
}

bool closeFloat(float left, float right) {
  return fabsf(left - right) <= 0.000001f;
}

bool matchesHardwareConfiguration() {
  return preferences.getBool("configSet", false) &&
         closeFloat(preferences.getFloat("rgLift", -1.0f), Config::LIFT_AD623_RG_OHMS) &&
         closeFloat(preferences.getFloat("rgDrag", -1.0f), Config::DRAG_AD623_RG_OHMS) &&
         closeFloat(preferences.getFloat("refLift", -1.0f), Config::LIFT_AD623_REFERENCE_VOLTS) &&
         closeFloat(preferences.getFloat("refDrag", -1.0f), Config::DRAG_AD623_REFERENCE_VOLTS) &&
         closeFloat(preferences.getFloat("negRefLift", -1.0f),
                    Config::LIFT_ADC_NEGATIVE_REFERENCE_VOLTS) &&
         closeFloat(preferences.getFloat("negRefDrag", -1.0f),
                    Config::DRAG_ADC_NEGATIVE_REFERENCE_VOLTS) &&
         preferences.getBool("refKnownL", false) == Config::LIFT_AD623_REFERENCE_KNOWN &&
         preferences.getBool("refKnownD", false) == Config::DRAG_AD623_REFERENCE_KNOWN &&
         preferences.getUChar("adcMode", 255) == static_cast<uint8_t>(Config::ADC_INPUT_MODE) &&
         preferences.getUShort("adcGain", 0xFFFF) == static_cast<uint16_t>(Config::ADS1115_GAIN) &&
         preferences.getUChar("liftChannel", 255) == Config::LIFT_ADC_POSITIVE_CHANNEL &&
         preferences.getUChar("dragChannel", 255) == Config::DRAG_ADC_POSITIVE_CHANNEL;
}

bool saveHardwareConfiguration() {
  const bool saved =
      preferences.putFloat("rgLift", Config::LIFT_AD623_RG_OHMS) > 0 &&
      preferences.putFloat("rgDrag", Config::DRAG_AD623_RG_OHMS) > 0 &&
      preferences.putFloat("refLift", Config::LIFT_AD623_REFERENCE_VOLTS) > 0 &&
      preferences.putFloat("refDrag", Config::DRAG_AD623_REFERENCE_VOLTS) > 0 &&
      preferences.putFloat("negRefLift", Config::LIFT_ADC_NEGATIVE_REFERENCE_VOLTS) > 0 &&
      preferences.putFloat("negRefDrag", Config::DRAG_ADC_NEGATIVE_REFERENCE_VOLTS) > 0 &&
      preferences.putBool("refKnownL", Config::LIFT_AD623_REFERENCE_KNOWN) > 0 &&
      preferences.putBool("refKnownD", Config::DRAG_AD623_REFERENCE_KNOWN) > 0 &&
      preferences.putUChar("adcMode", static_cast<uint8_t>(Config::ADC_INPUT_MODE)) > 0 &&
      preferences.putUShort("adcGain", static_cast<uint16_t>(Config::ADS1115_GAIN)) > 0 &&
      preferences.putUChar("liftChannel", Config::LIFT_ADC_POSITIVE_CHANNEL) > 0 &&
      preferences.putUChar("dragChannel", Config::DRAG_ADC_POSITIVE_CHANNEL) > 0 &&
      preferences.putBool("configSet", true) > 0;
  return saved;
}

bool convertInput(bool lift, ChannelSample& sample) {
  const bool readOk = lift ? Inputs::readLift(sample) : Inputs::readDrag(sample);
  if (!readOk) {
    sample.forceValid = false;
    return false;
  }
  if (!Calculations::amplifierConfigured(lift)) {
    sample.forceValid = false;
    sample.error = "AD623 gain resistor/reference not configured";
    return false;
  }

  const float gain = Calculations::amplifierGain(lift);
  const float outputToGround = sample.adcVolts +
      (Config::ADC_INPUT_MODE == Config::AdcInputMode::Differential
           ? negativeInputReferenceVolts(lift)
           : 0.0f);
  sample.inputEquivalentVolts =
      (outputToGround - referenceVolts(lift)) / gain;
  sample.error = nullptr;

  float coefficient;
  float zero;
  bool hasZero;
  {
    CalculationLock lock;
    if (!lock.locked()) {
      sample.forceValid = false;
      sample.error = "calibration state lock unavailable";
      return false;
    }
    coefficient =
        lift ? liftForceNPerInputVolt : dragForceNPerInputVolt;
    zero = lift ? liftZeroInputVolts : dragZeroInputVolts;
    hasZero = zeroKnown;
  }
  if (!hasZero || !isfinite(coefficient) ||
      fabsf(coefficient) < 1.0e-12f) {
    sample.forceValid = false;
    sample.error = "force calibration required";
    return true;
  }

  sample.forceNewtons = (sample.inputEquivalentVolts - zero) * coefficient;
  sample.forceValid = isfinite(sample.forceNewtons);
  if (!sample.forceValid) {
    sample.error = "invalid calibrated force";
  }
  return sample.forceValid;
}

bool persistCalibration() {
  if (!preferencesAvailable) {
    return false;
  }
  const bool liftOk = preferences.putFloat("liftZero", liftZeroInputVolts) > 0;
  const bool dragOk = preferences.putFloat("dragZero", dragZeroInputVolts) > 0;
  const bool liftCoeffOk = preferences.putFloat("liftCoeff", liftForceNPerInputVolt) > 0;
  const bool dragCoeffOk = preferences.putFloat("dragCoeff", dragForceNPerInputVolt) > 0;
  const bool zeroOk = preferences.putBool("zeroKnown", zeroKnown) > 0;
  return liftOk && dragOk && liftCoeffOk && dragCoeffOk && zeroOk;
}

bool computeKnownLoadCoefficient(bool lift, float knownForceN,
                                 float& coefficient, String& error) {
  if (!Calculations::amplifierConfigured(lift)) {
    error = lift ? Calculations::liftConfigurationError()
                 : Calculations::dragConfigurationError();
    return false;
  }
  if (!isfinite(knownForceN) || knownForceN <= 0.0f) {
    error = "Known calibration loads must be positive finite forces in newtons.";
    return false;
  }

  ChannelSample sample;
  if (!convertInput(lift, sample) || !sample.adcValid) {
    error = String(lift ? "Lift: " : "Drag: ") +
        (sample.error ? sample.error : "invalid ADC sample");
    return false;
  }
  float zero;
  bool hasZero;
  {
    CalculationLock lock;
    if (!lock.locked()) {
      error = "Calibration state lock unavailable.";
      return false;
    }
    zero = lift ? liftZeroInputVolts : dragZeroInputVolts;
    hasZero = zeroKnown;
  }
  if (!hasZero) {
    error = "Zero both load cells before applying calibration loads.";
    return false;
  }

  const float deltaVolts = sample.inputEquivalentVolts - zero;
  if (!isfinite(deltaVolts) || fabsf(deltaVolts) < 1.0e-9f) {
    error = String(lift ? "Lift" : "Drag") +
        " calibration signal is too small; verify load and wiring.";
    return false;
  }

  coefficient = knownForceN / deltaVolts;
  return isfinite(coefficient);
}

}  // namespace

namespace Calculations {

bool begin() {
  calculationMutex = xSemaphoreCreateMutex();
  calibrationOperationMutex = xSemaphoreCreateMutex();
  if (!calculationMutex || !calibrationOperationMutex) {
    Serial.println("ERROR: Could not create calibration state locks.");
    return false;
  }
  preferencesAvailable = preferences.begin("windcal", false);
  if (!preferencesAvailable) {
    Serial.println("ERROR: Calibration preferences unavailable; calibration will not persist.");
    return false;
  }

  if (!matchesHardwareConfiguration()) {
    liftForceNPerInputVolt = 0.0f;
    dragForceNPerInputVolt = 0.0f;
    liftZeroInputVolts = 0.0f;
    dragZeroInputVolts = 0.0f;
    zeroKnown = false;
    if (!saveHardwareConfiguration() || !persistCalibration()) {
      Serial.println("ERROR: Could not save initial hardware calibration settings.");
      return false;
    }
    Serial.println("Hardware configuration changed or no calibration saved; calibration reset.");
    return true;
  }

  liftForceNPerInputVolt = preferences.getFloat("liftCoeff", 0.0f);
  dragForceNPerInputVolt = preferences.getFloat("dragCoeff", 0.0f);
  liftZeroInputVolts = preferences.getFloat("liftZero", 0.0f);
  dragZeroInputVolts = preferences.getFloat("dragZero", 0.0f);
  zeroKnown = preferences.getBool("zeroKnown", false);
  return true;
}

bool amplifierConfigured(bool lift) {
  const float resistor = gainResistor(lift);
  return isfinite(resistor) && resistor > 0.0f && referenceKnown(lift) &&
         isfinite(referenceVolts(lift));
}

float amplifierGain(bool lift) {
  const float resistor = gainResistor(lift);
  if (!isfinite(resistor) || resistor <= 0.0f) {
    return 0.0f;
  }
  return 1.0f + (100000.0f / resistor);
}

bool zeroSensors(String& error) {
  CalibrationOperationLock operationLock;
  if (!operationLock.locked()) {
    error = "Calibration operation lock unavailable.";
    return false;
  }
  ChannelSample lift;
  ChannelSample drag;
  if (!amplifierConfigured(true) || !amplifierConfigured(false)) {
    error = "Enter both AD623 gain resistors and verified REF voltages in include/Config.h.";
    return false;
  }
  if (!Inputs::readLift(lift) || !Inputs::readDrag(drag)) {
    error = String("Cannot zero sensors: ") +
        (lift.error ? lift.error : (drag.error ? drag.error : "invalid ADC reading"));
    return false;
  }

  const float newLiftZeroInputVolts =
      (lift.adcVolts +
       (Config::ADC_INPUT_MODE == Config::AdcInputMode::Differential
            ? negativeInputReferenceVolts(true)
            : 0.0f) -
       referenceVolts(true)) /
      amplifierGain(true);
  const float newDragZeroInputVolts =
      (drag.adcVolts +
       (Config::ADC_INPUT_MODE == Config::AdcInputMode::Differential
            ? negativeInputReferenceVolts(false)
            : 0.0f) -
       referenceVolts(false)) /
      amplifierGain(false);
  {
    CalculationLock lock;
    if (!lock.locked()) {
      error = "Calibration state lock unavailable.";
      return false;
    }
    liftZeroInputVolts = newLiftZeroInputVolts;
    dragZeroInputVolts = newDragZeroInputVolts;
    zeroKnown = true;
    if (!persistCalibration()) {
      error = "Zero offsets were captured but could not be saved to ESP32 preferences.";
      return false;
    }
  }
  return true;
}

bool calibrate(bool setLift, float liftKnownForceN,
               bool setDrag, float dragKnownForceN, String& error) {
  CalibrationOperationLock operationLock;
  if (!operationLock.locked()) {
    error = "Calibration operation lock unavailable.";
    return false;
  }
  if (!setLift && !setDrag) {
    error = "Provide at least one known load in newtons.";
    return false;
  }

  float newLiftCoefficient;
  float newDragCoefficient;
  {
    CalculationLock lock;
    if (!lock.locked()) {
      error = "Calibration state lock unavailable.";
      return false;
    }
    newLiftCoefficient = liftForceNPerInputVolt;
    newDragCoefficient = dragForceNPerInputVolt;
  }
  if (setLift &&
      !computeKnownLoadCoefficient(true, liftKnownForceN,
                                   newLiftCoefficient, error)) {
    return false;
  }
  if (setDrag &&
      !computeKnownLoadCoefficient(false, dragKnownForceN,
                                   newDragCoefficient, error)) {
    return false;
  }

  {
    CalculationLock lock;
    if (!lock.locked()) {
      error = "Calibration state lock unavailable.";
      return false;
    }
    liftForceNPerInputVolt = newLiftCoefficient;
    dragForceNPerInputVolt = newDragCoefficient;
    if (!persistCalibration()) {
      error = "Calibration calculated but could not be saved to ESP32 preferences.";
      return false;
    }
  }
  return true;
}

bool readLift(ChannelSample& sample) {
  return convertInput(true, sample);
}

bool readDrag(ChannelSample& sample) {
  return convertInput(false, sample);
}

bool liftCalibrated() {
  CalculationLock lock;
  if (!lock.locked()) {
    return false;
  }
  return amplifierConfigured(true) && zeroKnown &&
         isfinite(liftForceNPerInputVolt) &&
         fabsf(liftForceNPerInputVolt) > 1.0e-12f;
}

bool dragCalibrated() {
  CalculationLock lock;
  if (!lock.locked()) {
    return false;
  }
  return amplifierConfigured(false) && zeroKnown &&
         isfinite(dragForceNPerInputVolt) &&
         fabsf(dragForceNPerInputVolt) > 1.0e-12f;
}

bool fullyCalibrated() {
  return liftCalibrated() && dragCalibrated();
}

bool zeroed() {
  CalculationLock lock;
  if (!lock.locked()) {
    return false;
  }
  return zeroKnown;
}

float liftForceCoefficient() {
  CalculationLock lock;
  return lock.locked() ? liftForceNPerInputVolt : 0.0f;
}

float dragForceCoefficient() {
  CalculationLock lock;
  return lock.locked() ? dragForceNPerInputVolt : 0.0f;
}

const char* liftConfigurationError() {
  if (!isfinite(Config::LIFT_AD623_RG_OHMS) ||
      Config::LIFT_AD623_RG_OHMS <= 0.0f) {
    return "Lift AD623 RG is not configured";
  }
  if (!Config::LIFT_AD623_REFERENCE_KNOWN) {
    return "Lift AD623 REF voltage is not configured";
  }
  return "Lift amplifier configuration invalid";
}

const char* dragConfigurationError() {
  if (!isfinite(Config::DRAG_AD623_RG_OHMS) ||
      Config::DRAG_AD623_RG_OHMS <= 0.0f) {
    return "Drag AD623 RG is not configured";
  }
  if (!Config::DRAG_AD623_REFERENCE_KNOWN) {
    return "Drag AD623 REF voltage is not configured";
  }
  return "Drag amplifier configuration invalid";
}

}  // namespace Calculations
