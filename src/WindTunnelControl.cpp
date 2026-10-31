#include "WindTunnelControl.h"

#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "Calculations.h"
#include "Config.h"
#include "Inputs.h"

namespace {

bool testRunning = false;
float requestedSpeedMph = 25.0f;
uint32_t requestedDurationSeconds = 30;
uint16_t requestedSampleRateHz = 20;
uint32_t testStartMs = 0;
uint32_t lastSampleMs = 0;
String selectedObject = "NACA 4424";
String selectedTeam = "Team Alpha";
String selectedUnits = "N";
String errorMessage;
ChannelSample currentLift;
ChannelSample currentDrag;
std::vector<MeasurementRecord> records;
SemaphoreHandle_t stateMutex = nullptr;

class StateLock {
 public:
  StateLock()
      : _locked(stateMutex && xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {}
  ~StateLock() {
    if (_locked) {
      xSemaphoreGive(stateMutex);
    }
  }
  bool locked() const { return _locked; }

 private:
  bool _locked;
};

void setFanOutput(float mph) {
  if (!Config::FAN_PWM_ENABLED) {
    return;
  }
  const float boundedSpeed = constrain(mph, 0.0f, 40.0f);
  const uint32_t duty = static_cast<uint32_t>((boundedSpeed / 40.0f) * 255.0f);
  ledcWrite(Config::FAN_PWM_CHANNEL, duty);
}

bool validObject(const String& name) {
  return name == "NACA 4424" || name == "NACA 6412";
}

bool validUnits(const String& unitsValue) {
  return unitsValue == "N" || unitsValue == "lbf" || unitsValue == "g";
}

}  // namespace

namespace WindTunnelControl {

void begin() {
  stateMutex = xSemaphoreCreateMutex();
  if (!stateMutex) {
    Serial.println("ERROR: Could not create wind tunnel state lock.");
    return;
  }
  if (Config::FAN_PWM_ENABLED) {
    pinMode(Config::FAN_PWM_PIN, OUTPUT);
    ledcSetup(Config::FAN_PWM_CHANNEL, Config::FAN_PWM_FREQUENCY_HZ, 8);
    ledcAttachPin(Config::FAN_PWM_PIN, Config::FAN_PWM_CHANNEL);
    setFanOutput(0.0f);
  }
}

bool start(float speedMph, uint32_t durationSeconds, uint16_t sampleRateHz,
           const String& objectName, const String& teamName,
           const String& unitsValue, String& error) {
  if (!Calculations::fullyCalibrated()) {
    error = "Configure both AD623 amplifiers, zero both channels, and calibrate lift and drag before starting a measurement.";
    return false;
  }
  if (!Inputs::available()) {
    error = "ADS1115 is unavailable.";
    return false;
  }
  if (!isfinite(speedMph) || speedMph < 20.0f || speedMph > 40.0f) {
    error = "Wind speed setpoint must be between 20 and 40 mph.";
    return false;
  }
  if (durationSeconds == 0 || durationSeconds > 3600) {
    error = "Measurement duration must be between 1 and 3600 seconds.";
    return false;
  }
  if (sampleRateHz == 0 || sampleRateHz > 100) {
    error = "Sampling rate must be between 1 and 100 Hz.";
    return false;
  }
  if (!validObject(objectName) || !validUnits(unitsValue)) {
    error = "Invalid test object or force units.";
    return false;
  }
  {
    StateLock lock;
    if (!lock.locked()) {
      error = "Wind tunnel state lock unavailable.";
      return false;
    }
    if (testRunning) {
      error = "A test is already running.";
      return false;
    }

    requestedSpeedMph = speedMph;
    requestedDurationSeconds = durationSeconds;
    requestedSampleRateHz = sampleRateHz;
    selectedObject = objectName;
    selectedTeam = teamName;
    selectedUnits = unitsValue;
    errorMessage = "";
    records.clear();
    testStartMs = millis();
    lastSampleMs = testStartMs - (1000U / sampleRateHz);
    testRunning = true;
  }
  setFanOutput(speedMph);
  return true;
}

void stop() {
  {
    StateLock lock;
    if (lock.locked()) {
      testRunning = false;
    }
  }
  setFanOutput(0.0f);
}

void update() {
  const uint32_t now = millis();
  uint32_t startedAt;
  uint32_t durationMs;
  uint32_t intervalMs;
  float speedSetpoint;
  {
    StateLock lock;
    if (!lock.locked() || !testRunning) {
      return;
    }
    startedAt = testStartMs;
    durationMs = requestedDurationSeconds * 1000UL;
    intervalMs = 1000U / requestedSampleRateHz;
    speedSetpoint = requestedSpeedMph;
    if (now - lastSampleMs >= intervalMs) {
      lastSampleMs = now;
    } else {
      return;
    }
  }

  if (now - startedAt >= durationMs) {
    stop();
    return;
  }

  ChannelSample lift;
  ChannelSample drag;
  const bool liftValid = Calculations::readLift(lift);
  const bool dragValid = Calculations::readDrag(drag);
  if (!liftValid || !dragValid) {
    const String error = String("Measurement stopped: ") +
        (!liftValid
             ? (lift.error ? lift.error : "invalid lift reading")
             : (drag.error ? drag.error : "invalid drag reading"));
    {
      StateLock lock;
      if (lock.locked()) {
        errorMessage = error;
      }
    }
    Serial.println(error);
    stop();
    return;
  }

  MeasurementRecord record;
  record.elapsedMs = now - startedAt;
  record.speedSetpointMph = speedSetpoint;
  record.liftNewtons = lift.forceNewtons;
  record.dragNewtons = drag.forceNewtons;
  {
    StateLock lock;
    if (!lock.locked() || !testRunning) {
      return;
    }
    currentLift = lift;
    currentDrag = drag;
    records.push_back(record);
    if (records.size() > 3000) {
      records.erase(records.begin());
    }
  }
}

bool running() {
  StateLock lock;
  return lock.locked() && testRunning;
}

bool controllerEnabled() {
  return Config::FAN_PWM_ENABLED;
}

float speedSetpointMph() {
  StateLock lock;
  return lock.locked() ? requestedSpeedMph : 0.0f;
}

uint32_t durationSeconds() {
  StateLock lock;
  return lock.locked() ? requestedDurationSeconds : 0;
}

uint16_t sampleRateHz() {
  StateLock lock;
  return lock.locked() ? requestedSampleRateHz : 0;
}

String objectName() {
  StateLock lock;
  return lock.locked() ? selectedObject : String();
}

String teamName() {
  StateLock lock;
  return lock.locked() ? selectedTeam : String();
}

String units() {
  StateLock lock;
  return lock.locked() ? selectedUnits : String();
}

String lastError() {
  StateLock lock;
  return lock.locked() ? errorMessage : "Wind tunnel state lock unavailable";
}

bool latestLift(ChannelSample& sample) {
  StateLock lock;
  if (!lock.locked()) {
    return false;
  }
  sample = currentLift;
  return sample.adcValid;
}

bool latestDrag(ChannelSample& sample) {
  StateLock lock;
  if (!lock.locked()) {
    return false;
  }
  sample = currentDrag;
  return sample.adcValid;
}

std::vector<MeasurementRecord> historySnapshot() {
  StateLock lock;
  return lock.locked() ? records : std::vector<MeasurementRecord>();
}

void clearHistory() {
  StateLock lock;
  if (lock.locked()) {
    records.clear();
  }
}

}  // namespace WindTunnelControl
