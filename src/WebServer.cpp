#include "WebServer.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <memory>
#include <vector>

#include "Calculations.h"
#include "Config.h"
#include "ESP10_Inputs.h"
#include "WindTunnelControl.h"

namespace {
struct TestSettings {
  float startSpeedMph = 25.0f;
  float endSpeedMph = 25.0f;
  float speedIncrementMph = 0.5f;
  float setpointMph = 25.0f;
  uint32_t stabilizationSeconds = 10;
  float steadyStateErrorNewtons = 0.05f;
  uint16_t movingAverageWindowMs = 1000;
  uint32_t steadyStateHoldSeconds = 2;
  uint16_t sampleRateHz = 20;
  String team;
  String airfoil = "NACA 4424";
  String units = "N";
};

struct CalibrationPoint {
  float weightGrams;
  float liftVolts;
  float dragVolts;
};

struct TeamRecord {
  String name;
  std::vector<CalibrationPoint> calibration;
};

struct TestSample {
  uint32_t elapsedMs;
  float setpointMph;
  float liftRaw;
  float dragRaw;
  float liftVolts;
  float dragVolts;
  float liftNewtons;
  float dragNewtons;
  bool calibrationLoaded;
};

struct CsvStreamState {
  std::vector<TestSample> samples;
  String pending;
  size_t nextSample = 0;
};

constexpr size_t MaxTeams = 20;
constexpr size_t MaxTeamNameLength = 32;
constexpr size_t MaxCalibrationPoints = 51;
constexpr size_t MaxMovingAverageSamples = 500;
constexpr char StationDataPath[] = "/station.json";

AsyncWebServer server(80);
AsyncWebSocket measurementSocket("/api/ws");
SemaphoreHandle_t stateMutex = nullptr;
TestSettings settings;
std::vector<TeamRecord> teams;
String currentTeam;
std::vector<TestSample> testSamples;
bool testRunning = false;
uint32_t testStartedAt = 0;
uint32_t speedStepStartedAt = 0;
uint32_t lastSampleAt = 0;
uint32_t lastLoggedSequence = 0;
uint32_t testStartSequence = 0;
uint32_t lastWifiAttemptAt = 0;
bool wifiAddressPrinted = false;
bool spiffsMounted = false;
String pendingMeasurementBatch;
uint8_t pendingMeasurementCount = 0;
uint32_t measurementBatchStartedAt = 0;
float liftStabilityValues[MaxMovingAverageSamples] = {};
float dragStabilityValues[MaxMovingAverageSamples] = {};
size_t stabilityNext = 0;
size_t stabilityCount = 0;
float liftStabilitySum = 0.0f;
float dragStabilitySum = 0.0f;
uint32_t steadyStateStartedAt = 0;
bool steadyStateTimerRunning = false;
bool steadyStateAchieved = false;
constexpr uint8_t MaxMeasurementBatchSize = 10;
constexpr uint32_t MeasurementBatchIntervalMs = 250;

void resetSteadyStateTracking() {
  memset(liftStabilityValues, 0, sizeof(liftStabilityValues));
  memset(dragStabilityValues, 0, sizeof(dragStabilityValues));
  stabilityNext = 0;
  stabilityCount = 0;
  liftStabilitySum = 0.0f;
  dragStabilitySum = 0.0f;
  steadyStateStartedAt = 0;
  steadyStateTimerRunning = false;
  steadyStateAchieved = false;
}

bool updateSteadyState(float lift, float drag, const TestSettings &current,
                       uint32_t now) {
  const size_t windowSamples = min(
      MaxMovingAverageSamples,
      max(static_cast<size_t>(1),
          (static_cast<size_t>(current.movingAverageWindowMs) *
               current.sampleRateHz +
           999) /
              1000));
  if (stabilityCount == windowSamples) {
    liftStabilitySum -= liftStabilityValues[stabilityNext];
    dragStabilitySum -= dragStabilityValues[stabilityNext];
  } else {
    ++stabilityCount;
  }
  liftStabilityValues[stabilityNext] = lift;
  dragStabilityValues[stabilityNext] = drag;
  liftStabilitySum += lift;
  dragStabilitySum += drag;
  stabilityNext = (stabilityNext + 1) % windowSamples;

  if (stabilityCount < windowSamples) {
    steadyStateTimerRunning = false;
    return false;
  }
  const float liftAverage = liftStabilitySum / stabilityCount;
  const float dragAverage = dragStabilitySum / stabilityCount;
  const bool withinError =
      fabsf(lift - liftAverage) <= current.steadyStateErrorNewtons &&
      fabsf(drag - dragAverage) <= current.steadyStateErrorNewtons;
  if (!withinError) {
    steadyStateTimerRunning = false;
    steadyStateStartedAt = 0;
    steadyStateAchieved = false;
    return false;
  }
  if (!steadyStateTimerRunning) {
    steadyStateTimerRunning = true;
    steadyStateStartedAt = now;
  }
  steadyStateAchieved =
      now - steadyStateStartedAt >= current.steadyStateHoldSeconds * 1000U;
  return steadyStateAchieved;
}

TeamRecord *findTeam(const String &name) {
  for (TeamRecord &team : teams) {
    if (team.name == name) return &team;
  }
  return nullptr;
}

bool validCalibration(const std::vector<CalibrationPoint> &points) {
  if (points.size() < 2 || points.size() > MaxCalibrationPoints ||
      points.front().weightGrams > 0.0f ||
      points.back().weightGrams < 0.0f) {
    return false;
  }
  bool liftIncreasing = points[1].liftVolts > points[0].liftVolts;
  bool dragIncreasing = points[1].dragVolts > points[0].dragVolts;
  for (size_t i = 0; i < points.size(); ++i) {
    const CalibrationPoint &point = points[i];
    if (!isfinite(point.weightGrams) || !isfinite(point.liftVolts) ||
        !isfinite(point.dragVolts) || fabsf(point.liftVolts) > 10.0f ||
        fabsf(point.dragVolts) > 10.0f) {
      return false;
    }
    if (i > 0 &&
        (point.weightGrams <= points[i - 1].weightGrams ||
         (liftIncreasing ? point.liftVolts <= points[i - 1].liftVolts
                         : point.liftVolts >= points[i - 1].liftVolts) ||
         (dragIncreasing ? point.dragVolts <= points[i - 1].dragVolts
                         : point.dragVolts >= points[i - 1].dragVolts))) {
      return false;
    }
  }
  return true;
}

bool writeStationData(const std::vector<TeamRecord> &records,
                      const String &selectedTeam) {
  JsonDocument document;
  document["currentTeam"] = selectedTeam;
  JsonArray teamArray = document["teams"].to<JsonArray>();
  for (const TeamRecord &team : records) {
    JsonObject item = teamArray.add<JsonObject>();
    item["name"] = team.name;
    JsonArray points = item["calibration"].to<JsonArray>();
    for (const CalibrationPoint &point : team.calibration) {
      JsonObject entry = points.add<JsonObject>();
      entry["weightGrams"] = point.weightGrams;
      entry["liftVolts"] = point.liftVolts;
      entry["dragVolts"] = point.dragVolts;
    }
  }

  File file = SPIFFS.open("/station.tmp", FILE_WRITE);
  if (!file) return false;
  const size_t expected = measureJson(document);
  const size_t written = serializeJson(document, file);
  file.close();
  if (written != expected) {
    SPIFFS.remove("/station.tmp");
    return false;
  }
  const bool hadPrevious = SPIFFS.exists(StationDataPath);
  if (SPIFFS.exists("/station.bak")) {
    SPIFFS.remove("/station.bak");
  }
  if (hadPrevious && !SPIFFS.rename(StationDataPath, "/station.bak")) {
    SPIFFS.remove("/station.tmp");
    return false;
  }
  if (!SPIFFS.rename("/station.tmp", StationDataPath)) {
    if (hadPrevious) SPIFFS.rename("/station.bak", StationDataPath);
    SPIFFS.remove("/station.tmp");
    return false;
  }
  if (SPIFFS.exists("/station.bak")) {
    SPIFFS.remove("/station.bak");
  }
  return true;
}

void loadStationData() {
  if (!SPIFFS.exists(StationDataPath) && SPIFFS.exists("/station.bak")) {
    SPIFFS.rename("/station.bak", StationDataPath);
  }
  if (!SPIFFS.exists(StationDataPath)) return;
  File file = SPIFFS.open(StationDataPath, FILE_READ);
  if (!file) {
    Serial.println("ERROR: Could not read team/calibration data from SPIFFS.");
    return;
  }
  JsonDocument document;
  const DeserializationError error = deserializeJson(document, file);
  file.close();
  if (error || !document["teams"].is<JsonArray>()) {
    Serial.println("ERROR: Team/calibration data is invalid; ignoring it.");
    return;
  }

  std::vector<TeamRecord> loaded;
  for (JsonObjectConst item : document["teams"].as<JsonArrayConst>()) {
    const char *name = item["name"] | "";
    if (name[0] == '\0' || strlen(name) > MaxTeamNameLength ||
        loaded.size() >= MaxTeams) {
      continue;
    }
    TeamRecord team;
    team.name = name;
    for (JsonObjectConst point : item["calibration"].as<JsonArrayConst>()) {
      team.calibration.push_back(
          {point["weightGrams"] | NAN, point["liftVolts"] | NAN,
           point["dragVolts"] | NAN});
    }
    if (!team.calibration.empty() && !validCalibration(team.calibration)) {
      team.calibration.clear();
      Serial.println("ERROR: Invalid saved calibration ignored for a team.");
    }
    loaded.push_back(team);
  }
  teams = loaded;
  const char *selected = document["currentTeam"] | "";
  if (findTeam(String(selected)) != nullptr) currentTeam = selected;
}

float interpolateWeightAtVoltage(const std::vector<CalibrationPoint> &points,
                                 float voltage, bool lift) {
  if (!isfinite(voltage)) return NAN;

  const float firstVoltage =
      lift ? points.front().liftVolts : points.front().dragVolts;
  const float lastVoltage =
      lift ? points.back().liftVolts : points.back().dragVolts;
  if (voltage < min(firstVoltage, lastVoltage) ||
      voltage > max(firstVoltage, lastVoltage)) {
    return NAN;
  }

  for (size_t i = 1; i < points.size(); ++i) {
    const float lower = lift ? points[i - 1].liftVolts : points[i - 1].dragVolts;
    const float upper = lift ? points[i].liftVolts : points[i].dragVolts;
    if ((voltage >= min(lower, upper)) && (voltage <= max(lower, upper))) {
      const float fraction = (voltage - lower) / (upper - lower);
      return points[i - 1].weightGrams +
             fraction * (points[i].weightGrams -
                         points[i - 1].weightGrams);
    }
  }
  return NAN;
}

float interpolateZeroVoltage(const std::vector<CalibrationPoint> &points,
                             bool lift) {
  const float firstWeight = points.front().weightGrams;
  size_t segment = 1;
  while (segment < points.size() - 1 && points[segment].weightGrams < 0.0f) {
    ++segment;
  }
  if (firstWeight > 0.0f) segment = 1;
  else if (points.back().weightGrams < 0.0f) segment = points.size() - 1;

  const CalibrationPoint &a = points[segment - 1];
  const CalibrationPoint &b = points[segment];
  const float aVoltage = lift ? a.liftVolts : a.dragVolts;
  const float bVoltage = lift ? b.liftVolts : b.dragVolts;
  const float fraction = -a.weightGrams / (b.weightGrams - a.weightGrams);
  return aVoltage + fraction * (bVoltage - aVoltage);
}

float calibratedForceNewtons(const TeamRecord *team, float voltage,
                             bool lift) {
  if (team == nullptr || !validCalibration(team->calibration)) return NAN;
  const float grams = interpolateWeightAtVoltage(team->calibration, voltage, lift);
  const float zeroVoltage = interpolateZeroVoltage(team->calibration, lift);
  const float zeroGrams =
      interpolateWeightAtVoltage(team->calibration, zeroVoltage, lift);
  if (!isfinite(grams) || !isfinite(zeroGrams)) return NAN;
  return (grams - zeroGrams) * 0.00980665f;
}

bool lockState() {
  return stateMutex != nullptr &&
         xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE;
}

void unlockState() { xSemaphoreGive(stateMutex); }

void sendJson(AsyncWebServerRequest *request, int status,
              JsonDocument &document) {
  String body;
  serializeJson(document, body);
  request->send(status, "application/json", body);
}

void sendError(AsyncWebServerRequest *request, int status,
               const char *message) {
  JsonDocument response;
  response["error"] = message;
  sendJson(request, status, response);
}

void addTunnelStatus(JsonDocument &response) {
  const WindTunnelControlStatus status = getWindTunnelControlStatus();
  char dacAddress[7];
  snprintf(dacAddress, sizeof(dacAddress), "0x%02X",
           Config::DacI2cAddress);
  response["dacType"] = "DFRobot DFR0971";
  response["dacAddress"] = dacAddress;
  response["dacChannel"] = Config::DacOutputChannel;
  response["dacBenchTestEnabled"] =
      Config::DacBenchTestEnabled && !Config::TunnelHardwareControlEnabled;
  response["targetSpeedMph"] = status.targetSpeedMph;
  response["calibratedSpeedMph"] = status.targetSpeedMph;
  if (status.calculationAvailable) {
    response["estimatedFanRPM"] = status.estimatedFanRpm;
    response["calculatedVoltage"] = status.calculatedVoltage;
  } else {
    response["estimatedFanRPM"] = nullptr;
    response["calculatedVoltage"] = nullptr;
  }
  response["outputEnabled"] = status.outputEnabled;
  response["dacConnected"] = status.dacConnected;
  if (isfinite(status.commandedDacVoltage)) {
    response["commandedVoltage"] = status.commandedDacVoltage;
  } else {
    response["commandedVoltage"] = nullptr;
  }
  response["commandAccepted"] = status.commandAccepted;
  response["outputVoltageVerified"] = status.outputVoltageVerified;
  response["benchTestActive"] = status.benchTestActive;
  response["running"] = status.running;
  response["tunnelRunning"] = status.running;
  response["stabilizationElapsedMs"] = status.stabilizationElapsedMs;
  response["stabilizationPeriodMs"] = Config::TunnelStabilizationPeriodMs;
  response["stabilizationTimerComplete"] =
      status.stabilizationTimerComplete;
  response["steadyStateVerified"] = status.steadyStateVerified;
  if (status.benchTestActive) {
    response["controlMode"] = "bench-test";
  } else if (Config::TunnelHardwareControlEnabled &&
             Config::TunnelSafetyPrerequisitesVerified &&
             Config::TunnelSafeStopVerified) {
    response["controlMode"] = "hardware";
  } else {
    response["controlMode"] = "simulation";
  }
  response["message"] = status.outputEnabled
                            ? "DAC command issued; physical output voltage has not been measured."
                            : "No physical wind-tunnel command is active; windspeed is shown as the calibrated estimate.";
}

bool validStartSettings(JsonDocument &document, TestSettings &result) {
  const char *team = document["team"] | "";
  const char *airfoil = document["object"] | "";
  const char *units = document["units"] | "N";
  const float startSpeed = document["startSpeed"] | -1.0f;
  const float endSpeed = document["endSpeed"] | -1.0f;
  const float speedIncrement = document["speedIncrement"] | -1.0f;
  const float stabilizationInput = document["stabilizationSeconds"] | 0.0f;
  const float steadyErrorInput = document["steadyStateErrorNewtons"] | NAN;
  const float movingAverageWindowInput =
      document["movingAverageWindowMs"] | NAN;
  const float steadyStateHoldInput =
      document["steadyStateHoldSeconds"] | NAN;
  const float sampleRateInput = document["sampleRate"] | 0.0f;

  if (team[0] == '\0' || strlen(team) > MaxTeamNameLength ||
      !isfinite(startSpeed) || !isfinite(endSpeed) ||
      !isfinite(speedIncrement) || startSpeed < Config::MinTunnelSpeedMph ||
      startSpeed > Config::MaxTunnelSpeedMph ||
      endSpeed < Config::MinTunnelEndSpeedMph ||
      endSpeed > Config::MaxTunnelSpeedMph || speedIncrement <= 0.0f ||
      fabsf(startSpeed / Config::TunnelSpeedStepMph -
            roundf(startSpeed / Config::TunnelSpeedStepMph)) > 0.001f ||
      fabsf(endSpeed / Config::TunnelSpeedStepMph -
            roundf(endSpeed / Config::TunnelSpeedStepMph)) > 0.001f ||
      fabsf(speedIncrement / Config::TunnelSpeedStepMph -
            roundf(speedIncrement / Config::TunnelSpeedStepMph)) > 0.001f) {
    return false;
  }
  const float span = fabsf(endSpeed - startSpeed);
  const float stepCount = span / speedIncrement;
  if (stepCount > 40.0f ||
      fabsf(stepCount - roundf(stepCount)) > 0.001f) {
    return false;
  }
  if (strcmp(airfoil, "NACA 4424") != 0 &&
      strcmp(airfoil, "NACA 6412") != 0) {
    return false;
  }
  if (strcmp(units, "N") != 0 && strcmp(units, "lbf") != 0 &&
      strcmp(units, "g") != 0) {
    return false;
  }
  if (!isfinite(stabilizationInput) || stabilizationInput < 1.0f ||
      stabilizationInput > Config::MaxTestDurationSeconds ||
      floorf(stabilizationInput) != stabilizationInput ||
      (roundf(stepCount) + 1.0f) * stabilizationInput >
          Config::MaxTestDurationSeconds) {
    return false;
  }
  if (!isfinite(steadyErrorInput) || steadyErrorInput <= 0.0f ||
      steadyErrorInput > 100.0f ||
      !isfinite(movingAverageWindowInput) ||
      movingAverageWindowInput < 100.0f ||
      movingAverageWindowInput > 10000.0f ||
      floorf(movingAverageWindowInput) != movingAverageWindowInput ||
      !isfinite(steadyStateHoldInput) || steadyStateHoldInput < 1.0f ||
      steadyStateHoldInput > stabilizationInput ||
      floorf(steadyStateHoldInput) != steadyStateHoldInput) {
    return false;
  }
  if (!isfinite(sampleRateInput) || sampleRateInput < 1.0f ||
      sampleRateInput > Config::MaxLoggingSampleRateHz ||
      floorf(sampleRateInput) != sampleRateInput) {
    return false;
  }

  result.startSpeedMph = startSpeed;
  result.endSpeedMph = endSpeed;
  result.speedIncrementMph = speedIncrement;
  result.setpointMph = startSpeed;
  result.stabilizationSeconds = static_cast<uint32_t>(stabilizationInput);
  result.steadyStateErrorNewtons = steadyErrorInput;
  result.movingAverageWindowMs =
      static_cast<uint16_t>(movingAverageWindowInput);
  result.steadyStateHoldSeconds =
      static_cast<uint32_t>(steadyStateHoldInput);
  result.sampleRateHz = static_cast<uint16_t>(sampleRateInput);
  result.team = team;
  result.airfoil = airfoil;
  result.units = units;
  return true;
}

void handleStart(AsyncWebServerRequest *request, uint8_t *body, size_t length,
                 size_t index, size_t total) {
  if (index != 0 || length != total || total > 1024) {
    sendError(request, 400, "Invalid or oversized request body");
    return;
  }

  JsonDocument document;
  if (deserializeJson(document, body, length)) {
    sendError(request, 400, "Request body must be valid JSON");
    return;
  }

  TestSettings requested;
  if (!validStartSettings(document, requested)) {
    sendError(request, 400, "Invalid test settings");
    return;
  }
  if (!isAdsConnected()) {
    sendError(request, 503, "ADS1115 is not available");
    return;
  }
  if (!lockState()) {
    sendError(request, 503, "Test state is unavailable");
    return;
  }
  if (testRunning) {
    unlockState();
    sendError(request, 409, "A test is already running");
    return;
  }
  if (findTeam(requested.team) == nullptr || currentTeam != requested.team) {
    unlockState();
    sendError(request, 400, "Select an existing team before starting a test");
    return;
  }

  const char *controlError = nullptr;
  if (!setWindTunnelTargetSpeed(requested.setpointMph, &controlError) ||
      !startWindTunnelControl(&controlError)) {
    unlockState();
    sendError(request, 400,
              controlError != nullptr ? controlError
                                      : "Wind tunnel control could not start.");
    return;
  }

  settings = requested;
  testSamples.clear();
  resetSteadyStateTracking();
  testRunning = true;
  testStartedAt = millis();
  speedStepStartedAt = testStartedAt;
  lastSampleAt = testStartedAt - (1000U / settings.sampleRateHz);
  testStartSequence = getMeasurementValues().sequence;
  lastLoggedSequence = testStartSequence;
  unlockState();

  JsonDocument response;
  response["running"] = true;
  response["stabilizationSeconds"] = requested.stabilizationSeconds;
  response["setpoint"] = requested.setpointMph;
  addTunnelStatus(response);
  sendJson(request, 200, response);
}

void handleStop(AsyncWebServerRequest *request) {
  if (!lockState()) {
    sendError(request, 503, "Test state is unavailable");
    return;
  }
  testRunning = false;
  unlockState();
  const char *error = nullptr;
  const bool stopped = stopWindTunnelControl(&error);

  JsonDocument response;
  response["running"] = false;
  addTunnelStatus(response);
  if (!stopped) {
    response["error"] = error != nullptr ? error : "Tunnel safe-stop failed.";
  }
  sendJson(request, stopped ? 200 : 503, response);
}

void handleTunnelSetSpeed(AsyncWebServerRequest *request, uint8_t *body,
                          size_t length, size_t index, size_t total) {
  if (index != 0 || length != total || total > 1024) {
    sendError(request, 400, "Invalid or oversized request body");
    return;
  }

  JsonDocument document;
  if (deserializeJson(document, body, length)) {
    sendError(request, 400, "Request body must be valid JSON");
    return;
  }

  const float mph = document["targetSpeedMph"] | -1.0f;
  const char *error = nullptr;
  if (!setWindTunnelTargetSpeed(mph, &error)) {
    sendError(request, 400, error != nullptr ? error : "Invalid target speed.");
    return;
  }

  JsonDocument response;
  addTunnelStatus(response);
  sendJson(request, 200, response);
}

void handleTunnelStart(AsyncWebServerRequest *request) {
  const char *error = nullptr;
  if (!startWindTunnelControl(&error)) {
    sendError(request, 409,
              error != nullptr ? error : "Wind tunnel control could not start.");
    return;
  }

  JsonDocument response;
  addTunnelStatus(response);
  sendJson(request, 200, response);
}

void handleTunnelStop(AsyncWebServerRequest *request) {
  if (!lockState()) {
    sendError(request, 503, "Test state is unavailable");
    return;
  }
  testRunning = false;
  unlockState();
  const char *error = nullptr;
  const bool stopped = stopWindTunnelControl(&error);

  JsonDocument response;
  addTunnelStatus(response);
  if (!stopped) {
    response["error"] = error != nullptr ? error : "Tunnel safe-stop failed.";
  }
  sendJson(request, stopped ? 200 : 503, response);
}

void handleDacBenchOutput(AsyncWebServerRequest *request, uint8_t *body,
                          size_t length, size_t index, size_t total) {
  if (!Config::DacBenchTestEnabled) {
    sendError(request, 403, "DAC bench-test mode is disabled in Config.h.");
    return;
  }
  if (index != 0 || length != total || total > 1024) {
    sendError(request, 400, "Invalid or oversized request body");
    return;
  }

  JsonDocument document;
  if (deserializeJson(document, body, length)) {
    sendError(request, 400, "Request body must be valid JSON");
    return;
  }

  const float voltage = document["voltage"] | -1.0f;
  const char *error = nullptr;
  if (!setDACBenchTestVoltage(voltage, &error)) {
    sendError(request, 409,
              error != nullptr ? error : "DAC bench command was rejected.");
    return;
  }
  JsonDocument response;
  addTunnelStatus(response);
  sendJson(request, 200, response);
}

void handleDacBenchStop(AsyncWebServerRequest *request) {
  const char *error = nullptr;
  if (!stopDACBenchTest(&error)) {
    sendError(request, 409,
              error != nullptr ? error : "DAC bench stop command failed.");
    return;
  }
  JsonDocument response;
  addTunnelStatus(response);
  sendJson(request, 200, response);
}

void handleTunnelStatus(AsyncWebServerRequest *request) {
  JsonDocument response;
  addTunnelStatus(response);
  sendJson(request, 200, response);
}

void handleCalibration(AsyncWebServerRequest *request) {
  if (!lockState()) {
    sendError(request, 503, "Calibration state is unavailable");
    return;
  }
  const TeamRecord *team = findTeam(currentTeam);
  JsonDocument response;
  response["team"] = currentTeam;
  response["calibrationLoaded"] =
      team != nullptr && validCalibration(team->calibration);
  if (team != nullptr) {
    JsonArray points = response["points"].to<JsonArray>();
    for (const CalibrationPoint &point : team->calibration) {
      JsonObject item = points.add<JsonObject>();
      item["weightGrams"] = point.weightGrams;
      item["liftVolts"] = point.liftVolts;
      item["dragVolts"] = point.dragVolts;
    }
  }
  unlockState();
  sendJson(request, 200, response);
}

void handleSaveCalibration(AsyncWebServerRequest *request, uint8_t *body,
                           size_t length, size_t index, size_t total) {
  if (index != 0 || length != total || total > 8192) {
    sendError(request, 400, "Invalid or oversized request body");
    return;
  }
  JsonDocument document;
  if (deserializeJson(document, body, length)) {
    sendError(request, 400, "Request body must be valid JSON");
    return;
  }

  const float startWeight = document["startWeightGrams"] | NAN;
  const float endWeight = document["endWeightGrams"] | NAN;
  const float increment = document["incrementGrams"] | NAN;
  JsonArrayConst entries = document["points"].as<JsonArrayConst>();
  if (!isfinite(startWeight) || !isfinite(endWeight) ||
      !isfinite(increment) || startWeight >= endWeight || increment <= 0.0f ||
      startWeight > 0.0f || endWeight < 0.0f || entries.isNull()) {
    sendError(request, 400,
              "Calibration weights must be valid and bracket zero weight");
    return;
  }
  if (entries.size() < 2 || entries.size() > MaxCalibrationPoints) {
    sendError(request, 400,
              "Calibration table must contain 2 to 51 weight points");
    return;
  }

  std::vector<CalibrationPoint> points;
  points.reserve(entries.size());
  for (JsonObjectConst entry : entries) {
    const float weight = entry["weightGrams"] | NAN;
    const float liftVolts = entry["liftVolts"] | NAN;
    const float dragVolts = entry["dragVolts"] | NAN;
    if (!isfinite(weight)) {
      sendError(request, 400, "Calibration table weights must be finite");
      return;
    }
    points.push_back({weight, liftVolts, dragVolts});
  }
  if (fabsf(points.front().weightGrams - startWeight) > 0.001f ||
      fabsf(points.back().weightGrams - endWeight) > 0.001f) {
    sendError(request, 400,
              "Start and end weights must match the first and last table rows");
    return;
  }
  if (!validCalibration(points)) {
    sendError(request, 400,
              "Voltages must be finite and strictly monotonic for each sensor");
    return;
  }

  if (!lockState()) {
    sendError(request, 503, "Calibration state is unavailable");
    return;
  }
  if (testRunning) {
    unlockState();
    sendError(request, 409, "Stop the test before changing calibration");
    return;
  }
  std::vector<TeamRecord> updatedTeams = teams;
  TeamRecord *updated = nullptr;
  for (TeamRecord &team : updatedTeams) {
    if (team.name == currentTeam) {
      updated = &team;
      break;
    }
  }
  if (updated == nullptr) {
    unlockState();
    sendError(request, 409, "Create or select a team before calibrating");
    return;
  }
  updated->calibration = points;
  if (!writeStationData(updatedTeams, currentTeam)) {
    unlockState();
    sendError(request, 500, "Could not save calibration data to ESP32 storage");
    return;
  }
  teams = updatedTeams;
  const String savedTeam = currentTeam;
  unlockState();

  JsonDocument response;
  response["team"] = savedTeam;
  response["calibrationLoaded"] = true;
  response["pointCount"] = points.size();
  sendJson(request, 200, response);
}

void handleTeams(AsyncWebServerRequest *request) {
  if (!lockState()) {
    sendError(request, 503, "Team data is unavailable");
    return;
  }
  JsonDocument response;
  response["currentTeam"] = currentTeam;
  JsonArray teamArray = response["teams"].to<JsonArray>();
  for (const TeamRecord &team : teams) {
    JsonObject item = teamArray.add<JsonObject>();
    item["name"] = team.name;
    item["calibrationLoaded"] = validCalibration(team.calibration);
  }
  unlockState();
  sendJson(request, 200, response);
}

void handleCreateTeam(AsyncWebServerRequest *request, uint8_t *body,
                      size_t length, size_t index, size_t total) {
  if (index != 0 || length != total || total > 1024) {
    sendError(request, 400, "Invalid or oversized request body");
    return;
  }
  JsonDocument document;
  if (deserializeJson(document, body, length)) {
    sendError(request, 400, "Request body must be valid JSON");
    return;
  }
  const char *nameInput = document["name"] | "";
  String name(nameInput);
  name.trim();
  if (name.isEmpty() || name.length() > MaxTeamNameLength) {
    sendError(request, 400, "Team name must contain 1 to 32 characters");
    return;
  }
  for (size_t i = 0; i < name.length(); ++i) {
    if (static_cast<uint8_t>(name[i]) < 32) {
      sendError(request, 400, "Team name contains an invalid character");
      return;
    }
  }
  if (!lockState()) {
    sendError(request, 503, "Team data is unavailable");
    return;
  }
  if (testRunning) {
    unlockState();
    sendError(request, 409, "Stop the test before creating a team");
    return;
  }
  if (teams.size() >= MaxTeams) {
    unlockState();
    sendError(request, 409, "The ESP32 team storage limit has been reached");
    return;
  }
  if (findTeam(name) != nullptr) {
    unlockState();
    sendError(request, 409, "A team with that name already exists");
    return;
  }
  std::vector<TeamRecord> updatedTeams = teams;
  updatedTeams.push_back({name, {}});
  if (!writeStationData(updatedTeams, name)) {
    unlockState();
    sendError(request, 500, "Could not save team data to ESP32 storage");
    return;
  }
  teams = updatedTeams;
  currentTeam = name;
  unlockState();
  JsonDocument response;
  response["currentTeam"] = name;
  sendJson(request, 201, response);
}

void handleSelectTeam(AsyncWebServerRequest *request, uint8_t *body,
                      size_t length, size_t index, size_t total) {
  if (index != 0 || length != total || total > 1024) {
    sendError(request, 400, "Invalid or oversized request body");
    return;
  }
  JsonDocument document;
  if (deserializeJson(document, body, length)) {
    sendError(request, 400, "Request body must be valid JSON");
    return;
  }
  const char *name = document["name"] | "";
  if (!lockState()) {
    sendError(request, 503, "Team data is unavailable");
    return;
  }
  if (testRunning) {
    unlockState();
    sendError(request, 409, "Stop the test before changing teams");
    return;
  }
  if (findTeam(String(name)) == nullptr) {
    unlockState();
    sendError(request, 404, "Team does not exist");
    return;
  }
  if (!writeStationData(teams, String(name))) {
    unlockState();
    sendError(request, 500, "Could not save the selected team to ESP32 storage");
    return;
  }
  currentTeam = name;
  unlockState();
  JsonDocument response;
  response["currentTeam"] = name;
  sendJson(request, 200, response);
}

void handleStatus(AsyncWebServerRequest *request) {
  bool running = false;
  bool stable = false;
  TestSettings current;
  String selectedTeam;
  bool teamCalibrationLoaded = false;
  if (!lockState()) {
    sendError(request, 503, "Test state is unavailable");
    return;
  }
  running = testRunning;
  current = settings;
  stable = steadyStateAchieved;
  selectedTeam = currentTeam;
  const TeamRecord *team = findTeam(currentTeam);
  teamCalibrationLoaded =
      team != nullptr && validCalibration(team->calibration);
  unlockState();

  JsonDocument response;
  response["wifiConnected"] = WiFi.status() == WL_CONNECTED;
  response["adsAvailable"] = isAdsConnected();
  response["adsError"] = getAdsError();
  const MeasurementValues values = getMeasurementValues();
  response["measurementsReady"] = isAdsConnected() && values.hasSample;
  response["running"] = running;
  response["tunnelStatus"] = running ? "Running" : "No active test";
  response["calibrationLoaded"] = teamCalibrationLoaded;
  response["calibratedSpeedMph"] = current.setpointMph;
  response["speed"] = current.setpointMph;
  response["setpoint"] = current.setpointMph;
  response["startSpeedMph"] = current.startSpeedMph;
  response["endSpeedMph"] = current.endSpeedMph;
  response["speedIncrementMph"] = current.speedIncrementMph;
  response["stabilizationSeconds"] = current.stabilizationSeconds;
  response["steadyStateErrorNewtons"] = current.steadyStateErrorNewtons;
  response["movingAverageWindowMs"] = current.movingAverageWindowMs;
  response["steadyStateHoldSeconds"] = current.steadyStateHoldSeconds;
  response["steadyState"] = running && stable;
  response["sampleRate"] = current.sampleRateHz;
  response["team"] = running ? current.team : selectedTeam;
  response["object"] = current.airfoil;
  response["units"] = current.units;
  addTunnelStatus(response);
  response["running"] = running;
  sendJson(request, 200, response);
}

void handleExport(AsyncWebServerRequest *request) {
  std::shared_ptr<CsvStreamState> stream =
      std::make_shared<CsvStreamState>();
  if (!lockState()) {
    sendError(request, 503, "Test data is unavailable");
    return;
  }
  stream->samples = testSamples;
  unlockState();

  stream->pending =
      "elapsed_ms,lift_adc_counts,drag_adc_counts,lift_volts,drag_volts,"
      "lift_newtons,drag_newtons,calibration_loaded,actual_speed_mph,"
      "setpoint_mph\r\n";
  AsyncWebServerResponse *response = request->beginChunkedResponse(
      "text/csv",
      [stream](uint8_t *buffer, size_t maxLength, size_t) -> size_t {
        size_t written = 0;
        while (written < maxLength) {
          if (stream->pending.isEmpty()) {
            if (stream->nextSample >= stream->samples.size()) {
              break;
            }
            const TestSample &sample = stream->samples[stream->nextSample++];
            stream->pending = String(sample.elapsedMs);
            stream->pending += "," + String(sample.liftRaw, 2);
            stream->pending += "," + String(sample.dragRaw, 2);
            stream->pending += "," + String(sample.liftVolts, 6);
            stream->pending += "," + String(sample.dragVolts, 6);
            if (sample.calibrationLoaded) {
              stream->pending += "," + String(sample.liftNewtons, 6);
              stream->pending += "," + String(sample.dragNewtons, 6);
              stream->pending += ",true,,";
            } else {
              stream->pending += ",,,false,,";
            }
            stream->pending += String(sample.setpointMph, 1);
            stream->pending += "\r\n";
          }

          const size_t count =
              min(maxLength - written, stream->pending.length());
          memcpy(buffer + written, stream->pending.c_str(), count);
          stream->pending.remove(0, count);
          written += count;
        }
        return written;
      });
  response->addHeader("Content-Disposition",
                      "attachment; filename=wind-tunnel-data.csv");
  request->send(response);
}

void beginWifi() {
  WiFi.mode(WIFI_STA);
  if (strlen(WIFI_SSID) == 0) {
    Serial.println(
        "Wi-Fi credentials not configured; add include/WiFiSecrets.h.");
    return;
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiAttemptAt = millis();
}

void configureRoutes() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!spiffsMounted || !SPIFFS.exists("/index.html")) {
      request->send(500, "text/plain",
                    "Dashboard missing; upload the SPIFFS filesystem image.");
      return;
    }
    request->send(SPIFFS, "/index.html", "text/html");
  });
  server.on("/api/status", HTTP_GET, handleStatus);
  measurementSocket.enable(true);
  measurementSocket.cleanupClients();
  server.addHandler(&measurementSocket);
  server.on("/api/start", HTTP_POST, [](AsyncWebServerRequest *) {}, nullptr,
            handleStart);
  server.on("/api/stop", HTTP_POST, handleStop);
  server.on("/api/tunnel/set-speed", HTTP_POST, [](AsyncWebServerRequest *) {},
            nullptr, handleTunnelSetSpeed);
  server.on("/api/tunnel/start", HTTP_POST, handleTunnelStart);
  server.on("/api/tunnel/stop", HTTP_POST, handleTunnelStop);
  server.on("/api/tunnel/status", HTTP_GET, handleTunnelStatus);
  server.on("/api/tunnel/bench-output", HTTP_POST, [](AsyncWebServerRequest *) {},
            nullptr, handleDacBenchOutput);
  server.on("/api/tunnel/bench-stop", HTTP_POST, handleDacBenchStop);
  server.on("/api/teams", HTTP_GET, handleTeams);
  server.on("/api/teams", HTTP_POST, [](AsyncWebServerRequest *) {}, nullptr,
            handleCreateTeam);
  server.on("/api/team/select", HTTP_POST, [](AsyncWebServerRequest *) {},
            nullptr, handleSelectTeam);
  server.on("/api/calibration", HTTP_GET, handleCalibration);
  server.on("/api/calibration", HTTP_POST, [](AsyncWebServerRequest *) {},
            nullptr, handleSaveCalibration);
  server.on("/api/export.csv", HTTP_GET, handleExport);
}
}  // namespace

void initializeWebServer() {
  stateMutex = xSemaphoreCreateMutex();
  if (stateMutex == nullptr) {
    Serial.println("ERROR: Could not create the web-server state mutex.");
    return;
  }

  spiffsMounted = SPIFFS.begin(true);
  if (!spiffsMounted) {
    Serial.println("ERROR: SPIFFS could not be mounted.");
  } else {
    loadStationData();
  }

  beginWifi();
  configureRoutes();
  server.begin();
  Serial.println("Asynchronous web server started on port 80.");
}

void updateWebServer() {
  updateWindTunnelControl();
  if (WiFi.status() == WL_CONNECTED && !wifiAddressPrinted) {
    Serial.print("Wi-Fi connected. Dashboard: http://");
    Serial.println(WiFi.localIP());
    wifiAddressPrinted = true;
  } else if (WiFi.status() != WL_CONNECTED) {
    wifiAddressPrinted = false;
    if (strlen(WIFI_SSID) > 0 &&
        millis() - lastWifiAttemptAt >= 15000U) {
      WiFi.reconnect();
      lastWifiAttemptAt = millis();
    }
  }

  const MeasurementValues values = getMeasurementValues();
  bool stopTunnel = false;
  const char *stopError = nullptr;
  if (!lockState()) {
    return;
  }

  if (testRunning &&
      (steadyStateAchieved ||
       millis() - speedStepStartedAt >= settings.stabilizationSeconds * 1000U)) {
    const float direction =
        settings.endSpeedMph >= settings.startSpeedMph ? 1.0f : -1.0f;
    const float nextSpeed =
        settings.setpointMph + direction * settings.speedIncrementMph;
    const bool withinSweep =
        direction > 0.0f ? nextSpeed <= settings.endSpeedMph + 0.001f
                         : nextSpeed >= settings.endSpeedMph - 0.001f;
    if (withinSweep &&
        updateWindTunnelTargetSpeedDuringTest(nextSpeed, &stopError)) {
      settings.setpointMph = nextSpeed;
      speedStepStartedAt = millis();
      resetSteadyStateTracking();
    } else {
      testRunning = false;
      stopTunnel = true;
    }
  }

  const uint32_t sampleInterval = 1000U / settings.sampleRateHz;
  if (testRunning && values.hasSample &&
      values.sequence != lastLoggedSequence &&
      millis() - lastSampleAt >= sampleInterval) {
    lastLoggedSequence = values.sequence;
    lastSampleAt = millis();
    const TeamRecord *team = findTeam(settings.team);
    const float lift =
        team != nullptr ? calibratedForceNewtons(team, values.liftVolts, true)
                        : NAN;
    const float drag =
        team != nullptr ? calibratedForceNewtons(team, values.dragVolts, false)
                        : NAN;
    const bool calibrated = isfinite(lift) && isfinite(drag);
    if (calibrated) {
      updateSteadyState(lift, drag, settings, millis());
    } else {
      resetSteadyStateTracking();
    }
    const bool steadyState = steadyStateAchieved;
    if (testSamples.size() < Config::MaxCsvSamples) {
      testSamples.push_back(
          {lastSampleAt - testStartedAt,
           settings.setpointMph,
           values.liftRaw,
           values.dragRaw,
           values.liftVolts,
           values.dragVolts,
           lift,
           drag,
           calibrated});
    }

    if (measurementSocket.count() > 0) {
      JsonDocument measurement;
      measurement["running"] = true;
      measurement["adsAvailable"] = true;
      measurement["setpoint"] = settings.setpointMph;
      measurement["speed"] = settings.setpointMph;
      measurement["steadyState"] = steadyState;
      measurement["liftRaw"] = values.liftRaw;
      measurement["dragRaw"] = values.dragRaw;
      measurement["liftVolts"] = values.liftVolts;
      measurement["dragVolts"] = values.dragVolts;
      measurement["sampleSequence"] = values.sequence;
      if (calibrated) {
        measurement["lift"] = lift;
        measurement["drag"] = drag;
      } else {
        measurement["lift"] = nullptr;
        measurement["drag"] = nullptr;
      }
      if (calibrated && steadyState) {
        measurement["steadyStateLift"] = lift;
        measurement["steadyStateDrag"] = drag;
      } else {
        measurement["steadyStateLift"] = nullptr;
        measurement["steadyStateDrag"] = nullptr;
      }
      measurement["calibrationLoaded"] = calibrated;
      if (pendingMeasurementCount == 0) {
        pendingMeasurementBatch = "";
        measurementBatchStartedAt = millis();
      } else {
        pendingMeasurementBatch += ',';
      }
      serializeJson(measurement, pendingMeasurementBatch);
      ++pendingMeasurementCount;
    }
  }
  unlockState();

  if (measurementSocket.count() == 0) {
    pendingMeasurementBatch = "";
    pendingMeasurementCount = 0;
  } else if (pendingMeasurementCount > 0 &&
             (pendingMeasurementCount >= MaxMeasurementBatchSize ||
              millis() - measurementBatchStartedAt >=
                  MeasurementBatchIntervalMs)) {
    String batch;
    batch.reserve(pendingMeasurementBatch.length() + 2);
    batch = '[';
    batch += pendingMeasurementBatch;
    batch += ']';
    if (measurementSocket.availableForWriteAll()) {
      measurementSocket.textAll(batch);
    }
    pendingMeasurementBatch = "";
    pendingMeasurementCount = 0;
  }
  if (stopTunnel) {
    const char *error = nullptr;
    if (!stopWindTunnelControl(&error)) {
      Serial.print("ERROR: Tunnel safe-stop failed: ");
      Serial.println(error != nullptr ? error : "unknown error");
    } else if (stopError != nullptr) {
      Serial.print("ERROR: Speed sweep stopped: ");
      Serial.println(stopError);
    }
  }
}
