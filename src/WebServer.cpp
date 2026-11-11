#include "WebServer.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <Wire.h>
#include <cstdlib>
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
  std::vector<CalibrationPoint> calibration4424;
  std::vector<CalibrationPoint> calibration6412;
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

constexpr size_t MaxStartRequestBytes = 1024;

struct StartRequestBody {
  size_t total;
  size_t received;
  uint8_t data[MaxStartRequestBytes];
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

bool validCalibrationObject(const String &object) {
  return object == "NACA 4424" || object == "NACA 6412";
}

std::vector<CalibrationPoint> *calibrationForObject(TeamRecord &team,
                                                    const String &object) {
  if (object == "NACA 4424") return &team.calibration4424;
  if (object == "NACA 6412") return &team.calibration6412;
  return nullptr;
}

const std::vector<CalibrationPoint> *
calibrationForObject(const TeamRecord &team, const String &object) {
  if (object == "NACA 4424") return &team.calibration4424;
  if (object == "NACA 6412") return &team.calibration6412;
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
    JsonArray calibrations = item["calibrations"].to<JsonArray>();
    const char *objects[] = {"NACA 4424", "NACA 6412"};
    const std::vector<CalibrationPoint> *objectPoints[] = {
        &team.calibration4424, &team.calibration6412};
    for (size_t objectIndex = 0; objectIndex < 2; ++objectIndex) {
      JsonObject calibration = calibrations.add<JsonObject>();
      calibration["object"] = objects[objectIndex];
      JsonArray points = calibration["points"].to<JsonArray>();
      for (const CalibrationPoint &point : *objectPoints[objectIndex]) {
        JsonObject entry = points.add<JsonObject>();
        entry["weightGrams"] = point.weightGrams;
        entry["liftVolts"] = point.liftVolts;
        entry["dragVolts"] = point.dragVolts;
      }
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
    JsonArrayConst calibrations = item["calibrations"].as<JsonArrayConst>();
    if (!calibrations.isNull()) {
      for (JsonObjectConst calibration : calibrations) {
        const String object = calibration["object"] | "";
        std::vector<CalibrationPoint> *points =
            calibrationForObject(team, object);
        if (points == nullptr) continue;
        for (JsonObjectConst point :
             calibration["points"].as<JsonArrayConst>()) {
          points->push_back({point["weightGrams"] | NAN,
                             point["liftVolts"] | NAN,
                             point["dragVolts"] | NAN});
        }
      }
    } else {
      for (JsonObjectConst point : item["calibration"].as<JsonArrayConst>()) {
        team.calibration4424.push_back(
            {point["weightGrams"] | NAN, point["liftVolts"] | NAN,
             point["dragVolts"] | NAN});
      }
    }
    for (const String object : {String("NACA 4424"), String("NACA 6412")}) {
      std::vector<CalibrationPoint> *points =
          calibrationForObject(team, object);
      if (!points->empty() && !validCalibration(*points)) {
        points->clear();
        Serial.println("ERROR: Invalid saved object calibration ignored.");
      }
    }
    loaded.push_back(team);
  }
  teams = loaded;
  const char *selected = document["currentTeam"] | "";
  if (findTeam(String(selected)) != nullptr) currentTeam = selected;
}

struct CalibrationPolynomial {
  double coefficients[3] = {};
  double center = 0.0;
  double scale = 1.0;
};

double calibrationX(const CalibrationPoint &point, bool lift,
                    bool voltageToWeight) {
  return voltageToWeight ? (lift ? point.liftVolts : point.dragVolts)
                         : point.weightGrams;
}

double calibrationY(const CalibrationPoint &point, bool lift,
                    bool voltageToWeight) {
  return voltageToWeight ? point.weightGrams
                         : (lift ? point.liftVolts : point.dragVolts);
}

bool solveCalibrationPolynomial(const std::vector<CalibrationPoint> &points,
                                bool lift, bool voltageToWeight,
                                uint8_t degree,
                                CalibrationPolynomial &polynomial) {
  double minimumX = calibrationX(points.front(), lift, voltageToWeight);
  double maximumX = minimumX;
  for (const CalibrationPoint &point : points) {
    const double x = calibrationX(point, lift, voltageToWeight);
    minimumX = min(minimumX, x);
    maximumX = max(maximumX, x);
  }
  if (maximumX <= minimumX) return false;
  polynomial.center = (minimumX + maximumX) * 0.5;
  polynomial.scale = (maximumX - minimumX) * 0.5;
  polynomial.coefficients[0] = 0.0;
  polynomial.coefficients[1] = 0.0;
  polynomial.coefficients[2] = 0.0;

  double sums[5] = {};
  double weightedSums[3] = {};
  for (const CalibrationPoint &point : points) {
    const double x =
        (calibrationX(point, lift, voltageToWeight) - polynomial.center) /
        polynomial.scale;
    const double y = calibrationY(point, lift, voltageToWeight);
    double power = 1.0;
    for (uint8_t exponent = 0; exponent <= degree * 2; ++exponent) {
      sums[exponent] += power;
      if (exponent <= degree) weightedSums[exponent] += y * power;
      power *= x;
    }
  }

  double matrix[3][4] = {};
  for (uint8_t row = 0; row <= degree; ++row) {
    for (uint8_t column = 0; column <= degree; ++column) {
      matrix[row][column] = sums[row + column];
    }
    matrix[row][degree + 1] = weightedSums[row];
  }
  for (uint8_t column = 0; column <= degree; ++column) {
    uint8_t pivot = column;
    for (uint8_t row = column + 1; row <= degree; ++row) {
      if (fabs(matrix[row][column]) > fabs(matrix[pivot][column])) {
        pivot = row;
      }
    }
    if (fabs(matrix[pivot][column]) < 1e-12) return false;
    if (pivot != column) {
      for (uint8_t entry = column; entry <= degree + 1; ++entry) {
        const double value = matrix[column][entry];
        matrix[column][entry] = matrix[pivot][entry];
        matrix[pivot][entry] = value;
      }
    }
    const double divisor = matrix[column][column];
    for (uint8_t entry = column; entry <= degree + 1; ++entry) {
      matrix[column][entry] /= divisor;
    }
    for (uint8_t row = 0; row <= degree; ++row) {
      if (row == column) continue;
      const double factor = matrix[row][column];
      for (uint8_t entry = column; entry <= degree + 1; ++entry) {
        matrix[row][entry] -= factor * matrix[column][entry];
      }
    }
  }
  for (uint8_t coefficient = 0; coefficient <= degree; ++coefficient) {
    polynomial.coefficients[coefficient] =
        matrix[coefficient][degree + 1];
  }
  return true;
}

double evaluateCalibrationPolynomial(const CalibrationPolynomial &polynomial,
                                     double x) {
  const double normalized = (x - polynomial.center) / polynomial.scale;
  return polynomial.coefficients[0] +
         normalized * (polynomial.coefficients[1] +
                       normalized * polynomial.coefficients[2]);
}

bool fitCalibrationPolynomial(const std::vector<CalibrationPoint> &points,
                              bool lift, bool voltageToWeight,
                              CalibrationPolynomial &polynomial) {
  const double xDirection =
      calibrationX(points.back(), lift, voltageToWeight) -
      calibrationX(points.front(), lift, voltageToWeight);
  const double yDirection =
      calibrationY(points.back(), lift, voltageToWeight) -
      calibrationY(points.front(), lift, voltageToWeight);
  const double expectedDirection = xDirection * yDirection > 0.0 ? 1.0 : -1.0;
  const uint8_t firstDegree = points.size() >= 3 ? 2 : 1;
  for (int degree = firstDegree; degree >= 1; --degree) {
    if (!solveCalibrationPolynomial(points, lift, voltageToWeight,
                                    static_cast<uint8_t>(degree),
                                    polynomial)) {
      continue;
    }
    const double derivativeAtMinimum =
        polynomial.coefficients[1] -
        2.0 * polynomial.coefficients[2];
    const double derivativeAtMaximum =
        polynomial.coefficients[1] +
        2.0 * polynomial.coefficients[2];
    if (expectedDirection * derivativeAtMinimum > 1e-9 &&
        expectedDirection * derivativeAtMaximum > 1e-9) {
      return true;
    }
  }
  return false;
}

float calibratedForceNewtons(const TeamRecord *team, const String &object,
                             float voltage, bool lift) {
  if (team == nullptr || !isfinite(voltage)) return NAN;
  const std::vector<CalibrationPoint> *points =
      calibrationForObject(*team, object);
  if (points == nullptr || !validCalibration(*points)) return NAN;
  CalibrationPolynomial weightFromVoltage;
  CalibrationPolynomial voltageFromWeight;
  if (!fitCalibrationPolynomial(*points, lift, true, weightFromVoltage) ||
      !fitCalibrationPolynomial(*points, lift, false, voltageFromWeight)) {
    return NAN;
  }
  const double firstVoltage =
      lift ? points->front().liftVolts : points->front().dragVolts;
  const double lastVoltage =
      lift ? points->back().liftVolts : points->back().dragVolts;
  if (voltage < min(firstVoltage, lastVoltage) ||
      voltage > max(firstVoltage, lastVoltage)) {
    return NAN;
  }
  const double zeroVoltage =
      evaluateCalibrationPolynomial(voltageFromWeight, 0.0);
  const double grams = evaluateCalibrationPolynomial(weightFromVoltage, voltage);
  const double zeroGrams =
      evaluateCalibrationPolynomial(weightFromVoltage, zeroVoltage);
  if (!isfinite(grams) || !isfinite(zeroGrams)) return NAN;
  return static_cast<float>((grams - zeroGrams) * 0.00980665);
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

I2cBusScanResult scanI2cBus(TwoWire &bus) {
  I2cBusScanResult result{};
  result.busReady = bus.begin();
  if (!result.busReady) {
    return result;
  }

  for (uint8_t address = 0x03; address <= 0x77; ++address) {
    bus.beginTransmission(address);
    const uint8_t error = bus.endTransmission();
    if (error == 0) {
      if (result.deviceCount < sizeof(result.deviceAddresses)) {
        result.deviceAddresses[result.deviceCount++] = address;
      } else {
        result.overflow = true;
      }
    } else if (error != 2) {
      result.scanError = true;
    }
  }
  return result;
}

bool scanContainsAddress(const I2cBusScanResult &scan, uint8_t address) {
  for (uint8_t i = 0; i < scan.deviceCount; ++i) {
    if (scan.deviceAddresses[i] == address) {
      return true;
    }
  }
  return false;
}

void addI2cScanResult(JsonObject busDocument,
                      const I2cBusScanResult &scan, int sdaPin, int sclPin) {
  busDocument["ready"] = scan.busReady;
  busDocument["sdaPin"] = sdaPin;
  busDocument["sclPin"] = sclPin;
  busDocument["scanError"] = scan.scanError;
  busDocument["overflow"] = scan.overflow;
  JsonArray devices = busDocument["devices"].to<JsonArray>();
  for (uint8_t i = 0; i < scan.deviceCount; ++i) {
    char address[7];
    snprintf(address, sizeof(address), "0x%02X",
             scan.deviceAddresses[i]);
    devices.add(address);
  }
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

void handleStartBody(AsyncWebServerRequest *request, uint8_t *body,
                     size_t length) {
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

void handleStart(AsyncWebServerRequest *request, uint8_t *body, size_t length,
                 size_t index, size_t total) {
  if (total > MaxStartRequestBytes) {
    sendError(request, 400, "Invalid or oversized request body");
    return;
  }

  auto *buffer = static_cast<StartRequestBody *>(request->_tempObject);
  if (index == 0) {
    free(buffer);
    request->_tempObject = nullptr;
    buffer = static_cast<StartRequestBody *>(
        calloc(1, sizeof(StartRequestBody)));
    if (buffer == nullptr) {
      request->_tempObject = nullptr;
      sendError(request, 503, "Could not buffer the start request");
      return;
    }
    buffer->total = total;
    request->_tempObject = buffer;
  }
  if (buffer == nullptr || total != buffer->total ||
      index != buffer->received || length > total - buffer->received) {
    free(buffer);
    request->_tempObject = nullptr;
    sendError(request, 400, "Invalid or incomplete request body");
    return;
  }

  memcpy(buffer->data + index, body, length);
  buffer->received += length;
  if (buffer->received < buffer->total) {
    return;
  }

  request->_tempObject = nullptr;
  handleStartBody(request, buffer->data, buffer->received);
  free(buffer);
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

void handleRawInputs(AsyncWebServerRequest *request) {
  const LoadCellReadings readings = getLoadCellReadings();
  JsonDocument response;
  response["adsAvailable"] = isAdsConnected();
  response["adsError"] = getAdsError();
  response["sequence"] = readings.sequence;
  response["liftValid"] = readings.lift.valid;
  response["liftRaw"] = readings.lift.raw;
  response["liftVolts"] = readings.lift.volts;
  response["dragValid"] = readings.drag.valid;
  response["dragRaw"] = readings.drag.raw;
  response["dragVolts"] = readings.drag.volts;
  sendJson(request, 200, response);
}

void handleI2cScan(AsyncWebServerRequest *request) {
  const bool dacInitialized = initializeDAC();
  const I2cBusScanResult adsScan = scanI2cBus(Wire);
  I2cBusScanResult dacScan{};
  const bool dacBusReady = scanDACI2cBus(dacScan);

  JsonDocument response;
  response["scanRange"] = "0x03-0x77";
  response["adsExpectedAddress"] = Config::AdsAddress;
  response["adsExpectedFound"] =
      scanContainsAddress(adsScan, Config::AdsAddress);
  response["dacExpectedAddress"] = Config::DacI2cAddress;
  response["dacExpectedFound"] =
      scanContainsAddress(dacScan, Config::DacI2cAddress);
  response["dacInitialized"] = dacInitialized;
  response["dacOutputReadbackAvailable"] = false;
  JsonObject adsBus = response["adsBus"].to<JsonObject>();
  addI2cScanResult(adsBus, adsScan, Config::I2cSdaPin,
                   Config::I2cSclPin);
  JsonObject dacBus = response["dacBus"].to<JsonObject>();
  addI2cScanResult(dacBus, dacScan, Config::DacI2cSdaPin,
                   Config::DacI2cSclPin);
  if (!dacBusReady) {
    response["dacBus"]["ready"] = false;
  }
  sendJson(request, 200, response);
}

void handleCalibration(AsyncWebServerRequest *request) {
  const String object = request->hasParam("object")
                            ? request->getParam("object")->value()
                            : "NACA 4424";
  if (!validCalibrationObject(object)) {
    sendError(request, 400, "Select a supported test object");
    return;
  }
  if (!lockState()) {
    sendError(request, 503, "Calibration state is unavailable");
    return;
  }
  const TeamRecord *team = findTeam(currentTeam);
  const std::vector<CalibrationPoint> *points =
      team != nullptr ? calibrationForObject(*team, object) : nullptr;
  JsonDocument response;
  response["team"] = currentTeam;
  response["object"] = object;
  response["calibrationLoaded"] =
      points != nullptr && validCalibration(*points);
  if (points != nullptr) {
    JsonArray pointArray = response["points"].to<JsonArray>();
    for (const CalibrationPoint &point : *points) {
      JsonObject item = pointArray.add<JsonObject>();
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
  const String object = document["object"] | "";
  JsonArrayConst entries = document["points"].as<JsonArrayConst>();
  if (!validCalibrationObject(object) ||
      !isfinite(startWeight) || !isfinite(endWeight) ||
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
  std::vector<CalibrationPoint> *objectCalibration =
      calibrationForObject(*updated, object);
  if (objectCalibration == nullptr) {
    unlockState();
    sendError(request, 400, "Select a supported test object");
    return;
  }
  *objectCalibration = points;
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
  response["object"] = object;
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
    item["calibrationLoaded"] =
        validCalibration(team.calibration4424) ||
        validCalibration(team.calibration6412);
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
  updatedTeams.push_back({name, {}, {}});
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
  const String requestedObject =
      request->hasParam("object") ? request->getParam("object")->value()
                                 : "NACA 4424";
  if (!validCalibrationObject(requestedObject)) {
    sendError(request, 400, "Select a supported test object");
    return;
  }
  bool running = false;
  bool stable = false;
  TestSettings current;
  String selectedTeam;
  String selectedObject;
  bool teamCalibrationLoaded = false;
  if (!lockState()) {
    sendError(request, 503, "Test state is unavailable");
    return;
  }
  running = testRunning;
  current = settings;
  stable = steadyStateAchieved;
  selectedTeam = currentTeam;
  selectedObject = running ? current.airfoil : requestedObject;
  const TeamRecord *team = findTeam(currentTeam);
  const std::vector<CalibrationPoint> *points =
      team != nullptr ? calibrationForObject(*team, selectedObject) : nullptr;
  teamCalibrationLoaded =
      points != nullptr && validCalibration(*points);
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
  response["object"] = selectedObject;
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
  server.on("/api/inputs/raw", HTTP_GET, handleRawInputs);
  server.on("/api/i2c/scan", HTTP_GET, handleI2cScan);
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
        team != nullptr
            ? calibratedForceNewtons(team, settings.airfoil, values.liftVolts,
                                     true)
            : NAN;
    const float drag =
        team != nullptr
            ? calibratedForceNewtons(team, settings.airfoil, values.dragVolts,
                                     false)
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
