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
  float setpointMph = 25.0f;
  uint32_t durationSeconds = 30;
  uint16_t sampleRateHz = 20;
  String team = "Team 1";
  String airfoil = "NACA 4424";
  String units = "N";
};

struct TestSample {
  uint32_t elapsedMs;
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
  float setpointMph = 25.0f;
};

AsyncWebServer server(80);
SemaphoreHandle_t stateMutex = nullptr;
TestSettings settings;
std::vector<TestSample> testSamples;
bool testRunning = false;
uint32_t testStartedAt = 0;
uint32_t lastSampleAt = 0;
uint32_t lastLoggedSequence = 0;
uint32_t testStartSequence = 0;
uint32_t lastWifiAttemptAt = 0;
bool wifiAddressPrinted = false;
bool spiffsMounted = false;

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
  response["targetSpeedMph"] = status.targetSpeedMph;
  if (status.calculationAvailable) {
    response["estimatedFanRPM"] = status.estimatedFanRpm;
    response["calculatedVoltage"] = status.calculatedVoltage;
  } else {
    response["estimatedFanRPM"] = nullptr;
    response["calculatedVoltage"] = nullptr;
  }
  response["outputEnabled"] = status.outputEnabled;
  response["dacConnected"] = status.dacConnected;
  response["actualSpeedMph"] = nullptr;
  response["running"] = status.running;
  response["tunnelRunning"] = status.running;
  response["stabilizationElapsedMs"] = status.stabilizationElapsedMs;
  response["stabilizationPeriodMs"] = Config::TunnelStabilizationPeriodMs;
  response["stabilizationTimerComplete"] =
      status.stabilizationTimerComplete;
  response["steadyStateVerified"] = status.steadyStateVerified;
  response["controlMode"] = "simulation";
  response["message"] =
      "Simulation only; physical output is disabled and actual wind speed is unknown.";
}

bool validStartSettings(JsonDocument &document, TestSettings &result) {
  const char *team = document["team"] | "";
  const char *airfoil = document["object"] | "";
  const char *units = document["units"] | "N";
  const float setpoint = document["speed"] | 25.0f;
  const float durationInput = document["duration"] | 0.0f;
  const float sampleRateInput = document["sampleRate"] | 0.0f;

  if (!isfinite(setpoint) || setpoint < Config::MinTunnelSpeedMph ||
      setpoint > Config::MaxTunnelSpeedMph ||
      fabsf(setpoint / Config::TunnelSpeedStepMph -
            roundf(setpoint / Config::TunnelSpeedStepMph)) > 0.001f) {
    return false;
  }
  if (strcmp(team, "Team 1") != 0 && strcmp(team, "Team 2") != 0 &&
      strcmp(team, "Team 3") != 0 && strcmp(team, "Team 4") != 0 &&
      strcmp(team, "Team 5") != 0) {
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
  if (!isfinite(durationInput) || durationInput < 1.0f ||
      durationInput > Config::MaxTestDurationSeconds ||
      floorf(durationInput) != durationInput) {
    return false;
  }
  const float maxSampleRateHz =
      1000.0f / Config::InputSamplingIntervalMs;
  if (!isfinite(sampleRateInput) || sampleRateInput < 1.0f ||
      sampleRateInput > maxSampleRateHz ||
      floorf(sampleRateInput) != sampleRateInput) {
    return false;
  }

  result.setpointMph = setpoint;
  result.durationSeconds = static_cast<uint32_t>(durationInput);
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
  testRunning = true;
  testStartedAt = millis();
  lastSampleAt = testStartedAt - (1000U / settings.sampleRateHz);
  testStartSequence = getMeasurementValues().sequence;
  lastLoggedSequence = testStartSequence;
  unlockState();

  JsonDocument response;
  response["running"] = true;
  response["durationSeconds"] = requested.durationSeconds;
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
  stopWindTunnelControl();

  JsonDocument response;
  response["running"] = false;
  addTunnelStatus(response);
  sendJson(request, 200, response);
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
  stopWindTunnelControl();

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
  JsonDocument response;
  response["calibrationLoaded"] = isCalibrationLoaded();
  response["error"] =
      "No valid calibration values are configured; calibration was not changed.";
  sendJson(request, 409, response);
}

void handleStatus(AsyncWebServerRequest *request) {
  bool running = false;
  TestSettings current;
  if (!lockState()) {
    sendError(request, 503, "Test state is unavailable");
    return;
  }
  running = testRunning;
  current = settings;
  unlockState();

  JsonDocument response;
  response["wifiConnected"] = WiFi.status() == WL_CONNECTED;
  response["adsAvailable"] = isAdsConnected();
  response["adsError"] = getAdsError();
  const MeasurementValues values = getMeasurementValues();
  response["measurementsReady"] = isAdsConnected() && values.hasSample;
  response["running"] = running;
  response["tunnelStatus"] = running ? "Running" : "No active test";
  response["calibrationLoaded"] = isCalibrationLoaded();
  response["actualSpeed"] = nullptr;
  response["speed"] = nullptr;
  response["setpoint"] = current.setpointMph;
  response["durationSeconds"] = current.durationSeconds;
  response["sampleRate"] = current.sampleRateHz;
  response["team"] = current.team;
  response["object"] = current.airfoil;
  response["units"] = current.units;
  addTunnelStatus(response);
  response["running"] = running;
  sendJson(request, 200, response);
}

void handleData(AsyncWebServerRequest *request) {
  bool running = false;
  float setpoint = 25.0f;
  uint32_t firstSequence = 0;
  if (!lockState()) {
    sendError(request, 503, "Test state is unavailable");
    return;
  }
  running = testRunning;
  setpoint = settings.setpointMph;
  firstSequence = testStartSequence;
  unlockState();

  const MeasurementValues values = getMeasurementValues();
  JsonDocument response;
  response["running"] = running;
  response["adsAvailable"] = isAdsConnected();
  response["setpoint"] = setpoint;
  response["speed"] = nullptr;
  response["steadyState"] = false;

  if (running && isAdsConnected() && values.hasSample &&
      values.sequence != firstSequence) {
    response["liftRaw"] = values.liftRaw;
    response["dragRaw"] = values.dragRaw;
    response["liftVolts"] = values.liftVolts;
    response["dragVolts"] = values.dragVolts;
    response["sampleSequence"] = values.sequence;
    if (values.calibrationLoaded) {
      response["lift"] = values.liftNewtons;
      response["drag"] = values.dragNewtons;
    } else {
      response["lift"] = nullptr;
      response["drag"] = nullptr;
    }
  } else {
    response["liftRaw"] = nullptr;
    response["dragRaw"] = nullptr;
    response["liftVolts"] = nullptr;
    response["dragVolts"] = nullptr;
    response["lift"] = nullptr;
    response["drag"] = nullptr;
  }
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
  stream->setpointMph = settings.setpointMph;
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
            stream->pending += String(stream->setpointMph, 1);
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
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/start", HTTP_POST, [](AsyncWebServerRequest *) {}, nullptr,
            handleStart);
  server.on("/api/stop", HTTP_POST, handleStop);
  server.on("/api/tunnel/set-speed", HTTP_POST, [](AsyncWebServerRequest *) {},
            nullptr, handleTunnelSetSpeed);
  server.on("/api/tunnel/start", HTTP_POST, handleTunnelStart);
  server.on("/api/tunnel/stop", HTTP_POST, handleTunnelStop);
  server.on("/api/tunnel/status", HTTP_GET, handleTunnelStatus);
  server.on("/api/calibration", HTTP_POST, handleCalibration);
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
  if (!lockState()) {
    return;
  }

  if (testRunning &&
      millis() - testStartedAt >= settings.durationSeconds * 1000U) {
    testRunning = false;
    unlockState();
    stopWindTunnelControl();
    return;
  }

  const uint32_t sampleInterval = 1000U / settings.sampleRateHz;
  if (testRunning && values.hasSample &&
      values.sequence != lastLoggedSequence &&
      millis() - lastSampleAt >= sampleInterval) {
    lastLoggedSequence = values.sequence;
    lastSampleAt = millis();
    if (testSamples.size() < Config::MaxCsvSamples) {
      testSamples.push_back(
          {lastSampleAt - testStartedAt,
           values.liftRaw,
           values.dragRaw,
           values.liftVolts,
           values.dragVolts,
           values.liftNewtons,
           values.dragNewtons,
           values.calibrationLoaded});
    }
  }
  unlockState();
}
