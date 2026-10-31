#include "UserInterface.h"

#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include "Calculations.h"
#include "Config.h"
#include "Inputs.h"
#include "WindTunnelControl.h"

namespace {

AsyncWebServer server(80);
bool uiFilesystemAvailable = false;

String serializeDocument(JsonDocument& document) {
  String output;
  serializeJson(document, output);
  return output;
}

void sendJsonError(AsyncWebServerRequest* request, uint16_t status,
                   const String& message) {
  JsonDocument document;
  document["ok"] = false;
  document["error"] = message;
  request->send(status, "application/json", serializeDocument(document));
}

bool parseRequestBody(AsyncWebServerRequest* request, JsonDocument& document) {
  if (!request->hasParam("plain", true)) {
    return false;
  }
  const String body = request->getParam("plain", true)->value();
  return body.length() > 0 && !deserializeJson(document, body);
}

uint16_t parseSampleRate(JsonVariantConst value) {
  if (value.is<int>()) {
    const int rate = value.as<int>();
    return rate > 0 ? static_cast<uint16_t>(rate) : 0;
  }
  const String text = value.as<String>();
  const int rate = text.toInt();
  return rate > 0 ? static_cast<uint16_t>(rate) : 0;
}

String statusJson() {
  JsonDocument document;
  document["connected"] = true;
  document["filesystemAvailable"] =
      uiFilesystemAvailable && SPIFFS.exists("/index.html");
  document["adsAvailable"] = Inputs::available();
  document["adcInputMode"] = Inputs::modeName();
  document["adsGain"] = Inputs::gainName();
  document["adsFullScaleVolts"] = Inputs::fullScaleVolts();
  document["running"] = WindTunnelControl::running();
  document["speed"] = WindTunnelControl::speedSetpointMph();
  document["setpoint"] = WindTunnelControl::speedSetpointMph();
  document["speedIsSetpoint"] = true;
  document["object"] = WindTunnelControl::objectName();
  document["team"] = WindTunnelControl::teamName();
  document["units"] = WindTunnelControl::units();
  document["zeroed"] = Calculations::zeroed();
  document["calibrationLoaded"] = Calculations::fullyCalibrated();
  document["liftCalibrated"] = Calculations::liftCalibrated();
  document["dragCalibrated"] = Calculations::dragCalibrated();
  document["liftAmplifierConfigured"] = Calculations::amplifierConfigured(true);
  document["dragAmplifierConfigured"] = Calculations::amplifierConfigured(false);
  document["liftGain"] = Calculations::amplifierGain(true);
  document["dragGain"] = Calculations::amplifierGain(false);
  document["liftForceNPerInputVolt"] = Calculations::liftForceCoefficient();
  document["dragForceNPerInputVolt"] = Calculations::dragForceCoefficient();
  document["fanControlEnabled"] = WindTunnelControl::controllerEnabled();
  document["tunnelStatus"] =
      !WindTunnelControl::running()
          ? "Ready"
          : (WindTunnelControl::controllerEnabled()
                 ? "Running"
                 : "Running; fan output disabled");
  document["espStatus"] = "Connected";
  document["error"] = WindTunnelControl::lastError();
  return serializeDocument(document);
}

String dataJson() {
  ChannelSample lift;
  ChannelSample drag;
  const bool liftValid = Calculations::readLift(lift);
  const bool dragValid = Calculations::readDrag(drag);

  JsonDocument document;
  document["speed"] = WindTunnelControl::speedSetpointMph();
  document["liftValid"] = liftValid;
  document["dragValid"] = dragValid;
  document["running"] = WindTunnelControl::running();
  if (liftValid) {
    document["lift"] = lift.forceNewtons;
  } else {
    document["lift"] = nullptr;
    document["liftError"] = lift.error ? lift.error : "invalid lift reading";
  }
  if (dragValid) {
    document["drag"] = drag.forceNewtons;
  } else {
    document["drag"] = nullptr;
    document["dragError"] = drag.error ? drag.error : "invalid drag reading";
  }
  return serializeDocument(document);
}

String csvDocument() {
  String csv = "elapsed_ms,speed_setpoint_mph,lift_N,drag_N\r\n";
  const std::vector<MeasurementRecord> records =
      WindTunnelControl::historySnapshot();
  for (const MeasurementRecord& record : records) {
    csv += String(record.elapsedMs) + ",";
    csv += String(record.speedSetpointMph, 3) + ",";
    csv += String(record.liftNewtons, 6) + ",";
    csv += String(record.dragNewtons, 6) + "\r\n";
  }
  return csv;
}

void registerRoutes() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!uiFilesystemAvailable || !SPIFFS.exists("/index.html")) {
      request->send(503, "text/plain",
                    "Web UI unavailable: upload data/index.html to the ESP32 SPIFFS filesystem.");
      return;
    }
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!uiFilesystemAvailable || !SPIFFS.exists("/index.html")) {
      request->send(503, "text/plain",
                    "Web UI unavailable: upload data/index.html to the ESP32 SPIFFS filesystem.");
      return;
    }
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", statusJson());
  });

  server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!Inputs::available()) {
      sendJsonError(request, 503, "ADS1115 unavailable");
      return;
    }
    request->send(200, "application/json", dataJson());
  });

  server.on("/api/start", HTTP_POST, [](AsyncWebServerRequest* request) {
    JsonDocument document;
    if (!parseRequestBody(request, document)) {
      sendJsonError(request, 400, "Invalid JSON request body");
      return;
    }

    String error;
    const bool started = WindTunnelControl::start(
        document["speed"] | 25.0f,
        document["duration"] | 30U,
        parseSampleRate(document["sampleRate"]),
        String(document["object"] | "NACA 4424"),
        String(document["team"] | "Team Alpha"),
        String(document["units"] | "N"),
        error);
    if (!started) {
      sendJsonError(request, 400, error);
      return;
    }
    JsonDocument response;
    response["ok"] = true;
    response["fanControlEnabled"] = WindTunnelControl::controllerEnabled();
    request->send(200, "application/json", serializeDocument(response));
  });

  server.on("/api/stop", HTTP_POST, [](AsyncWebServerRequest* request) {
    WindTunnelControl::stop();
    request->send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/zero", HTTP_POST, [](AsyncWebServerRequest* request) {
    String error;
    if (!Calculations::zeroSensors(error)) {
      sendJsonError(request, 400, error);
      return;
    }
    request->send(200, "application/json", "{\"ok\":true,\"zeroed\":true}");
  });

  server.on("/api/calibration", HTTP_POST,
            [](AsyncWebServerRequest* request) {
    JsonDocument document;
    if (!parseRequestBody(request, document)) {
      sendJsonError(request, 400, "Invalid JSON request body");
      return;
    }

    const JsonVariantConst liftValue = document["liftKnownForceN"];
    const JsonVariantConst dragValue = document["dragKnownForceN"];
    const bool setLift = liftValue.is<float>() || liftValue.is<int>();
    const bool setDrag = dragValue.is<float>() || dragValue.is<int>();
    String error;
    if (!Calculations::calibrate(
            setLift, setLift ? liftValue.as<float>() : 0.0f,
            setDrag, setDrag ? dragValue.as<float>() : 0.0f,
            error)) {
      sendJsonError(request, 400, error);
      return;
    }

    JsonDocument response;
    response["ok"] = true;
    response["liftCalibrated"] = Calculations::liftCalibrated();
    response["dragCalibrated"] = Calculations::dragCalibrated();
    response["calibrationLoaded"] = Calculations::fullyCalibrated();
    request->send(200, "application/json", serializeDocument(response));
  });

  server.on("/api/export.csv", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* response =
        request->beginResponse(200, "text/csv", csvDocument());
    response->addHeader("Content-Disposition",
                        "attachment; filename=wind_tunnel_test.csv");
    request->send(response);
  });

  server.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "Not found");
  });
}

}  // namespace

namespace UserInterface {

void begin(bool filesystemAvailable) {
  uiFilesystemAvailable =
      filesystemAvailable && SPIFFS.exists("/index.html");
  if (!uiFilesystemAvailable) {
    Serial.println("Web UI file missing from SPIFFS; API server will still start.");
  }
  registerRoutes();
  server.begin();
  Serial.println("Web server started");
}

}  // namespace UserInterface
