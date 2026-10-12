
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <vector>

const char* ssid = "NCSU-WiFi";
const char* password = "NCState@MES3";
const uint32_t WIFI_TIMEOUT_MS = 20000;
const uint8_t ADS1115_ADDRESS = 0x48;
const uint8_t FAN_PWM_PIN = 25;
const uint8_t FAN_PWM_CHANNEL = 0;

class ADS1115Driver {
 public:
  bool begin(uint8_t address = ADS1115_ADDRESS) {
    _address = address;
    Wire.beginTransmission(_address);
    const uint8_t error = Wire.endTransmission();
    return error == 0;
  }

  void setGain(uint16_t gain) {
    _gain = gain;
  }

  int16_t readSingleEnded(uint8_t channel) {
    if (channel > 3) {
      channel = 0;
    }

    uint16_t config = 0x8000;
    config |= static_cast<uint16_t>(0x04 | (channel & 0x03)) << 12;
    config |= (_gain & 0x0007) << 9;
    config |= 0x0100;
    config |= 0x00E0;
    config |= 0x0003;

    writeRegister(0x01, config);
    delay(1);

    while ((readRegister(0x01) & 0x8000) != 0) {
      delay(1);
    }

    const uint16_t raw = readRegister(0x00);
    return static_cast<int16_t>(raw);
  }

  float computeVolts(int16_t raw) const {
    return static_cast<float>(raw) * 0.0001875f;
  }

 private:
  uint8_t _address = ADS1115_ADDRESS;
  uint16_t _gain = 0x0000;

  void writeRegister(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>(value >> 8));
    Wire.write(static_cast<uint8_t>(value & 0xFF));
    Wire.endTransmission();
  }

  uint16_t readRegister(uint8_t reg) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(static_cast<int>(_address), 2u);
    if (Wire.available() < 2) {
      return 0;
    }

    const uint8_t high = Wire.read();
    const uint8_t low = Wire.read();
    return static_cast<uint16_t>((high << 8) | low);
  }
};

AsyncWebServer server(80);
ADS1115Driver ads;
bool filesystemAvailable = false;
bool adsAvailable = false;

struct MeasurementSample {
  unsigned long timestamp;
  float speed;
  float lift;
  float drag;
};

struct WindTunnelState {
  bool running = false;
  bool calibrationLoaded = false;
  bool zeroed = false;
  float setSpeedMph = 25.0f;
  float currentSpeedMph = 0.0f;
  float liftValueN = 0.0f;
  float dragValueN = 0.0f;
  uint32_t durationMs = 30000;
  uint32_t sampleIntervalMs = 250;
  uint32_t startTimeMs = 0;
  uint32_t lastSampleMs = 0;
  String objectName = "NACA 4424";
  String units = "N";
  String teamName = "Team Alpha";
  float liftZeroVolts = 0.0f;
  float dragZeroVolts = 0.0f;
  float liftScale = 1.0f;
  float dragScale = 1.0f;
};

WindTunnelState tunnel;
std::vector<MeasurementSample> samples;

String jsonEscape(const String& source) {
  String output;
  output.reserve(source.length());
  for (size_t i = 0; i < source.length(); ++i) {
    char c = source[i];
    if (c == '"') {
      output += "\\\"";
    } else if (c == '\\') {
      output += "\\\\";
    } else if (c == '\n') {
      output += "\\n";
    } else if (c == '\r') {
      output += "\\r";
    } else if (c == '\t') {
      output += "\\t";
    } else {
      output += c;
    }
  }
  return output;
}

int parseSampleRateHz(const String& source) {
  const int valueStart = source.indexOf(' ');
  if (valueStart < 0) {
    return 20;
  }

  const String numeric = source.substring(0, valueStart);
  const int parsed = numeric.toInt();
  return (parsed > 0) ? parsed : 20;
}

void setFanOutput(float mph) {
  const float clamped = constrain(mph, 0.0f, 40.0f);
  const uint32_t duty = static_cast<uint32_t>((clamped / 40.0f) * 255.0f);
  ledcWrite(FAN_PWM_CHANNEL, duty);
}

void saveSample(float speed, float lift, float drag) {
  MeasurementSample sample;
  sample.timestamp = millis();
  sample.speed = speed;
  sample.lift = lift;
  sample.drag = drag;
  samples.push_back(sample);

  while (samples.size() > 300) {
    samples.erase(samples.begin());
  }
}

String buildStatusJson() {
  JsonDocument doc;
  doc["connected"] = true;
  doc["filesystemAvailable"] = filesystemAvailable;
  doc["adsAvailable"] = adsAvailable;
  doc["running"] = tunnel.running;
  doc["speed"] = tunnel.currentSpeedMph;
  doc["setpoint"] = tunnel.setSpeedMph;
  doc["object"] = tunnel.objectName;
  doc["team"] = tunnel.teamName;
  doc["units"] = tunnel.units;
  doc["zeroed"] = tunnel.zeroed;
  doc["calibrationLoaded"] = tunnel.calibrationLoaded;
  doc["tunnelStatus"] = tunnel.running ? "Running" : "Ready";
  doc["espStatus"] = "Connected";
  String output;
  serializeJson(doc, output);
  return output;
}

String buildDataJson() {
  JsonDocument doc;
  doc["speed"] = tunnel.currentSpeedMph;
  doc["lift"] = tunnel.liftValueN;
  doc["drag"] = tunnel.dragValueN;
  doc["running"] = tunnel.running;
  String output;
  serializeJson(doc, output);
  return output;
}

String buildCsv() {
  String csv = "timestamp,speed_mph,lift_N,drag_N\n";
  for (const MeasurementSample& sample : samples) {
    csv += String(sample.timestamp) + "," + String(sample.speed, 3) + "," + String(sample.lift, 6) + "," + String(sample.drag, 6) + "\n";
  }
  return csv;
}

float readAverageVolts(uint8_t channel) {
  if (!adsAvailable) {
    return 0.0f;
  }

  int32_t total = 0;
  constexpr uint8_t SAMPLE_COUNT = 8;
  for (uint8_t i = 0; i < SAMPLE_COUNT; ++i) {
    total += ads.readSingleEnded(channel);
  }
  return ads.computeVolts(static_cast<int16_t>(total / SAMPLE_COUNT));
}

void updateMeasurements() {
  if (!tunnel.running) {
    return;
  }
  if (!adsAvailable) {
    tunnel.running = false;
    setFanOutput(0.0f);
    return;
  }

  const uint32_t now = millis();
  if (now - tunnel.lastSampleMs < tunnel.sampleIntervalMs) {
    return;
  }

  tunnel.lastSampleMs = now;

  const float liftVolts = readAverageVolts(0);
  const float dragVolts = readAverageVolts(1);

  const float liftForce = (liftVolts - tunnel.liftZeroVolts) * tunnel.liftScale;
  const float dragForce = (dragVolts - tunnel.dragZeroVolts) * tunnel.dragScale;

  tunnel.liftValueN = liftForce;
  tunnel.dragValueN = dragForce;

  if (tunnel.currentSpeedMph < tunnel.setSpeedMph) {
    tunnel.currentSpeedMph += 0.35f;
  } else if (tunnel.currentSpeedMph > tunnel.setSpeedMph) {
    tunnel.currentSpeedMph -= 0.35f;
  }

  setFanOutput(tunnel.currentSpeedMph);
  saveSample(tunnel.currentSpeedMph, tunnel.liftValueN, tunnel.dragValueN);

  if (now - tunnel.startTimeMs >= tunnel.durationMs) {
    tunnel.running = false;
    setFanOutput(0.0f);
  }
}

void beginWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT_MS) {
    delay(500);
    Serial.print('.');
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi connection failed - starting local AP fallback");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("WindTunnel-ESP32", "mes301demo");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  }
}

bool parseJsonBody(AsyncWebServerRequest* request, JsonDocument& doc) {
  if (!request->hasParam("plain", true)) {
    return false;
  }

  String payload = request->getParam("plain", true)->value();
  if (payload.length() == 0) {
    return false;
  }

  DeserializationError err = deserializeJson(doc, payload);
  return !err;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(FAN_PWM_PIN, OUTPUT);
  ledcSetup(FAN_PWM_CHANNEL, 20000, 8);
  ledcAttachPin(FAN_PWM_PIN, FAN_PWM_CHANNEL);
  setFanOutput(0.0f);

  filesystemAvailable = SPIFFS.begin(true);
  if (!filesystemAvailable) {
    Serial.println("SPIFFS Mount Failed");
  }

  Wire.begin();
  adsAvailable = ads.begin(ADS1115_ADDRESS);
  if (!adsAvailable) {
    Serial.println("ADS1115 not found");
  } else {
    ads.setGain(0);
    Serial.println("ADS1115 ready");
  }

  beginWifi();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!filesystemAvailable || !SPIFFS.exists("/index.html")) {
      request->send(503, "text/plain", "Web UI unavailable: upload data/index.html to the ESP32 SPIFFS filesystem.");
      return;
    }
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!filesystemAvailable || !SPIFFS.exists("/index.html")) {
      request->send(503, "text/plain", "Web UI unavailable: upload data/index.html to the ESP32 SPIFFS filesystem.");
      return;
    }
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", buildStatusJson());
  });

  server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest* request) {
    String response = buildDataJson();
    request->send(adsAvailable ? 200 : 503, "application/json", response);
  });

  server.on("/api/start", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!adsAvailable) {
      request->send(503, "application/json", "{\"ok\":false,\"error\":\"ADS1115 unavailable\"}");
      return;
    }
    JsonDocument doc;
    if (!parseJsonBody(request, doc)) {
      request->send(400, "application/json", "{\"ok\":false,\"error\":\"invalid body\"}");
      return;
    }

    tunnel.running = true;
    tunnel.startTimeMs = millis();
    tunnel.lastSampleMs = 0;
    tunnel.setSpeedMph = constrain(float(doc["speed"] | 25.0f), 20.0f, 40.0f);
    tunnel.objectName = String(doc["object"] | "NACA 4424");
    tunnel.units = String(doc["units"] | "N");
    tunnel.teamName = String(doc["team"] | "Team Alpha");
    tunnel.durationMs = uint32_t(doc["duration"] | 30000);
    tunnel.sampleIntervalMs = 1000 / max(1, parseSampleRateHz(String(doc["sampleRate"] | "20 Hz")));
    tunnel.calibrationLoaded = true;

    if (!tunnel.zeroed) {
      tunnel.liftZeroVolts = readAverageVolts(0);
      tunnel.dragZeroVolts = readAverageVolts(1);
      tunnel.zeroed = true;
    }

    setFanOutput(tunnel.setSpeedMph);
    request->send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/stop", HTTP_POST, [](AsyncWebServerRequest* request) {
    tunnel.running = false;
    setFanOutput(0.0f);
    request->send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/zero", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!adsAvailable) {
      request->send(503, "application/json", "{\"ok\":false,\"error\":\"ADS1115 unavailable\"}");
      return;
    }
    tunnel.liftZeroVolts = readAverageVolts(0);
    tunnel.dragZeroVolts = readAverageVolts(1);
    tunnel.zeroed = true;
    tunnel.liftValueN = 0.0f;
    tunnel.dragValueN = 0.0f;
    request->send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/calibration", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!adsAvailable) {
      request->send(503, "application/json", "{\"ok\":false,\"error\":\"ADS1115 unavailable\"}");
      return;
    }
    JsonDocument doc;
    if (!parseJsonBody(request, doc)) {
      request->send(400, "application/json", "{\"ok\":false,\"error\":\"invalid body\"}");
      return;
    }

    tunnel.objectName = String(doc["object"] | tunnel.objectName);
    tunnel.calibrationLoaded = true;
    tunnel.liftScale = 3.5f;
    tunnel.dragScale = 2.8f;

    if (!tunnel.zeroed) {
      tunnel.liftZeroVolts = readAverageVolts(0);
      tunnel.dragZeroVolts = readAverageVolts(1);
      tunnel.zeroed = true;
    }

    request->send(200, "application/json", "{\"ok\":true,\"object\":\"" + jsonEscape(tunnel.objectName) + "\"}");
  });

  server.on("/api/export.csv", HTTP_GET, [](AsyncWebServerRequest* request) {
    String csv = buildCsv();
    AsyncWebServerResponse* response = request->beginResponse(200, "text/csv", csv);
    response->addHeader("Content-Disposition", "attachment; filename=wind_tunnel_test.csv");
    request->send(response);
  });

  server.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "Not found");
  });

  server.begin();
  Serial.println("Web server started");
}

void loop() {
  updateMeasurements();
  delay(10);
}
