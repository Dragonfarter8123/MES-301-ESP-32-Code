
#include <SPIFFS.h>
#include <WiFi.h>
#include "Calculations.h"
#include "Config.h"
#include "Inputs.h"
#include "UserInterface.h"
#include "WindTunnelControl.h"

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nMES 301 Wind Tunnel Controller");

  Inputs::begin();
  Calculations::begin();
  WindTunnelControl::begin();

  const bool filesystemAvailable = SPIFFS.begin(false);
  if (!filesystemAvailable) {
    Serial.println("SPIFFS mount failed; web API will still start.");
  }

  WiFi.mode(WIFI_STA);
  bool wifiConnected = false;
  if (Config::WIFI_SSID[0] != '\0') {
    WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
    Serial.print("Connecting to configured Wi-Fi");
    const uint32_t startedAt = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - startedAt < Config::WIFI_CONNECT_TIMEOUT_MS) {
      delay(250);
      Serial.print('.');
    }
    wifiConnected = WiFi.status() == WL_CONNECTED;
  }

  if (wifiConnected) {
    Serial.println();
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("Configured Wi-Fi unavailable; starting fallback access point.");
    WiFi.mode(WIFI_AP);
    if (WiFi.softAP(Config::FALLBACK_AP_SSID,
                    Config::FALLBACK_AP_PASSWORD)) {
      Serial.print("Access point: ");
      Serial.println(Config::FALLBACK_AP_SSID);
      Serial.print("AP IP Address: ");
      Serial.println(WiFi.softAPIP());
    } else {
      Serial.println("ERROR: Could not start fallback access point.");
    }
  }

  UserInterface::begin(filesystemAvailable);
}

void loop() {
  WindTunnelControl::update();
  Inputs::printDiagnosticsIfDue();
  delay(5);
}
