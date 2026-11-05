#include <Arduino.h>

#include "Calculations.h"
#include "ESP10_Inputs.h"
#include "WebServer.h"
#include "WindTunnelControl.h"

void setup() {
  Serial.begin(115200);
  initializeLoadCellInputs();
  initializeMeasurements();
  initializeWindTunnelControl();
  initializeWebServer();
}

void loop() {
  updateLoadCellInputs();
  updateMeasurements();
  updateWindTunnelControl();
  updateWebServer();
  delay(0);
}
