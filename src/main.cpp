#include <Arduino.h>

#include "Calculations.h"
#include "ESP10_Inputs.h"
#include "WebServer.h"

void setup() {
  Serial.begin(115200);
  initializeLoadCellInputs();
  initializeMeasurements();
  initializeWebServer();
}

void loop() {
  updateLoadCellInputs();
  updateMeasurements();
  updateWebServer();
  delay(1);
}
