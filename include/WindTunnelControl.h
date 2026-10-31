#pragma once

#include <Arduino.h>
#include <vector>
#include "WindTunnelTypes.h"

namespace WindTunnelControl {

void begin();
bool start(float speedMph, uint32_t durationSeconds, uint16_t sampleRateHz,
           const String& objectName, const String& teamName, const String& units,
           String& error);
void stop();
void update();
bool running();
bool controllerEnabled();
float speedSetpointMph();
uint32_t durationSeconds();
uint16_t sampleRateHz();
String objectName();
String teamName();
String units();
String lastError();
bool latestLift(ChannelSample& sample);
bool latestDrag(ChannelSample& sample);
std::vector<MeasurementRecord> historySnapshot();
void clearHistory();

}  // namespace WindTunnelControl
