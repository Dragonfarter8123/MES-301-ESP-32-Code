#pragma once

#include "WindTunnelTypes.h"

namespace Inputs {

bool begin();
bool available();
bool readLift(ChannelSample& sample);
bool readDrag(ChannelSample& sample);
void printDiagnosticsIfDue();
const char* modeName();
const char* gainName();
float fullScaleVolts();

}  // namespace Inputs
