#pragma once

#include "WindTunnelTypes.h"

namespace Calculations {

bool begin();
bool amplifierConfigured(bool lift);
float amplifierGain(bool lift);
bool zeroSensors(String& error);
bool calibrate(bool setLift, float liftKnownForceN,
               bool setDrag, float dragKnownForceN, String& error);
bool readLift(ChannelSample& sample);
bool readDrag(ChannelSample& sample);
bool liftCalibrated();
bool dragCalibrated();
bool fullyCalibrated();
bool zeroed();
float liftForceCoefficient();
float dragForceCoefficient();
const char* liftConfigurationError();
const char* dragConfigurationError();

}  // namespace Calculations
