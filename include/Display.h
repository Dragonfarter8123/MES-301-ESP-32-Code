#pragma once

#include <Arduino.h>
#include <Wire.h>

bool initializeIpDisplay();
void updateNetworkDisplay(const String &ipAddress, const String &ssid);
uint8_t getIpDisplayI2cAddress();
bool isIpDisplayInitialized();
