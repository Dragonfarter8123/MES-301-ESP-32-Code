#include "Display.h"

#include <DFRobot_LcdDisplay.h>
#include <memory>
#include <new>

#include "Config.h"

namespace {
std::unique_ptr<DFRobot_Lcd_IIC> display;
bool displayReady = false;
uint8_t ipLabelId = 0;
uint8_t ssidLabelId = 0;
uint8_t displayI2cAddress = 0;

uint8_t scanForDisplayAddress() {
  uint8_t firstOtherAddress = 0;
  uint8_t otherAddressCount = 0;
  bool preferredAddressFound = false;
  bool anyAddressFound = false;

  Serial.println("Scanning I2C bus GPIO 21/22 for the DFR0997...");
  for (uint8_t address = 0x03; address <= 0x77; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() != 0) {
      continue;
    }

    Serial.printf("I2C device found at 0x%02X\n", address);
    anyAddressFound = true;
    if (address == Config::AdsAddress) {
      continue;
    }
    if (address == Config::DisplayI2cAddress) {
      preferredAddressFound = true;
    } else {
      if (firstOtherAddress == 0) {
        firstOtherAddress = address;
      }
      ++otherAddressCount;
    }
  }

  if (!anyAddressFound) {
    Serial.println("I2C scan found no devices on GPIO 21/22.");
  }
  if (preferredAddressFound) {
    return Config::DisplayI2cAddress;
  }
  if (otherAddressCount == 1) {
    return firstOtherAddress;
  }
  if (otherAddressCount > 1) {
    Serial.println(
        "ERROR: Multiple non-ADS1115 I2C devices found; cannot safely select the display address.");
  } else {
    Serial.printf("ERROR: No display candidate found; default address 0x%02X was not detected.\n",
                  Config::DisplayI2cAddress);
  }
  return 0;
}
}

bool initializeIpDisplay() {
  delay(Config::DisplayStartupDelayMs);
  if (!Wire.begin(Config::DisplayI2cSdaPin, Config::DisplayI2cSclPin)) {
    Serial.println("ERROR: Could not start shared I2C bus on GPIO21/22.");
    return false;
  }

  displayI2cAddress = scanForDisplayAddress();
  if (displayI2cAddress == 0) {
    return false;
  }

  display.reset(new (std::nothrow)
                    DFRobot_Lcd_IIC(&Wire, displayI2cAddress));
  if (!display) {
    Serial.println("ERROR: Could not allocate the DFR0997 display driver.");
    return false;
  }
  if (!display->begin()) {
    Serial.printf("ERROR: DFR0997 initialization failed at 0x%02X.\n",
                  displayI2cAddress);
    return false;
  }

  display->cleanScreen();
  display->setBackgroundColor(BLACK);
  const uint8_t ipTitleLabelId =
      display->drawString(10, 12, "IP Address", 1, LIGHTGREY);
  ipLabelId = display->drawString(10, 34, "0.0.0.0", 0, WHITE);
  const uint8_t ssidTitleLabelId =
      display->drawString(10, 90, "Wi-Fi Network", 1, LIGHTGREY);
  ssidLabelId = display->drawString(10, 112, "Not connected", 1, CYAN);
  if (ipTitleLabelId == 0 || ipLabelId == 0 || ssidTitleLabelId == 0 ||
      ssidLabelId == 0) {
    Serial.println("ERROR: Could not create DFR0997 status text.");
    return false;
  }
  displayReady = true;
  updateNetworkDisplay("0.0.0.0", "Not connected");
  Serial.printf("DFR0997 display initialized at I2C address 0x%02X.\n",
                displayI2cAddress);
  return true;
}

void updateNetworkDisplay(const String &ipAddress, const String &ssid) {
  if (displayReady && display && ipLabelId != 0 && ssidLabelId != 0) {
    display->updateString(ipLabelId, 10, 34, ipAddress, 0, WHITE);
    display->updateString(ssidLabelId, 10, 112, ssid, 1, CYAN);
  }
}

uint8_t getIpDisplayI2cAddress() { return displayI2cAddress; }

bool isIpDisplayInitialized() { return displayReady; }

