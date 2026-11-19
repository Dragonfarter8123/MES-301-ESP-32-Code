#pragma once

#include <stddef.h>

namespace DacCalibration {

struct RpmVoltagePoint {
  float rpm;
  float voltage;
};

struct SpeedVoltagePoint {
  float mph;
  float voltage;
};

constexpr RpmVoltagePoint RpmVoltageData[] = {
    {329.0f, 1.05f},  {347.0f, 1.1f},   {496.0f, 1.5f},
    {677.0f, 2.0f},   {858.0f, 2.5f},   {1039.0f, 3.0f},
    {1218.0f, 3.5f},  {1397.0f, 4.0f},  {1577.0f, 4.5f},
    {1756.0f, 5.0f},  {1933.0f, 5.5f},  {2111.0f, 6.0f},
    {2290.0f, 6.5f},  {2470.0f, 7.0f},  {2646.0f, 7.5f},
    {2825.0f, 8.0f},  {3003.0f, 8.5f},  {3183.0f, 9.0f},
};

constexpr size_t RpmVoltageDataCount =
    sizeof(RpmVoltageData) / sizeof(RpmVoltageData[0]);

// Airspeed and estimated voltage-control pairs from the supplied workbook.
constexpr SpeedVoltagePoint SpeedVoltageData[] = {
    {20.54f, 4.67f}, {22.78f, 5.18f}, {25.03f, 5.69f},
    {27.27f, 6.20f}, {29.51f, 6.71f}, {31.76f, 7.22f},
    {34.00f, 7.73f}, {36.24f, 8.24f}, {38.48f, 8.75f},
    {39.98f, 9.09f},
};

constexpr size_t SpeedVoltageDataCount =
    sizeof(SpeedVoltageData) / sizeof(SpeedVoltageData[0]);

}  // namespace DacCalibration
