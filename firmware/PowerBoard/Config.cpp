#include <Arduino.h>
#include "Config.h"
#include "Globals.h"

const uint8_t inaAddresses[INA_COUNT] = {
    0x40, // U2
    0x41, // U3
    0x42, // U4
    0x43, // U5
    0x44  // U6
};

const char* inaNames[INA_COUNT] = {
    "U2",
    "U3",
    "U4",
    "U5",
    "U6"
};

const uint8_t controlPins[INA_COUNT] = {
    13, // U2
    12, // U3
    11, // U4
    10, // U5
    9   // U6
};

const uint32_t shuntResistors_uOhm[INA_COUNT] = {
    3000, // U2 má 3 mOhm
    5000, // U3 má 5 mOhm
    5000,
    5000,
    5000
};

const float currentCalibrationFactors[INA_COUNT] = {
    1.000f,
    1.000f,
    1.000f,
    1.000f,
    1.000f
};

const float currentLimits_mA[INA_COUNT] = {
    15000.0f,
    4000.0f,
    5000.0f,
    6000.0f,
    14000.0f
};

bool showAddressRow = true;
bool showOnlineRow = true;
bool showOutputRow = true;
bool showLimitRow = true;
bool showVoltageRow = true;
bool showCurrentRow = true;
bool showCalcRow = true;
bool showCalRow = true;
bool showTempRow = true;