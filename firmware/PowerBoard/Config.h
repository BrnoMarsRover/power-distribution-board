#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>
#include "Globals.h"

extern const uint8_t inaAddresses[INA_COUNT];
extern const char* inaNames[INA_COUNT];
extern const uint8_t controlPins[INA_COUNT];
extern const uint32_t shuntResistors_uOhm[INA_COUNT];
extern const float currentCalibrationFactors[INA_COUNT];
extern const float currentLimits_mA[INA_COUNT];

// Over-current protection master switch. Defaults to false because some high-side
// drivers on this board are bridged and cannot be opened - see Config.cpp.
extern bool ocpEnabled;
// Consecutive over-limit samples required before OCP acts (inrush debounce).
extern const uint8_t ocpTripSamples;

extern bool showAddressRow;
extern bool showOnlineRow;
extern bool showOutputRow;
extern bool showLimitRow;
extern bool showVoltageRow;
extern bool showCurrentRow;
extern bool showCalcRow;
extern bool showCalRow;
extern bool showTempRow;

#endif