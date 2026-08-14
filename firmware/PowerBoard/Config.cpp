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

// Limits derived from the hardware they protect, using the NOMINAL rail voltage
// (the measured voltage sags under load, which would move the limit around).
// U3..U6 each sit behind a 60 W DC-DC, so the limit is that converter's continuous
// rating: I = 60 W / V_nominal. The previous values allowed 19..62 % more power than
// the converters can deliver - e.g. U3 at 4000 mA on a 24 V rail is ~97 W.
//
// U2 is the battery feed and has no converter of its own; 15 A is left as it was.
// It is also the only limit above the 8 A ceiling of the CURRENT register, which is
// why the over-current check now uses the shunt-derived value (see readINAData).
const float currentLimits_mA[INA_COUNT] = {
    15000.0f, // U2  battery feed, unchanged (no DC-DC, 3 mOhm shunt)
     2500.0f, // U3  60 W / 24 V   (was 4000 -> ~97 W through a 60 W supply)
     4000.0f, // U4  60 W / 15 V   (was 5000 -> ~76 W)
     5000.0f, // U5  60 W / 12 V   (was 6000 -> ~73 W)
    12000.0f  // U6  60 W /  5 V   (was 14000 -> ~71 W)
};

// OVER-CURRENT PROTECTION MASTER SWITCH.
// Default OFF: on this board some high-side drivers were bridged after inrush current
// tripped them, so those branches cannot actually be opened in hardware. Tripping them
// in software would only produce a misleading "TRIPPED" state while current keeps
// flowing. With this false the firmware still MEASURES and still reports an over-limit
// condition (see branchOverLimit) - it just never switches anything off.
// Toggle at runtime with "OCP ON" / "OCP OFF", query with "OCP".
bool ocpEnabled = false;

// Consecutive over-limit samples required before acting, once OCP is enabled.
// The loop samples every 500 ms, so 2 means ~1 s of sustained overload. This exists
// so that inrush - the thing that damaged the drivers in the first place - cannot
// nuisance-trip a branch.
const uint8_t ocpTripSamples = 2;

bool showAddressRow = true;
bool showOnlineRow = true;
bool showOutputRow = true;
bool showLimitRow = true;
bool showVoltageRow = true;
bool showCurrentRow = true;
bool showCalcRow = true;
bool showCalRow = true;
bool showTempRow = true;