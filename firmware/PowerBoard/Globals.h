#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include "INA238.h"

#define INA_COUNT 5

extern bool inaOnline[INA_COUNT];

extern float vbusValues[INA_COUNT];
extern float vshuntValues[INA_COUNT];
extern float currentValues[INA_COUNT];
extern float calcCurrentValues[INA_COUNT];
extern float calibratedCurrentValues[INA_COUNT];
extern float tempValues[INA_COUNT];

extern bool branchActive[INA_COUNT];
extern bool branchManualOff[INA_COUNT];
extern bool branchTripped[INA_COUNT];

// Observation, not action: set whenever the measured current exceeds the branch limit,
// whether or not ocpEnabled is true. On a board where a bridged driver cannot be
// opened, this is the only thing that tells you a branch is being overloaded.
extern bool branchOverLimit[INA_COUNT];
// Consecutive over-limit samples so far; reset as soon as the current drops back.
extern uint8_t branchOverLimitCount[INA_COUNT];
// Cumulative OCP trip count, never reset - lets a host notice a trip it was not
// watching for, including one that already auto-recovered.
extern uint32_t branchTripCounts[INA_COUNT];

extern uint32_t branchTrippedTime[INA_COUNT];

extern char branchStatusMsgs[INA_COUNT][64];

extern INA_Class INA;

extern volatile uint8_t devicesFound;

#endif