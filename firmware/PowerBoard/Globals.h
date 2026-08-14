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

extern uint32_t branchTrippedTime[INA_COUNT];

extern char branchStatusMsgs[INA_COUNT][64];

extern INA_Class INA;

extern volatile uint8_t devicesFound;

#endif