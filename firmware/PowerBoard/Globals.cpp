#include "Globals.h"

bool inaOnline[INA_COUNT];

float vbusValues[INA_COUNT];
float vshuntValues[INA_COUNT];
float currentValues[INA_COUNT];
float calcCurrentValues[INA_COUNT];
float calibratedCurrentValues[INA_COUNT];
float tempValues[INA_COUNT];

bool branchActive[INA_COUNT] = {
    true, true, true, true, true
};

bool branchManualOff[INA_COUNT] = {
    false, false, false, false, false
};

bool branchTripped[INA_COUNT] = {
    false, false, false, false, false
};

uint32_t branchTrippedTime[INA_COUNT] = {
    0, 0, 0, 0, 0
};

char branchStatusMsgs[INA_COUNT][64];

INA_Class INA(INA_COUNT);

volatile uint8_t devicesFound = UINT8_MAX;