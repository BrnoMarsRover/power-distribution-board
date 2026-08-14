#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>

void initializeSystem();
void processSerialCommand();
void readINAData();
void printTable();
void setBranch(uint8_t index, bool state);
void printSingleStatus(uint8_t index);
int8_t getDeviceIndexByAddress(uint8_t addr);

#endif