#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>

void initializeSystem();
void processSerialCommand();
void readINAData();
void printTable();
void setBranch(uint8_t index, bool state);
void printSingleStatus(uint8_t index);
void printRawDump();
void printOutput();
void printTelemetryJson();
void emitEventJson(const char* event, uint8_t index);
void replyOk(const String& cmd, const char* text);
void replyErr(const String& cmd, const char* text);
int8_t getDeviceIndexByAddress(uint8_t addr);

#endif