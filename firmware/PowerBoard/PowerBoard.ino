#include <Wire.h>
#include "INA238.h"

#include "Globals.h"
#include "Config.h"
#include "Functions.h"

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 10000UL);

    initializeSystem();
}

void loop()
{
    processSerialCommand();
    readINAData();
    printOutput();   // JSON by default, human table after the HUMAN command
    delay(500);
}