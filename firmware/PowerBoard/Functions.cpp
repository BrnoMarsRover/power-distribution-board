#include "Functions.h"
#include <Wire.h>
#include "Globals.h"
#include "Config.h"

int8_t getDeviceIndexByAddress(uint8_t addr)
{
    for (uint8_t i = 0; i < devicesFound; i++)
    {
        if (INA.getDeviceAddress(i) == addr)
        {
            return i;
        }
    }
    return -1;
}

void setBranch(uint8_t index, bool state)
{
    if (index >= INA_COUNT) return;

    branchActive[index] = state;
    branchManualOff[index] = !state;
    branchTripped[index] = false;

    digitalWrite(controlPins[index], state ? HIGH : LOW);

    if (state) {
        strcpy(branchStatusMsgs[index], "MANUAL ON");
    } else {
        strcpy(branchStatusMsgs[index], "MANUAL OFF");
    }
}

void printSingleStatus(uint8_t index)
{
    if (index >= INA_COUNT) return;

    bool physicallyActive = (index == 0) ? branchActive[0] : (branchActive[index] && branchActive[0]);

    Serial.println();
    Serial.println("======================================");
    Serial.print("Branch      : "); Serial.println(inaNames[index]);
    Serial.print("Address     : 0x"); Serial.println(inaAddresses[index], HEX);
    Serial.print("Online      : "); Serial.println(inaOnline[index] ? "YES" : "NO");
    Serial.print("Output      : ");
    if (branchTripped[index]) Serial.println("TRIPPED");
    else if (physicallyActive) Serial.println("ACTIVE");
    else Serial.println("OFF");

    Serial.print("Voltage [V] : "); Serial.println(vbusValues[index], 3);
    Serial.print("Current [mA]: "); Serial.println(currentValues[index], 3);
    Serial.print("Calc [mA]   : "); Serial.println(calcCurrentValues[index], 3);
    Serial.print("Cal [mA]    : "); Serial.println(calibratedCurrentValues[index], 3);
    Serial.print("Temp [C]    : "); Serial.println(tempValues[index], 1);
    Serial.println("======================================");
}

void processSerialCommand()
{
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "OFF" || cmd == "0") {
        for (uint8_t i = 0; i < INA_COUNT; i++) setBranch(i, false);
        Serial.println("ALL BRANCHES OFF");
        return;
    }

    if (cmd == "ON" || cmd == "1") {
        for (uint8_t i = 0; i < INA_COUNT; i++) setBranch(i, true);
        Serial.println("ALL BRANCHES ON");
        return;
    }

    if (cmd == "STATUS") {
        for (uint8_t i = 0; i < INA_COUNT; i++) printSingleStatus(i);
        return;
    }

    for (uint8_t i = 0; i < INA_COUNT; i++) {
        String uName = String(inaNames[i]);
        String shortNum = String(i + 2);

        if (cmd == ("OFF " + uName) || cmd == ("OFF " + shortNum) || cmd == ("0" + shortNum)) {
            setBranch(i, false);
            Serial.print(uName); Serial.println(" OFF");
            return;
        }

        if (cmd == ("ON " + uName) || cmd == ("ON " + shortNum) || cmd == ("1" + shortNum)) {
            setBranch(i, true);
            Serial.print(uName); Serial.println(" ON");
            return;
        }

        if (cmd == ("STATUS " + uName) || cmd == ("STATUS " + shortNum)) {
            printSingleStatus(i);
            return;
        }
    }
    Serial.print("UNKNOWN COMMAND: "); Serial.println(cmd);
}

void initializeSystem()
{
    Serial.println("\n======================================\nPOWER BOARD MONITOR START\n======================================");
    for (uint8_t i = 0; i < INA_COUNT; i++) {
        pinMode(controlPins[i], OUTPUT);
        digitalWrite(controlPins[i], LOW); // Štartujeme bezpečne vo vypnutom stave
        branchActive[i] = false;
        branchManualOff[i] = true;
        strcpy(branchStatusMsgs[i], "INIT OFF");
    }

    Wire1.setSDA(26);
    Wire1.setSCL(27);
    Wire1.begin();
    Wire1.setClock(100000);
    delay(100);

    while (devicesFound == UINT8_MAX || devicesFound == 0) {
        // Inicializujeme knižnicu s parametrami pre U2 (8A, 5000 uOhm)
        devicesFound = INA.begin(8.0f, 5000);

        if (devicesFound == UINT8_MAX || devicesFound == 0) {
            Serial.println("No INA238 found");
            delay(500);
        }
    }

    Serial.print("INA238 FOUND: "); Serial.println(devicesFound);

    for (uint8_t i = 0; i < INA_COUNT; i++) {
        branchActive[i] = true;
        branchManualOff[i] = false;
        digitalWrite(controlPins[i], HIGH);
        strcpy(branchStatusMsgs[i], "OK");
    }
    delay(100);
}

void readINAData()
{
    uint32_t currentTime = millis();

    for (uint8_t i = 0; i < INA_COUNT; i++) {
        inaOnline[i] = true;

        vbusValues[i] = (float)INA.getBusMilliVolts(i) / 1000.0f;
        vshuntValues[i] = (float)INA.getShuntMicroVolts(i) / 1000.0f;
        currentValues[i] = (float)INA.getBusMicroAmps(i) / 1000.0f;

        calcCurrentValues[i] = (vshuntValues[i] * 1000000.0f) / shuntResistors_uOhm[i];
        calibratedCurrentValues[i] = currentValues[i] * currentCalibrationFactors[i];
        tempValues[i] = INA.getDieTemperature(i);

        bool physicallyActive = (i == 0) ? branchActive[0] : (branchActive[i] && branchActive[0]);

        // OCP
        if (physicallyActive && !branchManualOff[i] && calibratedCurrentValues[i] > currentLimits_mA[i]) {
            branchTripped[i] = true;
            branchActive[i] = false;
            digitalWrite(controlPins[i], LOW);
            branchTrippedTime[i] = currentTime;
            strcpy(branchStatusMsgs[i], "OCP TRIPPED");
        }

        // AUTO RECOVERY
        if (!branchManualOff[i] && branchTripped[i] && (currentTime - branchTrippedTime[i] >= 5000UL)) {
            branchActive[i] = true;
            branchTripped[i] = false;
            digitalWrite(controlPins[i], HIGH);
            strcpy(branchStatusMsgs[i], "OK");
        }
    }
}

void printTable()
{
    for (int i = 0; i < 20; i++) Serial.println();

    Serial.println("=================================================================================================");
    Serial.println("| Parameter  |    U2    |    U3    |    U4    |    U5    |    U6    |");
    Serial.println("=================================================================================================");

    if (showAddressRow) {
        Serial.print("| Address    |");
        for (uint8_t i = 0; i < INA_COUNT; i++) {
            char buffer[12];
            snprintf(buffer, sizeof(buffer), "   0x%02X ", inaAddresses[i]);
            Serial.print(buffer); Serial.print("|");
        }
        Serial.println();
    }
    Serial.println("-------------------------------------------------------------------------------------------------");

    if (showOnlineRow) {
        Serial.print("| Online     |");
        for (uint8_t i = 0; i < INA_COUNT; i++) Serial.print(inaOnline[i] ? "   YES   |" : "    NO   |");
        Serial.println();
    }
    Serial.println("-------------------------------------------------------------------------------------------------");

    if (showOutputRow) {
        Serial.print("| Output SW  |");
        for (uint8_t i = 0; i < INA_COUNT; i++) {
            bool physicallyActive = (i == 0) ? branchActive[0] : (branchActive[i] && branchActive[0]);
            if (branchTripped[i]) Serial.print(" TRIPPED |");
            else if (physicallyActive) Serial.print("  ACTIVE |");
            else Serial.print("    OFF  |");
        }
        Serial.println();
    }
    Serial.println("-------------------------------------------------------------------------------------------------");

    if (showLimitRow) {
        Serial.print("| Limit [mA] |");
        for (uint8_t i = 0; i < INA_COUNT; i++) {
            char buffer[12];
            snprintf(buffer, sizeof(buffer), "%10.1f", currentLimits_mA[i]);
            Serial.print(buffer); Serial.print("|");
        }
        Serial.println();
    }
    Serial.println("-------------------------------------------------------------------------------------------------");

    if (showVoltageRow) {
        Serial.print("| VBus [V]   |");
        for (uint8_t i = 0; i < INA_COUNT; i++) {
            char buffer[12];
            snprintf(buffer, sizeof(buffer), "%10.3f", vbusValues[i]);
            Serial.print(buffer); Serial.print("|");
        }
        Serial.println();
    }
    Serial.println("-------------------------------------------------------------------------------------------------");

    if (showCurrentRow) {
        Serial.print("| Current[mA]|");
        for (uint8_t i = 0; i < INA_COUNT; i++) {
            char buffer[12];
            snprintf(buffer, sizeof(buffer), "%10.3f", currentValues[i]);
            Serial.print(buffer); Serial.print("|");
        }
        Serial.println();
    }
    Serial.println("-------------------------------------------------------------------------------------------------");

    if (showCalcRow) {
        Serial.print("| Calc[mA]   |");
        for (uint8_t i = 0; i < INA_COUNT; i++) {
            char buffer[12];
            snprintf(buffer, sizeof(buffer), "%10.3f", calcCurrentValues[i]);
            Serial.print(buffer); Serial.print("|");
        }
        Serial.println();
    }
    Serial.println("-------------------------------------------------------------------------------------------------");

    if (showCalRow) {
        Serial.print("| Cal[mA]    |");
        for (uint8_t i = 0; i < INA_COUNT; i++) {
            char buffer[12];
            snprintf(buffer, sizeof(buffer), "%10.3f", calibratedCurrentValues[i]);
            Serial.print(buffer); Serial.print("|");
        }
        Serial.println();
    }
    Serial.println("-------------------------------------------------------------------------------------------------");

    if (showTempRow) {
        Serial.print("| Temp [C]   |");
        for (uint8_t i = 0; i < INA_COUNT; i++) {
            char buffer[12];
            snprintf(buffer, sizeof(buffer), "%10.1f", tempValues[i]);
            Serial.print(buffer); Serial.print("|");
        }
        Serial.println();
    }
    Serial.println("=================================================================================================");
}