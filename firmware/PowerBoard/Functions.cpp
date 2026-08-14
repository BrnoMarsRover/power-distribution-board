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

    Serial.print("OCP         : ");
    Serial.print(ocpEnabled ? "ENABLED" : "DISABLED");
    if (branchOverLimit[index]) {
        Serial.print("  >>> OVER LIMIT (");
        Serial.print(branchOverLimitCount[index]);
        Serial.print(" samples)");
    }
    Serial.println();
    Serial.print("Trips       : "); Serial.println(branchTripCounts[index]);

    Serial.print("Voltage [V] : "); Serial.println(vbusValues[index], 3);
    Serial.print("Current [mA]: "); Serial.println(currentValues[index], 3);
    Serial.print("Calc [mA]   : "); Serial.println(calcCurrentValues[index], 3);
    Serial.print("Cal [mA]    : "); Serial.println(calibratedCurrentValues[index], 3);
    Serial.print("Temp [C]    : "); Serial.println(tempValues[index], 1);
    Serial.println("======================================");
}

void printRawDump()
{
    Serial.println();
    Serial.println("=========================== RAW REGISTER DUMP ===========================");
    Serial.print("devicesFound: "); Serial.print(devicesFound);
    Serial.print("   INA_COUNT: "); Serial.println(INA_COUNT);
    if (devicesFound != INA_COUNT) {
        Serial.println("!! devicesFound != INA_COUNT - device indices no longer line up with");
        Serial.println("!! inaAddresses[]/inaNames[]/controlPins[], readings may be attributed");
        Serial.println("!! to the wrong branch.");
    }

    for (uint8_t i = 0; i < INA_COUNT; i++) {
        char buf[128];

        // Read straight from the chip. Registers are read back so that a failed write
        // in initDevice() becomes visible - writeWord() discards the I2C result, so a
        // NACKed configuration write is otherwise completely silent.
        uint16_t dieId   = INA.readRegister(INA_DIE_ID_REGISTER, i);
        uint16_t cfg     = INA.readRegister(INA_CONFIGURATION_REGISTER, i);
        uint16_t adccfg  = INA.readRegister(INA_ADC_CONFIGURATION_REGISTER, i);
        uint16_t cal     = INA.readRegister(INA_CALIBRATION_REGISTER, i);
        uint16_t vbusRaw = INA.getBusRaw(i);
        int16_t  vshRaw  = (int16_t)INA.getShuntRaw(i);
        int16_t  curRaw  = INA.getBusMicroAmpsRaw(i);
        int16_t  tmpRaw  = INA.getDieTemperatureRaw(i);

        uint32_t rShunt  = INA.getMicroOhmR(i);
        float    cLsb    = INA.getCurrentLSB(i);

        // What SHUNT_CAL should contain for this device, per the INA238 datasheet:
        // SHUNT_CAL = 819.2e6 * CURRENT_LSB * R_SHUNT
        uint16_t calExpect = (uint16_t)(819200000.0f * cLsb * ((float)rShunt / 1000000.0f));

        Serial.println("-------------------------------------------------------------------------");
        snprintf(buf, sizeof(buf), "[%u] %-3s  addr 0x%02X   configured: R=%lu uOhm  currentLSB=%.6f A",
                 i, inaNames[i], INA.getDeviceAddress(i),
                 (unsigned long)rShunt, cLsb);
        Serial.println(buf);

        snprintf(buf, sizeof(buf), "    DEVICE_ID  0x%04X  %s", dieId,
                 (dieId == INA_DIE_ID_VALUE) ? "(INA238 ok)" : "(UNEXPECTED - read failed or wrong part)");
        Serial.println(buf);

        snprintf(buf, sizeof(buf), "    CONFIG     0x%04X  ADCRANGE=%u %s", cfg,
                 (unsigned)((cfg >> 4) & 0x01),
                 (cfg == 0x0000) ? "(as written)" : "(NOT what initDevice wrote - 0x0000)");
        Serial.println(buf);

        snprintf(buf, sizeof(buf), "    ADC_CONFIG 0x%04X  MODE=0x%X AVG=%u %s", adccfg,
                 (unsigned)((adccfg >> 12) & 0x0F), (unsigned)(adccfg & 0x07),
                 (adccfg == 0xAB6A) ? "(as written)" : "(NOT what initDevice wrote - 0xAB6A)");
        Serial.println(buf);

        snprintf(buf, sizeof(buf), "    SHUNT_CAL  0x%04X = %u   expected %u %s",
                 cal, cal, calExpect,
                 (cal == calExpect) ? "(match)" : "(MISMATCH)");
        Serial.println(buf);

        // Raw values plus the conversion the firmware applies, so the two can be
        // compared against an external meter without guessing at the scaling.
        snprintf(buf, sizeof(buf), "    VBUS   raw 0x%04X = %5u  x 3.125 mV = %9.3f V",
                 vbusRaw, vbusRaw, (float)vbusRaw * 3.125f / 1000.0f);
        Serial.println(buf);

        snprintf(buf, sizeof(buf), "    VSHUNT raw 0x%04X = %6d  x 5 uV    = %9.3f mV",
                 (uint16_t)vshRaw, vshRaw, (float)vshRaw * 5.0f / 1000.0f);
        Serial.println(buf);

        snprintf(buf, sizeof(buf), "    CURRENT raw 0x%04X = %6d  x LSB     = %9.3f mA  (saturates at +-32767)",
                 (uint16_t)curRaw, curRaw, (float)curRaw * cLsb * 1000.0f);
        Serial.println(buf);

        snprintf(buf, sizeof(buf), "    DIETEMP raw 0x%04X = %6d  >>4 x0.125 = %9.1f C",
                 (uint16_t)tmpRaw, tmpRaw, (float)(tmpRaw >> 4) * 0.125f);
        Serial.println(buf);

        // Shunt-derived current: independent of SHUNT_CAL and of the CURRENT register,
        // and the value the over-current check uses.
        snprintf(buf, sizeof(buf), "    shunt-derived current            = %9.3f mA  <- used by OCP",
                 ((float)vshRaw * 5.0f / 1000.0f) * 1000000.0f / (float)(rShunt ? rShunt : 1));
        Serial.println(buf);
    }
    Serial.println("=========================================================================");
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

    // Read-only diagnostic: unconverted registers plus a read-back of the config, so a
    // silently failed initDevice() write or an implausible raw value becomes visible.
    if (cmd == "RAW") {
        printRawDump();
        return;
    }

    // Over-current protection master switch. Default off - see Config.cpp.
    if (cmd == "OCP") {
        Serial.print("OCP IS ");
        Serial.println(ocpEnabled ? "ENABLED" : "DISABLED");
        return;
    }

    if (cmd == "OCP ON" || cmd == "OCP 1") {
        ocpEnabled = true;
        for (uint8_t i = 0; i < INA_COUNT; i++) branchOverLimitCount[i] = 0;
        Serial.println("OCP ENABLED - branches will be switched off above their limit");
        Serial.println("WARNING: bridged high-side drivers cannot be opened in hardware");
        return;
    }

    if (cmd == "OCP OFF" || cmd == "OCP 0") {
        ocpEnabled = false;
        // Release anything currently held off by a trip, so disabling protection does
        // not silently leave a branch dark.
        for (uint8_t i = 0; i < INA_COUNT; i++) {
            branchOverLimitCount[i] = 0;
            if (branchTripped[i] && !branchManualOff[i]) {
                branchTripped[i] = false;
                branchActive[i] = true;
                digitalWrite(controlPins[i], HIGH);
                strcpy(branchStatusMsgs[i], "OK");
            }
        }
        Serial.println("OCP DISABLED - limits are monitored and reported only");
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
    Serial.println("COMMANDS: ON | OFF | ON <U2..U6|2..6> | OFF <U2..U6|2..6>");
    Serial.println("          STATUS [<U2..U6|2..6>] | OCP [ON|OFF] | RAW");
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

        // The over-current check uses the SHUNT-DERIVED current, not the INA CURRENT
        // register. The register is scaled by current_LSB = maxBusAmps / 32768 with
        // maxBusAmps = 8.0 for every device, so it saturates at ~7.9998 A - below U2's
        // 15 A and U6's 12 A limits, which could therefore never be reached.
        // The shunt path has both more range (+-54.6 A on U2's 3 mOhm, +-32.8 A on the
        // 5 mOhm branches at ADCRANGE=0) and finer resolution (1.67 mA / 1.0 mA vs
        // 244 uA), so it is the better basis for protection on every branch.
        const float protectionCurrent_mA = calcCurrentValues[i];

        // OVER-LIMIT DETECTION - always evaluated, even with OCP disabled, so that an
        // overloaded branch is still visible in the table and in telemetry.
        if (physicallyActive && !branchManualOff[i] && protectionCurrent_mA > currentLimits_mA[i]) {
            if (branchOverLimitCount[i] < 255) branchOverLimitCount[i]++;
            branchOverLimit[i] = true;
        } else {
            branchOverLimitCount[i] = 0;
            branchOverLimit[i] = false;
        }

        // OCP - only ever switches anything when explicitly enabled. Default is off
        // because some high-side drivers on this board are bridged and cannot be
        // opened, so a software trip would report TRIPPED while current kept flowing.
        if (ocpEnabled) {
            if (branchOverLimit[i] && branchOverLimitCount[i] >= ocpTripSamples && !branchTripped[i]) {
                branchTripped[i] = true;
                branchActive[i] = false;
                digitalWrite(controlPins[i], LOW);
                branchTrippedTime[i] = currentTime;
                branchTripCounts[i]++;
                strcpy(branchStatusMsgs[i], "OCP TRIPPED");
            }

            // AUTO RECOVERY
            if (!branchManualOff[i] && branchTripped[i] && (currentTime - branchTrippedTime[i] >= 5000UL)) {
                branchActive[i] = true;
                branchTripped[i] = false;
                branchOverLimitCount[i] = 0;
                digitalWrite(controlPins[i], HIGH);
                strcpy(branchStatusMsgs[i], "OK");
            }
        } else if (branchOverLimit[i]) {
            strcpy(branchStatusMsgs[i], "OVER LIMIT (OCP OFF)");
        }
    }
}

void printTable()
{
    for (int i = 0; i < 20; i++) Serial.println();

    Serial.println("=================================================================================================");
    Serial.print("| Parameter  |    U2    |    U3    |    U4    |    U5    |    U6    |   OCP: ");
    Serial.println(ocpEnabled ? "ENABLED  |" : "DISABLED |");
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

    // Over-limit is reported whether or not OCP is enabled, so an overloaded branch is
    // never invisible just because protection is switched off.
    Serial.print("| OverLimit  |");
    for (uint8_t i = 0; i < INA_COUNT; i++) {
        if (branchOverLimit[i]) Serial.print("  >LIMIT |");
        else Serial.print("    ok   |");
    }
    Serial.println();
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