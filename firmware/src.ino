#include <Wire.h>
#include "INA238.h"

INA_Class INA(5);
volatile uint8_t devicesFound = UINT8_MAX;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 10000UL);

  Serial.println("Initializing...");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);

  while (devicesFound == UINT8_MAX)  // Loop until we find the first device
  {
    devicesFound = INA.begin(5, 3000);  // +/- 1 Amps maximum for 0.003 Ohm resistor
    for(uint8_t i = 0; i < devicesFound; i++) 
    {
      //INA.reset(deviceNumber);
    }

    if (devicesFound == UINT8_MAX) {
      Serial.println("No INA found. Waiting 5s and retrying...");
      delay(5000);
    }
    Serial.print("Number of found INA238 devices: "); Serial.println(devicesFound);
  }

  INA.setAveraging(1, 0);
  INA.setBusConversion(1052, 0);
  INA.setShuntConversion(1052, 0);
  INA.setTempConversion(1052, 0);
  //INA.setAdcRange(0, 0);
  INA.setMode(INA_MODE_CONTINUOUS_ALL, 0);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(13, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(10, HIGH);
  digitalWrite(9, HIGH);
  delay(2000);

  float shuntVoltage = INA.getShuntMicroVolts(0);
  Serial.println("Gate opened");
  Serial.print("Shunt voltage [uV]: "); Serial.println(shuntVoltage);
  uint16_t busVoltage = INA.getBusMilliVolts(0);
  Serial.print("Bus voltage [mV]: "); Serial.println(busVoltage);
  float busCurrent = INA.getBusMicroAmps(0);
  Serial.print("Bus current [uA]: "); Serial.println(busCurrent);
  Serial.print("Should report [mA]: "); Serial.println(float(shuntVoltage) / 1000.0f / 0.003f);
  float dieTemp = INA.getDieTemperature(0);
  Serial.print("Die temperature [°C]: "); Serial.println(dieTemp);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(13, LOW);
  digitalWrite(12, LOW);
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
  digitalWrite(9, LOW);
  delay(2000);

  Serial.println("Gate closed");
  shuntVoltage = INA.getShuntMicroVolts(0);
  Serial.print("Shunt voltage [uV]: "); Serial.println(shuntVoltage);
  busVoltage = INA.getBusMilliVolts(0);
  Serial.print("Bus voltage [mV]: "); Serial.println(busVoltage);
  busCurrent = INA.getBusMicroAmps(0);
  Serial.print("Bus current [uA]: "); Serial.println(busCurrent);
  Serial.print("Should report [mA]: "); Serial.println(float(shuntVoltage) / 1000.0f / 0.003f);
  dieTemp = INA.getDieTemperature(0);
  Serial.print("Die temperature [°C]: "); Serial.println(dieTemp);
}