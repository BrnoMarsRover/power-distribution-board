/*!
 * Arduino Library for accessing the INA238 power measurement devices\n\n
 * See main library header file "INA.h" for details and license information
 */

#include "INA238.h"
#include <Wire.h>

inaDet::inaDet() {}

INA_Class::INA_Class(uint8_t expectedDevices) : _expectedDevices(expectedDevices) {
  _DeviceArray = new inaDet[_expectedDevices];
}

INA_Class::~INA_Class() {
  if (_expectedDevices) { delete[] _DeviceArray; }
}

uint16_t INA_Class::readWord(const uint8_t addr, const uint8_t deviceAddress) const {
  Wire.beginTransmission(deviceAddress);
  Wire.write(addr);
  Wire.endTransmission();
  delayMicroseconds(I2C_DELAY);
  Wire.requestFrom(deviceAddress, (uint8_t)2);
  return ((uint16_t)Wire.read() << 8) | Wire.read();
}

uint32_t INA_Class::read3Bytes(const uint8_t addr, const uint8_t deviceAddress) const {
  Wire.beginTransmission(deviceAddress);
  Wire.write(addr);
  Wire.endTransmission();
  delayMicroseconds(I2C_DELAY);
  Wire.requestFrom(deviceAddress, (uint8_t)3);
  return ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | ((uint32_t)Wire.read());
}

void INA_Class::writeWord(const uint8_t addr, const uint16_t data, const uint8_t deviceAddress) const {
  Wire.beginTransmission(deviceAddress);
  Wire.write(addr);
  Wire.write((uint8_t)(data >> 8));
  Wire.write((uint8_t)data);
  Wire.endTransmission();
  delayMicroseconds(I2C_DELAY);
}

void INA_Class::loadInaFromIndex(const uint8_t deviceNumber) {
  if (deviceNumber == _currentINA || deviceNumber > _DeviceCount) return;
  
  _currentINA = deviceNumber;
  ina = _DeviceArray[deviceNumber];
}

void INA_Class::saveInaToIndex(const uint8_t deviceNumber) {
  if (deviceNumber > _DeviceCount) return;

  _DeviceArray[deviceNumber] = ina;
}

void INA_Class::setI2CSpeed(const uint32_t i2cSpeed) const {
  Wire.setClock(i2cSpeed);
}

uint8_t INA_Class::begin(const uint16_t maxBusAmps, const uint32_t microOhmR) {
  uint16_t originalRegister, tempRegister;
  if (_DeviceCount == 0)  // Enumerate all devices on first call
  {
    uint8_t maxDevices = 16;
    Wire.begin();

    for (uint8_t deviceAddress = 0x40; deviceAddress <= 0x4F; deviceAddress++)  // Loop for each I2C addr
    {
      Wire.beginTransmission(deviceAddress);
      uint8_t good = Wire.endTransmission();
      if (good == 0 && _DeviceCount < maxDevices)
      {
        originalRegister = readWord(INA_CONFIGURATION_REGISTER, deviceAddress);
        writeWord(INA_CONFIGURATION_REGISTER, INA_RESET_DEVICE, deviceAddress);  // Force reset
        tempRegister = readWord(INA_CONFIGURATION_REGISTER, deviceAddress);      // Read reset reg.
        if (tempRegister == INA_RESET_DEVICE)  // If the register wasn't reset then not an INA
        {
          Serial.print("Found I2C device which is not INA device on address: "); Serial.println(deviceAddress);
          writeWord(INA_CONFIGURATION_REGISTER, originalRegister, deviceAddress);  // restore value
        } else {
          tempRegister = readWord(INA_DIE_ID_REGISTER, deviceAddress);  // Read the INA high-reg
          if(tempRegister == INA_DIE_ID_VALUE) {
            Serial.print("Found INA238 device on address: "); Serial.println(deviceAddress);
          } else {
            Serial.print("Found invalid INA device on address: "); Serial.println(deviceAddress);
            break;
          }

          ina.operatingMode = INA_DEFAULT_OPERATING_MODE;  // Default to continuous mode
          ina.address    = deviceAddress;
          ina.maxBusAmps = maxBusAmps > 1022 ? 1022 : maxBusAmps;  // Clamp to maximum of 1022A
          ina.microOhmR  = microOhmR;
          ina.busVoltage_LSB       = INA_BUS_VOLTAGE_LSB;
          ina.shuntVoltage_LSB     = INA_SHUNT_VOLTAGE_LSB_ADC0;
          ina.current_LSB          = ((float)ina.maxBusAmps) / (float)(32768.0f);
          ina.power_LSB            = 0.2f * ina.current_LSB;
          ina.temp_LSB             = INA_TEMP_LSB;

          initDevice(_DeviceCount);

          saveInaToIndex(_DeviceCount);
          _DeviceCount += 1;
        }
      }
    }
  }

  _currentINA = UINT8_MAX;  // Force read on next call
  return _DeviceCount;
}

void INA_Class::initDevice(const uint8_t deviceNumber) {

  /* Determine optimal programmable gain with maximum accuracy so no chance of an overflow */
  uint8_t programmableGain = 0;
  uint16_t maxShuntmV = ina.maxBusAmps * ina.microOhmR / 1000;  // Compute maximum shunt mV
  Serial.print("Computed maximum shunt voltage drop [mV]: "); Serial.println(maxShuntmV);
  if (maxShuntmV <= 40)
    programmableGain = 1;  // +-40.96mV
  else
    programmableGain = 0;  // +-163.84mV

  uint16_t tempRegister = readWord(INA_CONFIGURATION_REGISTER, ina.address);
  tempRegister &= INA_CONFIG_ADC_RANGE_MASK;
  tempRegister |= programmableGain << 4;
  writeWord(INA_CONFIGURATION_REGISTER, tempRegister, ina.address);
 
  float shunt = float(ina.microOhmR) / 1000000.0f;
  float calibration = 819000000.2f * float(ina.current_LSB) * shunt;
  if(programmableGain == 1) {
    ina.shuntVoltage_LSB = INA_SHUNT_VOLTAGE_LSB_ADC1;
  } else {
    ina.shuntVoltage_LSB = INA_SHUNT_VOLTAGE_LSB_ADC0;
    calibration *= 4;
  }
  writeWord(INA_CALIBRATION_REGISTER, uint16_t(calibration), ina.address);

  Serial.print("Shunt: "); Serial.println(shunt * 1000.0f);
  Serial.print("Current LSB: "); Serial.println(ina.current_LSB * 1000000.0f);
  Serial.print("Calibration value: "); Serial.println(uint16_t(calibration));
}

void INA_Class::setBusConversion(const uint32_t convTime, const uint8_t deviceNumber) {
  for (uint8_t i = 0; i < _DeviceCount; i++)
  {
    if (deviceNumber == UINT8_MAX || deviceNumber % _DeviceCount == i)
    {
      loadInaFromIndex(i);
      uint16_t convRate;
      if (convTime >= 4120)
        convRate = 7;
      else if (convTime >= 2074)
        convRate = 6;
      else if (convTime >= 1052)
        convRate = 5;
      else if (convTime >= 540)
        convRate = 4;
      else if (convTime >= 280)
        convRate = 3;
      else if (convTime >= 150)
        convRate = 2;
      else if (convTime >= 84)
        convRate = 1;
      else
        convRate = 0;
      uint16_t configRegister = readWord(INA_ADC_CONFIGURATION_REGISTER, ina.address);
      configRegister &= ~INA_CONFIG_BADC_MASK;
      configRegister |= convRate << 9;
      writeWord(INA_ADC_CONFIGURATION_REGISTER, configRegister, ina.address);
    }
  }
}

void INA_Class::setShuntConversion(const uint32_t convTime, const uint8_t deviceNumber) {
  for (uint8_t i = 0; i < _DeviceCount; i++) {
    if (deviceNumber == UINT8_MAX || deviceNumber % _DeviceCount == i)
    {
      loadInaFromIndex(i);
      uint16_t convRate;
      if (convTime >= 4120)
        convRate = 7;
      else if (convTime >= 2074)
        convRate = 6;
      else if (convTime >= 1052)
        convRate = 5;
      else if (convTime >= 540)
        convRate = 4;
      else if (convTime >= 280)
        convRate = 3;
      else if (convTime >= 150)
        convRate = 2;
      else if (convTime >= 84)
        convRate = 1;
      else
        convRate = 0;
      uint16_t configRegister = readWord(INA_ADC_CONFIGURATION_REGISTER, ina.address);
      configRegister &= ~INA_CONFIG_SADC_MASK;
      configRegister |= convRate << 6;
      writeWord(INA_ADC_CONFIGURATION_REGISTER, configRegister, ina.address);
    }
  }
}

void INA_Class::setTempConversion(const uint32_t convTime, const uint8_t deviceNumber) {
  for (uint8_t i = 0; i < _DeviceCount; i++) {
    if (deviceNumber == UINT8_MAX || deviceNumber % _DeviceCount == i)
    {
      loadInaFromIndex(i);
      uint16_t convRate;
      if (convTime >= 4120)
        convRate = 7;
      else if (convTime >= 2074)
        convRate = 6;
      else if (convTime >= 1052)
        convRate = 5;
      else if (convTime >= 540)
        convRate = 4;
      else if (convTime >= 280)
        convRate = 3;
      else if (convTime >= 150)
        convRate = 2;
      else if (convTime >= 84)
        convRate = 1;
      else
        convRate = 0;
      uint16_t configRegister = readWord(INA_ADC_CONFIGURATION_REGISTER, ina.address);
      configRegister &= ~INA_CONFIG_TADC_MASK;
      configRegister |= convRate << 3;
      writeWord(INA_ADC_CONFIGURATION_REGISTER, configRegister, ina.address);
    }
  }
}

void INA_Class::setAdcRange(const bool adcRange, const uint8_t deviceNumber) {
  uint16_t configRegister;
  for (uint8_t i = 0; i < _DeviceCount; i++) {
    if (deviceNumber == UINT8_MAX || deviceNumber % _DeviceCount == i)
    {
      loadInaFromIndex(i);
      configRegister = readWord(INA_CONFIGURATION_REGISTER, ina.address);
      configRegister &= ~INA_CONFIG_ADC_RANGE_MASK;
      configRegister |= adcRange << 4;
      writeWord(INA_CONFIGURATION_REGISTER, configRegister, ina.address);

      float shunt = float(ina.microOhmR) / 1000000.0f;
      float calibration = 819000000.2f * float(ina.current_LSB) * shunt;
      if(adcRange == 0) {
        ina.shuntVoltage_LSB = INA_SHUNT_VOLTAGE_LSB_ADC0;
      } else {
        ina.shuntVoltage_LSB = INA_SHUNT_VOLTAGE_LSB_ADC1;
        calibration *= 4;
      }
      writeWord(INA_CALIBRATION_REGISTER, uint16_t(calibration), ina.address);
    }
  }
}

uint8_t INA_Class::getDeviceAddress(const uint8_t deviceNumber) {
  if (deviceNumber > _DeviceCount) return 0;
  loadInaFromIndex(deviceNumber);
  return (ina.address);
}

void INA_Class::reset(const uint8_t deviceNumber) {
  for (uint8_t i = 0; i < _DeviceCount; i++)
  {
    if (deviceNumber == UINT8_MAX || deviceNumber % _DeviceCount == i)
    {
      loadInaFromIndex(i);
      writeWord(INA_CONFIGURATION_REGISTER, INA_RESET_DEVICE, ina.address);
      initDevice(i);
    }
  }
}

void INA_Class::setMode(const uint8_t mode, const uint8_t deviceNumber) {
  for (uint8_t i = 0; i < _DeviceCount; i++)
  {
    if (deviceNumber == UINT8_MAX || deviceNumber % _DeviceCount == i)
    {
      loadInaFromIndex(i);
      uint16_t configRegister = readWord(INA_ADC_CONFIGURATION_REGISTER, ina.address);
      configRegister &= ~INA_CONFIG_MODE_MASK;
      ina.operatingMode = B00001111 & mode;
      saveInaToIndex(i);
      configRegister |= ina.operatingMode << 12;
      writeWord(INA_ADC_CONFIGURATION_REGISTER, configRegister, ina.address);
    }
  }
}

void INA_Class::setAveraging(const uint16_t averages, const uint8_t deviceNumber) {
  for (uint8_t i = 0; i < _DeviceCount; i++)
  {
    if (deviceNumber == UINT8_MAX || deviceNumber % _DeviceCount == i)
    {
      loadInaFromIndex(i);
      uint16_t averageIndex;
      if (averages >= 1024)
        averageIndex = 7;
      else if (averages >= 512)
        averageIndex = 6;
      else if (averages >= 256)
        averageIndex = 5;
      else if (averages >= 128)
        averageIndex = 4;
      else if (averages >= 64)
        averageIndex = 3;
      else if (averages >= 16)
        averageIndex = 2;
      else if (averages >= 4)
        averageIndex = 1;
      else
        averageIndex = 0;
      uint16_t configRegister = readWord(INA_CONFIGURATION_REGISTER, ina.address);
      configRegister &= ~INA_CONFIG_AVG_MASK;
      configRegister |= averageIndex;
      writeWord(INA_CONFIGURATION_REGISTER, configRegister, ina.address);
    }
  }
}

uint16_t INA_Class::getBusMilliVolts(const uint8_t deviceNumber) {
  uint16_t busVoltage = getBusRaw(deviceNumber);
  busVoltage = busVoltage * ina.busVoltage_LSB / 1000.0f;
  return (busVoltage);
}

uint16_t INA_Class::getBusRaw(const uint8_t deviceNumber) {
  loadInaFromIndex(deviceNumber);
  uint16_t raw = readWord(INA_BUS_VOLTAGE_REGISTER, ina.address);
  //TODO: Properly handle triggered mode
  // if (!bitRead(ina.operatingMode, 2) && bitRead(ina.operatingMode, 1))  // Triggered & bus active
  // {
  //   uint16_t configRegister =
  //       readWord(INA_CONFIGURATION_REGISTER, ina.address);               // Get current value
  //   writeWord(INA_CONFIGURATION_REGISTER, configRegister, ina.address);  // Write to trigger next
  // }
  return (raw);
}

float INA_Class::getShuntMicroVolts(const uint8_t deviceNumber) {
  float shuntVoltage = getShuntRaw(deviceNumber);
  shuntVoltage = shuntVoltage * float(ina.shuntVoltage_LSB) / 1000.0f;
  return (shuntVoltage);
}

uint16_t INA_Class::getShuntRaw(const uint8_t deviceNumber) {
  uint16_t raw;
  loadInaFromIndex(deviceNumber);
  raw = readWord(INA_SHUNT_VOLTAGE_REGISTER, ina.address);
  //TODO: Properly handle triggered mode
  // if (!bitRead(ina.operatingMode, 2) && bitRead(ina.operatingMode, 0))  // Triggered & shunt active
  // {
  //   uint16_t configRegister = readWord(INA_CONFIGURATION_REGISTER, ina.address);  // Get current reg
  //   writeWord(INA_CONFIGURATION_REGISTER, configRegister, ina.address);  // Write to trigger next
  // }  // of if-then triggered mode enabled
  return (raw);
}

float INA_Class::getBusMicroAmps(const uint8_t deviceNumber) {
  loadInaFromIndex(deviceNumber);
  float microAmps = getBusMicroAmpsRaw(deviceNumber);
  Serial.print("Raw microamps reading: "); Serial.println(microAmps);
  Serial.print("Current LSB: "); Serial.println(float(ina.current_LSB));
  microAmps *= float(ina.current_LSB);
  return (microAmps);
}

int16_t INA_Class::getBusMicroAmpsRaw(const uint8_t deviceNumber) {
  int16_t raw;
  loadInaFromIndex(deviceNumber);
  raw = readWord(INA_CURRENT_REGISTER, ina.address);
  Serial.print("Raw reading: "); Serial.println(raw);
  return (raw);
}

float INA_Class::getDieTemperature(const uint8_t deviceNumber) {
  loadInaFromIndex(deviceNumber);
  float temp = getDieTemperatureRaw(deviceNumber);
  temp *= ina.temp_LSB / 1000.0f;
  return (temp);
}

int16_t INA_Class::getDieTemperatureRaw(const uint8_t deviceNumber) {
  int16_t raw;
  loadInaFromIndex(deviceNumber);
  raw = readWord(INA_TEMP_REGISTER, ina.address) >> 4;
  return (raw);
}



