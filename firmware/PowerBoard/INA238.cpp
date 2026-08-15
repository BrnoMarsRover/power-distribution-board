#include "INA238.h"

inaDet::inaDet() {
  operatingMode = INA_MODE_CONTINUOUS_ALL;
  address = 0;
  maxBusAmps = 0;
  microOhmR = 0;
  shuntVoltage_LSB = INA_SHUNT_VOLTAGE_LSB_ADC0;
  busVoltage_LSB = INA_BUS_VOLTAGE_LSB;
  current_LSB = 0.0f;
  power_LSB = 0.0f;
  temp_LSB = INA_TEMP_LSB;
}

INA_Class::INA_Class(uint8_t expectedDevices) {
  _expectedDevices = expectedDevices;
  _DeviceArray = new inaDet[_expectedDevices];
  _DeviceCount = 0;
}

INA_Class::~INA_Class() {
  delete[] _DeviceArray;
}

uint8_t INA_Class::begin(const float maxBusAmps, const uint32_t microOhmR) {
  // Ochrana pred memory leak pri opakovanom volaní begin()
  if (_DeviceArray != nullptr) {
    delete[] _DeviceArray;
  }
  _DeviceArray = new inaDet[_expectedDevices];
  _DeviceCount = 0;
  
  uint8_t targetAddresses[] = {0x40, 0x41, 0x42, 0x43, 0x44};
  
  for (uint8_t i = 0; i < 5; i++) {
    uint8_t addr = targetAddresses[i];
    
    Wire1.beginTransmission(addr);
    if (Wire1.endTransmission() == 0) {
      uint16_t dieId = readWord(INA_DIE_ID_REGISTER, addr);
      if (dieId == INA_DIE_ID_VALUE) {
        if (_DeviceCount < _expectedDevices) {
          _DeviceArray[_DeviceCount].address = addr;
          
          // Pre U2 (index 0) priradíme 3mOhm bočník, pre ostatné 5mOhm podľa hardvéru
          if (addr == 0x40) {
             _DeviceArray[_DeviceCount].microOhmR = 3000;
          } else {
             _DeviceArray[_DeviceCount].microOhmR = 5000;
          }
          
          _DeviceArray[_DeviceCount].maxBusAmps = maxBusAmps;
          _DeviceArray[_DeviceCount].current_LSB = maxBusAmps / 32768.0f;
          
          _DeviceCount++;
        }
      }
    }
  }
  
  for (uint8_t i = 0; i < _DeviceCount; i++) {
    initDevice(i);
  }
  
  return _DeviceCount == 0 ? UINT8_MAX : _DeviceCount;
}

void INA_Class::initDevice(const uint8_t deviceNumber) {
  uint8_t addr = _DeviceArray[deviceNumber].address;
  uint32_t rShunt = _DeviceArray[deviceNumber].microOhmR;
  float cLsb = _DeviceArray[deviceNumber].current_LSB;
  
  // 1. Reset zariadenia
  writeWord(INA_CONFIGURATION_REGISTER, INA_RESET_DEVICE, addr);
  delay(10);
  
  // 2. ADC CONFIG: continuous bus + shunt + temperature, AVG = 16, 1052 us each.
  //
  // This used to be the literal 0xAB6A, and the comment above it claimed "Continuous
  // All" - but MODE lives in bits [15:12], and 0xA there is INA_MODE_CONTINUOUS_SHUNT,
  // i.e. SHUNT ONLY. The shunt (and so the current) kept converting, while BUS VOLTAGE
  // and DIE TEMPERATURE were frozen at whatever the device happened to convert before
  // this register was written - effectively the values at board boot.
  //
  // That is why voltages looked "miscalibrated": they were not wrong, they were stale.
  // The four regulated rails hid the bug almost perfectly, because a frozen reading of
  // a regulated output is indistinguishable from a correct one. Only the battery, which
  // actually sags as it discharges, ever exposed it.
  //
  // Built from the enum rather than a literal so the two cannot drift apart again.
  constexpr uint16_t adcConfig =
      ((uint16_t)INA_MODE_CONTINUOUS_ALL << 12) |  // MODE   [15:12] bus + shunt + temp
      ((uint16_t)0x5 << 9)                      |  // VBUSCT  [11:9] 1052 us
      ((uint16_t)0x5 << 6)                      |  // VSHCT    [8:6] 1052 us
      ((uint16_t)0x5 << 3)                      |  // VTCT     [5:3] 1052 us
      ((uint16_t)0x2);                             // AVG      [2:0] 16 samples
  // Full cycle is now 3 x 1052 us x 16 = ~50.5 ms, comfortably inside the 500 ms
  // telemetry period; before, with shunt only, it was ~16.8 ms.
  static_assert((int)INA_MODE_CONTINUOUS_ALL == 0xF,
                "MODE nibble comes from ina_Mode ordering; INA_MODE_CONTINUOUS_ALL must be 0xF");
  static_assert(adcConfig == 0xFB6A, "ADC_CONFIG must encode to 0xFB6A");
  writeWord(INA_ADC_CONFIGURATION_REGISTER, adcConfig, addr);
  delay(10);
  
  // 3. CONFIG: ADCRANGE = 0
  writeWord(INA_CONFIGURATION_REGISTER, 0x0000, addr);
  delay(10);
  
  // 4. Zápis kalibračného registra pre konkrétny prúd a bočník čipu
  if (rShunt > 0) {
    float shuntResistorOhms = (float)rShunt / 1000000.0f;
    uint16_t calibrationValue = (uint16_t)(819200000.0f * cLsb * shuntResistorOhms);
    writeWord(INA_CALIBRATION_REGISTER, calibrationValue, addr);
  }
}

void INA_Class::setMode(const uint8_t mode, const uint8_t deviceNumber) {}
void INA_Class::setAveraging(const uint16_t averages, const uint8_t deviceNumber) {}
void INA_Class::setBusConversion(const uint32_t convTime, const uint8_t deviceNumber) {}
void INA_Class::setShuntConversion(const uint32_t convTime, const uint8_t deviceNumber) {}
void INA_Class::setTempConversion(const uint32_t convTime, const uint8_t deviceNumber) {}
void INA_Class::setAdcRange(const bool adcRange, const uint8_t deviceNumber) {}

uint16_t INA_Class::getBusMilliVolts(const uint8_t deviceNumber) {
  if (deviceNumber >= _DeviceCount) return 0;
  uint8_t addr = _DeviceArray[deviceNumber].address;
  
  uint16_t raw = readWord(INA_BUS_VOLTAGE_REGISTER, addr);
  return (uint16_t)((float)raw * 3.125f);
}

float INA_Class::getShuntMicroVolts(const uint8_t deviceNumber) {
  if (deviceNumber >= _DeviceCount) return 0.0f;
  uint8_t addr = _DeviceArray[deviceNumber].address;
  
  int16_t raw = (int16_t)readWord(INA_SHUNT_VOLTAGE_REGISTER, addr);
  return (float)raw * 5.0f;
}

float INA_Class::getBusMicroAmps(const uint8_t deviceNumber) {
  if (deviceNumber >= _DeviceCount) return 0.0f;
  uint8_t addr = _DeviceArray[deviceNumber].address;
  float cLsb = _DeviceArray[deviceNumber].current_LSB;
  
  int16_t raw = (int16_t)readWord(INA_CURRENT_REGISTER, addr);
  return (float)raw * cLsb * 1000000.0f;
}

float INA_Class::getDieTemperature(const uint8_t deviceNumber) {
  if (deviceNumber >= _DeviceCount) return -999.0f;
  uint8_t addr = _DeviceArray[deviceNumber].address;
  
  int16_t raw = (int16_t)readWord(INA_TEMP_REGISTER, addr);
  raw = raw >> 4;
  return (float)raw * 0.125f;
}

uint8_t INA_Class::getDeviceAddress(const uint8_t deviceNumber) {
  if (deviceNumber >= _DeviceCount) return 0;
  return _DeviceArray[deviceNumber].address;
}

// ---------------------------------------------------------------------------------
// Diagnostics. These expose the unconverted register contents and the per-device
// scaling actually in use, so a RAW dump can show what the chip really holds
// independently of any conversion the normal getters apply.
// ---------------------------------------------------------------------------------

uint16_t INA_Class::readRegister(const uint8_t reg, const uint8_t deviceNumber) {
  if (deviceNumber >= _DeviceCount) return 0xFFFF;
  return readWord(reg, _DeviceArray[deviceNumber].address);
}

float INA_Class::getCurrentLSB(const uint8_t deviceNumber) {
  if (deviceNumber >= _DeviceCount) return 0.0f;
  return _DeviceArray[deviceNumber].current_LSB;
}

uint32_t INA_Class::getMicroOhmR(const uint8_t deviceNumber) {
  if (deviceNumber >= _DeviceCount) return 0;
  return _DeviceArray[deviceNumber].microOhmR;
}

// The four *Raw getters below were declared in the header but never defined, so any
// call to them would have failed at link time. Defined here because the RAW dump
// needs them.
uint16_t INA_Class::getBusRaw(const uint8_t deviceNumber) {
  if (deviceNumber >= _DeviceCount) return 0xFFFF;
  return readWord(INA_BUS_VOLTAGE_REGISTER, _DeviceArray[deviceNumber].address);
}

uint16_t INA_Class::getShuntRaw(const uint8_t deviceNumber) {
  if (deviceNumber >= _DeviceCount) return 0xFFFF;
  return readWord(INA_SHUNT_VOLTAGE_REGISTER, _DeviceArray[deviceNumber].address);
}

int16_t INA_Class::getBusMicroAmpsRaw(const uint8_t deviceNumber) {
  if (deviceNumber >= _DeviceCount) return 0;
  return (int16_t)readWord(INA_CURRENT_REGISTER, _DeviceArray[deviceNumber].address);
}

int16_t INA_Class::getDieTemperatureRaw(const uint8_t deviceNumber) {
  if (deviceNumber >= _DeviceCount) return 0;
  return (int16_t)readWord(INA_TEMP_REGISTER, _DeviceArray[deviceNumber].address);
}

uint16_t INA_Class::readWord(const uint8_t reg, const uint8_t deviceAddress) const {
  Wire1.beginTransmission(deviceAddress);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return 0xFFFF;
  
  Wire1.requestFrom(deviceAddress, (uint8_t)2);
  if (Wire1.available() >= 2) {
    uint8_t msb = Wire1.read();
    uint8_t lsb = Wire1.read();
    return (uint16_t)((msb << 8) | lsb);
  }
  return 0xFFFF;
}

void INA_Class::writeWord(const uint8_t reg, const uint16_t data, const uint8_t deviceAddress) const {
  Wire1.beginTransmission(deviceAddress);
  Wire1.write(reg);
  Wire1.write((data >> 8) & 0xFF);
  Wire1.write(data & 0xFF);
  Wire1.endTransmission();
  delayMicroseconds(I2C_DELAY);
}

void INA_Class::loadInaFromIndex(const uint8_t deviceNumber) {}
void INA_Class::saveInaToIndex(const uint8_t deviceNumber) {}