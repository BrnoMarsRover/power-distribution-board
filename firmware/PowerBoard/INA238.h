#ifndef ARDUINO
#define ARDUINO 0
#endif
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Wire.h>

#ifndef INA__Class_h
#define INA__Class_h

typedef struct inaDet {
  uint8_t  operatingMode : 4;  ///< 0-15        Default to continuous mode
  uint32_t address : 7;        ///< 0-127       I2C Address of device
  uint32_t maxBusAmps : 10;    ///< 0-1023      Store initialization value
  uint32_t microOhmR : 20;     ///< 0-1,048,575 Store initialization value
  uint16_t shuntVoltage_LSB;   ///< Device dependent LSB factor
  uint16_t busVoltage_LSB;     ///< Device dependent LSB factor
  float    current_LSB;        ///< Amperage LSB
  float    power_LSB;          ///< Wattage LSB
  uint32_t temp_LSB;           ///< Temp LSB
  inaDet();                    ///< struct constructor
} inaDet;

enum ina_Mode {
  INA_MODE_SHUTDOWN,
  INA_MODE_TRIGGERED_BUS,
  INA_MODE_TRIGGERED_SHUNT,
  INA_MODE_TRIGGERED_VOLT,
  INA_MODE_TRIGGERED_TEMP,
  INA_MODE_TRIGGERED_TEMP_BUS,
  INA_MODE_TRIGGERED_TEMP_SHUNT,
  INA_MODE_TRIGGERED_ALL,
  INA_MODE_POWER_DOWN,
  INA_MODE_CONTINUOUS_BUS,
  INA_MODE_CONTINUOUS_SHUNT,
  INA_MODE_CONTINUOUS_VOLT,
  INA_MODE_CONTINUOUS_TEMP,
  INA_MODE_CONTINUOUS_TEMP_BUS,
  INA_MODE_CONTINUOUS_TEMP_SHUNT,
  INA_MODE_CONTINUOUS_ALL
};

#ifndef INA_I2C_MODES
#define INA_I2C_MODES
const uint32_t INA_I2C_STANDARD_MODE{100000};
const uint32_t INA_I2C_FAST_MODE{400000};
const uint32_t INA_I2C_FAST_MODE_PLUS{1000000};
const uint32_t INA_I2C_HIGH_SPEED_MODE{3400000};
#endif

const uint8_t  I2C_DELAY{10};

const uint8_t  INA_DIE_ID_REGISTER{0x3F};
const uint16_t INA_DIE_ID_VALUE{0x2381};
const uint8_t  INA_DEFAULT_OPERATING_MODE{0x0F}; // Režim Continuous All (Bity MODE v ADC_CONFIG sú 1010)
const uint16_t INA_RESET_DEVICE{0x8000};
const uint8_t  INA_CONFIGURATION_REGISTER{0};
const uint8_t  INA_ADC_CONFIGURATION_REGISTER{1};
const uint8_t  INA_CALIBRATION_REGISTER{2};
const uint8_t  INA_SHUNT_VOLTAGE_REGISTER{4};
const uint8_t  INA_BUS_VOLTAGE_REGISTER{5};
const uint8_t  INA_TEMP_REGISTER{6};
const uint8_t  INA_CURRENT_REGISTER{7};
const uint8_t  INA_POWER_REGISTER{8};

const uint16_t INA_BUS_VOLTAGE_LSB{3125};        ///< LSB 3.125mV
const uint16_t INA_SHUNT_VOLTAGE_LSB_ADC0{5000}; ///< LSB 5uV (ADCRANGE = 0)
const uint16_t INA_SHUNT_VOLTAGE_LSB_ADC1{1250}; ///< LSB 1.25uV (ADCRANGE = 1)
const uint16_t INA_TEMP_LSB{125};                ///< LSB 125m°C

class INA_Class {
 public:
  INA_Class(uint8_t expectedDevices = 1);
  ~INA_Class();
  uint8_t     begin(const float maxBusAmps, const uint32_t microOhmR);
  void        setI2CSpeed(const uint32_t i2cSpeed = INA_I2C_STANDARD_MODE) const;
  void        setMode(const uint8_t mode, const uint8_t deviceNumber = UINT8_MAX);
  void        setAveraging(const uint16_t averages, const uint8_t deviceNumber = UINT8_MAX);
  void        setBusConversion(const uint32_t convTime, const uint8_t deviceNumber = UINT8_MAX);
  void        setShuntConversion(const uint32_t convTime, const uint8_t deviceNumber = UINT8_MAX);
  void        setTempConversion(const uint32_t convTime, const uint8_t deviceNumber = UINT8_MAX);
  void        setAdcRange(const bool adcRange, const uint8_t deviceNumber = UINT8_MAX);
  uint16_t    getBusMilliVolts(const uint8_t deviceNumber = 0);
  uint16_t    getBusRaw(const uint8_t deviceNumber = 0);
  float       getShuntMicroVolts(const uint8_t deviceNumber = 0);
  uint16_t    getShuntRaw(const uint8_t deviceNumber = 0);
  float       getBusMicroAmps(const uint8_t deviceNumber = 0);
  int16_t     getBusMicroAmpsRaw(const uint8_t deviceNumber = 0);
  uint16_t    getBusMicroWatts(const uint8_t deviceNumber = 0);
  float       getDieTemperature(const uint8_t deviceNumber = 0);
  int16_t     getDieTemperatureRaw(const uint8_t deviceNumber = 0);
  uint8_t     getDeviceAddress(const uint8_t deviceNumber = 0);
  void        reset(const uint8_t deviceNumber = 0);
  bool        conversionFinished(const uint8_t deviceNumber = 0);
  void        waitForConversion(const uint8_t deviceNumber = UINT8_MAX);
 
 private:
  uint16_t    readWord(const uint8_t addr, const uint8_t deviceAddress) const;
  void        writeWord(const uint8_t addr, const uint16_t data, const uint8_t deviceAddress) const;
  void        loadInaFromIndex(const uint8_t deviceNumber);
  void        saveInaToIndex(const uint8_t deviceNumber);
  void        initDevice(const uint8_t deviceNumber);

  uint8_t    _expectedDevices{1};
  uint8_t    _DeviceCount{0};
  uint8_t    _currentINA{UINT8_MAX};
  inaDet* _DeviceArray;
  inaDet     ina;
};
#endif