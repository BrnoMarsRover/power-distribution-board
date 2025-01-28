/*!
 @file INA.h

 @brief INA Class library header file

 @mainpage Arduino library to support the INA238 sensor

 @section Library_intro_section Description

 TODO

 @section license GNU General Public License v3.0

 This program is free software: you can redistribute it and/or modify it under the terms of the GNU
 General Public License as published by the Free Software Foundation, either version 3 of the
 License, or (at your option) any later version. This program is distributed in the hope that it
 will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details. You should
 have received a copy of the GNU General Public License along with this program.  If not, see
 <http://www.gnu.org/licenses/>.

@section author Author

Written by Stanislav Svědiroh <stanislav.svediroh@vut.cz>
Inspired by Arnd <Arnd@Zanduino.Com> at https://www.github.com/SV-Zanshin
*/

#ifndef ARDUINO
/*! Define macro if not defined yet */
#define ARDUINO 0
#endif
#if ARDUINO >= 100 /* Use old library if IDE is prior to V1.0 */
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#ifndef INA__Class_h
/*! Guard code definition to prevent multiple includes */
#define INA__Class_h

/*! typedef contains a packed bit-level definition of information stored on a device */
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
  INA_MODE_SHUTDOWN,             ///< Device powered down
  INA_MODE_TRIGGERED_BUS,        ///< Triggered bus, no shunt and temp
  INA_MODE_TRIGGERED_SHUNT,      ///< Triggered shunt, no bus and temp
  INA_MODE_TRIGGERED_VOLT,       ///< Triggered voltages, no temp
  INA_MODE_TRIGGERED_TEMP,       ///< Triggered temp, no voltages
  INA_MODE_TRIGGERED_TEMP_BUS,   ///< Triggered temp and bus, no shunt
  INA_MODE_TRIGGERED_TEMP_SHUNT, ///< Triggered temp and shunt, no bus
  INA_MODE_TRIGGERED_ALL,        ///< Triggered temp, shunt and bus
  INA_MODE_POWER_DOWN,           ///< Device powered down
  INA_MODE_CONTINUOUS_BUS,       ///< Continuous bus, no shunt and temp
  INA_MODE_CONTINUOUS_SHUNT,     ///< Continuous shunt, no bus and temp
  INA_MODE_CONTINUOUS_VOLT,      ///< Continuous voltages, no temp
  INA_MODE_CONTINUOUS_TEMP,      ///< Continuous temp, no voltages
  INA_MODE_CONTINUOUS_TEMP_BUS,  ///< Continuous temp and bus, no shunt
  INA_MODE_CONTINUOUS_TEMP_SHUNT,///< Continuous temp and shunt, no bus
  INA_MODE_CONTINUOUS_ALL        ///< Continuous temp, shunt and bus
};

#ifndef INA_I2C_MODES                             // I2C related constants
#define INA_I2C_MODES                             ///< Guard code to prevent multiple defs
const uint32_t INA_I2C_STANDARD_MODE{100000};     ///< Default normal I2C 100KHz speed
const uint32_t INA_I2C_FAST_MODE{400000};         ///< Fast mode
const uint32_t INA_I2C_FAST_MODE_PLUS{1000000};   ///< Really fast mode
const uint32_t INA_I2C_HIGH_SPEED_MODE{3400000};  ///< Turbo mode
#endif

const uint8_t  I2C_DELAY{10};                    ///< Microsecond delay on I2C writes

const uint8_t  INA_DIE_ID_REGISTER{0x3F};        ///< Hard-coded Die ID for INA238
const uint16_t INA_DIE_ID_VALUE{0x2381};         ///< Hard-coded Die ID for INA238
const uint8_t  INA_DEFAULT_OPERATING_MODE{0x0F}; ///< Default continuous mode
const uint16_t INA_RESET_DEVICE{0x8000};           ///< Reset Device Flag
const uint8_t  INA_CONFIGURATION_REGISTER{0};    ///< Configuration Register address
const uint8_t  INA_ADC_CONFIGURATION_REGISTER{1};///< ADC Configuration Register address
const uint8_t  INA_CALIBRATION_REGISTER{2};      ///< Calibration Register address
const uint8_t  INA_SHUNT_VOLTAGE_REGISTER{4};    ///< Shunt Voltage Register address
const uint8_t  INA_BUS_VOLTAGE_REGISTER{5};      ///< Bus Voltage Register address
const uint8_t  INA_TEMP_REGISTER{6};             ///< Die Temp Register address
const uint8_t  INA_CURRENT_REGISTER{7};          ///< Current Register address
const uint8_t  INA_POWER_REGISTER{8};            ///< Power Register address
const uint16_t INA_BUS_VOLTAGE_LSB{3125};        ///< LSB 3.125mV
const uint16_t INA_SHUNT_VOLTAGE_LSB_ADC0{5000}; ///< LSB 5uV
const uint16_t INA_SHUNT_VOLTAGE_LSB_ADC1{1250}; ///< LSB 1.25uV
const uint16_t INA_POWER_LSB{5};                 ///< LSB 3.125mV
const uint16_t INA_TEMP_LSB{125};                ///< LSB 125m°C

const uint16_t INA_CONFIG_MODE_MASK{0xF000};     ///< Bits 15-12
const uint16_t INA_CONFIG_AVG_MASK{0x0007};      ///< ADC Config Register Bits 0-2 masked
const uint16_t INA_CONFIG_SADC_MASK{0x01C0};     ///< ADC Config Register Bits 6-8 masked
const uint16_t INA_CONFIG_BADC_MASK{0x0E00};     ///< ADC Config Register Bits 9-11 masked
const uint16_t INA_CONFIG_TADC_MASK{0x0038};     ///< ADC Config Register Bits 3-5 masked
const uint16_t INA_CONFIG_ADC_RANGE_MASK{0x0010};///< ADC Config Register Bit 4 masked

class INA_Class {
 public:
  INA_Class(uint8_t expectedDevices = 1);
  ~INA_Class();
  uint8_t     begin(const uint16_t maxBusAmps, const uint32_t microOhmR);
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
  bool        alertOnConversion(const bool alertState, const uint8_t deviceNumber = UINT8_MAX);
  bool        alertOnShuntOverVoltage(const bool alertState, const int32_t milliVolts, const uint8_t deviceNumber = UINT8_MAX);
  bool        alertOnShuntUnderVoltage(const bool alertState, const int32_t milliVolts, const uint8_t deviceNumber = UINT8_MAX);
  bool        alertOnBusOverVoltage(const bool alertState, const int32_t milliVolts, const uint8_t deviceNumber = UINT8_MAX);
  bool        alertOnBusUnderVoltage(const bool alertState, const int32_t milliVolts, const uint8_t deviceNumber = UINT8_MAX);
  bool        alertOnPowerOverLimit(const bool alertState, const int32_t milliAmps, const uint8_t deviceNumber = UINT8_MAX);
 
 private:
  uint16_t    readWord(const uint8_t addr, const uint8_t deviceAddress) const;
  uint32_t    read3Bytes(const uint8_t addr, const uint8_t deviceAddress) const;
  void        writeWord(const uint8_t addr, const uint16_t data, const uint8_t deviceAddress) const;
  void        loadInaFromIndex(const uint8_t deviceNumber);
  void        saveInaToIndex(const uint8_t deviceNumber);
  void        initDevice(const uint8_t deviceNumber);

  uint8_t    _expectedDevices{1};
  uint8_t    _DeviceCount{0};         ///< Total number of devices detected
  uint8_t    _currentINA{UINT8_MAX};  ///< Stores current INA device number
  inaDet*     _DeviceArray;           ///< INA device structure
  inaDet     ina;                     ///< INA device structure
};
#endif
