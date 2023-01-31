//
//   SDL_Arduino_INA3221 Library
//   SDL_Arduino_INA3221.cpp Arduino code - runs in continuous mode
//   Version 1.2
//   SwitchDoc Labs   September 2019
// modified for 3 different shunt resistors
// adding on chip averaging

/*
    Initial code from INA219 code (Basically just a core structure left)
    @author   K.Townsend (Adafruit Industries)
  @license  BSD (see BSDlicense.txt)
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#include "INA3221.h"

/**************************************************************************/
/*!
    @brief  Sends a single command byte over I2C
*/
/**************************************************************************/
void SDL_Arduino_INA3221::wireWriteRegister(uint8_t reg, uint16_t *value)
{
  Wire.beginTransmission(INA3221_i2caddr);
#if ARDUINO >= 100
  Wire.write(reg);                  // Register
  Wire.write((*value >> 8) & 0xFF); // Upper 8-bits
  Wire.write(*value & 0xFF);        // Lower 8-bits
#else
  Wire.send(reg);           // Register
  Wire.send(*value >> 8);   // Upper 8-bits
  Wire.send(*value & 0xFF); // Lower 8-bits
#endif
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
void SDL_Arduino_INA3221::wireReadRegister(uint8_t reg, uint16_t *value)
{

  Wire.beginTransmission(INA3221_i2caddr);
#if ARDUINO >= 100
  Wire.write(reg); // Register
#else
  Wire.send(reg);           // Register
#endif
  Wire.endTransmission();

  delay(1); // Max 12-bit conversion time is 586us per sample

  Wire.requestFrom(INA3221_i2caddr, (uint8_t)2);
#if ARDUINO >= 100
  // Shift values to create properly formed integer
  *value = ((Wire.read() << 8) | Wire.read());
#else
  // Shift values to create properly formed integer
  *value = ((Wire.receive() << 8) | Wire.receive());
#endif
}

//
void SDL_Arduino_INA3221::INA3221SetConfig(void)
{

  // Set Config register to take into account the settings above
  uint16_t config = INA3221_CONFIG_ENABLE_CHAN1 |
                    INA3221_CONFIG_ENABLE_CHAN2 |
                    INA3221_CONFIG_ENABLE_CHAN3 |
                    INA3221_CONFIG_AVG1 |
                    INA3221_CONFIG_VBUS_CT2 |
                    INA3221_CONFIG_VSH_CT2 |
                    INA3221_CONFIG_MODE_2 |
                    INA3221_CONFIG_MODE_1 |
                    INA3221_CONFIG_MODE_0;
  wireWriteRegister(INA3221_REG_CONFIG, &config);
}

/**************************************************************************/
/*!
    @brief  Instantiates a new SDL_Arduino_INA3221 class
*/
/**************************************************************************/
SDL_Arduino_INA3221::SDL_Arduino_INA3221(uint8_t addr, float shuntresistor_1, float shuntresistor_2, float shuntresistor_3)
{

  INA3221_i2caddr = addr;
  INA3221_shuntresistor[0] = shuntresistor_1;
  INA3221_shuntresistor[1] = shuntresistor_2;
  INA3221_shuntresistor[2] = shuntresistor_3;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (defaults to 32V and 2A for calibration values)
*/
/**************************************************************************/
void SDL_Arduino_INA3221::begin()
{
  Wire.begin();
  // Set chip to known config values to start
  INA3221SetConfig();

  /*   Serial.print("shunt r1=");
    Serial.println(INA3221_shuntresistor[0]);
    Serial.print("shunt r2=");
    Serial.println(INA3221_shuntresistor[1]);
    Serial.print("shunt r3=");
    Serial.println(INA3221_shuntresistor[2]);
    Serial.print("address=");
    Serial.println(INA3221_i2caddr);
    delay(5000); */
}

/**************************************************************************/
/*!
    @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t SDL_Arduino_INA3221::getBusVoltage_raw(int channel)
{
  uint16_t value;
  wireReadRegister(INA3221_REG_BUSVOLTAGE_1 + (channel - 1) * 2, &value);

  /*   Serial.print("BusV_raw");
    Serial.print(channel);
    Serial.print("= ");
    Serial.print(value, HEX);
    Serial.print(" / ");
    Serial.print(value, DEC);
    Serial.print(" / ");
    Serial.println((int16_t)(value), DEC); */

  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
  return (int16_t)(value);
}

/**************************************************************************/
/*!
    @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t SDL_Arduino_INA3221::getShuntVoltage_raw(int channel)
{
  uint16_t value;
  wireReadRegister(INA3221_REG_SHUNTVOLTAGE_1 + (channel - 1) * 2, &value);

  /*   Serial.print("ShuntV_raw");
    Serial.print(channel);
    Serial.print("= ");
    Serial.print(value, HEX);
    Serial.print(" / ");
    Serial.print(value, DEC);
    Serial.print(" / ");
    Serial.println((int16_t)(value), DEC); */

  return (int16_t)value;
}

/**************************************************************************/
/*!
    @brief  Gets the shunt voltage in mV (so +-163.8mV)
*/
/**************************************************************************/
float SDL_Arduino_INA3221::getShuntVoltage_mV(int channel)
{
  int16_t value;
  value = getShuntVoltage_raw(channel);
  return value * 0.005;
}

/**************************************************************************/
/*!
    @brief  Gets the buss voltage in volts
*/
/**************************************************************************/
float SDL_Arduino_INA3221::getBusVoltage_V(int channel)
{
  int16_t value = getBusVoltage_raw(channel);
  return value * 0.001;
}

/**************************************************************************/
/*!
    @brief  Gets the current value in mA, taking into account the
            config settings and current LSB
*/
/**************************************************************************/
float SDL_Arduino_INA3221::getCurrent_mA(int channel)
{
  float valueDec = getShuntVoltage_mV(channel) / INA3221_shuntresistor[channel - 1];

  return valueDec;
}

/**************************************************************************/
/*!
    @brief  Gets the Manufacturers ID
*/
/**************************************************************************/
int SDL_Arduino_INA3221::getManufID()
{
  uint16_t value;
  wireReadRegister(0xFE, &value);
  return value;
}

/**************************************************************************/
/*!
    @brief  mix shunt vals
*/
/**************************************************************************/
// void setShuntRes(float res_ch1, float res_ch2, float res_ch3);
void SDL_Arduino_INA3221::setShuntRes(float res_ch1, float res_ch2, float res_ch3)
{
  INA3221_shuntresistor[0] = res_ch1;
  INA3221_shuntresistor[1] = res_ch2;
  INA3221_shuntresistor[2] = res_ch3;
}

void SDL_Arduino_INA3221::setAveragingMode(ina3221_avg_mode_t mode)
{
  conf_reg_t conf_reg;

  wireReadRegister(INA3221_REG_CONFIG, (uint16_t *)&conf_reg);
  conf_reg.avg_mode = mode;
  wireWriteRegister(INA3221_REG_CONFIG, (uint16_t *)&conf_reg);
}

void SDL_Arduino_INA3221::setBusConversionTime(ina3221_conv_time_t convTime)
{
  conf_reg_t conf_reg;

  wireReadRegister(INA3221_REG_CONFIG, (uint16_t *)&conf_reg);
  conf_reg.bus_conv_time = convTime;
  wireWriteRegister(INA3221_REG_CONFIG, (uint16_t *)&conf_reg);
}

void SDL_Arduino_INA3221::setShuntConversionTime(ina3221_conv_time_t convTime)
{
  conf_reg_t conf_reg;

  wireReadRegister(INA3221_REG_CONFIG, (uint16_t *)&conf_reg);
  conf_reg.shunt_conv_time = convTime;
  wireWriteRegister(INA3221_REG_CONFIG, (uint16_t *)&conf_reg);
}

void SDL_Arduino_INA3221::setChannelEnable(ina3221_ch_t channel)
{
  conf_reg_t conf_reg;

  wireReadRegister(INA3221_REG_CONFIG, (uint16_t *)&conf_reg);

  switch (channel)
  {
  case INA3221_CH1:
    conf_reg.ch1_en = 1;
    break;
  case INA3221_CH2:
    conf_reg.ch2_en = 1;
    break;
  case INA3221_CH3:
    conf_reg.ch3_en = 1;
    break;
  case INA3221_CH_NUM:
    // conf_reg.ch3_en = 0;
    break;
  }

  wireWriteRegister(INA3221_REG_CONFIG, (uint16_t *)&conf_reg);
}

void SDL_Arduino_INA3221::setChannelDisable(ina3221_ch_t channel)
{
  conf_reg_t conf_reg;

  wireReadRegister(INA3221_REG_CONFIG, (uint16_t *)&conf_reg);

  switch (channel)
  {
  case INA3221_CH1:
    conf_reg.ch1_en = 0;
    break;
  case INA3221_CH2:
    conf_reg.ch2_en = 0;
    break;
  case INA3221_CH3:
    conf_reg.ch3_en = 0;
    break;
  case INA3221_CH_NUM:
    // conf_reg.ch3_en = 0;
    break;
  }

  wireWriteRegister(INA3221_REG_CONFIG, (uint16_t *)&conf_reg);
}