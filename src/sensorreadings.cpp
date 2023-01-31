
#include <Arduino.h>
// #include "INA3221.h"
#include "OLED.h"

// //#include "mqttController.h"

/* void ReadSwitches(Select_SW *SwState) // Adafruit_SSD1306 *OLED_Display)
{
    int SSWAutoPin = 4;   // auto/man pos  BUTTON_A
    int SSWAlarmPin = 36; // alarm pos BUTTON_B
    int SSWOffPin = 39;   // off pos BUTTON_C
    int SSWPumpPin = 34;  // pump pos BUTTON_C

    if (digitalRead(SSWAutoPin) == 0)
    {
        SwState->Switch_Auto = 1;
    }
    else
    {
        SwState->Switch_Auto = 0;
    }

    if (!digitalRead(SSWAlarmPin))
    {
        SwState->Switch_Alarm = 1;
    }
    else
    {
        SwState->Switch_Alarm = 0;
    }

    if (!digitalRead(SSWOffPin))
    {
        SwState->Switch_Off = 1;
    }
    else
    {
        SwState->Switch_Off = 0;
    }

    if (!digitalRead(SSWPumpPin))
    {
        SwState->Switch_Pump = 1;
    }
    else
    {
        SwState->Switch_Pump = 0;
    }
}
 */

void ReadLevelSensor(SDL_Arduino_INA3221 *LevSensor, LevelSensor *SensorLevelVal)
{

    int SensorFailType = 0;
    static int SensorFailCount = 0;
    float current_ma[3];
    float voltage[3];
    float shunt[3];
    float LoadV[3];
    const int Chan1 = 1;
    const int Chan2 = 2;
    const int Chan3 = 3;
    // scal vals convert ma to mm
    double in_min[3] = {4.1, 4.98, 4.0};
    double in_max[3] = {9.36, 20.00, 19.92};
    double out_min[3] = {40.0, 240.0, 4.0};
    double out_max[3] = {110.23, 3000, 300};

    // read first chan
    current_ma[INA3221_CH1] = LevSensor->getCurrent_mA(Chan1) * 1000;
    voltage[INA3221_CH1] = LevSensor->getBusVoltage_V(Chan1);
    shunt[INA3221_CH1] = LevSensor->getShuntVoltage_mV(Chan1); /// 1000000
    LoadV[INA3221_CH1] = voltage[INA3221_CH1] + (shunt[INA3221_CH1]);
    // read 2nd chan
    current_ma[INA3221_CH2] = LevSensor->getCurrent_mA(Chan2) * 1000;
    voltage[INA3221_CH2] = LevSensor->getBusVoltage_V(Chan2);
    shunt[INA3221_CH2] = LevSensor->getShuntVoltage_mV(Chan2); /// 1000000
    LoadV[INA3221_CH2] = voltage[INA3221_CH2] + (shunt[INA3221_CH2]);
    // read 3rd chan
    current_ma[INA3221_CH3] = LevSensor->getCurrent_mA(Chan3) * 1000;
    voltage[INA3221_CH3] = LevSensor->getBusVoltage_V(Chan3);
    shunt[INA3221_CH3] = LevSensor->getShuntVoltage_mV(Chan3); /// 1000000
    LoadV[INA3221_CH3] = voltage[INA3221_CH3] + (shunt[INA3221_CH3]);

    /*  // SensorLevelVal struct
    int ShuntVRaw = 0;
    float ShuntVmv
    int BusVRaw = 0;
    float BusV
    float ShuntImA
    float LoadV
    float power_mW = 0;
    float DepthIn
    int DepthMM
    */

    SensorLevelVal->DepthMM = mapf(current_ma[INA3221_CH2], in_min[INA3221_CH2], in_max[INA3221_CH2], out_min[INA3221_CH2], out_max[INA3221_CH2]);

    SensorLevelVal->ShuntImA = current_ma[INA3221_CH2];
    SensorLevelVal->BusV = voltage[INA3221_CH2];
    SensorLevelVal->ShuntVmv = shunt[INA3221_CH2];
    SensorLevelVal->LoadV = LoadV[INA3221_CH2];
    SensorLevelVal->DepthIn = SensorLevelVal->DepthMM / 25.4;
/*     Serial.println("Vals In Struct SensorLevelVal for C2");
    Serial.print("ShuntImA: ");
    Serial.print(SensorLevelVal->ShuntImA);
    Serial.print(" BusV: ");
    Serial.print(SensorLevelVal->BusV);
    Serial.print(" ShuntVmv: ");
    Serial.print(SensorLevelVal->ShuntVmv);
    Serial.print(" LoadV: ");
    Serial.print(SensorLevelVal->LoadV);
    Serial.print(" DepthMM: ");
    Serial.print(SensorLevelVal->DepthMM);
    Serial.print(" DepthIN: ");
    Serial.println(SensorLevelVal->DepthIn); */

    // test for bad reading
    if (SensorLevelVal->ShuntImA < 3.5)
    {

        SensorFailCount++;
        SensorFailType = 1;
    }
    else if (SensorLevelVal->ShuntImA > 21.0)
    {
        SensorFailCount++;
        SensorFailType = 2;
    }
    else
    {
        SensorFailCount = 0;
        SensorFailType = 0;
    }

    //  SensorFailCount = 0; ////////////////////////////////////
    if (SensorFailCount > 5)
    {
        Serial.print("Sensor Fail:");
        SensorFailCount = 0;

        switch (SensorFailType)
        {
        case 1:
            Serial.print("Sensor Not Found");
            break;
        case 2:
            Serial.print("Sensor Failed");
            break;
        default:
            Serial.print("Something went wrong");
            break;
        }
    }
}

// BME_Sensor *ReadSensor(Adafruit_BME280 *bme) //, Adafruit_SSD1306 *OLED_Display)
void ReadEnvSensor(Adafruit_BME280 *EnvSensor, BME_Sensor *SensorEnvVal)
{

    // read sensor and load vars

    SensorEnvVal->f_temperature = EnvSensor->readTemperature();
    SensorEnvVal->f_humidity = EnvSensor->readHumidity();
    SensorEnvVal->f_pressure = EnvSensor->readPressure() / 100.0F;
    SensorEnvVal->f_altitude = EnvSensor->readAltitude(SEALEVELPRESSURE_HPA);
}

double mapf(double var, double InMin, double InMax, double OutMin, double OutMax)
{
    return (var - InMin) * (OutMax - OutMin) / (InMax - InMin) + OutMin;
}