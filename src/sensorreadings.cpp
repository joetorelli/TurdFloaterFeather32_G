
#include <Arduino.h>
#include "sensor_readings.h"
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

// returns Status of sensor ans fill struc with values
int ReadLevelSensor(SDL_Arduino_INA3221 *LevSensor, LevelSensor *SensorLevelVal, int ChanNum)
{

    int SensorFailType = 0;
    // static int SensorFailCount = 0;
    float current_ma[3];
    float voltage[3];
    float shunt[3];
    float LoadV[3];

    // 1st element = scal vals for 12v
    // 2nd element = scal vals convert ma to mm chan
    // 3rd element = scal vals for 5v
    double in_min[3] = {0, 4.98, 0};
    double in_max[3] = {1, 20.00, 1};
    double out_min[3] = {0, 240.0, 0};
    double out_max[3] = {1, 3000, 1};

    current_ma[ChanNum] = LevSensor->getCurrent_mA(ChanNum + 1) * 1000;
    voltage[ChanNum] = LevSensor->getBusVoltage_V(ChanNum + 1);
    shunt[ChanNum] = LevSensor->getShuntVoltage_mV(ChanNum + 1); /// 1000000
    LoadV[ChanNum] = voltage[ChanNum] + (shunt[ChanNum]);

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

    SensorLevelVal->ShuntImA = current_ma[ChanNum];
    SensorLevelVal->BusV = voltage[ChanNum];
    SensorLevelVal->ShuntVmv = shunt[ChanNum];
    SensorLevelVal->LoadV = LoadV[ChanNum];

    // SensorLevelVal->DepthMM = mapf(current_ma[INA3221_CH2], in_min[INA3221_CH2], in_max[INA3221_CH2], out_min[INA3221_CH2], out_max[INA3221_CH2]);

    // SensorLevelVal->ShuntImA = current_ma[INA3221_CH2];
    // SensorLevelVal->BusV = voltage[INA3221_CH2];
    // SensorLevelVal->ShuntVmv = shunt[INA3221_CH2];
    // SensorLevelVal->LoadV = LoadV[INA3221_CH2];
    // SensorLevelVal->DepthIn = SensorLevelVal->DepthMM / 25.4;

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

    // test for 12v bad reading
    if (ChanNum == 0)
    {
        if (SensorLevelVal->ShuntImA < 0) //////////// set for low 12v
        {

            // SensorFailCount++;
            SensorFailType = 1;
        }
        else if (SensorLevelVal->ShuntImA > 1) //////////// set for hi 12v over 350ma
        {
            // SensorFailCount++;
            SensorFailType = 2;
        }
        else //////////// good 12v
        {
            // SensorFailCount = 0;
            SensorFailType = 0;
        }

        /*         //  SensorFailCount = 0;                            ///////////////////////////////////
                if (SensorFailType != 0)
                {


                    if (SensorFailCount > 5)
                    {

                        // Serial.print("Sensor Fail:");
                        SensorFailCount = 0;

                    }
                } */
    }

    // test for Sensor bad reading
    if (ChanNum == 1)
    {
        if (SensorLevelVal->ShuntImA < 3.5) // test for no sensor
        {
            // SensorFailCount++;
            SensorFailType = 1;

            // pass bad val
            SensorLevelVal->DepthMM = -1;
            SensorLevelVal->DepthIn = -1;
        }
        else if (SensorLevelVal->ShuntImA > 21.0) // test for bad sensor
        {
            // SensorFailCount++;
            SensorFailType = 2;

            // pass bad val
            SensorLevelVal->DepthMM = -1;
            SensorLevelVal->DepthIn = -1;
        }
        else // good sensor
        {
            // SensorFailCount = 0;
            SensorFailType = 0;

            // pass val
            SensorLevelVal->DepthMM = mapf(current_ma[ChanNum], in_min[ChanNum], in_max[ChanNum], out_min[ChanNum], out_max[ChanNum]);
            SensorLevelVal->DepthIn = SensorLevelVal->DepthMM / 25.4;
        }

        /*         //  SensorFailCount = 0; ////////////////////////////////////
                if (SensorFailCount > 5)
                {

                    // Serial.print("Sensor Fail:");
                    SensorFailCount = 0;

                                 switch (SensorFailType)
                                {
                                case 0:
                                    Serial.println("Sensor OK");
                                    return SensorFailType;
                                case 1:
                                    Serial.println("Sensor Not Found");
                                    return SensorFailType;
                                    break;
                                case 2:
                                    Serial.println("Sensor Failed");
                                    return SensorFailType;
                                    break;
                                default:
                                    Serial.println("Something went wrong");
                                    return SensorFailType;
                                    break;
                                }
                } */
    }

    // test for ps 5v bad reading
    if (ChanNum == 2)
    {
        if (SensorLevelVal->ShuntImA < 0) //////////// set for low 5v
        {

            // SensorFailCount++;
            SensorFailType = 1;
        }
        else if (SensorLevelVal->ShuntImA > 1) //////////// set for hi 5v
        {
            // SensorFailCount++;
            SensorFailType = 2;
        }
        else //////////// good 5v
        {
            // SensorFailCount = 0;
            SensorFailType = 0;
        }

        //  SensorFailCount = 0; ////////////////////////////////////
        /*         if (SensorFailCount > 5)
                {

                    // Serial.print("Sensor Fail:");
                    SensorFailCount = 0;

                                switch (SensorFailType)
                                {
                                case 0:
                                    Serial.println("Sensor OK");
                                    return SensorFailType;
                                case 1:
                                    Serial.println("Sensor Not Found");
                                    return SensorFailType;
                                    break;
                                case 2:
                                    Serial.println("Sensor Failed");
                                    return SensorFailType;
                                    break;
                                default:
                                    Serial.println("Something went wrong");
                                    return SensorFailType;
                                    break;
                                }
                } */
    }
    return SensorFailType;
}

double mapf(double var, double InMin, double InMax, double OutMin, double OutMax)
{
    return (var - InMin) * (OutMax - OutMin) / (InMax - InMin) + OutMin;
}