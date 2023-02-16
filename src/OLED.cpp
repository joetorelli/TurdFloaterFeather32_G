#include <Arduino.h>
#include "OLED.h"
// #include "settings.h"
// #include "sensor_readings.h"

void DisplayLevelSensor(Adafruit_SSD1327 *Disp, LevelSensor *SenLevVal)
{
    /*  SenLevVal Struct
        int ShuntVRaw = 0;
        float ShuntVmv = 0;
        int BusVRaw = 0;
        float BusV = 0;
        float ShuntImA = 0;
        float LoadV = 0;
        float power_mW = 0;
        float DepthIn = 0;
        int DepthMM = 0; */

    /*     Disp->print("MA: ");
        Disp->print(SenLevVal->ShuntImA, 2);
        Disp->print(" MV: ");
        Disp->println(SenLevVal->ShuntVmv, 2); */
    Disp->setTextSize(2);

    Disp->printf("  %d mm\n", SenLevVal->DepthMM);
    // Disp->print(SenLevVal->DepthMM);
    // Disp->print(" MM");
    // Disp->print(" IN: ");
    // Disp->println(SenLevVal->DepthIn, 1);
    Disp->setTextSize(1);
}

// just used to show save count on display
void OLED_Light(Adafruit_SSD1327 *Disp, double LT, LevelSensor *SenLevVal)
{

    Disp->print("#");
    Disp->print(LT, 0);
    // Disp->display();
    // Disp->print(" MM:");

    // Disp->print(SenLevVal->depthIntMM);
}

void OLED_Time(Adafruit_SSD1327 *Disp, DateTime *RTCClk)
{ // line 2

    Disp->println();
    if (RTCClk->hour() < 10)
    {
        Disp->print('0');
    }
    Disp->print(RTCClk->hour(), DEC);

    Disp->print(':');

    if (RTCClk->minute() < 10)
    {
        Disp->print('0');
    }
    Disp->print(RTCClk->minute(), DEC);

    Disp->print(':');

    if (RTCClk->second() < 10)
    {
        Disp->print('0');
    }
    Disp->print(RTCClk->second(), DEC);
}

void OLED_Date(Adafruit_SSD1327 *Disp, DateTime *RTCClk)
{ // line 2
    Disp->print(" ");
    if (RTCClk->day() < 10)
    {
        Disp->print('0');
    }
    Disp->print(RTCClk->day(), DEC);

    Disp->print('/');

    if (RTCClk->month() < 10)
    {
        Disp->print('0');
    }
    Disp->print(RTCClk->month(), DEC);

    Disp->print('/');

    if (RTCClk->year() < 10)
    {
        Disp->print('0');
    }
    Disp->println(RTCClk->year(), DEC);
}

void OLED_Day(Adafruit_SSD1327 *Disp, DateTime *RTCClk)

{
    char daysOfTheWeek[7][12] = {" Sunday", " Monday", " Tuesday", " Wednesday", " Thursday", " Friday", " Saturday"};
    Disp->print(daysOfTheWeek[RTCClk->dayOfTheWeek()]);
}
