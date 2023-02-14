#include <Arduino.h>
#include "OLED.h"
// #include "settings.h"
// #include "sensor_readings.h"


void DisplayLevelSensor(Adafruit_SSD1306 *Disp, LevelSensor *SenLevVal)
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

    Disp->print("MM: ");
    Disp->print(SenLevVal->DepthMM);
    Disp->print(" IN: ");
    Disp->println(SenLevVal->DepthIn, 1);
}


// just used to show save count on display
void OLED_Light(Adafruit_SSD1306 *Disp, double LT, LevelSensor *SenLevVal)
{

    Disp->print("#");
    Disp->print(LT, 0);
    // Disp->display();
    // Disp->print(" MM:");

    // Disp->print(SenLevVal->depthIntMM);
}

// read and display button press
void DisplaySwitches(Adafruit_SSD1306 *Disp, Select_SW *SS)
{ // line 1
    Disp->print("SSA:");
    if (SS->Switch_Auto == 0)
    {
        Disp->print("0");
    }
    else
    {
        Disp->print("1");
    }

    Disp->print(" SSB:");
    if (SS->Switch_Alarm == 0)
    {
        Disp->print("0");
    }
    else
    {
        Disp->print("1");
    }

    Disp->print(" SSC:");
    if (SS->Switch_Off == 0)
    {
        Disp->print("0");
    }
    else
    {
        Disp->print("1");
    }

    Disp->print(" SSD:");
    if (SS->Switch_Pump == 0)
    {
        Disp->print("0");
    }
    else
    {
        Disp->print("1");
    }
}


// BME Sensor
void DisplayEnvSensor(Adafruit_SSD1306 *Disp, BME_Sensor *SenEnvVal)
// Adafruit_BME280 *bme)
{ // line 3
    // Disp->println();
    // digitalWrite(UpdateLED, HIGH);

    // Temperature
    // print to serial port
    DEBUGPRINT(SenEnvVal->f_temperature);
    DEBUGPRINT(" °C ");

    Disp->print(SenEnvVal->f_temperature);
    Disp->print("C ");

    // Humidity
    // print to serial port
    DEBUGPRINT(SenEnvVal->f_humidity);
    DEBUGPRINTLN("%");

    Disp->print(SenEnvVal->f_humidity);
    Disp->println("% ");
    /*
    // Pressure
    //print to serial port
    DEBUGPRINT(SenVal->f_pressure);
    DEBUGPRINTLN(" hPa");

    Disp->print(SenVal->f_pressure);
    Disp->println("hpa ");

    // Appx altitude
    //print to serial port
    DEBUGPRINT(SenVal->f_altitude);
    DEBUGPRINTLN(" m");

    Disp->print(SenVal->f_altitude);
    Disp->println("m ");
*/
    //  ******  Send Data to AdaIO   ******
    // Temp->save(f_temperature);
    // Hum->save(f_humidity);
    // LEDControl->save(IFTTT_Flag);
    // Pres->save(f_pressure);
    // Alt->save(f_altitude);

    // update AdaIO count
    // DisplayTheCount(OLED_Display);

    // digitalWrite(UpdateLED, LOW);
    // print to serial port
    // DEBUGPRINTLN("-----v2----");
}



void OLED_Time(Adafruit_SSD1306 *Disp, DateTime *RTCClk)
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

void OLED_Date(Adafruit_SSD1306 *Disp, DateTime *RTCClk)
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

void OLED_Day(Adafruit_SSD1306 *Disp, DateTime *RTCClk)

{
    char daysOfTheWeek[7][12] = {" Sunday", " Monday", " Tuesday", " Wednesday", " Thursday", " Friday", " Saturday"};
    Disp->print(daysOfTheWeek[RTCClk->dayOfTheWeek()]);
}






/* void OLED_Range(Adafruit_SSD1306 *Disp, SRFRanges *Rngs)
{
    Disp->print(" Rng: ");
    Disp->print(Rngs->Range);
    Disp->print("in");
} */