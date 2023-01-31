#ifndef OLED_H

#define OLED_H
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "RTClib.h"
#include "settings.h"
#include "sensor_readings.h"
#include <Adafruit_BME280.h>
// #include "SRF.h"
// #include "ezTime.h"

/******************   OLED_Display  *******************/
#define SCREEN_WIDTH 128 //  OLED_Display width, in pixels
#define SCREEN_HEIGHT 64 //  OLED_Display height, in pixels
// Configure orientation of the display.
// 0 = none, 1 = 90 degrees clockwise, 2 = 180 degrees, 3 = 270 degrees CW
#define ROTATION 0

#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)

void DisplayLevelSensor(Adafruit_SSD1306 *Disp, LevelSensor *SenLevVal);
void DisplayEnvSensor(Adafruit_SSD1306 *Disp, BME_Sensor *SenEnvVal);
void DisplaySwitches(Adafruit_SSD1306 *Disp, Select_SW *SwState);

void OLED_Time(Adafruit_SSD1306 *Disp, DateTime *RTCClk);
void OLED_Date(Adafruit_SSD1306 *Disp, DateTime *RTCClk);
void OLED_Day(Adafruit_SSD1306 *Disp, DateTime *RTCClk);

////void OLED_Range(Adafruit_SSD1306 *Disp, SRFRanges *Rngs);
void OLED_Light(Adafruit_SSD1306 *Disp, double LT, LevelSensor *SenLevVal);
#endif