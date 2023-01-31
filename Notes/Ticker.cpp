
/******************  ticker example ********************/
#include <Arduino.h>
#include <Ticker.h>

#define LED_PINA 21
#define LED_PINB 22

Ticker blinker;
Ticker toggler;
Ticker changer;
Ticker tickerSetHigh;
Ticker tickerSetLow;
Ticker periodicTicker;
Ticker onceTicker;
Ticker periodicTickerPassed;

float blinkerPace = 0.1;      // seconds
const float togglePeriod = 5; // seconds
int executionsCount = 0;
int maxExecutionsCount = 10;

void setPin(int state)
{
  digitalWrite(LED_PINB, state);
}

void change()
{
  blinkerPace = 0.5;
}

void blink()
{
  digitalWrite(LED_PINA, !digitalRead(LED_PIN));
}

void toggle()
{
  static bool isBlinking = false;
  if (isBlinking)
  {
    blinker.detach();
    isBlinking = false;
  }
  else
  {
    blinker.attach(blinkerPace, blink);
    isBlinking = true;
  }
  digitalWrite(LED_PINA, LOW); // make sure LED on on after toggling (pin LOW = led ON)
}

void periodicPrint()
{
  Serial.println("printing in periodic function.");
}

void oncePrint()
{
  Serial.println("printing in once function.");
}

void periodicPrintPassed(int maxExecutionsCount)
{
  // callback implementation
  Serial.print("printing in periodic function. Exec nr: ");
  Serial.println(executionsCount + 1);

  executionsCount++;

  if (executionsCount >= maxExecutionsCount)
  {
    periodicTicker.detach();
  }
}

void setup()
{
  pinMode(LED_PINA, OUTPUT);
  pinMode(LED_PINB, OUTPUT);
  toggler.attach(togglePeriod, toggle);
  changer.once(30, change);

  // every 25 ms, call setPin(0)
  tickerSetLow.attach_ms(25, setPin, 0);

  // every 26 ms, call setPin(1)
  tickerSetHigh.attach_ms(26, setPin, 1);

  periodicTicker.attach_ms(5000, periodicPrint);
  // pass a value to periodicTicker
  periodicTickerPassed.attach_ms(5000, periodicPrintPassed, maxExecutionsCount);

  onceTicker.once_ms(10000, oncePrint);
}

void loop()
{
}