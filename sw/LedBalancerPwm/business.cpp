/*
Author:  Stefano Di Paolo
License: MIT, https://en.wikipedia.org/wiki/MIT_License
Date:    2017-12-31

Led Balancer (http://www.microentropie.com)
Arduino nano based.

Main project entry.
The following libraries are required:
* LiquidCrystal-I2C (https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library)
* ManyButtons (https://github.com/microentropie/ManyButtons)
* IEC60063 (https://github.com/microentropie/IEC60063)

Sources repository: https://github.com/microentropie/
*/

#include <Arduino.h>
#include "display.h"

#include <IEC60063_E12.h>


extern const char *PROGRAM;

#define Vref 5.0
#define Rsense 47.0

void ComputeValues(byte ledNumber, byte pwm, unsigned int adcSum_i, unsigned int qtSamples_i, unsigned int adcSum_u, unsigned int qtSamples_u, float &ma, float &v)
{
  if (qtSamples_i <= 0 || qtSamples_u <= 0)
  {
    ma = v = 0;
  }
  else
  {
    ma = (float)adcSum_i * Vref * pwm * 1000.0 / (1024.0 * 255.0 * Rsense * (float)qtSamples_i);
    v = (float)(adcSum_u - adcSum_i) * Vref / (1024.0 * (float)qtSamples_u);
  }
}

void ComputeR(float &ma, float &v, float &vcc, float &r)
{
  if (ma <= 0.0 || vcc < v)
    r = 0;
  else
    r = (vcc - v) * 1000.0 / ma;
}

void DisplayLedParams(byte ledNumber, float &ma, float &v)
{
  String newValues = String("LED");
  newValues += String(ledNumber);
  newValues += " ";
  newValues += String(ma, 1);
  newValues += "mA ";
  newValues += String(v, 1);
  newValues += "V";
  DisplayIfChanged(1, newValues);
}

void DisplayR(byte row, float &vcc, float &r)
{
  String newValues = String(vcc, 1);
  newValues += "V: R=";
  newValues += String(r, 0);
  newValues += " [";
  //newValues += String(E12Value(r), 0);
  //newValues += E12FormattedValue(r);
  char buf[EseriesBufSize];
  newValues += String() + E12FormattedValue(buf, sizeof(buf), r, 'R');
  newValues += "]";
  DisplayIfChanged(row, newValues);
}

void ShowPwm(int dutyCycle)
{
  String newValues = PROGRAM;
  if (dutyCycle < 255)
  {

    newValues += " pwm";
    newValues += String(dutyCycle * 100 / 255);
    newValues += "%";
  }
  DisplayIfChanged(0, newValues);
}

void ShowRawData(byte ledNumber, byte pwm, unsigned int adcSum_i, unsigned int qtSamples_i, unsigned int adcSum_u, unsigned int qtSamples_u)
{
  String newValues = PROGRAM;
  newValues += " RAWdata";
  DisplayIfChanged(0, newValues);

  newValues = "LED";
  newValues += String(ledNumber);
  newValues += " pwm=";
  newValues += String(pwm);
  newValues += " adcSum:";
  DisplayIfChanged(1, newValues);

  newValues = "i=";
  newValues += String(adcSum_i);
  newValues += "/";
  newValues += String(qtSamples_i);
  if (qtSamples_i > 0)
  {
    newValues += "=";
    newValues += String(adcSum_i / qtSamples_i);
  }
  DisplayIfChanged(2, newValues);

  newValues = "u=";
  newValues += String(adcSum_u);
  newValues += "/";
  newValues += String(qtSamples_u);
  if (qtSamples_u > 0)
  {
    newValues += "=";
    newValues += String(adcSum_u / qtSamples_u);
  }
  DisplayIfChanged(3, newValues);
}
