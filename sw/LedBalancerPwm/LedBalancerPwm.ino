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

#include "pinsMap.h"
#include "pwmAdc.h"
#include "noPwmAdc.h"


/*
#include <avr/io.h>
#include <avr/wdt.h>

#define Reset_AVR() wdt_enable(WDTO_30MS); while(1) {}
*/


// ADC parameters:
#define QtSamples 64 // every AD channel will be read <QtSamples> times
#define Clock_ms 300 // how many milliseconds between each LED read 

#include "display.h"
#include <ManyButtons.h>
#include "business.h"

ManyButtons_PinsAre(BUTTON_LEFT_PIN, BUTTON_RIGHT_PIN, BUTTON_ESC_PIN, BUTTON_ENTER_PIN, BUTTON_MAIN_PIN, BUTTON_1_PIN, BUTTON_2_PIN, BUTTON_3_PIN);

//#define _DEBUG

unsigned long ms;
const char *PROGRAM = "Led Balancer";

bool pwmOn = false;
bool rawData = false;

void setup()
{
  Serial.begin(74880);
  Serial.println(PROGRAM);
  //Serial.flush();

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // set PCB led off (nano led on)

  display_init(PROGRAM);
  //Serial.println("lcd init complete");
  //Serial.flush();
  delay(3000);          // wait for a while
  DisplayIfChanged(0, "MAIN toggle R / PWM");
  DisplayIfChanged(1, "  (must set jumpers)");
  DisplayIfChanged(2, "- and + change PWM");
  DisplayIfChanged(3, "ESC toggle raw data");
  delay(8000);          // wait for a while

  // initialize the buttons lib (set buttons pin as pull-ups ...):
  ManyButtons::Init(mbEVENT_RELEASE | mbEVENT_LONGPRESS);
  /*
  Serial.print("buttons init complete, nb:");
  Serial.println(MultipleButtonsClass::numbuttons);
  Serial.print("left pin: ");
  Serial.println(MultipleButtonsClass::DigitalPin[0]);
  Serial.flush();
  */
  pwmOn = false;

  if(pwmOn)
    pwmAdc_init(EXTERNAL);
  else
    noPwmAdc_init(EXTERNAL);

  //
  ms = millis();
}



int i_pin = LED1_I_PIN;
int u_pin = LED1_U_PIN;
char pwmCmd = 0;
byte ledNumber = 1;
bool pressedLong = false;

void loop()
{
  ManyButtons::Check();
  if (millis() - ms < Clock_ms)
    return; // wait until the proper time comes

  pwmCmd = 0;
  buttonStatus b = ManyButtons::getButtonEvent();
  if (b.event != mbEVENT_NO_EVENT)
  {
#ifdef _DEBUG
    Serial.print("[");
    Serial.print(millis());
    Serial.print(" ms] pin: ");
    Serial.print(b.pin);
    Serial.print(" event: ");
    Serial.print(b.event);
    Serial.print(" ");
#endif //_DEBUG
    switch (b.pin)
    {
    case BUTTON_1_PIN:
      i_pin = LED1_I_PIN; u_pin = LED1_U_PIN; ledNumber = 1; Serial.println("1");
      break;
    case BUTTON_2_PIN:
      i_pin = LED2_I_PIN; u_pin = LED2_U_PIN; ledNumber = 2; Serial.println("2");
      break;
    case BUTTON_3_PIN:
      i_pin = LED3_I_PIN; u_pin = LED3_U_PIN; ledNumber = 3; Serial.println("3");
      break;
    case BUTTON_LEFT_PIN:
      pwmCmd='-'; Serial.println("-");
      pressedLong = (b.event == mbEVENT_LONGPRESS);
      break;
    case BUTTON_RIGHT_PIN:
      pwmCmd='+'; Serial.println("+");
      pressedLong = (b.event == mbEVENT_LONGPRESS);
      break;
    case BUTTON_ESC_PIN:
      Serial.println("ESC");
      rawData=!rawData;
      break;
    case BUTTON_ENTER_PIN:
      Serial.println("ENTER");
      break;
    case BUTTON_MAIN_PIN:
      Serial.println("MAIN");
      //Reset_AVR()
      pwmOn=!pwmOn;
      if(pwmOn)
        pwmAdc_init(EXTERNAL);
      else
        noPwmAdc_init(EXTERNAL);
      break;
    default:
      Serial.println("unknown");
    }
  }

  setCurrentLed(ledNumber);

  // pwm set
  int dutyCycle;
  if(pwmOn)
  {
    dutyCycle = pwmGet(ledNumber);

  if(b.event == mbEVENT_NO_EVENT)
    b.pin = ManyButtons::getPressedButton();
#ifdef _DEBUG
  if(b.pin > 0)
  {
    Serial.print("pressedButtonPin: ");
    Serial.print(pin);
    Serial.print(" pressedLong: ");
    Serial.println(pressedLong);
  }
#endif //_DEBUG
  if(b.pin == BUTTON_LEFT_PIN || b.pin == BUTTON_RIGHT_PIN)
  {
    if(b.pin == BUTTON_LEFT_PIN)
    {
      dutyCycle -= pressedLong? 5 : 1;
      if(dutyCycle < 0)
        dutyCycle=0;
    }
    else
    {
      dutyCycle += pressedLong? 5 : 1;
      if(dutyCycle > 255)
        dutyCycle=255;
    }
    pwmSet(ledNumber, dutyCycle);
    dutyCycle = pwmGet(ledNumber); // re-read effective value. Pwm is limited in this circuit: cannot reach 0 nor 255
  }
  else
    pressedLong = false;
  }
  else
    dutyCycle = 255;
  
  // ADC read:
  unsigned int i_adcSum = 0;
  unsigned int u_adcSum = 0;
  unsigned int i_qtSamples, u_qtSamples;
  if(pwmOn)
  {  
    analogGet(i_pin, i_adcSum, i_qtSamples);
    analogGet(u_pin, u_adcSum, u_qtSamples);
  }
  else
  {
    i_qtSamples = u_qtSamples = QtSamples;
    noPwm_analogGet(i_pin, i_adcSum, i_qtSamples);
    noPwm_analogGet(u_pin, u_adcSum, u_qtSamples);
  }
  
  ManyButtons::Check();

  if(rawData)
    ShowRawData(ledNumber, dutyCycle, i_adcSum, i_qtSamples, u_adcSum, u_qtSamples);
  else
  {
    ShowPwm(dutyCycle);
    float ma, v, vcc, r;
    ComputeValues(ledNumber, dutyCycle, i_adcSum, i_qtSamples, u_adcSum, u_qtSamples, ma, v);
    ManyButtons::Check();
    DisplayLedParams(ledNumber, ma, v);
    ManyButtons::Check();
    vcc = 5.0;
    ComputeR(ma, v, vcc, r);
    DisplayR(2, vcc, r);
    ManyButtons::Check();
    vcc = 3.3;
    ComputeR(ma, v, vcc, r);
    DisplayR(3, vcc, r);
  //ManyButtons::Check();
  }

  ms += Clock_ms; // prepare for next step
}

