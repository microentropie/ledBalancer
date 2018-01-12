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

#include <arduino.h>

#include "pinsMap.h"


#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


/*
 Need to read 2 analog values (i and u) from a pwm driven led.
 Read has only sense in the low transition of pwm (otherwise will read 0) i.e. when LED is powered;
 see schematic for details.
 Set interrupt from PWM timers so we know when to start ADC.
 As the same pwm => led requires 2 dinstinct reads, and the cannot be done at the same time,
 adc read are alternated.
 (1) when HIGH-to-LOW transition of pwm
 (2) start ADC
 (3) store result when done

 NOTE: the ADC registers have been reprogrammed in order to make quicker ADC conversions (prescaler set to x8)
 a full ADC read (and store into array) occurs in abt 15-20 us from within Interrupt handler.
*/

#define DEBUG_LED_ON {PORTB &= ~(_BV(PINB5));}
#define DEBUG_LED_OFF {PORTB |= _BV(PINB5);}



void noPwmAdc_init(byte adcReference)
{
  // REF02 IC is connected to the AREF Arduino pin,
  // if the following command is missing, ATMEGA may be damaged
  analogReference(adcReference);
  //analog_reference = adcReference;

  // init ADC (the standard Arduino Uno initialization code)
  ADCSRA = 0;
  sbi(ADCSRA, ADPS2); // set prescaler to 128 => 125 KHz
  sbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);
  sbi(ADCSRA, ADEN);  // enable ADC

  // turn off PWM:
  analogWrite(PWM_1_PIN, 255);
  analogWrite(PWM_2_PIN, 255);
  analogWrite(PWM_3_PIN, 255);
}


void noPwm_setCurrentLed(byte numLed)
{
}


// ATMEGA and ATTINY use 16 bit integers:
// as ADC is 10 bit wide, a max of 64 samples can be stored
void noPwm_analogGet(byte pin, unsigned int &adc, unsigned int &qtSamples)
{
  if (qtSamples < 1) qtSamples = 1;
  else if (qtSamples > 64) qtSamples = 64;

  adc = 0;
  DEBUG_LED_ON;
  for (int i = 0; i < qtSamples; ++i)
    adc += analogRead(pin);
  DEBUG_LED_OFF;
}


byte noPwm_pwmGet(byte numLed)
{
  // pwm not enabled
  return 0;
}

void noPwm_pwmSet(byte pin, int value)
{
  // pwm not enabled
}
