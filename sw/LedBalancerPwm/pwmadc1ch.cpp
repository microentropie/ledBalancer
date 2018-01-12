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

#include "pwmadc.h"

#ifndef ARDUINO_AVR_UNO
#error ("Unsupported CPU. This library is for ATMEGA 328 only")
#endif //ARDUINO_AVR_UNO

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/*
https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
Timer output  Arduino output  Pin name
OC0A          6               PD6
OC0B          5               PD5
OC1A          9               PB1
OC1B          10              PB2
OC2A          11              PB3
OC2B          3               PD3
*/

// AtMega 328 has 8 ADC pins, multiplexed: only 1 conversion at a time can occur
#define QtAdc 8

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

// AtMega and AtTiny use 16 bit integers and have a 
// 10 bit ADC. The value (0 .. 1023) can be added up to 64 times
// 1023 * 64 = 65472
static volatile uint16_t adcRead[QtAdc];
static volatile uint16_t adcQtsamples[QtAdc];

extern uint8_t analog_reference; // reference to the Arduino internal variable used in wiring_analog.c



void DoConversion(byte adcPin, byte outPinBitPortB)
{
  sbi(ADCSRA, ADEN);  // enable ADC
  //while (bit_is_set(ADCSRA, ADSC)); // wait for conversion to complete
#if defined(ADCSRB) && defined(MUX5)
  // the MUX5 bit of ADCSRB selects whether we're reading from channels
  // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((adcPin >> 3) & 0x01) << MUX5);
#endif 
#if defined(ADMUX)
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = (analog_reference << 4) | (adcPin & 0x07);
#else
  ADMUX = (analog_reference << 6) | (adcPin & 0x07);
#endif
#endif
  sbi(ADCSRA, ADSC); // start conversion

  while (bit_is_set(ADCSRA, ADSC)); // wait for conversion to complete
  if (!(PINB & outPinBitPortB)) // only accept adc value if pwm pin was not changed in the meantime
  {
    adcRead[adcPin] += ADC; // read 10 bit value from ADC
    ++adcQtsamples[adcPin];
  }
  cbi(ADCSRA, ADEN);  // disable ADC (required before channel change)
}



static volatile byte currentLed;

// ISR for LED1
// alternatively read ADC0 and ADC3
// PWM pin is PB1
static volatile byte adcMux03 = 0;
ISR(TIMER1_COMPA_vect)
{
  // with the ADC prescaler settings:  this interrupt will take
  // x16                               ~16 us
  // x32                               ~35 us
  if (currentLed != 1) return;
  if (PINB & _BV(PINB1)) return; // only read adc when pwm out is high

  // do ADC conversion
  byte adcPin = (adcMux03++ & 1) ? LED1_I_PIN : LED1_U_PIN;
  if (adcQtsamples[adcPin] < 64) // adc buffer full ?
  {
    DEBUG_LED_ON;
    DoConversion(adcPin, _BV(PINB1));
    DEBUG_LED_OFF;
  }
}

// ISR for LED2
// alternatively read ADC1 and ADC6
// PWM pin is PB2
static volatile byte adcMux16 = 0;
ISR(TIMER1_COMPB_vect)
{
  // with the ADC prescaler settings:  this interrupt will take
  // x16                               ~16 us
  // x32                               ~35 us
  if (currentLed != 2) return;
  if (PINB & _BV(PINB2)) return; // only read adc when pwm out is high

  // do ADC conversion
  byte adcPin = (adcMux16++ & 1) ? LED2_I_PIN : LED2_U_PIN;
  if (adcQtsamples[adcPin] < 64) // adc buffer full ?
  {
    DEBUG_LED_ON;
    DoConversion(adcPin, _BV(PINB2));
    DEBUG_LED_OFF;
  }
}


// ISR for LED3
// alternatively read ADC2 and ADC7
// PWM pin is PB3
static volatile byte adcMux27 = 0;
ISR(TIMER2_COMPA_vect)
{
  // with the ADC prescaler settings:  this interrupt will take
  // x16                               ~16 us
  // x32                               ~35 us
  if (currentLed != 3) return;
  if (PINB & _BV(PINB3)) return; // only read adc when pwm out is high

  // do ADC conversion
  byte adcPin = (adcMux27++ & 1) ? LED3_I_PIN : LED3_U_PIN;
  if (adcQtsamples[adcPin] < 64) // adc buffer full ?
  {
    DEBUG_LED_ON;
    DoConversion(adcPin, _BV(PINB3));
    DEBUG_LED_OFF;
  }
}

#define QtLed 3
static byte currentPwm[QtLed];
#define PWM_MIN 14

void pwmAdc_init(byte adcReference)
{
  int i;

  // REF02 IC is connected to the AREF Arduino pin,
  // if the following command is missing, ATMEGA may be damaged
  analogReference(adcReference);
  //analog_reference = adcReference;

  // init adc cache
  for (i = 0; i < QtAdc; ++i)
  {
    adcRead[i] = 0;
    adcQtsamples[i] = 0;
  }

  // start pwm:
  for (i = 0; i < QtLed; ++i)
    currentPwm[i] = 15;

  pinMode(PWM_1_PIN, OUTPUT);
  pinMode(PWM_2_PIN, OUTPUT);
  pinMode(PWM_3_PIN, OUTPUT);

  pwmSet(1, currentPwm[0]);
  pwmSet(2, currentPwm[1]);
  pwmSet(3, currentPwm[2]);

  currentLed = 1;

  // change pwm frequency

  cli();

  // init ADC
  /*
  Manual page 253
  By default, the successive approximation circuitry requires an input clock frequency between 50
  kHz and 200 kHz to get maximum resolution. If a lower resolution than 10 bits is needed, the
  input clock frequency to the ADC can be higher than 200 kHz to get a higher sample rate.
  */
  // Arduino Uno 16 MHz => ADC freq = 16E6 / 16 = 1 MHz
  // Arduino Uno 16 MHz => ADC freq = 16E6 / 32 = 500 KHz
  // Arduino Uno 16 MHz => ADC freq = 16E6 / 64 = 250 KHz
  ADCSRA = 0;
  sbi(ADCSRA, ADPS2); // set prescaler to 64 => 250 KHz
  sbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  /*
  sbi(ADCSRA, ADPS2); // set prescaler to x32 => 500 KHz
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);
  */
  /*
  sbi(ADCSRA, ADPS2); // set prescaler to x16 => 1 MHz
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  */
  sbi(ADCSRA, ADEN);  // enable ADC

  // enable INTerrupts for Timers:
  // enable timer compare interrupt for timer1A and Timer1B
  TIMSK1 |= _BV(OCIE1A) | _BV(OCIE1B);

  // enable timer compare interrupt for timer2A
  TIMSK2 |= _BV(OCIE2A);

  sei();
}


void setCurrentLed(byte numLed)
{
  currentLed = numLed;
}


// ATMEGA and ATTINY use 16 bit integers:
// as ADC is 10 bit wide, a max of 64 samples can be stored
void analogGet(byte pin, unsigned int &adc, unsigned int &qtSamples)
{
  if (pin < 0 || pin >= QtAdc)
  {
    adc = qtSamples = 0;
    return;
  }
  cli();
  adc = adcRead[pin];
  qtSamples = adcQtsamples[pin]; // multiply by duty cycle
  adcRead[pin] = 0;
  adcQtsamples[pin] = 0;
  sei();
}


static byte pwmPin(byte numLed)
{
  byte pin;

  if (numLed == 1)
    pin = PWM_1_PIN;
  else if (numLed == 2)
    pin = PWM_2_PIN;
  else if (numLed == 3)
    pin = PWM_3_PIN;
  else
    pin = 255;

  return pin;
}

byte pwmGet(byte numLed)
{
  return currentPwm[numLed - 1];
}

void pwmSet(byte numLed, byte value)
{
  if (value < PWM_MIN) value = PWM_MIN;
  else if (value > 254) value = 254;
  // a minumum pwm value is required for the ADC to do conversion
  // a maximum value is set becausethe pwm must remain active (or ISR will not take place and ADC won't read-in)

  currentPwm[numLed - 1] = value;
  analogWrite(pwmPin(numLed), 255 - value); // a transistor inverts the signal, see schematic
}
