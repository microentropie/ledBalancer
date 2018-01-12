#ifndef _PWM_ADC_H_
#define _PWM_ADC_H_

extern void pwmAdc_init(byte adcReference);
extern void setCurrentLed(byte numLed);
extern void analogGet(byte pin, unsigned int &adc, unsigned int &qtSamples);
extern byte pwmGet(byte numLed);
extern void pwmSet(byte numLed, byte value);

#endif //_PWM_ADC_H_
