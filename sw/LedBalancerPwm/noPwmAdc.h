#ifndef _NO_PWM_ADC_H_
#define _NO_PWM_ADC_H_

extern void noPwmAdc_init(byte adcReference);
//extern void noPwm_setCurrentLed(byte numLed);
extern void noPwm_analogGet(byte pin, unsigned int &adc, unsigned int &qtSamples);
//extern byte noPwm_pwmGet(byte numLed);
//extern void noPwm_pwmSet(byte pin, int value);

#endif //_NO_PWM_ADC_H_
