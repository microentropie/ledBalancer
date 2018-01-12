#ifndef BUSINESS_H_
#define BUSINESS_H_

#ifndef byte
typedef uint8_t byte;
#endif //byte

void ComputeValues(byte ledNumber, byte pwm, unsigned int adcSum_i, unsigned int qtSamples_i, unsigned int adcSum_u, unsigned int qtSamples_u, float &ma, float &v);
void ComputeR(float &ma, float &v, float &vcc, float &r);
float E12Value(float r);
void DisplayLedParams(byte ledNumber, float &ma, float &v);
void DisplayR(byte row, float &vcc, float &r);
void ShowPwm(int dutyCycle);
void ShowRawData(byte ledNumber, byte pwm, unsigned int adcSum_i, unsigned int qtSamples_i, unsigned int adcSum_u, unsigned int qtSamples_u);

#endif //BUSINESS_H_
