#ifndef _PinsMap_h_
#define _PinsMap_h_

// Pwm pins:
#define PWM_1_PIN 9  //PB1 => driven by Timer1A
#define PWM_2_PIN 10 //PB2 => driven by Timer1B
#define PWM_3_PIN 11 //PB3 => driven by Timer2A

// ADC pins:
#define LED1_I_PIN 0 // connected to PWM_1_PIN
#define LED1_U_PIN 3 // connected to PWM_1_PIN

#define LED2_I_PIN 1 // connected to PWM_2_PIN
#define LED2_U_PIN 6 // connected to PWM_2_PIN

#define LED3_I_PIN 2 // connected to PWM_3_PIN
#define LED3_U_PIN 7 // connected to PWM_3_PIN

// Buttons          Uno pin: AVR Port:
#define BUTTON_LEFT_PIN   6  // PD6
#define BUTTON_RIGHT_PIN 12  // PB4
#define BUTTON_ESC_PIN    8  // PB0
#define BUTTON_ENTER_PIN  7  // PD7
#define BUTTON_MAIN_PIN   2  // PD2
#define BUTTON_1_PIN      3  // PD3
#define BUTTON_2_PIN      4  // PD4
#define BUTTON_3_PIN      5  // PD5

#endif //_PinsMap_h_
