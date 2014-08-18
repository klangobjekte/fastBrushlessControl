#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#endif // DEFINITIONS_H

//#define PWM_80HZ_PHASE
//#define PWM_80HZ_FAST
// Define Brushless PWM Mode, uncomment ONE setting
#define PWM_32KHZ_PHASE  // Resolution 8 bit for PWM
//#define PWM_8KHZ_FAST    // Resolution 8 bit for PWM
//define PWM_4KHZ_PHASE   // Resolution 8 bit for PWM
//#define NO_PWM_LOOP

#ifdef PWM_32KHZ_PHASE
  #define CC_FACTOR 32
#endif
#ifdef PWM_4KHZ_PHASE
  #define CC_FACTOR 4
#endif
#ifdef PWM_8KHZ_FAST
  #define CC_FACTOR 8
#endif
#ifdef NO_PWM_LOOP
  #define CC_FACTOR 1
#endif

#define MOTORUPDATE_FREQ 500                 // in Hz, 1000 is default
#define LOOPUPDATE_FREQ MOTORUPDATE_FREQ     // loop control sample rate equals motor update rate
#define DT_FLOAT (1.0/LOOPUPDATE_FREQ*1.024) // loop controller sample period dT
#define DT_INT_MS (1000/MOTORUPDATE_FREQ)    // dT, integer, (ms)
#define DT_INT_INV (MOTORUPDATE_FREQ)        // dT, integer, inverse, (Hz)





// Hardware Abstraction for Motor connectors,
// DO NOT CHANGE UNLES YOU KNOW WHAT YOU ARE DOING !!!
// http://arduino.cc/en/Tutorial/SecretsOfArduinoPWM
//http://astro.neutral.org/arduino/arduino-pwm-pins-frequency.shtml
#define PWM_A_MOTOR1 OCR2A // Pin 10
#define PWM_B_MOTOR1 OCR1B // Pin 12
#define PWM_C_MOTOR1 OCR1A // Pin 11

#define PWM_A_MOTOR0 OCR0A // Pin 13
#define PWM_B_MOTOR0 OCR0B // Pin 3
#define PWM_C_MOTOR0 OCR2B // Pin 9

#define N_SIN 256

