

/*
A Programm to test Output Pins 3 9 and 10
for Developing a fast Brushless Gimbal Control
*/


#include "variables.h"
#include "definitions.h"
#include "Timer1.h"

//cbi and sbi are standard (AVR) methods for setting, or clearing, bits in PORT (and other) variables.
//To use them pass them a PORT variable, and a pin to set (or clear).
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif




void calcSinusArray(){
  for(int i=0; i<N_SIN; i++){
    //pwmSinMotor[i] =  (sin(2.0 * i / N_SIN * 3.14159265) * 127)+127;
    pwmSinMotor[i] =  sin(2.0 * i / N_SIN * 3.14159265) * 127.0;
  }
}


void setupMotors(){
  // Direction Register:
  DDRD = DDRD | B00001000; //Set Pin3 as Output
  DDRB = DDRB | B00001110; //Set Pin9 and 10 and 11 as Output
  //DDRD=0xFC;      // direction variable for port D - make em all outputs except serial pins 0 & 1
  //DDRB=0xFF;      // direction variable for port B - all outputs

  //Set Pin3 to High
  //PORTD = PORTD | B00001000;
  //Set Pin9 and 10 and 11 to High
  // PORTB = PORTB | B00001110;


#ifdef PWM_80HZ_FAST //30,6
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) |  _BV(WGM10);; //use Phase-correct PWM on Timer 0
  TCCR0B = _BV(CS02) | _BV(CS00); // Prescalefactor 1024
  TCCR1A = _BV(WGM12) |_BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS12) | _BV(CS10); // Prescalefactor 1024
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) |_BV(WGM20);
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); // Prescalefactor 1024
#endif

#ifdef PWM_80HZ_PHASE //30,6
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); //use Phase-correct PWM on Timer 0
  TCCR0B = _BV(CS02) | _BV(CS00); // Prescalefactor 1024
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS12) | _BV(CS10); // Prescalefactor 1024
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); // Prescalefactor 1024
#endif

#ifdef PWM_8KHZ_FAST
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01)| _BV(WGM10);
  TCCR0B = _BV(CS01); // Prescalefactor 8
  TCCR1A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS11); // Prescalefactor 8
  TCCR2A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM21)| _BV(WGM20);
  TCCR2B = _BV(CS21); // Prescalefactor 8
#endif
#ifdef PWM_32KHZ_PHASE
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
  TCCR0B = _BV(CS00); // No Prescaling
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS10); // No Prescaling
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20); // No Prescaling
#endif
#ifdef PWM_4KHZ_PHASE//8
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); //use Phase-correct PWM on Timer 0
  TCCR0B = _BV(CS01); // Prescalefactor 8
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS11); // Prescalefactor 8
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS21); // Prescalefactor 8
#endif

  // http://www.instructables.com/id/Arduino-Timer-Interrupts/step2/Structuring-Timer-Interrupts/
  // http://letsmakerobots.com/content/arduino-101-timers-and-interrupts
  // http://arduinodiy.wordpress.com/2012/02/28/timer-interrupts/
  // disable interrupts to avoid timing distortion
  sbi (TIMSK1,TOIE1);              // enable Timer1 overflow Interrupt
    //TIMSK0 &= ~_BV(TOIE1);
  cbi (TIMSK0,TOIE1);              // disable Timer0 !!! delay() is now not available
  //sbi (TIMSK2,TOIE2);              // enable Timer2 Interrupt dont do this!

  // sei: http://playground.arduino.cc/Main/AVR
  sei(); // enable interrupts

  // Setting the Output Pins to 0
  OCR2A = 0;  //11  APIN
  OCR2B = 0;  //D3
  OCR1A = 0;  //D9  CPIN
  OCR1B = 0;  //D10 BPIN
  OCR0A = 0;  //D6
  OCR0B = 0;  //D5

  cli(); // avr function to enable interrubts
  calcSinusArray();
  sei(); // avr function to disable interrubts


  //dfreq=1000.0;                    // initial output frequency = 1000.o Hz
  //dfreq=0.0;                    // initial output frequency = 1000.o Hz
  //tword_m=pow(2,32)*dfreq/refclk;  // calulate DDS new tuning word

}



void setMotorMove(uint8_t motorNumber, int MotorPos, uint16_t maxPWM)
{
  uint16_t posStep;
  uint16_t pwm_a;
  uint16_t pwm_b;
  uint16_t pwm_c;

  // fetch pwm from sinus table
  posStep = MotorPos & 0xff;
  pwm_a = pwmSinMotor[(uint8_t)posStep];
  pwm_b = pwmSinMotor[(uint8_t)(posStep + 85)];
  pwm_c = pwmSinMotor[(uint8_t)(posStep + 170)];

  // apply power factor
  pwm_a = maxPWM * pwm_a;
  pwm_a = pwm_a >> 8;
  pwm_a += 128;

  pwm_b = maxPWM * pwm_b;
  pwm_b = pwm_b >> 8;
  pwm_b += 128;

  pwm_c = maxPWM * pwm_c;
  pwm_c = pwm_c >> 8;
  pwm_c += 128;

  // set motor pwm variables
  if (motorNumber == 0)
  {
    pwm_a_motor0 = (uint8_t)pwm_a;
    pwm_b_motor0 = (uint8_t)pwm_b;
    pwm_c_motor0 = (uint8_t)pwm_c;
  }

  if (motorNumber == 1)
  {
    pwm_a_motor1 = (uint8_t)pwm_a;
    pwm_b_motor1 = (uint8_t)pwm_b;
    pwm_c_motor1 = (uint8_t)pwm_c;
  }
}




// Timer2 Interrupt Service at 31372,550 KHz = 32uSec
// this is the timebase REFCLOCK for the DDS generator
// FOUT = (M (REFCLK)) / (2 exp 32)
// runtime : 8 microseconds ( inclusive push and pop)
ISR(TIMER1_OVF_vect) {

  freqCounter++;
  if(freqCounter==(CC_FACTOR*1000/MOTORUPDATE_FREQ))
  {
    freqCounter=0;

    OCR1A = pwm_a_motor0;
    OCR1B = pwm_b_motor0;
    OCR2B = pwm_c_motor0;

    // update event
    motorUpdate = true;

  }

  // care for standard timers every 1 ms
  if ((freqCounter & 0x01f) == 0) {
    TIMER0_isr_emulation();
  }
}

// switch off motor power
void motorPowerOff() {
  setMotorMove(config.motorNumberPitch, 0, 0);
  setMotorMove(config.motorNumberRoll, 0, 0);
}



