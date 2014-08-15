
/*
 PWMallPins.pde
 Paul Badger 2007
 A program to illustrate one way to implement a PWM loop.
 This program fades LED's on Arduino digital pins 2 through 13 in a sinewave pattern.
 It could be modified to also modulate pins 0 & 1 and the analog pins.
 I didn't modulate pins 0 & 1 just because of the hassle of disconnecting the LEDs on the RX and TX pins (0 & 1).

 The PWM loop, as written, operates at about 175 HZ and is flicker-free.
 The trick to this of course is not doing too much math in between the PWM loop cycles.
 Long delays between the loops are going to show up as flicker. This is true especially of "Serial.print" debug statements
 which seem to hog the processor during transmission. Shorter (timewise) statements will just dim the maximum brightness of the LED's.
 There are a couple of lines of code (commented out) that implement a potentiometer as a speed control for the dimming.

 How it works: The PWM loop turns on all LED's whose values are greater than 0 at the start of the PWM loop.
 It then turns off the LED's as the loop variable is equal to the channel's PWM modulation value.
 Because the values are limited to 255 (just by the table), all LED's are turned off at the end of the PWM loop.
 This has the side effect of making any extra math, sensor reading. etc. will increase the "off" period of the LED's duty cycle. Consequently
 the brightest LED value (255), which is supposed to represent a "100%" duty cycle (on all the time), dosesn't really do that.
 More math, sensor reading etc will increase the "off time", dimming the LED's max brightness. You can (somewhat) make up for this dimming with
 smaller series resistors, since LED's can be overrated if they aren't on all of the time.

 The up side of this arrangement is that the LED's stay flicker free.
 Note that this program could easily be used to modulate motor speeds with the addition of driver transistors or MOSFET's.
 */
/*
A Programm to test Output Pins 3 9 and 10
for Developing a fast Brushless Gimbal Control
*/

//cbi and sbi are standard (AVR) methods for setting, or clearing, bits in PORT (and other) variables.
//To use them pass them a PORT variable, and a pin to set (or clear).
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


#define N_SIN 256



// variable for speed debug
long time;

// these constants set the rate of dimming
float pwmSpeed[14] = {0, 0, 1.2, 1.3, 1.4, 1.9, .9,
                  .8, .5, 1.2, 1.37, 1.47, .3, 3.2};

int outputpins[3] ={9,10,3};

// PWM values for 12 channels - 0 & 1 included but not used
int16_t pwmVal[14];

float pwmFloats[14];

// variables for various counters
int i, k, l, x, y, z, bufsize, pot;
int j;


uint8_t sinewave[N_SIN];

void calcSinusArray(){
  for(int i=0; i<N_SIN; i++){
    sinewave[i] =  (sin(2.0 * i / N_SIN * 3.14159265) * 127)+127;
    //pwmSinMotor[i] =  sin(2.0 * i / N_SIN * 3.14159265) * 127.0;
  }
}


const int motorPin1 =9;
const int motorPin2 =10;
const int motorPin3 =3;

double dfreq;
const double refclk=32250;      // not measured
volatile byte icnt;              // var inside interrupt
volatile byte icnt1;             // var inside interrupt
volatile byte c4ms;              // counter incremented all 4ms
volatile unsigned long phaccu;   // phase accumulator
volatile unsigned long tword_m;  // dds tuning word m


void setupMotors(){
  //Serial.begin(9600);
  Serial.begin(115200);
  Serial.println(F("Setup..."));
  Serial.println(j);
  calcSinusArray();


  // Direction Register:
  DDRD = DDRD | B00001000; //Set Pin3 as Output
  DDRB = DDRB | B00001110; //Set Pin9 and 10 and 11 as Output
  //DDRD=0xFC;      // direction variable for port D - make em all outputs except serial pins 0 & 1
  //DDRB=0xFF;      // direction variable for port B - all outputs

  //Set Pin3 to High
  //PORTD = PORTD | B00001000;
 //Set Pin9 and 10 and 11 to High
 // PORTB = PORTB | B00001110;


  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); //use Phase-correct PWM on Timer 0
  TCCR0B = _BV(CS00); // Prescalefactor on timer 0 = 31250 Hz;
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS10);
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);


  // http://www.instructables.com/id/Arduino-Timer-Interrupts/step2/Structuring-Timer-Interrupts/
  // http://letsmakerobots.com/content/arduino-101-timers-and-interrupts
  // http://arduinodiy.wordpress.com/2012/02/28/timer-interrupts/
  // disable interrupts to avoid timing distortion
  cbi (TIMSK0,TOIE0);              // disable Timer0 !!! delay() is now not available
  sbi (TIMSK1,TOIE1);              // enable Timer1 overflow Interrupt
  sbi (TIMSK2,TOIE2);              // enable Timer2 Interrupt

  dfreq=1000.0;                    // initial output frequency = 1000.o Hz
  tword_m=pow(2,32)*dfreq/refclk;  // calulate DDS new tuning word

}

// http://interface.khm.de/index.php/lab/experiments/arduino-dds-sinewave-generator/
void moveMotors()
{
  while(true){
     // timer / wait for a full second
     if (c4ms > 250) {
      //read Poti on analog pin 0 to adjust output frequency from 0..1023 Hz
      //c4ms=0;
      //dfreq=analogRead(0);

      //disble Timer2 Interrupt
      cbi (TIMSK2,TOIE2);

      tword_m=pow(200,32)*dfreq/refclk;
      //enable Timer1 overflow Interrupt
      sbi (TIMSK1,TOIE1);
      //enable Timer2 overflow Interrupt
      sbi (TIMSK2,TOIE2);
      //Serial.print(dfreq);
      //Serial.print("  ");
      //Serial.println(tword_m);
     }

   //sbi(PORTD,6); // Test / set PORTD,7 high to observe timing with a scope
   //cbi(PORTD,6); // Test /reset PORTD,7 high to observe timing with a scope
  }
 }



// Timer2 Interrupt Service at 31372,550 KHz = 32uSec
// this is the timebase REFCLOCK for the DDS generator
// FOUT = (M (REFCLK)) / (2 exp 32)
// runtime : 8 microseconds ( inclusive push and pop)
ISR(TIMER1_OVF_vect) {

  //sbi(PORTD,7);          // Test / set PORTD,7 high to observe timing with a oscope

  phaccu=phaccu+tword_m; // soft DDS, phase accu with 32 bits
  icnt=phaccu >> 24;     // use upper 8 bits for phase accu as frequency information
                         // read value fron ROM sine table and send to PWM DAC
  //OCR1A=pgm_read_byte_near(sinewave + icnt);
  //OCR1B=pgm_read_byte_near(sinewave + icnt);
  OCR2B=pgm_read_byte_near(sinewave + icnt);

  if(icnt1++ == 125) {  // increment variable c4ms all 4 milliseconds
    c4ms++;
    icnt1=0;
   }

   //cbi(PORTD,7);            // reset PORTD,7
}



