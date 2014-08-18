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



#include "motorControl.h"
int reading = 0;
int position = 0;
void motoraction(int MotorPos)
{
    // usable Values between 180(min Power and 256)
    uint16_t power = 180;
        setMotorMove(0,MotorPos,power);

}


void setup(){
    Serial.begin(115200);
    Serial.println(F("Setup Program..."));
    setupMotors();

    // switch off PWM Power
    motorPowerOff();

    // Read Config, initialize if version does not match or CRC fails
    //config.configSet = readConfigSetNumberFromEEPROM(); // get set number from EEPROM
    //readEEPROM();
    //if (config.versEEPROM != VERSION_EEPROM)
    //{
    //printMessage(MSG_WARNING, F("EEPROM version mismatch, initialized to default"));
    setDefaultParameters();
    //  writeEEPROM();
    //}

}

// http://interface.khm.de/index.php/lab/experiments/arduino-dds-sinewave-generator/
void loop(){

    if (Serial.available()>0){
        int inByte = Serial.read();
        reading=inByte - '0';
        Serial.println(F("Entry"));
        Serial.println(reading);
        if(motorUpdate){
            motorUpdate = false;
            if(reading == 1){
                motoraction(31);
            }
            if(reading == 2){
                motoraction(63);
            }
            if(reading == 3){
                motoraction(95);
            }
            if(reading == 4){
                motoraction(127);
            }
            if(reading == 5){
                motoraction(159);
            }
            if(reading == 6){
                motoraction(191);
            }
            if(reading == 7){
                if(position < 0)
                    position = 255;
                motoraction(position);
                position-=1;
            }
            if(reading == 8){
                if(position > 255)
                    position = 0;
                motoraction(position);
                position+=1;
            }
            if(reading == 9){
                for(int i = 0; i<7;i++){
                    for(int y= 0;y<256;y++){
                        for(long delay = 0;delay < 30;delay++){
                            // do nothing except use time;
                            Serial.println(F("Use Time..."));
                            //float result;
                            //float z =333.333333333;
                            //float y = 333.33333333;
                            //result = z * y;
                        }
                        motoraction(y);
                    }
                }
            }
            if(reading == 0){
                //motoraction(0);
                motorPowerOff();
            }
        }
    }

    // motor update t=6us (*)
    /*
    if (enableMotorUpdates)
    {
      // set pitch motor pwm
      MoveMotorPosSpeed(config.motorNumberPitch, pitchMotorDrive, maxPWMmotorPitchScaled);
      // set roll motor pwm
      MoveMotorPosSpeed(config.motorNumberRoll, rollMotorDrive, maxPWMmotorRollScaled);
    }
    */
    //moveMotors();
}



int main()
{
  init();
  setup();
  while (true)
    loop();
    
    return 0;
}
