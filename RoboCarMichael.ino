/* ****************************************************** *
 * Super Michael version of the code that actually works  *
 * ****************************************************** */

#include <Servo.h>

/* Constants */
#define PWM_STOP    1500
#define PWM_REVERSE PWM_STOP - 375 // maximal offset from center is 500
#define PWM_FORWARD PWM_STOP + 375 // a value of 375 is 75% of the maximum speed

/* Analog Inputs*/
int leftEyePin = 0;
int rightEyePin = 1;
int threshold = 2;  // To potentiometer
/* End Analog */

/* Digital Output */
int leftServoPin = 9;
int rightServoPin = 10;
Servo leftServo;
Servo rightServo;
/* End Digital Output*/


void setup()
{
  // set up servos  
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  
  // test sequence (right, stop, left, stop)
  leftServo.writeMicroseconds(PWM_STOP);
  rightServo.writeMicroseconds(PWM_FORWARD);
  delay(1000);
  leftServo.writeMicroseconds(PWM_STOP);
  rightServo.writeMicroseconds(PWM_STOP);
  delay(1000);
  leftServo.writeMicroseconds(PWM_REVERSE);
  rightServo.writeMicroseconds(PWM_STOP);
  delay(1000);
  leftServo.writeMicroseconds(PWM_STOP);
  rightServo.writeMicroseconds(PWM_STOP);
  delay(1000);
  
}


void loop()
{
  // read analog inputs
  int thresholdLevel = analogRead(threshold);
  int leftEyeVal = analogRead(leftEyePin);
  int rightEyeVal = analogRead(rightEyePin);
  
  /* 
   * If the light sensor sees something shiny, it goes low (towards 0)
   * If it does not see light, it goes high (towards 1023)
   *
   * The goal is to keep the line between the sensors, such that if we are
   * centered on the tape, the sensors hang off and see carpet.
   * if right goes low, stop right motor
   * if left goes low, stop left motor 
   * if both high, do nothing
   */
  
  if (rightEyeVal < thresholdLevel)
  {
    // if right goes low, stop right motor
    rightServo.writeMicroseconds(PWM_STOP);
  }
  else if ( leftEyeVal < thresholdLevel)
  {
    // if left goes low, stop left motor 
    leftServo.writeMicroseconds(PWM_STOP);
  }
  else
  {
    // both are high, drive normally
    leftServo.writeMicroseconds(PWM_REVERSE); // the left servo is backwards!
    rightServo.writeMicroseconds(PWM_FORWARD);
  }           
}
