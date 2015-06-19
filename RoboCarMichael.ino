/*******************************************************************************
 * Runs the line-following cars built during the TU Summer Academy
 * 
 * Author: Michael Ripley
 * Device: Arduino Uno R3
 * Date: 2015-06-19
 * Version: 1
 ******************************************************************************/

#include <Servo.h>

/* Begin Constants */
  #define PWM_STOP    90 // 0 is full reverse, 90 is stop, 180 is full forward
  #define PWM_REVERSE PWM_STOP - 90 // maximal offset from center is 90
  #define PWM_FORWARD PWM_STOP + 90 // an offset of 67 is 75% of the max speed
/* End Constants */

/* Analog Inputs*/
  int leftEyePin = 0;
  int rightEyePin = 1;
  int thresholdPin = 2;  // To potentiometer
/* End Analog */

/* Digital Output */
  int ledPin = 13; // built-in LED
  int leftServoPin = 9;
  int rightServoPin = 10;
/* End Digital Output*/

/* Begin Variable Declarations */
  bool ledState = false;
  Servo leftServo;
  Servo rightServo;
/* End Variable Declarations */


void setup()
{
  // enable serial printing
  Serial.begin(9600);

  // set the LED pin as a digital output
  pinMode(ledPin, OUTPUT);

  // delay for a second before doing anything to prevent current spikes
  delay(1000);
  
  // set up servos
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  
  // test sequence (right, stop, left, stop)
  // syntax: testServos( leftValue, rightValue )
  testServos(PWM_STOP, PWM_FORWARD);
  testServos(PWM_STOP, PWM_STOP);
  testServos(PWM_REVERSE, PWM_STOP);
  testServos(PWM_STOP, PWM_STOP);
}


void testServos(int leftValue, int rightValue)
{
  leftServo.write(leftValue);
  rightServo.write(rightValue);
  delay(1000);
}


void loop()
{
  // read analog inputs
  int thresholdLevel = analogRead(thresholdPin);
  int leftEyeDarkness = analogRead(leftEyePin);
  int rightEyeDarkness = analogRead(rightEyePin);
  
  
  // debug printing
  Serial.print("Reverse: ");
  Serial.println(PWM_REVERSE);
  Serial.print("Forward: ");
  Serial.println(PWM_FORWARD);

  // flash the LED
  digitalWrite(ledPin, ledState);
  ledState = !ledState;
  
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
  
  if (rightEyeDarkness < thresholdLevel)
  {
    // if right goes low (brighter light detected), stop right motor
    rightServo.write(PWM_STOP);
  }
  else if (leftEyeDarkness < thresholdLevel)
  {
    // if left goes low (brighter light detected), stop left motor 
    leftServo.write(PWM_STOP);
  }
  else
  {
    // both are high, drive normally
    leftServo.write(PWM_REVERSE); // the left servo is backwards!
    rightServo.write(PWM_FORWARD);
  }

  delay(50);
}
