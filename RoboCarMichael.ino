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
  #define DEBUG 1 // set to nonzero to enable debugging
  
  #define PWM_THROTTLE 0.75 // you can change this to throttle the servos
  #define PWM_STOP     90 // 0 is full reverse, 90 is stop, 180 is full forward
  #define PWM_REVERSE  PWM_STOP - (int)(90 * PWM_THROTTLE)
  #define PWM_FORWARD  PWM_STOP + (int)(90 * PWM_THROTTLE)
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
  #if DEBUG
    // enable serial printing
    Serial.begin(9600);
  #endif
  
  // set the LED pin as a digital output
  pinMode(ledPin, OUTPUT);

  // delay for a second before doing anything to prevent current spikes
  delay(1000);
  
  #if DEBUG
    Serial.println("Starting...");
  #endif
  
  // set up servos
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  
  // test sequence (right, stop, left, stop)
  testServos(PWM_STOP, PWM_FORWARD);
  testServos(PWM_STOP, PWM_STOP);
  testServos(PWM_REVERSE, PWM_STOP);
  testServos(PWM_STOP, PWM_STOP);
}

/**
 * Test the servos with the supplied values for one second
 * @param leftValue value to set the left servo to
 * @param rightValue value to set the right servo to
 */
void testServos(int leftValue, int rightValue)
{
  leftServo.write(leftValue);
  rightServo.write(rightValue);
  delay(1000); // block for 1 second
}


void loop()
{
  // read analog inputs
  int thresholdLevel = analogRead(thresholdPin);
  int leftEyeDarkness = analogRead(leftEyePin);
  int rightEyeDarkness = analogRead(rightEyePin);
  
  
  // debug printing
  #if DEBUG
    Serial.print("Threshold: ");
    Serial.println(thresholdLevel);
  #endif
  
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

   * Rotating the potentiometer counterclockwise raises its value, and clockwise
   * lowers it.  The measured bounds for the value were [0, 1023]
   * Turning the potentiometer completely CCW (1023) will make both sensors
   * always "see" tape.
   * Turning the potentiometer completely CW (0) will make both sensors always
   * "see" carpet.
   * 
   * The left servo is attached backwards, so it must be set to reverse if you
   * want it to move forward
   */
  
  
  if (thresholdLevel >= 1018) // allow a tiny ammount of jitter from 1023
  {
    /* if the potentiometer is completely CCW, normally we'd do the action for
     * when both sensors see tape. I'd rather stop the motors so that there is
     * a way to not drive the motors and still have the Arduino on. This
     * provides an easy way to stop the servos so they can be centered.
     */
     
     leftServo.write(PWM_STOP); // inverted
     rightServo.write(PWM_STOP);
  }
  else
  { // begin block to handle sensors
    bool rightSeesTape = rightEyeDarkness < thresholdLevel;
    bool leftSeesTape = leftEyeDarkness < thresholdLevel;
    
    if (rightSeesTape == leftSeesTape)
    {
      // neither sensor sees tape, drive normally...
      // OR both servos see tape! drive forwards and hope for the best!
      leftServo.write(PWM_REVERSE); // inverted
      rightServo.write(PWM_FORWARD);
    }
    else if (rightSeesTape)
    {
      // right sees tape, meaning we should turn left by stopping the right motor
      leftServo.write(PWM_REVERSE); // inverted
      rightServo.write(PWM_STOP);
    }
    else // it is implied that (leftSeesTape == true)
    {
      // left sees tape, meaning we should turn right by stopping the left motor
      leftServo.write(PWM_STOP);
      rightServo.write(PWM_FORWARD);
    }
  } // end block to handle sensors

  // delay between updates (in milliseconds)
  delay(50);
}
