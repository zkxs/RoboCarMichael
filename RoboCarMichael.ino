/*******************************************************************************
 * Runs the line-following cars built during the TU Summer Academy. The sensors
 * should straddle the tape line so that they are only over tape if the robot
 * needs to self-adjust. If the potentiometer is turned all the way CCW, the
 * servos will both stop, allowing them to be calibrated easily. While the
 * program is looping (begins after the startup test) the LED built into the
 * board (labeled 'L') will blink in a heartbeat. If something looks wrong, such
 * as the potentiometer being a short or an open, the heartbeat will be much
 * faster than normal as this causes the robot to stress out.
 * 
 * Author: Michael Ripley
 * Device: Arduino Uno R3
 * Date: 2015-06-19
 * Version: 2.0
 ******************************************************************************/

#include <Servo.h>

/* Begin Constants */
  // set to nonzero to enable serial debugging
  #define DEBUG 0
  
  // set to nonzero to enable startup test (it checks if the servos work)
  #define STARTUP_TEST 0 
  
  // you can change this to throttle the servos (percentage of max speed)
  #define PWM_THROTTLE 0.75
  
  // how much jitter to ignore in the potentiometer (0% tolerates no jitter)
  #define JITTER_PERCENT 0.05 
  
  // how long to pause at the end of each loop (in ms)
  #define LOOP_DELAY 50
  
  // How long to keep the LED on. Must not be less than LOOP_DELAY (in ms)
  #define LED_ON_TIME 50
  
  // How long to keep the LED of. Must not be less than LOOP_DELAY (in ms)
  #define LED_OFF_TIME 1000
  
  
  /* The following constants are computed from the previous values */
  
  #define PWM_STOP     90 // 0 is full reverse, 90 is stop, 180 is full forward
  #define PWM_REVERSE  PWM_STOP - (int)(90 * PWM_THROTTLE)
  #define PWM_FORWARD  PWM_STOP + (int)(90 * PWM_THROTTLE)
  
  #define THRESHOLD_JITTER (int)(1024 * JITTER_PERCENT)
  #define THRESHOLD_MIN THRESHOLD_JITTER
  #define THRESHOLD_MAX 1023 - THRESHOLD_JITTER

  // how long the LED stays on
  #define LED_BEAT_TRIP_COUNT (int)(LED_ON_TIME / LOOP_DELAY)
   // how often the LED pulses slowly
  #define LED_TRIP_COUNT_SLOW (int)(LED_OFF_TIME / LOOP_DELAY)
  #define LED_TRIP_COUNT_SLOW_BEAT (int)(LED_TRIP_COUNT_SLOW * 0.2)
  // how often the LED pulses quickly
  #define LED_TRIP_COUNT_FAST (int)(LED_TRIP_COUNT_SLOW * 0.2) 
  #define LED_TRIP_COUNT_FAST_BEAT (int)(LED_TRIP_COUNT_FAST * 0.2)
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
  bool ledState = false; // if the LED should be on or not
  bool ledBeat = false; // if we need to do a heartbeat next
  bool error = false; // if we have detected an error
  unsigned int ledTripCount = LED_TRIP_COUNT_SLOW;
  Servo leftServo;
  Servo rightServo;
/* End Variable Declarations */

/**
 * One-time setup
 */
void setup()
{
  #if DEBUG
    // enable serial printing
    Serial.begin(9600);
  #endif
  
  // set the LED pin as a digital output
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);
  
  // delay for a second before doing anything to prevent current spikes
  delay(1000);
  
  #if DEBUG
    Serial.println("Starting...");
  #endif
  
  // set up servos
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  
  // test sequence (right, stop, left, stop)
  #if STARTUP_TEST
    testServos(PWM_STOP, PWM_FORWARD);
    testServos(PWM_STOP, PWM_STOP);
    testServos(PWM_REVERSE, PWM_STOP);
    testServos(PWM_STOP, PWM_STOP);
  #endif
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

/**
 * Called repeatedly during program runtime
 */
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
  if (ledTripCount-- == 0)
  {
    ledState = !ledState;
    
    // reset trip count
    if (ledState)
    {
      ledTripCount = LED_BEAT_TRIP_COUNT;
    }
    else
    {
      // beat faster if something looks wrong
      if (error)
      {
        if (ledBeat) // if the next delay is between beats
        {
          ledTripCount = LED_TRIP_COUNT_FAST_BEAT;
        }
        else
        {
          ledTripCount = LED_TRIP_COUNT_FAST;
        }
        // reset error state
        error = false;
      }
      else
      {
        if (ledBeat) // if the next delay is between beats
        {
          ledTripCount = LED_TRIP_COUNT_SLOW_BEAT;
        }
        else
        {
          ledTripCount = LED_TRIP_COUNT_SLOW;
        }
      }
      ledBeat = !ledBeat;
    }
    
    digitalWrite(ledPin, ledState);
  } // end LED flashing code
  
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
  
  if (thresholdLevel >= THRESHOLD_MAX || thresholdLevel <= THRESHOLD_MIN)
  {
    // let people know if the potentiometer is shorted or open.
    error = true;
  }
  
  if (thresholdLevel >= THRESHOLD_MAX) // allow a tiny ammount of jitter from 1023
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
  delay(LOOP_DELAY);
}
