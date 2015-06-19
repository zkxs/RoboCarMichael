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

/* Begin Constants (all non-percentage constants are integral values) */
  // set to nonzero to enable serial debugging
  #define DEBUG 0
  
  // set to nonzero to enable startup test (it checks if the servos work)
  #define STARTUP_TEST 1

  // how long to delay on startup (in ms). This prevents current spikes.
  #define STARTUP_DELAY 250

  // how long a single servo test lasts (in ms)
  #define TEST_DURATION 250
  
  // you can change this to throttle the servos (percentage of max speed)
  #define PWM_THROTTLE 0.35
  
  // how much jitter to ignore in the potentiometer (0% tolerates no jitter)
  #define JITTER_PERCENT 0.05
  
  // how long to pause at the end of each loop (in ms)
  #define LOOP_DELAY 25
  
  // How long to keep the LED on. Must not be less than LOOP_DELAY (in ms)
  #define LED_ON_TIME 50
  
  // How long to keep the LED of. Must not be less than LOOP_DELAY (in ms)
  #define LED_OFF_TIME 1000
  
  // Minimum acceptable value of VCC (input voltage) (in mV)
  #define VCC_MINIMUM 4900
  
  
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
  #define LED_TRIP_COUNT_FAST (int)(LED_TRIP_COUNT_SLOW * 0.4) 
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
  delay(STARTUP_DELAY);
  
  #if DEBUG
    Serial.println("Starting...");
  #endif

  // check VCC
  long vcc = readVcc();
  if (vcc < VCC_MINIMUM)
  {
    #if DEBUG
      Serial.print("VCC was too low (");
      Serial.print(vcc / 1000.0);
      Serial.println("V) Aborting.");
    #endif
    die();
  }
  
  // set up servos
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  
  // test sequence (right, stop, left, stop)
  #if STARTUP_TEST
    testServos(PWM_STOP, PWM_FORWARD);
    testServos(PWM_STOP, PWM_REVERSE);
    testServos(PWM_STOP, PWM_STOP);
    testServos(PWM_REVERSE, PWM_STOP);
    testServos(PWM_FORWARD, PWM_STOP);
    testServos(PWM_STOP, PWM_STOP);
  #endif
}


/**
 * Test the servos with the supplied values for 0.5s
 * @param leftValue value to set the left servo to
 * @param rightValue value to set the right servo to
 */
void testServos(int leftValue, int rightValue)
{
  leftServo.write(leftValue);
  rightServo.write(rightValue);
  delay(TEST_DURATION); // block for 0.5 seconds
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
      long vcc = readVcc();
      if (vcc < VCC_MINIMUM)
      {
        error = true;
      }
      
      // debug printing
      #if DEBUG
        double vcc_double = vcc / 1000.0;
        Serial.print("vcc: ");
        Serial.println(vcc_double);
        Serial.print("Pot. Voltage: ");
        Serial.println(thresholdLevel / 1023.0 * vcc_double);
      #endif
      
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
          
          // reset error state
          error = false;
        }
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
} // end loop()


/**
 * 
 */
void die()
{
  while(true)
  {
    if (ledTripCount-- == 0)
    {
      ledState = !ledState;
      
      // reset trip count
      if (ledState)
      {
        ledTripCount = LED_BEAT_TRIP_COUNT;
      }
      else if (ledBeat) // if the next delay is between beats
      {
        ledTripCount = LED_TRIP_COUNT_FAST_BEAT;
        ledBeat = !ledBeat;
      }
      else
      {
        ledTripCount = LED_TRIP_COUNT_FAST;
        ledBeat = !ledBeat;
      }
      digitalWrite(ledPin, ledState);
    }
    delay(LOOP_DELAY);
  }
}

/**
 * Read the value of VCC (the power source). This has a pretty bad tolerance,
 * so don't use it for precise measurements.
 */
long readVcc()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
