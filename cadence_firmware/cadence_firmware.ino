/*

Cadence Driver - Version 5.1.0 - 10/27/2020

Trigger actuators (solenoids or pumps) based on human heartbeats via a pulse sensor or messages from a host PC.

Pins:
    ACTUATOR 1 - D2/D11
    ACTUATOR 2 - D3
    Status LED - D5
    Heartbeat Sensor 1 - A0
    Heartbeat Sensor 2 - A1

Usage Notes:
    * Edit "High level configuration constants" to tune most of the application.
    * Pulse Sensor sample aquisition and processing happens in the background via Timer 1 interrupt. 1mS sample rate.
    * PWM on pins 9 and 10 will not work when using this code!
    
    * The following variables are automatically updated by the ISR:
      pulse_non_resetting:  boolean that is made true whenever pulse is found. User must reset.
      pulse_resetting:      boolean that is made true whenever pulse is found. ISR sets to false when the beat has completed.

    * When the host PC connects to twitch, the red LED on the PCB will blink 3 times quickly.
    * If the host PC becomes disconnected from twitch, the red LED on the PCB will blink twice slowly.
    * Set ACTUATOR_*_SERIAL_CONTROL to True if you want the corresponding actuator to be controlled via the serial port. False if you want it to be controlled by it's pulse sensor
    * Set ACTUATOR_*_MOTOR to True if it is a pump and false if it is a solenoid.
      
For support:
    dev@esologic.com

*/

#include "float.h"
#include "wrapCounter.h"

/*
  Debug and test mode config
*/

// If set to true, will print debug messages to the serial console
#define DEBUG_MODE false

// If set to True, both drippers will drip given the settings of sound_test_loop.
// This should NEVER be enabled for production.
#define SOUND_TEST_MODE_ENABLED false

// Configure sending mock commands for testing
// This should NEVER be enabled for production. 
#define FAKE_COMMANDS_ENABLED false
#define TIME_BETWEEN_FAKE_COMMANDS 200
volatile unsigned long last_fake_command_time = 0;

/*
  High level configuration constants
*/

// If set to True, use the MOSFETs on the Cadence PCB to drive the solenoids or motors
// If set to False, it is assumed that the Adafruit Motor Sheid v2 (https://www.adafruit.com/product/1438) is being used to drive the actuators
#define ACTUATORS_CONTROLLED_WITH_MOSFET false

// Amount in milliseconds to hold solenoid on for if the actuator is a solenoid
#define SOLENOID_ENABLE_TIME  100

#define STATUS_LED_BLINK_OFF_TIME 100 // in ms. The amount of time for the status LED to be off when displaying a blink pattern to the user

// How long the pumps will be enabled in ms
#define ACTUATOR_1_MOTOR_ENABLE_TIME 155
#define ACTUATOR_2_MOTOR_ENABLE_TIME 153

// The PWM value for the motor when it is enabled, will be passed to `analogWrite`
#define ACTUATOR_1_MOTOR_PWM_VALUE 110
#define ACTUATOR_2_MOTOR_PWM_VALUE 80

#define ACTUATOR_1_SERIAL_CONTROL false
#define ACTUATOR_2_SERIAL_CONTROL true

#define ACTUATOR_1_MOTOR true
#define ACTUATOR_2_MOTOR true

/*
  Pin mappings
*/

// Pins of all inputs and outputs
#define PULSE1_PIN A0
#define PULSE2_PIN A1

#if ACTUATORS_CONTROLLED_WITH_MOSFET == true
  #define ACTUATOR1_PIN 11 // also soldered to 2
  #define ACTUATOR2_PIN 3
#else
  #include <Wire.h>
  #include <Adafruit_MotorShield.h>
  #include "utility/Adafruit_MS_PWMServoDriver.h"
  Adafruit_MotorShield AFMS = Adafruit_MotorShield();
  Adafruit_DCMotor *actuator_1 = AFMS.getMotor(1);
  Adafruit_DCMotor *actuator_2 = AFMS.getMotor(2);
#endif

#define STATUS_LED_PIN 5

/*
  Communication Protocol Def
 */

#define COMMAND_HEARTBEAT 0x01        // host PC sending a command for us to respond to to say we're still here
#define COMMAND_SERVICE_STARTED 0x02  // host PC saying the service is started and we should expect to get commands
#define COMMAND_SERVICE_CRASHED 0x03  // host PC informing us that they have crashed and we will not be getting any more commands
#define COMMAND_PULSE_LED 0x04        // host PC is telling us to trigger the actuator pin, and blink the LED
#define COMMAND_PULSE_NO_LED 0x05     // host PC is telling us to trigger the actuator pin without blinking the LED

// Every byte should be responsded to with the same byte.

/*
  Program Body
*/

#define NUM_PAIRS 2

volatile unsigned long last_beat_time[NUM_PAIRS] = {0, 0};  // used to find the time between beats
volatile unsigned long last_sample_time[NUM_PAIRS] = {0, 0}; // used to determine pulse timing

// To avoid false positives
#define MIN_TIME_BETWEEN_BEATS 400
 
// In the field, and using an osiclloscope, we expect the sensor to output the maximum voltage it can when a beat is seen. However, this is slightly lower than that to try and catch 100% of the beats
#define PULSE_START_READING_MIN_THRESHOLD 630
#define PULSE_FINISHED_READING_MAX_THRESHOLD 500

#define NUM_SPOT_SAMPLES 2

// these are used to detect when a finger is not there
#define VALUE_INTO_ANALYSIS_EVERY_N_SAMPLES 15

#define NUM_HISTORIC_ANALYSIS 150
#define ANALYSIS_MIN_POSITIVE_THRESHOLD 110
#define ANALYSIS_MAX_NEGATIVE_THRESHOLD 50
#define MIN_DISTANCE_FROM_NEGATIVE_ANALYSIS 3000  // in ms

volatile float analysis_history[NUM_PAIRS][NUM_HISTORIC_ANALYSIS] = { 0 };
wrapCounter analysis_history_index[NUM_PAIRS];
wrapCounter sample_entry_counter[NUM_PAIRS];
volatile unsigned long previous_negative_analysis_time[NUM_PAIRS] = { 0 };

// these are volatile because they are used inside of the ISR
volatile boolean pulse_resetting[NUM_PAIRS] = {false, false};     // true when inside a pulse, false otherwise
volatile boolean pulse_non_resetting[NUM_PAIRS] = {false, false}; // true when a new pulse starts, does not get set to false with the ISR

bool actuator_controlled_via_serial_port[NUM_PAIRS] = {ACTUATOR_1_SERIAL_CONTROL, ACTUATOR_2_SERIAL_CONTROL};
int pulse_sensor_pins[NUM_PAIRS] = {PULSE1_PIN, PULSE2_PIN};

#if ACTUATORS_CONTROLLED_WITH_MOSFET == true
  int actuator_pins[NUM_PAIRS] = {ACTUATOR1_PIN, ACTUATOR2_PIN};
#else
  Adafruit_DCMotor *actuator_controllers[NUM_PAIRS] = {actuator_1, actuator_2};
#endif

bool actuator_is_motor[NUM_PAIRS] = {ACTUATOR_1_MOTOR, ACTUATOR_2_MOTOR};
int motor_enable_times[NUM_PAIRS] = {ACTUATOR_1_MOTOR_ENABLE_TIME, ACTUATOR_2_MOTOR_ENABLE_TIME};
int motor_pwm_values[NUM_PAIRS] = {ACTUATOR_1_MOTOR_PWM_VALUE, ACTUATOR_2_MOTOR_PWM_VALUE};

bool pulse_sensor_enabled[NUM_PAIRS] = { false , false };
bool actuator_enabled[NUM_PAIRS] = {false, false};
volatile unsigned long actuation_start_time[NUM_PAIRS] = {0, 0};

int most_recent_drip_command_type;
bool serial_message_needs_responding_to = false;

void status_led_blink(int num_blinks, int on_time) {
  // Blink the LED in a given pattern. Indicates to user how things are going.
  for (int i = 0; i < num_blinks; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(on_time);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(STATUS_LED_BLINK_OFF_TIME);
  }
}

/*
 * Get the mean from an array of volatile long unsigned int
 */
float mean(volatile float * val, int array_length) {
  long unsigned int total = 0;
  for (int i = 0; i < array_length; i++) {
    total = total + val[i];
  }
  float avg = total/(float)array_length;
  return avg;
}

float variance(volatile float * val, int array_length) {
  float avg = mean(val, array_length);
  long unsigned int total = 0;
  for (int i = 0; i < array_length; i++) {
    total = total + (val[i] - avg) * (val[i] - avg);
  }

  return total/(float)array_length;  
}

/*
 * Get the standard deviation from an array of volatile long unsigned int
 */
float standard_deviation(volatile float * val, int array_length) {
  float v = variance(val, array_length);
  float std_dev = sqrt(v);
  return std_dev;
}

int lookup_actuator_enable_time(int actuator_index) {
  // TODO: could probably do this with a macro but not worth the complexity now
  if (actuator_is_motor[actuator_index]) {
    return motor_enable_times[actuator_index];
  } else {
    return SOLENOID_ENABLE_TIME;
  }
  
}

void change_actuator_state(int actuator_index, bool enabled) {

  if (actuator_is_motor[actuator_index]) {
    if (enabled) {
      #if ACTUATORS_CONTROLLED_WITH_MOSFET == true
        analogWrite(actuator_pins[actuator_index], motor_pwm_values[actuator_index]);
      #else
        actuator_controllers[actuator_index]->setSpeed(motor_pwm_values[actuator_index]);
        actuator_controllers[actuator_index]->run(FORWARD);
      #endif
    } else {
      #if ACTUATORS_CONTROLLED_WITH_MOSFET == true
        analogWrite(actuator_pins[actuator_index], 0);
      #else
        actuator_controllers[actuator_index]->run(RELEASE);
      #endif
    }
  } else {
      #if ACTUATORS_CONTROLLED_WITH_MOSFET == true
        digitalWrite(actuator_pins[actuator_index], enabled);
      #else
        if (enabled) {
          actuator_controllers[actuator_index]->setSpeed(255);
          actuator_controllers[actuator_index]->run(FORWARD);
        } else {
          actuator_controllers[actuator_index]->run(RELEASE);
        }
      #endif
  }
}

void update_is_person_attached_to_pulse_sensor(int sensor_index) {
  
  float current_standard_deviation = standard_deviation(analysis_history[sensor_index], NUM_HISTORIC_ANALYSIS);
  float current_mean = mean(analysis_history[sensor_index], NUM_HISTORIC_ANALYSIS);

  bool positive_analysis = (current_standard_deviation >= ANALYSIS_MIN_POSITIVE_THRESHOLD);
  bool negative_analysis = (current_standard_deviation <= ANALYSIS_MAX_NEGATIVE_THRESHOLD);

  unsigned long current_time = millis();

  if (negative_analysis) {
    previous_negative_analysis_time[sensor_index] = current_time;
  }

  if (positive_analysis) {
    if ( (current_time - previous_negative_analysis_time[sensor_index]) >= MIN_DISTANCE_FROM_NEGATIVE_ANALYSIS) {
        pulse_sensor_enabled[sensor_index] = true;
    } else {
      pulse_sensor_enabled[sensor_index] = false;
    }
  } else {
    pulse_sensor_enabled[sensor_index] = false;
  }
  
  #if DEBUG_MODE == true
    Serial.print(current_standard_deviation);
    Serial.print(",");
    Serial.print(ANALYSIS_MIN_POSITIVE_THRESHOLD);
    Serial.print(",");
    Serial.print(pulse_sensor_enabled[sensor_index]);
    Serial.print(",");
    Serial.print(analysis_history[sensor_index][analysis_history_index[sensor_index].value]);
    Serial.print(",");
    Serial.print(current_mean);
    Serial.print(",");
    Serial.println();
  #endif
}

void sound_test_loop() {
  
  const int motor_on_time = 80;
  const int time_between_drips = 100;

  while (true) {
    change_actuator_state(0, true);
    change_actuator_state(1, true);
    delay(motor_on_time);
    change_actuator_state(0, false);
    change_actuator_state(1, false);
    delay(time_between_drips);
  }
  
}

bool fake_command() {
  unsigned long current_time = millis();
  if (current_time - last_fake_command_time > TIME_BETWEEN_FAKE_COMMANDS) {
    last_fake_command_time = current_time;
    return true;
  }
  return false;
}

void setup() {
  
  // Initializes Timer1 to throw an interrupt every 1mS.
  TCCR1A = 0x00; // DISABLE OUTPUTS AND PWM ON DIGITAL PINS 9 & 10
  TCCR1B = 0x11; // GO INTO 'PHASE AND FREQUENCY CORRECT' MODE, NO PRESCALER
  TCCR1C = 0x00; // DON'T FORCE COMPARE
  TIMSK1 = 0x01; // ENABLE OVERFLOW INTERRUPT (TOIE1)
  ICR1 = 8000;   // TRIGGER TIMER INTERRUPT EVERY 1mS  
  sei();         // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED

  #if ACTUATORS_CONTROLLED_WITH_MOSFET == true
    pinMode(ACTUATOR1_PIN, OUTPUT);
    pinMode(ACTUATOR2_PIN, OUTPUT);
  #else
    AFMS.begin();
  #endif

  pinMode(STATUS_LED_PIN, OUTPUT);

  for (int pair_index = 0; pair_index < NUM_PAIRS; pair_index++) {
    analysis_history_index[pair_index] = wrapCounter(NUM_HISTORIC_ANALYSIS);
    sample_entry_counter[pair_index] = wrapCounter(VALUE_INTO_ANALYSIS_EVERY_N_SAMPLES);

    // fill this buffer with sensical values
    for (int sample_index = 0; sample_index < NUM_HISTORIC_ANALYSIS; sample_index++) {
      analysis_history[pair_index][sample_index] = analogRead(pulse_sensor_pins[pair_index]);
    }
  }

  Serial.begin(115200);

}

void loop() {
  
  #if SOUND_TEST_MODE_ENABLED == true
    sound_test_loop();
  #endif

  bool serial_actuator_enabled = false;

  // Process commands from a host PC
  if (Serial.available()) {
    byte command = Serial.read();
    switch (command) {
      case COMMAND_HEARTBEAT:
        Serial.write(COMMAND_HEARTBEAT);  // echo back
        break;
      case COMMAND_SERVICE_STARTED:
        status_led_blink(3, 300);  // three short blinks when we expect to start processing commands.
        Serial.write(COMMAND_SERVICE_STARTED);  // echo back
        break;
      case COMMAND_SERVICE_CRASHED:
        status_led_blink(2, 1000); // two long blinks if something bad happens on the pi end of things.
        Serial.write(COMMAND_SERVICE_CRASHED);  // echo back
        break;
      case COMMAND_PULSE_LED:
        digitalWrite(STATUS_LED_PIN, HIGH);
        serial_actuator_enabled = true;
        most_recent_drip_command_type = true;
        serial_message_needs_responding_to = true;
        most_recent_drip_command_type = COMMAND_PULSE_LED;
        break;
      case COMMAND_PULSE_NO_LED:
        serial_actuator_enabled = true;
        most_recent_drip_command_type = true;
        serial_message_needs_responding_to = true;
        most_recent_drip_command_type = COMMAND_PULSE_NO_LED;
        break;
    }
  }

  #if FAKE_COMMANDS_ENABLED == true
    serial_actuator_enabled |= fake_command();
    // there isn't going to be a host here but we still want to fake it
    serial_message_responded_to = true;
    drip_command_type = COMMAND_PULSE_NO_LED;
  #endif

  for (int pair_index = 0; pair_index < NUM_PAIRS; pair_index++) {

    bool start_actuator = false;

    // Either accept a command from the PC or read from the pulse sensor
    if (actuator_controlled_via_serial_port[pair_index] == true) {
      if (serial_actuator_enabled) {
        start_actuator = true;
      }
    } else {
      update_is_person_attached_to_pulse_sensor(pair_index);
      if ((pulse_non_resetting[pair_index] == true) && pulse_sensor_enabled[pair_index]) {
        pulse_non_resetting[pair_index] = false;
        start_actuator = true; 
      }
    }

    if (start_actuator) {
      if (actuator_enabled[pair_index] == false) {
        actuator_enabled[pair_index] = true;
        actuation_start_time[pair_index] = millis();
      }
    }

    if (actuator_enabled[pair_index]) {
      change_actuator_state(pair_index, HIGH);
      if (millis() - actuation_start_time[pair_index] > lookup_actuator_enable_time(pair_index)) {
        change_actuator_state(pair_index, LOW);
        actuator_enabled[pair_index] = false;
        if (actuator_controlled_via_serial_port[pair_index] == true) {
          digitalWrite(STATUS_LED_PIN, LOW);
          if (serial_message_needs_responding_to) {
            Serial.write(most_recent_drip_command_type);
            serial_message_needs_responding_to = true;
          }
        }
      }
    }
    
  }

}

// THIS IS THE TIMER 1 INTERRUPT SERVICE ROUTINE. 
// triggered every time Timer 1 overflows, every 1mS
ISR(TIMER1_OVF_vect) {
  
  for (int pair_index = 0; pair_index < NUM_PAIRS; pair_index++) {

    // Get a quick spot reading by taking multiple samples and passing along the average value
    float pulse_signal = 0;
    for (int i = 0; i < NUM_SPOT_SAMPLES; i++) {
      pulse_signal = pulse_signal + analogRead(pulse_sensor_pins[pair_index]);
    }
    pulse_signal = pulse_signal / NUM_SPOT_SAMPLES;
    
    last_sample_time[pair_index]++;  // keep track of the time with this variable (ISR triggered every 1mS

    int time_delta = last_sample_time[pair_index] - last_beat_time[pair_index];  // monitor the time since the last beat to avoid noise
    
    if ((pulse_signal > PULSE_START_READING_MIN_THRESHOLD) && (pulse_resetting[pair_index] == false) && (time_delta > MIN_TIME_BETWEEN_BEATS)) {            
      last_beat_time[pair_index] = last_sample_time[pair_index];  // keep track of time for next pulse
      pulse_non_resetting[pair_index] = true;  // set Quantified Self flag when beat is found and BPM gets updated, QS FLAG IS NOT CLEARED INSIDE THIS ISR
      pulse_resetting[pair_index] = true;  // set the pulse flag when we think there is a pulse
    }                       

    if (pulse_signal < PULSE_FINISHED_READING_MAX_THRESHOLD && pulse_resetting[pair_index] == true) {  // when the values are going down, it's the time between beats
      pulse_resetting[pair_index] = false;  // reset the pulse flag so we can do it again!
    }

    if (sample_entry_counter[pair_index].increment()) {
      analysis_history[pair_index][analysis_history_index[pair_index].value] = pulse_signal;
      analysis_history_index[pair_index].increment();
    }
     
  } 
}
