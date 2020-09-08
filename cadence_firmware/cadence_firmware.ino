/*

Cadence Driver - Version 4.0.0 - 9/7/2020

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


#define MIN_MEAN_ANALYSIS_VALUE 6000 // magic foo for seeing if someone is attached to sensor or not. See usage.
#define MIN_FILTER_MS 1000  // control how often the algo for detecting if a human is attached to sensor or not runs

/*
  Debug and test mode config
*/

#define DEBUG_MODE false

/*
  Pin mappings
*/

// Pins of all inputs and outputs
#define  PULSE1_PIN A0
#define  PULSE2_PIN A1
#define  ACTUATOR1_PIN 11 // also soldered to 2
#define  ACTUATOR2_PIN 3
#define  STATUS_LED_PIN 5

/*
  High level configuration constants
*/

// Amount in milliseconds to hold solenoid on for if the actuator is a solenoid
#define SOLENOID_ENABLE_TIME  100

// PWM value and motor on time if actuator is a motor
#define MOTOR_ENABLE_TIME 130
#define MOTOR_PWM_VALUE 180

#define STATUS_LED_BLINK_OFF_TIME 100 // in ms. The amount of time for the status LED to be off when displaying a blink pattern to the user

#define ACTUATOR_1_SERIAL_CONTROL false
#define ACTUATOR_2_SERIAL_CONTROL true

#define ACTUATOR_1_MOTOR true
#define ACTUATOR_2_MOTOR true

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

// these are used to detect when a finger is not there

#define NUM_HISTORIC_BEAT_TIMESTAMPS 10
volatile unsigned long beat_timestamp_history[NUM_PAIRS][NUM_HISTORIC_BEAT_TIMESTAMPS] = { 0 };
wrapCounter beat_timestamp_history_index[NUM_PAIRS];

volatile unsigned long previous_filter_time[NUM_PAIRS] = { 0 };

#define NUM_HISTORIC_ANALYSIS 5
volatile unsigned long analysis_history[NUM_PAIRS][NUM_HISTORIC_ANALYSIS] = { 0 };
wrapCounter analysis_history_index[NUM_PAIRS];

// these are volatile because they are used inside of the ISR
volatile boolean pulse_resetting[NUM_PAIRS] = {false, false};     // true when inside a pulse, false otherwise
volatile boolean pulse_non_resetting[NUM_PAIRS] = {false, false}; // true when a new pulse starts, does not get set to false with the ISR

bool actuator_controlled_via_serial_port[NUM_PAIRS] = {ACTUATOR_1_SERIAL_CONTROL, ACTUATOR_2_SERIAL_CONTROL};
int pulse_sensor_pins[NUM_PAIRS] = {PULSE1_PIN, PULSE2_PIN};
int actuator_pins[NUM_PAIRS] = {ACTUATOR1_PIN, ACTUATOR2_PIN}; 
bool actuator_is_motor[NUM_PAIRS] = {ACTUATOR_1_MOTOR, ACTUATOR_2_MOTOR};

bool pulse_sensor_enabled[NUM_PAIRS] = { false , false };
bool actuator_enabled[NUM_PAIRS] = {false, false};
volatile unsigned long actuation_start_time[NUM_PAIRS] = {0, 0};

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
float mean(volatile unsigned long * val, int array_length) {
  long unsigned int total = 0;
  for (int i = 0; i < array_length; i++) {
    total = total + val[i];
  }
  float avg = total/(float)array_length;
  return avg;
}

/*
 * Get the standard deviation from an array of volatile long unsigned int
 */
float standard_deviation(volatile unsigned long * val, int array_length) {
  float avg = mean(val, array_length);
  long unsigned int total = 0;
  for (int i = 0; i < array_length; i++) {
    total = total + (val[i] - avg) * (val[i] - avg);
  }

  float variance = total/(float)array_length;
  float std_dev = sqrt(variance);
  return std_dev;
}

int lookup_actuator_enable_time(int actuator_index) {
  // TODO: could probably do this with a macro but not worth the complexity now
  if (actuator_is_motor[actuator_index]) {
    return MOTOR_ENABLE_TIME;
  } else {
    return SOLENOID_ENABLE_TIME;
  }
  
}

void change_actuator_state(int actuator_index, bool enabled) {
  // TODO: could probably do this with a macro but not worth the complexity now
  if (actuator_is_motor[actuator_index]) {
    if (enabled) {
      analogWrite(actuator_pins[actuator_index], MOTOR_PWM_VALUE);  
    } else {
      analogWrite(actuator_pins[actuator_index], 0);  
    }
  } else {
    digitalWrite(actuator_pins[actuator_index], enabled);
  }
}

void update_is_person_attached_to_pulse_sensor(int sensor_index) {

  unsigned long current_time = millis();

  if (current_time - previous_filter_time[sensor_index] >= MIN_FILTER_MS) {
    previous_filter_time[sensor_index] = current_time;

    volatile unsigned long min_timestamp = beat_timestamp_history[sensor_index][0];
    volatile unsigned long max_timestamp = beat_timestamp_history[sensor_index][0];
  
    volatile unsigned long value;
    
    for (int i = 0; i < NUM_HISTORIC_BEAT_TIMESTAMPS; i++) {
    
      value = beat_timestamp_history[sensor_index][i];
    
      if (value < min_timestamp) {
        min_timestamp = value;
      }
      if (value > max_timestamp) {
        max_timestamp = value;
      }
    }

    volatile unsigned long diff = max_timestamp - min_timestamp;

    volatile unsigned long diff_mean = mean(analysis_history[sensor_index], NUM_HISTORIC_ANALYSIS);
    volatile unsigned long diff_std = standard_deviation(analysis_history[sensor_index], NUM_HISTORIC_ANALYSIS);
      
    analysis_history[sensor_index][analysis_history_index[sensor_index].value] = diff;
    analysis_history_index[sensor_index].increment();   

    #if DEBUG_MODE == true
      Serial.print("diff: ");
      Serial.print(diff);
      Serial.print(" means: ");
      Serial.print(diff_mean);
      Serial.print(" std: ");
      Serial.print(diff_std);
      Serial.println();
    #endif

    pulse_sensor_enabled[sensor_index] = (diff_mean > MIN_MEAN_ANALYSIS_VALUE);
    
  }

}

void setup() {
  
  // Initializes Timer1 to throw an interrupt every 1mS.
  TCCR1A = 0x00; // DISABLE OUTPUTS AND PWM ON DIGITAL PINS 9 & 10
  TCCR1B = 0x11; // GO INTO 'PHASE AND FREQUENCY CORRECT' MODE, NO PRESCALER
  TCCR1C = 0x00; // DON'T FORCE COMPARE
  TIMSK1 = 0x01; // ENABLE OVERFLOW INTERRUPT (TOIE1)
  ICR1 = 8000;   // TRIGGER TIMER INTERRUPT EVERY 1mS  
  sei();         // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED

  pinMode(ACTUATOR1_PIN, OUTPUT);
  pinMode(ACTUATOR2_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  for (int i = 0; i < NUM_PAIRS; i++) {
    beat_timestamp_history_index[i] = wrapCounter(NUM_HISTORIC_BEAT_TIMESTAMPS);
    analysis_history_index[i] = wrapCounter(NUM_HISTORIC_ANALYSIS);
  }

  Serial.begin(115200);

}

void loop() {

  bool serial_actuator_enabled = false;

  if (Serial.available()) {
    byte command = Serial.read();
    switch (command) {
      case COMMAND_HEARTBEAT:
        break;
      case COMMAND_SERVICE_STARTED:
        status_led_blink(3, 300);  // three short blinks when we expect to start processing commands.
        break;
      case COMMAND_SERVICE_CRASHED:
        status_led_blink(2, 1000); // two long blinks if something bad happens on the pi end of things.
        break;
      case COMMAND_PULSE_LED:
        serial_actuator_enabled = true;
        digitalWrite(STATUS_LED_PIN, HIGH);
        break;
      case COMMAND_PULSE_NO_LED:
        serial_actuator_enabled = true;
        break;
    }
    Serial.write(command);  // echo back
  }

  for (int pair_index = 0; pair_index < NUM_PAIRS; pair_index++) {

    bool start_actuator = false;

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
        }
      }
    }
 
  }

}

ISR(TIMER1_OVF_vect) {
  // THIS IS THE TIMER 1 INTERRUPT SERVICE ROUTINE. 
  // triggered every time Timer 1 overflows, every 1mS
  for (int pair_index = 0; pair_index < NUM_PAIRS; pair_index++) {

    int pulse_signal = analogRead(pulse_sensor_pins[pair_index]);   // read the pulse Sensor 
    last_sample_time[pair_index]++;                            // keep track of the time with this variable (ISR triggered every 1mS

    int time_delta = last_sample_time[pair_index] - last_beat_time[pair_index];  // monitor the time since the last beat to avoid noise
    
    if ((pulse_signal > 520) && (pulse_resetting[pair_index] == false) && (time_delta > 400)) {            

      beat_timestamp_history[pair_index][beat_timestamp_history_index[pair_index].value] = millis();
      beat_timestamp_history_index[pair_index].increment();
      
      last_beat_time[pair_index] = last_sample_time[pair_index];   // keep track of time for next pulse
      
      pulse_non_resetting[pair_index] = true;                 // set Quantified Self flag when beat is found and BPM gets updated, QS FLAG IS NOT CLEARED INSIDE THIS ISR
      pulse_resetting[pair_index] = true;                     // set the pulse flag when we think there is a pulse
    }                       

    if (pulse_signal < 500 && pulse_resetting[pair_index] == true) {  // when the values are going down, it's the time between beats
      pulse_resetting[pair_index] = false;                            // reset the pulse flag so we can do it again!
    }    
  } 
}
