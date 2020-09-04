/*

Cadence Driver - Version 3 - 1/8/2019 TODO: update!

This version of Cadence Driver uses none of the given pulsesensor code because the performance was not acceptable.

Pins:
    Solenoid 1 - D2
    Solenoid 2 - D3
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
      
For support:
    dev@esologic.com

*/

#include "wrapCounter.h"

/*
  Debug and test mode config
*/

#define DEBUG_MODE true

/*
  High level configuration constants
*/

// Amount in milliseconds to hold solenoids on for
#define SOLENOID_ENABLE_TIME  100
#define SERIAL_SOLENOID_PIN_INDEX 1 // 0 or 1 - this is the index in the solenoid pins that will be toggled by the serial port
#define STATUS_LED_BLINK_OFF_TIME 100 // in ms. The amount of time for the status LED to be off when displaying a blink pattern to the user

/*
  Communication Protocol Def
 */

#define COMMAND_HEARTBEAT 0x01        // host PC sending a command for us to respond to to say we're still here
#define COMMAND_SERVICE_STARTED 0x02  // host PC saying the service is started and we should expect to get commands
#define COMMAND_SERVICE_CRASHED 0x03  // host PC informing us that they have crashed and we will not be getting any more commands
#define COMMAND_PULSE_LED 0x04        // host PC is telling us to trigger the solenoid pin, and blink the LED
#define COMMAND_PULSE_NO_LED 0x05     // host PC is telling us to trigger the solenoid pin without blinking the LED

// Every byte should be responsded to with the same byte.

/*
  Program Body
*/

#define NUM_PAIRS 2
#define NUM_HISTORIC_BEAT_TIMES 5

volatile unsigned long last_beat_time[NUM_PAIRS] = {0, 0};  // used to find the time between beats
volatile unsigned long sample_counter[NUM_PAIRS] = {0, 0}; // used to determine pulse timing

volatile unsigned long beat_time_history[NUM_PAIRS][NUM_HISTORIC_BEAT_TIMES] = { 0 };

wrapCounter beat_history_index[NUM_PAIRS];

// these are volatile because they are used inside of the ISR
volatile boolean pulse_resetting[NUM_PAIRS] = {false, false};     // true when pulse wave is high, false when it's low
volatile boolean pulse_non_resetting[NUM_PAIRS] = {false, false}; // becomes true when pulse rate is determined. every 20 pulses

bool solenoid_enabled[NUM_PAIRS] = {false, false};
unsigned long solenoid_start[NUM_PAIRS] = {0, 0};

// Pins of all inputs and outputs
#define  PULSE1_PIN A0
#define  PULSE2_PIN A1
#define  SOLENOID1_PIN 2
#define  SOLENOID2_PIN 3
#define  STATUS_LED_PIN 5

int pulse_pins[NUM_PAIRS] = {PULSE1_PIN, PULSE2_PIN};
int solenoid_pins[NUM_PAIRS] = {SOLENOID1_PIN, SOLENOID2_PIN}; 

void setup() {
  
  // Initializes Timer1 to throw an interrupt every 1mS.
  TCCR1A = 0x00; // DISABLE OUTPUTS AND PWM ON DIGITAL PINS 9 & 10
  TCCR1B = 0x11; // GO INTO 'PHASE AND FREQUENCY CORRECT' MODE, NO PRESCALER
  TCCR1C = 0x00; // DON'T FORCE COMPARE
  TIMSK1 = 0x01; // ENABLE OVERFLOW INTERRUPT (TOIE1)
  ICR1 = 8000;   // TRIGGER TIMER INTERRUPT EVERY 1mS  
  sei();         // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED

  pinMode(SOLENOID1_PIN, OUTPUT);
  pinMode(SOLENOID2_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  for (int i = 0; i < NUM_PAIRS; i++) {
    beat_history_index[i] = wrapCounter(NUM_HISTORIC_BEAT_TIMES);
  }

  Serial.begin(115200);

}

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
 * Get the mean from an array of ints
 */
float get_mean(long unsigned int * val, int array_count) {
  long total = 0;
  for (int i = 0; i < array_count; i++) {
    total = total + val[i];
  }
  float avg = total/(float)array_count;
  return avg;
}

/*
 * Get the standard deviation from an array of ints
 */
float standard_deviation(long unsigned int * val, int array_count) {
  float avg = get_mean(val, array_count);
  long total = 0;
  for (int i = 0; i < array_count; i++) {
    total = total + (val[i] - avg) * (val[i] - avg);
  }

  float variance = total/(float)array_count;
  float std_dev = sqrt(variance);
  return std_dev;
}

void loop() {

  bool serial_solenoid_enabled = false;

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
        serial_solenoid_enabled = true;
        digitalWrite(STATUS_LED_PIN, HIGH);
        break;
      case COMMAND_PULSE_NO_LED:
        serial_solenoid_enabled = true;
        break;
    }
    Serial.write(command);  // echo back
  }

  for (int pair_index = 0; pair_index < NUM_PAIRS; pair_index++) {
    
    bool start_solenoid = false;

    if ((serial_solenoid_enabled) && (pair_index == SERIAL_SOLENOID_PIN_INDEX)) {
      start_solenoid = true;
    }

    if (pair_index != SERIAL_SOLENOID_PIN_INDEX) {
      if (pulse_non_resetting[pair_index] == true) {
        pulse_non_resetting[pair_index] = false;
        start_solenoid = true; 
      }
    }

    if (start_solenoid) {
      if (solenoid_enabled[pair_index] == false) {
        solenoid_enabled[pair_index] = true;
        solenoid_start[pair_index] = millis();
      }
    }

    if (solenoid_enabled[pair_index]) {
      digitalWrite(solenoid_pins[pair_index], HIGH);
      if (millis() - solenoid_start[pair_index] > SOLENOID_ENABLE_TIME) {
        digitalWrite(solenoid_pins[pair_index], LOW);
        solenoid_enabled[pair_index] = false;
        if (pair_index == SERIAL_SOLENOID_PIN_INDEX) {
          digitalWrite(STATUS_LED_PIN, LOW);
        }
      }
    }
  
  }

  #if DEBUG_MODE == true

  for (int pair_index = 0; pair_index < NUM_PAIRS; pair_index++) {
    Serial.print("Sensor ");
    Serial.print(pair_index);
    Serial.print(" values: ");
    for (int history_index = 0; history_index < NUM_HISTORIC_BEAT_TIMES; history_index++) {
      Serial.print(beat_time_history[pair_index][history_index]);
      Serial.print(", ");
    }
    float std = standard_deviation(beat_time_history[pair_index], NUM_HISTORIC_BEAT_TIMES);
    Serial.print("STD: ");
    Serial.print(std);
    Serial.print(" "); 
  }
  Serial.println();

  #endif
  
  delay(10);

}

ISR(TIMER1_OVF_vect) {
  // THIS IS THE TIMER 1 INTERRUPT SERVICE ROUTINE. 
  // triggered every time Timer 1 overflows, every 1mS
  for (int pair_index = 0; pair_index < NUM_PAIRS; pair_index++) {

    int pulse_signal = analogRead(pulse_pins[pair_index]);   // read the pulse Sensor 
    sample_counter[pair_index]++;                            // keep track of the time with this variable (ISR triggered every 1mS

    int time_delta = sample_counter[pair_index] - last_beat_time[pair_index];  // monitor the time since the last beat to avoid noise
    
    if ( (pulse_signal > 520) && (pulse_resetting[pair_index] == false) && (time_delta > 500) ) {
      beat_time_history[pair_index][beat_history_index[pair_index].value] = time_delta;
      beat_history_index[pair_index].increment();
      
      last_beat_time[pair_index] = sample_counter[pair_index];   // keep track of time for next pulse
      pulse_non_resetting[pair_index] = true;                 // set Quantified Self flag when beat is found and BPM gets updated, QS FLAG IS NOT CLEARED INSIDE THIS ISR
      pulse_resetting[pair_index] = true;                     // set the pulse flag when we think there is a pulse
    }                       

    if (pulse_signal < 500 && pulse_resetting[pair_index] == true) {  // when the values are going down, it's the time between beats
      pulse_resetting[pair_index] = false;                            // reset the pulse flag so we can do it again!
    }
  } 
}
