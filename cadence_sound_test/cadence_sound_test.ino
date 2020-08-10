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

volatile unsigned long last_beat_time[NUM_PAIRS] = {0, 0};  // used to find the time between beats
volatile unsigned long sample_counter[NUM_PAIRS] = {0, 0}; // used to determine pulse timing

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
  
  pinMode(SOLENOID1_PIN, OUTPUT);
  pinMode(SOLENOID2_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  Serial.begin(115200);

}

void loop() {

  const int num_pulses = 5;

  const int pulse_times[NUM_PAIRS] = {100, 100};
  const int time_between_pulses = 566;

  
  for (int pin = 0; pin < 2; pin++) {
    for (int pulse = 0; pulse < num_pulses; pulse++) {
      int p = solenoid_pins[pin];
      Serial.print(p);
      Serial.print(" ");
      Serial.println("high");
      digitalWrite(p, HIGH);
      delay(pulse_times[pin]);
      digitalWrite(p, LOW);
      Serial.print(p);
      Serial.print(" ");
      Serial.println("low");
      delay(time_between_pulses);
    }
  }
}
