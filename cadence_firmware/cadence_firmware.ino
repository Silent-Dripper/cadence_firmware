/*
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
#define TIME_BETWEEN_FAKE_COMMANDS 5000
#if FAKE_COMMANDS_ENABLED == true
  volatile unsigned long last_fake_command_time = 0;
#endif 

/*
  High level configuration constants
*/

#define AC_MOSFET 0  // Use the MOSFETs on the Cadence PCB to drive the solenoids or motors
#define AC_MOTOR_SHIELD 1  // Use the Adafruit Motor Sheid v2 (https://www.adafruit.com/product/1438) to drive the actuators
#define AC_TMC2208 2 // Use the TMC2208s on the Dripper PCB to drive the Stepper Motors, the MaschinenReich XP88-ST01

// Set this to one of the options above
#define ACTUATORS_CONTROL_MODE AC_TMC2208

// Amount in milliseconds to hold solenoid on for if the actuator is a solenoid
#define SOLENOID_ENABLE_TIME  100

#define STATUS_LED_BLINK_OFF_TIME 100 // in ms. The amount of time for the status LED to be off when displaying a blink pattern to the user

// Enable time is how long the motors will be on for a drip.
// PWM value is the analogWirte value written to the controller. This is how "fast" the motor will spin.
// These values must be tuned manually through experimentation
#if ACTUATORS_CONTROL_MODE == AC_MOSFET
  #define ACTUATOR_1_MOTOR_ENABLE_TIME 155
  #define ACTUATOR_2_MOTOR_ENABLE_TIME 153
  // these will be passed as analogWrite values
  #define ACTUATOR_1_MOTOR_DRIVE_STRENGTH 110
  #define ACTUATOR_2_MOTOR_DRIVE_STRENGTH 80
#elif ACTUATORS_CONTROL_MODE == AC_MOTOR_SHIELD
  #define ACTUATOR_1_MOTOR_ENABLE_TIME 155
  #define ACTUATOR_2_MOTOR_ENABLE_TIME 155
  // These will be written to the motor sheild's setSpeed method
  #define ACTUATOR_1_MOTOR_DRIVE_STRENGTH 150
  #define ACTUATOR_2_MOTOR_DRIVE_STRENGTH 150
#elif ACTUATORS_CONTROL_MODE == AC_TMC2208
  #define ACTUATOR_1_MOTOR_ENABLE_TIME 210
  #define ACTUATOR_2_MOTOR_ENABLE_TIME 210
  // These values are the number of steps the stepper motor will rotate when enabled
  // There is no check to make sure that all of the steps actually execute.
  // TODO - these constants are poorly named
  #define ACTUATOR_1_MOTOR_DRIVE_STRENGTH 890 // TODO - this should read from a pot to trim the value.
  #define ACTUATOR_2_MOTOR_DRIVE_STRENGTH ACTUATOR_1_MOTOR_DRIVE_STRENGTH  
#else
  #error "Invalid ACTUATORS_CONTROL_MODE"
#endif

#define ACTUATOR_1_SERIAL_CONTROL false
#define ACTUATOR_2_SERIAL_CONTROL true

#define ACTUATOR_1_MOTOR true
#define ACTUATOR_2_MOTOR true

// We can't drive non-motors with TMC2208, so stop compilation if user tries to configure this
#if ACTUATORS_CONTROL_MODE == AC_TMC2208
  #if ACTUATOR_1_MOTOR == false
    #error "Non-motors cannot be driven with TMC2208, Actuator 1 will not work."
  #endif
  #if ACTUATOR_2_MOTOR == false
    #error "Non-motors cannot be driven with TMC2208, Actuator 2 will not work."
  #endif
#endif

/*
  Pin mappings
*/

// Pins of all inputs and outputs
#define PULSE1_PIN A0
#define PULSE2_PIN A1

#define NUM_PAIRS 2

#if ACTUATORS_CONTROL_MODE == AC_MOSFET
  #define ACTUATOR1_PIN 11 // also soldered to 2
  #define ACTUATOR2_PIN 3
  int actuator_pins[NUM_PAIRS] = {ACTUATOR1_PIN, ACTUATOR2_PIN};
#elif ACTUATORS_CONTROL_MODE == AC_MOTOR_SHIELD
  #include <Wire.h>
  #include <Adafruit_MotorShield.h>
  #include "utility/Adafruit_MS_PWMServoDriver.h"
  Adafruit_MotorShield AFMS = Adafruit_MotorShield();
  Adafruit_DCMotor *actuator_1 = AFMS.getMotor(1);
  Adafruit_DCMotor *actuator_2 = AFMS.getMotor(2);
  Adafruit_DCMotor *actuator_controllers[NUM_PAIRS] = { actuator_1, actuator_2 };
#elif ACTUATORS_CONTROL_MODE == AC_TMC2208
  #include <TMCStepper.h>
  #define TMC_BAUD_RATE 115200
  #define TMC_PDN_DISABLE true   // Use UART 
  #define TMC_I_SCALE_ANALOG 0  // Adjust current from the register
  #define TMC_RMS_CURRENT 1000  // Set driver current 1A
  #define TMC_MICROSTEPS 16
  #define TMC_IRUN 9
  #define TMC_IHOLD 5
  #define TMC_GSTAT 0b111 
  #define TMC_1_SW_TX 2  // TMC2208 SoftwareSerial transmit pin
  #define TMC_1_SW_RX 3  // TMC2208 SoftwareSerial receive pin
  #define TMC_1_STEP_PIN 4  // Step
  #define TMC_1_DIR_PIN 5  // Direction
  #define TMC_1_EN_PIN 6  // Enable
  #define TMC_2_SW_TX 7  // TMC2208 SoftwareSerial transmit pin
  #define TMC_2_SW_RX 8  // TMC2208 SoftwareSerial receive pin
  #define TMC_2_STEP_PIN 9  // Step
  #define TMC_2_DIR_PIN 10  // Direction
  #define TMC_2_EN_PIN 11  // Enable
  #define R_SENSE 0.11f  // SilentStepStick series use 0.11
  TMC2208Stepper tmc_controllers[NUM_PAIRS] = { TMC2208Stepper(TMC_1_SW_RX, TMC_1_SW_TX, R_SENSE), TMC2208Stepper(TMC_2_SW_RX, TMC_2_SW_TX, R_SENSE) };
  int tmc_enable_pins[NUM_PAIRS] = { TMC_1_EN_PIN, TMC_2_EN_PIN }; 
  int tmc_step_pins[NUM_PAIRS] { TMC_1_STEP_PIN, TMC_2_STEP_PIN };
  // These two variables are used to control the motors
  volatile bool tmc_step_pin_value[NUM_PAIRS] = { false, false}; 
  volatile unsigned long tmc_remaining_steps[NUM_PAIRS] = { 0, 0 };
  volatile unsigned long steps_per_drip[NUM_PAIRS] = { ACTUATOR_1_MOTOR_DRIVE_STRENGTH, ACTUATOR_2_MOTOR_DRIVE_STRENGTH };
#else
  #error "Invalid ACTUATORS_CONTROL_MODE"
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

/*
  Program Body
*/

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

volatile int analysis_history[NUM_PAIRS][NUM_HISTORIC_ANALYSIS] = { 0 };
wrapCounter analysis_history_index[NUM_PAIRS];
wrapCounter sample_entry_counter[NUM_PAIRS];
volatile unsigned long previous_negative_analysis_time[NUM_PAIRS] = { 0 };

// these are volatile because they are used inside of the ISR
volatile boolean pulse_resetting[NUM_PAIRS] = {false, false};     // true when inside a pulse, false otherwise
volatile boolean pulse_non_resetting[NUM_PAIRS] = {false, false}; // true when a new pulse starts, does not get set to false with the ISR

bool actuator_controlled_via_serial_port[NUM_PAIRS] = {ACTUATOR_1_SERIAL_CONTROL, ACTUATOR_2_SERIAL_CONTROL};
int pulse_sensor_pins[NUM_PAIRS] = {PULSE1_PIN, PULSE2_PIN};

bool actuator_is_motor[NUM_PAIRS] = {ACTUATOR_1_MOTOR, ACTUATOR_2_MOTOR};
int motor_enable_times[NUM_PAIRS] = {ACTUATOR_1_MOTOR_ENABLE_TIME, ACTUATOR_2_MOTOR_ENABLE_TIME};
int motor_drive_strengths[NUM_PAIRS] = {ACTUATOR_1_MOTOR_DRIVE_STRENGTH, ACTUATOR_2_MOTOR_DRIVE_STRENGTH};

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
float mean(volatile int * val, int array_length) {
  long unsigned int total = 0;
  for (int i = 0; i < array_length; i++) {
    total = total + val[i];
  }
  float avg = total/(float)array_length;
  return avg;
}

float variance(volatile int * val, int array_length) {
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
float standard_deviation(volatile int * val, int array_length) {
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
    if (enabled) { // we want the motor to spin
      #if ACTUATORS_CONTROL_MODE == AC_MOSFET
        analogWrite(actuator_pins[actuator_index], motor_drive_strengths[actuator_index]);
      #elif ACTUATORS_CONTROL_MODE == AC_MOTOR_SHIELD
        actuator_controllers[actuator_index]->setSpeed(motor_drive_strengths[actuator_index]);
        actuator_controllers[actuator_index]->run(FORWARD);
      #elif ACTUATORS_CONTROL_MODE == AC_TMC2208
        digitalWrite(tmc_enable_pins[actuator_index], LOW);  // enable the driver
        tmc_remaining_steps[actuator_index] += steps_per_drip[actuator_index];
      #endif
    } else { // we want the motor to stop spinning
      #if ACTUATORS_CONTROL_MODE == AC_MOSFET
        analogWrite(actuator_pins[actuator_index], 0);
      #elif ACTUATORS_CONTROL_MODE == AC_MOTOR_SHIELD
        actuator_controllers[actuator_index]->run(RELEASE);
      #elif ACTUATORS_CONTROL_MODE == AC_TMC2208
        digitalWrite(tmc_enable_pins[actuator_index], HIGH);  // disable the driver
      #endif
    }
  } else { // meaning the actuator is a solenoid or something similar
      #if ACTUATORS_CONTROL_MODE == AC_MOSFET
        digitalWrite(actuator_pins[actuator_index], enabled);
      #elif ACTUATORS_CONTROL_MODE == AC_MOTOR_SHIELD
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

#if FAKE_COMMANDS_ENABLED == true
  bool fake_command() {
    unsigned long current_time = millis();
    if (current_time - last_fake_command_time > TIME_BETWEEN_FAKE_COMMANDS) {
      last_fake_command_time = current_time;
      return true;
    }
    return false;
  }
#endif

void setup() {

  cli();

  /*
   *  Configure TIMER1, responsible for sampling the pulse sensor.
   */

  // Turn on CTC (Clear timer on compare) mode for TIMER1.
  // If the timer's count ever gets to the value set in OCR1A. 
  // It will reset the timer's count after executing the ISR.
  TCCR1B = (1 << WGM12);
  // Set the TIMER2 prescaler to 1024.
  TCCR1B = (1 << CS12) | (1 << CS10);  

  // Set the compare match register for TIMER1 to trigger with a frequency of 1736hz.
  // Each time this value is reached, the pulse sensor is read.
  OCR1A = 8;// = (16*10^6) / (1736*1024) - 1 (must be <65536)

  // Enable the function inside of ISR(TIMER1_COMPA_vect).
  TIMSK1 |= (1 << OCIE1A);

  #if ACTUATORS_CONTROL_MODE == AC_TMC2208

    /*
     *  Configure TIMER2, responsible for driving the stepper motors.
     */
  
    // Turn on CTC (Clear timer on compare) mode for TIMER2.
    // If the timer's count ever gets to the value set in OCR2A. 
    // It will reset the timer's count after executing the ISR.
    TCCR2A = (1 << WGM21);
    // Set the TIMER2 prescaler to 128.
    TCCR2B = (1 << CS22) | (0 << CS21) | (1 << CS20);
    // Set the compare match register for TIMER2 to trigger with a frequency of ~9615.4hz.
    // A rising edge will be sent to the step pin of a TMC every other clock cycle, or in this case every 3mS.
    // If that TMC is enabled.
    OCR2A = 12;  // = (16*10^6) / (3000 * 128) - 1 (must be <255 because it's only 1 byte)
    // Enable the function inside of ISR(TIMER2_COMPA_vect).
    TIMSK2 |= (1 << OCIE2A); 
  #endif

  sei();

  // Initialize actuator controllers
  #if ACTUATORS_CONTROL_MODE == AC_MOSFET
    pinMode(ACTUATOR1_PIN, OUTPUT);
    pinMode(ACTUATOR2_PIN, OUTPUT);
  #elif ACTUATORS_CONTROL_MODE == AC_MOTOR_SHIELD
    AFMS.begin();
  #elif ACTUATORS_CONTROL_MODE == AC_TMC2208
    // Configure the first driver
    pinMode(TMC_1_EN_PIN, OUTPUT);
    pinMode(TMC_1_STEP_PIN, OUTPUT);
    pinMode(TMC_1_DIR_PIN, OUTPUT);
    digitalWrite(TMC_1_EN_PIN, LOW);  // enable the driver

     // Configure the second driver
    pinMode(TMC_2_EN_PIN, OUTPUT);
    pinMode(TMC_2_STEP_PIN, OUTPUT);
    pinMode(TMC_2_DIR_PIN, OUTPUT);
    digitalWrite(TMC_2_EN_PIN, LOW);  // enable the driver

    for (int i = 0; i < NUM_PAIRS; i++) {
      
      tmc_controllers[i].beginSerial(TMC_BAUD_RATE);
      tmc_controllers[i].begin();  // Initiate pins and registeries
      tmc_controllers[i].pdn_disable(TMC_PDN_DISABLE);
      tmc_controllers[i].I_scale_analog(TMC_I_SCALE_ANALOG);
      tmc_controllers[i].rms_current(TMC_RMS_CURRENT);

      /*
      * %0001 … %1000:
      * 128, 64, 32, 16, 8, 4, 2, FULLSTEP
      * Reduced microstep resolution.
      * The resolution gives the number of microstep entries per
      * sine quarter wave.
      * When choosing a lower microstep resolution, the driver
      * automatically uses microstep positions which result in a
      * symmetrical wave.
      * Number of microsteps per step pulse = 2^MRES
      * (Selection by pins unless disabled by GCONF.mstep_reg_select)
      */
      tmc_controllers[i].microsteps(TMC_MICROSTEPS);

      /*
      * IRUN (Reset default=31)
      * Motor run current (0=1/32 … 31=32/32)
      * Hint: Choose sense resistors in a way, that normal
      * IRUN is 16 to 31 for best microstep performance.
      */
      tmc_controllers[i].irun(TMC_IRUN);

       /*
      * IHOLD (Reset default: OTP)
      * Standstill current (0=1/32 … 31=32/32)
      * In combination with StealthChop mode, setting
      * IHOLD=0 allows to choose freewheeling or coil
      * short circuit (passive braking) for motor stand still.
      */
      tmc_controllers[i].ihold(TMC_IHOLD); 
    }
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

  #if DEBUG_MODE == true
    Serial.print("Current_Standard_Deviation");
    Serial.print(",");
    Serial.print("Analysis_Min_Positive_Threshold");
    Serial.print(",");
    Serial.print("Pulse_Sensor_Enabled");
    Serial.print(",");
    Serial.print("Latest_Raw_Value");
    Serial.print(",");
    Serial.print("Current_Mean");
    Serial.println();
  #endif

}

void loop() {
  
  #if SOUND_TEST_MODE_ENABLED == true
    while (true) {
      change_actuator_state(0, true);
      delay(250);
      change_actuator_state(0, false);
      delay(250);
    }
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
      if (start_actuator) {
        change_actuator_state(pair_index, HIGH);
      }
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

// THIS IS THE TIMER1 INTERRUPT SERVICE ROUTINE. 
ISR(TIMER1_COMPA_vect) {
  for (int pair_index = 0; pair_index < NUM_PAIRS; pair_index++) {
    int pulse_signal = analogRead(pulse_sensor_pins[pair_index]);
    last_sample_time[pair_index] = millis();
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

#if ACTUATORS_CONTROL_MODE == AC_TMC2208
  // THIS IS THE TIMER2 INTERRUPT SERVICE ROUTINE. 
  ISR(TIMER2_COMPA_vect) {    
    for (int pair_index = 0; pair_index < NUM_PAIRS; pair_index++) {
      if (tmc_remaining_steps[pair_index] > 0) {
        digitalWrite(tmc_step_pins[pair_index], tmc_step_pin_value[pair_index]);
        if (tmc_step_pin_value[pair_index] == true) {
          tmc_remaining_steps[pair_index] = tmc_remaining_steps[pair_index] - 1;
        }
        tmc_step_pin_value[pair_index] = !tmc_step_pin_value[pair_index];    
      } else if (tmc_remaining_steps[pair_index] == 0)  {
        digitalWrite(tmc_step_pins[pair_index], false);
        tmc_step_pin_value[pair_index] = false;
      }
    }
  }
#endif
