/*
  User-configurable tweaks to the firmware.
  Modify these at your own risk!

  Usage Notes:
    Set PLATFORM to change the type of PCB we're attached to.
    Set ACTUATOR_*_SERIAL_CONTROL to True if you want the corresponding actuator
  to be controlled via the serial port. False if you want it to be controlled by
  it's pulse sensor. Set ACTUATOR_*_MOTOR to True if it is a pump and false if
  it is a solenoid.
*/

/*
  Debug and test mode config
*/

// If set to true, will print debug messages to the serial console.
#define DEBUG_MODE false

/*
  High level configuration constants
*/

#define PLATFORM_CADENCE_PCB 0
#define PLATFORM_ADAFRUIT_MOTOR_SHIELD 1
#define PLATFORM_SILENT_DRIPPER_PCB 2

// Set this contstant to tell the firmware what type of drivers/other hardware
// are attached to the Arduino.
#define PLATFORM PLATFORM_SILENT_DRIPPER_PCB

// Defines if the given actuator will be controlled via a PC over the serial
// port. If false, the actuator will respond to it's corresponding fingerprint
// sensor.
#define ACTUATOR_1_SERIAL_CONTROL false
#define ACTUATOR_2_SERIAL_CONTROL true

#define AC_MOSFET                                                              \
  0 // Use the MOSFETs on the Cadence PCB to drive the solenoids or motors.
#define AC_MOTOR_SHIELD                                                        \
  1 // Use the Adafruit Motor Shield v2 (https://www.adafruit.com/product/1438)
    // to drive the actuators.
#define AC_TMC2208                                                             \
  2 // Use the TMC2208s on the Silent Dripper PCB to drive the Stepper Motors,
    // the MaschinenReich XP88-ST01.

#define ACTUATOR_1_MOTOR true
#define ACTUATOR_2_MOTOR true

#if PLATFORM == PLATFORM_CADENCE_PCB
#define ACTUATORS_CONTROL_MODE AC_MOSFET
// The Cadence PCB is capable of driving either a motor OR a solenoid, and is
// the only platform that supports both types of actuators. If you want to drive
// a solenoid, set these variables to `false`.
#define ACTUATOR_1_MOTOR true
#define ACTUATOR_2_MOTOR true
#elif PLATFORM == PLATFORM_ADAFRUIT_MOTOR_SHIELD
#define ACTUATORS_CONTROL_MODE AC_MOTOR_SHIELD
#elif PLATFORM == PLATFORM_SILENT_DRIPPER_PCB
#define ACTUATORS_CONTROL_MODE AC_TMC2208
// Number of Neopixels on board the PCB
#define NUM_LEDS 3
#define LED_BRIGHTNESS 25
// This LED is used to communicate the status of the communication with a PC
// host, see `status_led_blink` for more.
#define SERIAL_STATUS_LED_INDEX 2
#else
#error "Invalid PLATFORM"
#endif

// Enable time is how long the motors will be on for a drip.
// PWM value is the analogWirte value written to the controller. This is how
// "fast" the motor will spin. These values must be tuned manually through
// experimentation
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
// TODO: This could lead to some steps being missed.
// If you change the step frequency of the TMC's, and it looks like the drips
// are getting cut off, start here.
#define ACTUATOR_1_MOTOR_ENABLE_TIME 400
#define ACTUATOR_2_MOTOR_ENABLE_TIME ACTUATOR_1_MOTOR_ENABLE_TIME
#define STEPPER_PUMP_MIN_STEPS_PER_DRIP 200
#define STEPPER_PUMP_MAX_STEPS_PER_DRIP 1500
#else
#error "Invalid ACTUATORS_CONTROL_MODE"
#endif

/*
  Pin mappings
*/

// Heartbeat sensors are on the same pins across all hardware iterations.
#define PULSE1_PIN A0
#define PULSE2_PIN A1

#if PLATFORM == PLATFORM_CADENCE_PCB
#define ACTUATOR1_PIN 11 // also soldered to 2 on 2020 version of the board.
#define ACTUATOR2_PIN 3
#define STATUS_LED_PIN 5
#elif PLATFORM == PLATFORM_ADAFRUIT_MOTOR_SHIELD
#elif PLATFORM == PLATFORM_SILENT_DRIPPER_PCB
#define ACTUATOR_1_DRIP_SIZE_POT_PIN A2
#define ACTUATOR_2_DRIP_SIZE_POT_PIN A3
#define CALIBRATION_MODE_SWITCH_PIN 12
#define LED_DATA_PIN A5
#define LED_CLOCK_PIN A4
#define TMC_1_SW_TX 2    // TMC2208 #1 SoftwareSerial transmit pin
#define TMC_1_SW_RX 3    // TMC2208 #1 SoftwareSerial receive pin
#define TMC_1_STEP_PIN 4 // Step #1
#define TMC_1_DIR_PIN 5  // Direction #1
#define TMC_1_EN_PIN 6   // Enable #1
#define TMC_2_SW_TX 7    // TMC2208 #2 SoftwareSerial transmit pin
#define TMC_2_SW_RX 8    // TMC2208 #2 SoftwareSerial receive pin
#define TMC_2_STEP_PIN 9 // Step #2
#define TMC_2_DIR_PIN 10 // Direction #2
#define TMC_2_EN_PIN 11  // Enable #2
#else
#error "Invalid Platform Configuration"
#endif
