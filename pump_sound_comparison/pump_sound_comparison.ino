/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper
 * motor in alternating directions.
 */

#include <TMCStepper.h>

#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

// Define pins


#define SILENT_STEP_PIN 2 // Step on rising edge
#define SILENT_DIR_PIN 3 // Direction
#define SILENT_EN_PIN 4 // LOW: Driver enabled. HIGH: Driver disabled

 // SoftwareSerial pins
#define SW_RX_1 5
#define SW_TX_1 6

 // SoftwareSerial pins
#define SW_RX_2 7
#define SW_TX_2 8

#define MOSFET_PIN  9

// Match to your driver
// SilentStepStick series use 0.11
// UltiMachine Einsy and Archim2 boards use 0.2
// Panucatt BSD2660 uses 0.1
// Watterott TMC5160 uses 0.075
#define R_SENSE 0.11f

TMC2208Stepper driver_1(SW_RX_1, SW_TX_1, R_SENSE);  // Software serial
TMC2208Stepper driver_2(SW_RX_2, SW_TX_2, R_SENSE);  // Software serial


void setup() {
  pinMode(SILENT_EN_PIN, OUTPUT);
  pinMode(SILENT_STEP_PIN, OUTPUT);
  pinMode(SILENT_DIR_PIN, OUTPUT);

  pinMode(MOSFET_PIN, OUTPUT);
  
  digitalWrite(SILENT_EN_PIN, LOW); // Enable driver in hardware

  driver_1.beginSerial(115200); // SW UART drivers
  driver_1.begin(); //  SPI: Init CS pins and possible SW SPI pins
  
  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver_1.toff(5); // Enables driver in software
  driver_1.rms_current(600); // Set motor RMS current
  driver_1.microsteps(16); // Set microsteps to 1/16th
  driver_1.pwm_autoscale(true); // Needed for stealthChop

  driver_2.beginSerial(115200); // SW UART drivers
  driver_2.begin(); //  SPI: Init CS pins and possible SW SPI pins
  
  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver_2.toff(5); // Enables driver in software
  driver_2.rms_current(600); // Set motor RMS current
  driver_2.microsteps(16); // Set microsteps to 1/16th
  driver_2.pwm_autoscale(true); // Needed for stealthChop
  
}

bool shaft = false;

void loop() {
  digitalWrite(SILENT_STEP_PIN, HIGH);
  delayMicroseconds(160);
  digitalWrite(SILENT_STEP_PIN, LOW);
  delayMicroseconds(160);
}
