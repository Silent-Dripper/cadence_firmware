/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper
 * motor in alternating directions.
 */

#include <TMCStepper.h>

#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

// Define pins


#define SILENT_STEP_PIN  2 // Step on rising edge
#define SILENT_DIR_PIN   3 // Direction
#define SILENT_EN_PIN    4 // LOW: Driver enabled. HIGH: Driver disabled
#define SW_RX    5  // SoftwareSerial pins
#define SW_TX    6  //

#define EASY_STEP_PIN  7 // Step on rising edge
#define EASY_DIR_PIN   8 // Direction
#define EASY_EN_PIN    9 // LOW: Driver enabled. HIGH: Driver disabled

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

TMC2208Stepper driver(SW_RX, SW_TX, R_SENSE);                     // Software serial


void setup() {
  pinMode(SILENT_EN_PIN, OUTPUT);
  pinMode(SILENT_STEP_PIN, OUTPUT);
  pinMode(SILENT_DIR_PIN, OUTPUT);

  pinMode(EASY_EN_PIN, OUTPUT);
  pinMode(EASY_STEP_PIN, OUTPUT);
  pinMode(EASY_DIR_PIN, OUTPUT);
  
  digitalWrite(SILENT_EN_PIN, LOW);      // Enable driver in hardware
  digitalWrite(EASY_EN_PIN, LOW);      // Enable driver in hardware

  driver.beginSerial(115200);     // SW UART drivers

  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(600);        // Set motor RMS current
  driver.microsteps(16);          // Set microsteps to 1/16th

  driver.pwm_autoscale(true);     // Needed for stealthChop
}

bool shaft = false;

void loop() {

  int num_steps = 5000;

  for(int i = 0; i < num_steps; i++) {
    digitalWrite(EASY_STEP_PIN, HIGH);
    delayMicroseconds(160);
    digitalWrite(EASY_STEP_PIN, LOW);
    delayMicroseconds(160);
  }

  delay(1000);

  for(int i = 0; i < num_steps; i++) {
    digitalWrite(SILENT_STEP_PIN, HIGH);
    delayMicroseconds(160);
    digitalWrite(SILENT_STEP_PIN, LOW);
    delayMicroseconds(160);
  }

  delay(1000);
  
  
}
