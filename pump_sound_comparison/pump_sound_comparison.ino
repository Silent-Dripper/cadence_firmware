#include <TMCStepper.h>

#define NUM_STEPS 10000

#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

// Define pins


#define SILENT_STEP_PIN_1 2 // Step on rising edge
#define SILENT_STEP_PIN_2 4 // Step on rising edge

#define SILENT_DIR_PIN 7 // Direction
#define SILENT_EN_PIN 8 // LOW: Driver enabled. HIGH: Driver disabled

 // SoftwareSerial pins
#define SW_RX_1 9
#define SW_TX_1 10

 // SoftwareSerial pins
#define SW_RX_2 11
#define SW_TX_2 12

#define MOSFET_1_PIN 3
#define MOSFET_2_PIN 5
#define MOSFET_3_PIN 6

#define MOSFET_1_PWM 180
#define MOSFET_2_PWM 160
#define MOSFET_3_PWM 255

// Match to your driver
// SilentStepStick series use 0.11
// UltiMachine Einsy and Archim2 boards use 0.2
// Panucatt BSD2660 uses 0.1
// Watterott TMC5160 uses 0.075
#define R_SENSE 0.11f

TMC2208Stepper driver_1(SW_RX_1, SW_TX_1, R_SENSE);  // Software serial
TMC2208Stepper driver_2(SW_RX_2, SW_TX_2, R_SENSE);  // Software serial


void tmc_1_test() {
  for (int i = 0; i < NUM_STEPS; i++) {
    digitalWrite(SILENT_STEP_PIN_1, HIGH);
    delayMicroseconds(160);
    digitalWrite(SILENT_STEP_PIN_1, LOW);
    delayMicroseconds(160);
  }
}

void tmc_2_test() {
  for (int i = 0; i < NUM_STEPS; i++) {
    digitalWrite(SILENT_STEP_PIN_2, HIGH);
    delayMicroseconds(160);
    digitalWrite(SILENT_STEP_PIN_2, LOW);
    delayMicroseconds(160);
  }
}

void mosfet_1_test() {
  analogWrite(MOSFET_1_PIN, MOSFET_1_PWM);
  delay(1000);
  analogWrite(MOSFET_1_PIN, 0);
  delay(100);
}

void mosfet_2_test() {
  analogWrite(MOSFET_2_PIN, MOSFET_2_PWM);
  delay(1000);
  analogWrite(MOSFET_2_PIN, 0);
  delay(100);
}

void mosfet_3_test() {
  analogWrite(MOSFET_3_PIN, MOSFET_3_PWM);
  delay(1000);
  analogWrite(MOSFET_3_PIN, 0);
  delay(100);
}

void setup() {
  pinMode(SILENT_EN_PIN, OUTPUT);
  pinMode(SILENT_STEP_PIN_1, OUTPUT);
  pinMode(SILENT_STEP_PIN_2, OUTPUT);
  pinMode(SILENT_DIR_PIN, OUTPUT);

  pinMode(MOSFET_1_PIN, OUTPUT);
  pinMode(MOSFET_2_PIN, OUTPUT);
  pinMode(MOSFET_3_PIN, OUTPUT);
  
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

  mosfet_2_test();

  
}
