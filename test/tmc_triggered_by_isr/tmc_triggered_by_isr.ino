#include <TMCStepper.h>

#define TMC_1_SW_TX 2  // TMC2208 SoftwareSerial transmit pin
#define TMC_1_SW_RX 3  // TMC2208 SoftwareSerial receive pin
#define TMC_1_STEP_PIN 4  // Step
#define TMC_1_DIR_PIN 5  // Direction
#define TMC_1_EN_PIN 6  // Enable

#define R_SENSE 0.11f // Match to your driver
                     // SilentStepStick series use 0.11
                     // UltiMachine Einsy and Archim2 boards use 0.2
                     // Panucatt BSD2660 uses 0.1
                     // Watterott TMC5160 uses 0.075

TMC2208Stepper tmc_1 = TMC2208Stepper(TMC_1_SW_RX, TMC_1_SW_TX, R_SENSE); // Software serial

volatile unsigned long steps_to_step = 0;
bool step_pin_value = false;  //used to keep the state of the LED

#define TMC_BAUD_RATE 115200
#define TMC_PDN_DISABLE true
#define TMC_I_SCALE_ANALOG 0
#define TMC_RMS_CURRENT 1000
#define TMC_MICROSTEPS 0
#define TMC_IRUN 8
#define TMC_IHOLD 5
#define TMC_GSTAT 0b111
  

ISR(TIMER2_COMPA_vect){    //This is the interrupt request
  if (steps_to_step > 0) {
    digitalWrite(TMC_1_STEP_PIN, step_pin_value);
    if (step_pin_value == true) {
      steps_to_step = steps_to_step - 1;
    }
    step_pin_value = !step_pin_value;    
  }
  else if (steps_to_step == 0)  {
    digitalWrite(TMC_1_STEP_PIN, false);
    step_pin_value = false;
  }
}

void setup() {

  pinMode(TMC_1_EN_PIN, OUTPUT);
  pinMode(TMC_1_STEP_PIN, OUTPUT);
  pinMode(TMC_1_DIR_PIN, OUTPUT);
  digitalWrite(TMC_1_EN_PIN, LOW);

  tmc_1.beginSerial(TMC_BAUD_RATE);
  tmc_1.begin();  // Initiate pins and registeries
  tmc_1.pdn_disable(TMC_PDN_DISABLE); // Use UART
  tmc_1.I_scale_analog(TMC_I_SCALE_ANALOG); // Adjust current from the register
  tmc_1.rms_current(TMC_RMS_CURRENT); // Set driver current 1A
  tmc_1.microsteps(TMC_MICROSTEPS);
  //tmc_1.irun(TMC_IRUN);
  //tmc_1.ihold(TMC_IHOLD);

  // Turn on CTC (Clear timer on compare) mode.
  // If the timer's value ever gets to the value set in OCR2A. 
  // It will reset the timer's count after executing the ISR.
  TCCR2A = (1 << WGM21);

  // Set CS22 1024 prescaler
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
  
  // Set the compare match register for timer2 to trigger with a frequency of 1000hz.
  OCR2A = 11;// = (16*10^6) / (1000 * 1024) - 1 (must be <255 because it's only 1 byte)
 
  // Enable the function inside of ISR(TIMER2_COMPA_vect).
  TIMSK2 |= (1 << OCIE2A);

  Serial.begin(9600);
  Serial.println("Starting");
  
}

#define NUM_STEPS 25

void loop() {
  /*
  for (int i = 0; i < NUM_STEPS; i++) {
    digitalWrite(TMC_1_STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(TMC_1_STEP_PIN, LOW);
    delayMicroseconds(1000);
  }
  */
  
  steps_to_step = NUM_STEPS;
  
  delay(1000);
  
}
