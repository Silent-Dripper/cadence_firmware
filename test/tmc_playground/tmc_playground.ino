#include <TMCStepper.h>

#define TMC_BAUD_RATE 115200
#define TMC_PDN_DISABLE true
#define TMC_I_SCALE_ANALOG 0
#define TMC_RMS_CURRENT 1000
#define TMC_MICROSTEPS 0
#define TMC_IRUN 15
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

#define R_SENSE 0.11f // Match to your driver
                   // SilentStepStick series use 0.11
                   // UltiMachine Einsy and Archim2 boards use 0.2
                   // Panucatt BSD2660 uses 0.1
                   // Watterott TMC5160 uses 0.075

#define NUM_STEPS 100

TMC2208Stepper tmc_1 = TMC2208Stepper(TMC_1_SW_RX, TMC_1_SW_TX, R_SENSE); // Software serial
TMC2208Stepper tmc_2 = TMC2208Stepper(TMC_2_SW_RX, TMC_2_SW_TX, R_SENSE); // Software serial

volatile unsigned long tmc_remaining_steps = 0;
volatile bool tmc_step_pin_value = false;

// THIS IS THE TIMER2 INTERRUPT SERVICE ROUTINE. 
ISR(TIMER2_COMPA_vect) {
  if (tmc_remaining_steps > 0) {
    digitalWrite(TMC_1_STEP_PIN, tmc_step_pin_value);
    if (tmc_step_pin_value == true) {
      tmc_remaining_steps = tmc_remaining_steps - 1;
    }
    tmc_step_pin_value = !tmc_step_pin_value;    
  } else if (tmc_remaining_steps == 0)  {
    digitalWrite(TMC_1_STEP_PIN, false);
    tmc_step_pin_value = false;
  }
}

void setup() {

  // Turn on CTC (Clear timer on compare) mode.
  // If the timer's value ever gets to the value set in OCR2A. 
  // It will reset the timer's count after executing the ISR.
  TCCR2A = (1 << WGM21);
  // The timer2 prescaler to 128.
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
  // Set the compare match register for timer2 to trigger with a frequency of 1000hz.
  OCR2A = 25;// = (16*10^6) / (1000 * 1024) - 1 (must be <255 because it's only 1 byte)
  // Enable the function inside of ISR(TIMER2_COMPA_vect).
  TIMSK2 |= (1 << OCIE2A); 

  pinMode(13, OUTPUT);

  pinMode(TMC_1_EN_PIN, OUTPUT);
  pinMode(TMC_1_STEP_PIN, OUTPUT);
  pinMode(TMC_1_DIR_PIN, OUTPUT);
  digitalWrite(TMC_1_EN_PIN, LOW);

  pinMode(TMC_2_EN_PIN, OUTPUT);
  pinMode(TMC_2_STEP_PIN, OUTPUT);
  pinMode(TMC_2_DIR_PIN, OUTPUT);
  digitalWrite(TMC_2_EN_PIN, LOW);

  Serial.begin(9600);

  tmc_1.beginSerial(TMC_BAUD_RATE);
  tmc_2.beginSerial(TMC_BAUD_RATE);

  tmc_1.begin();  // Initiate pins and registeries
  tmc_2.begin();  // Initiate pins and registeries

  tmc_1.pdn_disable(TMC_PDN_DISABLE); // Use UART
  tmc_1.I_scale_analog(TMC_I_SCALE_ANALOG); // Adjust current from the register
  tmc_1.rms_current(TMC_RMS_CURRENT); // Set driver current 1A

  tmc_2.pdn_disable(TMC_PDN_DISABLE); // Use UART
  tmc_2.I_scale_analog(TMC_I_SCALE_ANALOG); // Adjust current from the register
  tmc_2.rms_current(TMC_RMS_CURRENT); // Set driver current 1A

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
  tmc_1.microsteps(TMC_MICROSTEPS);
  tmc_2.microsteps(TMC_MICROSTEPS);

  /*
  * IRUN (Reset default=31)
  * Motor run current (0=1/32 … 31=32/32)
  * Hint: Choose sense resistors in a way, that normal
  * IRUN is 16 to 31 for best microstep performance.
  */

  // 26 worked for no water
  tmc_1.irun(TMC_IRUN);
  tmc_2.irun(TMC_IRUN);

  /*
  * IHOLD (Reset default: OTP)
  * Standstill current (0=1/32 … 31=32/32)
  * In combination with StealthChop mode, setting
  * IHOLD=0 allows to choose freewheeling or coil
  * short circuit (passive braking) for motor stand still.
  */
  // 9 worked for no water
  tmc_1.ihold(TMC_IHOLD);
  tmc_2.ihold(TMC_IHOLD);

  Serial.println("Starting...");  
}

void loop() {
  /*
  digitalWrite(TMC_1_EN_PIN, LOW);
  digitalWrite(TMC_2_EN_PIN, LOW);
  digitalWrite(13, HIGH);
  for (int i = 0; i < NUM_STEPS; i++) {
    digitalWrite(TMC_1_STEP_PIN, HIGH);
    digitalWrite(TMC_2_STEP_PIN, HIGH);
    delayMicroseconds(1500);
    digitalWrite(TMC_1_STEP_PIN, LOW);
    digitalWrite(TMC_2_STEP_PIN, LOW);
    delayMicroseconds(1500);
  }
  digitalWrite(13, LOW);
  */

  tmc_remaining_steps = 45;
  
  delay(500);
  
}
