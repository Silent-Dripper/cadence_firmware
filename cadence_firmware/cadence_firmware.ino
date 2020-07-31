/*

Cadence Driver - Version 3 - 1/8/2019

This version of Cadence Driver uses none of the given pulsesensor code because the performance was not acceptable.
This version is based on the interrupt approach by Joel Murphy and Yury Gitman.

Pins:
    Solenoid 1 - D2
    Solenoid 2 - D3
    Heartbeat Sensor 1 - A0
    Heartbeat Sensor 2 - A1

Usage Notes:
    Edit "High level configuration constants" to tune most of the application.
    pulse Sensor sample aquisition and processing happens in the background via Timer 1 interrupt. 1mS sample rate.
    PWM on pins 9 and 10 will not work when using this code!
    
    The following variables are automatically updated:
      QS:     boolean that is made true whenever pulse is found and BPM is updated. User must reset.

For support:
    dev@esologic.com

*/

/*
  High level configuration constants
*/

// Amount in milliseconds to hold solenoids on for
#define SOLENOID_ENABLE_TIME  100

/*
  Program Body
*/

#define NUM_PAIRS 2

volatile unsigned long lastBeatTime[NUM_PAIRS] = {0, 0};  // used to find the time between beats
volatile unsigned long sampleCounter[NUM_PAIRS] = {0, 0}; // used to determine pulse timing

// these are volatile because they are used inside of the ISR
volatile boolean pulse_resetting[NUM_PAIRS] = {false, false};     // true when pulse wave is high, false when it's low
volatile boolean pulse_non_resetting[NUM_PAIRS] = {false, false}; // becomes true when pulse rate is determined. every 20 pulses

bool solenoid_enabled[NUM_PAIRS] = {false, false};
unsigned long solenoid_start[NUM_PAIRS] = {0, 0};

// Pins of all inputs and outputs
#define  PULSE1_PIN  A0
#define  PULSE2_PIN  A1
#define  SOLENOID1_PIN  2
#define  SOLENOID2_PIN  3

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

  // Set up solenoid driver pins
  pinMode(SOLENOID1_PIN, OUTPUT);
  pinMode(SOLENOID2_PIN, OUTPUT);

}

void loop() {

  for (int pair_index = 0; pair_index < NUM_PAIRS; pair_index++) {
    
    bool start_solenoid = false;

    if (pulse_non_resetting[pair_index] == true) {
      pulse_non_resetting[pair_index] = false;
      start_solenoid = true; 
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
      }
    }

  }
   
   delay(20);

}

// THIS IS THE TIMER 1 INTERRUPT SERVICE ROUTINE. 
ISR(TIMER1_OVF_vect) { // triggered every time Timer 1 overflows, every 1mS
  for (int pair_index = 0; pair_index < NUM_PAIRS; pair_index++) {

    int pulse_signal = analogRead(pulse_pins[pair_index]);  // read the pulse Sensor 
    sampleCounter[pair_index]++;                            // keep track of the time with this variable (ISR triggered every 1mS

    int time_delta = sampleCounter[pair_index]-lastBeatTime[pair_index];  // monitor the time since the last beat to avoid noise
    
    if ( (pulse_signal > 520) && (pulse_resetting[pair_index] == false) && (time_delta > 500) ) {  
      lastBeatTime[pair_index] = sampleCounter[pair_index];   // keep track of time for next pulse
      pulse_non_resetting[pair_index] = true;                 // set Quantified Self flag when beat is found and BPM gets updated, QS FLAG IS NOT CLEARED INSIDE THIS ISR
      pulse_resetting[pair_index] = true;                     // set the pulse flag when we think there is a pulse
    }                       

    if (pulse_signal < 500 && pulse_resetting[pair_index] == true) {  // when the values are going down, it's the time between beats
      pulse_resetting[pair_index] = false;                            // reset the pulse flag so we can do it again!
    }

  } 
}
