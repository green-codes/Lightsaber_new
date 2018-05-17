/** 
 * Sketch for my lightsaber control program. Will integrate
 * into the complete TWI/Sound lightsaber program when done. 
 * 
 * Note: Let's try not stack too high here; 32u4 has 2k RAM. 
 * 
 */

// base module for saber control
#include "saber_ctl.h" 

// LowPower library
#include "LowPower.h" 


// set up 
void setup() {

  // turn off built-in leds
  pinMode(LED_BUILTIN_TX, INPUT);
  pinMode(LED_BUILTIN_RX, INPUT);
  
  // light control
  pinMode(LIGHT_CTL, OUTPUT);
  digitalWrite(LIGHT_CTL, LOW);

  // amplifier power
  pinMode(AMP_PWR, OUTPUT);
  digitalWrite(AMP_PWR, LOW); 

  // MPU power
  pinMode(MPU_PWR, OUTPUT);
  digitalWrite(MPU_PWR, LOW); 

  // button indicator
  pinMode(BTN_IND, OUTPUT);
  digitalWrite(BTN_IND, LOW);

  // button
  pinMode(BTN_PIN, INPUT_PULLUP);

  // speaker pin
  pinMode(SPKR_PIN, OUTPUT);
  digitalWrite(SPKR_PIN, LOW); 

  // debug pin
  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(BTN_IND, LOW);


  // DEBUG
  if (DEBUG_MASTER) Serial.begin(9600);
    
}

// main loop
void loop() {

  // loop while holding button
  int c = 0;
  while (!digitalRead(BTN_PIN)) {

    // turn on button light before activating switch
    digitalWrite(BTN_IND, HIGH);

    // wait
    delay(10);
    c += 10;

    // if pushed for 400 ms, activate switch
    if(c >= 400) {
      handle_switch();
    }
  }
  // indicator light off when not holding
  digitalWrite(BTN_IND, LOW); 


  // NOTE: 100ms is the longest response time when the button is held down. 
  if (light_on || !(LOWPOWER_MASTER)) // no need to go to low power if saber is on
    delay(100); 
  else
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
  
}
