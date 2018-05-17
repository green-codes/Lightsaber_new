/** 
 * header for lightsaber BTN_PIN controls
 * 
 * Menu items
 * 1 : toggle pulse
 * 2 : toggle sound (if available) 
 * 3 : toggle both above; if either is already on, turn off both
 * 4 : toggle LED power saving mode
 * 5 : reset saber
 */

#include "pulse_sound.h" 


// helper for resetting the saber and stopping all operations
// disables peripehral power
// also waits 500ms before returning 
void reset() {

  // flags
  light_on = 0; 
  pulse_enabled = 0;
  sound_enabled = 0; 
  led_low_power = 0; 

  // running functions 
  saber_off();
  digitalWrite(LIGHT_CTL, LOW);
  digitalWrite(BTN_IND, LOW); 
  digitalWrite(AMP_PWR, LOW); 
  digitalWrite(DEBUG_PIN, LOW);

  delay(500);
}


// small helper for displaying flag status
// param: flag. 0 for off, 1 for on
// flash indicator once for off, twice for on 
void display_status(int s) {
  int i;
  for(int i = 0; i < (s ? 2 : 1); i++) {
    digitalWrite(BTN_IND, HIGH);
    delay(200);
    digitalWrite(BTN_IND, LOW);
    delay(200);
  }
}


// helper for displaying error code
// NOTE: displays error code after soft reset
// Errno 1: twi_init failure: - .--
// Errno 4: manual reset: .-. .
int err_short = 100;
int err_long = 300;
int err_pause = 200;
void display_error(int s) {

  // display error code
  if (s == 1) { // twi_init error: - .--
    digitalWrite(BTN_IND, HIGH);
    delay(err_long);
    digitalWrite(BTN_IND, LOW);
    delay(err_pause);
    
    delay(err_pause);
    
    digitalWrite(BTN_IND, HIGH);
    delay(err_short);
    digitalWrite(BTN_IND, LOW);
    delay(err_pause);
    digitalWrite(BTN_IND, HIGH);
    delay(err_long);
    digitalWrite(BTN_IND, LOW);
    delay(err_pause);
    digitalWrite(BTN_IND, HIGH);
    delay(err_long);
    digitalWrite(BTN_IND, LOW);
    delay(err_pause);
  }

  if (s == 4) { // manual reset: .-. .
    digitalWrite(BTN_IND, HIGH);
    delay(err_short);
    digitalWrite(BTN_IND, LOW);
    delay(err_pause);
    digitalWrite(BTN_IND, HIGH);
    delay(err_long);
    digitalWrite(BTN_IND, LOW);
    delay(err_pause);
    digitalWrite(BTN_IND, HIGH);
    delay(err_short);
    digitalWrite(BTN_IND, LOW);
    delay(err_pause);

    delay(err_pause);

    digitalWrite(BTN_IND, HIGH);
    delay(err_short);
    digitalWrite(BTN_IND, LOW);
    delay(err_pause);
  }
}


// handle menu operations
void handle_menu() {

  // stop all timers; disable peripherals
  saber_off();
  digitalWrite(AMP_PWR, LOW);

  // flash light while holding to indicate menu activated
  while(!digitalRead(BTN_PIN)) {
    tone(BTN_IND, 10);
  }
  noTone(BTN_IND); // stop flashing after entering menu


  // get user input for menu item
  // Note: if no user input for 1s, enter the next stage
  int c = 0; 
  int selection = 0; // menu item
  bool pressing = false; // flag to catch a press
  while (c <= 1000) {

    // enter if BTN_PIN press detected
    // for each press, increment selection by 1
    while (!digitalRead(BTN_PIN)) {
      digitalWrite(BTN_IND, HIGH); // turn on indicator
      if(!pressing) selection += 1; // increment selection
      pressing = true; // flip flag to catch the press
      c = 0; // reset time counter to 0;
      delay(10); // save some current draw while pressing  
    }
    digitalWrite(BTN_IND, LOW); // turn off indicator
    pressing = false; 
    
    // delay for loop
    delay(10); 
    c += 10; 
  }


  // play back user selection; if 0, ignore
  for (int i = 0; i < selection; i++) {
    digitalWrite(BTN_IND, HIGH);
    delay(300);
    digitalWrite(BTN_IND, LOW);
    delay(300);
  }


  // handle selection
  // TODO: NOTE: add flag switches here! 
  delay(500); // pause to separate displays
  if (selection == 1) { // switch pulse_enabled mode
    pulse_enabled = pulse_enabled ? 0 : 1; 
    display_status(pulse_enabled); 
  }
  if (selection == 2) { // switch sound
    if (SOUND_MASTER) { // disallow sound when SOUND_MASTER == 0
      sound_enabled = sound_enabled ? 0 : 1; 
      display_status(sound_enabled);
    }
    else {
      sound_enabled = 0; 
      display_status(0);
    }
  }
  if (selection == 3) { // set all flags at once
    if (pulse_enabled || sound_enabled) { // if either is on, turn off both
      pulse_enabled = 0; 
      sound_enabled = 0;
      display_status(0); 
    }
    else {  // otherwise turn on both
      pulse_enabled = 1;
      if (SOUND_MASTER) sound_enabled = 1; 
      display_status(1); 
    }
  }
  if (selection == 4) { // toggle LED power saving
    led_low_power = led_low_power ? 0 : 1; 
    display_status(led_low_power); 
  }
  if (selection == 5) { // reset everything immediately
    reset();
    display_error(4); 
    return; 
  }

  
  // show menu closing animation
  delay(300); 
  c = 0; 
  while (c <= 500) {
    tone(BTN_IND, 10); 
    delay(100);
    c += 100; 
  }
  noTone(BTN_IND); 
  
}


// handle delayed switch activation 
void handle_switch() {

  // turn off BTN_PIN light to indicate switch activation
  digitalWrite(BTN_IND, LOW);

  // flip the light switch immediately
  light_on = light_on ? 0 : 1;


  // power on or off the MPU as necessary
  // NOTE: both pulse and sound may use the MPU
  if (TWI_MASTER && light_on && (pulse_enabled || sound_enabled)) {
    digitalWrite(MPU_PWR, HIGH); 
    delay(10); // wait for the MPU to fully power on before init
    if (0 != twiInit()) display_error(1); 
  }
  else {
    //twiWriteReg(MPU6050_ADDRESS, MPU6050_PWR, 0x48); // put the MPU to sleep
    digitalWrite(MPU_PWR, LOW); 
    twi_init_complete = 0; // MPU powered down
  } 
  
  // Amplifier power
  if (light_on && sound_enabled) digitalWrite(AMP_PWR, HIGH); 
  else digitalWrite(AMP_PWR, LOW);

  
  // switch main light control  
  if (light_on) { 
    digitalWrite(LIGHT_CTL, HIGH);  // pull up no matter what; if PWM, will proceed
    if (sound_enabled || pulse_enabled || led_low_power) 
      start_pulse_sound();
  }
  else {
    saber_off(); // this pulls LIGHT_CTL low
  }

  // Entering menu
  // loop while still holding the button
  int c = 0; // time counter, in ms
  while(!digitalRead(BTN_PIN)) {
    delay(10);
    c += 10; 

    // if holding longer than 2s, activate the menu
    if (c >= 2000) {
      
      // turn off light and flip the flag
      light_on = LOW;
      digitalWrite(LIGHT_CTL, light_on);
      
      handle_menu(); 
    }
  }
  
}
