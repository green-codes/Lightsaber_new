/**
   Pulse and sound cycle controls

   * Original header: speaker_pcm

   Plays 8-bit PCM audio on pin 5 using pulse-width modulation (PWM).
   For Arduino Pro Micro with ATMega32U4 at 16 MHz.

   Uses two timers. The first changes the sample value 8000 times a second.
   The second holds pin 11 high for 0-255 ticks out of a 256-tick cycle,
   depending on sample value. The second timer repeats 62500 times per second
   (16000000 / 256) by setting it to 8-bit fast PWM, much faster than the
   playback rate (8000 Hz), so it almost sounds halfway decent, just really
   quiet on a PC speaker.

   Takes over Timer 1 (16-bit) for the 8000 Hz timer. This breaks PWM
   (analogWrite()) for Pro Micro pins 9 and 10. Takes Timer 3 (16-bit)
   for the pulse width modulation, breaking PWM for pin 5.

   References:
       http://www.uchobby.com/index.php/2007/11/11/arduino-sound-part-1/
       http://www.atmel.com/dyn/resources/prod_documents/doc2542.pdf
       http://www.evilmadscientist.com/article.php/avrdac
       http://gonium.net/md/2006/12/27/i-will-think-before-i-code/
       http://fly.cc.fer.hr/GDM/articles/sndmus/speaker2.html
       http://www.gamedev.net/reference/articles/article442.asp

   Michael Smith <michael@hurts.ca>
   
   * greencodes -- ported code to ATMega32U4 specifically for lightsaber builds  
   
   Now useing three timers: 1, 3 and 4. Timer1 is used to generate the 8000Hz 
   sampling cycle, and Timers 3 and 4 are used for PWM on the light controlling 
   pin and the sound pin. 

*/

#ifndef PULSE_SOUND_H
#define PULSE_SOUND_H

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "Arduino.h" // for digitalWrite

#include "config.h"
#include "twi_gyro.h"
#include "sounddata.h"


// global vars
volatile uint16_t pulse_sample;
volatile uint16_t sound_sample; // sample counters

uint8_t pulse_v = 128; // PWM value for light
uint8_t pulse_d = 0; // pulse direction; 0 for down, 1 for up


/*** TIMER INTERRUPT ***/
/*
   Fetches new samples (PWM values) for sound/pulse at 8000Hz.
*/
ISR(TIMER1_COMPA_vect) {

  // Sound / Timer3
  if (sound_enabled) {

    // power on sound FX
    if (power_on) {  // saber powering on
      if (sound_sample < sizeof(sound_on)) 
        OCR3A = pgm_read_byte(&sound_on[sound_sample++]);
      else {  // reached end of power on sound, switch to humming
        sound_sample = 0;
        power_on = 0; // clear the power on flag
                      // NOTE: only sound clears this flag. 
      }
    }

    // power off sound FX
    /*else if (power_off) {
      if (sound_sample < sizeof(sound_off)) 
        OCR3A = pgm_read_byte(&sound_off[sound_sample++]);
      else { 
        sound_sample = 0;
        power_off = 0; // clear the power off flag
      }
    }*/

    // humming sound
    else { 

      // if reached end of data, restart at sample 0
      if (sound_sample >= sounddata_length) sound_sample = 0;
  
      // load the next sample
      if (TWI_MASTER)
        OCR3A = ( pgm_read_byte(&sound_hum_slow[sound_sample]) * (1-ang_v) +
                  pgm_read_byte(&sound_hum_fast[sound_sample]) * ang_v );
      else OCR3A = pgm_read_byte(&sound_hum_slow[sound_sample]);
                
      ++sound_sample;  // increment counter to the next sample

    }
  }

  // Pulse / Timer4
  if (pulse_enabled) {

    /*
      // if reached end of data, restart at sample 0
      if (pulse_sample >= sounddata_length) pulse_sample = 0;
      // load the next sample
      OCR4B = pgm_read_byte(&sound_hum_slow[pulse_sample]);
      ++pulse_sample;  // increment counter to the next sample
    */

    // power on FX
    // NOTE: currently the power_on flag is only operated by the sound switch
    if (power_on) {
      pulse_v = pgm_read_byte(&sound_on[sound_sample]);  //TODO
    }

    // idling FX
    else {

      // TODO: better pulse FX? Gyro? 
      // Triangular wave at 8000/(100*2) = 40Hz
      if (pulse_v > 210) pulse_d = 0;
      if (pulse_v < 10) pulse_d = 1;  //TODO: use ang_v to increase lower bound; 
                                      //      this adds pulse frequency and brightness
      pulse_v = (pulse_d) ? pulse_v + 2 : pulse_v - 2;

    }

    // send PWM value to OCR4B; if LED power saving, dim
    if (led_low_power) OCR4B = (int) (pulse_v * led_pwr_ratio);
    else OCR4B = pulse_v; 
    
  }
  else {  // non-pulsing light
    if (led_low_power) OCR4B = (int) (255 * led_pwr_ratio);
    else OCR4B = 255;  // max brightness; NOTE: here just in case Timer 4 on (shouldn't). 
  }


  // DEBUG: doing ISR cycle?
  /*if (DEBUG_MASTER && (debug_count++ > 1000)) {
    debug_count = 0;
    debug_flag = (debug_flag) ? 0 : 1;
    digitalWrite(DEBUG_PIN, debug_flag);
  }*/

  // call TWI cycle function
  // NOTE: do NOT call when the MPU is powered down
  if (TWI_MASTER && (pulse_enabled || sound_enabled)) twiCycle();

}


/*** Sound/Pulse control ***/

// stops all timers; resets TWI; pulls light and sound pins low.
void saber_off()
{

  // ensure that important variables are reset
  power_on = 0; 
  power_off = 0; 
  light_on = 0;  // already done in saber_ctl, just to be safe
  
  // Disable the 8000Hz interrupt on Timer1.
  TIMSK1 &= ~_BV(OCIE1A);

  // Disable Timer1 completely.
  TCCR1B &= ~_BV(CS10);

  // Disable the PWM timers.
  TCCR3B &= ~_BV(CS30);
  TCCR4B &= ~_BV(CS40);

  // disconnect timers from OC3A and OC4B pins
  TCCR3A &= ~(_BV(COM3A1) | _BV(COM3A0));
  TCCR4A &= ~(_BV(COM4B1) | _BV(COM4B0));

  // reset the TWI bus
  if (TWI_MASTER) twiReset(); 

  // reset light and speaker pins
  digitalWrite(LIGHT_CTL, LOW);
  digitalWrite(BTN_IND, LOW);
  digitalWrite(AMP_PWR, LOW);
  digitalWrite(MPU_PWR, LOW);
  digitalWrite(SPKR_PIN, LOW);
  digitalWrite(DEBUG_PIN, LOW);

}

// set up timers necessary for pulse/low-power and sound
void start_pulse_sound()
{

  // disable interrupt to set up timers
  cli();

  // set power-on state
  // if (sound_enabled || pulse_enabled) power_on = 1; 

  // if sound_enabled, configure Timer 3 for PWM on the speaker pin
  if (sound_enabled) {

    power_on = 1; // the power_on flag is operated by the sound sample counter

    // Set fast PWM mode  (p.133/157)
    TCCR3A |= _BV(WGM30);
    TCCR3A &= ~_BV(WGM31);
    TCCR3B |= _BV(WGM32);

    // Do non-inverting PWM on pin OC3A (p.131)
    TCCR3A = (TCCR3A | _BV(COM3A1)) & ~_BV(COM3A0); // OC3A clear on compare match
    TCCR3A &= ~(_BV(COM3B1) | _BV(COM3B0)); // OC3B disconnected

    // Enable Timer; no prescaler (p.134)
    TCCR3B = (TCCR3B & ~(_BV(CS32) | _BV(CS31))) | _BV(CS30);

    // Set initial pulse width to 128.
    OCR3A = 128;

    sound_sample = 0;
  }


  // if pulse or low-power, configure Timer 4 for PWM on the light control pin
  if (pulse_enabled || led_low_power) {

    // Set fast PWM mode for Timer 4 (p.170)
    TCCR4D &= ~_BV(WGM40);
    TCCR4D &= ~_BV(WGM41);

    // set PWM to operate on OCR4B
    TCCR4A |= _BV(PWM4B);

    // connect OC4B; clear on compare match (p.166)
    TCCR4A = (TCCR4A | _BV(COM4B1)) & ~_BV(COM4B0);

    // disconnect OC4A, which shares pin with OC3A (sound)
    TCCR4A &= ~(_BV(COM4A1) | _BV(COM4A0));

    // Enable Timer 4, no prescaler (p.168)
    TCCR4B = (TCCR4B & ~(_BV(CS42) | _BV(CS41)) ) | _BV(CS40);

    // Set initial pulse width
    OCR4B = 128;

    pulse_sample = 0;
  }


  // Set up Timer 1 to send a sample every interrupt.

  // Set WGM to CTC mode (Clear Timer on Compare Match) (p.133)
  // Have to set OCR1A *after*, otherwise it gets reset to 0!
  // NOTE: OC1A/B/C disabled! Free to use Timer 4
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12); //
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));

  // No prescaler (p.134)
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

  // Set the compare register (OCR1A).
  // OCR1A is a 16-bit register, so we have to do this with
  // interrupts disabled to be safe.
  OCR1A = F_CPU / SAMPLE_RATE;    // 16e6 / 8000 = 2000

  // Enable interrupt when TCNT1 == OCR1A (p.136)
  TIMSK1 |= _BV(OCIE1A);


  // setup complete, resume interrupt (for all Timers)
  sei();

}

#endif // PULSE_SOUND_H
