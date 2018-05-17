/**
 * configs, pin mappings and global flags
 */

#ifndef CONFIG_H
#define CONFIG_H

// Master flags
#define LOWPOWER_MASTER   1
#define SOUND_MASTER      1
#define TWI_MASTER        1
#define DEBUG_MASTER      0

// sound definition
#define SAMPLE_RATE 8000

// Control pins
#define LIGHT_CTL     10
#define BTN_IND       8
#define BTN_PIN       A1
#define DEBUG_PIN     A2
// Speaker pins
#define SPKR_PIN      5
#define AMP_PWR       7
// TWI pins
#define MPU_PWR       9
#define SDAPIN        2
#define CLKPIN        3

// global status flags 
uint8_t light_on = 0;       // main LED control
uint8_t pulse_enabled = 0;  // LED pulse mode
uint8_t sound_enabled = 0;  // sound 
uint8_t led_low_power = 0;  // LED Powersaving
uint8_t power_on = 0;       // on during saber startup
uint8_t power_off = 0;      // on during saber shutdown
float led_pwr_ratio = 0.8 * 0.10; // latter is brightness fraction

// DEBUG flags and counter
int debug_count = 0; // counter shouldn't be too small
uint8_t debug_flag = 0;

#endif /*CONFIG_H*/
