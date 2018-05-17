/*
   TWI MPU6050 operations

   This header defines TWI-related functions for the MPU6050 gyro, which
   operates directly on registers and does not interfere with timers, etc.

   These functions should be called at timer interrupts or in sufficiently
   slow loops (depending on the speed of the I2C bus).
*/

#ifndef TWI_GYRO_H
#define TWI_GYRO_H

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "Arduino.h" // for digitalWrite
#include "twi.h"  // for twi TWSR values

#include "config.h"

#define TWI_RETRIES 10 // 5 retries before abandoning TWI operations


// TWI definitions -- do NOT include the Wire twi library, it's all defined in here
#define TWI_FREQ 100000L

// Basic TWI states
#define TWI_UNK   -1 // Error or unknown condition; twiInit() required
#define TWI_READY 0  // STOP condition set in TWCR
#define TWI_MRX   1
#define TWI_MTX   2
#define TWI_SRX   3
#define TWI_STX   4

// TWI session states
#define MPU_START         11 // at first START, sending SLA+W
#define MPU_TX_SLAW_ACK   12 // SLA+W ACK, sending data address
#define MPU_TX_DATA_ACK   13 // data address ACK, sending REP_START
#define MPU_REP_START     14 // at REP_START, slave waiting for SLA+R
#define MPU_RX_SLAR_ACK   15 // SLW+R ACK, slave starting transmission
#define MPU_RX_DATA_ACK   16 // slave data rcvd, ACK sent
#define MPU_RX_DATA_NACK  17 // slave data rcvd, NACK sent

// MPU6050 definitions
#define MPU6050_ADDRESS   0x68  // from MPU6050 datasheet
#define MPU6050_D_ADDRESS 0x3B  // starting data: ACCEL_XOUT_H
#define MPU6050_PWR       0x6B  // register for activating MPU
#define MPU6050_GYRO_CONF 0x1B  // register for configuring MPU gyro
#define MPU6050_GYRO_SET  0x10  // MPU gyro sensitivity setting; see register map
#define MPU6050_RST_CONF  0x68  // register for resetting MPU
#define MPU6050_RST_SET   0x07  // settings: reset all sensors (map p.37)
// NOTE: used magic number to set MPU6050_PWR


#define ANGS_NOR_FACTOR  32767 // MPU6050's ang_v normalization factor


/******* Global vars ********/

// global vars
double ang_v;  // abs. angular speed value for adjusting sound


// TWI vars
uint8_t twi_slarw;  // byte buffer for transmitting SLA+R or SLA+W packets
uint8_t twi_init_complete = 0;  // must set this before interrupts can update TWI
static volatile uint8_t twi_state = TWI_UNK;   // can't set to ready yet!


// MPU6050 raw readouts (p.29-31)
// NOTE: it actually saves time to read the entire struct from MPU6050...
// total: 14 bytes == about 20 cycles to read entire struct once
//    (reading 3 bytes individually will take 3*7 = 21 cycles)
int data_counter, data_union_size;
typedef union data_union_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t temp_h;
    uint8_t temp_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct
  {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temp;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  } value;
};

// instantiate data_union and get a pointer for writing
data_union_union data_union;
uint8_t * data_ptr = (uint8_t *) &data_union;


/* 
 * Force TWI bus reset; bus will be free after return
 * 
 * Clocks 8 cycles while keeping SDA high to let the slave finish
 * any remaining transmission, then clock and NACK to stop slave from 
 * further transmitting. A STOP is then sent to vacate the bus. 
 * 
 * Source: https://github.com/esp8266/Arduino/issues/1025 
 */
void twiReset() { 

  /*** Force physical STOP condition on the bus ***/ 
  
  //try i2c bus recovery at 100kHz = 5uS high, 5uS low
  pinMode(SDAPIN, OUTPUT);//keeping SDA high during recovery
  digitalWrite(SDAPIN, HIGH);
  pinMode(CLKPIN, OUTPUT);
  for (int i = 0; i < 10; i++) { //9nth cycle acts as NACK
    digitalWrite(CLKPIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(CLKPIN, LOW);
    delayMicroseconds(5);
  }

  //a STOP signal (SDA from low to high while CLK is high)
  digitalWrite(SDAPIN, LOW);
  delayMicroseconds(5);
  digitalWrite(CLKPIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(SDAPIN, HIGH);
  delayMicroseconds(2);
  //bus status is now : FREE
  pinMode(SDAPIN, INPUT);
  pinMode(CLKPIN, INPUT);

  /*** reset TWI control registers ***/

  TWCR = 0; // simply clear everything

  /*** cleanup ***/ 

  twi_state = TWI_READY; // bus is vacated after STOP
  
}


/*
   Writes one register (reg address + reg value) to TWI interface
   Code is fragile: if any error occurs during data writing, this returns -1.

   Process:
   START => SLA+W => send activation reg address => sent data byte => STOP
*/
int twiWriteReg(uint8_t address, uint8_t d_address, uint8_t data) {

  // return error if not TWI_READY; the caller is responsible
  if ( twi_state != TWI_READY ) return -1;

  // set TWI START condition; interrupt disabled
  TWCR = _BV(TWEN) | _BV(TWSTA) | _BV(TWINT);

  // wait for TWINT; max five cycles, if no TWINT, reset bus. 
  for (int i = 0; i < TWI_RETRIES; i++) { delay(1); if ( TWCR & _BV(TWINT) ) break; }
  if ( !( TWCR & _BV(TWINT) ) ) { twiReset(); return -1; }

  // check if TWI started
  if ( TW_STATUS != TW_START ) return -1;
  twi_state = MPU_START;

  // write SLA+W address packet
  twi_slarw = 0x0 | (address << 1); // 0 for write
  TWDR = twi_slarw;  // load address packet
  TWCR = ( _BV(TWEN) | _BV(TWINT) );

  // wait for TWINT
  for (int i = 0; i < TWI_RETRIES; i++) { delay(1); if ( TWCR & _BV(TWINT) ) break; }
  if ( !( TWCR & _BV(TWINT) ) ) { twiReset(); return -1; }
  

  // check if address packet ack'ed by MPU
  if ( TW_STATUS != TW_MT_SLA_ACK ) return -1;
  twi_state = MPU_TX_SLAW_ACK;

  // write data register address
  TWDR = d_address;
  TWCR = ( _BV(TWEN) | _BV(TWINT) );

  // wait for TWINT
  for (int i = 0; i < TWI_RETRIES; i++) { delay(1); if ( TWCR & _BV(TWINT) ) break; }
  if ( !( TWCR & _BV(TWINT) ) ) { twiReset(); return -1; }
  
  // check if data packet ack'ed by MPU
  if ( TW_STATUS != TW_MT_DATA_ACK ) return -1;
  twi_state = MPU_TX_DATA_ACK;

  // write register data
  TWDR = data;
  TWCR = ( _BV(TWEN) | _BV(TWINT) );

  // wait for TWINT
  for (int i = 0; i < TWI_RETRIES; i++) { delay(1); if ( TWCR & _BV(TWINT) ) break; }
  if ( !( TWCR & _BV(TWINT) ) ) { twiReset(); return -1; }
  
  // check if data packet ack'ed by MPU
  if ( TW_STATUS != TW_MT_DATA_ACK ) return -1;
  twi_state = MPU_TX_DATA_ACK;

  // set STOP condition
  TWCR = _BV(TWEN) | _BV(TWSTO) | _BV(TWINT);


  // set flag to allow new data transmission
  twi_state = TWI_READY;

  return 0;   // successful write
}



/*** TWI/GYRO OPERATIONS ***/

/*
   init TWI interface
   Returns 0 if successful, -1 if error
*/
int twiInit() {

  if (DEBUG_MASTER) {
    Serial.print("Initializing TWI bus; current TWCR/TWSR : "); 
    Serial.print(TWCR);
    Serial.print("/");
    Serial.println(TWSR);
  }

  // setup for buffer 
  data_union_size = sizeof( data_union );
  data_counter = 0;

  // Activate MPU and set gyroscope configuration
  for( uint8_t i = 0; i < TWI_RETRIES; i++ ) {

    // reset the TWI bus to the STOP condition
    twiReset();  // twi_state == TWI_READY

    // clear prescaler (TWPS0 and TWPS1) bits in TWSR (or we don't get 400kHz)
    TWSR = TWSR & ~( _BV(TWPS0) | _BV(TWPS1) );

    // set TWI bitrate, using TWI_FREQ = 400000L
    TWBR = 12;  // ( (16e6 / 4e5) - 16 ) / 2
  
    // try writing MPU startup registers
    // Note: steps - reset MPU => sleep off => gyro config
    if ( 0 == twiWriteReg(MPU6050_ADDRESS, MPU6050_RST_CONF, MPU6050_RST_SET) &&
         0 == twiWriteReg(MPU6050_ADDRESS, MPU6050_PWR, 0) &&
         0 == twiWriteReg(MPU6050_ADDRESS, MPU6050_GYRO_CONF, MPU6050_GYRO_SET) ) {
      // all registers written, TWI bus and MPU init complete
      twi_init_complete = 1;  // TWI init complete, ISR TWI cycle can start
      return 0;         
    }
    
  }

  // could not write to MPU startup registers
  return -1; 

}


/*
   do TWI cycle

   Note: Porting twi.c will likely cause mistakes, since I'm not doing ISR for TWI.
         Writing this myself.
         
   Process:
   START => SLA+W => Data address => REP_START => SLA+R => Data receive => NACK, STOP, ...

   Note: probably want a flag to mark restart...
*/
uint8_t hang_counter = 0;
void twiCycle() {

  //DEBUG
  /*Serial.print(TWCR);
  Serial.print("/");
  Serial.println(TWSR);
  */

  // check if TWI init process complete; if not, initialize and skip to next cycle. 
  if ( !twi_init_complete) { twiInit(); return; }

  // if initialized and in STOP state, start a new TWI session and skip to next cycle
  // Note: if a STOP was sent, TWINT is not set by TWI module.
  // Note: if executed immediately after setting a STOP, this may fail as the TWI
  //       module might not have time to process the STOP before the START is set.
  //       (Shouldn't be a problem when this is called at 8000Hz)
  if ( twi_state == TWI_READY ) {

    // send START condition
    TWCR = _BV(TWEN) | _BV(TWSTA) | _BV(TWINT);

    // set tw_state to transmitting
    twi_state = TWI_MTX;

    // reset data_counter
    data_counter = 0;
    return;
  }


  // if TWINT flag is not set, TWI operation has not completed; skip cycle.
  // NOTE: // Allow the TWI bus to hang for TWI_RETRIES cycles, then reset bus. 
  
  if ( !( TWCR & _BV(TWINT) ) ) {
    if (hang_counter++ < TWI_RETRIES) return; 
    twiInit(); return; 
  }
  hang_counter = 0; // no longer hanging, proceed

  // Check TWSR code and proceed
  switch (TW_STATUS) {

    // START:
    // send SLA+W packet
    case TW_START:
      twi_state = MPU_START;

      // build SLA+W packet
      twi_slarw = 0 | (MPU6050_ADDRESS << 1); // 0 for write

      TWDR = twi_slarw;  // load address packet
      TWCR = ( _BV(TWEN) | _BV(TWINT) );  // send slarw packet

      break;


    // SLA+W ACK:
    // slave selected, send data address packet
    case TW_MT_SLA_ACK:
      twi_state = MPU_TX_SLAW_ACK;

      // build data address packet
      twi_slarw = MPU6050_D_ADDRESS;

      // load packet and send
      TWDR = twi_slarw;
      TWCR = ( _BV(TWEN) | _BV(TWINT) );

      break;


    // Data address ACK:
    // slave received data address, send repeated START
    case TW_MT_DATA_ACK:
      twi_state = MPU_TX_DATA_ACK;

      // set START
      TWCR = _BV(TWEN) | _BV(TWSTA) | _BV(TWINT);

      break;


    // REP_START:
    // if TW_MTX:
    // if TW_MRX: slave ready to transmit, send SLA+R packet
    case TW_REP_START:
      twi_state = MPU_REP_START;

      // build SLA+R packet
      twi_slarw = 1 | (MPU6050_ADDRESS << 1); // 1 for read

      TWDR = twi_slarw;  // load address packet
      TWCR = ( _BV(TWEN) | _BV(TWINT) );  // send SLA+R packet

      break;


    // SLA+R ACK:
    // clear TWINT to start reception. make sure to send ACK after first read.
    case TW_MR_SLA_ACK:
      twi_state = MPU_RX_SLAR_ACK;

      // set flags to start data reception; setting TWEA to send ACK.
      TWCR = ( _BV(TWEN) | _BV(TWEA) | _BV(TWINT) );

      // set MRX flag
      twi_state = TWI_MRX;

      break;


    // Data byte received, ACK sent:
    // record data and clear TWINT. if second to last byte, set NACK
    case TW_MR_DATA_ACK:
      twi_state = MPU_RX_DATA_ACK;

      // record current data in TWDR
      data_ptr[data_counter] = TWDR;

      // check if second to last byte, then increment data_counter
      if ( data_counter++ < data_union_size - 2 )
        TWCR = ( _BV(TWEN) | _BV(TWEA) | _BV(TWINT) );  // send ACK
      else
        TWCR = ( _BV(TWEN) | _BV(TWINT) );  // send NACK

      break;


    // Last data byte received, NACK sent:
    // record last byte, swap high/low bytes in data_union, calculate angular
    // speed, then set STOP condition.
    // NOTE: the microcontroller has ~2000 CPU cycles to do all these (16M/8000)
    case TW_MR_DATA_NACK:
      twi_state = MPU_RX_DATA_NACK;

      // DEBUG: receiving complete data struct?
      if (DEBUG_MASTER && debug_count++ > 20) {
        debug_flag = debug_flag ? 0 : 1;
        digitalWrite(DEBUG_PIN, debug_flag);
        debug_count = 0;
        Serial.println(ang_v);
      }

      // record last byte
      data_ptr[data_counter] = TWDR;

      // Swap high/low bytes in data_union
      uint8_t swap;
      #define SWAP(x,y) swap = x; x = y; y = swap  // for convenience
      SWAP (data_union.reg.x_accel_h, data_union.reg.x_accel_l);
      SWAP (data_union.reg.y_accel_h, data_union.reg.y_accel_l);
      SWAP (data_union.reg.z_accel_h, data_union.reg.z_accel_l);
      SWAP (data_union.reg.temp_h, data_union.reg.temp_l);
      SWAP (data_union.reg.x_gyro_h, data_union.reg.x_gyro_l);
      SWAP (data_union.reg.y_gyro_h, data_union.reg.y_gyro_l);
      SWAP (data_union.reg.z_gyro_h, data_union.reg.z_gyro_l);

      // calculate normalized, absolute angular speed
      // Note: no need for abs since we're squaring the numbers
      // Note: Y axis removed, irrelevant to saber swing
      ang_v = sqrt( pow ( ((double) data_union.value.x_gyro) / ANGS_NOR_FACTOR , 2 ) +
                        pow ( ((double) data_union.value.z_gyro) / ANGS_NOR_FACTOR , 2 ) );
      ang_v = (ang_v > 1.0) ? 1.0 : ang_v;

      // set STOP
      TWCR = _BV(TWEN) | _BV(TWSTO) | _BV(TWINT);

      // reset tw_state flag to prepare for the next transmission
      twi_state = TWI_READY;

      break;


    // MT/MR arbitration lost (both 0x38)
    // Note: there will only ever be 1 master on the bus. Simply restart.
    // send a START to restart data transmission. (p.239, 242)
    case TW_MT_ARB_LOST:

      // send START condition to restart TWI session
      TWCR = _BV(TWEN) | _BV(TWSTA) | _BV(TWINT);

      break;


    // other cases: other error occured (unexpected NACK, TWINT stall, etc)
    // force TWI bus reinitialization
    default:
      twiInit();

  };  // end of switch block

}

#endif // TWI_GYRO_H
