/* 
 * File:   i2c_master_noint.h
 * Author: mdyehous
 *
 * Created on May 12, 2019, 7:55 AM
 */

#ifndef I2C_MASTER_NOINT_H
#define	I2C_MASTER_NOINT_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* I2C_MASTER_NOINT_H */


#ifndef I2C_MASTER_NOINT_H__
#define I2C_MASTER_NOINT_H__
#include<xc.h>
// Header file for i2c_master_noint.c
// helps implement use I2C2 as a master without using interrupts

void i2c_master_setup(void);              // set up I2C 1 as a master, at 100 kHz

void i2c_master_start(void);              // send a START signal
void i2c_master_restart(void);            // send a RESTART signal
void i2c_master_send(unsigned char byte); // send a byte (either an address or data)
unsigned char i2c_master_recv(void);      // receive a byte of data
void i2c_master_ack(int val);             // send an ACK (0) or NACK (1)
void i2c_master_stop(void);               // send a stop

#endif

