#include <math.h> 	//for sine wave plotting
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "NU32.h"
#include <stdint.h>
//#include "NU32.h"       // constants, funcs for startup and UART
// Demonstrates spi by accessing external ram
// PIC is the master, ram is the slave
// Uses microchip 23K256 ram chip (see the data sheet for protocol details)
// SDO1 -> SI
// SDI1 -> SO (pin F4 -> pin 2)
// SCK1 -> SCK (pin B14 -> pin 6)
// SS4 -> CS (pin B8 -> pin 1) ///////////////

#define CS LATBbits.LATB8       // chip select pin

/////

// DEVCFG0
#pragma config DEBUG = 0x3 // no debugging
#pragma config JTAGEN = 0x0 // no jtag
#pragma config ICESEL = 0x3 // use PGED1 and PGEC1
#pragma config PWP = 0x1 // no write protect
#pragma config BWP = 0x1 // no boot write protect
#pragma config CP = 0x1 // no code protect

// DEVCFG1
#pragma config FNOSC = 0x3 // use primary oscillator with pll
#pragma config FSOSCEN = 0x0 // turn off secondary oscillator
#pragma config IESO = 0x0 // no switching clocks
#pragma config POSCMOD = 0x2 // high speed crystal mode
#pragma config OSCIOFNC = 0x1 // disable secondary osc
#pragma config FPBDIV = 0x0 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = 0x2 // do not enable clock switch
#pragma config WDTPS = 0x14 // use slowest wdt
#pragma config WINDIS = 0x1 // wdt no window mode
#pragma config FWDTEN = 0x0 // wdt disabled
#pragma config FWDTWINSZ = 0x3 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = 0x1 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = 0x7 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = 0x1 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = 0x1 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = 0x0 // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = 0x0 // allow multiple reconfigurations
#pragma config IOL1WAY = 0x0 // allow multiple reconfigurations
#pragma config FUSBIDIO = 0x1 // USB pins controlled by USB module
#pragma config FVBUSONIO = 0x1 // USB BUSON controlled by USB module
/////

void init_spi1() {
  //RPB8Rbits.RPB8R = 0b0011; // ties SDO1 to RB8
  RPB5Rbits.RPB5R = 0b0011; // ties SDO1 to RB5
  SDI1Rbits.SDI1R = 0b0011; // ties SDI1 to RB11
  
  // Master - SPI1  
  // we manually control SS4 as a digital output (F12) ????????????????????
  // since the pic is just starting, we know that spi is off. We rely on defaults here
 
  // setup spi1 , all bits must be changed for SPI1 from SPI1
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x3;            // baud rate to 10 MHz [SPI1BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 1
  
}

// send a byte via spi to the DAC in particular (16 bits at a time) and return the response
unsigned char spi1_io_DAC(uint16_t input) {
  unsigned char left0 = (unsigned char) ((input >> 12) & 0xF);
  unsigned char left1 = (unsigned char) ((input >> 8) & 0xF);
  unsigned char left2 = (unsigned char) ((input >> 4) & 0xF);
  unsigned char left3 = (unsigned char) (input & 0xF);
  spi1_io(left0);
  spi1_io(left1);
  spi1_io(left2);
  return spi1_io(left3);
}

unsigned char spi1_io(unsigned char input) {
  SPI1BUF = input;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

//uint16_t packDAC(uint16_t ab, uint16_t buf, uint16_t gain, uint16_t shutdown, uint16_t n) {
//    return n | (ab << 15) | (buf << 14) | (gain << 13) | (shutdown << 12);
//}

int main(void) {

  init_spi1();

  while(1) {
	_CPO_SET_COUNT(0);
	//float f = 512 +512*sin(i*2*3.1415/1000*10);  //should make a 10Hz sin wave)
	i++;

	setVoltage(0,512);		//test
	setVoltage(1,256);		//test

	while(_CPO_GET_COUNT() < 2400000000/1000) {}  //check this is 24Million
    ;
  }
  return 0;
}



void setVoltage(char a, int v) {

	unsigned short t = 0;
	t= a << 15; //a is at the very end of the data transfer
	t = t | 0b01110000000000000;
	t = t | ((v&0b111111111111) <<2); //rejecting excessive bits (above 10)
	
	CS = 0;
	//spi1_io(t>>8);
	spi1_io(t);
    CS = 1;
	
}