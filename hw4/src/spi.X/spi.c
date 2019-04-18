#include<xc.h>           // processor SFR definitions
#include <math.h> 	//for sine wave plotting
#include <stdint.h>
#include<sys/attribs.h>  // __ISR macro
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
  SPI1BRG = 0x9C3; // this makes it 9600 baud //0x3;            // baud rate to 10 MHz [SPI1BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 1
  
}

unsigned char spi1_io(unsigned char input) {
  SPI1BUF = input;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

// send a byte via spi to the DAC in particular (16 bits at a time) and return the response
unsigned char spi1_io_DAC(uint16_t input) {
  unsigned char left = (unsigned char) ((input >> 8) & 0xFF);
  unsigned char right = (unsigned char) (input & 0xFF);
  spi1_io(left);
  return spi1_io(right);
}

void setVoltage(char a, int v) {

	unsigned short t = 0;
	t= a << 15; //a is at the very end of the data transfer
	t = t | 0b0111000000000000;
	t = t | ((v&0b111111111111));
	
	CS = 0;
	//spi1_io(t>>8);
	spi1_io_DAC(t);
    CS = 1;
	
}

int main(void) {
  
  __builtin_disable_interrupts();
  CS = 1;
  init_spi1();
  // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
  __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

  // 0 data RAM access wait states
  BMXCONbits.BMXWSDRM = 0x0;

  // enable multi vector interrupts
  INTCONbits.MVEC = 0x1;

  // disable JTAG to get pins back
  DDPCONbits.JTAGEN = 0;
  TRISAbits.TRISA4 = 0;
  TRISBbits.TRISB4 = 1;
  TRISBbits.TRISB8 = 0;
  LATAbits.LATA4 = 1; // LED pin high

  // do your TRIS and LAT commands here

  __builtin_enable_interrupts();
  
  uint32_t i = 0, j = 0;
  int sign = 150;
  int toAdd = 0;

  while(1) {
    //
    /*if (PORTBbits.RB4) {
            _CP0_SET_COUNT(0);
            LATAbits.LATA4 = 0;
            while(_CP0_GET_COUNT() < 12000) {
                ;
            }
            LATAbits.LATA4 = 1;
            _CP0_SET_COUNT(0);
            while(_CP0_GET_COUNT() < 12000) {
                ;
            }
    }*/
    //      
      
	_CP0_SET_COUNT(0);
	float f = 2048 + 2047 * sin(i*2*3.1415/300*10);  //should make a 10Hz sin wave)
    
    if ((toAdd + 2047 > 4094) || (toAdd + 2047 <= 0)) {
        sign *= -1;
    }
    
    int g = 2047 + toAdd;
    if (g >= 4094) {
        g = 4094;
    } else if (g < 0) {
        g = 0;
    }
	i++;
    toAdd += sign;
    //if (j >= 4094) {
    //    j = 0;
    //}
    //int rounded = (int) roundf((float) f);

	//setVoltage(0,512);		//test
	//setVoltage(1,256);		//test
    setVoltage(0, ceilf(f));
    setVoltage(1, g);
    //setVoltage(1, 1);

	//while(_CP0_GET_COUNT() < 100) {}  //check this is 24Million
    ;
  }
  return 0;
}