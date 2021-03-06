#include<xc.h>           // processor SFR definitions
#include <math.h> 	//for sine wave plotting
#include <stdint.h>
#include <stdio.h>
#include<sys/attribs.h>  // __ISR macro
//#include "NU32.h"       // constants, funcs for startup and UART
// Demonstrates spi by accessing external ram
// PIC is the master, ram is the slave
// Uses microchip 23K256 ram chip (see the data sheet for protocol details)
// SDO1 -> SI
// SDI1 -> SO (pin F4 -> pin 2)
// SCK1 -> SCK (pin B14 -> pin 6)
// SS4 -> CS (pin B8 -> pin 1) ///////////////

//#define CS LATBbits.LATB8       // chip select pin

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

/*void init_spi1() {
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
  
}*/

/*unsigned char spi1_io(unsigned char input) {
  SPI1BUF = input;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}*/

#define ILI9341_RED         0xF800

void writeBuffer(char *cbuf, unsigned short startX, unsigned short startY) {
    char len = 16;
    unsigned short newX = 0, newY = 0, index = 0, diffX = 6;
    unsigned char toUse;
    while (index < len) {
          toUse = (*(cbuf + index));
          newX = startX + (index * diffX);
          //newY = startY;
          writeChar(newX, startY, toUse);
          //writeChar(test + index * diffX, 200, 23);
          //writeChar(newX, newY, 23);//cbuf[index]);
          index++;
          /*_CP0_SET_COUNT(0);
          while (_CP0_GET_COUNT() < 100000) {
              ;
          }*/
          
      }
}

void writeProgressBar(unsigned short perc, unsigned short startX, unsigned short startY) {
    unsigned char i = 0;
    while (i < perc) {
        writeChar(startX-2+i, startY + 10, '|');
        i++;
    }
}

void clearBar(unsigned short startX, unsigned short startY, unsigned short len) {
    unsigned char i = 0;
    unsigned short diffX = 5;
    while (i < len) {
        writeCharClear(startX+diffX*i, startY);
        i++;
    }
}

int main(void) {
  
  __builtin_disable_interrupts();

  // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
  __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
  
  SPI1_init();
  LCD_init();

  LCD_clearScreen(0x0000);
//  _CP0_SET_COUNT(0);
//  while (_CP0_GET_COUNT() < 1000000) {
//      ;
//  }
//  unsigned short xtest = 100;
//  unsigned short ytest = 100;
//  LCD_drawPixel(xtest, ytest, ILI9341_RED);
//  LCD_drawPixel(xtest+1, ytest, ILI9341_RED);
//  LCD_drawPixel(xtest+2, ytest, ILI9341_RED);
//  LCD_drawPixel(xtest, ytest+1, ILI9341_RED);
//  LCD_drawPixel(xtest+1, ytest+1, ILI9341_RED);
//  LCD_drawPixel(xtest+2, ytest+1, ILI9341_RED);
//  LCD_drawPixel(xtest, ytest+2, ILI9341_RED);
//  LCD_drawPixel(xtest+1, ytest+2, ILI9341_RED);
//  LCD_drawPixel(xtest+2, ytest+2, ILI9341_RED);
  
  //writeChar(200, 200, 21);
  //writeChar(206, 200, 21);
  //writeChar(212, 200, 21);
  //writeChar(188, 200, 21);
  __builtin_enable_interrupts();
  
  //char index = 0;
  char len = 16;
  char cbuf[len];
  sprintf(cbuf, "Hello world %d!", 0);
  
  unsigned short startX = 20, startY = 20, diffX = 6, newX = 0, newY = 0;
  unsigned short test = 50;
  unsigned short perc = 0;
  unsigned char toUse;

  while(1) {
      clearBar(startX, startY, 20);
      clearBar(startX, startY+10, 20);
      clearBar(startX, startY+20, 20);
      sprintf(cbuf, "Hello world %d!", perc);
      _CP0_SET_COUNT(0);
      writeBuffer(&cbuf, startX, startY);
      writeProgressBar(perc, startX, startY);
      //writeChar(100, 200, 21);
      /*while (index < len) {
          toUse = cbuf[index];
          writeChar(test, 200, 23);
          writeChar(test + index * diffX, 200, 23);
          newX = startX + (index * diffX);
          newY = startY;
          writeChar(newX, startY, toUse);
          //writeChar(test + index * diffX, 200, 23);
          //writeChar(newX, newY, 23);//cbuf[index]);
          index++;
          _CP0_SET_COUNT(0);
          while (_CP0_GET_COUNT() < 100000) {
              ;
          }
          
      }*/
      //index = 0;
//      LCD_clearScreen(0x0000);
//      _CP0_SET_COUNT(0);
//      while (_CP0_GET_COUNT() < 1000000) {
//          ;
//      }
//      LCD_clearScreen(0xFFFF);
      int start = _CP0_GET_COUNT();
      _CP0_SET_COUNT(0);
      while (_CP0_GET_COUNT() < 800000) {
          ;
      }
      unsigned int fps = (48000000) / (start + _CP0_GET_COUNT());
      sprintf(cbuf, "FPS: %d        ", fps);
      writeBuffer(&cbuf, startX, startY+20);
      //LCD_clearScreen(0x0000);
      perc++;
      perc %= 101;
  }
  return 0;
}