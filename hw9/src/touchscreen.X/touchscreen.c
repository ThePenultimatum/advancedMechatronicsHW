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


/*
 * #define SLAVE_ADDR 0b01000000//0x40
#define GPIO 0x09
#define IODIR 0x00
*/
/////
#define SLAVE_ADDR_READ 0b11010111 //0x40
#define SLAVE_ADDR_WRITE 0b11010110 //0x40
#define CTRL1_XL 0b00010000
#define CTRL2_G 0b00010001
#define CTRL3_C 0b00010010
#define ALL_REG 0x20
#define ALL_BYTES 14
#define TEMP_REG 0x20
#define TEMP_BYTES 2
#define GYRO_REG 0x22
#define GYRO_BYTES 6
#define ACCEL_REG 0x28
#define ACCEL_BYTES 6

void setupHardware(void);
void writeToRegister(char, char);

void I2C_read_multiple(unsigned char address, unsigned char reg, unsigned char * imuData, int length) {
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR_WRITE);
    i2c_master_send(reg);
    //i2c_master_send(0b00000011);
    //i2c_master_stop();
        
    i2c_master_restart();
    
    i2c_master_send(SLAVE_ADDR_READ);
    //i2c_master_send(GPIO);
    char v;
    int ind = 0;
      
    for (ind = 0; ind-1 < length; ind++) {
        v &= 0x00;
        v = i2c_master_recv();
        imuData[ind] = v;
        i2c_master_ack(0); // want more characters
        //LATBbits.LATB9 = 0;
          
    }
    v &= 0x00;
    v = i2c_master_recv();
    imuData[length-1] = v;
    i2c_master_ack(1); // want no more characters
    i2c_master_stop();
}

void setupHardware() {
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    
    I2C2BRG = 500;//90;// = ((1/(2*Fsck) - T_PGD)F_pb) - 2 where T_PGD = 104ns
    
    I2C2CONbits.ON = 1; // enable I2C 1
    
    SPI1_init();
    LCD_init();
}

void writeToRegister(char reg, char byteval) {
    i2c_master_start();                     // Begin the start sequence
    i2c_master_send(SLAVE_ADDR_WRITE);// << 1);       // send the slave address, left shifted by 1, 
                                        // which clears bit 0, indicating a write
    //LATBbits.LATB8 = 1;
    i2c_master_send(reg);         // send a byte to the slave       
    //LATBbits.LATB8 = 1;
    i2c_master_send(byteval);         // send another byte to the slave
    i2c_master_stop();
}

#define ILI9341_RED         0xF800

void writeBuffer(char *cbuf, unsigned short startX, unsigned short startY, unsigned int len) {
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

void writeYBar(signed short yval) {
    unsigned char i = 0, maxval = 50;
    short minChange = 700, startX = 150, startY = 250;
    short sign = 1;
    ///////////////////////////////// round yval / 700 to nearest int
    while (i < abs(yval/700)) {
        if (yval < 0) {
            sign = -1;
        } else {
            sign = 1;
        }
        writeChar(startX, startY + sign * i, '_');
        i++;
    }
}

void writeXBar(signed short xval) {
    unsigned char i = 0, maxval = 50;
    short minChange = 700, startX = 150, startY = 250;
    short sign = 1;
    ////////////////////////////////// round xval / 700 to nearest int
    while (i < abs(xval/700)) {
        if (xval < 0) {
            sign = -1;
        } else {
            sign = 1;
        }
        writeChar(startX-2+sign*i, startY, '|');
        i++;
    }
}

void clearYBar() {
    unsigned char i = 0, maxval = 50;
    short minChange = 700, startX = 150, startY = 250;
    //short sign = 1;
    ///////////////////////////////// round yval / 700 to nearest int
    clearBarVert(startX, startY-50, 20);
}

void clearXBar() {
    unsigned char i = 0, maxval = 50;
    short minChange = 700, startX = 150, startY = 250;
    //short sign = 1;
    ////////////////////////////////// round xval / 700 to nearest int
    clearBar(0, startY, 200);
}

void clearBar(unsigned short startX, unsigned short startY, unsigned short len) {
    unsigned char i = 0;
    unsigned short diffX = 5;
    while (i < len) {
        writeCharClear(startX+diffX*i, startY);
        i++;
    }
}

void clearBarVert(unsigned short startX, unsigned short startY, unsigned short len) {
    unsigned char i = 0;
    unsigned short diffX = 5;
    while (i < len) {
        writeCharClear(startX, startY+diffX*i);
        i++;
    }
}

int main(void) {
  
  __builtin_disable_interrupts();

  setupHardware();
  // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
  __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
  
  INTCONbits.MVEC = 0x1;
  TRISBbits.TRISB9 = 0;
  LATBbits.LATB9 = 0;

  // disable JTAG to get pins back
  DDPCONbits.JTAGEN = 0;
  


  LCD_clearScreen(0x0000);
  
  writeToRegister(CTRL1_XL, 0b10000010); // 1.66 KHz, 2g, 100 Hz; page 46 of datasheets
  writeToRegister(CTRL2_G, 0b10001000); // 1.66 KHz, 1000 dps; page 48 of datasheets
  writeToRegister(CTRL3_C, 0b00000100); // set IF_INC to 1 to auto increment getister addresses, page 49 of datasheets
  __builtin_enable_interrupts();
  
  //char index = 0;
  char len = 16;
  char cbufProgress[len];
  char cbufHello[len];
  char cbufFPS[len];
  sprintf(cbufHello, "Hello world %d!", 0);
  char imuData[ALL_BYTES];
  char cbufDataTemp[40];
  char cbufDataGyro[40];
  char cbufDataAccel[40];
  char xdatabuf[40];
  char ydatabuf[40];
  char zdatabuf[40];
  
  unsigned short startX = 20, startY = 20, diffX = 6, newX = 0, newY = 0;
  unsigned short test = 50;
  unsigned short perc = 0;
  unsigned char toUse;
  signed short temp, gx, gy, gz, ax, ay, az, lastnewx = 0, lastnewy = 0;
  unsigned int x, y, z;
  int incrementer;
  
  writeProgressBar(40, 10, 120);
  writeProgressBar(40, 10, 126);
  writeProgressBar(40, 10, 132);
  writeProgressBar(40, 10, 138);
  writeProgressBar(40, 10, 144);
  writeProgressBar(40, 10, 150);
  
  writeProgressBar(40, 10, 160);
  writeProgressBar(40, 10, 166);
  writeProgressBar(40, 10, 172);
  writeProgressBar(40, 10, 178);
  writeProgressBar(40, 10, 184);
  writeProgressBar(40, 10, 190);
  
//  writeBuffer(cbufHello, startX, startY, 16);
//  writeProgressBar(perc, startX, startY);
//  
//  sprintf(cbufDataAccel, "Acc x,y: %hi, %hi!             ", ax, ay);
//  writeBuffer(cbufDataAccel, startX, startY+30, 40);
//  sprintf(cbufFPS, "FPS: %d        ", 0);
//  writeBuffer(cbufFPS, startX, startY+20, 16);
  incrementer = 0;
  sprintf(xdatabuf, "Increment variable: %d   ", incrementer);
  writeBuffer(xdatabuf, startX, startY+10, 30);
  
  sprintf(xdatabuf, "Touch position x: %d   ", 0);
  writeBuffer(xdatabuf, startX, startY+50, 30);
  sprintf(ydatabuf, "Touch position y: %d    ", 0);
  writeBuffer(ydatabuf, startX, startY+60, 30);
  sprintf(zdatabuf, "Touch position z: %d    ", 0);
  writeBuffer(zdatabuf, startX, startY+70, 30);

  while(1) {
      if (LATBbits.LATB9) {
          LATBbits.LATB9 = 0;
      } else {
          LATBbits.LATB9 = 1;
      }
      
//      clearBar(startX+diffX*12, startY, 6);
//      sprintf(cbufHello, "            %d!", perc);
//      _CP0_SET_COUNT(0);
//      writeBuffer(cbufHello, startX, startY, 16);
//      writeProgressBar(perc, startX, startY);
//      int start = _CP0_GET_COUNT();
//      _CP0_SET_COUNT(0);
//      while (_CP0_GET_COUNT() < 400000) {
//          ;
//      }
//      unsigned int fps = (48000000) / (start + _CP0_GET_COUNT());
//      
//      clearBar(startX+diffX*5, startY+20, 6);
//      sprintf(cbufFPS, "     %d        ", fps);
//      writeBuffer(cbufFPS, startX, startY+20, 16);
//      //LCD_clearScreen(0x0000);
//      perc++;
//      if (perc > 100) {
//          clearBar(startX, startY+10, 20);
//      }
//      perc %= 101;
//      
//      
//      I2C_read_multiple(SLAVE_ADDR_WRITE, ALL_REG, imuData, ALL_BYTES);
//      
//      ax = 2*((imuData[8] & 0x00FF)|((imuData[9] & 0x00FF)<<8)); // 2 * because of scaling
//      ay = 2*((imuData[10] & 0x00FF)|((imuData[11] & 0x00FF)<<8)); // 2 * because of scaling
//      
//      clearBar(startX+diffX*9, startY+30, 20);
//      sprintf(cbufDataAccel, "         %hi  %hi              ", ax, ay);
//      writeBuffer(cbufDataAccel, startX, startY+30, 40);
//      clearXBar();
//      writeXBar(ax);
//      clearYBar();
//      writeYBar(ay);
      
      x = 0;
      y = 0;
      z = 0;
      
      XPT2046_read(&x, &y, &z);
      
      unsigned short newx = (unsigned short) (((float) x) / 4096 * 240);
      unsigned short newy = (unsigned short) (((float) y) / 4096 * 320);
      
      clearBar(startX+diffX*18, startY+50, 20);
      clearBar(startX+diffX*18, startY+60, 20);
      clearBar(startX+diffX*18, startY+70, 20);
      
      sprintf(xdatabuf, "                  %d             ", newx);
      writeBuffer(xdatabuf, startX, startY+50, 30);
      sprintf(ydatabuf, "                  %d             ", newy);
      writeBuffer(ydatabuf, startX, startY+60, 30);
      sprintf(zdatabuf, "                  %d             ", z);
      writeBuffer(zdatabuf, startX, startY+70, 30);
      
      if ((lastnewx > 5) && (lastnewx < 66) && (lastnewy > 155) && (lastnewy < 200) && (x == 0) && (y == 0)) { //(y > 120) && (y < 156)) {
          incrementer++;
          //} else if ((y > 160) && (y < 196)) {
          //    incrementer--; 
          //}
        
      } else if ((lastnewx > 5) && (lastnewx < 66) && (lastnewy > 115) && (lastnewy < 155) && (x == 0) && (y == 0)) {
          incrementer--;
      }
      
      lastnewx = newx;
      lastnewy = newy;
      
      sprintf(xdatabuf, "                    %d                               ", incrementer);
      clearBar(startX+diffX*20, startY+10, 20);
      writeBuffer(xdatabuf, startX, startY+10, 30);    
      
      _CP0_SET_COUNT(0);
      while (_CP0_GET_COUNT() < 400000) {
          ;
      }
      // start 60 below the lowest written data above, then go 1 pixel bar up for each 700, down for each -700, left for each -700, right for each 700
  }
  return 0;
}