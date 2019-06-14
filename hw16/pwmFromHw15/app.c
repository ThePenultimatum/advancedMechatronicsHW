/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

#include<xc.h>           // processor SFR definitions
#include <math.h> 	//for sine wave plotting
#include <stdint.h>
#include <stdio.h>
#include<sys/attribs.h>  // __ISR macro

#include "i2c_master_noint.h"
#include "ili9341.h"

#include "filters.h"

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
#define FILTERBUFLEN 5

#define LENBUF0 16
#define LENBUF1 40
#define NUMFIRVALS 11

char cbufProgress[LENBUF0];
char cbufHello[LENBUF0];
char cbufFPS[LENBUF0];
char imuData[ALL_BYTES];
char cbufDataTemp[LENBUF1];
char cbufDataGyro[LENBUF1];
char cbufDataAccel[LENBUF1];
char xdatabuf[LENBUF1];
char ydatabuf[LENBUF1];
char zdatabuf[LENBUF1];
int movingAvgFilterBuf[FILTERBUFLEN];
int mafIndex = 0;
int initValMAF = 0;
int numMAFVals = FILTERBUFLEN;
int numFIRVals = NUMFIRVALS;
//initMAFBuffer(movingAvgFilterBuf, initValMAF, numMAFVals);

int movingAvgFilterFIRBuf[NUMFIRVALS];
int initValFIR = 0;
int firIndex = 0;

unsigned short startX = 20, startY = 20, diffX = 6, newX = 0, newY = 0;
unsigned short test = 50;
unsigned short perc = 0;
unsigned char toUse;
signed short temp, gx, gy, gz, ax, ay, az, mafAZ, iirAZ, firAZ, lastnewx = 0, lastnewy = 0;
unsigned int x, y, z;
int incrementer;
int shouldWriteNow = 0;
int numsWritten = 0;

float iirA = 0.9;
float iirB = 0.1;
int iirSoFar = 0;

unsigned char redpix[240];
unsigned char greenpix[240];
unsigned char bluepix[240];

volatile unsigned int oc1rsToSet = 0;
volatile unsigned int oc1rsDirectionToSet = 1;
volatile unsigned int oc2rsToSet = 0;
volatile unsigned int oc2rsDirectionToSet = 1;
volatile unsigned int pwmModulationVal = 0;
volatile unsigned int totalInterruptsPWM = 0;

float firArr[NUMFIRVALS]; // 10th order fir
//

uint8_t APP_MAKE_BUFFER_DMA_READY dataOut[APP_READ_BUFFER_SIZE];
uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
int len, i = 0;
int startTime = 0; // to remember the loop time

int incrementerVar = 0;

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

//unsigned short perc = 0;
unsigned char toUse;
signed short temp, gx, gy, gz, ax, ay, az;
/* TODO:  Add any necessary local functions.
*/

void __ISR(_TIMER_3_VECTOR, IPL5SOFT) Timer3ISR(void) {
    
    IFS0bits.T3IF = 0;
    
    // how many times has the interrupt occurred?
    totalInterruptsPWM++;
    pwmModulationVal = totalInterruptsPWM % (2400);
    
    if (oc1rsDirectionToSet) {
        oc1rsDirectionToSet = 0;
    } else {
        oc1rsDirectionToSet = 1;
    }
    
    if (oc2rsDirectionToSet) {
        oc2rsDirectionToSet = 0;
    } else {
        oc2rsDirectionToSet = 1;
    }
    
    /*if (pwmModulationVal == 0) {
        pwmModulationVal = 100;
    } else {
        pwmModulationVal = 0;
    }*/
    
    oc1rsToSet = pwmModulationVal;
    oc2rsToSet = pwmModulationVal;
    
    // set the duty cycle and direction pin
    OC1RS = oc1rsToSet;
    LATBbits.LATB9 = oc1rsDirectionToSet;
    OC2RS = oc2rsToSet;
    LATBbits.LATB13 = oc2rsDirectionToSet;
    
    

}

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
    
    I2C2BRG = 500;//500;//90;// = ((1/(2*Fsck) - T_PGD)F_pb) - 2 where T_PGD = 104ns
    
    I2C2CONbits.ON = 1; // enable I2C 1
    
    RPA0R = 0b0101; // set A0 to OC1
    RPB5R = 0b0101; // set B5 to OC2
    
    // TIMER 2
    T2CONbits.TCKPS = 0; // Timer2 prescaler N=1 (1:1)
    PR2 = 2399; // PR = PBCLK / N / desiredF - 1 ////////// 20KHz
    TMR2 = 0; // initial TMR2 count is 0
    // TIMER 3
    T3CONbits.TCKPS = 0b110; // Timer3 prescaler N=1 (1:64)
    PR3 = 7499; // PR = PBCLK / N / desiredF - 1 ////////// 100 Hz
    TMR3 = 0; // initial TMR2 count is 0
    // OC2 and OC3
    OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC1RS = 0; // duty cycle
    OC1R = 0; // initialize before turning OC1 on; afterward it is read-only
    OC1CONbits.OCTSEL = 0;
    OC2CONbits.OCM= 0b110;
    OC2RS = 0;
    OC2R = 0;
    OC2CONbits.OCTSEL = 0;
    // TURNING ON OC2 and TIMER2
    T2CONbits.ON = 1; // turn on Timer2
    OC1CONbits.ON = 1; // turn on OC1
    OC2CONbits.ON = 1; // turn on OC2
    T3CONbits.ON = 1;
    //
    
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


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    
    __builtin_disable_interrupts();

    setupHardware();
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
  
    INTCONbits.MVEC = 0x1;
    TRISBbits.TRISB9 = 0;////////////////////////////////////////////////////////////////////////////
    LATBbits.LATB9 = 0;//////////////////////////////////////////////////////////////////////////////
    TRISBbits.TRISB13 = 0;
    LATBbits.LATB13 = 0;
    //TRISAbits.TRISA0 = 0;
    //LATAbits.LATA0 = 0;
    //TRISBbits.TRISB5 = 0;
    //LATBbits.LATB5 = 0;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    
    // TIMER 3 Interrupt
    IPC3bits.T3IP = 5;
    IPC3bits.T3IS = 0;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    //
  


    LCD_clearScreen(0x0000);
  
    writeToRegister(CTRL1_XL, 0b10000010); // 1.66 KHz, 2g, 100 Hz; page 46 of datasheets
    writeToRegister(CTRL2_G, 0b10001000); // 1.66 KHz, 1000 dps; page 48 of datasheets
    writeToRegister(CTRL3_C, 0b00000100); // set IF_INC to 1 to auto increment getister addresses, page 49 of datasheets
    __builtin_enable_interrupts();
    
    char len = 16;
    char cbufProgress[len];
    char cbufHello[len];
    char cbufFPS[len];
    sprintf(cbufHello, "Hello world %d!", 0);
    char imuData[ALL_BYTES];
    char cbufDataTemp[40];
    char cbufDataGyro[40];
    char cbufDataAccel[40];
  
    unsigned short startX = 20, startY = 20, diffX = 6, newX = 0, newY = 0;
    unsigned short test = 50;
    unsigned short perc = 0;
    unsigned char toUse;
    signed short temp, gx, gy, gz, ax, ay, az;
    
    int iteratorInitPix = 0;
    for (iteratorInitPix = 0; iteratorInitPix < 240; iteratorInitPix++) {
        redpix[iteratorInitPix] = iteratorInitPix;
        greenpix[iteratorInitPix] = 240-iteratorInitPix;
        bluepix[iteratorInitPix] = iteratorInitPix % 50;
    }
    
    plotGraph(10, 10, 240, redpix);
    plotGraph(10, 70, 240, greenpix);
    plotGraph(10, 130, 240, bluepix);
  
    /*writeBuffer(cbufHello, startX, startY, 16);
    writeProgressBar(perc, startX, startY);
  
    sprintf(cbufDataAccel, "Acc x,y: %hi, %hi!             ", ax, ay);
    writeBuffer(cbufDataAccel, startX, startY+30, 40);
    sprintf(cbufFPS, "FPS: %d        ", 0);
    writeBuffer(cbufFPS, startX, startY+20, 16);*/
    
    /*
     * HW 1 here
    appData.state = APP_STATE_INIT;

     __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    TRISAbits.TRISA4 = 0;
    //TRISBbits.TRISB4 = 1;
    //TRISBbits.TRISB8 = 0;
    LATAbits.LATA4 = 1; // LED pin high
     * 
     * */

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            
            char len = 16;
            char cbufProgress[len];
            char cbufHello[len];
            char cbufFPS[len];
            char imuData[ALL_BYTES];
            char cbufDataTemp[40];
            char cbufDataGyro[40];
            char cbufDataAccel[40];
  
            unsigned short startX = 20, startY = 20, diffX = 6, newX = 0, newY = 0;
            unsigned short test = 50;
            //unsigned short perc = 0;
            //unsigned char toUse;
            //signed short temp, gx, gy, gz, ax, ay, az;
            
            /*if (LATBbits.LATB9) {
                LATBbits.LATB9 = 0;
            } else {
                LATBbits.LATB9 = 1;
            }*/
      
            /*clearBar(startX+diffX*12, startY, 6);
            sprintf(cbufHello, "            %d!", perc);
            _CP0_SET_COUNT(0);
            writeBuffer(cbufHello, startX, startY, 16);
            writeProgressBar(perc, startX, startY);
            int start = _CP0_GET_COUNT();
            _CP0_SET_COUNT(0);
            while (_CP0_GET_COUNT() < 400000) {
                ;
            }
            unsigned int fps = (48000000) / (start + _CP0_GET_COUNT());
      
            clearBar(startX+diffX*5, startY+20, 6);
            sprintf(cbufFPS, "     %d        ", fps);
            writeBuffer(cbufFPS, startX, startY+20, 16);
            //LCD_clearScreen(0x0000);
            perc++;
            if (perc > 100) {
                clearBar(startX, startY+10, 20);
            }
            perc %= 101;*/
      
      
            I2C_read_multiple(SLAVE_ADDR_WRITE, ALL_REG, imuData, ALL_BYTES);
      
            ax = 2*((imuData[8] & 0x00FF)|((imuData[9] & 0x00FF)<<8)); // 2 * because of scaling
            ay = 2*((imuData[10] & 0x00FF)|((imuData[11] & 0x00FF)<<8)); // 2 * because of scaling
      
            /*clearBar(startX+diffX*9, startY+30, 20);
            sprintf(cbufDataAccel, "         %d  %d              ", ax, ay);
            writeBuffer(cbufDataAccel, startX, startY+30, 40);
            clearXBar();
            writeXBar(ax);
            clearYBar();
            writeYBar(ay);*/
            // start 60 below the lowest written data above, then go 1 pixel bar up for each 700, down for each -700, left for each -700, right for each 700
            
            /*
             * HW 1
            LATAbits.LATA4 = 0;
            _CP0_SET_COUNT(0);
            while(_CP0_GET_COUNT() < 12000) {
               ;
            }
            while (!PORTBbits.RB4) {
                ;    
            }
            LATAbits.LATA4 = 1;
            _CP0_SET_COUNT(0);
            while(_CP0_GET_COUNT() < 12000) {
                ;
            }
            while (!PORTBbits.RB4) {
                ;     
            }
            */

        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */