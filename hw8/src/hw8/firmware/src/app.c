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

unsigned short perc = 0;
unsigned char toUse;
signed short temp, gx, gy, gz, ax, ay, az;
/* TODO:  Add any necessary local functions.
*/


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
    TRISBbits.TRISB9 = 0;
    LATBbits.LATB9 = 0;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
  


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
  
    writeBuffer(cbufHello, startX, startY, 16);
    writeProgressBar(perc, startX, startY);
  
    sprintf(cbufDataAccel, "Acc x,y: %hi, %hi!             ", ax, ay);
    writeBuffer(cbufDataAccel, startX, startY+30, 40);
    sprintf(cbufFPS, "FPS: %d        ", 0);
    writeBuffer(cbufFPS, startX, startY+20, 16);
    
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
            
            if (LATBbits.LATB9) {
                LATBbits.LATB9 = 0;
            } else {
                LATBbits.LATB9 = 1;
            }
      
            clearBar(startX+diffX*12, startY, 6);
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
            perc %= 101;
      
      
            I2C_read_multiple(SLAVE_ADDR_WRITE, ALL_REG, imuData, ALL_BYTES);
      
            ax = 2*((imuData[8] & 0x00FF)|((imuData[9] & 0x00FF)<<8)); // 2 * because of scaling
            ay = 2*((imuData[10] & 0x00FF)|((imuData[11] & 0x00FF)<<8)); // 2 * because of scaling
      
            clearBar(startX+diffX*9, startY+30, 20);
            sprintf(cbufDataAccel, "         %d  %d              ", ax, ay);
            writeBuffer(cbufDataAccel, startX, startY+30, 40);
            clearXBar();
            writeXBar(ax);
            clearYBar();
            writeYBar(ay);
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
