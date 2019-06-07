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

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

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


uint8_t APP_MAKE_BUFFER_DMA_READY dataOut[APP_READ_BUFFER_SIZE];
uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
int len, i = 0;
int startTime = 0; // to remember the loop time



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

/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
        USB_DEVICE_CDC_INDEX index,
        USB_DEVICE_CDC_EVENT event,
        void * pData,
        uintptr_t userData
        ) {
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *) userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch (event) {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *) pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *) pData)->breakDuration;

            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch (event) {
        case USB_DEVICE_EVENT_SOF:

            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuratio. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*) eventData;
            if (configuredEventData->configurationValue == 1) {
                /* Update LED to show configured state */

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t) & appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

//unsigned short perc = 0;
//unsigned char toUse;
//signed short temp, gx, gy, gz, ax, ay, az;


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

float firArr[NUMFIRVALS]; // 10th order fir
// 


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

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void) {
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if (appData.isConfigured == false) {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    } else {
        retVal = false;
    }

    return (retVal);
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

void APP_Initialize(void) {
        
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Reset other flags */
    appData.sofEventHasOccurred = false;
    //appData.isSwitchPressed = false;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];

    /* PUT YOUR LCD, IMU, AND PIN INITIALIZATIONS HERE */
    
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

    startTime = _CP0_GET_COUNT();
    
    //char lenBuf = 16;
    //char cbufProgress[lenBuf];
    //char cbufHello[lenBuf];
    //char cbufFPS[lenBuf];
    //char imuData[ALL_BYTES];
    //char cbufDataTemp[40];
    //char cbufDataGyro[40];
    //char cbufDataAccel[40];
    //char xdatabuf[40];
    //char ydatabuf[40];
    //char zdatabuf[40];
    
    int mafInitIterVal = 0;
    for (mafInitIterVal = 0; mafInitIterVal < numMAFVals; mafInitIterVal++) {
        movingAvgFilterBuf[mafInitIterVal] = initValMAF;
    }
    
    int firInitIterVal = 0;
    for (firInitIterVal = 0; firInitIterVal < numFIRVals; firInitIterVal++) {
        movingAvgFilterFIRBuf[firInitIterVal] = initValFIR;
    }
    firArr[0] = -0.0039;
    firArr[1] = 0.0000;
    firArr[2] = 0.0321;
    firArr[3] = 0.1167;
    firArr[4] = 0.2207;
    firArr[5] = 0.2687;
    firArr[6] = 0.2207;
    firArr[7] = 0.1167;
    firArr[8] = 0.0321;
    firArr[9] = 0.0000;
    firArr[10] = -0.0039;

    unsigned short startX = 20, startY = 20, diffX = 6, newX = 0, newY = 0;
    unsigned short test = 50;
    unsigned short perc = 0;
    unsigned char toUse;
    signed short temp, gx, gy, gz, ax, ay, az, mafAZ, iirAZ, firAZ, lastnewx = 0, lastnewy = 0;
    unsigned int x, y, z;
    int incrementer;
    
    
    sprintf(cbufHello, "Hello world %d!", 0);
    incrementer = 0;

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

    sprintf(xdatabuf, "Increment variable: %d   ", incrementer);
    writeBuffer(xdatabuf, startX, startY+10, 30);

    sprintf(xdatabuf, "Touch position x, rax_x: %d, %d  ", 0, 0);
    writeBuffer(xdatabuf, startX, startY+50, 30);
    sprintf(ydatabuf, "Touch position y, raw_y: %d, %d  ", 0, 0);
    writeBuffer(ydatabuf, startX, startY+60, 30);
    sprintf(zdatabuf, "Touch position z, raw_z: %d, %d  ", 0, 0);
    writeBuffer(zdatabuf, startX, startY+70, 30);
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    /* Update the application state machine based
     * on the current state */
    /*
    if (LATBbits.LATB9) {
        LATBbits.LATB9 = 0;
    } else {
        LATBbits.LATB9 = 1;
    }
    
    x = 0;
    y = 0;
    z = 0;
      
    XPT2046_read(&x, &y, &z);
    
    unsigned short newx = (unsigned short) (((float) x) / 4096 * 240);
    unsigned short newy = (unsigned short) (((float) y) / 4096 * 320);
      
    clearBar(startX+diffX*18, startY+50, 20);
    clearBar(startX+diffX*18, startY+60, 20);
    clearBar(startX+diffX*18, startY+70, 20);
      
    sprintf(xdatabuf, "                  %d, %d         ", newx, x);
    writeBuffer(xdatabuf, startX, startY+50, 30);
    sprintf(ydatabuf, "                  %d, %d         ", newy, y);
    writeBuffer(ydatabuf, startX, startY+60, 30);
    sprintf(zdatabuf, "                  %d, %d         ", z, z);
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
    
    I2C_read_multiple(SLAVE_ADDR_WRITE, ALL_REG, imuData, ALL_BYTES);
      
    ax = 2*((imuData[8] & 0x00FF)|((imuData[9] & 0x00FF)<<8)); // 2 * because of scaling
    ay = 2*((imuData[10] & 0x00FF)|((imuData[11] & 0x00FF)<<8)); // 2 * because of scaling
    */
    switch (appData.state) {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if (appData.isConfigured) {
                /* If the device is configured then lets start reading */
                appData.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:

            if (APP_StateReset()) {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

            appData.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
            if (appData.isReadComplete == true) {
                appData.isReadComplete = false;
                appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

                USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0,
                        &appData.readTransferHandle, appData.readBuffer,
                        APP_READ_BUFFER_SIZE);

                        /* AT THIS POINT, appData.readBuffer[0] CONTAINS A LETTER
                        THAT WAS SENT FROM THE COMPUTER */
                        /* YOU COULD PUT AN IF STATEMENT HERE TO DETERMINE WHICH LETTER
                        WAS RECEIVED (USUALLY IT IS THE NULL CHARACTER BECAUSE NOTHING WAS
                      TYPED) */
                
                if (appData.readBuffer[0] == 'r') {
                    shouldWriteNow = 1;
                    numsWritten = 0;
                }

                if (appData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID) {
                    appData.state = APP_STATE_ERROR;
                    break;
                }
            }

            break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:
        case APP_STATE_CHECK_TIMER:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was received or a switch was pressed.
             * The isReadComplete flag gets updated in the CDC event handler. */

             /* WAIT FOR 5HZ TO PASS OR UNTIL A LETTER IS RECEIVED */
            if (appData.isReadComplete || _CP0_GET_COUNT() - startTime > (48000000 / 2 / 100 )) { // Now 100 Hz after / 20
                appData.state = APP_STATE_SCHEDULE_WRITE;
            }


            break;


        case APP_STATE_SCHEDULE_WRITE:

            if (APP_StateReset()) {
                break;
            }

            /* Setup the write */
            
            appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData.isWriteComplete = false;
            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;

            /* PUT THE TEXT YOU WANT TO SEND TO THE COMPUTER IN dataOut
            AND REMEMBER THE NUMBER OF CHARACTERS IN len */
            /* THIS IS WHERE YOU CAN READ YOUR IMU, PRINT TO THE LCD, ETC */
            //sprintf(cbufDataAccel, "%hi %hi", ax, ay);
                        
            len = sprintf(dataOut,"%d", i);
            if (shouldWriteNow) {
                i++; // increment the index so we see a change in the text
                I2C_read_multiple(SLAVE_ADDR_WRITE, ALL_REG, imuData, ALL_BYTES);
      
                //ax = 2*((imuData[8] & 0x00FF)|((imuData[9] & 0x00FF)<<8)); // 2 * because of scaling
                //ay = 2*((imuData[10] & 0x00FF)|((imuData[11] & 0x00FF)<<8)); // 2 * because of scaling
                az = 2*((imuData[12] & 0x00FF)|((imuData[13] & 0x00FF)<<8)); // 2 * because of scaling
                
                // MAF
                replaceIndMAF(movingAvgFilterBuf, mafIndex, az);
                mafAZ = getAvgFilter(movingAvgFilterBuf, FILTERBUFLEN);
                mafIndex = getNextInd(mafIndex, FILTERBUFLEN);
                // IIR
                iirSoFar = iirA * iirSoFar + iirB * az;
                iirAZ = ceil(iirSoFar);
                // FIR
                replaceIndMAF(movingAvgFilterFIRBuf, firIndex, az);
                firAZ = ceil(getResFilterFIR(movingAvgFilterFIRBuf,firArr,numFIRVals));
                firIndex = getNextInd(firIndex, numFIRVals);
            
                //gx = 2*((imuData[2] & 0x00FF)|((imuData[3] & 0x00FF)<<8)); // 2 * because of scaling
                //gy = 2*((imuData[4] & 0x00FF)|((imuData[5] & 0x00FF)<<8)); // 2 * because of scaling
                //gz = 2*((imuData[6] & 0x00FF)|((imuData[7] & 0x00FF)<<8)); // 2 * because of scaling
                len = sprintf(dataOut,  "%d %d %d %d %d\r\n", i, az, mafAZ, iirAZ, firAZ);
                //ax, ay, az, gx, gy, gz);//"%d\r\n", i);
                numsWritten++;
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                    &appData.writeTransferHandle, dataOut, len,
                    USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                
                if (numsWritten >= 100) {
                    numsWritten = 0;
                    shouldWriteNow = 0;
                }
            } else {
                appData.isWriteComplete = true;
            }
            
            startTime = _CP0_GET_COUNT(); // reset the timer for acurate delays
            //}
            break;

        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if (appData.isWriteComplete == true) {
                //shouldWriteNow = 0;
                appData.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}



/*******************************************************************************
 End of File
 */