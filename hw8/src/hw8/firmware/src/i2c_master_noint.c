#include<xc.h>
// I2C Master utilities, 100 kHz, using polling rather than interrupts
// The functions must be callled in the correct order as per the I2C protocol
// I2C pins need pull-up resistors, 2k-10k

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

void i2c_master_setup(void) {
  I2C2BRG = 90;//233; // 233 for 100khz, 90 for 400khz            // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2 
                                    // look up PGD for your PIC32
  I2C2CONbits.ON = 1;               // turn on the I2C2 module
}

// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C2CONbits.SEN = 1;            // send the start bit
    while(I2C2CONbits.SEN) { ; }    // wait for the start bit to be sent
}

void i2c_master_restart(void) {     
    I2C2CONbits.RSEN = 1;           // send a restart 
    while(I2C2CONbits.RSEN) { ; }   // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
  I2C2TRN = byte;                   // if an address, bit 0 = 0 for write, 1 for read
  while(I2C2STATbits.TRSTAT) { ; }  // wait for the transmission to finish
  if(I2C2STATbits.ACKSTAT) {        // if this is high, slave has not acknowledged
      //LATBbits.LATB8 = 0;
      while (1) {;}
  }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C2CONbits.RCEN = 1;             // start receiving data
    while(!I2C2STATbits.RBF) { ; }    // wait to receive the data
    return I2C2RCV;                   // read and return the data
}

void i2c_master_ack(int val) {        // sends ACK = 0 (slave should send another byte)
                                      // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val;          // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;            // send ACKDT
    while(I2C2CONbits.ACKEN) { ; }    // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // send a STOP:
  I2C2CONbits.PEN = 1;                // comm is complete and master relinquishes bus
  while(I2C2CONbits.PEN) { ; }        // wait for STOP to complete
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
