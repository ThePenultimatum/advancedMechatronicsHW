#include "ov7670.h"

void ov7670_setup(){
    // set C8 to OC2 at 12MHz using Timer2
    RPC8Rbits.RPC8R = 0b0101; // C8 is OC2
    T2CONbits.TCKPS = 0; // Timer2 prescaler N=1 (1:1)
    PR2 = 3; // PR = PBCLK / N / desiredF - 1
    TMR2 = 0; // initial TMR2 count is 0
    OC2CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC2RS = 1; // duty cycle = 50%
    OC2R = 1; // initialize before turning on; afterward it is read-only
    T2CONbits.ON = 1; // turn on Timer2
    OC2CONbits.ON = 1; // turn on OC1
    
    // set camera to QCIF format
    unsigned char val = 0;
    /*
    // set 0x0C to 0b00001000
    i2c_master_start();
    i2c_master_send(OV7670_SCCB_ADDRESS<<1);
    i2c_master_send(0x0C);
    i2c_master_send(0b00001000); // enable scaling
    i2c_master_stop();
    
    // set 0x12 to 0b1000
    i2c_master_start();
    i2c_master_send(OV7670_SCCB_ADDRESS<<1);
    i2c_master_send(0x12);
    i2c_master_send(0b0001000); // //set to qcif 176x144
    i2c_master_stop();
    
    
    //i2c_master_start();
    //i2c_master_send(OV7670_SCCB_ADDRESS<<1);
    //i2c_master_send(0x3E);
    //i2c_master_send(0b00011100); 
    //i2c_master_stop();
     */
    
    writeCameraRegister(0x12, 0x80); // reset all camera registers to default value
 _CP0_SET_COUNT(0);
 while(_CP0_GET_COUNT()<48000000/2){}
  writeCameraRegister(0x3A, 0x04);
  writeCameraRegister(0x12, 0x08); // was 00
  writeCameraRegister(0x17, 0x13);
  writeCameraRegister(0x18, 0x01);
  writeCameraRegister(0x32, 0xB6);
  writeCameraRegister(0x19, 0x02);
  writeCameraRegister(0x1A, 0x7A);
  writeCameraRegister(0x03, 0x0A);
  writeCameraRegister(0x0C, 0x00);
  writeCameraRegister(0x3E, 0x00);
  writeCameraRegister(0x70, 0x3A);
  writeCameraRegister(0x71, 0x35);
  writeCameraRegister(0x72, 0x11);
  writeCameraRegister(0x73, 0xF0);
  writeCameraRegister(0xA2, 0x01);
  writeCameraRegister(0x15, 0x00);
  writeCameraRegister(0x7A, 0x20);
  writeCameraRegister(0x7B, 0x10);
  writeCameraRegister(0x7C, 0x1E);
  writeCameraRegister(0x7D, 0x35);
  writeCameraRegister(0x7E, 0x5A);
  writeCameraRegister(0x7F, 0x69);
  writeCameraRegister(0x80, 0x76);
  writeCameraRegister(0x81, 0x80);
  writeCameraRegister(0x82, 0x88);
  writeCameraRegister(0x83, 0x8F);
  writeCameraRegister(0x84, 0x96);
  writeCameraRegister(0x85, 0xA3);
  writeCameraRegister(0x86, 0xAF);
  writeCameraRegister(0x87, 0xC4);
  writeCameraRegister(0x88, 0xD7);
  writeCameraRegister(0x89, 0xE8);
  writeCameraRegister(0x13, 0xC0);
  writeCameraRegister(0x00, 0x00);
  writeCameraRegister(0x10, 0x00);
  writeCameraRegister(0x0D, 0x40);
  writeCameraRegister(0x14, 0x18);
  writeCameraRegister(0xA5, 0x05);
  writeCameraRegister(0xAB, 0x07);
  writeCameraRegister(0x24, 0x95);
  writeCameraRegister(0x25, 0x33);
  writeCameraRegister(0x26, 0xE3);
  writeCameraRegister(0x9F, 0x78);
  writeCameraRegister(0xA0, 0x68);
  writeCameraRegister(0xA1, 0x03);
  writeCameraRegister(0xA6, 0xD8);
  writeCameraRegister(0xA7, 0xD8);
  writeCameraRegister(0xA8, 0xF0);
  writeCameraRegister(0xA9, 0x90);
  writeCameraRegister(0xAA, 0x94);
  writeCameraRegister(0x13, 0xC5);
  writeCameraRegister(0x30, 0x00);
  writeCameraRegister(0x31, 0x00);
  writeCameraRegister(0x0E, 0x61);
  writeCameraRegister(0x0F, 0x4B);
  writeCameraRegister(0x16, 0x02);
  writeCameraRegister(0x1E, 0x07);
  writeCameraRegister(0x21, 0x02);
  writeCameraRegister(0x22, 0x91);
  writeCameraRegister(0x29, 0x07);
  writeCameraRegister(0x33, 0x0B);
  writeCameraRegister(0x35, 0x0B);
  writeCameraRegister(0x37, 0x1D);
  writeCameraRegister(0x38, 0x71);
  writeCameraRegister(0x39, 0x2A);
  writeCameraRegister(0x3C, 0x78);
  writeCameraRegister(0x4D, 0x40);
  writeCameraRegister(0x4E, 0x20);
  writeCameraRegister(0x69, 0x00);
  writeCameraRegister(0x74, 0x10);
  writeCameraRegister(0x8D, 0x4F);
  writeCameraRegister(0x8E, 0x00);
  writeCameraRegister(0x8F, 0x00);
  writeCameraRegister(0x90, 0x00);
  writeCameraRegister(0x91, 0x00);
  writeCameraRegister(0x96, 0x00);
  writeCameraRegister(0x9A, 0x00);
  writeCameraRegister(0xB0, 0x84);
  writeCameraRegister(0xB1, 0x0C);
  writeCameraRegister(0xB2, 0x0E);
  writeCameraRegister(0xB3, 0x82);
  writeCameraRegister(0xB8, 0x0A);
  writeCameraRegister(0x43, 0x0A);
  writeCameraRegister(0x44, 0xF0);
  writeCameraRegister(0x45, 0x34);
  writeCameraRegister(0x46, 0x58);
  writeCameraRegister(0x47, 0x28);
  writeCameraRegister(0x48, 0x3A);
  writeCameraRegister(0x59, 0x88);
  writeCameraRegister(0x5A, 0x88);
  writeCameraRegister(0x5B, 0x44);
  writeCameraRegister(0x5C, 0x67);
  writeCameraRegister(0x5D, 0x49);
  writeCameraRegister(0x5E, 0x0E);
  writeCameraRegister(0x6C, 0x0A);
  writeCameraRegister(0x6D, 0x55);
  writeCameraRegister(0x6E, 0x11);
  writeCameraRegister(0x6F, 0x9E);
  writeCameraRegister(0x6A, 0x40);
  writeCameraRegister(0x01, 0x40);
  writeCameraRegister(0x02, 0x60);
  writeCameraRegister(0x13, 0xC7);
  writeCameraRegister(0x4F, 0x80);
  writeCameraRegister(0x50, 0x80);
  writeCameraRegister(0x51, 0x00);
  writeCameraRegister(0x52, 0x22);
  writeCameraRegister(0x53, 0x5E);
  writeCameraRegister(0x54, 0x80);
  writeCameraRegister(0x58, 0x9E);
  writeCameraRegister(0x41, 0x08);
  writeCameraRegister(0x3F, 0x00);
  writeCameraRegister(0x75, 0x05);
  writeCameraRegister(0x76, 0xE1);
  writeCameraRegister(0x4C, 0x00);
  writeCameraRegister(0x77, 0x01);
  writeCameraRegister(0x3D, 0x48);
  writeCameraRegister(0x4B, 0x09);
  writeCameraRegister(0xC9, 0x60);
  writeCameraRegister(0x56, 0x40);
  writeCameraRegister(0x34, 0x11);
  writeCameraRegister(0x3B, 0x12);
  writeCameraRegister(0xA4, 0x82);
  writeCameraRegister(0x96, 0x00);
  writeCameraRegister(0x97, 0x30);
  writeCameraRegister(0x98, 0x20);
  writeCameraRegister(0x99, 0x30);
  writeCameraRegister(0x9A, 0x84);
  writeCameraRegister(0x9B, 0x29);
  writeCameraRegister(0x9C, 0x03);
  writeCameraRegister(0x9D, 0x4C);
  writeCameraRegister(0x9E, 0x3F);
  writeCameraRegister(0x78, 0x04);
  writeCameraRegister(0x79, 0x01);
  writeCameraRegister(0xC8, 0xF0);
  writeCameraRegister(0x79, 0x0F);
  writeCameraRegister(0xC8, 0x00);
  writeCameraRegister(0x79, 0x10);
  writeCameraRegister(0xC8, 0x7E);
  writeCameraRegister(0x79, 0x0A);
  writeCameraRegister(0xC8, 0x80);
  writeCameraRegister(0x79, 0x0B);
  writeCameraRegister(0xC8, 0x01);
  writeCameraRegister(0x79, 0x0C);
  writeCameraRegister(0xC8, 0x0F);
  writeCameraRegister(0x79, 0x0D);
  writeCameraRegister(0xC8, 0x20);
  writeCameraRegister(0x79, 0x09);
  writeCameraRegister(0xC8, 0x80);
  writeCameraRegister(0x79, 0x02);
  writeCameraRegister(0xC8, 0xC0);
  writeCameraRegister(0x79, 0x03);
  writeCameraRegister(0xC8, 0x40);
  writeCameraRegister(0x79, 0x05);
  writeCameraRegister(0xC8, 0x30);
  writeCameraRegister(0x79, 0x26);
  writeCameraRegister(0xFF, 0xFF);
  writeCameraRegister(0x15, 0x20);
  writeCameraRegister(0x0C, 0x08); // was 04
  writeCameraRegister(0x3E, 0x19);
  writeCameraRegister(0x72, 0x11);
  writeCameraRegister(0x73, 0xF1);
  writeCameraRegister(0x17, 0x16);
  writeCameraRegister(0x18, 0x04);
  writeCameraRegister(0x32, 0xA4);
  writeCameraRegister(0x19, 0x02);
  writeCameraRegister(0x1A, 0x7A);
  writeCameraRegister(0x03, 0x0A);
  writeCameraRegister(0xFF, 0xFF);
  writeCameraRegister(0x12, 0x08); // was 00
  writeCameraRegister(0x8C, 0x00);
  writeCameraRegister(0x04, 0x00);
  writeCameraRegister(0x40, 0xC0);
  writeCameraRegister(0x14, 0x6A);
  writeCameraRegister(0x4F, 0x80);
  writeCameraRegister(0x50, 0x80);
  writeCameraRegister(0x51, 0x00);
  writeCameraRegister(0x52, 0x22);
  writeCameraRegister(0x53, 0x5E);
  writeCameraRegister(0x54, 0x80);
  writeCameraRegister(0x3D, 0x40);
  writeCameraRegister(0xFF, 0xFF);
  writeCameraRegister(0x11, 0b0010); // clock was 1F
  
  writeCameraRegister(0x3E, 0x19);
  writeCameraRegister(0x0C, 0x08);  // was one higher
  writeCameraRegister(0x73, 0xF1);
  writeCameraRegister(0x12, 0x08); // was 0x10

  _CP0_SET_COUNT(0);
 while(_CP0_GET_COUNT()<48000000/2){}

}

void writeCameraRegister(unsigned char reg, unsigned char val){
    i2c_master_start();
    i2c_master_send(OV7670_SCCB_ADDRESS<<1);
    i2c_master_send(reg);
    i2c_master_send(val);
    i2c_master_stop();
}

int ov7670_count_horz(unsigned char * d){  
    int pclk = 0;
    int old_pclk = 0;
    int new_pclk = 0;
    //A8 as INT3/PCLK, B13 as INT2/HREF, C9 as INT1/VSYNC
    
    while(PORTCbits.RC9 == 0){}
    while(PORTCbits.RC9 == 1){} // got  new image
        
    while(PORTBbits.RB13 == 0){}
    while(PORTBbits.RB13 == 1){}//1
    while(PORTBbits.RB13 == 0){}
    while(PORTBbits.RB13 == 1){}//2
    while(PORTBbits.RB13 == 0){}
    while(PORTBbits.RB13 == 1){}//3
        
    pclk = 0;
    while(PORTBbits.RB13 == 0){}
    while(PORTBbits.RB13 == 1){ // the 4th row
        new_pclk = PORTAbits.RA8;
        if(old_pclk == 1 & new_pclk == 0){
            d[pclk] = PORTC;
            pclk++;
        }
        old_pclk = new_pclk;
    }
        
    return pclk;
}

int ov7670_count_vert(unsigned char * d){
    int rowclk = 0;
    int pclk = 0;
    int rowcount = 0;
    int old_rowclk = 0;
    int new_rowclk = 0;
    //A8 as INT3/PCLK, B13 as INT2/HREF, C9 as INT1/VSYNC
    
    while(PORTCbits.RC9 == 0){}
    while(PORTCbits.RC9 == 1){} // got a new image
    
    while(rowcount < 200){
        while(PORTBbits.RB13 == 0){}
        rowclk = 0;
        while(PORTBbits.RB13 == 1){ // wait for row to start
            new_rowclk = PORTAbits.RA8;
            if(old_rowclk == 1 & new_rowclk == 0){ // pixel clock changes
                rowclk++; // how many pixels you've seen
                if (rowclk == 100 | rowclk == 101) { // remember this pixel, brightness or color
                    pclk++;
                    d[pclk] = PORTC;
                }
            }
            old_rowclk = new_rowclk;
        }
        rowcount++;
    }
        
    return pclk;
}

