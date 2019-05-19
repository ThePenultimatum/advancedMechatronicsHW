
#include <xc.h>
#include "ili9341.h"

void LCD_init() {
    int time = 0;
    
    CS = 0; // CS
   
    LCD_command(ILI9341_SWRESET);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 7200000) {} // 300ms

    LCD_command(0xEF);
  	LCD_data(0x03);
	LCD_data(0x80);
	LCD_data(0x02);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xCF);
  	LCD_data(0x00);
	LCD_data(0xC1);
	LCD_data(0x30);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xED);
  	LCD_data(0x64);
	LCD_data(0x03);
	LCD_data(0x12);
    LCD_data(0x81);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xE8);
  	LCD_data(0x85);
	LCD_data(0x00);
	LCD_data(0x78);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xCB);
  	LCD_data(0x39);
	LCD_data(0x2C);
	LCD_data(0x00);
    LCD_data(0x34);
    LCD_data(0x02);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xF7);
  	LCD_data(0x20);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xEA);
  	LCD_data(0x00);
	LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_PWCTR1);
  	LCD_data(0x23);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_PWCTR2);
  	LCD_data(0x10);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_VMCTR1 );
  	LCD_data(0x3e);
    LCD_data(0x28);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_VMCTR2);
  	LCD_data(0x86);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_MADCTL);
  	LCD_data(0x48);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
/*    
    LCD_command(ILI9341_VSCRSADD);
  	LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
 */   
    LCD_command(ILI9341_PIXFMT);
  	LCD_data(0x55);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_FRMCTR1);
  	LCD_data(0x00);
    LCD_data(0x18);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command( ILI9341_DFUNCTR);
  	LCD_data(0x08);
    LCD_data(0x82);
    LCD_data(0x27);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xF2);
  	LCD_data(0); // 1
    LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_GAMMASET);
  	LCD_data(0x01);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_GMCTRP1);
  	LCD_data(0x0F);
    LCD_data(0x31);
    LCD_data(0x2B);
    LCD_data(0x0C);
    LCD_data(0x0E);
    LCD_data(0x08);
    LCD_data(0x4E);
    LCD_data(0xF1);
    LCD_data(0x37);
    LCD_data(0x07);
    LCD_data(0x10);
    LCD_data(0x03);
    LCD_data(0x0E);
    LCD_data(0x09);
    LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_GMCTRN1);
  	LCD_data(0x00);
    LCD_data(0x0E);
    LCD_data(0x14);
    LCD_data(0x03);
    LCD_data(0x11);
    LCD_data(0x07);
    LCD_data(0x31);
    LCD_data(0xC1);
    LCD_data(0x48);
    LCD_data(0x08);
    LCD_data(0x0F);
    LCD_data(0x0C);
    LCD_data(0x31);
    LCD_data(0x36);
    LCD_data(0x0F);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xB1);
  	LCD_data(0x00);
    LCD_data(0x10);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_SLPOUT);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_DISPON);
    
    CS = 1; // CS
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    
    CS = 0; // CS
    
    LCD_command(ILI9341_MADCTL);
    LCD_data(MADCTL_MX | MADCTL_BGR); // rotation
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    CS = 1; // CS
}

void SPI1_init() {
  SDI1Rbits.SDI1R = 0b0100; // B8 is SDI1
  RPA1Rbits.RPA1R = 0b0011; // A1 is SDO1
  TRISBbits.TRISB7 = 0; // CS is B7
  CS = 1; // CS starts high
  TRISBbits.TRISB10 = 0; // CSTOUCH is B
  CSTOUCH = 1; // CSTOUCH starts high

  // DC pin
  TRISBbits.TRISB15 = 0;
  DC = 1;
  
  SPI1CON = 0; // turn off the spi module and reset it
  SPI1BUF; // clear the rx buffer by reading from it
  SPI1BRG = 3; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0; // clear the overflow bit
  SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1; // master operation
  SPI1CONbits.ON = 1; // turn on spi1
}

unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void LCD_command(unsigned char com) {
    DC = 0; // DC
    spi_io(com);
    DC = 1; // DC
}

void LCD_data(unsigned char dat) {
    spi_io(dat);
}

void LCD_data16(unsigned short dat) {
    spi_io(dat>>8);
    spi_io(dat);
}

void LCD_setAddr(unsigned short x, unsigned short y, unsigned short w, unsigned short h) {
    LCD_command(ILI9341_CASET); // Column
    LCD_data16(x);
	LCD_data16(x+w-1);

	LCD_command(ILI9341_PASET); // Page
	LCD_data16(y);
	LCD_data16(y+h-1);

	LCD_command(ILI9341_RAMWR); // Into RAM
}

void LCD_drawPixel(unsigned short x, unsigned short y, unsigned short color) {
  // check boundary
    
    CS = 0; // CS
    
    LCD_setAddr(x,y,1,1);
    LCD_data16(color);
    
    CS = 1; // CS
}

void LCD_clearScreen(unsigned short color) {
    int i;
    
    CS = 0; // CS
    
    LCD_setAddr(0,0,ILI9341_TFTWIDTH,ILI9341_TFTHEIGHT);
	for (i = 0;i < ILI9341_TFTWIDTH*ILI9341_TFTHEIGHT; i++){
		LCD_data16(color);
	}
    
    CS = 1; // CS
}

void writeChar(unsigned short xTLCorner, unsigned short yTLCorner, unsigned char c) {
    unsigned short index = c - 0x20;
    if ((index < 0) | (index > 95) | ((xTLCorner + 5) >  239) | ((yTLCorner + 16) >  319)) {
        return;
    }
    char *cval;//[1][5]; 
    cval = &(ASCII[index]);
    unsigned char col, i = 0, j = 0, k = 0;
    for (i = 0; i < 5; i++) {
        col = *(cval + i);//0x5F;//(*cval) + i;//cval[0][i];
        for (j = 0; j < 8; j++) {
            if (col & (0x01 << j)) {
                LCD_drawPixel(xTLCorner + i, yTLCorner + j, ILI9341_RED);
            }
        }
    }
}

void writeCharClear (unsigned short xTLCorner, unsigned short yTLCorner) {
    //unsigned short index = 0x7C - 0x20;
    //char *cval;//[1][5]; 
    //cval = &(ASCII[index]);
    char ctest[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    unsigned char col, i = 0, j = 0, k = 0;
    for (i = 0; i < 5; i++) {
        col = *(ctest + i);//0x5F;//(*cval) + i;//cval[0][i];
        for (j = 0; j < 8; j++) {
            if (col & (0x01 << j)) {
                LCD_drawPixel(xTLCorner + i, yTLCorner + j, ILI9341_BLACK);
            }
        }
    }
}

unsigned int getZ(int z1, int z2) {
    return (z1 - z2) + 4095;
}

unsigned int getTouchscreenResultFromPieces(unsigned int val1, unsigned int val2) {
    return (((val1 << 8) | val2) >> 3) & 0x0FFF;
}

unsigned int touchscreenCommand(unsigned char address) {
    spi_io(address);
    unsigned int val1 = spi_io(0x00);
    unsigned int val2 = spi_io(0x00);
    return getTouchscreenResultFromPieces(val1, val2);
}

void XPT2046_read(unsigned short *x, unsigned short *y, unsigned int *z) {
    CSTOUCH = 0;
    
    int x1, x2, y1, y2, z11, z12, z1, z21, z22, z2;
    
    z1 = touchscreenCommand(Z1_ADDR_READ_TOUCH);
    //z11 = touchscreenCommand(0x00);
    //z12 = touchscreenCommand(0x00);
    //z1 = (((z11 << 8) | z12) >> 3) & 0x0FFF;
    
    //touchscreenCommand(Z2_ADDR_READ_TOUCH);
    //z21 = touchscreenCommand(0x00);
    //z22 = touchscreenCommand(0x00);
    z2 = touchscreenCommand(Z2_ADDR_READ_TOUCH);
    //z21 = touchscreenCommand(0x00);
    //z22 = touchscreenCommand(0x00);
    //z2 = (((z21 << 8) | z22) >> 3) & 0x0FFF;
    
    *x = 0;
    *y = 0;
    *z = getZ(z1, z2);
    
    if (*z > 50) {
            
        //touchscreenCommand(X_ADDR_READ_TOUCH);
        //x1 = touchscreenCommand(0x00);
        //x2 = touchscreenCommand(0x00);
        *x = touchscreenCommand(X_ADDR_READ_TOUCH);
        //x1 = touchscreenCommand(0x00);
        //x2 = touchscreenCommand(0x00);
        //*x = (((0b01111101 << 8) | 0b0000000000000000) >> 3) & 0x0FFF;

        //touchscreenCommand(Y_ADDR_READ_TOUCH);
        //y1 = touchscreenCommand(0x00);
        //y2 = touchscreenCommand(0x00);
        *y = touchscreenCommand(Y_ADDR_READ_TOUCH);
        //y1 = touchscreenCommand(0x00);
        //y2 = touchscreenCommand(0x00);
        //*y = (((y1 << 8) | y2) >> 3) & 0x0FFF;

        //touchscreenCommand(Z1_ADDR_READ_TOUCH);
        //z11 = touchscreenCommand(0x00);
        //z12 = touchscreenCommand(0x00);
    }
    
    CSTOUCH = 1;
}