#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <stdio.h>

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = ON // allow multiple reconfigurations
#pragma config IOL1WAY = ON // allow multiple reconfigurations
#pragma config FUSBIDIO = OFF // USB pins controlled by USB module
#pragma config FVBUSONIO = OFF // USB BUSON controlled by USB module


#include "ili9341.h"
#include "i2c_master_noint.h"
#include "ov7670.h"

#define DIR1 LATAbits.LATA10
#define DIR2 LATAbits.LATA7
#define USER PORTBbits.RB4



void startup() {
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // pin definitions
    ANSELA = 0;
    ANSELB = 0;
    TRISAbits.TRISA7 = 0; // DIR2
    DIR2 = 1;
    TRISAbits.TRISA10 = 0; // DIR1
    DIR1 = 1;
    TRISBbits.TRISB4 = 1; // USER
    
    // OC1 is B15, goes with DIR1
    RPB15Rbits.RPB15R = 0b0101;
    
    // OC4 is A4, goes with DIR2
    RPA4Rbits.RPA4R = 0b0101;
    
    // use Timer 3 for PWM
    T3CONbits.TCKPS = 0; // Timer prescaler N=1 (1:1)
    PR3 = 2399; // PR = PBCLK / N / desiredF - 1
    TMR3 = 0; // initial TMR count is 0
    OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC4CONbits.OCM = 0b110;
    OC1CONbits.OCTSEL = 1; // User Timer3
    OC4CONbits.OCTSEL = 1;
    OC1RS = 0; // duty cycle
    OC1R = 0; // initialize before turning OC1 on; afterward it is read-only
    OC4RS = 0;
    OC4R = 0;
    T3CONbits.ON = 1; // turn on Timer
    OC1CONbits.ON = 1; // turn on OC1
    OC4CONbits.ON = 1;
    
    // LCD uses SPI1: A0 is SDO, A1 is SDI, B5 is CST, B14 is SCK1, A9 is DC, B7 is CS
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_DARKGREEN);
    
    // Camera uses C0-7, C8 as OC2, A8 as INT3/PCLK, B13 as INT2/HREF, C9 as INT1/VSYNC, and I2C1
    i2c_master_setup();
    ov7670_setup();
    
    // B3 is available as SCL2, B2 is available as SDA2
}

int main() {

    __builtin_disable_interrupts();

    startup();

    __builtin_enable_interrupts();
    
    int I = 0;
    char message[100];
    unsigned char d[2000]; // this is the raw camera data, both brightness and color
    unsigned char bright[1000]; // this is for just the brightness data
    
    while(1) {

        I++;
        sprintf(message,"I = %d   ", I);
        drawString(140,82,message);
        
        // horizontal read
        /*
        int c = ov7670_count_horz(d);
        sprintf(message, "c = %d   ",c);
        drawString(140,92,message); // there are 290 rows
        
        int x = 0, x2 = 1;
        int y = 0;
        int dim = 0;
        for(x = 0; x < c/2; x++, x2=x2+2){
            dim = d[x2]>>3;
            bright[x] = d[x2];
            for(y=0;y<32;y++){
                if (y == dim){
                    LCD_drawPixel(y+30,x,ILI9341_BLACK);
                }
                else {
                    LCD_drawPixel(y+30,x,ILI9341_WHITE);
                }
            }
        }
        */
        
        // vertical read
        int c = ov7670_count_vert(d);
        sprintf(message, "c = %d   ",c);
        drawString(140,92,message);
        
        int x = 0, x2 = 0;
        int y = 0;
        int dim = 0;
        for(x = 0; x < c/2; x++, x2=x2+2){
            dim = d[x2]>>3;
            bright[x] = d[x2];
            for(y=0;y<32;y++){
                if (y == dim){
                    LCD_drawPixel(x,y+30,ILI9341_BLACK);
                }
                else {
                    LCD_drawPixel(x,y+30,ILI9341_WHITE);
                }
            }
        }
        
        // at this point, bright has c/2 values in it, each value is a brightness of a pixel
        // loop through bright and calculate where the middle of the dip or bump is
        // then set the motor speed and direction to follow the line
        int i = 0;
        int sum = 0;
        int sumR = 0;
        int com = 0;
        int avg = 0;
        // find the average brightness
        for (i=0;i<c/2;i++){
            sum = sum + bright[i];
        }
        avg = sum / c/2;
        // threshold and center of mass calculation
        sum = 0;
        for (i=0;i<c/2;i++){
            if (bright[i]<avg){
                // count this pixel
                LCD_drawPixel(i,30,ILI9341_BLUE); // visualize which pixels we're counting
                sum = sum + 255;
                sumR = sumR + 255*i;
            }
            else {
                LCD_drawPixel(i,30,ILI9341_WHITE);
                // don't count this pixel
            }
        }
        // only use com if the camera saw some data
        if (sum>10){
            com = sumR/sum;
        }
        else {
            com = c/2/2;
        }
        // draw the center of mass as a bar
        for(y=0;y<32;y++){
            LCD_drawPixel(com,y+30,ILI9341_RED);
        }
        int speed = 0;
        int e = 0;
        // try to keep com at c/2/2 using the motors
        DIR1 = 1; // depending on your motor directions these might be different
        DIR2 = 1;
        // if com is less than c/2/2, then slow down the left motor, full speed right motor
        // if com is more than c/2/2, then slow down right motor, full speed left motor
        // things to play with: the slope of the line, the value that determines when the motor is not full speed
        if (com < c/2/2){
            e = c/2/2 - com;
            speed = 2399 - (2399/c/2/2)*e; // when the com is all the way over, the motor is all off
            if(speed > 2399){
                speed = 2399;
            }
            if(speed < 0){
                speed = 0;
            }
            OC1RS = 2399;
            OC4RS = speed;
        }
        else {
            e = com - c/2/2;
            speed = 2399 - (2399/c/2/2)*e; // when the com is all the way over, the motor is all off
            if(speed > 2399){
                speed = 2399;
            }
            if(speed < 0){
                speed = 0;
            }
            OC1RS = speed;
            OC4RS = 2399;
        }

    }
}
