
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"

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

#define SLAVE_ADDR 0b01000000//0x40
#define GPIO 0x09
#define IODIR 0x00

void setupHardware(void);
void writeToRegister(char, char);

void setupHardware() {
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    
    I2C2BRG = 90;// = ((1/(2*Fsck) - T_PGD)F_pb) - 2 where T_PGD = 104ns
    
    I2C2CONbits.ON = 1; // enable I2C 1
}

void writeToRegister(char reg, char byteval) {
    i2c_master_start();                     // Begin the start sequence
    i2c_master_send(SLAVE_ADDR);// << 1);       // send the slave address, left shifted by 1, 
                                        // which clears bit 0, indicating a write
    //LATBbits.LATB8 = 1;
    i2c_master_send(reg);         // send a byte to the slave       
    //LATBbits.LATB8 = 1;
    i2c_master_send(byteval);         // send another byte to the slave
    i2c_master_stop();
}

int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    //TRISAbits.TRISA4 = 0;
    //TRISBbits.TRISB4 = 0;
    //TRISBbits.TRISB3 = 0;
    //TRISBbits.TRISB2 = 0;
    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB7 = 0;
    //LATAbits.LATA4 = 1; // LED pin high
    
    setupHardware();
    
    LATBbits.LATB8 = 0;
    LATBbits.LATB7 = 0;
    //LATBbits.LATB7 = 1;
    
    writeToRegister(IODIR, 0xF0);
    writeToRegister(GPIO, 0x00);//0b00000011);
    
    __builtin_enable_interrupts();
    
    /////////////////////////
    while(1) {
        /*
        i2c_master_start();
        i2c_master_send(SLAVE_ADDR);
        i2c_master_send(GPIO);
        i2c_master_send(0b00000011);
        i2c_master_stop();
        */
        
        //LATBbits.LATB7 = 1;
        //LATBbits.LATB8 = 1;
        
        //writeToRegister(GPIO, 0x03);
        i2c_master_start();
        i2c_master_send(SLAVE_ADDR);
        i2c_master_send(GPIO);
        //i2c_master_send(0b00000011);
        //i2c_master_stop();
        
        i2c_master_restart();
        
        i2c_master_send(SLAVE_ADDR | 0x01);
        //i2c_master_send(GPIO);
        
        char v = i2c_master_recv();
        i2c_master_ack(1);
        
        if (!(v & 0b10000000)) {
            LATBbits.LATB8 = 1;
            LATBbits.LATB7 = 1;
            //LATBbits.LATB7 = 0;
            i2c_master_stop();
            int i = 0;
            while (i < 10000) {
                i++;
                ;
            }
            writeToRegister(GPIO, 0x03);
        } else {
            i2c_master_stop();
            LATBbits.LATB8 = 0;
            LATBbits.LATB7 = 0;
            writeToRegister(GPIO, 0x00);
        }
        //i2c_master_stop();
         
        /*i2c_master_start();                   // send a RESTART so we can begin reading 
        i2c_master_send(SLAVE_ADDR);// << 1) | 0); // send slave address, left shifted by 1,
    
        i2c_master_send(GPIO);         // send a byte to the slave       
        i2c_master_send(0b00000000);
        i2c_master_stop(); */
        /*
        i2c_master_start();
        
        i2c_master_send(SLAVE_ADDR) | 0x01);
        i2c_master_send(GPIO);
       
        char v = i2c_master_recv();
        i2c_master_ack(1);//0);
        //v = i2c_master_recv();
        //i2c_master_ack(1);
        i2c_master_stop();
        */
        /*i2c_master_send(0x01);
        i2c_master_stop();*/
        
        //i2c_master_restart();
        
        /*
        i2c_master_send(SLAVE_ADDR | 0x01);
       
        char v = i2c_master_recv();
        i2c_master_ack(1);//0);
        //v = i2c_master_recv();
        //i2c_master_ack(1);
        //i2c_master_stop();
        
        if (v & 0b00000010) { //10000000) {
            i2c_master_restart();//i2c_master_start();                   // send a RESTART so we can begin reading 
            i2c_master_send(SLAVE_ADDR);// << 1) | 0); // send slave address, left shifted by 1,
    
            i2c_master_send(GPIO);         // send a byte to the slave       
            i2c_master_send(0x01);
            i2c_master_stop(); 
        } else {
            i2c_master_stop();
        }
        
        */
        
        /*i2c_master_start();                     // Begin the start sequence
        i2c_master_send(SLAVE_ADDR);// << 1);       // send the slave address, left shifted by 1, 
                                            // which clears bit 0, indicating a write
        //LATBbits.LATB8 = 1;
        i2c_master_send(IODIR);         // send a byte to the slave       
        //LATBbits.LATB8 = 1;
        i2c_master_send(0xF0);         // send another byte to the slave
        i2c_master_stop();
        i2c_master_start();                   // send a RESTART so we can begin reading 
        i2c_master_send(SLAVE_ADDR);// << 1) | 0); // send slave address, left shifted by 1,
    
        i2c_master_send(GPIO);         // send a byte to the slave       
        i2c_master_send(0x01);
        i2c_master_stop(); */ 
        
        //LATBbits.LATB8 = 1;        
        //LATBbits.LATB8 = 1;        
                                            // and then a 1 in lsb, indicating read
//	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
//	// remember the core timer runs at half the sysclk
        /*i2c_master_send((reading address of button << 1) | 1); // send slave address, left shifted by 1,
                                              // and then a 1 in lsb, indicating read
        master_read0 = i2c_master_recv(); 
//        
        if (master_read0) { // check to see if button is pressed
            i2c_master_ack(1);
            i2c_master_restart();
            i2c_master_send(GPIO << 1 | 0);//(writing address of LED << 1) | 0);       // send the slave address, left shifted by 1, 
                                                    // which clears bit 0, indicating a write
            i2c_master_send(0x01); // first just set GP0 to 1 and don't care about other inputs or outputs, then do the read of the button pin before setting the output
                                   //value to change LED);         // send a byte to the slave
            i2c_master_restart();
//            _CP0_SET_COUNT(0);
//            LATAbits.LATA4 = 0;
//            while(_CP0_GET_COUNT() < 12000) {
//                ;
//            }
//            LATAbits.LATA4 = 1;
//            _CP0_SET_COUNT(0);
//            while(_CP0_GET_COUNT() < 12000) {
//                ;
                 
        } else {
            i2c_master_ack(0);
        }*/
        ;
        
      //i2c_master_send(master_write1);         // send another byte to the slave
      //i2c_master_restart();                   // send a RESTART so we can begin reading 
      //i2c_master_send((SLAVE_ADDR << 1) | 1); // send slave address, left shifted by 1,
                                              // and then a 1 in lsb, indicating read
      //master_read0 = i2c_master_recv();       // receive a byte from the bus
      //i2c_master_ack(0);                      // send ACK (0): master wants another byte!
      //master_read1 = i2c_master_recv();       // receive another byte from the bus
      //i2c_master_ack(1);                      // send NACK (1):  master needs no more bytes
      //i2c_master_stop();                      // send STOP:  end transmission, give up bus

        /////////////// invert LED here the pin is RA4 / pin 12
    }
}