/*
  PIC18F57Q43 ROM RAM and UART emulation firmware
  This single source file contains all code

  Target: emuz80_ram48 - The computer with only Z80 and PIC18F57Q43
  Compiler: MPLAB XC8 v2.36
  Copyright (C) by Norihiro Kumagai, 2023
  Original Written by Tetsuya Suzuki
  Special thanks https://github.com/satoshiokue/
*/

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG2
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON      // PRLOCKED One-Way Set Enable bit (PRLOCKED bit can be cleared and set only once)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG3
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON      // Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit (IVTLOCKED bit can be cleared and set only once)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (Low-Power BOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG4
#pragma config BORV = VBOR_1P9  // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 1.9V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD module is disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (PPSLOCKED bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG5
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG6
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG7
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config DEBUG = OFF      // Background Debugger (Background Debugger disabled)

// CONFIG8
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not Write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not Write protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not Write protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG10
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include "param.h"
#include "iopin.h"
#include "xprintf.h"

#define Z80_CLK 8000000UL // Z80 clock frequency(Max 20MHz)

#define _XTAL_FREQ 64000000UL

#define TOGGLE do { LATE2 ^= 1; } while(0)

//unsigned char ram[RAM_SIZE]; // Equivalent to RAM
union {
    unsigned int w; // 16 bits Address
    struct {
        unsigned char l; // Address low 8 bits
        unsigned char h; // Address high 8 bits
    };
} ab;

addr_t break_address = 0; // break_address is zero, it is invalid
int ss_flag = 0;

#define db_setin() (TRISC = 0xff)
#define db_setout() (TRISC = 0x00)

static void reset_DFF(void);

// UART3 Transmit
void putchx(int c) {
    while(!U3TXBE); // Wait for Tx buffer empty
    U3TXB = (unsigned char)c; // Write data
}

// UART3 Recive
int getchx(void) {
    while(U3RXBE); // Wait while Rx buffer is empty
    return U3RXB; // Read data
}

void BUSRQ_on(void)
{
    LATE0 = 0;
}

void BUSRQ_off(void)
{
    //TRISE1 = 1; // RESET is Open-Drain and pulled-up, so
                // only do it input-mode is necessary
    LATE0 = 1;
}

void RESET_on(void)
{
    LATE1 = 0;
}

void RESET_off(void)
{
    //WPUE2 = 1;
    //TRISE2 = 1; // set as input
    LATE1 = 1;
}
#define DIRECT_MODE
#if defined(DIRECT_MODE)

void RESET_CS2(void)
{
    LATA2 = 0;
}

void SET_CS2()
{
    LATA2 = 1;
}

#define nop asm("  nop")

static addr_t cur_addr = 0;


void putDataBus(unsigned char c)
{
    if (!RE1) {
        RESET_off();
    }
    while (RD6);    // wait for /WAIT becomes L
    // now in wait
    //xprintf("%04X(%02X):%c %02X\n", cur_addr, PORTB, (RA5 ? 'W' : 'R'), c);
    if (!RA5) { // /RD == L
        RESET_CS2();
        // RD cycle
        nop; nop; nop;
        db_setout();
        TOGGLE;
        LATC = c;
        TOGGLE;
    }
end_of_cycle:
    while(RA0 && RA1); // Wait for IORQ == L or MREQ == L;
    BUSRQ_on();
    reset_DFF(); // reset D-FF, /DTACK be zero
    while(RA1 == 0); // Wait for DS = 1;
    TOGGLE;
    db_setin(); // Set data bus as input
    TOGGLE;
    SET_CS2();
    BUSRQ_off();
}

void setAddr(addr_t a)
{
    unsigned char h, l;
    ab.w = a;
    LATD = ((LATD & 0xc0) | (ab.h & 0x3f));
    LATB = ab.l;
}

unsigned char getDataBus()
{
    unsigned char c;
    if (!RE1) {
        RESET_off();
    }
    while (RD6);    // wait for /WAIT becomes L
    // now in wait
    //xprintf("%04X(%02X):%c ", cur_addr, PORTB, (RA5 ? 'W' : 'R'));
    if (!RA5) {
        // RD cycle
        TOGGLE;
        nop; nop; nop; nop;
        c = PORTC;
        //xprintf("%02X", c);
        TOGGLE;
    }
    //xprintf("\n");
end_of_cycle:
    while(RA0 && RA1); // Wait for IORQ == L or MREQ == L;
    BUSRQ_on();
    reset_DFF(); // reset D-FF, /DTACK be zero
    while(RA1 == 0); // Wait for DS = 1;
    TOGGLE;
    db_setin(); // Set data bus as input
    TOGGLE;
    BUSRQ_off();
    return c;
}

// peek, poke
char peek_ram(addr_t addr)
{
    char c;
    setAddr(addr);
    db_setin();
    LATA1 = 0;  // /MREQ = 0;
    LATA5 = 0;  // /RQ = 0;
    nop; nop; nop; nop; nop; nop; nop; nop;
    c = PORTC;
    LATA5 = 1;
    LATA1 = 1;
    return c;
}

void poke_ram(addr_t addr, char c)
{
    setAddr(addr);
    db_setout();
    LATC = c;
    LATA1 = 0;  // /MREQ = 0;
    LATA2 = 0;  // /WR = 0;
    nop; nop; nop; nop; nop; nop; nop; nop;
    LATA2 = 1;
    LATA1 = 1;
    db_setin();
}
    
#endif //DIRECT_MODE

static int uc = -1;
int getchr(void)
{
    static int count = 0;
    int c;
    if (uc >= 0) {
        c = uc;
        uc = -1;
        return c;
    }
    while ((c = getchx()) == '.' && count++ < 2);
    if (c == '.') {
        count = 0;
        return -1;
    }
    count = 0;
    return c;
}

void ungetchr(int c)
{
    uc = c;
}

int is_hex(char c)
{
    if ('0' <= c && c <= '9')
        return !0;
    c &= ~0x20;     // capitalize
    return ('A' <= c && c <= 'F');
}

int to_hex(char c)
{
    //xprintf("{%c}", c);
    if ('0' <= c && c <= '9')
        return c - '0';
    c &= ~0x20;
    if ('A' <= c && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

void clear_all(void)
{
    addr_t p = 0;
    int i = 0;
    do {
        if ((p & 0xfff) == 0) {
            xprintf("%X", i++);
        }
        poke_ram(p, 0);
    } while (p++ != 0xffff);
}

void manualboot(void)
{
    int c, cc, d, n, count;
    addr_t addr = 0, max = 0, min = RAM_TOP + RAM_SIZE;
    int addr_flag = 0;
    
    while (1) {
        while ((c = getchr()) == ' ' || c == '\t' || c == '\n' || c == '\r')
            ;   // skip white spaces
        if (c == -1)
            break;
        if (c == '!' && min < max) {
            xprintf("\n");
            // dump memory
            addr_t start, end;
            start = min & 0xfff0;
            end = max;
            //if (end > 0x40)
            //    end = 0x40;
            while (start < end) {
                if ((start & 0xf) == 0) {
                    xprintf("%04X ", start);  
                }
                d = ((unsigned short)peek_ram(start))<<8;
                d |= peek_ram(start + 1);
                xprintf("%04X ", d);
                if ((start & 0xf) == 0xe) {
                    xprintf("\n");
                }
                start += 2;                
            }
            if (ss_flag)
                xprintf("ss ");
            if (break_address)
                xprintf("%%%04X ", break_address);
            continue;
        }
        if (c == 's') { // start with single_step
            ss_flag = 1;
            break;   // start processor
        }
        if (c == 'g') { // start with no-single_step
            ss_flag = 0;
            break;      // start prosessor
        }
        if (c == ',') { // clear ram
            clear_all();
            continue;
        }
        addr_flag = ((c == '=') || (c == '%'));
        cc = c;
        //xprintf("[%c]", c);
        if (!addr_flag)
            ungetchr(c);
        // read one hex value
        n = 0;
        while ((d = to_hex((unsigned char)(c = getchr()))) >= 0) {
            n *= 16; n += d;
            //xprintf("(%x,%x)", n, d);
        }
        if (c < 0)
            break;
        if (d < 0) {
            if (addr_flag) {  // set address
                if (cc == '=')
                    addr = (addr_t)n;
                else if (cc == '%')
                    break_address = (addr_t)n;
            } else {
                if (RAM_TOP <= addr && addr < (RAM_TOP + RAM_SIZE)) {
                    //xprintf("[%04X] = %02X%02X\n", addr, ((n>>8)&0xff), (n & 0xff));
                    poke_ram(addr++, ((n>>8) & 0xff));
                    poke_ram(addr++, (n & 0xff));
                    if (max < addr)
                        max = addr;
                    if (addr - 2 < min)
                        min = addr - 2;
                }
            }
            continue;
        }
    }
}

#define GET_ADDR() (((unsigned long)(PORTD&0x3f)<<8) | PORTB)

//
// monitor
// monitor_mode: 1 ... DBG_PORT write
//               2 ... DBG_PORT read
//               0 ... other(usually single step mode)
//
void monitor(int monitor_mode)
{
    static int count = 0;
    static char buf[8];
    int c, d;
    unsigned long addr = GET_ADDR();
    
    xprintf("|%05lX %02X %c ", addr, PORTC, ((RA5) ? 'R' : 'W'));
    
    if (monitor_mode == 2) {    // DBG_PORT read
        xprintf(" IN>");
        xgets(buf, 7, 0);
        int i = 0, n = 0;
        while (i < 8 && (c = buf[i++]) && (d = to_hex((unsigned char)c)) >= 0) {
            n *= 16; n += d;
            //xprintf("(%x,%x)", n, d);
        }
        LATC = (unsigned char)n;
    } else {
        if (monitor_mode == 1) { // DBG_PORT write
            xprintf(" OUT: %02x", (int)PORTC);
        }
#if 0
        if ((c = getchr()) == '.')
            ss_flag = 0;
        else if (c == 's' || c == ' ')
            ss_flag = 1;
#endif
        xprintf("\n");
    }
}

static void reset_DFF(void)
{
    //TOGGLE;
    CLCSELECT = 2;      // CLC3 select
    CLCnGLS2 = 0x40;    // 1 for D-FF RESET
    CLCnGLS2 = 0x80;    // 0 for D-FF RESET
    //TOGGLE;
}

void march_test(void)
{
    addr_t p, last = 0xffff;
    int n = 0;
    p = 0;
    do {
        poke_ram(p, 0);
    } while (p++ != last);
    n++;
    p = 0;
    do {
        if (peek_ram(p) == 0)
            poke_ram(p, 1);
        else {
            xprintf("%d: fail at %04X\n", n, p);
            return;
        }
    } while (p++ != last);
    n++;
    p = last;
    do {
        if (peek_ram(p) == 0)
            poke_ram(p, 1);
        else {
            xprintf("%d: fail at %04X\n", n, p);
            return;
        }
    } while (p-- != 0);
    n++;
    p = last;
    do {
        if (peek_ram(p) == 1)
            poke_ram(p, 0);
        else {
            goto error;
        }
    } while (p-- != 0);
    xprintf("all ok\n");
    return;
error:
    xprintf("%d: fail at %04X\n", n, p);
    return;
}


#define db_setin() (TRISC = 0xff)
#define db_setout() (TRISC = 0x00)

// main routine
void main(void) {
    int monitor_mode = 0;
    int count;
    unsigned char cc;
    unsigned long addr;
    // System initialize
    OSCFRQ = 0x08; // 64MHz internal OSC

    // RE1: RESET output pin
    ANSELE1 = 0; // Disable analog function
    LATE1 = 0; // RESET assert
    TRISE1 = 0; // Set as output

    // RE2: TEST Pin output
    ANSELE2 = 0;
    TRISE2 = 0;
    LATE2 = 0;
    
    TOGGLE; TOGGLE;
    // xprintf initialize
    xdev_out(putchx);
    xdev_in(getchx);
    // CLC disable
    CLCSELECT = 0;
    CLCnCON &= ~0x80;
    CLCSELECT = 1;
    CLCnCON &= ~0x80;
    CLCSELECT = 2;
    CLCnCON &= ~0x80;

    // Address bus A15-A8 pin
    ANSELD = 0x00; // Disable analog function
    WPUD = 0xff; // Week pull up
    TRISD |= 0xff; // Set as input

    // Address bus A7-A0 pin
    ANSELB = 0x00; // Disable analog function
    WPUB = 0xff; // Week pull up
    TRISB = 0xff; // Set as input

    // Data bus D7-D0 pin
    ANSELC = 0x00; // Disable analog function
    WPUC = 0xff; // Week pull up
    TRISC = 0xff; // Set as input(default)

    // IO pin assignment
    
    // RE1: RESET output pin
    ANSELE1 = 0; // Disable analog function
    LATE1 = 0; // RESET assert
    TRISE1 = 0; // Set as output

    // RE0: BUSRQ output pin
    ANSELE0 = 0; // Disable analog function
    LATE0 = 1; // BUSRQ deassert
    TRISE0 = 0; // Set as output

    // RD6: WAIT output pin
    ANSELD6 = 0; // Disable analog function
    LATD6 = 1; // WAIT negate
    TRISD6 = 0; // Set as output

#if 0
    // In RAM40, no M1 pin is used
    // RB7: M1 input pin
    ANSELB5 = 0; // Disable analog function
    TRISB5 = 1; // Set as input
#endif
    
    // RD7: RFSH input pin
    ANSELD7 = 0;
    TRISD7 = 1; // set as input
    
    // RA0: IORQ input pin
    ANSELA0 = 0; // Disable analog function
    TRISA0 = 1; // Set as input

    // Z80 clock(RA3) by NCO FDC mode
    RA3PPS = 0x3f; // RA3 assign NCO1
    ANSELA3 = 0; // Disable analog function
    TRISA3 = 0; // NCO output pin
    NCO1INC = Z80_CLK * 2 / 61;
    NCO1INC = 0x80000;
    NCO1CLK = 0x00; // Clock source Fosc
    NCO1PFM = 0;  // FDC mode
    NCO1OUT = 1;  // NCO output enable
    NCO1EN = 1;   // NCO enable
    
    //RA3PPS = 0;     // internal clock disable

    // UART3 initialize
//    U3BRG = 416; // 9600bps @ 64MHz
    U3CON0 |= (1<<7);   // BRGS = 0, 4 baud clocks per bit
    U3BRG = 138;    // 115200bps @ 64MHz, BRG=0, 99%

    U3CON0 &= 0xf0; // clear U3MODE 0000 -> 8bit
    U3TXBE = U3RXBE = 0;    // clear tx/rx/buffer
    U3RXEN = 1; // Receiver enable
    U3TXEN = 1; // Transmitter enable

    // UART3 Receiver
    ANSELA7 = 0; // Disable analog function
    TRISA7 = 1; // RX set as input
    U3RXPPS = 0x07; //RA7->UART3:RX3;

    // UART3 Transmitter
    ANSELA6 = 0; // Disable analog function
    LATA6 = 1; // Default level
    TRISA6 = 0; // TX set as output
    RA6PPS = 0x26;  //RA6->UART3:TX3;

    // 1, 2, 5, 6: Port A, C
    // 3, 4, 7, 8: Port B, D
    RA4PPS = 0x0;  // LATA4 -> RA4 -> /OE
    RA2PPS = 0x0;  // LATA2 -> RA2 -> /WE
    RD6PPS = 0x0;  // LATD6 -> RD6 -> WAIT

    U3ON = 1; // Serial port enable
    xprintf(";");

    // BUSREQ and PIC controls MREQ/RD/WR
    // boot mode
    // RA5: /RD, SRAM /OE pin
    ANSELA5 = 0;
    LATA5 = 1;  // /RD negate
    TRISA5 = 0; // set as input
    
    // RA2: /WR, SRAM /WE pin
    ANSELA2 = 0;
    LATA2 = 1;  // /WR negate
    TRISA2 = 0; // set as input

    // RA1: MREQ input pin
    ANSELA1 = 0; // Disable analog function
    LATA1 = 1;  // /MREQ negate
    TRISA1 = 0; // Set as input
    
    // BUSRQ on, enter BUSRQ mode
    LATE0 = 0;  // /BUSRQ assert
    LATE1 = 1;  // start CPU
    nop; nop; nop; nop;
    nop; nop; nop; nop;
    TOGGLE;
    TRISD &= 0xc0;
    TRISB = 0;
#if 0
    // L-chika
    while (1) {
        LATE1 ^= 1;
        LATE0 ^= 1;
        for (int i = 0; i < 100; ++i) {
            __delay_ms(10);
        }
    }
#endif
    
    manualboot();
    
    // RESET again
    LATE1 = 0;
    LATE0 = 1;  // /BUSRQ deassert
    db_setin();
    TOGGLE;
    TOGGLE;
    // Re-initialze for CPU running
    // RA5: /RD, SRAM /OE pin
    ANSELA5 = 0;
    LATA5 = 1;  // /RD negate
    TRISA5 = 1; // set as input
    
    // RA2: /WR, SRAM /WE pin
    ANSELA2 = 0;
    LATA2 = 1;  // CS2 negate
    TRISA2 = 1; // set as input

    // RA1: MREQ input pin
    ANSELA1 = 0; // Disable analog function
    TRISA1 = 1; // Set as input


#if 1
    // Address bus A15-A8 pin
    ANSELD = 0x00; // Disable analog function
    WPUD = 0x3f; // Week pull up
    TRISD |= 0x3f; // Set as input

    // Address bus A7-A0 pin
    ANSELB = 0x00; // Disable analog function
    WPUB = 0xff; // Week pull up
    TRISB = 0xff; // Set as input

    // Data bus D7-D0 pin
    ANSELC = 0x00; // Disable analog function
    WPUC = 0xff; // Week pull up
    TRISC = 0xff; // Set as input(default)
#endif
    // reconfigurate CLC devices
    // CLC pin assign
    // 0, 1, 4, 5: Port A, C
    // 2, 3, 6, 7: Port B, D
    CLCIN1PPS = 0x00;   // RA0 <- /IORQ
#if 0
    CLCIN0PPS = 0x01;   // RA1 <- /MREQ
    CLCIN2PPS = 0x1F;   // RD7 <- /RFSH
    CLCIN3PPS = 0x1E;   // RD6 <- /WAIT
    CLCIN4PPS = 0x05;   // RA5 <- /RD
#endif
    
    // ============== CLC3 /WAIT
    CLCSELECT = 2;      // CLC3 select
    CLCnCON &= ~0x80;
    
    CLCnSEL0 = 1;       // D-FF CLK <- CLCIN1PPS <- /IORQ
    CLCnSEL1 = 127;     // D-FF D NC
    CLCnSEL2 = 127;     // D-FF SET NC
    CLCnSEL3 = 127;     // D-FF RESET NC
    
    CLCnGLS0 = 0x01;    // /IORQ ~|_  (inverted)
    CLCnGLS1 = 0x40;    // D-FF D NC (1 for D-FF D)
    CLCnGLS2 = 0x80;    // D-FF SET (soft reset)
    CLCnGLS3 = 0x00;    // 0 for D-FF RESET

    // reset D-FF
    CLCnGLS2 = 0x40;    // 1 for D-FF RESET
    CLCnGLS2 = 0x80;    // 0 for D-FF RESET

    CLCnPOL = 0x80;     // non inverted the CLC3 output
    CLCnCON = 0x84;     // Select D-FF (no interrupt)
        
    // Here, we have CLC's be intialized.
    // Now we change GPIO pins to be switched
    // 1, 2, 5, 6: Port A, C
    // 3, 4, 7, 8: Port B, D
    RD6PPS = 0x03;      // CLC3 -> RD6 -> /WAIT
    
    xprintf("start ss = %d, bp = %04X\n", ss_flag, break_address);
    // Z80 start
    //CLCDATA = 0x7;
    TRISD |= 0x3f;
    TRISB = 0xff;
    BUSRQ_off();
    reset_DFF();
    db_setin();
    TOGGLE;
    TOGGLE;
    TOGGLE;
    TOGGLE;
    RESET_off();    // RESET negate
    while(1){
        while(RD6);    // Wait for /WAIT == 9
        //TOGGLE;
        //TOGGLE;
        //if (!RA1)  // MREQ == 0, memory cycle, do nothing
        //    goto end_of_cycle;
        // IORQ == 0, IO R/W cycle or INTA cycle
        if(!RA5) { // RD == 0, Z80 read cycle (RW = 1)
            addr = PORTB;
            if (addr == UART_CREG){ // UART control register
                // PIR9 pin assign
                // U3TXIF ... bit 1, 0x02
                // U3RXIF ... bit 0, 0x01
                // U3FIFO bit assign
                // TXBE   ... bit 5, 0x20
                // TXBF   ... bit 4, 0x10
                // RXBE   ... bit 1, 0x02
                // RXBF   ... bit 0, 0x01
                //cc = (U3TXBE ? 0x20 : 0) | (U3RXBE ? 2 : 0);
                cc = PIR9;
                db_setout();
                TOGGLE;
                LATC = cc; // U3 flag
                TOGGLE;
                //for (int i = 6; i-- > 0; ) nop;
            } else if(addr == UART_DREG) { // UART data register
                cc = U3RXB; // U3 RX buffer
                while(!(PIR9 & 2));
                db_setout();
                TOGGLE;
                LATC = cc;
                TOGGLE;
                //for (int i = 6; i-- > 0; ) nop;
            //} else if((addr & 0xff00) == DBG_PORT) {
            //    monitor_mode = 2;   // DBG_PORT read
            } else { // invalid address
                db_setout();
                LATC = 0xff;
                //xprintf("%05lX: %02X %c bad\n", addr, PORTC, (RA5 ? 'R' : 'W'));
                //monitor_mode = 0;
            }
            if (ss_flag || monitor_mode) {
                xprintf("%05lX: %02X %c m%d\n", addr, PORTC, (RA5 ? 'R' : 'W'), monitor_mode);
                monitor(monitor_mode);
                monitor_mode = 0;
            }
        } else { // Z80 write cycle (RW = 0)
            addr = PORTB;
            // A19 == 1, I/O address area
            if(addr == UART_DREG) { // UART data register
                // this cycle actually starts at /MREQ down edge,
                // so too early PORTC reading will get garbage.
                // We need to wait for stable Z80 data bus.
                //for (int i = 5; i-- > 0; ) nop;
                TOGGLE;
                U3TXB = PORTC; // Write into U3 TX buffer
                TOGGLE;
                //xprintf("(%02X)", PORTC);
                //while(!(PIR9 & 2));
            //while(PIR9 & 2);
            //} else if((addr & 0xfff00) == DBG_PORT) {
            //    monitor_mode = 1;   // DBG_PORT write
            } else {
                // ignore write C000 or upper
                //xprintf("%05lX: %02X %c\n", addr, PORTC, (RA5 ? 'R' : 'W'));
                //monitor_mode = 1;
            }
        }
        if (ss_flag || monitor_mode) {
            xprintf("%05lX: %02X %c M%d\n", addr, PORTC, (RA5 ? 'R' : 'W'), monitor_mode);
            monitor(monitor_mode);
            monitor_mode = 0;
        }
    end_of_cycle:
        while(RA0 && RA1); // Wait for IORQ == L or MREQ == L;
        BUSRQ_on();
        reset_DFF(); // reset D-FF, /DTACK be zero
        while(RA0 == 0 || RA1 == 0); // Wait for DS = 1;
        //for (int i = 3; i-- > 0;) nop;
        TOGGLE;
        db_setin(); // Set data bus as input
        TOGGLE;
        BUSRQ_off();
    }
}

