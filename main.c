/**
This code runs on the TI MSP430 5529 launchpad
http://www.ti.com/tool/msp-exp430f5529lp?DCMP=msp-f5529&HQS=msp-f5529-b

it supports the led strips with the 800kHz WS2812 controller chips, e.g.  
http://www.adafruit.com/products/1138

it is partly based on the idea of UART patterns from robg:
http://forum.43oh.com/topic/3971-wearable-ws2812-strip-controllers/

but uses the DMA rather than a for() loop:
+ work is delegated to DMA, frees foreground application (easier to code color changes)
- needs app. 8x more RAM, as the LED bits must be deflated in advance

REF0 serves as the reference for the FLL. DCO is set to app. 24MHz. SPI UART runs 
from SMCLK (app.8MHz). XT2 is not used so porting the code to any 5xx derivate 
with the REFO should not be difficult

LED strip must be connected to P3.3 on the launchpad header

An LED strip of 60 LEDs takes about 2msec to update

The code serves demonstration purposes only, may need cleanup. Have fun!

marfis (fischer.carlito@gmail.com)

*/

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#define NR_OF_LEDS              60                                  // 60 LEDs strip, configure this here 

#define WS_BIT_0                0xe0                                // UART pattern for Bit 0
#define WS_BIT_1                0xf8                                // UART pattern for Bit 1
#define SIZE_OF_LED_ARRAY       (NR_OF_LEDS * 3 * 8)                // DMA buffer, 24 bytes per LED

#define max(a,b) (((a)>(b)) ? (a) : (b))                
#define min(a,b) (((a)<(b)) ? (a) : (b))                

#define st(x)      do { x } while (__LINE__ == -1)
#define SELECT_ACLK(source)   st(UCSCTL4 = (UCSCTL4 & ~(SELA_7))   | (source);) 
#define SELECT_MCLK(source)   st(UCSCTL4 = (UCSCTL4 & ~(SELM_7))   | (source);) 
#define SELECT_SMCLK(source)  st(UCSCTL4 = (UCSCTL4 & ~(SELS_7))   | (source);) 


uint8_t led_raw[SIZE_OF_LED_ARRAY]; // DMA space, incl. 50usec break time

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} led_color_t;      // led color struct


void hardware_clock_init(void) {
    uint8_t i;
    __disable_interrupt();               // Disable global interrupts
    // Enable XT2 pins
    P5SEL |= 0x0C;
    // Use REFO as the FLL reference
    UCSCTL3 = (UCSCTL3 & ~(SELREF_7)) | (SELREF_2);
    UCSCTL4 |= SELA__REFOCLK;          // ACLK source 
    uint16_t srRegisterState = __read_status_register() & SCG0;

    __bis_SR_register(SCG0);  
    // Set lowest possible DCOx, MODx
    UCSCTL0 = 0x0000;                         
    // Select DCO range 50MHz operation
    UCSCTL1 = DCORSEL_7 | 1;                      
    UCSCTL2 = FLLD_0 + 550;  
    __bic_SR_register(SCG0);  

    for (i=0; i<100; i++) {
        __delay_cycles(0xffff);
    }
    
    // check for oscillator fault
    while (SFRIFG1 & OFIFG) {                   
        UCSCTL7 &= ~(DCOFFG+XT1LFOFFG+XT2OFFG); 
        SFRIFG1 &= ~OFIFG;                      
    }

    SELECT_MCLK(SELM__DCOCLK);
    SELECT_ACLK(SELA__REFOCLK);
    SELECT_SMCLK(SELS__DCOCLK);
    UCSCTL7 = 0;        // Errata UCS11

    __bis_SR_register(srRegisterState);                      // Restore previous SCG0
}

// TimerA0 Interrupt Vector (TAIV) handler, re-enables the DMA
static void __attribute__((__interrupt__(TIMER0_A0_VECTOR))) timer_a_irq(void) {
    TA0CCR0 += 300;
    DMA2CTL |= DMAEN | DMAIE ;
    DMA2SA = (uint16_t)(&led_raw[1]); // source
    UCA0TXBUF = led_raw[0];
}

// the DMA interrupt of channel2 just exits from low power mode
// so that a foreground application can do its work
static void __attribute__((__interrupt__(DMA_VECTOR))) dma_irq(void) {
    switch(DMAIV) {
        case 0: break;
        case 2: break;
        case 4: break;                          // DMA1IFG = DMA Channel 1
        case 6: 
            // DMA transfer to LEDs finished, exit low power
            LPM0_EXIT;
            break;
        case 8: break;                          // DMA3IFG = DMA Channel 3
        case 10: break;                         // DMA4IFG = DMA Channel 4
        case 12: break;                         // DMA5IFG = DMA Channel 5
        case 14: break;                         // DMA6IFG = DMA Channel 6
        case 16: break;                         // DMA7IFG = DMA Channel 7
        default: break;
    }  
}
/* timer_init
 * 
 * initalizes timerA0 to 10msec intervals, CCRO, running from REFO (32kHz)
 *
 */
void timer_init(void) {
    TA0CCTL0 = CCIE;                
    TA0CCR0 = 300;
    // ACLK, continous up, clear TAR
    TA0CTL = TASSEL_1 + MC__CONTINOUS + TACLR;         
}
    
/*
 * init_spi_master
 * configures USCIA0 to SPI master ,8MHz clock rate, 8Bit, running from SMCLK
 *
 */
void init_spi_master(void) {
    P3SEL |= BIT3+BIT4;                       // P3.3,4 option select
    P2SEL |= BIT7;                            // P2.7 option select
    
    UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA0CTL0 |= UCMST+UCSYNC+UCCKPL+UCMSB;    // 3-pin, 8-bit SPI master
                                                // Clock polarity high, MSB
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 3;                              // /3 = ca.8MHz
    UCA0BR1 = 0;                              //
    UCA0MCTL = 0;                             // No modulation
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
}

/* SetVCoreUp
 * TI PMM driverlib Code to set Vcore to a given level
 *  
 * this is complicated code, but apparently is needed to come around
 * erratum FLASH37
 * removed some original comments to compress code
 *
 */
uint16_t SetVCoreUp(uint8_t level)
{
    uint16_t PMMRIE_backup, SVSMHCTL_backup, SVSMLCTL_backup;
    // Open PMM registers for write access
    PMMCTL0_H = 0xA5;
    // Disable dedicated Interrupts
    // Backup all registers
    PMMRIE_backup = PMMRIE;
    PMMRIE &= ~(SVMHVLRPE | SVSHPE | SVMLVLRPE | SVSLPE | SVMHVLRIE | SVMHIE | SVSMHDLYIE | SVMLVLRIE | SVMLIE | SVSMLDLYIE );
    SVSMHCTL_backup = SVSMHCTL;
    SVSMLCTL_backup = SVSMLCTL;
    // Clear flags
    PMMIFG = 0;
    
    // Set SVM highside to new level and check if a VCore increase is possible
    SVSMHCTL = SVMHE | SVSHE | (SVSMHRRL0 * level);    
    
    // Wait until SVM highside is settled
    while ((PMMIFG & SVSMHDLYIFG) == 0);
    // Clear flag
    PMMIFG &= ~SVSMHDLYIFG;
    // Check if a VCore increase is possible
    if ((PMMIFG & SVMHIFG) == SVMHIFG) {      // -> Vcc is too low for a Vcore increase
        // recover the previous settings
        PMMIFG &= ~SVSMHDLYIFG;
        SVSMHCTL = SVSMHCTL_backup;
        // Wait until SVM highside is settled
        while ((PMMIFG & SVSMHDLYIFG) == 0);
        // Clear all Flags
        PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);
        PMMRIE = PMMRIE_backup;                 // Restore PMM interrupt enable register
        PMMCTL0_H = 0x00;                       // Lock PMM registers for write access
        return 0;                // return: voltage not set
    }
    
    // Set also SVS highside to new level
    SVSMHCTL |= (SVSHRVL0 * level);
    PMMIFG &= ~SVSMHDLYIFG;
    // Set VCore to new level
    PMMCTL0_L = PMMCOREV0 * level;
    // Set SVM, SVS low side to new level
    SVSMLCTL = SVMLE | (SVSMLRRL0 * level) | SVSLE | (SVSLRVL0 * level);
    PMMIFG &= ~SVSMLDLYIFG;
    SVSMLCTL &= (SVSLRVL0+SVSLRVL1+SVSMLRRL0+SVSMLRRL1+SVSMLRRL2);
    SVSMLCTL_backup &= ~(SVSLRVL0+SVSLRVL1+SVSMLRRL0+SVSMLRRL1+SVSMLRRL2);
    SVSMLCTL |= SVSMLCTL_backup;
    SVSMHCTL &= (SVSHRVL0+SVSHRVL1+SVSMHRRL0+SVSMHRRL1+SVSMHRRL2);
    SVSMHCTL_backup &= ~(SVSHRVL0+SVSHRVL1+SVSMHRRL0+SVSMHRRL1+SVSMHRRL2);
    SVSMHCTL |= SVSMHCTL_backup;
    PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);
    PMMRIE = PMMRIE_backup;                   // Restore PMM interrupt enable register
    PMMCTL0_H = 0x00;                         // Lock PMM registers for write access
    return 1;
}


/* _set_single_color
 *  
 * writes one brightness byte at the given location in the led array
 * 
 * as the DMA uses the led array directly to output the bits over SPI
 * it must be deflated in advance in memory. One bit of WS2812 information 
 * is one byte to transfer over SPI 
 * -> bit1 = WS_BIT_1 and bit0 = WS_BIT_0
 * 
 * disadvantage is that it uses 8x more RAM... 
 * 
 * this function is normally called by ::set_LED
 * 
 * @param led_color_ptr     [in] pointer to location within the ::led_raw array
 * @param led_color         [in] single byte, represents the brightness
 */

void _set_single_color(uint8_t* led_color_ptr, uint8_t brightness)   {
    uint8_t i;
    uint8_t mask = 0x80;
    for (i=0; i<8; i++) {
        if (brightness & mask) {
            *led_color_ptr = WS_BIT_1;
        } else {
            *led_color_ptr = WS_BIT_0;
        }
        mask = mask >> 1;
        led_color_ptr++;
    }
}

/* set_LED
 *  
 * writes the color information (RGB) of the given LED number 
 * within the ::led_raw array
 * 
 * @param led_nr            [in] position of LED within the strip
 * @param led_color         [in] RGB information
 */
void set_LED(uint8_t led_nr, led_color_t* led_color) {
    uint8_t* led_ptr = led_raw + led_nr * 24;
    _set_single_color(led_ptr, led_color->g);
    _set_single_color(led_ptr+8, led_color->r);
    _set_single_color(led_ptr+16, led_color->b);
}
 
int main(void)
{
    uint16_t time = 0;
    led_color_t col;
    
    // dog
    WDTCTL = WDTPW + WDTHOLD;     
    
    // set vcore to highest value, mclk 24 MHz
    SetVCoreUp(0x01);
    SetVCoreUp(0x02);  
    SetVCoreUp(0x03);  
    // hw init
    hardware_clock_init();
    init_spi_master();
    timer_init();
    
    // internal LED and debug pins
    P1DIR |= BIT0;                            
    P2DIR |= BIT2;
    
    // channel 17 = UCATXIFG as trigger for DMA
    DMACTL0 = 0;               
    DMACTL1 = 17;               
    DMACTL2 = 0;               
    DMACTL3 = 0;               
    // DMA2 configuration
    // bytewise access for source and destination, increment source, single transfer
    DMA2CTL = DMADT_0 | DMASRCINCR_3 | DMASRCBYTE | DMADSTBYTE | DMAIE; 
    DMA2SA = (uint16_t)(&led_raw[1]); // source
    DMA2DA = (uint16_t)&UCA0TXBUF;    // destination
    DMA2SZ = SIZE_OF_LED_ARRAY-1;     // size in bytes
    
    __eint();
    // all set, now let the DMA run
    DMA2CTL |= DMAEN ;
    UCA0TXBUF = led_raw[0];
    while (1)                
    {
        time++;
        P1OUT ^= BIT0;
        LPM0;
        // DMA transfer has finished, you may calculate LED updates now
        // timer A is triggered app. every 10msec, so you have around 6msec time
        // to update the strip
        
        // this is a color fading thing
        for (uint8_t i=0; i<NR_OF_LEDS; i++){
            col.g = (time % 512) + i*4;
            col.r = (time % 256) + i;
            col.b = (time / 1024) + (NR_OF_LEDS - i);
            set_LED(i,&col);
        }
    }
}

