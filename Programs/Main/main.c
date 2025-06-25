#include "intrinsics.h"
#include "msp430f5529.h"
#include "ref.h"
#include <msp430.h>
#include <driverlib.h>
#include <stdint.h>
#include <stdio.h>  // <-- Added for sprintf

#define NUM_SAMPLES 256      // Number of ADC samples to average
#define SMCLKFreq 1000000.0
#define delay_ms_const(ms) __delay_cycles((SMCLKFreq / 1000UL) * (ms))

volatile unsigned int adc_result = 0;  // Stores the averaged ADC value
volatile unsigned char trigger = 0;    // Software trigger flag
volatile unsigned int i = 0;
volatile unsigned int timestamp = 0;
unsigned int DataProcessed;
volatile uint16_t samp_start;
volatile uint16_t samp_end;
volatile float samp_freq;
void configureADC();
void startConversion();
uint16_t adc_value = 0;
uint8_t nextion_value = 0;
volatile uint32_t lba = 2048;  // or 0 depending on your card layout
volatile uint8_t buffer[512];

char cmd0[]="sleep=0";
char cmd1[]="page 1";

///////////////////////////////////////////////////////////////////////////////
//                                                                           //
//                         MAIN FUNCTION                                     //
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer
    TA0CTL = TASSEL_2 | MC_2 | TACLR;  // SMCLK source, Continuous mode, Clear timer
    // Configure ADC and Trigger (Assume Button on P1.1 for manual trigger)
    configureADC();
    uart_init();
    P1DIR &= ~BIT1;            // P1.1 as input (Button)
    P1REN |= BIT1;             // Enable pull-up/down resistor
    P1OUT |= BIT1;             // Set pull-up resistor
    P1IE |= BIT1;              // Enable interrupt for P1.1
    P1IES |= BIT1;             // Trigger on falling edge
    P1IFG &= ~BIT1;            // Clear interrupt flag
    __enable_interrupt();      // Enable global interrupts
    while (1) {
        //__bis_SR_register(LPM0_bits + GIE);  // Enter low-power mode, wait for ISR
        if (trigger) {
            trigger = 0;          // Clear trigger
            startConversion();    // Run ADC sampling logic
            // Optionally: do something with adc_result here
            // Example: send to UART, store in buffer, display, etc.
        }
    }
}
///////////////////////////////////////////////////////////////////////////////
//                             END OF MAIN                                   //
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//                  Configure ADC for single-channel conversion              //
///////////////////////////////////////////////////////////////////////////////
void configureADC() {
    ADC12CTL0 = ADC12SHT0_4 | ADC12ON;                  // Sample-and-hold time, turn ADC on
    ADC12CTL1 = ADC12SHP;                               // Use sampling timer.
    ADC12CTL2 = ADC12RES_2;                             // Ensures 12-bit resolution (default might be lower).
    ADC12MCTL0 = ADC12INCH_1; // | REF_VREF2_5V;            // Channel A0, VR+ = 2.5V
    ADC12IE = ADC12IE0;                                 // Enable ADC interrupt
    REFCTL0 = REFON | REFVSEL_2;                        // Turn on 2.5V reference
    __delay_cycles(1000);                               // Allow time for reference to settle
}

///////////////////////////////////////////////////////////////////////////////
//                              UART MSP430-Nextion Disp                     //
///////////////////////////////////////////////////////////////////////////////
// Initialize UART (assumes UCA0, SMCLK = 1MHz, 9600 baud)
void uart_init(void) {
    UCA0CTL1 |= UCSWRST;
    UCA0CTL1 |= UCSSEL_2;

    UCA0BR0 = 104; //104 || 9600 baud
    UCA0BR1 = 0;
    UCA0MCTL = UCBRS_1;

    P3SEL |= BIT3;
    P3DIR |= BIT3;

    UCA0CTL1 &= ~UCSWRST;
}

// Send one character
void uart_send_byte(unsigned char byte) {
    while (!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = byte;
}

// Send null-terminated string
void uart_send_string(const char* str) {
    while (*str) {
        uart_send_byte(*str++);
    }
}

// Wakeup and Select Sensor Page on Nextion
void send_MCU_to_nextion_sensor_page(void){
        uart_send_string(cmd0);
    uart_send_byte(0xFF);
    uart_send_byte(0xFF);
    uart_send_byte(0xFF);
        uart_send_string(cmd1);
    uart_send_byte(0xFF);
    uart_send_byte(0xFF);
    uart_send_byte(0xFF);
}
// Convert uint8_t to ASCII and send
void Send_ADC_to_Nextion_Waveform(uint8_t val) {
    char cmd7[11];  // Buffer for formatted string
    sprintf(cmd7, "add 1,0,%u", val);

        uart_send_string(cmd7);
    uart_send_byte(0xFF);
    uart_send_byte(0xFF);
    uart_send_byte(0xFF);
}

// Update Datastore
void Send_ADC_to_Nextion_DataStore(uint16_t date, uint16_t adcval) {
    char cmd8[64];
    memset(cmd8, 0, sizeof(cmd8));  // Clear buffer before writing
    sprintf(cmd8, "data0.insert(\"%u^%u\")", date, adcval);

        uart_send_string(cmd8);
    uart_send_byte(0xFF);
    uart_send_byte(0xFF);
    uart_send_byte(0xFF);
}
///////////////////////////////////////////////////////////////////////////////
//                       Start ADC sampling and averaging                    //
///////////////////////////////////////////////////////////////////////////////
void startConversion() {

    for (i = 0; i < NUM_SAMPLES; i++) {
        samp_start = TA0R;
        ADC12CTL0 |= ADC12ENC | ADC12SC;   // Start conversion
        DataProcessed = 0;
        while (!(DataProcessed));   // Wait for conversion to complete
    }

}

///////////////////////////////////////////////////////////////////////////////
//                               Wait for Trigger                            //
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//                Button Interrupt: Set trigger flag when pressed            //
///////////////////////////////////////////////////////////////////////////////
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void) {
    trigger = 1;             // Set trigger flag
    send_MCU_to_nextion_sensor_page();
    P1IFG &= ~BIT1;          // Clear interrupt flag
    __bic_SR_register_on_exit(CPUOFF);  // Clear the CPUOFF bit to wake up the CPU
}

///////////////////////////////////////////////////////////////////////////////
//                      MCU-ADC Interrupt: Switch CMD                        //
///////////////////////////////////////////////////////////////////////////////
#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR(void) {
    switch (__even_in_range(ADC12IV, 34)) {
        case  0: break;        // No interrupt
        case  2: break;        // ADC12MEMx overflow
        case  4: break;        // ADC12 conversion time overflow
        case  6:               // ADC12MEM0 interrupt
                samp_end = TA0R;
                adc_value = ADC12MEM0;      
                samp_freq = 1/((samp_end - samp_start) / SMCLKFreq);
                ADC12IFG &= ~ADC12IFG0; // Clear interrupt flag
                DataProcessed = 1;
                nextion_value = (adc_value / 16);
                Send_ADC_to_Nextion_Waveform(nextion_value);
                Send_ADC_to_Nextion_DataStore(samp_end, adc_value);
                delay_ms_const(1000);
                break;
        case 8: break; //ADC12MEM1
        default: break;
    }
}
