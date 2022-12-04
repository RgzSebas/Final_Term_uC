#include "config_bits.h"
#include <stdint.h>
#include <stdio.h>
#include <pic18f57q43.h>
#include<xc.h>

#define _XTAL_FREQ 1000000

void __interrupt(irq(AD)) ADC_ISR(){
// Use sprintf() to save numerical values to arrays
}

void UART_Tx(char data);
uint16_t ADC_READ(uint8_t channel);

uint8_t led[12];

void main(){
    // Define variables
    uint8_t volt;
    uint8_t data ;
    
 
//----------------------------------------------------------------------------//
// I/O PORTS CONFIGURATION
 // Here we define the I/O port configuration for the 12 LEDs as OUTPUT
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC5 = 0;
    TRISCbits.TRISC6 = 0; 
    TRISCbits.TRISC7 = 0;
    TRISDbits.TRISD0 = 0;
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD2 = 0;
    TRISEbits.TRISE0 = 0;
//----------------------------------------------------------------------------//
// ADC INPUT FROM ANALOG TEMPERATURE SENSOR

// OUTPUT OF ADC

// UART TX AND TX PORTS
    U1CON0bits.TXEN = 1; // TX Transmit port 
    U1CON0bits.RXEN = 1; // RX Recieve port (although not used for this code) 
//----------------------------------------------------------------------------//
// ADC CONFIGURATION
    ANSELAbits.ANSELA1 = 1; // I/O Analog port is RA0
    ADCON2bits.MD = 0; // Mode ADC
    ADCON0bits.FM = 1; // 4 bits ADRESH MSB - ADRESL LSB 8 bits
 
//----------------------------------------------------------------------------//
// CHANNEL SELECTION
    TRISAbits.TRISA0 = 1; // Select  RA0
    ANSELAbits.ANSELA0 = 1; // Channel RA0
// ADC VOLTAGE REFERENCE SELECTION
    ADREFbits.NREF = 0; // VSS - Negative Voltage Reference
    ADREFbits.PREF = 0; // VDD - Positive Voltage Reference
// ADC CONVERSION CLOCK SOURCE
    ADCON0bits.ON = 1; // Turn ADC ON
// INTERRUPT CONTROL

// RESULT FORMATING
    ADCON0bits.CS = 0; // Clock Fosc
// CONVERSION TRIGGER SELECTION
    ADCON0bits.GO = 1; // Trigger conversion
// ADC ADQUISITION TIME
    ADCLK = 0b001111; // 500ns = 0.5us -> TAD
// ADC PRECHARGE TIME

// TURN ON ADC MODULE
    TRISAbits.TRISA1 = 1; // Turn ADC module ON
// START ADC CONVERSION

//----------------------------------------------------------------------------//
// UART CONFIGURATION
    U1RXPPS = 0x29; // RF1 -> RX 
    //U1TXB = 'A'; //->> TSR
//----------------------------------------------------------------------------//
// PIN MAPPING
    RF0PPS = 0x20; // RF0 -> TX
// TRANSMITTER ENABLING
    U1CON0bits.BRGS = 0; // Formula 16
    U1BRG = 416; // 9600 Baud rate
    U1CON0bits.MODE = 0; // Mode 8 bits UART
    U1CON1bits.ON = 1;   
    TRISFbits.TRISF1 = 1; // Enable
    ANSELFbits.ANSELF1 = 0; // Digital
    WPUFbits.WPUF1 = 0; // Pins Pull Up
    INLVLFbits.INLVLF1 = 0; // TTL
    SLRCONFbits.SLRF1 = 1; // Limit
    ODCONFbits.ODCF1 = 0; // Push Pull 
//----------------------------------------------------------------------------//
// INTERRUPTS CONFIGURATION
//----------------------------------------------------------------------------//
// GLOBAL INTERRUPTS CONFIGURATION

while(1){
 uint8_t adc =ADC_READ(1);
 //Conversion to VOLTS
volt = adc*0.0008056; 

//Send volts over UART
UART_Tx(volt);

//Turn LEDs proportional to sensor voltage
for(int i=0;i<12;i++){
led[12]=adc;
   LATCbits.LATC0 = led[0]; // LED 0
   LATCbits.LATC1 = led[1];
   LATCbits.LATC2 = led[2];
   LATCbits.LATC3 = led[3];
   LATCbits.LATC4 = led[4];
   LATCbits.LATC5 = led[5];
   LATCbits.LATC6 = led[6]; 
   LATCbits.LATC7 = led[7];
   LATBbits.LATB5 = led[8];
   LATDbits.LATD1 = led[9];
   LATDbits.LATD2 = led[10];
   LATEbits.LATE0 = led[11]; // LED 12
}
};

}

uint16_t ADC_READ(uint8_t channel){
    ADPCH = channel;
    ADCON0bits.GO = 1;  
    while(ADCON0bits.GO == 1);  
    return((uint16_t)((ADRESH<<8) + ADRESL));
  
}
void UART_Tx(char data){
    while(U1ERRIRbits.TXMTIF == 0); //  
    
    U1TXB = data; // 
}
