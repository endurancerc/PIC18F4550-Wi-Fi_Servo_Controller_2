/********************************************************************
 *
 *                			WiFi 8 Servo Controller
 *
 ********************************************************************
 *
 * Author               Date        Comment
 ********************************************************************
 * Phillip Pickett     3.1.09		Initial Release		
 * Phillip Pickett     9.4.09		Switch to set Wired LAN/Wireless
 *									LAN connection method.
 * Phillip Pickett     9.5.09		Save last state in memory to it
 *									doesnt reprogram on reset unless
 *									the mode is changed
 * Phillip Pickett     11.5.09	 	Production PCB modifications
 * Phillip Pickett     12.17.09	 	Protocol change
 * Phillip Pickett     1.16.10	 	Protocol change again
 ********************************************************************/

#include <p18f4550.h>
#include <delays.h>
#include <stdlib.h>
#include <stdio.h>
#include <usart.h>
#include <timers.h>
#include <string.h>
#include "typedefs.h"

#pragma udata
#define TIMER_PERIOD	0x752C //20.0045ms in timer ticks via 16 bit counter 

void usartInit(void);
int getData(char *buffer, unsigned char len);
unsigned int pulse1, pulse2, pulse3, pulse4, pulse5, pulse6, pulse7, pulse8;
void low_isr(void);
void high_isr(void);
extern void _startup (void);        // See c018i.c in your C18 compiler dir
char str[18];
char dollar;
int setLAN();
int setWiFi();
void writeByte(byte *addr,byte n);
byte readByte(byte *addr);
void step(int);

#pragma code high_vector=0x008
void interrupt_at_high_vector(void)
{
    _asm goto high_isr _endasm
}

#pragma interrupt high_isr
void high_isr(void) {
	if (INTCONbits.TMR0IF) {

		//Send Servo Pulse 1
		LATDbits.LATD1 = 1;
		Delay100TCYx(30);
		Delay100TCYx(pulse1); 
		LATDbits.LATD1 = 0;

		//Send Servo Pulse 2
		LATDbits.LATD0 = 1;
		Delay100TCYx(30);
		Delay100TCYx(pulse2); 
		LATDbits.LATD0 = 0;

		//Send Servo Pulse 3
		LATCbits.LATC2 = 1;
		Delay100TCYx(30);
		Delay100TCYx(pulse3); 
		LATCbits.LATC2 = 0;

		//Send Servo Pulse 4
		LATCbits.LATC1 = 1;
		Delay100TCYx(30);
		Delay100TCYx(pulse4); 
		LATCbits.LATC1 = 0;

		//Send Servo Pulse 5
		LATCbits.LATC0 = 1;
		Delay100TCYx(30);
		Delay100TCYx(pulse5); 
		LATCbits.LATC0 = 0;

		//Send Servo Pulse 6
		LATAbits.LATA2 = 1;
		Delay100TCYx(30);
		Delay100TCYx(pulse6); 
		LATAbits.LATA2 = 0;

		//Send Servo Pulse 7
		LATAbits.LATA1 = 1;
		Delay100TCYx(30);
		Delay100TCYx(pulse7); 
		LATAbits.LATA1 = 0;

		//Send Servo Pulse 8
		LATAbits.LATA0 = 1;
		Delay100TCYx(30);
		Delay100TCYx(pulse8); 
		LATAbits.LATA0 = 0;

	    WriteTimer0(TIMER_PERIOD);
	    INTCONbits.TMR0IF = 0;
	}
}

#pragma code
void main(void) {
	int lastState;			//holds the wifi/wired last state

	RCONbits.IPEN = 0;       // Priority Interrupt Enable (0=off)

	//Start timer 0 with interupt in 16 bit mode and a 1:8 prescale using the system clock
	OpenTimer0( TIMER_INT_ON &  T0_16BIT & T0_SOURCE_INT & T0_PS_1_8 );
	WriteTimer0( TIMER_PERIOD );// initialize the timer (value in timer ticks) 

	usartInit();

	//PWM Pins
	TRISAbits.TRISA0 = 0; //pin a0 output, output 1
	TRISAbits.TRISA1 = 0; //pin a1 output, output 2
	TRISAbits.TRISA2 = 0; //pin a2 output, output 3
	TRISCbits.TRISC0 = 0; //pin c0 output, output 4
	TRISCbits.TRISC1 = 0; //pin c1 output, output 5
	TRISCbits.TRISC2 = 0; //pin c2 output, output 6
	TRISDbits.TRISD0 = 0; //pin d0 output, output 7
	TRISDbits.TRISD1 = 0; //pin d1 output, output 8

	//Connection method switch
	TRISDbits.TRISD3 = 1; //pin d4 input, WiFi select
	TRISDbits.TRISD4 = 1; //pin d3 input, Wired select

	//LED Program status pin
	TRISDbits.TRISD2 = 0; 	//pin d2 output, program LED
	LATDbits.LATD2 = 0; 	//off

	//pre/post scale registers from right to left in increasing order
	// Timer2 prescale = 16
	T2CONbits.T2CKPS0 = 0;
	T2CONbits.T2CKPS1 = 1;
	
	// Timer2 postscale = 16
	T2CONbits.T2OUTPS0 = 1;
	T2CONbits.T2OUTPS1 = 1;
	T2CONbits.T2OUTPS2 = 1;
	T2CONbits.T2OUTPS3 = 1;

	// Timer period
	PR2 = 0xE9;  //233
	//83.3*16*16*(233+1)= 5ms

	// Timer2 is on
	T2CONbits.TMR2ON = 1;

    // Enable Timer2 interrupt
    PIE1bits.TMR2IE = 1;

	pulse1 = 150;
	pulse2 = 150;
	pulse3 = 150;
	pulse4 = 150;
	pulse5 = 150;
	pulse6 = 150;
	pulse7 = 150;
	pulse8 = 150;

	// 0 = wifi
	// 1 = wired
	lastState  = readByte(0);	//get the last state

	//If D2 is pressed on startup, set the mode to WiFi LAN
	if(!PORTDbits.RD3 && lastState != 0){
		LATDbits.LATD2 = 1; 	//on
		setWiFi();
		LATDbits.LATD2 = 0; 	//off
		
		writeByte(0,0);			//set the state to wifi
	}

	//If D2 is pressed on startup, set the mode to Wired LAN
	if(!PORTDbits.RD4 && lastState != 1){
		LATDbits.LATD2 = 1; 	//on
		setLAN();
		LATDbits.LATD2 = 0; 	//off
		
		writeByte(0,1);			//set the state to wired
	}

   	INTCONbits.GIE = 1;    // Global Interupt Enable (1 = on)

	while(1) {
		while(!DataRdyUSART());
		INTCONbits.GIE = 0;    // Global Interupt Enable (1 = on)
 		dollar = getcUSART(); 
		INTCONbits.GIE = 1;    // Global Interupt Enable (1 = on)
//		while(dollar != '0'){
		while(dollar != 0){
		   if (RCSTAbits.OERR) {
		       RCSTAbits.CREN=0;
		       RCSTAbits.CREN=1;
		   }

		    while(!DataRdyUSART());
			INTCONbits.GIE = 0;    // Global Interupt Enable (1 = on)
		    dollar = getcUSART();  
			INTCONbits.GIE = 1;    // Global Interupt Enable (1 = on)
		}

		INTCONbits.GIE = 0;    // Global Interupt Enable (1 = on)
		//getData(str,18); 		//once the '$' is found, get the rest of the string
		if( getData(str,8) == 0 ) {
			pulse1 = str[0];
			pulse2 = str[1];
			pulse3 = str[2];
			pulse4 = str[3];
			pulse5 = str[4];
			pulse6 = str[5];
			pulse7 = str[6];
			pulse8 = str[7];
			
			//LATDbits.LATD2 = 0;
		}
		INTCONbits.GIE = 1;    // Global Interupt Enable (1 = on)
	}
}

//12,000,000 ips
//12m / 1k = 12,000 = 1ms


void usartInit(void) {
	// configure USART
	OpenUSART( USART_TX_INT_OFF &
		USART_RX_INT_OFF &
		USART_ASYNCH_MODE &
		USART_EIGHT_BIT &
		USART_CONT_RX &
		USART_BRGH_LOW,
//		USART_BRGH_HIGH,
		77);

	//low + 77 = 9600 bps
	//high + 77 = 38400 bps
  
	//SPBRG = INT ( 48000000 / 9600 / 64 - 1 ) = 77
	// 77 = 9600bps
}

//get len number of bytes from the usart
//void getData(char *buffer, unsigned char len) {
int getData(char *buffer, unsigned char len) {
	char i;    // Length counter
	unsigned char data;

	for(i = 0; i < len; i++) {
		if (RCSTAbits.OERR) {
			//LATDbits.LATD2 = 1; //on
			RCSTAbits.CREN=0;
			RCSTAbits.CREN=1;
			return 1;
		}

    	while(!DataRdyUSART());	// Wait for data to be received

    	data = getcUSART();    	// Get a character from the USART
                           		// and save in the string
    	*buffer = data;
    	buffer++;              	// Increment the string pointer
	}
	return 0;
} 

//send the serial commands to set the Lantronix module to Wired mode
int setLAN(){
	int i = 0;

		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);

	//put the module in setup mode 
	for(i = 0; i < 10; i++){
		putcUSART('x');

		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
	}

	//press enter
	putcUSART(13);

	//enter into the Server menu
	putcUSART('0');

	//wait 2 seconds
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);

	//press enter
	putcUSART(13);

	//wait 2 seconds
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);

	//wired mode
	putcUSART('0');

	//wait 2 seconds
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);

	//press enter
	putcUSART(13);

	//enter key 9 times to exit the other menu items
	for(i = 0; i < 9; i++){
		putcUSART(13); 

		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
	}

	//wait 1 second
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);

	//choose 9 to save the settings and exit
	putcUSART('9');

	//press enter
	putcUSART(13);
}

//send the serial commands to set the Lantronix module to WiFi mode
int setWiFi(){
	int i = 0;

		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);

	//put the module in setup mode 
	for(i = 0; i < 10; i++){
		putcUSART('x');

		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
	}

	//press enter
	putcUSART(13);

	//enter into the Server menu
	putcUSART('0');

	//wait 2 seconds
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);

	//press enter
	putcUSART(13);

	//wait 2 seconds
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);

	//wifi mode
	putcUSART('1');

	//wait 2 seconds
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);

	//press enter
	putcUSART(13);

	//enter key 9 times to exit the other menu items
	for(i = 0; i < 9; i++){
		putcUSART(13); 

		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
		Delay10KTCYx(256);
	}

	//wait 1 second
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);
	Delay10KTCYx(256);

	//choose 9 to save the settings and exit
	putcUSART('9');

	//press enter
	putcUSART(13);
}


//write a byte to EEPROM memory
void writeByte(byte *addr, byte n) {
 
     EEADR = (byte)addr;
     EEDATA = n;
 
     EECON1bits.EEPGD = 0;      //Point to Data Memory
     EECON1bits.CFGS = 0;       //Access EEPROM
     EECON1bits.WREN = 1;
 
     //INTCONbits.GIE = 0;      //disable interrupts....NOOOOO Bad in this app so just dont write wile they are on
     EECON2 = 0x55;         	//Write 55
     EECON2 = 0xAA;				//Write AA
     EECON1bits.WR = 1; 		//enable write

     do {
     	ClrWdt();
     } while(EECON1bits.WR);	//occupied? 
 
     //INTCONbits.GIE = 1;		//turn on interrupts
     EECON1bits.WREN=0;         //disable write.
}
 

//read a byte from EEPROM memory
byte readByte(byte *addr) {
 
     EEADR =(byte)addr;			// Address to read
     EECON1bits.EEPGD = 0;      // Point to Data Memory
     EECON1bits.CFGS = 0;       // Access EEPROM
     EECON1bits.RD = 1;        	// EE Read
     return EEDATA;             // W = EEDATA
}


//47TCY
void step(int input){
	int i;
	for(i = 0; i < input; i++) {
		Delay10TCYx(4);
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
	}
}