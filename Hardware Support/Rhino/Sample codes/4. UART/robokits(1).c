#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <compat/deprecated.h>
#include <avr/interrupt.h>
#include "uart.h"

#define UART_BAUD_RATE 9600 

int main(void)
{
	sbi(DDRC, 6);
	sbi(DDRC, 7);
	sbi(PORTD, 6);
	sei();

	UART_INIT(UART_BAUD_RATE);	
	
	TCCR1A = _BV(WGM10) | _BV(COM1A1) | _BV(COM1B1);    	// enable 8 bit PWM, select inverted PWM
	TCCR1B = _BV(CS11) | _BV(WGM12);
	OCR1A = 0;
	OCR1B = 0;
	sbi(PORTC,2);
	sbi(PORTC,3);
	sbi(DDRD,4);
	sbi(DDRD,5);
	sbi(DDRC,2);
	sbi(DDRC,3);
	
	char b = 0, l = 0, r = 0;
	for (;;) // Loop forever 
	{
		while(b  != 'w')
			b = UART_GETCHAR();
		 		 
		 if(b == 'w')
		 {
			UART_PUTCHAR(b);
			while((l < '1') || (l > '9'))
				l = UART_GETCHAR();
			UART_PUTCHAR(l);
			while((r < '1') || (r > '9'))
				r = UART_GETCHAR();
			UART_PUTCHAR(r);
			
			cbi(PORTC, 2);
			cbi(PORTC, 3);
			
			OCR1A = (int) (10-l+'0')*12.8;
			OCR1B = (int) (10-r+'0')*12.8;
			
			b = l = r = 0;
		}
	}    
}