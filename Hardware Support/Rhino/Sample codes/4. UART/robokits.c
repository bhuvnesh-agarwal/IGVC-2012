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
	
	char b = 0, l1 = 0, l2 = 0, r1 = 0, r2 = 0;
	int l, r;
	for (;;) // Loop forever 
	{
		while(b  != 'w')
			b = UART_GETCHAR();
		 		 
		 if(b == 'w')
		 {
			UART_PUTCHAR(b);
			while((l1 < '0') || (l1 > '9'))
				l1 = UART_GETCHAR();
			UART_PUTCHAR(l1);
			while((l2 < '0') || (l2 > '9'))
				l2 = UART_GETCHAR();
			UART_PUTCHAR(l2);
			while((r1 < '0') || (r1 > '9'))
				r1 = UART_GETCHAR();
			UART_PUTCHAR(r1);
			while((r2 < '0') || (r2 > '9'))
				r2 = UART_GETCHAR();
			UART_PUTCHAR(r2);
			
			cbi(PORTC, 2);
			cbi(PORTC, 3);
			
			l1 -= '0';	l2 -= '0';
			r1 -= '0';	r2 -= '0';
			
			l = (int)l1*10 + (int)l2;
			r = (int)r1*10 + (int)r2;
			
			OCR1A = (int) (100-l)*1.28;
			OCR1B = (int) (100-r)*1.28;
			
			b = l = r = 0;
			l1 = l2 = r1 = r2 = 0;
		}
	}    
}
