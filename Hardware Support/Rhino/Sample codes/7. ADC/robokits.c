#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>				
#include <compat/deprecated.h>		
#include <avr/interrupt.h>

#include "lcd.h"

void LED1ON(void) {sbi(PORTC,7);}
void LED1OFF(void){cbi(PORTC,7);}
void TOGGLELED1(void) {if(bit_is_set(PORTC,7))cbi(PORTC,7); else sbi(PORTC,7);}

void LED2ON(void) {sbi(PORTC,6);}
void LED2OFF(void) {cbi(PORTC,6);}
void TOGGLELED2(void) {if(bit_is_set(PORTC,6))cbi(PORTC,6); else sbi(PORTC,6);}

char SWITCH1ON(void) {return(bit_is_clear(PIND,6));}
char SWITCH1OFF(void) {return(bit_is_set(PIND,6));}


int main(void)
{
    char buffer[7];
    int  num;

	sbi(DDRC,6);
	sbi(DDRC,7);
	sbi(PORTD,6);
	sei();
	
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);		

	LED1ON();
	LCD_INIT(LCD_DISP_ON);	
	

	
	while(1)
	{
		ADMUX=0;		
		ADCSRA |= _BV(ADSC);	
		while (ADCSRA & _BV(ADSC))
		{
		
		}
		num=ADCW;
        LCD_CLRSCR();
		itoa( num , buffer, 10);
        LCD_PRINT(buffer);
		_delay_ms(100);		
	}
}

