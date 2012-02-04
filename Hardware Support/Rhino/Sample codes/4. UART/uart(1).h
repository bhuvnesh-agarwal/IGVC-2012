#ifndef UART_H
#define UART_H

#define UART_BAUD_SEL(baudRate,xtalCpu) ((xtalCpu)/((baudRate)*16l)-1)
#define UART_BAUD_SEL_DOUBLE_SPEED(baudRate,xtalCpu) (((xtalCpu)/((baudRate)*8l)-1)|0x8000)

#define UART_RX_BUFFER 32
#define UART_TX_BUFFER 32

#if ( (UART_RX_BUFFER+UART_TX_BUFFER) >= (RAMEND-0x60 ) )
#error "size of UART_RX_BUFFER + UART_TX_BUFFER larger than size of SRAM"
#endif

/* 
** high byte error return code of UART_GETCHAR()
*/
#define UART_FRAME_ERROR      0x0800              /* Framing Error by UART       */
#define UART_OVERRUN_ERROR    0x0400              /* Overrun condition by UART   */
#define UART_BUFFER_OVERFLOW  0x0200              /* receive ringbuffer overflow */
#define UART_NO_DATA          0x0100              /* no receive data available   */


extern void UART_INIT(unsigned long baudrate);
extern unsigned int UART_GETCHAR(void);
extern void UART_PUTCHAR(unsigned char data);
extern void UART_PRINT(const char *s );

#endif // UART_H 

