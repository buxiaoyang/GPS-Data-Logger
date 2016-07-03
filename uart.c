/*------------------------------------------------*/
/* ATmega48/88/168/328 USART0 functions           */
/*------------------------------------------------*/


#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"

#define	SZ_BUF		512		/* Size of receiving FIFO [byte] */


typedef struct _fifo {
	uint16_t	wi;
	uint16_t	ri;
	uint16_t	ct;
	uint8_t 	buff[SZ_BUF];
} FIFO;
volatile FIFO RxFifo;


/*----------------------------------------------------*/
/* USART control                                      */
/*----------------------------------------------------*/


void uart_init (void)
{
	cli();
	UCSR0B = 0;
	RxFifo.ri = 0;
	RxFifo.wi = 0;
	RxFifo.ct = 0;
	UBRR0L = F_CPU / 16 / UART_BAUD - 1;	/* Enable USRAT0 in format of N81 */
	UCSR0B = _BV(RXCIE0)|_BV(RXEN0)|_BV(TXEN0);
	sei();
}



/* Get a received character */
uint8_t uart_get (void)	/* Return 0 if no data */
{
	uint16_t i;
	uint8_t d;


	cli();
	i = RxFifo.ct;
	sei();
	d = 0;
	if (i) {
		i = RxFifo.ri;
		d = RxFifo.buff[i++];
		RxFifo.ri = i % sizeof RxFifo.buff;
		cli();
		RxFifo.ct--;
		sei();
	}

	return d;
}


/* Send a byte */
void uart_put (uint8_t d)
{
	while (bit_is_clear(UCSR0A, UDRE0)) ;
	UDR0 = d;
}


/* USART0 RXC interrupt */
ISR(USART_RX_vect)
{
	uint8_t d;
	uint16_t i;


	d = UDR0;
	i = RxFifo.ct;
	if (i < sizeof RxFifo.buff) {
		RxFifo.ct = i + 1;
		i = RxFifo.wi;
		RxFifo.buff[i++] = d;
		RxFifo.wi = i % sizeof RxFifo.buff;
	}
}

