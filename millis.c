#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>
#include "millis.h"

// Timer1

// 1KHz - 65.28MHz
#define CLOCKSEL (_BV(CS10))
#define PRESCALER 1

#define REG_TCCRA		TCCR1A
#define REG_TCCRB		TCCR1B
#define REG_TIMSK		TIMSK1
#define REG_OCR			OCR1A
#define BIT_WGM			WGM12
#define BIT_OCIE		OCIE1A
#define ISR_VECT		TIMER1_COMPA_vect


#define SET_TCCRA()	(REG_TCCRA = 0)
#define SET_TCCRB()	(REG_TCCRB = _BV(BIT_WGM)|CLOCKSEL)


static volatile millis_t milliseconds;

// Initialise library
void millis_init()
{
	// Timer settings
	SET_TCCRA();
	SET_TCCRB();
	REG_TIMSK = _BV(BIT_OCIE);
	REG_OCR = ((F_CPU / PRESCALER) / 1000);
}

// Get current milliseconds
millis_t millis_get()
{
	millis_t ms;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		ms = milliseconds;
	}
	return ms;
}


// Reset milliseconds count to 0
void millis_reset()
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		milliseconds = 0;
	}
}

// Add time
void millis_add(millis_t ms)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		milliseconds += ms;
	}
}

// Subtract time
void millis_subtract(millis_t ms)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		milliseconds -= ms;
	}
}

ISR(ISR_VECT)
{
	++milliseconds;
}