#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/wdt.h>
#include "usart.h"
#include "ds18b20.h"

#define OUT_B_PORT		PORTB
#define LED_Pin			PB0
#define BLOCK           PB1
#define ON_ING			PB2
#define STARTER			PB3
#define Blink_RELAY		PB4
#define VOICE_RELAY		PB5

#define Diable_SECURITY    PC0
#define Enable_SECURITY		PC1
#define Ignition_IN			PC2
#define Temp_Pin			PC3
#define Door_Pin		PC4
//#define ACC_Pin			PC5

#define AUTOMAT			PD0
#define SHOCK_Sensor_Lite	PD2
#define SHOCK_Sensor_Hard	PD3
#define STOP_Pin		PD4
#define BUTTON          PD5
#define GENERATOR		PD6
#define NAKAL_ON_OFF	PD7

#define setBit(sfr, bit)     (_SFR_BYTE(sfr) |= (1 << bit))
#define clearBit(sfr, bit)   (_SFR_BYTE(sfr) &= ~(1 << bit))
#define toggleBit(sfr, bit) (_SFR_BYTE(sfr) ^= (1 << bit))
//#define getBit(sfr, bit)	(_SFR_BYTE(sfr) & (1<<(bit)))

#define PERIOD_PRESETS 15 //sec
#define MAX_TIME_SHOCK 7 //sec
#define LOCK_TIME_SHOCK 10 //sec
#define MAX_CNT_SHOCK 5 //cnt
#define ALARM_TIMER 90 //sec
#define HEATING_TIMER 900 //sec
#define BTN_DEBOUCE 200 //ms
#define DELAY_START_SECURITY 2
uint8_t heating = 0;
uint8_t security = 0;
uint8_t allPresets;
uint8_t startOK;
uint8_t flagAlarm;
uint8_t flagBlinkShock;
uint8_t flag2Vibro1 = 0;
uint8_t autoTransmission = 0;
uint8_t flagNakal = 0;
uint8_t flagOnePresets = 0;
uint8_t flagTwoPresets = 0;
uint32_t timeAlarmStart;			// save start alarm
uint32_t timeStarterON, timerHeating;
uint32_t timerOnePresets;			//timer one preset
uint32_t timeOverCnt = 0;
uint32_t timeStartSecurity = 0;
uint8_t tempFlagSec = 0;

volatile uint8_t cntVibro1 = 0;
volatile uint8_t flagVibro1 = 0;
volatile uint8_t timerBlink;
volatile uint8_t timerBlinkSec = 0;
volatile uint8_t Timer_alarm;
volatile uint32_t timeStartFirstCnt = 0;
volatile uint32_t seconds = 0;
volatile uint32_t milliseconds = 0;
volatile uint32_t tempmilliseconds = 0;


long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void timer0_init() // initialize timer0, interrupt and variable
{
	TCCR0 |= (1 << CS01) | (1 << CS00);
	TIMSK |= (1 << TOIE0);
}
void timer1_init()
{
	TCCR1B |= (1 << WGM12)|(1 << CS12);
	TCNT1 = 0;
	OCR1A = 31250;// 500 ms
	TIMSK |= (1 << OCIE1A);
}
void timer2_init() // initialize timer2, interrupt and variable
{
	OCR2 = 162;
	TCCR2 |= (1 << WGM21);// Set to CTC Mode
	TIMSK |= (1 << OCIE2); //Set interrupt on compare match
	TCCR2 |= (1 << CS20) | (1 << CS21) | (1 << CS22);
}
uint32_t millis_get()
{
	uint32_t millisec;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		millisec = milliseconds;
	}
	return millisec;
}
uint32_t seconds_get()
{
	uint32_t sec;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		sec = seconds;
	}
	return sec;
}
uint8_t poll_DoorClose()		//declaring a debounce function
{
	if (!bit_is_clear(PINC, Door_Pin)) {      /* button is pressed now */
		_delay_ms(200);
		if (!bit_is_clear(PINC, Door_Pin)) {            /* still pressed */
			return (1);
		}
	}
	return(0);
}
uint8_t poll_DoorOpen()		//declaring a debounce function
{
	if (bit_is_clear(PINC, Door_Pin)) {      /* button is pressed now */
		_delay_ms(200);
		if (bit_is_clear(PINC, Door_Pin)) {            /* still pressed */
			return (1);
		}
	}
	return(0);
}
uint8_t poll_btn() {
	if (bit_is_clear(PIND, BUTTON)) {      /* button is pressed now */
		_delay_ms(20);
		if (bit_is_clear(PIND, BUTTON)) {            /* still pressed */
			return (1);
		}
	}
	return(0);
}
double TempRead(){
	TSDS18x20 DS18x20;
	TSDS18x20 *pDS18x20 = &DS18x20;
	DS18x20_Init(pDS18x20,&PORTC,Temp_Pin);
	// Set DS18B20 resolution to 9 bits.
	DS18x20_SetResolution(pDS18x20,CONF_RES_9b);
	DS18x20_WriteScratchpad(pDS18x20);
	DS18x20_MeasureTemperature(pDS18x20);
	return(DS18x20_TemperatureValue(pDS18x20));
}
void SecurityON(){
	timeStartSecurity = seconds_get();
	tempFlagSec = 1;
	cntVibro1 = 0;
	timeStartFirstCnt = 0;
	flagVibro1 = 0;	
	USART_SendString("Press SecurityON\r\n");	
	if (bit_is_clear(PIND, GENERATOR))
	{
		setBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
	} else{
		setBit(GICR, INT0);
		setBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(500);
		clearBit(OUT_B_PORT, Blink_RELAY);	
	}
}
void SecurityOFF(){
	clearBit(OUT_B_PORT, Blink_RELAY);
	clearBit(OUT_B_PORT, VOICE_RELAY);
	clearBit(OUT_B_PORT, LED_Pin);
	clearBit(GICR, INT0);
	clearBit(OUT_B_PORT, BLOCK);
	security = 0;
	flagAlarm = 0;
	cntVibro1 = 0;
	timeStartFirstCnt = 0;
	flagVibro1 = 0;
	tempFlagSec = 0;
	setBit(OUT_B_PORT, Blink_RELAY);
	_delay_ms(300);
	clearBit(OUT_B_PORT, Blink_RELAY);
	_delay_ms(300);
	setBit(OUT_B_PORT, Blink_RELAY);
	_delay_ms(300);
	clearBit(OUT_B_PORT, Blink_RELAY);
	if (bit_is_clear(PIND, GENERATOR))
	{
		setBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(300);
		setBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
	}
	USART_SendString("Security off\r\n");
}
void ResetPresets(){
	allPresets = 0;
	flagOnePresets = 0;
	flagTwoPresets = 0;
	clearBit(OUT_B_PORT, Blink_RELAY);
	clearBit(OUT_B_PORT, VOICE_RELAY);
	clearBit(OUT_B_PORT, ON_ING);
}
void PresetsManual() {
	if (allPresets == 0 && flagOnePresets == 1 && (seconds_get() > timerOnePresets + PERIOD_PRESETS)){
		ResetPresets();
		USART_SendString("All reset timer\r\n");
		setBit(OUT_B_PORT, LED_Pin);
		_delay_ms(300);
		clearBit(OUT_B_PORT, LED_Pin);
	}
	if (flagOnePresets == 0 && bit_is_clear(PIND, STOP_Pin) && bit_is_clear(PINC, Ignition_IN) && bit_is_clear(PIND, GENERATOR) && poll_DoorClose())
	{
		setBit(OUT_B_PORT, ON_ING);
		flagOnePresets = 1;
		timerOnePresets = seconds_get();
		USART_SendString("One\r\n");
	}
	if (flagOnePresets == 1 && flagTwoPresets == 0 && bit_is_clear(PIND, STOP_Pin) && poll_DoorOpen())
	{
		flagTwoPresets = 1;
		USART_SendString("Two \r\n");
	}
	if (flagTwoPresets == 1 && allPresets == 0 && bit_is_clear(PIND, STOP_Pin) && poll_DoorClose())
	{
		allPresets = 1;
		flagOnePresets = 0;
		clearBit(OUT_B_PORT, ON_ING);
		setBit(OUT_B_PORT, LED_Pin);
		_delay_ms(300);
		clearBit(OUT_B_PORT, LED_Pin);
		USART_SendString("All presets \r\n");
	}
	if (flagOnePresets == 1 && (!bit_is_clear(PIND, STOP_Pin) || !bit_is_clear(PIND, GENERATOR)))
	{
		ResetPresets();
		USART_SendString("Manual reset\r\n");
	}
	if (allPresets == 1 && poll_DoorOpen())
	{
		ResetPresets();
		USART_SendString("Door reset\r\n");
	}

}
void HeatingStop() {
	clearBit(OUT_B_PORT, ON_ING);
	clearBit(OUT_B_PORT, STARTER);
	clearBit(OUT_B_PORT, Blink_RELAY);
	clearBit(OUT_B_PORT, VOICE_RELAY);
	startOK = 0;
	//USART_SendString("HeatingStop\r\n");
	clearBit(OUT_B_PORT, LED_Pin);
	timerHeating = 0;
	timeStarterON = 0;
}
void EngineStart(int cnt) {
	uint8_t count = 0;
	uint8_t maxTimeStarter = 5; //starter operating time sec
	clearBit(OUT_B_PORT, BLOCK);
	wdt_disable();
	while (count < cnt && !bit_is_clear(PINC, Ignition_IN) && bit_is_clear(PIND, STOP_Pin) && !bit_is_clear(PIND, GENERATOR)){
		count++;
		setBit(OUT_B_PORT, LED_Pin);
		setBit(OUT_B_PORT, ON_ING);         //on ignition 4 sec
		_delay_ms(500);
		clearBit(OUT_B_PORT, LED_Pin);
		_delay_ms(2000);
		if (flagNakal == 1)//Glow plugs. Quantity by temperature. 2 sec off ignition, 6 sec on
		{
			//USART_SendString("Glow plugs on\r\n");
			double temp = 0;
			temp = TempRead();
			int z = map(temp, 0, -25, 0, 5);
			for(; z > 0 ; z--){
				clearBit(OUT_B_PORT, ON_ING);
				_delay_ms(1900);
				if (poll_btn())
				{
					heating = 0;
					HeatingStop();
				}
				setBit(OUT_B_PORT, ON_ING);
				_delay_ms(6000);
				if (poll_btn())
				{
					heating = 0;
					HeatingStop();
				}
			}
		}
		if(bit_is_clear(PIND, STOP_Pin)) //If switch is pressed
		{
			setBit(OUT_B_PORT, Blink_RELAY);
			setBit(OUT_B_PORT, VOICE_RELAY);
			_delay_ms(500);
			clearBit(OUT_B_PORT, Blink_RELAY);
			clearBit(OUT_B_PORT, VOICE_RELAY);
			_delay_ms(500);
			setBit(OUT_B_PORT, Blink_RELAY);
			setBit(OUT_B_PORT, VOICE_RELAY);
			_delay_ms(500);
			clearBit(OUT_B_PORT, Blink_RELAY);
			clearBit(OUT_B_PORT, VOICE_RELAY);
			timeStarterON = seconds_get();
			
			setBit(OUT_B_PORT, STARTER);
			//USART_SendString("starter on\r\n");
			} else {
			HeatingStop();
			heating = 0;
			count = 0;
			break;
		}
		
		while (!bit_is_clear(PIND, GENERATOR) && (seconds_get() < (timeStarterON + maxTimeStarter)) && bit_is_clear(PIND, STOP_Pin)){}
		_delay_ms (100);
		clearBit(OUT_B_PORT, STARTER);
		if (bit_is_clear(PIND, GENERATOR))
		{
			timerHeating = seconds_get();
			startOK = 1;
			Timer_alarm = 0;
			clearBit(OUT_B_PORT, Blink_RELAY);
			wdt_enable(WDTO_2S);
			break;
		}
		//USART_SendString("starter off, wait 6 sec\r\n");
		maxTimeStarter = maxTimeStarter + 1;                             // + 1 sec
		HeatingStop();
		_delay_ms (5000);
	}
	if (startOK == 0)
	{
		//USART_SendString("heating = 0\r\n");
		heating = 0;
		clearBit(OUT_B_PORT, Blink_RELAY);
		clearBit(OUT_B_PORT, VOICE_RELAY);
		if (security == 1)
		{
			setBit(GICR, INT0);
			setBit(OUT_B_PORT, BLOCK);
		}
	}
	wdt_enable(WDTO_2S);
	//USART_SendString("quit of starting\r\n");
}
void Alarm(){
	timeAlarmStart = seconds_get();
	flagAlarm = 1;
	startOK = 0;
	timerHeating = 0;
	heating = 0;
	flagBlinkShock = 0;
	Timer_alarm = 0;
	clearBit(OUT_B_PORT, Blink_RELAY);
	clearBit(OUT_B_PORT, VOICE_RELAY);
	clearBit(OUT_B_PORT, STARTER);
	clearBit(OUT_B_PORT, ON_ING);
	setBit(OUT_B_PORT, BLOCK);
	clearBit(GICR, INT0); // off vibro1

}
void ShockSensor(){
	if (!flagAlarm && flagVibro1 == 0 && bit_is_clear(PIND, SHOCK_Sensor_Hard))
	{
		Alarm();
	}
	if (flagVibro1 == 1)
	{
		if (!flagAlarm && bit_is_clear(PIND, SHOCK_Sensor_Hard))
		{
			Alarm();
		}
		if (cntVibro1 > 0 && cntVibro1 < MAX_CNT_SHOCK && flagBlinkShock == 1)
		{
			flagBlinkShock = 0;
			setBit(OUT_B_PORT, BLOCK);
			setBit(OUT_B_PORT, Blink_RELAY);
			setBit(OUT_B_PORT, VOICE_RELAY);
			_delay_ms(300);
			clearBit(OUT_B_PORT, Blink_RELAY);
			clearBit(OUT_B_PORT, VOICE_RELAY);
			setBit(GICR, INT0); // on vibro1
			USART_SendString("shock\r\n");
		}
		if (cntVibro1 >= MAX_CNT_SHOCK && (seconds_get() > timeStartFirstCnt + MAX_TIME_SHOCK)){
			timeOverCnt = seconds_get();
			//cntVibro1 = 0;
			//timeStartFirstCnt = 0;
			flagVibro1 = 0;
			flag2Vibro1 = 1;
			clearBit(GICR, INT0);
			USART_SendString("lite off\r\n");
		}
	}

	if (flag2Vibro1 == 1 && (seconds_get() > timeOverCnt + LOCK_TIME_SHOCK))
	{
		timeOverCnt = 0;
		flag2Vibro1 = 0;
		timeStartFirstCnt = 0;
		cntVibro1 = 0;
		setBit(GICR, INT0);
		USART_SendString("lite on\r\n");
	}
	
}

int main(void){
	DDRB |= _BV(BLOCK);//out
	DDRB |= _BV(ON_ING);
	DDRB |= _BV(STARTER);
	DDRB |= _BV(Blink_RELAY);
	DDRB |= _BV(LED_Pin);
	DDRB |= _BV(VOICE_RELAY);
	
	DDRD &= ~_BV(AUTOMAT);//in
	DDRD &= ~_BV(STOP_Pin);
	DDRD &= ~_BV(BUTTON);
	DDRD &= ~_BV(GENERATOR);
	DDRD &= ~_BV(SHOCK_Sensor_Lite);
	DDRD &= ~_BV(SHOCK_Sensor_Hard);
	DDRD &= ~_BV(NAKAL_ON_OFF);
	
	DDRC &= ~_BV(Ignition_IN);
	DDRC &= ~_BV(Diable_SECURITY);
	DDRC &= ~_BV(Enable_SECURITY);
	DDRC &= ~_BV(Door_Pin);
	//DDRC &= ~_BV(ACC_Pin);
	
	PORTB &= ~(1 << BLOCK) | ~(1 << ON_ING) | ~(1 << STARTER) | ~(1 << Blink_RELAY) | ~(1 << LED_Pin) | ~(1 << VOICE_RELAY); //0
	PORTD  |= (1 << AUTOMAT) | (1 << STOP_Pin) | (1 << BUTTON) | (1 << GENERATOR) | (1 << SHOCK_Sensor_Lite) | (1 << SHOCK_Sensor_Hard) | (1 << NAKAL_ON_OFF);//1
	PORTC  |= (1 << Diable_SECURITY) | (1 << Enable_SECURITY) | (1 << Door_Pin)/* | (1 << ACC_Pin)*/ | (1 << Ignition_IN);
	timer0_init();
	timer1_init();
	timer2_init();
	USART_Init();
	MCUCR |= (1 << ISC01);
	wdt_enable(WDTO_2S);
	sei();
	//USART_SendString("Start\r");
	if (!bit_is_clear(PIND, AUTOMAT))
	{
		autoTransmission = 1;
	}
	if (!bit_is_clear(PIND, NAKAL_ON_OFF))
	{
		flagNakal = 1;
	}
	setBit(OUT_B_PORT, LED_Pin);
	_delay_ms(500);
	clearBit(OUT_B_PORT, LED_Pin);
	for (;;)
	{
		wdt_reset();
		if (heating == 1)
		{
			if (seconds_get() > timerHeating + HEATING_TIMER || !bit_is_clear(PIND, STOP_Pin) || poll_btn() || (startOK && !bit_is_clear(PIND, GENERATOR))){
				HeatingStop();
				//USART_SendString("Heating stop timer\r\n");
				heating = 0;
				if (security == 1)
				{
					setBit(GICR, INT0);
					setBit(OUT_B_PORT, BLOCK);
				}
			}
			if (security == 1)
			{
				if (!flagAlarm && poll_DoorOpen())
				{
					Alarm();
				}
				if (bit_is_clear(PINC, Diable_SECURITY))
				{
					SecurityOFF();
				}
				if (flagAlarm == 1 && seconds_get() > timeAlarmStart + ALARM_TIMER)
				{
					timeAlarmStart = 0;
					flagAlarm = 0;
					clearBit(OUT_B_PORT, Blink_RELAY);
					clearBit(OUT_B_PORT, VOICE_RELAY);
					clearBit(OUT_B_PORT, LED_Pin);
					setBit(GICR, INT0);
				}
			}
			if (!security)
			{
				if (!tempFlagSec && bit_is_clear(PINC, Enable_SECURITY) && poll_DoorClose())
				{
					SecurityON();

				}
				if (tempFlagSec && (seconds_get() > timeStartSecurity + DELAY_START_SECURITY))
				{
					tempFlagSec = 0;
					security = 1;
					USART_SendString("SecurityON\r\n");
				}
				if (tempFlagSec && bit_is_clear(PINC, Diable_SECURITY))
				{
					SecurityOFF();
				}
			}
		}
		if (heating == 0)
		{
			if (!autoTransmission)
			{
				PresetsManual();
			}
			if (allPresets == 1 && !flagAlarm && poll_btn() && !bit_is_clear(PINC, Ignition_IN) && !bit_is_clear(PIND, GENERATOR)) {
				heating = 1;
				clearBit(GICR, INT0);
				//USART_SendString("command - start!\r\n");
				EngineStart(3);
			}
			if (security == 1)
			{
				ShockSensor();
				if (!flagAlarm && poll_DoorOpen())
				{
					Alarm();
				}
				if (flagAlarm && seconds_get() > timeAlarmStart + ALARM_TIMER)
				{
					timeAlarmStart = 0;
					flagAlarm = 0;
					clearBit(OUT_B_PORT, Blink_RELAY);
					clearBit(OUT_B_PORT, VOICE_RELAY);
					clearBit(OUT_B_PORT, LED_Pin);
					setBit(GICR, INT0);
				}
				if (bit_is_clear(PINC, Diable_SECURITY))
				{
					SecurityOFF();
				}
			}
			if (!security)
			{
				if (!tempFlagSec && !bit_is_clear(PINC, Ignition_IN) && bit_is_clear(PINC, Enable_SECURITY) && poll_DoorClose())
				{
					SecurityON();

				}
				if (tempFlagSec && (seconds_get() > timeStartSecurity + DELAY_START_SECURITY))
				{
					tempFlagSec = 0;
					security = 1;
					USART_SendString("SecurityON\r\n");
				}
				if (tempFlagSec && bit_is_clear(PINC, Diable_SECURITY))
				{
					SecurityOFF();
				}
			}
		}
	}
}

ISR (INT0_vect) //sensor lite
{
	if (heating == 0){
		clearBit(GICR, INT0);
		//wdt_reset();
		uint16_t timer = 0;
		while(bit_is_clear(PIND, PD2)) { // button hold down
			timer++; // count how long button is pressed
			_delay_ms(1);
			if (timer > 800UL)
			{
				break;
			}
		}
		if(timer > BTN_DEBOUCE) { // software debouncing button
				++cntVibro1;
				flagBlinkShock = 1;
				if (cntVibro1 == 1)
				{
					timeStartFirstCnt = seconds_get();
					flagVibro1 = 1;
				}
		}
	}
}
ISR(TIMER0_OVF_vect){	//alarm
	TCNT0 += 6;
	milliseconds ++;
	tempmilliseconds ++;
	if (tempmilliseconds > 999)
	{
		seconds++;
		tempmilliseconds = 0;
		USART_Write_Int(cntVibro1);
		USART_SendString("\r\n");
	}
}
ISR(TIMER1_COMPA_vect)
{
	if (security)
	{
		toggleBit(OUT_B_PORT, LED_Pin);
	}
	if (flagAlarm)
	{
		toggleBit(OUT_B_PORT, Blink_RELAY);
		toggleBit(OUT_B_PORT, VOICE_RELAY);
	}
	else if (startOK)
	{
		toggleBit(OUT_B_PORT, Blink_RELAY);
		Timer_alarm = 0;
	}
}
ISR (TIMER2_COMP_vect)
{	
}
