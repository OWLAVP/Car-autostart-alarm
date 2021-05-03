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
#define ACC_Pin			PC5
#define RESET_Pin		PC6

#define AUTOMAT			PD0
#define SHOCK_Sensor_Lite	PD2
#define SHOCK_Sensor_Hard	PD3
#define STOP_Pin		PD4
#define BUTTON          PD5
#define GENERATOR		PD6
#define NAKAL_ON_OFF	PD7

#define MEMORY_LITE 1 //shock sensor lite
#define MEMORY_HARD 2 // shock sensor hard
#define MEMORY_DOOR 3 // sensor door
#define setBit(sfr, bit)     (_SFR_BYTE(sfr) |= (1 << bit))
#define clearBit(sfr, bit)   (_SFR_BYTE(sfr) &= ~(1 << bit))
#define toggleBit(sfr, bit) (_SFR_BYTE(sfr) ^= (1 << bit))
//#define getBit(sfr, bit)	(_SFR_BYTE(sfr) & (1<<(bit)))

#define PERIOD_PRESETS 20 //sec
#define LOCK_TIME_SHOCK 10 //sec
#define MAX_CNT_SHOCK 4 //cnt
#define ALARM_TIMER 90 //sec
#define HEATING_TIMER 900 //sec
#define BTN_DEBOUCE 10 //
#define DEBOUNCECYCLES 5 //5 cycles. timer 16 ms
#define DELAY_START_SECURITY 10

uint8_t heating = 0;
uint8_t security = 0;
uint8_t allPresets;
uint8_t startOK;
uint8_t flagAlarm;
uint8_t flagBlinkShock;
uint8_t autoTransmission = 0;
uint8_t flagNakal = 0;
uint8_t flagOnePresets = 0;
uint8_t flagTwoPresets = 0;
uint8_t tempFlagSec = 0;
uint8_t memory = 0;
uint32_t timeAlarmStart;			// save start alarm
uint32_t timerHeating;
uint32_t timerOnePresets;			//timer one preset
uint32_t timeStartSecurity = 0;
uint32_t timeStartFirstCnt = 0;

volatile uint8_t cntShock1 = 0;
volatile uint8_t cntTimer0 = 0;
volatile uint8_t cntTimer2 = 0;
volatile uint32_t seconds = 0;
volatile uint8_t integrator;
uint8_t output;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void timer0_init() // initialize timer0, interrupt and variable
{
	TCCR0A = 0;
	TCCR0B = 0;
	TCNT0 = 0;
	// 62.5 Hz (1000000/((249+1)*64))
	OCR0A = 249;
	TCCR0A |= (1 << WGM01);
	TCCR0B |= (1 << CS02) | (1 << CS00);
	TIMSK0 |= (1 << OCIE0A);
}
void timer1_init()
{
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;
	OCR1A = 0x3D08;
	TCCR1B |= (1 << WGM12);
	TIMSK1 |= (1 << OCIE1A);
	TCCR1B |= (1 << CS12) | (1 << CS10);
	// set prescaler to 1024 and start the timer
}
void timer2_init() // initialize timer2, interrupt and variable
{
	TCCR2A = 0;
	TCCR2B = 0;
	TCNT2 = 0;
	OCR2A = 249;
	TCCR2A |= (1 << WGM21);
	TCCR2B |= (1 << CS22) | (1 << CS20) | (1 << CS21);
	TIMSK2 |= (1 << OCIE2A);
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
uint8_t DoorClose()		//declaring a debounce function
{
	if (!bit_is_clear(PINC, Door_Pin)) {      /* button is pressed now */
		_delay_ms(100);
		if (!bit_is_clear(PINC, Door_Pin)) {            /* still pressed */
			return (1);
		}
	}
	return(0);
}
uint8_t DoorOpen()		//declaring a debounce function
{
	if (bit_is_clear(PINC, Door_Pin)) {      /* button is pressed now */
		_delay_ms(100);
		if (bit_is_clear(PINC, Door_Pin)) {            /* still pressed */
			return (1);
		}
	}
	return(0);
}
uint8_t BtnStart() {
	if (bit_is_clear(PIND, BUTTON)) {      /* button is pressed now */
		_delay_ms(300);
		if (bit_is_clear(PIND, BUTTON)) {            /* still pressed */
			return (1);
		}
	}
	return(0);
}
uint8_t Handbrake() {
	if (integrator == 0) {
		output = 0;
	}
	else if (integrator >= DEBOUNCECYCLES) {
		integrator = DEBOUNCECYCLES;
		output = 1;
	}
	return(output);
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
	cntShock1 = 0;
	if (bit_is_clear(PIND, GENERATOR))
	{
		setBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
		} else{
		setBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, Blink_RELAY);
	}
	//USART_SendString("Press SecurityON\r\n");
}
void SecurityOFF(){
	clearBit(OUT_B_PORT, Blink_RELAY);
	clearBit(OUT_B_PORT, VOICE_RELAY);
	clearBit(OUT_B_PORT, LED_Pin);
	clearBit(EIMSK, INT0);
	clearBit(OUT_B_PORT, BLOCK);
	security = 0;
	flagAlarm = 0;
	cntShock1 = 0;
	tempFlagSec = 0;
	wdt_reset();
	if (memory == MEMORY_LITE)
	{
		setBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(600);
		clearBit(OUT_B_PORT, VOICE_RELAY);
	}
	else if (memory == MEMORY_HARD)
	{
		setBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(300);
		setBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
	}
	else if (memory == MEMORY_DOOR)
	{
		setBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(300);
		setBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(300);
		setBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
		} else if (bit_is_clear(PIND, GENERATOR)){
		setBit(OUT_B_PORT, VOICE_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
		} else {
		setBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(300);
		setBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, Blink_RELAY);
	}
	memory = 0;
	//USART_SendString("Security off\r\n");
}
void ResetPresets(){
	allPresets = 0;
	flagOnePresets = 0;
	flagTwoPresets = 0;
	clearBit(OUT_B_PORT, Blink_RELAY);
	clearBit(OUT_B_PORT, VOICE_RELAY);
	clearBit(OUT_B_PORT, ON_ING);
}
void PresetsMT() {
	if (!allPresets && flagOnePresets && (seconds_get() > timerOnePresets + PERIOD_PRESETS)){
		ResetPresets();
		//USART_SendString("All reset timer\r\n");
	}
	if (!flagOnePresets && bit_is_clear(PINC, Ignition_IN) && bit_is_clear(PIND, GENERATOR) && DoorClose() && Handbrake())
	{
		setBit(OUT_B_PORT, ON_ING);
		flagOnePresets = 1;
		timerOnePresets = seconds_get();
		//USART_SendString("One\r\n");
	}
	if (flagOnePresets && !flagTwoPresets && bit_is_clear(PIND, STOP_Pin) && DoorOpen())
	{
		flagTwoPresets = 1;
		//USART_SendString("Two \r\n");
	}
	if (flagTwoPresets && !allPresets && bit_is_clear(PIND, STOP_Pin) && DoorClose())
	{
		allPresets = 1;
		flagOnePresets = 0;
		clearBit(OUT_B_PORT, ON_ING);
		setBit(OUT_B_PORT, LED_Pin);
		_delay_ms(300);
		clearBit(OUT_B_PORT, LED_Pin);
	}
	if (flagOnePresets && (!Handbrake() || !bit_is_clear(PIND, GENERATOR)))
	{
		ResetPresets();
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
	_delay_ms(500);
}
void EngineStart(int cnt) {
	uint8_t count = 0;
	uint8_t maxTimeStarter = 5; //starter operating time sec
	uint32_t timeStarterON;
	
	clearBit(OUT_B_PORT, BLOCK);
	while (count < cnt && !bit_is_clear(PINC, Ignition_IN) && bit_is_clear(PIND, STOP_Pin) && !bit_is_clear(PIND, GENERATOR)){
		wdt_reset();
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
				wdt_reset();
				clearBit(OUT_B_PORT, ON_ING);
				_delay_ms(2000);
				if (BtnStart())
				{
					heating = 0;
					timeStarterON = 0;
					HeatingStop();
				}
				wdt_reset();
				setBit(OUT_B_PORT, ON_ING);
				_delay_ms(5000);
				if (BtnStart())
				{
					heating = 0;
					timeStarterON = 0;
					HeatingStop();
				}
			}
		}
		if(Handbrake()) //If switch is pressed
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
			timeStarterON = 0;
			heating = 0;
			count = 0;
			break;
		}
		wdt_disable();
		while (!bit_is_clear(PIND, GENERATOR) && (seconds_get() < (timeStarterON + maxTimeStarter)) && bit_is_clear(PIND, STOP_Pin)){}
		_delay_ms (100);
		wdt_enable(WDTO_8S);
		clearBit(OUT_B_PORT, STARTER);
		if (bit_is_clear(PIND, GENERATOR))
		{
			timerHeating = seconds_get();
			startOK = 1;
			//USART_SendString("startOK\r\n");
			clearBit(OUT_B_PORT, Blink_RELAY);
			break;
		}
		//USART_SendString("starter off, wait 6 sec\r\n");
		maxTimeStarter = maxTimeStarter + 1;                             // + 1 sec
		wdt_reset();
		HeatingStop();
		_delay_ms (5000);
	}
	if (startOK == 0)
	{
		heating = 0;
		clearBit(OUT_B_PORT, Blink_RELAY);
		clearBit(OUT_B_PORT, VOICE_RELAY);
	}
	//USART_SendString("quit\r\n");
}
void Alarm(){
	timeAlarmStart = seconds_get();
	flagAlarm = 1;
	startOK = 0;
	timerHeating = 0;
	heating = 0;
	flagBlinkShock = 0;
	clearBit(OUT_B_PORT, Blink_RELAY);
	clearBit(OUT_B_PORT, VOICE_RELAY);
	clearBit(OUT_B_PORT, STARTER);
	clearBit(OUT_B_PORT, ON_ING);
	setBit(OUT_B_PORT, BLOCK);
	clearBit(EIMSK, INT0); // off shock sensor lite
}
void ShockSensor(){
	if (cntShock1 > 0 && !flagAlarm)
	{
		setBit(OUT_B_PORT, BLOCK);
		uint32_t now = seconds_get();
		if (cntShock1 == 1)
		{
			if (flagBlinkShock == 0)
			{
				timeStartFirstCnt = now + LOCK_TIME_SHOCK;		
				setBit(OUT_B_PORT, Blink_RELAY);
				setBit(OUT_B_PORT, VOICE_RELAY);
				_delay_ms(300);
				clearBit(OUT_B_PORT, Blink_RELAY);
				clearBit(OUT_B_PORT, VOICE_RELAY);
				flagBlinkShock = 1;
				memory = MEMORY_LITE;
			}
		}
		if (now > timeStartFirstCnt)
		{
			if (cntShock1 < MAX_CNT_SHOCK)
			{
				flagBlinkShock = 0;
			}
			else {
				timeStartFirstCnt = now + LOCK_TIME_SHOCK;
			}
			cntShock1 = 0;
		}
		setBit(EIMSK, INT0);
		if (bit_is_clear(PIND, SHOCK_Sensor_Hard))
		{
			Alarm();
			memory = MEMORY_HARD;
		}
	}
}

int main(void){
	DDRB = 0xff; //out
	DDRD = 0x00; //in
	DDRC = 0x00; //in
	
	PORTB = 0x00; //0
	PORTD = 0xff; //1
	PORTC = 0xff; //1
	if (!bit_is_clear(PIND, AUTOMAT))
	{
		autoTransmission = 1;
		allPresets = 1;
	}
	if (!bit_is_clear(PIND, NAKAL_ON_OFF))
	{
		flagNakal = 1;
	}
	setBit(OUT_B_PORT, LED_Pin);
	_delay_ms(500);
	clearBit(OUT_B_PORT, LED_Pin);
	timer0_init();
	timer1_init();
	timer2_init();
	USART_Init();
	EICRA |= (1 << ISC01);
	wdt_enable(WDTO_8S);
	sei();
	for (;;)
	{
		wdt_reset();
		if (heating == 1)
		{
			if (seconds_get() > timerHeating + HEATING_TIMER || !Handbrake() || BtnStart()){
				HeatingStop();
				//USART_SendString("Heating stop timer\r\n");
				heating = 0;
				if (security)
				{
					cntShock1 = 0;
					flagBlinkShock = 0;
					setBit(EIMSK, INT0);
				}
			}
			if (startOK && !bit_is_clear(PIND, GENERATOR))
			{
				HeatingStop();
				heating = 0;
				if (security)
				{
					cntShock1 = 0;
					flagBlinkShock = 0;
					setBit(EIMSK, INT0);
				}
			}
			if (security)
			{
				if (!flagAlarm && DoorOpen())
				{
					Alarm();
					memory = MEMORY_DOOR;
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
					cntTimer2 = 0;
					flagBlinkShock = 0;
					setBit(EIMSK, INT0);
				}
			}
			if (!security)
			{
				if (!tempFlagSec && bit_is_clear(PINC, Enable_SECURITY) && DoorClose())
				{
					SecurityON();

				}
				if (tempFlagSec)
				{
					if (seconds_get() > timeStartSecurity + DELAY_START_SECURITY)
					{
						tempFlagSec = 0;
						security = 1;
					}
					if (bit_is_clear(PINC, Diable_SECURITY))
					{
						SecurityOFF();
					}
				}
			}
		}
		if (heating == 0)
		{
			if (allPresets && DoorOpen())
			{
				ResetPresets();
			}
			if (security)
			{
				ShockSensor();
				if (!flagAlarm && DoorOpen())
				{
					Alarm();
					memory = MEMORY_DOOR;
				}
				if (flagAlarm && seconds_get() > timeAlarmStart + ALARM_TIMER)
				{
					timeAlarmStart = 0;
					flagAlarm = 0;
					cntTimer2 = 0;
					flagBlinkShock = 0;
					clearBit(OUT_B_PORT, Blink_RELAY);
					clearBit(OUT_B_PORT, VOICE_RELAY);
					clearBit(OUT_B_PORT, LED_Pin);
					setBit(EIMSK, INT0);
				}
				if (bit_is_clear(PINC, Diable_SECURITY))
				{
					SecurityOFF();
				}
			}
			if (!security)
			{
				if (!autoTransmission && !allPresets)
				{
					PresetsMT();
				}
				if (!tempFlagSec && !bit_is_clear(PINC, Ignition_IN) && bit_is_clear(PINC, Enable_SECURITY) && !bit_is_clear(PINC, Door_Pin))
				{
					SecurityON();

				}
				if (tempFlagSec)
				{
					if (seconds_get() > timeStartSecurity + DELAY_START_SECURITY)
					{
						tempFlagSec = 0;
						security = 1;
						cntTimer2 = 0;
						flagBlinkShock = 0;
						setBit(EIMSK, INT0);
					}
					if (bit_is_clear(PINC, Diable_SECURITY))
					{
						SecurityOFF();
					}
				}
			}
			if (allPresets && !flagAlarm && BtnStart() && !bit_is_clear(PINC, Ignition_IN) && !bit_is_clear(PIND, GENERATOR)) {
				heating = 1;
				clearBit(EIMSK, INT0);
				//USART_SendString("command - start!\r\n");
				EngineStart(3);
			}
		}
	}
}

ISR(INT0_vect) //sensor lite
{
	if (!heating){
		clearBit(EIMSK, INT0);
		uint8_t timer = 0;
		while(bit_is_clear(PIND, PD2)) { // button hold down
			timer++; // count how long button is pressed
			_delay_ms(1);
			if (timer > BTN_DEBOUCE)
			{
				++cntShock1;
				break;
			}
		}
	}
}
ISR(TIMER0_COMPA_vect){
	if (!bit_is_clear(PIND, STOP_Pin)) {
		if (integrator > 0) {
			integrator--;
		}
		} else if (integrator < DEBOUNCECYCLES) {
		integrator++;
	}
}
ISR(TIMER1_COMPA_vect)
{
	seconds++;
	//USART_Write_Int(cntShock1);
}
ISR(TIMER2_COMPA_vect)
{
	cntTimer2++;
	if (cntTimer2 > 31)// 496ms
	{
		cntTimer2 = 0;
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
		}
	}
}
