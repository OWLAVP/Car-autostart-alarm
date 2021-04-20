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
#define VIBRO_1			PD2
#define VIBRO_2			PD3
#define STOP_Pin		PD4
#define BUTTON          PD5
#define GENERATOR		PD6
#define NAKAL_ON_OFF	PD7

#define setBit(sfr, bit)     (_SFR_BYTE(sfr) |= (1 << bit))
#define clearBit(sfr, bit)   (_SFR_BYTE(sfr) &= ~(1 << bit))
#define toggleBit(sfr, bit) (_SFR_BYTE(sfr) ^= (1 << bit))
//#define getBit(sfr, bit)	(_SFR_BYTE(sfr) & (1<<(bit)))

#define BLINK_VOICE_ONE 1
#define BLINK_VOICE_TWO 2
#define BLINK_ONE 3
#define BLINK_TWO 4

#define PERIOD_PRESETS 20 //sec
#define MAX_TIME_SHOCK 7 //sec
#define LOCK_TIME_SHOCK 600 //sec
#define MAX_CNT_SHOCK 5 //cnt
#define ALARM_TIMER 90 //sec
#define HEATING_TIMER 900 //sec
uint8_t statusHeating = 0;
uint8_t statusSecurity = 0;
uint8_t allPresets;
uint8_t startOK;
uint8_t flagAlarm;
uint8_t flag2Vibro1 = 0;
uint8_t flagAutomat = 0;
uint8_t flagNakal = 0;
uint8_t flagOnePresets = 0;
uint8_t flagTwoPresets = 0;
uint32_t timeAlarmStart;			// save start alarm
uint32_t timeStarterON, timerHeating;
uint32_t timerOnePresets;			//timer one preset
uint32_t timeVibro1 = 0;
uint32_t lastChangeLed;
volatile uint8_t cntVibro1 = 0;
volatile uint8_t flagVibro1 = 0;
volatile uint8_t timerBlink;
volatile uint8_t timerBlinkSec = 0;
volatile uint8_t Timer_alarm;
volatile uint32_t timeVibro1Cnt = 0;
volatile uint32_t seconds = 0;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void timer0_init() // initialize timer0, interrupt and variable
{
	TCCR0 |= (1 << CS00) | (1 << CS02);
}
void timer1_init()
{
	// Timer settings every 1sec
	OCR1A = 15624;
	TCCR1B |= (1 << WGM12);
	TIMSK |= (1 << OCIE1A);
	TCCR1B |= (1 << CS12) | (1 << CS10);
}
void timer2_init() // initialize timer2, interrupt and variable
{
	OCR2 = 162;
	TCCR2 |= (1 << WGM21);// Set to CTC Mode
	TIMSK |= (1 << OCIE2); //Set interrupt on compare match
	TCCR2 |= (1 << CS20) | (1 << CS21) | (1 << CS22);
}
// Get current seconds
uint32_t millis_get()
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
		_delay_ms(20);
		if (!bit_is_clear(PINC, Door_Pin)) {            /* still pressed */
			return (1);
		}
	}
	return(0);
}
uint8_t poll_DoorOpen()		//declaring a debounce function
{
	if (bit_is_clear(PINC, Door_Pin)) {      /* button is pressed now */
		_delay_ms(20);
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

void Blink(uint8_t mode){
	wdt_reset();
	switch(mode){
		case BLINK_VOICE_ONE:
		setBit(OUT_B_PORT, VOICE_RELAY);
		setBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
		clearBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(500);
		break;
		case BLINK_VOICE_TWO:
		setBit(OUT_B_PORT, VOICE_RELAY);
		setBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
		clearBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(500);
		setBit(OUT_B_PORT, VOICE_RELAY);
		setBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
		clearBit(OUT_B_PORT, Blink_RELAY);
		break;
		case BLINK_ONE:
		setBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, Blink_RELAY);
		break;
		case BLINK_TWO:
		setBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(500);
		setBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, Blink_RELAY);
		break;
		default:
		break;
	}

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
	statusSecurity = 1;
	setBit(GICR, INT1);
	setBit(GICR, INT0);
	setBit(OUT_B_PORT, BLOCK);
	//USART_SendString("Enable_SECURITY\r\n");
	Blink(BLINK_TWO);
}
void SecurityOFF(){
	clearBit(TIMSK, TOIE0);
	clearBit(OUT_B_PORT, Blink_RELAY);
	clearBit(OUT_B_PORT, VOICE_RELAY);
	clearBit(OUT_B_PORT, LED_Pin);
	clearBit(GICR, INT0);
	clearBit(GICR, INT1);
	clearBit(OUT_B_PORT, BLOCK);
	statusSecurity = 0;
	flagAlarm = 0;
	Blink(BLINK_ONE);
	//USART_SendString("Security off\r\n");
}
void ResetPresets(){
	allPresets = 0;
	flagOnePresets = 0;
	flagTwoPresets = 0;
	clearBit(OUT_B_PORT, ON_ING);
}
void PresetsManual() {
	if (flagOnePresets == 1 && (!bit_is_clear(PIND, STOP_Pin) || !bit_is_clear(PIND, GENERATOR)))
	{
		ResetPresets();
		USART_SendString("Manual reset\r\n");
		_delay_ms(100);
	}
	if (allPresets == 1 && bit_is_clear(PINC, Door_Pin))
	{
		ResetPresets();
		USART_SendString("Door reset\r\n");
		_delay_ms(100);
	}
	
	if (flagOnePresets ==0 && !bit_is_clear(PINC, Door_Pin) && bit_is_clear(PIND, STOP_Pin) && bit_is_clear(PINC, Ignition_IN)  && bit_is_clear(PIND, GENERATOR))
	{
		setBit(OUT_B_PORT, ON_ING);
		flagOnePresets = 1;
		timerOnePresets = millis_get();
		USART_SendString("One\r\n");
		_delay_ms(100);
	}
	if (flagOnePresets == 1 && flagTwoPresets == 0 && bit_is_clear(PIND, STOP_Pin) && poll_DoorOpen())
	{
		flagTwoPresets = 1;
		USART_SendString("Two \r\n");
		_delay_ms(100);
	}
	if (flagTwoPresets == 1 && allPresets == 0 && bit_is_clear(PIND, STOP_Pin) && poll_DoorClose())
	{
		allPresets = 1;
		flagOnePresets = 0;
		clearBit(OUT_B_PORT, ON_ING);
		USART_SendString("All presets \r\n");
		_delay_ms(100);
	}
	if (allPresets == 0 && flagOnePresets == 1 && (millis_get() > timerOnePresets + PERIOD_PRESETS)){
		ResetPresets();
		USART_SendString("All reset timer\r\n");
		_delay_ms(100);
	}

}
void HeatingStop() {
	clearBit(OUT_B_PORT, ON_ING);
	clearBit(OUT_B_PORT, STARTER);
	Blink(BLINK_ONE);
	startOK = 0;
	//USART_SendString("HeatingStop\r\n");
	clearBit(OUT_B_PORT, LED_Pin);
	timerHeating = 0;
	timeStarterON = 0;
}
void EngineStart(int cnt ) {
	uint8_t count = 0;
	uint8_t maxTimeStarter = 5; //starter operating time sec

	clearBit(OUT_B_PORT, BLOCK);
	while (count < cnt && !bit_is_clear(PINC, Ignition_IN) && bit_is_clear(PIND, STOP_Pin)){
		//USART_SendString("Start cnt starter \r\n");
		wdt_disable();
		count++;
		Blink(BLINK_VOICE_TWO);
		clearBit(OUT_B_PORT, ON_ING);
		setBit(OUT_B_PORT, ON_ING);         //on ignition 4 sec
		_delay_ms(4000);
		if (flagNakal == 1)// nakal temp to cnt. 2 sec off ignition, 6 sec on
		{
			double temp = 0;
			temp = TempRead();
			_delay_ms(100);
			int z = map(temp, 0, -25, 0, 5);
			for(; z > 0 ; z--){
				clearBit(OUT_B_PORT, ON_ING);
				_delay_ms(2000);
				if (poll_btn())
				{
					statusHeating = 0;
					HeatingStop();
					//USART_SendString("heat off\r\n");
					//_delay_ms(20);
				}
				//USART_SendString("nakal \r\n");
				setBit(OUT_B_PORT, ON_ING);
				_delay_ms(6000);
				if (poll_btn())
				{
					statusHeating = 0;
					HeatingStop();
					//USART_SendString("heat off\r\n");
					//_delay_ms(20);
				}
				
			}
		}
		if(bit_is_clear(PIND, STOP_Pin)) //If switch is pressed
		{
			timeStarterON = millis_get();
			timeStarterON += maxTimeStarter;
			setBit(OUT_B_PORT, STARTER);
			//USART_SendString("starter on\r\n");
			} else {
			HeatingStop();
			statusHeating = 0;
			count = 0;
			break;
		}
		
		while (millis_get() < timeStarterON && bit_is_clear(PIND, STOP_Pin) && !bit_is_clear(PIND, GENERATOR)) {}
		_delay_ms (100);
		if (bit_is_clear(PIND, GENERATOR))
		{
			clearBit(OUT_B_PORT, STARTER);
			timerHeating = millis_get();
			startOK = 1;
			count = 0;
			wdt_enable(WDTO_2S);
			break;
		}
		clearBit(OUT_B_PORT, STARTER);
		//USART_SendString("starter off, wait 6 sec\r\n");
		maxTimeStarter = maxTimeStarter + 1;                             // + 1 sec
		HeatingStop();
		_delay_ms (5000);
	}
	if (startOK == 0)
	{
		USART_SendString("statusHeating = 0\r\n");
		statusHeating = 0;
		if (statusSecurity == 1)
		{
			setBit(GICR, INT0);
			setBit(GICR, INT1);
			setBit(OUT_B_PORT, BLOCK);
		}
	}
	wdt_enable(WDTO_2S);
	USART_SendString("quit of starting\r\n");
}
void Alarm(){
	timeAlarmStart = millis_get();
	flagAlarm = 1;
	startOK = 0;
	timerHeating = 0;
	statusHeating = 0;
	clearBit(OUT_B_PORT, STARTER);
	clearBit(OUT_B_PORT, ON_ING);
	setBit(OUT_B_PORT, BLOCK);
	clearBit(GICR, INT0); // off vibro1
	setBit(TIMSK, TOIE0); // on alarm timer0
}
void Vibro1Check(){

	if (flagVibro1 == 1)
	{
		if (cntVibro1 >= MAX_CNT_SHOCK && (millis_get() <= timeVibro1Cnt + MAX_TIME_SHOCK)){
			//USART_SendString("vibro1timer cnt\r\n");
			timeVibro1 = millis_get();
			cntVibro1 = 0;
			timeVibro1Cnt = 0;
			flagVibro1 = 0;
			flag2Vibro1 = 1;
			clearBit(GICR, INT0);
			//USART_SendString("vibro1 off\r\n");
		}
		if (cntVibro1 < MAX_CNT_SHOCK && (millis_get() > timeVibro1Cnt + MAX_TIME_SHOCK))
		{
			cntVibro1 = 0;
			timeVibro1Cnt = 0;
			flagVibro1 = 0;
			//USART_SendString("vibro1 reset\r\n");
		}
	}

	if (flag2Vibro1 == 1 && (millis_get() > timeVibro1 + LOCK_TIME_SHOCK))
	{
		timeVibro1 = 0;
		flag2Vibro1 = 0;
		setBit(GICR, INT0);
		//USART_SendString("vibro1 on\r\n");
	}
	
}
int main(void){
	//uint32_t now;
	DDRB |= _BV(BLOCK);//выход
	DDRB |= _BV(ON_ING);//выход
	DDRB |= _BV(STARTER);//выход
	DDRB |= _BV(Blink_RELAY);//выход
	DDRB |= _BV(LED_Pin);//выход
	DDRB |= _BV(VOICE_RELAY);//выход
	
	DDRD &= ~_BV(AUTOMAT);//вход
	DDRD &= ~_BV(STOP_Pin);//вход
	DDRD &= ~_BV(BUTTON);//вход
	DDRD &= ~_BV(GENERATOR);//вход
	DDRD &= ~_BV(VIBRO_1);//вход
	DDRD &= ~_BV(VIBRO_2);//вход
	DDRD &= ~_BV(NAKAL_ON_OFF);//вход
	
	DDRC &= ~_BV(Ignition_IN);//вход
	DDRC &= ~_BV(Diable_SECURITY);//вход
	DDRC &= ~_BV(Enable_SECURITY);//вход
	DDRC &= ~_BV(Door_Pin);//вход
	//DDRC &= ~_BV(ACC_Pin);//вход
	
	PORTB &= ~(1 << BLOCK) | ~(1 << ON_ING) | ~(1 << STARTER) | ~(1 << Blink_RELAY) | ~(1 << LED_Pin) | ~(1 << VOICE_RELAY); //установить "0" на линии
	PORTD  |= (1 << AUTOMAT) | (1 << STOP_Pin) | (1 << BUTTON) | (1 << GENERATOR) | (1 << VIBRO_1) | (1 << VIBRO_2) | (1 << NAKAL_ON_OFF);
	PORTC  |= (1 << Diable_SECURITY) | (1 << Enable_SECURITY) | (1 << Door_Pin)/* | (1 << ACC_Pin)*/ | (1 << Ignition_IN);
	timer0_init();
	timer1_init();
	timer2_init();
	USART_Init();
	wdt_enable(WDTO_2S);
	sei();
	USART_SendString("Start\r");
	if (bit_is_clear(PIND, AUTOMAT))
	{
		flagAutomat = 1;
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
		// 		 	now = millis_get();
		// 		 	if(now - lastChangeLed >= 1){
		// 		 		//PORTB ^= _BV(PORTB0);
		// 		 		/*double debugtemp = TempRead();*/
		// 		 		//USART_Write_Int(flagOnePresets);
		// 				//USART_SendString("jhhhjjh\r\n");
		// 		// 		// 		USART_Write_Word(VoltRead());
		// 		// 		// 		USART_SendString("\r\n");
		// 		 		lastChangeLed = now;
		// 		 	 }
		if (statusHeating == 1)
		{
			if (millis_get()> timerHeating + HEATING_TIMER || !bit_is_clear(PIND, STOP_Pin) || poll_btn() == 1 || (startOK == 1 && !bit_is_clear(PIND, GENERATOR))){
				HeatingStop();
				//USART_SendString("Heating stop timer\r\n");  // остановка прогрева если закончился таймера
				statusHeating = 0;
				if (statusSecurity == 1)
				{
					setBit(GICR, INT0);
					setBit(GICR, INT1);
					setBit(OUT_B_PORT, BLOCK);
				}
			}
			if (statusSecurity == 1)
			{
				if (poll_DoorOpen())
				{
					Alarm();
				}
				if (bit_is_clear(PINC, Diable_SECURITY))
				{
					SecurityOFF();
				}
				if (flagAlarm == 1 && millis_get() > timeAlarmStart + ALARM_TIMER)
				{
					timeAlarmStart = 0;
					flagAlarm = 0;
					clearBit(TIMSK, TOIE0);
					clearBit(OUT_B_PORT, Blink_RELAY);
					clearBit(OUT_B_PORT, VOICE_RELAY);
					clearBit(OUT_B_PORT, LED_Pin);
					setBit(GICR, INT0);
					setBit(GICR, INT1);
				}
			}
			if (statusSecurity == 0)
			{
				if (bit_is_clear(PINC, Enable_SECURITY) && !bit_is_clear(PINC, Door_Pin))
				{
					statusSecurity = 1;
					//USART_SendString("Enable_SECURITY\r\n");
					Blink(BLINK_TWO);
				}
			}
		}
		if (statusHeating == 0)
		{
			if (flagAutomat == 0)
			{
				PresetsManual();
			}
			else allPresets = 1;
			if (allPresets == 0)
			{
				if (statusSecurity == 1)
				{
					Vibro1Check();
					if (flagAlarm == 1 && millis_get() > timeAlarmStart + ALARM_TIMER)
					{
						timeAlarmStart = 0;
						flagAlarm = 0;
						clearBit(TIMSK, TOIE0);
						clearBit(OUT_B_PORT, Blink_RELAY);
						clearBit(OUT_B_PORT, VOICE_RELAY);
						clearBit(OUT_B_PORT, LED_Pin);
						setBit(GICR, INT0);
						setBit(GICR, INT1);
					}
					if (poll_DoorOpen()){
						Alarm();
					}
					if (bit_is_clear(PINC, Diable_SECURITY))
					{
						SecurityOFF();
					}
				}
				if (statusSecurity == 0 && !bit_is_clear(PINC, Door_Pin) && bit_is_clear(PINC, Enable_SECURITY))
				{
					SecurityON();
				}
			}
			if (allPresets == 1)
			{
				if (flagAlarm == 0 && poll_btn() && !bit_is_clear(PINC, Ignition_IN) && !bit_is_clear(PIND, GENERATOR)) {
					statusHeating = 1;
					clearBit(GICR, INT0);
					clearBit(GICR, INT1);
					//USART_SendString("command - start!");
					EngineStart(3);
				}
				if (statusSecurity == 1)
				{
					Vibro1Check();
					if (poll_DoorOpen())
					{
						Alarm();
					}
					if (flagAlarm == 1 && millis_get() > timeAlarmStart + ALARM_TIMER)
					{
						timeAlarmStart = 0;
						flagAlarm = 0;
						clearBit(TIMSK, TOIE0);
						clearBit(OUT_B_PORT, Blink_RELAY);
						clearBit(OUT_B_PORT, VOICE_RELAY);
						clearBit(OUT_B_PORT, LED_Pin);
						setBit(GICR, INT0);
						setBit(GICR, INT1);
					}
					if (bit_is_clear(PINC, Diable_SECURITY))
					{
						SecurityOFF();
					}
				}
				if (statusSecurity == 0 && !bit_is_clear(PINC, Door_Pin) && bit_is_clear(PINC, Enable_SECURITY))
				{
					SecurityON();
				}
			}
		}
	}
}

ISR (INT0_vect) //vibro1
{
	if (statusHeating == 0){
		++cntVibro1;
		if (cntVibro1 == 1)
		{
			timeVibro1Cnt = millis_get();
			flagVibro1 = 1;
		}
		Blink(BLINK_VOICE_ONE);
	}
}
ISR (INT1_vect) //vibro2
{
	if (statusHeating == 0)
	{
		Alarm();
	}
	
}
ISR(TIMER0_OVF_vect){	//alarm
	++Timer_alarm;
	if (Timer_alarm >= 37)
	{
		toggleBit(OUT_B_PORT, Blink_RELAY);
		toggleBit(OUT_B_PORT, VOICE_RELAY);
		Timer_alarm =  0;
	}

}
ISR(TIMER1_COMPA_vect)
{
	++seconds;
	if (statusSecurity == 1)
	{
		toggleBit(OUT_B_PORT, LED_Pin);

	}
}
ISR (TIMER2_COMP_vect)
{
	if (startOK == 1)
	{
		++timerBlink;
		if (timerBlink >= 60 )
		{
			toggleBit(OUT_B_PORT, Blink_RELAY);
			timerBlink =  0;
		}
	}

}
