#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
//#include "usart.h"
#include "ds18b20.h"
#include "millis.h"

#define OUT_B_PORT		PORTB
#define LED_Pin			PB0
#define BLOCK           PB1
#define ON_ING			PB2
#define STARTER			PB3 
#define Blink_RELAY		PB4
#define VOICE_RELAY		PB5
#define OUT_C_PORT		PORTC
#define Diable_SECURITY    PC0
#define Enable_SECURITY		PC1
#define Ignition_IN			PC2
#define Temp_Pin			PC3
#define Door_Pin		PC4
#define ACC_Pin			PC5
#define OUT_D_PORT		PORTD
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
#define getBit(sfr, bit)	(_SFR_BYTE(sfr) & (1<<(bit)))

uint8_t Heating = 0;
uint8_t StatusSecurity = 0;
uint8_t Timer_alarm, timerBlink, timerBlinkSec;
uint8_t flagOnePreset, OnePresets, TwoPresets, TreePreset, AllPresets;
uint8_t startOK;
uint8_t flagAlarm;
uint32_t timerOnePresets;			//timer one preset
uint16_t periodPreset = 15000;		//max time presets 15 sec
uint16_t maxTimeCntVibro1 = 7000;	// max time for maxCntVibro1 ms
uint16_t maxTimeVibro1 = 20000;		//lock vibro1 ms
uint8_t maxCntVibro1 = 5;			//count single shock
uint16_t AlarmTimer = 90000;		//time alarm 
uint32_t AlarmStart;			// save start alarm
uint16_t maxTimeStarter  = 5000; //starter operating time ms
uint32_t StarterTimeON, timerHeating;
uint32_t timeVibro1 = 0;
volatile uint32_t timeVibro1Cnt = 0;
volatile uint8_t cntVibro1 = 0;
volatile uint8_t flagVibro1 = 0;
uint8_t flag2Vibro1 = 0;
uint16_t Timer = 900000;
uint8_t flagAutomat = 0;
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}		
void timer0_init() // initialize timer0, interrupt and variable
 {
	 	clearBit(TCCR0A, WGM00);			// Configure WGM to be 0x00 for normal mode
	 	clearBit(TCCR0A, WGM01);
	 	clearBit(TCCR0B, WGM02);
	 	setBit(TCCR0B, CS00);				// Configure clock source to be clock io at 1024 pre-scale
	 	clearBit(TCCR0B, CS01);
	 	setBit(TCCR0B, CS02);
		clearBit(TIMSK0, TOIE0);
 }
void timer2_init() // initialize timer0, interrupt and variable
 {
	 	clearBit(TCCR2A, WGM20);			// Configure WGM to be 0x00 for normal mode
	 	clearBit(TCCR2A, WGM21);
	 	clearBit(TCCR2B, WGM22);
	 	setBit(TCCR2B, CS20);				// Configure clock source to be clock io at 1024 pre-scale
	 	clearBit(TCCR2B, CS21);
	 	setBit(TCCR2B, CS22);
	 	setBit(TIMSK2, TOIE2);
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

uint8_t poll_AccOFF(){	
			if (!bit_is_clear(PINC, ACC_Pin)) {      /* button is pressed now */
				_delay_ms(20);
				if (!bit_is_clear(PINC, ACC_Pin)) {            /* still pressed */
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

void Blink(){
		setBit(OUT_B_PORT, VOICE_RELAY);
		setBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
		clearBit(OUT_B_PORT, Blink_RELAY);		
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
void PresetsManual() {
		if (OnePresets == 1 && TwoPresets ==0 && (!bit_is_clear(PIND, STOP_Pin) || bit_is_clear(PINC, Door_Pin)))
		{
			AllPresets = 0;
			OnePresets = 0;
			TwoPresets = 0;
			TreePreset = 0;
			//USART_SendString("Manual reset\r\n");
			_delay_ms(1000);
			clearBit(OUT_B_PORT, ON_ING);
		}
		if (AllPresets == 0 && TwoPresets == 1 && !bit_is_clear(PIND, STOP_Pin))
		{
			AllPresets = 0;
			OnePresets = 0;
			TwoPresets = 0;
			TreePreset = 0;
			//USART_SendString("Manual reset\r\n");
			_delay_ms(1000);
			clearBit(OUT_B_PORT, ON_ING);			
		}
		if (OnePresets ==0 && !bit_is_clear(PINC, Door_Pin) && bit_is_clear(PIND, STOP_Pin) && bit_is_clear(PINC, Ignition_IN)  && bit_is_clear(PIND, GENERATOR))
		{
			setBit(OUT_B_PORT, ON_ING);
			OnePresets  = 1;
//			USART_SendString("flag one true\r\n");
			_delay_ms(100);
		}

		if (TwoPresets == 0 && OnePresets == 1 && poll_AccOFF() && bit_is_clear(PIND, GENERATOR) && bit_is_clear(PINC, Ignition_IN) && bit_is_clear(PIND, STOP_Pin))
		{
			flagOnePreset = 0;
			TwoPresets = 1;
			timerOnePresets = millis_get();
	//		USART_SendString("OnePresets\r\n");
		//	_delay_ms(500);		
	
		}
		if (AllPresets == 0 && TwoPresets == 1 && (millis_get() - timerOnePresets > periodPreset)){
			OnePresets = 0;
			TwoPresets = 0;
			TreePreset = 0;
			clearBit(OUT_B_PORT, ON_ING);
			flagOnePreset = 0;
		//	USART_SendString("all reset timer\r\n");
		//	_delay_ms(1000);
		}
		if (TwoPresets == 1 && OnePresets == 1 && bit_is_clear(PINC, ACC_Pin) &&  bit_is_clear(PIND, GENERATOR))
		{
			clearBit(OUT_B_PORT, ON_ING);
			OnePresets = 0;
			TwoPresets = 0;
		//	USART_SendString("OnePresets reset\r\n");
		//	_delay_ms(100);
		}
		if (flagOnePreset ==0  && !bit_is_clear(PIND, GENERATOR) && !bit_is_clear(PINC, Ignition_IN)){
			clearBit(OUT_B_PORT, ON_ING);
			flagOnePreset = 1;
			OnePresets = 0;
			TwoPresets = 0;
		//	USART_SendString("Presets reset\r\n");
		//	_delay_ms(100);			
		}
		if (TwoPresets == 1 && TreePreset == 0 && bit_is_clear(PIND, STOP_Pin) && poll_DoorOpen())
		{
			TreePreset = 1;
			OnePresets = 0;
		//	USART_SendString("two \r\n");
		//	_delay_ms(1000);
		}
		if (TreePreset == 1 && AllPresets == 0 && bit_is_clear(PIND, STOP_Pin) && poll_DoorClose())
		{
			AllPresets = 1;
			TwoPresets = 0;
			clearBit(OUT_B_PORT, ON_ING);		
		//	USART_SendString("all preset \r\n");
		//	_delay_ms(1000);
		}							
		if (AllPresets == 1 && poll_DoorOpen() && StatusSecurity == 0 )
 		{
 			AllPresets = 0;
 			OnePresets = 0;
 			TwoPresets = 0;
			 TreePreset = 0;
 		//	USART_SendString("Preset reset\r\n");
 		//	_delay_ms(1000);
 			
 	 	}
}
void HeatingStop() {                                // программа остановки прогрева двигателя
	clearBit(OUT_B_PORT, ON_ING);
	_delay_ms(50);
	Blink();	
	startOK = 0;
//	USART_SendString("HeatingStop\r\n");
	clearBit(OUT_B_PORT, LED_Pin);
	timerHeating = 0;
}
void EngineStart(int attempts ) {                                      // программа запуска двигателя
	uint8_t count = 0;
	//uint8_t z = 0;
	//зажигание выключено, счетчик числа попыток  меньше attempts
	while (count < attempts && !bit_is_clear(PINC, Ignition_IN) && bit_is_clear(PIND, STOP_Pin)){
	//	USART_SendString("Timer \r\n");
	//	_delay_ms(100);
		wdt_reset();
		count++;
		Blink();		
		clearBit(OUT_B_PORT, ON_ING);
		setBit(OUT_B_PORT, ON_ING);         // включаем зажигание, и выжидаем 4 сек.
		_delay_ms(4000);		
		// прогреваем свечи несколько раз пропорционально понижению температуры, греем по 6 сек. с паузой 2 сек.
		if (!bit_is_clear(PIND, NAKAL_ON_OFF))
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
					Heating = 0;
					HeatingStop();
			//		USART_SendString("heat off\r\n");
			//		_delay_ms(20);
				}				
			//	USART_SendString("nakal \r\n");
				wdt_reset();
				setBit(OUT_B_PORT, ON_ING);
				_delay_ms(6000);
				if (poll_btn())
				{
					Heating = 0;
					HeatingStop();
				//	USART_SendString("heat off\r\n");
				//	_delay_ms(20);				
				}
								
			}	
		}

		// если на ручнике то включаем реле стартера на время StTime
		if(bit_is_clear(PIND, STOP_Pin)) //If switch is pressed
		{
			wdt_reset();
			StarterTimeON = millis_get();
			setBit(OUT_B_PORT, STARTER); // включаем реле стартера
		//	USART_SendString("starter on\r\n");
			} else {
			HeatingStop();
			Heating = 0;
			count = 0;
			break;
		}
		_delay_ms(100);
		while (millis_get() < (StarterTimeON + maxTimeStarter) && bit_is_clear(PIND, STOP_Pin)) {
			_delay_ms(10);
			if (bit_is_clear(PIND, GENERATOR)) {                          // если детектировать по напряжению зарядки
			//	USART_SendString("start OK\r\n");
				clearBit(OUT_B_PORT, STARTER);
				timerHeating = millis_get();
				startOK = 1;
				count = 0;
				break; }                   // считаем старт успешным, выхдим из цикла запуска двигателя
			if (poll_btn())
			{
					Heating = 0;
					HeatingStop();
			//		USART_SendString("heat off\r\n");
					_delay_ms(20);
			}				
		}
		if (startOK == 1)
		{
			break;
		}
		clearBit(OUT_B_PORT, STARTER);
		wdt_reset();
	//	USART_SendString("starter off, wait 6 sec\r\n");
		maxTimeStarter = maxTimeStarter + 500;                             // увеличиваем время следующего старта на 0.2 сек.
		HeatingStop();                                 // отключаем все реле без бнуления таймера	
		_delay_ms (5000);
	}
	if (startOK == 0)
	{
		Heating = 0;
	}
	_delay_ms(200);
//	USART_SendString("quit of heat\r\n");
}
void Alarm(){
	AlarmStart = millis_get();
	flagAlarm = 1;
	startOK = 0;
	timerHeating = 0;
	setBit(OUT_B_PORT, BLOCK);
	clearBit(OUT_B_PORT, ON_ING);
	setBit(TIMSK0, TOIE0);
	clearBit(EIMSK, INT0); // off vibro1
}
void Vibro1Check(){
						if (flagVibro1 == 1)
						{
							if (cntVibro1 >= maxCntVibro1 && (millis_get() < timeVibro1Cnt + maxTimeCntVibro1)){
						//		USART_SendString("vibro1timer cnt\r\n");
								timeVibro1 = millis_get();
								cntVibro1 = 0;
								timeVibro1Cnt = 0;
								flagVibro1 = 0;
								flag2Vibro1 = 1;
								clearBit(EIMSK, INT0);
						//		USART_SendString("vibro1 off\r\n");
							}
							if (cntVibro1 < maxCntVibro1 && (millis_get() > timeVibro1Cnt + maxTimeCntVibro1))
							{
								cntVibro1 = 0;
								timeVibro1Cnt = 0;
								flagVibro1 = 0;
						//		USART_SendString("vibro1 reset\r\n");
							}
						}

						if (flag2Vibro1 == 1 && millis_get() > timeVibro1 + maxTimeVibro1)
						{
							timeVibro1 = 0;
							flag2Vibro1 = 0;
							setBit(EIMSK, INT0);
						//	USART_SendString("vibro1 on\r\n");
						}
	
}
int main(void){
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
		DDRC &= ~_BV(ACC_Pin);//вход
		
		PORTB &= ~(1 << BLOCK) | ~(1 << ON_ING) | ~(1 << STARTER) | ~(1 << Blink_RELAY) | ~(1 << LED_Pin) | ~(1 << VOICE_RELAY); //установить "0" на линии
		PORTD  |= (1 << AUTOMAT) | (1 << STOP_Pin) | (1 << BUTTON) | (1 << GENERATOR) | (1 << VIBRO_1) | (1 << VIBRO_2) | (1 << NAKAL_ON_OFF);
		PORTC  |= (1 << Diable_SECURITY) | (1 << Enable_SECURITY) | (1 << Door_Pin) | (1 << ACC_Pin) | (1 << Ignition_IN);

		  EICRA |= /*(1<<ISC00) | */(1<<ISC01);
		  EICRA |= /*(1<<ISC00) | */(1<<ISC11);
		  //разрешим внешние прерывания INT0
		 // EIMSK |= (1<<INT0);
		 // EIMSK |= (1<<INT1);
		millis_init();
		timer0_init();
		timer2_init();
		//USART_Init(9600);
		wdt_enable (WDTO_8S);
		sei();
		//USART_SendString("Start\r");
			if (!bit_is_clear(PIND, AUTOMAT))
			{
				flagAutomat = 1;
			}		

for (;;) 
{
	wdt_reset();
// 	millis_t now = millis_get();
// 	if(now - lastChangeLed >= 500){
// 		//PORTB ^= _BV(PORTB0);
// 		/*double debugtemp = TempRead();*/
// 		USART_Write_Long(TempRead());
// 		USART_SendString("\r\n");
// 		// 		USART_Write_Word(VoltRead());
// 		// 		USART_SendString("\r\n");
// 		lastChangeLed = now;
// 	 }
		if (Heating == 1)
		{
			if (millis_get()> timerHeating + Timer ){
				HeatingStop();
				//USART_SendString("Heating stop timer\r\n");  // остановка прогрева если закончился таймера
				Heating = 0;
			}
			if (!bit_is_clear(PIND, STOP_Pin) || poll_btn() == 1 || (startOK == 1 && !bit_is_clear(PIND, GENERATOR)))		// на прогреве отпустили ручник
			{
				Heating = 0;
				HeatingStop();
				//USART_SendString("heat off\r\n");
				//_delay_ms(20);
			}
			if (StatusSecurity == 1)
			{
				if (poll_DoorOpen())
				{
					Alarm();
				}
				if (bit_is_clear(PINC, Diable_SECURITY))
				{
					clearBit(TIMSK0, TOIE0);
					clearBit(OUT_B_PORT, Blink_RELAY);
					clearBit(OUT_B_PORT, VOICE_RELAY);
					clearBit(OUT_B_PORT, LED_Pin);
					clearBit(EIMSK, INT0);
					clearBit(EIMSK, INT1);
					clearBit(OUT_B_PORT, BLOCK);
					StatusSecurity = 0;
					flagAlarm = 0;
					//USART_SendString("Security off\r\n");
					_delay_ms(500);
				}
				if (flagAlarm == 1 && millis_get() > AlarmStart + AlarmTimer)
				{
					AlarmStart = 0;
					flagAlarm = 0;
					clearBit(TIMSK0, TOIE0);
					clearBit(OUT_B_PORT, Blink_RELAY);
					clearBit(OUT_B_PORT, VOICE_RELAY);
					clearBit(OUT_B_PORT, LED_Pin);
					setBit(EIMSK, INT0);					
				}							
			}
			if (StatusSecurity == 0)
			{

				if (bit_is_clear(PINC, Enable_SECURITY) && !bit_is_clear(PINC, Door_Pin) && !bit_is_clear(PINC, ACC_Pin))
				{
					StatusSecurity = 1;
					//USART_SendString("Enable_SECURITY\r\n");
					_delay_ms(200);
				}				
			}						
		}						
		if (Heating == 0)
		{
			if (flagAutomat == 0)
			{
				PresetsManual();
			}
			else AllPresets = 1;
			if (AllPresets == 0)
			{
				if (StatusSecurity == 1)
				{
					Vibro1Check();
					if (flagAlarm == 1 && millis_get() > AlarmStart + AlarmTimer)
					{
						AlarmStart = 0;
						flagAlarm = 0;
						clearBit(TIMSK0, TOIE0);
						clearBit(OUT_B_PORT, Blink_RELAY);
						clearBit(OUT_B_PORT, VOICE_RELAY);
						clearBit(OUT_B_PORT, LED_Pin);
						setBit(EIMSK, INT0);
					}
					if (poll_DoorOpen() || bit_is_clear(PINC, ACC_Pin)){
						Alarm();
						}	
					if (bit_is_clear(PINC, Diable_SECURITY))
					{
						clearBit(TIMSK0, TOIE0);
						clearBit(EIMSK, INT0);
						clearBit(EIMSK, INT1);
						clearBit(OUT_B_PORT, Blink_RELAY);
						clearBit(OUT_B_PORT, VOICE_RELAY);
						clearBit(OUT_B_PORT, LED_Pin);
						clearBit(OUT_B_PORT, BLOCK);
						StatusSecurity = 0;
						AlarmStart = 0;
						flagAlarm = 0;
						//USART_SendString("Security off\r\n");
						_delay_ms(500);
					}					
				}
				if (StatusSecurity == 0){
					if (bit_is_clear(PINC, Enable_SECURITY) && !bit_is_clear(PINC, Door_Pin) && !bit_is_clear(PINC, ACC_Pin))
					{
						StatusSecurity = 1;
						setBit(EIMSK, INT0);
						setBit(EIMSK, INT1);
						//USART_SendString("Enable_SECURITY\r\n");
						_delay_ms(500);
					}					
				}
			}
					if (AllPresets ==1)
						{
							if (flagAlarm == 0 && poll_btn() && !bit_is_clear(PINC, Ignition_IN) && !bit_is_clear(PIND, GENERATOR)) {
								Heating = 1;
								//USART_SendString("command - start!");
								EngineStart(3);		// включаем запуск если двигатель не в прогревве
							}
							if (StatusSecurity == 1)
							{
								Vibro1Check();
								if (poll_DoorOpen() && bit_is_clear(PINC, ACC_Pin))
								{
									Alarm();
								}
								if (flagAlarm == 1 && millis_get() > AlarmStart + AlarmTimer)
								{
									AlarmStart = 0;
									flagAlarm = 0;
									clearBit(TIMSK0, TOIE0);
									clearBit(OUT_B_PORT, Blink_RELAY);
									clearBit(OUT_B_PORT, VOICE_RELAY);
									clearBit(OUT_B_PORT, LED_Pin);
									setBit(EIMSK, INT0);
								}
								if (bit_is_clear(PINC, Diable_SECURITY))
								{
									clearBit(TIMSK0, TOIE0);
									clearBit(EIMSK, INT0);
									clearBit(EIMSK, INT1);
									clearBit(OUT_B_PORT, Blink_RELAY);
									clearBit(OUT_B_PORT, VOICE_RELAY);
									clearBit(OUT_B_PORT, LED_Pin);
									clearBit(OUT_B_PORT, BLOCK);
									StatusSecurity = 0;
									AlarmStart =0;
									flagAlarm = 0;
								//	USART_SendString("Security off\r\n");
									_delay_ms(500);
								}																
							}
							if (StatusSecurity == 0)
							{
								if (bit_is_clear(PINC, Enable_SECURITY) && !bit_is_clear(PINC, Door_Pin) && !bit_is_clear(PINC, ACC_Pin))
								{
									StatusSecurity = 1;
									setBit(EIMSK, INT1);
									setBit(EIMSK, INT0);
									//USART_SendString("Enable_SECURITY\r\n");
									_delay_ms(500);
								}
							}
						}
		}
	}		
}

ISR (INT0_vect) //vibro1
{
	if (Heating == 0){
		cntVibro1++;		
		if (cntVibro1 == 1)
		{
			timeVibro1Cnt = millis_get();
			flagVibro1 = 1;
		}
		setBit(OUT_B_PORT, VOICE_RELAY);
		setBit(OUT_B_PORT, Blink_RELAY);
		_delay_ms(300);
		clearBit(OUT_B_PORT, VOICE_RELAY);
		clearBit(OUT_B_PORT, Blink_RELAY);		
	}
}
ISR (INT1_vect) //vibro2
{
	if (Heating == 0)
	{
		Alarm();
	}
	
}
ISR(TIMER0_OVF_vect){	//alarm
	Timer_alarm++;
	if (Timer_alarm >= 35)
	{
			PORTB ^= (1<<Blink_RELAY);
			PORTB ^= (1<<VOICE_RELAY);
			Timer_alarm =  0;
	}
	
}
ISR(TIMER2_OVF_vect){
	if (startOK == 1)
	{
		timerBlink++;
		if (timerBlink >= 250)
		{
			PORTB ^= (1<<Blink_RELAY);
			timerBlink =  0;
		}		
	}
	if (StatusSecurity == 1)
	{
		timerBlinkSec++;
		if (timerBlinkSec >= 250)
		{
			PORTB ^= (1<<LED_Pin);
			timerBlinkSec =  0;
		}
	}

	
}
