/*****
 * Наименование: WaterChip 
 * Описание: Контроллер наполения бака и управления нагревом воды в нём
 * Дата создания: 16.08.2014
 * Дата обновления: 16.12.2014
 * Версия: 1.1 на опытную эксплуатацию
 * 
 * ChangeLog:
 * 1.0 реализованы все функции(нагрев, наполнение бака, контроль давления, индикация, обработка кнопок)
 * 1.1 переработана обработка кнопок, была по опросу, стала по прерыванию
 *     стала недоступка функция запроса уровня воды, нет алгоритма обработки длинного нажатия
 *
 * Реализованы и работают все фнкции
 * TODO:
 *  обработка длинного нажания
 *
 ***/

#include <avr/version.h>
#if __AVR_LIBC_VERSION__ < 10606UL
#error "please update to avrlibc 1.6.6 or newer, not tested with older versions"
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "spilight/spilight.h"
#include "uart/uart.h"
#include "uart/uart_addon.h"
#include "ds18b20/onewire.h"
#include "ds18b20/ds18x20.h"
#include "indication.h"
#include "keyboard.h"
#include "adc/atmega-adc.h"

#ifndef INFINITY
#define INFINITY 1
#endif
#ifndef F_CPU
#define F_CPU 8000000L
#endif

// АЦП
#define ADC_PIN 6
// Диапазон и и шаг для настройкии времени простоя ГЦН
#define DIAPASONE 1000
#define STEP 100
#define LEVELS DIAPASONE / STEP
#define PUMP_DELTA_INACTIVE_INTERVAL 5

//
#define PRESSURE_DDR		DDRC
#define PRESSURE_PINS		PINC
#define PRESSURE_SENS_PIN	PINC0
#define NO_PRESSURE 		0
#define PRESSURE 			1

// 
//#define LEVEL_SENS_DDR 	DDRC
//#define LEVEL_SENS_PINS PINC
#define MIN_LEVEL_PIN 		PINC5
#define MID_LEVEL_PIN   	PIND0
#define MAX_LEVEL_PIN   	PIND1
#define ERROR_LEVEL		 	0
#define MIN_LEVEL 			1
#define MID_LEVEL 			2
#define MAX_LEVEL 			3

// 
#define ON 	1
#define OFF 0

int16_t get_temper(uint8_t nSensors, uint8_t *error);

// 
/*
static   char codec8seg[] = {
  	 0xF2, // 1
	 0x48, // 2
	 0x60, // 3 
	 0x32, // 4
	 0x24, // 5
     0x04, // 6
	 0xF0, // 7
	 0x00, // 8
	 0x20, // 9
	 0x80, // 0
  };*/
//static unsigned int baudrate = 19200;

extern void (*display[])(uint8_t);

#define DISP_INTERVAL_VALUE 15 // подогнать в финале под задачу 
static uint8_t disp_interval = DISP_INTERVAL_VALUE; // интервал опроса датчика теперетуры ~ 1 мин
static uint8_t display_flag = 1;

#define MEASURE_INTERVAL_VALUE 250 // подогнать в финале под задачу 
static uint8_t measure_interval = MEASURE_INTERVAL_VALUE;
static uint8_t get_temper_flag = 1;

#define SETUP_TIMEOUT_VALUE 7
static uint8_t setup_timeout = SETUP_TIMEOUT_VALUE;
static volatile uint8_t setup_timeout_flag = 0;


#define NO_FAULTS            	0 
#define LEVEL_SENS_FAULT 		1
#define TEMPER_SENS_FAULT 		2
#define PRESSURE_SENS_FAULT 	3
static uint8_t faults = NO_FAULTS;


#define IDLE 		0
#define RUN 		1
#define SETUP 		2
#define FAULT 		3
#define PUMP_SETUP 	4
static uint8_t dev_state = IDLE;  

static volatile uint8_t blink = OFF;

// Регистр состояния устройства DVSR
#define EEIF 0  // флаг инициализации ПЗУ
//Бит      
// 0   EEIF        Флаг инициализации нужен для корректной работы с EEPROM если она не проинициализирована 0 - проинициализирована 1 - не проинициализирована
// 1   Reserved
// 2   Reserved
// 3   Reserved
// 4   Reserved
// 5   Reserved
// 6   Reserved
// 7   Reserved  
uint8_t EEMEM DVSR;
int16_t EEMEM nv_user_temper;

#define STAGE_0 0
#define STAGE_1 1
#define STAGE_2 2
#define STAGE_3 3
#define STAGE_4 4
#define STAGE_5 5

static uint8_t heat_anim_state = STAGE_0;
static uint8_t fill_tank_anim_state = STAGE_0;


// Управление помпой
#define TIK_PER_SEC 30 // ~1 min
#define SEC_PER_MIN 60
#define DEFAULT_INACTIVE_INTERVAL_MIN 0 // значение 
#define ACTIVE_INTERVAL_MIN 15 // [наш насос: 100 л / 7 л/мин  ~ 15 мин]  на начало работ значение выбрано 10
#define ACTIVE 1
#define INACTIVE 0
static volatile uint8_t pump_state = INACTIVE;
static volatile uint8_t pump_inactive_interval = DEFAULT_INACTIVE_INTERVAL_MIN; // значение задаётся с АЦП
static const volatile uint8_t pump_active_interval = ACTIVE_INTERVAL_MIN; // подбирается импирически
static volatile uint8_t minutes = 0; 
static volatile uint8_t seconds = SEC_PER_MIN;
static volatile uint8_t tik_per_sec = TIK_PER_SEC;
static volatile uint8_t inactive_minutes_setting = DEFAULT_INACTIVE_INTERVAL_MIN;
//static unit8_t seconds_setting = 0;


#define ADC_READ_INTERVAL 40;
static volatile uint8_t read_adc_flag = 0;
static volatile uint8_t adc_read_interval = ADC_READ_INTERVAL;

// прототипы функци местных
void nv_init(void);
static uint8_t search_sensors(void);
void start_setup_timeout(void);
uint8_t get_water_level(void);
uint8_t get_water_pressure(void);
uint8_t heat_water(int16_t water_temper);
uint8_t fill_tank(uint8_t water_level, uint8_t setting);
uint8_t process_water_tank(uint8_t water_level, uint8_t water_pressure, uint8_t half_tank_mode, int16_t water_temper);



// TERMO SENSOR =======================================================
#define MAXSENSORS 5
#define NEWLINESTR "\r\n"

uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
 
// ==================================================================


/*
 http://www.microcontrollerov.net/microcontrolleri/mega/AVR151-Inicializacija-i-ispolzovanie-interfejsa-SPI

  

  Доды на ногах
  PD5 нет
  PD6 нет
  PD7 есть
  PB0 есть

  Управление выбором сегментов
  SG1 PC0 младший
  SG2 PB1 старший
 */



static volatile uint8_t blink_t0 = OFF;

//-------------------------------------------------------------
ISR (TIMER0_OVF_vect)
{
  TCCR0B = 0;

  // алгоритм счета времени для управления помпой
  if (tik_per_sec <= 0) {
    tik_per_sec = TIK_PER_SEC;
	if (seconds <= 0) {
	  seconds = SEC_PER_MIN;
      minutes++;
      if (!pump_inactive_interval) {
        if (minutes >= /*ACTIVE_INTERVAL_MIN*/pump_active_interval) {
		  pump_state = ACTIVE;
		  minutes = 0;
		} 
      } else {
	    switch(pump_state) {
	    case ACTIVE:
          if (minutes >= /*ACTIVE_INTERVAL_MIN*/pump_active_interval) {
		    pump_state = INACTIVE;
		    minutes = 0;
		  } 
	      break;
        case INACTIVE:
	      if (minutes >= /*DEFAULT_INACTIVE_INTERVAL_MIN*/pump_inactive_interval) {
		    pump_state = ACTIVE;
		    minutes = 0;
		  }
		  break;
	    } // eof switch
	  } // eof if-else
	} else {
	  seconds--;
	}
  }	else {
    tik_per_sec--;
  }
  
  
  // считать интервал вывода значений на дисплей
  if (disp_interval <= 0) {
    disp_interval = DISP_INTERVAL_VALUE;
	display_flag = 1;
  } else {
    disp_interval--;
  }

  // считать интервал опроса АЦП
  if (adc_read_interval <= 0) {
    adc_read_interval = ADC_READ_INTERVAL;
	read_adc_flag = 1;
  } else {
    adc_read_interval--;
  }

  // считать интервал опроса термодатчика
  if (measure_interval <= 0) {
    measure_interval = MEASURE_INTERVAL_VALUE;
	get_temper_flag = 1;
  } else {
    measure_interval--;
  }
   
  TCNT0 = 0x01;
  TCCR0B |= (1 << CS02) | (1 << CS00);
  return;
}


//-------------------------------------------------------------
ISR (TIMER1_OVF_vect)
{
  TCCR1B = 0;
  // что-то делать

  if (setup_timeout <= 0) {
    setup_timeout_flag = 1;
	TCCR1B = 0; // stop the timer
  } else {
    setup_timeout--;
	TCNT1H = 0xF0;
    TCNT1L = 0x01;
    TCCR1B |= (1 << CS12) | (1 << CS10); 
  }
  
  return;
}

//=====================таймер для измерения времени нажатия кнопки, для длинных нажатий =====
/*volatile uint8_t long_press = 0;
#define LONG_PRESS_PRESCALER_VALUE 10
volatile uint8_t long_press_prc = LONG_PRESS_PRESCALER_VALUE; 
ISR (TIMER2_OVF_vect)
{
  TCCR2B = 0;
  if (long_press_prc-- > 0) {
    TCNT2 = 0;
    TCCR2B |= (1 << CS22) | (1 << CS20);
  } else {
    long_press = 1;
  }
  return;
}

void start_long_press_timeout()
{
  TIMSK2 |= (1 << TOIE2);
  TCNT2 = 0;
  TCCR2B |= (1 << CS22) | (1 << CS20); 
  return;
}

void stop_long_press_timeout()
{
  TIMSK2 &= ~(1 << TOIE2);
  TCCR2B = 0;
  return;
}
*/
//=============



//-------------------------------------------------------------
void start_setup_timeout(void)
{
  // настроить таймер Т1 для отсчёта одной секунды
  TIMSK1 |= (1 << TOIE1);
  TCCR1B = 0;
  TCNT1H = 0xF0; // от балды
  TCNT1L = 0x01; // от балды
  TCCR1B |= (1 << CS12) | (1 << CS10);
  setup_timeout_flag = 0;
  setup_timeout = SETUP_TIMEOUT_VALUE; 
  return;
}
//-------------------------------------------------------------
void restart_setup_timeout(void)
{
  TIMSK1 &= ~(1 << TOIE1);
  TCCR1B = 0;
  start_setup_timeout();
  return;
}
//-------------------------------------------------------------
void stop_setup_timeout(void)
{
  TIMSK1 &= ~(1 << TOIE1);
  TCCR1B = 0;
  setup_timeout_flag = 0;
  setup_timeout = SETUP_TIMEOUT_VALUE;
  return; 
}
//-------------------------------------------------------------
void pump_relay_on(void)
{
  DDRD |= (1 << PD2);
  PORTD |= (1 << PD2);
  return;
}
//-------------------------------------------------------------
void pump_relay_off(void)
{
  DDRD |= (1 << PD2);
  PORTD &= ~(1 << PD2);
  return;
}
//-------------------------------------------------------------

void heater_relay_on(void)
{
  DDRD |= (1 << PD5);
  PORTD |= (1 << PD5);
  return;
}
//-------------------------------------------------------------
void heater_relay_off(void)
{
  DDRD |= (1 << PD5);
  PORTD &= ~(1 << PD5);
  return;
}

//-------------------------------------------------------------
void valve_relay_on(void)
{
  DDRD |= (1 << PD3);
  PORTD |= (1 << PD3);
  return;
}
//-------------------------------------------------------------
void valve_relay_off(void)
{
  DDRD |= (1 << PD3);
  PORTD &= ~(1 << PD3);
  return;
}
//-------------------------------------------------------------
void startup_init(void)
{
  
  // настроить таймер Т0
  TIMSK0 |= (1 << TOIE0);
  TCNT0 = 0x01;
  TCCR0B |= (1 << CS02) | (1 << CS00);   

  
  // настроить таймер Т1 для отсчёта одной секунды
  TIMSK1 |= (1 << TOIE1);
  TCCR1B = 0;
  TCNT1H = 0xF0;
  TCNT1L = 0x01;
  TCCR1B |= (1 << CS12) | (1 << CS10);   

  // init_leds()   
  DDRD |= (1 << PD5);

  // урпавление сегментами инициализация стробирующих битов 
  // тут надо что-то придумать с дефайнами...
  DDRB |= (1 << PB1); // старший разряд SG2
  DDRB |= (1 << PB2); // младший разряд SG1


  // новое 20141216
  // настроить прерывания для обработки кнопок
  init_keys();  

  init_spi_master();
  // если прошивка стартует первый раз то принудительно проинитить ПЗУ
  nv_init();

  sei();

  // закрыть все заслонки, выключить насосы и тены
  heater_relay_off();
  pump_relay_off();
  valve_relay_off();


  // потушить все диоды
  no_water_pressure_led_off();
  half_tank_led_off();
  norm_led_off();
  fault_led_off();
  run_led_off();

  // очистить регистр отказов
  faults = NO_FAULTS;

  return;
}
//-------------------------------------------------------------
int main(void)
{
  startup_init();

  //__DEBUG
  //uart_init(UART_BAUD_SELECT(baudrate, F_CPU));
  //uart_puts("Hitter Started...\r\n");

  // переменные для работы с термодатчиком 
  uint8_t nSensors;
  int16_t water_temper;
  uint8_t error_sens;
  uint8_t key = NO_KEYS; 
  // получить значение последней сохранённой пользователской уставки тепмпературы
  eeprom_busy_wait();
  int16_t user_temper = eeprom_read_word(&nv_user_temper);  
  uint8_t half_tank_mode = OFF;
  uint8_t half_tank_led_state = ON;
  uint8_t blink_disp = ON; 
  uint8_t water_level = 0;
  uint8_t water_pressure = NO_PRESSURE;
  uint16_t adc_val = 0;
  //uint16_t adc_val_prev = 0; // УБРАТЬ
  uint8_t adc_level = 0;
  uint8_t prev_adc_level = 0;

  // произвести самотест устройства, если ошибка то прейти в сост. FAULT
  //start_self_test();
  
  // задержка перед запуском
  // ...
  set_8segf(HG1, "EF");
  _delay_ms(150);
  set_8segf(HG1, "BC");
  _delay_ms(150);
  set_8segf(HG1, "");
  set_8segf(HG2, "EF");
  _delay_ms(150);
  set_8segf(HG2, "BC");
  _delay_ms(150);

  set_8segf(HG1, "D");
  set_8segf(HG2, "D");
  _delay_ms(150);
  set_8segf(HG1, "G");
  set_8segf(HG2, "G");
  _delay_ms(150);
  set_8segf(HG1, "A");
  set_8segf(HG2, "A");
  _delay_ms(150);

  set_8segf(HG1, "D");
  set_8segf(HG2, "D");
  _delay_ms(150);  
  set_8segf(HG1, "DE");
  set_8segf(HG2, "DE");
  _delay_ms(150);
  set_8segf(HG1, "DEF");
  set_8segf(HG2, "DEF");
  _delay_ms(150);

  set_8segf(HG1, "DEF");
  set_8segf(HG2, "DEF");
  _delay_ms(150);
  set_8segf(HG1, "DEFA");
  set_8segf(HG2, "DEFA");

  _delay_ms(150);
  set_8segf(HG1, "DEFAB");
  set_8segf(HG2, "DEFAB");
  _delay_ms(150);
  set_8segf(HG1, "DEFABC");
  set_8segf(HG2, "DEFABC");
  _delay_ms(150);

  set_8segf(HG1, "");
  set_8segf(HG2, "");
  _delay_ms(500);

  // потушить все диоды
  no_water_pressure_led_on();
  half_tank_led_on();
  norm_led_on();
  fault_led_on();
  run_led_on();

  set_8segf(HG1, "DEFABGC");
  set_8segf(HG2, "DEFABGC");
  _delay_ms(500);
  

  // потушить все диоды
  no_water_pressure_led_off();
  half_tank_led_off();
  norm_led_off();
  fault_led_off();
  run_led_off();

  set_8segf(HG1, "");
  set_8segf(HG2, "");
  _delay_ms(500);

  // попытка избавиться от говна с АЦП
  adc_val = adc_read(ADC_PRESCALER_2, ADC_VREF_AVCC, ADC_PIN);
  _delay_ms(150);
  adc_val = adc_read(ADC_PRESCALER_2, ADC_VREF_AVCC, ADC_PIN);
  _delay_ms(150);
  
  // запомнить кущий уровень значения АЦП для события изменения значения
  adc_val = adc_read(ADC_PRESCALER_2, ADC_VREF_AVCC, ADC_PIN);
  // определение диаразона и установить интервал простоя ГЦН в минутах либо отключить эту опцию
	uint8_t done = 0;
	for (uint8_t level = 1; level < LEVELS && !done; level++) {
      uint16_t threshold_lo = (level-1) * STEP;
	  uint16_t threshold_hi = level * STEP;
	  // при попадении в диапазон, установить соотв. интервал времени простоя ГЦНа.
	  if (adc_val >= threshold_lo && adc_val < threshold_hi) {
        // расчёт интервала в зависимости от уровня
		// по идее шаг изменения времени простоя ГЦН = 5 минут
		// время простоя ГЦН = уровень * шаг изменения времени простоя ГЦН
		pump_inactive_interval = level * PUMP_DELTA_INACTIVE_INTERVAL;
		adc_level = prev_adc_level = level;
		done = 1;
	  }	  
	  // если значение вне диапазона, установить значение интрервала 
	  // времени простоя ГЦНа рвным нулю(Отключить эту опцию) 
	  if (adc_val > DIAPASONE) {
	    pump_inactive_interval = 0; //
		adc_level = prev_adc_level = level;
		done = 1;
	  } 
	} // eof for
  _delay_ms(150);
    
  // * -> [idle]

  while (INFINITY) {   	
	// ====================================================================
    // ЛОГИКА
	switch (dev_state) {
    case IDLE: // -------------------------------------------
	  // закрыть все заслонки, выключить насосы и тены
      heater_relay_off();
      pump_relay_off();
      valve_relay_off();
	  
	  // опросить датчик уровня воды в баке
	  water_level = get_water_level();
      // опросить датчик давления воды в трубе
	  water_pressure = get_water_pressure();    	
      // получить количесво доступных термодатчиков
      nSensors = search_sensors();	  

      // очистить регистр отказов
      faults = NO_FAULTS;
	        
	  // проверить нет ли отказов датчиков
      // если нет давления воды, то уйти в сост. Авария
	  if (!water_pressure) {
		no_water_pressure_led_on();
		//norm_led_off();
		//fault_led_on();
		//dev_state = FAULT;
		//faults |= (1 << PRESSURE_SENS_FAULT);
		// [idle] -> [fault]
	  } else {
	    no_water_pressure_led_off();
	  }

	  // если воникла ошибка датчика уровня, то уйти в сост. Авария
	  if (!water_level) {
	  	dev_state = FAULT;
		faults |= (1 << LEVEL_SENS_FAULT);        
	  	fault_led_on();
      	norm_led_off();
		// [idle] -> [fault]
	  }

      // если нет датчика температуры, уйти в сост. Авария 
	  if ( nSensors == 0 ) {
		error_sens++;
		dev_state = FAULT;
		faults |= (1 << TEMPER_SENS_FAULT);
		fault_led_on();
      	norm_led_off();
		// [idle] -> [fault]
	  }
	  
      

	  // если отказов нет, то производить действия предписанные сост. Простой 
	  if (!faults) {    
	    
		norm_led_on();
	    fault_led_off();
				
		// опрсить датчик если пришло время опроса
	    if (get_temper_flag) {
	      get_temper_flag = 0;
		  // замерять температуру воды в баке
		  water_temper = get_temper(nSensors, &error_sens);
		  // если возникла ошибка измерения температуры, уйти в обработку ошибок
		  //if (error_sens) { // тут не понятко нужна ли эта проверка или нет ???
		  //  error_sens = 0;
		//	fault_led_on();
		//	norm_led_off();
          //  dev_state = FAULT;
		    // [idle] -> [fault]
		  //} else {	      
            //иначе вывести полученную температуру на инидкатор
            if (display_flag) {
			  display_flag = 0;
			  put_temper(water_temper);
			}
		  //}
	    } // oef if
        
        //==============================================================
	   // опросисть АЦП для регириовки интервала бездействия помпы
       //uint16_t adc_read(uint8_t prescaler, uint8_t vref, uint8_t pin);  
	   if (read_adc_flag) {
	     read_adc_flag = 0;
	     adc_val = adc_read(ADC_PRESCALER_2, ADC_VREF_AVCC, ADC_PIN);     
	     // определение диаразона и установить интервал простоя ГЦН в минутах либо отключить эту опцию
	     uint8_t done = 0;
	     for (uint8_t level = 1; level < LEVELS && !done; level++) {
           uint16_t threshold_lo = (level-1) * STEP;
	       uint16_t threshold_hi = level * STEP;
	       // при попадении в диапазон, установить соотв. интервал времени простоя ГЦНа.
	       if (adc_val >= threshold_lo && adc_val < threshold_hi) {	         
			 // перейти в состояние насройки интревала простоя ГЦНа
		     adc_level = level;
		     done = 1;
	       }
      	  
	       // если значение вне диапазона, установить значение интрервала 
	       // времени простоя ГЦНа рвным нулю(Отключить эту опцию) 
	       if (adc_val > DIAPASONE) {
		     adc_level = level;
		     done = 1;
	       } 
	     } // eof for
	   } // eof if read adc flag
       	   
	   if (adc_level != prev_adc_level) {
	     // записать новое значение
	     prev_adc_level = adc_level;
	     dev_state = PUMP_SETUP;
		 for (uint8_t i = 0; i < 3; i++) {
		   set_8segf(HG1, "FE");
		   set_8segf(HG2, "BC");
           _delay_ms(100);
           set_8segf(HG1, "");
		   set_8segf(HG2, ""); 
		   _delay_ms(100);
         }
		 // задать вермя на установку по истечнии которого стостояние сменитсяч на Простой
		 start_setup_timeout();
	     // [idle] -> [pump_setup]
	   }     
	   //==============================================================

		// обработать нажатую клавишу, если таковая имеется
		key = get_key();
		switch (key) {
        case HALF_TANK_KEY:
	      //uart_puts("-------HALF----------\r\n");
	      if (half_tank_led_state) {	
	        half_tank_led_on();
		    half_tank_mode = ON;
		    half_tank_led_state = OFF;
	      } else {
	        half_tank_led_off();
	        half_tank_mode = OFF;
		    half_tank_led_state = ON;
	      }
	      //_delay_ms(200);
	      break;
	    case GET_LEVEL_KEY:
	      //uart_puts("-------LEVEL----------\r\n");
	      start_setup_timeout();
		  water_level = 0;
		  water_level = get_water_level();

		  if (bit_is_set(water_level, MIN_LEVEL)) {
		  //uart_puts("MIN\r\n");
		    set_8segf(HG1, "FED");
		    set_8segf(HG2, "DBC");
		  } //else 
		  if (bit_is_set(water_level, MID_LEVEL)) {
		    //uart_puts("MID\r\n");
		    set_8segf(HG1, "FEDG");
		    set_8segf(HG2, "BCDG");
		  } //else 
		  if (bit_is_set(water_level, MAX_LEVEL)) {
		    //uart_puts("MAX\r\n");
		    set_8segf(HG1, "FEDGA");
		    set_8segf(HG2, "BCDGA");
		  } //else {
		  // пустой бак
		   // set_8segf(HG1, "FE");
		   // set_8segf(HG2, "BC");
		  //}
	      // дать время на отсветку уровня воды в баке
		  while (!setup_timeout_flag) continue;
		  set_8segf(HG1, "");
		  set_8segf(HG2, "");
          break;		
		case RUN_KEY:
		  // [idle] -> [run]
		  dev_state = RUN;
	      run_led_on();		 
		  _delay_ms(200);
	      break;			 
	    case PLUS_KEY:		  
	    case MINUS_KEY:
		  // [idel] -> [setup]		
	      dev_state = SETUP;
		  for (uint8_t i = 0; i < 3; i++) {
		    set_8segf(HG1, "G");
		    set_8segf(HG2, "G");
            _delay_ms(100);
            set_8segf(HG1, "");
		    set_8segf(HG2, ""); 
		    _delay_ms(100);
          }
		  start_setup_timeout();
		  //_delay_ms(200);
	      break;
         
		} // oef switch
	  }	// oef if
	  break;
    case SETUP: // -------------------------------------------
	  if (display_flag) {
	    display_flag = 0;        
        
        switch (blink_disp) {
        case ON:
		  blink_disp = OFF;
          put_temper(user_temper);
		  break;
		case OFF:
		  blink_disp = ON;
          set_8segf(HG1, "");
		  set_8segf(HG2, "");
		  break;
		}
	  } // eof if
      
      // по таймауту на установку температуры
      if (setup_timeout_flag) {
	    // записать установленную температуру и уйти в IDLE
        // [setup] -> [idle]
		eeprom_busy_wait();
		eeprom_update_word(&nv_user_temper, user_temper);
	    dev_state = IDLE;		
	  } else {
	    // обработать изменения температуры пользователем
	    key = get_key();
		switch (key) {
	    case RUN_KEY:
		  // [setup] -> [run]
		  stop_setup_timeout();
		  run_led_on();
	      // записать установленную температуру и уйти в RUN
          eeprom_busy_wait();
		  eeprom_update_word(&nv_user_temper, user_temper);		  
		  dev_state = RUN;
		  _delay_ms(200);       
		  break;			 
	    case PLUS_KEY:
		  // увеличть значение желаемой температуры
		  restart_setup_timeout();
	      user_temper += 10;//++;
		  if (1000 < user_temper) {
		    user_temper = 990;
		  }
		  put_temper(user_temper);
	      break;
	    case MINUS_KEY:
		  // уменьшить значение желаемой температуры		
	      restart_setup_timeout();
		  user_temper -= 10;//--;
	      if (0 > user_temper) {
		    user_temper = 0;
		  }
		  put_temper(user_temper);
		  break;
        } // eof switch
	  } // eof if-else
	  break;
    case PUMP_SETUP: // -------------------------------------------
	  
	  //==============================================================
	  // опрос АЦП для регириовки интервала бездействия помпы
      //uint16_t adc_read(uint8_t prescaler, uint8_t vref, uint8_t pin);  
	  if (read_adc_flag) {
	    read_adc_flag = 0;
	    adc_val = adc_read(ADC_PRESCALER_2, ADC_VREF_AVCC, ADC_PIN);
	   
	    // сбросить счётчик времени до выхода из сост. Установка помпы
        if (adc_level != prev_adc_level) {
	      // записать новое значение
	      prev_adc_level = adc_level;
          restart_setup_timeout();	
	      // [idle] -> [pump_setup]
	    }
     
	    //==============================================================
	  
	    // по таймауту на установку времени простоя ГЦН уйти в сост. Простой
        if (setup_timeout_flag) {
	      // запомнить установленний интервал и уйти в IDLE
          // [pump_setup] -> [idle]
		  dev_state = IDLE;		
	    } else {	
	      // определение диаразона и установить интервал простоя ГЦН в минутах либо отключить эту опцию
	      uint8_t done = 0;
	      for (uint8_t level = 1; level < LEVELS && !done; level++) {
            uint16_t threshold_lo = (level-1) * STEP;
	        uint16_t threshold_hi = level * STEP;

	        // при попадении в диапазон, установить соотв. интервал времени простоя ГЦНа.
	        if (adc_val >= threshold_lo && adc_val < threshold_hi) {
              // расчёт интервала в зависимости от уровня
		      // по идее шаг изменения времени простоя ГЦН = 5 минут
		      // время простоя ГЦН = уровень * шаг изменения времени простоя ГЦН
		      pump_inactive_interval = level * PUMP_DELTA_INACTIVE_INTERVAL;
		      adc_level = level;
		      done = 1;
	       }
      	  
	       // если значение вне диапазона, установить значение интрервала 
	       // времени простоя ГЦНа рвным нулю(Отключить эту опцию) 
	       if (adc_val > DIAPASONE) {
	         pump_inactive_interval = 0; //
		     adc_level = level;
		     done = 1;
	       } 
	      } // eof for
	    } // eof if-else      
	  } // eof if adc read flag
	  
	  // отобразить интервал времени простоя ГЦН в минутах 
      if (display_flag) {
	    display_flag = 0;        
        
        switch (blink_disp) {
        case ON:
		  blink_disp = OFF;
          put_inactive_interval(pump_inactive_interval);
		  break;
		case OFF:
		  blink_disp = ON;
          set_8segf(HG1, "");
		  set_8segf(HG2, "");
		  break;
		}
	  } // eof if

	  break;
    case RUN: // -------------------------------------------
	 // _delay_ms(200);
	  //key = get_key();
	  //switch (key) {
	  //  case RUN_KEY:
		  //[run] -> [idle]
	//	  run_led_off();
	//	  dev_state = IDLE;
	//	  _delay_ms(200);          		 
	//	  break;
      //}
      dev_state = process_water_tank(water_level, water_pressure, half_tank_mode, water_temper);//, user_temper);
	  _delay_ms(200); // возможно не нужна
	  run_led_off();
	  // dev_state -> [idle]
	  //           -> [fault]
	  break;
    case FAULT: // -------------------------------------------      	  
	  // закрыть все заслонки, выключить насосы и тены
      heater_relay_off();
      pump_relay_off();
      valve_relay_off();
	  
	  // мигать диодом Отказ	          
      switch (blink) {
	  case ON:
	    fault_led_on();
        blink = OFF;
		_delay_ms(150);
        break;
	  case OFF:
	    fault_led_off();
	    blink = ON;
		_delay_ms(150);
		break;
	  }

      // выкатить на дисплей ошибку
	  if (display_flag) {
	    display_flag = 0;
	    set_8segf(HG1, "ADEFG");
		if (bit_is_set(faults, LEVEL_SENS_FAULT)) {
		  _delay_ms(200);
          put(HG2, LEVEL_SENS_FAULT);
		}
		if (bit_is_set(faults, TEMPER_SENS_FAULT)) {
		  _delay_ms(200);
          put(HG2, TEMPER_SENS_FAULT);
		}
		if (bit_is_set(faults, PRESSURE_SENS_FAULT)) {
		  _delay_ms(200);
          put(HG2, PRESSURE_SENS_FAULT);
		}
	  } // eof if
	  
      


	  // обработка отказов с попыткой восстановления	  
      if (bit_is_set(faults, LEVEL_SENS_FAULT)) {
	    // опросить датчик уровня воды в баке
	    water_level = get_water_level();
		if (water_level) {
		  faults &= ~(1 << LEVEL_SENS_FAULT);
		}
	  }

	  if (bit_is_set(faults, TEMPER_SENS_FAULT)) {
	  	// найти подключеные термодатчики	  	  
		nSensors = search_sensors();
		if (nSensors > 0) {
		  faults &= ~(1 << TEMPER_SENS_FAULT); 
	    }
	  }

	  if (bit_is_set(faults, PRESSURE_SENS_FAULT)) {
	  	 // опросить датчик давления воды в трубе
	     water_pressure = get_water_pressure();
		 if (water_pressure) {
		   no_water_pressure_led_off();
		   faults &= ~(1 << PRESSURE_SENS_FAULT);
		 }    
	  }

	  // обработать нажатую клавишу, если таковая имеется
	  // сброс неисправности(принудитлеьно прейти в сост. Простой)
	  key = get_key();
	  switch (key) {
	  case RUN_KEY:
	    // [fault] -> [idle]
	    dev_state = IDLE;
	    fault_led_off();		 
	    _delay_ms(200);
	    break;		  
      }
	  
	  // если отказ утранён то уйти в сост. Простой
	  if (!faults) {
	    // [fault] -> [idle]
	    dev_state = IDLE;
		fault_led_off();
	  }      
	  break;
	} // eof switch
	// --------------------------------------------------------------------

  } // eof main loop

  return 0;
}















//-------------------------------------------------------------
void nv_init(void)
{
  // прочитать региср состояния устройсво
  eeprom_busy_wait();
  uint8_t dvsreg = eeprom_read_byte(&DVSR);

  // если ПЗУ не проинициализирована то записать в нее занчения по умолчанию
  if (bit_is_set(dvsreg, EEIF)) {
  
    // записать занчение пользовательской температуы 350 раное 35 *С 
	eeprom_busy_wait();
	eeprom_update_word(&nv_user_temper, 350);

    // установить бит инициализации ПЗУ(регистр инверсный)
    dvsreg &= ~(1 << EEIF);
	// записать региср состояния устройства в ПЗУ
	eeprom_busy_wait(); 
    eeprom_update_byte(&DVSR, &dvsreg);
    
	// __DEBUG
	//uart0_puts("WR 'empty'|0 to EEMEM\r\n");
  }
  
  return;
}

//-------------------------------------------------------------
static uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	//uart_puts_P( NEWLINESTR " [*] Scanning Bus for DS18X20..." NEWLINESTR );
	
	ow_reset();

	nSensors = 0;
	
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) {
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) {
			//uart_puts_P( " [!] No Sensor found" NEWLINESTR );
			break;
		}
		
		if( diff == OW_DATA_ERR ) {
			//uart_puts_P( " [!] Bus Error" NEWLINESTR );
			break;
		}
		
		for( i=0; i < OW_ROMCODE_SIZE; i++ ) {
			gSensorIDs[nSensors][i] = id[i];
		}
		
		nSensors++;
	}
	
	return nSensors;
}
//-------------------------------------------------------------
int16_t get_temper(uint8_t nSensors, uint8_t *error)
{
  int16_t decicelsius = 0; 

  if ( DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL ) 
		== DS18X20_OK) {
		_delay_ms( DS18B20_TCONV_12BIT );
		for (uint8_t i = 0; i < nSensors; i++ ) {
			if ( DS18X20_read_decicelsius( &gSensorIDs[i][0], &decicelsius )
			     == DS18X20_OK ) {
				 // TODO output temperature for examle
			}
			else {
				*error++;
			}
		}
	} else {
		*error++;
	}

	return decicelsius;
}





//-------------------------------------------------------------
void self_test(void)
{

   return;
}




//-------------------------------------------------------------
uint8_t get_water_level(void)
{	
  uint8_t water_level = MIN_LEVEL;//ERROR_LEVEL;

  #define LEVEL_SENS_BOUNCE_DEALY 60

  // настроим порт на ввод
  DDRC &= ~(1 << MIN_LEVEL_PIN);
  DDRD &= ~(1 << MID_LEVEL_PIN);
  DDRD &= ~(1 << MAX_LEVEL_PIN);
  
	
  // проверям датчик уровня на отметке MIN
  if (bit_is_clear(PINC, MIN_LEVEL_PIN)) {
    _delay_ms(LEVEL_SENS_BOUNCE_DEALY);
    if (bit_is_clear(PINC, MIN_LEVEL_PIN)) {
      water_level |= (1 << MIN_LEVEL);
    }
  }
  // проверям датчик уровня на отметке MID
  if (bit_is_clear(PIND, MID_LEVEL_PIN)) {
	_delay_ms(LEVEL_SENS_BOUNCE_DEALY);
	if (bit_is_clear(PIND, MID_LEVEL_PIN)) {
	  water_level |= (1 << MID_LEVEL);
    }
  }
  // проверям датчик уровня на отметке MAX
  if (bit_is_clear(PIND, MAX_LEVEL_PIN)) {
	_delay_ms(LEVEL_SENS_BOUNCE_DEALY);
	if (bit_is_clear(PIND, MAX_LEVEL_PIN)) {
	  water_level |= (1 << MAX_LEVEL);
    }
  }

    
  // проверка на корректность измерений, если датчик уровня не исправен, то вернуть ошибку датчика
  //if (not MIN and (MAX or MID)) ERROR_LEVEL
  if (bit_is_clear(water_level, MIN_LEVEL) && (bit_is_set(water_level, MID_LEVEL) || bit_is_set(water_level, MAX_LEVEL))) {
    water_level = ERROR_LEVEL;
  }
  //if (MIN ana not MID and MAX) ERROR_LEVEL
  if (bit_is_set(water_level, MIN_LEVEL) && bit_is_clear(water_level, MID_LEVEL) && bit_is_set(water_level, MAX_LEVEL)) {
    water_level = ERROR_LEVEL;
  }
	
  return water_level;
}


//-------------------------------------------------------------
uint8_t get_water_pressure(void)
{
  uint8_t water_pressure = NO_PRESSURE;

  PRESSURE_DDR &= (1 << PRESSURE_SENS_PIN);

  #define PRESSURE_SENS_BOUNCE_DEALY 60

  // проверям датчик давления на наличие давления воды в трубе
  if (bit_is_clear(PRESSURE_PINS, PRESSURE_SENS_PIN)) {
	_delay_ms(PRESSURE_SENS_BOUNCE_DEALY);
	if (bit_is_clear(PRESSURE_PINS, PRESSURE_SENS_PIN)) {
	  water_pressure = PRESSURE;
    }
  }

  return water_pressure;
}


//-------------------------------------------------------------
// returns device state = IDLE | FAULT.
uint8_t heat_water(int16_t water_temper)//, int16_t user_temper)
{
  uint8_t next_state = IDLE;
  uint8_t nSensors = 0;
  uint8_t error = 0;
  uint8_t stop = 0;
  uint8_t key = NO_KEYS; 
  
  // получить значение последней сохранённой пользователской уставки тепмпературы
  eeprom_busy_wait();
  int16_t user_temper = eeprom_read_word(&nv_user_temper);

  // без это проверки, повисал или уходил незнамо куда, не разбирались
  if (user_temper <= water_temper) return next_state;
    
  // установить состояние анимации нагрева на начало
  heat_anim_state = STAGE_0;
	    
  // включить реле ТЭНа на время пока температура утсавки пользователя 
  // не сравняется с температурой воды в баке или не возникнет отказ
  heater_relay_on();
  while (water_temper < user_temper && !stop) {    	
    // обработать нажатую клавишу, если таковая имеется
	key = get_key();
	switch (key) {
	case RUN_KEY:
	  // [run] -> [idle]
	  dev_state = IDLE;
	  run_led_off();		 
	  _delay_ms(200);
	  stop = 1;
	  // выключить нагреватель
	  heater_relay_off(); 
	  break;
	}  	      
    
	// управлять главным рециркуляционным насосом
	switch (pump_state) {
	case ACTIVE:
	  pump_relay_on();
	  break;
    case INACTIVE:
	  pump_relay_off();
	  break;	
	}   
	
	// получить количество найденных термодатчиков
    nSensors = search_sensors();
	
    if (0 == nSensors) {
	  error++;
	}

	// опрсить датчик если пришло время опроса
	if (get_temper_flag) {
	  get_temper_flag = 0;
      // замерять температуру воды в баке
	  water_temper = get_temper(nSensors, &error);
	  // если возникла ошибка измерения температуры, уйти в обработку ошибок
	  if (error) {
        // [run]->[fault]
		next_state = FAULT;
		fault_led_on();
		norm_led_off();
		faults |= (1 << TEMPER_SENS_FAULT);
		stop = 1;		    
	   } else {	      
         //иначе вывести полученную температуру на инидкатор
         if (display_flag) {
		   heat_anim_state = STAGE_0;
	       display_flag = 0;             
		   put_temper(water_temper);
		   _delay_ms(1750); // просто магическое число подобранное ипирически - время отображения температуры :)
		 }
	   }
	   // задержка между измерениями
	   _delay_ms(50); // Магическая задержка
	   error = 0;
	} else { 
	  // анимация нагрева воды
	   switch (heat_anim_state) {
	   case STAGE_0: 
	     heat_anim_state = STAGE_1;
         set_8segf(HG1, "");
         set_8segf(HG2, "");
		 _delay_ms(300);
		 break;
	   case STAGE_1:
         heat_anim_state = STAGE_2;
         set_8segf(HG1, "E");
         set_8segf(HG2, "E");
		 _delay_ms(300);
	     break;
	   case STAGE_2:
         heat_anim_state = STAGE_3;
	     set_8segf(HG1, "EG");
         set_8segf(HG2, "EG");
	     _delay_ms(300);
		 break;
       case STAGE_3:
         heat_anim_state = STAGE_0;
	     set_8segf(HG1, "EGB");
         set_8segf(HG2, "EGB");
		 _delay_ms(300); 
	     break;
	   } // eof switch
	}// oef if-else
  } // eof while
  // выключить ТЭН
  heater_relay_off(); 
  return;
}

//-------------------------------------------------------------
uint8_t fill_tank(uint8_t water_level, uint8_t setting)
{
  uint8_t next_state = IDLE;
  uint8_t stop = 0;

  // открыть клапан
  valve_relay_on();
  // ждать наполнения бочки водой до уровня MAX
  while (bit_is_clear(water_level, /*MAX_LEVEL*/setting) && !stop) {
    water_level = get_water_level();
    // если возникла ошибка датчика уровня 
	if (!water_level) { // ERROR_LEVEL == water_level
	// [run]->[fault]
      next_state = FAULT;
	  //fault_led_on();
	  norm_led_off();
      faults |= (1 << LEVEL_SENS_FAULT);
	  stop = 1; 
	}
	// задержка между измерениями
		 _delay_ms(50); 
  } // eof while
  // закрыть клапан
  valve_relay_off();

  return next_state;  
}

//-------------------------------------------------------------
// returns device state = IDLE | FAULT.
uint8_t process_water_tank(uint8_t water_level, uint8_t water_pressure, uint8_t half_tank_mode, int16_t water_temper)//, int16_t user_temper)
{   
  uint8_t next_state = RUN;
  uint8_t stop = 0;
  uint8_t key;


  fill_tank_anim_state = STAGE_0;
  // иможет быть вынести наполнение бочки в процедуру???

  if (bit_is_set(water_level, MAX_LEVEL)) { 			 // MAX Level
	  next_state = heat_water(water_temper);//, user_temper);
  } else if (bit_is_set(water_level, MID_LEVEL)) { 		 // MID Level 
    if (half_tank_mode) {
	   next_state = heat_water(water_temper);//, user_temper);
	} else if (water_pressure) {
	   // while water_level != MAX open water valve
	   
	   // открыть клапан
	   valve_relay_on();
	   // ждать наполнения бочки водой до уровня MAX
	   while (bit_is_clear(water_level, MAX_LEVEL) && !stop) {
         water_level = get_water_level();
		 
		// обработать нажатую клавишу, если таковая имеется
		key = get_key();
		switch (key) {
		case RUN_KEY:
		  // [run] -> [idle]
		  next_state = IDLE;
	      run_led_off();		 
		  _delay_ms(200);
		  stop = 1;
		  // закрыть клапан
	      valve_relay_off();
		  //return next_state; // это надо пофиксить!!!!
	      break;
		}  	   
		 
		 // анимация наполнения бака
		 switch (fill_tank_anim_state) {
		 case STAGE_0:
		   fill_tank_anim_state = STAGE_1;
		   set_8segf(HG1, "EF");
		   set_8segf(HG2, "BC");
		   _delay_ms(300);
		   break;
         case STAGE_1:
           fill_tank_anim_state = STAGE_2;
		    set_8segf(HG1, "EFD");
			set_8segf(HG2, "DBC");
		   _delay_ms(300);
		   break;
         case STAGE_2:
           fill_tank_anim_state = STAGE_3;
		   set_8segf(HG1, "EFDG");
		   set_8segf(HG2, "DGBC");
		   _delay_ms(300);
		   break;
         case STAGE_3:
		   fill_tank_anim_state = STAGE_0;
           set_8segf(HG1, "EFDGA");
		   set_8segf(HG2, "DGABC");
		   _delay_ms(300);
		   break;
		 } // eof switch
		 
		 // если возникла ошибка датчика уровня 
		 if (!water_level) { // ERROR_LEVEL == water_level
		   // [run]->[fault]
		   next_state = FAULT;
		   fault_led_on();
		   norm_led_off();
           faults |= (1 << LEVEL_SENS_FAULT);
		   stop = 1; 
		 }
		 // задержка между измерениями
		 _delay_ms(50); 
	   }
       // закрыть клапан
	   valve_relay_off();
	   
	   //next_state = fill_tank(water_level, MID_LEVEL);
	   //if (FAULT != next_state /*&& IDLE != next_state*/) {
	   if (RUN == next_state) {
	     next_state = heat_water(water_temper);//, user_temper);
	   } 
	} else {
	  // [run]->[fault]
	  no_water_pressure_led_on();
	  fault_led_on();
	  norm_led_off();
	  faults |= (1 << PRESSURE_SENS_FAULT);
	  next_state = FAULT;
	} 
  }	else {//if (bit_is_set(water_level, MIN_LEVEL)) { 	// MIN Level
    if (half_tank_mode) {
	
	 if (water_pressure) {
	  	   
	   // открыть клапан
	   valve_relay_on();
	   // ждать наполнения бочки водой до уровня MID
	   while (bit_is_clear(water_level, MID_LEVEL) && !stop) {
         water_level = get_water_level();
         
		// обработать нажатую клавишу, если таковая имеется
		key = get_key();
		switch (key) {
		case RUN_KEY:
		  // [run] -> [idle]
		  next_state = IDLE;
	      run_led_off();		 
		  _delay_ms(200);
		  stop = 1;
		  // закрыть клапан
	      valve_relay_off();
		  //return next_state; // это надо пофиксить!!!!
	      break;
		} 	   
		 
		 // анимация наполнения бака
		 switch (fill_tank_anim_state) {
		 case STAGE_0:
		   fill_tank_anim_state = STAGE_1;
		   set_8segf(HG1, "EF");
		   set_8segf(HG2, "BC");
		   _delay_ms(300);
		   break;
         case STAGE_1:
           fill_tank_anim_state = STAGE_2;
		    set_8segf(HG1, "EFD");
			set_8segf(HG2, "DBC");
		   _delay_ms(300);
		   break;
         case STAGE_2:
           fill_tank_anim_state = STAGE_3;
		   set_8segf(HG1, "EFDG");
		   set_8segf(HG2, "DGBC");
		   _delay_ms(300);
		   break;
         case STAGE_3:
		   fill_tank_anim_state = STAGE_0;
           set_8segf(HG1, "EFDGA");
		   set_8segf(HG2, "DGABC");
		   _delay_ms(300);
		   break;
		 } // eof switch

		 // если возникла ошибка датчика уровня 
		 if (!water_level) { // ERROR_LEVEL == water_level
		   // [run]->[fault]
		   next_state = FAULT;
		   fault_led_on();
		   norm_led_off();
           faults |= (1 << LEVEL_SENS_FAULT);
		   stop = 1; 
		 }
		 // задержка между измерениями
		 _delay_ms(50); 
	   }
       // закрыть клапан
	   valve_relay_off();
	   
	   //next_state = fill_tank(water_level, MID_LEVEL);
	   //if (FAULT != next_state /*&& IDLE != next_state*/) {
	   if (RUN == next_state) {
	     next_state = heat_water(water_temper);//, user_temper);
	   }
	   
     } else {
	   // [run]->[fault]
	   no_water_pressure_led_on();
	   fault_led_on();
	   norm_led_off();
	   faults |= (1 << PRESSURE_SENS_FAULT);
	   next_state = FAULT;
	 }
	   
	    
	} else if (water_pressure) {	   
	   // открыть клапан
	   valve_relay_on();
	   // ждать наполнения бочки водой до уровня MAX
	   while (bit_is_clear(water_level, MAX_LEVEL) && !stop) {
         water_level = get_water_level();
		 
        // обработать нажатую клавишу, если таковая имеется
		key = get_key();
		switch (key) {
		case RUN_KEY:
		  // [run] -> [idle]
		  next_state = IDLE;
	      run_led_off();		 
		  _delay_ms(200);
		  stop = 1;
		  // закрыть клапан
	      valve_relay_off();
		  //return next_state; // это надо пофиксить!!!!
	      break;
		}  	   
		 
		 // анимация наполнения бака
		 switch (fill_tank_anim_state) {
		 case STAGE_0:
		   fill_tank_anim_state = STAGE_1;
		   set_8segf(HG1, "EF");
		   set_8segf(HG2, "BC");
		   _delay_ms(300);
		   break;
         case STAGE_1:
           fill_tank_anim_state = STAGE_2;
		    set_8segf(HG1, "EFD");
			set_8segf(HG2, "DBC");
		   _delay_ms(300);
		   break;
         case STAGE_2:
           fill_tank_anim_state = STAGE_3;
		   set_8segf(HG1, "EFDG");
		   set_8segf(HG2, "DGBC");
		   _delay_ms(300);
		   break;
         case STAGE_3:
		   fill_tank_anim_state = STAGE_0;
           set_8segf(HG1, "EFDGA");
		   set_8segf(HG2, "DGABC");
		   _delay_ms(300);
		   break;
		 } // eof switch		 

		 // если возникла ошибка датчика уровня 
		 if (!water_level) { // ERROR_LEVEL == water_level
		   // [run]->[fault]
		   next_state = FAULT;
		   fault_led_on();
		   norm_led_off();
           faults |= (1 << LEVEL_SENS_FAULT);
		   stop = 1; 
		 }
		 // задержка между измерениями
		 _delay_ms(50); 
	   }
       // закрыть клапан
	   valve_relay_off();
	   
	   //next_state = fill_tank(water_level, MAX_LEVEL);
	   //if (FAULT != next_state /*&& IDLE != next_state*/) {
	   if (RUN == next_state) {
	     next_state = heat_water(water_temper);//, user_temper);
	   }  
	} else {
	   // [run]->[fault]
	   no_water_pressure_led_on();
	   fault_led_on();
	   norm_led_off();
	   faults |= (1 << PRESSURE_SENS_FAULT);
	   next_state = FAULT;
	}
  }	//eof if-else if - else 
	       
  return next_state;
}


