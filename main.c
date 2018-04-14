/*****
 * ������������: WaterChip 
 * ��������: ���������� ��������� ���� � ���������� �������� ���� � ��
 * ���� ��������: 16.08.2014
 * ���� ����������: 16.12.2014
 * ������: 1.1 �� ������� ������������
 * 
 * ChangeLog:
 * 1.0 ����������� ��� �������(������, ���������� ����, �������� ��������, ���������, ��������� ������)
 * 1.1 ������������ ��������� ������, ���� �� ������, ����� �� ����������
 *     ����� ���������� ������� ������� ������ ����, ��� ��������� ��������� �������� �������
 *
 * ����������� � �������� ��� ������
 * TODO:
 *  ��������� �������� �������
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

// ���
#define ADC_PIN 6
// �������� � � ��� ��� ���������� ������� ������� ���
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

#define DISP_INTERVAL_VALUE 15 // ��������� � ������ ��� ������ 
static uint8_t disp_interval = DISP_INTERVAL_VALUE; // �������� ������ ������� ���������� ~ 1 ���
static uint8_t display_flag = 1;

#define MEASURE_INTERVAL_VALUE 250 // ��������� � ������ ��� ������ 
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

// ������� ��������� ���������� DVSR
#define EEIF 0  // ���� ������������� ���
//���      
// 0   EEIF        ���� ������������� ����� ��� ���������� ������ � EEPROM ���� ��� �� ������������������� 0 - ������������������� 1 - �� �������������������
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


// ���������� ������
#define TIK_PER_SEC 30 // ~1 min
#define SEC_PER_MIN 60
#define DEFAULT_INACTIVE_INTERVAL_MIN 0 // �������� 
#define ACTIVE_INTERVAL_MIN 15 // [��� �����: 100 � / 7 �/���  ~ 15 ���]  �� ������ ����� �������� ������� 10
#define ACTIVE 1
#define INACTIVE 0
static volatile uint8_t pump_state = INACTIVE;
static volatile uint8_t pump_inactive_interval = DEFAULT_INACTIVE_INTERVAL_MIN; // �������� ������� � ���
static const volatile uint8_t pump_active_interval = ACTIVE_INTERVAL_MIN; // ����������� �����������
static volatile uint8_t minutes = 0; 
static volatile uint8_t seconds = SEC_PER_MIN;
static volatile uint8_t tik_per_sec = TIK_PER_SEC;
static volatile uint8_t inactive_minutes_setting = DEFAULT_INACTIVE_INTERVAL_MIN;
//static unit8_t seconds_setting = 0;


#define ADC_READ_INTERVAL 40;
static volatile uint8_t read_adc_flag = 0;
static volatile uint8_t adc_read_interval = ADC_READ_INTERVAL;

// ��������� ������ �������
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

  

  ���� �� �����
  PD5 ���
  PD6 ���
  PD7 ����
  PB0 ����

  ���������� ������� ���������
  SG1 PC0 �������
  SG2 PB1 �������
 */



static volatile uint8_t blink_t0 = OFF;

//-------------------------------------------------------------
ISR (TIMER0_OVF_vect)
{
  TCCR0B = 0;

  // �������� ����� ������� ��� ���������� ������
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
  
  
  // ������� �������� ������ �������� �� �������
  if (disp_interval <= 0) {
    disp_interval = DISP_INTERVAL_VALUE;
	display_flag = 1;
  } else {
    disp_interval--;
  }

  // ������� �������� ������ ���
  if (adc_read_interval <= 0) {
    adc_read_interval = ADC_READ_INTERVAL;
	read_adc_flag = 1;
  } else {
    adc_read_interval--;
  }

  // ������� �������� ������ ������������
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
  // ���-�� ������

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

//=====================������ ��� ��������� ������� ������� ������, ��� ������� ������� =====
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
  // ��������� ������ �1 ��� ������� ����� �������
  TIMSK1 |= (1 << TOIE1);
  TCCR1B = 0;
  TCNT1H = 0xF0; // �� �����
  TCNT1L = 0x01; // �� �����
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
  
  // ��������� ������ �0
  TIMSK0 |= (1 << TOIE0);
  TCNT0 = 0x01;
  TCCR0B |= (1 << CS02) | (1 << CS00);   

  
  // ��������� ������ �1 ��� ������� ����� �������
  TIMSK1 |= (1 << TOIE1);
  TCCR1B = 0;
  TCNT1H = 0xF0;
  TCNT1L = 0x01;
  TCCR1B |= (1 << CS12) | (1 << CS10);   

  // init_leds()   
  DDRD |= (1 << PD5);

  // ���������� ���������� ������������� ������������ ����� 
  // ��� ���� ���-�� ��������� � ���������...
  DDRB |= (1 << PB1); // ������� ������ SG2
  DDRB |= (1 << PB2); // ������� ������ SG1


  // ����� 20141216
  // ��������� ���������� ��� ��������� ������
  init_keys();  

  init_spi_master();
  // ���� �������� �������� ������ ��� �� ������������� ���������� ���
  nv_init();

  sei();

  // ������� ��� ��������, ��������� ������ � ����
  heater_relay_off();
  pump_relay_off();
  valve_relay_off();


  // �������� ��� �����
  no_water_pressure_led_off();
  half_tank_led_off();
  norm_led_off();
  fault_led_off();
  run_led_off();

  // �������� ������� �������
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

  // ���������� ��� ������ � ������������� 
  uint8_t nSensors;
  int16_t water_temper;
  uint8_t error_sens;
  uint8_t key = NO_KEYS; 
  // �������� �������� ��������� ���������� ��������������� ������� ������������
  eeprom_busy_wait();
  int16_t user_temper = eeprom_read_word(&nv_user_temper);  
  uint8_t half_tank_mode = OFF;
  uint8_t half_tank_led_state = ON;
  uint8_t blink_disp = ON; 
  uint8_t water_level = 0;
  uint8_t water_pressure = NO_PRESSURE;
  uint16_t adc_val = 0;
  //uint16_t adc_val_prev = 0; // ������
  uint8_t adc_level = 0;
  uint8_t prev_adc_level = 0;

  // ���������� �������� ����������, ���� ������ �� ������ � ����. FAULT
  //start_self_test();
  
  // �������� ����� ��������
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

  // �������� ��� �����
  no_water_pressure_led_on();
  half_tank_led_on();
  norm_led_on();
  fault_led_on();
  run_led_on();

  set_8segf(HG1, "DEFABGC");
  set_8segf(HG2, "DEFABGC");
  _delay_ms(500);
  

  // �������� ��� �����
  no_water_pressure_led_off();
  half_tank_led_off();
  norm_led_off();
  fault_led_off();
  run_led_off();

  set_8segf(HG1, "");
  set_8segf(HG2, "");
  _delay_ms(500);

  // ������� ���������� �� ����� � ���
  adc_val = adc_read(ADC_PRESCALER_2, ADC_VREF_AVCC, ADC_PIN);
  _delay_ms(150);
  adc_val = adc_read(ADC_PRESCALER_2, ADC_VREF_AVCC, ADC_PIN);
  _delay_ms(150);
  
  // ��������� ����� ������� �������� ��� ��� ������� ��������� ��������
  adc_val = adc_read(ADC_PRESCALER_2, ADC_VREF_AVCC, ADC_PIN);
  // ����������� ��������� � ���������� �������� ������� ��� � ������� ���� ��������� ��� �����
	uint8_t done = 0;
	for (uint8_t level = 1; level < LEVELS && !done; level++) {
      uint16_t threshold_lo = (level-1) * STEP;
	  uint16_t threshold_hi = level * STEP;
	  // ��� ��������� � ��������, ���������� �����. �������� ������� ������� ����.
	  if (adc_val >= threshold_lo && adc_val < threshold_hi) {
        // ������ ��������� � ����������� �� ������
		// �� ���� ��� ��������� ������� ������� ��� = 5 �����
		// ����� ������� ��� = ������� * ��� ��������� ������� ������� ���
		pump_inactive_interval = level * PUMP_DELTA_INACTIVE_INTERVAL;
		adc_level = prev_adc_level = level;
		done = 1;
	  }	  
	  // ���� �������� ��� ���������, ���������� �������� ���������� 
	  // ������� ������� ���� ����� ����(��������� ��� �����) 
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
    // ������
	switch (dev_state) {
    case IDLE: // -------------------------------------------
	  // ������� ��� ��������, ��������� ������ � ����
      heater_relay_off();
      pump_relay_off();
      valve_relay_off();
	  
	  // �������� ������ ������ ���� � ����
	  water_level = get_water_level();
      // �������� ������ �������� ���� � �����
	  water_pressure = get_water_pressure();    	
      // �������� ��������� ��������� �������������
      nSensors = search_sensors();	  

      // �������� ������� �������
      faults = NO_FAULTS;
	        
	  // ��������� ��� �� ������� ��������
      // ���� ��� �������� ����, �� ���� � ����. ������
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

	  // ���� ������� ������ ������� ������, �� ���� � ����. ������
	  if (!water_level) {
	  	dev_state = FAULT;
		faults |= (1 << LEVEL_SENS_FAULT);        
	  	fault_led_on();
      	norm_led_off();
		// [idle] -> [fault]
	  }

      // ���� ��� ������� �����������, ���� � ����. ������ 
	  if ( nSensors == 0 ) {
		error_sens++;
		dev_state = FAULT;
		faults |= (1 << TEMPER_SENS_FAULT);
		fault_led_on();
      	norm_led_off();
		// [idle] -> [fault]
	  }
	  
      

	  // ���� ������� ���, �� ����������� �������� ������������ ����. ������� 
	  if (!faults) {    
	    
		norm_led_on();
	    fault_led_off();
				
		// ������� ������ ���� ������ ����� ������
	    if (get_temper_flag) {
	      get_temper_flag = 0;
		  // �������� ����������� ���� � ����
		  water_temper = get_temper(nSensors, &error_sens);
		  // ���� �������� ������ ��������� �����������, ���� � ��������� ������
		  //if (error_sens) { // ��� �� ������� ����� �� ��� �������� ��� ��� ???
		  //  error_sens = 0;
		//	fault_led_on();
		//	norm_led_off();
          //  dev_state = FAULT;
		    // [idle] -> [fault]
		  //} else {	      
            //����� ������� ���������� ����������� �� ���������
            if (display_flag) {
			  display_flag = 0;
			  put_temper(water_temper);
			}
		  //}
	    } // oef if
        
        //==============================================================
	   // ��������� ��� ��� ���������� ��������� ����������� �����
       //uint16_t adc_read(uint8_t prescaler, uint8_t vref, uint8_t pin);  
	   if (read_adc_flag) {
	     read_adc_flag = 0;
	     adc_val = adc_read(ADC_PRESCALER_2, ADC_VREF_AVCC, ADC_PIN);     
	     // ����������� ��������� � ���������� �������� ������� ��� � ������� ���� ��������� ��� �����
	     uint8_t done = 0;
	     for (uint8_t level = 1; level < LEVELS && !done; level++) {
           uint16_t threshold_lo = (level-1) * STEP;
	       uint16_t threshold_hi = level * STEP;
	       // ��� ��������� � ��������, ���������� �����. �������� ������� ������� ����.
	       if (adc_val >= threshold_lo && adc_val < threshold_hi) {	         
			 // ������� � ��������� �������� ��������� ������� ����
		     adc_level = level;
		     done = 1;
	       }
      	  
	       // ���� �������� ��� ���������, ���������� �������� ���������� 
	       // ������� ������� ���� ����� ����(��������� ��� �����) 
	       if (adc_val > DIAPASONE) {
		     adc_level = level;
		     done = 1;
	       } 
	     } // eof for
	   } // eof if read adc flag
       	   
	   if (adc_level != prev_adc_level) {
	     // �������� ����� ��������
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
		 // ������ ����� �� ��������� �� �������� �������� ���������� ��������� �� �������
		 start_setup_timeout();
	     // [idle] -> [pump_setup]
	   }     
	   //==============================================================

		// ���������� ������� �������, ���� ������� �������
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
		  // ������ ���
		   // set_8segf(HG1, "FE");
		   // set_8segf(HG2, "BC");
		  //}
	      // ���� ����� �� �������� ������ ���� � ����
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
      
      // �� �������� �� ��������� �����������
      if (setup_timeout_flag) {
	    // �������� ������������� ����������� � ���� � IDLE
        // [setup] -> [idle]
		eeprom_busy_wait();
		eeprom_update_word(&nv_user_temper, user_temper);
	    dev_state = IDLE;		
	  } else {
	    // ���������� ��������� ����������� �������������
	    key = get_key();
		switch (key) {
	    case RUN_KEY:
		  // [setup] -> [run]
		  stop_setup_timeout();
		  run_led_on();
	      // �������� ������������� ����������� � ���� � RUN
          eeprom_busy_wait();
		  eeprom_update_word(&nv_user_temper, user_temper);		  
		  dev_state = RUN;
		  _delay_ms(200);       
		  break;			 
	    case PLUS_KEY:
		  // �������� �������� �������� �����������
		  restart_setup_timeout();
	      user_temper += 10;//++;
		  if (1000 < user_temper) {
		    user_temper = 990;
		  }
		  put_temper(user_temper);
	      break;
	    case MINUS_KEY:
		  // ��������� �������� �������� �����������		
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
	  // ����� ��� ��� ���������� ��������� ����������� �����
      //uint16_t adc_read(uint8_t prescaler, uint8_t vref, uint8_t pin);  
	  if (read_adc_flag) {
	    read_adc_flag = 0;
	    adc_val = adc_read(ADC_PRESCALER_2, ADC_VREF_AVCC, ADC_PIN);
	   
	    // �������� ������� ������� �� ������ �� ����. ��������� �����
        if (adc_level != prev_adc_level) {
	      // �������� ����� ��������
	      prev_adc_level = adc_level;
          restart_setup_timeout();	
	      // [idle] -> [pump_setup]
	    }
     
	    //==============================================================
	  
	    // �� �������� �� ��������� ������� ������� ��� ���� � ����. �������
        if (setup_timeout_flag) {
	      // ��������� ������������� �������� � ���� � IDLE
          // [pump_setup] -> [idle]
		  dev_state = IDLE;		
	    } else {	
	      // ����������� ��������� � ���������� �������� ������� ��� � ������� ���� ��������� ��� �����
	      uint8_t done = 0;
	      for (uint8_t level = 1; level < LEVELS && !done; level++) {
            uint16_t threshold_lo = (level-1) * STEP;
	        uint16_t threshold_hi = level * STEP;

	        // ��� ��������� � ��������, ���������� �����. �������� ������� ������� ����.
	        if (adc_val >= threshold_lo && adc_val < threshold_hi) {
              // ������ ��������� � ����������� �� ������
		      // �� ���� ��� ��������� ������� ������� ��� = 5 �����
		      // ����� ������� ��� = ������� * ��� ��������� ������� ������� ���
		      pump_inactive_interval = level * PUMP_DELTA_INACTIVE_INTERVAL;
		      adc_level = level;
		      done = 1;
	       }
      	  
	       // ���� �������� ��� ���������, ���������� �������� ���������� 
	       // ������� ������� ���� ����� ����(��������� ��� �����) 
	       if (adc_val > DIAPASONE) {
	         pump_inactive_interval = 0; //
		     adc_level = level;
		     done = 1;
	       } 
	      } // eof for
	    } // eof if-else      
	  } // eof if adc read flag
	  
	  // ���������� �������� ������� ������� ��� � ������� 
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
	  _delay_ms(200); // �������� �� �����
	  run_led_off();
	  // dev_state -> [idle]
	  //           -> [fault]
	  break;
    case FAULT: // -------------------------------------------      	  
	  // ������� ��� ��������, ��������� ������ � ����
      heater_relay_off();
      pump_relay_off();
      valve_relay_off();
	  
	  // ������ ������ �����	          
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

      // �������� �� ������� ������
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
	  
      


	  // ��������� ������� � �������� ��������������	  
      if (bit_is_set(faults, LEVEL_SENS_FAULT)) {
	    // �������� ������ ������ ���� � ����
	    water_level = get_water_level();
		if (water_level) {
		  faults &= ~(1 << LEVEL_SENS_FAULT);
		}
	  }

	  if (bit_is_set(faults, TEMPER_SENS_FAULT)) {
	  	// ����� ����������� ������������	  	  
		nSensors = search_sensors();
		if (nSensors > 0) {
		  faults &= ~(1 << TEMPER_SENS_FAULT); 
	    }
	  }

	  if (bit_is_set(faults, PRESSURE_SENS_FAULT)) {
	  	 // �������� ������ �������� ���� � �����
	     water_pressure = get_water_pressure();
		 if (water_pressure) {
		   no_water_pressure_led_off();
		   faults &= ~(1 << PRESSURE_SENS_FAULT);
		 }    
	  }

	  // ���������� ������� �������, ���� ������� �������
	  // ����� �������������(������������� ������ � ����. �������)
	  key = get_key();
	  switch (key) {
	  case RUN_KEY:
	    // [fault] -> [idle]
	    dev_state = IDLE;
	    fault_led_off();		 
	    _delay_ms(200);
	    break;		  
      }
	  
	  // ���� ����� ������ �� ���� � ����. �������
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
  // ��������� ������ ��������� ���������
  eeprom_busy_wait();
  uint8_t dvsreg = eeprom_read_byte(&DVSR);

  // ���� ��� �� ������������������� �� �������� � ��� �������� �� ���������
  if (bit_is_set(dvsreg, EEIF)) {
  
    // �������� �������� ���������������� ���������� 350 ����� 35 *� 
	eeprom_busy_wait();
	eeprom_update_word(&nv_user_temper, 350);

    // ���������� ��� ������������� ���(������� ���������)
    dvsreg &= ~(1 << EEIF);
	// �������� ������ ��������� ���������� � ���
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

  // �������� ���� �� ����
  DDRC &= ~(1 << MIN_LEVEL_PIN);
  DDRD &= ~(1 << MID_LEVEL_PIN);
  DDRD &= ~(1 << MAX_LEVEL_PIN);
  
	
  // �������� ������ ������ �� ������� MIN
  if (bit_is_clear(PINC, MIN_LEVEL_PIN)) {
    _delay_ms(LEVEL_SENS_BOUNCE_DEALY);
    if (bit_is_clear(PINC, MIN_LEVEL_PIN)) {
      water_level |= (1 << MIN_LEVEL);
    }
  }
  // �������� ������ ������ �� ������� MID
  if (bit_is_clear(PIND, MID_LEVEL_PIN)) {
	_delay_ms(LEVEL_SENS_BOUNCE_DEALY);
	if (bit_is_clear(PIND, MID_LEVEL_PIN)) {
	  water_level |= (1 << MID_LEVEL);
    }
  }
  // �������� ������ ������ �� ������� MAX
  if (bit_is_clear(PIND, MAX_LEVEL_PIN)) {
	_delay_ms(LEVEL_SENS_BOUNCE_DEALY);
	if (bit_is_clear(PIND, MAX_LEVEL_PIN)) {
	  water_level |= (1 << MAX_LEVEL);
    }
  }

    
  // �������� �� ������������ ���������, ���� ������ ������ �� ��������, �� ������� ������ �������
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

  // �������� ������ �������� �� ������� �������� ���� � �����
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
  
  // �������� �������� ��������� ���������� ��������������� ������� ������������
  eeprom_busy_wait();
  int16_t user_temper = eeprom_read_word(&nv_user_temper);

  // ��� ��� ��������, ������� ��� ������ ������� ����, �� �����������
  if (user_temper <= water_temper) return next_state;
    
  // ���������� ��������� �������� ������� �� ������
  heat_anim_state = STAGE_0;
	    
  // �������� ���� ���� �� ����� ���� ����������� ������� ������������ 
  // �� ���������� � ������������ ���� � ���� ��� �� ��������� �����
  heater_relay_on();
  while (water_temper < user_temper && !stop) {    	
    // ���������� ������� �������, ���� ������� �������
	key = get_key();
	switch (key) {
	case RUN_KEY:
	  // [run] -> [idle]
	  dev_state = IDLE;
	  run_led_off();		 
	  _delay_ms(200);
	  stop = 1;
	  // ��������� �����������
	  heater_relay_off(); 
	  break;
	}  	      
    
	// ��������� ������� ���������������� �������
	switch (pump_state) {
	case ACTIVE:
	  pump_relay_on();
	  break;
    case INACTIVE:
	  pump_relay_off();
	  break;	
	}   
	
	// �������� ���������� ��������� �������������
    nSensors = search_sensors();
	
    if (0 == nSensors) {
	  error++;
	}

	// ������� ������ ���� ������ ����� ������
	if (get_temper_flag) {
	  get_temper_flag = 0;
      // �������� ����������� ���� � ����
	  water_temper = get_temper(nSensors, &error);
	  // ���� �������� ������ ��������� �����������, ���� � ��������� ������
	  if (error) {
        // [run]->[fault]
		next_state = FAULT;
		fault_led_on();
		norm_led_off();
		faults |= (1 << TEMPER_SENS_FAULT);
		stop = 1;		    
	   } else {	      
         //����� ������� ���������� ����������� �� ���������
         if (display_flag) {
		   heat_anim_state = STAGE_0;
	       display_flag = 0;             
		   put_temper(water_temper);
		   _delay_ms(1750); // ������ ���������� ����� ����������� ���������� - ����� ����������� ����������� :)
		 }
	   }
	   // �������� ����� �����������
	   _delay_ms(50); // ���������� ��������
	   error = 0;
	} else { 
	  // �������� ������� ����
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
  // ��������� ���
  heater_relay_off(); 
  return;
}

//-------------------------------------------------------------
uint8_t fill_tank(uint8_t water_level, uint8_t setting)
{
  uint8_t next_state = IDLE;
  uint8_t stop = 0;

  // ������� ������
  valve_relay_on();
  // ����� ���������� ����� ����� �� ������ MAX
  while (bit_is_clear(water_level, /*MAX_LEVEL*/setting) && !stop) {
    water_level = get_water_level();
    // ���� �������� ������ ������� ������ 
	if (!water_level) { // ERROR_LEVEL == water_level
	// [run]->[fault]
      next_state = FAULT;
	  //fault_led_on();
	  norm_led_off();
      faults |= (1 << LEVEL_SENS_FAULT);
	  stop = 1; 
	}
	// �������� ����� �����������
		 _delay_ms(50); 
  } // eof while
  // ������� ������
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
  // ������ ���� ������� ���������� ����� � ���������???

  if (bit_is_set(water_level, MAX_LEVEL)) { 			 // MAX Level
	  next_state = heat_water(water_temper);//, user_temper);
  } else if (bit_is_set(water_level, MID_LEVEL)) { 		 // MID Level 
    if (half_tank_mode) {
	   next_state = heat_water(water_temper);//, user_temper);
	} else if (water_pressure) {
	   // while water_level != MAX open water valve
	   
	   // ������� ������
	   valve_relay_on();
	   // ����� ���������� ����� ����� �� ������ MAX
	   while (bit_is_clear(water_level, MAX_LEVEL) && !stop) {
         water_level = get_water_level();
		 
		// ���������� ������� �������, ���� ������� �������
		key = get_key();
		switch (key) {
		case RUN_KEY:
		  // [run] -> [idle]
		  next_state = IDLE;
	      run_led_off();		 
		  _delay_ms(200);
		  stop = 1;
		  // ������� ������
	      valve_relay_off();
		  //return next_state; // ��� ���� ���������!!!!
	      break;
		}  	   
		 
		 // �������� ���������� ����
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
		 
		 // ���� �������� ������ ������� ������ 
		 if (!water_level) { // ERROR_LEVEL == water_level
		   // [run]->[fault]
		   next_state = FAULT;
		   fault_led_on();
		   norm_led_off();
           faults |= (1 << LEVEL_SENS_FAULT);
		   stop = 1; 
		 }
		 // �������� ����� �����������
		 _delay_ms(50); 
	   }
       // ������� ������
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
	  	   
	   // ������� ������
	   valve_relay_on();
	   // ����� ���������� ����� ����� �� ������ MID
	   while (bit_is_clear(water_level, MID_LEVEL) && !stop) {
         water_level = get_water_level();
         
		// ���������� ������� �������, ���� ������� �������
		key = get_key();
		switch (key) {
		case RUN_KEY:
		  // [run] -> [idle]
		  next_state = IDLE;
	      run_led_off();		 
		  _delay_ms(200);
		  stop = 1;
		  // ������� ������
	      valve_relay_off();
		  //return next_state; // ��� ���� ���������!!!!
	      break;
		} 	   
		 
		 // �������� ���������� ����
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

		 // ���� �������� ������ ������� ������ 
		 if (!water_level) { // ERROR_LEVEL == water_level
		   // [run]->[fault]
		   next_state = FAULT;
		   fault_led_on();
		   norm_led_off();
           faults |= (1 << LEVEL_SENS_FAULT);
		   stop = 1; 
		 }
		 // �������� ����� �����������
		 _delay_ms(50); 
	   }
       // ������� ������
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
	   // ������� ������
	   valve_relay_on();
	   // ����� ���������� ����� ����� �� ������ MAX
	   while (bit_is_clear(water_level, MAX_LEVEL) && !stop) {
         water_level = get_water_level();
		 
        // ���������� ������� �������, ���� ������� �������
		key = get_key();
		switch (key) {
		case RUN_KEY:
		  // [run] -> [idle]
		  next_state = IDLE;
	      run_led_off();		 
		  _delay_ms(200);
		  stop = 1;
		  // ������� ������
	      valve_relay_off();
		  //return next_state; // ��� ���� ���������!!!!
	      break;
		}  	   
		 
		 // �������� ���������� ����
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

		 // ���� �������� ������ ������� ������ 
		 if (!water_level) { // ERROR_LEVEL == water_level
		   // [run]->[fault]
		   next_state = FAULT;
		   fault_led_on();
		   norm_led_off();
           faults |= (1 << LEVEL_SENS_FAULT);
		   stop = 1; 
		 }
		 // �������� ����� �����������
		 _delay_ms(50); 
	   }
       // ������� ������
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


