/*****
 * name: WaterChip 
 * ���������� ��������� ���� � ���������� �������� ���� � ��
 * ����: 16.08.2014
 * ������: 0.1b
 * 
 *
 ***/
#ifndef INDICATION_H
#define INDICATION_H

#include "spilight/spilight.h"

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

#define NORM_LED_DDR	DDRD
#define FAULT_LED_DDR	DDRD
#define RUN_LED_DDR		DDRB

#define NORM_LED_PORT	PORTD
#define FAULT_LED_PORT	PORTD
#define RUN_LED_PORT	PORTB

#define NORM_LED	PD7 
#define RUN_LED 	PB0
#define FAULT_LED	PD6


// 8 SEGMENTS INDICATION
#define HG1_STROBE PB2
#define HG2_STROBE PB1
#define HG1 0
#define HG2 1
#define CLR_CODE 0xFE
// SEGMENTS BITS
#define A_SEG 1
#define B_SEG 2
#define C_SEG 3
#define D_SEG 4
#define E_SEG 5
#define F_SEG 6
#define G_SEG 7
#define H_SEG 8

//
void set_8segf(uint8_t hg, char *segments);
void set_hg1(uint8_t code);
void set_hg2(uint8_t code);
// ���� �����
void norm_led_on(void);
void norm_led_off(void);
// ���� ������
void fault_led_on(void);
void fault_led_off(void);
// ���� ������
void run_led_on(void);
void run_led_off(void);
// ���� ��� ����
void half_tank_led_on(void);
void half_tank_led_off(void);
// ���� ��� �������� ���� � ����
void no_water_pressure_led_on(void);
void no_water_pressure_led_off(void);
// ��� ������ ���� ���������� ���������, �� ���� ���� ��� ��� �����������...
//ow_set_bus( volatile uint8_t* in,
//	volatile uint8_t* out,
//	volatile uint8_t* ddr,
//	uint8_t pin );
// ������ �������������
//	ow_set_bus(&PIND, &PORTD, &DDRD, PD4);
void set_hg1(uint8_t code);
void set_hg2(uint8_t code);
void set_8segf(uint8_t hg, char *segments);
void put(uint8_t hg, uint8_t c);
// ������������� �������
void put_temper(int16_t decicelsius);
void put_inactive_interval(int16_t interval);
void put_fault(uint8_t faultno);
#endif //INDICATION_H
