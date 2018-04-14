/*****
 * name: WaterChip 
 * Контроллер наполения бака и управления нагревом воды в нём
 * Дата: 16.08.2014
 * Версия: 0.1b
 * 
 *
 ***/
#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <avr/version.h>
#if __AVR_LIBC_VERSION__ < 10606UL
#error "please update to avrlibc 1.6.6 or newer, not tested with older versions"
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdint.h>

// KEYS
#define KEYB_DDR		DDRC
#define KEYB_PINS		PINC
#define RUN_PIN			PINC1 
#define PLUS_PIN		PINC2
#define MINUS_PIN		PINC3
#define HALF_TANK_PIN	PINC4
#define NO_KEYS			0
#define RUN_KEY			1 
#define PLUS_KEY		2
#define MINUS_KEY		3
#define HALF_TANK_KEY	4
#define GET_LEVEL_KEY	5

// typedef key_t ...
void init_keys(void);
uint8_t get_key(void);

#endif //KEYBOARD_H
