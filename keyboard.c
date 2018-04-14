#include "keyboard.h"

//extern uint8_t long_press;

//================
volatile char irq8 = 0;
volatile char irq9 = 0;
volatile char irq10 = 0;
volatile char irq11 = 0;
// ??????? ?????????? ??????[???????? ???-??]
// http://www.avrfreaks.net/forum/pcint1-not-working-pcint0-works-fine
// There are three interrupt vectors:
//ISR(PCINT0_vect){} // for pins PCINT0-PCINT7   (PB0-PB7)  
//ISR(PCINT1_vect){} // for pins PCINT8-PCINT14  (PC0-PC6)
//ISR(PCINT2_vect){} // for pins PCINT16-PCINT23 (PD0-PD7)
//static volatile char enc_direction = 0;
ISR (PCINT1_vect)
{
//  define RUN_PIN	    PINC1 
//#define PLUS_PIN		PINC2
//#define MINUS_PIN		PINC3
//#define HALF_TANK_PIN	PINC4
  _delay_ms(20);

  // RUN key
  if (bit_is_clear(PINC, PINC1)) {
    irq8 = 1;
  }
   
  // PLUS key
  if (bit_is_clear(PINC, PINC2)) {
    irq9 = 1;
  } 
  
  // MINUS key
  if (bit_is_clear(PINC, PINC3)) {
    irq10 = 1;
  }

  // HALF TANK key
  if (bit_is_clear(PINC, PINC4)) {
    irq11 = 1;
	//long_press = 0;
	//start_long_press_timeout();
  }

  // проверить не длинное ли нажатие
  if (bit_is_set(PINC, PINC4)) {
    //if (long_press) {
	//  irq11 = 2;
	//}
	//stop_long_press_timeout();
  }
  
  return;
}


void init_keys(void)
{
  PCICR |= (1 << PCIE1); // Any change on any enabled PCINT14..8 pin will cause an interrupt.
  PCMSK1 |= (1 << PCINT12) | (1 << PCINT11) | (1 << PCINT10) | (1 << PCINT9);//(1 << PCINT11) | (1 << PCINT10) | (1 << PCINT9) | (1 << PCINT8); // Each PCINT14..8-bit selects whether pin change interrupt is enabled on the corresponding I/O pin. 
  return;
}


uint8_t get_key(void)
{
  uint8_t key = NO_KEYS;

  //#define KEY_BOUNCE_DEALY 15

  // настроим порт на ввод
  //KEYB_DDR &= ~(1 << RUN_PIN);
  //KEYB_DDR &= ~(1 << PLUS_PIN);
  //KEYB_DDR &= ~(1 << MINUS_PIN);
  //KEYB_DDR &= ~(1 << HALF_TANK_PIN);
	
  // проверям нажата ли какая-либо кнопка Работа
  if (irq8) {
    irq8 = 0;
    key = RUN_KEY;
  }
  // проверям нажата ли какая-либо кнопка Плус
  if (irq9) {
    irq9 = 0;
	key = PLUS_KEY;
  }
  // проверям нажата ли какая-либо кнопка Минус
  if (irq10) {
	irq10 = 0;
	key = MINUS_KEY;
  }
  // проверям нажата ли какая-либо кнопка Пол бака
  /*if (bit_is_clear(KEYB_PINS, HALF_TANK_PIN)) {
	_delay_ms(KEY_BOUNCE_DEALY);	
	if (bit_is_clear(KEYB_PINS, HALF_TANK_PIN)) {      
	   key = HALF_TANK_KEY;
	   _delay_ms(KEY_BOUNCE_DEALY);	
       // проверить не удерживатся ли кнопка для альтернативной функции
	   if (bit_is_clear(KEYB_PINS, HALF_TANK_PIN)) {
	     _delay_ms(500);
	     if (bit_is_clear(KEYB_PINS, HALF_TANK_PIN)) {      
            key = GET_LEVEL_KEY;
         }
	   }    
    }
  }*/
  if (irq11) {    
	irq11 = 0;
	key = HALF_TANK_KEY;
	if (2 == irq11) {
	  key = GET_LEVEL_KEY;
	}
  }

  return key;	  
}
