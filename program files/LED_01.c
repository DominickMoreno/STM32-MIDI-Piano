//LED_01.c wmh 2013-02-03 : fixups to allow compiling in gcc
/*----------------------------------------------------------------------------
 * Name:    LED.c
 * Purpose: low level LED functions
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2011 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

 //!!added stuff to get it to compile
  #include <stdint.h>					//various versions of this in yagarto -- gives unint32_t and other definitions
	#include <stdlib.h>
  #define __IO  

  //things we found in stm32f4xx.h and copied here to to squelch compiler complaints
  #define PERIPH_BASE           ((uint32_t)0x40000000)
  #define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
  #define RCC_BASE              (AHB1PERIPH_BASE + 0x3800)
  #define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000)
typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint16_t BSRRL;    /*!< GPIO port bit set/reset low register,  Address offset: 0x18      */
  __IO uint16_t BSRRH;    /*!< GPIO port bit set/reset high register, Address offset: 0x1A      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef; 

  //#define GPIOA               	((GPIO_TypeDef *) GPIOA_BASE)
  
typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
} RCC_TypeDef;
  
  #define RCC                 ((RCC_TypeDef *) RCC_BASE)

	//#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0A00)
  #define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)

	#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
  #define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)

	#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)
  #define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)

  #define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00)
  #define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
  
//!!things we commented out to squelch compiler complaints			
//#include "STM32F4xx.h"

#include "LED.h"


const unsigned long led_mask[] = {1UL << 12, 1UL << 13, 1UL << 14, 1UL << 15};

/*----------------------------------------------------------------------------
  initialize LED Pins
 *----------------------------------------------------------------------------*/

//Only use PA1
void init_GPIOA() {
	RCC->AHB1ENR  |= ((1UL <<  0) );       

  GPIOA->MODER    &= ~((3UL << 2*1)); 
  GPIOA->MODER    |=  ((1UL << 2*1)); 
  GPIOA->OTYPER   &= ~((1UL <<   1));
  GPIOA->OSPEEDR  &= ~((3UL << 2*1)); 
  GPIOA->OSPEEDR  |=  ((2UL << 2*1)); 
  GPIOA->PUPDR    &= ~((3UL << 2*1));  
  GPIOA->PUPDR    |=  ((1UL << 2*1));
}

//Use ports PB0, PB1, PB4, PB5, PB11
void init_GPIOB() {
	RCC->AHB1ENR  |= ((1UL <<  1) );        

  GPIOB->MODER    &= ~((3UL << 2*0) |
                       (3UL << 2*1) |
                       (3UL << 2*4) |
                       (3UL << 2*5) |
											 (3UL << 2*11)   ); 
  GPIOB->MODER    |=  ((1UL << 2*0) |
                       (1UL << 2*1) |
                       (1UL << 2*4) |
                       (1UL << 2*5) |
											 (1UL << 2*11)   ); 
  GPIOB->OTYPER   &= ~((1UL << 0) |
                       (1UL << 1) |
                       (1UL << 4) |
                       (1UL << 5) |
											 (1UL << 11)   );   
  GPIOB->OSPEEDR  &= ~((3UL << 2*0) |
                       (3UL << 2*1) |
                       (3UL << 2*4) |
                       (3UL << 2*5) |
											 (3UL << 2*11)    ); 
  GPIOB->OSPEEDR  |=  ((2UL << 2*0) |
                       (2UL << 2*1) |
                       (2UL << 2*4) |
                       (2UL << 2*5) |
											 (2UL << 2*11)    ); 
  GPIOB->PUPDR    &= ~((3UL << 2*0) |
                       (3UL << 2*1) |
                       (3UL << 2*4) |
                       (3UL << 2*5) |
											 (3UL << 2*11)   );
  GPIOB->PUPDR    |=  ((1UL << 2*0) |
                       (1UL << 2*1) |
                       (1UL << 2*4) |
                       (1UL << 2*5) |
											 (1UL << 2*11)   ); 
}

//Only use ports PC1, PC2, PC4, PC5
void init_GPIOC() {
	RCC->AHB1ENR  |= ((1UL <<  2) );      

  GPIOC->MODER    &= ~((3UL << 2*1) |
                       (3UL << 2*2) |
                       (3UL << 2*4) |
                       (3UL << 2*5) |
											 (3UL << 2*11) ); 
  GPIOC->MODER    |=  ((1UL << 2*1) |
                       (1UL << 2*2) |
                       (1UL << 2*4) |
                       (1UL << 2*5) |
											 (1UL << 2*11) ); 
  GPIOC->OTYPER   &= ~((1UL << 1) |
                       (1UL << 2) |
                       (1UL << 4) |
                       (1UL << 5) |
											 (1UL << 11) ); 
  GPIOC->OSPEEDR  &= ~((3UL << 2*1) |
                       (3UL << 2*2) |
                       (3UL << 2*4) |
                       (3UL << 2*5) |
											 (3UL << 2*11) );
  GPIOC->OSPEEDR  |=  ((2UL << 2*1) |
                       (2UL << 2*2) |
                       (2UL << 2*4) |
                       (2UL << 2*5) |
											 (2UL << 2*11) ); 
  GPIOC->PUPDR    &= ~((3UL << 2*1) |
                       (3UL << 2*2) |
                       (3UL << 2*4) |
                       (3UL << 2*5) |
											 (3UL << 2*11) );  
  GPIOC->PUPDR    |=  ((1UL << 2*1) |
                       (1UL << 2*2) |
                       (1UL << 2*4) |
                       (1UL << 2*5) |
											 (1UL << 2*11) );
}

//Only use port PD2, but leave PD12-PD15 on for kicks and giggles
void init_GPIOD() {

	RCC->AHB1ENR  |= ((1UL <<  3) );     

  GPIOD->MODER    &= ~((3UL << 2*12) |
                       (3UL << 2*13) |
                       (3UL << 2*14) |
                       (3UL << 2*15) |
											 (3UL << 2*2)   );   
  GPIOD->MODER    |=  ((1UL << 2*12) |
                       (1UL << 2*13) | 
                       (1UL << 2*14) | 
                       (1UL << 2*15) |
											 (1UL << 2*2)   ); 
  GPIOD->OTYPER   &= ~((1UL <<   12) |
                       (1UL <<   13) |
                       (1UL <<   14) |
                       (1UL <<   15) |
											 (1UL <<   2)   );
  GPIOD->OSPEEDR  &= ~((3UL << 2*12) |
                       (3UL << 2*13) |
                       (3UL << 2*14) |
                       (3UL << 2*15) |
											 (3UL << 2*2)   );  
  GPIOD->OSPEEDR  |=  ((2UL << 2*12) |
                       (2UL << 2*13) | 
                       (2UL << 2*14) | 
                       (2UL << 2*15) |
											 (2UL << 2*2)   ); 
  GPIOD->PUPDR    &= ~((3UL << 2*12) |
                       (3UL << 2*13) |
                       (3UL << 2*14) |
                       (3UL << 2*15) |
											 (3UL << 2*2) );  
  GPIOD->PUPDR    |=  ((1UL << 2*12) |
                       (1UL << 2*13) | 
                       (1UL << 2*14) | 
                       (1UL << 2*15) |
											 (1UL << 2*12) ); 
}

void init_GPIOs() {
	init_GPIOA();
	init_GPIOB();
	init_GPIOC();
	init_GPIOD();
}

/*----------------------------------------------------------------------------
  Function that turns on requested LED
 *----------------------------------------------------------------------------*/
void LED_On (unsigned int num) {
	
  if (num < LED_NUM) {
    GPIOD->BSRRL = led_mask[num];
  }
}

/*----------------------------------------------------------------------------
  Function that turns off requested LED
 *----------------------------------------------------------------------------*/
void LED_Off (unsigned int num) {

  if (num < LED_NUM) {
    GPIOD->BSRRH = led_mask[num];
  }
}

/*----------------------------------------------------------------------------
  Function that outputs value to LEDs
 *----------------------------------------------------------------------------*/
void LED_Out(unsigned int value) {
  int i;

  for (i = 0; i < LED_NUM; i++) {
    if (value & (1<<i)) {
      LED_On (i);
    } else {
      LED_Off(i);
    }
  }
}

/* Bonus functions! I originally thought hw 4 part 1 asked us to 
** write the random-LED-write function in C, so I did that
*/

//Shamelessly stolen xorshift from wikipedia. Used for pseudo-RNG
uint32_t x, y, z, w;

/* initialize to hard coded random values I got from an RNG online
** so it will always go in the same order, but that order is "random"
*/
void RNG_INIT() {
	uint32_t systick_val = get_systick();
	x = 0x4f2aa0ba * systick_val * systick_val;
	y = 0x5529e2bb  * systick_val * systick_val;
	z = 0xb79a56a6 * systick_val * systick_val;
	w = 0xc372159a * systick_val * systick_val;
}

//Implements xor-shift (not to be confused with xor-cism)
uint32_t xorshift128(void) {
    uint32_t t = x ^ (x << 11);
    x = y; y = z; z = w;
    return w = w ^ (w >> 19) ^ t ^ (t >> 8);
}


/* Turns on random LEDs that exist in provided ports at
** provided bit locations
*/
void LED_On_Port(uint32_t* LEDs, uint32_t* bits) {
	int i;
	uint32_t bit;
	uint32_t orig_word;
	uint32_t rand_num;
	uint32_t rng_mask = 0xf;
	uint32_t lsb_mask = 0x1;
	uint32_t bits_on[4];
	uint32_t on_off_bit = 0x0;
	unsigned int n = 4;
	
	rand_num = xorshift128();
	
	/* Make an array that determines which LEDs will actually
	** be turned on (based on random number
	*/
	for (i = 0; i < n; i++) {
		on_off_bit = lsb_mask & rand_num;
		bits_on[i] = on_off_bit;
		rand_num = rand_num >> 1;
	}
	
	for (i = 0; i < n; i++) {
		//Turn on each port if it pleases the gods of RNG
		
		if (bits_on[i] == 1) {
			//get the original word in that port
			orig_word = *((uint32_t* )LEDs[i]); 
			
			//Set the bit that will turn this LED on
			bit = 0x1;
			bit = bit << bits[i];
			
			*((uint32_t* )LEDs[i]) = bit | orig_word;
		}

	}
}

void LED_Ports_SVC(uint32_t num) {
	
	if (num >= 0 && num < 8) {
		ASM_SVC_Call(num);
	}
	
	return;
}

/* 		an_cat_C_clk replaces the value of systick, since I kept getting weird things
 *	from it. It is incremented every systick interrupt, and its parity is tied to
 *	the anode and cathode clocks. 
 *
 *		inc/get_an_cat_clk() are functions that, respectively, increment and retrieve
 *	the value of the an_cat_C_clk, which are meant to be called from assembly
*/
extern volatile uint32_t an_cat_C_clk = 0;

void inc_an_cat_clk() {
	an_cat_C_clk++;
}

uint32_t get_an_cat_clk() {
	return an_cat_C_clk;
}

uint32_t get_ctr_mod_8() {
	return (uint32_t)(an_cat_C_clk % 8);
}