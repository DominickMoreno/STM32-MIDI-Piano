//STM32F4main01.c wmh 2013-02-02 : trying t get a barebones 'main()' to compile
/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
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

//global variables used in CortexM4asmOps_01.asm but defined here
	int Cint;

	#include "stm32f4xx_gpio.h"
	#include "stm32f4xx_tim.h"
	#include "stm32f4xx_rcc.h"
	#include "misc.h"
	#include "switch_queue.h"

 
//!!added stuff to get it to compile
  #include <stdint.h>	//various versions of this in yagarto -- gives unint32_t and other definitions
	#include <stdlib.h>
	#include <math.h>

	#define NUM_SWITCHES 13
	
  uint32_t SystemCoreClock; 	//!!found in system_stm32f4xx.c, added here as global but initialized in main()
  

void SystemCoreClockUpdate(void)
{
  uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
  
  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock source */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04:  /* HSE used as system clock source */
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08:  /* PLL used as system clock source */

      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_P
         */    
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      
      if (pllsource != 0)
      {
        /* HSE used as PLL clock source */
        pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);      
      }

      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
      SystemCoreClock = pllvco/pllp;
      break;
    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }
}

volatile uint32_t msTicks;                      /* counts 1ms timeTicks       */
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;
}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {                                              
  uint32_t curTicks;

//!!temporary -- replaced delay mediated by SysTick_Handeler() with software delay
//  curTicks = msTicks;
//  while ((msTicks - curTicks) < dlyTicks);

  
	curTicks = 0x12345;
	while(curTicks-- > 0);
  return;
}


/*----------------------------------------------------------------------------
  Function that initializes Button pins
 *----------------------------------------------------------------------------*/
void BTN_Init(void) {

  RCC->AHB1ENR  |= ((1UL <<  0) );              /* Enable GPIOA clock         */

  GPIOA->MODER    &= ~((3UL << 2*0)  );         /* PA.0 is input              */
  GPIOA->OSPEEDR  &= ~((3UL << 2*0)  );         /* PA.0 is 50MHz Fast Speed   */
  GPIOA->OSPEEDR  |=  ((2UL << 2*0)  ); 
  GPIOA->PUPDR    &= ~((3UL << 2*0)  );         /* PA.0 is no Pull up         */
}

/*----------------------------------------------------------------------------
  Function that read Button pins
 *----------------------------------------------------------------------------*/
uint32_t BTN_Get(void) {

 return (GPIOA->IDR & (1UL << 0));
}

void InitializeLEDs()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpioStructure);

    GPIO_WriteBit(GPIOD, GPIO_Pin_12 | GPIO_Pin_13, Bit_RESET);
}

/* ****************************************************************** **
**									Timer and Interrupt Initializations								**
** ****************************************************************** */

//initializes TIM2 timer
void init_TIM2_timer() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TIM_TimeBaseInitTypeDef timerInitStructure; 
	timerInitStructure.TIM_Prescaler = 20;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 40; 
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_Cmd(TIM2, ENABLE);
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

//initializes TIM3 timer
void init_TIM3_timer() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_TimeBaseInitTypeDef timerInitStructure; 
	timerInitStructure.TIM_Prescaler = 200;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 10;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &timerInitStructure);
	TIM_Cmd(TIM3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

//initializes TIM4 timer
void init_TIM4_timer() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	TIM_TimeBaseInitTypeDef timerInitStructure; 
	timerInitStructure.TIM_Prescaler = 1000;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 5000; 
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &timerInitStructure);
	TIM_Cmd(TIM4, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}

//initializes all (TIM2, TIM3, TIM4), timers
void init_timers() {
	init_TIM2_timer();
	init_TIM3_timer();
	init_TIM4_timer();
}

//initializes TIM2 interrupt (must initialize timer first)
void init_TIM2_interrupt() {
	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
}

//initializes TIM3 interrupt (must initialize timer first)
void init_TIM3_interrupt() {
	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
}

//initializes TIM4 interrupt (must initialize timer first)
void init_TIM4_interrupt() {
	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM4_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
}

//initializes all (TIM2, TIM3, TIM4) interrupts 
void init_interrupts() {
	init_TIM2_interrupt();
	init_TIM3_interrupt();
	init_TIM4_interrupt();
}

//initializes all interrupts and their timers
void init_timer_interrupts() {
	init_timers();
	init_interrupts();
}

/* ****************************************************************** **
**													Global Variables													**
** ****************************************************************** */

//vdisplay[0-3] are for corresponding digits [1-4]
//vdisplay[4] is for the punctuation mark
int16_t vdisplay[5];

//Need to remember previous and current switch status
//between TIM2 interrupts
uint16_t prev_switch_statuses;
uint16_t curr_switch_statuses;

//switch queue object
switch_queue_t sw_events;

//determines which set of buttons are read in a given interrupt
uint16_t switch_read_ctr;

//Determines which 7-seg digit is displayed in a given interrupt
uint16_t dig;

typedef enum {C5, E5, G5, B6, OFF} waveform_type_t;
waveform_type_t active_waveform = OFF;

uint16_t sound_on = 1; //0 = On, 1 = off
uint16_t piano_on = 1; //0 = on, 1 = off
uint16_t volume = 12; //volume - 12 is highest, 1 is lowest
int volume_shift = 0; //determines which way we shift the waveforms: -1 = left, 0 = none, 1 = right

/* ****************************************************************** **
**													Helper Functions													**
** ****************************************************************** */

//Since programming languages are dumb we need a helper function with
//a little bit of overhead to take the strictly nonnegative modulo
//of a number
int16_t nonnegative_modulo(int16_t x, int16_t n) {
	return ((x % n) + n) % n;
}

//Plays a given note based on the current active_waveform (a global variable) using a note
//provided in the function argument. Also requires the corresponding note's ping and pong
//buffer addresses. Manages the 7-seg and volume as well
void play_note(waveform_type_t note, uint16_t* note_ping_buffer, uint16_t* note_pong_buffer) {
	uint16_t* active_ping_buffer = get_active_ping_buffer();
	uint16_t* active_pong_buffer = get_active_pong_buffer();
	uint16_t i;
	
	//If the note given is already being played turn it off
	if(note == active_waveform) {
		
		for(i = 0; i < 16; i++) {
			active_ping_buffer[i] = 0;
			active_pong_buffer[i] = 0;
		}
		
		//turn this off so the 7-seg knows what to display
		note = OFF;
		
	} else {
		//adjust magnitude of notes in the given buffer to account for volume, and then
		//place that modified note into the active ping-pong buffers
		uint16_t amt_to_shift = 12 - volume;
		
		for(i = 0; i < 16; i++) {
			active_ping_buffer[i] = note_ping_buffer[i] >> amt_to_shift;
			active_pong_buffer[i] = note_pong_buffer[i] >> amt_to_shift;
		}
	}
	
	//Write the correct note to the 7-seg and set the correct active waveform
	switch(note) {
		case C5:
			active_waveform = C5;
			vdisplay[0] = 10;
			vdisplay[1] = 5;
			break;
		case E5:
			active_waveform = E5;
			vdisplay[0] = 11;
			vdisplay[1] = 5;
			break;
		case G5:
			active_waveform = G5;
			vdisplay[0] = 12;
			vdisplay[1] = 5;
			break;
		case B6:
			active_waveform = B6;
			vdisplay[0] = 13;
			vdisplay[1] = 6;
			break;
		case OFF:
			active_waveform = OFF;
			vdisplay[0] = vdisplay[1] = 0;
			break;
		default:
			//should never happen - catch impossible state
			while(1 == 1) {}
	}
}

/* ****************************************************************** **
**													Interrupt Handlers												**
** ****************************************************************** */

//Interrupt Handler for TIM2
//Manages switch presses. Records which switch is pressed or released, as well
//the timestamp that corresponds with it. Data is enqueued to the global switch
//queue, which is dequeued in 
void TIM2_IRQHandler() {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    
		//The global var that keeps track of time is a field in the switch events
		//	object. Increment it here. If I were a good software engineer I would
		//	make getter/setter functions but...
		sw_events.time++;
		uint16_t i;
		
		//Get current status of only a subset of the buttons. This serves two
		//purposes: the first that each interrupt isn't bogged down by reading
		//every switch during every interrupt. The second purpose is that this
		//implicitly manages switch debouncing because it doesn't constantly
		//sample the same switch 
		switch(switch_read_ctr) {
			case 0:
				//To set the nth bit in "number" to x:
				//		number ^= (-x ^ number) & (1 << n);
				//I found this on Stack Overflow
				curr_switch_statuses ^=(-read_switch(1) ^ prev_switch_statuses) & (1 << 1);
				curr_switch_statuses ^=(-read_switch(2) ^ prev_switch_statuses) & (1 << 2);
				curr_switch_statuses ^=(-read_switch(3) ^ prev_switch_statuses) & (1 << 3);
				break;
			case 1:
				curr_switch_statuses ^=(-read_switch(4) ^ prev_switch_statuses) & (1 << 4);
				curr_switch_statuses ^=(-read_switch(5) ^ prev_switch_statuses) & (1 << 5);
				curr_switch_statuses ^=(-read_switch(6) ^ prev_switch_statuses) & (1 << 6);
				break;
			case 2:
				curr_switch_statuses ^=(-read_switch(7) ^ prev_switch_statuses) & (1 << 7);
				curr_switch_statuses ^=(-read_switch(8) ^ prev_switch_statuses) & (1 << 8);
				curr_switch_statuses ^=(-read_switch(9) ^ prev_switch_statuses) & (1 << 9);
				break;
			case 3:
				curr_switch_statuses ^=(-read_switch(10) ^ prev_switch_statuses) & (1 << 10);
				curr_switch_statuses ^=(-read_switch(11) ^ prev_switch_statuses) & (1 << 11);
				curr_switch_statuses ^=(-read_switch(12) ^ prev_switch_statuses) & (1 << 12);
				curr_switch_statuses ^=(-read_switch(13) ^ prev_switch_statuses) & (1 << 13);
				break;
			default:
				//everything is on fire and broken. hang forever
				while(1) {} 
		}
		
		switch_read_ctr = (switch_read_ctr + 1) % 4;
		
		/* The following are button statuses and the manner in which
		**		they should be handled:
		**
		**		previous | current | result
		**		---------+---------+--------
		**				0		 |	  0    | hold press
		**				0		 |    1    | release
		** 				1		 |    0		 | press
		**				1    |    1    | hold release/off
		**
		*/
		uint16_t prev = prev_switch_statuses;
		uint16_t curr = curr_switch_statuses;
		
		prev = prev >> 1;
		curr = curr >> 1;
		
		//Compare each current button's status to its predecessor
		for(i = 1; i <= NUM_SWITCHES; i++) {
			uint16_t mask = 0x1;
			uint16_t prev_lsb = mask & prev;
			uint16_t curr_lsb = mask & curr;
			
			
			//IMPORTANT: even buttons (the ones on the bottom row) appear to be broken
			//on my board (likely due to my bad soddering). The switches will do things
			//like short (ie say they're pressed) multiple times even when not touched.
			//There are lots of other strange behavior. I'm confident its a strictly 
			//hardware issue because my switch/debouncing logic works perfectly on all
			//of the buttons on the TOP row but not the bottom. 
			if(prev_lsb == 0 && curr_lsb == 0) {
				//hold press - do nothing
			} else if(prev_lsb == 0 && curr_lsb == 1) {
				//release - change digit
				
				if(i % 2 != 0) { //make sure it's not a button on the bottom row (b/c they're broken)
					enqueue_sw_event(&sw_events, 0, i, sw_events.time);
				}				
				
			} else if(prev_lsb == 1 && curr_lsb == 0) {
				//press - record the event but do nothing
				if(i % 2 != 0) { //make sure it's not a button on the bottom row (b/c they're broken)
					enqueue_sw_event(&sw_events, 1, i, sw_events.time);
				}
			} else if(prev_lsb == 1 && curr_lsb == 1) {
				//hold release/off - do nothing
				
			} else {
				//catch impossible state
				while(1 == 1) {}
			}
			
			prev = prev >> 1;
			curr = curr >> 1;
		}
		
		prev_switch_statuses = curr_switch_statuses;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
	}
}

//TIM3 interrupt handler
//manages switch events for buttons
//Button map:
//	1: Play/turn off C5
//	3: Play/turn off E5
//	5: Play/turn off G5
//	7: Play/turn off B6
//	9: Volume down
//	11: Volume up
//	13: On/off
void TIM3_IRQHandler() {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		uint16_t *active_ping_buffer, *active_pong_buffer;
		uint16_t i;
		swevent_t ev = getswitch(&sw_events);
		
		if(is_not_null_event(ev) == SUCCESS && ev.pr == 0) {
			//If there is an event to process
			uint16_t v;
			switch(ev.sw) {
				case 1:
					//C5 button
					if(piano_on == 0) {
						play_note(C5, get_C5_ping_buffer(), get_C5_pong_buffer());
					} 
					
					break;
				case 3:
					//E5 button
					if(piano_on == 0) {
						play_note(E5, get_E5_ping_buffer(), get_E5_pong_buffer());
					}
				
					break;
				case 5:
					//G5 button
					if(piano_on == 0) {
						play_note(G5, get_G5_ping_buffer(), get_G5_pong_buffer());
					}
		
					break;
				case 7:
					//B6 button
					if(piano_on == 0) {
						play_note(B6, get_B6_ping_buffer(), get_B6_pong_buffer());
					}
				
					break;
				case 9:
					//Volume down button
					if(piano_on != 0) {
						break;
					}
					
					if(volume != 1) {
						volume_shift = 1;
						volume--;
					}
					
					if(volume > 9) {
						vdisplay[2] = 1;
					} else {
						vdisplay[2] = 0;
					}
					
					vdisplay[3] = volume % 10;
					
					break;
				case 11:
					//Volume up button
					if(piano_on != 0) {
						break;
					}
				
					if(volume != 12) {
						//has room to go up
						volume_shift = -1;
						volume++;
					}
					
					if(volume > 9) {
						vdisplay[2] = 1;
					} else {
						vdisplay[2] = 0;
					}
					
					vdisplay[3] = volume % 10;
					
					break;
				case 13:
					//On/Off button
					if(piano_on == 1) {
						//it's off, turn it back on
						piano_on = 0;
					} else if(piano_on == 0) {
						//it's on, turn it off
						piano_on = 1;
						
						active_ping_buffer = get_active_ping_buffer();
						active_pong_buffer = get_active_pong_buffer();
						
						for(i = 0; i < 16; i++) {
							active_ping_buffer[i] = 0;
							active_pong_buffer[i] = 0;
						}
						
						active_waveform = OFF;
						
						volume = 12;
						vdisplay[0] = vdisplay[1] = 0;
						vdisplay[2] = 1;
						vdisplay[3] = 2;
					}
				
					break;
				default:
					//broken - hang here forever
					while(1) {}
			}
		}
		
		if(piano_on == 0) {
				write_sev_seg(dig, vdisplay[dig-1]);
			dig = (dig % 4) + 1;
		}
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

//Interrupt Handler for TIM4
//	manages volume
void TIM4_IRQHandler() {
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
		uint16_t* active_ping_buffer = get_active_ping_buffer();
		uint16_t* active_pong_buffer = get_active_pong_buffer();
		
		uint16_t i;
		
		if(volume_shift == -1) {
			//shift left to increase volume
			
			for(i = 0; i < 16; i++) {
				active_ping_buffer[i] = active_ping_buffer[i] << 1;
				active_pong_buffer[i] = active_pong_buffer[i] << 1;
			}
			
		} else if(volume_shift == 1) {
			//shift right to decrease volume
			
			for(i = 0; i < 16; i++) {
				active_ping_buffer[i] = active_ping_buffer[i] >> 1;
				active_pong_buffer[i] = active_pong_buffer[i] >> 1;
			}
			
		} else if(volume_shift == 0) {
			//do-nothing
		} else {
			//impossible state - catch here
			while(1 == 1) {}
		}
		
		volume_shift = 0;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

//Wrapper to the assembly code that manages active ping-pong buffers
void DMA1_Stream5_IRQHandler() {
	write_to_DMA();
}

/* ****************************************************************** **
**											Initialization and Main												**
** ****************************************************************** */

void initialization() {
	//initialize all digits in 7-seg to 0
	vdisplay[0] = vdisplay[1] = 0;
	vdisplay[2] = 1;
	vdisplay[3] = 2;
	
	//Select 7-seg digit #1 to start (cycles on [1,4])
	dig = 1;
	
	//Since a "1" means 'not pressed', we want to start all switches at a status
	//of "not pressed" - this means putting 1 in bits [1,13], which is 0x3FFE
	curr_switch_statuses = 0x3FFE;
	prev_switch_statuses = 0x3FFE;
	
	//Used by TIM2_IRQHandler to determine which of 3 switches to read
	switch_read_ctr = 0;
	
	//constructor for a switch_queue
	init_switch_container(&sw_events);
	
	//initialize hardware
	init_asm_vals();
	init_GPIOs();
	init_timer_interrupts();
	
}

int main() {
	swevent_t ev;
	
	//init GPIO, timers, interrupts, 7-seg and switch container
	initialization();
	
	asm("bl.W (0x08010000+1)");		//go up to DAC1_TIM7_DMA1_init();	
	for(;;) {} //this is never actually reached because the DAC-DMA takes over control flow
}
