@;CortexM4asmOps_01.asm wmh 2013-02-25 : ARM instruction examples taken from NXP LPC17xx manual !!identify

@; --- characterize target syntax, processor
	.syntax unified				@; ARM Unified Assembler Language (UAL). 
								@; Code written using UAL can be assembled 
								@; for ARM, Thumb-2, or pre-Thumb-2 Thumb
	.thumb						@; Use thmb instructions only

.include "macro_defs.asm"


@; --- begin RAM allocation for variables								

	.data						@; start the _initialized_ RAM data section
@; globals
	
	@;Globally available location in memory that the PEND_SV handler will use to determine
	@;		which LED to turn on/off
	.align 4
	.global LED_NUM_ON_OFF
LED_NUM_ON_OFF: .word 0x00
	
	.align 4
	.global Dint
Dint: 	.word  0xFFFFFFFF   	@; these won't actually be initialized unless we do it ourselves 
Dshort:	.hword 0xABCD       	@; this won't be global unless we make it so
Dchar: 	.byte  0x55   		    @;       
  
	.bss						@;start the uninitialized RAM data section				 
@; global uninitialized variables 
	.align	4					@;pad memory if necessary to align on word boundary for word storage 
local_bss_begin:				@;marker for start of locally defined (this sourcefile) .bss variables
	.comm	Garray,256			@;allocate 256 bytes of static storage for uninitialized global storage
	.comm	Gint,4				@;allocate word of static storage (4 bytes) for an uninitialized global int variable
	.comm	Gshort,2			@;allocate half-word of static storage (2 bytes) for an uninitialized global short int variable 
	.comm	Gchar,1				@;allocate byte of static storage (1 bytes) for an uninitialized global unsigned char variable 
@; local uninitialized variables
	.align	4					@;pad memory if necessary to align on word boundary for word storage 
	.lcomm	Larray,256			@;allocate 256 bytes of storage for an uninitialized local array
	.lcomm	Lint,4				@;allocate word of static storage (4 bytes) for an uninitialized local int variable
	.lcomm	Lshort,2			@;allocate half-word of static storage (2 bytes) for an uninitialized local short int variable 
	.lcomm	Lchar,1				@;allocate byte of static storage (1 bytes) for an uninitialized local unsigned char variable 
	.align 4					@;so end-marker is on a word boundary
local_bss_end:					@;marker for end of locally defined (this sourcefile) .bss variables

@; --- constant definitions (symbol macros -- these do not use storage)
	.equ const,0x12345678
	.equ struc,0x12345670	

@; --- begin code memory
	.text						@;start the code section

	.global CortexM4asmOps_init @; make this function visible everywhere
	.thumb_func					@; make sure it starts in thumb mode
CortexM4asmOps_init: @; initialize variables defined in this sourcefile
	@; initialize globals in .data
	ldr r0,=0xFFFFFFFF			@; initialize 'Dint'
	ldr r1,=Dint
	str r0,[r1]
	@;begin mine
	LDR r0,=Dint
	LDR r1, [r0]
	@; end mine
	movw r0,#0xABCD				@;  initialize 'Dshort'
	ldr r1,=Dshort
	strh r0,[r1]
	mov r0,#0x55				@;  initialize 'Dchar'
	ldr r1,=Dchar
	strb r0,[r1]
	@; initialize .bss
	ldr r1,=local_bss_begin		
	ldr r3,=local_bss_end
	subs r3, r3, r1			@; length of uninitialized local .bss data section
	beq 2f					@; Skip if none
	mov r2, #0				@; value to initialize .bss with
1: 	@;!!local label which I can 'b 1b' branch backward to. Oooo, delicious. 
	strb r2, [r1],#1		@; Store zero
	subs r3, r3, #1			@; Decrement counter
	bgt 1b					@; Repeat until done
2:  @;!!local label which I can 'b 1f' branch forward to. 
	BX LR

	.global asmDelay 			@; make this function visible everywhere
	.thumb_func					@; make sure it starts in thumb mode
asmDelay:						@; short software delay
	MOVW    R3, #0xFFFF			@; r3=0x0000FFFF
	MOVT    R3, #0x0000			@; ..
delay_loop:						@; repeat here
	CBZ     R3, delay_exit		@; r3 == 0?
	SUB     R3, R3, #1			@; 	no --
	B       delay_loop			@;	  continue 
delay_exit:						@;  yes --
	BX      LR					@;    return to caller
 
	
	.global asmLED_On
	.thumb_func
asmLED_On:
	LDR r1,=0x40020C18		@;r4 = word in memory that controls the LEDs
	LDR r2,=0x1				@;get a bit
	LDR r3,=12				@;that bit will need to be shifted at least 12 times, since
							@;	bit 12 is where the first of the LED setter bits are
	ADD r3,r0				@;r6 += r0. This is to make sure the bit reaches whatever
							@;	LED setter bit it needs to - eg 12, 13, 14, 15
	LSL r2,r2,r3			@;r5 left shifted {r6} number of times
	STR r2,[r1] 			@;(R4) <- R5. Write that bit to the BSR register
	BX LR					@;return

	.global asmLED_Off
	.thumb_func
asmLED_Off:
	LDR r1,=0x40020C18		@; r4 = word in memory that controls the LEDs
	LDR r2,=0x1				@; get a bit
	LDR r3,=28				@;that bit will need to be shifted at least 12 times, since
							@;	bit 28 is where the first of the LED resetter bits are
	ADD r3,r0				@;r6 += r0. This is to make sure the bit reaches whatever
							@;	LED setter bit it needs to - eg 28, 29, 30, 31
	LSL r2,r2,r3			@; r5 left shifted {r6} number of times
	STR r2,[r1] 			@;(R4) <- R5. Write that bit to the BSR register
	BX LR					@;return
	
	
	.global ASM_SVC_Call
	.thumb_func
ASM_SVC_Call:

	@; I made my own switch case with CMP instructions, because
	@;		TBB is dumb and didn't want to play nicely
	CMP R0, #0
	BEQ SVC_0
	
	CMP R0, #1
	BEQ SVC_1
	
	CMP R0, #2
	BEQ SVC_2
	
	CMP R0, #3
	BEQ SVC_3
	
	CMP R0, #4
	BEQ SVC_4
	
	CMP R0, #5
	BEQ SVC_5
	
	CMP R0, #6
	BEQ SVC_6
	
	CMP R0, #7
	BEQ SVC_7

SVC_0:
	SVC #0
	BX LR;
	
SVC_1:
	SVC #1
	BX LR;

SVC_2:
	SVC #2
	BX LR;
	
SVC_3:
	SVC #3
	BX LR;
	
SVC_4:
	SVC #4
	BX LR;
	
SVC_5:
	SVC #5
	BX LR;
	
SVC_6:
	SVC #6
	BX LR;
	
SVC_7:
	SVC #7
	BX LR;
	
	.global SVC_Handler
	.thumb_func
SVC_Handler:

	@;Shamelessly stolen from SysInt_demo01.asm on how to set the PENDSV flag
	ldr r1,=ICSR			@;Interrupt Control and State Register
	mov r0,(1<<PENDSVSET)	@;PendSV set bit 	
	str r0,[r1]
	
	@; get the PC
	LDR R0, [SP, #24] @; Get the PC before interrupt
	
	@;We actually want the preceding PC, since the current one is the instruction after the SVC all
	SUB R0, R0, #2 
	LDR R1, [R0] @; Load the instruction SVC #xxx
	
	@; Extract the lower byte of the opcode (which contains the SVC constant)
	MOV R2, 0xff
	AND R0, R1, R2 
	
	@; Set the global that determines which LED to turn on/off
	LDR r1,=LED_NUM_ON_OFF
	STR r0, [r1]
	
	BX LR
	
	.equ ICSR,0xE000ED04		@;Interrupt Control and State Register
	.equ PENDSVSET,28			@;bit location in ICSR to set PendSV interrupt pending
	
	.global PendSV_Handler
	.thumb_func
PendSV_Handler:
	
	@;Load the global that determines which LED to turn on/off
	LDR R1,=LED_NUM_ON_OFF
	LDR R0, [R1]
	
	@;If the const is even, turn that LED on. If odd, turn it off
	MOV R3, #0x1 @; LSB mask
	push {LR}
	
	@;ANDS R3, R0, R1	@; original
	ANDS R3, R3, R0
	BEQ turn_leds_on
	
turn_leds_off:
	@; turn them off
	SUB R0, R0, #1
	LSR R0, R0, #1
	BL asmLED_Off
	B svc_done
	
turn_leds_on:
	@;turn them on
	LSR R0, R0, #1
	BL asmLED_On
	
svc_done:
	
	pop {LR}
	BX LR
	
	@;from TAbboud
	.global init_asm_vals		@;Initialize Cathode, Anode, and Switch pins
	.thumb_func
init_asm_vals:
	.set GPIOA_BASE, 0x40020000
	.set GPIOB_BASE, 0x40020400
	.set GPIOC_BASE, 0x40020800
	.set GPIOD_BASE, 0x40020C00
	.set RCC_AHB1ENR, 0x40023830
	.set STD_OUTPIN, 0
	.set PULLUP_INPIN, 2
	
	SET_bit RCC_AHB1ENR,0					@;enable clock for GPIOA
	SET_bit RCC_AHB1ENR,1					@;		""		   GPIOB
	SET_bit RCC_AHB1ENR,2					@;      ""         GPIOC
	SET_bit RCC_AHB1ENR,3					@;      ""         GPIOD
	
	PORTBIT_init STD_OUTPIN,GPIOC_BASE,11	@;	PC11 AN_CLK			STD_OUTPIN == 01, load 01 into PC4 location i.e. make it an output pin
	PORTBIT_init STD_OUTPIN,GPIOB_BASE,4	@;	PB4	 AN_EN	
	PORTBIT_init STD_OUTPIN,GPIOD_BASE,2	@;	PD2	 CA_CLK
	PORTBIT_init STD_OUTPIN,GPIOC_BASE,1	@;	PC1	 CA_EN	
	PORTBIT_init STD_OUTPIN,GPIOC_BASE,5	@;	PC5	 CA_A/LED5-COLON/SW_11-12
	PORTBIT_init STD_OUTPIN,GPIOB_BASE,1	@;	PB1	 CA_B/LED6-DIGIT4/SW_7-8
	PORTBIT_init STD_OUTPIN,GPIOA_BASE,1	@;	PA1	 CA_C/LED1-DIGIT2/SW_13
	PORTBIT_init STD_OUTPIN,GPIOB_BASE,5	@;	PB5	 CA_D-/SW_1-2
	PORTBIT_init STD_OUTPIN,GPIOB_BASE,11	@;	PB11 CA_E-AN_R/SW_3-4
	PORTBIT_init STD_OUTPIN,GPIOC_BASE,2	@;	PC2	 CA_F/LED4-DIGIT1
	PORTBIT_init STD_OUTPIN,GPIOC_BASE,4	@;	PC4	 CA_G/LED2-DIGIT3/SW_9-10
	PORTBIT_init STD_OUTPIN,GPIOB_BASE,0	@;	PB0	 CA_DP/LED3-AN_G/SW_5-6
bx LR
	
	.ltorg
	
@; "int read_switch(int switch_num)" reads the status (pressed or not pressed)
@;		of the provided switch number. Returns as follows:
@;
@;		0: pressed
@;		1: not pressed
@;
@; This uses Tony Abboud's macro to read the switch.
	.global read_switch
	.thumb_func
@;Place the switch to be read in R0, must be in [1,13]
read_switch:

	PUSH {LR}

	MOV R1, #1
	SUB R0, R0, R1

sw_table_1:
	TBH [PC, R0]
tbb_table_rs_1:
	.hword ((sw_1 - tbb_table_rs_1)/2)
	.hword ((sw_2 - tbb_table_rs_1)/2)
	.hword ((sw_3 - tbb_table_rs_1)/2)
	.hword ((sw_4 - tbb_table_rs_1)/2)
	.hword ((sw_5 - tbb_table_rs_1)/2)
	.hword ((sw_6 - tbb_table_rs_1)/2)
	.hword ((sw_7 - tbb_table_rs_1)/2)
	.hword ((sw_8 - tbb_table_rs_1)/2)
	.hword ((sw_9 - tbb_table_rs_1)/2)
	.hword ((sw_10 - tbb_table_rs_1)/2)
	.hword ((sw_11 - tbb_table_rs_1)/2)
	.hword ((sw_12 - tbb_table_rs_1)/2)
	.hword ((sw_13 - tbb_table_rs_1)/2)
	
sw_1:
	SWITCH_read GPIOB_BASE,5, GPIOA_BASE,15
	B rs_done
sw_2:
	SWITCH_read GPIOB_BASE,5, GPIOC_BASE,8
	B rs_done
sw_3:
	SWITCH_read GPIOB_BASE,11, GPIOA_BASE,15
	B rs_done
sw_4:
	SWITCH_read GPIOB_BASE,11, GPIOC_BASE,8
	B rs_done
	
	.ltorg
	
sw_5:
	SWITCH_read GPIOB_BASE,0, GPIOA_BASE,15
	B rs_done
sw_6:
	SWITCH_read GPIOB_BASE,0, GPIOC_BASE,8
	B rs_done
sw_7:
	SWITCH_read GPIOB_BASE,1, GPIOA_BASE,15 
	B rs_done
sw_8:
	SWITCH_read GPIOB_BASE,1, GPIOC_BASE,8  
	B rs_done
	
	.ltorg
	
sw_9:
	SWITCH_read GPIOC_BASE,4, GPIOA_BASE,15
	B rs_done
sw_10:
	SWITCH_read GPIOC_BASE,4,  GPIOC_BASE,8
	B rs_done
sw_11:
	SWITCH_read GPIOC_BASE,5, GPIOA_BASE,15
	B rs_done
sw_12:
	SWITCH_read GPIOC_BASE,5, GPIOC_BASE,8
	B rs_done
	
	.ltorg
	
sw_13:
	SWITCH_read GPIOA_BASE,1, GPIOA_BASE,15

rs_done:
	POP {LR}
	BX LR
	
@; "void dig_select(int digit)" chooses one of the digits 1-4 to prepare for
@;		a value to be displayed on. For example if you wish to display
@;		"7" on digit 3, you would first have to call "dig_select(3)" in C
@;		or provide "3" in R0 when calling dig_select in ASM.
	.global dig_select
	.thumb_func
@; provide the digit, [1,4], in R0
dig_select:
	SUB R0,R0,1			@;digits are in [1,4], but branch is 0-indexed, so sub 1
	TBB [PC, R0]
tbb_table_ds:
	.byte ((d_1 - tbb_table_ds)/2)
	.byte ((d_2 - tbb_table_ds)/2)
	.byte ((d_3 - tbb_table_ds)/2)
	.byte ((d_4 - tbb_table_ds)/2)
	
	.thumb_func
d_1:
	ANODE_write 1,1,1,1,1,1,1,0
	B done_tbb_ds
d_2:
	ANODE_write 1,1,1,1,1,1,0,1
	B done_tbb_ds
d_3:
	ANODE_write 1,1,1,1,0,1,1,1
	B done_tbb_ds
d_4:
	ANODE_write 1,1,1,0,1,1,1,1
done_tbb_ds:
	BX LR
	
@; "void val_select(int value)"chooses one of the ten possible values,
@;		0-9, to be displayed on a previously chosen digit on the 7-seg
@;		display. For example, to display the value "7" on digit 3, once
@;		you've selected the digit you would call "val_select(7)" from C
@;		or provide "7" in R0 when calling from ASM.
	.global val_select
	.thumb_func
@;provide the value, [0,9], in R0. May expand
val_select:

	@;Because tbb can apparently only handle up to 6 branches, but we have more
	@;		values we need to choose than that, I use multiple tables. The first
	@;		table contains the patterns for values [0,5], the second has those
	@;		for [6,9]. This may need to be expanded in the future.
	MOV R2, #5						@;R1 = {val} - 5
	SUBS R1, R0, R2					@;	...
	BLE table_1						@;if the value was less than 5 we go to the first table
	@;B table_2		@;orig
	MOV R2, #4
	SUBS R0, R1, R2					@;R0 = {val} - 9
	BLE table_2						@;if the value was less than 10 we go to the second table
	B table_3						@;if the value was greater than 10 we go to the third table

.ltorg

table_1:
	TBB [PC, R0]
tbb_table_vs_1: @;writes digits on [0,5]
	.byte ((v_0 - tbb_table_vs_1)/2)
	.byte ((v_1 - tbb_table_vs_1)/2)
	.byte ((v_2 - tbb_table_vs_1)/2)
	.byte ((v_3 - tbb_table_vs_1)/2)
	.byte ((v_4 - tbb_table_vs_1)/2)
	.byte ((v_5 - tbb_table_vs_1)/2)
	
	.thumb_func
v_0:
	CATHODE_write 0,0,0,0,0,0,1,1
	B done_tbb_vs
v_1:
	CATHODE_write 1,0,0,1,1,1,1,1
	B done_tbb_vs
v_2:
	CATHODE_write 0,0,1,0,0,1,0,1
	B done_tbb_vs
v_3:
	CATHODE_write 0,0,0,0,1,1,0,1
	B done_tbb_vs
v_4:
	CATHODE_write 1,0,0,1,1,0,0,1
	B done_tbb_vs
v_5:
	CATHODE_write 0,1,0,0,1,0,0,1
	B done_tbb_vs

.ltorg

table_2: @;writes digits on [6,9]
	@; We've already subtracted 5 from the value, and since the values in this table
	@; are strictly in [6,9], we only need to subtract by 1 to get their 0-index
	MOV R0, #1
	SUB R0, R1, R0
	TBB [PC, R0]
	tbb_table_vs_2:
	.byte ((v_6 - tbb_table_vs_2)/2)
	.byte ((v_7 - tbb_table_vs_2)/2)
	.byte ((v_8 - tbb_table_vs_2)/2)
	.byte ((v_9 - tbb_table_vs_2)/2)
	
	.align 4
	.thumb_func
v_6:
	CATHODE_write 0,1,0,0,0,0,0,1
	B done_tbb_vs
v_7:
	CATHODE_write 0,0,0,1,1,1,1,1
	B done_tbb_vs
v_8:
	CATHODE_write 0,0,0,0,0,0,0,1
	B done_tbb_vs
v_9:
	CATHODE_write 0,0,0,1,1,0,0,1
	B done_tbb_vs
	
.ltorg
	
table_3: @;writes letters 'c', 'E', 'g', 'b'
	MOV R1, #1
	SUB R0, R0, R1
	TBB [PC, R0]
	tbb_table_vs_3:
	.byte ((v_c - tbb_table_vs_3)/2)
	.byte ((v_E - tbb_table_vs_3)/2)
	.byte ((v_g - tbb_table_vs_3)/2)
	.byte ((v_b - tbb_table_vs_3)/2)
	
	.align 4
	.thumb_func
v_c:
	CATHODE_write 1,1,1,0,0,1,0,1
	B done_tbb_vs
v_E:
	CATHODE_write 0,1,1,0,0,0,0,1
	B done_tbb_vs
v_g:
	CATHODE_write 0,0,0,0,1,0,0,1
	B done_tbb_vs
v_b:
	CATHODE_write 1,1,0,0,0,0,0,1
	B done_tbb_vs

done_tbb_vs:	
	BX LR
	
@; void write_sev_seg(int digit, int value)" writes the given value on the
@;		given digit in the 7-seg. For example, to write the value "7" on
@;		digit 3, you would call "write_sev_seg(3,7)" in C, or provide "3"
@;		and "7" in registers R0 and R1, respectively, when calling in ASM
	.global write_sev_seg
	.thumb_func
@;Provide the digit in [1,4] in R0, the value in [0,9] in R1
write_sev_seg:
	PUSH {LR}
	
	PUSH {R1}			@; hold on to R1, since it holds the value we'll be writing
	CLEAR_VAL			@; clear the previous value. Prevents ghosting
	BL dig_select		@; select the digit on the 7-seg
	POP {R0}			@; val_select expects the value in R0, so we pop the provided
	BL val_select		@;		value into R0
	
	POP {LR}
	BX LR

@; --- end of code/beginning of ROM data 
@;	.rodata						@; start of read-only data section
@; code memory area containing test data for testing 'load' instructions
@; We are putting this in ROM as so it doesn't have to be initialized before using it. 
@; In real applications it can be anywere in the address space. 
	.global	ROMdata				@; global label of test target data area
	.align						@; pad memory if necessary to align on word boundary for word storage 
ROMdata:						@; start of test data area	
	.byte 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F	@;16 bytes with contents = offset from start
	.byte 0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F	@;""
	.byte 0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F	@;""
	.byte 0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F	@;""
	.byte 0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F	@;""
	.byte 0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F	@;""
	.byte 0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F	@;""
	.byte 0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E,0x7F	@;""
	.byte 0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x8F	@;""
	.byte 0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0x9B,0x9C,0x9D,0x9E,0x9F	@;""
	.byte 0xA0,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xAB,0xAC,0xAD,0xAE,0xAF	@;""
	.byte 0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF	@;""
	.byte 0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xCB,0xCC,0xCD,0xCE,0xCF	@;""
	.byte 0xD0,0xD1,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE,0xDF	@;""
	.byte 0xE0,0xE1,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xEB,0xEC,0xED,0xEE,0xEF	@;""
	.byte 0xF0,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF	@;""
	 