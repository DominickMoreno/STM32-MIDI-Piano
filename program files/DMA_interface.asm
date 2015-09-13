@; DMA1Stream5int_svc.asm wmh 2015-05-16 : demo DMA interrupt service
@;DDM - Using professor's code for DMA controller. Renamed to "DMA1_Stream5_IRQHandler"
@; --- characterize target syntax, processor
	.syntax unified				@; ARM Unified Assembler Language (UAL). 
	.thumb						@; Use thumb instructions only


	.bss						@;start an uninitialized RAM data section
	.align 2					@;pad memory if necessary to align on a word boundary for word storage 
					
	.comm	PINGBUF,256			@;allocate 256 bytes/128 halfwords of static storage for uninitialized global storage			
	.comm	PONGBUF,256			@;allocate 256 bytes/128 halfwords of static storage for uninitialized global storage

	.global PINGPONG_count		@;used by DMAint_svc to determine which buffer will output next
	.comm PINGPONG_count,4
	.global PING_count			@;for debug
	.comm PING_count,4
	.global PONG_count			@;for debug
	.comm PONG_count,4

@;*** macros ***
@; desiderata : 
@;	- no side effects other than scratch registers
@;	- no local pool 'out of range' (i.e. use immediate values)

.macro MOV_imm32 reg val		@;example of use: MOV_imm32 r0,0x12345678 !!note: no '#' on immediate value
	movw \reg,#(0xFFFF & (\val))
	movt \reg,#((0xFFFF0000 & (\val))>>16)
.endm

.macro SET_bit addr bit @;logical OR position 'bit' at 'addr' with 1 
	MOV_imm32 r2,(\addr)
	ldr r1,[r2]
	ORR r1,#(1<<\bit)
	str r1,[r2]	
.endm

.macro TST_bit addr bit	@;read 'bit' at addr, return bit value in bit0 of r0 and 'Z' flag set/clear if bit=0/1
	MOV_imm32 r2,(\addr)
	ldr r0,[r2]
	ands r0,#(1<<\bit)
	lsr r0,#\bit
.endm

@; --- begin code memory
	.text	

	.include "stm32f4xx_DMA_registers01.inc"	
	
	@;.global DMA1Stream5int_svc	@;void DMA1Stream5int_svc(void);
	.global write_to_DMA @;void DMA1_Stream5_IRQHandler(uint16_t sound_flag);
	.thumb_func
write_to_DMA: @;copy DMADEMO_PINGBUF or DMADEMO_PONGBUF (above) into DMA1_PINGBUF and DMA1_PONGBUF in RAM (at top)

	MOV R3, R0			@;Save parameter (the sound flag) to determine if we will be playing sound or not

	.equ locTCIF5,	11	@;location of DMA stream 5  interrupt flag in DMA1_HISR _and_ location in DMA1_HIFCR to clear this flag
	.equ locHTIF5,	10	@; ""          				half-buffer interrupt flag  	""
	.equ locTEIF5,	 9	@; ""          				transfer error interrupt flag 	 ""
	.equ locDMEIF5,	 8	@; ""                       direct mode error interrupt  flag ""
	.equ locFEIF6,	 6	@; ""                       fifo error interrupt flag          ""
	.equ locCT,		19	@;location of 'current target' (CT) status bit in DMA1 stream 5 control register DMA1_S6CR
	
	@;earlier (non-interrupt) version polls TCIF5 to detect switch between buffers, then copies new data into currently not-selected buffer
	TST_bit	absDMA1_HISR,locTCIF5		@;did a buffer swap occur
	beq 9f								@;	no -- nothing to do, so go back
	
	@;here if DMA has switched to the other buffer
	SET_bit absDMA1_HIFCR,locTCIF5		@;reset the interrupt flag
	TST_bit absDMA1_S5CR,locCT			@;is buffer 0 now being processed?
	beq 1f								@;	yes -- so go update buffer 1

@;two buffers - he calls them "buffer 0" and "buffer 1"
4:	@;here when its buffer 0's turn to have its address updated (ok because buffer1 is currently being used by DMA)

	@;debug -- count number of times we update 'ping' buffer
	ldr r2,=PING_count					@;update count of number of switches between DMA buffers which have occurred
	ldr r1,[r2]							@; ..
	add r1,#1							@; ..
	str r1,[r2]							@; ..

	@;update PINGPONG_count and use it to determine which wave-shape to use (square or triangle)
	ldr r2,=PINGPONG_count				@;update count of number of switches between DMA buffers which have occurred
	ldr r1,[r2]							@; ..
	add r1,#1							@; ..
	str r1,[r2]							@; ..
	ands r1,#1							@;which pattern (ramp or square)
	beq 6f								@;	even values of PINGPONG count -- do ramp

5:	@;here to update buffer0 source address to that of squarewave +step
	LDR r1,=active_ping_buffer
	MOV_imm32 r2,absDMA1_S5M0AR			@;first buffer where DMA1 will get its data
	str r1,[r2]
	bx lr
	
6:	@;here to update buffer0 source address to that of triangle +ramp
	@;LDR r1,=triangle_ping_buffer
	LDR R1,=active_ping_buffer
	MOV_imm32 r2,absDMA1_S5M0AR			@;first buffer where DMA1 will get its data
	str r1,[r2]
	bx lr
	
1: 	@;here when it's buffer 1's turn to have its address updated (ok because buffer0 is currently being used by DMA)

	@;debug -- count number of times we update 'pong' buffer
	ldr r2,=PONG_count					@;update count of number of switches between DMA buffers which have occurred
	ldr r1,[r2]							@; ..
	add r1,#1							@; ..
	str r1,[r2]							@; ..

	@; use PINGPONG_count to determine which wave-shape to use (square or triangle)
	ldr r2,=PINGPONG_count				@;
	ldr r1,[r2]							@; ..
	ands r1,#1							@;which pattern (ramp or square)
	beq 3f								@;	even values of PINGPONG count -- do ramp

2:	@;here to update buffer1 source address to that of squarewave +step
	LDR R1,=active_pong_buffer
	MOV_imm32 r2,absDMA1_S5M1AR			@;first buffer where DMA1 will get its data
	str r1,[r2]
	bx lr
	
3:	@;here to update buffer1 source address to that of triangle wave -ramp
	LDR R1,=active_pong_buffer
	MOV_imm32 r2,absDMA1_S5M1AR			@;first buffer where DMA1 will get its data
	str r1,[r2]
	bx lr
	
9:	@;here when terminal count interrupt flag TCIF was not set ==> nothing to do
	bx lr
	
@;The following functions are getters called in C to get the addresses of the
@;active ping and pong buffers, as well as the waveforms of the 4 playable notes.
@;These getters are of the form:
@;			uint16_t* get_{active/C5/E5/G5/B6}_{ping/pong}_buffer(void);
	.global get_active_ping_buffer
	.thumb_func
get_active_ping_buffer:
	LDR R0,=active_ping_buffer
	BX LR
	
	.global get_active_pong_buffer
	.thumb_func
get_active_pong_buffer:
	LDR R0,=active_pong_buffer
	BX LR 
	
	.global get_C5_ping_buffer
	.thumb_func
get_C5_ping_buffer:
	LDR R0,=C5_ping_buffer
	BX LR
	
	.global get_C5_pong_buffer
	.thumb_func
get_C5_pong_buffer:
	LDR R0,=C5_pong_buffer
	BX LR
	
	.global get_E5_ping_buffer
	.thumb_func
get_E5_ping_buffer:
	LDR R0,=E5_ping_buffer
	BX LR
	
	.global get_E5_pong_buffer
	.thumb_func
get_E5_pong_buffer:
	LDR R0,=E5_pong_buffer
	BX LR
	
	.global get_G5_ping_buffer
	.thumb_func
get_G5_ping_buffer:
	LDR R0,=G5_ping_buffer
	BX LR
	
	.global get_G5_pong_buffer
	.thumb_func
get_G5_pong_buffer:
	LDR R0,=G5_pong_buffer
	BX LR
	
	.global get_B6_ping_buffer
	.thumb_func
get_B6_ping_buffer:
	LDR R0,=B6_ping_buffer
	BX LR
	
	.global get_B6_pong_buffer
	.thumb_func
get_B6_pong_buffer:
	LDR R0,=B6_pong_buffer
	BX LR
	
	.data
	.align 2 
@;address space for active ping-pong buffers, where the DMA reads for waveforms to play notes
active_ping_buffer:
	.hword 0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000
active_pong_buffer:
	.hword 0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000

@;hard coded 32-step note C5 (523.25 Hz)
C5_ping_buffer:
	.hword 0x329,0x1E8,0xD80,0xD80,0x1E8,0x329,0xE9E,0xC1F,0x0EF,0x4A5,0xF6E,0xA8D,0x04C,0x648,0xFE4,0x8DD
C5_pong_buffer:
	.hword 0x004,0x800,0xFFC,0x722,0x01C,0x9B8,0xFB4,0x572,0x093,0xB5C,0xF10,0x3E0,0x162,0xCD7,0xE18,0x280

@;hard coded 32-step note E5 (659.1 Hz)
E5_ping_buffer:
	.hword 0x800,0x4E3,0x244,0x08C,0x002,0x0BA,0x299,0x552,0x877,0xB89,0xE0D,0xF9C,0xFF7,0xF11,0xD0D,0xA3D
E5_pong_buffer:
	.hword 0x713,0x40E,0x1A8,0x043,0x017,0x12A,0x351,0x636,0x963,0xC58,0xE9D,0xFD7,0xFD5,0xE96,0xC4D,0x956

@;hard coded 32-step note G5 (783.99 Hz)
G5_ping_buffer:
	.hword 0xD48,0xE4B,0x36E,0x130,0xBCC,0xF3E,0x506,0x06C,0xA1F,0xFD1,0x6C4,0x00B,0x856,0xFFF,0x892,0x012
G5_pong_buffer:
	.hword 0x688,0xFC4,0xA59,0x080,0x4CE,0xF23,0xC01,0x151,0x33D,0xE25,0xD75,0x279,0x1EB,0xCD7,0xEA1,0x3E9

@;hard coded 32-step note B6 (932 Hz)
B6_ping_buffer:
	.hword 0x800,0x07A,0x2E7,0xC12,0xFDB,0x940,0x0FE,0x201,0xAF2,0xFFE,0xA78,0x1AE,0x140,0x9BF,0xFEF,0xBA1
B6_pong_buffer:
	.hword 0x286,0x0AA,0x881,0xFAE,0xCB3,0x381,0x042,0x740,0xF3D,0xDA7,0x498,0x00A,0x604,0xE9E,0xE77,0x5C4

