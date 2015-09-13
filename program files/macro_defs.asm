@;macros I've written

.include "TAbboud_stm32f4_P24v04_definitions.asm"
	
.macro CLEAR_VAL
	CATHODE_write 1,1,1,1,1,1,1,1
.endm
