@;STM32F4_P24v04_definitions03.inc wmh 2014-04-04 : FSMC registers added
@;STM32F4_P24v04_definitions02.inc wmh 2013-03-18 : register and pin definitions extended to P24v04, plus IO macros
@;STM32F4_P24v04_definitions01.inc wmh 2013-03-18 : register and pin definitions for STM32F4DISCOVERY; IO macros
@;stm32f4xx_01.inc wmh 2013-03-17 : derives from C header file C:\Keil_MDK460\ARM\INC\ST\STM32F4xx\stm32f4xx.h 
@;	The above file contains a subset of the original C header file and should be expanded from that source when new peripherals are used 
@;	in the following, selected '#define's are cribbed from stm32f4xx.h and converted to '.equ's 
												@in 'stm32f4xx.h' was ..
.equ PERIPH_BASE,(0x40000000)					@;#define PERIPH_BASE           (0x40000000)
.equ APB1PERIPH_BASE,PERIPH_BASE				@;#define APB1PERIPH_BASE       PERIPH_BASE
.equ AHB1PERIPH_BASE,(PERIPH_BASE + 0x00020000)	@;#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)

@; AHB1 peripherals
.equ GPIOA_BASE,(AHB1PERIPH_BASE + 0x0000)		@;#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000)
.equ GPIOB_BASE,(AHB1PERIPH_BASE + 0x0400)		@;#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
.equ GPIOC_BASE,(AHB1PERIPH_BASE + 0x0800)		@;#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)
.equ GPIOD_BASE,(AHB1PERIPH_BASE + 0x0C00)		@;#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00)
.equ GPIOE_BASE,(AHB1PERIPH_BASE + 0x1000)		@;#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000)
.equ GPIOF_BASE,(AHB1PERIPH_BASE + 0x1400)		@;#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400)
.equ GPIOG_BASE,(AHB1PERIPH_BASE + 0x1800)		@;#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800)
.equ GPIOH_BASE,(AHB1PERIPH_BASE + 0x1C00)		@;#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00)
.equ GPIOI_BASE,(AHB1PERIPH_BASE + 0x2000)		@;#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000)


.equ RCC_BASE,(AHB1PERIPH_BASE + 0x3800)		@;#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800)

.equ RCC_AHB1ENR,(RCC_BASE+0x30)
.equ RCC_AHB2ENR,(RCC_BASE+0x34)
.equ RCC_AHB3ENR,(RCC_BASE+0x38)

@;from ST_RM0090-reference_manual_for_STM32F40x,STM32F41x,STM32F42x STM32F43x_advanced_ARM-based_32-bit_MCUs_DM00031020.pdf section 7.1
@;	"Each general-purpose I/O port has four 32-bit configuration registers (GPIOx_MODER,
@;	GPIOx_OTYPER, GPIOx_OSPEEDR and GPIOx_PUPDR), two 32-bit data registers
@;	(GPIOx_IDR and GPIOx_ODR), a 32-bit set/reset register (GPIOx_BSRR), a 32-bit locking
@;	register (GPIOx_LCKR) and two 32-bit alternate function selection register (GPIOx_AFRH
@;	and GPIOx_AFRL)."
.equ MODER,0x00				@;__IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
.equ OTYPER,0x04			@;__IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
.equ OSPEEDR,0x08			@;__IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
.equ PUPDR,0x0C				@;__IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
.equ IDR,0x10				@;__IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
.equ ODR,0x14				@;__IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
.equ BSRR,0x18				@;!!added to original.  This is a _word_, where the low 16-bits sets bits and the hight 16-bits clear bits
.equ BSRRL,0x18				@;__IO uint16_t BSRRL;    /*!< GPIO port bit set/reset low register,  Address offset: 0x18      */
.equ BSRRH,0x1A				@;__IO uint16_t BSRRH;    /*!< GPIO port bit set/reset high register, Address offset: 0x1A      */
.equ LCKR,0x1C				@;__IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
.equ AFR1,0x20				@;__IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
.equ AFRL,AFR1				@;databook nomenclature
.equ AFR2,0x24				@;__IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
.equ AFRH,AFR2				@;databook nomenclature

.equ STD_OUTPIN,0	@;port pin initialization code -- see macro 'PORTBIT_init'
.equ STD_INPIN,1	@; ""
.equ PULLUP_INPIN,2	@; ""

.equ FSMC_R_BASE,           0xA0000000 				/*!< FSMC registers base address       */
.equ FSMC_Bank1_R_BASE,     (FSMC_R_BASE + 0x0000)	/*!< FSMC Bankx registers base address */
.equ FSMC_Bank1E_R_BASE,    (FSMC_R_BASE + 0x0104)	/*                -""-                 */  
.equ FSMC_Bank2_R_BASE,     (FSMC_R_BASE + 0x0060)  /*                -""-                 */
.equ FSMC_Bank3_R_BASE,     (FSMC_R_BASE + 0x0080)  /*                -""-                 */
.equ FSMC_Bank4_R_BASE,     (FSMC_R_BASE + 0x00A0)  /*                -""-                 */

@;FSMC registers (Table 220 in manual)
.equ FSMC_BCR1,0xA0000000	@; FSMC chip-select control registers
.equ FSMC_BCR2,0xA0000008	@; ""
.equ FSMC_BCR3,0xA0000010	@; ""
.equ FSMC_BCR4,0xA0000018	@; ""

.equ FSMC_BTR1,0xA0000004	@; FSMC chip-select timing registers
.equ FSMC_BTR2,0xA000000C	@; ""
.equ FSMC_BTR3,0xA0000014	@; ""
.equ FSMC_BTR4,0xA000001C	@; ""

.equ FSMC_BWTR1,0xA0000104	@; FSMC write timing registers
.equ FSMC_BWTR2,0xA000010C	@; ""
.equ FSMC_BWTR3,0xA0000114	@; ""
.equ FSMC_BWTR4,0xA000011C	@; ""

@;.equ FSMC_Bank1,          ((FSMC_Bank1_TypeDef *) FSMC_Bank1_R_BASE)
@;.equ FSMC_Bank1E,        ((FSMC_Bank1E_TypeDef *) FSMC_Bank1E_R_BASE)
@;.equ FSMC_Bank2,          ((FSMC_Bank2_TypeDef *) FSMC_Bank2_R_BASE)
@;.equ FSMC_Bank3,          ((FSMC_Bank3_TypeDef *) FSMC_Bank3_R_BASE)
@;.equ FSMC_Bank4,          ((FSMC_Bank4_TypeDef *) FSMC_Bank4_R_BASE)

@; misc. definitions to enable BusFault and MemManageFault exceptions for diagnostics
.equ SHCRS,0xE000ED24	@;System handler control and state register in "+ARM_Cortex-M4_user_guide_DUI0553A_cortex_m4_dgug.pdf", Table 4-12 Summary of the system control block registers
.equ BUSFAULTENA, 17 	@; BusFault enable bit: set to 1 to enable (in "" 4.3.9 System Handler Control and State Register)
.equ MEMFAULTENA, 16	@; MemManage enable bit, set to 1 to enable(in "" 4.3.9 System Handler Control and State Register)

@;*** macros ***
@; desiderata : 
@;	- no side effects other than scratch registers
@;	- no local pool 'out of range' (i.e. use immediate values)

.macro MOV_imm32 reg val		@;example of use: MOV_imm32 r0,0x12345678 !!note: no '#' on immediate value
	movw \reg,#(0xFFFF & (\val))
	movt \reg,#((0xFFFF0000 & (\val))>>16)
.endm

.macro WR_imm32 adr val			@;example of use: WR_imm32 0xE0000120,0x12345678 !!note: no '#' on immediate value
	MOV_imm32 r3, \adr
	MOV_imm32 r0, \val
	str r0,[r3]
.endm

.macro ORR_imm32 reg val		@;'bit set' -- example of use: ORR_imm32 r0,0x12345678 !!note: no '#' on immediate value	
	orr \reg,#(0x000000FF & (\val) )
	orr \reg,#(0x0000FF00 & (\val) )
	orr \reg,#(0x00FF0000 & (\val) )
	orr \reg,#(0xFF000000 & (\val) )
.endm	

.macro BIC_imm32 reg val		@;'bit clear' -- example of use: BIC_imm32 r0,0x12345678 !!note: no '#' on immediate value	
	bic \reg,#(0x000000FF & (\val) )
	bic \reg,#(0x0000FF00 & (\val) )
	bic \reg,#(0x00FF0000 & (\val) )
	bic \reg,#(0xFF000000 & (\val) )
.endm	

.macro PORTBIT_read GPIOx_BASE bit	@;read 'bit' of port GPIOx, return bit value in bit0 of r0 and 'Z' flag set/clear if bit=0/1
	MOV_imm32 r2,(\GPIOx_BASE)
	ldr r0,[r2,#IDR]
	ands r0,#(1<<\bit)
	lsr r0,#\bit
.endm	

.macro PORTBIT_write GPIOx_BASE bit value	@;set 'bit' of port GPIOx to value {0,1}
	MOV_imm32 r2,(\GPIOx_BASE)
	.ifeq \value	@;must write to upper 16 bits of BSSR to clear the bit
		mov r1,#( 1<<(16+\bit))
	.else			@;write to lower 16 bits of BSSR to set the bit
		mov r1,#( 1<<(\bit))
	.endif
	str r1,[r2,#BSRR]	
.endm

.macro SWITCH_read GPIOx_BASErow bitrow GPIOx_BASEcol bitcol 
@;write bitrow=0, read bitcol, write bitrow=1, return r0=bitcol=0/1
@; assumes all other bitrow's= 1
	PORTBIT_write \GPIOx_BASErow,\bitrow,0
	PORTBIT_read \GPIOx_BASEcol, \bitcol
	PORTBIT_write \GPIOx_BASErow,\bitrow,1
.endm
	
.macro SET_bit addr bit @;logical OR position 'bit' at 'addr' with 1 
	MOV_imm32 r2,(\addr)
	ldr r1,[r2]
	ORR r1,#(1<<\bit)
	str r1,[r2]	
.endm

.macro CLR_bit addr bit @;logical AND position 'bit' at 'addr' with 0 
	MOV_imm32 r2,(\addr)
	ldr r1,[r2]
	BIC r1,#(1<<\bit)
	str r1,[r2]	
.endm

.macro CATHODE_write A,B,C,D,E,F,G,DP		@;place 0/1 pattern on P24 display cathodes A-G,DP
	PORTBIT_write GPIOC_BASE,2,\A			@;	10_PC2	CA_A/LED5-COLON	""
	PORTBIT_write GPIOA_BASE,0,\B			@;	12_PA0	CA_B/LED6-DIGIT4 ""
	PORTBIT_write GPIOC_BASE,3,\C			@;	09_PC3	CA_C/LED1-DIGIT2 ""
	PORTBIT_write GPIOA_BASE,7,\D			@;	17_PA7	CA_D-            ""
	PORTBIT_write GPIOA_BASE,4,\E			@;	16_PA4	CA_E-AN_R        ""
	PORTBIT_write GPIOC_BASE,0,\F			@;	08_PC0	CA_F/LED4-DIGIT1 ""
	PORTBIT_write GPIOA_BASE,1,\G			@;	11_PA1	CA_G/LED2-DIGIT3 ""
	PORTBIT_write GPIOA_BASE,5,\DP			@;	15_PA5	CA_DP/LED3-AN_G  ""
	PORTBIT_write GPIOC_BASE,5,0			@;	19_PC5	CA_CLK	-- clock pattern into latch
	PORTBIT_write GPIOC_BASE,5,1			@;	19_PC5	CA_CLK      ""
.endm

.macro ANODE_write X,R,G,D4,D3,D2,D1,P		@;place 0/1 pattern on P24 display anodes R(ed),G(rn),D(igit)4,-3,-2,-1,P(unctuation)
	PORTBIT_write GPIOA_BASE,7,\X			@;	17_PA7	CA_D-	-- not used
	PORTBIT_write GPIOA_BASE,4,\R			@;	16_PA4	CA_E-AN_R
	PORTBIT_write GPIOA_BASE,5,\G			@;	15_PA5	CA_DP/LED3-AN_G	
	PORTBIT_write GPIOA_BASE,0,\D4			@;	12_PA0	CA_B/LED6-DIGIT4
	PORTBIT_write GPIOA_BASE,1,\D3			@;	11_PA1	CA_G/LED2-DIGIT3
	PORTBIT_write GPIOC_BASE,3,\D2			@;	09_PC3	CA_C/LED1-DIGIT2
	PORTBIT_write GPIOC_BASE,0,\D1			@;	08_PC0	CA_F/LED4-DIGIT1
	PORTBIT_write GPIOC_BASE,2,\P			@;	10_PC2	CA_A/LED5-COLON
	PORTBIT_write GPIOC_BASE,4,0			@;	20_PC4	AN_CLK 	-- clock pattern into latch
	PORTBIT_write GPIOC_BASE,4,1			@;	20_PC4	AN_CLK      ""
.endm

	
.macro PORTBIT_init type GPIOx_BASE bit	@;modeled after port D initialization in Keil 'LED_Init.c'
@;!!do the clock enables elsewhere
@;	@; original: "	RCC->AHB1ENR  |= ((1UL <<  3) );         /* Enable GPIOD clock                */"
@;	MOV_imm32 r2,RCC_AHB1ENR
@;	ldr r1,[r2]
@;	orr r1,#(1<<3)
@;	str r1,[r2]

.ifeq \type - 0 @;type 0 assembles standard port output bit configuration for bit
	MOV_imm32 r2,(\GPIOx_BASE)

	@; original: "	GPIOD->MODER    &= ~((3UL << 2*12) | (3UL << 2*13) | (3UL << 2*14) | (3UL << 2*15)  );/* PD.12..15 is output */"
	ldr r1,[r2,#MODER]
	BIC r1,(3 << (2*\bit))
	str r1,[r2,#MODER]
	
	@; original: "	GPIOD->MODER    |=  ((1UL << 2*12) | (1UL << 2*13) |  (1UL << 2*14) |  (1UL << 2*15)  );" 
	ldr r1,[r2,#MODER]
	ORR r1,(1 << (2*\bit))
	str r1,[r2,#MODER]

	@; original: "	GPIOD->OTYPER   &= ~((1UL <<   12) | (1UL <<   13) | (1UL <<   14) | (1UL <<   15)  );   /* PD.12..15 is output Push-Pull     */"
	ldr r1,[r2,#OTYPER]
	BIC r1,(1 << \bit)
	str r1,[r2,#OTYPER]
	
	@; original: "	GPIOD->OSPEEDR  &= ~((3UL << 2*12) | (3UL << 2*13) | (3UL << 2*14) | (3UL << 2*15)  );   /* PD.12..15 is 50MHz Fast Speed     */"
	ldr r1,[r2,#OSPEEDR]
	BIC r1,(3 << (2*\bit))
	str r1,[r2,#OSPEEDR]

	@; original: "	GPIOD->OSPEEDR  |=  ((2UL << 2*12) | (2UL << 2*13) |  (2UL << 2*14) |  (2UL << 2*15)  );"
	ldr r1,[r2,#OSPEEDR]
	ORR r1,(2 << (2*\bit))
	str r1,[r2,#OSPEEDR]
                 
	@; original: "	GPIOD->PUPDR    &= ~((3UL << 2*12) | (3UL << 2*13) | (3UL << 2*14) | (3UL << 2*15)  );   /* PD.12..15 is Pull up              */"	
	ldr r1,[r2,#PUPDR]
	BIC r1,(3 << (2*\bit))
	str r1,[r2,#PUPDR]

	@; original: "	GPIOD->PUPDR    |=  ((1UL << 2*12) | (1UL << 2*13) |  (1UL << 2*14) |  (1UL << 2*15)  );"
	ldr r1,[r2,#PUPDR]
	ORR r1,(1 << (2*\bit))
	str r1,[r2,#PUPDR]
	
	.else 
		.ifeq \type - 1 @;type 1 assembles standard port input bit configuration for bit
			MOV_imm32 r2,(\GPIOx_BASE)

			@; original: "	GPIOA->MODER    &= ~((3UL << 2*0)  );         /* PA.0 is input              */"
			ldr r1,[r2,#MODER]
			BIC r1,(3 << (2*\bit))
			str r1,[r2,#MODER]
			
			@; original: "	GPIOA->OSPEEDR  &= ~((3UL << 2*0)  );         /* PA.0 is 50MHz Fast Speed   */"
			ldr r1,[r2,#OSPEEDR]
			BIC r1,(3 << (2*\bit))
			str r1,[r2,#OSPEEDR]
			
			@; original: "	GPIOA->OSPEEDR  |=  ((2UL << 2*0)  );" 
			ldr r1,[r2,#OSPEEDR]
			ORR r1,(2 << (2*\bit))
			str r1,[r2,#OSPEEDR]
			
			@; original: "	GPIOA->PUPDR    &= ~((3UL << 2*0)  );         /* PA.0 is no Pull up         */"
			ldr r1,[r2,#PUPDR]
			BIC r1,(3 << (2*\bit))
			str r1,[r2,#PUPDR]
		
			.else @;check for more types
				.ifeq \type -2 @;type 2 assembles port input with pullup bit configuration for bit
					MOV_imm32 r2,(\GPIOx_BASE)

					@; original: "	GPIOA->MODER    &= ~((3UL << 2*0)  );         /* PA.0 is input              */"
					ldr r1,[r2,#MODER]
					BIC r1,(3 << (2*\bit))
					str r1,[r2,#MODER]
					
					@; original: "	GPIOA->OSPEEDR  &= ~((3UL << 2*0)  );         /* PA.0 is 50MHz Fast Speed   */"
					ldr r1,[r2,#OSPEEDR]
					BIC r1,(3 << (2*\bit))
					str r1,[r2,#OSPEEDR]
					
					@; original: "	GPIOA->OSPEEDR  |=  ((2UL << 2*0)  );" 
					ldr r1,[r2,#OSPEEDR]
					ORR r1,(2 << (2*\bit))
					str r1,[r2,#OSPEEDR]
					
					@; original: "	GPIOA->PUPDR    &= ~((3UL << 2*0)  );         /* PA.0 is no Pull up         */"
					ldr r1,[r2,#PUPDR]
					BIC r1,(3 << (2*\bit))
					str r1,[r2,#PUPDR]

					@; our addition: "	GPIOA->PUPDR    |= ~((1UL << 2*0)  );         /* now PA.0 is Pull up         */"
					ldr r1,[r2,#PUPDR]
					ORR r1,(1 << (2*\bit))
					str r1,[r2,#PUPDR]

				.else @;default when type is wrong
					.fail 0
				.endif
		.endif
	.endif
.endm
	
