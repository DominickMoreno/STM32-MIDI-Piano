Dominick Moreno, UMD ENEE 440 Spring 2015 final project
MIDI Piano

This project requires the following folders and files:

	1. Appropriate 'Blinky' uvproj file type
	2. 'makeST32F4Blinky.bat'. Ensure your relative file path to yagarto matches its actual location on your machine.
	3. All appropriate C and asm files
		e.g. switch_queue.c, macro_defs.asm, CortexM4asmOps_01.asm, STM32F4main01.c, stm32f4xx_tim.h, etc
	4. CS43L22_TIM7_DAC1_DMA1.hex
	5. Keil/Yagarto build environment/IDE
	6. ST Link utility

For user convenience it is also recommended, but not essential to have:
	Frequencies.m matlab script for generating waveforms
	STM32 MIDI Piano technical documentation and user manual

To build the project on a STM32F Discovery board with custom ENEE 440 board:

	1. Flash the CS43L22_TIM7_DAC1_DMA1.hex file into location 0x08001000 in the microcontroller's memory using the ST Link utility. 
	2. Load the Blinky.uvproj into Keil. Make sure the corresponding Keil sees the Blinky.AXF file as an object file and not a C or ASm file.
	3. Compile the project using the makeST32F4Blinky.bat makefile
	4. Make sure that that >Project > Options for Target > Utilities > Settings > Flash Download **DOES NOT** have "Erase Full Chip" checked, or you will lose your step 1 above work
	5. Connect the STM32F Discovery board to the custom ENEE 440 board, and connect the microcontroller to your build environment via USB. Load the project onto the board using Keil.
	6. Disconnect then reconnect power OR press SW_B on the custom board to begin running the project.

To use the features of the STM32 MIDI Piano, consult the user manual pdf.