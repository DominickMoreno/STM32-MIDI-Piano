REM  makeSTM32F4Blinky.bat wmh 2013-02-26 : compile STM32F4DISCOVERY LED demo and .asm opcode demo 
REM !!optional -L and -l switches allow linking to Cortex M3 library functions for divide, etc. 
set path=.\;C:\yagarto\bin;

REM assemble with '-g' omitted where we want to hide things in the AXF
arm-none-eabi-as -g -mcpu=cortex-m4 -o aDemo.o CortexM4asmOps_01.asm
arm-none-eabi-as -g -mcpu=cortex-m4 -o aStartup.o SimpleStartSTM32F4_01.asm
arm-none-eabi-as -g -mcpu=cortex-m4 -o aDMA_interface.o DMA_interface.asm
REM arm-none-eabi-as -g -mcpu=cortex-m4 -o aAbboudMacros.o TAbboud_stm32f4_P24v04_definitions.asm

REM compiling C
arm-none-eabi-gcc -I./ -c -mthumb -O0 -fdata-sections -ffunction-sections -g -mcpu=cortex-m4 -save-temps -D STM32F40_41xxx -D USE_STM324xG_EVAL ./stm32f4xx_gpio.c -o gpio.o
arm-none-eabi-gcc -I./ -c -mthumb -O0 -fdata-sections -ffunction-sections -g -mcpu=cortex-m4 -save-temps -D STM32F40_41xxx -D USE_STM324xG_EVAL ./stm32f4xx_tim.c -o tim.o
arm-none-eabi-gcc -I./ -c -mthumb -O0 -fdata-sections -ffunction-sections -g -mcpu=cortex-m4 -save-temps -D STM32F40_41xxx -D USE_STM324xG_EVAL ./stm32f4xx_rcc.c -o rcc.o
arm-none-eabi-gcc -I./ -c -mthumb -O0 -fdata-sections -ffunction-sections -g -mcpu=cortex-m4 -save-temps -D STM32F40_41xxx -D USE_STM324xG_EVAL ./misc.c -o misc.o
arm-none-eabi-gcc -I./ -c -mthumb -O0 -fdata-sections -ffunction-sections -g -mcpu=cortex-m4 -save-temps -D STM32F40_41xxx -D USE_STM324xG_EVAL ./stm324xg_eval-reduced.c -o eval.o
arm-none-eabi-gcc -I./ -c -mthumb -O0 -fdata-sections -ffunction-sections -g -mcpu=cortex-m4 -save-temps -D STM32F40_41xxx -D USE_STM324xG_EVAL ./switch_queue.c -o switch_queue.o

arm-none-eabi-gcc -I./ -c -mthumb -O0 -fdata-sections -ffunction-sections -g -mcpu=cortex-m4 -save-temps -D STM32F40_41xxx -D USE_STM324xG_EVAL system_stm32f4xx.c -o Csystem.o

arm-none-eabi-gcc -I./ -c -mthumb -O0 -fdata-sections -ffunction-sections -g -mcpu=cortex-m4 -save-temps STM32F4main01.c -D STM32F40_41xxx -D USE_STM324xG_EVAL -o cMain.o
arm-none-eabi-gcc -I./ -c -mthumb -O0 -fdata-sections -ffunction-sections -g -mcpu=cortex-m4 -save-temps LED_01.c -D STM32F40_41xxx -D USE_STM324xG_EVAL -o cLED.o

REM linking
arm-none-eabi-gcc -nostartfiles -g -Wl,--gc-sections,-u,main,-Map,Blinky.map,-T linkBlinkySTM32F4_01.ld -oBlinky.elf aStartup.o aDMA_interface.o tim.o rcc.o gpio.o misc.o aDemo.o cLED.o switch_queue.o cMain.o -lgcc

REM hex file
arm-none-eabi-objcopy -O ihex Blinky.elf Blinky.hex

REM AXF file
copy Blinky.elf Blinky.AXF
pause

REM list file
arm-none-eabi-objdump -S  Blinky.axf >Blinky.lst
