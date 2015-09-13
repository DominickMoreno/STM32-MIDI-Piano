Dominick Moreno
ENEE 440 - Spring 2015
HW 2

Problem 1) In order to calibrate the delay to an accurate second I did not need to change the blinking pattern in main().
	The pattern in the blinky program was simple enough to be able to time accurately, since it just goes back and
	forth in a three-quarters circle. Additionally, when blinking, each LED is on for as long as it is off. This
	means timing 10 "off->on" or "on->off" state changes times 10 delay functions. This was my method.

	Using the asmDelay() function already written as a template, I took an educated case and first tried decrementing
	down from 8,000,000 = 0x7A1200. I was lucky, in that this took exactly 20.0 seconds for 10 state changes to occur,
	meaning this was exactly twice as large of a number to decrement from as I desired. My second attempt I used
	4,000,000 = 0x3D0900, which to the best measure of accuracy I have for timing by hand (to the 10ths place), took
	exactly 10 seconds, as was desired.
	