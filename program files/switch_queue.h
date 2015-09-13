/** Header file for the switch_queue, an OOP-like object used to
* model consumer-producer behavior between "threads"
*
*/

#ifndef SWITCH_QUEUE_H
#define SWITCH_QUEUE_H

#ifndef SUCCESS
#define SUCCESS 1 == 1
#endif

#ifndef FAILURE
#define FAILURE !SUCCESS
#endif

#ifndef NUM_SWITCH_EVENTS
#define NUM_SWITCH_EVENTS 128
#endif

#include <stdint.h>
	
typedef struct swevent {
	unsigned int pr 	: 1;  //press-release bit: 1 -- switch was pressed; 0 -- switch was released
	unsigned int sw 	: 7;  //switch number for this event  (with room for expansion)
	unsigned int msec : 24; //time in milliseconds the event occurred (rolls over at 2**24 ms).
} swevent_t;

typedef struct switch_container {
	swevent_t events[NUM_SWITCH_EVENTS];
	uint16_t front; //Index of the next event to be read
	uint16_t back;  //Index of the next event to be written
	uint16_t size;
	uint32_t time;	//time in ms since initialization
} switch_queue_t;

void init_switch_container(switch_queue_t*);
int enqueue_sw_event(switch_queue_t*, uint16_t, uint16_t, uint32_t);
int is_not_null_event(swevent_t);
swevent_t getswitch(switch_queue_t*);

#endif