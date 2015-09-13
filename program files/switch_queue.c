/** Implementation file for the switch_queue, an OOP-like object used to
* model consumer-producer behavior between "threads"
*
*/
#include "switch_queue.h"

//constructor for switch_queue
void init_switch_container(switch_queue_t* c) {
	int i;

	//initialize all events to NULL, since obviously we start with no switch events
	for(i = 0; i < NUM_SWITCH_EVENTS; i++) {
		c->events[i].pr = 0;
		c->events[i].sw = 0;
		c->events[i].msec = 0;
	}
	
	//Set the front/back pointers to the beginning of the events array
	c->front = c->back = c->size = c->time = 0;
}

//Enqueus a switch event given a container, "press" status, switch number and
//time of occurrence in ms. Returns SUCCESS or FAILURE depeneding on if action
//was succesful
int enqueue_sw_event(switch_queue_t* c, uint16_t pr, uint16_t sw, uint32_t ms) {
	
	//mask out the bottom 1/7/24 bits of each parameter, respectively
	pr &= 0x1;
	sw &= 0x7F;
	ms &= 0xFFFFFF;
	
	if(c->size < NUM_SWITCH_EVENTS) {
		//enqueue - write these values to the back pointer/end of the queue
		(c->events)[c->back].pr = pr;
		(c->events)[c->back].sw = sw;
		(c->events)[c->back].msec = ms;
		
		//Increment size and the back pointer
		c->back = (c->back + 1) % NUM_SWITCH_EVENTS;
		(c->size)++;
	} else {
		//queue is full, cannot enqueue
		return FAILURE;
	}
	
	return SUCCESS;
	
}

//returns SUCCESS/FAILURE according to whether or not the event is a "null"
//one - ie does not correspond to an actual event. Used to determine if a 
//dequeue was succesful
int is_not_null_event(swevent_t e) {
	if(e.pr == 0 && e.sw == 0 && e.msec == 0) {
		return FAILURE;
	}
	
	return SUCCESS;
}

//returns the next switch event in the queue, or an empty switch event if empty
swevent_t getswitch(switch_queue_t* c) {
	swevent_t return_event;
	
	if(is_not_null_event((c->events)[c->front]) == SUCCESS && c->size != 0) {
		//container is not empty - can dequeue
		return_event.pr = (c->events)[c->front].pr;
		return_event.sw = (c->events)[c->front].sw;
		return_event.msec = (c->events)[c->front].msec;
		
		c->front = (((c->front + 1)% NUM_SWITCH_EVENTS) + NUM_SWITCH_EVENTS) % NUM_SWITCH_EVENTS;
		c->size--;
	} else {
		//container is empty - cannot dequeue
		return_event.pr = return_event.sw = return_event.msec = 0;
	}
	
	return return_event;
}