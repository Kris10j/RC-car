#include <stdint.h>
#include "circular_buff.h"

circ_buff_t buff;

void circ_init(circ_buff_t *buff) {
    buff->write = 0u;
    buff->read = 0u;
    buff->maxlen = RX_BUFF_SIZE_128;
}

int RX_push(circ_buff_t *buff, uint8_t data)
{
    uint8_t next = buff->write + 1u;
    if (next >= buff->maxlen){
    	next = 0u;
    }
    if (next == buff->read) {
    	return -1;									//full
    }
    buff->buffer[buff->write] = data;
    buff->write = next;
    return 0;
}

int RX_pop(circ_buff_t *buff, uint8_t *data)
{
    if (buff->write == buff->read) {
    	return -1;
    }
    *data = buff->buffer[buff->read];

    uint16_t next = (uint16_t)(buff->read + 1u);
    if (next >= buff->maxlen){
    	next = 0u;
    }
    buff->read = next;
    return 0;
}

int RX_available(circ_buff_t *buff){

	uint8_t value = ((buff->write - buff->read + buff->maxlen) % (buff->maxlen));
	return value;}

