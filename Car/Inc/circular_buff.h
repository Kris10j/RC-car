
#ifndef CIRCULAR_BUFF_H_
#define CIRCULAR_BUFF_H_

#include <stdint.h>

#define RX_BUFF_SIZE_128										128u

typedef struct {
    uint8_t buffer[RX_BUFF_SIZE_128];
    volatile uint8_t write;
    volatile uint8_t read;
    uint8_t maxlen;
} circ_buff_t;

extern circ_buff_t buff;

void circ_init(circ_buff_t *b);
int RX_push(circ_buff_t *buff, uint8_t data);
int RX_pop (circ_buff_t *buff, uint8_t *data);
int RX_available(circ_buff_t *b);

#endif /* CIRCULAR_BUFF_H_ */
