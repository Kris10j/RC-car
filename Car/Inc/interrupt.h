
#ifndef INTERRUPT_H_
#define INTERRUPT_H_
#include <stdint.h>


#define NVIC_ISER0  (*(volatile uint32_t*)0xE000E100u)
#define NVIC_ISER1  (*(volatile uint32_t*)0xE000E104u)

void NVIC_Priority(uint8_t irq_no, uint8_t priority_value);
void NVIC_Enable(uint8_t irq_no);



#endif /* INTERRUPT_H_ */
