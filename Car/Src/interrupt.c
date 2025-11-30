#include "interrupt.h"
#include "stm32f446re.h"

void NVIC_Priority(uint8_t irq_no, uint8_t priority_value)
{
    NVIC->IPR[irq_no] = (uint8_t)(priority_value << 4);
}

void NVIC_Enable(uint8_t irq_no)
{
    NVIC->ISER[irq_no >> 5] = (1u << (irq_no & 31u));
}
