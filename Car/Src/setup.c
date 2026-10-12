#include <stdint.h>
#include <stdlib.h>
#include "setup.h"
#include "stm32f446re.h"
#include "circular_buff.h"
#include "interrupt.h"
#include <math.h>


void simple_delay(void)
{
	for(volatile uint32_t i = 0; i < 5000 ; i++ ) { };
}

void GPIO_initialize (void)
{
	/*Peripheral clock enable */
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOA;
	RCC -> AHB1ENR |= 	RCC_AHB1ENR_GPIOB;
	RCC -> AHB1ENR |= 	RCC_AHB1ENR_GPIOC;

	/*PA0 servo motor configuration*/
	GPIOA->MODER &= CLEAR_MODER_PA0;
	GPIOA->MODER |= SET_PA0_ANALOG;
	GPIOA->PUPDR &= CLEAR_PA0_NO_PULL;

	/*PA1 servo motordriver AIN1 configuration output*/
	GPIOA->MODER &= CLEAR_MODER_PA1;
	GPIOA->MODER |= SET_PA1_OUTPUT;
	GPIOA->OTYPER &= CLEAR_PA1_OUT_PUSH_PULL;
	GPIOA->OSPEEDR &= CLEAR_OSPEEDR_PA1;
	GPIOA->OSPEEDR |= SET_PA1_HIGH_SPEED;
	GPIOA->PUPDR &= CLEAR_PA1_NO_PULL;
	GPIOA->PUPDR |= SET_PA1_PULL_DOWN;

	/*PA4 servo motordriver AIN2 configuration output*/
	GPIOA->MODER &= CLEAR_MODER_PA4;
	GPIOA->MODER |= SET_PA4_OUTPUT;
	GPIOA->OTYPER &= CLEAR_PA4_OUT_PUSH_PULL;
	GPIOA->OSPEEDR &= CLEAR_OSPEEDR_PA4;
	GPIOA->OSPEEDR |= SET_PA4_HIGH_SPEED;
	GPIOA->PUPDR &= CLEAR_PA4_NO_PULL;
	GPIOA->PUPDR |= SET_PA4_PULL_DOWN;

	/*PA8 Vry forward motion*/
	GPIOA->MODER &= CLEAR_MODER_PA8;
	GPIOA->MODER |= SET_PA8_AF;
	GPIOA->PUPDR &= CLEAR_PA8_NO_PULL;
	GPIOA->OTYPER &= CLEAR_PA8_OUT_PUSH_PULL;
	GPIOA->OSPEEDR &= CLEAR_OSPEEDR_PA8;
	GPIOA->OSPEEDR |= SET_PA8_HIGH_SPEED;
	GPIOA->AFR [1] &= CLEAR_AFR1_PA8;
	GPIOA->AFR [1] |= SET_AFR1_PA8;

	/*PA9 Vry backward motion*/
	GPIOA->MODER &= CLEAR_MODER_PA9;
	GPIOA->MODER |= SET_PA9_AF;
	GPIOA->PUPDR &= CLEAR_PA9_NO_PULL;
	GPIOA->OTYPER &= CLEAR_PA9_OUT_PUSH_PULL;
	GPIOA->OSPEEDR &= CLEAR_OSPEEDR_PA9;
	GPIOA->OSPEEDR |= SET_PA9_HIGH_SPEED;
	GPIOA->AFR [1] &= CLEAR_AFR1_PA9;
	GPIOA->AFR [1] |= SET_AFR1_PA9;

	/*PA10 CH3 PWM servo motor*/
	GPIOA->MODER &= CLEAR_MODER_PA10;
	GPIOA->MODER |= SET_PA10_AF;
	GPIOA->OTYPER &= CLEAR_PA10_OUT_PUSH_PULL;
	GPIOA->OSPEEDR &= CLEAR_OSPEEDR_PA10;
	GPIOA->OSPEEDR |= SET_PA10_HIGH_SPEED;
	GPIOA->PUPDR &= CLEAR_PA10_NO_PULL;
	GPIOA->AFR[1] &= CLEAR_AFR1_PA10;
	GPIOA->AFR[1] |= SET_AFR1_PA10;

	/*PB7 USART1_RX */
	GPIOB->MODER &= CLEAR_MODER_PB7;
	GPIOB->MODER |= SET_PB7_AF;
	GPIOB->PUPDR &= CLEAR_PB7_NO_PULL;
	GPIOB->PUPDR |= SET_PB7_PULL;
	GPIOB->AFR [0] &= CLEAR_AFR7_PB7;
	GPIOB->AFR [0] |= SET_AFR7_PB7;

	/*TRIG init - PB12, PB5, PB6, PB8, PB9, PB10*/
	GPIOB->MODER &= CLEAR_MODER_PB12;
	GPIOB->MODER |= SET_PB12_OUTPUT;
	GPIOB->OTYPER &= CLEAR_PB12_OUT_PUSH_PULL;
	GPIOB->OSPEEDR &= CLEAR_OSPEEDR_PB12;
	GPIOB->OSPEEDR |= SET_PB12_HIGH_SPEED;
	GPIOB->PUPDR &= CLEAR_PB12_NO_PULL;

	GPIOB->MODER &= CLEAR_MODER_PB5;
	GPIOB->MODER |= SET_PB5_OUTPUT;
	GPIOB->OTYPER &= CLEAR_PB5_OUT_PUSH_PULL;
	GPIOB->OSPEEDR &= CLEAR_OSPEEDR_PB5;
	GPIOB->OSPEEDR |= SET_PB5_HIGH_SPEED;
	GPIOB->PUPDR &= CLEAR_PB5_NO_PULL;

	GPIOB->MODER &= CLEAR_MODER_PB6;
	GPIOB->MODER |= SET_PB6_OUTPUT;
	GPIOB->OTYPER &= CLEAR_PB6_OUT_PUSH_PULL;
	GPIOB->OSPEEDR &= CLEAR_OSPEEDR_PB6;
	GPIOB->OSPEEDR |= SET_PB6_HIGH_SPEED;
	GPIOB->PUPDR &= CLEAR_PB6_NO_PULL;

	GPIOB->MODER &= CLEAR_MODER_PB8;
	GPIOB->MODER |= SET_PB8_OUTPUT;
	GPIOB->OTYPER &= CLEAR_PB8_OUT_PUSH_PULL;
	GPIOB->OSPEEDR &= CLEAR_OSPEEDR_PB8;
	GPIOB->OSPEEDR |= SET_PB8_HIGH_SPEED;
	GPIOB->PUPDR &= CLEAR_PB8_NO_PULL;

	GPIOB->MODER &= CLEAR_MODER_PB9;
	GPIOB->MODER |= SET_PB9_OUTPUT;
	GPIOB->OTYPER &= CLEAR_PB9_OUT_PUSH_PULL;
	GPIOB->OSPEEDR &= CLEAR_OSPEEDR_PB9;
	GPIOB->OSPEEDR |= SET_PB9_HIGH_SPEED;
	GPIOB->PUPDR &= CLEAR_PB9_NO_PULL;

	GPIOB->MODER &= CLEAR_MODER_PB10;
	GPIOB->MODER |= SET_PB10_OUTPUT;
	GPIOB->OTYPER &= CLEAR_PB10_OUT_PUSH_PULL;
	GPIOB->OSPEEDR &= CLEAR_OSPEEDR_PB10;
	GPIOB->OSPEEDR |= SET_PB10_HIGH_SPEED;
	GPIOB->PUPDR &= CLEAR_PB10_NO_PULL;

	/*ECHO init - PC4, PC5, PC6, PC8, PC9, PC10*/
	GPIOC->MODER &= CLEAR_MODER_PC4;
	GPIOC->PUPDR &= CLEAR_PC4_NO_PULL;
	GPIOC->PUPDR |= SET_PC4_PULL_DOWN;

	GPIOC->MODER &= CLEAR_MODER_PC5;
	GPIOC->PUPDR &= CLEAR_PC5_NO_PULL;
	GPIOC->PUPDR |= SET_PC5_PULL_DOWN;

	GPIOC->MODER &= CLEAR_MODER_PC6;
	GPIOC->PUPDR &= CLEAR_PC6_NO_PULL;
	GPIOC->PUPDR |= SET_PC6_PULL_DOWN;

	GPIOC->MODER &= CLEAR_MODER_PC8;
	GPIOC->PUPDR &= CLEAR_PC8_NO_PULL;
	GPIOC->PUPDR |= SET_PC8_PULL_DOWN;

	GPIOC->MODER &= CLEAR_MODER_PC9;
	GPIOC->PUPDR &= CLEAR_PC9_NO_PULL;
	GPIOC->PUPDR |= SET_PC9_PULL_DOWN;

	GPIOC->MODER &= CLEAR_MODER_PC10;
	GPIOC->PUPDR &= CLEAR_PC10_NO_PULL;
	GPIOC->PUPDR |= SET_PC10_PULL_DOWN;



}

void ADC1_initialize (void)
{
	/*RCC enable*/
	RCC->APB2ENR |= RCC_APB2ENR_ADC1;

	/*SMPR2 -> Sample time register set 56 cycles [011], bits 2:0*/
	ADC1 -> SMPR2 &= CLEAR_SMPR_PA0;
	ADC1 -> SMPR2 |= SET_SMPR_PA0;

	/*CR1 -> Control register resolution set 10 bit [01], bits 25:24*/
	ADC1 -> CR1 &= CLEAR_RES;
	ADC1 -> CR1 |= SET_RES_10_BIT;

	/*CR2 -> Data alignment (ALIGN) set right alignment, bit 11*/
	ADC1 -> CR2 &= CLEAR_RIGHT_ALIGN;
	ADC1 -> CR2 |= SET_RIGHT_ALIGN;

	/*CR2 -> Continuous conversion (CONT) set single conversion mode [0], bit 1*/
	ADC1 -> CR2 &= CLEAR_CONT_SINGLE;
	ADC1 -> CR2 |= SET_CONT_SINGLE;

	/*CR2 -> A/D converter ON/OFF (ADON) set enable ADC [1], bit 0*/
	ADC1 -> CR2 &= CLEAR_ADON;
	ADC1 -> CR2 |= SET_ADON_ENABLE;
}

void ADC1_start_conversion (void)
{
	ADC1->SR = 0;
	ADC1 -> CR2 |= SWSTART_ENABLE;
}

uint16_t ADC1_servo (void)
{
	/*SQR1 -> Regular channel sequence length settings (L), 1 conversion [0000]*/
	ADC1 -> SQR1 &= SQR1_CONV;

	/*SQR3 -> Select 0 channel*/
	ADC1 -> SQR3 &= SQR1_CH_0;

	/*Start conversion and after return DR*/
	ADC1_start_conversion();
	while (!(ADC1->SR & EOC_CONV_COMP));

	return (uint16_t)ADC1->DR;
}

void USART1_RX_initialize (void)
{
	/*RCC enable*/
	RCC ->APB2ENR |= RCC_APB2ENR_USART1;

	/*CR1 -> RE: Receiver enable, Bit 2 [1] */
	USART1 -> CR1 &= CLEAR_RECEIVER;
	USART1 -> CR1 |= RECEIVER_ENABLE;

	/*CR1 -> World length settings 1: 1 Start bit, 9 Data bits, n Stop bit, bit 12 [0]*/
	USART1 -> CR1 &= CLEAR_WORLD_LENGTH;
	USART1 -> CR1 |= SET_WORLD_LENGTH_8_BIT;

	/*CR1 -> Oversampling mode (OVER8), bit 15 [0]*/
	USART1 -> CR1 &= CLEAR_OVERSAMPLING;
	USART1 -> CR1 |= SET_OVERSAMPLING_16;

	/*CR2 -> Select 1 stop bit [STOP], Bits 13:12 [0]*/
	USART1 -> CR2 &= CLEAR_STOP_BITS;
	USART1 -> CR2 |= SET_STOP_BIT_1;


	/*Baud rate function run */
	USART1_baud_setting_9600();
}
void USART1_RX_interrupt (void)
{
	 USART1->CR1 |= RXNE_INTERRUPT_ENABLE;						//RXNE -> RX interrupt enable, set 1
	 USART1->CR3 |= (1u << 0);
	 /*SR, DR register clear*/
	 uint32_t sr = USART1->SR;
	 (void)sr;
 	  uint8_t dr = USART1->DR;
	 (void)dr;

	 NVIC->ICPR[37 >> 5] = (1u << (37 & 31u));
	 NVIC_Priority(37, 1);
	 NVIC_Enable(37);
}


void USART1_baud_setting_9600 (void)
{
	/*USART1 disable*/
	USART1_DISABLE;

	/*BBR calculation*/
	uint32_t usartdiv = ((F_CLOCK  * 100)/(8 * (2 - OVER8) * RX_BAUD_9600));				//USARTDIV = 10416,67
	uint32_t mantissa = usartdiv / 100;														//mantissa = 104
	uint32_t fraction = (((((usartdiv) - (mantissa * 100)) * 16) +50)) / 100;				//fraction =

	/*Mantissa and fraction load to BRR  */
	USART1 -> BRR = ((uint16_t) ((mantissa << 4)  | (fraction << 0)));

	/*USART1 enable*/
	USART1_ENABLE;
}

void USART1_selection(USART_values *sum_values)
{
	uint8_t start_bit = 0; uint8_t second_bit = 0; uint8_t Vrx_value_low = 0;
	uint8_t Vrx_value_high = 0; uint8_t stop_bit = 0; uint16_t value = 0;

	while(RX_available(&buff) >= 5){
		RX_pop(&buff, &start_bit);

		if(!(start_bit == 'F' || start_bit == 'B' || start_bit == 'T' ||start_bit == 'D')) continue;
		RX_pop(&buff, &second_bit);RX_pop(&buff, &Vrx_value_low);
		RX_pop(&buff, &Vrx_value_high);RX_pop(&buff, &stop_bit);

		if(second_bit != 0xAA) continue;

		if(stop_bit != 'S') continue;

	    value = (uint16_t)((((uint16_t)(Vrx_value_high & 0x03)) << 8) | (uint16_t)Vrx_value_low);

       if (value > ADC_MAX_1023) continue;

        switch (start_bit){
            case 'F': sum_values->forward = value; sum_values->deadzone_vry = 0; sum_values->backward = ADC_MAX_1023; break;

            case 'B': sum_values->backward = value; sum_values->deadzone_vry = 0; sum_values->forward = ADC_MIN_0; break;

            case 'T': sum_values->setpoint = value; break;

            case 'D':sum_values->deadzone_vry = value; sum_values->forward  = ADC_MIN_0; sum_values->backward = ADC_MAX_1023; break;

            default: break;
        }
        return;
    }
}

void TIM1_initialize (void)
{
	/*Clock enable*/
	RCC -> APB2ENR |= RCC_APB2ENR_TIM1;

	/*ARR*/
	TIM1 -> ARR = ARR_799;

	/*PSC*/
	TIM1 -> PSC = PSC_0;

	/*Output compare mode*/
	TIM1 -> CCMR1 |= OC1PE_ENABLE;								//Output Compare 1 preload enable
	TIM1 -> CCMR1 |= OC2PE_ENABLE;								//Output Compare 2 preload enable
	TIM1 -> CCMR2 |= OC3PE_ENABLE;								//Output Compare 3 preload enable
	TIM1 -> CCMR1 |= OC1M_PWM_MODE_1;							//PWM mode 1
	TIM1 -> CCMR1 |= OC2M_PWM_MODE_1;							//PWM mode 1
	TIM1 -> CCMR2 |= OC3M_PWM_MODE_1;							//PWM mode 1
	TIM1 -> CCMR1 &= CC1S_CLEAR;								//CC1 channel is configured as output
	TIM1 -> CCMR1 &= CC2S_CLEAR;								//CC2 channel is configured as output
	TIM1 -> CCMR2 &= CC3S_CLEAR;								//CC3 channel is configured as output

	/*CCER - Capture compare register*/
	TIM1->CCER |= CC1E_OUTPUT_ENABLE;
	TIM1->CCER |= CC2E_OUTPUT_ENABLE;
	TIM1->CCER |= CC3E_OUTPUT_ENABLE;
	TIM1->CCER &= CC1P_ACTIVE_HIGH;
	TIM1->CCER &= CC2P_ACTIVE_HIGH;
	TIM1->CCER &= CC3P_ACTIVE_HIGH;

	/*EGR - event generation register*/
    TIM1->EGR |= EGR_UPDATE_GANERATION_ENABLE;					//UG = 1

	/*BDTR - break and dead-time register*/
	TIM1->BDTR |= MAIN_OUTPUT_ENABLE;    					    // MOE = 1

	/*Counter enable*/
	TIM1 -> CR1 |= COUNTER_ENABLE;
}



uint16_t TIM1_value_scale(uint16_t adc_value)
{
	return (uint16_t)(((uint32_t)(adc_value * TIM1->ARR) / 1023));
}

void TIM8_initialize(void)
{
    /*Clock enable*/
    RCC->APB2ENR |= RCC_APB2ENR_TIM8;

    /*Stop counter */
    TIM8->CR1 &= ~COUNTER_ENABLE;

    /*1 tick = 1 us*/
    TIM8->PSC = PSC_15;

    /*16 bit maximum counter value*/
    TIM8->ARR = ARR_65535;

    /* Start from zero */
    TIM8->CNT = 0;

    /*Load PSC and ARR*/
    TIM8->EGR |= EGR_UPDATE_GANERATION_ENABLE;

    /*Enable counter*/
    TIM8->CR1 |= COUNTER_ENABLE;
}

void turn_selection(float out)
{
    if (out > 0) {
        GPIOA->BSRR = BSRR_AIN1_PA1_SET | BSRR_AIN2_PA4_RESET;
        TIM1->CCR[2] = (uint16_t) TIM1_value_scale(fabsf(out));
    }
    else if (out < 0) {
        GPIOA->BSRR = BSRR_AIN1_PA1_RESET | BSRR_AIN2_PA4_SET;
        TIM1->CCR[2] = (uint16_t) TIM1_value_scale(fabsf(out));
    }
    else {
        TIM1->CCR[2] = 0;
        GPIOA->BSRR = BSRR_AIN1_PA1_SET | BSRR_AIN2_PA4_SET;
    }
}

void car_start_position(USART_values* start) {
	start -> forward      = 512;
	start -> backward     = 512;
	start -> setpoint     = FEEDBACK_MID;
	start -> deadzone_vry = 512;
	start -> deadzone_vrx = 512;
}

void enable_fpu(void){
	volatile uint32_t *CPACR = (uint32_t*)0xE000ED88;
	*CPACR |= (0xF << 20);
}

void EXTI_initialize(void){

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* EXTI4 (PC4) */
    SYSCFG->EXTICR[1] &= ~(0xFu << 0);
    SYSCFG->EXTICR[1] |=  (EXTI_PORT_C << 0);

    /* EXTI5 (PC5) */
    SYSCFG->EXTICR[1] &= ~(0xFu << 4);
    SYSCFG->EXTICR[1] |=  (EXTI_PORT_C << 4);

    /* EXTI6 (PC6) */
    SYSCFG->EXTICR[1] &= ~(0xFu << 8);
    SYSCFG->EXTICR[1] |=  (EXTI_PORT_C << 8);


    /* EXTI8 (PC8) */
    SYSCFG->EXTICR[2] &= ~(0xFu << 0);
    SYSCFG->EXTICR[2] |=  (EXTI_PORT_C << 0);

    /* EXTI9 (PC9) */
    SYSCFG->EXTICR[2] &= ~(0xFu << 4);
    SYSCFG->EXTICR[2] |=  (EXTI_PORT_C << 4);

    /* EXTI10 (PC10) */
    SYSCFG->EXTICR[2] &= ~(0xFu << 8);
    SYSCFG->EXTICR[2] |=  (EXTI_PORT_C << 8);


    /*IMR -> Interrupt Mask Register (Enable Interrupt)*/
    EXTI->IMR |= EXTI_LINE_4;
    EXTI->IMR |= EXTI_LINE_5;
    EXTI->IMR |= EXTI_LINE_6;
    EXTI->IMR |= EXTI_LINE_8;
    EXTI->IMR |= EXTI_LINE_9;
    EXTI->IMR |= EXTI_LINE_10;

    /* PR -> Pending Register (reset)*/
    EXTI->PR = EXTI_LINE_4;
    EXTI->PR = EXTI_LINE_5;
    EXTI->PR = EXTI_LINE_6;
    EXTI->PR = EXTI_LINE_8;
    EXTI->PR = EXTI_LINE_9;
    EXTI->PR = EXTI_LINE_10;

    /* RTSR -> Rising Trigger Selection Register*/
    EXTI->RTSR |= EXTI_LINE_4;
    EXTI->RTSR |= EXTI_LINE_5;
    EXTI->RTSR |= EXTI_LINE_6;
    EXTI->RTSR |= EXTI_LINE_8;
    EXTI->RTSR |= EXTI_LINE_9;
    EXTI->RTSR |= EXTI_LINE_10;

    /*FTSR -> Falling Trigger Selection Register*/
    EXTI->FTSR |= EXTI_LINE_4;
    EXTI->FTSR |= EXTI_LINE_5;
    EXTI->FTSR |= EXTI_LINE_6;
    EXTI->FTSR |= EXTI_LINE_8;
    EXTI->FTSR |= EXTI_LINE_9;
    EXTI->FTSR |= EXTI_LINE_10;


    NVIC->ISER[0] |= (1 << 10);

    /* Clear NVIC pending interrupt */
    NVIC->ICPR[IRQ_EXTI4 >> 5] = (1u << (IRQ_EXTI4 & 31u));
    NVIC->ICPR[IRQ_EXTI9_5 >> 5] = (1u << (IRQ_EXTI9_5 & 31u));
    NVIC->ICPR[IRQ_EXTI15_10 >> 5] = (1u << (IRQ_EXTI15_10 & 31u));

    /* Interrupt priority */
    NVIC_Priority(IRQ_EXTI4, 2);
    NVIC_Priority(IRQ_EXTI9_5, 2);
    NVIC_Priority(IRQ_EXTI15_10, 2);

    /* Enable NVIC interrupts */
    NVIC_Enable(IRQ_EXTI4);
    NVIC_Enable(IRQ_EXTI9_5);
    NVIC_Enable(IRQ_EXTI15_10);

}

void ultrasonic_trigger(uint8_t trig_pin)
{
    uint16_t trigger_start;
    uint32_t timeout = 1000u;

    GPIOB->BSRR = (1u << trig_pin);
    trigger_start = (uint16_t)TIM8->CNT;

    /* Várakozás 10 us-ra biztonsági timeout-tal */
    while (((uint16_t)((uint16_t)TIM8->CNT - trigger_start) < 10u) && (timeout > 0u))
    {
        timeout--;
    }

    GPIOB->BSRR = (1u << (trig_pin + 16u));
}


