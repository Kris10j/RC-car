#include <setup.h>
#include "STM32G030F6P6.h"

void simple_delay(void){

	for(volatile uint32_t i = 0; i < 5000 ; i++ ){};	//1ms
}

void GPIOA_initialize(void)
{
	/* RCC enable*/
	RCC->IOPENR |= (1u << 0);							//GPIO clock enable

	/*A0 VRx pin joystick*/
	GPIOA->MODER &= ~(3u << (PIN_0*2));					//analog mode clear
	GPIOA->MODER |= (3u << (PIN_0*2));					//analog mode set
	GPIOA->PUPDR &= ~(3u << (PIN_0*2));					//No-pull

	/*A1 VRy pin joystick*/
	GPIOA->MODER &= ~(3u << (PIN_1*2));					//analog mode clear
	GPIOA->MODER |= (3u << (PIN_1*2));					//analog mode set
	GPIOA->PUPDR &= ~(3u << (PIN_1*2));					//No-pull

	/*A2 pin USART2 TX*/
	GPIOA->MODER &= ~(3u << PIN_2*2);					//AF clear
	GPIOA->MODER |= (2u << PIN_2*2);					//AF set
	GPIOA->OTYPER &= ~(1u << PIN_2);					//Push-pull
	GPIOA->AFR[0] &= ~(15u << PIN_2*4);					//AFRL clear (AF1)
	GPIOA->AFR[0] |= (1u << PIN_2*4);					//AFRL set (AF1)
}

void ADC_initialize(void)
{

	/*RCC enable*/
	RCC->APBENR2 |= (1u << 20);							//ADC clock enable

	/*ADEN enable, only when ADCAL = 0, ADSTP = 0, ADSTART = 0, ADDIS = 0, ADEN = 0, and ADVREGEN = 1.*/
	ADC->CR &= ~(1u << 31);								//ADCAL = 0 (ADC calibration) -> bit 31 !!!!!!!!
	ADC->CR	&= ~(1u << 4);								//ADSTP = 0 (ADC stop conversion command ) -> bit 4
	ADC->CR	&= ~(1u << 2);								//ADSTART = 0 (ADC start conversion command) -> bit 2
	ADC->CR	&= ~(1u << 1);								//ADDIS = 0 (ADC disable command)->bit 1
	ADC->CR	&= ~(1u << 28);								//ADVREGEN = 0 (ADC Voltage Regulator Enable) -> bit 28

	/* ADVERGEN = 1*/
	ADC->CR	|= (1u << 28);								//ADVREGEN = 1 (ADC Voltage Regulator Enable) -> bit 28
	simple_delay();

	/*Sampling time register (101: 39.5 ADC clock cycles)*/
	ADC->SMPR &= ~(0b111 << 0);    // clear
	ADC->SMPR |=  (0b101 << 0);    // set

	/*Select align right */
	ADC->CFGR1 &= ~(1u << 5);   						// ALIGN = 0

	/*Calibration (ADCAL)  -> ADVREGEN = 1, ADEN = 0, AUTOFF = 0*/
	ADC->CR |= (1u << 31);
	while(ADC->CR & (1u << 31));

	/*Select Data resolution 10 bit (01: 10 bits) */
	ADC -> CFGR1 &= ~(3u << 3);
	ADC -> CFGR1 |= (1u << 3);

	/*Set single conversion*/
	ADC->CFGR1 &= ~(1u << 13);							//CONT = 0 (single/continuous conversion mode) -> bit 13

	/*ADEN enable*/
	ADC->CR	&= ~(1u << 0);								//ADEN = 0 (ADC enable command) -> bit 0
	ADC->CR	|= (1u << 0);								//ADEN = 1 (ADC enable command) -> bit 0
	while(!((ADC -> ISR) & (1u << 0)));
	ADC -> ISR |= (1u << 0);							//ADRDY = 1
}


void ADC_start_conversion (void)
{
	//ADC start conversion command
	ADC->CR	|= (1u << 2);								//ADSTART = 1 (ADC start conversion command) -> bit 2
}

uint16_t ADC_read_Vrx(void)
{
	/* Select Vrx (A0)*/
	ADC->CHSELR &= ~(1u  << 1);							//CHSEL = 0 (channel selection register) -> bit 1 (A1)
	ADC->CHSELR &= ~(1u << 0);							//CHSEL = 0 (channel selection register) -> bit 0 (A0)
	ADC->CHSELR |= (1u << 0);							//CHSEL = 1 (channel selection register) -> bit 0 (A0)

	ADC_start_conversion();

	while (!(ADC ->ISR & (1u << 2)));					//waiting while not EOC = 1
	return ADC -> DR;									//Read DR register
}

uint16_t ADC_read_Vry(void)
{
	/*Vry (A1) channel */
	ADC->CHSELR &= ~(1u << 0);							//CHSEL = 0 (channel selection register) -> bit 0 (A0)
	ADC->CHSELR &= ~(1u  << 1);							//CHSEL = 0 (channel selection register) -> bit 1 (A1)
	ADC->CHSELR |= (1u  << 1);							//CHSEL = 1 (channel selection register) -> bit 1 (A1)

	ADC_start_conversion();

	while (!(ADC ->ISR & (1u << 2)));					//waiting while not EOC = 1
	return ADC -> DR;									//Read DR register
}

void USART_TX_initialize(void)
{
	RCC->APBENR1 |= (1u << 17);							//USART2 clock enable

	/*Tx enable */
	USART2 -> CR1 &= ~(1u << 3);						//clear, bit 3
	USART2 -> CR1 |= (1u << 3);							//set, bit 3

	/*Word length settings (M[1:0] = ‘00’: 1 start bit, 8 Data bits, n Stop bit)*/
	USART2 -> CR1 &= ~(1u << 28);						//M1 = 0, bit 28
	USART2 -> CR1 |= (0u << 28);						//M1 = 0, bit 28
	USART2 -> CR1 &= ~(1u << 12);						//M0 = 0  bit 12
	USART2 -> CR1 |= (0u << 12);						//M0 = 0  bit 12

	/*select 1 stop bit (STOP[1:0] = '00")*/
	USART2 ->CR2 &= ~ (0b11 << 28);			//clear
	USART2 -> CR2 |= (0b00<< 28);			//set

	/*USART baudrate init*/
	USART_baud_setting_9600();
}

void USART_send (uint8_t adc_data)
{
	while(!(USART2 -> ISR & (1u << 7)));				//TXFNF
	USART2 -> TDR = adc_data;							//Write/send TDR register
	while(!(USART2->ISR & (1u << 6)));					//Waiting transmission complete (TC)
}

void USART_baud_setting_9600 (void)
{
	/*UART disable*/
	USART2 ->CR1 &= ~(0b1 << 0);			// UE = 0

	/*Select oversampling mode = 16 (Bit 15 OVER8: Oversampling mode)*/
	USART2 ->CR1 &= ~(0b1 << 15);

	/*Calculate BRR (oversampling by 16 formula reference manual)*/
	uint32_t usart_div = ((F_CK_16_MHZ)/(BAUD_9600));			// usart_div = 1666,666667 ( baud = 1666 -> error rate: +0,03%)
	USART2 ->BRR = usart_div;

	/*UART enable*/
	USART2 ->CR1 |= (0b1 << 0);			// UE = 1
}

