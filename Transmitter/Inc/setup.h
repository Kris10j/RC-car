/*
 * Setup.h
 *
 *  Created on: Aug 16, 2025
 *      Author: kiscs
 */

#ifndef SETUP_H_
#define SETUP_H_

#include <stdint.h>


void GPIOA_initialize(void);
void ADC_initialize(void);
void ADC_start_conversion (void);
uint16_t ADC_read_Vrx(void);
uint16_t ADC_read_Vry(void);
void USART_TX_initialize (void);
void USART_send (uint8_t adc_data);
void USART_baud_setting_9600 (void);
void simple_delay(void);


/* Pin number*/
#define PIN_0							0u
#define PIN_1							1u
#define PIN_2							2u

/*GPIO port mode register MODER[1:0] settings*/
#define INPUT							0u
#define OUTPUT							1u
#define	AF								2u
#define ANALOG							3u

/*f_ck macro*/
#define F_CK_16_MHZ						16000000u
#define BAUD_9600						9600u
#define OVER_SAMPLING_16				16u


#endif /* SETUP_H_ */
