
#include <stdint.h>
#include <stdio.h>
#include "stm32G030F6P6.h"

int main(void)
{
	GPIOA_initialize();
	ADC_initialize();
	USART_TX_initialize();
	USART_baud_setting_9600();


	while(1){
		uint16_t Vrx_value = ADC_read_Vrx() & 0x3FF;													//cut 10 bits
		uint16_t Vry_value = ADC_read_Vry() & 0x3FF;													//cut 10 bits

		uint8_t Vrx_value_low = (Vrx_value) & 0xFF;														//extract the lower 8 bits
		uint8_t Vrx_value_high = (Vrx_value >> 8) & 0x03;												//extract the upper 2 bits
		uint8_t Vry_value_low = (Vry_value) & 0xFF;														//extract the lower 8 bits
		uint8_t Vry_value_high = (Vry_value >> 8) & 0x03;												//extract the upper 2 bits

		/*Vry -> forward and backward*/
		if(Vry_value > 582){																			//forward
				USART_send('F'); USART_send(0xAA); USART_send(Vry_value_low);
				USART_send(Vry_value_high); USART_send('S'); simple_delay();
			}
			else if(Vry_value < 442){																	//backward
				USART_send('B'); USART_send(0xAA); USART_send(Vry_value_low);
				USART_send(Vry_value_high); USART_send('S'); simple_delay();
			}

		//Deadzone Vry
		if((Vry_value >= 442) && (Vry_value <= 582))  													//Y mid
		{
				USART_send('D'); USART_send(0xAA); USART_send(Vry_value_low);
				USART_send(Vry_value_high); USART_send('S'); simple_delay();
		}

		/*Vrx -> turning*/
		if(Vrx_value){
			USART_send('T'); USART_send(0xAA); USART_send(Vrx_value_low);
			USART_send(Vrx_value_high); USART_send('S'); simple_delay();
		}
	}
}






