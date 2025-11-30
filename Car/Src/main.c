#include <stdint.h>
#include "setup.h"
#include "stm32f446re.h"
#include "circular_buff.h"
#include "PID.h"

extern PI_t pi;
USART_values usart_val;
uint16_t forward;
uint16_t backward;
uint16_t deadzone_vry;
float output = 0;

int main(void)
{
	circ_init(&buff);
    car_start_position(&usart_val);
    pi_init(&pi);
	GPIO_initialize();
    USART1_RX_initialize();
    USART1_RX_interrupt();
	TIM1_initialize();
    ADC1_initialize();
    enable_fpu();

    while(1){

    	//Sorting directions
    	USART1_selection(&usart_val);

    	//Forward and backward calculate
    	forward = usart_val.forward;
    	backward = usart_val.backward;
    	deadzone_vry = usart_val.deadzone_vry;

    	//Read servo position
    	uint16_t feedback_servo = ADC1_servo();
    	uint16_t setpoint = usart_val.setpoint;
    	int16_t setpoint_calculate = (int16_t)(setpoint - TRANSMITTER_MID) + FEEDBACK_MID;

    	if (setpoint_calculate < FEEDBACK_MIN) setpoint_calculate = FEEDBACK_MIN;
    	if (setpoint_calculate > FEEDBACK_MAX) setpoint_calculate = FEEDBACK_MAX;

    	int16_t error = setpoint_calculate - feedback_servo;

    	if ((error > -60) && (error < 60)) {
    	    output = 0;
    	} else {
    	    output = pi_update(&pi, setpoint_calculate, feedback_servo);
    	}

    	//Moving forward
    	if (forward > DEADZONE_MAX) {
    	    TIM1->CCR[0] = 0;        															// RPWM (CH1) off
    	    TIM1->CCR[1] = TIM1_value_scale(forward/2);  										// LPWM (CH2) PWM on
    		}
    	//Moving backward
    		else if (backward < DEADZONE_MIN) {
    			TIM1->CCR[1] = 0;        														// LPWM off
    			TIM1->CCR[0] = TIM1_value_scale((1023 - backward)/2); 							// RPWM PWM on
    			}
    		else if (((deadzone_vry >= DEADZONE_MIN) && (deadzone_vry <= DEADZONE_MAX))){
    	    TIM1->CCR[0] = 0;        															// LPWM off
    		TIM1->CCR[1] = 0;        															// RPWM off
    		}

    	turn_selection(output); 																// CH3 + AIN1/2
    }
}

void USART1_IRQHandler(void){

	uint32_t SR = USART1->SR;

	if((SR) & (FRAMING_ERROR | NOISE_DETECTED | OVERRUN_ERROR)){
		uint8_t flush_dr = USART1->DR;
		(void)flush_dr;
		return;
	}
	if(SR & RXNE_READY){
		uint8_t push = USART1->DR;
		RX_push(&buff, push);
    }
}


