#include <avoidance.h>
#include <stdint.h>
#include "setup.h"
#include "stm32f446re.h"
#include "circular_buff.h"
#include "PID.h"
#include "ultrasonic.h"


extern PI_t pi;
USART_values usart_val;
uint16_t forward;
uint16_t backward;
uint16_t deadzone_vry;
float output = 0;
avoidance_t avoidance_direction = AVOID_NONE;


typedef enum {
    STATE_INIT,
    STATE_MANUAL,
	STATE_AVOIDANCE
}State;

State state= STATE_INIT;

int main(void)
{

    while(1){

    	switch (state){

    	case STATE_INIT:
    		circ_init(&buff);
    	    car_start_position(&usart_val);
    	    pi_init(&pi);
    		GPIO_initialize();
    	    USART1_RX_initialize();
    		TIM1_initialize();
    		TIM8_initialize();
    		EXTI_initialize();
    	    ADC1_initialize();
    	    enable_fpu();
    	    USART1_RX_interrupt();


    	    state = STATE_MANUAL;

    	    break;

    	case STATE_MANUAL:

    		//Sorting directions
    		USART1_selection(&usart_val);

    		//Forward and backward calculate
    		forward = usart_val.forward;
    		backward = usart_val.backward;
    		deadzone_vry = usart_val.deadzone_vry;

    		//Read servo position
    		uint16_t feedback_servo = ADC1_servo();
    		uint16_t setpoint = usart_val.setpoint;
    		int16_t setpoint_calculate = ((int16_t)TRANSMITTER_MID - (int16_t)setpoint) + FEEDBACK_MID;

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
    		      TIM1->CCR[0] = 0;        															//RPWM (CH1) off
    		      TIM1->CCR[1] = TIM1_value_scale((forward*70)/100);  								//LPWM (CH2) PWM on
    		    	}
    		 //Moving backward
    		     	else if (backward < DEADZONE_MIN) {
    		    		TIM1->CCR[1] = 0;        													//LPWM off
    		    		TIM1->CCR[0] = TIM1_value_scale(((1023 - backward) * 70) / 100); 			//RPWM PWM on
    		    		}
    		    	else{
    		    	    TIM1->CCR[0] = 0;        													//LPWM off
    		    		TIM1->CCR[1] = 0;        													//RPWM off
    		    		}

    		    	turn_selection(output); 														//CH3 + AIN1/2
    		    	ultrasonic_echo_calculate();
    		    	ultrasonic_update();
    		    	avoidance_direction = obstacle_avoidance();

    		    	if ((avoidance_direction != AVOID_NONE) && (avoidance_direction != AVOID_RETURN_MANUAL))
    		    	{
    		    		TIM1->CCR[0] = 0; TIM1->CCR[1] = 0;
    		    	    state = STATE_AVOIDANCE;
    		    	}

    		    	break;

    	case STATE_AVOIDANCE:

    	    /* Ultrasonic measurement */
    	    ultrasonic_echo_calculate();
    	    ultrasonic_update();

    	    /* Avoidance decision */
    	    avoidance_direction = obstacle_avoidance();

    	    /* There is no obstacle */
    	    if (avoidance_direction == AVOID_NONE)
    	    {
    	        TIM1->CCR[0] = 0; TIM1->CCR[1] = 0;

    	        state = STATE_MANUAL;
    	        break;
    	    }

    	    /*  There is no safe way*/
    	    if (avoidance_direction == AVOID_RETURN_MANUAL)
    	    {
    	        TIM1->CCR[0] = 0; TIM1->CCR[1] = 0;

    	        state = STATE_MANUAL;
    	        break;
    	    }


    	    /* Read servo position */
    	    uint16_t avoidance_feedback_servo = ADC1_servo();
    	    int16_t avoidance_error = (int16_t)steering_linear_value - (int16_t)avoidance_feedback_servo;


    	    /* PI steering */
    	    if ((avoidance_error > -60) && (avoidance_error < 60))
    	    {
    	        output = 0;
    	    }
    	    else
    	    {
    	        output = pi_update(&pi, (int16_t)steering_linear_value, avoidance_feedback_servo);
    	    }

    	    turn_selection(output);


    	    /* Forward avoidance */
    	    if ((avoidance_direction == AVOID_LEFT) || (avoidance_direction == AVOID_RIGHT))
    	    {
    	        TIM1->CCR[1] = 0;

    	        TIM1->CCR[0] = TIM1_value_scale((uint16_t)speed_linear_value);
    	    }


    	    /* Reverse avoidance */
    	    else if ((avoidance_direction == AVOID_REVERSE_LEFT) || (avoidance_direction == AVOID_REVERSE_RIGHT))
    	    {
    	        TIM1->CCR[0] = 0;
    	        TIM1->CCR[1] = TIM1_value_scale((uint16_t)speed_linear_value);

    	    }

    	    break;
    	}
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

void EXTI4_IRQHandler(void)
{
    if (EXTI->PR & EXTI_LINE_4)
    {
        EXTI->PR = EXTI_LINE_4;

        if (GPIOC->IDR & ECHO_PC4_PIN)
        {
            echo_start_pc4 = (uint16_t)TIM8->CNT;
        }
        else
        {
            echo_end_pc4 = (uint16_t)TIM8->CNT;
            echo_time_pc4 = (uint16_t)(echo_end_pc4 - echo_start_pc4);
            echo_ready_pc4 = 1u;
        }
    }
}

void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & EXTI_LINE_5)
    {
        EXTI->PR = EXTI_LINE_5;

        if (GPIOC->IDR & ECHO_PC5_PIN)
        {
            echo_start_pc5 = (uint16_t)TIM8->CNT;
        }
        else
        {
            echo_end_pc5 = (uint16_t)TIM8->CNT;
            echo_time_pc5 = (uint16_t)(echo_end_pc5 - echo_start_pc5);
            echo_ready_pc5 = 1u;
        }
    }

    if (EXTI->PR & EXTI_LINE_6)
    {
        EXTI->PR = EXTI_LINE_6;

        if (GPIOC->IDR & ECHO_PC6_PIN)
        {
            echo_start_pc6 = (uint16_t)TIM8->CNT;
        }
        else
        {
            echo_end_pc6 = (uint16_t)TIM8->CNT;
            echo_time_pc6 = (uint16_t)(echo_end_pc6 - echo_start_pc6);
            echo_ready_pc6 = 1u;
        }
    }

    if (EXTI->PR & EXTI_LINE_8)
    {
        EXTI->PR = EXTI_LINE_8;

        if (GPIOC->IDR & ECHO_PC8_PIN)
        {
            echo_start_pc8 = (uint16_t)TIM8->CNT;
        }
        else
        {
            echo_end_pc8 = (uint16_t)TIM8->CNT;
            echo_time_pc8 = (uint16_t)(echo_end_pc8 - echo_start_pc8);
            echo_ready_pc8 = 1u;
        }
    }

    if (EXTI->PR & EXTI_LINE_9)
    {
        EXTI->PR = EXTI_LINE_9;

        if (GPIOC->IDR & ECHO_PC9_PIN)
        {
            echo_start_pc9 = (uint16_t)TIM8->CNT;
        }
        else
        {
            echo_end_pc9 = (uint16_t)TIM8->CNT;
            echo_time_pc9 = (uint16_t)(echo_end_pc9 - echo_start_pc9);
            echo_ready_pc9 = 1u;
        }
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & EXTI_LINE_10)
    {
        EXTI->PR = EXTI_LINE_10;

        if (GPIOC->IDR & ECHO_PC10_PIN)
        {
            echo_start_pc10 = (uint16_t)TIM8->CNT;
        }
        else
        {
            echo_end_pc10 = (uint16_t)TIM8->CNT;
            echo_time_pc10 = (uint16_t)(echo_end_pc10 - echo_start_pc10);
            echo_ready_pc10 = 1u;
        }
    }
}


