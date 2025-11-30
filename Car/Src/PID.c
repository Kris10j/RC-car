#include <stdint.h>
#include "PID.h"
#include "stm32f446re.h"
#include <math.h>

PI_t pi;
extern USART_values usart_val;

void pi_init(PI_t *pid)
{
    pid->Kp = 5.0f;
    pid->Ki = 0.001f;
    pid->delta_t = 0.0025f;				//400Hz
    pid->previous_error = 0.0f;
    pid->previous_integral = 0.0f;
    pid->previous_output = 0.0f;
    pid->output_min = -991.0f;
    pid->output_max =  991.0f;
}

float pi_update(PI_t *pi, int16_t setpoint_transm, int16_t feedback_servo){

	/*Calculate error*/
	int16_t error = (int16_t)setpoint_transm - (int16_t)feedback_servo;

	/*Calculate proportinal*/
	float proportional = (float)(pi->Kp * error);

	/*Calculate integral*/
	pi->previous_integral = (float)(pi->previous_integral + (pi->Ki*(((error + pi->previous_error) / 2) * pi->delta_t)));
	float integral = pi->previous_integral;

	/*output*/
	float output = (float)(proportional + integral);

	 if(output > pi->output_max){
		 output = pi->output_max;
	 }
	 if(output < pi->output_min){
		 output = pi->output_min;
	 }

	 pi->previous_error = error;
	 pi->previous_output = output;

	return output;
}
