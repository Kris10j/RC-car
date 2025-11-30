#ifndef PID_H_
#define PID_H_

#include <stdint.h>

typedef struct
{
	volatile float Kp, Ki;
	volatile float delta_t;
	volatile float previous_error;
	volatile float previous_integral;
	volatile float previous_output;
	volatile float output_min, output_max;
}PI_t;


void pi_init(PI_t *p);
float pi_update(PI_t *pid, int16_t setpoint, int16_t feedback);



#endif /* PID_H_ */
