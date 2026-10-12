#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include <stdint.h>


/* DISTANCE */
extern volatile uint16_t distance_pc4_cm;
extern volatile uint16_t distance_pc5_cm;
extern volatile uint16_t distance_pc6_cm;
extern volatile uint16_t distance_pc8_cm;
extern volatile uint16_t distance_pc9_cm;
extern volatile uint16_t distance_pc10_cm;


/* PC4 */
extern volatile uint16_t echo_start_pc4;
extern volatile uint16_t echo_end_pc4;
extern volatile uint16_t echo_time_pc4;
extern volatile uint8_t echo_ready_pc4;

extern volatile uint32_t rising_counter_pc4;
extern volatile uint32_t falling_counter_pc4;


/* PC5 */
extern volatile uint16_t echo_start_pc5;
extern volatile uint16_t echo_end_pc5;
extern volatile uint16_t echo_time_pc5;
extern volatile uint8_t echo_ready_pc5;

extern volatile uint32_t rising_counter_pc5;
extern volatile uint32_t falling_counter_pc5;


/* PC6 */
extern volatile uint16_t echo_start_pc6;
extern volatile uint16_t echo_end_pc6;
extern volatile uint16_t echo_time_pc6;
extern volatile uint8_t echo_ready_pc6;

extern volatile uint32_t rising_counter_pc6;
extern volatile uint32_t falling_counter_pc6;


/* PC8 */
extern volatile uint16_t echo_start_pc8;
extern volatile uint16_t echo_end_pc8;
extern volatile uint16_t echo_time_pc8;
extern volatile uint8_t echo_ready_pc8;

extern volatile uint32_t rising_counter_pc8;
extern volatile uint32_t falling_counter_pc8;


/* PC9 */
extern volatile uint16_t echo_start_pc9;
extern volatile uint16_t echo_end_pc9;
extern volatile uint16_t echo_time_pc9;
extern volatile uint8_t echo_ready_pc9;

extern volatile uint32_t rising_counter_pc9;
extern volatile uint32_t falling_counter_pc9;


/* PC10 */
extern volatile uint16_t echo_start_pc10;
extern volatile uint16_t echo_end_pc10;
extern volatile uint16_t echo_time_pc10;
extern volatile uint8_t echo_ready_pc10;

extern volatile uint32_t rising_counter_pc10;
extern volatile uint32_t falling_counter_pc10;

/* DEBUG */
extern volatile uint32_t exti4_counter;


/* FUNCTIONS */
void ultrasonic_echo_calculate(void);
void ultrasonic_update(void);


#endif /* ULTRASONIC_H_ */
