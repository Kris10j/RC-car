
#include <stdint.h>
#include "ultrasonic.h"
#include "setup.h"
#include "stm32f446re.h"


/* DISTANCE */
volatile uint16_t distance_pc4_cm  = ULTRASONIC_MAX_DISTANCE_CM;
volatile uint16_t distance_pc5_cm  = ULTRASONIC_MAX_DISTANCE_CM;
volatile uint16_t distance_pc6_cm  = ULTRASONIC_MAX_DISTANCE_CM;
volatile uint16_t distance_pc8_cm  = ULTRASONIC_MAX_DISTANCE_CM;
volatile uint16_t distance_pc9_cm  = ULTRASONIC_MAX_DISTANCE_CM;
volatile uint16_t distance_pc10_cm = ULTRASONIC_MAX_DISTANCE_CM;


/* PC4 */
volatile uint16_t echo_start_pc4 = 0u;
volatile uint16_t echo_end_pc4 = 0u;
volatile uint16_t echo_time_pc4 = 0u;
volatile uint8_t echo_ready_pc4 = 0u;

volatile uint32_t rising_counter_pc4 = 0u;
volatile uint32_t falling_counter_pc4 = 0u;


/* PC5 */
volatile uint16_t echo_start_pc5 = 0u;
volatile uint16_t echo_end_pc5 = 0u;
volatile uint16_t echo_time_pc5 = 0u;
volatile uint8_t echo_ready_pc5 = 0u;

volatile uint32_t rising_counter_pc5 = 0u;
volatile uint32_t falling_counter_pc5 = 0u;


/* PC6 */
volatile uint16_t echo_start_pc6 = 0u;
volatile uint16_t echo_end_pc6 = 0u;
volatile uint16_t echo_time_pc6 = 0u;
volatile uint8_t echo_ready_pc6 = 0u;

volatile uint32_t rising_counter_pc6 = 0u;
volatile uint32_t falling_counter_pc6 = 0u;


/* PC8 */
volatile uint16_t echo_start_pc8 = 0u;
volatile uint16_t echo_end_pc8 = 0u;
volatile uint16_t echo_time_pc8 = 0u;
volatile uint8_t echo_ready_pc8 = 0u;

volatile uint32_t rising_counter_pc8 = 0u;
volatile uint32_t falling_counter_pc8 = 0u;


/* PC9 */
volatile uint16_t echo_start_pc9 = 0u;
volatile uint16_t echo_end_pc9 = 0u;
volatile uint16_t echo_time_pc9 = 0u;
volatile uint8_t echo_ready_pc9 = 0u;

volatile uint32_t rising_counter_pc9 = 0u;
volatile uint32_t falling_counter_pc9 = 0u;


/* PC10 */
volatile uint16_t echo_start_pc10 = 0u;
volatile uint16_t echo_end_pc10 = 0u;
volatile uint16_t echo_time_pc10 = 0u;
volatile uint8_t echo_ready_pc10 = 0u;

volatile uint32_t rising_counter_pc10 = 0u;
volatile uint32_t falling_counter_pc10 = 0u;


/* DEBUG */
volatile uint32_t exti4_counter = 0u;


/* INTERNAL */
static uint8_t current_sensor = 0u;
static uint16_t last_trigger = 0u;



void ultrasonic_update(void){
    if ((uint16_t)((uint16_t)TIM8->CNT - last_trigger) >= 60000u){
        last_trigger = (uint16_t)TIM8->CNT;

        switch (current_sensor)
        {
            case 0u: ultrasonic_trigger(TRIG_FRONT_LEFT_PIN); break;

            case 1u: ultrasonic_trigger(TRIG_FRONT_CENTER_PIN); break;

            case 2u: ultrasonic_trigger(TRIG_FRONT_RIGHT_PIN); break;

            case 3u: ultrasonic_trigger(TRIG_REAR_LEFT_PIN); break;

            case 4u: ultrasonic_trigger(TRIG_REAR_CENTER_PIN); break;

            case 5u: ultrasonic_trigger(TRIG_REAR_RIGHT_PIN); break;

            default: current_sensor = 0u; break;
        }

        current_sensor++;

        if (current_sensor >= 6u)
        {
            current_sensor = 0u;
        }
    }
}




void ultrasonic_echo_calculate(void)
{
    if (echo_ready_pc4 == 1u) {
        echo_ready_pc4 = 0u;
        distance_pc4_cm = echo_time_pc4 / 58u;

        if (distance_pc4_cm > ULTRASONIC_MAX_DISTANCE_CM)
            {
                distance_pc4_cm = ULTRASONIC_MAX_DISTANCE_CM;
            }
    }


    if (echo_ready_pc5 == 1u){
        echo_ready_pc5 = 0u;
        distance_pc5_cm = echo_time_pc5 / 58u;

        if (distance_pc5_cm > ULTRASONIC_MAX_DISTANCE_CM)
            {
                distance_pc5_cm = ULTRASONIC_MAX_DISTANCE_CM;
            }
    }


    if (echo_ready_pc6 == 1u){
        echo_ready_pc6 = 0u;
        distance_pc6_cm = echo_time_pc6 / 58u;

        if (distance_pc6_cm > ULTRASONIC_MAX_DISTANCE_CM)
            {
                distance_pc6_cm = ULTRASONIC_MAX_DISTANCE_CM;
            }
    }


    if (echo_ready_pc8 == 1u){
        echo_ready_pc8 = 0u;
        distance_pc8_cm = echo_time_pc8 / 58u;

        if (distance_pc8_cm > ULTRASONIC_MAX_DISTANCE_CM)
            {
                distance_pc8_cm = ULTRASONIC_MAX_DISTANCE_CM;
            }
    }


    if (echo_ready_pc9 == 1u){
        echo_ready_pc9 = 0u;
        distance_pc9_cm = echo_time_pc9 / 58u;

        if (distance_pc9_cm > ULTRASONIC_MAX_DISTANCE_CM)
            {
                distance_pc9_cm = ULTRASONIC_MAX_DISTANCE_CM;
            }
    }


    if (echo_ready_pc10 == 1u){
        echo_ready_pc10 = 0u;
        distance_pc10_cm = echo_time_pc10 / 58u;

        if (distance_pc10_cm > ULTRASONIC_MAX_DISTANCE_CM)
            {
                distance_pc10_cm = ULTRASONIC_MAX_DISTANCE_CM;
            }
    }
}



