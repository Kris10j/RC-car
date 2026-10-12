#include "setup.h"
#include "stm32f446re.h"
#include "stdint.h"
#include "avoidance.h"
#include "ultrasonic.h"


float steering_linear_value = 0.0f;
float speed_linear_value = 0.0f;



float speed_linear_calculation(uint16_t distance)
{
    float max_speed = (1023.0f * 40.0f) / 100.0f;
    float min_speed = (1023.0f * 20.0f) / 100.0f;

    if (distance >= OBSTACLE_DISTANCE_CM){ return max_speed; }

    if (distance <= CRITICAL_DISTANCE_CM){ return min_speed; }

    return min_speed + ((float)(distance - CRITICAL_DISTANCE_CM) / (float)(OBSTACLE_DISTANCE_CM - CRITICAL_DISTANCE_CM)) * (max_speed - min_speed);
}



float steering_linear_calculation(uint16_t distance, avoidance_t direction)
{
    float steering;

    if (distance >= OBSTACLE_DISTANCE_CM) { return FEEDBACK_MID; }

    if (distance <= CRITICAL_DISTANCE_CM)
    {
        if ((direction == AVOID_LEFT) || (direction == AVOID_REVERSE_LEFT)){
        	return FEEDBACK_MIN; }

        else {
            return FEEDBACK_MAX;
        }
    }

    steering = 2.0f * (((float)(OBSTACLE_DISTANCE_CM - distance) / (float)(OBSTACLE_DISTANCE_CM - CRITICAL_DISTANCE_CM)));

    if (steering > 1.0f) {steering = 1.0f;}

    if ((direction == AVOID_LEFT) || (direction == AVOID_REVERSE_LEFT))
    {
        return FEEDBACK_MID - steering * (FEEDBACK_MID - FEEDBACK_MIN);
    }

    else
    {
        return FEEDBACK_MID + steering * (FEEDBACK_MAX - FEEDBACK_MID);
    }
}


avoidance_t obstacle_avoidance(void)
{
    uint16_t front_left   = distance_pc4_cm;
    uint16_t front_center = distance_pc5_cm;
    uint16_t front_right  = distance_pc6_cm;

    uint16_t rear_left    = distance_pc8_cm;
    uint16_t rear_center  = distance_pc9_cm;
    uint16_t rear_right   = distance_pc10_cm;


    if ((front_left   > OBSTACLE_DISTANCE_CM) && (front_center > OBSTACLE_DISTANCE_CM) && (front_right  > OBSTACLE_DISTANCE_CM))
    {
        steering_linear_value = FEEDBACK_MID; speed_linear_value = 0.0f;

        return AVOID_NONE;
    }

    if ((front_left   <= CRITICAL_DISTANCE_CM) || (front_center <= CRITICAL_DISTANCE_CM) || (front_right  <= CRITICAL_DISTANCE_CM))
    {
        if ((rear_left   > CRITICAL_DISTANCE_CM) && (rear_center > CRITICAL_DISTANCE_CM) && (rear_right  > CRITICAL_DISTANCE_CM))
        {
            if (rear_left >= rear_right)
            {
                steering_linear_value = steering_linear_calculation(rear_right, AVOID_REVERSE_LEFT);
                speed_linear_value = speed_linear_calculation(rear_right);

                return AVOID_REVERSE_LEFT;
            }
            else
            {
                steering_linear_value =  steering_linear_calculation(rear_left, AVOID_REVERSE_RIGHT);
                speed_linear_value = speed_linear_calculation(rear_left);

                return AVOID_REVERSE_RIGHT;
            }
        }

        steering_linear_value = FEEDBACK_MID;
        speed_linear_value = 0.0f;

        return AVOID_RETURN_MANUAL;
    }

    if (front_center <= OBSTACLE_DISTANCE_CM)
    {
        if (front_left >= front_right)
        {
            steering_linear_value = steering_linear_calculation(front_center, AVOID_LEFT);

            speed_linear_value =
                speed_linear_calculation(front_center);

            return AVOID_LEFT;
        }
        else
        {
            steering_linear_value = steering_linear_calculation(front_center, AVOID_RIGHT);
            speed_linear_value = speed_linear_calculation(front_center);

            return AVOID_RIGHT;
        }
    }

    if (front_left <= OBSTACLE_DISTANCE_CM)
    {
        steering_linear_value = steering_linear_calculation(front_left, AVOID_RIGHT);
        speed_linear_value = speed_linear_calculation(front_left);

        return AVOID_RIGHT;
    }

    if (front_right <= OBSTACLE_DISTANCE_CM)
    {
        steering_linear_value = steering_linear_calculation(front_right, AVOID_LEFT);
        speed_linear_value = speed_linear_calculation(front_right);

        return AVOID_LEFT;
    }

    steering_linear_value = FEEDBACK_MID;
    speed_linear_value = 0.0f;

    return AVOID_RETURN_MANUAL;
}
