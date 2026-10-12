#ifndef AVOIDANCE_H_
#define AVOIDANCE_H_

#include <stdint.h>

#define OBSTACLE_DISTANCE_CM   80u
#define CRITICAL_DISTANCE_CM   40u

extern float steering_linear_value;
extern float speed_linear_value;

typedef enum
{
    AVOID_NONE,

    AVOID_FORWARD,
    AVOID_LEFT,
    AVOID_RIGHT,

    AVOID_REVERSE_LEFT,
    AVOID_REVERSE_RIGHT,

    AVOID_RETURN_MANUAL

} avoidance_t;


float speed_linear_calculation(uint16_t distance);
float steering_linear_calculation(uint16_t distance, avoidance_t direction);
avoidance_t obstacle_avoidance(void);

#endif /* AVOIDANCE_H_ */
