
#include "aeb_logic.h"

#include <stddef.h>

#define AEB_BRAKE_DISTANCE_CM    30.0
#define AEB_WARNING_DISTANCE_CM  70.0

void aeb_update(const aeb_input_t *input,
                aeb_output_t *output)
{
    if (output == NULL) {
        return;
    }

    /* Fail-safe defaults: stop and signal a fault. */
    output->state = AEB_STATE_FAULT;
    output->motor_stop = 1;
    output->buzzer_on = 1;
    output->red_led_on = 1;
    output->green_led_on = 0;

    if (input == NULL) {
        return;
    }

    if (!input->front_sensor_valid ||
        !input->second_sensor_valid ||
        !input->imu_valid ||
        input->front_distance_cm <= 0.0 ||
        input->second_distance_cm <= 0.0 ||
        input->speed_mps < 0.0) {
        return;
    }

    double nearest_distance =
        input->front_distance_cm < input->second_distance_cm
        ? input->front_distance_cm
        : input->second_distance_cm;

    if (nearest_distance <= AEB_BRAKE_DISTANCE_CM) {
        output->state = AEB_STATE_BRAKING;
        output->motor_stop = 1;
        output->buzzer_on = 1;
        output->red_led_on = 1;
        output->green_led_on = 0;
        return;
    }

    if (nearest_distance <= AEB_WARNING_DISTANCE_CM) {
        output->state = AEB_STATE_WARNING;
        output->motor_stop = 0;
        output->buzzer_on = 1;
        output->red_led_on = 1;
        output->green_led_on = 0;
        return;
    }

    output->state = AEB_STATE_CLEAR;
    output->motor_stop = 0;
    output->buzzer_on = 0;
    output->red_led_on = 0;
    output->green_led_on = 1;
}
