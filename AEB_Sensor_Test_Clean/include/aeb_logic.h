
#ifndef AEB_LOGIC_H
#define AEB_LOGIC_H

typedef enum {
    AEB_STATE_FAULT = 0,
    AEB_STATE_CLEAR,
    AEB_STATE_WARNING,
    AEB_STATE_BRAKING
} aeb_state_t;

typedef struct {
    double front_distance_cm;
    double second_distance_cm;
    double speed_mps;

    int front_sensor_valid;
    int second_sensor_valid;
    int imu_valid;
} aeb_input_t;

typedef struct {
    aeb_state_t state;
    int motor_stop;
    int buzzer_on;
    int red_led_on;
    int green_led_on;
} aeb_output_t;

void aeb_update(const aeb_input_t *input,
                aeb_output_t *output);

#endif
