
#ifndef HCSR04_H
#define HCSR04_H

typedef struct {
    int trigger_gpio;
    int echo_gpio;
} hcsr04_t;

/*
 * Initialize the sensor GPIO pins.
 * Returns 0 on success, -1 on failure.
 */
int hcsr04_init(hcsr04_t *sensor,
                int trigger_gpio,
                int echo_gpio);

/*
 * Read distance in centimetres.
 * Returns 0 on success, -1 on timeout or I/O error.
 */
int hcsr04_read_cm(hcsr04_t *sensor, double *distance_cm);

#endif
