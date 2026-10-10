
#include "hcsr04.h"
#include "public/rpi_gpio.h"

#include <errno.h>
#include <stdint.h>
#include <time.h>

#define HCSR04_TRIGGER_US       10
#define HCSR04_TIMEOUT_NS       30000000LL
#define HCSR04_POLL_INTERVAL_NS 10000LL

static void delay_ns(long long ns)
{
    struct timespec req;

    req.tv_sec = (time_t)(ns / 1000000000LL);
    req.tv_nsec = (long)(ns % 1000000000LL);

    while (nanosleep(&req, &req) == -1 && errno == EINTR) {
        /* Continue sleeping after interruption. */
    }
}

static long long monotonic_ns(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return -1;
    }

    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

int hcsr04_init(hcsr04_t *sensor,
                int trigger_gpio,
                int echo_gpio)
{
    if (sensor == NULL ||
        trigger_gpio < 0 || trigger_gpio >= GPIO_COUNT ||
        echo_gpio < 0 || echo_gpio >= GPIO_COUNT ||
        trigger_gpio == echo_gpio) {
        return -1;
    }

    sensor->trigger_gpio = trigger_gpio;
    sensor->echo_gpio = echo_gpio;

    if (rpi_gpio_setup(trigger_gpio, GPIO_OUT) != GPIO_SUCCESS) {
        return -1;
    }

    if (rpi_gpio_setup(echo_gpio, GPIO_IN) != GPIO_SUCCESS) {
        return -1;
    }

    if (rpi_gpio_output(trigger_gpio, GPIO_LOW) != GPIO_SUCCESS) {
        return -1;
    }

    return 0;
}

int hcsr04_read_cm(hcsr04_t *sensor, double *distance_cm)
{
    unsigned level;
    long long start;
    long long deadline;
    long long pulse_start;
    long long pulse_end;

    if (sensor == NULL || distance_cm == NULL) {
        return -1;
    }

    /* Ensure echo is low before starting a new measurement. */
    start = monotonic_ns();
    if (start < 0) {
        return -1;
    }

    deadline = start + HCSR04_TIMEOUT_NS;

    do {
        if (rpi_gpio_input(sensor->echo_gpio, &level) != GPIO_SUCCESS) {
            return -1;
        }

        if (level == GPIO_LOW) {
            break;
        }

        if (monotonic_ns() >= deadline) {
            return -1;
        }

        delay_ns(HCSR04_POLL_INTERVAL_NS);
    } while (1);

    if (rpi_gpio_output(sensor->trigger_gpio, GPIO_LOW) != GPIO_SUCCESS) {
        return -1;
    }

    delay_ns(2000000LL);

    if (rpi_gpio_output(sensor->trigger_gpio, GPIO_HIGH) != GPIO_SUCCESS) {
        return -1;
    }

    delay_ns(HCSR04_TRIGGER_US * 1000LL);

    if (rpi_gpio_output(sensor->trigger_gpio, GPIO_LOW) != GPIO_SUCCESS) {
        return -1;
    }

    /* Wait for echo to rise. */
    start = monotonic_ns();
    if (start < 0) {
        return -1;
    }

    deadline = start + HCSR04_TIMEOUT_NS;

    do {
        if (rpi_gpio_input(sensor->echo_gpio, &level) != GPIO_SUCCESS) {
            return -1;
        }

        if (level == GPIO_HIGH) {
            break;
        }

        if (monotonic_ns() >= deadline) {
            return -1;
        }

        delay_ns(HCSR04_POLL_INTERVAL_NS);
    } while (1);

    pulse_start = monotonic_ns();
    if (pulse_start < 0) {
        return -1;
    }

    /* Wait for echo to fall. */
    deadline = pulse_start + HCSR04_TIMEOUT_NS;

    do {
        if (rpi_gpio_input(sensor->echo_gpio, &level) != GPIO_SUCCESS) {
            return -1;
        }

        if (level == GPIO_LOW) {
            break;
        }

        if (monotonic_ns() >= deadline) {
            return -1;
        }

        delay_ns(HCSR04_POLL_INTERVAL_NS);
    } while (1);

    pulse_end = monotonic_ns();
    if (pulse_end < pulse_start) {
        return -1;
    }

    /* HC-SR04 nominal conversion: echo microseconds / 58 = cm. */
    *distance_cm = (double)(pulse_end - pulse_start) / 58000.0;

    return 0;
}
