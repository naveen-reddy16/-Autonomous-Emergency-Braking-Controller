#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <signal.h>

#include "public/rpi_gpio.h"
#include "mpu6050.h"
#include "hcsr04.h"
#include "aeb_logic.h"

/* BCM GPIO assignments */
#define US_REAR_TRIG   23   /* Ultrasonic 1: rear  */
#define US_REAR_ECHO   24
#define US_FRONT_TRIG  27   /* Ultrasonic 2: front */
#define US_FRONT_ECHO  22

#define LEFT_IN1        5
#define LEFT_IN2        6
#define RIGHT_IN3      13
#define RIGHT_IN4      19

#define BUZZER_PIN     26
#define RED_LED_PIN    20
#define GREEN_LED_PIN  21

#define I2C_DEVICE "/dev/i2c1"

/* ---- MPU6050 field mapping: ADJUST to match mpu6050.h ---- */
#define IMU_FWD_G(d)   ((d).accel_x)   /* axis pointing along car's forward */
#define IMU_UP_G(d)    ((d).accel_z)   /* axis pointing up (gravity)        */
#define IMU_YAW_DPS(d) ((d).gyro_z)    /* rotation about vertical axis      */

#define G_TO_MPS2          9.80665
#define IMU_CAL_SAMPLES    50
#define TILT_MIN_UP_G      0.5     /* below this: flipped / lifted / tilted */
#define ACCEL_DEADBAND_G   0.02    /* ignore noise when integrating         */
#define MAX_SPEED_MPS      3.0
#define YAW_WARN_DPS       30.0    /* veering warning while driving         */
#define STILL_RESET_S      0.5     /* zero speed this long after stop       */

#define FILTER_SAMPLES    3
#define CLOSE_CM         10.0
#define REAR_STOP_CM     15.0
#define DIR_CHANGE_MS   300
#define PRINT_EVERY_N     5

typedef struct {
    double samples[FILTER_SAMPLES];
    int count;
    int next;
} distance_filter_t;

static volatile sig_atomic_t g_run = 1;

static void on_signal(int sig)
{
    (void)sig;
    g_run = 0;
}

/* Absolute value without needing libm. */
static double abs_d(double x)
{
    return (x < 0.0) ? -x : x;
}

static void delay_ms(long milliseconds)
{
    struct timespec req;
    req.tv_sec = milliseconds / 1000;
    req.tv_nsec = (milliseconds % 1000) * 1000000L;
    while (nanosleep(&req, &req) == -1) {
        if (errno != EINTR) {
            break;
        }
    }
}

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static double median3(double a, double b, double c)
{
    double t;
    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { t = a; a = b; b = t; }
    return b;
}

static double filter_distance(distance_filter_t *f, double raw_cm)
{
    f->samples[f->next] = raw_cm;
    f->next = (f->next + 1) % FILTER_SAMPLES;
    if (f->count < FILTER_SAMPLES) {
        f->count++;
    }

    if (raw_cm <= CLOSE_CM || f->count < FILTER_SAMPLES) {
        return raw_cm;
    }
    return median3(f->samples[0], f->samples[1], f->samples[2]);
}

/* ---- All four motors always get the same command ---- */
typedef enum { DRIVE_COAST, DRIVE_FORWARD, DRIVE_REVERSE, DRIVE_BRAKE } drive_cmd_t;

static drive_cmd_t g_last_cmd = DRIVE_COAST;
static int g_cmd_valid = 0;

static int write_pins(int a, int b)
{
    int r1 = rpi_gpio_output(LEFT_IN1,  a);
    int r3 = rpi_gpio_output(RIGHT_IN3, a);
    int r2 = rpi_gpio_output(LEFT_IN2,  b);
    int r4 = rpi_gpio_output(RIGHT_IN4, b);

    if (r1 != GPIO_SUCCESS || r2 != GPIO_SUCCESS ||
        r3 != GPIO_SUCCESS || r4 != GPIO_SUCCESS) {
        rpi_gpio_output(LEFT_IN1,  GPIO_LOW);
        rpi_gpio_output(RIGHT_IN3, GPIO_LOW);
        rpi_gpio_output(LEFT_IN2,  GPIO_LOW);
        rpi_gpio_output(RIGHT_IN4, GPIO_LOW);
        return -1;
    }
    return 0;
}

static int drive_all(drive_cmd_t cmd)
{
    int a = GPIO_LOW, b = GPIO_LOW;

    if (g_cmd_valid && cmd == g_last_cmd) {
        return 0;
    }

    if ((g_last_cmd == DRIVE_FORWARD && cmd == DRIVE_REVERSE) ||
        (g_last_cmd == DRIVE_REVERSE && cmd == DRIVE_FORWARD)) {
        write_pins(GPIO_HIGH, GPIO_HIGH);
        delay_ms(DIR_CHANGE_MS);
    }

    switch (cmd) {
    case DRIVE_FORWARD: a = GPIO_HIGH; b = GPIO_LOW;  break;
    case DRIVE_REVERSE: a = GPIO_LOW;  b = GPIO_HIGH; break;
    case DRIVE_BRAKE:   a = GPIO_HIGH; b = GPIO_HIGH; break;
    case DRIVE_COAST:
    default:            break;
    }

    if (write_pins(a, b) != 0) {
        g_last_cmd = DRIVE_COAST;
        g_cmd_valid = 1;
        return -1;
    }

    g_last_cmd = cmd;
    g_cmd_valid = 1;
    return 0;
}

static void motor_stop(void)     { drive_all(DRIVE_COAST); }
static void motor_brake(void)    { drive_all(DRIVE_BRAKE); }
static int  motor_forward(void)  { return drive_all(DRIVE_FORWARD); }
static int  motor_reverse(void)  { return drive_all(DRIVE_REVERSE); }

/* ---- IMU: calibration, speed estimate, tilt check ---- */
typedef struct {
    double fwd_bias_g;     /* forward-axis offset measured at rest */
    double speed_mps;      /* signed: + forward, - reverse         */
    double last_time_s;
    double still_since_s;  /* when motors last stopped driving     */
} imu_state_t;

static int imu_calibrate(imu_state_t *s)
{
    mpu6050_data_t d;
    double sum = 0.0;
    int n = 0, i;

    for (i = 0; i < IMU_CAL_SAMPLES; i++) {
        if (mpu6050_read(&d) == 0) {
            sum += IMU_FWD_G(d);
            n++;
        }
        delay_ms(10);
    }
    if (n < IMU_CAL_SAMPLES / 2) {
        return -1;
    }
    s->fwd_bias_g = sum / n;
    s->speed_mps = 0.0;
    s->last_time_s = now_s();
    s->still_since_s = s->last_time_s;
    return 0;
}

/* Integrate acceleration into speed. Drifts, so it is zeroed whenever
   the motors have been stopped for STILL_RESET_S. */
static void imu_update_speed(imu_state_t *s, const mpu6050_data_t *d,
                             drive_cmd_t cmd)
{
    double t = now_s();
    double dt = t - s->last_time_s;
    double acc_g = IMU_FWD_G(*d) - s->fwd_bias_g;

    s->last_time_s = t;

    if (cmd == DRIVE_FORWARD || cmd == DRIVE_REVERSE) {
        s->still_since_s = t;
    }

    if (t - s->still_since_s > STILL_RESET_S) {
        s->speed_mps = 0.0;
        return;
    }

    if (abs_d(acc_g) > ACCEL_DEADBAND_G && dt > 0.0 && dt < 1.0) {
        s->speed_mps += acc_g * G_TO_MPS2 * dt;
    }
    if (s->speed_mps >  MAX_SPEED_MPS) s->speed_mps =  MAX_SPEED_MPS;
    if (s->speed_mps < -MAX_SPEED_MPS) s->speed_mps = -MAX_SPEED_MPS;
}

static int imu_tilt_ok(const mpu6050_data_t *d)
{
    return abs_d(IMU_UP_G(*d)) >= TILT_MIN_UP_G;
}

/* ---- Motor test ---- */
static void set_side_raw(int in_a, int in_b, int a, int b)
{
    rpi_gpio_output(in_a, a);
    rpi_gpio_output(in_b, b);
}

static void run_motor_test(void)
{
    puts("TEST 1/4: LEFT side only (2 s)");
    set_side_raw(LEFT_IN1, LEFT_IN2, GPIO_HIGH, GPIO_LOW);
    delay_ms(2000);
    set_side_raw(LEFT_IN1, LEFT_IN2, GPIO_LOW, GPIO_LOW);
    delay_ms(1000);

    puts("TEST 2/4: RIGHT side only (2 s)");
    set_side_raw(RIGHT_IN3, RIGHT_IN4, GPIO_HIGH, GPIO_LOW);
    delay_ms(2000);
    set_side_raw(RIGHT_IN3, RIGHT_IN4, GPIO_LOW, GPIO_LOW);
    delay_ms(1000);

    puts("TEST 3/4: ALL FOUR forward (2 s)");
    g_cmd_valid = 0;
    drive_all(DRIVE_FORWARD);
    delay_ms(2000);
    drive_all(DRIVE_COAST);
    delay_ms(1000);

    puts("TEST 4/4: ALL FOUR reverse (2 s)");
    drive_all(DRIVE_REVERSE);
    delay_ms(2000);
    drive_all(DRIVE_COAST);

    puts("Motor test finished.");
}

static int setup_outputs(void)
{
    const int pins[] = {
        LEFT_IN1, LEFT_IN2, RIGHT_IN3, RIGHT_IN4,
        BUZZER_PIN, RED_LED_PIN, GREEN_LED_PIN
    };
    const size_t count = sizeof(pins) / sizeof(pins[0]);
    size_t i;

    for (i = 0; i < count; i++) {
        if (rpi_gpio_setup(pins[i], GPIO_OUT) != GPIO_SUCCESS) {
            return -1;
        }
        if (rpi_gpio_output(pins[i], GPIO_LOW) != GPIO_SUCCESS) {
            return -1;
        }
    }
    return 0;
}

static void update_indicators(const aeb_output_t *output)
{
    rpi_gpio_output(BUZZER_PIN,    output->buzzer_on    ? GPIO_HIGH : GPIO_LOW);
    rpi_gpio_output(RED_LED_PIN,   output->red_led_on   ? GPIO_HIGH : GPIO_LOW);
    rpi_gpio_output(GREEN_LED_PIN, output->green_led_on ? GPIO_HIGH : GPIO_LOW);
}

int main(int argc, char *argv[])
{
    const char *i2c_device = I2C_DEVICE;
    int test_mode = 0;

    hcsr04_t sensor_front;
    hcsr04_t sensor_rear;
    mpu6050_data_t imu_data;
    imu_state_t imu = {0};
    aeb_input_t input = {0};
    aeb_output_t output = {0};
    distance_filter_t filter_front = {0};
    distance_filter_t filter_rear = {0};

    int gpio_initialized = 0;
    int imu_initialized = 0;
    int rc = EXIT_FAILURE;
    unsigned long loop_count = 0;

    memset(&imu_data, 0, sizeof(imu_data));

    if (argc > 1) {
        if (strcmp(argv[1], "--test") == 0) {
            test_mode = 1;
        } else {
            i2c_device = argv[1];
        }
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    puts("AEB Controller starting.");

    gpio_initialized = 1;
    if (setup_outputs() != 0) {
        fprintf(stderr, "Failed to configure GPIO outputs.\n");
        goto cleanup;
    }
    motor_stop();

    if (test_mode) {
        run_motor_test();
        rc = EXIT_SUCCESS;
        goto cleanup;
    }

    if (mpu6050_init(i2c_device) != 0) {
        fprintf(stderr, "MPU6050 initialization failed.\n");
        goto cleanup;
    }
    imu_initialized = 1;

    puts("Calibrating IMU: keep the car still...");
    if (imu_calibrate(&imu) != 0) {
        fprintf(stderr, "IMU calibration failed.\n");
        goto cleanup;
    }

    if (hcsr04_init(&sensor_front, US_FRONT_TRIG, US_FRONT_ECHO) != 0) {
        fprintf(stderr, "Front ultrasonic (US2) initialization failed.\n");
        goto cleanup;
    }
    if (hcsr04_init(&sensor_rear, US_REAR_TRIG, US_REAR_ECHO) != 0) {
        fprintf(stderr, "Rear ultrasonic (US1) initialization failed.\n");
        goto cleanup;
    }

    puts("Sensor interfaces initialized.");

    while (g_run) {
        double raw_front = 0.0, raw_rear = 0.0;
        double dist_front = 0.0, dist_rear = 0.0;
        int front_ok, rear_ok, imu_ok, tilt_ok = 1, veering = 0;
        const char *action;

        front_ok = (hcsr04_read_cm(&sensor_front, &raw_front) == 0);
        delay_ms(60);
        rear_ok  = (hcsr04_read_cm(&sensor_rear, &raw_rear) == 0);
        imu_ok   = (mpu6050_read(&imu_data) == 0);

        if (front_ok) dist_front = filter_distance(&filter_front, raw_front);
        if (rear_ok)  dist_rear  = filter_distance(&filter_rear, raw_rear);

        if (imu_ok) {
            imu_update_speed(&imu, &imu_data, g_last_cmd);
            tilt_ok = imu_tilt_ok(&imu_data);
            veering = (g_last_cmd == DRIVE_FORWARD ||
                       g_last_cmd == DRIVE_REVERSE) &&
                      abs_d(IMU_YAW_DPS(imu_data)) > YAW_WARN_DPS;
        }

        input.front_sensor_valid  = front_ok;
        input.second_sensor_valid = rear_ok;
        input.imu_valid           = imu_ok && tilt_ok;
        input.front_distance_cm   = dist_front;
        input.second_distance_cm  = dist_rear;
        input.speed_mps           = abs_d(imu.speed_mps);

        aeb_update(&input, &output);

        if (!(front_ok && imu_ok)) {
            motor_stop();
            action = "STOP"
            		"";
        } else if (!tilt_ok) {
            motor_stop();
            action = "STOP (tilted/lifted)";
        } else if (output.state == AEB_STATE_CLEAR) {
            if (motor_forward() != 0) {
                fprintf(stderr, "Motor GPIO command failed.\n");
            }
            action = "FORWARD";
        } else if (rear_ok && dist_rear > REAR_STOP_CM) {
            if (motor_reverse() != 0) {
                fprintf(stderr, "Motor GPIO command failed.\n");
            }
            action = "FORWARD";
        } else {
            motor_brake();
            action = "BRAKE (blocked front and rear)";
        }

        update_indicators(&output);

        if ((loop_count++ % PRINT_EVERY_N) == 0) {
            printf("FRONT=%.1f(%d) REAR=%.1f(%d) IMU=%s speed=%.2f m/s "
                   "yaw=%.1f%s AEB=%d -> %s\n",
                   dist_front, front_ok, dist_rear, rear_ok,
                   imu_ok ? "OK" : "FAIL", imu.speed_mps,
                   imu_ok ? IMU_YAW_DPS(imu_data) : 0.0,
                   veering ? " VEERING" : "",
                   (int)output.state, action);
        }

        delay_ms(10);
    }

    rc = EXIT_SUCCESS;

cleanup:
    g_cmd_valid = 0;
    g_last_cmd = DRIVE_COAST;
    motor_stop();
    rpi_gpio_output(BUZZER_PIN, GPIO_LOW);
    rpi_gpio_output(RED_LED_PIN, GPIO_LOW);
    rpi_gpio_output(GREEN_LED_PIN, GPIO_LOW);

    if (gpio_initialized) {
        rpi_gpio_cleanup();
    }
    if (imu_initialized) {
        mpu6050_close();
    }
    return rc;
}
