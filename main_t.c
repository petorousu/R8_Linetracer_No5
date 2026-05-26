#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <pigpiod_if2.h>

/*
 * センサー GPIO
 * 左から S1, S2, S3, S4, S5 とする
 */
#define S1 5
#define S2 6
#define S3 13
#define S4 19
#define S5 26

#define SENSOR_COUNT 5

/*
 * PCA9685 PWM ユニット設定
 */
#define PWMI2CADR 0x40
#define PWMI2CCH 1

#define PWM_MODE1 0x00
#define PWM_MODE2 0x01
#define PWM_PRESCALE 0xFE

#define PWM_0_ON_L 0x06
#define PWM_0_ON_H 0x07
#define PWM_0_OFF_L 0x08
#define PWM_0_OFF_H 0x09

/*
 * モータードライバ接続先
 * 右モーター
 */
#define ENA_PWM 8
#define IN1_PWM 9
#define IN2_PWM 10

/*
 * 左モーター
 */
#define ENB_PWM 13
#define IN3_PWM 11
#define IN4_PWM 12

/*
 * PWM設定
 * PCA9685は 0〜4095 のPWM幅を使う
 */
#define PWM_MAX_COUNT 4095

/*
 * motor_drive() に渡す速度範囲
 * -16 〜 16 として扱う
 */
#define MOTOR_MAX 16
#define MOTOR_MIN -16

/*
 * ライントレース制御パラメータ
 */
#define BASE_SPEED 6
#define KP 3
#define KD 2

/*
 * センサー値の意味
 *
 * 黒線を読んだときに gpio_read() が 1 を返すなら 1
 * 黒線を読んだときに gpio_read() が 0 を返すなら 0
 *
 * 実機で逆ならここを 0 に変更する
 */
#define LINE_DETECTED_VALUE 1

static volatile sig_atomic_t running = 1;

static int last_error = 0;
static int previous_error = 0;

void sigHandler(int sig);
void initHardware(int *pd, int *fd);
void stopHardware(int pd, int fd);

uint8_t readSensorsAsBits(int pd);
void controlLineTracePD(int pd, int fd, uint8_t sensors);

int clamp(int value, int min, int max);

void set_pwm_output(int pd, int fd, int channel, int on_count, int off_count);
void set_pwm_full_on(int pd, int fd, int channel);
void set_pwm_full_off(int pd, int fd, int channel);

int motor_drive(int pd, int fd, int left_motor, int right_motor);

int main(void)
{
    int pd;
    int fd;

    signal(SIGINT, sigHandler);
    signal(SIGTERM, sigHandler);

    initHardware(&pd, &fd);

    motor_drive(pd, fd, 0, 0);

    while (running)
    {
        uint8_t sensors = readSensorsAsBits(pd);

        printf("sensors = 0x%02X\n", sensors);

        controlLineTracePD(pd, fd, sensors);

        time_sleep(0.01);
    }

    printf("Stopping...\n");

    stopHardware(pd, fd);

    return 0;
}

void sigHandler(int sig)
{
    (void)sig;
    running = 0;
}

void initHardware(int *pd, int *fd)
{
    *pd = pigpio_start(NULL, NULL);

    if (*pd < 0)
    {
        fprintf(stderr, "pigpio connection failed.\n");
        exit(EXIT_FAILURE);
    }

    *fd = i2c_open(*pd, PWMI2CCH, PWMI2CADR, 0);

    if (*fd < 0)
    {
        fprintf(stderr, "Failed to open I2C.\n");
        pigpio_stop(*pd);
        exit(EXIT_FAILURE);
    }

    /*
     * PCA9685 初期化
     * 100Hz程度に設定
     */
    i2c_write_byte_data(*pd, *fd, PWM_MODE1, 0x10);
    i2c_write_byte_data(*pd, *fd, PWM_PRESCALE, 61);
    i2c_write_byte_data(*pd, *fd, PWM_MODE1, 0x00);

    time_sleep(0.001);

    i2c_write_byte_data(*pd, *fd, PWM_MODE1, 0xA0);
    i2c_write_byte_data(*pd, *fd, PWM_MODE2, 0x04);

    set_mode(*pd, S1, PI_INPUT);
    set_mode(*pd, S2, PI_INPUT);
    set_mode(*pd, S3, PI_INPUT);
    set_mode(*pd, S4, PI_INPUT);
    set_mode(*pd, S5, PI_INPUT);

    printf("Init success.\n");
}

void stopHardware(int pd, int fd)
{
    motor_drive(pd, fd, 0, 0);

    i2c_close(pd, fd);
    pigpio_stop(pd);
}

uint8_t readSensorsAsBits(int pd)
{
    const int gpios[SENSOR_COUNT] = {S1, S2, S3, S4, S5};

    uint8_t sensors = 0x00;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        int value = gpio_read(pd, gpios[i]);

        if (value == LINE_DETECTED_VALUE)
        {
            sensors |= (uint8_t)(1U << i);
        }
    }

    return sensors;
}

void controlLineTracePD(int pd, int fd, uint8_t sensors)
{
    /*
     * 左から右へ -2, -1, 0, 1, 2
     */
    const int weights[SENSOR_COUNT] = {-2, -1, 0, 1, 2};

    int sum = 0;
    int count = 0;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if ((sensors & (uint8_t)(1U << i)) != 0)
        {
            sum += weights[i];
            count++;
        }
    }

    int left_speed;
    int right_speed;

    if (count > 0)
    {
        /*
         * 複数センサーが反応した場合は平均位置を使う
         */
        int error = sum / count;

        int derivative = error - previous_error;
        int turn = KP * error + KD * derivative;

        previous_error = error;
        last_error = error;

        /*
         * error > 0 ならラインが右寄り
         * 右へ曲がるため、左を速く・右を遅くする
         */
        left_speed = BASE_SPEED + turn;
        right_speed = BASE_SPEED - turn;
    }
    else
    {
        /*
         * ラインを見失った場合
         * 最後に見えていた方向へ旋回して探す
         */
        if (last_error < 0)
        {
            left_speed = -5;
            right_speed = 8;
        }
        else if (last_error > 0)
        {
            left_speed = 8;
            right_speed = -5;
        }
        else
        {
            left_speed = 4;
            right_speed = 4;
        }
    }

    left_speed = clamp(left_speed, MOTOR_MIN, MOTOR_MAX);
    right_speed = clamp(right_speed, MOTOR_MIN, MOTOR_MAX);

    motor_drive(pd, fd, left_speed, right_speed);
}

int clamp(int value, int min, int max)
{
    if (value < min)
    {
        return min;
    }

    if (value > max)
    {
        return max;
    }

    return value;
}

void set_pwm_output(int pd, int fd, int channel, int on_count, int off_count)
{
    int reg = PWM_0_ON_L + 4 * channel;

    on_count = clamp(on_count, 0, PWM_MAX_COUNT);
    off_count = clamp(off_count, 0, PWM_MAX_COUNT);

    i2c_write_byte_data(pd, fd, reg + 0, on_count & 0xFF);
    i2c_write_byte_data(pd, fd, reg + 1, (on_count >> 8) & 0x0F);
    i2c_write_byte_data(pd, fd, reg + 2, off_count & 0xFF);
    i2c_write_byte_data(pd, fd, reg + 3, (off_count >> 8) & 0x0F);
}

void set_pwm_full_on(int pd, int fd, int channel)
{
    int reg = PWM_0_ON_L + 4 * channel;

    i2c_write_byte_data(pd, fd, reg + 0, 0x00);
    i2c_write_byte_data(pd, fd, reg + 1, 0x10);
    i2c_write_byte_data(pd, fd, reg + 2, 0x00);
    i2c_write_byte_data(pd, fd, reg + 3, 0x00);
}

void set_pwm_full_off(int pd, int fd, int channel)
{
    int reg = PWM_0_ON_L + 4 * channel;

    i2c_write_byte_data(pd, fd, reg + 0, 0x00);
    i2c_write_byte_data(pd, fd, reg + 1, 0x00);
    i2c_write_byte_data(pd, fd, reg + 2, 0x00);
    i2c_write_byte_data(pd, fd, reg + 3, 0x10);
}

int motor_drive(int pd, int fd, int left_motor, int right_motor)
{
    left_motor = clamp(left_motor, MOTOR_MIN, MOTOR_MAX);
    right_motor = clamp(right_motor, MOTOR_MIN, MOTOR_MAX);

    int left_power = abs(left_motor);
    int right_power = abs(right_motor);

    int left_pwm = left_power * PWM_MAX_COUNT / MOTOR_MAX;
    int right_pwm = right_power * PWM_MAX_COUNT / MOTOR_MAX;

    /*
     * 左モーター方向制御
     */
    if (left_motor > 0)
    {
        set_pwm_full_on(pd, fd, IN3_PWM);
        set_pwm_full_off(pd, fd, IN4_PWM);
    }
    else if (left_motor < 0)
    {
        set_pwm_full_off(pd, fd, IN3_PWM);
        set_pwm_full_on(pd, fd, IN4_PWM);
    }
    else
    {
        set_pwm_full_off(pd, fd, IN3_PWM);
        set_pwm_full_off(pd, fd, IN4_PWM);
    }

    /*
     * 右モーター方向制御
     */
    if (right_motor > 0)
    {
        set_pwm_full_on(pd, fd, IN1_PWM);
        set_pwm_full_off(pd, fd, IN2_PWM);
    }
    else if (right_motor < 0)
    {
        set_pwm_full_off(pd, fd, IN1_PWM);
        set_pwm_full_on(pd, fd, IN2_PWM);
    }
    else
    {
        set_pwm_full_off(pd, fd, IN1_PWM);
        set_pwm_full_off(pd, fd, IN2_PWM);
    }

    /*
     * 速度PWM
     */
    if (left_motor == 0)
    {
        set_pwm_full_off(pd, fd, ENB_PWM);
    }
    else
    {
        set_pwm_output(pd, fd, ENB_PWM, 0, left_pwm);
    }

    if (right_motor == 0)
    {
        set_pwm_full_off(pd, fd, ENA_PWM);
    }
    else
    {
        set_pwm_output(pd, fd, ENA_PWM, 0, right_pwm);
    }

    return 0;
}