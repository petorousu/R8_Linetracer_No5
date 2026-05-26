//
// Created by stellarcielo on 2026/05/12.
//

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <stdint.h>
#include <pigpiod_if2.h>

#define S1 5
#define S2 6
#define S3 13
#define S4 19
#define S5 26

// PWM ユニットの I2C アドレス
// i2cdetect で確認可能、違っていたら修正して下さい
#define PWMI2CADR 0x40
// PWM ユニットが接続されている I2C のチャネル番号
#define PWMI2CCH 1
// モータードライバの各入力が接続されている PWM ユニットのチャネル番号
// 右側のモーター：パワーユニットの K1 または K2 に接続（説明書は誤り）
// ENA は PWM 駆動に使う（1 でブリッジ動作、0 はブリッジオフ）
// IN1 と IN2 は右車輪の回転方向を決める（後進：0,1、前進：1,0）（0,0 と 1,1 はブレーキ）
#define ENA_PWM 8
#define IN1_PWM 9
#define IN2_PWM 10
// 左側のモーター：パワーユニットの K3 または K4 に接続（説明書は誤り）
// ENB は PWM 駆動に使う（1 でブリッジ動作、0 はブリッジオフ）
// IN3 と IN4 は左車輪の回転方向を決める（後進：0,1、前進：1,0）（0,0 と 1,1 はブレーキ）
#define ENB_PWM 13
#define IN3_PWM 11
#define IN4_PWM 12
// PWM モジュールのレジスタ番号
#define PWM_MODE1 0
#define PWM_MODE2 1
#define PWM_SUBADR1 2
#define PWM_SUBADR2 3
#define PWM_SUBADR3 4
#define PWM_ALLCALL 5
// PWM 番号×4+PWM_0_??_? でレジスタ番号は求まる
#define PWM_0_ON_L 6
#define PWM_0_ON_H 7
#define PWM_0_OFF_L 8
#define PWM_0_OFF_H 9
// PWM 出力定数
#define PWMFULLON 16
#define PWMFULLOFF 0
// プリスケーラのレジスタ番号
// PWM 周波数を決めるレジスタ番号、100Hz なら 61 をセット
#define PWM_PRESCALE 254

#define SENSOR_COUNT 5

#define MOTOR_MAX 16
#define MOTOR_MIN -16

#define BASE_SPEED 8
#define KP 4
#define KD 2

void initHard(int *pd, int *fd);
void sigHandler(int sig);
int motor_drive(int pd, int fd, int lm, int rm);
uint8_t readAllSensors(int pd, int gpios[]);

volatile sig_atomic_t running = 1;

static int last_error = 0;
static int previous_error = 0;

int clamp(int value, int min, int max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void controlLineTracePD(int pd, int fd, uint8_t sensors)
{
    int weights[SENSOR_COUNT] = {-2, -1, 0, 1, 2};

    int sum = 0;
    int count = 0;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if ((sensors & (1 << i)) != 0)
        {
            sum += weights[i];
            count++;
        }
    }

    int left_speed;
    int right_speed;

    if (count > 0)
    {
        int error = sum / count;
        int derivative = error - previous_error;

        int turn = KP * error + KD * derivative;

        previous_error = error;
        last_error = error;

        left_speed = BASE_SPEED + turn;
        right_speed = BASE_SPEED - turn;
    }
    else
    {
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

int main(void){
    int pd, fd;
    int gpios[] = {S1, S2, S3, S4, S5};
    //1バイトの変数のためcharを使っています。5つのセンサーをまとめてビット列で管理するためです.

    signal(SIGINT, sigHandler);
    signal(SIGTERM, sigHandler);

    initHard(&pd, &fd);
    motor_drive(pd, fd, 0, 0);
    //printf("reset");

    while (running){
        uint8_t sensors = readAllSensors(pd, gpios);

        controlLineTracePD(pd, fd, sensors);

        time_sleep(0.01);
    }

    printf("Stopping...\n\n");
    motor_drive(pd, fd, 0, 0);
    pigpio_stop(pd);
    return 0;
}

void sigHandler(int sig){
    running = 0;
}

void initHard(int *pd, int *fd){
    if ((*pd = pigpio_start(NULL, NULL)) < 0)
    {
        fprintf(stderr, "pigpio connection failed.\n");
        fprintf(stderr, "pigpio check start.\n");
        exit(EXIT_FAILURE);
    }
    // I2C 接続と PWM ユニットの初期化
    *fd = i2c_open(*pd,PWMI2CCH,PWMI2CADR, 0);
    if (*fd < 0)
    {
        fprintf(stderr, "Failed to init I2C.\n");
        exit(EXIT_FAILURE);
    }
    i2c_write_byte_data(*pd, *fd,PWM_PRESCALE, 61); //PWM 周期 10ms に設定
    printf("PWM 周期は 10ms です。値の更新間隔に注意して下さい。\n");
    i2c_write_byte_data(*pd, *fd,PWM_MODE1, 0x10); //SLEEP mode
    i2c_write_byte_data(*pd, *fd,PWM_MODE1, 0); //normal mode
    time_sleep(0.001); // wait for stabilizing internal oscillator
    i2c_write_byte_data(*pd, *fd,PWM_MODE1, 0xA0); //Restart all PWM ch

    set_mode(*pd, S1, PI_INPUT);
    set_mode(*pd, S2, PI_INPUT);
    set_mode(*pd, S3, PI_INPUT);
    set_mode(*pd, S4, PI_INPUT);
    set_mode(*pd, S5, PI_INPUT);

    printf("Init success.\n");
}

uint8_t readAllSensors(int pd, int gpios[]){
    //printf("r");
    uint8_t sensors = 0x00;
    for (int i = 0; i < 5; i++)
    {
        sensors += ((uint8_t)gpio_read(pd, gpios[i]) & 0x01) << i;
    }

    return sensors;
}