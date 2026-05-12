//
// Created by stellarcielo on 2026/05/12.
//

#include <stdio.h>
#include <stdlib.h>
#include <pigpiod_if2.h>

#define S1 5
#define S2 6
#define S3 13
#define S4 19
#define S5 26

void readAllSensors(int sensors[]);

int main(void){
    int pd;
    int sensors[5];

    if ((pd = pigpio_start(NULL, NULL)) < 0)
    {
        fprintf(stderr, "pigpio connection failed.\n");
        fprintf(stderr, "pigpio check start.\n");
        exit(EXIT_FAILURE);
    }

    set_mode(pd, S1, PI_INPUT);
    set_mode(pd, S2, PI_INPUT);
    set_mode(pd, S3, PI_INPUT);
    set_mode(pd, S4, PI_INPUT);
    set_mode(pd, S5, PI_INPUT);

    for (i = 0; i < 10; i++){
        readAllSensors(sensors);
        for (int j = 0; j < 5; j++) printf("%dsensor = %d", j, sensors[j]);
        time_sleep(1.0);
    }

    return 0;
}