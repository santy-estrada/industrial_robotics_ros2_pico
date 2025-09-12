#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

static bool dirs[2] = {0, 0};

void init_motor(uint pinA1, uint pinA2){
    gpio_init(pinA1);
    gpio_set_dir(pinA1, GPIO_OUT);
    gpio_init(pinA2);
    gpio_set_dir(pinA2, GPIO_OUT);
}

void mv_ccw(const uint pins[3]){
    gpio_put(pins[0], 0);
    gpio_put(pins[1], 1);

    dirs[pins[2]] = 1;
}

void mv_cw(const uint pins[3]){
    gpio_put(pins[0], 1);
    gpio_put(pins[1], 0);

    dirs[pins[2]] = 0;
}

void stop_motor(const uint pins[3]){
    gpio_put(pins[0], 0);
    gpio_put(pins[1], 0);
}

void resume_motor(const uint pins[3]){
    if (dirs[pins[2]] == 0) {
        mv_cw(pins);
    } else {
        mv_ccw(pins);
    }
}

void toggle_dir(const uint pins[3]){
    dirs[pins[2]] = !dirs[pins[2]];
    if (dirs[pins[2]]) {
        mv_ccw(pins);
    } else {
        mv_cw(pins);
    }
}