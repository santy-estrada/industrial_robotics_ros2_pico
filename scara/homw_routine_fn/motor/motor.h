#ifndef MOTOR_H
#define MOTOR_H

//Initialize motor with 2 pins
void init_motor(uint pinA1, uint pinA2);

//Set motor to counter clockwise (positive) movement
void mv_ccw(const uint pins[3]);

//Set motor to clockwise (negative) movement
void mv_cw(const uint pins[3]);

//Stop motor direction pins
void stop_motor(const uint pins[3]);

//Resume the motor movement
void resume_motor(const uint pins[3]);

//Toggle motor direction
void toggle_dir(const uint pins[3]);

#endif
