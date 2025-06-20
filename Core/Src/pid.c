/*
 * pid.c
 *
 *  Created on: Jun 20, 2025
 *      Author: user
 */


#include "pid.h"
#include <math.h>

void pid_init(PID *pid, float kp, float ki, float kd, float windup_limit, float derivative_threshold) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->windup_limit = windup_limit;
    pid->derivative_threshold = derivative_threshold;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

float pid_advance(PID *pid, float error, float dt) {
    float derivative = 0.0f;

	// Compute derivative with threshold filter
	float d_error = (error - pid->prev_error) / dt;
	if (fabsf(error - pid->prev_error) > pid->derivative_threshold) {
		derivative = 0.0f;
	} else {
		derivative = d_error;
	}

	// Accumulate integral
	pid->integral += error * dt;

	// Clamp integral (anti-windup)
	if (pid->integral > pid->windup_limit) {
		pid->integral = pid->windup_limit;
	} else if (pid->integral < -pid->windup_limit) {
		pid->integral = -pid->windup_limit;
	}


    pid->prev_error = error;

    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return pid->output;
}
