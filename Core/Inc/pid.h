/*
 * pid.h
 *
 *  Created on: Jun 20, 2025
 *      Author: user
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output;
    float windup_limit;
    float derivative_threshold;
} PID;

void pid_init(PID *pid, float kp, float ki, float kd, float windup_limit, float derivative_threshold);
float pid_advance(PID *pid, float error, float dt);


#endif /* INC_PID_H_ */
