#pragma once


class PIDControl {
public:
    float kp, ki, kd;
    float integral_clamp, integral_decay;
    float curr_integral;
    float last_error;

    PIDControl(float kp, float ki, float kd, float integral_clamp, float integral_decay) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;

        this->integral_clamp = integral_clamp;
        this->integral_decay = integral_decay;

        curr_integral = 0;
        last_error = 0;
    }

    float update(float error, float dt) {
        curr_integral += ki * error * dt;
        curr_integral = constrain(curr_integral, -integral_clamp, integral_clamp);
        curr_integral *= integral_decay;

        float derivative = (error - last_error) / dt;
        last_error = error;

        return kp * error + curr_integral + kd * derivative;
    }
};
