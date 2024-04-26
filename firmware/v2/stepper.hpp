#pragma once

#include "./utils.hpp"


// use stepper motor with a servo like api.
class StepperServo {
public:
    StepperServo() {
        // ena, dir, step
        pinMode(3, OUTPUT);
        pinMode(4, OUTPUT);
        pinMode(5, OUTPUT);

        position = 0;
        velocity = 0;
        direction = false;
        enabled = true;

        set_dir(false);
        set_enable(true);
    }

    void set_enable(bool ena) {
        digitalWrite(3, (ena ? LOW : HIGH));
        enabled = ena;
    }

    // turn to degrees position, returning after working for max_time ms.
    // if job is completed, will delay for the remaining time.
    void turn_to(float target_deg, long max_time) {
        if (!enabled) {
            return;
        }

        const unsigned long time_start = millis();

        target_deg = constrainf(target_deg, -MAX_POS, MAX_POS);
        long target = degrees_to_steps(target_deg);

        int i = ST_UPDATE_INTERVAL;
        // value set in first iteration
        long step_time;
        long delta;
        while (millis() - time_start < max_time) {
            if (i >= ST_UPDATE_INTERVAL) {
                i = 0;

                delta = abs(target - position);

                // calculate magnitude of target velocity
                float target_vel;
                if (delta > DECEL_BEGIN) {
                    target_vel = 1;
                } else {
                    target_vel = (float)delta / DECEL_BEGIN;
                }
                target_vel = constrainf(target_vel, 0, 1);

                // check direction
                if (target < position) {
                    target_vel = -target_vel;
                }

                // update velocity via weighted average
                velocity = velocity * (1 - ACCEL) + target_vel * ACCEL;

                // update step time
                step_time = vel_to_st(velocity);
            }

            i++;
            if (delta >= 20) {
                do_step(velocity > 0);
            }
            delayMicroseconds(step_time);
        }
    }

private:
    // steps per revolution
    const int SPR = 400 * 50.9;
    // max steps possible in both directions
    const int MAX_POS = degrees_to_steps(45);
    // min and max step time (us)
    const int MIN_ST = 100;
    const int MAX_ST = 1500;
    // start decelerating when this many steps left.
    const int DECEL_BEGIN = degrees_to_steps(5);
    // weighted average factor
    const float ACCEL = 0.2;
    // recalculate step time every x steps (as opposed to every step) for performance.
    const int ST_UPDATE_INTERVAL = 10;

    // steps position
    long position;
    // velocity; -1 to 1.
    float velocity;
    bool direction;
    bool enabled;

    float steps_to_degrees(long steps) {
        return (float)steps * 360 / SPR;
    }

    long degrees_to_steps(float degrees) {
        return degrees * SPR / 360;
    }

    // convert abs(velocity) to step time; i.e. always returns positive number.
    // 0 to 1 -> MAX_ST to MIN_ST
    int vel_to_st(float vel) {
        return (int)mapf(constrainf(fabs(vel), 0, 1), 0, 1, MAX_ST, MIN_ST);
    }

    void set_dir(bool dir) {
        digitalWrite(4, (dir ? LOW : HIGH));
        this->direction = dir;
    }

    void do_step(bool dir) {
        if (dir && position >= MAX_POS || !dir && position <= -MAX_POS) {
            velocity = 0;
            return;
        }
        set_dir(dir);
        digitalWrite(5, HIGH);
        digitalWrite(5, LOW);
        position += dir ? 1 : -1;
    }
};
