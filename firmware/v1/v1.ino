struct StepperDelta {
    long steps;
    bool dir;
};

// pin 3, 4, 5: ena, dir, pul
class Stepper {
public:
    const long SPR = 400 * 19.19;
    // Max rotation in each direction.
    const long MAX_STEPS = degrees_to_steps(45);
    const long MIN_STEP_TIME = 500;

    Stepper() {
        pinMode(3, OUTPUT);
        pinMode(4, OUTPUT);
        pinMode(5, OUTPUT);

        position = 0;
    }

    // TODO maybe use float?
    long steps_to_degrees(long steps) {
        return steps * 360L / SPR;
    }

    long degrees_to_steps(long degrees) {
        return degrees * SPR / 360L;
    }

    void set_enable(bool ena) {
        digitalWrite(3, (ena ? LOW : HIGH));
    }

    void set_dir(bool dir) {
        digitalWrite(4, (dir ? HIGH : LOW));
    }

    // step_time: us
    void rotate_for(long steps, bool dir, long step_time) {
        step_time = max(step_time, MIN_STEP_TIME);

        set_enable(true);
        set_dir(dir);

        for (long i = 0; i < steps; i++) {
            digitalWrite(5, HIGH);
            digitalWrite(5, LOW);
            delayMicroseconds(step_time);
        }

        if (dir) {
            position += steps;
        } else {
            position -= steps;
        }
    }

    void rotate_for(StepperDelta job, long step_time) {
        rotate_for(job.steps, job.dir, step_time);
    }

    // target: degrees
    // will take into account MAX_STEPS
    StepperDelta compute_rotate_to(long target) {
        target = degrees_to_steps(target);
        target = constrain(target, -MAX_STEPS, MAX_STEPS);
        bool dir = target > position;
        long delta = abs(target - position);
        StepperDelta ret;
        ret.dir = dir;
        ret.steps = delta;
        return ret;
    }

    void rotate_to(long target, long step_time) {
        StepperDelta job = compute_rotate_to(target);
        rotate_for(job, step_time);
    }

    // rotate_to, but limited to a max working time.
    // max_time: ms
    void rotate_to_limited(long target, long step_time, long max_time) {
        const long max_steps = 1000L * max_time / step_time;
        StepperDelta job = compute_rotate_to(target);
        if (job.steps > max_steps) {
            job.steps = max_steps;
        } else {
            if (job.steps > 0) {
                step_time = 1000L * max_time / job.steps;
            }
        }
        rotate_for(job, step_time);
    }

private:
    long position;
};


Stepper steering;


void setup() {
}

void loop() {
    int ctrl = pulseIn(8, HIGH);
    ctrl = constrain(ctrl, 1000, 2000) / 25 * 25;
    long position = map(ctrl, 1000, 2000, -45, 45);
    steering.rotate_to_limited(position, steering.MIN_STEP_TIME, 30);
}
