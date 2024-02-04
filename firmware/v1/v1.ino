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
    const long MIN_STEP_TIME = 700;

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

private:
    long position;
};


Stepper steering;


void setup() {
}

void loop() {
    int ctrl = pulseIn(8, HIGH);
    long position = map(constrain(ctrl, 1000, 2000), 1000, 2000, -45, 45);
    StepperDelta job = steering.compute_rotate_to(position);

    // slower for low magnitude rotations.
    long saturate_steps = 300;
    long slowest = 2000;
    long fastest = 600;
    long step_time;
    if (job.steps > saturate_steps) {
        step_time = fastest;
    } else {
        step_time = map(job.steps, 0, saturate_steps, slowest, fastest);
    }

    if (job.steps > 20) {
        job.steps = min(job.steps, 300);
        steering.rotate_for(job, step_time);
    }
}
