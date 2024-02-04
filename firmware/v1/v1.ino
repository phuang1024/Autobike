struct StepperDelta {
  long steps;
  bool dir;
};


// pin 3, 4, 5: ena, dir, pul
class Stepper {
public:
  Stepper() {
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);

    position = 0;
  }

  // step_time: us
  void do_steps(long steps, bool dir, long step_time) {
    digitalWrite(4, (dir ? HIGH : LOW));
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

  void do_steps(StepperDelta job, long step_time) {
    do_steps(job.steps, job.dir, step_time);
  }

  // target: degrees
  StepperDelta compute_rotate_to(long target) {
    target = degrees_to_steps(target);
    bool dir = target > position;
    long delta = abs(target - position);
    StepperDelta ret;
    ret.dir = dir;
    ret.steps = delta;
    return ret;
  }

  // TODO maybe use float?
  long steps_to_degrees(long steps) {
    return steps * 360L / spr;
  }

  long degrees_to_steps(long degrees) {
    return degrees * spr / 360L;
  }

private:
  // TODO this is wrong
  const long spr = 200 * 19.19;
  long position;
};


Stepper steering;


void setup() {
}

void loop() {
  int ctrl = pulseIn(8, HIGH);
  long position = map(constrain(ctrl, 1000, 2000), 1000, 2000, -90, 90);
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
    steering.do_steps(job, step_time);
  }
}