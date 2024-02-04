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

  // target: degrees
  // max_steps: Stop if we reach this value.
  void rotate_to(long target, long step_time, long max_steps = 0) {
    target = degrees_to_steps(target);
    dir = target > position;
    long delta = abs(target - position);
    if (max_steps > 0) {
      delta = min(delta, max_steps);
    }
    do_steps(delta, dir, step_time);
  }

  // TODO maybe use float?
  long steps_to_degrees(long steps) {
    return steps * 360L / spr;
  }

  long degrees_to_steps(long degrees) {
    return degrees * spr / 360L;
  }

private:
  const long spr = 200 * 19.19;
  const long min_step_time = 1000;
  long position;
};


Stepper steering;


void setup() {
}

void loop() {
  int ctrl = pulseIn(10, HIGH);
  long position = map(constrain(ctrl, 1000, 2000), 1000, 2000, -45, 45);
  steering.rotate_to(position, 5000, 100);
}