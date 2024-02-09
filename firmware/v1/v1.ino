#include <IBusBM.h>


// use stepper motor with a servo like api.
class StepperServo {
public:
    StepperServo() {
        pos = 0;
        last_st = MAX_ST;
    }

    // turn to degrees position, returning after working for max_time ms.
    void turn_to(int target, long max_time) {
        const unsigned long time_start = millis();

        target = constrain(target, -MAX_POS, MAX_POS);
        target = degrees_to_steps(target);

        while (millis() - time_start < max_time) {
            const long delta = abs(target - pos);
            if (abs(target - pos) < 50) {
                return;
            }

            long target_st;
            if (delta > DECEL_BEGIN) {
                target_st = MIN_ST;
            } else {
                target_st = map(delta, 0, DECEL_BEGIN, MAX_ST, MIN_ST);
            }

            target_st = constrain(target_st, MIN_ST, MAX_ST);
            if (target < pos) {
                target_st = -target_st;
            }

            last_st = last_st * (1 - ACCEL) + target_st * ACCEL;
            do_step(last_st > 0);
            delayMicroseconds(abs(last_st));

            /*
            Serial.print(pos);
            Serial.print(' ');
            Serial.print(delta);
            Serial.print(' ');
            Serial.print(target_st);
            Serial.println();
            delay(100);
            */
        }
    }

private:
    const int SPR = 800 * 19.19;
    // degrees, plus/minus
    const int MAX_POS = degrees_to_steps(45);
    const int MIN_ST = 300;
    const int MAX_ST = 2000;
    // start decelerating when this many steps left.
    const int DECEL_BEGIN = 300;
    // weighted average factor
    const float ACCEL = 0.1;

    // steps
    long pos;
    // negative means rotating in neg dir
    long last_st;

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

    void do_step(bool dir) {
        if (dir && pos >= MAX_POS || !dir && pos <= -MAX_POS) {
            return;
        }
        set_dir(dir);
        digitalWrite(5, HIGH);
        digitalWrite(5, LOW);
        pos += dir ? 1 : -1;
    }
};


IBusBM ibus;
StepperServo steering;

void setup() {
    Serial.begin(9600);

    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);

    // receiving on RX1 (pin 19 on Mega)
    ibus.begin(Serial1);
}

void loop() {
    uint16_t val = ibus.readChannel(0);
    long position = map(val, 1000, 2000, -45, 45);
    steering.turn_to(position, 50);
}