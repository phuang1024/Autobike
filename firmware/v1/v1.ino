#include <IBusBM.h>

#include "./imu.h"
#include "./stepper.h"


IBusBM ibus;
StepperServo steering;
PIDControl pid_balance(0, 0, 0, 1, 0.99);

void setup() {
    Serial.begin(9600);

    imu_init();

    // receiving on RX1 (pin 19 on Mega)
    ibus.begin(Serial1);

    while (true) {
        uint16_t val = ibus.readChannel(0);
        long position = map(val, 1000, 2000, -45, 45);
        steering.turn_to(position, 50);
    }
}
