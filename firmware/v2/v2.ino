#include <IBusBM.h>

#include "./imu.hpp"
#include "./pid.hpp"
#include "./stepper.hpp"
#include "./utils.hpp"

IBusBM ibus;
StepperServo steering;


void initalize() {
    // drive motor pins
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    digitalWrite(9, LOW);

    delay(100);

    Serial.begin(115200);
    // receiving on RX1 (pin 19 on Mega)
    ibus.begin(Serial1, 1);
}


void main_loop() {
    const int loop_interval = 50;
    while (true) {
        // read RC
        uint16_t rx_steering = ibus.readChannel(0);
        uint16_t rx_throttle = ibus.readChannel(2);
        uint16_t rx_enable = ibus.readChannel(4);

        // update steering
        steering.set_enable(rx_enable > 1500);

        long steering_pos = (long)mapf(ctrl, -1, 1, -45, 45);
        steering.turn_to(steering_pos, loop_interval);

        // update drive motor
        digitalWrite(10, rx_enable > 1500);
        analogWrite(9, map(rx_throttle, 1000, 2000, 0, 255));
    }
}


void setup() {
    initalize();

    main_loop();
}


void loop() {
}
