#include <IBusBM.h>

#include "./imu.h"
#include "./pid.h"
#include "./stepper.h"
#include "./utils.h"


IBusBM ibus;
StepperServo steering;
PIDControl pid_balance(0, 0, 0, 1, 0.99);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    // receiving on RX1 (pin 19 on Mega)
    ibus.begin(Serial1, 1);

    delay(2000);
    imu_init();
    delay(2000);

    // stabilized via moving average
    float ax = 0, gx = 0;
    float imu_avg = 0.7;

    // loop interval: 50ms
    // read remote: every loop
    // update pid: every loop
    const int loop_interval = 50;
    while (true) {
        // read RC
        uint16_t rx_steering = ibus.readChannel(0);
        uint16_t rx_throttle = ibus.readChannel(2);
        uint16_t rx_enable = ibus.readChannel(4);

        // read IMU
        IMURead imu = imu_read_avg(5, 10);
        ax = ax * imu_avg + imu.ax * (1 - imu_avg);
        gx = gx * imu_avg + imu.gx * (1 - imu_avg);

        // update steering enable
        steering.set_enable(rx_enable > 1500);

        // tmp steering test
        //long steering_pos = map(rx_steering, 1000, 2000, -45, 45);
        long steering_pos = (long)mapf(ax, -0.1, 0.1, -45, 45);
        steering.turn_to(steering_pos, loop_interval);
    }
}
