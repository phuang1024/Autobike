#include <IBusBM.h>

#include "./imu.hpp"
#include "./pid.hpp"
#include "./stepper.hpp"
#include "./utils.hpp"


// experimentally determined
const float IMU_AX_CENTER = 0.030846428571428573;

IBusBM ibus;
StepperServo steering;
PIDControl pid_balance(5, 0.1, 2000, .2, 1);


void initalize() {
    delay(100);

    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < 4; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }

    Serial.begin(115200);
    Wire.begin();
    // receiving on RX1 (pin 19 on Mega)
    ibus.begin(Serial1, 1);

    delay(1000);
    imu_init();
    delay(1000);

    digitalWrite(LED_BUILTIN, HIGH);
}


void main_loop() {
    // stabilized via moving average
    float ax = 0, gx = 0;
    float imu_avg = 0.7;

    const int loop_interval = 30;
    while (true) {
        // read RC
        uint16_t rx_steering = ibus.readChannel(0);
        uint16_t rx_throttle = ibus.readChannel(2);
        uint16_t rx_enable = ibus.readChannel(4);

        // read IMU
        IMURead imu = imu_read_avg(3, 1000);
        ax = ax * imu_avg + (imu.ax - IMU_AX_CENTER) * (1 - imu_avg);
        gx = gx * imu_avg + imu.gx * (1 - imu_avg);

        // update steering enable
        steering.set_enable(rx_enable > 1500);

        float error = ax;
        float ctrl = pid_balance.update(ax, loop_interval);
        Serial.println(ctrl);
        long steering_pos = (long)mapf(ctrl, -1, 1, -45, 45);
        steering.turn_to(steering_pos, loop_interval);

        // tmp steering test
        //long steering_pos = (long)mapf(ax, -0.1, 0.1, -45, 45);
        //steering.turn_to(steering_pos, loop_interval);
    }
}


// rc control steering
void test_steering() {
    while (true) {
        uint16_t rx_steering = ibus.readChannel(0);
        uint16_t rx_enable = ibus.readChannel(4);

        steering.set_enable(rx_enable > 1500);
        long steering_pos = map(rx_steering, 1000, 2000, -45, 45);
        steering.turn_to(steering_pos, 30);
    }
}


// print: ax EMA_ax
void test_imu() {
    Averager ax_avg(0.8);
    Predictor ax_pred;

    while (true) {
        IMURead imu = imu_read_avg(10, 100);

        ax_avg.update(imu.ax);
        ax_pred.update(ax_avg.val);
        float ax_pred_val = ax_pred.predict(3);

        Serial.print(imu.ax, 6);
        Serial.print(' ');
        Serial.print(ax_avg.val, 6);
        Serial.print(' ');
        Serial.print(ax_pred_val, 6);
        Serial.print(' ');
        Serial.println();

        delay(50);
    }
}


void setup() {
    initalize();

    test_imu();
    //test_steering();

    main_loop();
}


void loop() {
}
