#pragma once

#include <Wire.h>

const int I2C_IMU = 0x68;


struct IMURead {
    // Absolute position (accelerometer).
    float ax, ay, az;
    // Rotational velocity (gyroscope).
    float gx, gy, gz;
    uint16_t temp;
};


void imu_init() {
    Wire.beginTransmission(I2C_IMU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
}


uint16_t read_u16() {
    return (Wire.read() << 8) | Wire.read();
}


/**
 * Converts to signed 16bit, then divide by 32768
 * Out range = -1 to 1
 */
float nrm_u16(uint16_t v) {
    return (float)(int16_t(v)) / 32768.0f;
}

IMURead imu_read() {
    IMURead res;

    // Request data
    Wire.beginTransmission(I2C_IMU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_IMU, 14, true);

    // Get data
    res.ax = nrm_u16(read_u16());
    res.ay = nrm_u16(read_u16());
    res.az = nrm_u16(read_u16());
    res.temp = read_u16();
    res.gx = nrm_u16(read_u16());
    res.gy = nrm_u16(read_u16());
    res.gz = nrm_u16(read_u16());

    return res;
}

// count: Number of readings to average.
// delay: Delay between readings in us.
IMURead imu_read_avg(int count, int delay) {
    IMURead res = {0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < count; i++) {
        IMURead r = imu_read();
        res.ax += r.ax;
        res.ay += r.ay;
        res.az += r.az;
        res.gx += r.gx;
        res.gy += r.gy;
        res.gz += r.gz;
        res.temp += r.temp;
        delayMicroseconds(delay);
    }
    res.ax /= count;
    res.ay /= count;
    res.az /= count;
    res.gx /= count;
    res.gy /= count;
    res.gz /= count;
    res.temp /= count;
    return res;
}

void print_imu(IMURead& imu) {
    Serial.print(imu.ax); Serial.print(' ');
    Serial.print(imu.ay); Serial.print(' ');
    Serial.print(imu.az); Serial.print(' ');
    Serial.print(imu.gx); Serial.print(' ');
    Serial.print(imu.gy); Serial.print(' ');
    Serial.print(imu.gz); Serial.print(' ');
    Serial.print(imu.temp); Serial.print(' ');
    Serial.println();
}
