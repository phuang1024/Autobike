#include <Wire.h>

const int GYRO_INT_FAC = 20;
const float GYRO_INT_DECAY = .998;
const float A_EMA_FAC = .99;
const float G_EMA_FAC = .99;

const int I2C_IMU = 0x68;


// factor 0 means no filtering, 1 means no change
struct EMA {
    float fac;
    float val;

    EMA(float factor) {
        fac = factor;
        val = 0;
    }

    float update(float v) {
        val = val * fac + v * (1-fac);
        return val;
    }
};


struct IMURead {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
};


void imu_init() {
    Wire.beginTransmission(I2C_IMU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
}


int16_t read_u16() {
    return (int16_t)((Wire.read() << 8) | Wire.read());
}


IMURead imu_read() {
    IMURead res;

    // Request data
    Wire.beginTransmission(I2C_IMU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_IMU, 14, true);

    // Get data
    res.ax = read_u16();
    res.ay = read_u16();
    res.az = read_u16();
    res.temp = read_u16();
    res.gx = read_u16();
    res.gy = read_u16();
    res.gz = read_u16();

    return res;
}


void print_imu(IMURead& imu) {
    Serial.print(imu.ax); Serial.print(' ');
    //Serial.print(imu.ay); Serial.print(' ');
    //Serial.print(imu.az); Serial.print(' ');
    Serial.print(imu.gx); Serial.print(' ');
    //Serial.print(imu.gy); Serial.print(' ');
    //Serial.print(imu.gz); Serial.print(' ');
    //Serial.print(imu.temp); Serial.print(' ');
    Serial.println();
}


void setup() {
    Serial.begin(115200);
    Wire.begin();

    imu_init();

    // We want low_pass(accel) + high_pass(gyro)
    // low_pass is EMA
    // high_pass is value - EMA

    int gyro_int = 0;
    EMA accel_ema(A_EMA_FAC);
    EMA gyro_ema(G_EMA_FAC);

    for (int i = 0; ; i++) {
        IMURead imu = imu_read();
        gyro_int += imu.gx / GYRO_INT_FAC;
        gyro_int *= GYRO_INT_DECAY;

        accel_ema.update(imu.ax);
        gyro_ema.update(gyro_int);

        int filtered = accel_ema.val + (gyro_int - gyro_ema.val);

        if (i % 10 == 0) {
            Serial.print(imu.ax); Serial.print(' ');
            Serial.print(gyro_int); Serial.print(' ');
            Serial.print(filtered); Serial.print(' ');
            Serial.println();
        }
    }
}

void loop() {
}
