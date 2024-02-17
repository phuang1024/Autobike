#pragma once


float mapf(float v, float old_min, float old_max, float new_min, float new_max) {
    return (v-old_min) / (old_max-old_min) * (new_max-new_min) + new_min;
}

float constrainf(float v, float min_v, float max_v) {
    return min(max(v, min_v), max_v);
}


// EMA
// factor 0 means no filtering, 1 means no change
struct Averager {
    float fac;
    float val;

    Averager(float factor) {
        fac = factor;
        val = 0;
    }

    float update(float v) {
        val = val * fac + v * (1-fac);
        return val;
    }
};


// memory and compute efficient queue implementation.
// TODO hardcoded max length.
struct LoopQueue {
    const int len = 10;
    const float data[len];
    int head;

    LoopQueue() {
        head = 0;
        for (int i = 0; i < len; i++) {
            data[i] = 0;
        }
    }

    void push(float v) {
        head = (head + 1) % len;
        data[head] = v;
    }

    // i = 0 is latest element; i = 1 is the one before that, etc.
    float get(int i) {
        return data[(head - i) % len];
    }
};


// Taylor series prediction
// TODO currently hardcoded 3rd order
struct Predictor {
    LoopQueue data;

    Predictor() {
    }

    void update(float v) {
        data.push(v);
    }

    float predict(int steps) {
        const float dt = 1;

        const float
            a = data.get(0),
            b = data.get(1),
            c = data.get(2),
            d = data.get(3);

        float val = a;
        float deriv = (a - b) / dt;
        float deriv2 = (a - 2*b + c) / (dt*dt);
        float deriv3 = (a - 3*b + 3*c - d) / (dt*dt*dt);

        for (int i = 0; i < steps; i++) {
            val += deriv * dt;
            deriv += deriv2 * dt;
            deriv2 += deriv3 * dt;
        }

        return val;
    }
};