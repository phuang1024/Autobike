#pragma once


float mapf(float v, float old_min, float old_max, float new_min, float new_max) {
    return (v-old_min) / (old_max-old_min) * (new_max-new_min) + new_min;
}

float constrainf(float v, float min_v, float max_v) {
    return min(max(v, min_v), max_v);
}


// EMA
// factor 0 means no filtering, 1 means complete ema prev value
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
    const static int len = 10;
    float data[len];
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
        int index = head - i;
        if (index < 0) {
            index += len;
        }
        return data[index];
    }
};


// Taylor series prediction
// TODO currently hardcoded 3rd order (or less).
struct Predictor {
    LoopQueue data;

    Predictor() {
    }

    void update(float v) {
        data.push(v);
    }

    float predict(int steps) {
        const float dt = 1;

        const int stretch = 4;
        const float
            a = data.get(0),
            b = data.get(stretch),
            c = data.get(2*stretch),
            d = data.get(3*stretch);

        float val = a;
        float deriv = (a - b) / dt;
        //float deriv2 = (a - 2*b + c) / (dt*dt);
        //float deriv3 = (a - 3*b + 3*c - d) / (dt*dt*dt);

        for (int i = 0; i < steps; i++) {
            //deriv2 += deriv3 * dt;
            //deriv += deriv2 * dt;
            val += deriv * dt;
        }

        return val;
    }
};


// simple timer for code profiling.
struct Timer {
    long start;

    Timer() {
        start = millis();
    }

    long elapsed() {
        return millis() - start;
    }

    void reset() {
        start = millis();
    }
};
