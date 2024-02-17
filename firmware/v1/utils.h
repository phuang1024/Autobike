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


// Taylor series prediction
// TODO currently hardcoded 3rd order
struct Predictor {
    float a, b, c, d;

    Predictor() {
        a = 0;
        b = 0;
        c = 0;
        d = 0;
    }

    void update(float v) {
        d = c;
        c = b;
        b = a;
        a = v;
    }

    float predict(int steps) {
        const float dt = 1;

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
