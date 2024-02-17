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
struct Predictor {

};
