#pragma once

inline void scale_input(
    float* data,
    int length,
    const float* mean,
    const float* std,
    float eps = 1e-6f
) {
    for (int i = 0; i < length; i++) {
        data[i] = (data[i] - mean[i]) / (std[i] + eps);
    }
}
