// include/FilterUtils.h

#pragma once

/**
 * @brief Applies a low-pass filter to the input data.
 *
 * This function applies an exponential moving average to smooth the input data.
 * The degree of filtering is controlled by the alpha parameter. Higher alpha values
 * retain more of the input data, while lower values provide more smoothing.
 *
 * @param current_value The latest data point to be filtered.
 * @param previous_filtered_value The previous filtered data point.
 * @param alpha The smoothing factor, between 0 and 1.
 * @return The filtered data point.
 */
inline float lowPassFilter(float current_value, float previous_filtered_value, float alpha) {
    return alpha * current_value + (1.0f - alpha) * previous_filtered_value;
}
