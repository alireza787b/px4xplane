// include/FilterUtils.h

#pragma once
#include <deque>
#include <algorithm>


/**
 * @brief Applies a low-pass filter to the input data.
 *
 * This function applies an exponential moving average to smooth the input data.
 * The degree of filtering is controlled by the alpha parameter.
 *
 * @param current_value The latest data point to be filtered.
 * @param previous_filtered_value The previous filtered data point from the window.
 * @param alpha The smoothing factor, between 0 and 1.
 * @return The filtered data point.
 */
inline float lowPassFilter(float current_value, float previous_filtered_value, float alpha) {
    return alpha * current_value + (1.0f - alpha) * previous_filtered_value;
}

/**
 * @brief Applies a median filter to the input data.
 *
 * This function sorts the window of the most recent data points and returns
 * the median value, which helps to eliminate outliers.
 *
 * @param new_value The latest data point.
 * @param window A deque maintaining the window of previous data points.
 * @return The median-filtered data point.
 */
inline float medianFilter(float new_value, std::deque<float>& window) {
    if (window.size() >= ConfigManager::MEDIAN_FILTER_WINDOW_SIZE) {
        window.pop_front();
    }
    window.push_back(new_value);

    // Copy and sort the window to find the median
    std::deque<float> sorted_window = window;
    std::sort(sorted_window.begin(), sorted_window.end());

    return sorted_window[sorted_window.size() / 2]; // Return the median value
}


