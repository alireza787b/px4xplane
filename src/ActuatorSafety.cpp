#include "ActuatorSafety.h"

#include <algorithm>
#include <cmath>

namespace ActuatorSafety {

float clampFinite(float value, float minValue, float maxValue, float fallback)
{
    if (!std::isfinite(value)) {
        return fallback;
    }

    if (minValue > maxValue) {
        std::swap(minValue, maxValue);
    }

    if (!std::isfinite(minValue) || !std::isfinite(maxValue)) {
        return fallback;
    }

    return std::clamp(value, minValue, maxValue);
}

bool isFiniteRange(float minValue, float maxValue)
{
    return std::isfinite(minValue) && std::isfinite(maxValue) && minValue != maxValue;
}

float scaleActuatorCommand(float input, float inputMin, float inputMax, float outputMin, float outputMax)
{
    if (!isFiniteRange(inputMin, inputMax) || !isFiniteRange(outputMin, outputMax)) {
        return 0.0f;
    }

    const float safeInput = clampFinite(input, inputMin, inputMax, 0.0f);
    const float scale = (outputMax - outputMin) / (inputMax - inputMin);
    const float scaled = outputMin + scale * (safeInput - inputMin);

    return clampFinite(scaled, outputMin, outputMax, 0.0f);
}

} // namespace ActuatorSafety
